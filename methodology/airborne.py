import argparse
import json
from pathlib import Path
from time import perf_counter, time

import numpy as np
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Inputs, Fixed
from methodology import electrical, motion, qualification, sensing, source_fingerprints
from methodology.cad import magnetics


def euler_rate_axes(pose):
    pitch, yaw = pose[4:]
    return np.array([[np.cos(yaw) * np.cos(pitch), -np.sin(yaw), 0],
                     [np.sin(yaw) * np.cos(pitch), np.cos(yaw), 0], [-np.sin(pitch), 0, 1]])


def euler_rates_to_world(pose, rates):
    return euler_rate_axes(pose) @ rates


def smooth_reference(origin, translation, rotation_vector, time, duration):
    if duration <= 0:
        raise ValueError('Motion duration must be positive')
    phase = np.clip(time / duration, 0., 1.)
    position = phase ** 3 * (10 - 15 * phase + 6 * phase ** 2)
    speed = 30 * phase ** 2 * (1 - phase) ** 2 / duration
    acceleration = 60 * phase * (1 - phase) * (1 - 2 * phase) / duration ** 2
    displacement = np.r_[translation, rotation_vector]
    rotation = Rotation.from_rotvec(position * displacement[3:]) * Rotation.from_euler('xyz', origin[3:])
    pose = np.r_[origin[:3] + position * displacement[:3], rotation.as_euler('xyz')]
    return pose, speed * displacement, acceleration * displacement


class AirborneController:
    def __init__(self, construction, identities, current_limit):
        self.geometry, self.array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
        self.identities = tuple(identities)
        if tuple(self.array.identities) != self.identities:
            raise ValueError('Controller and driver coil windows differ')
        levitation_sim.use_geometry(self.geometry)
        self.magnets = levitation_sim.magnet_layout_from_geometry()
        self.field_evaluator = None
        self.nominal_matrix = None
        self.current_budget = ((1 - Fixed.qualification_current_reserve_fraction) * current_limit
                               - Fixed.current_sense_offset_residual / Fixed.current_sense_resistance
                               - Fixed.max_current_command_error - Fixed.qualification_current_transient_allowance)

    def command(self, estimate, reference, reference_velocity=None, reference_acceleration=None):
        if not estimate['valid']:
            raise ValueError('Airborne control requires a usable sensor-only pose')
        pose, velocity = np.asarray(estimate['pose']), np.asarray(estimate['velocity'])
        rotation = Rotation.from_euler('xyz', pose[3:])
        error = np.r_[pose[:3] - reference[:3], (rotation * Rotation.from_euler('xyz', reference[3:]).inv()).as_rotvec()]
        angular_velocity = euler_rates_to_world(pose, velocity[3:])
        reference_velocity = np.zeros(6) if reference_velocity is None else np.asarray(reference_velocity)
        reference_acceleration = np.zeros(6) if reference_acceleration is None else np.asarray(reference_acceleration)
        acceleration = reference_acceleration + qualification.pose_correction(
            error, np.r_[velocity[:3], angular_velocity] - reference_velocity, np.zeros(6))
        target = np.zeros(6)
        target[:3] = self.geometry.weight / self.geometry.gravity * acceleration[:3]
        target[2] += self.geometry.weight
        world_inertia = rotation.as_matrix() @ np.diag(self.geometry.inertias) @ rotation.as_matrix().T
        target[3:] = world_inertia @ acceleration[3:] + np.cross(angular_velocity, world_inertia @ angular_velocity)
        levitation_sim.use_geometry(self.geometry)
        _, center = levitation_sim.place_piece(self.magnets, *pose)
        matrix = levitation_sim.actuator_matrix(self.magnets, self.array, center, self.field_evaluator)
        self.nominal_matrix = matrix
        required, initial = levitation_sim.min_peak_current(matrix, target)
        if required > self.current_budget:
            return dict(accepted=False, required_current_A=required, currents=None)
        currents = levitation_sim.min_loss_current(matrix, target, self.current_budget, initial_currents=initial)
        return dict(accepted=True, required_current_A=required, currents=currents)

    def measured_acceleration(self, estimate, readings, now):
        if not estimate['valid'] or self.nominal_matrix is None or readings.identities != self.identities:
            raise ValueError('Acceleration prediction requires estimated pose, nominal magnetics and matching measured currents')
        if readings.sample_times_s[-1] != now:
            raise ValueError('Acceleration prediction requires a fresh current sample')
        pose, velocity = np.asarray(estimate['pose']), np.asarray(estimate['velocity'])
        wrench = self.nominal_matrix @ readings.decode(now)[-1]
        rotation = Rotation.from_euler('xyz', pose[3:])
        axes = euler_rate_axes(pose)
        angular_velocity = axes @ velocity[3:]
        inertia = rotation.as_matrix() @ np.diag(self.geometry.inertias) @ rotation.as_matrix().T
        angular_acceleration = np.linalg.solve(inertia, wrench[3:] - np.cross(angular_velocity, inertia @ angular_velocity))
        yaw_velocity = axes[:, 2] * velocity[5]
        transport = (np.cross(yaw_velocity + axes[:, 1] * velocity[4], axes[:, 0]) * velocity[3]
                     + np.cross(yaw_velocity, axes[:, 1]) * velocity[4])
        return np.r_[wrench[:3] * self.geometry.gravity / self.geometry.weight - [0, 0, self.geometry.gravity],
                     np.linalg.solve(axes, angular_acceleration - transport)]


def acquire_stationary(plant, sensors, tracker):
    delay = sensing.hall_available_time(0, len(sensors.gain))
    count = int(np.ceil(Fixed.hall_tracking_acquisition_duration * Fixed.pose_feedback_rate)) + 1
    for index in range(count, -1, -1):
        plant.data.time = -delay - index / Fixed.pose_feedback_rate
        packet = sensors.sample_static(plant, {})
        estimate = tracker.consume(packet, {}, packet.available_time_s)
    plant.data.time = 0
    if not estimate['valid']:
        raise ValueError('Stationary sensor acquisition failed before the airborne interface check')
    return estimate


DRIVER_STATE = ('currents', 'command', 'filtered_command', 'integral', 'voltage', 'requested_voltage')
TRACKER_ARRAY_STATE = ('position', 'velocity', 'disturbance')
TRACKER_CLOCK_STATE = ('last_delivery', 'measurement_time', 'acquisition_start')


class AirborneSimulation:
    def __init__(self, construction, duration, disturbed=False, initialize=True, time_refinement=1, magnetic_refinement=None,
                 motion_scenario='hover'):
        if motion_scenario not in ('hover', 'local_move'):
            raise ValueError('Unknown airborne motion scenario')
        self.motion_scenario = motion_scenario
        self.started = perf_counter()
        self.duration = duration
        self.disturbed = disturbed
        self.time_refinement = time_refinement
        self.magnetic_refinement = min(Fixed.current_demand_mesh_refinements) if magnetic_refinement is None else magnetic_refinement
        if time_refinement < 1 or int(time_refinement) != time_refinement or self.magnetic_refinement < 1:
            raise ValueError('Numerical refinement must not coarsen the baseline resolution')
        self.plant = motion.MotionPlant(construction, magnetic_refinement=self.magnetic_refinement)
        self.fields = sensing.HallFieldModel(construction, magnetic_refinement=Fixed.hall_conductor_refinement * self.magnetic_refinement)
        self.sensors = sensing.VirtualHallArray(self.fields)
        nominal_fields = self.fields if self.magnetic_refinement == min(Fixed.current_demand_mesh_refinements) else sensing.HallFieldModel(construction)
        if nominal_fields.identities != self.fields.identities:
            raise ValueError('Physical and nominal Hall models require matching coil identities')
        self.tracker = sensing.HallTracker(sensing.HallPoseEstimator(nominal_fields))
        self.identities = tuple(self.plant.array.identities)
        self.drivers = electrical.CoilDrivers(construction, self.identities)
        self.current_sensors = electrical.VirtualCurrentSensors(self.identities)
        self.controller = AirborneController(construction, self.identities, self.drivers.current_limit)
        self.period = min(Fixed.motion_time_step, 1 / (Fixed.current_regulator_rate * Fixed.qualification_integration_substeps)) / time_refinement
        intervals = np.array([1 / Fixed.current_command_rate, 1 / Fixed.current_regulator_rate,
                              1 / Fixed.pose_feedback_rate, Fixed.airborne_checkpoint_interval, duration])
        ticks = np.rint(intervals / self.period).astype(int)
        if not np.allclose(ticks * self.period, intervals, atol=Fixed.motion_verification_tolerance, rtol=0) or np.any(ticks <= 0):
            raise ValueError('Airborne diagnostic clocks do not align with the numerical step')
        self.command_tick, self.regulator_tick, self.pose_tick, self.checkpoint_tick, self.total_steps = ticks
        self.plant.model.opt.timestep = self.period
        self.reference = np.zeros(6)
        self.reference[2] = Fixed.airborne_reference_gap_fraction * (self.plant.geometry.max_flight_gap - self.plant.geometry.gap)
        self.field_indices = [self.fields.identities.index(identity) for identity in self.identities]
        self.field_times, self.field_values, self.coil_field_values, self.readouts, self.trace = [], [], [], [], []
        self.step = 0
        self.next_scan = 0
        self.scan_delay = sensing.hall_available_time(0, len(self.fields.points))
        self.peaks = dict(current_A=0., voltage_V=0., copper_loss_W=0., back_emf_V=0., position_error_m=0., angle_error_rad=0.,
                          estimated_position_error_m=0., estimated_angle_error_rad=0., required_current_A=0.)
        self.voltage_saturated_updates = 0
        self.delivered_scans = 0
        self.stop_reason = None
        self.finished = False
        self.source = dict(self.plant.provenance, **source_fingerprints(__file__, electrical.__file__, sensing.__file__, qualification.__file__))
        if initialize:
            self.plant.reset(position=np.array([0., 0., self.plant.geometry.center_of_mass_height]) + self.reference[:3])
            if disturbed:
                displacement = Fixed.qualification_initial_error_fraction * Fixed.qualification_position_error / np.sqrt(3)
                rotation = Rotation.from_rotvec(np.ones(3) * Fixed.qualification_initial_error_fraction * Fixed.qualification_angle_error / np.sqrt(3))
                self.plant.reset(position=self.plant.data.qpos[:3] + displacement, rotation=rotation)
            acquire_stationary(self.plant, self.sensors, self.tracker)

    def reference_state(self, time):
        if self.motion_scenario == 'hover':
            return self.reference.copy(), np.zeros(6), np.zeros(6)
        return smooth_reference(self.reference, Fixed.airborne_motion_translation, Fixed.airborne_motion_rotation,
            time - Fixed.airborne_motion_start, Fixed.airborne_motion_duration)

    def advance(self):
        if self.finished:
            raise ValueError('Cannot advance a finished airborne trial')
        now = self.step * self.period
        self.process_events(now, self.step % self.regulator_tick == 0,
                            self.step % self.command_tick == 0, self.step == self.total_steps)
        if self.finished:
            return
        self.advance_physics(self.period)
        self.step += 1

    def physical_fields(self):
        truth = self.plant.truth()
        pose = np.r_[truth['com_position_m'] - [0, 0, self.plant.geometry.center_of_mass_height], truth['rotation'].as_euler('xyz')]
        coil = self.sensors.actual_coil_field[:, self.field_indices] @ self.drivers.currents
        return self.fields.magnet_field(pose) + coil, coil

    def moving_hall_frame(self, scan_start):
        return self.sensors.sample_moving(np.array(self.field_times), np.array(self.field_values), scan_start,
            np.array(self.coil_field_values), self.drivers.inductance / self.drivers.resistance)

    def process_events(self, now, regulator_due, command_due, complete):
        truth = self.plant.truth()
        true_pose = np.r_[truth['com_position_m'] - [0, 0, self.plant.geometry.center_of_mass_height], truth['rotation'].as_euler('xyz')]
        if not self.field_times or self.field_times[-1] != now:
            total, coil = self.physical_fields()
            self.field_times.append(now)
            self.field_values.append(total)
            self.coil_field_values.append(coil)
        if regulator_due:
            self.readouts.append(self.current_sensors.sample(self.drivers.currents, now))
        scan_start = self.next_scan / Fixed.pose_feedback_rate
        if now >= sensing.hall_available_time(scan_start, len(self.fields.points)):
            packet = self.moving_hall_frame(scan_start)
            available = [reading for reading in self.readouts if reading.available_time_s <= packet.available_time_s]
            history = electrical.CurrentReadings(self.identities, np.concatenate([reading.sample_times_s for reading in available]),
                np.vstack([reading.adc_codes for reading in available]), available[-1].available_time_s)
            estimate = self.tracker.consume(packet, history, now)
            if not estimate['valid']:
                self.stop_reason = 'tracking_' + estimate['reason']
            self.delivered_scans += 1
            self.next_scan += 1
        predicted = self.tracker.predict(now)
        if not predicted['valid']:
            self.stop_reason = 'tracking_' + predicted['reason']
        if command_due and self.stop_reason is None:
            allocation = self.controller.command(predicted, *self.reference_state(now))
            self.peaks['required_current_A'] = max(self.peaks['required_current_A'], float(allocation['required_current_A']))
            if allocation['accepted']:
                self.drivers.set_command(allocation['currents'])
            else:
                self.stop_reason = 'current_allocation_exceeds_reserved_budget'
        if regulator_due and self.stop_reason is None:
            self.drivers.regulate(self.readouts[-1], now)
            self.tracker.record_acceleration(now, self.controller.measured_acceleration(predicted, self.readouts[-1], now))
            self.voltage_saturated_updates += int(np.any(np.abs(self.drivers.requested_voltage) > self.drivers.voltage_limit))
        clearance = self.observe_physics(truth)
        if predicted['valid']:
            estimated_pose = np.array(predicted['pose'])
            self.peaks['estimated_position_error_m'] = max(self.peaks['estimated_position_error_m'], float(np.linalg.norm(estimated_pose[:3] - true_pose[:3])))
            self.peaks['estimated_angle_error_rad'] = max(self.peaks['estimated_angle_error_rad'], float((Rotation.from_euler('xyz', estimated_pose[3:]) * truth['rotation'].inv()).magnitude()))
        self.finished = bool(self.stop_reason is not None or complete)
        if command_due or self.finished:
            self.trace.append(dict(time_s=now, pose=true_pose.tolist(), estimated_pose=predicted['pose'] if predicted['valid'] else None,
                                   reference_pose=self.reference_state(now)[0].tolist(),
                                   currents_A=self.drivers.currents.tolist(),
                                   peak_current_A=float(np.max(np.abs(self.drivers.currents))), clearance_m=float(clearance),
                                   required_current_A=self.peaks['required_current_A']))
        if self.finished:
            return

    def observe_physics(self, truth):
        true_pose = np.r_[truth['com_position_m'] - [0, 0, self.plant.geometry.center_of_mass_height], truth['rotation'].as_euler('xyz')]
        failure, clearance = motion.airborne_observation(self.plant, truth, self.drivers, self.fields.base_radius)
        if failure is not None:
            self.stop_reason = failure
        reference = self.reference_state(self.plant.data.time)[0]
        position_error = float(np.linalg.norm(true_pose[:3] - reference[:3]))
        angle_error = float((truth['rotation'] * Rotation.from_euler('xyz', reference[3:]).inv()).magnitude())
        self.peaks['position_error_m'] = max(self.peaks['position_error_m'], position_error)
        self.peaks['angle_error_rad'] = max(self.peaks['angle_error_rad'], angle_error)
        self.peaks['current_A'] = max(self.peaks['current_A'], float(np.max(np.abs(self.drivers.currents))))
        self.peaks['voltage_V'] = max(self.peaks['voltage_V'], float(np.max(np.abs(self.drivers.voltage))))
        self.peaks['copper_loss_W'] = max(self.peaks['copper_loss_W'], float(self.drivers.copper_resistance * self.drivers.currents @ self.drivers.currents))
        if self.stop_reason is None and (position_error > Fixed.qualification_position_error or angle_error > Fixed.qualification_angle_error):
            self.stop_reason = 'tracking_error_budget_exceeded'
        return clearance

    def advance_physics(self, period, matrix=None, enabled=True):
        self.plant.model.opt.timestep = period
        truth = self.plant.truth()
        if matrix is None:
            matrix = self.plant.magnetic_matrix(window_center=self.reference[:2])
        if tuple(self.plant.array.identities) != self.identities:
            raise ValueError('Physical coil window changed during the local airborne check')
        back_emf = matrix.T @ np.r_[truth['linear_velocity_m_per_s'], truth['angular_velocity_rad_per_s']]
        self.peaks['back_emf_V'] = max(self.peaks['back_emf_V'], float(np.max(np.abs(back_emf))))
        average_currents = self.drivers.advance(back_emf, period, enabled)
        self.plant.advance_wrench(matrix @ average_currents)
        self.integration_energy_defect = (self.plant.mechanical_energy_change(truth, self.plant.truth())
            - float(back_emf @ average_currents) * period)
        return back_emf

    def checkpoint_due(self):
        return self.step % self.checkpoint_tick == 0

    def checkpoint_state(self):
        return dict(schema='airborne-coupled-v5', source_sha256=self.source, duration_s=self.duration, disturbed=self.disturbed,
                    motion_scenario=self.motion_scenario,
                    time_refinement=self.time_refinement, magnetic_refinement=self.magnetic_refinement,
                    plant=self.plant.checkpoint_state(), step=int(self.step), next_scan=self.next_scan,
                    drivers={name: getattr(self.drivers, name).tolist() for name in DRIVER_STATE},
                    tracker={**{name: None if getattr(self.tracker, name) is None else getattr(self.tracker, name).tolist() for name in TRACKER_ARRAY_STATE},
                             **{name: getattr(self.tracker, name) for name in TRACKER_CLOCK_STATE}},
                    nominal_matrix=None if self.controller.nominal_matrix is None else self.controller.nominal_matrix.tolist(),
                    acceleration_times=self.tracker.acceleration_times,
                    accelerations=np.asarray(self.tracker.accelerations).tolist(),
                    sensors=dict(offset=self.sensors.offset.tolist(), gain=self.sensors.gain.tolist(),
                                 actual_coil_field=self.sensors.actual_coil_field.tolist(), random=self.sensors.random.bit_generator.state),
                    current_sensors=dict(offset=self.current_sensors.offset.tolist(), gain=self.current_sensors.gain.tolist(),
                                         random=self.current_sensors.random.bit_generator.state),
                    field_times=self.field_times, field_values=np.asarray(self.field_values).tolist(),
                    coil_field_values=np.asarray(self.coil_field_values).tolist(),
                    readouts=[dict(sample_times_s=row.sample_times_s.tolist(), adc_codes=row.adc_codes.tolist(), available_time_s=row.available_time_s)
                              for row in self.readouts], trace=self.trace, peaks=self.peaks,
                    voltage_saturated_updates=self.voltage_saturated_updates, delivered_scans=self.delivered_scans,
                    finished=self.finished, stop_reason=self.stop_reason)

    def save_checkpoint(self, path):
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(json.dumps(self.checkpoint_state(), allow_nan=False) + '\n')
        temporary.replace(path)

    def load_checkpoint(self, path):
        state = json.loads(path.read_text())
        if state['schema'] != 'airborne-coupled-v5' or state['source_sha256'] != self.source:
            raise ValueError('Coupled checkpoint schema, source or dependencies differ')
        if (state['duration_s'] != self.duration or state['disturbed'] != self.disturbed
                or state['time_refinement'] != self.time_refinement or state['magnetic_refinement'] != self.magnetic_refinement):
            raise ValueError('Coupled checkpoint scenario differs')
        self.restore_checkpoint(state)

    def restore_checkpoint(self, state):
        if state['motion_scenario'] != self.motion_scenario:
            raise ValueError('Coupled checkpoint motion scenario differs')
        self.plant.restore_checkpoint(state['plant'])
        self.controller.nominal_matrix = None if state['nominal_matrix'] is None else np.array(state['nominal_matrix'])
        self.tracker.acceleration_times = state['acceleration_times']
        self.tracker.accelerations = [np.array(value) for value in state['accelerations']]
        for name in DRIVER_STATE:
            setattr(self.drivers, name, np.array(state['drivers'][name]))
        for name in TRACKER_ARRAY_STATE:
            setattr(self.tracker, name, None if state['tracker'][name] is None else np.array(state['tracker'][name]))
        for name in TRACKER_CLOCK_STATE:
            setattr(self.tracker, name, state['tracker'][name])
        for target, values in ((self.sensors, state['sensors']), (self.current_sensors, state['current_sensors'])):
            target.offset, target.gain = np.array(values['offset']), np.array(values['gain'])
            target.random.bit_generator.state = values['random']
        self.sensors.actual_coil_field = np.array(state['sensors']['actual_coil_field'])
        self.field_values = [np.array(row) for row in state['field_values']]
        self.coil_field_values = [np.array(row) for row in state['coil_field_values']]
        self.readouts = [electrical.CurrentReadings(self.identities, np.array(row['sample_times_s']), np.array(row['adc_codes']), row['available_time_s'])
                         for row in state['readouts']]
        for name in ('step', 'next_scan', 'field_times', 'trace', 'peaks', 'voltage_saturated_updates', 'delivered_scans', 'finished', 'stop_reason'):
            setattr(self, name, state[name])

    def report(self):
        return dict(scope='Airborne sensor/driver/MuJoCo trial; provisional model, not flight-envelope qualification',
            source_sha256=self.source, duration_completed_s=float(self.plant.data.time), intended_duration_s=self.duration,
            disturbed=self.disturbed, reference_pose=self.reference.tolist(), motion_scenario=self.motion_scenario,
            finished=self.finished, stop_reason=self.stop_reason,
            passed=bool(self.finished and self.stop_reason is None and self.delivered_scans > 0
                        and self.peaks['current_A'] <= self.drivers.current_limit
                        and self.peaks['estimated_position_error_m'] <= Fixed.qualification_position_error
                        and self.peaks['estimated_angle_error_rad'] <= Fixed.qualification_angle_error),
            peaks=self.peaks, delivered_moving_scans=self.delivered_scans, voltage_saturated_updates=self.voltage_saturated_updates,
            physics_time_step_s=self.period, elapsed_s=perf_counter() - self.started, trace=self.trace,
            time_refinement=self.time_refinement, magnetic_refinement=self.magnetic_refinement,
            force_quadrature_orders=dict(physical=self.plant.array.force_order,
                nominal=Fixed.winding_force_quadrature_order * min(Fixed.current_demand_mesh_refinements)),
            wire_segments_mm=dict(hall_physical=Fixed.winding_magnetic_segment_length / self.fields.magnetic_refinement,
                hall_nominal=Fixed.winding_magnetic_segment_length / Fixed.hall_conductor_refinement),
            hardware_rates_Hz=dict(command=Fixed.current_command_rate, regulator=Fixed.current_regulator_rate, feedback=Fixed.pose_feedback_rate),
            hall_conversion=dict(native_bits=Fixed.hall_adc_native_bits, samples_per_sensor=Fixed.hall_samples_per_sensor,
                sample_rate_Hz=Fixed.hall_adc_sample_rate, sensor_noise_rms_T=Fixed.hall_output_noise),
            current_conversion=dict(native_bits=Fixed.current_feedback_bits, sample_rate_Hz=Fixed.current_regulator_rate,
                quantum_A=electrical.current_quantum(), sensor_noise_rms_A=Fixed.current_feedback_noise,
                command_step_A=2 * Fixed.max_current_command_error),
            control_dynamics_Hz=dict(pose=Fixed.pose_controller_frequency,
                                     observer=Fixed.pose_controller_frequency * Fixed.pose_observer_frequency_ratio),
            mechanical_properties={key: self.plant.metadata[key] for key in
                ('king_mass_kg', 'king_com_from_magnet_bottom_m', 'king_inertia_kg_m2')},
            limitations=['Stationary sensor-only acquisition precedes release; setup motion is frozen and excluded from controlled-flight evidence. Currents start at zero.',
                'Hall conversion is an ideal uniform quantizer at the stated native precision; converter noise, nonlinearity, reference errors and multiplexing settling need hardware qualification.',
                'Fixed local coil and sensor windows, nominal CAD mass and magnet properties only; commanded movement does not implement window handoffs.',
                'Independent RL windings with estimated self-inductance, averaged voltages and constant winding/driver/shunt resistance; no PWM ripple, mutual inductance, bus dynamics or thermal feedback.',
                'Provisional synchronous current ADCs at the regulator rate, with offset, gain error, noise and quantization. Channel throughput and latency are not hardware-qualified.',
                'Hall correction reconstructs current history from available measurements and nominal filter startup. Interpolation accuracy requires numerical and measurement-rate checks.',
                'Pose controller uses PD tracking with reference velocity and acceleration feedforward. Observer propagation uses measured-current acceleration from the held nominal allocation matrix plus an estimated residual; current calibration and nominal force-model errors remain.',
                'No conductive-sheet drag or neighbours; no time/force-mesh convergence, uncertainty qualification or embedded compute-time evidence.',
                'Video replays stored physical poses with interpolation; it does not rerun dynamics.'])


def run_simulation(simulation, output, checkpoint, deadline=None, progress=None):
    while not simulation.finished:
        if deadline is not None and time() >= deadline:
            simulation.save_checkpoint(checkpoint)
            report = simulation.report()
            output.write_text(json.dumps(report, indent=Fixed.report_precision, allow_nan=False) + '\n')
            return report
        simulation.advance()
        if simulation.checkpoint_due() or simulation.finished:
            if progress is not None:
                progress(simulation.report())
            simulation.save_checkpoint(checkpoint)
            report = simulation.report()
            output.write_text(json.dumps(report, indent=Fixed.report_precision, allow_nan=False) + '\n')
            print(f'{simulation.plant.data.time:.6f} s: peak current {simulation.peaks["current_A"]:.3f} A, stop={simulation.stop_reason}', flush=True)
    return simulation.report()


def run(output, checkpoint, resume=False, trial=False):
    construction = magnetics.ConstructionModel()
    simulation = AirborneSimulation(construction, Fixed.qualification_recovery_duration if trial else Fixed.airborne_check_duration,
                                    disturbed=trial, initialize=not resume)
    if resume:
        simulation.load_checkpoint(checkpoint)
    return run_simulation(simulation, output, checkpoint)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run and resume the sensor/driver/magnetic/MuJoCo airborne trial.')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('results') / 'airborne.json')
    parser.add_argument('--checkpoint', type=Path, default=Path(__file__).with_name('results') / 'airborne_checkpoint.json')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--trial', action='store_true', help='Attempt the longer hover/recovery scenario instead of the short interface check.')
    arguments = parser.parse_args()
    run(arguments.output, arguments.checkpoint, arguments.resume, arguments.trial)

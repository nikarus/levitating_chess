import argparse
from copy import copy, deepcopy
from datetime import datetime
import json
from pathlib import Path
from time import perf_counter

import mujoco
import numpy as np
from scipy.integrate import solve_ivp

from model import Inputs, Fixed
from methodology import adaptive_airborne, airborne, airborne_verification, sensing, source_fingerprints
from methodology.cad import magnetics
from methodology.magnetic_atlas import MagneticBackend


class FilteredTrajectory:
    def __init__(self, physical, filtered):
        self.physical = physical
        self.filtered = filtered

    def __call__(self, time):
        return np.concatenate((self.physical(time), self.filtered(time)), axis=0)


def sample_scan_segment(segment, scan, start, hall_filter):
    sample_times = start + Fixed.hall_power_settle_time + np.arange(scan['samples'].size) / Fixed.hall_adc_sample_rate
    indices = np.flatnonzero((np.arange(scan['samples'].size) >= scan['sample_count'])
        & (sample_times >= segment.t[0]) & (sample_times <= segment.t[-1]))
    if len(indices):
        values = segment.sol(sample_times[indices])[hall_filter]
        values -= scan['initial_filter'][:, None] * np.exp(-2 * np.pi * Fixed.hall_sensor_bandwidth * (sample_times[indices] - start))
        scan['samples'].ravel()[indices] = values[indices // Fixed.hall_samples_per_sensor, np.arange(len(indices))]
        scan['sample_count'] += len(indices)


class JointDynamics:
    def __init__(self, plant, drivers, field_count, force_matrix, field_signal):
        self.plant = copy(plant)
        self.plant.data = mujoco.MjData(plant.model)
        self.drivers = drivers
        self.force_matrix = force_matrix
        self.field_signal = field_signal
        count = len(drivers.currents)
        self.current = slice(13, 13 + count)
        self.command_filter = slice(self.current.stop, self.current.stop + count)
        self.work = slice(self.command_filter.stop, self.command_filter.stop + 2)
        self.hall_filter = slice(self.work.stop, self.work.stop + field_count)
        self.size = self.hall_filter.stop
        self.evaluations = 0
        self.filter_evaluations = 0
        self.peak_trial_back_emf = 0.
        self.energy_budget = adaptive_airborne.integration_energy_budget(plant)
        self.atol = np.r_[np.full(3, Fixed.adaptive_position_tolerance / np.sqrt(3)),
            np.full(4, Fixed.adaptive_angle_tolerance / 4),
            np.full(3, Fixed.adaptive_velocity_tolerance / np.sqrt(3)),
            np.full(3, Fixed.adaptive_angular_velocity_tolerance / np.sqrt(3)),
            np.full(2 * count, Fixed.adaptive_current_tolerance),
            np.full(2, self.energy_budget * Fixed.joint_work_tolerance_fraction)] / np.sqrt(self.work.stop)
        self.filter_atol = Fixed.adaptive_field_tolerance / np.sqrt(field_count)

    def pack(self, plant, hall_filter):
        return np.r_[plant.data.qpos, plant.data.qvel, self.drivers.currents,
            self.drivers.filtered_command, np.zeros(2), hall_filter]

    def set_stage(self, time, state):
        if state.shape != (self.work.stop,) or not np.isfinite(state).all():
            raise ValueError('Joint integration state is malformed or nonfinite')
        norm = np.linalg.norm(state[3:7])
        if norm == 0:
            raise ValueError('Joint integration quaternion has zero norm')
        mujoco.mj_resetData(self.plant.model, self.plant.data)
        self.plant.data.qpos[:] = state[:7]
        self.plant.data.qpos[3:] /= norm
        self.plant.data.qvel[:] = state[7:13]
        self.plant.data.time = time
        mujoco.mj_forward(self.plant.model, self.plant.data)
        if self.plant.data.ncon:
            raise ValueError('Joint airborne integration encountered unsupported contact')

    def derivative(self, time, state):
        self.set_stage(time, state)
        self.evaluations += 1
        matrix = self.force_matrix(self.plant)
        if tuple(self.plant.array.identities) != self.drivers.identities:
            raise ValueError('Joint integration changed the physical coil window')
        truth = self.plant.truth()
        currents = np.where(self.conducting, state[self.current], 0.)
        emf = matrix.T @ np.r_[truth['linear_velocity_m_per_s'], truth['angular_velocity_rad_per_s']]
        if np.any(~self.enabled & (np.abs(emf) >= Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction)):
            raise ValueError('Disabled-channel back-EMF reaches the rail; re-conduction is unsupported')
        self.peak_trial_back_emf = max(self.peak_trial_back_emf, float(np.max(np.abs(emf))))
        self.plant.apply_wrench(matrix @ currents)
        mujoco.mj_forward(self.plant.model, self.plant.data)
        if np.any(self.plant.data.warning.number):
            raise ValueError('MuJoCo warning during joint forward dynamics')
        result = np.empty(self.work.stop)
        result[:3] = state[7:10]
        mujoco.mju_derivQuat(result[3:7], state[3:7], truth['angular_velocity_rad_per_s'])
        result[7:13] = self.plant.data.qacc
        result[self.current] = np.where(self.conducting,
            (self.voltage - self.drivers.resistance * currents - emf) / self.drivers.inductance, 0.)
        result[self.command_filter] = 2 * np.pi * Fixed.setpoint_filter_cutoff * (self.drivers.command - state[self.command_filter])
        result[self.work] = [emf @ currents, (self.voltage - self.drivers.resistance * currents) @ currents]
        return result

    def integrate_filter(self, solution, initial, tolerance_scale):
        physical = solution.sol
        def derivative(time, filtered):
            state = physical(time)
            self.set_stage(time, state)
            self.filter_evaluations += 1
            currents = np.where(self.conducting, state[self.current], 0.)
            return 2 * np.pi * Fixed.hall_sensor_bandwidth * (self.field_signal(self.plant, currents) - filtered)
        filtered = solve_ivp(derivative, (solution.t[0], solution.t[-1]), initial, method='RK45',
            rtol=Fixed.joint_relative_tolerance * tolerance_scale, atol=self.filter_atol * tolerance_scale,
            max_step=Fixed.adaptive_max_step, dense_output=True)
        if not filtered.success:
            raise ValueError('Passive Hall integration failed: ' + filtered.message)
        solution.sol = FilteredTrajectory(physical, filtered.sol)
        solution.y = solution.sol(solution.t)

    def integrate(self, start, end, initial, enabled=True, tolerance_scale=1):
        if end <= start or tolerance_scale <= 0 or not np.isfinite(tolerance_scale):
            raise ValueError('Joint integration requires a positive interval and tolerance scale')
        self.enabled = np.broadcast_to(np.asarray(enabled), self.drivers.currents.shape).copy()
        phase_drivers = copy(self.drivers)
        phase_drivers.currents = initial[self.current]
        self.voltage = phase_drivers.terminal_voltage(self.enabled)
        self.conducting = self.enabled | (initial[self.current] != 0)
        state = initial.copy()
        segments = []
        while start < end:
            channels = np.flatnonzero(~self.enabled & self.conducting)
            signs = np.sign(state[self.current][channels])
            events = []
            for channel in channels:
                def extinction(time, values, index=self.current.start + channel):
                    return values[index]
                extinction.terminal = True
                extinction.direction = -np.sign(state[self.current][channel])
                events.append(extinction)
            solution = solve_ivp(self.derivative, (start, end), state[:self.work.stop], method='RK45',
                rtol=Fixed.joint_relative_tolerance * tolerance_scale, atol=self.atol * tolerance_scale,
                max_step=Fixed.adaptive_max_step, dense_output=True, events=events)
            if not solution.success:
                raise ValueError('Joint RK45 integration failed: ' + solution.message)
            if solution.t[-1] <= start:
                raise ValueError('Joint extinction handling did not advance time')
            self.integrate_filter(solution, state[self.hall_filter], tolerance_scale)
            segments.append(solution)
            state = solution.y[:, -1].copy()
            start = float(solution.t[-1])
            if solution.status == 1:
                extinguished = np.array([bool(len(times)) for times in solution.t_events])
                if len(channels) > 1:
                    slopes = self.derivative(start, state[:self.work.stop])[self.current][channels]
                    currents = state[self.current][channels]
                    clock_bound = np.abs(slopes) * Fixed.adaptive_clock_tolerance
                    if np.any(signs * currents < -clock_bound):
                        raise ValueError('Disabled current passed extinction before the located event')
                    extinguished |= (signs * slopes < 0) & (np.abs(currents) <= clock_bound)
                state[self.current.start + channels[extinguished]] = 0.
                self.conducting[channels[extinguished]] = False
                solution.y[:, -1] = state
        return segments

    def commit(self, plant, time, state):
        norm = np.linalg.norm(state[3:7])
        if abs(norm - 1) > Fixed.joint_quaternion_norm_tolerance:
            raise ValueError('Accepted quaternion drift exceeds the integration limit')
        plant.data.qpos[:] = state[:7]
        plant.data.qpos[3:] /= norm
        plant.data.qvel[:] = state[7:13]
        plant.data.time = time
        plant.data.qfrc_applied[:] = 0
        plant.data.xfrc_applied[:] = 0
        mujoco.mj_forward(plant.model, plant.data)
        self.drivers.currents = state[self.current].copy()
        self.drivers.filtered_command = state[self.command_filter].copy()


class JointAirborneSimulation(MagneticBackend, airborne.AirborneSimulation):
    def __init__(self, construction, duration, disturbed=False, initialize=True, tolerance_scale=1,
                 atlas=None, magnetic_mode='direct'):
        super().__init__(construction, duration, disturbed=disturbed, initialize=initialize)
        self.configure_atlas(atlas, magnetic_mode)
        self.construction = construction
        self.tolerance_scale = tolerance_scale
        self.clock = 0.
        self.next_command = self.next_regulator = self.next_hall_start = 0
        self.next_checkpoint = 1
        self.last_checkpoint_due = False
        self.hall_filter = np.zeros(len(self.fields.points))
        self.pending_scans = {}
        self.dynamics = JointDynamics(self.plant, self.drivers, len(self.fields.points),
            lambda plant: plant.magnetic_matrix(window_center=self.reference[:2]), self.field_signal)
        self.accounting = dict(accepted_steps=0, signed_mechanical_defect_J=0., absolute_mechanical_defect_J=0.,
            signed_total_defect_J=0., absolute_total_defect_J=0., energy_budget_J=self.dynamics.energy_budget,
            minimum_step_s=None, maximum_step_s=0.)
        self.source.update(source_fingerprints(__file__, adaptive_airborne.__file__))

    def field_signal(self, plant, currents):
        truth = plant.truth()
        pose = np.r_[truth['com_position_m'] - [0, 0, plant.geometry.center_of_mass_height], truth['rotation'].as_euler('xyz')]
        field = self.fields.magnet_field(pose) + self.sensors.actual_coil_field[:, self.field_indices] @ currents
        return self.sensors.gain * field + self.sensors.offset

    def moving_hall_frame(self, scan_start):
        scan = self.pending_scans.pop(self.next_scan)
        if scan['sample_count'] != scan['samples'].size or scan_start != self.next_scan / Fixed.pose_feedback_rate:
            raise ValueError('Joint Hall scan is incomplete or belongs to a different acquisition')
        return self.sensors.encode_signal(scan['samples'], scan_start)

    def checkpoint_due(self):
        return self.last_checkpoint_due

    def advance(self):
        if self.finished:
            raise ValueError('Cannot advance a finished joint trial')
        now = self.clock
        if abs(now - self.next_hall_start / Fixed.pose_feedback_rate) <= Fixed.adaptive_clock_tolerance:
            self.pending_scans[self.next_hall_start] = dict(initial_filter=self.hall_filter.copy(),
                samples=np.zeros((len(self.fields.points), Fixed.hall_samples_per_sensor)), sample_count=0)
            self.next_hall_start += 1
        regulator_due = abs(now - self.next_regulator / Fixed.current_regulator_rate) <= Fixed.adaptive_clock_tolerance
        command_due = abs(now - self.next_command / Fixed.current_command_rate) <= Fixed.adaptive_clock_tolerance
        self.process_events(now, regulator_due, command_due, abs(now - self.duration) <= Fixed.adaptive_clock_tolerance)
        self.next_command += int(command_due)
        self.next_regulator += int(regulator_due)
        self.last_checkpoint_due = False
        if self.finished:
            return
        boundaries = [self.next_command / Fixed.current_command_rate, self.next_regulator / Fixed.current_regulator_rate,
            sensing.hall_available_time(self.next_scan / Fixed.pose_feedback_rate, len(self.fields.points)),
            self.next_hall_start / Fixed.pose_feedback_rate,
            self.next_checkpoint * Fixed.airborne_checkpoint_interval, self.duration]
        boundary = min(boundaries)
        initial = self.dynamics.pack(self.plant, self.hall_filter)
        segments = self.dynamics.integrate(now, boundary, initial, tolerance_scale=self.tolerance_scale)
        previous = initial
        for segment in segments:
            for index in range(1, len(segment.t)):
                time, state = float(segment.t[index]), segment.y[:, index]
                before = self.plant.truth()
                self.dynamics.commit(self.plant, time, state)
                mechanical = self.plant.mechanical_energy_change(before, self.plant.truth())
                inductive = self.drivers.inductance * np.sum((state[self.dynamics.current] - previous[self.dynamics.current])
                    * (state[self.dynamics.current] + previous[self.dynamics.current])) / 2
                work = state[self.dynamics.work] - previous[self.dynamics.work]
                for name, defect in (('mechanical', mechanical - work[0]), ('total', mechanical + inductive - work[1])):
                    self.accounting['signed_' + name + '_defect_J'] += float(defect)
                    self.accounting['absolute_' + name + '_defect_J'] += float(abs(defect))
                period = time - self.clock
                self.accounting['accepted_steps'] += 1
                self.accounting['minimum_step_s'] = period if self.accounting['minimum_step_s'] is None else min(self.accounting['minimum_step_s'], period)
                self.accounting['maximum_step_s'] = max(self.accounting['maximum_step_s'], period)
                self.clock = time
                self.hall_filter = state[self.dynamics.hall_filter].copy()
                self.observe_physics(self.plant.truth())
                if max(self.accounting['absolute_mechanical_defect_J'], self.accounting['absolute_total_defect_J']) > self.dynamics.energy_budget:
                    self.stop_reason = 'joint_accumulated_energy_limit'
                if self.stop_reason is not None:
                    self.finished = True
                    return
                previous = state
            for scan_index, scan in self.pending_scans.items():
                start = scan_index / Fixed.pose_feedback_rate
                sample_scan_segment(segment, scan, start, self.dynamics.hall_filter)
        self.step += 1
        self.peaks['back_emf_V'] = self.dynamics.peak_trial_back_emf
        if abs(self.clock - self.next_checkpoint * Fixed.airborne_checkpoint_interval) <= Fixed.adaptive_clock_tolerance:
            self.next_checkpoint += 1
            self.last_checkpoint_due = True

    def checkpoint_state(self):
        state = super().checkpoint_state()
        state['schema'] = 'airborne-joint-v3'
        state['construction'] = self.construction_state()
        state['joint'] = {name: getattr(self, name) for name in ('tolerance_scale', 'clock', 'next_command', 'next_regulator',
            'next_checkpoint', 'last_checkpoint_due', 'next_hall_start', 'accounting')}
        state['joint'].update(hall_filter=self.hall_filter.tolist(),
            pending_scans={str(key): dict(initial_filter=scan['initial_filter'].tolist(), samples=scan['samples'].tolist(),
                sample_count=scan['sample_count']) for key, scan in self.pending_scans.items()},
            evaluations=self.dynamics.evaluations, filter_evaluations=self.dynamics.filter_evaluations,
            peak_trial_back_emf=self.dynamics.peak_trial_back_emf)
        return state

    def construction_state(self):
        return dict(piece_properties=deepcopy(self.construction.piece_properties),
            use_cad_mass=self.construction.use_cad_mass, return_depth=self.construction.return_depth)

    @classmethod
    def from_checkpoint(cls, path, atlas=None):
        state = json.loads(path.read_text())
        if state['schema'] != 'airborne-joint-v3':
            raise ValueError('Joint checkpoint schema differs')
        construction = magnetics.ConstructionModel(recorded_piece_properties=state['construction']['piece_properties'])
        construction.use_cad_mass = state['construction']['use_cad_mass']
        construction.return_depth = state['construction']['return_depth']
        simulation = cls(construction, state['duration_s'], disturbed=state['disturbed'], initialize=False,
            tolerance_scale=state['joint']['tolerance_scale'], atlas=atlas, magnetic_mode=state['atlas']['mode'])
        simulation.load_checkpoint(path)
        return simulation

    def load_checkpoint(self, path):
        state = json.loads(path.read_text())
        if state['duration_s'] != self.duration or state['disturbed'] != self.disturbed or state['joint']['tolerance_scale'] != self.tolerance_scale:
            raise ValueError('Joint checkpoint scenario differs')
        self.restore_checkpoint(state)

    def restore_checkpoint(self, state):
        if state['schema'] != 'airborne-joint-v3' or state['source_sha256'] != self.source:
            raise ValueError('Joint checkpoint schema or source differs')
        if state['construction'] != self.construction_state():
            raise ValueError('Joint checkpoint CAD construction differs')
        self.validate_atlas(state)
        super().restore_checkpoint(state)
        for name, value in state['joint'].items():
            if name in ('evaluations', 'filter_evaluations', 'peak_trial_back_emf'):
                setattr(self.dynamics, name, value)
            elif name == 'pending_scans':
                self.pending_scans = {int(key): dict(initial_filter=np.array(scan['initial_filter']),
                    samples=np.array(scan['samples']), sample_count=scan['sample_count']) for key, scan in value.items()}
            else:
                setattr(self, name, np.array(value) if name == 'hall_filter' else deepcopy(value))

    def report(self):
        report = super().report()
        report.update(integration_mode='joint_RK45', physics_time_step_s=None, tolerance_scale=self.tolerance_scale,
            integration=dict(self.accounting, rhs_evaluations=self.dynamics.evaluations,
                filter_evaluations=self.dynamics.filter_evaluations))
        report['limitations'].append('Joint RK45 integrates motion and current; passive Hall filtering uses its dense trajectory. Local RMS tolerances and accumulated work diagnostics are not global trajectory bounds. Back-EMF peak includes trial stages. Contact, reconduction and coil handoffs are unsupported.')
        return report


def screen(output, atlas=None):
    if output.exists():
        raise ValueError('Joint integration output exists; preserve its evidence')
    output.mkdir(parents=True)
    construction = magnetics.ConstructionModel()
    simulation = JointAirborneSimulation(construction, Fixed.airborne_check_duration)
    initial = deepcopy(simulation.checkpoint_state())
    simulation.save_checkpoint(output / 'initial_checkpoint.json')
    summary = dict(scope='One direct-field regulator interval with physical Hall filtering; cost/refinement screen, not flight qualification',
        source_sha256=simulation.source, status='running', cases={},
        absolute_component_tolerances=simulation.dynamics.atol.tolist(), filter_absolute_tolerance=simulation.dynamics.filter_atol,
        relative_tolerance=Fixed.joint_relative_tolerance)
    states = {}
    cases = [('nominal', 1), ('tighter', 1 / Fixed.qualification_refinement_factor)]
    if atlas is not None:
        cases.append(('atlas', 1))
        summary.update(scope='One regulator interval: direct refinement and atlas comparison; not coupled-flight or combined-budget qualification',
                       atlas_identity=atlas.identity)
    for name, scale in cases:
        if name == 'atlas':
            simulation = JointAirborneSimulation(construction, Fixed.airborne_check_duration,
                atlas=atlas, magnetic_mode='atlas_forces')
            twin = simulation.checkpoint_state()
            if {key: value for key, value in twin.items() if key != 'atlas'} != {key: value for key, value in initial.items() if key != 'atlas'}:
                raise ValueError('Direct and atlas initial states differ beyond the declared backend')
            simulation.save_checkpoint(output / 'atlas_initial_checkpoint.json')
        else:
            simulation.restore_checkpoint(deepcopy(initial))
            simulation.tolerance_scale = scale
        started = perf_counter()
        simulation.advance()
        summary['cases'][name] = dict(elapsed_s=perf_counter() - started, completed_time_s=simulation.clock,
            stop_reason=simulation.stop_reason, integration=simulation.report()['integration'])
        states[name] = dict(position=simulation.plant.data.qpos[:3].copy(), rotation=simulation.plant.truth()['rotation'],
            velocity=simulation.plant.data.qvel[:3].copy(), angular_velocity=simulation.plant.truth()['angular_velocity_rad_per_s'],
            current=simulation.drivers.currents.copy(), field=simulation.hall_filter.copy())
        simulation.save_checkpoint(output / (name + '_checkpoint.json'))
        if simulation.stop_reason is not None:
            summary['status'] = 'failed'
            airborne_verification.save_summary(output / 'summary.json', summary)
            return summary
        airborne_verification.save_summary(output / 'summary.json', summary)
    limits = dict(position=Fixed.adaptive_position_tolerance, angle=Fixed.adaptive_angle_tolerance,
        velocity=Fixed.adaptive_velocity_tolerance, angular_velocity=Fixed.adaptive_angular_velocity_tolerance,
        current=Fixed.adaptive_current_tolerance, field=Fixed.adaptive_field_tolerance)
    differences = {key: float(np.linalg.norm(states['nominal'][key] - states['tighter'][key])) for key in
        ('position', 'velocity', 'angular_velocity')}
    differences.update(angle=float((states['nominal']['rotation'] * states['tighter']['rotation'].inv()).magnitude()),
        current=float(np.max(np.abs(states['nominal']['current'] - states['tighter']['current']))),
        field=float(np.max(np.abs(states['nominal']['field'] - states['tighter']['field']))))
    summary.update(differences=differences, limits=limits,
        status='completed' if all(differences[key] <= limits[key] for key in limits) else 'refinement_failed')
    if atlas is not None:
        actual, expected = states['atlas'], states['nominal']
        delta = {key: float(np.linalg.norm(actual[key] - expected[key])) for key in ('position', 'velocity', 'angular_velocity')}
        delta.update(angle=float((actual['rotation'] * expected['rotation'].inv()).magnitude()),
            current=float(np.max(np.abs(actual['current'] - expected['current']))),
            field=float(np.max(np.abs(actual['field'] - expected['field']))))
        restored = JointAirborneSimulation.from_checkpoint(output / 'atlas_checkpoint.json', atlas=atlas)
        if restored.checkpoint_state() != simulation.checkpoint_state():
            raise ValueError('Atlas checkpoint reconstruction differs')
        summary.update(atlas_differences=delta, atlas_checkpoint_exact=True)
        if any(delta[key] > limits[key] for key in limits):
            summary['status'] = 'atlas_comparison_failed'
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Short joint-integration verification; not sustained flight qualification.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--deadline', type=datetime.fromisoformat)
    parser.add_argument('--screen', action='store_true')
    parser.add_argument('--atlas', type=Path)
    args = parser.parse_args()
    if args.screen:
        from methodology.magnetic_atlas import MagneticFieldAtlas
        screen(args.output, None if args.atlas is None else MagneticFieldAtlas.load(args.atlas))
    elif args.atlas is not None:
        parser.error('Atlas use requires the bounded screen')
    elif args.output.exists() or args.deadline is None or args.deadline.tzinfo is None:
        raise ValueError('Use a fresh output directory and a timezone-aware deadline')
    else:
        args.output.mkdir(parents=True)
        simulation = JointAirborneSimulation(magnetics.ConstructionModel(), Fixed.airborne_check_duration)
        simulation.save_checkpoint(args.output / 'initial_checkpoint.json')
        airborne.run_simulation(simulation, args.output / 'summary.json', args.output / 'checkpoint.json', args.deadline.timestamp())

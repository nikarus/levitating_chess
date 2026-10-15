import argparse
import hashlib
import json
from pathlib import Path
from time import perf_counter
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Inputs, Fixed
from methodology import adaptive_airborne, airborne, airborne_inputs, airborne_verification, electrical, magnetic_atlas, motion, sensing, source_fingerprints
from methodology.cad import magnetics


def handoff_poses(array):
    cases = []
    offset = min(Fixed.active_window_handoff_offsets)
    for axis in range(2):
        for sign in (-1, 1):
            boundary = (sign * (array.window - array.short) / 2 + array.short_pitch / 2) % array.short_pitch - array.short_pitch / 2
            for side in (-1, 1):
                pose = np.zeros(6)
                pose[axis] = boundary + side * offset
                pose[:3] = array.rotation.apply(pose[:3])
                cases.append(dict(name=f'handoff_axis_{axis}_boundary_{sign}_side_{side}', pose=pose,
                    handoff_group=f'{axis}_{sign}', handoff_side=side))
    return cases


def verification_poses(geometry, array, layout):
    cases = [dict(name=f'existing_pose_{index}', pose=pose) for index, pose in enumerate(
        magnetic_atlas.validation_poses(geometry, array, layout))]
    for gap in (geometry.gap, geometry.max_flight_gap):
        for axis in range(2):
            for sign in (-1, 1):
                pose = np.zeros(6)
                pose[2] = gap - geometry.gap
                pose[3 + axis] = sign * levitation_sim.rim_tilt_limit(layout, gap)
                cases.append(dict(name=f'gap_{gap}_tilt_axis_{axis}_sign_{sign}', pose=pose))
    return cases + handoff_poses(array)


def matrix_refinement_metrics(original, refined, current_limit):
    if original.shape != refined.shape or original.ndim != 2 or original.shape[0] != 6:
        raise ValueError('Magnetic matrix comparison requires matching six-axis coil columns')
    difference = refined - original
    metrics = {}
    for name, section in (('force', slice(0, 3)), ('torque', slice(3, 6))):
        scale = np.max(np.linalg.norm(refined[section], axis=0))
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError('Magnetic comparison requires nonzero finite reference authority')
        errors = np.linalg.norm(difference[section], axis=0)
        metrics[name + '_relative_column_error'] = float(np.max(errors) / scale)
        metrics[name + '_error_bound_at_channel_limits'] = float(current_limit * np.sum(errors))
    if not all(np.isfinite(value) for value in metrics.values()):
        raise ValueError('Magnetic comparison produced nonfinite metrics')
    return metrics


def verify_magnetic_envelope(output, handoffs_only=False):
    if output.exists():
        raise ValueError('Physical-verification output exists; preserve its evidence')
    started = perf_counter()
    construction = magnetics.ConstructionModel()
    refinements = (min(Fixed.current_demand_mesh_refinements), max(Fixed.current_demand_mesh_refinements))
    if refinements[0] == refinements[1]:
        raise ValueError('Physical verification requires independent conductor refinements')
    arrays, fields = [], []
    for refinement in refinements:
        geometry, array, _ = construction(Inputs.cad_coil_layout, refinement)
        arrays.append(array)
        fields.append(sensing.HallFieldModel(construction, refinement))
    levitation_sim.use_geometry(geometry)
    layout = levitation_sim.magnet_layout_from_geometry()
    if fields[0].identities != fields[1].identities or not np.array_equal(fields[0].points, fields[1].points):
        raise ValueError('Conductor refinement must preserve Hall positions and physical coil identities')
    field_difference = fields[1].coil_field - fields[0].coil_field
    output.mkdir(parents=True)
    summary = dict(scope='Static direct-field CAD conductor refinement across sampled poses; no controller or time integration',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, magnetic_atlas.__file__, sensing.__file__)),
        refinements=refinements, segment_lengths_mm=[Fixed.winding_magnetic_segment_length / refinement for refinement in refinements],
        comparison_limit=Fixed.qualification_convergence_fraction,
        limit_scope='Existing relative numerical-change screen; not a newly qualified force or global sensor-error budget',
        hall_field=dict(maximum_column_difference_T_per_A=float(np.max(np.abs(field_difference))),
            maximum_field_difference_bound_T=float(Fixed.driver_channel_current * np.max(np.sum(np.abs(field_difference), axis=1))),
            bound_scope='Absolute row-sum bound with every physical channel at its rating; not an actual operating current pattern'),
        status='running', handoffs_only=handoffs_only, physical_envelope_qualified=False, cases=[],
        limitations=['Sampled static configurations do not establish continuous-domain error bounds or moving-trajectory convergence.',
            'Both refinements retain the same filament-centreline approximation, material properties and local terminal closure.',
            'Hall-field error is measured; a global analogue-field acceptance budget remains to be established.'])
    path = output / 'summary.json'
    airborne_verification.save_summary(path, summary)
    cases = handoff_poses(arrays[0]) if handoffs_only else verification_poses(geometry, arrays[0], layout)
    for case in cases:
        matrices, identities = [], []
        for array in arrays:
            array.build(center=case['pose'][:2])
            if not set(array.identities) <= set(fields[0].identities):
                raise ValueError('Verification window exceeds the physical patch')
            _, center = levitation_sim.place_piece(layout, *case['pose'])
            matrices.append(levitation_sim.actuator_matrix(layout, array, center))
            identities.append(array.identities.copy())
        if identities[0] != identities[1]:
            raise ValueError('Conductor refinement changed active coil identity or order')
        metrics = matrix_refinement_metrics(*matrices, Fixed.driver_channel_current)
        passed = all(metrics[key] <= Fixed.qualification_convergence_fraction
            for key in ('force_relative_column_error', 'torque_relative_column_error'))
        summary['cases'].append(dict(case, pose=case['pose'].tolist(), coil_identities=identities[0],
            matrices=[matrix.tolist() for matrix in matrices], metrics=metrics, relative_screen_passed=passed))
        summary.update(elapsed_s=perf_counter() - started)
        airborne_verification.save_summary(path, summary)
        if not passed:
            summary.update(status='relative_refinement_screen_failed')
            airborne_verification.save_summary(path, summary)
            return summary
    groups = {}
    for case in summary['cases']:
        if 'handoff_group' in case:
            groups.setdefault(case['handoff_group'], []).append(set(map(tuple, case['coil_identities'])))
    if not all(len(group) == 2 and group[0] != group[1] for group in groups.values()):
        summary.update(status='handoff_coverage_failed')
        airborne_verification.save_summary(path, summary)
        raise ValueError('Handoff probes did not straddle a coil-window change')
    summary.update(status='completed', handoff_groups_verified=len(groups),
        maxima={key:max(case['metrics'][key] for case in summary['cases']) for key in summary['cases'][0]['metrics']})
    airborne_verification.save_summary(path, summary)
    return summary


def prescribed_vertical_speed(geometry):
    return 2 * (geometry.max_flight_gap - geometry.gap) / Inputs.worst_phase_dwell


def moving_scenarios(geometry, layout):
    gap = (geometry.gap + geometry.max_flight_gap) / 2
    pose = np.zeros(6)
    pose[2] = gap - geometry.gap
    pose[5] = np.radians(Fixed.pose_yaws_deg[len(Fixed.pose_yaws_deg) // 2])
    lateral_velocity = np.zeros(6)
    lateral_velocity[:2] = geometry.flight_speed / np.sqrt(2)
    lateral_velocity[5] = 2 * np.radians(Inputs.target_yaw_angle_deg) / Inputs.target_yaw_time
    tilt = levitation_sim.rim_tilt_limit(layout, gap)
    tilted_pose = pose.copy()
    tilted_pose[3:5] = min(Fixed.pose_tilt_fractions) * tilt / np.sqrt(2)
    tilted_velocity = np.zeros(6)
    tilted_velocity[2] = prescribed_vertical_speed(geometry)
    tilted_velocity[3:5] = 2 * tilt / Inputs.target_tilt_time / np.sqrt(2)
    return [dict(name=name, pose=position.tolist(), world_velocity=velocity.tolist(), duration_s=Fixed.airborne_check_duration)
        for name, position, velocity in (('translation_yaw', pose, lateral_velocity), ('tilt_vertical', tilted_pose, tilted_velocity))]


class PrescribedPhysicalTrial:
    advance_physics = airborne.AirborneSimulation.advance_physics

    def __init__(self, construction, atlas, scenario, step, refinement, inputs=None, full_coil_patch=False):
        self.started = perf_counter()
        self.scenario, self.fixed_step = scenario, step
        self.refinement = refinement
        self.plant = motion.MotionPlant(construction, magnetic_refinement=refinement, full_coil_patch=full_coil_patch)
        self.plant.field_evaluator = atlas.field
        self.reference = np.asarray(scenario['pose'])
        rotation = Rotation.from_euler('xyz', self.reference[3:])
        velocity = np.asarray(scenario['world_velocity'])
        self.plant.reset(position=self.reference[:3] + [0, 0, self.plant.geometry.center_of_mass_height],
            rotation=rotation, velocity=np.r_[velocity[:3], rotation.inv().apply(velocity[3:])])
        self.matrix = self.plant.magnetic_matrix(window_center=self.reference[:2])
        self.identities = tuple(self.plant.array.identities)
        self.drivers = electrical.CoilDrivers(construction, self.identities)
        self.fields = sensing.HallFieldModel(construction, refinement)
        self.field_indices = [self.fields.identities.index(identity) for identity in self.identities]
        if inputs is None:
            inertia = rotation.as_matrix() @ np.diag(self.plant.geometry.inertias) @ rotation.as_matrix().T
            target = np.r_[0, 0, self.plant.geometry.weight, np.cross(velocity[3:], inertia @ velocity[3:])]
            limit = self.drivers.current_limit * (1 - Fixed.qualification_current_reserve_fraction)
            selected = (np.array([identity in set(map(tuple, scenario['selected_before'])) for identity in self.identities])
                if full_coil_patch else np.ones(len(self.identities), dtype=bool))
            peak, initial = levitation_sim.min_peak_current(self.matrix[:, selected], target)
            if peak > limit:
                raise ValueError('Prescribed-input initialization exceeds the reserved current budget')
            currents = np.zeros(len(self.identities))
            currents[selected] = levitation_sim.min_loss_current(self.matrix[:, selected], target, limit, initial_currents=initial)
            base_voltage = self.drivers.resistance * currents + self.matrix.T @ velocity
            pulse = self.drivers.resistance * Fixed.qualification_current_transient_allowance * (-1.) ** np.arange(len(currents))
            if full_coil_patch:
                pulse[:] = 0
            inputs = dict(identities=self.identities, initial_currents_A=currents.tolist(),
                base_voltage_V=base_voltage.tolist(), pulse_voltage_V=pulse.tolist())
            if 'switch_time_s' in scenario:
                after = np.array([identity in set(map(tuple, scenario['selected_after'])) for identity in self.identities])
                peak, initial = levitation_sim.min_peak_current(self.matrix[:, after], target)
                if peak > limit:
                    raise ValueError('Incoming prescribed allocation exceeds the reserved current budget')
                after_currents = np.zeros(len(self.identities))
                after_currents[after] = levitation_sim.min_loss_current(self.matrix[:, after], target, limit, initial_currents=initial)
                inputs.update(switch_time_s=scenario['switch_time_s'], selected_before=scenario['selected_before'],
                    selected_after=scenario['selected_after'],
                    after_voltage_V=(self.drivers.resistance * after_currents + self.matrix.T @ velocity).tolist())
                if np.max(np.abs(inputs['after_voltage_V'])) > self.drivers.voltage_limit:
                    raise ValueError('Incoming prescribed voltage exceeds the configured bus limit')
        if tuple(map(tuple, inputs['identities'])) != self.identities:
            raise ValueError('Prescribed electrical inputs changed physical coil identities')
        self.inputs = inputs
        self.input_identity = hashlib.sha256(json.dumps(inputs, sort_keys=True).encode()).hexdigest()
        self.drivers.currents = np.array(inputs['initial_currents_A'])
        self.drivers.voltage = np.array(inputs['base_voltage_V']) + np.array(inputs['pulse_voltage_V'])
        if max(np.max(np.abs(np.array(inputs['base_voltage_V']) + sign * np.array(inputs['pulse_voltage_V'])))
               for sign in (-1, 1)) > self.drivers.voltage_limit:
            raise ValueError('Prescribed voltage pulse exceeds the configured bus limit')
        self.peaks = dict(back_emf_V=0.)
        self.stop_reason = None
        self.trace = []
        self.last_magnet, self.last_coil = self.physical_fields()
        self.raw_history = [dict(time_s=0., magnet_T=self.last_magnet.tolist(), coil_T=self.last_coil.tolist())]
        self.filtered_magnet, self.filtered_coil = self.last_magnet.copy(), self.last_coil.copy()
        self.source = dict(self.plant.provenance, **source_fingerprints(__file__, airborne.__file__, electrical.__file__,
            sensing.__file__, magnetic_atlas.__file__, airborne_inputs.__file__, airborne_verification.__file__, electrical.qualification.__file__))
        self.atlas_identity = atlas.identity
        self.observe()
        self.record()

    def physical_fields(self):
        truth = self.plant.truth()
        pose = np.r_[truth['com_position_m'] - [0, 0, self.plant.geometry.center_of_mass_height], truth['rotation'].as_euler('xyz')]
        return self.fields.magnet_field(pose), self.fields.coil_field[:, self.field_indices] @ self.drivers.currents

    def observe(self):
        truth = self.plant.truth()
        minimum_height = self.plant.geometry.center_of_mass_height
        maximum_height = minimum_height + (self.plant.geometry.max_flight_gap - self.plant.geometry.gap)
        clearance = motion.bottom_film_clearance(truth['com_position_m'], truth['rotation'], self.plant.geometry.center_of_mass_height,
            self.fields.base_radius, self.plant.metadata['board_top_m'])
        if self.plant.data.ncon or clearance <= 0:
            self.stop_reason = 'contact'
        elif not minimum_height <= truth['com_position_m'][2] <= maximum_height:
            self.stop_reason = 'outside_sampled_gap_range'
        elif np.max(np.abs(self.drivers.currents)) > self.drivers.current_limit:
            self.stop_reason = 'current_limit_exceeded'
        elif not np.isfinite(np.r_[self.plant.integration_state(), self.drivers.currents, self.last_magnet, self.last_coil]).all():
            self.stop_reason = 'nonfinite_physical_state'

    def record(self):
        truth = self.plant.truth()
        self.trace.append(dict(time_s=truth['time_s'],
            pose=np.r_[truth['com_position_m'] - [0, 0, self.plant.geometry.center_of_mass_height], truth['rotation'].as_euler('xyz')].tolist(),
            world_velocity=np.r_[truth['linear_velocity_m_per_s'], truth['angular_velocity_rad_per_s']].tolist(),
            currents_A=self.drivers.currents.tolist(), wrench=(self.matrix @ self.drivers.currents).tolist(),
            analogue_field_T=(self.filtered_magnet + self.filtered_coil).tolist()))

    def advance_interval(self, index):
        start, end = index / Fixed.current_regulator_rate, (index + 1) / Fixed.current_regulator_rate
        self.drivers.voltage = np.array(self.inputs['base_voltage_V']) + (-1.) ** index * np.array(self.inputs['pulse_voltage_V'])
        count = round((end - start) / self.fixed_step)
        if count < 1 or abs(count * self.fixed_step - (end - start)) > Fixed.adaptive_clock_tolerance:
            raise ValueError('Prescribed-input step does not align with voltage boundaries')
        for substep in range(count):
            self.advance_physics(self.fixed_step, self.matrix)
            self.plant.data.time = start + (substep + 1) * self.fixed_step
            self.matrix = self.plant.magnetic_matrix(window_center=self.reference[:2])
            magnet, coil = self.physical_fields()
            self.accept_fields(float(self.plant.data.time), magnet, coil, self.fixed_step)
            self.observe()
            if self.stop_reason:
                break
        self.record()

    def accept_fields(self, time, magnet, coil, period):
        self.filtered_magnet = sensing.hall_filter_step(self.filtered_magnet, self.last_magnet, magnet, period)
        self.filtered_coil = sensing.hall_filter_step(self.filtered_coil, self.last_coil, coil, period,
            self.drivers.inductance / self.drivers.resistance)
        self.last_magnet, self.last_coil = magnet, coil
        self.raw_history.append(dict(time_s=time, magnet_T=magnet.tolist(), coil_T=coil.tolist()))

    def report(self):
        return dict(scope='Prescribed voltage and initial motion; production physical integrator without sensors, observer or regulator feedback',
            passed=False, physical_envelope_qualified=False, scenario=self.scenario, source_sha256=self.source,
            duration_completed_s=float(self.plant.data.time), step_s=self.fixed_step, magnetic_refinement=self.refinement,
            inputs=self.inputs, input_identity=self.input_identity, atlas_identity=self.atlas_identity,
            mechanical_properties=self.plant.metadata, stop_reason=self.stop_reason, trace=self.trace,
            hardware_rates_Hz=dict(prescribed_voltage=Fixed.current_regulator_rate), elapsed_s=perf_counter()-self.started,
            limitations=['Two-millisecond moving fragments with a fixed physical coil window; no handoff or full-move qualification.',
                'Initial currents and holding voltage are calculated once and reused unchanged for both refinements.',
                'Alternating voltage offsets use the existing current-transient allowance; these are a diagnostic stimulus, not PWM modeling.',
                'Ideal analogue Hall filter starts at equilibrium; no sensor noise, calibration mismatch or ADC feedback.',
                'Position, orientation and current use existing convergence limits; velocity, force/torque and analogue-field changes are reported without new acceptance budgets.'])

    def save(self, output, role):
        airborne_verification.save_summary(output / (role + '.json'), self.report())
        airborne_verification.save_summary(output / (role + '_checkpoint.json'), dict(
            scope='Physical diagnostic evidence; not a resumable airborne-controller checkpoint', source_sha256=self.source,
            scenario=self.scenario, input_identity=self.input_identity, atlas_identity=self.atlas_identity,
            plant=self.plant.checkpoint_state(), drivers={name:getattr(self.drivers, name).tolist() for name in airborne.DRIVER_STATE},
            filtered_magnet_T=self.filtered_magnet.tolist(), filtered_coil_T=self.filtered_coil.tolist(),
            raw_history=self.raw_history, stop_reason=self.stop_reason))


class CoilTurnoffTrial(PrescribedPhysicalTrial):
    def __init__(self, construction, atlas, scenario, step, refinement, inputs=None):
        super().__init__(construction, atlas, scenario, step, refinement, inputs, full_coil_patch=True)
        self.enabled = np.array([identity in set(map(tuple, scenario['selected_after'])) for identity in self.identities])
        self.outgoing = np.array([identity in set(map(tuple, scenario['selected_before']))
            and not self.enabled[index] for index, identity in enumerate(self.identities)])
        self.filtered_currents = self.drivers.currents.copy()
        self.turnoff_wrench = (self.matrix[:, self.outgoing] @ self.drivers.currents[self.outgoing]).tolist()
        self.initial_outgoing_current = float(np.max(np.abs(self.drivers.currents[self.outgoing])))
        voltage = self.drivers.terminal_voltage(self.enabled)
        stops = self.drivers.inductance / self.drivers.resistance * electrical.qualification.rl_stopping_exponent(
            self.drivers.currents, (voltage - self.matrix.T @ np.asarray(scenario['world_velocity'])) / self.drivers.resistance, self.enabled)
        self.initial_stop_times = stops[self.outgoing].tolist()
        if not np.isfinite(self.initial_stop_times).all() or self.initial_outgoing_current <= 0:
            raise ValueError('Turnoff diagnostic requires outgoing stored current with finite decay time')

    def integrate_switched_fields(self, period, matrix):
        initial = self.drivers.currents.copy()
        voltage = self.drivers.terminal_voltage(self.enabled)
        back_emf = airborne.AirborneSimulation.advance_physics(self, period, matrix, self.enabled)
        midpoint_current, _ = electrical.qualification.rl_interval(initial, voltage, back_emf,
            self.drivers.resistance, self.drivers.inductance, period / 2, self.enabled)
        self.midpoint_coil = self.fields.coil_field[:, self.field_indices] @ midpoint_current
        self.filtered_currents = sensing.filtered_coil_current_step(self.filtered_currents, initial, voltage, back_emf,
            self.drivers.resistance, self.drivers.inductance, period, self.enabled)
        magnet, coil = self.physical_fields()
        self.filtered_magnet = sensing.hall_filter_step(self.filtered_magnet, self.last_magnet, magnet, period)
        self.filtered_coil = self.fields.coil_field[:, self.field_indices] @ self.filtered_currents
        self.last_magnet, self.last_coil = magnet, coil

    def apply_prescribed_switch(self, time):
        self.drivers.voltage = np.array(self.inputs['base_voltage_V'])

    def advance_interval(self, index):
        start, end = index / Fixed.current_regulator_rate, (index + 1) / Fixed.current_regulator_rate
        self.apply_prescribed_switch(start)
        count = round((end-start) / self.fixed_step)
        if count < 1 or abs(count * self.fixed_step - (end-start)) > Fixed.adaptive_clock_tolerance:
            raise ValueError('Turnoff diagnostic step does not align with observations')
        for substep in range(count):
            self.integrate_switched_fields(self.fixed_step, self.matrix)
            self.plant.data.time = start + (substep+1) * self.fixed_step
            self.matrix = self.plant.magnetic_matrix()
            self.raw_history.append(dict(time_s=float(self.plant.data.time), magnet_T=self.last_magnet.tolist(),
                coil_T=self.last_coil.tolist(), outgoing_current_A=self.drivers.currents[self.outgoing].tolist()))
            self.observe()
            if self.stop_reason:
                break
        self.record()

    def report(self):
        report = super().report()
        report.update(scope='Prescribed coil turnoff from rest with persistent full-patch forces and exact clipped-current Hall filtering',
            selected_before=self.scenario['selected_before'], selected_after=self.scenario['selected_after'],
            outgoing_identities=[identity for identity, outgoing in zip(self.identities,self.outgoing) if outgoing],
            initial_outgoing_current_A=self.initial_outgoing_current,
            initial_outgoing_wrench=self.turnoff_wrench, initial_held_geometry_stop_times_s=self.initial_stop_times,
            final_outgoing_current_A=float(np.max(np.abs(self.drivers.currents[self.outgoing]))),
            switching_state=dict(enabled=self.enabled.tolist(), filtered_currents_A=self.filtered_currents.tolist()))
        report['limitations'] = ['A short prescribed turnoff from rest, not a moving handoff or feedback-controlled transfer.',
            'Uses the existing ideal opposite half-bus clamp until zero; diode drops, PWM, mutual inductance and bus transients remain unmodeled.',
            'Off-channel back-EMF reaching a rail is explicitly rejected; induced diode re-conduction is not modeled.',
            'Full physical CAD patch remains in force and Hall calculations even for unselected coils.',
            'Position, angle and current use existing convergence limits; global Hall-field accuracy remains unqualified.']
        return report



class CoilHandoffTrial(CoilTurnoffTrial):
    def __init__(self, construction, atlas, scenario, step, refinement, inputs=None):
        super().__init__(construction, atlas, scenario, step, refinement, inputs)
        before, after = (set(map(tuple, scenario[key])) for key in ('selected_before', 'selected_after'))
        self.incoming = np.array([identity in after - before for identity in self.identities])
        if not self.incoming.any() or not self.outgoing.any():
            raise ValueError('Moving handoff requires incoming and outgoing physical coils')
        self.midpoint_coil = self.last_coil.copy()
        self.apply_prescribed_switch(0.)

    def apply_prescribed_switch(self, time):
        switched = time >= self.inputs['switch_time_s'] - Fixed.adaptive_clock_tolerance
        selected = set(map(tuple, self.inputs['selected_after' if switched else 'selected_before']))
        self.enabled = np.array([identity in selected for identity in self.identities])
        self.drivers.voltage = np.array(self.inputs['after_voltage_V' if switched else 'base_voltage_V'])

    def report(self):
        report = super().report()
        report.update(scope='Moving diagonal coil handoff with shared prescribed switching time and voltages; no feedback',
            incoming_identities=[identity for identity, incoming in zip(self.identities, self.incoming) if incoming],
            final_incoming_current_A=float(np.max(np.abs(self.drivers.currents[self.incoming]))))
        report['limitations'][0] = 'A short moving boundary crossing with a prescribed event, not position-triggered selection or controlled flight.'
        return report


def moving_handoff_scenario(geometry, array, duration=None):
    probes = handoff_poses(array)
    boundary = sum((np.mean([case['pose'] for case in probes if case['handoff_group'] == group], axis=0)
        for group in ('0_1', '1_-1')))
    velocity = array.rotation.apply(np.array([geometry.flight_speed, geometry.flight_speed, 0.]) / np.sqrt(2))
    switch_time = 1 / Fixed.current_regulator_rate if duration is None else duration / 2
    pose = boundary.copy()
    pose[:3] -= velocity * switch_time
    pose[2] = (geometry.max_flight_gap - geometry.gap) / 2
    selections = []
    for time in (0., 2 * switch_time):
        array.build(center=(pose[:3] + velocity * time)[:2])
        selections.append(array.identities.copy())
    return dict(name='diagonal_coil_handoff', pose=pose.tolist(), world_velocity=np.r_[velocity, np.zeros(3)].tolist(),
        duration_s=2 * switch_time, switch_time_s=switch_time,
        selected_before=selections[0], selected_after=selections[1])



def gap_handoff_scenarios(geometry, array, duration=None):
    scenarios = []
    for name, gap, direction in (('lower_gap', geometry.gap, 1), ('upper_gap', geometry.max_flight_gap, -1)):
        scenario = moving_handoff_scenario(geometry, array, duration)
        scenario.update(name=name, initial_gap_m=gap)
        scenario['pose'][2] = gap - geometry.gap
        scenario['world_velocity'][2] = direction * prescribed_vertical_speed(geometry)
        scenarios.append(scenario)
    return scenarios


def run_prescribed_trial(trial, reference, output, role):
    trial.save(output, role)
    intervals = round(trial.scenario['duration_s'] * Fixed.current_regulator_rate)
    if abs(intervals / Fixed.current_regulator_rate - trial.scenario['duration_s']) > Fixed.adaptive_clock_tolerance:
        raise ValueError('Diagnostic duration does not align with prescribed voltage boundaries')
    comparison = None
    for index in range(intervals):
        if trial.stop_reason:
            break
        trial.advance_interval(index)
        if trial.stop_reason is None and reference is not None:
            if reference['input_identity'] != trial.input_identity:
                raise ValueError('Refinement changed prescribed electrical inputs')
            prefix = dict(reference, trace=reference['trace'][:len(trial.trace)])
            comparison = moving_comparison(prefix, trial.report())
            if not comparison['converged']:
                trial.stop_reason = 'physical_comparison_failed'
        if trial.stop_reason or (index + 1) % round(Fixed.airborne_checkpoint_interval * Fixed.current_regulator_rate) == 0:
            trial.save(output, role)
    trial.save(output, role)
    return comparison


def verify_coil_turnoff(output):
    if output.exists():
        raise ValueError('Turnoff output exists; preserve its evidence')
    construction = magnetics.ConstructionModel()
    atlas = SimpleNamespace(identity=None, field=None)
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    probes = [case for case in handoff_poses(array) if case['handoff_group']=='0_1']
    selections=[]
    for probe in probes:
        array.build(center=probe['pose'][:2])
        selections.append(array.identities.copy())
    if not set(selections[1]) < set(selections[0]):
        raise ValueError('Selected boundary must remove coils without adding new ones')
    pose=np.mean([probe['pose'] for probe in probes],axis=0)
    pose[2]=(geometry.max_flight_gap-geometry.gap)/2
    scenario=dict(name='outgoing_coil_decay',pose=pose.tolist(),world_velocity=np.zeros(6).tolist(),
        duration_s=Fixed.airborne_checkpoint_interval,selected_before=selections[0],selected_after=selections[1])
    output.mkdir(parents=True)
    summary=dict(scope='Persistent physical coils and prescribed current decay; not moving-flight qualification',
        status='running',physical_envelope_qualified=False,cases={})
    trials={}
    for role,step,refinement in (
        ('reference',Fixed.adaptive_reference_step,min(Fixed.current_demand_mesh_refinements)),
        ('time_refined',Fixed.adaptive_reference_step/Fixed.qualification_refinement_factor,min(Fixed.current_demand_mesh_refinements)),
        ('wire_refined',Fixed.adaptive_reference_step/Fixed.qualification_refinement_factor,max(Fixed.current_demand_mesh_refinements))):
        summary['active_case']=role
        airborne_verification.save_summary(output/'summary.json',summary)
        trial=CoilTurnoffTrial(construction,atlas,scenario,step,refinement,
            None if role=='reference' else trials['reference']['inputs'])
        if role=='reference':
            intervals=int(np.ceil(max(trial.initial_stop_times)*Fixed.current_regulator_rate))+1
            scenario['duration_s']=intervals/Fixed.current_regulator_rate
        reference=None if role=='reference' else trials['reference' if role=='time_refined' else 'time_refined']
        comparison=run_prescribed_trial(trial,reference,output,role)
        if not trial.stop_reason and trial.report()['final_outgoing_current_A']!=0:
            trial.stop_reason='outgoing_current_did_not_reach_zero'
        trial.save(output,role)
        trials[role]=trial.report()
        summary['cases'][role]=dict(stop_reason=trial.stop_reason,duration_s=float(trial.plant.data.time),
            elapsed_s=trial.report()['elapsed_s'],comparison=comparison)
        if trial.stop_reason:
            summary.update(status='failed',active_case=None)
            airborne_verification.save_summary(output/'summary.json',summary)
            return summary
    summary.update(status='completed',active_case=None)
    airborne_verification.save_summary(output/'summary.json',summary)
    return summary


class AdaptivePrescribedTrial(PrescribedPhysicalTrial):
    advance = adaptive_airborne.AdaptiveAirborneSimulation.advance
    physical_snapshot = adaptive_airborne.AdaptiveAirborneSimulation.physical_snapshot
    restore_physics = adaptive_airborne.AdaptiveAirborneSimulation.restore_physics
    step_error = adaptive_airborne.AdaptiveAirborneSimulation.step_error
    field_error = adaptive_airborne.AdaptiveAirborneSimulation.field_error

    def __init__(self, construction, atlas, reference):
        super().__init__(construction, atlas, reference['scenario'], None, reference['magnetic_refinement'], reference['inputs'])
        self.initialize_adaptation()

    def initialize_adaptation(self):
        self.duration = self.scenario['duration_s']
        self.clock, self.step = 0., 0
        self.next_command, self.next_regulator, self.next_scan = 0, 0, 0
        self.next_checkpoint = 1
        self.proposed_step = Fixed.adaptive_max_step
        self.last_checkpoint_due, self.finished = False, False
        self.tolerance_scale = 1
        self.sensors = SimpleNamespace(gain=np.ones(len(self.fields.points)))
        self.adaptation = dict(accepted=0, rejected=0, force_evaluations=0, maximum_accepted_error=0., intervals=[])
        self.source.update(source_fingerprints(adaptive_airborne.__file__))

    def force_matrix(self):
        self.adaptation['force_evaluations'] += 1
        return self.plant.magnetic_matrix(window_center=self.reference[:2])

    def endpoint(self):
        magnet, coil = self.physical_fields()
        return dict(truth=self.plant.truth(), currents=self.drivers.currents.copy(), total=magnet + coil, coil=coil)

    def observe_physics(self, truth):
        self.observe()

    def append_endpoint(self, time, endpoint):
        self.accept_fields(time, endpoint['total'] - endpoint['coil'], endpoint['coil'], time - self.raw_history[-1]['time_s'])

    def process_events(self, now, regulator_due, command_due, complete):
        if regulator_due:
            self.drivers.voltage = np.array(self.inputs['base_voltage_V']) + (-1.) ** self.next_regulator * np.array(self.inputs['pulse_voltage_V'])
        if now >= sensing.hall_available_time(self.next_scan / Fixed.pose_feedback_rate, len(self.fields.points)):
            self.next_scan += 1
        self.observe()
        self.finished = bool(self.stop_reason or complete)
        if (regulator_due or self.finished) and now > self.trace[-1]['time_s']:
            self.matrix = self.force_matrix()
            self.record()

    def report(self):
        report = super().report()
        report.update(integration_mode='production_adaptive_step_doubling', adaptation=self.adaptation)
        return report



class AdaptiveCoilHandoffTrial(CoilHandoffTrial):
    advance = AdaptivePrescribedTrial.advance
    initialize_adaptation = AdaptivePrescribedTrial.initialize_adaptation
    force_matrix = AdaptivePrescribedTrial.force_matrix
    observe_physics = AdaptivePrescribedTrial.observe_physics
    step_error = AdaptivePrescribedTrial.step_error

    def __init__(self, construction, atlas, reference):
        super().__init__(construction, atlas, reference['scenario'], None, reference['magnetic_refinement'], reference['inputs'])
        self.initialize_adaptation()

    def advance_physics(self, period, matrix):
        self.integrate_switched_fields(period, matrix)

    def physical_snapshot(self):
        state = adaptive_airborne.AdaptiveAirborneSimulation.physical_snapshot(self)
        state['fields'] = {name: getattr(self, name).copy() for name in
            ('filtered_currents', 'filtered_magnet', 'filtered_coil', 'last_magnet', 'last_coil', 'midpoint_coil')}
        return state

    def restore_physics(self, state):
        adaptive_airborne.AdaptiveAirborneSimulation.restore_physics(self, state)
        for name, value in state['fields'].items():
            setattr(self, name, value.copy())

    def endpoint(self):
        return dict(truth=self.plant.truth(), currents=self.drivers.currents.copy(),
            total=self.last_magnet + self.last_coil, coil=self.last_coil.copy(),
            filtered=self.filtered_magnet + self.filtered_coil, midpoint_coil=self.midpoint_coil.copy())

    def field_error(self, initial, coarse, midpoint, fine, period):
        coarse_mid_magnet = (initial['total'] - initial['coil'] + coarse['total'] - coarse['coil']) / 2
        return float(max(np.max(np.abs(fine['total'] - coarse['total'])),
            np.max(np.abs(midpoint['total'] - coarse['midpoint_coil'] - coarse_mid_magnet)),
            np.max(np.abs(fine['filtered'] - coarse['filtered']))))

    def append_endpoint(self, time, endpoint):
        self.raw_history.append(dict(time_s=time, magnet_T=(endpoint['total'] - endpoint['coil']).tolist(),
            coil_T=endpoint['coil'].tolist(), outgoing_current_A=endpoint['currents'][self.outgoing].tolist()))

    def process_events(self, now, regulator_due, command_due, complete):
        if regulator_due:
            self.apply_prescribed_switch(now)
        if now >= sensing.hall_available_time(self.next_scan / Fixed.pose_feedback_rate, len(self.fields.points)):
            self.next_scan += 1
        self.observe()
        self.finished = bool(self.stop_reason or complete)
        if (regulator_due or self.finished) and now > self.trace[-1]['time_s']:
            self.matrix = self.force_matrix()
            self.record()

    def report(self):
        report = super().report()
        report.update(integration_mode='production_adaptive_step_doubling', adaptation=self.adaptation)
        return report



def run_adaptive_prescribed_trial(trial, reference, output, role):
    trial.save(output, role)
    compared_rows, comparison = 0, None
    while not trial.finished:
        trial.advance()
        if len(trial.trace) != compared_rows and trial.stop_reason is None:
            comparison = moving_comparison(dict(reference, trace=reference['trace'][:len(trial.trace)]), trial.report())
            compared_rows = len(trial.trace)
            if not comparison['converged']:
                trial.stop_reason, trial.finished = 'physical_comparison_failed', True
        if trial.last_checkpoint_due or trial.finished:
            trial.save(output, role)
    trial.save(output, role)
    return comparison


def verify_moving_handoff(output, scenario=None, atlas=None):
    if output.exists():
        raise ValueError('Moving handoff output exists; preserve its evidence')
    construction = magnetics.ConstructionModel()
    if atlas is None:
        atlas = SimpleNamespace(identity=None, field=None)
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    if scenario is None:
        scenario = moving_handoff_scenario(geometry, array)
    output.mkdir(parents=True)
    summary = dict(scope='Moving prescribed handoff, independent time/conductor refinement and adaptive comparison',
        status='running', physical_envelope_qualified=False, cases={})
    trials = {}
    for role, step, refinement in (
        ('reference', Fixed.adaptive_reference_step, min(Fixed.current_demand_mesh_refinements)),
        ('time_refined', Fixed.adaptive_reference_step / Fixed.qualification_refinement_factor, min(Fixed.current_demand_mesh_refinements)),
        ('wire_refined', Fixed.adaptive_reference_step / Fixed.qualification_refinement_factor, max(Fixed.current_demand_mesh_refinements)),
        ('adaptive', None, min(Fixed.current_demand_mesh_refinements))):
        summary['active_case'] = role
        airborne_verification.save_summary(output / 'summary.json', summary)
        reference = None if role == 'reference' else trials['reference' if role == 'time_refined' else 'time_refined']
        if reference is not None:
            construction = airborne_verification.recorded_construction(reference)
        if role == 'adaptive':
            trial = AdaptiveCoilHandoffTrial(construction, atlas, reference)
        else:
            trial = CoilHandoffTrial(construction, atlas, scenario, step, refinement,
                None if role == 'reference' else trials['reference']['inputs'])
        if role != 'reference' and (trial.input_identity != reference['input_identity']
                or json.loads(json.dumps(trial.plant.metadata)) != json.loads(json.dumps(reference['mechanical_properties']))):
            raise ValueError('Handoff refinement changed prescribed inputs or CAD properties')
        if role == 'adaptive':
            comparison = run_adaptive_prescribed_trial(trial, reference, output, role)
        else:
            comparison = run_prescribed_trial(trial, reference, output, role)
        if not trial.stop_reason and (trial.report()['final_outgoing_current_A'] != 0
                or trial.report()['final_incoming_current_A'] <= 0):
            trial.stop_reason = 'handoff_current_coverage_failed'
        array.build(center=trial.trace[-1]['pose'][:2])
        crossed_boundary = set(array.identities) == set(map(tuple, scenario['selected_after']))
        if not trial.stop_reason and not crossed_boundary:
            trial.stop_reason = 'geometric_handoff_not_crossed'
        trial.save(output, role)
        trials[role] = trial.report()
        summary['cases'][role] = dict(stop_reason=trial.stop_reason, duration_s=float(trial.plant.data.time),
            elapsed_s=trial.report()['elapsed_s'], comparison=comparison, geometric_handoff_crossed=crossed_boundary)
        if trial.stop_reason:
            summary.update(status='failed', active_case=None)
            airborne_verification.save_summary(output / 'summary.json', summary)
            return summary
    summary.update(status='completed', active_case=None)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary



def verify_gap_handoffs(output):
    if output.exists():
        raise ValueError('Gap handoff output exists; preserve its evidence')
    construction = magnetics.ConstructionModel()
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    output.mkdir(parents=True)
    summary = dict(scope='Prescribed moving handoffs starting at both gap endpoints with inward vertical velocity',
        status='running', physical_envelope_qualified=False, cases={}, active_case=None,
        source_sha256=source_fingerprints(__file__), gap_range_m=[geometry.gap, geometry.max_flight_gap])
    for scenario in gap_handoff_scenarios(geometry, array):
        name = scenario['name']
        summary['active_case'] = name
        airborne_verification.save_summary(output / 'summary.json', summary)
        result = verify_moving_handoff(output / name, scenario)
        summary['cases'][name] = dict(status=result['status'], evidence=f'{name}/summary.json')
        if result['status'] != 'completed':
            summary.update(status='failed', active_case=None)
            airborne_verification.save_summary(output / 'summary.json', summary)
            return summary
    summary.update(status='completed', active_case=None)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


def verify_adaptive_moving(output, atlas_path, reference_folder):
    if output.exists():
        raise ValueError('Adaptive moving output exists; preserve its evidence')
    reference_summary_path = reference_folder / 'summary.json'
    reference_summary = json.loads(reference_summary_path.read_text())
    if reference_summary['status'] != 'completed':
        raise ValueError('Adaptive comparison requires completed fixed-step moving verification')
    references = {}
    root = Path(__file__).resolve().parent.parent
    for name in reference_summary['cases']:
        path = reference_folder / name / 'time_refined.json'
        reference = json.loads(path.read_text())
        if (reference['stop_reason'] or reference['duration_completed_s'] != reference['scenario']['duration_s']
                or reference['step_s'] != Fixed.adaptive_reference_step / Fixed.qualification_refinement_factor):
            raise ValueError('Moving reference is incomplete or has a different reference step')
        for filename, digest in reference['source_sha256'].items():
            if filename != Path(__file__).resolve().relative_to(root).as_posix() and hashlib.sha256((root / filename).read_bytes()).hexdigest() != digest:
                raise ValueError(f'Moving reference physical dependency changed: {filename}')
        references[name] = reference
    construction = airborne_verification.recorded_construction(next(iter(references.values())))
    atlas = magnetic_atlas.MagneticFieldAtlas.load(atlas_path)
    output.mkdir(parents=True)
    summary = dict(scope='Production adaptive integration against saved moving fine-step physical references; no controller feedback',
        status='running', physical_envelope_qualified=False,
        reused_evidence_sha256=source_fingerprints(reference_summary_path,
            *(reference_folder / name / 'time_refined.json' for name in references)),
        reuse_scope='Physical dependencies unchanged; diagnostic runner gains an adaptive adapter and shares its unchanged field-filter calculation.', cases={})
    for name, reference in references.items():
        summary['active_case'] = name
        airborne_verification.save_summary(output / 'summary.json', summary)
        trial = AdaptivePrescribedTrial(construction, atlas, reference)
        if (trial.input_identity != reference['input_identity'] or trial.atlas_identity != reference['atlas_identity']
                or json.loads(json.dumps(trial.plant.metadata)) != json.loads(json.dumps(reference['mechanical_properties']))):
            raise ValueError('Adaptive comparison changed prescribed inputs, atlas or CAD properties')
        comparison = run_adaptive_prescribed_trial(trial, reference, output, name)
        summary['cases'][name] = dict(stop_reason=trial.stop_reason, duration_s=trial.clock,
            elapsed_s=trial.report()['elapsed_s'], comparison=comparison,
            adaptation={key:value for key,value in trial.adaptation.items() if key != 'intervals'})
        if trial.stop_reason:
            summary.update(status='failed', active_case=None)
            airborne_verification.save_summary(output / 'summary.json', summary)
            return summary
        airborne_verification.save_summary(output / 'summary.json', summary)
    summary.update(status='completed', active_case=None)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


def moving_comparison(reference, candidate):
    result = airborne_inputs.physical_comparison(reference, candidate)
    for key in ('world_velocity', 'wrench', 'analogue_field_T'):
        difference = np.array([row[key] for row in reference['trace']]) - np.array([row[key] for row in candidate['trace']])
        if key == 'analogue_field_T':
            result[key + '_maximum_difference'] = float(np.max(np.abs(difference)))
        else:
            result[key + '_maximum_component_differences'] = np.max(np.abs(difference), axis=0).tolist()
    return result


def verify_moving_inputs(output, atlas_path):
    if output.exists():
        raise ValueError('Moving verification output exists; preserve its evidence')
    construction = magnetics.ConstructionModel()
    atlas = magnetic_atlas.MagneticFieldAtlas.load(atlas_path)
    geometry, _, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    levitation_sim.use_geometry(geometry)
    scenarios = moving_scenarios(geometry, levitation_sim.magnet_layout_from_geometry())
    output.mkdir(parents=True)
    summary = dict(scope='Short moving prescribed-input physical verification; no controller development',
        status='running', physical_envelope_qualified=False, cases={})
    for scenario in scenarios:
        folder = output / scenario['name']
        folder.mkdir()
        rows = summary['cases'][scenario['name']] = {}
        trials = {}
        for role, step, refinement in (
                ('reference', Fixed.adaptive_reference_step, min(Fixed.current_demand_mesh_refinements)),
                ('time_refined', Fixed.adaptive_reference_step / Fixed.qualification_refinement_factor, min(Fixed.current_demand_mesh_refinements)),
                ('wire_refined', Fixed.adaptive_reference_step / Fixed.qualification_refinement_factor, max(Fixed.current_demand_mesh_refinements))):
            summary['active_case'] = [scenario['name'], role]
            airborne_verification.save_summary(output / 'summary.json', summary)
            inputs = None if role == 'reference' else trials['reference']['inputs']
            trial = PrescribedPhysicalTrial(construction, atlas, scenario, step, refinement, inputs)
            reference = None if role == 'reference' else trials['reference' if role == 'time_refined' else 'time_refined']
            comparison = run_prescribed_trial(trial, reference, folder, role)
            trials[role] = trial.report()
            rows[role] = dict(stop_reason=trial.stop_reason, elapsed_s=trials[role]['elapsed_s'],
                duration_s=trials[role]['duration_completed_s'], comparison=comparison)
            if trial.stop_reason:
                summary.update(status='failed', active_case=None)
                airborne_verification.save_summary(output / 'summary.json', summary)
                return summary
            airborne_verification.save_summary(output / 'summary.json', summary)
    summary.update(status='completed', active_case=None)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Verify physical-model accuracy with the controller disconnected.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--handoffs-only', action='store_true')
    parser.add_argument('--moving-atlas', type=Path)
    parser.add_argument('--adaptive-reference', type=Path)
    parser.add_argument('--turnoff', action='store_true')
    parser.add_argument('--moving-handoff', action='store_true')
    parser.add_argument('--gap-handoffs', action='store_true')
    args = parser.parse_args()
    if args.gap_handoffs:
        if args.moving_handoff or args.turnoff or args.moving_atlas or args.adaptive_reference or args.handoffs_only:
            parser.error('Gap handoffs are a separate direct-field verification mode')
        verify_gap_handoffs(args.output)
    elif args.moving_handoff:
        if args.turnoff or args.moving_atlas or args.adaptive_reference or args.handoffs_only:
            parser.error('Moving handoff is a separate direct-field verification mode')
        verify_moving_handoff(args.output)
    elif args.turnoff:
        if args.moving_atlas or args.adaptive_reference or args.handoffs_only:
            parser.error('Turnoff verification uses the direct full-patch field and is a separate mode')
        verify_coil_turnoff(args.output)
    elif args.moving_atlas:
        if args.handoffs_only:
            parser.error('Moving verification and static handoff-only verification are separate modes')
        if args.adaptive_reference:
            verify_adaptive_moving(args.output, args.moving_atlas, args.adaptive_reference)
        else:
            verify_moving_inputs(args.output, args.moving_atlas)
    else:
        if args.adaptive_reference or args.turnoff:
            parser.error('Dynamic verification requires --moving-atlas')
        verify_magnetic_envelope(args.output, args.handoffs_only)

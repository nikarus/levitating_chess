import argparse
import cProfile
import hashlib
import json
from pathlib import Path
from time import perf_counter
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from scipy.optimize import linprog
from scipy.integrate import OdeSolution, solve_ivp
from scipy.spatial.transform import Rotation
from scipy.sparse import diags

import levitation_sim
from model import Inputs, Fixed, Constants
from methodology import airborne_verification, coupling_verification, electrical, joint_airborne, motion, physics_verification, sensing, source_fingerprints
from methodology.cad import magnetics


def analogue_field_budget():
    samples, correlation, quantum = sensing.hall_readout()
    noise = Fixed.hall_output_noise * np.sqrt(correlation.mean())
    return dict(limit_T=float(Fixed.qualification_convergence_fraction * noise),
        nominal_scan_noise_rms_T=float(noise), numerical_fraction=Fixed.qualification_convergence_fraction,
        samples_per_channel=samples, adc_quantum_T=float(quantum),
        rationale='Allocate the existing numerical-error fraction of modeled scan-averaged analogue noise to total deterministic numerical field error; no ADC-code matching.',
        scope='Engineering accuracy allocation under the nominal sensor assumptions, not a probabilistic bound or hardware noise specification.')


def current_box_bound(difference, current_limit):
    if difference.ndim != 2 or 0 in difference.shape or not np.isfinite(difference).all() or not np.isfinite(current_limit) or current_limit <= 0:
        raise ValueError('Field bound requires a finite sensor/channel matrix and a positive current limit')
    channel_bounds = current_limit * np.sum(np.abs(difference), axis=1)
    worst = int(np.argmax(channel_bounds))
    pattern = current_limit * np.sign(difference[worst])
    return dict(maximum_T=float(channel_bounds[worst]), per_sensor_T=channel_bounds.tolist(),
        worst_sensor=worst, attaining_currents_A=pattern.tolist(),
        attained_T=float(abs(difference[worst] @ pattern)))


def cad_field_quadrature(path, points, order, return_depth):
    positions, elements = magnetics.curved_conductor_samples(path, order, return_depth)
    return sampled_conductor_field(points, positions, elements)


def sampled_conductor_field(points, positions, elements):
    fields = []
    for point in points:
        displacement = point - positions
        distances = np.linalg.norm(displacement, axis=1)
        if np.any(distances == 0):
            raise ValueError('Field reference cannot evaluate on its conductor')
        fields.append(Constants.vacuum_permeability / (4 * np.pi) *
            np.sum(np.cross(elements, displacement) / distances[:, None] ** 3, axis=0))
    return np.asarray(fields)


def cad_coil_field_reference(construction, array, points, order):
    local_points = array.rotation.inv().apply(points)
    translated = np.concatenate([local_points - [placement['x'], placement['y'], placement['top'] - array.height]
        for placement in array.placements])
    field = cad_field_quadrature(construction.path, translated, order, construction.return_depth)
    return field[:, 2].reshape(len(array.coils), len(points)).T


def verify_cad_reference(record, output):
    if output.exists():
        raise ValueError('Accuracy evidence exists; preserve its results')
    retained = json.loads(record.read_text())
    construction = magnetics.ConstructionModel()
    _, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.coupling_refinements))
    array.window = Fixed.cad_patch_cells_per_side * array.short_pitch
    array.build()
    if tuple(map(tuple, retained['coil_identities'])) != tuple(array.identities):
        raise ValueError('Field reference coil identities differ from retained maps')
    current_sources = magnetics.fingerprints()
    if any(retained['source_sha256'][name] != digest for name, digest in current_sources.items()):
        raise ValueError('CAD field reference requires matching physical source identities')
    points = np.asarray(retained['sensor_points_m'])
    output.mkdir(parents=True)
    summary = dict(status='running', physical_envelope_qualified=False, budget=analogue_field_budget(),
        scope='Independent Biot-Savart quadrature on the curved CAD centerline and the same ideal return path; fixed sensor geometry, all bounded current combinations.',
        source_sha256=dict(current_sources, **source_fingerprints(__file__, sensing.__file__)),
        input_sha256=source_fingerprints(record), orders=Fixed.coupling_flux_orders, cases=[],
        limitations=['Reference quadrature convergence is empirical, not a certified remainder bound.',
            'The filament, ideal return path, material and fixed construction assumptions remain shared with production.'])
    started = perf_counter()
    for order in Fixed.coupling_flux_orders:
        summary['active_order'] = order
        airborne_verification.save_summary(output / 'summary.json', summary)
        matrix = cad_coil_field_reference(construction, array, points, order)
        summary['cases'].append(dict(order=order, coil_field_T_per_A=matrix.tolist()))
    reference = np.asarray(summary['cases'][-1]['coil_field_T_per_A'])
    summary['reference_changes'] = [dict(coarse_order=a['order'], fine_order=b['order'],
        bound=current_box_bound(np.asarray(a['coil_field_T_per_A']) - b['coil_field_T_per_A'], Fixed.driver_channel_current))
        for a, b in zip(summary['cases'][:-1], summary['cases'][1:])]
    summary['production_errors'] = [dict(refinement=case['refinement'],
        bound=current_box_bound(np.asarray(case['coil_field_T_per_A']) - reference, Fixed.driver_channel_current))
        for case in retained['cases']]
    nominal = next(row for row in summary['production_errors'] if row['refinement'] == Fixed.hall_conductor_refinement)
    summary['nominal_refinement'] = Fixed.hall_conductor_refinement
    summary['nominal_map_meets_reference_budget'] = nominal['bound']['maximum_T'] <= summary['budget']['limit_T']
    summary.update(status='completed', active_order=None, elapsed_s=perf_counter()-started)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


def verify_field_budget(output):
    if output.exists():
        raise ValueError('Accuracy evidence exists; preserve its results')
    output.mkdir(parents=True)
    summary = dict(scope='Stationary coil-to-Hall map convergence and all-channel current-box bound; no feedback or motion integration',
        status='running', physical_envelope_qualified=False, budget=analogue_field_budget(),
        current_limit_A=Fixed.driver_channel_current, refinements=Fixed.coupling_refinements,
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, sensing.__file__)),
        cases=[], comparisons=[], limitations=[
            'The induced infinity norm bounds differences between discretized maps, not their unknown error against the continuous CAD wire.',
            'All channels at their rating is a conservative superset of selected drive patterns; the attaining pattern need not be commanded by the controller.',
            'Coil geometry and Hall positions are fixed. Magnet-field interpolation and temporal/filter reconstruction need separate allocations.',
            'The filter and sample averaging have nonnegative unit-or-smaller gain for bounded input error; this transports the static coil-map difference bound, not other errors.'])
    construction = magnetics.ConstructionModel()
    started = perf_counter()
    fields = []
    for refinement in Fixed.coupling_refinements:
        summary['active_refinement'] = refinement
        airborne_verification.save_summary(output / 'summary.json', summary)
        model = sensing.HallFieldModel(construction, refinement)
        if fields and (model.identities != fields[0].identities or not np.array_equal(model.points, fields[0].points)):
            raise ValueError('Conductor refinement changed sensor positions or physical coil identities')
        fields.append(model)
        summary['cases'].append(dict(refinement=refinement, coil_field_T_per_A=model.coil_field.tolist()))
        if len(fields) > 1:
            difference = model.coil_field - fields[-2].coil_field
            bound = current_box_bound(difference, Fixed.driver_channel_current)
            summary['comparisons'].append(dict(coarse_refinement=Fixed.coupling_refinements[len(fields)-2],
                fine_refinement=refinement, bound=bound, below_total_field_budget=bound['maximum_T'] <= summary['budget']['limit_T']))
        airborne_verification.save_summary(output / 'summary.json', summary)
    finest = fields[-1].coil_field
    summary['against_finest'] = [dict(refinement=refinement, bound=current_box_bound(field.coil_field - finest, Fixed.driver_channel_current))
        for refinement, field in zip(Fixed.coupling_refinements[:-1], fields[:-1])]
    summary['sensor_points_m'] = fields[0].points.tolist()
    summary['coil_identities'] = fields[0].identities
    summary['consecutive_difference_ratios'] = [a['bound']['maximum_T'] / b['bound']['maximum_T']
        for a, b in zip(summary['comparisons'][:-1], summary['comparisons'][1:])]
    summary.update(status='completed', active_refinement=None, elapsed_s=perf_counter()-started,
        nominal_refinement=Fixed.hall_conductor_refinement,
        nominal_map_meets_reference_budget=next(row for row in summary['against_finest']
            if row['refinement'] == Fixed.hall_conductor_refinement)['bound']['maximum_T'] <= summary['budget']['limit_T'])
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


def assess_force_accumulation(record, coupling_record, output):
    if output.exists():
        raise ValueError('Accuracy evidence exists; preserve its results')
    retained = json.loads(record.read_text())
    coupling = json.loads(coupling_record.read_text())['checks']['smooth_coupling']
    mass = coupling['mass_kg']
    horizon = Fixed.airborne_hover_duration
    position_limit = Fixed.qualification_position_error * Fixed.qualification_convergence_fraction
    limit = Fixed.driver_channel_current * (1 - Fixed.qualification_current_reserve_fraction)
    target = np.array([0., 0., mass * Constants.gravity, 0., 0., 0.])
    rows = []
    for case in retained['cases']:
        coarse, fine = np.asarray(case['matrices'])
        scale = np.max(np.abs(coarse), axis=1)
        if np.any(scale == 0):
            raise ValueError('Force allocation screen requires authority in every wrench component')
        difference = fine - coarse
        for sign in (-1, 1):
            allocation = linprog(sign * difference[2], A_eq=coarse / scale[:, None], b_eq=target / scale,
                bounds=[(-limit, limit)] * coarse.shape[1], method='highs')
            if not allocation.success:
                raise ValueError('Recorded force-map screen could not allocate nominal static support')
            residual = coarse @ allocation.x - target
            if np.max(np.abs(residual) / scale) > Fixed.motion_verification_tolerance:
                raise ValueError('Nominal support allocation has an excessive residual')
            wrench_difference = difference @ allocation.x
            discrepancy = abs(float(wrench_difference[2]))
            rows.append(dict(name=case['name'], pose=case['pose'], sign=sign,
                coil_identities=case['coil_identities'], currents_A=allocation.x.tolist(),
                nominal_support_residual=residual.tolist(), wrench_difference=wrench_difference.tolist(),
                held_vertical_displacement_m=discrepancy * horizon ** 2 / (2 * mass)))
    worst = max(rows, key=lambda row: row['held_vertical_displacement_m'])
    summary = dict(status='completed', physical_envelope_qualified=False,
        scope='Recorded static force-map analysis: feasible nominal gravity support maximizing vertical coarse/fine force disagreement. No motion or control simulation.',
        input_sha256=source_fingerprints(record, coupling_record), source_sha256=source_fingerprints(__file__),
        recorded_force_sources=retained['source_sha256'], recorded_mass_sources=json.loads(coupling_record.read_text())['source_sha256'],
        mass_kg=mass, horizon_s=horizon, position_limit_m=position_limit, current_limit_A=limit,
        force_allowance_if_discrepancy_were_held_N=2 * mass * position_limit / horizon ** 2,
        segment_lengths_mm=retained['segment_lengths_mm'], cases=rows, worst_case=worst['name'],
        maximum_held_vertical_displacement_m=worst['held_vertical_displacement_m'],
        held_discrepancy_screen_passed=worst['held_vertical_displacement_m'] <= position_limit,
        limitations=['The held-discrepancy displacement is an illustrative constant-force calculation, not a prediction or bound for a moving trajectory.',
            'The reference is a finer discretization, not exact magnetic truth. Only retained configurations are analyzed.',
            'Adversarial allocations obey static support and reserved channel limits, but need not be chosen by the loss-minimizing controller and are not thermally qualified.',
            'This exposes the missing force-to-trajectory budget; it neither proves a real-world control failure nor supplies the variable-geometry error amplification bound.'])
    output.mkdir(parents=True)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


def verify_curved_force(output):
    if output.exists():
        raise ValueError('Accuracy evidence exists; preserve its results')
    output.mkdir(parents=True)
    construction = magnetics.ConstructionModel()
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    layout = levitation_sim.magnet_layout_from_geometry()
    mass = construction.piece_properties['mass_g'] / 1000
    horizon = Fixed.airborne_hover_duration
    limits = dict(force=2 * mass * Fixed.qualification_position_error * Fixed.qualification_convergence_fraction / horizon ** 2,
        torque=2 * min(geometry.inertias) * Fixed.qualification_angle_error * Fixed.qualification_convergence_fraction / horizon ** 2)
    summary = dict(status='running', physical_envelope_qualified=False,
        scope='Curved CAD conductor force quadrature: retained static mechanisms and independent curved-surface flux derivatives; no motion or feedback.',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, coupling_verification.__file__, physics_verification.__file__)),
        orders=Fixed.coupling_flux_orders, horizon_s=horizon, held_discrepancy_limits=limits,
        limit_scope='Force/torque discrepancy which, if held without amplification, would consume the entire existing trajectory error allowance. A screen, not a global dynamics bound.',
        cases=[], limitations=['Quadrature convergence is empirical and sampled in pose; no continuous-domain remainder bound.',
            'CAD centerline, ideal return, material assumptions and the permanent-magnet field library remain shared.',
            'The flux reference avoids the Lorentz force/torque formula but shares the curved-path quadrature; its surface and derivative resolutions are refined separately.'])
    started = perf_counter()
    samples = {order: magnetics.curved_conductor_samples(construction.path, order, construction.return_depth)
        for order in Fixed.coupling_flux_orders}
    for case in physics_verification.verification_poses(geometry, array, layout):
        summary['active_case'] = case['name']
        airborne_verification.save_summary(output / 'summary.json', summary)
        array.build(center=case['pose'][:2])
        _, center = levitation_sim.place_piece(layout, *case['pose'])
        matrices, elapsed, counts = [], [], []
        for order in Fixed.coupling_flux_orders:
            quadrature = magnetics.curved_force_quadrature(array, samples[order])
            before = perf_counter()
            matrices.append(quadrature.matrix(layout.collection, center))
            elapsed.append(perf_counter() - before)
            counts.append(len(quadrature.points))
        comparisons = [physics_verification.matrix_refinement_metrics(matrix, matrices[-1], Fixed.driver_channel_current)
            for matrix in matrices[:-1]]
        passes = [all(metrics[key + '_error_bound_at_channel_limits'] <= limits[key] for key in limits) for metrics in comparisons]
        baseline = levitation_sim.CoilForceQuadrature(array.coils)
        before = perf_counter()
        baseline_matrix = baseline.matrix(layout.collection, center)
        baseline_time = perf_counter() - before
        summary['cases'].append(dict(name=case['name'], pose=case['pose'].tolist(), coil_identities=array.identities,
            matrices=[matrix.tolist() for matrix in matrices], points=counts, evaluation_seconds=elapsed,
            comparisons_to_finest=comparisons, held_discrepancy_screen_passed=passes,
            baseline=dict(points=len(baseline.points), evaluation_seconds=baseline_time,
                comparison_to_finest=physics_verification.matrix_refinement_metrics(baseline_matrix, matrices[-1], Fixed.driver_channel_current))))
        airborne_verification.save_summary(output / 'summary.json', summary)
        if not passes[-1]:
            summary.update(status='force_accuracy_screen_failed', elapsed_s=perf_counter()-started)
            airborne_verification.save_summary(output / 'summary.json', summary)
            return summary
    pose = np.r_[np.zeros(3), np.radians(Fixed.motion_check_orientation_deg)]
    pose[2] = (geometry.max_flight_gap - geometry.gap) / 2
    array.build(center=pose[:2])
    _, center = levitation_sim.place_piece(layout, *pose)
    quadrature = magnetics.curved_force_quadrature(array, samples[max(Fixed.coupling_flux_orders)])
    matrix = quadrature.matrix(layout.collection, center)
    index = int(np.argmax(np.linalg.norm(matrix[:3], axis=0)))
    begin, end = quadrature.filament_bounds[index:index+2]
    points, elements = quadrature.points[begin:end], quadrature.vectors[begin:end]
    wire = array.coils[index][0]
    anchor = wire.orientation.apply(wire.vertices[0]) + wire.position
    def flux(source, filament, order):
        return filament.current * coupling_verification.curved_surface_flux(source, points, elements, anchor, order)
    summary['active_case'] = 'independent_curved_flux'
    airborne_verification.save_summary(output / 'summary.json', summary)
    fluxes = [coupling_verification.flux_wrench(layout.collection, wire, center, order, max(Fixed.coupling_refinements), flux)
        for order in Fixed.coupling_flux_orders]
    derivative_check = coupling_verification.flux_wrench(layout.collection, wire, center, max(Fixed.coupling_flux_orders),
        Fixed.coupling_refinements[-2], flux)
    flux_metrics = [physics_verification.matrix_refinement_metrics(left[:, None], fluxes[-1][:, None], Fixed.driver_channel_current)
        for left in (matrix[:, index], fluxes[-2], derivative_check)]
    flux_pass = all(metric[key + '_error_bound_at_channel_limits'] <= limits[key] for metric in flux_metrics for key in limits)
    summary['flux_reference'] = dict(pose=pose.tolist(), coil_identity=array.identities[index], line_wrench_per_A=matrix[:, index].tolist(),
        flux_wrenches_per_A=[value.tolist() for value in fluxes], derivative_refined_from=derivative_check.tolist(),
        metrics=flux_metrics, passed=flux_pass)
    summary.update(status='completed' if flux_pass else 'flux_reference_failed', active_case=None,
        passing_orders=[order for index, order in enumerate(Fixed.coupling_flux_orders[:-1])
            if all(case['held_discrepancy_screen_passed'][index] for case in summary['cases'])], elapsed_s=perf_counter()-started)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


def reference_filtered_scan(field_at, start, channels, breakpoints, method):
    samples, _, _ = sensing.hall_readout()
    adc_times = start + Fixed.hall_power_settle_time + np.arange(channels * samples) / Fixed.hall_adc_sample_rate
    boundaries = np.unique([start, *[time for time in breakpoints if start < time < adc_times[-1]], adc_times[-1]])
    values = np.empty((len(adc_times), 2 * channels))
    state = np.zeros(2 * channels)
    scale = Fixed.hall_output_noise
    rate = 2 * np.pi * Fixed.hall_sensor_bandwidth
    for left, right in zip(boundaries[:-1], boundaries[1:]):
        options = dict(jac=diags(np.full(2 * channels, -rate), format='csc')) if method == 'Radau' else {}
        solution = solve_ivp(lambda time, value: rate * (field_at(time) / scale - value),
            (left, right), state, method=method, rtol=Fixed.coupling_reference_tolerance,
            atol=Fixed.coupling_reference_tolerance, dense_output=True, **options)
        if not solution.success or solution.t[-1] != right:
            raise ValueError('Independent Hall-filter reference failed')
        selected = (adc_times >= left) & (adc_times <= right)
        if np.any(selected):
            values[selected] = solution.sol(adc_times[selected]).T * scale
        state = solution.y[:, -1]
    sensor = np.repeat(np.arange(channels), samples)
    return np.stack([values[np.arange(len(adc_times)), sensor + section * channels].reshape(channels, samples).mean(axis=1)
        for section in range(2)])


def sensor_spatial_budget(construction, array, fields):
    reference_maps = [cad_coil_field_reference(construction, array, fields.points, order) for order in Fixed.coupling_flux_orders[-2:]]
    map_error = current_box_bound(fields.coil_field - reference_maps[-1], Fixed.driver_channel_current)
    map_reference_error = current_box_bound(reference_maps[-2] - reference_maps[-1], Fixed.driver_channel_current)
    spatial = (1 + Fixed.hall_calibrated_gain_bound) * (1 + Fixed.coil_field_subtraction_error) * (
        np.asarray(map_error['per_sensor_T']) + map_reference_error['per_sensor_T'])
    return dict(production_refinement=fields.magnetic_refinement, reference_orders=Fixed.coupling_flux_orders[-2:],
        production_T_per_A=fields.coil_field.tolist(), reference_T_per_A=[value.tolist() for value in reference_maps],
        error=map_error, reference_disagreement=map_reference_error, calibrated_bound_T=spatial.tolist())


def sensor_scan_comparison(actual, references, spatial, budget, superposition_error=0):
    gain = 1 + Fixed.hall_calibrated_gain_bound
    coil_gain = 1 + Fixed.coil_field_subtraction_error
    difference = np.abs(actual - references[-1])
    reference_difference = np.abs(references[0] - references[-1])
    temporal = gain * (difference[0] + coil_gain * difference[1] + coil_gain * superposition_error)
    reference_bound = gain * (reference_difference[0] + coil_gain * reference_difference[1])
    combined = spatial + temporal + reference_bound
    reference_passed = bool(np.max(reference_bound) <= budget['limit_T'] * Fixed.coupling_relative_tolerance)
    return dict(actual_components_T=actual.tolist(), reference_components_T=references[-1].tolist(),
        temporal_and_reconstruction_T=temporal.tolist(), reference_disagreement_T=reference_bound.tolist(),
        combined_bound_T=combined.tolist(), reference_passed=reference_passed,
        passed=bool(np.max(combined) <= budget['limit_T'] and reference_passed))


def verify_sensor_budget(record, output):
    if output.exists():
        raise ValueError('Accuracy evidence exists; preserve its results')
    retained = json.loads(record.read_text())
    if retained['status'] != 'completed' or not retained['checks']['variable_coupling']['passed']:
        raise ValueError('Sensor budget requires passing independent coupling evidence')
    if retained['source_sha256'] != source_fingerprints(*retained['source_sha256']):
        raise ValueError('Coupling evidence has different calculation sources')
    output.mkdir(parents=True)
    started = perf_counter()
    construction = magnetics.ConstructionModel()
    fields = sensing.HallFieldModel(construction)
    _, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    driven_identity = array.identities[0]
    array.window = Fixed.cad_patch_cells_per_side * array.short_pitch
    array.build()
    if tuple(array.identities) != fields.identities:
        raise ValueError('Independent coil reference and sensor patch identities differ')
    summary = dict(status='running', physical_envelope_qualified=False, budget=analogue_field_budget(),
        scope='Combined coil-map, temporal trajectory and scan/filter reconstruction error for the independent manufactured coupling benchmark; direct magnet fields, no feedback or atlas.',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, sensing.__file__, coupling_verification.__file__)),
        input_sha256=source_fingerprints(record), cases=[],
        limitations=['Independent ODE and conductor refinement disagreements are empirical reference checks, not certified remainder bounds.',
            'Manufactured force equations isolate numerical mechanisms; this is not a CAD-force flight or a continuous-domain trajectory bound.',
            'Magnetic calibration and gain factors transport numerical bounds; physical noise is not used to cancel numerical error.',
            'No atlas is used. Cached-field accuracy must be allocated and verified before enabling it.'])
    path = output / 'summary.json'
    airborne_verification.save_summary(path, summary)
    summary['coil_map'] = sensor_spatial_budget(construction, array, fields)
    spatial = np.asarray(summary['coil_map']['calibrated_bound_T'])
    airborne_verification.save_summary(path, summary)
    column = fields.coil_field[:, fields.identities.index(driven_identity)]

    def field_components(state):
        pose = np.array([0., 0., (fields.geometry.max_flight_gap - fields.geometry.gap) / 2 + state[0], 0., 0., state[2]])
        return np.r_[fields.magnet_field(pose), column * state[4]]

    for case in retained['checks']['variable_coupling']['cases']:
        if case['fixed_step_s'] is not None or case['tolerance_scale'] != 1:
            continue
        parameters = case['parameters']
        trial = SimpleNamespace(mass=parameters['mass_kg'], inertia=parameters['inertia_kg_m2'], span=parameters['length_m'],
            vertical_coupling=parameters['force_coefficient'], angular_coupling=parameters['torque_coefficient'],
            initial=np.asarray(case['initial_state']), current=case['initial_state'][4], duration=case['duration_s'],
            switch_time=parameters['switch_time_s'], plant=SimpleNamespace(geometry=fields.geometry),
            drivers=SimpleNamespace(resistance=parameters['resistance_ohm'], inductance=parameters['inductance_H'], voltage=[parameters['voltage_V']]))
        solution = coupling_verification.variable_coupling_solution(trial, 'Radau', Fixed.qualification_refinement_factor)
        times = np.array([row['time_s'] for row in case['trace']])
        state_reference = solution.sample(times)
        if not np.allclose(state_reference, case['reference_states'], atol=Fixed.motion_verification_tolerance, rtol=0):
            raise ValueError('Independent continuous solution does not reproduce recorded reference states')
        components = np.array([field_components(row['state']) for row in case['trace']])
        count = len(fields.points)
        row = dict(switched=case['switched'], scans=[])
        summary['cases'].append(row)
        breakpoints = [trial.switch_time] + ([] if solution.extinction_s is None else [solution.extinction_s])
        field_at = lambda time: field_components(solution.sample(time)[0])
        scan = 0
        while sensing.hall_available_time(scan / Fixed.pose_feedback_rate, count) <= trial.duration:
            start = scan / Fixed.pose_feedback_rate
            references = [reference_filtered_scan(field_at, start, count, breakpoints, method) for method in ('DOP853', 'Radau')]
            actual = np.stack([sensing.filtered_scan(times, components[:, :count], start).mean(axis=1),
                sensing.filtered_scan(times, components[:, count:], start, trial.drivers.inductance / trial.drivers.resistance).mean(axis=1)])
            comparison = sensor_scan_comparison(actual, references, spatial, summary['budget'])
            row['scans'].append(dict(start_s=start, **comparison))
            airborne_verification.save_summary(path, summary)
            if not comparison['passed']:
                summary.update(status='combined_field_budget_failed', elapsed_s=perf_counter()-started)
                airborne_verification.save_summary(path, summary)
                return summary
            scan += 1
        if not row['scans']:
            raise ValueError('Sensor budget needs at least one delivered scan')
    summary.update(status='completed', elapsed_s=perf_counter()-started)
    airborne_verification.save_summary(path, summary)
    return summary


def verify_joint_sensor_budget(output):
    if output.exists():
        raise ValueError('Accuracy evidence exists; preserve its results')
    output.mkdir(parents=True)
    started = perf_counter()
    construction = magnetics.ConstructionModel()
    plant = motion.MotionPlant(construction)
    fields = sensing.HallFieldModel(construction)
    count = len(fields.points)
    _, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    driven_identity = array.identities[0]
    array.window = Fixed.cad_patch_cells_per_side * array.short_pitch
    array.build()
    if tuple(array.identities) != fields.identities:
        raise ValueError('Independent coil reference and sensor patch identities differ')
    column = fields.coil_field[:, fields.identities.index(driven_identity)]
    summary = dict(status='running', physical_envelope_qualified=False, budget=analogue_field_budget(),
        scope='Joint RK45 dense motion/current trajectory and passive filtering against independent manufactured coupling and CAD Hall references; powered and switched signals, no feedback or atlas.',
        source_sha256=dict(plant.provenance, **source_fingerprints(__file__, joint_airborne.__file__, sensing.__file__, coupling_verification.__file__)),
        piece_properties=construction.piece_properties, cases=[])
    path = output / 'summary.json'
    airborne_verification.save_summary(path, summary)
    summary['coil_map'] = sensor_spatial_budget(construction, array, fields)
    spatial = np.asarray(summary['coil_map']['calibrated_bound_T'])
    airborne_verification.save_summary(path, summary)
    for switched in (False, True):
        trial = coupling_verification.VariableCouplingTrial(construction, plant, switched)
        reference = coupling_verification.variable_coupling_solution(trial, 'Radau', Fixed.qualification_refinement_factor)
        def field_components(state):
            pose = np.array([0., 0., trial.span / 2 + state[0], 0., 0., state[2]])
            return np.r_[fields.magnet_field(pose), column * state[4]]
        trajectories = []
        metrics = []
        for component in ('magnet', 'coil', 'total'):
            def field_signal(stage, currents):
                truth = stage.truth()
                pose = np.r_[truth['com_position_m'] - [0, 0, stage.geometry.center_of_mass_height], truth['rotation'].as_euler('xyz')]
                magnet = fields.magnet_field(pose)
                coil = column * currents[0]
                return magnet if component == 'magnet' else coil if component == 'coil' else magnet + coil
            dynamics = joint_airborne.JointDynamics(plant, trial.drivers, count, trial.coupling_matrix, field_signal)
            state = dynamics.pack(plant, np.zeros(count))
            segments = []
            boundaries = np.arange(round(trial.duration * Fixed.current_regulator_rate) + 1) / Fixed.current_regulator_rate
            for left, right in zip(boundaries[:-1], boundaries[1:]):
                pieces = dynamics.integrate(left, right, state, enabled=left < trial.switch_time)
                segments.extend(pieces)
                state = pieces[-1].y[:, -1]
            trajectory = OdeSolution([segments[0].t[0], *[segment.t[-1] for segment in segments]], [segment.sol for segment in segments])
            trajectories.append((segments, trajectory))
            metrics.append(dict(component=component, force_evaluations=dynamics.evaluations, filter_evaluations=dynamics.filter_evaluations))
        row = dict(switched=switched, duration_s=trial.duration, metrics=metrics, scans=[])
        summary['cases'].append(row)
        times = np.unique(np.concatenate([segment.t for segment in trajectories[-1][0]]))
        values = trajectories[-1][1](times).T
        actual_state = np.c_[values[:, 2] - trial.height, values[:, 9],
            Rotation.from_quat(values[:, 3:7], scalar_first=True).as_rotvec()[:, 2], values[:, 12],
            values[:, dynamics.current.start], values[:, dynamics.work.stop - 1]]
        errors = np.max(np.abs(actual_state[:, :5] - reference.sample(times)[:, :5]), axis=0)
        limits = np.array([Fixed.adaptive_position_tolerance, Fixed.adaptive_velocity_tolerance,
            Fixed.adaptive_angle_tolerance, Fixed.adaptive_angular_velocity_tolerance, Fixed.adaptive_current_tolerance])
        energy_error = float(np.max(np.abs(coupling_verification.variable_energy_residual(trial, actual_state))))
        row['physical_reference'] = dict(errors=errors.tolist(), limits=limits.tolist(), energy_error_J=energy_error,
            energy_limit_J=dynamics.energy_budget, passed=bool(np.all(errors <= limits) and energy_error <= dynamics.energy_budget))
        if not row['physical_reference']['passed']:
            summary.update(status='joint_reference_failed', elapsed_s=perf_counter()-started)
            airborne_verification.save_summary(path, summary)
            return summary
        breakpoints = [trial.switch_time] + ([] if reference.extinction_s is None else [reference.extinction_s])
        scan_index = 0
        while sensing.hall_available_time(scan_index / Fixed.pose_feedback_rate, count) <= trial.duration:
            start = scan_index / Fixed.pose_feedback_rate
            actual = []
            for segments, trajectory in trajectories:
                scan = dict(initial_filter=trajectory(start)[dynamics.hall_filter],
                    samples=np.zeros((count, Fixed.hall_samples_per_sensor)), sample_count=0)
                for segment in segments:
                    joint_airborne.sample_scan_segment(segment, scan, start, dynamics.hall_filter)
                if scan['sample_count'] != scan['samples'].size:
                    raise ValueError('Joint sensing benchmark did not acquire a complete scan')
                actual.append(scan['samples'].mean(axis=1))
            references = [reference_filtered_scan(lambda time: field_components(reference.sample(time)[0]),
                start, count, breakpoints, method) for method in ('DOP853', 'Radau')]
            superposition_error = np.abs(actual[2] - actual[0] - actual[1])
            comparison = sensor_scan_comparison(np.array(actual[:2]), references, spatial, summary['budget'], superposition_error)
            row['scans'].append(dict(start_s=start, total_filter_superposition_error_T=superposition_error.tolist(), **comparison))
            airborne_verification.save_summary(path, summary)
            if not comparison['passed']:
                summary.update(status='combined_field_budget_failed', elapsed_s=perf_counter()-started)
                airborne_verification.save_summary(path, summary)
                return summary
            scan_index += 1
        if not row['scans']:
            raise ValueError('Joint sensor budget needs a delivered scan')
    summary.update(status='completed', elapsed_s=perf_counter()-started)
    airborne_verification.save_summary(path, summary)
    return summary



class JointTrajectoryRecorder:
    def __init__(self, simulation):
        self.original_integrate = simulation.dynamics.integrate
        self.original_encode = simulation.sensors.encode_signal
        self.segments = []
        self.scans = []

    def integrate(self, *args, **kwargs):
        result = self.original_integrate(*args, **kwargs)
        self.segments.extend(result)
        return result

    def encode_signal(self, signal, start):
        self.scans.append(dict(start_s=start, samples_T=signal.copy().tolist()))
        return self.original_encode(signal, start)

    def save_dense(self, path):
        arrays = dict(segment_bounds_s=np.array([[segment.t[0], segment.t[-1]] for segment in self.segments]))
        for component in ('physical', 'filtered'):
            interpolants = [interpolant for segment in self.segments for interpolant in getattr(segment.sol, component).interpolants]
            for attribute in ('t_old', 't', 'y_old', 'Q'):
                arrays[component + '_' + attribute] = np.array([getattr(item, attribute) for item in interpolants])
            arrays[component + '_segment_counts'] = np.array([len(getattr(segment.sol, component).interpolants) for segment in self.segments])
        temporary = path.with_suffix('.tmp')
        with temporary.open('wb') as stream:
            np.savez_compressed(stream, **arrays)
        temporary.replace(path)
        return hashlib.sha256(path.read_bytes()).hexdigest()


def accelerated_sensor_comparison(actual, components, spatial, budget):
    nominal = components['direct'][-1]
    atlas = components['atlas'][-1]
    refined = components['refined'][-1]
    interpolation = np.sum(np.abs(atlas - nominal), axis=0)
    integration = np.sum(np.abs(nominal - refined), axis=0)
    filtering = np.abs(actual - np.sum(atlas, axis=0))
    reference = sum(np.sum(np.abs(values[0] - values[-1]), axis=0) for values in components.values())
    combined = spatial + interpolation + integration + filtering + reference
    reference_passed = bool(np.max(reference) <= budget['limit_T'] * Fixed.coupling_relative_tolerance)
    return dict(interpolation_trajectory_T=interpolation.tolist(), integration_refinement_T=integration.tolist(),
        production_filter_T=filtering.tolist(), reference_disagreement_T=reference.tolist(),
        combined_bound_T=combined.tolist(), reference_passed=reference_passed,
        passed=bool(reference_passed and np.max(combined) <= budget['limit_T']))


def verify_atlas_sensor_budget(folder, output):
    from methodology import airborne, benchmark_magnetics, hover
    from methodology.magnetic_atlas import MagneticFieldAtlas
    if output.exists():
        raise ValueError('Preserve existing atlas sensing evidence')
    gate = json.loads((folder / 'coupled/summary.json').read_text())
    if gate['status'] != 'completed':
        raise ValueError('Atlas sensing requires passing coupled evidence')
    for name, digest in gate['source_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError('Atlas sensing source differs from coupled evidence')
    atlas = MagneticFieldAtlas.load(folder / 'fields/atlas.npz')
    if atlas.identity != gate['atlas_identity']:
        raise ValueError('Atlas sensing cache differs from coupled evidence')
    output.mkdir(parents=True)
    started = perf_counter()
    summary = dict(status='running', scope='Combined numerical sensing discrepancy for bounded coupled startup; empirical refinement, not a global error certificate or sustained hover.',
        budget=analogue_field_budget(), source_sha256=gate['source_sha256'], atlas_identity=atlas.identity,
        verifier_source_sha256=source_fingerprints(__file__, benchmark_magnetics.__file__),
        inputs_sha256={name: hashlib.sha256((folder / name).read_bytes()).hexdigest() for name in
            ('coupled/summary.json', 'coupled/direct.json', 'coupled/atlas.json',
             'integration/initial_checkpoint.json', 'integration/atlas_initial_checkpoint.json')}, cases={}, scans=[])
    path = output / 'summary.json'
    airborne_verification.save_summary(path, summary)
    references = {}
    signals = {}
    for name, initial, scale in (('direct', 'initial_checkpoint.json', 1),
            ('atlas', 'atlas_initial_checkpoint.json', 1),
            ('refined', 'initial_checkpoint.json', 1 / Fixed.qualification_refinement_factor)):
        simulation = joint_airborne.JointAirborneSimulation.from_checkpoint(folder / 'integration' / initial,
            atlas=atlas if name == 'atlas' else None)
        simulation.tolerance_scale = scale
        if simulation.source != gate['source_sha256'] or simulation.duration != Fixed.adaptive_benchmark_duration:
            raise ValueError('Atlas sensing scenario differs from the bounded gate')
        if name == 'direct':
            _, array, _ = simulation.construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
            array.window = Fixed.cad_patch_cells_per_side * array.short_pitch
            array.build()
            if tuple(array.identities) != simulation.fields.identities:
                raise ValueError('Sensing spatial reference coil order differs')
            summary['coil_map'] = sensor_spatial_budget(simulation.construction, array, simulation.fields)
            if max(summary['coil_map']['calibrated_bound_T']) > summary['budget']['limit_T']:
                summary['status'] = 'spatial_budget_failed'
                airborne_verification.save_summary(path, summary)
                return summary
        recorder = JointTrajectoryRecorder(simulation)
        segments, scans = recorder.segments, recorder.scans
        def progress(report):
            hover.check_operating_limits(simulation, report)
        profiler = cProfile.Profile()
        with patch.object(simulation.dynamics, 'integrate', recorder.integrate), patch.object(simulation.sensors, 'encode_signal', recorder.encode_signal):
            if name == 'atlas':
                report = profiler.runcall(airborne.run_simulation, simulation, output / (name + '.json'),
                    output / (name + '_checkpoint.json'), progress=progress)
            else:
                report = airborne.run_simulation(simulation, output / (name + '.json'),
                    output / (name + '_checkpoint.json'), progress=progress)
        row = dict(passed=report['passed'], elapsed_s=report['elapsed_s'], scans=scans)
        summary['cases'][name] = row
        if name == 'atlas':
            row['profile'] = benchmark_magnetics.profile_statistics(profiler)
        row['dense_sha256'] = recorder.save_dense(output / (name + '_dense.npz'))
        row['dense_format'] = 'SciPy RK45 RkDenseOutput t_old, t, y_old and Q; physical and filtered components, in execution order'
        if name in ('direct', 'atlas'):
            prior = json.loads((folder / 'coupled' / (name + '.json')).read_text())
            row['exact_trace_replay'] = report['trace'] == prior['trace']
        if report['passed']:
            row['coupled_comparison'] = airborne_verification.compare_trials(
                json.loads((folder / 'coupled/direct.json').read_text()), report)
        airborne_verification.save_summary(path, summary)
        if (not report['passed'] or (name in ('direct', 'atlas') and not row['exact_trace_replay'])
                or not row['coupled_comparison']['converged']):
            summary['status'] = 'coupled_or_replay_failed'
            airborne_verification.save_summary(path, summary)
            return summary
        trajectory = OdeSolution([segments[0].t[0], *[segment.t[-1] for segment in segments]], [segment.sol for segment in segments])
        def field_components(time):
            state = trajectory(time)
            pose = np.r_[state[:3] - [0, 0, simulation.plant.geometry.center_of_mass_height],
                Rotation.from_quat(state[3:7], scalar_first=True).as_euler('xyz')]
            magnet = simulation.fields.magnet_field(pose)
            coil = simulation.sensors.actual_coil_field[:, simulation.field_indices] @ state[simulation.dynamics.current]
            return np.r_[simulation.sensors.gain * magnet + simulation.sensors.offset, simulation.sensors.gain * coil]
        references[name] = []
        signals[name] = scans
        for scan in scans:
            values = [reference_filtered_scan(field_components, scan['start_s'], len(simulation.fields.points),
                [segment.t[-1] for segment in segments], method) for method in ('DOP853', 'Radau')]
            references[name].append(values)
        row['independent_components_T'] = [[value.tolist() for value in pair] for pair in references[name]]
        airborne_verification.save_summary(path, summary)
        reference_limit = summary['budget']['limit_T'] * Fixed.coupling_relative_tolerance
        if any(np.max(np.sum(np.abs(pair[0] - pair[-1]), axis=0)) > reference_limit for pair in references[name]):
            summary['status'] = 'sensor_reference_failed'
            airborne_verification.save_summary(path, summary)
            return summary
        if name == 'atlas':
            for index, scan in enumerate(scans):
                partial = accelerated_sensor_comparison(np.mean(scan['samples_T'], axis=1),
                    dict(direct=references['direct'][index], atlas=references['atlas'][index], refined=references['direct'][index]),
                    np.array(summary['coil_map']['calibrated_bound_T']), summary['budget'])
                if not partial['passed']:
                    summary.update(status='partial_sensor_budget_failed', partial_comparison=partial)
                    airborne_verification.save_summary(path, summary)
                    return summary
    if not signals['atlas'] or any([scan['start_s'] for scan in signals[name]] !=
            [scan['start_s'] for scan in signals['atlas']] for name in signals):
        raise ValueError('Atlas sensing needs matching delivered scans')
    for index, scan in enumerate(signals['atlas']):
        comparison = accelerated_sensor_comparison(np.mean(scan['samples_T'], axis=1),
            {name: values[index] for name, values in references.items()},
            np.array(summary['coil_map']['calibrated_bound_T']), summary['budget'])
        summary['scans'].append(dict(start_s=scan['start_s'], **comparison))
    summary.update(status='completed' if all(scan['passed'] for scan in summary['scans']) else 'combined_field_budget_failed',
        elapsed_s=perf_counter() - started)
    airborne_verification.save_summary(path, summary)
    return summary


def diagnose_error_growth(output):
    if output.exists():
        raise ValueError('Accuracy evidence exists; preserve its results')
    output.mkdir(parents=True)
    construction = magnetics.ConstructionModel()
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    layout = levitation_sim.magnet_layout_from_geometry()
    pose = np.zeros(6)
    pose[2] = (geometry.max_flight_gap - geometry.gap) / 2
    _, center = levitation_sim.place_piece(layout, *pose)
    matrix = levitation_sim.actuator_matrix(layout, array, center)
    drivers = electrical.CoilDrivers(construction, array.identities)
    target = np.array([0., 0., geometry.weight, 0., 0., 0.])
    peak, initial = levitation_sim.min_peak_current(matrix, target)
    limit = drivers.current_limit * (1 - Fixed.qualification_current_reserve_fraction)
    if peak > limit:
        raise ValueError('Error-growth reference cannot support gravity within reserved current limits')
    currents = levitation_sim.min_loss_current(matrix, target, limit, initial_currents=initial)
    residual = matrix @ currents - target
    if np.max(np.abs(residual)) > Fixed.motion_verification_tolerance or np.max(np.abs(currents * drivers.resistance)) > drivers.voltage_limit:
        raise ValueError('Error-growth equilibrium is outside the physical electrical model')
    mass = np.r_[np.full(3, geometry.weight / geometry.gravity), geometry.inertias]
    count = len(currents)
    horizon = Fixed.airborne_hover_duration
    position_limit = Fixed.qualification_position_error * Fixed.qualification_convergence_fraction
    angle_limit = Fixed.qualification_angle_error * Fixed.qualification_convergence_fraction
    coordinate_scales = np.r_[np.full(3, position_limit), np.full(3, angle_limit)]
    scales = np.r_[coordinate_scales, coordinate_scales / horizon, np.full(count, drivers.current_limit)]
    summary = dict(status='running', physical_envelope_qualified=False,
        scope='Local error-amplification diagnostic at a gravity-supporting equilibrium, including winding dynamics and motion-induced back-EMF, with fixed applied voltages and no feedback.',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, electrical.__file__, electrical.qualification.__file__)),
        pose=pose.tolist(), coil_identities=array.identities, currents_A=currents.tolist(), voltage_V=(currents * drivers.resistance).tolist(),
        force_quadrature_order=array.force_order, equilibrium_residual=residual.tolist(), horizon_s=horizon,
        generalized_mass=mass.tolist(), resistance_ohm=drivers.resistance, inductance_H=drivers.inductance,
        matrix=matrix.tolist(), scaling=scales.tolist(), cases=[],
        limitations=['This is a local linearization, not a global motion prediction or a test of the feedback controller.',
            'Extrapolated modal growth applies only while the perturbation remains within the linear neighborhood.',
            'Positive open-loop growth is physical instability under fixed voltages; it does not establish a numerical bug or unstable controlled flight.'])
    started = perf_counter()
    for refinement in Fixed.coupling_refinements[-2:]:
        summary['active_derivative_refinement'] = refinement
        airborne_verification.save_summary(output / 'summary.json', summary)
        stiffness, current_eigenvalues = levitation_sim.open_loop_stiffness(layout, array, currents, pose, refinement)
        generator = np.block([[np.zeros((6, 6)), np.eye(6), np.zeros((6, count))],
            [stiffness / mass[:, None], np.zeros((6, 6)), matrix / mass[:, None]],
            [np.zeros((count, 6)), -matrix.T / drivers.inductance, -np.eye(count) * drivers.resistance / drivers.inductance]])
        eigenvalues = np.linalg.eigvals(generator)
        scaled = np.linalg.eigvals(generator * scales[None, :] / scales[:, None])
        rate = float(max(eigenvalues.real))
        summary['cases'].append(dict(derivative_refinement=refinement, stiffness=stiffness.tolist(),
            fixed_current_growth_per_s=float(max(current_eigenvalues.real)), fixed_voltage_growth_per_s=rate,
            scaled_fixed_voltage_growth_per_s=float(max(scaled.real)), eigenvalues=[[v.real, v.imag] for v in eigenvalues],
            e_fold_time_s=1 / rate if rate > 0 else None, extrapolated_log10_modal_growth=rate * horizon / np.log(10)))
    rates = np.array([[case['fixed_voltage_growth_per_s'], case['scaled_fixed_voltage_growth_per_s']] for case in summary['cases']])
    agreement = float(np.max(np.abs(rates - rates[-1, -1])) / max(abs(rates[-1, -1]), Fixed.singular_value_tolerance))
    summary.update(status='completed' if agreement <= Fixed.coupling_relative_tolerance else 'linearization_unresolved',
        active_derivative_refinement=None, growth_relative_disagreement=agreement, relative_limit=Fixed.coupling_relative_tolerance,
        elapsed_s=perf_counter()-started)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Assess numerical accuracy by mechanism and declared error budget.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cad-reference', type=Path, help='Retained coil-field summary to compare against independent curved-path quadrature.')
    parser.add_argument('--force-record', type=Path, help='Retained static force maps for a force-to-trajectory budget screen.')
    parser.add_argument('--coupling-record', type=Path, help='Retained independent coupling report supplying the CAD mass.')
    parser.add_argument('--curved-force', action='store_true', help='Check curved CAD force quadrature and its independent flux reference.')
    parser.add_argument('--error-growth', action='store_true', help='Check open-loop amplification of small physical integration errors.')
    parser.add_argument('--sensor-record', type=Path, help='Passing variable-coupling record for the combined analogue sensing budget.')
    parser.add_argument('--joint-sensor', action='store_true', help='Verify the joint integrator and passive dense-trajectory Hall filtering.')
    parser.add_argument('--atlas-sensor', type=Path, help='Passing joint atlas evidence folder for the combined sensing check and accelerated profile.')
    args = parser.parse_args()
    if args.atlas_sensor:
        if args.joint_sensor or args.sensor_record or args.error_growth or args.curved_force or args.force_record or args.coupling_record or args.cad_reference:
            raise ValueError('Atlas sensing verification does not accept other calculation modes')
        verify_atlas_sensor_budget(args.atlas_sensor, args.output)
    elif args.joint_sensor:
        if args.sensor_record or args.error_growth or args.curved_force or args.force_record or args.coupling_record or args.cad_reference:
            raise ValueError('Joint sensor verification does not accept other calculation modes')
        verify_joint_sensor_budget(args.output)
    elif args.sensor_record:
        if args.error_growth or args.curved_force or args.force_record or args.coupling_record or args.cad_reference:
            raise ValueError('Sensor budget requires only the variable-coupling record')
        verify_sensor_budget(args.sensor_record, args.output)
    elif args.error_growth:
        if args.curved_force or args.force_record or args.coupling_record or args.cad_reference:
            raise ValueError('Error-growth diagnosis does not accept other calculation modes')
        diagnose_error_growth(args.output)
    elif args.curved_force:
        if args.force_record or args.coupling_record or args.cad_reference:
            raise ValueError('Curved force verification does not accept recorded-input modes')
        verify_curved_force(args.output)
    elif args.force_record:
        if args.cad_reference or args.coupling_record is None:
            raise ValueError('Force assessment requires only the force and coupling records')
        assess_force_accumulation(args.force_record, args.coupling_record, args.output)
    elif args.cad_reference:
        verify_cad_reference(args.cad_reference, args.output)
    else:
        verify_field_budget(args.output)

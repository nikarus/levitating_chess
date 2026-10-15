import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.optimize import linprog

import levitation_sim
from model import Inputs, Fixed, Constants
from methodology import airborne_verification, coupling_verification, physics_verification, sensing, source_fingerprints
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
    fields = []
    for point in points:
        displacement = point - positions
        distances = np.linalg.norm(displacement, axis=1)
        if np.any(distances == 0):
            raise ValueError('Field reference cannot evaluate on its conductor')
        fields.append(Constants.vacuum_permeability / (4 * np.pi) *
            np.sum(np.cross(elements, displacement) / distances[:, None] ** 3, axis=0))
    return np.asarray(fields)


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
    local_points = array.rotation.inv().apply(points)
    translated = np.concatenate([local_points - [placement['x'], placement['y'], placement['top'] - array.height]
        for placement in array.placements])
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
        field = cad_field_quadrature(construction.path, translated, order, construction.return_depth)
        matrix = field[:, 2].reshape(len(array.coils), len(points)).T
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Assess numerical accuracy by mechanism and declared error budget.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cad-reference', type=Path, help='Retained coil-field summary to compare against independent curved-path quadrature.')
    parser.add_argument('--force-record', type=Path, help='Retained static force maps for a force-to-trajectory budget screen.')
    parser.add_argument('--coupling-record', type=Path, help='Retained independent coupling report supplying the CAD mass.')
    parser.add_argument('--curved-force', action='store_true', help='Check curved CAD force quadrature and its independent flux reference.')
    args = parser.parse_args()
    if args.curved_force:
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

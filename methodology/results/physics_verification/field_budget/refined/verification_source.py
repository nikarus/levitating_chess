import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np
from numpy.polynomial.legendre import leggauss
from build123d import PositionMode

from model import Inputs, Fixed, Constants
from methodology import airborne_verification, sensing, source_fingerprints
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
    nodes, weights = leggauss(order)
    nodes, weights = (nodes + 1) / 2, weights / 2
    positions, elements = [], []
    for edge in path.edges().sort_by(path):
        lengths = nodes * edge.length
        positions.extend(tuple(edge.position_at(length, PositionMode.LENGTH)) for length in lengths)
        elements.extend(np.asarray(tuple(edge.tangent_at(length, PositionMode.LENGTH))) * edge.length * weight
            for length, weight in zip(lengths, weights))
    start, end = np.asarray(tuple(path @ 0)), np.asarray(tuple(path @ 1))
    corners = (end, np.r_[end[:2], -return_depth], np.r_[start[:2], -return_depth], start)
    for first, second in zip(corners[:-1], corners[1:]):
        positions.extend(first + nodes[:, None] * (second - first))
        elements.extend(weights[:, None] * (second - first))
    positions, elements = np.asarray(positions) / 1000, np.asarray(elements) / 1000
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Assess numerical accuracy by mechanism and declared error budget.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cad-reference', type=Path, help='Retained coil-field summary to compare against independent curved-path quadrature.')
    args = parser.parse_args()
    if args.cad_reference:
        verify_cad_reference(args.cad_reference, args.output)
    else:
        verify_field_budget(args.output)

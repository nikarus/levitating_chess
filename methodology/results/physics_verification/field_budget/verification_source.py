import argparse
from pathlib import Path
from time import perf_counter

import numpy as np

from model import Fixed
from methodology import airborne_verification, coupling_verification, sensing, source_fingerprints
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
    if difference.ndim != 2 or not np.isfinite(difference).all() or current_limit <= 0:
        raise ValueError('Field bound requires a finite sensor/channel matrix and a positive current limit')
    channel_bounds = current_limit * np.sum(np.abs(difference), axis=1)
    worst = int(np.argmax(channel_bounds))
    pattern = current_limit * np.sign(difference[worst])
    return dict(maximum_T=float(channel_bounds[worst]), per_sensor_T=channel_bounds.tolist(),
        worst_sensor=worst, attaining_currents_A=pattern.tolist(),
        attained_T=float(abs(difference[worst] @ pattern)))


def verify_field_budget(output):
    if output.exists():
        raise ValueError('Accuracy evidence exists; preserve its results')
    output.mkdir(parents=True)
    summary = dict(scope='Stationary coil-to-Hall map convergence and all-channel current-box bound; no feedback or motion integration',
        status='running', physical_envelope_qualified=False, budget=analogue_field_budget(),
        current_limit_A=Fixed.driver_channel_current, refinements=Fixed.coupling_refinements,
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, sensing.__file__, coupling_verification.__file__)),
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
        nominal_map_meets_reference_budget=summary['against_finest'][0]['bound']['maximum_T'] <= summary['budget']['limit_T'])
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Assess numerical accuracy by mechanism and declared error budget.')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    verify_field_budget(args.output)

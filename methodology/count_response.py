import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from model import Fixed
from methodology import airborne_diagnostics, airborne_verification, hover, source_fingerprints


def load_response_start(report_path, atlas_path):
    report = json.loads(report_path.read_text())
    checkpoint_path = report_path.with_name(report_path.stem + '_checkpoint.json')
    state = json.loads(checkpoint_path.read_text())
    simulation = hover.simulation_from_evidence(report, atlas_path, False, report['fixed_reference_step_s'])
    if state['schema'] == 'airborne-count-intervention-v1':
        simulation.checkpoint_schema = state['schema']
        simulation.source.update(source_fingerprints(airborne_diagnostics.__file__))
    simulation.load_checkpoint(checkpoint_path)
    if simulation.trace != report['trace'] or simulation.peaks != report['peaks']:
        raise ValueError('Response start report and checkpoint disagree')
    return simulation


def continue_response(simulation):
    if simulation.stop_reason not in ('peer_trial_failed', 'count_response_comparison_failed'):
        raise ValueError('Response extension must not clear an operating failure')
    hover.check_operating_limits(simulation, simulation.report())
    if simulation.stop_reason not in ('peer_trial_failed', 'count_response_comparison_failed'):
        raise ValueError('Response start exceeds operating limits')
    report = simulation.report()
    if (report['peaks']['estimated_position_error_m'] > Fixed.qualification_position_error
            or report['peaks']['estimated_angle_error_rad'] > Fixed.qualification_angle_error
            or report['peaks']['position_error_m'] > Fixed.qualification_position_error
            or report['peaks']['angle_error_rad'] > Fixed.qualification_angle_error
            or report['peaks']['current_A'] > simulation.drivers.current_limit):
        raise ValueError('Response start exceeds operating limits')
    simulation.stop_reason = None
    simulation.finished = False
    simulation.checkpoint_schema = 'airborne-count-response-v1'
    simulation.source.update(source_fingerprints(__file__))


def save_response(simulation, output, name):
    simulation.save_checkpoint(output / (name + '_checkpoint.json'))
    report = simulation.report()
    report.update(scope='Bounded single-count response; different sensor input, not numerical convergence or flight qualification', passed=False)
    airborne_verification.save_summary(output / (name + '.json'), report)


def response_worker(name, report_path, atlas_path, output, stop):
    simulation = load_response_start(report_path, atlas_path)
    original_stop = simulation.stop_reason
    start = simulation.clock
    continue_response(simulation)
    target = Fixed.airborne_refinement_duration
    if start >= target:
        raise ValueError('Response extension requires a later target')
    while not simulation.finished and simulation.clock < target - Fixed.adaptive_clock_tolerance:
        simulation.advance()
        if simulation.checkpoint_due() or simulation.finished:
            hover.check_operating_limits(simulation, simulation.report())
            if simulation.stop_reason is not None:
                stop.set()
            elif stop.is_set():
                simulation.stop_reason = 'peer_operating_failure'
                simulation.finished = True
            save_response(simulation, output, name)
    save_response(simulation, output, name)
    return dict(name=name, resumed_time_s=start, original_stop_reason=original_stop,
        duration_completed_s=simulation.clock, stop_reason=simulation.stop_reason,
        reached_target=simulation.clock >= target - Fixed.adaptive_clock_tolerance,
        elapsed_s=simulation.report()['elapsed_s'], report=hover.comparison_snapshot(simulation))


def analyze_response(reports, simulation, output):
    common = min(report['duration_completed_s'] for report in reports)
    common_times = set.intersection(*({row['time_s'] for row in report['trace'] if row['time_s'] <= common} for report in reports))
    selected = [dict(report, trace=[row for row in report['trace'] if row['time_s'] in common_times]) for report in reports]
    comparison = airborne_verification.compare_trials(*selected, trace_only=True)
    rows = []
    for original, changed in zip(*(report['trace'] for report in selected)):
        wrenches = []
        for sample in (original, changed):
            pose = np.array(sample['pose'])
            simulation.plant.reset(position=pose[:3] + [0, 0, simulation.plant.geometry.center_of_mass_height],
                rotation=Rotation.from_euler('xyz', pose[3:]))
            matrix = simulation.plant.magnetic_matrix(window_center=simulation.reference[:2])
            wrenches.append(matrix @ sample['currents_A'])
        difference = wrenches[1] - wrenches[0]
        rows.append(dict(time_s=changed['time_s'],
            current_difference_A=float(np.max(np.abs(np.array(changed['currents_A']) - original['currents_A']))),
            position_difference_m=float(np.linalg.norm(np.array(changed['pose'][:3]) - original['pose'][:3])),
            orientation_difference_rad=float((Rotation.from_euler('xyz', changed['pose'][3:]) * Rotation.from_euler('xyz', original['pose'][3:]).inv()).magnitude()),
            force_difference_N=float(np.linalg.norm(difference[:3])),
            force_difference_weight_fraction=float(np.linalg.norm(difference[:3]) / simulation.plant.geometry.weight),
            torque_difference_Nm=float(np.linalg.norm(difference[3:])),
            copper_loss_W=[float(simulation.drivers.copper_resistance * np.dot(sample['currents_A'], sample['currents_A'])) for sample in (original, changed)]))
    airborne_verification.save_summary(output / 'response.json', dict(
        scope='Instantaneous magnetic forces and COM torque at saved observation times; not integration-step extrema',
        source_sha256=simulation.source, trace=rows))
    return dict(common_duration_s=common, different_input_comparison=comparison,
        maxima={key: max(rows, key=lambda row: row[key]) for key in (
            'current_difference_A', 'position_difference_m', 'orientation_difference_rad', 'force_difference_N', 'torque_difference_Nm')},
        endpoint=rows[-1])


def run(reference_path, changed_path, atlas_path, output):
    if output.exists():
        raise ValueError('Response output already exists; preserve its evidence')
    starts = [load_response_start(path, atlas_path) for path in (reference_path, changed_path)]
    for simulation in starts:
        continue_response(simulation)
    if (starts[0].fixed_step != starts[1].fixed_step or starts[0].atlas_configuration != starts[1].atlas_configuration):
        raise ValueError('Response branches must retain identical integration and atlas settings')
    output.mkdir(parents=True)
    summary = dict(scope='Bounded single-count response extension; operating stops retained, inter-run current comparison reported without stopping',
        source_sha256=source_fingerprints(__file__), inputs_sha256=source_fingerprints(reference_path, changed_path,
            reference_path.with_name(reference_path.stem + '_checkpoint.json'), changed_path.with_name(changed_path.stem + '_checkpoint.json')),
        target_duration_s=Fixed.airborne_refinement_duration, status='running', cases={},
        numerical_qualification_passed=False, flight_qualification_passed=False, additional_count_interventions=0)
    airborne_verification.save_summary(output / 'summary.json', summary)
    context = multiprocessing.get_context('spawn')
    with context.Manager() as manager:
        stop = manager.Event()
        with ProcessPoolExecutor(max_workers=len(starts), mp_context=context) as executor:
            futures = [executor.submit(response_worker, name, path, atlas_path, output, stop)
                for name, path in (('baseline', reference_path), ('changed_count', changed_path))]
            for future in futures:
                future.add_done_callback(lambda completed: stop.set() if completed.exception() is not None else None)
            reports = {}
            for future in as_completed(futures):
                result = future.result()
                reports[result['name']] = result.pop('report')
                summary['cases'][result['name']] = result
                airborne_verification.save_summary(output / 'summary.json', summary)
    summary.update(status='completed' if all(case['reached_target'] and case['stop_reason'] is None for case in summary['cases'].values()) else 'operating_limit_failed')
    summary['response'] = analyze_response([reports['baseline'], reports['changed_count']], starts[0], output)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Continue the saved single-count experiment with operating stops and diagnostic trajectory comparisons.')
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--changed', type=Path, required=True)
    parser.add_argument('--atlas', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.reference, args.changed, args.atlas, args.output)

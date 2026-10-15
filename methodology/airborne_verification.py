import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import multiprocessing
from pathlib import Path
from time import time

import numpy as np
from scipy.spatial.transform import Rotation

from model import Fixed
from methodology import airborne, source_fingerprints
from methodology.airborne_video import render_report
from methodology.cad import magnetics


def compare_trials(baseline, refined, trace_only=False):
    times = np.array([row['time_s'] for row in baseline['trace']])
    refined_times = np.array([row['time_s'] for row in refined['trace']])
    if times.shape != refined_times.shape or not np.allclose(times, refined_times, atol=Fixed.motion_verification_tolerance, rtol=0):
        raise ValueError('Refinement comparison requires complete matching observation times')
    if baseline['hardware_rates_Hz'] != refined['hardware_rates_Hz']:
        raise ValueError('Numerical refinement must preserve hardware sampling rates')
    differences = {}
    budgets = {}
    for key in ('pose', 'estimated_pose'):
        original = np.array([row[key] for row in baseline['trace']])
        candidate = np.array([row[key] for row in refined['trace']])
        differences[key + '_position_m'] = float(np.max(np.linalg.norm(original[:, :3] - candidate[:, :3], axis=1)))
        rotation_difference = Rotation.from_euler('xyz', original[:, 3:]) * Rotation.from_euler('xyz', candidate[:, 3:]).inv()
        differences[key + '_angle_rad'] = float(np.max(rotation_difference.magnitude()))
        budgets[key + '_position_m'] = Fixed.qualification_position_error
        budgets[key + '_angle_rad'] = Fixed.qualification_angle_error
    currents = np.array([row['currents_A'] for row in baseline['trace']])
    refined_currents = np.array([row['currents_A'] for row in refined['trace']])
    differences['current_trace_A'] = float(np.max(np.abs(currents - refined_currents)))
    budgets['current_trace_A'] = Fixed.driver_channel_current * Fixed.qualification_current_reserve_fraction
    for key in (() if trace_only else ('current_A', 'required_current_A', 'voltage_V', 'copper_loss_W')):
        differences['peak_' + key] = abs(baseline['peaks'][key] - refined['peaks'][key])
        budgets['peak_' + key] = (budgets['current_trace_A'] if key.endswith('current_A')
                                 else max(baseline['peaks'][key], refined['peaks'][key]))
    limits = {key: budget * Fixed.qualification_convergence_fraction for key, budget in budgets.items()}
    checks = {key: differences[key] <= limits[key] for key in differences}
    if not trace_only:
        checks['both_trials_passed'] = baseline['passed'] and refined['passed']
    return dict(differences=differences, limits=limits, checks=checks, converged=all(checks.values()))


def save_summary(path, summary):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(summary, indent=Fixed.report_precision, allow_nan=False) + '\n')
    temporary.replace(path)


def recorded_construction(report):
    construction = magnetics.ConstructionModel()
    properties = report['mechanical_properties']
    construction.piece_properties.update(mass_g=properties['king_mass_kg'] * 1000,
        center_mm=(np.array(properties['king_com_from_magnet_bottom_m']) * 1000).tolist(),
        inertia_kg_m2=properties['king_inertia_kg_m2'])
    return construction


def compare_progress(reference, report, simulation, role):
    reference = dict(reference, trace=reference['trace'][:len(report['trace'])])
    comparison = compare_trials(reference, report, trace_only=True)
    if role == 'repeat':
        comparison['checks']['identical_trace'] = reference['trace'] == report['trace']
        comparison['converged'] = all(comparison['checks'].values())
    if not comparison['converged']:
        comparison['stopped_at_s'] = report['trace'][-1]['time_s']
        simulation.stop_reason = 'repeatability_failed' if role == 'repeat' else 'numerical_comparison_failed'
        simulation.finished = True
    return comparison


def remaining_case(baseline, output, role, deadline, cancelled):
    time_refinement = Fixed.qualification_refinement_factor if role == 'time_refined' else 1
    magnetic_refinement = baseline['magnetic_refinement'] * (Fixed.qualification_refinement_factor if role == 'magnetic_refined' else 1)
    simulation = airborne.AirborneSimulation(recorded_construction(baseline), baseline['intended_duration_s'],
        disturbed=baseline['disturbed'], time_refinement=time_refinement, magnetic_refinement=magnetic_refinement)
    if simulation.source != baseline['source_sha256'] or simulation.report()['mechanical_properties'] != baseline['mechanical_properties']:
        cancelled.set()
        raise ValueError('Reused baseline simulation sources or recorded CAD mass properties differ')
    comparison = None

    def progress(report):
        nonlocal comparison
        if report['trace'] and report['stop_reason'] is None:
            comparison = compare_progress(baseline, report, simulation, role)
        if simulation.stop_reason is not None:
            cancelled.set()
        elif cancelled.is_set():
            simulation.finished = True
            simulation.stop_reason = 'peer_trial_failed'

    report = airborne.run_simulation(simulation, output / (role + '.json'), output / (role + '_checkpoint.json'),
                                    deadline=deadline, progress=progress)
    if report['passed']:
        comparison = compare_trials(baseline, report)
        if role == 'repeat':
            comparison['checks']['identical_trace'] = baseline['trace'] == report['trace']
            comparison['converged'] = all(comparison['checks'].values())
    if not report['passed'] or comparison is None or not comparison['converged']:
        cancelled.set()
    return dict(report=report, comparison=comparison)


def verified_saved_trial(path):
    report = json.loads(path.read_text())
    checkpoint_path = path.with_name(path.stem + '_checkpoint.json')
    checkpoint = json.loads(checkpoint_path.read_text())
    if not report['passed'] or not report['finished'] or report['stop_reason'] is not None:
        raise ValueError('Only completed passing evidence can be reused')
    if (report['trace'] != checkpoint['trace'] or report['peaks'] != checkpoint['peaks']
            or report['source_sha256'] != checkpoint['source_sha256']
            or not checkpoint['finished'] or checkpoint['stop_reason'] is not None):
        raise ValueError('Saved report and checkpoint disagree')
    return report


def run_remaining(baseline_path, screen_path, deadline):
    output = baseline_path.parent
    summary_path = output / 'summary.json'
    if summary_path.exists() or any((output / (role + suffix)).exists()
            for role in ('repeat', 'time_refined', 'magnetic_refined') for suffix in ('.json', '_checkpoint.json')):
        raise ValueError('Remaining verification already has evidence; inspect it before another launch')
    baseline = verified_saved_trial(baseline_path)
    screen = json.loads(screen_path.read_text())
    names = ('screen_baseline', 'screen_repeat', 'screen_time_refined', 'screen_magnetic_refined')
    if (screen['status'] != 'completed' or not all(name in screen['cases'] for name in names)
            or not all(screen['comparisons'][name]['converged'] for name in names[1:])):
        raise ValueError('Complete passing short-screen evidence is required')
    for name, digest in baseline['source_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError(f'Reused baseline source differs: {name}')
    short_reports = {}
    for name in names:
        report = verified_saved_trial(screen_path.parent / (name + '.json'))
        if (report['intended_duration_s'] != Fixed.airborne_refinement_screen_duration
                or not all(report['source_sha256'].get(name) == digest for name, digest in baseline['source_sha256'].items())):
            raise ValueError('Short screen uses different duration or simulation sources')
        short_reports[name] = report
    if short_reports['screen_baseline']['trace'] != short_reports['screen_repeat']['trace'] or not all(
            compare_trials(short_reports['screen_baseline'], short_reports[name])['converged'] for name in names[1:]):
        raise ValueError('Saved short-screen comparisons do not reproduce under current acceptance limits')
    if (baseline['intended_duration_s'] != Fixed.airborne_refinement_duration or baseline['time_refinement'] != 1
            or baseline['magnetic_refinement'] != min(Fixed.current_demand_mesh_refinements) or not baseline['disturbed']):
        raise ValueError('Reused baseline is not the normal recovery trial')
    reused_paths = [baseline_path, screen_path, *(screen_path.parent / (name + '.json') for name in names)]
    reused_paths.extend(path.with_name(path.stem + '_checkpoint.json') for path in [baseline_path, *reused_paths[2:]])
    summary = dict(scope='Saved baseline, exact repeat, then concurrent independent time/magnetic refinements; not hover qualification',
        source_sha256=source_fingerprints(__file__), started_utc=datetime.now(timezone.utc).isoformat(),
        deadline_utc=deadline.isoformat(), status='running', active_cases=['repeat'],
        reused_evidence_sha256=source_fingerprints(*reused_paths),
        cases={'baseline': dict(status='reused', passed=True, duration_completed_s=baseline['duration_completed_s'])}, comparisons={})
    save_summary(summary_path, summary)

    def record(role, result):
        report, comparison = result['report'], result['comparison']
        summary['cases'][role] = {key: report[key] for key in ('passed', 'finished', 'stop_reason', 'duration_completed_s', 'elapsed_s', 'peaks')}
        summary['comparisons'][role] = comparison
        if not report['passed'] or comparison is None or not comparison['converged']:
            summary['status'] = 'verification_failed'
        save_summary(summary_path, summary)

    context = multiprocessing.get_context('spawn')
    with context.Manager() as manager:
        cancelled = manager.Event()
        record('repeat', remaining_case(baseline, output, 'repeat', deadline.timestamp(), cancelled))
        if not cancelled.is_set() and time() < deadline.timestamp():
            summary['active_cases'] = ['time_refined', 'magnetic_refined']
            save_summary(summary_path, summary)
            with ProcessPoolExecutor(max_workers=len(summary['active_cases']), mp_context=context) as executor:
                futures = {executor.submit(remaining_case, baseline, output, role, deadline.timestamp(), cancelled): role
                           for role in summary['active_cases']}
                for future in as_completed(futures):
                    if future.exception() is not None:
                        cancelled.set()
                        summary['status'] = 'worker_failed'
                        summary['failed_case'] = futures[future]
                        save_summary(summary_path, summary)
                    record(futures[future], future.result())
                    summary['active_cases'].remove(futures[future])
        if summary['status'] == 'running':
            summary['status'] = 'completed' if len(summary['comparisons']) == 3 else 'deadline_reached'
    summary['active_cases'] = []
    summary['finished_utc'] = datetime.now(timezone.utc).isoformat()
    save_summary(summary_path, summary)
    print(json.dumps(summary), flush=True)
    return summary


def run_batch(output, deadline, video_output, extended_hover=True, screen_only=False, magnetic_first=False):
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / 'summary.json'
    if summary_path.exists():
        raise ValueError('Verification output already contains a batch; inspect its evidence before replacing it')
    baseline_mesh = min(Fixed.current_demand_mesh_refinements)
    stages = [('screen_', Fixed.airborne_refinement_screen_duration)]
    if not screen_only:
        stages.append(('', Fixed.airborne_refinement_duration))
    refinements = [('time_refined', Fixed.qualification_refinement_factor, baseline_mesh),
                   ('magnetic_refined', 1, baseline_mesh * Fixed.qualification_refinement_factor)]
    if magnetic_first:
        refinements.reverse()
    cases = [(prefix + role, duration, time_refinement, magnetic_refinement, prefix + 'baseline', role)
        for prefix, duration in stages
        for role, time_refinement, magnetic_refinement in (
            ('baseline', 1, baseline_mesh), ('repeat', 1, baseline_mesh), *refinements)]
    if extended_hover and not screen_only:
        cases.append(('extended_hover', Fixed.airborne_hover_duration, 1, baseline_mesh, 'baseline', 'extended_hover'))
    summary = dict(scope='Short numerical screen only; not hover qualification' if screen_only else
                   'Short numerical screen, longer comparisons, then optional nominal hover; not flight-envelope qualification',
                   source_sha256=source_fingerprints(__file__), started_utc=datetime.now(timezone.utc).isoformat(),
                   deadline_utc=deadline.isoformat(), status='running', active_case=None, cases={}, comparisons={})
    save_summary(summary_path, summary)
    reports = {}
    latest_output = None
    construction = magnetics.ConstructionModel()
    for name, duration, time_refinement, magnetic_refinement, reference_name, role in cases:
        if time() >= deadline.timestamp():
            summary['status'] = 'deadline_reached'
            break
        if name == 'extended_hover' and not all(row['converged'] for row in summary['comparisons'].values()):
            summary['status'] = 'numerical_comparison_failed'
            break
        summary['active_case'] = name
        summary['cases'][name] = dict(status='initializing', intended_duration_s=duration,
                                      time_refinement=time_refinement, magnetic_refinement=magnetic_refinement)
        save_summary(summary_path, summary)
        simulation = airborne.AirborneSimulation(construction, duration, disturbed=True,
                         time_refinement=time_refinement, magnetic_refinement=magnetic_refinement)
        simulation.source.update(summary['source_sha256'])
        report_path = output / (name + '.json')

        def progress(report):
            if role in ('repeat', 'time_refined', 'magnetic_refined') and report['trace'] and report['stop_reason'] is None:
                prefix = compare_progress(reports[reference_name], report, simulation, role)
                if not prefix['converged']:
                    summary['comparisons'][name] = prefix
            summary['cases'][name].update(status='completed' if report['finished'] else 'running',
                duration_completed_s=report['duration_completed_s'], elapsed_s=report['elapsed_s'],
                passed=report['passed'], stop_reason=report['stop_reason'], peaks=report['peaks'])
            save_summary(summary_path, summary)

        report = airborne.run_simulation(simulation, report_path, output / (name + '_checkpoint.json'),
                                        deadline=deadline.timestamp(), progress=progress)
        progress(report)
        reports[name] = report
        latest_output = report_path
        if not report['finished']:
            summary['cases'][name]['status'] = 'deadline_reached'
            summary['status'] = 'deadline_reached'
            break
        if not report['passed']:
            status = report['stop_reason'] if report['stop_reason'] in ('repeatability_failed', 'numerical_comparison_failed') else 'trial_failed'
            summary['cases'][name]['status'] = status
            summary['status'] = status
            break
        if role in ('repeat', 'time_refined', 'magnetic_refined'):
            summary['comparisons'][name] = compare_trials(reports[reference_name], report)
            if role == 'repeat':
                identical = reports[reference_name]['trace'] == report['trace']
                summary['comparisons'][name]['checks']['identical_trace'] = identical
                summary['comparisons'][name]['converged'] &= identical
                if not identical:
                    summary['status'] = 'repeatability_failed'
                    break
            save_summary(summary_path, summary)
            if not summary['comparisons'][name]['converged']:
                summary['status'] = 'numerical_comparison_failed'
                break
    else:
        summary['status'] = 'completed' if all(row['converged'] for row in summary['comparisons'].values()) else 'numerical_comparison_failed'
    summary['active_case'] = None
    summary['finished_utc'] = datetime.now(timezone.utc).isoformat()
    save_summary(summary_path, summary)
    if video_output is not None and latest_output is not None and len(reports[latest_output.stem]['trace']) > 1:
        summary['video'] = render_report(latest_output, video_output)
        save_summary(summary_path, summary)
    print(json.dumps(summary), flush=True)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run independent resolution checks and gated extended hover without supervision.')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('results') / 'airborne_verification')
    parser.add_argument('--deadline', type=datetime.fromisoformat, required=True, help='Timezone-aware deadline; incomplete trials remain unqualified.')
    parser.add_argument('--video-output', type=Path, default=Path(__file__).with_name('cad') / 'output' / 'airborne_verification.mp4')
    parser.add_argument('--no-video', action='store_true', help='Retain numerical evidence without rendering another preview.')
    parser.add_argument('--magnetic-first', action='store_true', help='Check magnetic refinement before time refinement, retaining both required comparisons.')
    parser.add_argument('--reuse-baseline', type=Path, help='Reuse this normal recovery report; run its exact repeat then both refinements concurrently in its folder.')
    parser.add_argument('--screen-evidence', type=Path, help='Completed short-screen summary required with --reuse-baseline.')
    scope = parser.add_mutually_exclusive_group()
    scope.add_argument('--verification-only', action='store_true', help='Finish after the repeat/time/magnetic checks, before extended hover.')
    scope.add_argument('--screen-only', action='store_true', help='Stop after the short repeat/time/magnetic screen, even if it passes.')
    arguments = parser.parse_args()
    if arguments.deadline.tzinfo is None or arguments.deadline.timestamp() <= time():
        parser.error('Deadline must be a future timezone-aware instant')
    if arguments.reuse_baseline is not None:
        if arguments.screen_evidence is None or not arguments.verification_only or not arguments.no_video or arguments.screen_only:
            parser.error('Baseline reuse requires --screen-evidence, --verification-only and --no-video')
        run_remaining(arguments.reuse_baseline, arguments.screen_evidence, arguments.deadline)
    else:
        if arguments.screen_evidence is not None:
            parser.error('--screen-evidence requires --reuse-baseline')
        run_batch(arguments.output, arguments.deadline, None if arguments.no_video else arguments.video_output,
                  extended_hover=not arguments.verification_only, screen_only=arguments.screen_only, magnetic_first=arguments.magnetic_first)

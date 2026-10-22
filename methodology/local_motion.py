import argparse
from datetime import datetime
import hashlib
import json
import subprocess
import sys
from time import perf_counter
from pathlib import Path
from unittest.mock import patch
from zipfile import ZipFile, ZIP_DEFLATED

import numpy as np
from scipy.spatial.transform import Rotation

from model import Fixed
from methodology import airborne, airborne_verification, hover, numerical_accuracy, source_fingerprints
from methodology.cad import magnetics
from methodology.joint_airborne import JointAirborneSimulation
from methodology.magnetic_atlas import MagneticFieldAtlas
from methodology.numerical_accuracy import JointTrajectoryRecorder


def settling_limits():
    return dict(position_error_m=Fixed.qualification_position_error, angle_error_rad=Fixed.qualification_angle_error,
        speed_m_per_s=Fixed.airborne_settling_speed, angular_speed_rad_per_s=Fixed.airborne_settling_angular_speed)


def settling_metrics(states, reference, center_height):
    positions = states[:, :3] - [0, 0, center_height]
    rotations = Rotation.from_quat(states[:, 3:7], scalar_first=True)
    return dict(position_error_m=float(np.max(np.linalg.norm(positions - reference[:3], axis=1))),
        angle_error_rad=float(np.max((rotations * Rotation.from_euler('xyz', reference[3:]).inv()).magnitude())),
        speed_m_per_s=float(np.max(np.linalg.norm(states[:, 7:10], axis=1))),
        angular_speed_rad_per_s=float(np.max(np.linalg.norm(states[:, 10:13], axis=1))))


def observe_settling(simulation, segments, result):
    for segment in segments:
        if segment.t[-1] < result['start_s']:
            continue
        times = np.unique(np.r_[segment.t, (segment.t[:-1] + segment.t[1:]) / 2,
            max(segment.t[0], result['start_s'])])
        times = times[(times >= result['start_s']) & (times <= result['end_s'])]
        if not len(times):
            continue
        metrics = settling_metrics(segment.sol(times).T, simulation.reference_state(result['end_s'])[0],
            simulation.plant.geometry.center_of_mass_height)
        result['samples'] += len(times)
        result['observed_until_s'] = float(times[-1])
        for name, value in metrics.items():
            result['peaks'][name] = max(result['peaks'][name], value)
        result['limits_met'] = all(result['peaks'][name] <= limit for name, limit in result['limits'].items())
        if not result['limits_met'] and simulation.stop_reason is None:
            simulation.stop_reason = 'endpoint_settling_limits_exceeded'
            simulation.finished = True
    result['passed'] = bool(result['samples'] and result['limits_met']
        and result['observed_until_s'] >= result['end_s'] - Fixed.adaptive_clock_tolerance)


def verify_recorded_sensing(simulation, folder, scans):
    from methodology import airborne_diagnostics
    started = perf_counter()
    reference = airborne_diagnostics.MatchedHallReference(simulation.fields, simulation.checkpoint_state()['sensors'],
        simulation.identities, simulation.plant.geometry.center_of_mass_height, simulation.dynamics.current)
    trajectory = airborne_diagnostics.recorded_dense_trajectory(folder / 'dense.npz', 'physical')
    checks = []
    for scan in scans:
        check = reference.check(trajectory, scan)
        checks.append(check)
        if not check['passed']:
            break
    return dict(passed=bool(checks) and len(checks) == len(scans) and all(row['passed'] for row in checks),
        checks=checks, elapsed_s=perf_counter() - started)


def run(output, reference_folder, field_folder):
    if output.exists():
        raise ValueError('Preserve existing local-motion evidence')
    reference = json.loads((reference_folder / 'summary.json').read_text())
    initial = json.loads((reference_folder / 'initial_checkpoint.json').read_text())
    field_screen = json.loads((field_folder / 'summary.json').read_text())
    if not reference['passed'] or field_screen['status'] != 'field_screen_passed':
        raise ValueError('Local motion requires passing hover and current-source field evidence')
    for name, digest in field_screen['source_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError('Field-screen source changed: ' + name)
    atlas = MagneticFieldAtlas.load(field_folder / 'atlas.npz')
    if atlas.identity != field_screen['atlas_identity']:
        raise ValueError('Field-screen atlas identity differs')
    construction = magnetics.ConstructionModel(recorded_piece_properties=initial['construction']['piece_properties'])
    construction.use_cad_mass = initial['construction']['use_cad_mass']
    construction.return_depth = initial['construction']['return_depth']
    duration = Fixed.airborne_motion_start + Fixed.airborne_motion_duration + Fixed.airborne_motion_settle_duration
    simulation = JointAirborneSimulation(construction, duration, atlas=atlas, magnetic_mode='atlas_forces',
        motion_scenario='local_move')
    output.mkdir(parents=True)
    source = dict(simulation.source, **source_fingerprints(__file__, numerical_accuracy.__file__, hover.__file__, airborne_verification.__file__))
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in source:
            archive.write(name, name)
    simulation.save_checkpoint(output / 'initial_checkpoint.json')
    context = dict(status='preflight', scope='First smooth local translation and rotation with sensor-only control; not full flight qualification',
        source_sha256=source, atlas_identity=atlas.identity,
        inputs_sha256=source_fingerprints(reference_folder / 'summary.json', reference_folder / 'initial_checkpoint.json', field_folder / 'summary.json'),
        trajectory=dict(translation_m=Fixed.airborne_motion_translation, rotation_vector_rad=Fixed.airborne_motion_rotation,
            start_s=Fixed.airborne_motion_start, movement_duration_s=Fixed.airborne_motion_duration,
            endpoint_observation_s=Fixed.airborne_motion_settle_duration),
        preflight=[], hover_prefix_comparisons=[], scans=[])
    context_path = output / 'evidence.json'
    airborne_verification.save_summary(context_path, context)
    count = int(round(duration / Fixed.airborne_checkpoint_interval))
    for time in np.linspace(0., duration, count + 1):
        pose, velocity, acceleration = simulation.reference_state(time)
        estimate = dict(valid=True, pose=pose, velocity=np.r_[velocity[:3],
            np.linalg.solve(airborne.euler_rate_axes(pose), velocity[3:])])
        allocation = simulation.controller.command(estimate, pose, velocity, acceleration)
        context['preflight'].append(dict(time_s=float(time), accepted=allocation['accepted'],
            required_current_A=float(allocation['required_current_A'])))
        if not allocation['accepted']:
            context['status'] = 'preflight_current_failed'
            airborne_verification.save_summary(context_path, context)
            return context
    simulation.load_checkpoint(output / 'initial_checkpoint.json')
    recorder = JointTrajectoryRecorder(simulation)

    def progress(report):
        hover.check_operating_limits(simulation, report)
        if simulation.stop_reason is None and report['duration_completed_s'] <= Fixed.airborne_motion_start and report['trace']:
            prefix = dict(reference, trace=reference['trace'][:len(report['trace'])])
            comparison = airborne_verification.compare_trials(prefix, report, trace_only=True)
            context['hover_prefix_comparisons'].append(dict(time_s=report['duration_completed_s'], **comparison))
            if not comparison['converged'] and simulation.stop_reason is None:
                simulation.stop_reason = 'stationary_prefix_regression_failed'
                simulation.finished = True
        context['scans'] = recorder.scans
        context['dense_sha256'] = recorder.save_dense(output / 'dense.npz')
        context.update(completed_duration_s=report['duration_completed_s'], stop_reason=simulation.stop_reason,
            status='failed' if simulation.stop_reason is not None else 'running')
        airborne_verification.save_summary(context_path, context)

    context['status'] = 'running'
    airborne_verification.save_summary(context_path, context)
    with patch.object(simulation.dynamics, 'integrate', recorder.integrate), patch.object(simulation.sensors, 'encode_signal', recorder.encode_signal):
        report = airborne.run_simulation(simulation, output / 'summary.json', output / 'checkpoint.json', progress=progress)
    context.update(status='passed' if report['passed'] else 'failed', elapsed_s=report['elapsed_s'])
    airborne_verification.save_summary(context_path, context)
    return context


def run_cached(output, field_folder, library, resume=False, deadline=None, readiness=False, pause_at=None, start_checkpoint=None):
    started = perf_counter()
    from methodology.coil_field import CoilFieldCache
    from methodology.magnetic_spline import native_evaluation
    if output.exists() != resume:
        raise ValueError('Use a fresh output directory, or explicitly resume an existing run')
    screen = json.loads((field_folder / 'summary.json').read_text())
    cache = CoilFieldCache.load(field_folder / 'atlas.npz')
    if (screen['status'] != 'field_screen_passed' or screen['atlas_identity'] != cache.identity
            or screen['source_sha256'] != cache.provenance['source_sha256']):
        raise ValueError('Production coil-cache screen or identity differs')
    inputs = source_fingerprints(field_folder / 'summary.json', field_folder / 'atlas.npz')
    if start_checkpoint is not None:
        inputs.update(source_fingerprints(start_checkpoint))
    checkpoint = output / 'checkpoint.json'
    evidence_path = output / 'evidence.json'
    with native_evaluation(cache, library):
        duration = (Fixed.airborne_readiness_duration if readiness else
            Fixed.airborne_motion_start + Fixed.airborne_motion_duration + Fixed.airborne_motion_settle_duration)
        if resume:
            context = json.loads(evidence_path.read_text())
            if inputs != context['inputs_sha256']:
                raise ValueError('Local-motion cache inputs changed')
            for name, digest in context['source_sha256'].items():
                if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
                    raise ValueError('Local-motion source changed: ' + name)
            simulation = JointAirborneSimulation.from_checkpoint(checkpoint, atlas=cache)
            if simulation.clock != context['completed_duration_s'] or simulation.duration != duration:
                raise ValueError('Local-motion checkpoint and recording clocks differ')
        else:
            if start_checkpoint is None:
                simulation = JointAirborneSimulation(magnetics.ConstructionModel(), duration, atlas=cache,
                    magnetic_mode='coil_cache', motion_scenario='local_move')
            else:
                simulation = JointAirborneSimulation.from_checkpoint(start_checkpoint, atlas=cache)
                if simulation.clock != 0 or simulation.duration != duration or simulation.motion_scenario != 'local_move':
                    raise ValueError('Local-motion start requires an initial checkpoint for this scenario')
                if simulation.checkpoint_state() != json.loads(start_checkpoint.read_text()):
                    raise ValueError('Local-motion initial checkpoint did not restore exactly')
            output.mkdir(parents=True)
            source = dict(simulation.source, **source_fingerprints(__file__, numerical_accuracy.__file__, hover.__file__))
            with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
                for name in source:
                    archive.write(name, name)
            simulation.save_checkpoint(output / 'initial_checkpoint.json')
            context = dict(scope='Sensor-feedback local movement using the production coil cache; bounded flight evidence',
                source_sha256=source, inputs_sha256=inputs, atlas_identity=cache.identity, scans=[], dense_sha256=None, runtime_s={})
            if not readiness:
                if not 0 < Fixed.airborne_settling_window <= Fixed.airborne_motion_settle_duration:
                    raise ValueError('Settling window must fit inside the stationary endpoint hold')
                context['settling'] = dict(start_s=duration - Fixed.airborne_settling_window, end_s=duration,
                    samples=0, observed_until_s=None, peaks=dict.fromkeys(settling_limits(), 0.),
                    limits=settling_limits(), limits_met=True, passed=False)
        recorder = JointTrajectoryRecorder(simulation)
        if resume and context['dense_sha256'] is not None:
            if hashlib.sha256((output / 'dense.npz').read_bytes()).hexdigest() != context['dense_sha256']:
                raise ValueError('Local-motion dense recording changed')
            recorder.restore_dense(output / 'dense.npz', context['scans'])

        previous_runtime = dict(context['runtime_s'])
        setup_elapsed = perf_counter() - started
        recording_elapsed = 0.
        observed_segments = 0
        run_started = perf_counter()

        def progress(report):
            nonlocal recording_elapsed, observed_segments
            recording_started = perf_counter()
            hover.check_operating_limits(simulation, report)
            if not readiness:
                observe_settling(simulation, recorder.segments[observed_segments:], context['settling'])
                observed_segments = len(recorder.segments)
            context.update(scans=recorder.scans, dense_sha256=recorder.save_dense(output / 'dense.npz'),
                completed_duration_s=simulation.clock, stop_reason=simulation.stop_reason,
                status=('failed' if simulation.stop_reason is not None or (simulation.finished and not report['passed'])
                    else 'passed' if simulation.finished else 'paused'))
            recording_elapsed += perf_counter() - recording_started
            times = dict(setup=setup_elapsed, integration=recorder.integration_elapsed_s,
                recording=recording_elapsed, run=perf_counter() - run_started, total=perf_counter() - started)
            context['runtime_s'] = {name: previous_runtime.get(name, 0.) + value for name, value in times.items()}
            airborne_verification.save_summary(evidence_path, context)

        with patch.object(simulation.dynamics, 'integrate', recorder.integrate), patch.object(simulation.sensors, 'encode_signal', recorder.encode_signal):
            report = airborne.run_simulation(simulation, output / 'summary.json', checkpoint, deadline=deadline, progress=progress, pause_at=pause_at)
        progress(report)
        simulation.save_checkpoint(checkpoint)
        airborne_verification.save_summary(output / 'summary.json', simulation.report())
        if not readiness and recorder.scans and simulation.finished:
            sensing = verify_recorded_sensing(simulation, output, recorder.scans)
            passed = simulation.report()['passed'] and context['settling']['passed'] and sensing['passed']
            context.update(status='passed' if passed else 'failed', verification_passed=passed)
            context['runtime_s']['verification'] = sensing['elapsed_s']
            airborne_verification.save_summary(evidence_path, context)
            verification = dict(source_sha256=context['source_sha256'],
                inputs_sha256=source_fingerprints(output / 'summary.json', evidence_path, checkpoint, output / 'dense.npz'),
                control_passed=simulation.report()['passed'], settling=context['settling'],
                sensing=sensing, passed=passed)
            airborne_verification.save_summary(output / 'verification.json', verification)
        return context


def verify_readiness(output, field_folder, library, deadline, reference_run=None):
    import levitation_sim
    from methodology import airborne_diagnostics, coil_field, magnetic_spline, physics_verification
    if output.exists():
        raise ValueError('Preserve existing readiness evidence')
    output.mkdir(parents=True)
    started = perf_counter()
    cache = coil_field.CoilFieldCache.load(field_folder / 'atlas.npz')
    screen = json.loads((field_folder / 'summary.json').read_text())
    if screen['status'] != 'field_screen_passed' or screen['atlas_identity'] != cache.identity:
        raise ValueError('Readiness requires passing production field evidence')
    result = dict(status='running', scope='Integrated coil-cache readiness for the nominal local CAD movement, not flight-envelope qualification',
        source_sha256=dict(cache.provenance['source_sha256'], **source_fingerprints(__file__, airborne_diagnostics.__file__)),
        input_sha256=source_fingerprints(field_folder / 'summary.json', field_folder / 'atlas.npz', library, library.with_suffix('.json')),
        cache_identity=cache.identity, checks={}, matrix_checks=[], processes=[])

    def save():
        result['elapsed_s'] = perf_counter() - started
        airborne_verification.save_summary(output / 'summary.json', result)

    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in result['source_sha256']:
            archive.write(name, name)
    save()
    with magnetic_spline.native_evaluation(cache, library) as runtime:
        simulation = JointAirborneSimulation(magnetics.ConstructionModel(), Fixed.airborne_readiness_duration,
            initialize=False, atlas=cache, magnetic_mode='coil_cache', motion_scenario='local_move')
        result['runtime'] = runtime
        for time in (0., Fixed.airborne_readiness_duration, Fixed.airborne_motion_start + Fixed.airborne_motion_duration):
            pose, _, _ = simulation.reference_state(time)
            _, center = levitation_sim.place_piece(simulation.controller.magnets, *pose)
            matrix = simulation.controller.matrix_evaluator(simulation.controller.magnets.collection, center)
            reference = levitation_sim.actuator_matrix(simulation.controller.magnets, simulation.controller.array, center)
            errors = physics_verification.matrix_refinement_metrics(matrix, reference, Fixed.driver_channel_current)
            passed = all(errors[key] <= Fixed.atlas_matrix_relative_tolerance
                for key in ('force_relative_column_error', 'torque_relative_column_error'))
            result['matrix_checks'].append(dict(time_s=time, pose=pose.tolist(), errors=errors, passed=passed))
            if not passed:
                result['status'] = 'matrix_failed'
                save()
                return result
    result['checks']['integrated_matrices'] = True
    if reference_run is None:
        reference_run = output / 'uninterrupted'
        jobs = [('uninterrupted', reference_run, [])]
    else:
        recorded = airborne_diagnostics.verify_recorded_archive(reference_run)
        if not recorded['passed'] or recorded['source_sha256'] != simulation.source:
            raise ValueError('Retained readiness reference differs from current physical sources or did not pass')
        result['input_sha256'].update(source_fingerprints(*[reference_run / name for name in
            ('summary.json', 'initial_checkpoint.json', 'checkpoint.json', 'evidence.json', 'dense.npz', 'sources.zip')]))
        result['checks']['uninterrupted'] = True
        jobs = []
    start = reference_run / 'initial_checkpoint.json'
    jobs.extend([('pause', output / 'restarted', ['--pause-for-restart', '--start-checkpoint', str(start)]),
        ('resume', output / 'restarted', ['--resume', '--start-checkpoint', str(start)])])
    result['reference_run'] = str(reference_run)
    save()
    for label, folder, options in jobs:
        command = [sys.executable, '-m', 'methodology.local_motion', '--coil-cache', '--readiness',
            '--output', str(folder), '--fields', str(field_folder), '--native-library', str(library),
            '--deadline', deadline.isoformat(), *options]
        process_started = perf_counter()
        with (output / (label + '.log')).open('w') as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        result['processes'].append(dict(stage=label, command=command, returncode=process.returncode,
            elapsed_s=perf_counter() - process_started))
        if process.returncode:
            result['status'] = label + '_process_failed'
            save()
            return result
        context = json.loads((folder / 'evidence.json').read_text())
        state = json.loads((folder / 'checkpoint.json').read_text())
        if label == 'pause':
            samples = [scan['sample_count'] for scan in state['joint']['pending_scans'].values()]
            expected = context['status'] == 'paused' and state['stop_reason'] is None and any(count > 0 for count in samples)
            result['restart_pause'] = dict(time_s=state['joint']['clock'], pending_sample_counts=samples)
        else:
            expected = context['status'] == 'passed'
        result['checks'][label] = expected
        if not expected:
            result['status'] = label + '_check_failed'
            save()
            return result
        save()
    folders = [reference_run, output / 'restarted']
    initial = [json.loads((folder / 'initial_checkpoint.json').read_text()) for folder in folders]
    result['checks']['initial_checkpoint_exact'] = initial[0] == initial[1]
    states = [json.loads((folder / 'checkpoint.json').read_text()) for folder in folders]
    contexts = [json.loads((folder / 'evidence.json').read_text()) for folder in folders]
    result['checks']['checkpoint_exact'] = states[0] == states[1]
    result['checks']['scans_exact'] = contexts[0]['scans'] == contexts[1]['scans']
    with np.load(folders[0] / 'dense.npz', allow_pickle=False) as first, np.load(folders[1] / 'dense.npz', allow_pickle=False) as second:
        result['checks']['dense_exact'] = set(first.files) == set(second.files) and all(np.array_equal(first[name], second[name]) for name in first.files)
    result['runtime_s'] = {name: context['runtime_s'] for name, context in zip(('uninterrupted', 'restarted'), contexts)}
    if not all(result['checks'].values()):
        result['status'] = 'restart_mismatch'
        save()
        return result
    with magnetic_spline.native_evaluation(cache, library):
        simulation = JointAirborneSimulation.from_checkpoint(folders[0] / 'checkpoint.json', atlas=cache)
        sensing = verify_recorded_sensing(simulation, folders[0], contexts[0]['scans'])
        result['sensing_checks'] = sensing['checks']
        result['sensing_verification_s'] = sensing['elapsed_s']
    result['checks']['independent_sensing'] = sensing['passed']
    result['status'] = 'simulation_ready' if all(result['checks'].values()) else 'readiness_failed'
    save()
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run a bounded local movement with the verified joint solver.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path)
    parser.add_argument('--fields', type=Path, required=True)
    parser.add_argument('--coil-cache', action='store_true', help='Use the integrated coil-field backend.')
    parser.add_argument('--native-library', type=Path)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--start-checkpoint', type=Path, help='Start a repeated experiment from an identical production initial state.')
    parser.add_argument('--reference-run', type=Path, help='Reuse an unchanged completed readiness reference.')
    parser.add_argument('--readiness', action='store_true', help='Run the short integration-readiness movement prefix.')
    parser.add_argument('--pause-for-restart', action='store_true')
    parser.add_argument('--verify-ready', action='store_true', help='Run the bounded integrated-backend readiness checks.')
    parser.add_argument('--deadline', type=datetime.fromisoformat)
    args = parser.parse_args()
    if args.reference_run is not None and not args.verify_ready:
        parser.error('A retained reference is only used for readiness verification')
    if args.start_checkpoint is not None and (not args.coil_cache or args.verify_ready):
        parser.error('An initial checkpoint belongs to a normal coil-cache run')
    if args.pause_for_restart and (not args.readiness or args.resume or args.verify_ready):
        parser.error('A readiness restart pause requires a fresh short run')
    if args.coil_cache:
        if args.reference is not None or args.native_library is None or args.deadline is None or args.deadline.tzinfo is None:
            parser.error('Coil runs require a native library and timezone-aware deadline, without a hover reference')
        if args.verify_ready:
            if args.resume or args.readiness:
                parser.error('The readiness batch owns its short-run and resume options')
            result = verify_readiness(args.output, args.fields, args.native_library, args.deadline, args.reference_run)
        else:
            result = run_cached(args.output, args.fields, args.native_library, args.resume, args.deadline.timestamp(),
                args.readiness, Fixed.airborne_readiness_pause if args.pause_for_restart else None, args.start_checkpoint)
    else:
        if args.reference is None or args.native_library is not None or args.resume or args.deadline is not None or args.readiness or args.verify_ready or args.pause_for_restart:
            parser.error('Legacy local movement requires a hover reference without coil-cache options')
        result = run(args.output, args.reference, args.fields)
    print(json.dumps(dict(status=result['status'])), flush=True)

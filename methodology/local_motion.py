import argparse
import hashlib
import json
from pathlib import Path
from unittest.mock import patch
from zipfile import ZipFile, ZIP_DEFLATED

import numpy as np

from model import Fixed
from methodology import airborne, airborne_verification, hover, numerical_accuracy, source_fingerprints
from methodology.cad import magnetics
from methodology.joint_airborne import JointAirborneSimulation
from methodology.magnetic_atlas import MagneticFieldAtlas
from methodology.numerical_accuracy import JointTrajectoryRecorder


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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run a bounded local movement with the verified joint solver.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--fields', type=Path, required=True)
    args = parser.parse_args()
    result = run(args.output, args.reference, args.fields)
    print(json.dumps(dict(status=result['status'])), flush=True)

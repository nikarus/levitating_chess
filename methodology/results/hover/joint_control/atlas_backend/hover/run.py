import hashlib
import json
from pathlib import Path
from unittest.mock import patch

from model import Fixed
from methodology import airborne, airborne_verification, hover, numerical_accuracy, source_fingerprints
from methodology.cad import magnetics
from methodology.joint_airborne import JointAirborneSimulation
from methodology.magnetic_atlas import MagneticFieldAtlas
from methodology.numerical_accuracy import JointTrajectoryRecorder


output = Path(__file__).resolve().parent
backend = output.parent
reference_folder = backend.parent
context_path = output / 'comparison.json'
if context_path.exists() or (output / 'initial_checkpoint.json').exists():
    raise ValueError('Preserve existing accelerated hover evidence')
gate = json.loads((backend / 'sensing/summary.json').read_text())
if gate['status'] != 'completed':
    raise ValueError('Accelerated hover requires a passing combined sensing gate')
for name, digest in gate['source_sha256'].items():
    if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
        raise ValueError('Simulation source changed since the sensing gate: ' + name)
reference = json.loads((reference_folder / 'summary.json').read_text())
initial = json.loads((reference_folder / 'initial_checkpoint.json').read_text())
if not reference['passed'] or reference['intended_duration_s'] != Fixed.airborne_refinement_duration:
    raise ValueError('Direct hover reference does not match the intended horizon')
atlas = MagneticFieldAtlas.load(backend / 'fields/atlas.npz')
if atlas.identity != gate['atlas_identity']:
    raise ValueError('Atlas identity differs from the sensing gate')
construction = magnetics.ConstructionModel(recorded_piece_properties=initial['construction']['piece_properties'])
construction.use_cad_mass = initial['construction']['use_cad_mass']
construction.return_depth = initial['construction']['return_depth']
simulation = JointAirborneSimulation(construction, Fixed.airborne_refinement_duration,
    atlas=atlas, magnetic_mode='atlas_forces')
current = simulation.checkpoint_state()
original_state = {key: value for key, value in initial.items() if key not in ('schema', 'source_sha256', 'plant')}
current_state = {key: value for key, value in current.items() if key not in ('schema', 'source_sha256', 'plant', 'atlas')}
if original_state != current_state:
    raise ValueError('Fresh accelerated initial state differs from the direct reference')
if {key: value for key, value in initial['plant'].items() if key != 'source_sha256'} != {
        key: value for key, value in current['plant'].items() if key != 'source_sha256'}:
    raise ValueError('Fresh accelerated mechanics or CAD scene differs from the direct reference')
simulation.save_checkpoint(output / 'initial_checkpoint.json')
context = dict(status='running', scope='Cached/direct controlled-hover comparison over the retained reference horizon; not sustained or multi-piece flight qualification',
    initial_state_exact_except_source_schema_backend=True, source_sha256=simulation.source,
    runner_source_sha256=source_fingerprints(__file__, numerical_accuracy.__file__), atlas_identity=atlas.identity,
    inputs_sha256=source_fingerprints(reference_folder / 'summary.json', reference_folder / 'initial_checkpoint.json', backend / 'sensing/summary.json'),
    source_transition='Fresh current-source simulation from recorded CAD, with exact initial physical/controller/sensor state comparison; no old checkpoint was resumed or fingerprint replaced.',
    comparisons=[], scans=[])
recorder = JointTrajectoryRecorder(simulation)


def progress(report):
    hover.check_operating_limits(simulation, report)
    if report['trace']:
        prefix = dict(reference, trace=reference['trace'][:len(report['trace'])])
        comparison = airborne_verification.compare_trials(prefix, report, trace_only=True)
        context['comparisons'].append(dict(time_s=report['duration_completed_s'], **comparison))
        if not comparison['converged'] and simulation.stop_reason is None:
            simulation.stop_reason = 'atlas_hover_comparison_failed'
            simulation.finished = True
    context['scans'] = recorder.scans
    context['dense_sha256'] = recorder.save_dense(output / 'dense.npz')
    context.update(completed_duration_s=report['duration_completed_s'], stop_reason=simulation.stop_reason,
        status='failed' if simulation.stop_reason is not None else 'running')
    airborne_verification.save_summary(context_path, context)


airborne_verification.save_summary(context_path, context)
with patch.object(simulation.dynamics, 'integrate', recorder.integrate), patch.object(simulation.sensors, 'encode_signal', recorder.encode_signal):
    report = airborne.run_simulation(simulation, output / 'summary.json', output / 'checkpoint.json', progress=progress)
if report['passed']:
    context['final_comparison'] = airborne_verification.compare_trials(reference, report)
    context['status'] = 'passed' if context['final_comparison']['converged'] else 'comparison_failed'
else:
    context['status'] = 'failed'
context['elapsed_s'] = report['elapsed_s']
airborne_verification.save_summary(context_path, context)
print(json.dumps(dict(status=context['status'], completed_s=report['duration_completed_s'], stop_reason=report['stop_reason'])), flush=True)

import json
from pathlib import Path

from model import Fixed
from methodology import airborne, airborne_verification, hover, source_fingerprints
from methodology.joint_airborne import JointAirborneSimulation
from methodology.magnetic_atlas import MagneticFieldAtlas


output = Path(__file__).resolve().parent
root = output.parent
summary_path = output / 'summary.json'
if summary_path.exists():
    raise ValueError('Preserve existing coupled comparison evidence')
gate = json.loads((root / 'integration' / 'summary.json').read_text())
if gate['status'] != 'completed':
    raise ValueError('Joint atlas interval comparison has not passed')
atlas = MagneticFieldAtlas.load(root / 'fields' / 'atlas.npz')
if atlas.identity != gate['atlas_identity']:
    raise ValueError('Verified atlas identity differs')
summary = dict(scope='Bounded joint sensor-feedback comparison; not sustained hover or combined sensing-budget qualification',
    status='running', atlas_identity=atlas.identity, runner_source_sha256=source_fingerprints(__file__),
    cases={}, comparisons={})
reports = {}
for name, initial, scale in (
        ('direct', 'initial_checkpoint.json', 1),
        ('atlas', 'atlas_initial_checkpoint.json', 1),
        ('tighter', 'atlas_initial_checkpoint.json', 1 / Fixed.qualification_refinement_factor)):
    simulation = JointAirborneSimulation.from_checkpoint(root / 'integration' / initial,
        atlas=None if name == 'direct' else atlas)
    if simulation.source != gate['source_sha256'] or simulation.duration != Fixed.adaptive_benchmark_duration:
        raise ValueError('Coupled comparison source or duration differs from its gate')
    simulation.tolerance_scale = scale
    summary['active_case'] = name
    summary['source_sha256'] = simulation.source
    airborne_verification.save_summary(summary_path, summary)

    def progress(report):
        hover.check_operating_limits(simulation, report)
        if name != 'direct' and report['trace'] and simulation.stop_reason is None:
            reference = dict(reports['direct'], trace=reports['direct']['trace'][:len(report['trace'])])
            comparison = airborne_verification.compare_trials(reference, report, trace_only=True)
            summary['comparisons'][name] = comparison
            if not comparison['converged']:
                simulation.stop_reason = 'atlas_coupled_comparison_failed'
                simulation.finished = True
        airborne_verification.save_summary(summary_path, summary)

    report = airborne.run_simulation(simulation, output / (name + '.json'),
        output / (name + '_checkpoint.json'), progress=progress)
    reports[name] = report
    summary['cases'][name] = {key: report[key] for key in ('passed', 'finished', 'stop_reason',
        'duration_completed_s', 'elapsed_s', 'peaks', 'integration', 'delivered_moving_scans')}
    if name != 'direct':
        incomplete = not report['finished'] or report['stop_reason'] is not None
        reference = (dict(reports['direct'], trace=reports['direct']['trace'][:len(report['trace'])])
                     if incomplete else reports['direct'])
        summary['comparisons'][name] = airborne_verification.compare_trials(reference, report, trace_only=incomplete)
    if not report['passed'] or (name != 'direct' and not summary['comparisons'][name]['converged']):
        summary['status'] = 'failed'
        airborne_verification.save_summary(summary_path, summary)
        break
else:
    summary.update(status='completed', active_case=None)
    airborne_verification.save_summary(summary_path, summary)
print(json.dumps(summary), flush=True)

import hashlib
import json
from pathlib import Path
import subprocess

from model import Fixed
from methodology import airborne, airborne_verification, hover, source_fingerprints
from methodology.cad import magnetics
from methodology.joint_airborne import JointAirborneSimulation


output = Path(__file__).resolve().parent
root = output.parents[3]
evidence = root / 'methodology/results/physics_verification/joint_passive_filter'
qualification = json.loads((evidence / 'qualification.json').read_text())
if qualification['status'] != 'ready_for_bounded_airborne_control_review':
    raise ValueError('Controlled run requires the verified joint candidate')
for name, digest in qualification['source_sha256'].items():
    if hashlib.sha256((root / name).read_bytes()).hexdigest() != digest:
        raise ValueError('Verified source changed: ' + name)
if (output / 'initial_checkpoint.json').exists() or (output / 'summary.json').exists():
    raise ValueError('Controlled run evidence already exists; preserve it')
record = json.loads((evidence / 'direct_screen/initial_checkpoint.json').read_text())
construction = magnetics.ConstructionModel(recorded_piece_properties=record['construction']['piece_properties'])
construction.use_cad_mass = record['construction']['use_cad_mass']
construction.return_depth = record['construction']['return_depth']
simulation = JointAirborneSimulation(construction, Fixed.airborne_refinement_duration)
simulation.save_checkpoint(output / 'initial_checkpoint.json')
simulation.save_checkpoint(output / 'checkpoint.json')
airborne_verification.save_summary(output / 'summary.json', simulation.report())
context = dict(scope='First sensor-controlled nominal king hover with the verified joint integrator; bounded startup, not sustained flight qualification',
    git_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
    target_duration_s=simulation.duration, source_sha256=simulation.source,
    runner_source_sha256=source_fingerprints(__file__),
    verification_sha256=source_fingerprints(evidence / 'qualification.json'),
    dependencies=qualification['dependencies'], initial_state='Stationary sensor acquisition, interior release, zero winding currents',
    checkpoint_interval_s=Fixed.airborne_checkpoint_interval,
    stopping_rules='Existing physical/current/tracking/energy guards, plus existing estimated-tracking limits at every checkpoint',
    status='running')
airborne_verification.save_summary(output / 'run_context.json', context)
report = airborne.run_simulation(simulation, output / 'summary.json', output / 'checkpoint.json',
    progress=lambda report: hover.check_operating_limits(simulation, report))
context.update(status='passed' if report['passed'] else 'failed',
    completed_duration_s=report['duration_completed_s'], stop_reason=report['stop_reason'])
airborne_verification.save_summary(output / 'run_context.json', context)
print(json.dumps(dict(status=context['status'], completed_s=report['duration_completed_s'],
    stop_reason=report['stop_reason'], peaks=report['peaks'], integration=report['integration']), indent=Fixed.report_precision))

import argparse
import json
from pathlib import Path
from types import MethodType
from unittest.mock import patch

import numpy as np

from model import Fixed
from methodology import airborne_verification, electrical, hover, source_fingerprints
from methodology.magnetic_atlas import AtlasAirborneSimulation


original_simulation_factory = hover.simulation_from_evidence
original_reference_worker = hover.reference_worker


def set_unrounded_command(drivers, command):
    electrical.CoilDrivers.set_command(drivers, command)
    drivers.command = np.clip(np.asarray(command, dtype=float), -drivers.current_limit, drivers.current_limit)


def unrounded_report(simulation):
    report = AtlasAirborneSimulation.report(simulation)
    report['scope'] = 'Coupled command-rounding isolation; ideal command precision, not a hardware qualification'
    report['current_conversion']['command_step_A'] = 0.
    report['command_rounding'] = 'disabled_for_diagnosis'
    report['limitations'].append('Only command rounding is disabled; current clipping, filtering, sensor ADCs, noise, calibration and hardware rates remain unchanged.')
    return report


def disable_rounding(simulation):
    simulation.drivers.set_command = MethodType(set_unrounded_command, simulation.drivers)
    simulation.report = MethodType(unrounded_report, simulation)
    simulation.checkpoint_schema = 'airborne-unrounded-command-v1'
    simulation.source.update(source_fingerprints(__file__))


def simulation_factory(evidence, atlas_path, initialize, fixed_step=None):
    simulation = original_simulation_factory(evidence, atlas_path, initialize, fixed_step)
    disable_rounding(simulation)
    return simulation


def reference_worker(connection, evidence, atlas_path, output, window, deadline):
    with patch.object(hover, 'simulation_from_evidence', simulation_factory):
        original_reference_worker(connection, evidence, atlas_path, output, window, deadline)


def run(baseline_path, reference_path, initial_path, atlas_path, output):
    if output.exists():
        raise ValueError('Command-precision output already exists; preserve its evidence')
    baseline = json.loads(baseline_path.read_text())
    reference = json.loads(reference_path.read_text())
    target = reference['duration_completed_s']
    saved = original_simulation_factory(reference, atlas_path, False, reference['fixed_reference_step_s'])
    saved.load_checkpoint(reference_path.with_name(reference_path.stem + '_checkpoint.json'))
    if saved.trace != reference['trace'] or saved.clock != target:
        raise ValueError('Recorded reference report and checkpoint disagree')
    simulation = original_simulation_factory(baseline, atlas_path, False)
    simulation.load_checkpoint(initial_path)
    if simulation.clock != 0 or simulation.finished or simulation.source != saved.source:
        raise ValueError('Command isolation requires the matching original release state')
    disable_rounding(simulation)
    output.mkdir(parents=True)
    summary = dict(scope='Adaptive versus fine integration with command rounding disabled in both; original rounded pair retained separately',
        source_sha256=simulation.source, inputs_sha256=source_fingerprints(baseline_path, reference_path, initial_path),
        atlas_identity=saved.atlas_configuration['identity'], target_duration_s=target,
        original_command_step_A=2 * Fixed.max_current_command_error, diagnostic_command_step_A=0.,
        status='running', active_window=None, windows={})
    gate = hover.NumericalGate(simulation, baseline, atlas_path, output, summary, float('inf'))
    hover.save_trial(simulation, output, 'adaptive')
    with patch.object(hover, 'reference_worker', reference_worker):
        gate.start(dict(name='initial_reference', end_s=target))
        while not simulation.finished and simulation.clock < target - Fixed.adaptive_clock_tolerance:
            simulation.advance()
            if simulation.checkpoint_due() or simulation.finished:
                gate.progress(simulation.report())
                hover.save_trial(simulation, output, 'adaptive')
        gate.close()
    hover.save_trial(simulation, output, 'adaptive')
    complete = simulation.clock >= target - Fixed.adaptive_clock_tolerance and simulation.stop_reason is None
    summary.update(status='completed' if complete else 'failed', duration_completed_s=simulation.clock,
        stop_reason=simulation.stop_reason, control_qualification_passed=False,
        elapsed_s=simulation.report()['elapsed_s'])
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Isolate command rounding using paired coupled adaptive/fine integrations.')
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--initial', type=Path, required=True)
    parser.add_argument('--atlas', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    print(json.dumps(run(arguments.baseline, arguments.reference, arguments.initial, arguments.atlas, arguments.output)), flush=True)

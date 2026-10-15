import argparse
from copy import copy
from datetime import datetime
import hashlib
import json
import multiprocessing
from pathlib import Path
from time import time

import numpy as np

from model import Fixed
from methodology import airborne, airborne_verification, source_fingerprints
from methodology.magnetic_atlas import AtlasAirborneSimulation, MagneticFieldAtlas


def simulation_from_evidence(evidence, atlas_path, initialize, fixed_step=None):
    simulation = AtlasAirborneSimulation(airborne_verification.recorded_construction(evidence),
        MagneticFieldAtlas.load(atlas_path), 'atlas_forces', Fixed.airborne_hover_duration,
        initialize=initialize, fixed_step=fixed_step)
    simulation.source.update(source_fingerprints(__file__))
    return simulation


def comparison_snapshot(simulation):
    report = simulation.report()
    if not report['trace'] or report['trace'][-1]['time_s'] < simulation.clock:
        truth = simulation.plant.truth()
        estimate = copy(simulation.tracker).predict(simulation.clock)
        row = dict(time_s=simulation.clock,
            pose=np.r_[truth['com_position_m'] - [0, 0, simulation.plant.geometry.center_of_mass_height],
                       truth['rotation'].as_euler('xyz')].tolist(),
            estimated_pose=estimate['pose'] if estimate['valid'] else None,
            currents_A=simulation.drivers.currents.tolist())
        report['trace'] = [*report['trace'], row]
    return report


def compare_prefixes(baseline, reference):
    comparison = airborne_verification.compare_trials(baseline, reference)
    del comparison['checks']['both_trials_passed']
    comparison['checks']['both_prefixes_within_limits'] = all(
        report['stop_reason'] is None
        and report['peaks']['estimated_position_error_m'] <= Fixed.qualification_position_error
        and report['peaks']['estimated_angle_error_rad'] <= Fixed.qualification_angle_error
        for report in (baseline, reference))
    comparison['converged'] = all(comparison['checks'].values())
    return comparison


def save_trial(simulation, output, name):
    simulation.save_checkpoint(output / (name + '_checkpoint.json'))
    airborne_verification.save_summary(output / (name + '.json'), simulation.report())


def check_operating_limits(simulation, report):
    if (report['peaks']['estimated_position_error_m'] > Fixed.qualification_position_error
            or report['peaks']['estimated_angle_error_rad'] > Fixed.qualification_angle_error):
        simulation.stop_reason = simulation.stop_reason or 'estimated_tracking_error_budget_exceeded'
    if report['peaks']['current_A'] > simulation.drivers.current_limit:
        simulation.stop_reason = simulation.stop_reason or 'current_limit_exceeded'
    if simulation.stop_reason is not None:
        simulation.finished = True


def reference_worker(connection, evidence, atlas_path, output, window, deadline):
    checkpoint = output / (window['name'] + '_checkpoint.json')
    resumed = checkpoint.exists()
    simulation = simulation_from_evidence(evidence, atlas_path, False,
        Fixed.adaptive_reference_step if resumed else None)
    simulation.load_checkpoint(checkpoint if resumed else output / (window['name'] + '_start.json'))
    simulation.fixed_step = Fixed.adaptive_reference_step

    def synchronize():
        check_operating_limits(simulation, simulation.report())
        save_trial(simulation, output, window['name'])
        connection.send(dict(kind='progress', report=comparison_snapshot(simulation)))
        proceed = connection.recv()
        if not proceed and not simulation.finished:
            simulation.stop_reason = 'peer_trial_failed'
            simulation.finished = True
            save_trial(simulation, output, window['name'])

    if resumed:
        synchronize()
    while not simulation.finished and simulation.clock < window['end_s'] - Fixed.adaptive_clock_tolerance:
        if simulation.checkpoint_due() and time() >= deadline:
            break
        simulation.advance()
        if simulation.checkpoint_due() or simulation.finished:
            synchronize()
    report = simulation.report()
    connection.send(dict(kind='complete', reached_end=simulation.clock >= window['end_s'] - Fixed.adaptive_clock_tolerance,
        stop_reason=simulation.stop_reason, elapsed_s=report['elapsed_s'], duration_completed_s=simulation.clock))
    connection.close()


class NumericalGate:
    def __init__(self, simulation, evidence, atlas_path, output, summary, deadline):
        self.simulation, self.evidence, self.atlas_path = simulation, evidence, atlas_path
        self.output, self.summary, self.deadline = output, summary, deadline
        self.connection = None
        self.process = None
        self.pending = None

    def start(self, window, resume=False):
        self.summary['active_window'] = window
        if not resume:
            self.simulation.save_checkpoint(self.output / (window['name'] + '_start.json'))
            self.summary['windows'][window['name']] = dict(start_s=self.simulation.clock, end_s=window['end_s'],
                scope='Independent integration from release' if self.simulation.clock == 0 else 'Local integration check from the shared saved state; not global convergence')
        airborne_verification.save_summary(self.output / 'summary.json', self.summary)
        context = multiprocessing.get_context('spawn')
        self.connection, child = context.Pipe()
        self.process = context.Process(target=reference_worker,
            args=(child, self.evidence, self.atlas_path, self.output, window, self.deadline), daemon=True)
        self.process.start()
        child.close()
        if resume:
            self.pending = self.connection.recv()
            if self.pending['kind'] == 'progress' and self.pending['report']['duration_completed_s'] == self.simulation.clock:
                self.progress(self.simulation.report())

    def finish(self, message):
        window = self.summary['active_window']
        self.summary['windows'][window['name']].update(message)
        self.process.join()
        if self.process.exitcode != 0:
            raise RuntimeError('Fine reference worker exited unsuccessfully')
        self.connection.close()
        self.connection, self.process = None, None
        if message['reached_end'] or message['stop_reason'] is not None:
            self.summary['active_window'] = None
        if message['stop_reason'] is not None:
            self.simulation.stop_reason = self.simulation.stop_reason or 'reference_trial_failed'
            self.simulation.finished = True

    def progress(self, report):
        check_operating_limits(self.simulation, report)
        if self.connection is not None:
            message = self.connection.recv() if self.pending is None else self.pending
            self.pending = None
            if message['kind'] == 'complete':
                self.finish(message)
            else:
                reference = message['report']
                if self.simulation.stop_reason is None and reference['stop_reason'] is None:
                    comparison = compare_prefixes(comparison_snapshot(self.simulation), reference)
                    name = self.summary['active_window']['name']
                    self.summary['windows'][name]['comparison'] = comparison
                    self.summary['windows'][name]['compared_through_s'] = self.simulation.clock
                    if not comparison['converged']:
                        self.simulation.stop_reason = 'numerical_comparison_failed'
                        self.simulation.finished = True
                elif reference['stop_reason'] is not None:
                    self.simulation.stop_reason = self.simulation.stop_reason or 'reference_trial_failed'
                    self.simulation.finished = True
                proceed = self.simulation.stop_reason is None
                self.connection.send(proceed)
                window = self.summary['active_window']
                if not proceed or reference['finished'] or reference['duration_completed_s'] >= window['end_s'] - Fixed.adaptive_clock_tolerance:
                    self.finish(self.connection.recv())
        if self.connection is None and self.simulation.stop_reason is None and time() < self.deadline:
            index = round(self.simulation.clock / Fixed.qualification_recovery_duration)
            start = index * Fixed.qualification_recovery_duration
            name = 'local_' + str(index)
            if (index > 0 and abs(self.simulation.clock - start) <= Fixed.adaptive_clock_tolerance
                    and start + Fixed.airborne_check_duration < self.simulation.duration
                    and name not in self.summary['windows']):
                self.start(dict(name=name, end_s=start + Fixed.airborne_check_duration))
        self.summary.update(status='failed' if self.simulation.stop_reason is not None else 'running',
            duration_completed_s=self.simulation.clock, stop_reason=self.simulation.stop_reason)
        airborne_verification.save_summary(self.output / 'summary.json', self.summary)

    def close(self):
        while self.connection is not None:
            message = self.connection.recv() if self.pending is None else self.pending
            self.pending = None
            if message['kind'] == 'complete':
                self.finish(message)
            else:
                self.connection.send(self.simulation.stop_reason is None)


def run(output, qualification, deadline, resume=False):
    qualified = json.loads(qualification.read_text())
    if qualified['status'] != 'completed' or not all(row['converged'] for row in qualified['comparisons'].values()):
        raise ValueError('Hover requires passing short numerical qualification')
    evidence = airborne_verification.verified_saved_trial(qualification.with_name('adaptive.json'))
    root = Path(__file__).resolve().parent.parent
    if any(hashlib.sha256((root / filename).read_bytes()).hexdigest() != digest for filename, digest in evidence['source_sha256'].items()):
        raise ValueError('Qualified simulation source differs')
    atlas_path = qualification.with_name('atlas.npz').resolve()
    simulation = simulation_from_evidence(evidence, atlas_path, not resume)
    if simulation.atlas_configuration['identity'] != qualified['atlas_identity']:
        raise ValueError('Qualified atlas identity differs')
    if resume:
        summary = json.loads((output / 'summary.json').read_text())
        if summary['status'] not in ('running', 'deadline'):
            raise ValueError('Only an unfinished hover job can resume')
        if summary['qualification_sha256'] != hashlib.sha256(qualification.read_bytes()).hexdigest():
            raise ValueError('Qualification evidence changed')
        simulation.load_checkpoint(output / 'adaptive_checkpoint.json')
    else:
        if output.exists():
            raise ValueError('Hover output exists; preserve previous evidence')
        output.mkdir(parents=True)
        summary = dict(scope='One-second disturbed hover with an independent first-50-ms reference and local later checks; not complete flight qualification',
            status='running', qualification_sha256=hashlib.sha256(qualification.read_bytes()).hexdigest(),
            source_sha256=simulation.source, atlas_identity=qualified['atlas_identity'],
            target_duration_s=simulation.duration, active_window=None, windows={})
        save_trial(simulation, output, 'adaptive')
    gate = NumericalGate(simulation, evidence, atlas_path, output, summary, deadline)
    if not resume:
        gate.start(dict(name='initial_reference', end_s=Fixed.airborne_refinement_duration))
    elif summary['active_window'] is not None:
        gate.start(summary['active_window'], resume=True)
    report = airborne.run_simulation(simulation, output / 'adaptive.json', output / 'adaptive_checkpoint.json',
        deadline=deadline, progress=gate.progress)
    gate.close()
    save_trial(simulation, output, 'adaptive')
    report = simulation.report()
    numerical_passed = all(row.get('reached_end', False) and row.get('comparison', {}).get('converged', False)
        for row in summary['windows'].values())
    summary.update(status='completed' if report['passed'] and numerical_passed else ('failed' if report['finished'] else 'deadline'),
        control_passed=report['passed'], duration_completed_s=simulation.clock, stop_reason=simulation.stop_reason,
        numerical_checks_passed=numerical_passed)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run resumable hover with automatic bounded finer-step comparisons.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--qualification', type=Path, required=True)
    parser.add_argument('--deadline', type=datetime.fromisoformat, required=True)
    parser.add_argument('--resume', action='store_true')
    arguments = parser.parse_args()
    if arguments.deadline.tzinfo is None:
        parser.error('Deadline must include a timezone')
    print(json.dumps(run(arguments.output, arguments.qualification, arguments.deadline.timestamp(), arguments.resume)), flush=True)

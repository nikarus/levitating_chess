import argparse
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
import hashlib
import json
import multiprocessing
from pathlib import Path

import numpy as np

from model import Fixed
from methodology import airborne, airborne_verification, electrical, hover, sensing, source_fingerprints
from methodology.magnetic_atlas import AtlasAirborneSimulation, MagneticFieldAtlas
from methodology.joint_airborne import JointAirborneSimulation


def reconstruct_instructions(drivers, initial, checkpoint, commands):
    for name in airborne.DRIVER_STATE:
        setattr(drivers, name, np.array(initial['drivers'][name]))
    command_at = {row['time_s']: row['command_A'] for row in commands}
    reading_at = {row['sample_times_s'][0]: electrical.CurrentReadings(drivers.identities,
        np.array(row['sample_times_s']), np.array(row['adc_codes']), row['available_time_s'])
        for row in checkpoint['readouts']}
    instructions = {}
    names = tuple(name for name in airborne.DRIVER_STATE if name != 'currents')
    for interval in checkpoint['adaptive']['adaptation']['intervals']:
        now = interval['start_s']
        if now in command_at:
            drivers.set_command(command_at[now])
        if now in reading_at:
            drivers.regulate(reading_at[now], now)
            instructions[now] = {name: getattr(drivers, name).tolist() for name in names}
        drivers.advance(np.zeros_like(drivers.currents), interval['interval_s'])
    if len(instructions) != len(reading_at):
        raise ValueError('Instruction reconstruction missed a recorded regulator update')
    for name in names:
        if not np.array_equal(getattr(drivers, name), checkpoint['drivers'][name]):
            raise ValueError(f'Reconstructed final controller state differs: {name}')
    return instructions


def physical_comparison(reference, candidate):
    reports = [dict(report, trace=[dict(row, estimated_pose=row['pose']) for row in report['trace']])
               for report in (reference, candidate)]
    result = airborne_verification.compare_trials(*reports, trace_only=True)
    for section in ('differences', 'limits', 'checks'):
        result[section] = {name: value for name, value in result[section].items() if not name.startswith('estimated_pose')}
    result['converged'] = all(result['checks'].values())
    return result


class SharedInputSimulation(AtlasAirborneSimulation):
    def configure(self, initial_path, instructions, target, fixed_step):
        self.source.update(source_fingerprints(hover.__file__))
        self.load_checkpoint(initial_path)
        if self.clock != 0 or self.finished:
            raise ValueError('Shared-input diagnosis requires the original release state')
        self.instructions = instructions
        self.instruction_identity = hashlib.sha256(json.dumps(instructions, sort_keys=True).encode()).hexdigest()
        self.target = target
        self.fixed_step = fixed_step
        self.checkpoint_schema = 'airborne-shared-input-v1'
        self.source.update(source_fingerprints(__file__))

    def physical_row(self):
        truth = self.plant.truth()
        return dict(time_s=self.clock,
            pose=np.r_[truth['com_position_m'] - [0, 0, self.plant.geometry.center_of_mass_height],
                       truth['rotation'].as_euler('xyz')].tolist(), currents_A=self.drivers.currents.tolist())

    def process_events(self, now, regulator_due, command_due, complete):
        if not self.field_times or self.field_times[-1] != now:
            total, coil = self.physical_fields()
            self.append_endpoint(now, dict(total=total, coil=coil))
        if regulator_due:
            for name, value in self.instructions[now].items():
                setattr(self.drivers, name, np.array(value))
        if now >= sensing.hall_available_time(self.next_scan / Fixed.pose_feedback_rate, len(self.fields.points)):
            self.next_scan += 1
        self.observe_physics(self.plant.truth())
        if np.max(np.abs(self.drivers.currents)) > self.drivers.current_limit:
            self.stop_reason = 'current_limit_exceeded'
        self.finished = bool(self.stop_reason is not None or complete)
        if command_due or self.finished:
            self.trace.append(self.physical_row())

    def report(self):
        report = super().report()
        trace = [*self.trace]
        if not trace or trace[-1]['time_s'] < self.clock:
            trace.append(self.physical_row())
        report.update(scope='Continuous plant under reconstructed recorded voltage instructions; sensors and feedback are not closed',
            passed=False, diagnostic_target_s=self.target, instruction_identity=self.instruction_identity,
            diagnostic_completed=self.clock >= self.target - Fixed.adaptive_clock_tolerance and self.stop_reason is None,
            trace=trace)
        report['limitations'] = ['Reuses the recorded calibrated/noisy/quantized controller outputs as fixed voltage instructions.',
            'Actual currents, back-EMF, magnetic forces and rigid-body dynamics are recomputed at every physical step.',
            'Hall delivery boundaries are retained, but no new sensor observation drives the controller.',
            'This diagnostic cannot establish sensor-only control or long-duration reference convergence.']
        return report

    def checkpoint_state(self):
        state = super().checkpoint_state()
        state['shared_input'] = dict(instruction_identity=self.instruction_identity, target_s=self.target)
        return state



class SharedJointInputSimulation(JointAirborneSimulation):
    physical_row = SharedInputSimulation.physical_row

    def configure_recording(self, initial, commands, regulators):
        if initial['schema'] != 'spline-evaluator-diagnostic-v1' or initial['joint']['clock'] != 0:
            raise ValueError('Joint drive replay requires a recorded diagnostic release')
        if initial['construction'] != self.construction_state() or initial['motion_scenario'] != self.motion_scenario:
            raise ValueError('Joint drive replay construction or reference differs')
        self.plant.restore_checkpoint(initial['plant'])
        for name in airborne.DRIVER_STATE:
            setattr(self.drivers, name, np.array(initial['drivers'][name]))
        for name in ('gain', 'offset', 'actual_coil_field'):
            setattr(self.sensors, name, np.array(initial['sensors'][name]))
        self.sensors.random.bit_generator.state = deepcopy(initial['sensors']['random'])
        for name in airborne.TRACKER_ARRAY_STATE:
            setattr(self.tracker, name, None if initial['tracker'][name] is None else np.array(initial['tracker'][name]))
        for name in airborne.TRACKER_CLOCK_STATE:
            setattr(self.tracker, name, initial['tracker'][name])
        self.hall_filter = np.array(initial['joint']['hall_filter'])
        self.commands = {row['time_s']: np.array(row['quantized_command_A']) for row in commands}
        self.instructions = {row['time_s']: row for row in regulators}
        self.instruction_identity = hashlib.sha256(json.dumps(dict(commands=commands, regulators=regulators), sort_keys=True).encode()).hexdigest()
        self.source.update(source_fingerprints(__file__))

    def process_events(self, now, regulator_due, command_due, complete):
        if not complete:
            if command_due:
                self.drivers.command = self.commands[now].copy()
            if regulator_due:
                for name in ('voltage', 'requested_voltage'):
                    setattr(self.drivers, name, np.array(self.instructions[now][name + '_V']))
        if now >= sensing.hall_available_time(self.next_scan / Fixed.pose_feedback_rate, len(self.fields.points)):
            self.moving_hall_frame(self.next_scan / Fixed.pose_feedback_rate)
            self.next_scan += 1
            self.delivered_scans += 1
        self.observe_physics(self.plant.truth())
        if np.max(np.abs(self.drivers.currents)) > self.drivers.current_limit:
            self.stop_reason = 'current_limit_exceeded'
        self.finished = bool(self.stop_reason or complete)
        if regulator_due or self.finished:
            self.trace.append(self.physical_row())

    def report(self):
        report = super().report()
        trace = list(self.trace)
        if not trace or trace[-1]['time_s'] < self.clock:
            trace.append(self.physical_row())
        report.update(scope='Joint physical integration under recorded commands and voltages; no sensor feedback',
            passed=False, diagnostic_completed=self.finished and self.stop_reason is None,
            instruction_identity=self.instruction_identity, trace=trace)
        report['limitations'] = ['Recomputes physical currents, filtered setpoints, back-EMF, motion and passive Hall signals under identical recorded drive instructions.',
            'The observer and current regulator do not receive the new sensor readings; this is a physical refinement diagnostic, not a controlled flight.',
            'Only the recorded movement prefix and its magnetic domain are tested.']
        return report

    def checkpoint_state(self):
        state = super().checkpoint_state()
        state.update(schema='airborne-joint-shared-input-v1', instruction_identity=self.instruction_identity)
        return state


def run_case(role, evidence, initial_path, atlas_path, instructions, reference, target, output):
    step = None if role == 'adaptive' else Fixed.adaptive_reference_step
    if role == 'finer':
        step /= Fixed.qualification_refinement_factor
    simulation = SharedInputSimulation(airborne_verification.recorded_construction(evidence),
        MagneticFieldAtlas.load(atlas_path), evidence['atlas']['mode'], evidence['intended_duration_s'], initialize=False)
    simulation.configure(initial_path, instructions, target, step)
    comparisons = []
    while not simulation.finished and simulation.clock < target - Fixed.adaptive_clock_tolerance:
        simulation.advance()
        if simulation.checkpoint_due() or simulation.finished:
            report = simulation.report()
            if simulation.stop_reason is None:
                prefix = dict(reference, trace=[row for row in reference['trace'] if row['time_s'] <= simulation.clock])
                result = physical_comparison(prefix, report)
                if role == 'repeat':
                    result['checks']['identical_physical_trace'] = all(left[key] == right[key]
                        for left, right in zip(prefix['trace'], report['trace']) for key in ('pose', 'currents_A'))
                    result['converged'] = all(result['checks'].values())
                comparisons.append(dict(time_s=simulation.clock, **result))
                if not result['converged']:
                    simulation.stop_reason = 'shared_input_comparison_failed'
                    simulation.finished = True
            hover.save_trial(simulation, output, role)
    report = simulation.report()
    return dict(status='completed' if report['diagnostic_completed'] else 'failed',
        duration_completed_s=simulation.clock, elapsed_s=report['elapsed_s'], stop_reason=simulation.stop_reason,
        comparisons=comparisons, adaptation={name: value for name, value in simulation.adaptation.items() if name != 'intervals'})


def compare_sensor_signals(simulation, reference_checkpoint, candidate_checkpoint, duration):
    sensors = [deepcopy(simulation.sensors), deepcopy(simulation.sensors)]
    rows = []
    index = 0
    while sensing.hall_available_time(index / Fixed.pose_feedback_rate, len(simulation.fields.points)) <= duration:
        start = index / Fixed.pose_feedback_rate
        signals, packets = [], []
        for sensor, checkpoint in zip(sensors, (reference_checkpoint, candidate_checkpoint)):
            signal = sensor.moving_signal(np.array(checkpoint['field_times']), np.array(checkpoint['field_values']), start,
                np.array(checkpoint['coil_field_values']), simulation.drivers.inductance / simulation.drivers.resistance)
            signals.append(signal)
            packets.append(sensor.encode_signal(signal, start))
        rows.append(dict(start_time_s=start, maximum_analogue_difference_T=float(np.max(np.abs(signals[1] - signals[0]))),
            changed_adc_samples=int(np.count_nonzero(packets[1].adc_codes - packets[0].adc_codes))))
        index += 1
    return rows


def run(diagnosis_path, initial_path, atlas_path, output, full_reference=False):
    if output.exists():
        raise ValueError('Shared-input output already exists; preserve its evidence')
    diagnosis = json.loads(diagnosis_path.read_text())
    case = diagnosis['cases'][1]
    report_path = Path(case['report_path'])
    checkpoint_path = report_path.with_name(report_path.stem + '_checkpoint.json')
    if (hashlib.sha256(report_path.read_bytes()).hexdigest() != case['report_sha256']
            or hashlib.sha256(checkpoint_path.read_bytes()).hexdigest() != case['checkpoint_sha256']):
        raise ValueError('Shared-input diagnosis requires unchanged recorded reference inputs')
    evidence = json.loads(report_path.read_text())
    reference = hover.simulation_from_evidence(evidence, atlas_path, False, evidence['fixed_reference_step_s'])
    reference.load_checkpoint(checkpoint_path)
    reference_report = hover.comparison_snapshot(reference)
    checkpoint, initial = json.loads(checkpoint_path.read_text()), json.loads(initial_path.read_text())
    if reference.fixed_step != Fixed.adaptive_reference_step or checkpoint['adaptive']['adaptation']['rejected']:
        raise ValueError('Instruction reconstruction requires the fixed accepted reference intervals')
    instructions = reconstruct_instructions(reference.drivers, initial, checkpoint, case['commands'])
    reference.fixed_step = None
    reference.load_checkpoint(initial_path)
    output.mkdir(parents=True)
    summary = dict(scope='Shared-voltage continuous integration diagnosis, not feedback or flight qualification', status='running',
        source_sha256=source_fingerprints(__file__),
        inputs_sha256=source_fingerprints(diagnosis_path, initial_path, report_path, checkpoint_path),
        atlas_identity=reference.atlas_configuration['identity'], exact_final_controller_reconstruction=True,
        reference_duration_s=evidence['duration_completed_s'], full_reference_refinement=full_reference, cases={})
    airborne_verification.save_summary(output / 'summary.json', summary)
    if full_reference:
        summary['cases']['finer'] = run_case('finer', evidence, initial_path, atlas_path, instructions,
            reference_report, evidence['duration_completed_s'], output)
    else:
        context = multiprocessing.get_context('spawn')
        with ProcessPoolExecutor(max_workers=Fixed.qualification_parallel_workers, mp_context=context) as pool:
            futures = {role: pool.submit(run_case, role, evidence, initial_path, atlas_path, instructions, reference_report,
                Fixed.airborne_check_duration, output) for role in ('repeat', 'finer')}
            for role, future in futures.items():
                summary['cases'][role] = future.result()
        if all(case['status'] == 'completed' for case in summary['cases'].values()):
            summary['cases']['adaptive'] = run_case('adaptive', evidence, initial_path, atlas_path, instructions,
                reference_report, evidence['duration_completed_s'], output)
    for role, result in summary['cases'].items():
        candidate_checkpoint = json.loads((output / (role + '_checkpoint.json')).read_text())
        result['sensor_comparison'] = compare_sensor_signals(reference, checkpoint, candidate_checkpoint, result['duration_completed_s'])
    summary['status'] = 'completed' if all(case['status'] == 'completed' for case in summary['cases'].values()) else 'failed'
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Isolate continuous integration using identical recorded controller voltage instructions.')
    parser.add_argument('--diagnosis', type=Path, required=True)
    parser.add_argument('--initial', type=Path, required=True)
    parser.add_argument('--atlas', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--full-reference', action='store_true', help='Refine the complete recorded fixed reference with feedback disconnected.')
    arguments = parser.parse_args()
    print(json.dumps(run(arguments.diagnosis, arguments.initial, arguments.atlas, arguments.output, arguments.full_reference)), flush=True)

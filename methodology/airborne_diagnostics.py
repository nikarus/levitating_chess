import argparse
from copy import copy, deepcopy
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from time import perf_counter
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

import levitation_sim
from model import Inputs, Fixed
from methodology import airborne, airborne_verification, electrical, sensing, source_fingerprints
from methodology.cad import magnetics


def recorded_events(simulation, checkpoint):
    field_times, field_values = np.array(checkpoint['field_times']), np.array(checkpoint['field_values'])
    readouts = [electrical.CurrentReadings(simulation.identities, np.array(row['sample_times_s']),
                np.array(row['adc_codes']), row['available_time_s']) for row in checkpoint['readouts']]
    next_scan = 0
    adaptive = 'adaptive' in checkpoint
    event_times = ([row['start_s'] for row in checkpoint['adaptive']['adaptation']['intervals']]
                   if adaptive else np.arange(len(field_times)) * simulation.period)
    if adaptive and checkpoint['trace'] and checkpoint['trace'][-1]['time_s'] == checkpoint['adaptive']['clock']:
        event_times.append(checkpoint['adaptive']['clock'])
    readout_times = np.array([reading.sample_times_s[0] for reading in readouts])
    for step, now in enumerate(event_times):
        start = next_scan / Fixed.pose_feedback_rate
        frame, history = None, None
        available_time = (sensing.hall_available_time(start, len(simulation.fields.points))
                          if adaptive else start + simulation.scan_delay)
        if now >= available_time:
            frame = simulation.sensors.sample_moving(field_times, field_values, start,
                np.array(checkpoint['coil_field_values']), simulation.drivers.inductance / simulation.drivers.resistance)
            available = [reading for reading in readouts if reading.available_time_s <= frame.available_time_s]
            history = electrical.CurrentReadings(simulation.identities,
                np.concatenate([reading.sample_times_s for reading in available]),
                np.vstack([reading.adc_codes for reading in available]), available[-1].available_time_s)
            next_scan += 1
        readout_index = np.searchsorted(readout_times, now, side='right') - 1 if adaptive else step // simulation.regulator_tick
        yield step, now, frame, history, readouts[readout_index]
    if simulation.sensors.random.bit_generator.state != checkpoint['sensors']['random']:
        raise ValueError('Replayed Hall noise stream differs from the recorded run')


def replay(baseline, output):
    data = baseline.read_bytes()
    saved = json.loads(data)
    report, checkpoint = saved['report'], saved['checkpoint']
    if report['source_sha256'] != checkpoint['source_sha256']:
        raise ValueError('Baseline report and sensor history have different sources')
    simulation = airborne.AirborneSimulation(magnetics.ConstructionModel(), report['intended_duration_s'],
                                             disturbed=report['disturbed'])
    if any(report['source_sha256'].get(name) != digest for name, digest in simulation.source.items()):
        raise ValueError('Observer replay requires the recorded simulation sources')
    passive = sensing.HallTracker(simulation.tracker.estimator)
    for name in airborne.TRACKER_ARRAY_STATE:
        setattr(passive, name, getattr(simulation.tracker, name).copy())
    for name in airborne.TRACKER_CLOCK_STATE:
        setattr(passive, name, getattr(simulation.tracker, name))
    trackers = dict(without_current_input=passive, measured_current_input=simulation.tracker)
    trace_times = np.array([row['time_s'] for row in report['trace']])
    trace_poses = np.array([row['pose'] for row in report['trace']])
    rows = []
    peaks = {name: dict(position_m=0., orientation_rad=0.) for name in trackers}
    for step, now, packet, history, readout in recorded_events(simulation, checkpoint):
        if packet is not None:
            for tracker in trackers.values():
                result = tracker.consume(packet, history, now)
                if not result['valid']:
                    raise ValueError(f'Replay lost tracking: {result}')
        estimates = {name: tracker.predict(now) for name, tracker in trackers.items()}
        truth = np.array([np.interp(now, trace_times, trace_poses[:, axis]) for axis in range(6)])
        row = dict(time_s=now)
        for name, estimate in estimates.items():
            if not estimate['valid']:
                raise ValueError(f'Replay predictor unavailable: {estimate}')
            pose = np.array(estimate['pose'])
            errors = dict(position_m=float(np.linalg.norm(pose[:3] - truth[:3])),
                orientation_rad=float((Rotation.from_euler('xyz', pose[3:]) * Rotation.from_euler('xyz', truth[3:]).inv()).magnitude()))
            for key, value in errors.items():
                peaks[name][key] = max(peaks[name][key], value)
            row[name] = errors
        if step % simulation.command_tick == 0 or step == checkpoint['step']:
            rows.append(row)
        if step % simulation.command_tick == 0:
            levitation_sim.use_geometry(simulation.controller.geometry)
            _, center = levitation_sim.place_piece(simulation.controller.magnets, *estimates['measured_current_input']['pose'])
            simulation.controller.nominal_matrix = levitation_sim.actuator_matrix(
                simulation.controller.magnets, simulation.controller.array, center)
        if step % simulation.regulator_tick == 0:
            simulation.tracker.record_acceleration(now, simulation.controller.measured_acceleration(
                estimates['measured_current_input'], readout, now))
        if step % simulation.checkpoint_tick == 0:
            print(f'Replayed {now:.6f} s', flush=True)
    result = dict(scope='Observer comparison on identical recorded Hall/current inputs; physical motion is not rerun',
        baseline_sha256=hashlib.sha256(data).hexdigest(), baseline_source_sha256=report['source_sha256'],
        source_sha256=dict(simulation.source, **source_fingerprints(__file__)), duration_s=report['duration_completed_s'],
        noise_stream_reproduced=True, peaks=peaks, trace=rows,
        limitations=['Errors use linear interpolation of the saved pose trace for scoring only.',
                     'The recorded physical trajectory belongs to the failed controller run; improved estimation does not prove corrected flight.',
                     'Nominal acceleration holds the allocation matrix between command ticks and uses measured currents with their original errors.'])
    output.write_text(json.dumps(result, indent=Fixed.report_precision) + '\n')
    print(json.dumps(peaks), flush=True)


def captured_commands(case, simulation):
    rows, frame_index = [], 0
    period = case['report']['physics_time_step_s']
    command_tick = round(1 / Fixed.current_command_rate / period)
    regulator_tick = round(1 / Fixed.current_regulator_rate / period)
    for step in range(round(case['report']['duration_completed_s'] / period) + 1):
        now = step * period
        if frame_index < len(case['frames']) and now >= case['frames'][frame_index]['available_time_s']:
            raw, fit = case['frames'][frame_index], case['fits'][frame_index]
            frame = sensing.HallFrame(np.array(raw['adc_codes']), np.array(raw['sample_times_s']), raw['available_time_s'])
            if not np.array_equal(simulation.tracker.propagate(float(frame.sample_times_s.mean())), fit['prediction']):
                raise ValueError('Captured command replay differs from the recorded estimator prediction')
            with patch.object(simulation.tracker.estimator, 'estimate', return_value=fit['estimate']):
                result = simulation.tracker.consume(frame, {}, now)
            if not result['valid']:
                raise ValueError('Captured command replay lost tracking')
            frame_index += 1
        estimate = simulation.tracker.predict(now)
        if step % command_tick == 0:
            allocation = simulation.controller.command(estimate, simulation.reference)
            if not allocation['accepted']:
                raise ValueError('Captured command replay exceeds its current budget')
            matrix, currents = simulation.controller.nominal_matrix, allocation['currents']
            equality = np.vstack((matrix, np.ones(len(currents))))
            analytic = np.linalg.lstsq(equality, np.r_[matrix @ currents, 0], rcond=None)[0]
            if np.max(np.abs(analytic)) >= simulation.controller.current_budget:
                raise ValueError('Unbounded allocation comparison requires inactive current limits')
            rows.append(dict(time_s=now, estimate=estimate, command_A=currents.tolist(),
                optimizer_difference_A=float(np.max(np.abs(currents - analytic)))))
        if step % regulator_tick == 0:
            index = step // regulator_tick
            readout = electrical.CurrentReadings(simulation.identities,
                np.array([case['current_sample_times_s'][index]]), np.array(case['current_adc_codes'][index])[None, :], now)
            simulation.tracker.record_acceleration(now, simulation.controller.measured_acceleration(estimate, readout, now))
    if not np.array_equal([row['estimate']['pose'] for row in rows], [row['estimated_pose'] for row in case['report']['trace']]):
        raise ValueError('Captured command replay differs from the saved estimated trajectory')
    return rows


def count_response_diagnosis(capture_path, baseline, checkpoint_path, output):
    if output.exists():
        raise ValueError('Count-response output already exists; preserve its evidence')
    capture_bytes, report_bytes, checkpoint_bytes = (path.read_bytes() for path in (capture_path, baseline, checkpoint_path))
    captured, report, checkpoint = (json.loads(value) for value in (capture_bytes, report_bytes, checkpoint_bytes))
    coarse = captured['cases'][0]
    if (report['source_sha256'] != checkpoint['source_sha256'] or report['trace'] != checkpoint['trace']
            or any(report['source_sha256'].get(name) != digest for name, digest in coarse['report']['source_sha256'].items())):
        raise ValueError('Count-response inputs must describe the same source revision and matching report/checkpoint')
    construction = airborne_verification.recorded_construction(coarse['report'])
    short_commands = []
    for case in captured['cases']:
        scenario = case['report']
        simulation = airborne.AirborneSimulation(construction, scenario['intended_duration_s'],
            disturbed=scenario['disturbed'], time_refinement=scenario['time_refinement'], magnetic_refinement=scenario['magnetic_refinement'])
        short_commands.append(captured_commands(case, simulation))
    time_response = []
    for left, right in zip(*short_commands):
        if left['time_s'] != right['time_s']:
            raise ValueError('Captured command comparisons must share observation times')
        time_response.append(dict(time_s=left['time_s'],
            pose_difference=(np.array(right['estimate']['pose']) - left['estimate']['pose']).tolist(),
            velocity_difference=(np.array(right['estimate']['velocity']) - left['estimate']['velocity']).tolist(),
            command_difference_A=float(np.max(np.abs(np.array(right['command_A']) - left['command_A'])))))
    simulation = airborne.AirborneSimulation(airborne_verification.recorded_construction(report), report['intended_duration_s'],
        disturbed=report['disturbed'], time_refinement=report['time_refinement'], magnetic_refinement=report['magnetic_refinement'])
    simulation.tracker.acceleration_times = checkpoint['acceleration_times']
    simulation.tracker.accelerations = [np.array(value) for value in checkpoint['accelerations']]
    trackers = dict(recorded=simulation.tracker, first_count_reverted=deepcopy(simulation.tracker))
    rows, changed_samples, frame_index = [], [], 0
    for step, now, frame, history, _ in recorded_events(simulation, checkpoint):
        if frame is not None:
            altered = frame
            if frame_index == 0:
                codes = np.array(coarse['frames'][0]['adc_codes'])
                if not np.array_equal(frame.sample_times_s, coarse['frames'][0]['sample_times_s']):
                    raise ValueError('Count-response packets have different sample times')
                difference = codes - frame.adc_codes
                if np.count_nonzero(difference) != 1 or np.max(np.abs(difference)) != 1:
                    raise ValueError('Count-response probe requires exactly one changed ADC count')
                changed_samples = np.argwhere(difference).tolist()
                altered = sensing.HallFrame(codes, frame.sample_times_s, frame.available_time_s)
            for name, tracker in trackers.items():
                result = tracker.consume(frame if name == 'recorded' else altered, history, now)
                if not result['valid']:
                    raise ValueError('Count-response replay lost tracking')
            frame_index += 1
        if step % simulation.command_tick == 0:
            row = dict(time_s=now)
            for name, tracker in trackers.items():
                estimate = tracker.predict(now)
                allocation = simulation.controller.command(estimate, simulation.reference)
                if not allocation['accepted']:
                    raise ValueError('Count-response replay exceeds its current budget')
                row[name] = dict(estimate=estimate, command_A=allocation['currents'].tolist())
            row['command_difference_A'] = float(np.max(np.abs(np.array(row['recorded']['command_A'])
                                                               - row['first_count_reverted']['command_A'])))
            rows.append(row)
        if step % simulation.checkpoint_tick == 0:
            print(f'Count-response replay {now:.6f} s', flush=True)
    if not np.array_equal([row['recorded']['estimate']['pose'] for row in rows], [row['estimated_pose'] for row in report['trace']]):
        raise ValueError('Count-response baseline does not reproduce the saved estimated trajectory exactly')
    reconstruction = []
    fields = simulation.tracker.estimator.field_model
    indices = [fields.identities.index(identity) for identity in simulation.identities]
    for raw in coarse['frames']:
        frame = sensing.HallFrame(np.array(raw['adc_codes']), np.array(raw['sample_times_s']), raw['available_time_s'])
        times = np.array(coarse['current_sample_times_s'])
        available = times <= frame.available_time_s
        history = electrical.CurrentReadings(simulation.identities, times[available],
            np.array(coarse['current_adc_codes'])[available], times[available][-1])
        linear, _, _ = sensing.measured_coil_fields(fields, history, frame)
        exponential = sensing.filtered_scan(history.sample_times_s, history.decode(frame.available_time_s) @ fields.coil_field[:, indices].T,
            raw['start_time_s'], coarse['electrical_time_constant_s']).mean(axis=1)
        physical = sensing.filtered_scan(np.array(coarse['field_times_s']), np.array(coarse['coil_fields_T']) * coarse['hall_gain'],
            raw['start_time_s'], coarse['electrical_time_constant_s']).mean(axis=1)
        reconstruction.append(dict(start_time_s=raw['start_time_s'],
            linear_maximum_error_T=float(np.max(np.abs(linear - physical))),
            exponential_maximum_error_T=float(np.max(np.abs(exponential - physical)))))
    peak = max(rows, key=lambda row: row['command_difference_A'])
    result = dict(scope='Recorded-data isolation of the first Hall ADC count change; physical motion is not rerun',
        capture_sha256=hashlib.sha256(capture_bytes).hexdigest(), baseline_sha256=hashlib.sha256(report_bytes).hexdigest(),
        checkpoint_sha256=hashlib.sha256(checkpoint_bytes).hexdigest(), baseline_source_sha256=report['source_sha256'],
        source_sha256=dict(simulation.source, **source_fingerprints(__file__)),
        changed_source_files=[name for name, digest in simulation.source.items() if report['source_sha256'].get(name) != digest],
        exact_captured_prediction_and_pose_replay=True, exact_recorded_pose_replay=True, exact_recorded_noise_replay=True,
        time_refinement_response=time_response,
        maximum_optimizer_difference_A=max(row['optimizer_difference_A'] for commands in short_commands for row in commands),
        isolated_count=dict(changed_sample_indices=changed_samples, peak_command_difference_A=peak['command_difference_A'],
            peak_time_s=peak['time_s'], final_command_difference_A=rows[-1]['command_difference_A'], trace=rows),
        coil_reconstruction=reconstruction,
        limitations=['Current commands are counterfactual outputs, not new physical coil currents.',
            'Both isolated-count trackers receive the same later Hall packets, measured currents and recorded nominal acceleration history.',
            'A decaying isolated response does not establish stability of the coupled physical feedback loop or explain its entire later divergence.',
            'Current source differences are explicit; exact recorded pose/noise reproduction is required before writing evidence. No changed-source checkpoint is resumed.',
            'The reconstruction comparison retains calibration and current-sensing errors; its physical field is used for diagnostic scoring only.',
            'No physical noise, hardware rate or acceptance limit is changed.'])
    output.write_text(json.dumps(result, indent=Fixed.report_precision, allow_nan=False) + '\n')
    return result


def single_count_difference(original, changed):
    difference = np.asarray(changed) - np.asarray(original)
    if np.count_nonzero(difference) != 1 or np.max(np.abs(difference)) != 1:
        raise ValueError('Count-feedback probe requires exactly one observed ADC count change')
    return difference


def changed_count_frame(frame, original, difference):
    if not np.array_equal(frame.adc_codes, original):
        raise ValueError('Count-feedback baseline does not reproduce the recorded packet')
    single_count_difference(original, np.asarray(original) + difference)
    return sensing.HallFrame(frame.adc_codes + difference, frame.sample_times_s, frame.available_time_s)


def recorded_count_feedback(diagnosis_path, initial_path, atlas_path, output):
    from methodology import hover
    if output.exists():
        raise ValueError('Count-feedback output already exists; preserve its evidence')
    diagnosis = json.loads(diagnosis_path.read_text())
    original_case, refined_case = diagnosis['cases']
    packet_index = next(index for index, (left, right) in enumerate(zip(original_case['packets'], refined_case['packets']))
        if left['adc_codes'] != right['adc_codes'])
    original_packet, refined_packet = [case['packets'][packet_index] for case in diagnosis['cases']]
    difference = single_count_difference(original_packet['adc_codes'], refined_packet['adc_codes'])
    for case in diagnosis['cases']:
        path = Path(case['report_path'])
        checkpoint = path.with_name(path.stem + '_checkpoint.json')
        if (hashlib.sha256(path.read_bytes()).hexdigest() != case['report_sha256']
                or hashlib.sha256(checkpoint.read_bytes()).hexdigest() != case['checkpoint_sha256']):
            raise ValueError('Count-feedback input evidence has changed')
    reference_path = Path(original_case['report_path'])
    reference_report = json.loads(reference_path.read_text())
    reference_checkpoint = reference_path.with_name(reference_path.stem + '_checkpoint.json')
    reference_state = json.loads(reference_checkpoint.read_text())
    reference = hover.simulation_from_evidence(reference_report, atlas_path, False, reference_report['fixed_reference_step_s'])
    reference.load_checkpoint(reference_checkpoint)
    if reference.trace != reference_report['trace'] or reference.peaks != reference_report['peaks'] or reference.stop_reason != 'peer_trial_failed':
        raise ValueError('Retained reference must match its checkpoint and have stopped only with its peer')
    reference_snapshot = hover.comparison_snapshot(reference)
    reference_snapshot['stop_reason'] = None
    simulation = hover.simulation_from_evidence(reference_report, atlas_path, False)
    simulation.load_checkpoint(initial_path)
    if simulation.clock != 0 or simulation.finished or simulation.source != reference.source:
        raise ValueError('Count-feedback requires the matching original release state')
    simulation.fixed_step = reference.fixed_step
    output.mkdir(parents=True)
    summary = dict(scope='One recorded Hall count changed under identical fixed integration; not numerical or flight qualification',
        source_sha256=source_fingerprints(__file__), simulation_source_sha256=simulation.source.copy(),
        inputs_sha256=source_fingerprints(diagnosis_path, initial_path, reference_path, reference_checkpoint),
        atlas_identity=reference.atlas_configuration['identity'], fixed_step_s=simulation.fixed_step,
        target_duration_s=reference.clock, packet_start_s=original_packet['start_time_s'],
        packet_delivery_s=original_packet['delivery_time_s'], changed_adc_samples=np.argwhere(difference).tolist(),
        adc_count_change=int(difference[np.nonzero(difference)][0]), injected_packets=0,
        reference_reused=True, control_qualification_passed=False, status='running', comparisons=[],
        limitations=['Later sensor readings are generated by the changed coupled trajectory; only one packet is deliberately altered.',
            'Existing comparison and operating limits stop the diagnostic; a stopped interval cannot establish later decay or stability.'])
    summary_path = output / 'summary.json'
    airborne_verification.save_summary(summary_path, summary)
    encode = simulation.sensors.encode_signal
    packets = []

    def record_signal(signal, start):
        if start == original_packet['start_time_s']:
            simulation.save_checkpoint(output / 'shared_start.json')
        frame = encode(signal, start)
        if len(packets) <= packet_index:
            expected = original_case['packets'][len(packets)]
            if start != expected['start_time_s'] or not np.array_equal(frame.adc_codes, expected['adc_codes']):
                raise ValueError('Count-feedback prefix does not reproduce the original Hall packets')
        if start == original_packet['start_time_s']:
            if summary['injected_packets']:
                raise ValueError('Count-feedback intervention repeated')
            state = simulation.checkpoint_state()
            for key in ('field_times', 'field_values', 'coil_field_values', 'readouts', 'trace', 'acceleration_times', 'accelerations'):
                if state[key] != reference_state[key][:len(state[key])]:
                    raise ValueError(f'Count-feedback prefix differs in {key}')
            intervals = state['adaptive']['adaptation']['intervals']
            if intervals != reference_state['adaptive']['adaptation']['intervals'][:len(intervals)]:
                raise ValueError('Count-feedback prefix integration steps differ')
            frame = changed_count_frame(frame, original_packet['adc_codes'], difference)
            summary.update(injected_packets=1, exact_prefix_reproduction=True)
            simulation.checkpoint_schema = 'airborne-count-intervention-v1'
            simulation.source.update(source_fingerprints(__file__))
        packets.append(dict(start_time_s=start, delivery_time_s=simulation.clock, adc_codes=frame.adc_codes.tolist()))
        return frame

    with patch.object(simulation.sensors, 'encode_signal', record_signal):
        while not simulation.finished and simulation.clock < reference.clock - Fixed.adaptive_clock_tolerance:
            simulation.advance()
            if simulation.checkpoint_due() or simulation.finished:
                snapshot = hover.comparison_snapshot(simulation)
                hover.check_operating_limits(simulation, snapshot)
                if simulation.stop_reason is None:
                    prefix = dict(reference_snapshot, trace=[row for row in reference_snapshot['trace'] if row['time_s'] <= simulation.clock])
                    comparison = airborne_verification.compare_trials(prefix, snapshot, trace_only=True)
                    if abs(simulation.clock - reference.clock) <= Fixed.adaptive_clock_tolerance:
                        comparison = hover.compare_prefixes(snapshot, reference_snapshot)
                    summary['comparisons'].append(dict(time_s=simulation.clock, **comparison))
                    if not comparison['converged']:
                        simulation.stop_reason = 'count_response_comparison_failed'
                        simulation.finished = True
                hover.save_trial(simulation, output, 'changed_count')
                summary.update(duration_completed_s=simulation.clock, stop_reason=simulation.stop_reason, packets=packets)
                airborne_verification.save_summary(summary_path, summary)
                print(f'Count response {simulation.clock:.6f} s: {simulation.stop_reason}', flush=True)
    if summary['injected_packets'] != 1:
        raise ValueError('Recorded count intervention was not reached exactly once')
    summary.update(status='completed' if simulation.stop_reason is None else 'failed', elapsed_s=simulation.report()['elapsed_s'])
    airborne_verification.save_summary(summary_path, summary)
    return summary


def count_feedback_diagnosis(capture_path, comparison_path, output):
    if output.exists():
        raise ValueError('Count-feedback output already exists; preserve its evidence')
    capture_bytes, comparison_bytes = capture_path.read_bytes(), comparison_path.read_bytes()
    capture, comparison = json.loads(capture_bytes), json.loads(comparison_bytes)
    if comparison['baseline_capture_sha256'] != hashlib.sha256(capture_bytes).hexdigest():
        raise ValueError('Count-feedback inputs do not describe the same early capture')
    coarse = capture['cases'][0]
    original_codes = np.array(coarse['frames'][0]['adc_codes'])
    changed_codes = np.array(comparison['comparisons'][0]['refined_adc_codes'])
    difference = single_count_difference(original_codes, changed_codes)
    simulation = airborne.AirborneSimulation(airborne_verification.recorded_construction(coarse['report']), Fixed.airborne_refinement_screen_duration,
        disturbed=coarse['report']['disturbed'], time_refinement=coarse['report']['time_refinement'],
        magnetic_refinement=coarse['report']['magnetic_refinement'])
    simulation.source.update(source_fingerprints(__file__))
    output.mkdir(parents=True)
    summary = dict(scope='Single-count sensitivity with identical numerical resolution; a coupled diagnostic, not a refinement qualification',
        capture_sha256=hashlib.sha256(capture_bytes).hexdigest(), comparison_sha256=hashlib.sha256(comparison_bytes).hexdigest(),
        source_sha256=simulation.source, status='running', cases={}, changed_adc_samples=np.argwhere(difference).tolist(),
        adc_count_change=int(difference[np.nonzero(difference)][0]),
        limitations=['Only the first moving Hall packet is deliberately changed; later readings arise from each trial own coupled motion and currents.',
            'The configured short screen is completed to measure the response after the expected initial comparison-limit breach.',
            'Operating limits still stop a failed trial. No noise, hardware rates, physical geometry or numerical resolution is changed.',
            'This isolates sensitivity to one count; it does not qualify hover, the flight envelope or numerical convergence.'])
    summary_path = output / 'summary.json'
    airborne_verification.save_summary(summary_path, summary)
    encode = simulation.sensors.encode_signal
    states = []
    with TemporaryDirectory(prefix='flying-chess-count-') as temporary:
        initial = Path(temporary) / 'initial.json'
        simulation.save_checkpoint(initial)
        for name in ('baseline', 'changed_count'):
            simulation.load_checkpoint(initial)
            simulation.started = perf_counter()
            packets = []

            def record_signal(signal, start):
                frame = encode(signal, start)
                if not packets:
                    if not np.array_equal(frame.adc_codes, original_codes):
                        raise ValueError('Count-feedback baseline does not reproduce the captured first packet')
                    if name == 'changed_count':
                        frame = changed_count_frame(frame, original_codes, difference)
                packets.append(dict(start_time_s=start, adc_codes=frame.adc_codes.tolist()))
                return frame

            summary['active_case'] = name
            airborne_verification.save_summary(summary_path, summary)
            with patch.object(simulation.sensors, 'encode_signal', record_signal):
                report = airborne.run_simulation(simulation, output / (name + '.json'), output / (name + '_checkpoint.json'))
            summary['cases'][name] = dict(passed=report['passed'], stop_reason=report['stop_reason'],
                duration_s=report['duration_completed_s'], peaks=report['peaks'], packets=packets)
            states.append((deepcopy(simulation.sensors.random.bit_generator.state), deepcopy(simulation.current_sensors.random.bit_generator.state)))
            airborne_verification.save_summary(summary_path, summary)
            if not report['passed']:
                summary['status'] = 'operating_limit_failed'
                airborne_verification.save_summary(summary_path, summary)
                return summary
    if states[0] != states[1]:
        raise ValueError('Count-feedback trials consumed different noise streams')
    reports = [json.loads((output / (name + '.json')).read_text()) for name in ('baseline', 'changed_count')]
    summary.update(status='completed', active_case=None, identical_noise_streams=True,
        comparison=airborne_verification.compare_trials(*reports))
    airborne_verification.save_summary(summary_path, summary)
    summary['field_attribution'] = count_feedback_fields(capture_path, output)
    return summary


def count_feedback_fields(capture_path, output):
    capture_bytes = capture_path.read_bytes()
    capture = json.loads(capture_bytes)
    summary_path = output / 'summary.json'
    summary = json.loads(summary_path.read_bytes())
    if summary['status'] != 'completed' or summary['capture_sha256'] != hashlib.sha256(capture_bytes).hexdigest():
        raise ValueError('Field attribution requires the completed matching count-feedback diagnostic')
    if 'field_attribution' in summary:
        raise ValueError('Field attribution already exists; preserve its evidence')
    checkpoints = {name: json.loads((output / (name + '_checkpoint.json')).read_bytes()) for name in summary['cases']}
    baseline, changed = checkpoints['baseline'], checkpoints['changed_count']
    for name, state in checkpoints.items():
        report = json.loads((output / (name + '.json')).read_bytes())
        if report['trace'] != state['trace'] or report['source_sha256'] != state['source_sha256']:
            raise ValueError('Field attribution report and checkpoint differ')
    if (baseline['field_times'] != changed['field_times'] or baseline['sensors']['gain'] != changed['sensors']['gain']
            or baseline['sensors']['offset'] != changed['sensors']['offset']):
        raise ValueError('Field attribution requires identical clocks and Hall calibration')
    construction = airborne_verification.recorded_construction(capture['cases'][0]['report'])
    _, array, _ = construction(Inputs.cad_coil_layout, baseline['magnetic_refinement'])
    identities = tuple(array.identities)
    fields = sensing.HallFieldModel(construction)
    sensors = sensing.VirtualHallArray(fields)
    sensors.gain, sensors.offset = np.array(baseline['sensors']['gain']), np.array(baseline['sensors']['offset'])
    sensors.random.bit_generator.state = capture['cases'][0]['frames'][0]['noise_state_before']
    times = np.array(baseline['field_times'])
    coils = {name: np.array(state['coil_field_values']) for name, state in checkpoints.items()}
    magnets = {name: np.array(state['field_values']) - coils[name] for name, state in checkpoints.items()}
    time_constant = capture['cases'][0]['electrical_time_constant_s']
    rows = []
    for index, original in enumerate(summary['cases']['baseline']['packets']):
        start = original['start_time_s']
        noise_state = deepcopy(sensors.random.bit_generator.state)
        signals, packets = {}, {}
        for label, magnet_source, coil_source in (
                ('baseline', 'baseline', 'baseline'), ('motion_only', 'changed_count', 'baseline'),
                ('coils_only', 'baseline', 'changed_count'), ('both', 'changed_count', 'changed_count')):
            signals[label] = sensors.moving_signal(times, magnets[magnet_source] + coils[coil_source], start, coils[coil_source], time_constant)
            sensors.random.bit_generator.state = deepcopy(noise_state)
            packets[label] = sensors.encode_signal(signals[label], start)
        if not np.array_equal(packets['baseline'].adc_codes, original['adc_codes']):
            raise ValueError('Field attribution does not reproduce the baseline ADC packet')
        expected = summary['cases']['changed_count']['packets'][index]
        if start != expected['start_time_s']:
            raise ValueError('Field attribution packet clocks differ')
        if index == 0:
            if not np.array_equal(packets['both'].adc_codes, original['adc_codes']):
                raise ValueError('Physical histories differ before the deliberate count change')
            packets['both'] = sensing.HallFrame(np.array(expected['adc_codes']), packets['both'].sample_times_s, packets['both'].available_time_s)
        elif not np.array_equal(packets['both'].adc_codes, expected['adc_codes']):
            raise ValueError('Field attribution does not reproduce the changed-count ADC packet')
        corrections = {}
        for name, state in checkpoints.items():
            frame = packets['baseline' if name == 'baseline' else 'both']
            available = [reading for reading in state['readouts'] if reading['available_time_s'] <= frame.available_time_s]
            history = electrical.CurrentReadings(identities, np.concatenate([reading['sample_times_s'] for reading in available]),
                np.vstack([reading['adc_codes'] for reading in available]), available[-1]['available_time_s'])
            corrections[name] = sensing.measured_coil_fields(fields, history, frame)[0]
        motion = (signals['motion_only'] - signals['baseline']).mean(axis=1)
        coil_field = (signals['coils_only'] - signals['baseline']).mean(axis=1)
        correction = corrections['changed_count'] - corrections['baseline']
        decoded = (packets['both'].adc_codes - packets['baseline'].adc_codes).mean(axis=1) * sensing.hall_readout()[2]
        quantization = decoded - (signals['both'] - signals['baseline']).mean(axis=1)
        rows.append(dict(start_time_s=start,
            changed_adc_samples={name: int(np.count_nonzero(frame.adc_codes - packets['baseline'].adc_codes))
                                 for name, frame in packets.items() if name != 'baseline'},
            mean_channel_motion_difference_T=motion.tolist(), mean_channel_coil_field_difference_T=coil_field.tolist(),
            mean_channel_uncancelled_coil_difference_T=(coil_field - correction).tolist(),
            mean_channel_quantization_difference_T=quantization.tolist(),
            mean_channel_estimator_input_difference_T=(decoded - correction).tolist()))
    if sensors.random.bit_generator.state != baseline['sensors']['random'] or sensors.random.bit_generator.state != changed['sensors']['random']:
        raise ValueError('Field attribution does not reproduce the recorded Hall noise stream')
    summary['field_attribution'] = dict(source_sha256=source_fingerprints(__file__, sensing.__file__, electrical.__file__),
        exact_adc_and_noise_replay=True, trace=rows,
        limitations=['Motion-only and coils-only signals are counterfactual combinations of saved fields, not new physical trajectories.',
            'The first packet includes the deliberate single-count intervention; subsequent quantization differences arise naturally.',
            'Identical added analog noise cancels in the field difference; ADC rounding still depends on that same noise realization.',
            'The uncancelled coil contribution includes current sensing, interpolation and calibration mismatch; it is not attributed to one of those sources here.'])
    airborne_verification.save_summary(summary_path, summary)
    return summary['field_attribution']


def recorded_simulation(baseline, checkpoint_path, parameter_comparison=False, atlas_path=None):
    report_bytes, checkpoint_bytes = baseline.read_bytes(), checkpoint_path.read_bytes()
    report, checkpoint = json.loads(report_bytes), json.loads(checkpoint_bytes)
    if checkpoint['schema'] != 'airborne-coupled-v4' and atlas_path is None:
        raise ValueError('Recorded fixed-step diagnostics require the fixed-step checkpoint schema; adaptive replay needs its recorded event clock')
    if report['source_sha256'] != checkpoint['source_sha256'] or report['trace'] != checkpoint['trace']:
        raise ValueError('Report and recorded sensor history do not describe the same flight')
    construction = airborne_verification.recorded_construction(report)
    if atlas_path is None:
        simulation = airborne.AirborneSimulation(construction, report['intended_duration_s'],
            disturbed=report['disturbed'], time_refinement=report['time_refinement'],
            magnetic_refinement=report['magnetic_refinement'])
    else:
        from methodology.magnetic_atlas import AtlasAirborneSimulation, MagneticFieldAtlas
        simulation = AtlasAirborneSimulation(construction, MagneticFieldAtlas.load(atlas_path),
            report['atlas']['mode'], report['intended_duration_s'], disturbed=report['disturbed'],
            fixed_step=report['fixed_reference_step_s'], tolerance_scale=report['tolerance_scale'],
            magnetic_refinement=report['magnetic_refinement'])
        if checkpoint['schema'] != simulation.checkpoint_schema or checkpoint['atlas'] != simulation.atlas_configuration:
            raise ValueError('Recorded replay schema or magnetic atlas differs')
        root = Path(__file__).resolve().parent.parent
        if any(hashlib.sha256((root / name).read_bytes()).hexdigest() != digest
               for name, digest in report['source_sha256'].items()):
            raise ValueError('Adaptive replay requires unchanged recorded sources')
    differences = [name for name, digest in simulation.source.items() if report['source_sha256'].get(name) != digest]
    if differences and (not parameter_comparison or set(differences) != {'model.py'}):
        raise ValueError(f'Feedback replay simulation sources differ: {differences}')
    simulation.tracker.acceleration_times = checkpoint['acceleration_times']
    simulation.tracker.accelerations = [np.array(value) for value in checkpoint['accelerations']]
    return report_bytes, checkpoint_bytes, report, checkpoint, simulation


def coil_sampling_attribution(cases):
    sampled = []
    for case in cases:
        path = Path(case['report_path'])
        report_bytes = path.read_bytes()
        checkpoint_bytes = path.with_name(path.stem + '_checkpoint.json').read_bytes()
        if (hashlib.sha256(report_bytes).hexdigest() != case['report_sha256']
                or hashlib.sha256(checkpoint_bytes).hexdigest() != case['checkpoint_sha256']):
            raise ValueError('Coil-sampling attribution requires unchanged recorded inputs')
        checkpoint = json.loads(checkpoint_bytes)
        drivers = electrical.CoilDrivers(airborne_verification.recorded_construction(json.loads(report_bytes)), ())
        times = np.array(checkpoint['field_times'])
        current_times = np.array(case['current_sample_times_s'])
        indices = np.searchsorted(times, current_times)
        if not np.array_equal(times[indices], current_times):
            raise ValueError('Current samples must coincide with recorded physical field samples')
        fields = np.array(checkpoint['coil_field_values'])[indices] * checkpoint['sensors']['gain']
        sampled.append([dict(start_time_s=packet['start_time_s'], **{
            name: sensing.filtered_scan(current_times[current_times <= packet['available_time_s']],
                fields[current_times <= packet['available_time_s']], packet['start_time_s'], time_constant).mean(axis=1)
            for name, time_constant in (('linear_T', None), ('exponential_T', drivers.inductance / drivers.resistance))})
            for packet in case['packets']])
    rows = []
    for left, right, left_sample, right_sample in zip(cases[0]['packets'], cases[1]['packets'], *sampled):
        actual = np.array(right['mean_coil_field_T']) - left['mean_coil_field_T']
        correction = np.array(right['subtracted_coil_field_T']) - left['subtracted_coil_field_T']
        linear = right_sample['linear_T'] - left_sample['linear_T']
        exponential = right_sample['exponential_T'] - left_sample['exponential_T']
        rows.append(dict(start_time_s=left['start_time_s'],
            differential_linear_sampling_error_T=(actual - linear).tolist(),
            differential_exponential_sampling_error_T=(actual - exponential).tolist(),
            differential_readout_and_map_error_T=(linear - correction).tolist(),
            differential_total_coil_error_T=(actual - correction).tolist()))
    return dict(source_sha256=source_fingerprints(__file__, sensing.__file__, electrical.__file__), trace=rows,
        limitations=['Exact physical coil fields sampled at current-readout times isolate interpolation without current-sensor or map error.',
            'Readout/map residual combines current quantization, noise, gain/offset and coil/Hall calibration; these are not individually isolated.',
            'Exponential reconstruction is an ideal-field counterfactual; a controller must use measured currents and a qualified electrical time constant.'])


def coil_map_refinement(cases):
    report_bytes = Path(cases[0]['report_path']).read_bytes()
    if hashlib.sha256(report_bytes).hexdigest() != cases[0]['report_sha256']:
        raise ValueError('Coil-map refinement requires the unchanged baseline report')
    report = json.loads(report_bytes)
    construction = airborne_verification.recorded_construction(report)
    refinements = [case['magnetic_refinement'] for case in cases]
    if refinements[1] <= refinements[0]:
        raise ValueError('Coil-map refinement requires increasing magnetic resolutions')
    refinements.append(refinements[-1] * Fixed.qualification_refinement_factor)
    _, array, _ = construction(Inputs.cad_coil_layout, refinements[0])
    currents = np.array([row['currents_A'] for row in report['trace']])
    if currents.shape[1] != len(array.identities):
        raise ValueError('Recorded currents do not match the reconstructed active coil window')
    matrices = []
    for refinement in refinements:
        fields = sensing.HallFieldModel(construction, magnetic_refinement=Fixed.hall_conductor_refinement * refinement)
        selection = [fields.identities.index(identity) for identity in array.identities]
        matrices.append(fields.coil_field[:, selection])
    comparisons = [dict(coarse=left, refined=right,
        relative_matrix_difference=float(np.linalg.norm(second - first) / np.linalg.norm(second)),
        maximum_field_difference_on_recorded_currents_T=float(np.max(np.abs(currents @ (second - first).T))))
        for left, right, first, second in zip(refinements[:-1], refinements[1:], matrices[:-1], matrices[1:])]
    return dict(source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, sensing.__file__)),
        scope='Coil-to-Hall map refinement at identical recorded baseline currents; no feedback or physical rerun',
        comparisons=comparisons,
        limitations=['Uncalibrated nominal field maps are compared at fixed sensor locations.',
            'This checks coil-field discretization only; force convergence and coupled behavior still require independent verification.'])


def recorded_resolution_diagnosis(baseline, comparison, output, atlas_path=None):
    if output.exists():
        raise ValueError('Recorded-resolution output already exists; preserve its evidence')
    cases = []
    for path in (baseline, comparison):
        checkpoint_path = path.with_name(path.stem + '_checkpoint.json')
        report_bytes, checkpoint_bytes, report, checkpoint, simulation = recorded_simulation(path, checkpoint_path, atlas_path=atlas_path)
        times = np.array(checkpoint['field_times'])
        coils = np.array(checkpoint['coil_field_values'])
        fields = np.array(checkpoint['field_values'])
        packets, commands = [], []
        initial_noise_state = deepcopy(simulation.sensors.random.bit_generator.state)
        command_times = {row['time_s'] for row in report['trace']}
        for step, now, frame, history, _ in recorded_events(simulation, checkpoint):
            if frame is not None:
                start = len(packets) / Fixed.pose_feedback_rate
                signal = simulation.sensors.moving_signal(times, fields, start, coils,
                    simulation.drivers.inductance / simulation.drivers.resistance)
                coil_signal = sensing.filtered_scan(times, coils * simulation.sensors.gain, start,
                    simulation.drivers.inductance / simulation.drivers.resistance)
                correction = sensing.measured_coil_fields(simulation.tracker.estimator.field_model, history, frame)[0]
                prediction = simulation.tracker.propagate(float(frame.sample_times_s.mean()))
                result = simulation.tracker.consume(frame, history, now)
                if not result['valid']:
                    raise ValueError('Recorded-resolution replay lost tracking')
                packets.append(dict(start_time_s=start, delivery_time_s=now, available_time_s=frame.available_time_s,
                    sample_times_s=frame.sample_times_s.tolist(), adc_codes=frame.adc_codes.tolist(),
                    analog_field_T=signal.tolist(), mean_coil_field_T=coil_signal.mean(axis=1).tolist(),
                    mean_magnet_field_T=(signal - coil_signal).mean(axis=1).tolist(),
                    subtracted_coil_field_T=correction.tolist(), prediction=np.array(prediction).tolist(),
                    estimate=result['estimate']))
            if now in command_times:
                estimate = simulation.tracker.predict(now)
                allocation = simulation.controller.command(estimate, simulation.reference)
                if not allocation['accepted']:
                    raise ValueError('Recorded-resolution controller replay exceeded its current budget')
                simulation.drivers.set_command(allocation['currents'])
                commands.append(dict(time_s=now, estimate=estimate, command_A=allocation['currents'].tolist(),
                    quantized_command_A=simulation.drivers.command.tolist()))
        if not np.array_equal([row['estimate']['pose'] for row in commands], [row['estimated_pose'] for row in report['trace']]):
            raise ValueError('Recorded-resolution replay does not exactly reproduce the saved estimated trajectory')
        cases.append(dict(report_path=str(path), report_sha256=hashlib.sha256(report_bytes).hexdigest(),
            checkpoint_sha256=hashlib.sha256(checkpoint_bytes).hexdigest(), recorded_source_sha256=report['source_sha256'],
            time_refinement=report['time_refinement'], magnetic_refinement=report['magnetic_refinement'],
            current_sample_times_s=[row['sample_times_s'][0] for row in checkpoint['readouts']],
            current_adc_codes=[row['adc_codes'][0] for row in checkpoint['readouts']],
            initial_noise_state=initial_noise_state,
            initial_hall_calibration=dict(gain=simulation.sensors.gain.tolist(), offset=simulation.sensors.offset.tolist()),
            packets=packets, commands=commands))
        print(f'Exact recorded observer/noise replay: {path.name}', flush=True)
    coarse, fine = cases
    if (coarse['initial_hall_calibration'] != fine['initial_hall_calibration']
            or coarse['initial_noise_state'] != fine['initial_noise_state']):
        raise ValueError('Recorded-resolution comparison requires identical Hall calibration and initial noise state')
    current_count = min(len(case['current_sample_times_s']) for case in cases)
    if coarse['current_sample_times_s'][:current_count] != fine['current_sample_times_s'][:current_count]:
        raise ValueError('Recorded-resolution comparison requires matching current-readout clocks')
    current_difference = np.array(fine['current_adc_codes'][:current_count]) - coarse['current_adc_codes'][:current_count]
    changed_rows = np.flatnonzero(np.any(current_difference, axis=1))
    comparisons = []
    for left, right in zip(coarse['packets'], fine['packets']):
        if left['sample_times_s'] != right['sample_times_s'] or left['available_time_s'] != right['available_time_s']:
            raise ValueError('Recorded-resolution comparison changed Hall sampling clocks')
        analog = np.array(right['analog_field_T']) - left['analog_field_T']
        codes = np.array(right['adc_codes']) - left['adc_codes']
        decoded = codes.mean(axis=1) * sensing.hall_readout()[2]
        coil = np.array(right['mean_coil_field_T']) - left['mean_coil_field_T']
        correction = np.array(right['subtracted_coil_field_T']) - left['subtracted_coil_field_T']
        motion = np.array(right['mean_magnet_field_T']) - left['mean_magnet_field_T']
        comparisons.append(dict(start_time_s=left['start_time_s'], changed_hall_adc_samples=int(np.count_nonzero(codes)),
            maximum_analog_difference_T=float(np.max(np.abs(analog))),
            maximum_decoded_difference_T=float(np.max(np.abs(decoded))),
            maximum_motion_difference_T=float(np.max(np.abs(motion))),
            maximum_coil_difference_T=float(np.max(np.abs(coil))),
            maximum_uncancelled_coil_difference_T=float(np.max(np.abs(coil - correction))),
            maximum_rounding_difference_T=float(np.max(np.abs(decoded - analog.mean(axis=1)))),
            maximum_corrected_field_difference_T=float(np.max(np.abs(decoded - correction))),
            changed_current_adc_samples_before_delivery=int(np.count_nonzero(current_difference[
                np.array(coarse['current_sample_times_s'][:current_count]) <= left['available_time_s']]))))
    result = dict(scope='Exact saved-input observer/controller replay and field attribution; no physical trajectory rerun',
        source_sha256=dict(simulation.source, **source_fingerprints(__file__)), exact_observer_and_noise_replay=True,
        first_changed_current_readout_s=None if not len(changed_rows) else coarse['current_sample_times_s'][changed_rows[0]],
        current_adc_step_A=electrical.current_quantum(), hall_adc_step_T=sensing.hall_readout()[2],
        cases=cases, comparisons=comparisons, coil_sampling_attribution=coil_sampling_attribution(cases),
        limitations=['Recorded nominal acceleration and physical fields are held fixed; replayed commands are not fed back into the plant.',
            'Quantization differences cancel common injected noise only when acquisition, noise parameters and sampling clocks match.',
            'Uncancelled coil differences combine sensing, calibration and interpolation effects; they do not identify one cause alone.'])
    if fine['magnetic_refinement'] != coarse['magnetic_refinement']:
        result['coil_map_refinement'] = coil_map_refinement(cases)
    output.write_text(json.dumps(result, indent=Fixed.report_precision, allow_nan=False) + '\n')
    print(json.dumps(dict(first_changed_current_readout_s=result['first_changed_current_readout_s'], comparisons=comparisons)), flush=True)
    return result


def refine_recorded_hover(baseline, comparison, initial_checkpoint, atlas_path, output, fixed_reference=False):
    from methodology import hover
    if output.exists():
        raise ValueError('Refinement output already exists; preserve its evidence')
    baseline_report, reference_report = (json.loads(path.read_text()) for path in (baseline, comparison))
    reference_checkpoint = comparison.with_name(comparison.stem + '_checkpoint.json')
    reference = hover.simulation_from_evidence(reference_report, atlas_path, False,
        fixed_step=reference_report['fixed_reference_step_s'])
    reference.load_checkpoint(reference_checkpoint)
    if reference.report()['trace'] != reference_report['trace'] or reference.peaks != reference_report['peaks']:
        raise ValueError('Reference report and checkpoint disagree')
    if reference.stop_reason != 'peer_trial_failed':
        raise ValueError('This diagnostic requires a reference stopped only by its peer comparison')
    reference_snapshot = hover.comparison_snapshot(reference)
    reference_snapshot['stop_reason'] = None
    simulation = hover.simulation_from_evidence(baseline_report, atlas_path, False)
    simulation.load_checkpoint(initial_checkpoint)
    if simulation.clock != 0 or simulation.finished or simulation.source != reference.source:
        raise ValueError('Refinement must start at the matching recorded release state')
    if fixed_reference:
        simulation.fixed_step = reference.fixed_step / Fixed.qualification_refinement_factor
    else:
        simulation.tolerance_scale /= Fixed.qualification_refinement_factor
    name = 'refined_reference' if fixed_reference else 'tighter'
    output.mkdir(parents=True)
    summary = dict(scope='Bounded numerical refinement from release against retained fine integration; not sustained hover qualification',
        source_sha256=source_fingerprints(__file__), simulation_source_sha256=simulation.source,
        inputs_sha256=source_fingerprints(baseline, comparison, reference_checkpoint, initial_checkpoint),
        atlas_identity=reference.atlas_configuration['identity'], tolerance_scale=simulation.tolerance_scale,
        integration_mode='fixed_event_aligned' if fixed_reference else 'adaptive', fixed_step_s=simulation.fixed_step,
        target_duration_s=reference.clock, original_reference_stop_reason=reference.stop_reason,
        status='running', comparisons=[])
    airborne_verification.save_summary(output / 'summary.json', summary)
    while not simulation.finished and simulation.clock < reference.clock - Fixed.adaptive_clock_tolerance:
        simulation.advance()
        if simulation.checkpoint_due() or simulation.finished:
            snapshot = hover.comparison_snapshot(simulation)
            hover.check_operating_limits(simulation, snapshot)
            if simulation.stop_reason is None:
                prefix = dict(reference_snapshot, trace=[row for row in reference_snapshot['trace'] if row['time_s'] <= simulation.clock])
                result = airborne_verification.compare_trials(prefix, snapshot, trace_only=True)
                if abs(simulation.clock - reference.clock) <= Fixed.adaptive_clock_tolerance:
                    result = hover.compare_prefixes(snapshot, reference_snapshot)
                if not result['converged']:
                    simulation.stop_reason = 'numerical_comparison_failed'
                    simulation.finished = True
                summary['comparisons'].append(dict(time_s=simulation.clock, **result))
            summary.update(duration_completed_s=simulation.clock, stop_reason=simulation.stop_reason)
            hover.save_trial(simulation, output, name)
            airborne_verification.save_summary(output / 'summary.json', summary)
            print(f'{name} comparison {simulation.clock:.6f} s: {simulation.stop_reason}', flush=True)
    summary.update(status='completed' if simulation.stop_reason is None and simulation.clock >= reference.clock - Fixed.adaptive_clock_tolerance else 'failed',
        elapsed_s=simulation.report()['elapsed_s'], adaptation=simulation.adaptation)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


def first_packet_response(diagnosis_path, atlas_path):
    diagnosis = json.loads(diagnosis_path.read_text())
    if 'first_packet_response' in diagnosis:
        raise ValueError('First-packet response already exists; preserve its evidence')
    cases = diagnosis['cases']
    simulations, frames, histories, commands = [], [], [], []
    for case in cases:
        path = Path(case['report_path'])
        checkpoint_path = path.with_name(path.stem + '_checkpoint.json')
        report_bytes, checkpoint_bytes, _, _, simulation = recorded_simulation(path, checkpoint_path, atlas_path=atlas_path)
        if (hashlib.sha256(report_bytes).hexdigest() != case['report_sha256']
                or hashlib.sha256(checkpoint_bytes).hexdigest() != case['checkpoint_sha256']):
            raise ValueError('First-packet response requires unchanged replay inputs')
        packet = case['packets'][0]
        frames.append(sensing.HallFrame(np.array(packet['adc_codes']), np.array(packet['sample_times_s']), packet['available_time_s']))
        times = np.array(case['current_sample_times_s'])
        available = times <= packet['available_time_s']
        histories.append(electrical.CurrentReadings(simulation.identities, times[available],
            np.array(case['current_adc_codes'])[available], times[available][-1]))
        commands.append(next(row for row in case['commands'] if row['time_s'] >= packet['delivery_time_s']))
        simulations.append(simulation)
    if commands[0]['time_s'] != commands[1]['time_s']:
        raise ValueError('First-packet command clocks differ')
    responses = {}
    for name, state_index, frame_index, history_index in (
            ('baseline', 0, 0, 0), ('reference', 1, 1, 1),
            ('baseline_with_reference_hall', 0, 1, 0),
            ('baseline_with_reference_current_readouts', 0, 0, 1),
            ('baseline_with_both_reference_readouts', 0, 1, 1)):
        simulation = simulations[state_index]
        tracker = deepcopy(simulation.tracker)
        packet = cases[state_index]['packets'][0]
        result = tracker.consume(frames[frame_index], histories[history_index], packet['delivery_time_s'])
        estimate = tracker.predict(commands[state_index]['time_s'])
        allocation = simulation.controller.command(estimate, simulation.reference)
        if not result['valid'] or not allocation['accepted']:
            raise ValueError('First-packet intervention lost tracking or current allocation')
        simulation.drivers.set_command(allocation['currents'])
        if name in ('baseline', 'reference') and (estimate != commands[state_index]['estimate']
                or allocation['currents'].tolist() != commands[state_index]['command_A']):
            raise ValueError('First-packet intervention does not reproduce the original command exactly')
        responses[name] = dict(estimate=estimate, command_A=allocation['currents'].tolist(),
            quantized_command_A=simulation.drivers.command.tolist())
    target = responses['reference']
    for response in responses.values():
        response.update(maximum_command_difference_from_reference_A=float(np.max(np.abs(
            np.array(response['command_A']) - target['command_A']))),
            maximum_quantized_command_difference_from_reference_A=float(np.max(np.abs(
                np.array(response['quantized_command_A']) - target['quantized_command_A']))))
    diagnosis['first_packet_response'] = dict(source_sha256=source_fingerprints(__file__),
        command_time_s=commands[0]['time_s'], exact_original_command_replay=True, responses=responses,
        limitations=['One saved Hall packet/current-readout intervention; no physical trajectory is rerun.',
            'Recorded acceleration histories are retained; this isolates the first command response, not later closed-loop stability.'])
    airborne_verification.save_summary(diagnosis_path, diagnosis)
    return diagnosis['first_packet_response']


def current_wrench_components(matrix, difference):
    normalized = matrix / np.linalg.norm(matrix, axis=1)[:, None]
    if levitation_sim.matrix_rank(normalized) != matrix.shape[0]:
        raise ValueError('Current attribution requires full wrench rank')
    _, _, basis = np.linalg.svd(normalized, full_matrices=False)
    active = basis.T @ (basis @ difference)
    null = difference - active
    return dict(wrench=(matrix @ difference).tolist(),
        active_current_norm_A=float(np.linalg.norm(active)), null_current_norm_A=float(np.linalg.norm(null)),
        total_current_norm_A=float(np.linalg.norm(difference)), maximum_current_difference_A=float(np.max(np.abs(difference))))


def force_attribution(diagnosis_path, atlas_path, output):
    from methodology import hover
    if output.exists():
        raise ValueError('Force attribution already exists; preserve its evidence')
    diagnosis = json.loads(diagnosis_path.read_text())
    simulations, reports, commands = [], [], []
    for case in diagnosis['cases']:
        path = Path(case['report_path'])
        report_bytes, checkpoint_bytes, report, checkpoint, simulation = recorded_simulation(
            path, path.with_name(path.stem + '_checkpoint.json'), atlas_path=atlas_path)
        if (hashlib.sha256(report_bytes).hexdigest() != case['report_sha256']
                or hashlib.sha256(checkpoint_bytes).hexdigest() != case['checkpoint_sha256']):
            raise ValueError('Force attribution requires unchanged recorded replay inputs')
        simulation.restore_checkpoint(checkpoint)
        simulations.append(simulation)
        reports.append(hover.comparison_snapshot(simulation))
        commands.append({row['time_s']: row for row in case['commands']})
    if ([row['time_s'] for row in reports[0]['trace']] != [row['time_s'] for row in reports[1]['trace']]
            or simulations[0].identities != simulations[1].identities):
        raise ValueError('Force attribution requires matching observation times and coil identities')
    geometry = simulations[1].plant.geometry
    rows = []
    for left, right in zip(reports[0]['trace'], reports[1]['trace']):
        matrices, nominal = [], []
        now = left['time_s']
        for simulation, sample, recorded_commands in zip(simulations, (left, right), commands):
            pose = np.array(sample['pose'])
            simulation.plant.reset(position=pose[:3] + [0, 0, simulation.plant.geometry.center_of_mass_height],
                rotation=Rotation.from_euler('xyz', pose[3:]))
            matrices.append(simulation.plant.magnetic_matrix(window_center=simulation.reference[:2]))
            if now in recorded_commands:
                levitation_sim.use_geometry(simulation.controller.geometry)
                _, center = levitation_sim.place_piece(simulation.controller.magnets, *recorded_commands[now]['estimate']['pose'])
                nominal.append(levitation_sim.actuator_matrix(simulation.controller.magnets, simulation.controller.array,
                    center, simulation.controller.field_evaluator))
        currents = [np.array(sample['currents_A']) for sample in (left, right)]
        wrenches = [matrix @ current for matrix, current in zip(matrices, currents)]
        current_component = current_wrench_components(matrices[1], currents[0] - currents[1])
        pose_component = (matrices[0] - matrices[1]) @ currents[0]
        difference = wrenches[0] - wrenches[1]
        row = dict(time_s=now, physical_wrenches=wrenches, physical_wrench_difference=difference,
            current_component=current_component, pose_component=pose_component,
            force_difference_weight_fraction=float(np.linalg.norm(difference[:3]) / geometry.weight),
            vertical_difference_weight_fraction=float(abs(difference[2]) / geometry.weight),
            lateral_difference_weight_fraction=float(np.linalg.norm(difference[:2]) / geometry.weight),
            torque_difference_Nm=float(np.linalg.norm(difference[3:])),
            equivalent_rim_force_weight_fraction=float(np.linalg.norm(difference[3:]) / geometry.weight / simulation.fields.base_radius))
        if now in commands[0] and now in commands[1]:
            raw = [np.array(command[now]['command_A']) for command in commands]
            rounded = [np.array(command[now]['quantized_command_A']) for command in commands]
            row.update(raw_command_component=current_wrench_components(matrices[1], raw[0] - raw[1]),
                rounded_command_component=current_wrench_components(matrices[1], rounded[0] - rounded[1]),
                rounding_wrench_difference=matrices[1] @ (rounded[0] - raw[0] - rounded[1] + raw[1]),
                nominal_requested_wrenches=[matrix @ current for matrix, current in zip(nominal, raw)])
        rows.append(row)
    selected = sorted({int(np.argmax([row['current_component']['maximum_current_difference_A'] for row in rows])),
        int(np.argmax([row['torque_difference_Nm'] for row in rows])),
        next(index for index, row in enumerate(rows) if row.get('rounded_command_component', {}).get('maximum_current_difference_A', 0) > 0)})
    interventions = []
    selected_times = sorted({max(time for time in commands[0] if time <= rows[index]['time_s']) for index in selected})
    for now in selected_times:
        original, reference = [command[now] for command in commands]
        simulation = simulations[0]
        targets = {}
        for name, estimate in (('baseline', original['estimate']), ('reference', reference['estimate']),
                ('baseline_pose_reference_velocity', dict(original['estimate'], velocity=reference['estimate']['velocity']))):
            allocation = simulation.controller.command(estimate, simulation.reference)
            if not allocation['accepted']:
                raise ValueError('Force-attribution counterfactual exceeds the reserved current budget')
            if name in ('baseline', 'reference') and not np.array_equal(allocation['currents'],
                    (original if name == 'baseline' else reference)['command_A']):
                raise ValueError('Force-attribution controller replay differs from the recorded command')
            targets[name] = simulation.controller.nominal_matrix @ allocation['currents']
        interventions.append(dict(time_s=now, targets=targets,
            velocity_contribution=targets['baseline'] - targets['baseline_pose_reference_velocity'],
            pose_contribution=targets['baseline_pose_reference_velocity'] - targets['reference']))
    metrics = {}
    for key in ('force_difference_weight_fraction', 'vertical_difference_weight_fraction', 'lateral_difference_weight_fraction',
                'torque_difference_Nm', 'equivalent_rim_force_weight_fraction'):
        worst = max(rows, key=lambda row: row[key])
        metrics[key] = dict(maximum=worst[key], time_s=worst['time_s'])
    for key in ('current_component', 'raw_command_component', 'rounded_command_component'):
        components = [row[key] for row in rows if key in row]
        total = sum(row['total_current_norm_A'] ** 2 for row in components)
        metrics[key] = dict(null_fraction_of_squared_current_difference=sum(row['null_current_norm_A'] ** 2 for row in components) / total,
            active_fraction_of_squared_current_difference=sum(row['active_current_norm_A'] ** 2 for row in components) / total)
    result = dict(scope='Saved-state instantaneous force/COM-torque and current-subspace attribution; no motion rerun or new acceptance limits',
        source_sha256=dict(simulations[0].source, **source_fingerprints(__file__)),
        diagnosis_sha256=hashlib.sha256(diagnosis_path.read_bytes()).hexdigest(),
        inputs=[{key: case[key] for key in ('report_path', 'report_sha256', 'checkpoint_sha256')} for case in diagnosis['cases']],
        atlas_identity=simulations[0].atlas_configuration['identity'], weight_N=geometry.weight,
        mass_kg=geometry.weight / geometry.gravity, base_radius_m=simulation.fields.base_radius,
        metrics=metrics, trace=rows, command_interventions=interventions,
        limitations=['Wrenches are reconstructed instantaneously at saved command times and the final endpoint, not the original held/interval-average forces or full-rate extrema.',
            'Current and pose contributions use the reference pose as the shared linearization; their sum equals the instantaneous physical wrench difference.',
            'Null fractions describe squared current-difference norms across saved samples, not fractions of coil power or proof of harmlessness.',
            'Velocity interventions change only the saved estimated Euler-coordinate velocity at selected command times; they do not rerun the observer or physical feedback.',
            'Weight and equivalent rim force are interpretation scales, not newly qualified force or torque acceptance budgets.'])
    output.write_text(json.dumps(result, default=lambda value: value.tolist(), indent=Fixed.report_precision, allow_nan=False) + '\n')
    return result


def precision_diagnosis(capture_path, recordings, output):
    if output.exists():
        raise ValueError('Precision-screen output already exists; preserve its evidence')
    capture_bytes = capture_path.read_bytes()
    capture = json.loads(capture_bytes)
    summary_bytes = (recordings / 'summary.json').read_bytes()
    summary = json.loads(summary_bytes)
    if summary['status'] != 'completed' or summary['capture_sha256'] != hashlib.sha256(capture_bytes).hexdigest():
        raise ValueError('Precision screen requires the matching completed single-count trials')
    names = ('baseline', 'changed_count')
    reports = {name: json.loads((recordings / (name + '.json')).read_bytes()) for name in names}
    checkpoints = {name: json.loads((recordings / (name + '_checkpoint.json')).read_bytes()) for name in names}
    for name in names:
        if (reports[name]['trace'] != checkpoints[name]['trace']
                or reports[name]['source_sha256'] != checkpoints[name]['source_sha256']):
            raise ValueError('Precision screen report and checkpoint differ')
    bits_recorded = round(np.log2(Fixed.hall_supply_voltage / Fixed.hall_sensitivity / capture['hall_ADC_step_T']))
    if bits_recorded not in Fixed.hall_adc_precision_candidates or Fixed.hall_samples_per_sensor != len(capture['cases'][0]['frames'][0]['adc_codes'][0]):
        raise ValueError('Precision screen must include the original ADC and preserve its sample count')
    with patch.object(Fixed, 'hall_adc_native_bits', bits_recorded):
        report = reports['baseline']
        simulation = airborne.AirborneSimulation(airborne_verification.recorded_construction(report), report['intended_duration_s'],
            disturbed=report['disturbed'], time_refinement=report['time_refinement'], magnetic_refinement=report['magnetic_refinement'])
        simulation.tracker.acceleration_times = checkpoints['baseline']['acceleration_times']
        simulation.tracker.accelerations = [np.array(value) for value in checkpoints['baseline']['accelerations']]
        probe = copy(simulation.sensors)
        probe.random = np.random.default_rng(Fixed.hall_virtual_seed)
        arrays = {name: {key: np.array(checkpoints[name][key]) for key in ('field_times', 'field_values', 'coil_field_values')} for name in names}
        observations = {bits: [] for bits in Fixed.hall_adc_precision_candidates}
        noise_state = deepcopy(simulation.sensors.random.bit_generator.state)
        replayed, frame_index = [], 0
        for step, now, frame, history, _ in recorded_events(simulation, checkpoints['baseline']):
            if frame is not None:
                midpoint = float(frame.sample_times_s.mean())
                prediction = simulation.tracker.propagate(midpoint)
                recorded = simulation.tracker.consume(frame, history, now)
                if not recorded['valid']:
                    raise ValueError('Precision-screen reference replay lost tracking')
                start = summary['cases']['baseline']['packets'][frame_index]['start_time_s']
                signals, histories, truths = {}, dict(baseline=history), {}
                for name in names:
                    state = checkpoints[name]
                    if state['sensors']['gain'] != checkpoints['baseline']['sensors']['gain'] or state['sensors']['offset'] != checkpoints['baseline']['sensors']['offset']:
                        raise ValueError('Precision screen requires identical Hall calibration')
                    signals[name] = simulation.sensors.moving_signal(arrays[name]['field_times'], arrays[name]['field_values'], start,
                        arrays[name]['coil_field_values'], simulation.drivers.inductance / simulation.drivers.resistance)
                    trace = reports[name]['trace']
                    truths[name] = np.array([np.interp(midpoint, [row['time_s'] for row in trace], [row['pose'][axis] for row in trace]) for axis in range(6)])
                    if name != 'baseline':
                        readings = [reading for reading in state['readouts'] if reading['available_time_s'] <= frame.available_time_s]
                        histories[name] = electrical.CurrentReadings(simulation.identities,
                            np.concatenate([reading['sample_times_s'] for reading in readings]),
                            np.vstack([reading['adc_codes'] for reading in readings]), readings[-1]['available_time_s'])
                for bits in Fixed.hall_adc_precision_candidates:
                    cases, packets = {}, {}
                    with patch.object(Fixed, 'hall_adc_native_bits', bits):
                        for name in names:
                            probe.random.bit_generator.state = deepcopy(noise_state)
                            candidate = probe.encode_signal(signals[name], start)
                            if (not np.array_equal(candidate.sample_times_s, frame.sample_times_s)
                                    or candidate.available_time_s != frame.available_time_s
                                    or probe.random.bit_generator.state != simulation.sensors.random.bit_generator.state):
                                raise ValueError('Precision change altered scan timing or the injected noise sequence')
                            if bits == bits_recorded:
                                expected = summary['cases'][name]['packets'][frame_index]['adc_codes']
                                if name == 'changed_count' and frame_index == 0:
                                    expected = summary['cases']['baseline']['packets'][frame_index]['adc_codes']
                                if not np.array_equal(candidate.adc_codes, expected):
                                    raise ValueError('Precision-screen reference ADC does not reproduce the saved packet')
                            estimate = simulation.tracker.estimator.estimate(candidate, histories[name], prediction)
                            if bits == bits_recorded and name == 'baseline' and estimate != recorded['estimate']:
                                raise ValueError('Precision-screen reference fit differs from the recorded-input replay')
                            candidate_pose = np.array(estimate['best_candidate'])
                            cases[name] = dict(estimate=estimate,
                                position_error_m=float(np.linalg.norm(candidate_pose[:3] - truths[name][:3])),
                                angle_error_rad=float((Rotation.from_euler('xyz', candidate_pose[3:]) * Rotation.from_euler('xyz', truths[name][3:]).inv()).magnitude()))
                            packets[name] = candidate
                        rounding = (packets['changed_count'].adc_codes - packets['baseline'].adc_codes).mean(axis=1) * sensing.hall_readout()[2]
                        rounding -= (signals['changed_count'] - signals['baseline']).mean(axis=1)
                    observations[bits].append(dict(start_time_s=start, cases=cases, differential_rounding_T=rounding.tolist()))
                noise_state = deepcopy(simulation.sensors.random.bit_generator.state)
                frame_index += 1
                print(f'Precision screen scan {start:.6f} s', flush=True)
            if step % simulation.command_tick == 0:
                replayed.append(simulation.tracker.predict(now)['pose'])
        if not np.array_equal(replayed, [row['estimated_pose'] for row in report['trace']]):
            raise ValueError('Precision-screen reference does not reproduce the saved estimated trajectory exactly')
    sensor_mean_noise = float(Fixed.hall_output_noise * np.sqrt(sensing.hall_readout()[1].mean()))
    candidates = []
    for bits, rows in observations.items():
        fits = [case for row in rows for case in row['cases'].values()]
        rounding = np.array([row['differential_rounding_T'] for row in rows])
        candidates.append(dict(native_bits=bits, all_fits_accepted=all(case['estimate']['accepted'] for case in fits),
            maximum_position_error_m=max(case['position_error_m'] for case in fits),
            maximum_angle_error_rad=max(case['angle_error_rad'] for case in fits),
            differential_rounding_rms_T=float(np.sqrt(np.mean(rounding ** 2))),
            differential_rounding_peak_T=float(np.max(np.abs(rounding))), trace=rows))
    result = dict(scope='ADC precision comparison on fixed recorded analog fields and measured currents, with common recorded estimator priors',
        capture_sha256=hashlib.sha256(capture_bytes).hexdigest(), recordings_summary_sha256=hashlib.sha256(summary_bytes).hexdigest(),
        recorded_source_sha256=report['source_sha256'], source_sha256=dict(simulation.source, **source_fingerprints(__file__)),
        exact_reference_pose_and_ADC_replay=True, identical_noise_and_scan_timing=True, samples_per_sensor=Fixed.hall_samples_per_sensor,
        sensor_mean_noise_rms_T=sensor_mean_noise, candidates=candidates,
        limitations=['The recorded physical trajectories and current histories belong to the original ADC, not to newly coupled candidate trials.',
            'All candidate fits use the same baseline prior pose/velocity; this isolates conversion effects without establishing candidate observer or feedback stability.',
            'The deliberate first-packet count intervention is omitted when redigitizing analog fields; later recorded physical fields retain its consequences.',
            'True poses are interpolated from saved traces for scoring only and are not supplied to the estimator.',
            'Converter precision is ideal native quantization; ADC noise, nonlinearity, reference accuracy and mux settling are not component-qualified.',
            'No hardware rate, analog sensor noise, calibration error or flight acceptance limit is changed.'])
    output.write_text(json.dumps(result, indent=Fixed.report_precision, allow_nan=False) + '\n')
    print(json.dumps([{key: value for key, value in candidate.items() if key != 'trace'} for candidate in candidates]), flush=True)
    return result


def sensor_resolution_diagnosis(baseline, output, comparison=None):
    if output.exists():
        raise ValueError('Sensor-resolution output already exists; preserve its evidence')
    baseline_bytes = None if baseline is None else baseline.read_bytes()
    baseline_report = None if baseline_bytes is None else json.loads(baseline_bytes)
    comparison_report = None if comparison is None else json.loads(comparison.read_bytes())
    duration = Fixed.airborne_check_duration
    if comparison_report is not None:
        if (comparison_report['magnetic_refinement'] != baseline_report['magnetic_refinement']
                or comparison_report['time_refinement'] != Fixed.qualification_refinement_factor
                or baseline_report['time_refinement'] != 1):
            raise ValueError('Targeted physical capture requires the baseline and its time-refined report')
        differing = [left['time_s'] for left, right in zip(baseline_report['trace'], comparison_report['trace'])
            if left['estimated_pose'] != right['estimated_pose']]
        if not differing:
            raise ValueError('Targeted capture requires a recorded estimated-pose disagreement')
        duration = max(duration, np.ceil(min(differing) / Fixed.airborne_checkpoint_interval) * Fixed.airborne_checkpoint_interval)
    construction = magnetics.ConstructionModel() if baseline_report is None else airborne_verification.recorded_construction(baseline_report)
    cases = []
    for refinement in (1, Fixed.qualification_refinement_factor):
        simulation = airborne.AirborneSimulation(construction, duration,
            disturbed=True if baseline_report is None else baseline_report['disturbed'], time_refinement=refinement,
            magnetic_refinement=None if baseline_report is None else baseline_report['magnetic_refinement'])
        if baseline_report is not None and any(baseline_report['source_sha256'].get(name) != digest for name, digest in simulation.source.items()):
            raise ValueError('Sensor-resolution diagnosis requires unchanged baseline simulation sources')
        frames, fits = [], []
        original_encode = simulation.sensors.encode_signal
        original_estimate = simulation.tracker.estimator.estimate

        def record_encoding(signal, start):
            noise_state = deepcopy(simulation.sensors.random.bit_generator.state)
            before = json.dumps(noise_state, sort_keys=True)
            frame = original_encode(signal, start)
            frames.append(dict(start_time_s=start, analog_field_T=np.asarray(signal).tolist(),
                adc_codes=frame.adc_codes.tolist(), sample_times_s=frame.sample_times_s.tolist(),
                available_time_s=frame.available_time_s,
                noise_state_before=noise_state,
                noise_state_before_sha256=hashlib.sha256(before.encode()).hexdigest(),
                noise_state_after_sha256=hashlib.sha256(json.dumps(simulation.sensors.random.bit_generator.state, sort_keys=True).encode()).hexdigest()))
            return frame

        def record_estimate(frame, currents, prediction):
            estimate = original_estimate(frame, currents, prediction)
            coil, _, _ = sensing.measured_coil_fields(simulation.tracker.estimator.field_model, currents, frame)
            fits.append(dict(estimate=estimate, subtracted_coil_field_T=coil.tolist(),
                prediction=None if prediction is None else [np.asarray(value).tolist() for value in prediction]))
            return estimate

        with patch.object(simulation.sensors, 'encode_signal', record_encoding), \
             patch.object(simulation.tracker.estimator, 'estimate', record_estimate):
            while not simulation.finished:
                simulation.advance()
                if simulation.step % simulation.checkpoint_tick == 0:
                    print(f'Sensor-resolution case {refinement}: {simulation.step * simulation.period:.6f} s', flush=True)
        cases.append(dict(time_refinement=refinement, report=simulation.report(), frames=frames, fits=fits,
            field_times_s=simulation.field_times, total_fields_T=np.array(simulation.field_values).tolist(),
            coil_fields_T=np.array(simulation.coil_field_values).tolist(),
            hall_gain=simulation.sensors.gain.tolist(), hall_offset_T=simulation.sensors.offset.tolist(),
            electrical_time_constant_s=simulation.drivers.inductance / simulation.drivers.resistance,
            current_sample_times_s=np.concatenate([row.sample_times_s for row in simulation.readouts]).tolist(),
            current_adc_codes=np.vstack([row.adc_codes for row in simulation.readouts]).tolist()))
    coarse, fine = cases
    if baseline_report is not None and coarse['report']['trace'] != baseline_report['trace'][:len(coarse['report']['trace'])]:
        raise ValueError('Short sensor diagnostic does not reproduce the original baseline trace')
    if comparison_report is not None and fine['report']['trace'] != comparison_report['trace'][:len(fine['report']['trace'])]:
        raise ValueError('Short sensor diagnostic does not reproduce the original refined trace')
    if len(coarse['frames']) != len(fine['frames']) or not coarse['frames']:
        raise ValueError('Sensor diagnostic requires matching nonempty Hall sequences')
    comparisons = []
    quantum = sensing.hall_readout()[2]
    for index, (left, right) in enumerate(zip(coarse['frames'], fine['frames'])):
        if (left['sample_times_s'] != right['sample_times_s'] or left['available_time_s'] != right['available_time_s']
                or left['noise_state_before_sha256'] != right['noise_state_before_sha256']
                or left['noise_state_after_sha256'] != right['noise_state_after_sha256']):
            raise ValueError('Refinement changed Hall sample times or the noise sequence')
        analog_difference = np.array(right['analog_field_T']) - left['analog_field_T']
        code_difference = np.array(right['adc_codes']) - left['adc_codes']
        coil_difference = np.array(fine['fits'][index]['subtracted_coil_field_T']) - coarse['fits'][index]['subtracted_coil_field_T']
        interpolation_components = {}
        for component in ('coil', 'magnet'):
            fields = np.array(fine['coil_fields_T'])
            if component == 'magnet':
                fields = np.array(fine['total_fields_T']) - fields
            fields *= fine['hall_gain']
            time_constant = fine['electrical_time_constant_s'] if component == 'coil' else None
            dense = sensing.filtered_scan(np.array(fine['field_times_s']), fields, right['start_time_s'], time_constant)
            sparse = sensing.filtered_scan(np.array(fine['field_times_s'])[::Fixed.qualification_refinement_factor],
                fields[::Fixed.qualification_refinement_factor], right['start_time_s'], time_constant)
            interpolation_components[component] = dense - sparse
        interpolation_difference = sum(interpolation_components.values())
        sampled = simulation.sensors.moving_signal(np.array(fine['field_times_s']),
            np.array(fine['total_fields_T']), right['start_time_s'], np.array(fine['coil_fields_T']), fine['electrical_time_constant_s'])
        if not np.array_equal(sampled, np.array(right['analog_field_T'])):
            raise ValueError('Saved field history does not exactly reproduce the analog Hall scan')
        before_delivery = np.array(coarse['current_sample_times_s']) <= left['available_time_s']
        comparisons.append(dict(start_time_s=left['start_time_s'], identical_noise_and_sample_times=True,
            maximum_analog_difference_T=float(np.max(np.abs(analog_difference))),
            maximum_analog_difference_ADC_steps=float(np.max(np.abs(analog_difference)) / quantum),
            changed_adc_samples=int(np.count_nonzero(code_difference)), adc_samples=int(code_difference.size),
            maximum_code_change=int(np.max(np.abs(code_difference))),
            maximum_mean_decoded_difference_T=float(np.max(np.abs(code_difference.mean(axis=1))) * quantum),
            maximum_coil_correction_difference_T=float(np.max(np.abs(coil_difference))),
            prediction_difference=None if coarse['fits'][index]['prediction'] is None else (
                np.array(fine['fits'][index]['prediction']) - coarse['fits'][index]['prediction']).tolist(),
            changed_current_adc_samples_before_delivery=int(np.count_nonzero(
                (np.array(coarse['current_adc_codes']) - fine['current_adc_codes'])[before_delivery])),
            field_interpolation_on_same_fine_trajectory=dict(
                maximum_total_difference_T=float(np.max(np.abs(interpolation_difference))),
                maximum_coil_difference_T=float(np.max(np.abs(interpolation_components['coil']))),
                maximum_magnet_difference_T=float(np.max(np.abs(interpolation_components['magnet']))),
                remaining_difference_on_common_grid_T=float(np.max(np.abs(analog_difference - interpolation_difference)))),
            fitted_pose_difference=(np.array(fine['fits'][index]['estimate']['pose']) - coarse['fits'][index]['estimate']['pose']).tolist()))
    result = dict(scope='Targeted early Hall scans under independent time refinement; full physics retained, no controller changes',
        source_sha256=dict(simulation.source, **source_fingerprints(__file__)),
        baseline_sha256=None if baseline_bytes is None else hashlib.sha256(baseline_bytes).hexdigest(),
        exact_baseline_prefix=baseline_report is not None,
        exact_refined_prefix=comparison_report is not None,
        comparison_sha256=None if comparison is None else hashlib.sha256(comparison.read_bytes()).hexdigest(),
        hall_ADC_step_T=quantum, cases=cases, comparisons=comparisons,
        coupled_comparison=airborne_verification.compare_trials(coarse['report'], fine['report']),
        changed_current_adc_samples=int(np.count_nonzero(np.array(coarse['current_adc_codes']) - fine['current_adc_codes'])),
        limitations=['Only the configured short interface interval is rerun; it does not qualify sustained flight or prove convergence.',
            'Interpolation is isolated by discarding intermediate samples of the same fine trajectory; the remaining common-grid difference includes integration and force-averaging errors.',
            'Identical RNG states, sample arrays and unchanged encoding establish identical injected Hall noise.'])
    output.write_text(json.dumps(result, indent=Fixed.report_precision, allow_nan=False) + '\n')
    print(json.dumps(comparisons), flush=True)
    return result


def magnetic_sensor_diagnosis(capture_path, baseline, checkpoint_path, output):
    if output.exists():
        raise ValueError('Magnetic sensor diagnosis already exists; preserve its evidence')
    capture_bytes = capture_path.read_bytes()
    captured = json.loads(capture_bytes)
    coarse = captured['cases'][0]
    report_bytes, checkpoint_bytes, report, checkpoint, simulation = recorded_simulation(baseline, checkpoint_path)
    if any(coarse['report']['source_sha256'].get(name) != digest for name, digest in simulation.source.items()):
        raise ValueError('Compared sensor captures must use the same simulation sources')
    duration = coarse['report']['intended_duration_s']
    fields, coils, times = (np.array(checkpoint[key]) for key in ('field_values', 'coil_field_values', 'field_times'))
    probe = copy(simulation.sensors)
    probe.random = np.random.default_rng(Fixed.hall_virtual_seed)
    comparisons, replayed = [], []
    for step, now, frame, history, _ in recorded_events(simulation, checkpoint):
        if now > duration:
            break
        if frame is not None:
            index = len(comparisons)
            left = coarse['frames'][index]
            start = left['start_time_s']
            analog = simulation.sensors.moving_signal(times, fields, start, coils,
                simulation.drivers.inductance / simulation.drivers.resistance)
            probe.random.bit_generator.state = left['noise_state_before']
            reproduced = probe.encode_signal(analog, start)
            if not np.array_equal(frame.adc_codes, reproduced.adc_codes):
                raise ValueError('Magnetic replay did not reproduce the same injected Hall noise')
            prediction = simulation.tracker.propagate(float(frame.sample_times_s.mean()))
            result = simulation.tracker.consume(frame, history, now)
            if not result['valid']:
                raise ValueError('Magnetic replay lost the recorded usable estimate')
            correction, _, _ = sensing.measured_coil_fields(simulation.tracker.estimator.field_model, history, frame)
            delta = frame.adc_codes - np.array(left['adc_codes'])
            comparisons.append(dict(start_time_s=start, identical_injected_noise=True,
                maximum_analog_difference_T=float(np.max(np.abs(analog - left['analog_field_T']))),
                changed_adc_samples=int(np.count_nonzero(delta)), maximum_adc_code_change=int(np.max(np.abs(delta))),
                maximum_coil_correction_difference_T=float(np.max(np.abs(correction - coarse['fits'][index]['subtracted_coil_field_T']))),
                prediction_difference=(np.array(prediction) - coarse['fits'][index]['prediction']).tolist(),
                fitted_pose_difference=(np.array(result['estimate']['pose']) - coarse['fits'][index]['estimate']['pose']).tolist(),
                refined_analog_field_T=analog.tolist(), refined_adc_codes=frame.adc_codes.tolist()))
        if step % simulation.command_tick == 0:
            replayed.append(simulation.tracker.predict(now)['pose'])
    if not np.array_equal(replayed, [row['estimated_pose'] for row in report['trace'][:len(replayed)]]):
        raise ValueError('Magnetic sensor replay differs from the recorded estimated trajectory')
    nominal = simulation.tracker.estimator.field_model.coil_field
    physical = simulation.fields.coil_field
    result = dict(scope='Early magnetic-refinement sensor replay; recorded physical motion, no new force integration',
        source_sha256=dict(simulation.source, **source_fingerprints(__file__)),
        baseline_capture_sha256=hashlib.sha256(capture_bytes).hexdigest(),
        refined_report_sha256=hashlib.sha256(report_bytes).hexdigest(), checkpoint_sha256=hashlib.sha256(checkpoint_bytes).hexdigest(),
        exact_refined_estimate_prefix=True, duration_s=duration, comparisons=comparisons,
        coil_map_relative_norm_change=float(np.linalg.norm(physical - nominal) / np.linalg.norm(nominal)),
        coil_map_maximum_change_T_per_A=float(np.max(np.abs(physical - nominal))))
    output.write_text(json.dumps(result, indent=Fixed.report_precision, allow_nan=False) + '\n')
    print(json.dumps([{key: value for key, value in row.items() if key not in
        ('refined_analog_field_T', 'refined_adc_codes')} for row in comparisons]), flush=True)
    return result


def observer_diagnosis(baseline, checkpoint_path, output):
    report_bytes, checkpoint_bytes, report, checkpoint, simulation = recorded_simulation(baseline, checkpoint_path, True)
    ratios = Fixed.pose_observer_ratio_candidates
    if Fixed.pose_observer_frequency_ratio not in ratios:
        raise ValueError('Observer candidates must include the baseline setting for exact replay verification')
    sensors = {str(seed): deepcopy(simulation.sensors) for seed in Fixed.hall_validation_noise_seeds}
    for seed, sensor in sensors.items():
        sensor.random = np.random.default_rng(int(seed))
    cases = {(seed, ratio): deepcopy(simulation.tracker) for seed in ('recorded', *sensors) for ratio in ratios}
    traces = {key: [] for key in cases}
    packet_hashes = {seed: hashlib.sha256() for seed in ('recorded', *sensors)}
    field_times, fields = np.array(checkpoint['field_times']), np.array(checkpoint['field_values'])
    for step, now, frame, history, _ in recorded_events(simulation, checkpoint):
        if frame is not None:
            start = frame.sample_times_s[0, 0] - Fixed.hall_power_settle_time
            packets = dict(recorded=frame, **{seed: sensor.sample_moving(field_times, fields, start,
                np.array(checkpoint['coil_field_values']), simulation.drivers.inductance / simulation.drivers.resistance)
                for seed, sensor in sensors.items()})
            for seed, packet in packets.items():
                packet_hashes[seed].update(packet.adc_codes.tobytes())
                for ratio in ratios:
                    with patch.object(Fixed, 'pose_observer_frequency_ratio', ratio):
                        cases[seed, ratio].consume(packet, history, now)
        if step % simulation.command_tick == 0:
            for key, tracker in cases.items():
                traces[key].append(dict(time_s=now, **tracker.predict(now)))
        if step % simulation.checkpoint_tick == 0:
            print(f'Observer candidates replayed through {now:.6f} s', flush=True)
    original = traces['recorded', Fixed.pose_observer_frequency_ratio]
    if not all(row['valid'] for row in original) or not np.array_equal(
            [row['pose'] for row in original], [row['estimated_pose'] for row in report['trace']]):
        raise ValueError('Baseline observer replay must exactly reproduce recorded estimates before comparing parameters')
    times = np.array([row['time_s'] for row in report['trace']])
    poses = np.array([row['pose'] for row in report['trace']])
    velocities = np.gradient(poses, times, axis=0)
    summaries = []
    for (seed, ratio), rows in traces.items():
        if not np.array_equal(times, [row['time_s'] for row in rows]):
            raise ValueError('Observer replay times differ from recorded trajectory')
        valid = np.array([row['valid'] for row in rows])
        summary = dict(noise_sequence=seed, observer_frequency_ratio=ratio,
            observer_frequency_Hz=ratio * Fixed.pose_controller_frequency, all_estimates_valid=bool(valid.all()),
            valid_estimates=int(valid.sum()), observations=len(rows), trace=rows)
        if np.any(valid):
            estimates = np.array([row['pose'] for row in rows if row['valid']])
            rates = np.array([row['velocity'] for row in rows if row['valid']])
            position_errors = np.linalg.norm(estimates[:, :3] - poses[valid, :3], axis=1)
            angle_errors = (Rotation.from_euler('xyz', estimates[:, 3:]) * Rotation.from_euler('xyz', poses[valid, 3:]).inv()).magnitude()
            rate_errors = rates - velocities[valid]
            summary.update(peak_position_error_m=float(position_errors.max()), peak_angle_error_rad=float(angle_errors.max()),
                linear_velocity_error_rms_m_s=float(np.sqrt(np.mean(np.sum(rate_errors[:, :3] ** 2, axis=1)))),
                euler_rate_error_rms_rad_s=float(np.sqrt(np.mean(np.sum(rate_errors[:, 3:] ** 2, axis=1)))),
                within_pose_limits=bool(valid.all() and position_errors.max() <= Fixed.qualification_position_error
                                       and angle_errors.max() <= Fixed.qualification_angle_error))
        else:
            summary['within_pose_limits'] = False
        summaries.append(summary)
        print(json.dumps({key: value for key, value in summary.items() if key != 'trace'}), flush=True)
    result = dict(scope='Sensor-only observer parameter and Hall-noise screening on a fixed recorded physical trajectory',
        baseline_sha256=hashlib.sha256(report_bytes).hexdigest(), checkpoint_sha256=hashlib.sha256(checkpoint_bytes).hexdigest(),
        baseline_source_sha256=report['source_sha256'], source_sha256=dict(simulation.source, **source_fingerprints(__file__)),
        exact_baseline_pose_replay=True, exact_baseline_noise_replay=True,
        adc_sha256={seed: digest.hexdigest() for seed, digest in packet_hashes.items()}, cases=summaries,
        limitations=['All candidates start from the same acquired observer state; a fresh closed-loop trial must repeat acquisition at the chosen setting.',
            'Recorded nominal acceleration and current ADC inputs are held fixed, not recomputed for each candidate observer.',
            'Additional seeds vary post-release Hall noise only; calibration errors, initial acquisition and current noise remain recorded values.',
            'True poses and finite-difference velocities score results only; candidate trackers receive sensor packets and nominal acceleration inputs.',
            'Parameter-file differences are recorded and require exact baseline pose/noise reproduction; this does not resume a changed-source checkpoint.',
            'These are selection screens, not closed-loop stability, numerical convergence or hardware qualification.'])
    output.write_text(json.dumps(result, indent=Fixed.report_precision, allow_nan=False) + '\n')
    return result


def feedback_diagnosis(baseline, checkpoint_path, output):
    report_bytes, checkpoint_bytes, report, checkpoint, simulation = recorded_simulation(baseline, checkpoint_path)
    trackers = dict(recorded=simulation.tracker, analog_hall_signal=deepcopy(simulation.tracker),
                    exact_coil_subtraction=deepcopy(simulation.tracker),
                    exact_pose_measurements=deepcopy(simulation.tracker))
    times = np.array([row['time_s'] for row in report['trace']])
    poses = np.array([row['pose'] for row in report['trace']])
    velocities = np.gradient(poses, times, axis=0)
    field_times = np.array(checkpoint['field_times'])
    coil_fields = np.array(checkpoint['coil_field_values'])
    time_constant = simulation.drivers.inductance / simulation.drivers.resistance
    original_correction = sensing.measured_coil_fields

    def analog_readout(field_model, readings, frame):
        field, magnitude, uncertainty = original_correction(field_model, readings, frame)
        start = frame.sample_times_s[0, 0] - Fixed.hall_power_settle_time
        analog = simulation.sensors.moving_signal(field_times,
            np.array(checkpoint['field_values']), start, coil_fields, time_constant).mean(axis=1)
        quantization = sensing.hall_readout()[2]
        decoded = ((frame.adc_codes + 0.5) * quantization - Fixed.hall_supply_voltage / 2 / Fixed.hall_sensitivity).mean(axis=1)
        return field + decoded - analog, magnitude, uncertainty

    def exact_correction(field_model, readings, frame):
        _, magnitude, uncertainty = original_correction(field_model, readings, frame)
        start = frame.sample_times_s[0, 0] - Fixed.hall_power_settle_time
        return sensing.filtered_scan(field_times, coil_fields * simulation.sensors.gain, start, time_constant).mean(axis=1), magnitude, uncertainty

    def exact_position(frame, readings, prediction):
        midpoint = float(frame.sample_times_s.mean())
        return dict(accepted=True, reason='diagnostic_exact_position',
                    pose=[float(np.interp(midpoint, times, values)) for values in poses.T])

    rows = []
    for step, now, frame, history, _ in recorded_events(simulation, checkpoint):
        if frame is not None:
            consumed = dict(recorded=trackers['recorded'].consume(frame, history, now))
            with patch.object(sensing, 'measured_coil_fields', analog_readout):
                consumed['analog_hall_signal'] = trackers['analog_hall_signal'].consume(frame, history, now)
            with patch.object(sensing, 'measured_coil_fields', exact_correction):
                consumed['exact_coil_subtraction'] = trackers['exact_coil_subtraction'].consume(frame, history, now)
            with patch.object(trackers['exact_pose_measurements'].estimator, 'estimate', exact_position):
                consumed['exact_pose_measurements'] = trackers['exact_pose_measurements'].consume(frame, history, now)
            if not all(value['valid'] for value in consumed.values()):
                raise ValueError(f'Feedback diagnostic lost tracking: {consumed}')
        if step % simulation.command_tick == 0:
            estimates = {name: tracker.predict(now) for name, tracker in trackers.items()}
            if not all(value['valid'] for value in estimates.values()):
                raise ValueError(f'Feedback diagnostic prediction unavailable: {estimates}')
            rows.append(dict(time_s=now, estimates=estimates))
    if not np.array_equal(times, [row['time_s'] for row in rows]):
        raise ValueError('Replay and recorded trace observation times differ')
    replayed = np.array([row['estimates']['recorded']['pose'] for row in rows])
    if not np.array_equal(replayed, [row['estimated_pose'] for row in report['trace']]):
        raise ValueError('Feedback replay does not reproduce the recorded pose estimates exactly')
    metrics = {}
    for name in trackers:
        errors = np.array([row['estimates'][name]['velocity'] for row in rows]) - velocities
        metrics[name] = dict(linear_velocity_error_rms_m_s=float(np.sqrt(np.mean(np.sum(errors[:, :3] ** 2, axis=1)))),
            euler_rate_error_rms_rad_s=float(np.sqrt(np.mean(np.sum(errors[:, 3:] ** 2, axis=1)))),
            peak_linear_velocity_error_m_s=float(np.max(np.linalg.norm(errors[:, :3], axis=1))),
            peak_euler_rate_error_rad_s=float(np.max(np.linalg.norm(errors[:, 3:], axis=1))))
        settled = errors[times >= Fixed.hall_tracking_acquisition_duration]
        if len(settled):
            metrics[name]['after_acquisition_duration'] = dict(start_time_s=Fixed.hall_tracking_acquisition_duration,
                linear_velocity_error_rms_m_s=float(np.sqrt(np.mean(np.sum(settled[:, :3] ** 2, axis=1)))),
                euler_rate_error_rms_rad_s=float(np.sqrt(np.mean(np.sum(settled[:, 3:] ** 2, axis=1)))))
    angular_errors = np.linalg.norm(np.array([row['estimates']['recorded']['velocity'][3:] for row in rows]) - velocities[:, 3:], axis=1)
    indices = sorted({int(np.argmax(angular_errors)), int(np.argmax([row['peak_current_A'] for row in report['trace']]))})
    commands = []
    for index in indices:
        estimate = rows[index]['estimates']['recorded']
        original = simulation.controller.command(estimate, simulation.reference)
        matrix = simulation.controller.nominal_matrix.copy()
        cases = {}
        with patch.object(levitation_sim, 'actuator_matrix', return_value=matrix):
            for name, velocity in (('recorded', estimate['velocity']), ('recorded_motion_velocity', velocities[index]),
                                   ('zero_velocity', np.zeros_like(velocities[index]))):
                command = original if name == 'recorded' else simulation.controller.command(dict(estimate, velocity=velocity), simulation.reference)
                currents = command['currents']
                cases[name] = dict(accepted=command['accepted'], required_peak_A=command['required_current_A'],
                    command_peak_A=None if currents is None else float(np.max(np.abs(currents))),
                    wrench=None if currents is None else (matrix @ currents).tolist())
        commands.append(dict(time_s=times[index], estimated_velocity=estimate['velocity'],
                             recorded_motion_velocity=velocities[index].tolist(), cases=cases))
    result = dict(scope='Offline feedback isolation on a fixed recorded flight; not corrected closed-loop flight',
        baseline_sha256=hashlib.sha256(report_bytes).hexdigest(), checkpoint_sha256=hashlib.sha256(checkpoint_bytes).hexdigest(),
        source_sha256=dict(simulation.source, **source_fingerprints(__file__)),
        exact_pose_replay=True, exact_noise_replay=True, duration_s=report['duration_completed_s'],
        velocity_errors=metrics, command_counterfactuals=commands, trace=rows,
        limitations=['Truth-assisted counterfactuals are diagnostics only; the flight controller remains sensor-only.',
            'All cases use identical recorded nominal acceleration inputs and the same acquired initial observer state.',
            'Analog Hall readout removes post-release noise and ADC quantization only; calibration errors, coil correction and fit uncertainty weights remain unchanged.',
            'Velocity scoring differentiates the saved pose trace, not full-rate MuJoCo velocities; Euler rates are used consistently.',
            'Exact coil subtraction uses recorded physical coil fields and within-step RL reconstruction; it is never supplied to the production controller.',
            'Exact position measurements interpolate recorded poses at scan midpoints and preserve packet delay.',
            'Selected current commands hold the estimated pose and allocation matrix fixed; they are not achieved currents or new hardware targets.'])
    output.write_text(json.dumps(result, indent=Fixed.report_precision, allow_nan=False) + '\n')
    print(json.dumps(metrics), flush=True)
    return result


def numerical_diagnosis(baseline, output):
    saved = baseline.read_bytes()
    report = json.loads(saved)
    construction = magnetics.ConstructionModel()
    center = construction.piece_properties['center_mm'][2]
    result = dict(scope='Identical-ADC pose-fit sensitivity and fixed-state magnetic refinement; not closed-loop qualification',
                  source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, sensing.__file__, airborne.__file__)),
                  baseline_sha256=hashlib.sha256(saved).hexdigest(), fits={}, fixed_states=[])

    def default_derivative(*args, **kwargs):
        kwargs.pop('jac')
        return least_squares(*args, **kwargs)

    for name, fit in (('automatic_derivative', default_derivative), ('physical_centered_derivative', least_squares)):
        rows = []
        for height in (center, np.nextafter(center, np.inf)):
            construction.piece_properties['center_mm'][2] = height
            packets = []
            original = sensing.VirtualHallArray.sample_static

            def record(sensor, *args, **kwargs):
                frame = original(sensor, *args, **kwargs)
                packets.append(frame.adc_codes)
                return frame

            with patch.object(sensing, 'least_squares', fit), patch.object(sensing.VirtualHallArray, 'sample_static', record):
                simulation = airborne.AirborneSimulation(construction, Fixed.airborne_check_duration, disturbed=True)
            estimate = simulation.tracker.predict(0)
            rows.append(dict(center_mm=height, pose=estimate['pose'], velocity=estimate['velocity'],
                             adc_sha256=hashlib.sha256(np.array(packets).tobytes()).hexdigest()))
        differences = np.abs(np.array(rows[0]['pose']) - rows[1]['pose'])
        result['fits'][name] = dict(cases=rows, identical_adc=rows[0]['adc_sha256'] == rows[1]['adc_sha256'],
            max_position_difference_m=float(differences[:3].max()), max_angle_difference_rad=float(differences[3:].max()))
        print(json.dumps(result['fits'][name]), flush=True)
    construction.piece_properties['center_mm'][2] = center
    arrays, fields = {}, {}
    for refinement in Fixed.current_demand_mesh_refinements:
        geometry, arrays[refinement], _ = construction(Inputs.cad_coil_layout, refinement)
        fields[refinement] = sensing.HallFieldModel(construction, Fixed.hall_conductor_refinement * refinement)
    levitation_sim.use_geometry(geometry)
    magnets = levitation_sim.magnet_layout_from_geometry()
    trace = report['trace']
    indices = sorted({int(np.argmax([row['peak_current_A'] for row in trace])),
                      int(np.argmax(Rotation.from_euler('xyz', [row['pose'][3:] for row in trace]).magnitude()))})
    for index in indices:
        sample = trace[index]
        values = []
        for refinement, array in arrays.items():
            _, origin = levitation_sim.place_piece(magnets, *sample['pose'])
            matrix = levitation_sim.actuator_matrix(magnets, array, origin)
            selection = [fields[refinement].identities.index(identity) for identity in array.identities]
            values.append(dict(refinement=refinement, wrench=(matrix @ sample['currents_A']).tolist(),
                coil_hall_T=(fields[refinement].coil_field[:, selection] @ sample['currents_A']).tolist()))
        first, last = values[0], values[-1]
        change = np.array(last['wrench']) - first['wrench']
        result['fixed_states'].append(dict(time_s=sample['time_s'], values=values,
            relative_force_change=float(np.linalg.norm(change[:3]) / np.linalg.norm(first['wrench'][:3])),
            relative_torque_change=float(np.linalg.norm(change[3:]) / np.linalg.norm(first['wrench'][3:])),
            max_coil_hall_change_T=float(np.max(np.abs(np.array(last['coil_hall_T']) - first['coil_hall_T'])))))
    output.write_text(json.dumps(result, indent=Fixed.report_precision, allow_nan=False) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Diagnose airborne numerical and sensing sensitivity using saved inputs and targeted coupled trials.')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--numerics', action='store_true', help='Diagnose pose-fit sensitivity and magnetic refinement at identical states.')
    mode.add_argument('--feedback', action='store_true', help='Isolate feedback errors using recorded sensors and diagnostic truth substitutions.')
    mode.add_argument('--observer', action='store_true', help='Compare observer settings and independent Hall noise on recorded motion.')
    mode.add_argument('--sensor-resolution', action='store_true', help='Capture the first moving sensor scan under time refinement.')
    mode.add_argument('--count-response', action='store_true', help='Isolate a recorded one-count Hall change without rerunning physical motion.')
    mode.add_argument('--count-feedback', action='store_true', help='Run identical short coupled trials with one observed Hall count changed.')
    mode.add_argument('--precision', action='store_true', help='Compare ADC precision on saved analog fields with unchanged physical noise and scan timing.')
    mode.add_argument('--recorded-resolution', action='store_true', help='Attribute a refinement disagreement using each report and its adjacent checkpoint.')
    mode.add_argument('--refine-hover', action='store_true', help='Tighten adaptation from the recorded release against retained fine hover evidence.')
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--comparison', type=Path, help='Time-refined report for sensor capture, or early magnetic capture for the single-count feedback probe.')
    parser.add_argument('--capture', type=Path, help='Existing coarse sensor capture to compare with a recorded magnetic-refinement checkpoint.')
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--atlas', type=Path, help='Qualified atlas required to replay an adaptive atlas recording.')
    parser.add_argument('--fixed-reference', action='store_true', help='With --refine-hover, halve the recorded fixed reference step instead of adaptive tolerances.')
    parser.add_argument('--output', type=Path)
    arguments = parser.parse_args()
    if arguments.fixed_reference and not arguments.refine_hover:
        parser.error('--fixed-reference requires --refine-hover')
    if arguments.atlas is not None and not (arguments.recorded_resolution or arguments.refine_hover or arguments.count_feedback):
        parser.error('--atlas requires --recorded-resolution, --refine-hover or --count-feedback')
    if not (arguments.sensor_resolution or arguments.count_response or arguments.count_feedback or arguments.precision or arguments.recorded_resolution or arguments.refine_hover) and (arguments.comparison is not None or arguments.capture is not None):
        parser.error('--comparison and --capture require a sensor/count diagnostic')
    results = Path(__file__).with_name('results')
    if arguments.refine_hover:
        if any(value is None for value in (arguments.baseline, arguments.comparison, arguments.checkpoint, arguments.atlas, arguments.output)) or arguments.capture is not None:
            parser.error('--refine-hover requires --baseline, --comparison, --checkpoint (release), --atlas and --output')
        refine_recorded_hover(arguments.baseline, arguments.comparison, arguments.checkpoint, arguments.atlas, arguments.output, arguments.fixed_reference)
    elif arguments.recorded_resolution:
        if any(value is None for value in (arguments.baseline, arguments.comparison, arguments.output)) or arguments.capture is not None or arguments.checkpoint is not None:
            parser.error('--recorded-resolution requires --baseline, --comparison and --output; checkpoints are adjacent to reports')
        recorded_resolution_diagnosis(arguments.baseline, arguments.comparison, arguments.output, arguments.atlas)
    elif arguments.precision:
        if any(value is None for value in (arguments.capture, arguments.baseline, arguments.output)) or arguments.comparison is not None or arguments.checkpoint is not None:
            parser.error('--precision requires --capture, --baseline (recordings folder) and --output, without --comparison or --checkpoint')
        precision_diagnosis(arguments.capture, arguments.baseline, arguments.output)
    elif arguments.count_feedback and arguments.atlas is not None:
        if any(value is None for value in (arguments.capture, arguments.checkpoint, arguments.output)) or arguments.baseline is not None or arguments.comparison is not None:
            parser.error('Atlas --count-feedback requires --capture (recorded diagnosis), --checkpoint (release) and --output')
        recorded_count_feedback(arguments.capture, arguments.checkpoint, arguments.atlas, arguments.output)
    elif arguments.count_feedback:
        if any(value is None for value in (arguments.capture, arguments.comparison, arguments.output)) or arguments.baseline is not None or arguments.checkpoint is not None:
            parser.error('--count-feedback requires --capture, --comparison and --output, without --baseline or --checkpoint')
        count_feedback_diagnosis(arguments.capture, arguments.comparison, arguments.output)
    elif arguments.count_response:
        if any(value is None for value in (arguments.capture, arguments.baseline, arguments.checkpoint, arguments.output)) or arguments.comparison is not None:
            parser.error('--count-response requires --capture, --baseline, --checkpoint and --output, without --comparison')
        count_response_diagnosis(arguments.capture, arguments.baseline, arguments.checkpoint, arguments.output)
    elif arguments.sensor_resolution:
        if arguments.output is None:
            parser.error('--sensor-resolution requires an explicit --output')
        if arguments.capture is not None:
            if arguments.checkpoint is None or arguments.baseline is None or arguments.comparison is not None:
                parser.error('--capture requires its magnetic --baseline report and --checkpoint, without --comparison')
            magnetic_sensor_diagnosis(arguments.capture, arguments.baseline, arguments.checkpoint, arguments.output)
            raise SystemExit
        if arguments.checkpoint is not None:
            parser.error('A sensor checkpoint replay requires --capture')
        if arguments.comparison is not None and arguments.baseline is None:
            parser.error('--comparison requires --baseline')
        sensor_resolution_diagnosis(arguments.baseline, arguments.output, arguments.comparison)
    elif arguments.feedback or arguments.observer:
        if arguments.baseline is None or arguments.checkpoint is None:
            parser.error('Sensor replay requires an explicit --baseline report and matching --checkpoint')
        (observer_diagnosis if arguments.observer else feedback_diagnosis)(arguments.baseline, arguments.checkpoint,
            arguments.output if arguments.output is not None else results / ('airborne_observer.json' if arguments.observer else 'airborne_feedback.json'))
    else:
        if arguments.checkpoint is not None:
            parser.error('--checkpoint requires --feedback or --observer')
        baseline = arguments.baseline if arguments.baseline is not None else results / ('airborne_verification/baseline.json' if arguments.numerics else 'airborne_failure.json')
        output = arguments.output if arguments.output is not None else results / ('airborne_numerics.json' if arguments.numerics else 'airborne_diagnosis.json')
        (numerical_diagnosis if arguments.numerics else replay)(baseline, output)

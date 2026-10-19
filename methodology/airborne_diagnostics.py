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
    if checkpoint['schema'] != 'airborne-coupled-v5' and atlas_path is None:
        raise ValueError('Recorded fixed-step diagnostics require the fixed-step checkpoint schema; adaptive replay needs its recorded event clock')
    if report['source_sha256'] != checkpoint['source_sha256'] or report['trace'] != checkpoint['trace']:
        raise ValueError('Report and recorded sensor history do not describe the same flight')
    construction = airborne_verification.recorded_construction(report)
    if atlas_path is None:
        simulation = airborne.AirborneSimulation(construction, report['intended_duration_s'],
            disturbed=report['disturbed'], time_refinement=report['time_refinement'],
            magnetic_refinement=report['magnetic_refinement'], motion_scenario=checkpoint['motion_scenario'])
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


def recorded_dense_trajectory(path, component):
    from scipy.integrate import OdeSolution
    from scipy.integrate._ivp.rk import RkDenseOutput
    with np.load(path, allow_pickle=False) as saved:
        starts, ends, values, coefficients = [saved[component + '_' + key] for key in ('t_old', 't', 'y_old', 'Q')]
    if (not len(starts) or np.any(ends <= starts) or not np.array_equal(starts[1:], ends[:-1])
            or not all(np.isfinite(value).all() for value in (starts, ends, values, coefficients))):
        raise ValueError('Recorded dense trajectory has invalid intervals or values')
    return OdeSolution(np.r_[starts[0], ends], [RkDenseOutput(*row) for row in zip(starts, ends, values, coefficients)])


def filtered_recorded_scan(trajectory, start, channels, order, evaluate=None):
    from numpy.polynomial.legendre import leggauss
    samples = Fixed.hall_samples_per_sensor
    times = start + Fixed.hall_power_settle_time + np.arange(channels * samples) / Fixed.hall_adc_sample_rate
    if start < trajectory.t_min or times[-1] > trajectory.t_max:
        raise ValueError('Recorded filter query extends beyond the saved trajectory')
    boundaries = np.unique(np.r_[start, times, trajectory.ts[(trajectory.ts > start) & (trajectory.ts < times[-1])]])
    nodes, weights = leggauss(order)
    rate = 2 * np.pi * Fixed.hall_sensor_bandwidth
    halves = np.diff(boundaries) / 2
    queries = boundaries[:-1, None] + (nodes[None, :] + 1) * halves[:, None]
    values = (trajectory if evaluate is None else evaluate)(queries.ravel()).reshape(channels, len(halves), order)
    state, result = np.zeros(channels), np.empty(len(times))
    for interval, (left, right) in enumerate(zip(boundaries[:-1], boundaries[1:])):
        state = state * np.exp(-rate * (right - left)) + values[:, interval] @ (weights * rate * halves[interval] * np.exp(-rate * (right - queries[interval])))
        index = np.searchsorted(times, right)
        if index < len(times) and times[index] == right:
            result[index] = state[index // samples]
    return result.reshape(channels, samples)


def matched_sensor_metrics(actual, references, field_difference):
    reference_error = float(np.max(np.abs(references[0] - references[-1])))
    filter_error = float(np.max(np.abs(actual - references[-1])))
    limit = Fixed.atlas_field_tolerance
    reference_limit = limit * Fixed.coupling_relative_tolerance
    bound = filter_error + reference_error + field_difference
    return dict(filter_error_T=filter_error, quadrature_disagreement_T=reference_error,
        field_implementation_difference_T=field_difference, combined_error_T=bound,
        limit_T=limit, reference_limit_T=reference_limit,
        passed=bool(bound <= limit and reference_error <= reference_limit and field_difference <= reference_limit))


class MatchedHallReference:
    def __init__(self, fields, calibration, identities, center_height, current):
        from methodology.benchmark_magnetics import hall_evaluators
        self.points = fields.points
        self.evaluators = hall_evaluators(deepcopy(fields.magnets))
        self.gain, self.offset = (np.array(calibration[key]) for key in ('gain', 'offset'))
        indices = [fields.identities.index(identity) for identity in identities]
        self.coil_field = np.array(calibration['actual_coil_field'])[:, indices]
        self.center_height, self.current = center_height, current

    def check(self, trajectory, scan, include_samples=False):
        def poses(states):
            return np.c_[states[:, :3] - [0, 0, self.center_height],
                Rotation.from_quat(states[:, 3:7], scalar_first=True).as_euler('xyz')]

        def signal(times):
            result = []
            for first in range(0, len(times), Fixed.hall_field_batch_frames):
                states = trajectory(times[first:first + Fixed.hall_field_batch_frames]).T
                pose = poses(states)
                magnet = self.evaluators['functional'](np.tile(self.points, (len(states), 1)),
                    np.repeat(pose, len(self.points), axis=0)).reshape(len(states), len(self.points)).T
                result.append(self.gain[:, None] * (magnet + self.coil_field @ states[:, self.current].T) + self.offset[:, None])
            return np.concatenate(result, axis=1)

        start = scan['start_s']
        actual = np.array(scan['samples_T'])
        end = start + Fixed.hall_power_settle_time + (actual.size - 1) / Fixed.hall_adc_sample_rate
        differences = [float(np.max(np.abs(self.gain * (self.evaluators['current'](self.points, pose)
            - self.evaluators['functional'](self.points, pose))))) for pose in poses(trajectory(np.array([start, end])).T)]
        references = [filtered_recorded_scan(trajectory, start, len(self.points), order, signal)
            for order in Fixed.coupling_flux_orders[-2:]]
        result = dict(start_s=start, **matched_sensor_metrics(actual, references, max(differences)))
        if include_samples:
            result['reference_samples_T'] = references[-1].tolist()
        return result


def verify_recorded_archive(folder):
    from zipfile import ZipFile
    batch = json.loads((folder / 'summary.json').read_text())
    with ZipFile(folder / 'sources.zip') as archive:
        for name, digest in batch['source_sha256'].items():
            if hashlib.sha256(archive.read(name)).hexdigest() != digest:
                raise ValueError('Recorded source archive changed: ' + name)
    return batch


def verify_recorded_batch(folder):
    batch = verify_recorded_archive(folder)
    for name, digest in batch['source_sha256'].items():
        if Path(name).resolve() != Path(__file__).resolve() and hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError('Physical replay source changed: ' + name)
    return batch



def recorded_filter_command(trajectory, time, selection):
    index = np.searchsorted(trajectory.ts, time, side='right') - 1
    if index < 0 or index >= len(trajectory.interpolants):
        raise ValueError('Command replay requires a recorded interval after the event')
    polynomial = trajectory.interpolants[index]
    if abs(polynomial.t_old - time) > Fixed.adaptive_clock_tolerance:
        raise ValueError('Command replay event does not start a saved integration interval')
    return polynomial.y_old[selection] + polynomial.Q[selection, 0] / (2 * np.pi * Fixed.setpoint_filter_cutoff)



def same_native_runtime(first, second):
    return ({key: value for key, value in first.items() if key != 'blas_libraries'}
            == {key: value for key, value in second.items() if key != 'blas_libraries'}
            and sorted(json.dumps(value, sort_keys=True) for value in first['blas_libraries'])
            == sorted(json.dumps(value, sort_keys=True) for value in second['blas_libraries']))


def replay_recorded_controller(folder, native_folder, cache_folder, output):
    from contextlib import ExitStack, nullcontext
    from zipfile import ZipFile, ZIP_DEFLATED
    from methodology import coil_field, coil_field_verification, magnetic_atlas, magnetic_spline

    if output.exists():
        raise ValueError('Preserve existing controller replay evidence')
    batch = verify_recorded_batch(folder)
    names = tuple(batch['cases'])
    saved = {name: {key: json.loads((folder / name / filename).read_text()) for key, filename in
        (('initial', 'initial_snapshot.json'), ('final', 'snapshot.json'), ('evidence', 'evidence.json'))} for name in names}
    initial = saved[names[0]]['initial']
    construction = magnetics.ConstructionModel(recorded_piece_properties=initial['construction']['piece_properties'])
    construction.use_cad_mass = initial['construction']['use_cad_mass']
    construction.return_depth = initial['construction']['return_depth']
    library = native_folder / 'native/magnetic_spline.so'
    cache = coil_field_verification.DiagnosticCoilCache(cache_folder, library, batch['candidate']['order'], True)
    atlas = magnetic_atlas.MagneticFieldAtlas.load(folder / 'atlas.npz')
    if cache.identity != batch['candidate'] or atlas.identity != batch['atlas_identity']:
        raise ValueError('Controller replay magnetic identity differs')
    inputs = [folder / 'summary.json', folder / 'sources.zip', folder / 'atlas.npz', cache_folder / 'summary.json', library]
    for name in names:
        inputs.extend(folder / name / filename for filename in ('initial_snapshot.json', 'snapshot.json', 'evidence.json', 'dense.npz'))
    sources = dict(batch['source_sha256'], **source_fingerprints(__file__))
    result = dict(status='running', scope='Controller and observer replay on saved inputs; no physical integration',
        source_sha256=sources, input_sha256=source_fingerprints(*inputs), cases={}, comparisons={},
        limitations=['Recorded acceleration histories are replayed and independently checked, not recomputed into a new trajectory.',
            'Common-backend command comparisons hold the estimated state fixed; they do not predict a counterfactual flight.',
            'No numerical, control or operating acceptance limit is changed.'])
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(name, name)
    started = perf_counter()
    with ExitStack() as resources:
        runtime = resources.enter_context(magnetic_spline.native_evaluation(atlas, library))
        if not same_native_runtime(runtime, batch['runtime']):
            raise ValueError('Controller replay native runtime differs')
        cache.evaluate = resources.enter_context(magnetic_spline.spline_workers(cache.native, Fixed.native_spline_workers))
        fields = sensing.HallFieldModel(construction, magnetic_refinement=Fixed.hall_conductor_refinement * initial['magnetic_refinement'])
        _, array, _ = construction(Inputs.cad_coil_layout, initial['magnetic_refinement'])
        identities = tuple(array.identities)
        drivers = electrical.CoilDrivers(construction, identities)
        controller = airborne.AirborneController(construction, identities, drivers.current_limit)
        controller.field_evaluator = atlas.field
        controller.array.force_quadrature = controller.array.create_force_quadrature()
        queries = coil_field.CoilSurfaceQueries(controller.array, min(Fixed.coupling_flux_orders))
        filtered = slice(13 + len(identities), 13 + 2 * len(identities))
        origin = np.zeros(6)
        origin[2] = Fixed.airborne_reference_gap_fraction * (controller.geometry.max_flight_gap - controller.geometry.gap)
        result['coil_identities'] = identities

        def candidate_matrix(quadrature, source, center, field_evaluator=None):
            if quadrature is not controller.array.force_quadrature:
                raise ValueError('Unexpected force quadrature in controller-only replay')
            return queries.matrix(source, center, cache)

        for name in names:
            state, final, evidence = (saved[name][key] for key in ('initial', 'final', 'evidence'))
            if (state['construction'] != initial['construction'] or state['source_sha256'] != batch['source_sha256']
                    or final['source_sha256'] != batch['source_sha256'] or state['motion_scenario'] != 'local_move'):
                raise ValueError('Controller replay construction, source or scenario differs')
            path = folder / name / 'dense.npz'
            if hashlib.sha256(path.read_bytes()).hexdigest() != evidence['dense_sha256']:
                raise ValueError('Controller replay dense evidence changed')
            trajectory = recorded_dense_trajectory(path, 'physical')
            sensors = sensing.VirtualHallArray(fields)
            for key in ('gain', 'offset', 'actual_coil_field'):
                setattr(sensors, key, np.array(state['sensors'][key]))
            sensors.random.bit_generator.state = deepcopy(state['sensors']['random'])
            tracker = sensing.HallTracker(sensing.HallPoseEstimator(fields))
            for key in airborne.TRACKER_ARRAY_STATE:
                setattr(tracker, key, np.array(state['tracker'][key]))
            for key in airborne.TRACKER_CLOCK_STATE:
                setattr(tracker, key, state['tracker'][key])
            tracker.acceleration_times = final['acceleration_times']
            tracker.accelerations = [np.array(value) for value in final['accelerations']]
            for key in airborne.DRIVER_STATE:
                setattr(drivers, key, np.array(state['drivers'][key]))
            readouts = [electrical.CurrentReadings(identities, np.array(row['sample_times_s']), np.array(row['adc_codes']),
                row['available_time_s']) for row in final['readouts']]
            reads = {row.sample_times_s[0]: row for row in readouts}
            traces = {row['time_s']: row for row in final['trace']}
            accelerations = dict(zip(final['acceleration_times'], final['accelerations']))
            commands, packets, regulators = [], [], []
            errors = dict(pose=0., command_A=0., acceleration=0.)
            next_scan = 0
            cached = state['evaluator']['force_method'] == 'coil_cache'
            for now in final['field_times']:
                if next_scan < len(evidence['scans']):
                    scan = evidence['scans'][next_scan]
                    if now >= sensing.hall_available_time(scan['start_s'], len(fields.points)):
                        frame = sensors.encode_signal(np.array(scan['samples_T']), scan['start_s'])
                        available = [reading for reading in readouts if reading.available_time_s <= frame.available_time_s]
                        history = electrical.CurrentReadings(identities, np.concatenate([row.sample_times_s for row in available]),
                            np.vstack([row.adc_codes for row in available]), available[-1].available_time_s)
                        consumed = tracker.consume(frame, history, now)
                        if not consumed['valid']:
                            raise ValueError('Recorded controller replay lost tracking')
                        packets.append(dict(time_s=now, start_s=scan['start_s'], adc_codes=frame.adc_codes.tolist(),
                            position=tracker.position.tolist(), velocity=tracker.velocity.tolist(), disturbance=tracker.disturbance.tolist()))
                        next_scan += 1
                estimate = tracker.predict(now)
                if not estimate['valid']:
                    raise ValueError('Recorded controller prediction unavailable')
                if now in traces:
                    errors['pose'] = max(errors['pose'], float(np.max(np.abs(np.array(estimate['pose']) - traces[now]['estimated_pose']))))
                    reference = airborne.smooth_reference(origin, Fixed.airborne_motion_translation, Fixed.airborne_motion_rotation,
                        now - Fixed.airborne_motion_start, Fixed.airborne_motion_duration)
                    with patch.object(levitation_sim.CoilForceQuadrature, 'matrix', candidate_matrix) if cached else nullcontext():
                        allocation = controller.command(estimate, *reference)
                    if not allocation['accepted']:
                        raise ValueError('Recorded controller allocation rejected')
                    nominal = controller.nominal_matrix.copy()
                    drivers.set_command(allocation['currents'])
                    inferred = recorded_filter_command(trajectory, now, filtered)
                    errors['command_A'] = max(errors['command_A'], float(np.max(np.abs(inferred - drivers.command))))
                    common = controller.command(estimate, *reference) if cached else allocation
                    if not common['accepted']:
                        raise ValueError('Common-backend diagnostic allocation rejected')
                    controller.nominal_matrix = nominal
                    commands.append(dict(time_s=now, estimate=estimate, command_A=allocation['currents'].tolist(),
                        quantized_command_A=drivers.command.tolist(), common_backend_command_A=common['currents'].tolist(),
                        target_wrench=(nominal @ allocation['currents']).tolist()))
                if now in reads:
                    drivers.filtered_command = trajectory(now)[filtered].copy()
                    drivers.regulate(reads[now], now)
                    acceleration = controller.measured_acceleration(estimate, reads[now], now)
                    errors['acceleration'] = max(errors['acceleration'], float(np.max(np.abs(acceleration - accelerations[now]))))
                    regulators.append(dict(time_s=now, adc_codes=reads[now].adc_codes.tolist(),
                        filtered_command_A=drivers.filtered_command.tolist(), voltage_V=drivers.voltage.tolist(),
                        requested_voltage_V=drivers.requested_voltage.tolist()))
            errors['tracker'] = max(float(np.max(np.abs(getattr(tracker, key) - final['tracker'][key]))) for key in airborne.TRACKER_ARRAY_STATE)
            errors['driver'] = max(float(np.max(np.abs(getattr(drivers, key) - final['drivers'][key])))
                for key in ('command', 'integral', 'voltage', 'requested_voltage'))
            passed = (all(value <= Fixed.motion_verification_tolerance for value in errors.values())
                and sensors.random.bit_generator.state == final['sensors']['random']
                and next_scan == len(evidence['scans'])
                and all(getattr(tracker, key) == final['tracker'][key] for key in airborne.TRACKER_CLOCK_STATE))
            result['cases'][name] = dict(reproduced=passed, replay_errors=errors, commands=commands, packets=packets, regulators=regulators)
            result['elapsed_s'] = perf_counter() - started
            airborne_verification.save_summary(output / 'summary.json', result)
            if not passed:
                result['status'] = 'replay_mismatch'
                airborne_verification.save_summary(output / 'summary.json', result)
                return result
        for left, right in ((names[0], names[1]), (names[0], names[2]), (names[1], names[2])):
            first, second = result['cases'][left], result['cases'][right]
            rows = []
            for a, b in zip(first['commands'], second['commands']):
                if a['time_s'] != b['time_s']:
                    raise ValueError('Recorded command clocks differ')
                rows.append(dict(time_s=a['time_s'], **{key: float(np.max(np.abs(np.array(b[key]) - a[key])))
                    for key in ('command_A', 'quantized_command_A', 'common_backend_command_A')},
                    velocity_difference=(np.array(b['estimate']['velocity']) - a['estimate']['velocity']).tolist(),
                    target_wrench_difference=(np.array(b['target_wrench']) - a['target_wrench']).tolist()))
            counts = {}
            for kind in ('packets', 'regulators'):
                counts[kind] = [dict(time_s=a['time_s'], changed_counts=int(np.count_nonzero(np.array(b['adc_codes']) - a['adc_codes'])),
                    maximum_count_difference=int(np.max(np.abs(np.array(b['adc_codes']) - a['adc_codes']))))
                    for a, b in zip(first[kind], second[kind])]
            result['comparisons'][left + '_vs_' + right] = dict(commands=rows, sensor_counts=counts,
                maximum_command_difference_A=max(row['command_A'] for row in rows),
                maximum_quantized_command_difference_A=max(row['quantized_command_A'] for row in rows),
                maximum_common_backend_difference_A=max(row['common_backend_command_A'] for row in rows))
    result.update(status='reproduced', elapsed_s=perf_counter() - started)
    airborne_verification.save_summary(output / 'summary.json', result)
    return result



def recorded_controller_response(folder, replay_folder, native_folder, output):
    from zipfile import ZipFile, ZIP_DEFLATED
    from methodology import magnetic_atlas, magnetic_spline
    if output.exists():
        raise ValueError('Preserve existing controller response evidence')
    batch = verify_recorded_batch(folder)
    replay = verify_recorded_batch(replay_folder)
    if replay['status'] != 'reproduced':
        raise ValueError('Controller intervention requires a reproduced original replay')
    for name, digest in replay['input_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError('Controller replay input changed: ' + name)
    names = ('magnet_atlas', 'coil_cache_tighter')
    saved = {name: json.loads((folder / name / 'snapshot.json').read_text()) for name in names}
    initial = json.loads((folder / names[0] / 'initial_snapshot.json').read_text())
    comparison = replay['comparisons']['_vs_'.join(names)]
    rows = comparison['commands']
    jump = max(range(1, len(rows)), key=lambda index: rows[index]['command_A'] - rows[index - 1]['command_A'])
    packets = comparison['sensor_counts']['packets']
    selected = sorted({next(index for index, row in enumerate(packets) if row['changed_counts']),
        max(index for index, row in enumerate(packets) if row['time_s'] <= rows[jump]['time_s'])})
    construction = magnetics.ConstructionModel(recorded_piece_properties=initial['construction']['piece_properties'])
    construction.use_cad_mass = initial['construction']['use_cad_mass']
    construction.return_depth = initial['construction']['return_depth']
    atlas = magnetic_atlas.MagneticFieldAtlas.load(folder / 'atlas.npz')
    result = dict(scope='One-packet input substitutions at saved observer states; no physical trajectory or control tuning',
        source_sha256=dict(replay['source_sha256'], **source_fingerprints(__file__)),
        input_sha256=source_fingerprints(folder / 'summary.json', replay_folder / 'summary.json', replay_folder / 'sources.zip'),
        cases=[], limitations=['Each substitution retains the selected recorded acceleration history and prior observer state.',
            'Hall and current signals change together in real flight; swapping only one is a causal diagnostic, not a realizable sensor mode.',
            'This identifies a local command response, not long-term disturbance recovery or a passed movement.'])
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in result['source_sha256']:
            archive.write(name, name)
    started = perf_counter()
    with magnetic_spline.native_evaluation(atlas, native_folder / 'native/magnetic_spline.so') as runtime:
        if not same_native_runtime(runtime, batch['runtime']):
            raise ValueError('Controller response runtime differs')
        fields = sensing.HallFieldModel(construction, magnetic_refinement=Fixed.hall_conductor_refinement * initial['magnetic_refinement'])
        identities = tuple(tuple(value) for value in replay['coil_identities'])
        drivers = electrical.CoilDrivers(construction, identities)
        controller = airborne.AirborneController(construction, identities, drivers.current_limit)
        controller.field_evaluator = atlas.field
        origin = np.zeros(6)
        origin[2] = Fixed.airborne_reference_gap_fraction * (controller.geometry.max_flight_gap - controller.geometry.gap)
        for index in selected:
            frames, histories, trackers, originals = [], [], [], []
            command_time = next(row['time_s'] for row in rows if row['time_s'] >= packets[index]['time_s'])
            for name in names:
                case = replay['cases'][name]
                packet = case['packets'][index]
                codes = np.array(packet['adc_codes'])
                times = packet['start_s'] + Fixed.hall_power_settle_time + np.arange(codes.size).reshape(codes.shape) / Fixed.hall_adc_sample_rate
                frame = sensing.HallFrame(codes, times, sensing.hall_available_time(packet['start_s'], len(fields.points)))
                frames.append(frame)
                readouts = [row for row in saved[name]['readouts'] if row['available_time_s'] <= frame.available_time_s]
                histories.append(electrical.CurrentReadings(identities, np.concatenate([row['sample_times_s'] for row in readouts]),
                    np.vstack([row['adc_codes'] for row in readouts]), readouts[-1]['available_time_s']))
                tracker = sensing.HallTracker(sensing.HallPoseEstimator(fields))
                previous = case['packets'][index - 1] if index else initial['tracker']
                for key in airborne.TRACKER_ARRAY_STATE:
                    setattr(tracker, key, np.array(previous[key]))
                tracker.last_delivery = previous['time_s'] if index else initial['tracker']['last_delivery']
                tracker.measurement_time = (float((previous['start_s'] + Fixed.hall_power_settle_time
                    + np.arange(codes.size) / Fixed.hall_adc_sample_rate).mean()) if index else initial['tracker']['measurement_time'])
                tracker.acquisition_start = initial['tracker']['acquisition_start']
                tracker.acceleration_times = saved[name]['acceleration_times']
                tracker.accelerations = [np.array(value) for value in saved[name]['accelerations']]
                trackers.append(tracker)
                originals.append(next(row for row in case['commands'] if row['time_s'] == command_time))
            reference = airborne.smooth_reference(origin, Fixed.airborne_motion_translation, Fixed.airborne_motion_rotation,
                command_time - Fixed.airborne_motion_start, Fixed.airborne_motion_duration)
            responses = {}
            for label, state_index, hall_index, current_index in (
                    ('baseline', 0, 0, 0), ('tighter', 1, 1, 1), ('baseline_with_tighter_hall', 0, 1, 0),
                    ('baseline_with_tighter_currents', 0, 0, 1), ('baseline_with_both_tighter_readings', 0, 1, 1)):
                tracker = deepcopy(trackers[state_index])
                consumed = tracker.consume(frames[hall_index], histories[current_index], packets[index]['time_s'])
                estimate = tracker.predict(command_time)
                allocation = controller.command(estimate, *reference)
                if not consumed['valid'] or not allocation['accepted']:
                    raise ValueError('Recorded sensor intervention lost tracking or allocation')
                if label in ('baseline', 'tighter') and (estimate != originals[state_index]['estimate'] or not np.allclose(
                        allocation['currents'], originals[state_index]['common_backend_command_A'], rtol=0, atol=Fixed.motion_verification_tolerance)):
                    raise ValueError('Controller response did not reproduce the saved estimate/command')
                drivers.set_command(allocation['currents'])
                responses[label] = dict(estimate=estimate, command_A=allocation['currents'].tolist(),
                    quantized_command_A=drivers.command.tolist(), fitted_pose=consumed['estimate']['pose'])
            for key in ('pose', 'velocity'):
                estimate = deepcopy(responses['baseline']['estimate'])
                estimate[key] = responses['tighter']['estimate'][key]
                allocation = controller.command(estimate, *reference)
                if not allocation['accepted']:
                    raise ValueError('State-component intervention allocation rejected')
                responses['baseline_with_tighter_' + key] = dict(estimate=estimate, command_A=allocation['currents'].tolist())
            for response in responses.values():
                response['maximum_command_change_A'] = float(np.max(np.abs(np.array(response['command_A']) - responses['baseline']['command_A'])))
            decoded = [(frame.adc_codes + 0.5) * sensing.hall_readout()[2] - Fixed.hall_supply_voltage / 2 / Fixed.hall_sensitivity for frame in frames]
            correction = [sensing.measured_coil_fields(fields, history, frame)[0] for history, frame in zip(histories, frames)]
            result['cases'].append(dict(scan_start_s=replay['cases'][names[0]]['packets'][index]['start_s'],
                delivery_time_s=packets[index]['time_s'], command_time_s=command_time, responses=responses,
                maximum_decoded_hall_difference_T=float(np.max(np.abs(decoded[1].mean(axis=1) - decoded[0].mean(axis=1)))),
                maximum_coil_subtraction_difference_T=float(np.max(np.abs(correction[1] - correction[0]))),
                maximum_corrected_hall_difference_T=float(np.max(np.abs(decoded[1].mean(axis=1) - decoded[0].mean(axis=1) - correction[1] + correction[0])))))
    result.update(status='reproduced', elapsed_s=perf_counter() - started)
    airborne_verification.save_summary(output / 'summary.json', result)
    return result


def verify_recorded_sensing(folder, output, worst_only=False):
    from threadpoolctl import threadpool_limits
    from zipfile import ZipFile, ZIP_DEFLATED
    from methodology import benchmark_magnetics
    if output.exists():
        raise ValueError('Preserve existing matched-input sensing evidence')
    batch = verify_recorded_batch(folder)
    selected_start = None
    if worst_only:
        from itertools import combinations
        scans = {name: json.loads((folder / name / 'evidence.json').read_text())['scans'] for name in batch['cases']}
        differences = []
        for left, right in combinations(scans, 2):
            if [row['start_s'] for row in scans[left]] != [row['start_s'] for row in scans[right]]:
                raise ValueError('Worst-scan attribution requires identical saved scan clocks')
            differences.extend((float(np.max(np.abs(np.array(a['samples_T']) - b['samples_T']))), a['start_s'])
                for a, b in zip(scans[left], scans[right]))
        selected_start = max(differences)[1]
    first = next(iter(batch['cases']))
    initial = json.loads((folder / first / 'initial_snapshot.json').read_text())
    construction = magnetics.ConstructionModel(recorded_piece_properties=initial['construction']['piece_properties'])
    construction.use_cad_mass = initial['construction']['use_cad_mass']
    construction.return_depth = initial['construction']['return_depth']
    geometry, array, _ = construction(Inputs.cad_coil_layout, initial['magnetic_refinement'])
    fields = sensing.HallFieldModel(construction, magnetic_refinement=Fixed.hall_conductor_refinement * initial['magnetic_refinement'])
    current = slice(13, 13 + len(array.identities))
    inputs = [folder / 'summary.json', folder / 'sources.zip']
    for name in batch['cases']:
        inputs.extend(folder / name / filename for filename in ('initial_snapshot.json', 'snapshot.json', 'evidence.json', 'dense.npz'))
    report = dict(status='running', scope='Independent Hall convolution on each recorded physical trajectory; no closed-loop rerun or flight qualification',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, sensing.__file__, benchmark_magnetics.__file__)),
        input_sha256=source_fingerprints(*inputs), cases={}, selected_scan_start_s=selected_start)
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in report['source_sha256']:
            archive.write(name, name)
    started = perf_counter()
    with threadpool_limits(limits=Fixed.native_spline_blas_threads, user_api='blas'):
        for name in batch['cases']:
            state = json.loads((folder / name / 'initial_snapshot.json').read_text())
            evidence = json.loads((folder / name / 'evidence.json').read_text())
            if state['construction'] != initial['construction'] or state['source_sha256'] != batch['source_sha256']:
                raise ValueError('Recorded sensing construction or source identity differs')
            path = folder / name / 'dense.npz'
            if hashlib.sha256(path.read_bytes()).hexdigest() != evidence['dense_sha256']:
                raise ValueError('Recorded trajectory hash differs')
            reference = MatchedHallReference(fields, state['sensors'], array.identities, geometry.center_of_mass_height, current)
            trajectory = recorded_dense_trajectory(path, 'physical')
            report['cases'][name] = []
            for scan in evidence['scans']:
                if worst_only and scan['start_s'] != selected_start:
                    continue
                result = reference.check(trajectory, scan, include_samples=worst_only)
                report['cases'][name].append(result)
                report['elapsed_s'] = perf_counter() - started
                if not result['passed']:
                    report['status'] = 'matched_input_sensing_failed'
                airborne_verification.save_summary(output / 'summary.json', report)
                if not result['passed']:
                    return report
    if worst_only:
        report['comparisons'] = {}
        for left, right in combinations(scans, 2):
            a, b = (np.array(next(row['samples_T'] for row in scans[name] if row['start_s'] == selected_start))
                for name in (left, right))
            reference_a, reference_b = (np.array(report['cases'][name][0]['reference_samples_T']) for name in (left, right))
            actual = b - a
            physical = reference_b - reference_a
            filtering = (b - reference_b) - (a - reference_a)
            worst = np.unravel_index(np.argmax(np.abs(actual)), actual.shape)
            closure = float(np.max(np.abs(actual - physical - filtering)))
            reference_bound = sum(report['cases'][name][0]['quadrature_disagreement_T']
                + report['cases'][name][0]['field_implementation_difference_T'] for name in (left, right))
            if closure > Fixed.atlas_field_tolerance * Fixed.coupling_relative_tolerance:
                raise ValueError('Recorded Hall error decomposition does not close')
            report['comparisons'][left + '_vs_' + right] = dict(maximum_raw_difference_T=float(np.max(np.abs(actual))),
                maximum_physical_history_difference_T=float(np.max(np.abs(physical))),
                maximum_filter_error_difference_T=float(np.max(np.abs(filtering))),
                reference_error_bound_T=reference_bound, closure_error_T=closure,
                worst_raw_sample=dict(channel=int(worst[0]), sample=int(worst[1]),
                    total_T=float(actual[worst]), physical_history_T=float(physical[worst]),
                    filtering_T=float(filtering[worst]), reference_filter_error_T=float((a - reference_a)[worst]),
                    candidate_filter_error_T=float((b - reference_b)[worst])))
        report['scope'] = 'Worst saved Hall scan: independent physical-history versus filtering-error attribution; original failed gate unchanged'
    report['status'] = 'passed'
    airborne_verification.save_summary(output / 'summary.json', report)
    return report


def attribute_recorded_coil_signal(folder, output):
    from scipy.integrate import OdeSolution
    from scipy.integrate._ivp.rk import RkDenseOutput
    from threadpoolctl import threadpool_limits
    from zipfile import ZipFile, ZIP_DEFLATED
    from methodology import numerical_accuracy, joint_airborne

    if output.exists():
        raise ValueError('Preserve existing recorded coil-signal attribution')
    batch = verify_recorded_batch(folder)
    names = ('magnet_atlas', 'coil_cache', 'coil_cache_tighter')
    saved = {name: {key: json.loads((folder / name / filename).read_text()) for key, filename in
        (('initial', 'initial_snapshot.json'), ('final', 'snapshot.json'), ('report', 'summary.json'), ('evidence', 'evidence.json'))}
        for name in names}
    original = saved[names[0]]['initial']
    construction = magnetics.ConstructionModel(recorded_piece_properties=original['construction']['piece_properties'])
    construction.use_cad_mass = original['construction']['use_cad_mass']
    construction.return_depth = original['construction']['return_depth']
    _, array, _ = construction(Inputs.cad_coil_layout, original['magnetic_refinement'])
    active = tuple(array.identities)
    array.window = Fixed.cad_patch_cells_per_side * array.short_pitch
    array.build()
    all_coils = tuple(array.identities)
    fields = sensing.HallFieldModel(construction,
        magnetic_refinement=Fixed.hall_conductor_refinement * original['magnetic_refinement'])
    if fields.identities != all_coils:
        raise ValueError('Nominal Hall map ordering differs from recorded coils')
    indices = [all_coils.index(identity) for identity in active]
    inputs = [folder / 'summary.json', folder / 'sources.zip']
    for name in names:
        inputs.extend(folder / name / filename for filename in ('initial_snapshot.json', 'snapshot.json', 'summary.json', 'evidence.json', 'dense.npz'))
    result = dict(status='running', scope='Recorded coil-field/filter attribution only; no dynamics rerun, counterfactual controller intervention or flight qualification',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, sensing.__file__, numerical_accuracy.__file__, joint_airborne.__file__)),
        input_sha256=source_fingerprints(*inputs), original_batch_status=batch['status'],
        limits=dict(raw_signal_T=Fixed.atlas_field_tolerance, independent_reference_T=Fixed.atlas_field_tolerance * Fixed.coupling_relative_tolerance),
        active_coil_identities=active, hall_map_coil_identities=all_coils, quadrature_orders=Fixed.coupling_flux_orders[-2:],
        cases={}, comparisons={}, limitations=[
            'The remainder contains real magnet motion plus numerical filtering/replay error; it is not a pure numerical-error estimate.',
            'Coil-field separation uses each saved calibrated map and current polynomial, without altering either trajectory or using truth in its controller.',
            'Exact-current interpolation counterfactuals use truth only for error attribution; the controller still receives measured current codes.',
            'Exponential reconstruction uses nominal L/R; varying back-EMF and within-sample switching/extinction are not exactly exponential.',
            'Common injected noise cancels in differences between runs; the corresponding noise/quantization components still include quantization effects.',
            'The original failed gate remains failed and its acceptance limits are unchanged.'])
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in result['source_sha256']:
            archive.write(name, name)
    started = perf_counter()
    with threadpool_limits(limits=Fixed.native_spline_blas_threads, user_api='blas'):
        for name, case in saved.items():
            initial, final, report, evidence = (case[key] for key in ('initial', 'final', 'report', 'evidence'))
            if (initial['sensors'] != original['sensors'] or initial['construction'] != original['construction']
                    or final['trace'] != report['trace'] or final['source_sha256'] != batch['source_sha256']
                    or final['sensors']['actual_coil_field'] != initial['sensors']['actual_coil_field']):
                raise ValueError('Recorded signal attribution requires matching source, calibration, construction and traces')
            dense_path = folder / name / 'dense.npz'
            if hashlib.sha256(dense_path.read_bytes()).hexdigest() != evidence['dense_sha256']:
                raise ValueError('Recorded dense trajectory changed')
            physical = recorded_dense_trajectory(dense_path, 'physical')
            filtered = recorded_dense_trajectory(dense_path, 'filtered')
            count = len(initial['drivers']['currents'])
            if count != len(active):
                raise ValueError('Recorded current vector differs from the active coil ordering')
            current = slice(13, 13 + count)
            gain = np.array(initial['sensors']['gain'])
            matrix = np.array(initial['sensors']['actual_coil_field'])
            if matrix.shape != (len(gain), len(all_coils)):
                raise ValueError('Recorded Hall map differs from the reconstructed coil ordering')
            calibrated = gain[:, None] * matrix[:, indices]
            nominal = fields.coil_field[:, indices]
            coil = OdeSolution(physical.ts, [RkDenseOutput(piece.t_old, piece.t,
                calibrated @ piece.y_old[current], calibrated @ piece.Q[current]) for piece in physical.interpolants])
            nominal_coil = OdeSolution(physical.ts, [RkDenseOutput(piece.t_old, piece.t,
                nominal @ piece.y_old[current], nominal @ piece.Q[current]) for piece in physical.interpolants])
            readouts = final['readouts']
            readout_times = np.concatenate([row['sample_times_s'] for row in readouts])
            exact_currents = physical(readout_times)[current].T
            calibrated_currents = exact_currents * final['current_sensors']['gain'] + final['current_sensors']['offset']

            sensor = sensing.VirtualHallArray.__new__(sensing.VirtualHallArray)
            sensor.gain = gain
            sensor.random = np.random.default_rng()
            sensor.random.bit_generator.state = deepcopy(initial['sensors']['random'])
            rows = []
            for scan in evidence['scans']:
                start = scan['start_s']
                actual = np.array(scan['samples_T'])
                frame = sensor.encode_signal(actual, start)
                times = frame.sample_times_s
                channels = np.repeat(np.arange(len(gain)), Fixed.hall_samples_per_sensor)
                reconstructed = (filtered(times.ravel())[channels, np.arange(times.size)]
                    - filtered(start)[channels] * np.exp(-2 * np.pi * Fixed.hall_sensor_bandwidth * (times.ravel() - start))).reshape(times.shape)
                replay_error = float(np.max(np.abs(reconstructed - actual)))
                estimates = [filtered_recorded_scan(coil, start, len(gain), order) for order in result['quadrature_orders']]
                nominal_estimates = [filtered_recorded_scan(nominal_coil, start, len(gain), order)
                    for order in result['quadrature_orders']]
                disagreement = max(float(np.max(np.abs(pair[0] - pair[-1]))) for pair in (estimates, nominal_estimates))
                available = readout_times <= frame.available_time_s
                history = electrical.CurrentReadings(active, readout_times[available],
                    np.vstack([row['adc_codes'] for row in readouts])[available], readout_times[available][-1])
                measured = sensing.measured_coil_fields(fields, history, frame)[0]
                linear = sensing.filtered_scan(readout_times[available], exact_currents[available] @ nominal.T, start).mean(axis=1)
                calibrated_exponential = sensing.filtered_scan(readout_times[available], calibrated_currents[available] @ nominal.T,
                    start, fields.coil_time_constant).mean(axis=1)
                exponential = sensing.filtered_scan(readout_times[available], exact_currents[available] @ nominal.T,
                    start, fields.coil_time_constant).mean(axis=1)
                measured_linear = sensing.filtered_scan(history.sample_times_s, history.decode(frame.available_time_s) @ nominal.T, start).mean(axis=1)
                decoded = ((frame.adc_codes + 0.5) * sensing.hall_readout()[2]
                    - Fixed.hall_supply_voltage / 2 / Fixed.hall_sensitivity).mean(axis=1)
                components = dict(magnet_motion_and_filter=actual.mean(axis=1) - estimates[-1].mean(axis=1),
                    coil_calibration=estimates[-1].mean(axis=1) - nominal_estimates[-1].mean(axis=1),
                    current_reconstruction=nominal_estimates[-1].mean(axis=1) - exponential,
                    current_calibration=exponential - calibrated_exponential,
                    current_noise_and_quantization=calibrated_exponential - measured,
                    hall_noise_and_quantization=decoded - actual.mean(axis=1))
                corrected = decoded - measured
                decomposition_error = float(np.max(np.abs(sum(components.values()) - corrected)))
                if decomposition_error > result['limits']['independent_reference_T']:
                    raise ValueError('Corrected Hall attribution does not close')
                rows.append(dict(start_s=start, adc_codes=frame.adc_codes.tolist(), available_time_s=frame.available_time_s,
                    dense_replay_error_T=replay_error, coil_reference_disagreement_T=disagreement,
                    filtered_coil_T=estimates[-1].tolist(), corrected_signal_T=corrected.tolist(),
                    corrected_signal_components_T={key: value.tolist() for key, value in components.items()},
                    decomposition_error_T=decomposition_error,
                    linear_reconstruction_error_T=(nominal_estimates[-1].mean(axis=1) - linear).tolist(),
                    corrected_linear_signal_T=(decoded - measured_linear).tolist(), reference_passed=max(replay_error, disagreement) <= result['limits']['independent_reference_T']))
            random_matches = sensor.random.bit_generator.state == final['sensors']['random']
            result['cases'][name] = dict(scans=rows, sensor_random_state_reproduced=random_matches)
            if not random_matches or not rows or not all(row['reference_passed'] for row in rows):
                result['status'] = 'recorded_reference_failed'
                airborne_verification.save_summary(output / 'summary.json', result)
                return result
        for first, second in ((names[0], names[1]), (names[0], names[2]), (names[1], names[2])):
            rows = []
            first_scans, second_scans = saved[first]['evidence']['scans'], saved[second]['evidence']['scans']
            if [row['start_s'] for row in first_scans] != [row['start_s'] for row in second_scans]:
                raise ValueError('Recorded comparison requires identical scan clocks')
            for actual_first, actual_second, coil_first, coil_second in zip(first_scans, second_scans,
                    result['cases'][first]['scans'], result['cases'][second]['scans']):
                total = np.array(actual_second['samples_T']) - actual_first['samples_T']
                coil_difference = np.array(coil_second['filtered_coil_T']) - coil_first['filtered_coil_T']
                remaining = total - coil_difference
                maximum = np.unravel_index(np.argmax(np.abs(total)), total.shape)
                reference_bound = sum(row['coil_reference_disagreement_T'] for row in (coil_first, coil_second))
                rows.append(dict(start_s=actual_first['start_s'], maximum_raw_difference_T=float(np.max(np.abs(total))),
                    maximum_coil_difference_T=float(np.max(np.abs(coil_difference))),
                    maximum_remaining_difference_T=float(np.max(np.abs(remaining))),
                    reference_disagreement_bound_T=reference_bound,
                    remaining_with_reference_bound_T=float(np.max(np.abs(remaining))) + reference_bound,
                    within_original_signal_limit=bool(np.max(np.abs(remaining)) + reference_bound <= result['limits']['raw_signal_T']),
                    worst_raw_sample=dict(channel=int(maximum[0]), sample=int(maximum[1]),
                        total_T=float(total[maximum]), coil_T=float(coil_difference[maximum]), remaining_T=float(remaining[maximum])),
                    remaining_difference_T=remaining.tolist(),
                    corrected_signal_difference_T=(np.array(coil_second['corrected_signal_T']) - coil_first['corrected_signal_T']).tolist(),
                    corrected_signal_component_differences_T={key: (np.array(coil_second['corrected_signal_components_T'][key])
                        - coil_first['corrected_signal_components_T'][key]).tolist() for key in coil_first['corrected_signal_components_T']},
                    corrected_linear_difference_T=(np.array(coil_second['corrected_linear_signal_T'])
                        - coil_first['corrected_linear_signal_T']).tolist()))
            result['comparisons'][first + '_vs_' + second] = rows
    result.update(status='diagnosed', elapsed_s=perf_counter() - started)
    airborne_verification.save_summary(output / 'summary.json', result)
    return result


def recorded_physical_solution(recorder):
    from scipy.integrate import OdeSolution
    pieces = [piece for segment in recorder.segments for piece in segment.sol.physical.interpolants]
    return OdeSolution(np.r_[pieces[0].t_old, [piece.t for piece in pieces]], pieces)


def compare_drive_trajectory(reference, report, simulation):
    from methodology.airborne_inputs import physical_comparison
    rows = []
    for row in report['trace']:
        if row['time_s'] < reference.ts[0] or row['time_s'] > reference.ts[-1] + Fixed.adaptive_clock_tolerance:
            raise ValueError('Matched drive comparison cannot extrapolate recorded physics')
        state = reference(row['time_s'])
        pose = np.r_[state[:3] - [0, 0, simulation.plant.geometry.center_of_mass_height],
            Rotation.from_quat(state[3:7], scalar_first=True).as_euler('xyz')]
        rows.append(dict(time_s=row['time_s'], pose=pose.tolist(), currents_A=state[simulation.dynamics.current].tolist()))
    return physical_comparison(dict(report, trace=rows), report)


def record_joint_drive(simulation, process_events, commands, regulators, now, regulator_due, command_due, complete):
    process_events(now, regulator_due, command_due, complete)
    if simulation.stop_reason is not None or complete:
        return
    if command_due:
        commands.append(dict(time_s=now, quantized_command_A=simulation.drivers.command.tolist()))
    if regulator_due:
        regulators.append(dict(time_s=now, voltage_V=simulation.drivers.voltage.tolist(),
            requested_voltage_V=simulation.drivers.requested_voltage.tolist()))


def compare_controller_allocation(simulation, command, reference_matrix, checks, *args):
    candidate = command(*args)
    if not candidate['accepted']:
        return candidate
    nominal = simulation.controller.nominal_matrix.copy()
    with patch.object(levitation_sim.CoilForceQuadrature, 'matrix', reference_matrix):
        reference = command(*args)
    simulation.controller.nominal_matrix = nominal
    limit = Fixed.driver_channel_current * Fixed.qualification_current_reserve_fraction * Fixed.qualification_convergence_fraction
    difference = float(np.max(np.abs(reference['currents'] - candidate['currents']))) if reference['accepted'] else None
    passed = reference['accepted'] and difference <= limit
    checks.append(dict(time_s=simulation.clock, passed=passed, reference_accepted=reference['accepted'],
        command_difference_A=difference, limit_A=limit))
    if not passed:
        simulation.stop_reason = 'matched_controller_allocation_failed'
    return candidate


def check_recorded_sensor_scans(recorder, reference, checks):
    if len(checks) == len(recorder.scans):
        return True
    trajectory = recorded_physical_solution(recorder)
    for scan in recorder.scans[len(checks):]:
        result = reference.check(trajectory, scan)
        checks.append(result)
        if not result['passed']:
            return False
    return True


def advance_comparison_checkpoint(simulation):
    while not simulation.finished:
        simulation.advance()
        if simulation.checkpoint_due():
            break



def replay_joint_drive(folder, native_folder, cache_folder, output):
    from contextlib import ExitStack
    from zipfile import ZipFile, ZIP_DEFLATED
    from methodology import airborne_inputs, coil_field, coil_field_verification, magnetic_atlas, magnetic_spline, numerical_accuracy
    if output.exists():
        raise ValueError('Preserve existing joint drive replay evidence')
    batch = verify_recorded_batch(folder)
    replay_folder = folder / 'controller_replay'
    replay = verify_recorded_batch(replay_folder)
    if replay['status'] != 'reproduced':
        raise ValueError('Joint drive replay requires reproduced controller outputs')
    for name, digest in replay['input_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError('Recorded drive replay input changed: ' + name)
    role = 'coil_cache'
    initial = json.loads((folder / role / 'initial_snapshot.json').read_text())
    final = json.loads((folder / role / 'snapshot.json').read_text())
    evidence = json.loads((folder / role / 'evidence.json').read_text())
    construction = magnetics.ConstructionModel(recorded_piece_properties=initial['construction']['piece_properties'])
    construction.use_cad_mass = initial['construction']['use_cad_mass']
    construction.return_depth = initial['construction']['return_depth']
    atlas = magnetic_atlas.MagneticFieldAtlas.load(folder / 'atlas.npz')
    library = native_folder / 'native/magnetic_spline.so'
    cache = coil_field_verification.DiagnosticCoilCache(cache_folder, library, batch['candidate']['order'], True)
    if cache.identity != batch['candidate'] or atlas.identity != batch['atlas_identity']:
        raise ValueError('Joint drive replay magnetic identity differs')
    reference = recorded_dense_trajectory(folder / role / 'dense.npz', 'physical')
    signals = {scan['start_s']: np.array(scan['samples_T']) for scan in evidence['scans']}
    result = dict(status='running', scope='Tighter joint physical integration with identical recorded drive signals; no closed-loop qualification',
        source_sha256=dict(batch['source_sha256'], **source_fingerprints(__file__, airborne_inputs.__file__)),
        input_sha256=source_fingerprints(folder / 'summary.json', replay_folder / 'summary.json', replay_folder / 'sources.zip'),
        comparisons=[], limitations=['This isolates physics integration from changed digitized sensor decisions.',
            'The original closed-loop failure and its limits remain unchanged.'])
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in result['source_sha256']:
            archive.write(name, name)
    started = perf_counter()
    with ExitStack() as resources:
        runtime = resources.enter_context(magnetic_spline.native_evaluation(atlas, library))
        if not same_native_runtime(runtime, batch['runtime']):
            raise ValueError('Joint drive replay runtime differs')
        cache.evaluate = resources.enter_context(magnetic_spline.spline_workers(cache.native, Fixed.native_spline_workers))
        simulation = airborne_inputs.SharedJointInputSimulation(construction, final['joint']['clock'], initialize=False,
            tolerance_scale=1 / Fixed.qualification_refinement_factor, atlas=atlas, magnetic_mode='atlas_forces',
            motion_scenario=initial['motion_scenario'])
        simulation.configure_recording(initial, replay['cases'][role]['commands'], replay['cases'][role]['regulators'])
        simulation.source.update(result['source_sha256'])
        simulation.plant.array.force_quadrature = simulation.plant.array.create_force_quadrature()
        queries = coil_field.CoilSurfaceQueries(simulation.plant.array, min(Fixed.coupling_flux_orders))
        def candidate_matrix(quadrature, source, center, field_evaluator=None):
            if quadrature is not simulation.plant.array.force_quadrature:
                raise ValueError('Joint drive replay encountered another force quadrature')
            return queries.matrix(source, center, cache)
        recorder = numerical_accuracy.JointTrajectoryRecorder(simulation)
        resources.enter_context(patch.object(levitation_sim.CoilForceQuadrature, 'matrix', candidate_matrix))
        resources.enter_context(patch.object(simulation.dynamics, 'integrate', recorder.integrate))
        resources.enter_context(patch.object(simulation.sensors, 'encode_signal', recorder.encode_signal))
        sensor_reference = MatchedHallReference(simulation.fields, initial['sensors'], simulation.identities,
            simulation.plant.geometry.center_of_mass_height, simulation.dynamics.current)
        checks = []
        simulation.save_checkpoint(output / 'initial_snapshot.json')
        while not simulation.finished:
            advance_comparison_checkpoint(simulation)
            report = simulation.report()
            comparison = compare_drive_trajectory(reference, report, simulation)
            raw_difference = max((float(np.max(np.abs(np.array(scan['samples_T']) - signals[scan['start_s']])))
                for scan in recorder.scans), default=0.)
            comparison.update(time_s=simulation.clock, raw_signal_difference_T=raw_difference)
            comparison['checks']['raw_signal'] = raw_difference <= Fixed.atlas_field_tolerance
            comparison['checks']['matched_input_sensing'] = check_recorded_sensor_scans(recorder, sensor_reference, checks)
            comparison['converged'] = all(comparison['checks'].values())
            result['comparisons'].append(comparison)
            if not comparison['converged'] and simulation.stop_reason is None:
                simulation.stop_reason, simulation.finished = 'recorded_drive_refinement_failed', True
            simulation.save_checkpoint(output / 'snapshot.json')
            result.update(elapsed_s=perf_counter() - started, duration_completed_s=simulation.clock,
                stop_reason=simulation.stop_reason, integration=simulation.accounting, peaks=simulation.peaks,
                instruction_identity=simulation.instruction_identity, matched_input_sensing=checks,
                dense_sha256=recorder.save_dense(output / 'dense.npz'))
            airborne_verification.save_summary(output / 'summary.json', result)
        result['status'] = 'passed' if simulation.stop_reason is None and all(row['converged'] for row in result['comparisons']) else 'failed'
        airborne_verification.save_summary(output / 'summary.json', result)
    return result


def compare_coil_coupled(folder, cache_folder, verification_path, output, movement=None, matched_drive=False):
    from contextlib import ExitStack
    from zipfile import ZipFile, ZIP_DEFLATED
    from methodology import airborne_inputs, coil_field, coil_field_verification, hover, magnetic_atlas, magnetic_spline, numerical_accuracy
    from methodology.benchmark_magnetics import recorded_force_geometry, save_spline_diagnostic
    from methodology.joint_airborne import JointAirborneSimulation

    if output.exists():
        raise ValueError('Preserve existing coupled coil-field evidence')
    gate = json.loads(verification_path.read_text())
    if gate['status'] != 'passed':
        raise ValueError('Coupled coil comparison requires passing derivative and work verification')
    for name, digest in dict(gate['source_sha256'], **gate['input_sha256']).items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError('Coil coupling verification source or input changed: ' + name)
    library = folder / 'native/magnetic_spline.so'
    cache = coil_field_verification.DiagnosticCoilCache(cache_folder, library, gate['candidate']['order'], True)
    if cache.identity != gate['candidate']:
        raise ValueError('Coil cache differs from its coupling verification')
    initial_path = folder / 'initial_checkpoint.json'
    recorded = json.loads(initial_path.read_text())
    geometry_path = next(Path(name) for name in cache.report['input_sha256'] if name.endswith('initial_checkpoint.json'))
    if recorded['construction'] != json.loads(geometry_path.read_text())['construction']:
        raise ValueError('Coupled trial construction differs from the verified coil geometry')
    duration = Fixed.airborne_check_duration
    scenario = 'hover'
    inputs = source_fingerprints(verification_path, cache_folder / 'summary.json', initial_path, folder / 'atlas.npz')
    if movement is not None:
        retained = json.loads((movement / 'summary.json').read_text())
        retained_initial = json.loads((movement / 'initial_checkpoint.json').read_text())
        duration = Fixed.airborne_motion_start + Fixed.airborne_motion_duration + Fixed.airborne_motion_settle_duration
        scenario = 'local_move'
        if (not retained['passed'] or retained['motion_scenario'] != scenario or not np.isclose(retained['duration_completed_s'], duration, rtol=0, atol=Fixed.adaptive_clock_tolerance)
                or retained_initial['construction'] != recorded['construction']):
            raise ValueError('Movement comparison requires the complete matching retained local movement')
        inputs.update(source_fingerprints(movement / 'summary.json', movement / 'initial_checkpoint.json'))
    construction = magnetics.ConstructionModel(recorded_piece_properties=recorded['construction']['piece_properties'])
    construction.use_cad_mass = recorded['construction']['use_cad_mass']
    construction.return_depth = recorded['construction']['return_depth']
    layout, array = recorded_force_geometry(recorded)
    canonical = coil_field.CoilSurfaceQueries(array, min(Fixed.coupling_flux_orders))
    levitation_sim.place_piece(layout)
    with np.load(folder / 'atlas.npz', allow_pickle=False) as saved:
        bounds = saved['bounds']
        reference_order = min(Fixed.atlas_spline_orders) if movement is None else int(saved['orders'][0])
        if movement is not None and not np.all(saved['orders'] == reference_order):
            raise ValueError('Movement reference requires the retained uniform spline order')
    output.mkdir(parents=True)
    started = perf_counter()
    atlas = magnetic_atlas.MagneticFieldAtlas(layout.collection, bounds,
        min(Fixed.atlas_grid_spacings), reference_order)
    atlas.save(output / 'atlas.npz')
    reference_build_s = perf_counter() - started
    sources = dict(gate['source_sha256'], **source_fingerprints(__file__, hover.__file__,
        airborne_verification.__file__, numerical_accuracy.__file__, airborne_inputs.__file__))
    with ExitStack() as resources:
        runtime = resources.enter_context(magnetic_spline.native_evaluation(atlas, library))
        cache.evaluate = resources.enter_context(magnetic_spline.spline_workers(cache.native, Fixed.native_spline_workers))
        simulation = JointAirborneSimulation(construction, duration, atlas=atlas, magnetic_mode='atlas_forces', motion_scenario=scenario)
        simulation.source.update(sources)
        initial = deepcopy(simulation.checkpoint_state())
        report = dict(scope='Bounded coupled coil-field comparison from one acquired state; not full flight-envelope qualification',
            motion_scenario=scenario, duration_s=duration, reference_order=reference_order,
            status='running', source_sha256=simulation.source, runtime=runtime, atlas_identity=atlas.identity,
            candidate=cache.identity, reference_build_s=reference_build_s,
            sensor_gate='Independent convolution on each physical history; cross-run raw differences are also gated for matched drive',
            input_sha256=inputs, cases={}, comparisons={})
        with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
            for name in simulation.source:
                archive.write(name, name)
        airborne_verification.save_summary(output / 'summary.json', report)
        if matched_drive:
            report.update(scope='Matched-drive numerical comparison plus one independent sensor-feedback movement; disturbance recovery remains unqualified',
                qualification_basis='Identical recorded commands and voltages; unchanged physical/current/sensing limits',
                feedback_robustness='pending', speed_comparison='Not comparable: reference replays omit controller computation')
        cases, queries = {}, {}
        commands, regulators, allocation_checks = [], [], []
        reference_matrix = levitation_sim.CoilForceQuadrature.matrix
        specifications = [('magnet_atlas', False, 1), ('coil_cache', True, 1)]
        if movement is not None:
            specifications.append(('coil_cache_tighter', True, 1 / Fixed.qualification_refinement_factor))
        if matched_drive:
            specifications = [('coil_cache', True, 1), ('magnet_atlas', False, 1),
                ('coil_cache_tighter', True, 1 / Fixed.qualification_refinement_factor)]
        for name, cached, scale in specifications:
            shared = matched_drive and name != 'coil_cache'
            if shared:
                simulation = airborne_inputs.SharedJointInputSimulation(construction, duration, initialize=False,
                    atlas=atlas, magnetic_mode='atlas_forces', motion_scenario=scenario)
                simulation.configure_recording(dict(deepcopy(initial), schema='spline-evaluator-diagnostic-v1'), [], [])
                simulation.source.update(sources)
            elif name != 'magnet_atlas':
                simulation = JointAirborneSimulation(construction, duration, initialize=False,
                    atlas=atlas, magnetic_mode='atlas_forces', motion_scenario=scenario)
                simulation.source.update(sources)
                simulation.restore_checkpoint(deepcopy(initial))
            simulation.tolerance_scale = scale
            if cached:
                for selected in (simulation.plant.array, simulation.controller.array):
                    selected.force_quadrature = selected.create_force_quadrature()
                    query = coil_field.CoilSurfaceQueries(selected, canonical.order)
                    if any(first.shape != second.shape or not np.allclose(first, second,
                            atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
                            for first, second in ((query.positions, canonical.positions), (query.elements, canonical.elements))):
                        raise ValueError('Coupled coil quadrature differs from the verified cache geometry')
                    queries[selected.force_quadrature] = query
            path = output / name
            path.mkdir()
            identity = dict(force_method='coil_cache' if cached else 'magnet_atlas', native_runtime=runtime,
                coil_cache=cache.identity if cached else None, tolerance_scale=scale,
                scope='Prescribed-drive physical replay; controller inactive' if shared else 'Diagnostic force-matrix override for plant and controller; direct sensing unchanged')
            save_spline_diagnostic(simulation, identity, path / 'initial_snapshot.json')
            cases[name] = dict(simulation=simulation, cached=cached, shared=shared, path=path, identity=identity,
                recorder=numerical_accuracy.JointTrajectoryRecorder(simulation), elapsed_s=0., simulation_elapsed_s=0.,
                verification_elapsed_s=0., sensor_checks=[], sensor_reference=MatchedHallReference(simulation.fields,
                    initial['sensors'], simulation.identities, simulation.plant.geometry.center_of_mass_height,
                    simulation.dynamics.current))

        def candidate_matrix(quadrature, source, center, field_evaluator=None):
            return queries[quadrature].matrix(source, center, cache)

        def compare(reference_name, candidate_name):
            reference, candidate = cases[reference_name], cases[candidate_name]
            if matched_drive:
                comparison = compare_drive_trajectory(recorded_physical_solution(reference['recorder']),
                    candidate['summary'], candidate['simulation'])
            else:
                comparison = airborne_verification.compare_trials(reference['summary'], candidate['summary'], trace_only=True)
            scans = reference['recorder'].scans, candidate['recorder'].scans
            same_times = [row['start_s'] for row in scans[0]] == [row['start_s'] for row in scans[1]]
            comparison['checks']['same_scan_times'] = same_times
            if same_times and scans[0]:
                difference = float(np.max(np.abs(np.array([row['samples_T'] for row in scans[0]])
                    - np.array([row['samples_T'] for row in scans[1]]))))
                comparison.update(max_analogue_difference_T=difference, analogue_limit_T=Fixed.atlas_field_tolerance)
                comparison['raw_difference_is_diagnostic'] = not matched_drive
                if matched_drive:
                    comparison['checks']['raw_signal'] = difference <= Fixed.atlas_field_tolerance
            comparison['checks']['matched_input_sensing'] = all(check['passed'] for case in (reference, candidate) for check in case['sensor_checks'])
            if reference['simulation'].finished and candidate['simulation'].finished:
                if matched_drive:
                    comparison['checks']['completed_without_stop'] = all(case['simulation'].stop_reason is None
                        and abs(case['simulation'].clock - duration) <= Fixed.adaptive_clock_tolerance for case in (reference, candidate))
                else:
                    comparison['checks'].update(airborne_verification.compare_trials(reference['summary'], candidate['summary'])['checks'])
                comparison['checks']['delivered_scans'] = bool(scans[0]) and bool(scans[1])
            comparison.update(converged=all(comparison['checks'].values()), compared_through_s=candidate['summary']['duration_completed_s'])
            return comparison

        while not all(case['simulation'].finished for case in cases.values()):
            for name, case in cases.items():
                simulation, recorder, path, identity = (case[key] for key in ('simulation', 'recorder', 'path', 'identity'))
                simulation.started = perf_counter() - case['elapsed_s']
                if case['shared']:
                    simulation.commands = {row['time_s']: np.array(row['quantized_command_A']) for row in commands}
                    simulation.instructions = {row['time_s']: row for row in regulators}
                    simulation.instruction_identity = hashlib.sha256(json.dumps(dict(commands=commands, regulators=regulators), sort_keys=True).encode()).hexdigest()
                with ExitStack() as trial:
                    if matched_drive and not case['shared']:
                        process_events = simulation.process_events
                        trial.enter_context(patch.object(simulation, 'process_events',
                            lambda *args: record_joint_drive(simulation, process_events, commands, regulators, *args)))
                    if case['cached']:
                        trial.enter_context(patch.object(levitation_sim.CoilForceQuadrature, 'matrix', candidate_matrix))
                    if matched_drive and not case['shared']:
                        command = simulation.controller.command
                        trial.enter_context(patch.object(simulation.controller, 'command',
                            lambda *args: compare_controller_allocation(simulation, command, reference_matrix, allocation_checks, *args)))
                    trial.enter_context(patch.object(simulation.dynamics, 'integrate', recorder.integrate))
                    trial.enter_context(patch.object(simulation.sensors, 'encode_signal', recorder.encode_signal))
                    advance_started = perf_counter()
                    advance_comparison_checkpoint(simulation)
                    case['simulation_elapsed_s'] += perf_counter() - advance_started
                verification_started = perf_counter()
                if simulation.stop_reason is None and not check_recorded_sensor_scans(recorder, case['sensor_reference'], case['sensor_checks']):
                    simulation.stop_reason = 'matched_input_sensing_failed'
                    simulation.finished = True
                case['verification_elapsed_s'] += perf_counter() - verification_started
                hover.check_operating_limits(simulation, simulation.report())
                case['summary'] = simulation.report()
                if (case['shared'] if matched_drive else case['cached']) and simulation.stop_reason is None:
                    references = (['coil_cache'] if name == 'magnet_atlas' else ['coil_cache', 'magnet_atlas']) if matched_drive else (['magnet_atlas'] if name == 'coil_cache' else ['magnet_atlas', 'coil_cache'])
                    for reference_name in references:
                        comparison = compare(reference_name, name)
                        report['comparisons'][reference_name + '_vs_' + name] = comparison
                        if not comparison['converged']:
                            simulation.stop_reason = 'numerical_comparison_failed'
                            simulation.finished = True
                            break
                save_spline_diagnostic(simulation, identity, path / 'snapshot.json')
                if matched_drive and not case['shared']:
                    airborne_verification.save_summary(output / 'drive.json', dict(commands=commands, regulators=regulators, allocation_checks=allocation_checks))
                airborne_verification.save_summary(path / 'evidence.json', dict(scans=recorder.scans, matched_input_sensing=case['sensor_checks'],
                    dense_sha256=recorder.save_dense(path / 'dense.npz')))
                case['summary'] = dict(simulation.report(), evaluator=identity)
                case['elapsed_s'] = case['summary']['elapsed_s']
                airborne_verification.save_summary(path / 'summary.json', case['summary'])
                report['cases'][name] = {key: case['summary'][key] for key in ('passed', 'elapsed_s', 'duration_completed_s',
                    'delivered_moving_scans', 'stop_reason', 'peaks', 'integration')}
                report['cases'][name].update(simulation_elapsed_s=case['simulation_elapsed_s'],
                    verification_elapsed_s=case['verification_elapsed_s'], checked_scans=len(case['sensor_checks']),
                    maximum_sensor_error_T=max((check['combined_error_T'] for check in case['sensor_checks']), default=None))
                if matched_drive:
                    report.update(allocation_checks=allocation_checks, drive_sha256=source_fingerprints(output / 'drive.json'))
                report.update(total_elapsed_s=perf_counter() - started,
                    status='running' if simulation.stop_reason is None else name + '_failed')
                airborne_verification.save_summary(output / 'summary.json', report)
                print(f"{name}: {simulation.clock:.6f} s, stop={simulation.stop_reason}", flush=True)
                if simulation.stop_reason is not None:
                    return report
        if matched_drive:
            passed = cases['coil_cache']['summary']['passed'] and all(row['converged'] for row in report['comparisons'].values())
            report.update(status='numerical_passed_feedback_pending' if passed else 'incomplete',
                numerical_passed=passed, closed_loop_movement_passed=cases['coil_cache']['summary']['passed'],
                drive_sha256=source_fingerprints(output / 'drive.json'), total_elapsed_s=perf_counter() - started)
        else:
            report.update(status='passed' if all(case['summary']['passed'] for case in cases.values()) else 'incomplete',
                coupled_speedup=cases['magnet_atlas']['simulation_elapsed_s'] / cases['coil_cache']['simulation_elapsed_s'],
                validated_run_speedup=cases['magnet_atlas']['elapsed_s'] / cases['coil_cache']['elapsed_s'],
                total_elapsed_s=perf_counter() - started)
        airborne_verification.save_summary(output / 'summary.json', report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Diagnose airborne numerical and sensing sensitivity using saved inputs and targeted coupled trials.')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--drive-replay', type=Path, help='Replay verified fixed drive signals with tighter physics integration.')
    mode.add_argument('--controller-response', type=Path, help='Isolate saved Hall/current inputs at the largest command change.')
    mode.add_argument('--controller-replay', type=Path, help='Replay saved controller decisions without physical integration.')
    mode.add_argument('--matched-sensing', type=Path, help='Check independent Hall convolution on saved matching physical inputs.')
    mode.add_argument('--coil-signal-attribution', type=Path, help='Separate coil-field changes using saved dense trajectories; no dynamics rerun.')
    mode.add_argument('--coil-cache', type=Path, help='Compare a verified coil cache against the native baseline; --comparison supplies its work/derivative report.')
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
    parser.add_argument('--movement', type=Path, help='With --coil-cache, compare the retained local-movement folder and tighter integration.')
    parser.add_argument('--worst-scan', action='store_true', help='With --matched-sensing, attribute the largest saved cross-run Hall discrepancy.')
    parser.add_argument('--matched-drive', action='store_true', help='With --coil-cache and --movement, compare physics on identical recorded drive signals.')
    parser.add_argument('--fixed-reference', action='store_true', help='With --refine-hover, halve the recorded fixed reference step instead of adaptive tolerances.')
    parser.add_argument('--output', type=Path)
    arguments = parser.parse_args()
    if arguments.movement is not None and arguments.coil_cache is None:
        parser.error('--movement requires --coil-cache')
    if arguments.worst_scan and arguments.matched_sensing is None:
        parser.error('--worst-scan requires --matched-sensing')
    if arguments.matched_drive and (arguments.coil_cache is None or arguments.movement is None):
        parser.error('--matched-drive requires --coil-cache and --movement')
    if arguments.fixed_reference and not arguments.refine_hover:
        parser.error('--fixed-reference requires --refine-hover')
    if arguments.atlas is not None and not (arguments.recorded_resolution or arguments.refine_hover or arguments.count_feedback):
        parser.error('--atlas requires --recorded-resolution, --refine-hover or --count-feedback')
    if not (arguments.drive_replay or arguments.controller_response or arguments.controller_replay or arguments.coil_cache or arguments.sensor_resolution or arguments.count_response or arguments.count_feedback or arguments.precision or arguments.recorded_resolution or arguments.refine_hover) and (arguments.comparison is not None or arguments.capture is not None):
        parser.error('--comparison and --capture require a sensor/count diagnostic')
    results = Path(__file__).with_name('results')
    if arguments.drive_replay:
        if any(value is None for value in (arguments.baseline, arguments.comparison, arguments.output)):
            parser.error('--drive-replay requires --baseline (native folder), --comparison (coil cache) and --output')
        replay_joint_drive(arguments.drive_replay, arguments.baseline, arguments.comparison, arguments.output)
    elif arguments.controller_response:
        if any(value is None for value in (arguments.baseline, arguments.comparison, arguments.output)):
            parser.error('--controller-response requires --baseline (native folder), --comparison (controller replay) and --output')
        recorded_controller_response(arguments.controller_response, arguments.comparison, arguments.baseline, arguments.output)
    elif arguments.controller_replay:
        if any(value is None for value in (arguments.baseline, arguments.comparison, arguments.output)):
            parser.error('--controller-replay requires --baseline (native folder), --comparison (coil cache) and --output')
        replay_recorded_controller(arguments.controller_replay, arguments.baseline, arguments.comparison, arguments.output)
    elif arguments.matched_sensing:
        if arguments.output is None or any(value is not None for value in (arguments.baseline, arguments.comparison, arguments.capture, arguments.checkpoint, arguments.atlas)):
            parser.error('--matched-sensing requires only its recordings folder and --output')
        verify_recorded_sensing(arguments.matched_sensing, arguments.output, arguments.worst_scan)
    elif arguments.coil_signal_attribution:
        if arguments.output is None or any(value is not None for value in (arguments.baseline, arguments.comparison, arguments.capture, arguments.checkpoint, arguments.atlas)):
            parser.error('--coil-signal-attribution requires only its recordings folder and --output')
        attribute_recorded_coil_signal(arguments.coil_signal_attribution, arguments.output)
    elif arguments.coil_cache:
        if any(value is None for value in (arguments.baseline, arguments.comparison, arguments.output)) or arguments.capture is not None or arguments.checkpoint is not None:
            parser.error('--coil-cache requires --baseline (native folder), --comparison (verification) and --output')
        compare_coil_coupled(arguments.baseline, arguments.coil_cache, arguments.comparison, arguments.output, arguments.movement, arguments.matched_drive)
    elif arguments.refine_hover:
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

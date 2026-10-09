import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import levitation_sim
import model
from methodology import design, source_fingerprints


def reference_state(case, time):
    pose = np.asarray(case['pose'], dtype=float).copy()
    velocity = np.asarray(case['velocity'], dtype=float)
    acceleration = np.asarray(case['acceleration'], dtype=float)
    pose[:3] += velocity * time + acceleration * time ** 2 / 2
    return pose, velocity + acceleration * time, acceleration


def stress_cases(board, piece, geometry):
    repeat = geometry.coil_lattice_repeat
    phase = geometry.coil_long / 2
    peak_phase = geometry.magnets_per_period * geometry.magnet_lateral_edge
    edge = np.array([board.motor_width, board.motor_height]) / 2000 - piece.diameter / 2000
    yaws = np.radians(model.Fixed.pose_yaws_deg)
    hot = dict(mass_factor=1 + model.Fixed.qualification_mass_fraction,
               com_factor=1 + model.Fixed.qualification_com_fraction,
               inertia_factor=1 - model.Fixed.qualification_inertia_fraction,
               inductance_factor=max(model.Fixed.qualification_inductance_factors),
               copper_temperature=model.Inputs.coil_bed_temp_limit,
               magnet_temperature=model.Fixed.max_touch_temperature, sensor_errors=True)
    nominal = dict(mass_factor=1, com_factor=1, inertia_factor=1, inductance_factor=1,
                   copper_temperature=model.Fixed.material_reference_temperature,
                   magnet_temperature=model.Fixed.material_reference_temperature, sensor_errors=False)
    cases = []

    def add(name, xy, gap, yaw, direction, stress, perturb):
        moving = bool(np.linalg.norm(direction) > 0)
        velocity = np.append(np.asarray(direction) * geometry.flight_speed, 0)
        duration = float(repeat / max(abs(velocity[:2]))) if moving else model.Fixed.qualification_recovery_duration
        cases.append(dict(name=name, pose=[*xy, gap - geometry.gap, 0, 0, yaw],
                          velocity=velocity.tolist(), acceleration=[0, 0, 0], duration=duration,
                          perturb=perturb, moving=moving, **stress))

    add('nominal_hover', [phase, phase], geometry.gap, min(yaws), [0, 0], nominal, True)
    add('hot_heavy_hover', [peak_phase, peak_phase], geometry.max_flight_gap, min(yaws), [0, 0], hot, True)
    add('hot_corner_hover', edge, geometry.max_flight_gap, yaws[len(yaws) // 2], [0, 0], hot, True)
    light = dict(hot, mass_factor=1 - model.Fixed.qualification_mass_fraction,
                 com_factor=1 - model.Fixed.qualification_com_fraction,
                 inertia_factor=1 + model.Fixed.qualification_inertia_fraction,
                 inductance_factor=min(model.Fixed.qualification_inductance_factors))
    add('light_low_inductance_hover', [phase, phase], geometry.gap, max(yaws), [0, 0], light, True)
    add('hot_handoff_x', [0, peak_phase], geometry.max_flight_gap, min(yaws), [1, 0], hot, False)
    add('hot_handoff_y', [peak_phase, 0], geometry.max_flight_gap, min(yaws), [0, 1], hot, False)
    add('hot_crossed_handoffs', [0, 0], geometry.max_flight_gap, yaws[len(yaws) // 2], np.ones(2) / np.sqrt(2), hot, False)
    add('hot_edge_flight', [-repeat / 2, edge[1]], geometry.max_flight_gap, max(yaws), [1, 0], hot, False)
    add('hot_accelerating_handoff', [0, peak_phase], geometry.max_flight_gap, min(yaws), [1, 0], hot, False)
    cases[-1]['acceleration'][0] = geometry.cruise_accels[0]
    cases[-1]['velocity'][0] -= cases[-1]['acceleration'][0] * cases[-1]['duration']
    return cases


def physical_track_array(case, coil, config, half_extents, refinement):
    array = levitation_sim.verification_coil_array(coil.control_cells_per_side, config.coil_height / 1000, refinement)
    windings = {}
    count = int(np.ceil(case['duration'] * model.Fixed.current_command_rate))
    for time in np.linspace(0, case['duration'], count + 1):
        pose, _, _ = reference_state(case, time)
        for center, weight in levitation_sim.blended_window_centers(pose[:2]):
            if weight > 0:
                array.build(center=center, half_extents=half_extents)
                windings.update(zip(array.identities, array.coils))
    array.identities = list(windings)
    array.coils = list(windings.values())
    return array


def bounded_sensor_error(time):
    phases = np.arange(6) * np.pi / 3
    bias = model.Fixed.qualification_noise_bias_fraction
    signal = bias + (1 - bias) * np.sin(2 * np.pi * model.Fixed.pose_controller_frequency * time + phases)
    return signal * np.repeat([model.Fixed.qualification_position_error,
                              model.Fixed.qualification_angle_error], 3) / np.sqrt(3)


def propagate_estimate(position, velocity, accelerations, period):
    accelerations = np.asarray(accelerations).reshape(-1, 6)
    duration = len(accelerations) * period
    remaining_times = (len(accelerations) - np.arange(len(accelerations)) - 0.5) * period
    return (position + velocity * duration + np.sum(accelerations * remaining_times[:, None], axis=0) * period,
            velocity + np.sum(accelerations, axis=0) * period)


def observer_update(position, velocity, disturbance, measurement, period, accelerations):
    pole = np.exp(-2 * np.pi * model.Fixed.pose_controller_frequency
                  * model.Fixed.pose_observer_frequency_ratio * period)
    predicted, predicted_velocity = propagate_estimate(position, velocity, np.asarray(accelerations) + disturbance,
                                                        period / len(accelerations))
    innovation = measurement - predicted
    return (predicted + (1 - pole ** 3) * innovation,
            predicted_velocity + 3 * (1 - pole) ** 2 * (1 + pole) / (2 * period) * innovation,
            disturbance + (1 - pole) ** 3 / period ** 2 * innovation)


def rl_step(current, voltage, back_emf, resistance, inductance, period, enabled=True):
    decay = np.exp(-resistance * period / inductance)
    next_current = current * decay + (voltage - back_emf) / resistance * (1 - decay)
    return np.where(~np.asarray(enabled) & (next_current * current <= 0), 0, next_current)


def current_regulator(error, integral, resistance, inductance, voltage_limit):
    frequency = 2 * np.pi * model.Fixed.current_loop_bandwidth
    requested = inductance * frequency * error + integral
    voltage = np.clip(requested, -voltage_limit, voltage_limit)
    integrate = (voltage == requested) | (error * requested < 0)
    integral = integral + integrate * resistance * frequency * error / model.Fixed.current_regulator_rate
    return voltage, integral, requested


def simulate_case(case, board, coil, piece, config, control, time_refinement=1, mesh_refinement=1, field_refinement=1):
    geometry = levitation_sim.G
    half_extents = np.array([board.motor_width, board.motor_height]) / 2000
    track = physical_track_array(case, coil, config, half_extents, mesh_refinement)
    indices = {identity: index for index, identity in enumerate(track.identities)}
    selector = levitation_sim.verification_coil_array(coil.control_cells_per_side, config.coil_height / 1000, mesh_refinement)
    nominal = levitation_sim.magnet_layout_from_geometry()
    actual = levitation_sim.magnet_layout_from_geometry()
    actual.center_of_mass_height *= case['com_factor']
    remanence_factor = 1 + model.Constants.ndfeb_br_tempco * (case['magnet_temperature'] - model.Fixed.material_reference_temperature)
    resistance = config.resistance * (1 + model.Constants.copper_resistivity_tempco
                                     * (case['copper_temperature'] - model.Fixed.material_reference_temperature))
    inductance = control.inductance * case['inductance_factor']
    nominal_mass = piece.mass / 1000
    mass = nominal_mass * case['mass_factor']
    body_inertia = geometry.inertias * case['inertia_factor']
    pose, velocity, _ = reference_state(case, 0)
    velocity = velocity.copy()
    initial_rotation = Rotation.from_euler('xyz', pose[3:])
    rotation = initial_rotation
    angular_velocity = np.zeros(3)
    position_budget = (model.Fixed.flight_position_error_gap_fraction if case['moving'] else model.Fixed.position_error_gap_fraction) * geometry.gap
    if case['perturb']:
        pose[:3] += model.Fixed.qualification_initial_error_fraction * position_budget / np.sqrt(3)
        rotation = Rotation.from_rotvec(np.ones(3) / np.sqrt(3) * model.Fixed.qualification_initial_error_fraction
                                       * model.Inputs.max_tip_position_error / (piece.box_height / 1000 - piece.com_height)) * rotation
    pose[3:] = rotation.as_euler('xyz')
    period = 1 / (model.Fixed.current_regulator_rate * model.Fixed.qualification_integration_substeps * time_refinement)
    intervals = [1 / model.Fixed.pose_feedback_rate, 1 / model.Fixed.current_command_rate,
                 1 / model.Fixed.current_regulator_rate, 1 / (model.Fixed.qualification_field_rate * field_refinement),
                 model.Fixed.qualification_sensor_latency]
    ticks = [int(round(interval / period)) for interval in intervals]
    if any(abs(tick * period - interval) > model.Fixed.allocator_tolerance for tick, interval in zip(ticks, intervals)):
        raise ValueError('Simulation clocks and sensor latency must be integer multiples of the integration period')
    pose_tick, command_tick, regulator_tick, field_tick, latency_tick = ticks
    estimate = np.zeros(6)
    estimated_velocity = np.zeros(6)
    estimated_disturbance = np.zeros(6)
    history = []
    command_history = []
    previous_sample_step = 0
    currents = np.zeros(len(indices))
    command = currents.copy()
    enabled = np.zeros(len(indices), dtype=bool)
    filtered_command = currents.copy()
    voltage = currents.copy()
    current_integral = currents.copy()
    sense_error = np.zeros(len(indices))
    if case['sensor_errors']:
        sense_error[:] = model.Fixed.current_sense_offset_residual / model.Fixed.current_sense_resistance
    metrics = dict(peak_tip_error_m=0.0, peak_position_error_m=0.0, peak_requested_current_A=0.0,
                   peak_actual_current_A=0.0, peak_voltage_V=0.0, peak_copper_loss_W=0.0,
                   peak_terminal_power_W=0.0, peak_midpoint_current_A=0.0, voltage_saturated_steps=0,
                   command_saturated_updates=0, minimum_rim_clearance_m=float('inf'),
                   component_minimum_peak_A=0.0)
    copper_energy = 0.0
    trace = []
    steps = int(np.ceil(case['duration'] / period))
    omega = 2 * np.pi * model.Fixed.pose_controller_frequency
    allocation_budget = ((1 - model.Fixed.qualification_current_reserve_fraction) * config.current_limit
                         - model.Fixed.current_sense_offset_residual / model.Fixed.current_sense_resistance
                         - model.Fixed.max_current_command_error - model.Fixed.qualification_current_transient_allowance)
    reference_drag = levitation_sim.conductive_sheet_response(geometry.gap + case['pose'][2], config.coil_height / 1000)
    actual_matrix = None
    for step in range(steps + 1):
        time = step * period
        reference, reference_velocity, reference_acceleration = reference_state(case, time)
        reference_rotation = Rotation.from_euler('xyz', reference[3:])
        error = np.concatenate((pose[:3] - reference[:3], (rotation * reference_rotation.inv()).as_rotvec()))
        history.append(error.copy())
        if step % pose_tick == 0:
            sample_step = max(0, step - latency_tick)
            measurement = history[sample_step].copy()
            if case['sensor_errors']:
                measurement += bounded_sensor_error(sample_step * period)
            if step == 0:
                estimate = measurement
            elif sample_step > previous_sample_step:
                estimate, estimated_velocity, estimated_disturbance = observer_update(
                    estimate, estimated_velocity, estimated_disturbance, measurement, (sample_step - previous_sample_step) * period,
                    command_history[previous_sample_step:sample_step])
            previous_sample_step = sample_step
        if step % command_tick == 0:
            predicted_error, predicted_velocity = propagate_estimate(
                estimate, estimated_velocity,
                np.asarray(command_history[previous_sample_step:step]).reshape(-1, 6) + estimated_disturbance, period)
            estimated_pose = reference.copy()
            estimated_pose[:3] += predicted_error[:3]
            estimated_rotation = Rotation.from_rotvec(predicted_error[3:]) * reference_rotation
            estimated_pose[3:] = estimated_rotation.as_euler('xyz')
            _, center = levitation_sim.place_piece(nominal, *estimated_pose)
            components = levitation_sim.window_components(nominal, selector, reference[:2], center, half_extents)
            correction = (-omega ** 2 * predicted_error - 2 * model.Fixed.pose_controller_damping * omega * predicted_velocity
                          - estimated_disturbance)
            target = np.zeros(6)
            target[:3] = nominal_mass * (reference_acceleration + correction[:3])
            target[2] += geometry.weight
            speed = np.linalg.norm(reference_velocity[:2])
            if speed > 0:
                target[:2] += sum(levitation_sim.drag_components_at_speed(speed, reference_drag).values()) * reference_velocity[:2] / speed
            inertia_rotation = estimated_rotation.as_matrix()
            nominal_world_inertia = inertia_rotation @ np.diag(geometry.inertias) @ inertia_rotation.T
            target[3:] = nominal_world_inertia @ correction[3:]
            allocated, required = levitation_sim.allocate_window_components(components, target, config.turns * allocation_budget)
            command[:] = 0
            enabled[:] = False
            for identity, ampere_turns in allocated.items():
                command[indices[identity]] = ampere_turns / config.turns
                enabled[indices[identity]] = True
            metrics['component_minimum_peak_A'] = max(metrics['component_minimum_peak_A'], required / config.turns)
            requested_peak = float(np.max(np.abs(command)))
            metrics['peak_requested_current_A'] = max(metrics['peak_requested_current_A'], requested_peak)
            metrics['command_saturated_updates'] += int(requested_peak > config.current_limit)
            command = np.clip(command, -config.current_limit, config.current_limit)
            resolution = 2 * model.Fixed.max_current_command_error
            command = np.clip(np.round(command / resolution) * resolution, -config.current_limit, config.current_limit)
            if step == 0:
                currents = command.copy()
                filtered_command = command.copy()
                current_integral = config.resistance * command
        if step % field_tick == 0:
            pose[3:] = rotation.as_euler('xyz')
            _, center = levitation_sim.place_piece(actual, *pose)
            actual_matrix = levitation_sim.actuator_matrix(actual, track, center) * config.turns * remanence_factor
            actual_drag = levitation_sim.conductive_sheet_response(geometry.gap + pose[2], config.coil_height / 1000)
        generalized_velocity = np.concatenate((velocity, angular_velocity))
        back_emf = actual_matrix.T @ generalized_velocity
        if step % regulator_tick == 0:
            feedback = currents + sense_error
            voltage, current_integral, requested_voltage = current_regulator(
                filtered_command - feedback, current_integral, config.resistance,
                control.inductance, config.usable_drive_voltage)
            metrics['voltage_saturated_steps'] += int(np.max(np.abs(requested_voltage[enabled])) > config.usable_drive_voltage)
            current_integral[~enabled] = 0
        voltage[~enabled] = -config.bus_voltage * model.Fixed.coil_bus_voltage_fraction * np.sign(currents[~enabled])
        next_currents = rl_step(currents, voltage, back_emf, resistance, inductance, period, enabled)
        average_currents = (currents + next_currents) / 2
        wrench = actual_matrix @ average_currents
        speed = np.linalg.norm(velocity[:2])
        if speed > 0:
            wrench[:2] -= sum(levitation_sim.drag_components_at_speed(speed, actual_drag).values()) * velocity[:2] / speed
        acceleration = wrench[:3] / mass - [0, 0, geometry.gravity]
        world_rotation = rotation.as_matrix()
        world_inertia = world_rotation @ np.diag(body_inertia) @ world_rotation.T
        angular_acceleration = np.linalg.solve(world_inertia, wrench[3:] - np.cross(angular_velocity, world_inertia @ angular_velocity))
        tip_offset = np.array([0, 0, piece.box_height / 1000 - actual.center_of_mass_height])
        actual_tip = pose[:3] + rotation.apply(tip_offset)
        reference_tip = reference[:3] + reference_rotation.apply(tip_offset)
        tip_error = np.linalg.norm((actual_tip - reference_tip)[:2])
        position_error = np.linalg.norm(error[:3])
        rim_offset = np.array([0, 0, -actual.center_of_mass_height])
        axis_z = rotation.apply([0, 0, 1])
        clearance = (geometry.gap + pose[2] + actual.center_of_mass_height + rotation.apply(rim_offset)[2]
                     - piece.diameter / 2000 * np.linalg.norm(axis_z[:2]) - geometry.surface_stack)
        copper_loss = resistance * np.dot(currents, currents)
        metrics['peak_tip_error_m'] = max(metrics['peak_tip_error_m'], float(tip_error))
        metrics['peak_position_error_m'] = max(metrics['peak_position_error_m'], float(position_error))
        metrics['minimum_rim_clearance_m'] = min(metrics['minimum_rim_clearance_m'], float(clearance))
        metrics['peak_actual_current_A'] = max(metrics['peak_actual_current_A'], float(np.max(np.abs(currents))))
        metrics['peak_voltage_V'] = max(metrics['peak_voltage_V'], float(np.max(np.abs(voltage[enabled]))))
        metrics['peak_midpoint_current_A'] = max(metrics['peak_midpoint_current_A'], abs(float(currents.sum())))
        metrics['peak_copper_loss_W'] = max(metrics['peak_copper_loss_W'], float(copper_loss))
        metrics['peak_terminal_power_W'] = max(metrics['peak_terminal_power_W'], float(voltage @ currents))
        tracking_failure = position_error > position_budget or tip_error > model.Inputs.max_tip_position_error or clearance < geometry.tilt_rim_clearance
        if step % pose_tick == 0 or tracking_failure or step == steps:
            trace.append(dict(time_s=time, error=error.tolist(), tip_error_m=float(tip_error),
                              peak_current_A=float(np.max(np.abs(currents))), copper_loss_W=float(copper_loss),
                              disturbance_estimate=estimated_disturbance.tolist()))
        command_history.append(correction.copy())
        if tracking_failure:
            break
        if step == steps:
            break
        pose[:3] += velocity * period + acceleration * period ** 2 / 2
        velocity += acceleration * period
        rotation = Rotation.from_rotvec(angular_velocity * period + angular_acceleration * period ** 2 / 2) * rotation
        angular_velocity += angular_acceleration * period
        currents = next_currents
        filtered_command = command + (filtered_command - command) * np.exp(-2 * np.pi * model.Fixed.setpoint_filter_cutoff * period)
        copper_energy += copper_loss * period
    metrics['duration_completed_s'] = time
    metrics['mean_copper_loss_W'] = copper_energy / time if time > 0 else float(copper_loss)
    metrics['current_reserve_fraction'] = 1 - max(metrics['peak_requested_current_A'], metrics['peak_actual_current_A']) / config.current_limit
    checks = dict(completed=time >= case['duration'], tip_error=metrics['peak_tip_error_m'] <= model.Inputs.max_tip_position_error,
                  position_error=metrics['peak_position_error_m'] <= position_budget,
                  rim_clearance=metrics['minimum_rim_clearance_m'] >= geometry.tilt_rim_clearance,
                  current_reserve=metrics['current_reserve_fraction'] >= model.Fixed.qualification_current_reserve_fraction,
                  midpoint_current=metrics['peak_midpoint_current_A'] <= model.Fixed.midpoint_balancer_current_rating,
                  no_current_clipping=metrics['command_saturated_updates'] == 0)
    return dict(case=case, metrics=metrics, checks=checks, passed=all(checks.values()), physical_coils=len(indices),
                time_refinement=time_refinement, mesh_refinement=mesh_refinement, field_refinement=field_refinement, trace=trace)


def compare_refinement(baseline, refined, current_limit, gap):
    baseline_times = np.array([point['time_s'] for point in baseline['trace']])
    refined_times = np.array([point['time_s'] for point in refined['trace']])
    common_times = baseline_times[baseline_times <= refined_times[-1]]
    baseline_position = np.array([point['error'][:3] for point in baseline['trace']])[:len(common_times)]
    refined_position = np.array([point['error'][:3] for point in refined['trace']])
    interpolated_position = np.column_stack([np.interp(common_times, refined_times, component)
                                             for component in refined_position.T])
    position_budget = gap * (model.Fixed.flight_position_error_gap_fraction if baseline['case']['moving']
                             else model.Fixed.position_error_gap_fraction)
    fraction = model.Fixed.qualification_convergence_fraction
    differences = dict(position_trace_m=float(np.max(np.linalg.norm(baseline_position - interpolated_position, axis=1))))
    checks = dict(position_trace=differences['position_trace_m'] <= fraction * position_budget,
                  acceptance_checks=baseline['checks'] == refined['checks'])
    scales = dict(peak_tip_error_m=model.Inputs.max_tip_position_error, peak_position_error_m=position_budget,
                  peak_requested_current_A=current_limit * model.Fixed.qualification_current_reserve_fraction,
                  peak_actual_current_A=current_limit * model.Fixed.qualification_current_reserve_fraction)
    for metric in ('peak_copper_loss_W', 'peak_terminal_power_W'):
        scales[metric] = max(abs(baseline['metrics'][metric]), abs(refined['metrics'][metric]))
    for metric, scale in scales.items():
        differences[metric] = abs(baseline['metrics'][metric] - refined['metrics'][metric])
        checks[metric] = differences[metric] <= fraction * scale
    return dict(differences=differences, checks=checks, converged=all(checks.values()))


def qualification_setup():
    board = design.BoardGeometry()
    coil = design.CoilBed(board)
    piece = design.Piece(board, design.HalbachArray(board))
    geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
    print('Recalculating magnetic measurements for the adopted winding', flush=True)
    measurements = levitation_sim.measure(geometry)
    config = design.selected_winding(coil, piece, measurements)
    print(f'Testing {config.turns} turns, {config.wire.label}, {config.bus_voltage:g} V', flush=True)
    magnet = levitation_sim.magnet_layout_from_geometry()
    array = levitation_sim.verification_coil_array(coil.control_cells_per_side, config.coil_height / 1000)
    _, center = levitation_sim.place_piece(magnet)
    matrix = levitation_sim.actuator_matrix(magnet, array, center)
    _, currents = levitation_sim.min_peak_current(matrix, [0, 0, geometry.weight, 0, 0, 0])
    _, eigenvalues = levitation_sim.open_loop_stiffness(magnet, array, currents)
    control = design.Control(coil, config, dict(instability_growth_rate=float(max(eigenvalues.real))))
    return board, coil, piece, config, control, geometry


def calculate_qualification(selected_cases=()):
    board, coil, piece, config, control, geometry = qualification_setup()
    cases = stress_cases(board, piece, geometry)
    if selected_cases:
        if not set(selected_cases).issubset({case['name'] for case in cases}):
            raise ValueError('Unknown qualification case')
        cases = [case for case in cases if case['name'] in selected_cases]
    results = []
    with ProcessPoolExecutor(max_workers=model.Fixed.qualification_parallel_workers,
                             initializer=levitation_sim.use_geometry, initargs=(geometry,)) as pool:
        futures = [pool.submit(simulate_case, case, board, coil, piece, config, control) for case in cases]
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            print(result['case']['name'], json.dumps(result['metrics']), flush=True)
        results.sort(key=lambda result: [case['name'] for case in cases].index(result['case']['name']))
        worst = min(results, key=lambda result: result['metrics']['current_reserve_fraction'])
        time_future = pool.submit(simulate_case, worst['case'], board, coil, piece, config, control,
                                  time_refinement=model.Fixed.qualification_refinement_factor)
        mesh_future = pool.submit(simulate_case, worst['case'], board, coil, piece, config, control,
                                  mesh_refinement=model.Fixed.qualification_refinement_factor)
        field_future = pool.submit(simulate_case, worst['case'], board, coil, piece, config, control,
                                   field_refinement=model.Fixed.qualification_refinement_factor)
        refined, mesh_refined, field_refined = time_future.result(), mesh_future.result(), field_future.result()
    return qualification_report(piece, config, control, geometry, results,
                                {('time', worst['case']['name']): refined,
                                 ('mesh', worst['case']['name']): mesh_refined,
                                 ('field', worst['case']['name']): field_refined})


def qualification_report(piece, config, control, geometry, results, refinements):
    baselines = {result['case']['name']: result for result in results}
    refinement_rows = [dict(kind=kind, result=result,
                            comparison=compare_refinement(baselines[name], result, config.current_limit, geometry.gap))
                       for (kind, name), result in refinements.items()]
    convergence = bool(refinement_rows) and all(row['comparison']['converged'] for row in refinement_rows)
    return dict(candidate=dict(current_limit_A=config.current_limit, bus_voltage_V=config.bus_voltage,
                               per_coil_voltage_V=config.usable_drive_voltage, pose_rate_Hz=control.pose_update_rate,
                               command_rate_Hz=model.Fixed.current_command_rate, pwm_rate_Hz=model.Fixed.driver_pwm_frequency,
                               regulator_rate_Hz=model.Fixed.current_regulator_rate,
                               instantaneous_current_target_A=model.Fixed.driver_peak_current,
                               switching_ripple_bound_A=control.current_ripple_peak,
                               steady_switch_peak_bound_A=control.switch_current_bound,
                               current_bandwidth_Hz=model.Fixed.current_loop_bandwidth, wire=config.wire.label,
                               turns=config.turns, resistance_ohm=config.resistance, inductance_estimate_H=control.inductance,
                               mass_g=piece.mass, com_height_mm=piece.com_height * 1000),
                results=results, refinements=refinement_rows, converged=convergence,
                sampled_candidate_passed=convergence and all(result['passed'] for result in results + list(refinements.values())),
                product_qualified=False,
                limitations=['Local hover recovery and one-period fast-flight segments, not full lift/fly/land trajectories or global robust stability.',
                             'Mass, COM, inertia, sensor error, latency and inductance bounds are assumptions, not measured confidence intervals.',
                             'PWM-averaged voltages; exact independent RL updates; separate steady-cycle ripple screen, no switched electromechanical trajectory, mutual inductance or bus/midpoint dynamics.',
                             'Sampled PI current regulator with conditional integration at voltage saturation; nominal gains Kp=L*2*pi*f and Ki=R*2*pi*f. This response is a hardware requirement, not a verified comparator implementation.',
                             'Outgoing channels freewheel through ideal split-rail clamps until current reaches zero; unused future coils are open circuit. Regenerative energy absorption is not sized here.',
                             'Nonlinear actuator and conductive-sheet coupling held between numerical field updates; current-reserve-critical case independently refined in integration, field refresh and force mesh at unchanged controller rates. Reference paths have constant height.',
                             'Current-pattern selection uses the planned position; six-axis wrench allocation uses the estimated actual pose.',
                             'Pre-energized initial currents; no takeoff/contact transitions, deliberate large tilt/yaw manoeuvres, neighbour coupling or full-board routing.',
                             'Conductive drag remains a scalar planar approximation; hardware sensing throughput, actual winding availability and thermal acceptance remain separate gates.'],
                source_sha256=source_fingerprints(model.__file__, design.__file__, levitation_sim.__file__, __file__))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', action='append', default=[])
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'results' / 'qualification.json')
    arguments = parser.parse_args()
    report = calculate_qualification(arguments.case)
    arguments.output.write_text(json.dumps(report, indent=model.Fixed.report_precision) + '\n')
    print(json.dumps({key: report[key] for key in ('sampled_candidate_passed', 'converged', 'product_qualified')}))

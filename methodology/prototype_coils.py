import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
from pathlib import Path

import numpy as np

import levitation_sim
import model
from model import Inputs, Fixed
from methodology import coil_layouts, design, source_fingerprints


def prototype_geometry(name):
    candidate = next(row for row in Inputs.coil_layout_candidates if row[0] == name)
    pitch = Fixed.prototype_wire_outer_diameter + Fixed.prototype_turn_spacing
    wall = Inputs.prototype_radial_turns * pitch + Fixed.coil_winding_clearance
    coil = design.CoilGeometry(candidate[2], candidate[3], wall, wall + Inputs.prototype_inner_corner_radius)
    wire = design.Wire(f'{Inputs.prototype_wire_diameter:g} mm round, Grade 2 maximum OD', pitch, pitch,
                       np.pi * (Inputs.prototype_wire_diameter / 2000) ** 2)
    turns = Inputs.prototype_radial_turns * Inputs.prototype_axial_layers
    winding = design.WindingGeometry(coil, Inputs.prototype_wire_diameter, Inputs.prototype_axial_layers,
                                     wire=wire, radial_turns=Inputs.prototype_radial_turns,
                                     lead_length=Fixed.prototype_lead_length, connection_length=(turns - 1) * pitch,
                                     resistance_per_m=max(Fixed.prototype_wire_resistance_per_m))
    if winding.layers % 2 or Inputs.prototype_inner_corner_radius <= 0:
        raise ValueError('Prototype requires paired winding layers and a positive inner bend radius')
    return candidate, coil, winding


def prototype_array(name, refinement=1):
    candidate, coil, winding = prototype_geometry(name)
    angle = dict(Inputs.prototype_layouts)[name]
    return coil_layouts.LayoutArray(candidate[1], coil, winding, angle, refinement, Inputs.prototype_coil_spacing)


def activation_factors(array, world_position):
    local = array.rotation.inv().apply([*world_position, 0])[:2]
    centers = np.array([[part['x'], part['y']] for part in array.placements])
    extents = np.array([[part['width'], part['length']] for part in array.placements]) / 2
    margin = array.window / 2 - np.abs(centers - local) - extents
    fraction = np.clip(margin / (Fixed.prototype_handoff_taper / 1000), 0, 1)
    return np.prod(fraction ** 2 * (3 - 2 * fraction), axis=1)


def geometry_record(name):
    candidate, coil, winding = prototype_geometry(name)
    array = prototype_array(name)
    return dict(name=name, pattern=candidate[1], rotation_deg=dict(Inputs.prototype_layouts)[name],
                outer_dimensions_mm=[coil.outer_width, coil.outer_length], wall_mm=coil.winding_radial_width,
                inner_window_mm=[winding.inner_window_width, winding.inner_window_length],
                inner_corner_radius_mm=Inputs.prototype_inner_corner_radius, outer_corner_radius_mm=coil.corner_radius,
                bare_wire_diameter_mm=Inputs.prototype_wire_diameter, maximum_insulated_diameter_mm=Fixed.prototype_wire_outer_diameter,
                packing_pitch_mm=winding.wire.axial_pitch, radial_turns=winding.turns_per_layer, axial_layers=winding.layers,
                turns=winding.turns, height_mm=winding.coil_height, gap_mm=Inputs.prototype_coil_spacing,
                short_pitch_mm=array.short_pitch * 1000, repeat_mm=array.repeat * 1000,
                wire_length_m=winding.length_per_winding, connection_allowance_mm=winding.connection_length,
                lead_length_each_mm=winding.lead_length, resistance_20C_range_ohm=(np.array(Fixed.prototype_wire_resistance_per_m) * winding.length_per_winding).tolist(),
                resistance_20C_screen_ohm=winding.resistance,
                estimated_inductance_mH=design.winding_inductance(coil, winding) * 1000)


def electrical_trace(currents, times, back_emf, resistance, inductance):
    slopes = np.gradient(currents, times, axis=0, edge_order=2)
    inductive_voltage = max(Fixed.qualification_inductance_factors) * inductance * slopes
    voltage = resistance * currents + inductive_voltage + back_emf
    bound = resistance * np.abs(currents) + np.abs(inductive_voltage) + np.abs(back_emf)
    return slopes, voltage, bound


def evaluate_path(job, *, sample_fractions=None, physical_model=None):
    name, angle, offset, yaw, samples, refinement = job
    _, coil, winding = prototype_geometry(name)
    if physical_model is None:
        geometry, _ = coil_layouts.reference_state()
        array = prototype_array(name, refinement)
    else:
        geometry, array, winding = physical_model(name, refinement)
    coil_layouts.validate_layout(array)
    magnet = levitation_sim.magnet_layout_from_geometry()
    direction = np.array([np.cos(np.radians(angle)), np.sin(np.radians(angle)), 0])
    transverse = np.array([-direction[1], direction[0], 0])
    world_direction = array.rotation.apply(direction)
    fractions = np.linspace(0, 1, samples) if sample_fractions is None else np.asarray(sample_fractions)
    if len(fractions) != samples or np.any(np.diff(fractions) <= 0):
        raise ValueError('Path samples must be distinct and ordered')
    distances = fractions * array.repeat
    acceleration = geometry.cruise_accels[0]
    initial_speed_squared = geometry.flight_speed ** 2 - 2 * acceleration * array.repeat
    if initial_speed_squared <= 0:
        raise ValueError('Handoff path is too long for the prescribed accelerating motion')
    speeds = np.sqrt(initial_speed_squared + 2 * acceleration * distances)
    times = (speeds - np.sqrt(initial_speed_squared)) / acceleration
    response = levitation_sim.conductive_sheet_response_at_depths(
        geometry.max_flight_gap + array.height + Fixed.pcb_thickness / 2000,
        geometry.max_flight_gap + array.height + Fixed.pcb_thickness / 1000 + Fixed.radiator_standoff_below_pcb / 1000)
    weight, com, br, resistance = coil_layouts.stress_parameters(geometry, winding, True)
    cases = []
    for distance, speed in zip(distances, speeds):
        position = array.rotation.apply(direction * distance + transverse * offset * array.short_pitch)
        array.build(center=position[:2])
        factors = activation_factors(array, position[:2])
        active = factors > 0
        array.coils = [windings for windings, enabled in zip(array.coils, active) if enabled]
        array.identities = [identity for identity, enabled in zip(array.identities, active) if enabled]
        factors = factors[active]
        _, center = levitation_sim.place_piece(magnet, x=position[0], y=position[1],
                                               z=geometry.max_flight_gap - geometry.gap, yaw=np.radians(yaw))
        matrix = coil_layouts.stressed_wrench(levitation_sim.actuator_matrix(magnet, array, center), geometry, com, br)
        drag = sum(levitation_sim.drag_components_at_speed(speed, response).values()) * br ** 2
        target = np.array([*(world_direction[:2] * (weight / geometry.gravity * acceleration + drag)), weight, 0, 0, 0])
        peak, initial = levitation_sim.min_peak_current(matrix, target, current_factors=factors)
        cases.append(dict(matrix=matrix, target=target, factors=factors, peak=peak, initial=initial,
                          identities=array.identities.copy(), position=position[:2], speed=speed))
    budget = Fixed.driver_channel_current * (1 - Fixed.qualification_current_reserve_fraction)
    limit = max(budget, max(case['peak'] for case in cases))
    identities = sorted(set(identity for case in cases for identity in case['identities']))
    indices = {identity: index for index, identity in enumerate(identities)}
    currents = np.zeros((samples, len(identities)))
    back_emf = np.zeros_like(currents)
    max_residual = 0
    for row, case in enumerate(cases):
        allocated = levitation_sim.min_loss_current(case['matrix'], case['target'], limit * case['factors'], initial_currents=case['initial'])
        selected = [indices[identity] for identity in case['identities']]
        currents[row, selected] = allocated
        velocity = np.array([*(world_direction[:2] * case['speed']), 0, 0, 0, 0])
        back_emf[row, selected] = case['matrix'].T @ velocity
        max_residual = max(max_residual, float(np.max(np.abs(case['matrix'] @ allocated - case['target']))), abs(float(allocated.sum())))
    inductance = design.winding_inductance(coil, winding)
    slopes, voltage, voltage_bound = electrical_trace(currents, times, back_emf, resistance, inductance)
    loss = resistance * np.sum(currents ** 2, axis=1)
    ripple = design.split_rail_ripple_peak(winding.resistance, min(Fixed.qualification_inductance_factors) * inductance,
                                          Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction, Fixed.driver_pwm_frequency)
    required = max(case['peak'] for case in cases)
    available_voltage = Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction * Fixed.usable_bus_voltage_fraction
    return dict(name=name, path_angle_deg=angle, offset_fraction=offset, yaw_deg=yaw, samples=samples,
                mesh_refinement=refinement, duration_s=float(times[-1] - times[0]), distance_mm=float(np.ptp(distances)) * 1000,
                peak_required_rating_A=required, allocation_limit_A=limit, peak_allocated_current_A=float(np.max(np.abs(currents))),
                peak_copper_loss_W=float(max(loss)), peak_current_slope_A_per_s=float(np.max(np.abs(slopes))),
                estimated_peak_voltage_V=float(np.max(np.abs(voltage))), available_voltage_V=available_voltage,
                voltage_bound_peak_V=float(np.max(voltage_bound)),
                current_reserve_pass=required <= budget,
                averaged_voltage_pass=float(np.max(voltage_bound)) <= available_voltage,
                estimated_ripple_peak_A=ripple, ripple_pass=ripple <= Fixed.driver_max_ripple_current,
                max_wrench_or_midpoint_residual=max_residual, identities=identities,
                time_s=times.tolist(), position_mm=(np.array([case['position'] for case in cases]) * 1000).tolist(),
                current_A=currents.tolist(), copper_loss_W=loss.tolist(),
                required_rating_A=[case['peak'] for case in cases], voltage_bound_V=np.max(voltage_bound, axis=1).tolist())


def compare_paths(coarse, fine):
    common = fine['current_A'][::2]
    identities = sorted(set(tuple(identity) for row in (coarse, fine) for identity in row['identities']))
    matrices = []
    for row, values in ((coarse, coarse['current_A']), (fine, common)):
        matrix = np.zeros((len(values), len(identities)))
        for index, identity in enumerate(row['identities']):
            matrix[:, identities.index(tuple(identity))] = np.asarray(values)[:, index]
        matrices.append(matrix)
    changes = {key: abs(fine[key] / coarse[key] - 1) for key in
               ('peak_required_rating_A', 'peak_copper_loss_W', 'voltage_bound_peak_V', 'peak_current_slope_A_per_s')}
    changes['common_position_current'] = float(np.max(np.abs(matrices[0] - matrices[1]))) / coarse['peak_allocated_current_A']
    return dict(relative_changes=changes, passed=max(changes.values()) <= Fixed.qualification_convergence_fraction)


def run(output, refine):
    fingerprints = source_fingerprints(model.__file__, design.__file__, levitation_sim.__file__, coil_layouts.__file__, __file__)
    if refine:
        report = json.loads(output.read_text())
        selected = []
        for name, _ in Inputs.prototype_layouts:
            rows = [row for row in report['paths'] if row['name'] == name]
            for metric in ('peak_required_rating_A', 'voltage_bound_peak_V'):
                row = max(rows, key=lambda item: item[metric])
                if row not in selected:
                    selected.append(row)
        jobs = [(row['name'], row['path_angle_deg'], row['offset_fraction'], row['yaw_deg'],
                 2 * row['samples'] - 1, row['mesh_refinement']) for row in selected]
    else:
        report = dict(scope='Rounded round-wire prototype; prescribed accelerating level motion at the maximum flight gap; hot/heavy king; tapered channel bounds',
                      geometries=[geometry_record(name) for name, _ in Inputs.prototype_layouts], source_sha256=fingerprints,
                      limitations=['Prescribed motion and averaged RL voltage estimates, not closed-loop stability or frequency qualification.',
                                   'Assembly and wire tolerances are allowances, not supplier confirmation of a finished winding.',
                                   'Closed-turn force approximation excludes leads and inter-turn connections; their length is included in resistance.',
                                   'Spatial thermal behavior, mutual inductance, tilted/yawing motion, neighbours, board edges and reset remain unqualified.'])
        jobs = [(name, angle, offset, yaw, Fixed.prototype_path_samples, min(Fixed.current_demand_mesh_refinements))
                for name, _ in Inputs.prototype_layouts for angle in Fixed.prototype_path_angles_deg
                for offset in Fixed.prototype_path_offsets for yaw in Fixed.prototype_path_yaws_deg]
    rows = []
    with ProcessPoolExecutor(max_workers=Fixed.qualification_parallel_workers) as pool:
        pending = {pool.submit(evaluate_path, job): job for job in jobs}
        for future in as_completed(pending):
            row = future.result()
            rows.append(row)
            print(json.dumps({key: value for key, value in row.items() if key not in
                              ('identities', 'time_s', 'position_mm', 'current_A', 'copper_loss_W', 'estimated_voltage_V',
                               'required_rating_A', 'voltage_bound_V')}), flush=True)
    if refine:
        report['refined_paths'] = rows
        report['refinement_source_sha256'] = fingerprints
        report['sampling_checks'] = [dict(name=coarse['name'], path_angle_deg=coarse['path_angle_deg'],
                                           offset_fraction=coarse['offset_fraction'], yaw_deg=coarse['yaw_deg'],
                                           **compare_paths(coarse, next(row for row in rows if all(row[key] == coarse[key] for key in
                                                         ('name', 'path_angle_deg', 'offset_fraction', 'yaw_deg'))))) for coarse in selected]
    else:
        report['paths'] = sorted(rows, key=lambda row: (row['name'], row['path_angle_deg'], row['offset_fraction'], row['yaw_deg']))
    output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')


def check_mesh(output):
    fingerprints = source_fingerprints(model.__file__, design.__file__, levitation_sim.__file__, coil_layouts.__file__, __file__)
    report = json.loads(output.read_text())
    checks = []
    for name, _ in Inputs.prototype_layouts:
        rows = [row for row in report['refined_paths'] if row['name'] == name]
        for metric, trace_key in (('peak_required_rating_A', 'required_rating_A'), ('voltage_bound_peak_V', 'voltage_bound_V')):
            row = max(rows, key=lambda item: item[metric])
            values = np.asarray(row[trace_key])
            start = int(np.clip(np.argmax(values) - 1, 0, row['samples'] - 3))
            fractions = np.arange(start, start + 3) / (row['samples'] - 1)
            results = []
            for refinement in Fixed.current_demand_mesh_refinements:
                result = evaluate_path((name, row['path_angle_deg'], row['offset_fraction'], row['yaw_deg'], len(fractions), refinement),
                                       sample_fractions=fractions)
                results.append({key: result[key] for key in ('mesh_refinement', 'peak_required_rating_A', 'peak_copper_loss_W', 'voltage_bound_peak_V')})
            changes = {key: abs(results[-1][key] / results[0][key] - 1) for key in results[0] if key != 'mesh_refinement'}
            check = dict(name=name, metric=metric, path_angle_deg=row['path_angle_deg'], yaw_deg=row['yaw_deg'],
                         offset_fraction=row['offset_fraction'], fractions=fractions.tolist(), values=results,
                         relative_changes=changes, passed=max(changes.values()) <= Fixed.qualification_convergence_fraction)
            checks.append(check)
            print(json.dumps(check), flush=True)
    report['mesh_checks'] = checks
    report['mesh_source_sha256'] = fingerprints
    output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')


def plot_paths(output):
    import matplotlib.pyplot as plt

    report = json.loads(output.read_text())
    figure, axes = plt.subplots(3, len(Inputs.prototype_layouts), figsize=np.array(Fixed.cad_preview_size) / Fixed.layout_plot_dpi,
                               sharex='col', layout='constrained')
    for column, (name, _) in enumerate(Inputs.prototype_layouts):
        row = max((row for row in report['refined_paths'] if row['name'] == name), key=lambda row: row['voltage_bound_peak_V'])
        times = np.array(row['time_s']) * 1000
        axes[0, column].plot(times, row['current_A'])
        axes[0, column].axhline(row['allocation_limit_A'], color='black', linestyle='dashed')
        axes[0, column].axhline(-row['allocation_limit_A'], color='black', linestyle='dashed')
        axes[0, column].set_title(f'{name}: path {row["path_angle_deg"]:g}°, yaw {row["yaw_deg"]:g}°')
        axes[0, column].set_ylabel('Individual coil currents (A)')
        axes[1, column].plot(times, row['copper_loss_W'], color='sienna')
        axes[1, column].set_ylabel('Total copper loss (W)')
        axes[2, column].plot(times, row['voltage_bound_V'], color='royalblue')
        axes[2, column].axhline(row['available_voltage_V'], color='black', linestyle='dashed', label='Available average voltage')
        axes[2, column].set(xlabel='Time (ms)', ylabel='RL voltage bound (V)')
        axes[2, column].legend()
    figure.suptitle('Rounded round-wire prototype: prescribed coil handoffs\nHot/heavy king; averaged electrical screen, not feedback qualification')
    figure.savefig(output.with_suffix('.png'), dpi=Fixed.layout_plot_dpi)
    plt.close(figure)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check practical winding dimensions and prescribed coil handoffs.')
    parser.add_argument('--refine', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('results') / 'prototype_coils.json')
    arguments = parser.parse_args()
    if arguments.check:
        check_mesh(arguments.output)
    elif arguments.plot:
        plot_paths(arguments.output)
    else:
        run(arguments.output, arguments.refine)

import json
from pathlib import Path

import numpy as np

import levitation_sim
import model
from methodology import design, source_fingerprints


def pose_description(case):
    return {
        "gap_mm": float(case["gap"] * 1000),
        "x_mm": float(case["x"] * 1000),
        "y_mm": float(case["y"] * 1000),
        "yaw_deg": float(np.degrees(case["yaw"])),
        "direction_xy": case["direction"].tolist(),
    }


def case_current_breakdown(case, geometry, turns):
    weight = geometry.weight
    hover = np.array([0, 0, weight, 0, 0, 0])
    acceleration = hover.copy()
    acceleration[:2] = weight / geometry.gravity * geometry.cruise_accels[0] * case["direction"]
    targets = {"hover": hover, "acceleration": acceleration, "acceleration_and_drag": case["target"]}
    rows = {}
    for name, target in targets.items():
        peak, _ = levitation_sim.min_peak_current(case["wrench"], target)
        force_only_peak, _ = levitation_sim.min_peak_current(case["wrench"][:3], target[:3])
        rows[name] = {"six_axis_peak_A": float(peak / turns), "force_only_peak_A": float(force_only_peak / turns)}
    return rows


def analyse_cases(cases, geometry, config):
    rows = [{"case": case, "breakdown": case_current_breakdown(case, geometry, config.turns)} for case in cases]
    maxima = {}
    for scenario in rows[0]["breakdown"]:
        maxima[scenario] = {}
        for constraint in rows[0]["breakdown"][scenario]:
            worst = max(rows, key=lambda row: row["breakdown"][scenario][constraint])
            maxima[scenario][constraint] = {"current_A": worst["breakdown"][scenario][constraint],
                                           "pose": pose_description(worst["case"])}
    worst = max(rows, key=lambda row: row["case"]["peak_ampere_turns"])
    by_direction = []
    for direction in levitation_sim.horizontal_wrench_targets(1.0, 0.0)[:, :2]:
        selected = [row for row in rows if np.array_equal(row["case"]["direction"], direction)]
        by_direction.append({"direction_xy": direction.tolist(),
                             "acceleration_peak_A": max(row["breakdown"]["acceleration"]["six_axis_peak_A"] for row in selected),
                             "full_peak_A": max(row["breakdown"]["acceleration_and_drag"]["six_axis_peak_A"] for row in selected)})
    return worst["case"], {"case_count": len(cases), "envelope_maxima": maxima,
                           "worst_pose": pose_description(worst["case"]),
                           "same_pose_breakdown": worst["breakdown"], "directions": by_direction}


def torque_and_mass_sensitivity(case, geometry, piece, turns):
    wrench = case["wrench"]
    target = case["target"]
    constraints = {"force_only": [0, 1, 2], "force_and_roll": [0, 1, 2, 3],
                   "force_and_pitch": [0, 1, 2, 4], "force_and_yaw": [0, 1, 2, 5],
                   "force_roll_pitch": [0, 1, 2, 3, 4], "six_axis": list(range(6))}
    torque_rows = {}
    for name, selected_rows in constraints.items():
        peak, currents = levitation_sim.min_peak_current(wrench[selected_rows], target[selected_rows])
        torque_rows[name] = {"peak_A": float(peak / turns), "torque_mNm": ((wrench @ currents)[3:] * 1000).tolist()}
    heights = {"configured_COM": geometry.center_of_mass_height,
               "shell_centroid_counterfactual": piece.shell_com_height,
               "magnet_midplane_counterfactual": piece.magnet_com_height}
    mass_rows = {}
    for name, height in heights.items():
        displacement = np.array([0, 0, geometry.center_of_mass_height - height])
        shifted = wrench.copy()
        shifted[3:] += np.cross(displacement, wrench[:3].T).T
        peak, _ = levitation_sim.min_peak_current(shifted, target)
        mass_rows[name] = {"COM_height_mm": float(height * 1000), "peak_A": float(peak / turns)}
    return {"torque_constraints": torque_rows, "COM_sensitivity": mass_rows}


def geometry_checks(case, geometry, coil, config):
    magnet = levitation_sim.magnet_layout_from_geometry()
    _, center = levitation_sim.place_piece(magnet, x=case["x"], y=case["y"],
                                           z=case["gap"] - geometry.gap, yaw=case["yaw"])
    height = config.coil_height / 1000
    refinement_rows = []
    for factor in model.Fixed.current_demand_mesh_refinements:
        array = levitation_sim.verification_coil_array(coil.control_cells_per_side, height, factor)
        matrix = levitation_sim.actuator_matrix(magnet, array, center)
        peak, _ = levitation_sim.min_peak_current(matrix, case["target"])
        refinement_rows.append({"refinement_factor": factor, "segment_mesh": array.meshing,
                                "radial_filaments": array.filaments_radial, "axial_filaments": array.filaments_axial,
                                "peak_A": float(peak / config.turns)})
    array = levitation_sim.verification_coil_array(coil.control_cells_per_side, height)
    array.build(center=(case["x"], case["y"]))
    matrix = levitation_sim.actuator_matrix(magnet, array, center)
    peak, _ = levitation_sim.min_peak_current(matrix, case["target"])
    levitation_sim.place_piece(magnet)
    return {"mesh_refinement_at_original_worst_pose": refinement_rows,
            "integer_pitch_window_shift_at_original_worst_pose": {"coil_count": len(array.coils), "peak_A": float(peak / config.turns)}}


def handoff_analysis(case, geometry, coil, config, sampled_peak):
    magnet = levitation_sim.magnet_layout_from_geometry()
    height = config.coil_height / 1000
    array = levitation_sim.verification_coil_array(coil.control_cells_per_side, height)
    pairs = []
    peak = sampled_peak
    for axis in range(2):
        for boundary in levitation_sim.coil_window_handoffs():
            for offset in model.Fixed.active_window_handoff_offsets:
                solutions = []
                for side in (-1, 1):
                    position = [case["x"], case["y"]]
                    position[axis] = boundary + side * offset
                    array.build(center=position)
                    _, center = levitation_sim.place_piece(magnet, x=position[0], y=position[1],
                                                           z=case["gap"] - geometry.gap, yaw=case["yaw"])
                    wrench = levitation_sim.actuator_matrix(magnet, array, center)
                    required, initial = levitation_sim.min_peak_current(wrench, case["target"])
                    peak = max(peak, required)
                    raw_identities = tuple(array.identities)
                    components = {raw_identities: (wrench, initial, 0.0)}
                    for selected_center, weight in levitation_sim.blended_window_centers(position):
                        array.build(center=selected_center)
                        identities = tuple(array.identities)
                        if identities not in components:
                            wrench = levitation_sim.actuator_matrix(magnet, array, center)
                            required, initial = levitation_sim.min_peak_current(wrench, case["target"])
                            peak = max(peak, required)
                            components[identities] = (wrench, initial, 0.0)
                        wrench, initial, previous_weight = components[identities]
                        components[identities] = (wrench, initial, previous_weight + weight)
                    solutions.append((raw_identities, components))
                pairs.append((axis, boundary, offset, solutions))
    rows = []
    maximum_residual = 0.0
    blended_peak = 0.0
    blended_coil_count = 0
    for axis, boundary, offset, solutions in pairs:
        patterns = []
        for raw_identities, components in solutions:
            weighted_components = {identities: (wrench, weight)
                                   for identities, (wrench, _, weight) in components.items() if weight > 0}
            blended_ampere_turns, _ = levitation_sim.allocate_window_components(weighted_components, case["target"], peak)
            blended = {identity: current / config.turns for identity, current in blended_ampere_turns.items()}
            columns = {}
            for identities, (matrix, _) in weighted_components.items():
                columns.update(zip(identities, matrix.T))
            achieved = sum(columns[identity] * current for identity, current in blended_ampere_turns.items())
            wrench, initial, _ = components[raw_identities]
            abrupt = levitation_sim.min_loss_current(wrench, case["target"], peak, initial_currents=initial) / config.turns
            maximum_residual = max(maximum_residual, float(np.max(np.abs(achieved - case["target"]))), abs(sum(blended.values())))
            blended_peak = max(blended_peak, max(abs(current) for current in blended.values()))
            blended_coil_count = max(blended_coil_count, len(blended))
            patterns.append({"abrupt": dict(zip(raw_identities, abrupt)), "blended": blended})
        row = {"axis": "xy"[axis], "boundary_mm": float(boundary * 1000), "offset_mm": offset * 1000}
        for policy in ("abrupt", "blended"):
            before, after = (pattern[policy] for pattern in patterns)
            identities = before.keys() | after.keys()
            delta = max(abs(after.get(identity, 0) - before.get(identity, 0)) for identity in identities)
            row[policy] = {"max_channel_change_A": float(delta),
                           "equivalent_slew_A_per_s_at_cruise_speed": float(delta * geometry.flight_speed / (2 * offset)),
                           "physical_coils_in_union": len(identities)}
        rows.append(row)
    if maximum_residual > model.Fixed.allocator_residual_tolerance or blended_peak > peak / config.turns * (1 + model.Fixed.allocator_limit_tolerance):
        raise RuntimeError("Blended handoff violates force/torque balance, zero-sum current or channel rating")
    width = levitation_sim.coil_window_transition_width()
    return {"allocation_limit_A": float(peak / config.turns), "rows": rows,
            "blended_peak_A": float(blended_peak), "blended_coil_count": blended_coil_count,
            "maximum_wrench_or_current_sum_residual": maximum_residual,
            "transition_width_mm": float(width * 1000), "transition_time_at_cruise_ms": float(width / geometry.flight_speed * 1000),
            "baseline_current_command_period_ms": 1000 / model.Fixed.current_command_rate,
            "baseline_command_updates_per_transition": float(width / geometry.flight_speed * model.Fixed.current_command_rate),
            "scope": "Worst interior pose gap/yaw/force target; both translation axes and every handoff plane. Abrupt selection and smooth blending use the same component current limit. Full-path dynamics and correction reserve remain unverified."}


def edge_window_analysis(geometry, board, coil, config):
    magnet = levitation_sim.magnet_layout_from_geometry()
    height = config.coil_height / 1000
    array = levitation_sim.verification_coil_array(coil.control_cells_per_side, height)
    half_extents = np.array([board.motor_width, board.motor_height]) / 2000
    center_limits = half_extents - board.base_diameter / 2000
    worst = None
    cases = 0
    coil_counts = set()
    for gap in (geometry.gap, geometry.max_flight_gap):
        thrust = geometry.weight / geometry.gravity * geometry.cruise_accels[0] + levitation_sim.conductive_drag(geometry.flight_speed, gap, height)
        for yaw in np.radians(model.Fixed.pose_yaws_deg):
            for x_sign in (-1, 0, 1):
                for y_sign in (-1, 0, 1):
                    if x_sign == y_sign == 0:
                        continue
                    x, y = center_limits * [x_sign, y_sign]
                    array.build(center=(x, y), half_extents=half_extents)
                    coil_counts.add(len(array.coils))
                    _, center = levitation_sim.place_piece(magnet, x=x, y=y, z=gap - geometry.gap, yaw=yaw)
                    wrench = levitation_sim.actuator_matrix(magnet, array, center)
                    for target in levitation_sim.horizontal_wrench_targets(thrust, geometry.weight):
                        peak, _ = levitation_sim.min_peak_current(wrench, target)
                        cases += 1
                        if worst is None or peak > worst["peak_ampere_turns"]:
                            worst = {"gap": gap, "x": x, "y": y, "yaw": yaw, "direction": target[:2] / thrust,
                                     "peak_ampere_turns": peak, "coil_count": len(array.coils)}
    return {"case_count": cases, "coil_counts": sorted(coil_counts),
            "peak_A": float(worst["peak_ampere_turns"] / config.turns), "worst_pose": pose_description(worst),
            "scope": "Four outer motor-area corners and four edge midpoints, with the king base fully supported and complete coil windows shifted inward to fit the motor footprint; not continuous edge coverage."}


def moving_window_analysis(geometry, board, coil, config):
    allocations = levitation_sim.operating_allocations(coil.control_cells_per_side, config.coil_height / 1000,
                                                       config.turns * config.current_limit, include_flight_cases=True, follow_piece=True, include_contact=False)
    cases = allocations["flight_cases"]
    worst = max(cases, key=lambda case: case["peak_ampere_turns"])
    return {"case_count": len(cases), "lattice_repeat_mm": geometry.coil_lattice_repeat * 1000,
            "sampled_phases_mm": (levitation_sim.coil_window_phases() * 1000).tolist(),
            "coil_count": allocations["flight"]["coil_count"],
            "peak_A": float(allocations["flight"]["peak_ampere_turns"] / config.turns),
            "worst_copper_loss_W": float(config.resistance * allocations["flight"]["worst_squared_sum"] / config.turns ** 2),
            "worst_pose": pose_description(worst), "same_pose_breakdown": case_current_breakdown(worst, geometry, config.turns),
            "handoff": handoff_analysis(worst, geometry, coil, config, allocations["flight"]["peak_ampere_turns"]),
            "edge_probe": edge_window_analysis(geometry, board, coil, config)}


def calculate_current_demand():
    board = design.BoardGeometry()
    coil = design.CoilBed(board)
    halbach = design.HalbachArray(board)
    piece = design.Piece(board, halbach)
    geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
    measurements = levitation_sim.measure(geometry)
    config = design.selected_winding(coil, piece, measurements)
    allocations = levitation_sim.operating_allocations(coil.control_cells_per_side, config.coil_height / 1000,
                                                       config.turns * config.current_limit, include_flight_cases=True, include_contact=False)
    worst, analysis = analyse_cases(allocations["flight_cases"], geometry, config)
    analysis.update(torque_and_mass_sensitivity(worst, geometry, piece, config.turns))
    analysis.update(geometry_checks(worst, geometry, coil, config))
    analysis["moving_window_candidate"] = moving_window_analysis(geometry, board, coil, config)
    currents = levitation_sim.min_loss_current(worst["wrench"], worst["target"], allocations["flight"]["peak_ampere_turns"],
                                               initial_currents=worst["currents"]) / config.turns
    analysis["worst_pose_solution"] = {"coil_currents_A": currents.tolist(), "target_wrench_N_Nm": worst["target"].tolist(),
                                        "achieved_wrench_N_Nm": (worst["wrench"] @ (currents * config.turns)).tolist(),
                                        "sum_current_A": float(currents.sum()),
                                        "copper_loss_W": float(config.resistance * np.sum(currents ** 2))}
    analysis["physical_inputs"] = {"piece_mass_g": piece.mass, "magnet_mass_g": halbach.magnet_mass,
                                   "shell_mass_g": piece.shell_mass, "COM_height_mm": geometry.center_of_mass_height * 1000,
                                   "tilt_inertia_kg_m2": piece.tilt_inertia, "yaw_inertia_kg_m2": piece.yaw_inertia,
                                   "weight_N": geometry.weight, "acceleration_m_s2": geometry.cruise_accels[0],
                                   "acceleration_force_N": geometry.weight / geometry.gravity * geometry.cruise_accels[0],
                                   "peak_speed_m_s": geometry.flight_speed,
                                   "drag_components_N": levitation_sim.conductive_drag_components(geometry.flight_speed, worst["gap"], config.coil_height / 1000),
                                   "wire": config.wire.label, "turns": config.turns, "coil_height_mm": config.coil_height,
                                   "resistance_ohm": config.resistance, "baseline_driver_limit_A": model.Fixed.driver_channel_current}
    analysis["preliminary_screen"] = {"cold_current_A": config.current_limit / config.sprint_current_margin,
                                        "hot_current_margin": config.hot_sprint_current_margin,
                                        "method": "reference-height scalar coupling, all horizontal directions, no conductive drag"}
    analysis["limitations"] = ["The default report retains fixed-window allocation; smooth moving-window commutation remains a diagnostic candidate until complete trajectories, neighbour constraints and delayed dynamics are verified.",
                               "Counterfactual COM and shifted-window values apply to the original worst pose only; they are not new global minima.",
                               "The configured king uses a uniformly scaled hollow shell and bottom magnets; this is not a CAD or measured mass distribution.",
                               "Current contributions are coupled and non-additive; every row is a newly optimized allocation.",
                               "Mesh refinement checks one fixed-window pose. Moving-window phase samples, edge probes and handoff checks do not establish continuous whole-board coverage.",
                               "No delayed closed-loop dynamics, hot-current reserve or supplier-confirmed winding is established by this diagnostic."]
    analysis["source_sha256"] = source_fingerprints(model.__file__, design.__file__, levitation_sim.__file__, __file__)
    return analysis


if __name__ == "__main__":
    report = calculate_current_demand()
    (Path(__file__).parent / 'results' / 'current_demand.json').write_text(json.dumps(report, indent=model.Fixed.report_precision) + '\n')
    print(json.dumps(report, indent=model.Fixed.report_precision))

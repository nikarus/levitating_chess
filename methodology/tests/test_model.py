import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import levitation_sim
import model
from methodology import design
from methodology import qualification


class ModelMigrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        folder = Path(design.__file__).parent
        paths = (Path(model.__file__), Path(levitation_sim.__file__), *folder.glob('*.py'), *folder.joinpath('cad').glob('*.py'))
        cls.production_sources = {path: ast.parse(path.read_text()) for path in paths}

    def setUp(self):
        self.geometry = SimpleNamespace(settings=model.Fixed)
        levitation_sim.use_geometry(self.geometry)

    def test_king_com_includes_bottom_magnet_mass(self):
        board = design.BoardGeometry()
        magnets = design.HalbachArray(board)
        piece = design.Piece(board, magnets)
        self.assertAlmostEqual(piece.mass, piece.shell_mass + magnets.magnet_mass)
        self.assertAlmostEqual(piece.shell_mass * (piece.shell_com_height - piece.com_height)
                               + magnets.magnet_mass * (piece.magnet_com_height - piece.com_height), 0)
        self.assertLess(piece.com_height, piece.shell_com_height)
        self.assertGreater(piece.com_height, piece.magnet_com_height)

    def test_piece_inertias_reduce_to_the_magnet_block_without_shell(self):
        board = design.BoardGeometry()
        magnets = design.HalbachArray(board)
        with patch.object(model.Inputs, "piece_shell_shape_factor", 0):
            piece = design.Piece(board, magnets)
        self.assertEqual(piece.mass, magnets.magnet_mass)
        self.assertEqual(piece.com_height, piece.magnet_com_height)
        self.assertAlmostEqual(piece.tilt_inertia, magnets.magnet_mass / 1000
                               * ((board.platform_side / 1000) ** 2 + (model.Inputs.magnet_thickness / 1000) ** 2) / 12)
        self.assertAlmostEqual(piece.yaw_inertia, magnets.magnet_mass / 1000 * (board.platform_side / 1000) ** 2 / 6)

    def test_piece_inertias_reduce_to_a_hollow_shell_without_magnets(self):
        board = design.BoardGeometry()
        piece = design.Piece(board, SimpleNamespace(magnet_mass=0))
        outer_radius = piece.diameter / 2000
        inner_radius = outer_radius - model.Inputs.plastic_wall_thickness / 1000
        self.assertEqual(piece.com_height, piece.shell_com_height)
        self.assertGreater(piece.yaw_inertia, piece.mass / 1000 * inner_radius ** 2 / 2)
        self.assertLess(piece.yaw_inertia, piece.mass / 1000 * outer_radius ** 2)
        self.assertGreater(piece.tilt_inertia, 0)

    def test_empty_buffer_has_no_components_or_cost(self):
        buffer = design.EnergyBuffer(model.Inputs.selected_bus_voltage, 1, 0, 0)
        self.assertEqual((buffer.cell_count, buffer.oring_count, buffer.management_count), (0, 0, 0))
        self.assertEqual((buffer.price, buffer.mass, buffer.peak_power, buffer.usable_energy), (0, 0, 0, 0))

    def test_positive_buffer_meets_energy_and_power_requirements(self):
        buffer = design.EnergyBuffer(model.Inputs.selected_bus_voltage, 1, 500, 1000)
        self.assertGreaterEqual(buffer.usable_energy, 1000)
        self.assertGreaterEqual(buffer.peak_power, 500)
        self.assertGreater(buffer.price, 0)

    def test_insulation_fits_allocated_winding_wall(self):
        coil = design.CoilBed(design.BoardGeometry())
        for thickness in model.Inputs.winding_conductor_thicknesses:
            wire = design.rectangular_wire(coil.conductor_radial_width, thickness)
            occupied = wire.radial_pitch * model.Fixed.turns_per_radial_layer + model.Fixed.coil_winding_clearance
            self.assertLessEqual(occupied, coil.winding_radial_width + model.Fixed.allocator_limit_tolerance)
            self.assertGreater(wire.copper_area, 0)

    def test_minimum_loss_respects_wrench_balance_and_channel_limits(self):
        matrix = np.array([[1.0, 2.0, -1.0]])
        target = np.array([1.0])
        peak, initial = levitation_sim.min_peak_current(matrix, target)
        resistances = np.array([10.0, 1.0, 1.0])
        currents = levitation_sim.min_loss_current(matrix, target, peak * 2, resistances, initial)
        np.testing.assert_allclose(matrix @ currents, target, atol=model.Fixed.allocator_residual_tolerance)
        self.assertAlmostEqual(float(currents.sum()), 0)
        self.assertLessEqual(float(np.max(np.abs(currents))), peak * 2 + model.Fixed.allocator_limit_tolerance)
        self.assertLess(float(np.sum(resistances * currents ** 2)), float(np.sum(resistances * initial ** 2)))

    def test_joint_shared_coils_can_make_individual_targets_incompatible(self):
        first = np.array([[1.0, -1.0]])
        second = np.array([[1.0, -1.0]])
        levitation_sim.min_peak_current(first, [1.0])
        levitation_sim.min_peak_current(second, [-1.0])
        with self.assertRaises(RuntimeError):
            levitation_sim.min_peak_current(np.vstack((first, second)), [1.0, -1.0])

    def test_peak_allocation_is_invariant_to_constraint_units(self):
        matrix = np.column_stack((np.eye(3), -np.ones(3)))
        target = np.array([1.0, 2.0, 3.0])
        peak, currents = levitation_sim.min_peak_current(matrix, target)
        scales = np.array([1e-8, 1e-4, 1e4])
        scaled_peak, scaled_currents = levitation_sim.min_peak_current(matrix * scales[:, None], target * scales)
        self.assertAlmostEqual(peak, scaled_peak)
        np.testing.assert_allclose(currents, scaled_currents, atol=model.Fixed.allocator_residual_tolerance)
        np.testing.assert_allclose(matrix @ scaled_currents, target, atol=model.Fixed.allocator_residual_tolerance)

    def test_peak_rating_contains_the_returned_physical_currents(self):
        physical_peak = 1 + model.Fixed.allocator_tolerance
        result = SimpleNamespace(success=True, x=np.array([physical_peak, -physical_peak, 1]))
        with patch.object(levitation_sim, "linprog", return_value=result):
            peak, currents = levitation_sim.min_peak_current(np.array([[1, -1]]), [2 * physical_peak])
        self.assertEqual(peak, max(abs(currents)))
        allocated = levitation_sim.min_loss_current(np.array([[1, -1]]), [2 * physical_peak], peak,
                                                    initial_currents=currents)
        np.testing.assert_allclose(allocated, currents, atol=model.Fixed.allocator_residual_tolerance)

    def test_winding_screen_catches_negative_direction_demand(self):
        wrench = np.column_stack((np.eye(6), -np.ones(6)))
        levitation_sim.use_geometry(SimpleNamespace(settings=model.Fixed, coil_long=1, gap=0))
        with (patch.object(model.Fixed, "pose_yaws_deg", (0,)),
              patch.object(model.Fixed, "pose_phase_fractions", (0,)),
              patch.object(model.Fixed, "pose_tilt_fractions", ()),
              patch.object(levitation_sim, "magnet_layout_from_geometry"),
              patch.object(levitation_sim, "coil_array_from_geometry"),
              patch.object(levitation_sim, "place_piece", return_value=(None, np.zeros(3))),
              patch.object(levitation_sim, "rim_tilt_limit", return_value=0),
              patch.object(levitation_sim, "actuator_matrix", return_value=wrench)):
            sweep = levitation_sim.flight_worst_case(1, 1, 1, (0,), 1, 1, (1,))
        positive_peak, _ = levitation_sim.min_peak_current(wrench, [0, 1, 1, 0, 0, 0])
        negative_peak, _ = levitation_sim.min_peak_current(wrench, [0, -1, 1, 0, 0, 0])
        self.assertGreater(negative_peak, positive_peak)
        self.assertGreaterEqual(sweep["max_cruise_peak_at"][0], negative_peak)

    def test_reduced_basis_preserves_physical_current_and_wrench_constraints(self):
        matrix = np.array([[1.0, -1.0]])
        basis = np.array([[1.0], [-1.0]])
        peak, currents = levitation_sim.min_peak_current_in_basis(matrix, [1.0], basis)
        np.testing.assert_allclose(matrix @ currents, [1.0], atol=model.Fixed.allocator_residual_tolerance)
        self.assertAlmostEqual(float(currents.sum()), 0)
        self.assertAlmostEqual(float(np.max(np.abs(currents))), peak)

    def test_optional_mode_diagnostic_preserves_full_rank_basis_authority(self):
        wrench = np.column_stack((np.eye(6), -np.ones(6)))
        rows = levitation_sim.local_current_mode_analysis([wrench], [wrench], 1, 1, 1)
        self.assertEqual({row["basis_source"] for row in rows}, {"6dof", "operational", "hybrid"})
        for row in rows:
            self.assertAlmostEqual(row["worst_peak_current_ratio"], 1)

    def test_height_interpolation_rejects_unmeasured_geometry(self):
        measurements = {"coil_height_coupling_heights_mm": [1, 2], "coil_height_coupling_factors": [1, 0.5]}
        self.assertEqual(design.interpolate_height_coupling(measurements, 1.5), 0.75)
        with self.assertRaises(ValueError):
            design.interpolate_height_coupling(measurements, 3)

    def test_reset_population_and_requested_play_rate_fit_the_scenarios(self):
        config = SimpleNamespace(turns=10, resistance=1, bus_voltage=24)
        allocation = {"peak_ampere_turns": 10, "mean_squared_sum": 100, "worst_squared_sum": 200,
                      "mean_absolute_sum": 10, "worst_absolute_sum": 20, "coil_count": 4}
        allocations = {name: allocation for name in ("flight", "reset", "breakaway", "hover")}
        allocations.update(contact_tipping_margin=2, contact_pair={"peak_ampere_turns": 20})
        scenarios = design.MotionScenarios(config, allocations, SimpleNamespace(current_ripple_peak=0))
        for _, total_power, local_power in scenarios.reset_segments:
            self.assertAlmostEqual(total_power, local_power * model.Inputs.piece_count)
        self.assertAlmostEqual(sum(duration for duration, _, _ in scenarios.reset_segments), model.Inputs.reset_duration)
        self.assertGreaterEqual(scenarios.max_composite_moves_per_minute, model.Inputs.sustained_moves_per_minute)

    def test_invalid_reset_phase_timing_is_rejected(self):
        config = SimpleNamespace(turns=10, resistance=1, bus_voltage=24)
        allocation = {"peak_ampere_turns": 10, "mean_squared_sum": 100, "worst_squared_sum": 200,
                      "mean_absolute_sum": 10, "worst_absolute_sum": 20, "coil_count": 4}
        allocations = {name: allocation for name in ("flight", "reset", "breakaway", "hover")}
        with patch.object(model.Inputs, "reset_breakaway_duration", model.Inputs.reset_duration * 2):
            with self.assertRaises(ValueError):
                design.MotionScenarios(config, allocations, SimpleNamespace(current_ripple_peak=0))

    def test_every_declared_parameter_has_a_source_consumer(self):
        sources = list(self.production_sources.values())
        attribute_reads = {node.attr for source in sources for node in ast.walk(source)
                           if isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Load)}
        for definition in sources[0].body:
            if isinstance(definition, ast.ClassDef) and definition.name in ("Inputs", "Fixed", "Constants"):
                for assignment in definition.body:
                    if isinstance(assignment, ast.Assign):
                        self.assertIn(assignment.targets[0].id, attribute_reads)

    def test_neighbour_wrench_preserves_force_and_torque_components(self):
        board = design.BoardGeometry()
        coil = design.CoilBed(board)
        magnets = design.HalbachArray(board)
        piece = design.Piece(board, magnets)
        geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
        levitation_sim.use_geometry(geometry)
        force = levitation_sim.neighbour_force(geometry.magnet_lateral_edge, geometry.magnet_thickness,
                                               geometry.periods_per_side, board.base_diameter / 1000)
        opposite = levitation_sim.neighbour_force(geometry.magnet_lateral_edge, geometry.magnet_thickness,
                                                  geometry.periods_per_side, -board.base_diameter / 1000)
        self.assertEqual(force.shape, (6,))
        np.testing.assert_allclose(force[:3] + opposite[:3], 0, atol=model.Fixed.allocator_residual_tolerance)

    def test_thermal_infeasibility_is_recorded_without_material_extrapolation(self):
        thermal = design.RadiatorCooling.__new__(design.RadiatorCooling)
        thermal.background_power = 0
        thermal.thermal_conductance = 1
        thermal.material_failures = []
        temperature = thermal.steady_temperature(1000000)
        self.assertTrue(np.isinf(temperature))
        self.assertEqual(len(thermal.material_failures), 1)
        with self.assertRaises(ValueError):
            thermal.hot_power_factor(model.Fixed.magnet_max_operating_temperature + 1)

    def test_supply_drawdown_tracks_peak_not_just_end_deficit(self):
        supply = design.PowerSupply.__new__(design.PowerSupply)
        supply.unit_rating = 10
        peak, remaining = supply.trace_drawdown([(1, 20), (1, 0)])
        self.assertEqual((peak, remaining), (10, 0))

    def test_unverified_requirements_prevent_design_acceptance(self):
        checks = design.StatusChecks.__new__(design.StatusChecks)
        checks.results = {"example": True}
        checks.unverified = ("routing",)
        self.assertFalse(checks.accepted)
        checks.unverified = ()
        self.assertTrue(checks.accepted)
        checks.results["example"] = False
        self.assertFalse(checks.accepted)

    def test_no_embedded_float_tuning_values_outside_parameter_classes(self):
        for filename, source in self.production_sources.items():
            for definition in source.body:
                if isinstance(definition, ast.ClassDef) and definition.name in ("Inputs", "Fixed", "Constants"):
                    continue
                for node in ast.walk(definition):
                    if isinstance(node, ast.Constant) and isinstance(node.value, float):
                        self.assertIn(node.value, (0.0, 0.5, 1.0, 1000000.0), f"{filename}:{node.lineno}")

    def test_rl_response_matches_analytic_decay_and_back_emf_equilibrium(self):
        current = np.array([1.0, -1.0])
        resistance, inductance, period = 2, 4, 1
        actual = qualification.rl_step(current, 0, 0, resistance, inductance, period)
        np.testing.assert_allclose(actual, current * np.exp(-resistance * period / inductance))
        back_emf = np.array([3, -3])
        voltage = resistance * current + back_emf
        np.testing.assert_allclose(qualification.rl_step(current, voltage, back_emf, resistance, inductance, period), current)
        first_half = qualification.rl_step(current, voltage, back_emf, resistance, inductance, period / 2)
        np.testing.assert_allclose(qualification.rl_step(first_half, voltage, back_emf, resistance, inductance, period / 2), current)

    def test_pose_observer_recovers_constant_velocity_without_true_velocity_input(self):
        position, velocity, disturbance = np.zeros(6), np.zeros(6), np.zeros(6)
        true_velocity = np.arange(6)
        period = 1 / model.Fixed.pose_feedback_rate
        steps = round(model.Fixed.qualification_recovery_duration / period)
        for step in range(steps):
            measurement = true_velocity * step * period
            position, velocity, disturbance = qualification.observer_update(
                position, velocity, disturbance, measurement, period, np.zeros((1, 6)))
        np.testing.assert_allclose(position, measurement, atol=model.Fixed.allocator_residual_tolerance)
        np.testing.assert_allclose(velocity, true_velocity, atol=model.Fixed.allocator_residual_tolerance)
        np.testing.assert_allclose(disturbance, 0, atol=model.Fixed.allocator_residual_tolerance)

    def test_pose_observer_identifies_unmodeled_acceleration_from_pose_only(self):
        position, velocity, disturbance = np.zeros(6), np.zeros(6), np.zeros(6)
        unmodeled_acceleration = np.arange(6)
        period = 1 / model.Fixed.pose_feedback_rate
        steps = round(model.Fixed.qualification_recovery_duration / period)
        for step in range(steps):
            time = (step + 1) * period
            measurement = unmodeled_acceleration * time ** 2 / 2
            position, velocity, disturbance = qualification.observer_update(
                position, velocity, disturbance, measurement, period, np.zeros((1, 6)))
        np.testing.assert_allclose(position, measurement, atol=model.Fixed.allocator_residual_tolerance)
        np.testing.assert_allclose(velocity, unmodeled_acceleration * time, atol=model.Fixed.allocator_residual_tolerance)
        np.testing.assert_allclose(disturbance, unmodeled_acceleration, atol=model.Fixed.allocator_residual_tolerance)

    def test_pose_prediction_accounts_for_commanded_acceleration_and_sample_age(self):
        accelerations = np.tile(np.arange(6), (4, 1))
        period = 1 / model.Fixed.current_command_rate
        duration = len(accelerations) * period
        position, velocity = qualification.propagate_estimate(np.zeros(6), np.zeros(6), accelerations, period)
        np.testing.assert_allclose(position, np.arange(6) * duration ** 2 / 2)
        np.testing.assert_allclose(velocity, np.arange(6) * duration)
        observed_position, observed_velocity, disturbance = qualification.observer_update(
            np.zeros(6), np.zeros(6), np.zeros(6), position, duration, accelerations)
        np.testing.assert_allclose(observed_position, position)
        np.testing.assert_allclose(observed_velocity, velocity)
        np.testing.assert_allclose(disturbance, 0)

    def test_inactive_coil_freewheels_to_zero_without_reversing_or_induced_short_circuit_current(self):
        currents = np.array([1, -1, 0])
        voltage = -np.sign(currents) * model.Inputs.selected_bus_voltage
        result = qualification.rl_step(currents, voltage, np.ones(3), 1, 1, 1, enabled=False)
        np.testing.assert_array_equal(result, np.zeros(3))
        enabled = qualification.rl_step(currents, voltage, np.ones(3), 1, 1, 1)
        self.assertLess(enabled[0], 0)
        self.assertGreater(enabled[1], 0)
        self.assertNotEqual(enabled[2], 0)

    def test_current_regulator_does_not_wind_up_against_voltage_limit(self):
        error = np.array([1, -1, -1])
        integral = np.array([2, -2, 2])
        voltage, updated, _ = qualification.current_regulator(error, integral, 1, 0, 1)
        np.testing.assert_array_equal(voltage, [1, -1, 1])
        np.testing.assert_array_equal(updated[:2], integral[:2])
        self.assertLess(updated[2], integral[2])

    def test_current_regulator_update_does_not_change_with_pwm_carrier(self):
        arguments = (np.ones(2), np.zeros(2), 1, 0, 1)
        baseline = qualification.current_regulator(*arguments)
        with patch.object(model.Fixed, 'driver_pwm_frequency', model.Fixed.driver_pwm_frequency * model.Fixed.qualification_refinement_factor):
            changed = qualification.current_regulator(*arguments)
        for original, revised in zip(baseline, changed):
            np.testing.assert_array_equal(original, revised)

    def test_switching_ripple_bound_encloses_periodic_rl_waveforms(self):
        bound = design.split_rail_ripple_peak(1, 1, 1, 1)
        for duty in np.linspace(0, 1, 101):
            def cycle(initial):
                peak = qualification.rl_step(initial, 1, 0, 1, 1, duty)
                return qualification.rl_step(peak, -1, 0, 1, 1, 1 - duty)
            zero_response = cycle(0)
            decay = cycle(1) - zero_response
            trough = zero_response / (1 - decay)
            peak = qualification.rl_step(trough, 1, 0, 1, 1, duty)
            mean = 2 * duty - 1
            self.assertLessEqual(max(abs(peak - mean), abs(trough - mean)), bound + model.Fixed.allocator_residual_tolerance)
        self.assertGreaterEqual(bound, np.tanh(1 / 4))
        self.assertLess(design.split_rail_ripple_peak(1, 1, 1, 2), bound)

    def test_convergence_checks_trajectory_and_current_not_only_tip_peak(self):
        gap = model.Inputs.magnet_to_coil_distance / 1000
        position_error = model.Fixed.position_error_gap_fraction * gap
        metrics = dict(peak_tip_error_m=position_error, peak_position_error_m=position_error,
                       peak_requested_current_A=1, peak_actual_current_A=1,
                       peak_copper_loss_W=1, peak_terminal_power_W=1)
        baseline = dict(case=dict(moving=False), metrics=metrics, checks=dict(completed=True),
                        trace=[dict(time_s=0, error=[position_error, 0, 0, 0, 0, 0]),
                               dict(time_s=1, error=[0, 0, 0, 0, 0, 0])])
        identical = qualification.compare_refinement(baseline, baseline, model.Fixed.driver_channel_current, gap)
        self.assertTrue(identical['converged'])
        displaced = json.loads(json.dumps(baseline))
        displaced['trace'][0]['error'][0] *= -1
        comparison = qualification.compare_refinement(baseline, displaced, model.Fixed.driver_channel_current, gap)
        self.assertFalse(comparison['converged'])
        self.assertTrue(comparison['checks']['peak_tip_error_m'])
        self.assertFalse(comparison['checks']['position_trace'])
        changed_current = json.loads(json.dumps(baseline))
        changed_current['metrics']['peak_actual_current_A'] += model.Fixed.driver_channel_current * model.Fixed.qualification_current_reserve_fraction
        self.assertFalse(qualification.compare_refinement(
            baseline, changed_current, model.Fixed.driver_channel_current, gap)['checks']['peak_actual_current_A'])

    def test_blended_allocation_preserves_wrench_and_reports_excess_demand(self):
        matrix = np.column_stack((np.eye(6), -np.ones(6)))
        identities = tuple(range(7))
        target = np.arange(6)
        current_limit = 1
        currents, required = levitation_sim.allocate_window_components({identities: (matrix, 1)}, target, current_limit)
        pattern = np.array([currents[identity] for identity in identities])
        self.assertGreater(required, current_limit)
        self.assertGreater(max(abs(pattern)), current_limit)
        np.testing.assert_allclose(matrix @ pattern, target, atol=model.Fixed.allocator_residual_tolerance)
        self.assertAlmostEqual(sum(pattern), 0)

    def test_candidate_pose_rate_is_explicit_and_separate_from_growth_heuristic(self):
        coil = SimpleNamespace(footprint_area=1)
        config = SimpleNamespace(turns=1, coil_height=1, resistance=1, current_limit=1, usable_drive_voltage=1, bus_voltage=1)
        control = design.Control(coil, config, dict(instability_growth_rate=1))
        self.assertEqual(control.pose_update_rate, model.Fixed.pose_feedback_rate)
        self.assertEqual(control.pose_rate_screen, model.Inputs.control_loop_bandwidth_margin)

    def test_dynamic_case_definitions_are_serializable_and_respect_peak_speed(self):
        board = design.BoardGeometry()
        coil = design.CoilBed(board)
        piece = design.Piece(board, design.HalbachArray(board))
        geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
        cases = qualification.stress_cases(board, piece, geometry)
        self.assertEqual(len(json.loads(json.dumps(cases))), len(cases))
        for case in cases:
            for time in (0, case['duration']):
                _, velocity, _ = qualification.reference_state(case, time)
                self.assertLessEqual(np.linalg.norm(velocity), geometry.flight_speed + model.Fixed.allocator_limit_tolerance)

    def test_dynamic_integrator_preserves_exact_hover_equilibrium(self):
        board = design.BoardGeometry()
        coil = design.CoilBed(board)
        piece = design.Piece(board, design.HalbachArray(board))
        geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
        levitation_sim.use_geometry(geometry)
        config = SimpleNamespace(turns=1, coil_height=model.Fixed.nominal_coil_height_for_field,
                                 resistance=1, current_limit=model.Fixed.driver_channel_current,
                                 bus_voltage=model.Inputs.selected_bus_voltage,
                                 usable_drive_voltage=model.Inputs.selected_bus_voltage * model.Fixed.coil_bus_voltage_fraction
                                 * model.Fixed.usable_bus_voltage_fraction)
        array = levitation_sim.verification_coil_array(coil.control_cells_per_side, config.coil_height / 1000)
        matrix = np.column_stack((np.eye(6), -np.eye(6), np.zeros((6, len(array.identities) - 12))))
        matrix[2] *= geometry.weight / 2
        columns = dict(zip(array.identities, matrix.T))
        case = qualification.stress_cases(board, piece, geometry)[0]
        case.update(pose=[0] * 6, perturb=False, duration=1 / model.Fixed.pose_feedback_rate)
        def fixed_field(layout, selected, center):
            return np.column_stack([columns[identity] for identity in selected.identities])
        update_counts = []
        for refinement in ({}, {'time_refinement': model.Fixed.qualification_refinement_factor},
                           {'field_refinement': model.Fixed.qualification_refinement_factor}):
            with patch.object(levitation_sim, 'actuator_matrix', side_effect=fixed_field) as force_updates, \
                 patch.object(qualification, 'observer_update', wraps=qualification.observer_update) as pose_updates, \
                 patch.object(qualification, 'current_regulator', wraps=qualification.current_regulator) as pwm_updates, \
                 patch.object(levitation_sim, 'allocate_window_components', wraps=levitation_sim.allocate_window_components) as command_updates:
                result = qualification.simulate_case(case, board, coil, piece, config, SimpleNamespace(inductance=1), **refinement)
            self.assertTrue(result['passed'])
            self.assertLess(result['metrics']['peak_position_error_m'], model.Fixed.allocator_residual_tolerance)
            self.assertLess(result['metrics']['peak_tip_error_m'], model.Fixed.allocator_residual_tolerance)
            json.dumps(result)
            update_counts.append((force_updates.call_count, pose_updates.call_count, pwm_updates.call_count, command_updates.call_count))
        self.assertEqual(update_counts[0], update_counts[1])
        self.assertEqual(update_counts[0][1:], update_counts[2][1:])
        self.assertGreater(update_counts[2][0], update_counts[0][0])

    def test_contact_allocation_uses_surface_support_without_losing_contact(self):
        geometry = SimpleNamespace(settings=model.Fixed, weight=1.0, gravity=model.Constants.gravity,
                                   magnets_per_period=model.Inputs.magnets_per_period,
                                   periods_per_side=model.Inputs.periods_per_side,
                                   magnet_lateral_edge=model.Inputs.magnet_lateral_edge / 1000,
                                   base_corner_standoff=model.Fixed.base_corner_standoff / 1000,
                                   center_of_mass_height=0.04)
        levitation_sim.use_geometry(geometry)
        wrench = np.column_stack((np.eye(6), -np.ones(6)))
        friction = model.Fixed.sliding_friction_coefficient
        acceleration = 0.2
        equality, target, inequalities, bounds = levitation_sim.contact_constraints(wrench, [1, 0], friction, acceleration, np.zeros(6))
        peak, initial = levitation_sim.min_peak_current(equality, target, inequalities, bounds)
        currents = levitation_sim.min_loss_current(equality, target, peak, initial_currents=initial,
                                                   contact_inequality=inequalities, contact_bounds=bounds)
        force = wrench @ currents
        normal = geometry.weight - force[2]
        self.assertGreaterEqual(normal, model.Fixed.minimum_contact_normal_fraction * geometry.weight - model.Fixed.allocator_residual_tolerance)
        self.assertAlmostEqual(force[0] - friction * normal, geometry.weight / geometry.gravity * acceleration)
        pressure_center_x = force[4] / normal + geometry.center_of_mass_height * friction
        pressure_center_y = -force[3] / normal
        friction_yaw = pressure_center_y * friction * normal
        self.assertAlmostEqual(force[5] + friction_yaw, 0)
        radius = (geometry.magnets_per_period * geometry.periods_per_side * geometry.magnet_lateral_edge / 2 * np.sqrt(2)
                  + geometry.base_corner_standoff)
        self.assertLessEqual(np.hypot(pressure_center_x, pressure_center_y), radius + model.Fixed.allocator_residual_tolerance)
        self.assertLessEqual(float(np.max(inequalities @ currents - bounds)), model.Fixed.allocator_residual_tolerance)

    def test_numeric_tuning_defaults_are_not_hidden_in_simulator(self):
        source = ast.parse(Path(levitation_sim.__file__).read_text())
        for function in (node for node in ast.walk(source) if isinstance(node, ast.FunctionDef)):
            for default in function.args.defaults:
                if isinstance(default, ast.Constant) and isinstance(default.value, (int, float)):
                    self.assertIn(default.value, (0, 1), function.name)
                self.assertFalse(isinstance(default, ast.Tuple) and any(isinstance(item, ast.Constant) and isinstance(item.value, (int, float)) and item.value != 0 for item in default.elts), function.name)

    def test_contact_balances_external_wrench_and_pressure_centre_friction(self):
        geometry = SimpleNamespace(settings=model.Fixed, weight=1.0, gravity=model.Constants.gravity,
                                   magnets_per_period=model.Inputs.magnets_per_period,
                                   periods_per_side=model.Inputs.periods_per_side,
                                   magnet_lateral_edge=model.Inputs.magnet_lateral_edge / 1000,
                                   base_corner_standoff=model.Fixed.base_corner_standoff / 1000,
                                   center_of_mass_height=0.04)
        levitation_sim.use_geometry(geometry)
        wrench = np.column_stack((np.eye(6), -np.ones(6)))
        direction = np.array([1, 1]) / np.sqrt(2)
        external = np.array([0.03, -0.02, 0.1, 0.001, -0.002, 0.003])
        friction = model.Fixed.sliding_friction_coefficient
        acceleration = 0.2
        equality, target, inequalities, bounds = levitation_sim.contact_constraints(wrench, direction, friction, acceleration, external)
        _, currents = levitation_sim.min_peak_current(equality, target, inequalities, bounds)
        applied = wrench @ currents + external
        normal = geometry.weight - applied[2]
        contact_force = np.append(-friction * normal * direction, normal)
        pressure_position = np.array([applied[4] / normal + geometry.center_of_mass_height * friction * direction[0],
                                      -applied[3] / normal + geometry.center_of_mass_height * friction * direction[1],
                                      -geometry.center_of_mass_height])
        np.testing.assert_allclose(applied[3:] + np.cross(pressure_position, contact_force), np.zeros(3),
                                   atol=model.Fixed.allocator_residual_tolerance)
        np.testing.assert_allclose(applied[:3] + contact_force - [0, 0, geometry.weight],
                                   np.append(geometry.weight / geometry.gravity * acceleration * direction, 0),
                                   atol=model.Fixed.allocator_residual_tolerance)

    def test_diagnostic_window_shift_keeps_coils_on_the_original_lattice(self):
        board = design.BoardGeometry()
        coil = design.CoilBed(board)
        piece = design.Piece(board, design.HalbachArray(board))
        geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
        levitation_sim.use_geometry(geometry)
        array = levitation_sim.coil_array_from_geometry(coil.control_cells_per_side, geometry.coil_height, 1,
                                                        model.Fixed.verification_force_mesh, model.Fixed.coil_radial_filaments,
                                                        model.Fixed.minimum_axial_filaments)
        vertices = [winding[0].vertices.copy() for winding in array.coils]
        array.build()
        for original, rebuilt in zip(vertices, array.coils):
            np.testing.assert_array_equal(original, rebuilt[0].vertices)
        array.build(center=(geometry.coil_long / 2, geometry.coil_long / 2))
        self.assertEqual(len(array.coils), len(vertices))
        per_family = len(vertices) // model.Fixed.herringbone_orientation_families
        for index, (original, shifted) in enumerate(zip(vertices, array.coils)):
            displacement = shifted[0].vertices - original
            np.testing.assert_allclose(displacement, np.tile(displacement[0], (len(original), 1)),
                                       atol=model.Fixed.allocator_residual_tolerance)
            pitches = (geometry.coil_short, geometry.coil_long) if index < per_family else (geometry.coil_long, geometry.coil_short)
            lattice_steps = displacement[0, :2] / pitches
            np.testing.assert_allclose(lattice_steps, np.round(lattice_steps), atol=model.Fixed.allocator_residual_tolerance)
            self.assertEqual(displacement[0, 2], 0)

    def test_drag_breakdown_matches_total(self):
        geometry = SimpleNamespace(settings=model.Fixed, constants=model.Constants,
                                   magnets_per_period=model.Inputs.magnets_per_period,
                                   magnet_lateral_edge=model.Inputs.magnet_lateral_edge / 1000)
        levitation_sim.use_geometry(geometry)
        with patch.object(levitation_sim, "eddy_image_force", return_value=1):
            components = levitation_sim.conductive_drag_components(1, 0.004, 0.003)
            self.assertEqual(set(components), {"pcb_copper", "slotted_plate", "plate_web"})
            self.assertAlmostEqual(sum(components.values()), levitation_sim.conductive_drag(1, 0.004, 0.003))

    def test_different_window_sizes_use_identical_physical_coils(self):
        board = design.BoardGeometry()
        coil = design.CoilBed(board)
        piece = design.Piece(board, design.HalbachArray(board))
        geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
        levitation_sim.use_geometry(geometry)
        arrays = [levitation_sim.coil_array_from_geometry(size, geometry.coil_height, 1,
                                                         model.Fixed.verification_force_mesh, model.Fixed.coil_radial_filaments,
                                                         model.Fixed.minimum_axial_filaments)
                  for size in (coil.control_cells_per_side, coil.control_cells_per_side + 1)]
        larger = dict(zip(arrays[1].identities, arrays[1].coils))
        for identity, winding in zip(arrays[0].identities, arrays[0].coils):
            np.testing.assert_array_equal(winding[0].vertices, larger[identity][0].vertices)
        repeat = geometry.coil_lattice_repeat
        np.testing.assert_allclose([repeat / geometry.coil_short, repeat / geometry.coil_long],
                                   np.round([repeat / geometry.coil_short, repeat / geometry.coil_long]))
        phases = levitation_sim.coil_window_phases()
        self.assertTrue(np.all(phases >= 0) and np.all(phases < repeat))
        self.assertTrue(set(levitation_sim.coil_window_handoffs()).issubset(phases))

    def test_lattice_repeat_handles_fractional_coil_pitch(self):
        with patch.object(model.Inputs, "coils_per_period", 3):
            coil = design.CoilBed(design.BoardGeometry())
        self.assertAlmostEqual(coil.lattice_repeat / coil.outer_width, round(coil.lattice_repeat / coil.outer_width))
        self.assertAlmostEqual(coil.lattice_repeat / coil.outer_length, round(coil.lattice_repeat / coil.outer_length))
        self.assertEqual(coil.lattice_repeat, 60)

    def test_coil_handoff_preserves_shared_physical_channel_positions(self):
        board = design.BoardGeometry()
        coil = design.CoilBed(board)
        piece = design.Piece(board, design.HalbachArray(board))
        geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
        levitation_sim.use_geometry(geometry)
        array = levitation_sim.coil_array_from_geometry(coil.control_cells_per_side, geometry.coil_height, 1,
                                                        model.Fixed.verification_force_mesh, model.Fixed.coil_radial_filaments,
                                                        model.Fixed.minimum_axial_filaments)
        offset = min(model.Fixed.active_window_handoff_offsets)
        array.build(center=(geometry.coil_short / 2 - offset, 0))
        before = dict(zip(array.identities, array.coils))
        array.build(center=(geometry.coil_short / 2 + offset, 0))
        after = dict(zip(array.identities, array.coils))
        self.assertNotEqual(set(before), set(after))
        self.assertEqual(len(before), len(after))
        for identity in before.keys() & after.keys():
            np.testing.assert_array_equal(before[identity][0].vertices, after[identity][0].vertices)

    def test_crossed_handoffs_blend_on_one_physical_lattice(self):
        board = design.BoardGeometry()
        coil = design.CoilBed(board)
        piece = design.Piece(board, design.HalbachArray(board))
        geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
        levitation_sim.use_geometry(geometry)
        array = levitation_sim.verification_coil_array(coil.control_cells_per_side, geometry.coil_height)
        position = (geometry.coil_short / 2, geometry.coil_long / 2)
        selections = levitation_sim.blended_window_centers(position)
        self.assertEqual(len(selections), 4)
        self.assertAlmostEqual(sum(weight for _, weight in selections), 1)
        physical_coils = {}
        for center, weight in selections:
            self.assertGreater(weight, 0)
            array.build(center=center)
            for identity, winding in zip(array.identities, array.coils):
                if identity in physical_coils:
                    np.testing.assert_array_equal(winding[0].vertices, physical_coils[identity])
                physical_coils[identity] = winding[0].vertices
        self.assertGreater(len(physical_coils), coil.control_bed_bodies)
        self.assertLessEqual(len(physical_coils), coil.control_bed_bodies * model.Inputs.drive_look_ahead_factor)
        width = levitation_sim.coil_window_transition_width()
        for sign in (-1, 1):
            endpoint = (position[0] + sign * width / 2, 0)
            weights = [weight for _, weight in levitation_sim.blended_window_centers(endpoint)]
            self.assertAlmostEqual(sum(weights), 1)
            self.assertAlmostEqual(max(weights), 1)

    def test_blended_window_components_reuse_each_physical_coil_once(self):
        board = design.BoardGeometry()
        coil = design.CoilBed(board)
        piece = design.Piece(board, design.HalbachArray(board))
        geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
        levitation_sim.use_geometry(geometry)
        array = levitation_sim.verification_coil_array(coil.control_cells_per_side, geometry.coil_height)
        position = (geometry.coil_short / 2, geometry.coil_long / 2)
        def indexed_matrix(layout, selected, center):
            return np.tile(np.arange(len(selected.coils)), (6, 1))
        with patch.object(levitation_sim, 'actuator_matrix', side_effect=indexed_matrix) as evaluate:
            components = levitation_sim.window_components(None, array, position, np.zeros(3))
        self.assertEqual(evaluate.call_count, 1)
        self.assertAlmostEqual(sum(weight for _, weight in components.values()), 1)
        indices = {identity: index for index, identity in enumerate(array.identities)}
        for identities, (matrix, _) in components.items():
            np.testing.assert_array_equal(matrix[0], [indices[identity] for identity in identities])

    def test_board_edge_keeps_a_complete_window_of_whole_coil_bodies(self):
        board = design.BoardGeometry()
        coil = design.CoilBed(board)
        piece = design.Piece(board, design.HalbachArray(board))
        geometry = levitation_sim.SimGeometry(model.Inputs, model.Fixed, model.Constants, board, coil, piece)
        levitation_sim.use_geometry(geometry)
        array = levitation_sim.coil_array_from_geometry(coil.control_cells_per_side, geometry.coil_height, 1,
                                                        model.Fixed.verification_force_mesh, model.Fixed.coil_radial_filaments,
                                                        model.Fixed.minimum_axial_filaments)
        half_extents = np.array([board.motor_width, board.motor_height]) / 2000
        array.build(center=half_extents)
        unbounded = dict(zip(array.identities, array.coils))
        array.build(center=half_extents, half_extents=half_extents)
        self.assertGreater(len(array.coils), 0)
        self.assertEqual(len(array.coils), len(unbounded))
        self.assertNotEqual(set(array.identities), set(unbounded))
        for identity, winding in zip(array.identities, array.coils):
            first = winding[0].vertices[:, :2]
            center = (first.max(axis=0) + first.min(axis=0)) / 2
            half_size = np.array([geometry.coil_short, geometry.coil_long] if identity[0] == "x"
                                 else [geometry.coil_long, geometry.coil_short]) / 2
            self.assertTrue(np.all(np.abs(center) + half_size <= half_extents + model.Fixed.allocator_limit_tolerance))


if __name__ == "__main__":
    unittest.main()

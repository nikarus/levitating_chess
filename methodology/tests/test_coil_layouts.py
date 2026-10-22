import unittest
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Inputs, Fixed
from methodology import coil_layouts, design


class CoilLayoutTests(unittest.TestCase):
    def setUp(self):
        coil_layouts.reference_state()

    def test_paired_wiring_is_local_complete_and_fixed_across_windows(self):
        identities = [('x', x, y) for x in range(-4, 4) for y in range(-4, 4)]
        basis = coil_layouts.driver_connection_basis(identities, 'series_same')
        np.testing.assert_array_equal(np.sum(basis != 0, axis=0), 2)
        np.testing.assert_array_equal(np.sum(basis != 0, axis=1), 1)
        pairs = []
        for column in basis.T:
            pair = [identities[index] for index in np.flatnonzero(column)]
            self.assertEqual(sum(abs(a - b) for a, b in zip(pair[0][1:], pair[1][1:])), 1)
            pairs.append(set(pair))
        reduced = identities[1:]
        shifted = coil_layouts.driver_connection_basis(reduced, 'series_same')
        self.assertEqual(shifted.shape[1], basis.shape[1] - 1)
        for column in shifted.T:
            self.assertIn({reduced[index] for index in np.flatnonzero(column)}, pairs)

    def test_series_connections_preserve_power_and_driver_midpoint_balance(self):
        identities = [('x', x, y) for x in range(2) for y in range(2)]
        for connection in ('series_same', 'series_opposed'):
            basis = coil_layouts.driver_connection_basis(identities, connection)
            currents = np.array([1., -1.])
            physical = basis @ currents
            self.assertEqual(currents.sum(), 0)
            self.assertEqual(physical @ physical, np.sum(basis ** 2, axis=0) @ currents ** 2)
            np.testing.assert_array_equal(np.sum(basis, axis=0), 0 if connection == 'series_opposed' else 2)

    def test_channel_pilot_reuses_round_wire_for_every_candidate(self):
        from methodology import prototype_coils
        for _, name, angle, spacing, connection in Inputs.channel_layout_candidates:
            candidate, coil, winding = prototype_coils.prototype_geometry(name)
            array = coil_layouts.LayoutArray(candidate[1], coil, winding, angle, 1, spacing)
            coil_layouts.validate_layout(array)
            basis = coil_layouts.driver_connection_basis(array.identities, connection)
            self.assertGreater(basis.shape[1], 0)
            self.assertEqual(winding.turns, Inputs.prototype_radial_turns * Inputs.prototype_axial_layers)

    def test_pose_screen_reproduces_retained_level_reference(self):
        from methodology import prototype_coils
        from methodology.cad import magnetics
        construction = magnetics.ConstructionModel()
        geometry, _, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
        magnet = levitation_sim.magnet_layout_from_geometry()
        name, layout, angle, spacing, _ = Inputs.channel_layout_candidates[0]
        candidate, coil, winding = prototype_coils.prototype_geometry(layout)
        array = coil_layouts.LayoutArray(candidate[1], coil, winding, angle, 1, spacing)
        saved = json.loads((Path(__file__).parents[1] / 'results/channel_layouts/wider_support/summary.json').read_text())
        array.window = saved['support_window_mm'] / 1000
        expected = next(row for row in saved['results'] if row['name'] == name)['rows'][0]
        actual, _ = coil_layouts.channel_pose_case(array, winding, geometry, magnet, expected['phase'],
            geometry.max_flight_gap, expected['yaw_deg'], 0., 0.)
        self.assertEqual(actual['augmented_rank'], expected['augmented_rank'])
        np.testing.assert_allclose([d['minimum_peak_current_A'] for d in actual['demands']],
            [d['minimum_peak_current_A'] for d in expected['demands']], rtol=Fixed.allocator_residual_tolerance)

    def test_rotation_requests_use_world_inertia_at_tilted_pose(self):
        geometry, _ = coil_layouts.reference_state()
        magnet = levitation_sim.magnet_layout_from_geometry()
        levitation_sim.place_piece(magnet, roll=0.2, pitch=-0.1, yaw=0.4)
        rotation = magnet.collection.orientation.as_matrix()
        inertia = rotation @ np.diag(geometry.inertias) @ rotation.T
        targets = dict(coil_layouts.channel_wrench_targets(geometry, magnet, geometry.max_flight_gap))
        for axis in range(3):
            acceleration = np.linalg.solve(inertia, targets[f'torque_{axis}_1'][3:])
            self.assertGreater(acceleration[axis], 0)
            np.testing.assert_allclose(np.delete(acceleration, axis), 0, atol=Fixed.allocator_residual_tolerance)
            np.testing.assert_allclose(targets[f'torque_{axis}_1'][3:], -targets[f'torque_{axis}_-1'][3:])

    def test_hard_pose_collection_retains_extrema_and_source_locations(self):
        report = json.loads((Path(__file__).parents[1] / 'results/channel_layouts/pose_comparison/summary.json').read_text())
        poses = coil_layouts.channel_pose_collection(report)
        keys = [json.dumps(coil_layouts.channel_case_specification(pose)) for pose in poses]
        self.assertEqual(len(keys), len(set(keys)))
        self.assertTrue(any(len(pose['origins']) > 1 for pose in poses))
        for result in report['results']:
            rows = result['coarse'] + result['spatial'] + result['angular_height_refinement']
            for scope, subset in (('all_poses', rows), ('level_only', [row for row in rows if row['tilt_fraction'] == 0])):
                for metric in ('minimum_peak_current_A', 'copper_loss_W'):
                    expected = max(demand[metric] for row in subset for demand in row['demands'] if demand[metric] is not None)
                    origins = [origin for pose in poses for origin in pose['origins']
                        if origin['candidate'] == result['name'] and origin['scope'] == scope and origin['criterion'] == metric]
                    self.assertEqual(len(origins), 1)
                    self.assertEqual(origins[0]['observed_value'], expected)
        for pose in poses:
            for origin in pose['origins']:
                result = next(result for result in report['results'] if result['name'] == origin['candidate'])
                row = result[origin['stage']][origin['row_index']]
                self.assertEqual(coil_layouts.channel_case_specification(pose), coil_layouts.channel_case_specification(row))

    def test_hard_pose_collection_rejects_incomplete_evidence(self):
        with self.assertRaisesRegex(ValueError, 'completed static comparison'):
            coil_layouts.channel_pose_collection(dict(status='incomplete'))

    def test_sweep_grid_preserves_wire_fit_and_nonoverlap(self):
        from methodology import prototype_coils
        candidates = coil_layouts.channel_sweep_candidates()
        self.assertEqual(len(candidates), len({row['name'] for row in candidates}))
        for row in candidates:
            coil, winding = prototype_coils.round_wire_geometry(row['width_mm'], row['length_mm'])
            array = coil_layouts.LayoutArray(row['pattern'], coil, winding, row['rotation_deg'],
                min(Fixed.current_demand_mesh_refinements), row['spacing_mm'])
            coil_layouts.validate_layout(array)
            self.assertEqual(winding.turns, Inputs.prototype_radial_turns * Inputs.prototype_axial_layers)
            self.assertGreaterEqual(winding.inner_window_width, 2 * Inputs.prototype_inner_corner_radius)
            self.assertGreaterEqual(winding.inner_window_length, 2 * Inputs.prototype_inner_corner_radius)
            self.assertGreater(coil_layouts.channel_density(array, row['connection']), 0)

    def test_generalized_pose_solver_matches_recorded_pair_currents_and_losses(self):
        from methodology import prototype_coils
        from methodology.cad import magnetics
        construction = magnetics.ConstructionModel()
        geometry, _, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
        saved = json.loads((Path(__file__).parents[1] / 'results/channel_layouts/wider_support/summary.json').read_text())
        for name, layout, angle, spacing, connection in Inputs.channel_layout_candidates:
            if connection == 'independent':
                continue
            candidate, coil, winding = prototype_coils.prototype_geometry(layout)
            array = coil_layouts.LayoutArray(candidate[1], coil, winding, angle, min(Fixed.current_demand_mesh_refinements), spacing)
            array.window = saved['support_window_mm'] / 1000
            expected = next(row for row in saved['results'] if row['name'] == name)['rows'][0]
            magnet = levitation_sim.magnet_layout_from_geometry()
            actual, _ = coil_layouts.channel_pose_case(array, winding, geometry, magnet, expected['phase'],
                geometry.max_flight_gap, expected['yaw_deg'], 0., 0., connection)
            self.assertEqual(actual['augmented_rank'], expected['augmented_rank'])
            self.assertEqual(actual['channels'], expected['channels'])
            for old, new in zip(expected['demands'], actual['demands']):
                self.assertEqual(old['current_reserve_met'], new['current_reserve_met'])
                for key in ('minimum_peak_current_A', 'copper_loss_W'):
                    if old[key] is None:
                        self.assertIsNone(new[key])
                    else:
                        np.testing.assert_allclose(new[key], old[key], rtol=Fixed.allocator_residual_tolerance)

    def test_stacked_reference_reproduces_existing_physical_coils(self):
        candidate = Inputs.coil_layout_candidates[0]
        coil, winding = coil_layouts.candidate_geometry(candidate)
        candidate_array = coil_layouts.LayoutArray(candidate[1], coil, winding, 0, 1)
        original = levitation_sim.verification_coil_array(Inputs.control_cells_per_side, winding.coil_height / 1000)
        original_coils = dict(zip(original.identities, original.coils))
        self.assertEqual(set(candidate_array.identities), set(original.identities))
        for identity, filaments in zip(candidate_array.identities, candidate_array.coils):
            for actual, expected in zip(filaments, original_coils[identity]):
                np.testing.assert_allclose(actual.vertices, expected.vertices, atol=Fixed.allocator_limit_tolerance)
                self.assertAlmostEqual(actual.current, expected.current * winding.turns)

    def test_single_layer_patterns_have_no_overlapping_envelopes(self):
        for candidate in Inputs.coil_layout_candidates[1:]:
            coil, winding = coil_layouts.candidate_geometry(candidate)
            array = coil_layouts.LayoutArray(candidate[1], coil, winding, 0, 1)
            for phase in np.linspace(0, array.repeat, Fixed.layout_screen_phase_samples):
                array.build(center=(phase, phase / 2))
                coil_layouts.validate_layout(array)
                self.assertEqual({part['top'] for part in array.placements}, {-Inputs.magnet_to_coil_distance / 1000})

    def test_parquet_repeats_without_changing_physical_coil_identity(self):
        candidate = next(row for row in Inputs.coil_layout_candidates if row[1] == 'herringbone')
        coil, winding = coil_layouts.candidate_geometry(candidate)
        array = coil_layouts.LayoutArray(candidate[1], coil, winding, 0, 1)
        original = {identity: filaments[0].vertices.copy() for identity, filaments in zip(array.identities, array.coils)}
        array.build(center=(array.short, array.short))
        for identity, filaments in zip(array.identities, array.coils):
            if identity in original:
                np.testing.assert_array_equal(original[identity], filaments[0].vertices)
        array.build(center=(array.repeat, 0))
        shifted = sorted(tuple(np.round(filaments[0].vertices[0] - [array.repeat, 0, 0], 12)) for filaments in array.coils)
        expected = sorted(tuple(np.round(vertices[0], 12)) for vertices in original.values())
        self.assertEqual(shifted, expected)

    def test_wider_wall_changes_copper_area_and_resistance_in_shared_model(self):
        baseline = design.WindingGeometry(design.CoilGeometry(10, 10, 1), Inputs.selected_conductor_thickness, Inputs.selected_winding_layers)
        wider = design.WindingGeometry(design.CoilGeometry(10, 10, 2), Inputs.selected_conductor_thickness, Inputs.selected_winding_layers)
        self.assertEqual(baseline.turns, wider.turns)
        self.assertEqual(baseline.coil_height, wider.coil_height)
        self.assertGreater(wider.cross_section_area, baseline.cross_section_area)
        self.assertLess(wider.resistance, baseline.resistance)

    def test_minimum_loss_handles_a_pose_exactly_at_its_peak_current_limit(self):
        geometry, _ = coil_layouts.reference_state()
        candidate = Inputs.coil_layout_candidates[0]
        coil, winding = coil_layouts.candidate_geometry(candidate)
        array = coil_layouts.LayoutArray(candidate[1], coil, winding, 0, 1)
        position = np.array([1 / 8, 7 / 8]) * array.repeat
        array.build(center=position)
        magnet = levitation_sim.magnet_layout_from_geometry()
        _, center = levitation_sim.place_piece(magnet, x=position[0], y=position[1],
                                               z=geometry.max_flight_gap - geometry.gap, yaw=np.pi / 8)
        weight, com, br, _ = coil_layouts.stress_parameters(geometry, winding, True)
        matrix = coil_layouts.stressed_wrench(levitation_sim.actuator_matrix(magnet, array, center), geometry, com, br)
        response = levitation_sim.conductive_sheet_response(geometry.max_flight_gap, array.height)
        drag = sum(levitation_sim.drag_components_at_speed(geometry.flight_speed, response).values())
        thrust = weight / geometry.gravity * geometry.cruise_accels[0] + drag * br ** 2
        target = np.array([-thrust / np.sqrt(2), thrust / np.sqrt(2), weight, 0, 0, 0])
        peak, initial = levitation_sim.min_peak_current(matrix, target)
        currents = levitation_sim.min_loss_current(matrix, target, peak, initial_currents=initial)
        np.testing.assert_allclose(matrix @ currents, target, atol=Fixed.allocator_residual_tolerance)
        self.assertAlmostEqual(float(currents.sum()), 0)
        self.assertLessEqual(float(np.max(np.abs(currents))), peak + Fixed.allocator_limit_tolerance)
        self.assertLessEqual(float(currents @ currents), float(initial @ initial) + Fixed.allocator_residual_tolerance)


if __name__ == '__main__':
    unittest.main()

import unittest

import numpy as np

import levitation_sim
from model import Inputs, Fixed
from methodology import coil_layouts, design


class CoilLayoutTests(unittest.TestCase):
    def setUp(self):
        coil_layouts.reference_state()

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

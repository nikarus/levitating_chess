import unittest

import numpy as np

import levitation_sim
from model import Inputs, Fixed
from methodology import coil_layouts, prototype_coils


class PrototypeCoilTests(unittest.TestCase):
    def setUp(self):
        coil_layouts.reference_state()

    def test_worst_case_insulated_wire_fits_radially_and_axially(self):
        for name, _ in Inputs.prototype_layouts:
            _, coil, winding = prototype_coils.prototype_geometry(name)
            occupied_width = (winding.turns_per_layer - 1) * winding.wire.radial_pitch + Fixed.prototype_wire_outer_diameter
            occupied_height = (winding.layers - 1) * winding.wire.axial_pitch + Fixed.prototype_wire_outer_diameter
            self.assertLess(occupied_width, coil.winding_radial_width)
            self.assertLess(occupied_height, winding.coil_height)
            self.assertEqual(winding.layers % 2, 0)
            self.assertEqual(winding.turns, Inputs.prototype_radial_turns * Inputs.prototype_axial_layers)
            self.assertAlmostEqual(winding.resistance, winding.length_per_winding * max(Fixed.prototype_wire_resistance_per_m))

    def test_rounded_filament_length_converges_to_winding_perimeter(self):
        _, coil, winding = prototype_coils.prototype_geometry(Inputs.prototype_layouts[0][0])
        wall = coil.winding_radial_width
        errors = []
        for refinement in Fixed.current_demand_mesh_refinements:
            outline = levitation_sim.rounded_rectangle_outline(coil.outer_width - wall, coil.outer_length - wall,
                        coil.corner_radius - wall / 2, Fixed.prototype_corner_segments * refinement)
            length = np.linalg.norm(np.diff(outline, axis=0), axis=1).sum()
            errors.append(abs(length - winding.average_length_per_turn))
            np.testing.assert_array_equal(outline[0], outline[-1])
        self.assertLess(errors[-1], errors[0])

    def test_tapered_peak_allocation_preserves_physical_midpoint_balance(self):
        matrix = np.array([[1.0, -1.0, 0.0]])
        factors = np.array([1.0, 0.2, 1.0])
        peak, initial = levitation_sim.min_peak_current(matrix, [1.0], current_factors=factors)
        self.assertAlmostEqual(peak, 5 / 6)
        np.testing.assert_allclose(matrix @ initial, [1], atol=Fixed.allocator_residual_tolerance)
        self.assertAlmostEqual(float(initial.sum()), 0)
        current = levitation_sim.min_loss_current(matrix, [1], factors, initial_currents=initial)
        np.testing.assert_allclose(current, [0.8, -0.2, -0.6], atol=Fixed.allocator_residual_tolerance)

    def test_outgoing_coil_current_vanishes_with_its_capacity(self):
        matrix = np.array([[1.0, 0.0, -1.0]])
        for capacity in (0.1, 0.01, 0.001):
            factors = np.array([1, 1, capacity])
            _, initial = levitation_sim.min_peak_current(matrix, [1], current_factors=factors)
            current = levitation_sim.min_loss_current(matrix, [1], factors, initial_currents=initial)
            self.assertLessEqual(abs(current[-1]), capacity + Fixed.allocator_limit_tolerance)
            np.testing.assert_allclose(matrix @ current, [1], atol=Fixed.allocator_residual_tolerance)
            self.assertAlmostEqual(float(current.sum()), 0)

    def test_spaced_layouts_keep_physical_coils_fixed_during_handoffs(self):
        for name, _ in Inputs.prototype_layouts:
            array = prototype_coils.prototype_array(name)
            original = {identity: filaments[0].vertices.copy() for identity, filaments in zip(array.identities, array.coils)}
            position = array.rotation.apply([array.short_pitch / 2, array.short_pitch / 2, 0])[:2]
            array.build(center=position)
            coil_layouts.validate_layout(array)
            factors = prototype_coils.activation_factors(array, position)
            self.assertTrue(np.all((factors >= 0) & (factors <= 1)))
            for identity, filaments in zip(array.identities, array.coils):
                if identity in original:
                    np.testing.assert_array_equal(original[identity], filaments[0].vertices)

    def test_voltage_bound_covers_inductance_uncertainty_despite_cancellation(self):
        times = np.array([0., 1., 2.])
        currents = np.array([[2., -2.], [1., -1.], [0., 0.]])
        back_emf = np.zeros_like(currents)
        slopes, _, bound = prototype_coils.electrical_trace(currents, times, back_emf, 1., 1.)
        for factor in Fixed.qualification_inductance_factors:
            voltage = currents + factor * slopes + back_emf
            self.assertTrue(np.all(np.abs(voltage) <= bound))


if __name__ == '__main__':
    unittest.main()

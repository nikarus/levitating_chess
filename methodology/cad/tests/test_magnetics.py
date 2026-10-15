import unittest

import numpy as np

import levitation_sim
from model import Inputs, Fixed, Constants
from scipy.spatial.transform import Rotation
from methodology import coil_layouts
from methodology.cad import magnetics


class CadMagneticTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.construction = magnetics.ConstructionModel()

    def setUp(self):
        self.construction.use_cad_mass = True
        self.construction.return_depth = 0

    def test_conductor_closes_and_length_converges_to_cad(self):
        errors = []
        for refinement in Fixed.current_demand_mesh_refinements:
            segment_length = Fixed.winding_magnetic_segment_length / refinement
            vertices = magnetics.sampled_path(self.construction.path, segment_length)
            errors.append(self.construction.path.length - np.linalg.norm(np.diff(vertices, axis=0), axis=1).sum())
            self.assertLessEqual(np.max(np.linalg.norm(np.diff(vertices, axis=0), axis=1)),
                                 segment_length + Fixed.cad_geometry_tolerance)
            closed = magnetics.closed_conductor(self.construction.path, segment_length, Fixed.pcb_thickness)
            np.testing.assert_array_equal(closed[0], closed[-1])
            self.assertAlmostEqual(closed[:, 2].min(), -Fixed.pcb_thickness / 1000)
        self.assertGreater(errors[0], 0)
        self.assertLess(errors[-1], errors[0])

    def test_array_uses_cad_placement_and_one_ampere_through_all_turns(self):
        geometry, array, _ = self.construction(Inputs.cad_coil_layout, 1)
        for placement, filaments in zip(array.placements, array.coils):
            loop, = filaments
            local = array.rotation.inv().apply(loop.vertices)
            expected = array.conductor_vertices + [placement['x'], placement['y'], placement['top'] - array.height]
            np.testing.assert_allclose(local, expected, atol=Fixed.cad_geometry_tolerance / 1000, rtol=0)
            self.assertEqual(loop.current, 1)
            self.assertLess(loop.vertices[:, 2].max(), -geometry.gap)
        original = {identity: coil[0].vertices.copy() for identity, coil in zip(array.identities, array.coils)}
        array.build(center=array.rotation.apply([array.short_pitch / 2, 0, 0])[:2])
        for identity, coil in zip(array.identities, array.coils):
            if identity in original:
                np.testing.assert_array_equal(coil[0].vertices, original[identity])

    def test_cad_mass_and_both_tilt_inertias_reach_the_simulator(self):
        geometry, _, configuration = self.construction(Inputs.cad_coil_layout, 1)
        properties = self.construction.piece_properties
        self.assertAlmostEqual(geometry.weight, properties['mass_g'] / 1000 * Constants.gravity)
        self.assertAlmostEqual(geometry.center_of_mass_height, properties['center_mm'][2] / 1000)
        np.testing.assert_array_equal(geometry.inertias, np.diag(properties['inertia_kg_m2']))
        magnet = levitation_sim.magnet_layout_from_geometry()
        _, center = levitation_sim.place_piece(magnet)
        self.assertEqual(center[2], geometry.center_of_mass_height)
        self.assertAlmostEqual(configuration.length_per_winding, self.construction.path.length / 1000)
        self.construction.use_cad_mass = False
        old_geometry, _, _ = self.construction(Inputs.cad_coil_layout, 1)
        reference, _ = coil_layouts.reference_state()
        self.assertEqual(old_geometry.weight, reference.weight)
        self.assertNotEqual(old_geometry.center_of_mass_height, geometry.center_of_mass_height)

    def test_curved_force_samples_follow_coil_transforms_and_reject_vertex_edits(self):
        _, array, _ = self.construction(Inputs.cad_coil_layout, 1)
        first = array.create_force_quadrature()
        self.assertEqual(array.force_order, Fixed.winding_force_quadrature_order)
        count = first.filament_bounds[1]
        rotation = Rotation.from_euler('xyz', np.radians(Fixed.motion_check_orientation_deg))
        displacement = np.array([array.short_pitch, -array.short_pitch, array.height])
        coil = array.coils[0][0]
        coil.rotate(rotation, anchor=np.zeros(3))
        coil.move(displacement)
        second = array.create_force_quadrature()
        np.testing.assert_allclose(second.points[:count], rotation.apply(first.points[:count]) + displacement,
            atol=Fixed.singular_value_tolerance, rtol=0)
        np.testing.assert_allclose(second.vectors[:count], rotation.apply(first.vectors[:count]),
            atol=Fixed.singular_value_tolerance, rtol=0)
        np.testing.assert_array_equal(second.points[count:], first.points[count:])
        self.assertFalse(first.matches(array.coils))
        coil.vertices = coil.vertices + displacement
        with self.assertRaisesRegex(ValueError, 'edited conductor vertices'):
            array.create_force_quadrature()

    def test_production_force_uses_curved_samples_and_refreshes_after_handoff(self):
        _, array, _ = self.construction(Inputs.cad_coil_layout, 1)
        layout = levitation_sim.magnet_layout_from_geometry()
        _, center = levitation_sim.place_piece(layout)
        field = lambda source, points: np.broadcast_to(np.ones(3), points.shape)
        expected = array.create_force_quadrature().matrix(layout.collection, center, field)
        actual = levitation_sim.actuator_matrix(layout, array, center, field)
        np.testing.assert_array_equal(actual, expected)
        before = array.force_quadrature
        array.build(center=array.rotation.apply([array.short_pitch / 2, 0, 0])[:2])
        expected = array.create_force_quadrature().matrix(layout.collection, center, field)
        np.testing.assert_array_equal(levitation_sim.actuator_matrix(layout, array, center, field), expected)
        self.assertIsNot(array.force_quadrature, before)


if __name__ == '__main__':
    unittest.main()

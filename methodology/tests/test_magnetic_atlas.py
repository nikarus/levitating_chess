from types import SimpleNamespace
from pathlib import Path
import tempfile
import unittest

import magpylib as magpy
import numpy as np
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Fixed
from methodology.magnetic_atlas import MagneticFieldAtlas


class MagneticAtlasTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        levitation_sim.use_geometry(SimpleNamespace(settings=Fixed))
        cls.source = magpy.Collection(magpy.magnet.Cuboid(dimension=(0.01, 0.01, 0.005),
            polarization=(0.2, -0.3, 1), position=(0, 0, 0.0025)))
        cls.bounds = np.array([[-0.004, -0.004, -0.012], [0.004, 0.004, -0.009]])
        cls.atlas = MagneticFieldAtlas(cls.source, cls.bounds, min(Fixed.atlas_grid_spacings), min(Fixed.atlas_spline_orders))

    def test_off_grid_and_domain_boundary_fields_match_direct(self):
        random = np.random.default_rng(Fixed.atlas_validation_seed)
        points = random.uniform(*self.bounds, size=(Fixed.atlas_validation_points, 3))
        points = np.vstack((points, self.bounds))
        actual = self.atlas.field(self.source, points)
        expected = self.source.getB(points)
        self.assertLess(np.max(np.linalg.norm(actual - expected, axis=1)), Fixed.atlas_field_tolerance)

    def test_grid_excludes_magnet_interior_and_rejects_crossing_domain(self):
        axes, values = MagneticFieldAtlas.sample_grid(self.source, self.bounds, max(Fixed.atlas_grid_spacings))
        self.assertLess(axes[2][-1], 0)
        self.assertTrue(np.isfinite(values).all())
        with self.assertRaisesRegex(ValueError, 'below'):
            MagneticFieldAtlas.sample_grid(self.source, np.array([[-0.006, -0.006, -0.001], [0.006, 0.006, 0.006]]), max(Fixed.atlas_grid_spacings))

    def test_both_spline_orders_reproduce_independent_grid_nodes(self):
        spacing = max(Fixed.atlas_grid_spacings)
        axes, values = MagneticFieldAtlas.sample_grid(self.source, self.bounds, spacing)
        interior = [axis[(axis >= self.bounds[0, index]) & (axis <= self.bounds[1, index])] for index, axis in enumerate(axes)]
        points = np.stack(np.meshgrid(*interior, indexing='ij'), axis=-1).reshape(-1, 3)
        expected = self.source.getB(points)
        for order in Fixed.atlas_spline_orders:
            with self.subTest(order=order):
                atlas = MagneticFieldAtlas(self.source, self.bounds, spacing, order, axes, values)
                np.testing.assert_allclose(atlas.field(self.source, points), expected,
                    atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)

    def test_rigid_transform_and_force_torque_anchor(self):
        source = self.source.copy()
        rotation = Rotation.from_euler('xyz', [0.31, -0.17, 1.2])
        translation = np.array([0.012, -0.007, 0.003])
        source.rotate(rotation, anchor=(0, 0, 0))
        source.move(translation)
        local = np.array([[0.0013, -0.0027, -0.0103], [-0.0011, 0.0007, -0.0097]])
        points = rotation.apply(local) + translation
        np.testing.assert_allclose(self.atlas.field(source, points), source.getB(points), atol=Fixed.atlas_field_tolerance, rtol=0)
        vertices = rotation.apply(np.array([[-0.003, -0.003, -0.01], [0.003, -0.003, -0.01],
            [0.003, 0.003, -0.01], [-0.003, 0.003, -0.01], [-0.003, -0.003, -0.01]])) + translation
        wire = magpy.current.Polyline(vertices=vertices, current=1, meshing=8)
        quadrature = levitation_sim.CoilForceQuadrature([[wire]])
        center = translation + rotation.apply([0, 0, 0.01])
        actual = quadrature.matrix(source, center, self.atlas.field)
        expected = quadrature.matrix(source, center)
        for start in (0, 3):
            np.testing.assert_allclose(actual[start:start + 3], expected[start:start + 3],
                atol=np.linalg.norm(expected[start:start + 3]) * Fixed.atlas_matrix_relative_tolerance, rtol=0)
        shift = np.array([0.003, -0.002, 0.001])
        shifted = quadrature.matrix(source, center + shift, self.atlas.field)
        np.testing.assert_allclose(shifted[3:], actual[3:] - np.cross(shift, actual[:3].T).T,
            atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)

    def test_changed_magnets_and_out_of_domain_queries_fail(self):
        source = self.source.copy()
        source.children[0].polarization *= 1.01
        with self.assertRaisesRegex(ValueError, 'magnet geometry'):
            self.atlas.field(source, self.bounds)
        with self.assertRaisesRegex(ValueError, 'outside'):
            self.atlas.field(self.source, self.bounds + [0, 0, 1])
        with self.assertRaisesRegex(ValueError, 'finite'):
            self.atlas.field(self.source, [[np.nan, 0, 0]])

    def test_persistent_cache_is_exact_and_detects_changed_coefficients(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'atlas.npz'
            self.atlas.save(path)
            loaded = MagneticFieldAtlas.load(path)
            self.assertEqual(loaded.identity, self.atlas.identity)
            np.testing.assert_array_equal(loaded.field(self.source, self.bounds), self.atlas.field(self.source, self.bounds))
            with self.assertRaisesRegex(ValueError, 'already exists'):
                loaded.save(path)
            with np.load(path, allow_pickle=False) as saved:
                values = dict(saved)
            values['coefficients'] = values['coefficients'].copy()
            values['coefficients'].flat[0] += Fixed.atlas_field_tolerance
            np.savez(path, **values)
            with self.assertRaisesRegex(ValueError, 'digest'):
                MagneticFieldAtlas.load(path)


if __name__ == '__main__':
    unittest.main()

from types import SimpleNamespace
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from scipy.interpolate import NdBSpline, make_interp_spline

import magpylib as magpy
import numpy as np
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Fixed
from methodology.magnetic_atlas import MagneticFieldAtlas, MagneticBackend
from methodology.magnetic_spline import TensorSplineEvaluator, build, native_evaluation


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


class NativeSplineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.library = build(Path(cls.directory.name) / 'native')

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def spline(self, orders):
        random = np.random.default_rng(Fixed.atlas_validation_seed)
        knots = tuple(np.r_[np.zeros(k), np.linspace(0, 1, k + 1) ** 2, np.ones(k)] for k in orders)
        shape = tuple(len(t) - k - 1 for t, k in zip(knots, orders))
        return NdBSpline(knots, random.normal(size=(*shape, 3)), orders, extrapolate=False)

    def test_nonuniform_knots_boundaries_and_repeated_knots_match_reference(self):
        random = np.random.default_rng(Fixed.atlas_validation_seed)
        for degree in Fixed.atlas_spline_orders:
            spline = self.spline((degree,) * 3)
            evaluator = TensorSplineEvaluator(spline, self.library)
            interior = np.repeat(np.linspace(0., 1., degree + 1)[:, None] ** 2, 3, axis=1)
            points = np.vstack((random.uniform(size=(Fixed.atlas_validation_points, 3)),
                interior, np.maximum(0, np.nextafter(interior, -np.inf)), np.minimum(1, np.nextafter(interior, np.inf))))
            np.testing.assert_allclose(evaluator(points), spline(points), atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)

    def test_anisotropic_degrees_and_strided_coefficients(self):
        spline = self.spline((*Fixed.atlas_spline_orders, min(Fixed.atlas_spline_orders)))
        evaluator = TensorSplineEvaluator(SimpleNamespace(c=np.asfortranarray(spline.c), t=spline.t, k=spline.k), self.library)
        points = np.random.default_rng(Fixed.atlas_validation_seed).uniform(size=(Fixed.atlas_validation_points, 3))
        np.testing.assert_allclose(evaluator(points), spline(points), atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)

    def test_independent_vector_polynomial_and_empty_queries(self):
        order = max(Fixed.atlas_spline_orders)
        coordinates = np.linspace(0., 1., order + 1)
        x, y, z = np.meshgrid(coordinates, coordinates, coordinates, indexing='ij')
        coefficients = np.stack((x*x + y*z, x*y*z, x-y+z), axis=-1)
        knots = []
        for axis in range(3):
            spline = make_interp_spline(coordinates, coefficients, k=order, axis=axis)
            coefficients = np.moveaxis(spline.c, 0, axis)
            knots.append(spline.t)
        evaluator = TensorSplineEvaluator(NdBSpline(tuple(knots), coefficients, order), self.library)
        points = np.random.default_rng(Fixed.atlas_validation_seed).uniform(size=(Fixed.atlas_validation_points, 3))
        x, y, z = points.T
        expected = np.column_stack((x*x+y*z, x*y*z, x-y+z))
        np.testing.assert_allclose(evaluator(points), expected, atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
        self.assertEqual(evaluator(np.empty((0, 3))).shape, (0, 3))

    def test_invalid_queries_and_coefficients_fail_before_native_call(self):
        spline = self.spline((min(Fixed.atlas_spline_orders),) * 3)
        evaluator = TensorSplineEvaluator(spline, self.library)
        for points in ([[np.nan, 0, 0]], [[0, np.inf, 0]], [[0, 0]], [[-1, 0, 0]], [[0, 0, 2]]):
            with self.assertRaises(ValueError):
                evaluator(points)
        malformed = SimpleNamespace(c=spline.c[..., :2], t=spline.t, k=spline.k)
        with self.assertRaises(ValueError):
            TensorSplineEvaluator(malformed, self.library)

    def test_diagnostic_snapshots_cannot_resume_as_production(self):
        from methodology.benchmark_magnetics import save_spline_diagnostic
        from methodology.joint_airborne import JointAirborneSimulation
        state = {'schema': 'airborne-joint-v4'}
        simulation = SimpleNamespace(checkpoint_state=lambda: dict(state))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'snapshot.json'
            save_spline_diagnostic(simulation, {'method': 'native'}, path)
            with self.assertRaisesRegex(ValueError, 'schema'):
                JointAirborneSimulation.from_checkpoint(path)
        self.assertEqual(state['schema'], 'airborne-joint-v4')

    def test_parallel_queries_preserve_order_and_repeat_exactly(self):
        from methodology.benchmark_magnetics import spline_workers, worker_candidates
        evaluator = TensorSplineEvaluator(self.spline((max(Fixed.atlas_spline_orders),) * 3), self.library)
        points = np.random.default_rng(Fixed.atlas_validation_seed).uniform(size=(Fixed.atlas_validation_points + 1, 3))
        for workers in worker_candidates():
            with spline_workers(evaluator, workers) as parallel:
                for query in (points, points[::-1], points[:1], points[:0], points):
                    np.testing.assert_array_equal(parallel(query), evaluator(query))

    def test_parallel_invalid_query_propagates_failure(self):
        from methodology.benchmark_magnetics import spline_workers, worker_candidates
        evaluator = TensorSplineEvaluator(self.spline((min(Fixed.atlas_spline_orders),) * 3), self.library)
        with spline_workers(evaluator, max(worker_candidates())) as parallel:
            with self.assertRaisesRegex(ValueError, 'outside'):
                parallel(np.array([[0, 0, 0], [0, 0, 2]], dtype=float))
            with self.assertRaisesRegex(ValueError, 'finite'):
                parallel(np.array([[0, 0, 0], [0, np.nan, 0]]))

    def test_changed_native_binary_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            library = Path(folder) / self.library.name
            library.write_bytes(self.library.read_bytes() + b'changed')
            library.with_suffix('.json').write_bytes(self.library.with_suffix('.json').read_bytes())
            with self.assertRaisesRegex(ValueError, 'binary'):
                TensorSplineEvaluator(self.spline((min(Fixed.atlas_spline_orders),) * 3), library)

    def test_native_runtime_identity_restoration_and_checkpoint_rejection(self):
        spline = self.spline((min(Fixed.atlas_spline_orders),) * 3)
        atlas = SimpleNamespace(identity='test-atlas', interpolator=spline,
            evaluator_identity=dict(method='scipy'), evaluator_sources={})
        atlas.field = lambda source, points: atlas.interpolator(points)
        backend = MagneticBackend()
        backend.controller, backend.plant, backend.source = SimpleNamespace(), SimpleNamespace(), {}
        with native_evaluation(atlas, self.library) as identity:
            backend.configure_atlas(atlas, 'atlas_forces')
            saved = dict(atlas=deepcopy(backend.atlas_configuration))
            backend.validate_atlas(saved)
            self.assertEqual(identity['workers'], Fixed.native_spline_workers)
            self.assertTrue(all(item['num_threads'] == Fixed.native_spline_blas_threads for item in identity['blas_libraries']))
            np.testing.assert_allclose(backend.plant.field_evaluator(None, np.array([[0., 0., 0.]])),
                spline([[0., 0., 0.]]), atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
            for key in ('workers', 'blas_threads', 'build'):
                invalid = deepcopy(saved)
                invalid['atlas']['evaluator'][key] = 'changed'
                with self.assertRaisesRegex(ValueError, 'backend or atlas'):
                    backend.validate_atlas(invalid)
            with self.assertRaisesRegex(ValueError, 'unmodified'):
                with native_evaluation(atlas, self.library):
                    self.fail('Nested evaluator must be rejected')
        self.assertIs(atlas.interpolator, spline)
        self.assertEqual(atlas.evaluator_identity, dict(method='scipy'))
        with self.assertRaisesRegex(ValueError, 'runtime'):
            backend.plant.field_evaluator(None, np.zeros((1, 3)))


if __name__ == '__main__':
    unittest.main()

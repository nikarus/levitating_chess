from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from model import Inputs, Fixed
from methodology import coil_field, magnetic_atlas, magnetic_spline
from methodology.cad import magnetics
from methodology.joint_airborne import JointAirborneSimulation
from methodology.tests.test_joint_airborne import substitute_force


def affine_cache(array):
    queries = coil_field.CoilSurfaceQueries(array, min(Fixed.coupling_flux_orders))
    cache = coil_field.CoilFieldCache.__new__(coil_field.CoilFieldCache)
    cache.bounds = np.array([[-1., -1., 0.], [1., 1., 1.]])
    cache.source_state = np.column_stack((queries.positions, queries.elements))
    axes = [np.linspace(lower, upper, max(Fixed.atlas_spline_orders) + 1) for lower, upper in cache.bounds.T]
    points = np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1)
    cache.interpolator = magnetic_atlas.tensor_product_spline(axes, points * [1., 1., -2.], Fixed.coil_cache_spline_order)
    cache.coefficients = cache.interpolator.c
    cache.shape = cache.coefficients.shape[:3]
    cache.provenance = cache.current_provenance()
    cache.identity = cache.content_identity()
    return cache


def continue_cached_checkpoint(folder):
    cache = coil_field.CoilFieldCache.load(folder / 'atlas.npz')
    with magnetic_spline.native_evaluation(cache, folder / 'native/magnetic_spline.so'):
        restored = JointAirborneSimulation.from_checkpoint(folder / 'checkpoint.json', atlas=cache)
        substitute_force(restored)
        restored.save_checkpoint(folder / 'restored.json')
        while not restored.finished:
            restored.advance()
        restored.save_checkpoint(folder / 'continued.json')


class CoilCacheTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.construction = magnetics.ConstructionModel()
        cls.geometry, cls.array, _ = cls.construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
        cls.directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        cls.library = magnetic_spline.build(Path(cls.directory.name) / 'native')

    def test_persistence_integrity_source_and_diagnostic_schema_guards(self):
        cache = affine_cache(self.array)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'atlas.npz'
            cache.save(path)
            restored = coil_field.CoilFieldCache.load(path)
            self.assertEqual(restored.identity, cache.identity)
            np.testing.assert_array_equal(restored(cache.bounds), cache(cache.bounds))
            with patch.object(coil_field.CoilFieldCache, 'current_provenance', return_value={}):
                with self.assertRaisesRegex(ValueError, 'source or dependencies'):
                    coil_field.CoilFieldCache.load(path)
            with np.load(path, allow_pickle=False) as saved:
                values = dict(saved)
            original = values['coefficients'].copy()
            values['coefficients'].flat[0] += Fixed.atlas_field_tolerance
            np.savez(path, **values)
            with self.assertRaisesRegex(ValueError, 'digest'):
                coil_field.CoilFieldCache.load(path)
            values['coefficients'] = original
            metadata = json.loads(str(values['metadata']))
            metadata['schema'] = 'coil-field-screen-v1'
            values['metadata'] = json.dumps(metadata)
            np.savez(path, **values)
            with self.assertRaisesRegex(ValueError, 'schema'):
                coil_field.CoilFieldCache.load(path)

    def test_geometry_domain_and_runtime_guards(self):
        cache = affine_cache(self.array)
        _, changed_array, _ = self.construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
        changed_array.coils[0][0].current = 2
        with self.assertRaisesRegex(ValueError, 'unit-current'):
            cache.bind(changed_array)
        for points in (cache.bounds + 2, np.zeros(3), np.full((1, 3), np.nan)):
            with self.assertRaisesRegex(ValueError, 'domain'):
                cache(points)
        with self.assertRaisesRegex(ValueError, 'moving-magnet'):
            cache.field(None, cache.bounds)
        with self.assertRaisesRegex(ValueError, 'native runtime'):
            JointAirborneSimulation(self.construction, Fixed.airborne_check_duration, initialize=False,
                atlas=cache, magnetic_mode='coil_cache')
        with magnetic_spline.native_evaluation(cache, self.library):
            simulation = JointAirborneSimulation(self.construction, Fixed.airborne_check_duration,
                atlas=cache, magnetic_mode='coil_cache')
            self.assertIs(simulation.plant.matrix_evaluator, simulation.dynamics.plant.matrix_evaluator)
            expected = coil_field.CoilSurfaceQueries(simulation.plant.array, min(Fixed.coupling_flux_orders))
            center = simulation.plant.truth()['com_position_m']
            actual = simulation.plant.magnetic_matrix(window_center=simulation.reference[:2])
            np.testing.assert_array_equal(actual, expected.matrix(simulation.plant.magnets.collection, center, cache))
            with patch('levitation_sim.min_peak_current', return_value=(2 * simulation.controller.current_budget, None)):
                simulation.controller.command(dict(valid=True, pose=simulation.reference, velocity=np.zeros(6)), simulation.reference)
            self.assertTrue(simulation.controller.nominal_matrix.flags.f_contiguous)
            np.testing.assert_allclose(simulation.controller.nominal_matrix, actual,
                atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
            stage = simulation.dynamics.plant
            stage.reset(position=center)
            np.testing.assert_array_equal(simulation.dynamics.force_matrix(stage), actual)
            wire = simulation.plant.array.coils[0][0]
            wire.current = 2
            with self.assertRaisesRegex(ValueError, 'geometry changed'):
                simulation.plant.magnetic_matrix(window_center=simulation.reference[:2])
            wire.current = 1
            saved = simulation.checkpoint_state()
            for key in ('identity', 'evaluator', 'mode'):
                invalid = deepcopy(saved)
                invalid['atlas'][key] = 'changed'
                with self.assertRaisesRegex(ValueError, 'backend or atlas'):
                    simulation.restore_checkpoint(invalid)
        with self.assertRaisesRegex(ValueError, 'runtime'):
            simulation.plant.magnetic_matrix(window_center=simulation.reference[:2])

    def test_cached_checkpoint_restarts_in_fresh_process_with_pending_sensing(self):
        folder = Path(self.directory.name)
        cache = affine_cache(self.array)
        cache.save(folder / 'atlas.npz')
        with magnetic_spline.native_evaluation(cache, self.library):
            simulation = JointAirborneSimulation(self.construction, Fixed.airborne_check_duration,
                atlas=cache, magnetic_mode='coil_cache', motion_scenario='local_move')
            substitute_force(simulation)
            while simulation.clock <= Fixed.hall_power_settle_time:
                simulation.advance()
            self.assertGreater(simulation.pending_scans[0]['sample_count'], 0)
            simulation.save_checkpoint(folder / 'checkpoint.json')
            saved = simulation.checkpoint_state()
            subprocess.run([sys.executable, '-c',
                'from pathlib import Path; import sys; from methodology.tests.test_coil_field import continue_cached_checkpoint; continue_cached_checkpoint(Path(sys.argv[1]))',
                str(folder)], check=True)
            self.assertEqual(saved, json.loads((folder / 'restored.json').read_text()))
            while not simulation.finished:
                simulation.advance()
            self.assertEqual(simulation.checkpoint_state(), json.loads((folder / 'continued.json').read_text()))

    def test_grid_sampling_and_exterior_limits(self):
        queries = SimpleNamespace(positions=np.array([[0., 0., 0.]]), direct=lambda points: points ** 2)
        spacing = Fixed.coil_cache_grid_spacing
        bounds = np.array([[-spacing, -spacing, spacing], [spacing, spacing, 2 * spacing]])
        axes = coil_field.coil_grid_axes(queries, bounds, spacing)
        values = coil_field.sample_coil_grid(queries, axes)
        expected = np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1) ** 2
        np.testing.assert_array_equal(values, expected)
        for invalid in (bounds[::-1], bounds - [0, 0, 1], np.full((2, 3), np.nan)):
            with self.assertRaises(ValueError):
                coil_field.coil_grid_axes(queries, invalid, spacing)
        with patch.object(Fixed, 'atlas_max_storage_bytes', 0):
            with self.assertRaisesRegex(ValueError, 'memory budget'):
                coil_field.coil_grid_axes(queries, bounds, spacing)


if __name__ == '__main__':
    unittest.main()

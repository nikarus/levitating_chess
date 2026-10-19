from types import SimpleNamespace
import unittest
from unittest.mock import patch

import magpylib as magpy
import numpy as np
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Fixed
from methodology.benchmark_magnetics import library_actuator_matrix


class MagneticForceTests(unittest.TestCase):
    def setUp(self):
        levitation_sim.use_geometry(SimpleNamespace(settings=Fixed))
        self.array = levitation_sim.CoilArray(2, 0.01, 0.02, 0.001, 0.001, 5, 0.003, 0.001, 4, 2, 2, 2)
        self.layout = SimpleNamespace(collection=magpy.Collection(magpy.magnet.Cuboid(
            dimension=(0.01, 0.01, 0.005), polarization=(0.2, -0.3, 1), position=(0, 0, 0.005))))
        self.center = np.array([0, 0, 0.012])

    def test_rigid_work_path_uses_world_angular_velocity_and_closes(self):
        from methodology.coil_field_verification import rigid_segment, work_integral
        start = np.r_[self.center, np.radians(Fixed.motion_check_orientation_deg)]
        finish = np.r_[-self.center, -start[3:]]
        pose, tangent = rigid_segment(start, finish, Fixed.coupling_excitation_fraction)
        step = Fixed.angle_derivative_step
        before, _ = rigid_segment(start, finish, Fixed.coupling_excitation_fraction - step)
        after, _ = rigid_segment(start, finish, Fixed.coupling_excitation_fraction + step)
        actual = (Rotation.from_euler('xyz', after[3:]) * Rotation.from_euler('xyz', before[3:]).inv()).as_rotvec() / (2 * step)
        np.testing.assert_allclose(tangent[3:], actual, atol=Fixed.motion_verification_tolerance, rtol=Fixed.matrix_rank_tolerance)
        matrix = np.ones((6, 1))
        work = work_integral(lambda pose: matrix, [start, finish], min(Fixed.coupling_flux_orders))
        np.testing.assert_allclose(work, tangent @ matrix, atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
        closed = work_integral(lambda pose: matrix, [start, finish, start], min(Fixed.coupling_flux_orders))
        np.testing.assert_allclose(closed, 0, atol=Fixed.singular_value_tolerance, rtol=0)

    def test_diagnostic_coil_domain_rejects_extrapolation_before_evaluation(self):
        from methodology.coil_field_verification import DiagnosticCoilCache
        cache = DiagnosticCoilCache.__new__(DiagnosticCoilCache)
        cache.bounds = np.array([-self.center - 1, self.center + 1])
        cache.evaluate = lambda points: np.zeros_like(points)
        np.testing.assert_array_equal(cache(np.zeros((1, 3))), np.zeros((1, 3)))
        for points in (cache.bounds * 2, np.full((1, 3), np.nan), np.zeros(3)):
            with self.assertRaises(ValueError):
                cache(points)

    def test_magnet_surface_force_and_torque_match_uniform_and_linear_fields(self):
        from model import Constants
        from methodology.coupling_verification import magnet_surface_quadrature, magnet_surface_wrench
        source = self.layout.collection
        source.rotate(Rotation.from_euler('xyz', np.radians(Fixed.motion_check_orientation_deg)), anchor=self.center)
        magnet = source.sources_all[0]
        moment = magnet.orientation.apply(magnet.polarization) * np.prod(magnet.dimension) / Constants.vacuum_permeability
        field = np.ones(3)
        gradient = np.diag([1., 1., -2.]) / self.array.short
        points, charges = magnet_surface_quadrature(source, min(Fixed.coupling_flux_orders))
        for derivative in (np.zeros((3, 3)), gradient):
            fields = field + (points - magnet.position) @ derivative.T
            force = derivative @ moment
            torque = np.cross(moment, field) + np.cross(magnet.position - self.center, force)
            actual = magnet_surface_wrench(points, charges, self.center, fields)
            np.testing.assert_allclose(actual, np.r_[force, torque],
                atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
        np.testing.assert_allclose(charges.sum(), 0, atol=Fixed.singular_value_tolerance, rtol=0)

    def test_magnet_surface_rejects_invalid_geometry_and_fields(self):
        from methodology.coupling_verification import magnet_surface_quadrature, magnet_surface_wrench
        with self.assertRaises(ValueError):
            magnet_surface_quadrature(magpy.Collection(magpy.magnet.Sphere(diameter=1, polarization=(0, 0, 1))),
                                      min(Fixed.coupling_flux_orders))
        points, charges = magnet_surface_quadrature(self.layout.collection, min(Fixed.coupling_flux_orders))
        for fields in (np.zeros((len(points), 2)), np.full(points.shape, np.nan)):
            with self.assertRaises(ValueError):
                magnet_surface_wrench(points, charges, self.center, fields)

    def test_reused_coil_fields_preserve_rotation_translation_and_geometry_guard(self):
        from model import Constants
        from methodology.coil_field import CoilSurfaceQueries
        rotation = Rotation.from_euler('xyz', np.radians(Fixed.motion_check_orientation_deg))
        vertices = self.array.coils[0][0].vertices
        offsets = np.array([np.zeros(3), self.center])
        coils = [[magpy.current.Polyline(vertices=rotation.apply(vertices + offset), current=1)] for offset in offsets]
        samples = [(wire[0].vertices[:-1] + wire[0].vertices[1:]) / 2 for wire in coils]
        elements = [np.diff(wire[0].vertices, axis=0) for wire in coils]
        quadrature = levitation_sim.CoilForceQuadrature.from_samples(coils, samples, elements)
        array = SimpleNamespace(coils=coils, rotation=rotation, height=0., force_quadrature=quadrature,
            placements=[dict(x=offset[0], y=offset[1], top=offset[2]) for offset in offsets])
        queries = CoilSurfaceQueries(array, min(Fixed.coupling_flux_orders))
        points = np.asarray([self.center, -self.center])
        expected = np.concatenate([rotation.inv().apply(points) - offset for offset in offsets])
        np.testing.assert_allclose(queries.local_queries(points), expected,
            atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
        field = np.ones(3)
        matrix = queries.matrix(self.layout.collection, self.center, lambda points: np.broadcast_to(field, points.shape))
        magnet = self.layout.collection.sources_all[0]
        moment = magnet.polarization * np.prod(magnet.dimension) / Constants.vacuum_permeability
        wrench = np.r_[np.zeros(3), np.cross(moment, rotation.apply(field))]
        np.testing.assert_allclose(matrix, np.repeat(wrench[:, None], len(coils), axis=1),
            atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
        quadrature.points[-1] += self.center
        with self.assertRaises(ValueError):
            CoilSurfaceQueries(array, min(Fixed.coupling_flux_orders))

    def test_curved_path_field_matches_circular_loop_axis(self):
        from build123d import Edge, Wire
        from model import Constants
        from methodology.numerical_accuracy import cad_field_quadrature
        radius = self.array.short / 2
        path = Wire([Edge.make_circle(radius * 1000)])
        heights = radius * np.asarray(Fixed.coupling_refinements)
        points = np.column_stack((np.zeros((len(heights), 2)), heights))
        expected = Constants.vacuum_permeability * radius ** 2 / (2 * (radius ** 2 + heights ** 2) ** 1.5)
        actual = cad_field_quadrature(path, points, max(Fixed.coupling_flux_orders), 0)
        np.testing.assert_allclose(actual[:, 2], expected, rtol=Fixed.motion_verification_tolerance, atol=0)
        np.testing.assert_allclose(actual[:, :2], 0, atol=Fixed.singular_value_tolerance, rtol=0)

    def test_current_box_bound_covers_cancellation_and_attains_extreme(self):
        from itertools import product
        from methodology.numerical_accuracy import current_box_bound
        difference = np.array([[1., -2., 0.], [-1., 1., 2.]]) * Fixed.hall_output_noise
        limit = Fixed.driver_channel_current
        bound = current_box_bound(difference, limit)
        vertices = np.asarray(list(product((-limit, limit), repeat=difference.shape[1])))
        self.assertAlmostEqual(bound['maximum_T'], np.max(np.abs(vertices @ difference.T)))
        self.assertAlmostEqual(bound['maximum_T'], bound['attained_T'])
        np.testing.assert_allclose(bound['per_sensor_T'], np.max(np.abs(vertices @ difference.T), axis=0))
        for invalid in (0, -limit, np.nan, np.inf):
            with self.assertRaises(ValueError):
                current_box_bound(difference, invalid)

    def test_curved_force_matches_loop_in_uniform_and_linear_fields(self):
        from build123d import Edge, Wire
        from methodology.cad.magnetics import curved_conductor_samples
        from methodology.coupling_verification import curved_surface_flux
        radius = self.array.short / 2
        path = Wire([Edge.make_circle(radius * 1000)])
        points, elements = curved_conductor_samples(path, max(Fixed.coupling_flux_orders), 0)
        filament = self.array.coils[0][0]
        quadrature = levitation_sim.CoilForceQuadrature.from_samples([[filament]], [points], [elements])
        field = np.ones(3)
        moment = np.array([0., 0., np.pi * radius ** 2 * filament.current])
        for gradient in (0., 1 / radius):
            def evaluate(source, locations):
                return field + gradient * np.column_stack((locations[:, 2], np.zeros(len(locations)), locations[:, 0]))
            force = np.array([-moment[2] * gradient, 0., 0.])
            torque = -np.cross(moment, field) + np.cross(-self.center, force)
            actual = quadrature.matrix(self.layout.collection, self.center, evaluate)[:, 0]
            np.testing.assert_allclose(actual, np.r_[force, torque], atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
        with patch('levitation_sim.magnetic_field', side_effect=lambda source, points: np.broadcast_to(field, points.shape)):
            flux = curved_surface_flux(self.layout.collection, points, elements, points[0], max(Fixed.coupling_flux_orders))
        self.assertAlmostEqual(flux * filament.current, np.dot(moment, field), delta=Fixed.singular_value_tolerance)

    def test_sampled_force_quadrature_rejects_incomplete_or_nonfinite_samples(self):
        for points, vectors in (([], []), ([np.zeros((1, 3))], [np.zeros((2, 3))]),
                                ([np.full((1, 3), np.nan)], [np.zeros((1, 3))])):
            with self.assertRaises(ValueError):
                levitation_sim.CoilForceQuadrature.from_samples([[self.array.coils[0][0]]], points, vectors)

    def test_independent_surface_flux_matches_oriented_area(self):
        from methodology.coupling_verification import surface_flux
        field = np.ones(3)
        wire = self.array.coils[0][0]
        wire.rotate(Rotation.from_euler('xyz', np.radians(Fixed.motion_check_orientation_deg)), anchor=self.center)
        vertices = wire.orientation.apply(wire.vertices) + wire.position
        expected = wire.current * np.dot(np.cross(vertices[:-1], vertices[1:]).sum(axis=0) / 2, field)
        with patch('levitation_sim.magnetic_field', side_effect=lambda source, points: np.broadcast_to(field, points.shape)):
            for order in Fixed.coupling_flux_orders:
                self.assertAlmostEqual(surface_flux(self.layout.collection, wire, order), expected,
                    delta=abs(expected) * Fixed.motion_verification_tolerance)
            wire.vertices = wire.vertices[::-1]
            self.assertAlmostEqual(surface_flux(self.layout.collection, wire, max(Fixed.coupling_flux_orders)), -expected,
                delta=abs(expected) * Fixed.motion_verification_tolerance)
            wire.vertices = wire.vertices[:-1]
            with self.assertRaises(ValueError):
                surface_flux(self.layout.collection, wire, max(Fixed.coupling_flux_orders))

    def test_uniform_field_matches_closed_loop_dipole_torque(self):
        field = np.ones(3)
        wires = self.array.coils[0]
        moment = np.zeros(3)
        for wire in wires:
            vertices = wire.orientation.apply(wire.vertices) + wire.position
            np.testing.assert_array_equal(vertices[0], vertices[-1])
            moment += wire.current * np.cross(vertices[:-1], vertices[1:]).sum(axis=0) / 2
        actual = levitation_sim.CoilForceQuadrature([wires]).matrix(
            self.layout.collection, self.center, lambda sources, points: np.broadcast_to(field, points.shape))[:, 0]
        np.testing.assert_allclose(actual[:3], 0, atol=Fixed.singular_value_tolerance, rtol=0)
        np.testing.assert_allclose(actual[3:], -np.cross(moment, field), atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)

    def test_uniform_field_matches_straight_wire_force_and_torque(self):
        source = self.array.coils[0][0]
        wire = magpy.current.Polyline(vertices=source.vertices[:2], current=source.current,
            meshing=source.meshing, position=source.position, orientation=source.orientation)
        vertices = wire.orientation.apply(wire.vertices) + wire.position
        field = np.ones(3)
        expected_force = -wire.current * np.cross(vertices[-1] - vertices[0], field)
        expected_torque = np.cross(vertices.mean(axis=0) - self.center, expected_force)
        actual = levitation_sim.CoilForceQuadrature([[wire]]).matrix(
            self.layout.collection, self.center, lambda sources, points: np.broadcast_to(field, points.shape))[:, 0]
        np.testing.assert_allclose(actual, np.r_[expected_force, expected_torque],
            atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)

    def assert_matches_library(self):
        actual = levitation_sim.actuator_matrix(self.layout, self.array, self.center)
        expected = library_actuator_matrix(self.layout, self.array, self.center)
        np.testing.assert_allclose(actual, expected, atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
        return actual

    def test_transformed_multifilament_windings_match_library(self):
        self.assert_matches_library()
        self.layout.collection.rotate(Rotation.from_euler('xyz', [0.2, -0.1, 0.7]), anchor=self.center)
        self.layout.collection.move([0.001, -0.002, 0.003])
        for index, filament in enumerate(self.array.coils[0]):
            filament.rotate(Rotation.from_euler('z', 0.1 * index), anchor=[0, 0, 0])
            filament.move([0.001, 0, 0])
            filament.meshing = index + 1
            filament.current *= (-1) ** index
        self.assert_matches_library()

    def test_reuses_geometry_but_recalculates_fields_and_torque_anchor(self):
        original = self.assert_matches_library()
        cached = self.array.force_quadrature
        self.layout.collection.move([0.002, 0, 0])
        moved = self.assert_matches_library()
        self.assertIs(self.array.force_quadrature, cached)
        self.assertFalse(np.array_equal(original, moved))
        displacement = np.array([0.003, -0.002, 0.001])
        self.center += displacement
        shifted = self.assert_matches_library()
        self.assertIs(self.array.force_quadrature, cached)
        np.testing.assert_allclose(shifted[3:], moved[3:] - np.cross(displacement, moved[:3].T).T,
                                   atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)

    def test_cache_invalidates_on_every_filament_input(self):
        self.assert_matches_library()
        filament = self.array.coils[0][0]
        changes = dict(current=filament.current * 2, meshing=filament.meshing * 2,
                       vertices=filament.vertices + [0.001, 0, 0], position=np.array([0, 0.001, 0]),
                       orientation=Rotation.from_euler('z', 0.2))
        for attribute, value in changes.items():
            with self.subTest(attribute=attribute):
                previous = self.array.force_quadrature
                setattr(filament, attribute, value)
                self.assert_matches_library()
                self.assertIsNot(self.array.force_quadrature, previous)

    def test_changed_window_and_reordered_columns_match_library(self):
        self.assert_matches_library()
        previous = self.array.force_quadrature
        self.array.build(center=(self.array.short, self.array.long))
        self.assert_matches_library()
        self.assertIsNot(self.array.force_quadrature, previous)
        previous = self.array.force_quadrature
        self.array.coils.reverse()
        self.assert_matches_library()
        self.assertIsNot(self.array.force_quadrature, previous)

    def test_serial_and_parallel_batches_preserve_results(self):
        with patch.object(Fixed, 'force_field_workers', 1):
            reference = self.assert_matches_library()
        for workers, batch_size in ((1, 1), (Fixed.force_field_workers, 7)):
            with self.subTest(workers=workers, batch_size=batch_size), \
                    patch.object(Fixed, 'force_field_workers', workers), \
                    patch.object(Fixed, 'force_field_batch_points', batch_size):
                actual = self.assert_matches_library()
                np.testing.assert_array_equal(actual, reference)

    def test_nonpositive_computation_settings_are_rejected(self):
        for name in ('force_field_workers', 'force_field_batch_points'):
            with self.subTest(setting=name), patch.object(Fixed, name, 0), self.assertRaises(ValueError):
                levitation_sim.actuator_matrix(self.layout, self.array, self.center)


if __name__ == '__main__':
    unittest.main()

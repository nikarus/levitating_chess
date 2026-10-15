import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from model import Inputs, Fixed, Constants
from methodology import motion
from methodology.cad import magnetics


class MotionInterfaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.construction = magnetics.ConstructionModel()
        cls.plant = motion.MotionPlant(cls.construction)

    def setUp(self):
        self.plant.reset()

    def test_cad_mass_and_full_inertia_survive_mesh_import(self):
        plant = self.plant
        body = plant.king_id
        self.assertAlmostEqual(plant.model.body_mass[body], self.construction.piece_properties['mass_g'] / 1000)
        np.testing.assert_allclose(plant.data.xipos[body], np.asarray(self.construction.piece_properties['center_mm']) / 1000,
                                   atol=Fixed.motion_verification_tolerance, rtol=0)
        axes = Rotation.from_quat(plant.model.body_iquat[body], scalar_first=True).as_matrix()
        tensor = axes @ np.diag(plant.model.body_inertia[body]) @ axes.T
        np.testing.assert_allclose(tensor, self.construction.piece_properties['inertia_kg_m2'],
                                   atol=Fixed.motion_verification_tolerance, rtol=0)
        np.testing.assert_array_equal(plant.model.dof_damping, 0)
        self.assertEqual(plant.metadata['physical_coils'], Fixed.cad_patch_cells_per_side ** 2)

    def test_physical_patch_columns_do_not_disappear_when_selected_center_changes(self):
        plant = motion.MotionPlant(self.construction, full_coil_patch=True)
        matrix = plant.magnetic_matrix(window_center=(0, 0))
        identities = tuple(plant.array.identities)
        shifted = plant.magnetic_matrix(window_center=(plant.array.short_pitch, 0))
        self.assertEqual(set(identities), plant.physical_identities)
        self.assertEqual(tuple(plant.array.identities), identities)
        np.testing.assert_array_equal(shifted, matrix)
        self.assertTrue(np.all(np.linalg.norm(matrix, axis=0) > 0))

    def test_world_force_and_torque_have_correct_sign_and_no_extra_gravity(self):
        plant = self.plant
        rotation = Rotation.from_euler('xyz', Fixed.motion_check_orientation_deg, degrees=True)
        position = plant.data.qpos[:3].copy()
        position[2] += plant.geometry.max_flight_gap - plant.geometry.gap
        plant.reset(position=position, rotation=rotation)
        wrench = np.array([plant.geometry.weight, 0, plant.geometry.weight, 0, 0, plant.geometry.weight * plant.metadata['base_radius_m']])
        plant.apply_wrench(wrench)
        mujoco.mj_forward(plant.model, plant.data)
        np.testing.assert_allclose(plant.data.qacc[:3], [Constants.gravity, 0, 0], atol=Fixed.motion_verification_tolerance)
        tensor = rotation.as_matrix() @ np.asarray(plant.metadata['king_inertia_kg_m2']) @ rotation.as_matrix().T
        np.testing.assert_allclose(rotation.apply(plant.data.qacc[3:]), np.linalg.solve(tensor, wrench[3:]),
                                   atol=Fixed.motion_verification_tolerance)
        plant.apply_wrench(np.zeros(6))
        mujoco.mj_forward(plant.model, plant.data)
        np.testing.assert_allclose(plant.data.qacc[:3], [0, 0, -Constants.gravity], atol=Fixed.motion_verification_tolerance)

    def test_contact_primitives_preserve_cad_surface_and_bottom_film(self):
        plant = self.plant
        surface = plant.model.geom('Playing_surface').id
        skin = plant.model.geom('Piece_bottom_skin').id
        self.assertEqual(plant.model.geom_type[surface], mujoco.mjtGeom.mjGEOM_PLANE)
        self.assertEqual(plant.model.geom_type[skin], mujoco.mjtGeom.mjGEOM_CYLINDER)
        self.assertAlmostEqual(plant.data.geom_xpos[surface, 2],
                               plant.metadata['board_top_m'])
        self.assertAlmostEqual(plant.data.geom_xpos[skin, 2] - plant.model.geom_size[skin, 1], -Fixed.piece_bottom_skin / 1000)
        self.assertAlmostEqual(plant.model.geom_size[skin, 0], plant.metadata['base_radius_m'])

    def test_applied_wrench_is_replaced_instead_of_accumulated(self):
        plant = self.plant
        wrench = [0, 0, plant.geometry.weight, 0, 0, 0]
        plant.advance_wrench(wrench)
        plant.advance_wrench(wrench)
        np.testing.assert_allclose(plant.data.qvel, 0, atol=Fixed.motion_verification_tolerance)
        plant.advance_wrench(np.zeros(6))
        self.assertAlmostEqual(plant.data.qvel[2], -Constants.gravity * plant.model.opt.timestep)

    def test_free_fall_energy_defect_matches_semi_implicit_step(self):
        plant = self.plant
        original_period = plant.model.opt.timestep
        mass = plant.model.body_mass[plant.king_id]
        height = plant.geometry.center_of_mass_height + (plant.geometry.max_flight_gap - plant.geometry.gap) / 2
        for refinement in Fixed.coupling_refinements:
            plant.reset(position=np.array([0., 0., height]))
            period = Fixed.adaptive_max_step / refinement
            plant.model.opt.timestep = period
            plant.advance_wrench(np.zeros(6))
            state = plant.truth()
            residual = mass * state['linear_velocity_m_per_s'][2] ** 2 / 2 + mass * Constants.gravity * (state['com_position_m'][2] - height)
            expected = -mass * Constants.gravity ** 2 * period ** 2 / 2
            self.assertEqual(plant.data.ncon, 0)
            self.assertAlmostEqual(residual / expected, 1., delta=Fixed.coupling_relative_tolerance)
        plant.model.opt.timestep = original_period

    def test_mechanical_energy_change_matches_native_energy_at_rotated_states(self):
        plant = self.plant
        height = plant.geometry.center_of_mass_height + (plant.geometry.max_flight_gap - plant.geometry.gap) / 2
        rotation = Rotation.from_euler('xyz', Fixed.motion_check_orientation_deg, degrees=True)
        velocity = np.r_[np.arange(1, 4) * Fixed.qualification_position_error,
            np.arange(3, 0, -1) * Fixed.qualification_angle_error] * Fixed.pose_controller_frequency
        states, energies = [], []
        for orientation, speed in ((rotation, velocity), (rotation.inv(), -2 * velocity)):
            plant.reset(position=np.array([0., 0., height]), rotation=orientation, velocity=speed)
            states.append(plant.truth())
            mujoco.mj_energyPos(plant.model, plant.data)
            mujoco.mj_energyVel(plant.model, plant.data)
            energies.append(float(np.sum(plant.data.energy)))
        expected = energies[1] - energies[0]
        self.assertAlmostEqual(plant.mechanical_energy_change(*states) / expected, 1., delta=Fixed.motion_verification_tolerance)

    def test_variable_benchmark_rejection_restores_energy_and_motion(self):
        from methodology.coupling_verification import VariableCouplingTrial
        original_period = self.plant.model.opt.timestep
        rejected = VariableCouplingTrial(self.construction, self.plant, False)
        with patch.object(rejected, 'step_error', side_effect=[dict(position=2.), dict(position=0.5)]):
            rejected.advance()
        expected_state = rejected.plant.integration_state().copy()
        expected_currents = rejected.drivers.currents.copy()
        expected_trace = rejected.trace.copy()
        expected_work = rejected.work
        self.assertEqual(rejected.adaptation['rejected'], 1)
        direct = VariableCouplingTrial(self.construction, self.plant, False)
        direct.proposed_step = Fixed.adaptive_max_step / 2
        with patch.object(direct, 'step_error', return_value=dict(position=0.5)):
            direct.advance()
        np.testing.assert_array_equal(direct.plant.integration_state(), expected_state)
        np.testing.assert_array_equal(direct.drivers.currents, expected_currents)
        self.assertEqual(direct.trace, expected_trace)
        self.assertEqual(direct.work, expected_work)
        for key in ('signed_energy_defect_J', 'absolute_energy_defect_J'):
            self.assertEqual(direct.adaptation[key], rejected.adaptation[key])
        self.plant.model.opt.timestep = original_period

    def test_adaptive_energy_budget_enforces_accumulation_without_local_state_checks(self):
        from methodology.coupling_verification import VariableCouplingTrial
        original_period = self.plant.model.opt.timestep
        trial = VariableCouplingTrial(self.construction, self.plant, True)
        with patch.object(trial, 'step_error', return_value=dict(position=0.)):
            while not trial.finished:
                trial.advance()
        self.assertIsNone(trial.stop_reason)
        self.assertGreater(trial.adaptation['rejected'], 0)
        self.assertLessEqual(trial.adaptation['absolute_energy_defect_J'], trial.adaptation['energy_budget_J'])
        for interval in trial.adaptation['intervals']:
            self.assertLessEqual(interval['errors']['energy'], 1)
        self.plant.model.opt.timestep = original_period

    def test_spinning_body_matches_newton_euler_gyroscopic_term(self):
        plant = self.plant
        rotation = Rotation.from_euler('xyz', Fixed.motion_check_orientation_deg, degrees=True)
        spin = np.array([1, -1, 1]) * np.radians(Inputs.target_yaw_angle_deg) / Inputs.target_yaw_time
        position = plant.data.qpos[:3].copy()
        position[2] += plant.geometry.max_flight_gap - plant.geometry.gap
        plant.reset(position=position, rotation=rotation, velocity=[0, 0, 0, *spin])
        plant.apply_wrench([0, 0, plant.geometry.weight, 0, 0, 0])
        mujoco.mj_forward(plant.model, plant.data)
        tensor = np.asarray(plant.metadata['king_inertia_kg_m2'])
        expected = np.linalg.solve(tensor, -np.cross(spin, tensor @ spin))
        np.testing.assert_allclose(plant.data.qacc[3:], expected, atol=Fixed.motion_verification_tolerance)
        np.testing.assert_allclose(plant.truth()['angular_velocity_rad_per_s'], rotation.apply(spin),
                                   atol=Fixed.motion_verification_tolerance)

    def test_analytic_motion_contact_and_refinement_checks(self):
        report = motion.mechanical_checks(self.plant, self.construction)
        self.assertTrue(report['passed'], report)
        json.dumps(report)

    def test_checkpoint_replays_identically(self):
        plant = self.plant
        wrench = [plant.geometry.weight, 0, plant.geometry.weight, 0, 0, 0]
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'motion.json'
            plant.advance_wrench(wrench)
            plant.save_checkpoint(path)
            plant.advance_wrench(wrench)
            expected_position, expected_velocity = plant.data.qpos.copy(), plant.data.qvel.copy()
            expected_time = plant.data.time
            plant.reset()
            plant.load_checkpoint(path)
            plant.advance_wrench(wrench)
            np.testing.assert_array_equal(plant.data.qpos, expected_position)
            np.testing.assert_array_equal(plant.data.qvel, expected_velocity)
            self.assertEqual(plant.data.time, expected_time)

    def test_checkpoint_rejects_changed_scene(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'motion.json'
            self.plant.save_checkpoint(path)
            checkpoint = json.loads(path.read_text())
            checkpoint['scene_sha256'] = 'different'
            path.write_text(json.dumps(checkpoint))
            with self.assertRaises(ValueError):
                self.plant.load_checkpoint(path)

    def test_force_nan_and_geometry_outside_patch_fail_loudly(self):
        with self.assertRaises(ValueError):
            self.plant.advance_wrench([np.nan, 0, 0, 0, 0, 0])
        self.plant.reset(position=[self.plant.array.window, self.plant.array.window, self.plant.geometry.center_of_mass_height])
        with self.assertRaises(ValueError):
            self.plant.magnetic_matrix()


if __name__ == '__main__':
    unittest.main()

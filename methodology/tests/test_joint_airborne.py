from pathlib import Path
from copy import deepcopy
import json
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock

import numpy as np
from scipy.linalg import expm
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation

from model import Inputs, Fixed, Constants
from methodology import electrical, motion
from methodology.cad import magnetics
from methodology.coupling_verification import VariableCouplingTrial, variable_coupling_solution, variable_energy_residual
from methodology.joint_airborne import JointDynamics, JointAirborneSimulation


class JointDynamicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.construction = magnetics.ConstructionModel()
        cls.plant = motion.MotionPlant(cls.construction)

    def setUp(self):
        self.height = self.plant.geometry.center_of_mass_height + (self.plant.geometry.max_flight_gap - self.plant.geometry.gap) / 2
        self.plant.reset(position=np.array([0., 0., self.height]))
        self.drivers = electrical.CoilDrivers(self.construction, tuple(self.plant.array.identities))
        self.matrix = np.zeros((6, len(self.drivers.currents)))
        self.dynamics = JointDynamics(self.plant, self.drivers, 1, lambda plant: self.matrix, lambda plant, currents: np.zeros(1))

    def test_free_fall_and_trial_isolation(self):
        before = self.plant.integration_state().copy()
        initial = self.dynamics.pack(self.plant, np.zeros(1))
        duration = Fixed.airborne_check_duration
        result = self.dynamics.integrate(0., duration, initial)[-1]
        np.testing.assert_array_equal(self.plant.integration_state(), before)
        np.testing.assert_array_equal(self.drivers.currents, 0)
        self.dynamics.commit(self.plant, duration, result.y[:, -1])
        self.assertAlmostEqual(self.plant.data.qpos[2], self.height - Constants.gravity * duration ** 2 / 2,
            delta=Fixed.motion_verification_tolerance)
        self.assertAlmostEqual(self.plant.data.qvel[2], -Constants.gravity * duration, delta=Fixed.motion_verification_tolerance)

    def test_constant_electromechanical_coupling_matches_matrix_exponential(self):
        coupling = self.plant.geometry.weight / Fixed.driver_channel_current
        self.matrix[2, 0] = coupling
        self.drivers.currents[0] = Fixed.driver_channel_current * Fixed.coupling_excitation_fraction
        self.drivers.voltage[0] = self.drivers.resistance * self.drivers.currents[0]
        initial = self.dynamics.pack(self.plant, np.zeros(1))
        duration = Fixed.airborne_check_duration
        generator = np.array([[0, 1, 0, 0], [0, 0, coupling / self.plant.model.body_mass[self.plant.king_id], -Constants.gravity],
            [0, -coupling / self.drivers.inductance, -self.drivers.resistance / self.drivers.inductance,
                self.drivers.voltage[0] / self.drivers.inductance], [0, 0, 0, 0]])
        result = self.dynamics.integrate(0., duration, initial)[-1]
        expected = expm(generator * duration) @ [self.height, 0, self.drivers.currents[0], 1]
        actual = result.y[[2, 9, self.dynamics.current.start], -1]
        np.testing.assert_allclose(actual, expected[:3], atol=Fixed.coupling_reference_tolerance, rtol=Fixed.coupling_relative_tolerance ** 2)

    def test_rotated_principal_spin_matches_analytic_orientation(self):
        rotation = Rotation.from_euler('xyz', Fixed.motion_check_orientation_deg, degrees=True)
        principal = Rotation.from_quat(self.plant.model.body_iquat[self.plant.king_id], scalar_first=True)
        spin = principal.apply([0, 0, Fixed.pose_controller_frequency])
        self.plant.reset(position=np.array([0., 0., self.height]), rotation=rotation, velocity=np.r_[np.zeros(3), spin])
        duration = Fixed.airborne_check_duration
        result = self.dynamics.integrate(0., duration, self.dynamics.pack(self.plant, np.zeros(1)))[-1]
        actual = Rotation.from_quat(result.y[3:7, -1], scalar_first=True)
        expected = rotation * Rotation.from_rotvec(spin * duration)
        self.assertLess((actual * expected.inv()).magnitude(), Fixed.adaptive_angle_tolerance)
        np.testing.assert_allclose(result.y[10:13, -1], spin, atol=Fixed.adaptive_angular_velocity_tolerance, rtol=0)

    def test_hall_filter_dense_output_matches_analytic_ramp(self):
        duration = Fixed.airborne_check_duration
        slope = Fixed.hall_linear_range / duration
        self.dynamics.field_signal = lambda plant, currents: np.array([slope * plant.data.time])
        result = self.dynamics.integrate(0., duration, self.dynamics.pack(self.plant, np.zeros(1)))[-1]
        times = np.linspace(0., duration, Fixed.hall_samples_per_sensor)
        rate = 2 * np.pi * Fixed.hall_sensor_bandwidth
        expected = slope * (times - (1 - np.exp(-rate * times)) / rate)
        np.testing.assert_allclose(result.sol(times)[self.dynamics.hall_filter][0], expected,
            atol=Fixed.adaptive_field_tolerance, rtol=0)

    def test_mixed_axis_spin_matches_independent_euler_equations(self):
        rotation = Rotation.from_euler('xyz', Fixed.motion_check_orientation_deg, degrees=True)
        spin = np.array([1, -1, 1]) * np.radians(Inputs.target_yaw_angle_deg) / Inputs.target_yaw_time
        tensor = np.asarray(self.plant.metadata['king_inertia_kg_m2'])
        self.plant.reset(position=np.array([0., 0., self.height]), rotation=rotation, velocity=np.r_[np.zeros(3), spin])
        def derivative(time, values):
            scalar, vector, omega = values[0], values[1:4], values[4:]
            return np.r_[-np.dot(vector, omega) / 2, (scalar * omega + np.cross(vector, omega)) / 2,
                np.linalg.solve(tensor, -np.cross(omega, tensor @ omega))]
        duration = Fixed.airborne_check_duration
        reference = solve_ivp(derivative, (0., duration), np.r_[rotation.as_quat(scalar_first=True), spin],
            method='Radau', rtol=Fixed.coupling_reference_tolerance / Fixed.qualification_refinement_factor,
            atol=Fixed.coupling_reference_tolerance / Fixed.qualification_refinement_factor, dense_output=True)
        self.assertTrue(reference.success)
        result = self.dynamics.integrate(0., duration, self.dynamics.pack(self.plant, np.zeros(1)))[-1]
        times = np.linspace(0., duration, Fixed.hall_samples_per_sensor)
        actual, expected = result.sol(times), reference.sol(times)
        angles = (Rotation.from_quat(actual[3:7].T, scalar_first=True) * Rotation.from_quat(expected[:4].T, scalar_first=True).inv()).magnitude()
        self.assertLess(np.max(angles), Fixed.adaptive_angle_tolerance)
        np.testing.assert_allclose(actual[10:13], expected[4:], atol=Fixed.adaptive_angular_velocity_tolerance, rtol=0)

    def test_passive_filter_does_not_add_force_evaluations(self):
        initial = self.dynamics.pack(self.plant, np.zeros(1))
        duration = 1 / Fixed.current_regulator_rate
        quiet = self.dynamics.integrate(0., duration, initial)[-1]
        count = self.dynamics.evaluations
        self.dynamics.field_signal = lambda plant, currents: np.array([Fixed.hall_linear_range])
        transient = self.dynamics.integrate(0., duration, initial)[-1]
        self.assertEqual(self.dynamics.evaluations - count, count)
        np.testing.assert_array_equal(quiet.y[:self.dynamics.work.stop], transient.y[:self.dynamics.work.stop])
        times = np.linspace(0., duration, Fixed.hall_samples_per_sensor)
        expected = Fixed.hall_linear_range * (1 - np.exp(-2 * np.pi * Fixed.hall_sensor_bandwidth * times))
        np.testing.assert_allclose(transient.sol(times)[self.dynamics.hall_filter][0], expected,
            atol=Fixed.adaptive_field_tolerance, rtol=0)

    def test_clamp_extinction_matches_exact_electrical_solution(self):
        self.drivers.currents[0] = Fixed.driver_channel_current * Fixed.coupling_excitation_fraction
        initial_current = self.drivers.currents.copy()
        initial = self.dynamics.pack(self.plant, np.zeros(1))
        duration = Fixed.airborne_check_duration
        segments = self.dynamics.integrate(0., duration, initial, enabled=False)
        voltage = self.drivers.terminal_voltage(False)[0]
        equilibrium = voltage / self.drivers.resistance
        exact = self.drivers.inductance / self.drivers.resistance * np.log((initial_current[0] - equilibrium) / -equilibrium)
        self.assertEqual(len(segments), 2)
        self.assertAlmostEqual(segments[0].t[-1], exact, delta=Fixed.adaptive_clock_tolerance)
        self.assertEqual(segments[-1].y[self.dynamics.current.start, -1], 0.)
        self.assertEqual(self.drivers.currents[0], initial_current[0])

    def test_simultaneous_clamp_extinction_cannot_reverse_another_channel(self):
        for signs in ((1, 1), (-1, -1), (1, -1)):
            self.drivers.currents[:2] = np.array(signs) * Fixed.driver_channel_current * Fixed.coupling_excitation_fraction
            initial = self.dynamics.pack(self.plant, np.zeros(1))
            segments = self.dynamics.integrate(0., Fixed.airborne_check_duration, initial, enabled=False)
            self.assertEqual(len(segments), 2)
            np.testing.assert_array_equal(segments[0].y[self.dynamics.current, -1], 0.)
            np.testing.assert_array_equal(segments[-1].y[self.dynamics.current, -1], 0.)
            self.assertFalse(np.any(self.dynamics.conducting))

    def test_distinct_clamp_extinction_groups_remain_separate(self):
        scale = Fixed.qualification_refinement_factor
        self.drivers.currents[:4] = np.array([1., 1 / scale, -1 / scale, -1.]) * Fixed.driver_channel_current * Fixed.coupling_excitation_fraction
        voltage = np.abs(self.drivers.terminal_voltage(False)[:4])
        equilibrium = voltage / self.drivers.resistance
        expected = self.drivers.inductance / self.drivers.resistance * np.log((np.abs(self.drivers.currents[:4]) + equilibrium) / equilibrium)
        segments = self.dynamics.integrate(0., Fixed.airborne_check_duration,
            self.dynamics.pack(self.plant, np.zeros(1)), enabled=False)
        self.assertEqual(len(segments), 3)
        self.assertAlmostEqual(segments[0].t[-1], expected[1], delta=Fixed.adaptive_clock_tolerance)
        self.assertAlmostEqual(segments[1].t[-1], expected[0], delta=Fixed.adaptive_clock_tolerance)
        intermediate = segments[0].y[self.dynamics.current, -1]
        np.testing.assert_array_equal(intermediate[1:3], 0.)
        self.assertGreater(intermediate[0], 0.)
        self.assertLess(intermediate[3], 0.)
        np.testing.assert_array_equal(segments[-1].y[self.dynamics.current, -1], 0.)

    def test_variable_powered_and_switched_against_independent_reference(self):
        for switched in (False, True):
            trial = VariableCouplingTrial(self.construction, self.plant, switched)
            dynamics = JointDynamics(self.plant, trial.drivers, 1, trial.coupling_matrix, lambda plant, currents: np.zeros(1))
            reference = variable_coupling_solution(trial, 'Radau', Fixed.qualification_refinement_factor)
            initial = dynamics.pack(self.plant, np.zeros(1))
            powered = dynamics.integrate(0., trial.switch_time, initial)
            segments = powered
            if switched:
                segments += dynamics.integrate(trial.switch_time, trial.duration, powered[-1].y[:, -1], enabled=False)
            times = np.concatenate([segment.t for segment in segments])
            states = np.concatenate([segment.y.T for segment in segments])
            actual = np.c_[states[:, 2] - trial.height, states[:, 9],
                Rotation.from_quat(states[:, 3:7], scalar_first=True).as_rotvec()[:, 2], states[:, 12],
                states[:, dynamics.current.start], states[:, dynamics.work.stop - 1]]
            expected = reference.sample(times)
            limits = np.array([Fixed.adaptive_position_tolerance, Fixed.adaptive_velocity_tolerance,
                Fixed.adaptive_angle_tolerance, Fixed.adaptive_angular_velocity_tolerance, Fixed.adaptive_current_tolerance])
            self.assertTrue(np.all(np.max(np.abs(actual[:, :5] - expected[:, :5]), axis=0) <= limits))
            self.assertLess(np.max(np.abs(variable_energy_residual(trial, actual))), dynamics.energy_budget)


def substitute_force(simulation):
    matrix = np.zeros((6, len(simulation.identities)))
    simulation.dynamics.force_matrix = lambda plant: matrix
    simulation.controller.nominal_matrix = matrix
    simulation.controller.command = Mock(return_value=dict(accepted=True, required_current_A=0.,
        currents=np.zeros(len(simulation.identities))))


def replay_checkpoint(path):
    simulation = JointAirborneSimulation.from_checkpoint(path)
    substitute_force(simulation)
    simulation.save_checkpoint(path.with_name('restored.json'))
    while not simulation.finished:
        simulation.advance()
    simulation.save_checkpoint(path.with_name('continued.json'))


class JointScheduleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.construction = magnetics.ConstructionModel()

    def test_pending_adc_samples_and_hardware_events_replay_in_fresh_process(self):
        original = JointAirborneSimulation(self.construction, Fixed.airborne_check_duration)
        substitute_force(original)
        while original.clock <= Fixed.hall_power_settle_time:
            original.advance()
        self.assertGreater(original.pending_scans[0]['sample_count'], 0)
        self.assertEqual(original.delivered_scans, 0)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'checkpoint.json'
            original.save_checkpoint(path)
            saved = json.loads(path.read_text())
            for key in ('source_sha256', 'construction', 'plant'):
                invalid = deepcopy(saved)
                if key == 'source_sha256':
                    invalid[key] = {}
                elif key == 'construction':
                    invalid[key]['piece_properties']['mass_g'] *= Fixed.qualification_refinement_factor
                else:
                    invalid[key]['scene_sha256'] = ''
                with self.assertRaises(ValueError):
                    original.restore_checkpoint(invalid)
                self.assertEqual(original.checkpoint_state(), saved)
            subprocess.run([sys.executable, '-c',
                'from pathlib import Path; import sys; from methodology.tests.test_joint_airborne import replay_checkpoint; replay_checkpoint(Path(sys.argv[1]))',
                str(path)], check=True)
            self.assertEqual(saved, json.loads(path.with_name('restored.json').read_text()))
            while not original.finished:
                original.advance()
            self.assertEqual(original.checkpoint_state(), json.loads(path.with_name('continued.json').read_text()))
        self.assertIsNone(original.stop_reason)
        self.assertGreater(original.delivered_scans, 0)
        self.assertEqual(original.clock, Fixed.airborne_check_duration)
        self.assertLessEqual(original.accounting['absolute_total_defect_J'], original.dynamics.energy_budget)


if __name__ == '__main__':
    unittest.main()

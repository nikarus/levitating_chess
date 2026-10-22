from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.integrate import quad

from model import Fixed, Inputs, Constants
from methodology import airborne, electrical, sensing


class ElectricalInterfaceTests(unittest.TestCase):
    def test_hardware_requirements_scale_acquisition_without_changing_accuracy(self):
        first = electrical.hardware_interface_requirements(1, 1)
        second = electrical.hardware_interface_requirements(2, 2)
        for section in ('current_acquisition', 'hall_acquisition'):
            for key in ('aggregate_samples_per_s', 'raw_sample_payload_bits_per_s'):
                self.assertEqual(second[section][key], 2 * first[section][key])
        self.assertEqual(first['current_acquisition']['quantum_A'], electrical.current_quantum())
        self.assertEqual(first['current_acquisition']['quantum_A'], second['current_acquisition']['quantum_A'])
        sensors = electrical.VirtualCurrentSensors(('a', 'b'))
        self.assertLessEqual(np.max(np.abs(sensors.offset)), first['current_acquisition']['residual_offset_bound_A'])
        self.assertEqual(first['current_acquisition']['input_referred_noise_rms_V'],
            Fixed.current_feedback_noise * Fixed.current_sense_resistance)

    def test_hardware_requirements_reject_invalid_channel_counts(self):
        for count in (0, -1, True, 1.5):
            with self.assertRaises(ValueError):
                electrical.hardware_interface_requirements(count, 1)
            with self.assertRaises(ValueError):
                electrical.hardware_interface_requirements(1, count)

    def setUp(self):
        self.identities = ('coil-a', 'coil-b')
        construction = SimpleNamespace(configuration=SimpleNamespace(resistance=1., turns=10, coil_height=1.),
                                       coil=SimpleNamespace(footprint_area=100.))
        self.drivers = electrical.CoilDrivers(construction, self.identities)
        self.sensors = electrical.VirtualCurrentSensors(self.identities)

    def test_current_adc_quantization_and_saturation_are_explicit(self):
        self.sensors.offset[:] = 0
        self.sensors.gain[:] = 1
        current = np.array([1., -2.])
        with patch.object(Fixed, 'current_feedback_noise', 0):
            reading = self.sensors.sample(current, 0)
            np.testing.assert_allclose(reading.decode(0)[0], current, atol=electrical.current_quantum() / 2, rtol=0)
            saturated = self.sensors.sample(np.full(2, 2 * Fixed.driver_peak_current), 0)
        with self.assertRaises(ValueError):
            saturated.decode(0)

    def test_current_readout_rejects_future_missing_and_invalid_data(self):
        reading = self.sensors.sample(np.zeros(2), 1)
        invalid = [replace(reading, available_time_s=2), replace(reading, available_time_s=0),
                   replace(reading, identities=('coil-a', 'coil-a')), replace(reading, adc_codes=np.array([[np.nan, 1]])),
                   replace(reading, sample_times_s=np.array([1., 0.]), adc_codes=np.repeat(reading.adc_codes, 2, axis=0))]
        for candidate in invalid:
            with self.assertRaises(ValueError):
                candidate.decode(1)
        with self.assertRaises(ValueError):
            self.drivers.regulate(reading, 2)
        with self.assertRaises(ValueError):
            self.drivers.regulate(replace(reading, identities=('wrong', 'coil-b')), 1)

    def test_regulator_tracks_measured_current_not_hidden_true_current(self):
        self.sensors.offset[:] = Fixed.current_sense_offset_residual / Fixed.current_sense_resistance
        self.sensors.gain[:] = 1
        self.drivers.set_command(np.array([1., -1.]))
        period = 1 / Fixed.current_regulator_rate
        with patch.object(Fixed, 'current_feedback_noise', 0):
            for step in range(round(Fixed.hall_tracking_acquisition_duration / period)):
                reading = self.sensors.sample(self.drivers.currents, step * period)
                self.drivers.regulate(reading, step * period)
                for _ in range(Fixed.qualification_integration_substeps):
                    self.drivers.advance(np.zeros(2), period / Fixed.qualification_integration_substeps)
            measured = self.sensors.sample(self.drivers.currents, Fixed.hall_tracking_acquisition_duration).decode(Fixed.hall_tracking_acquisition_duration)[0]
        np.testing.assert_allclose(measured, [1, -1], atol=2 * electrical.current_quantum(), rtol=0)
        np.testing.assert_allclose(self.drivers.currents, np.array([1, -1]) - self.sensors.offset,
                                   atol=2 * electrical.current_quantum(), rtol=0)
        self.assertLessEqual(np.max(np.abs(self.drivers.voltage)), self.drivers.voltage_limit)

    def test_sensor_sequence_is_reproducible(self):
        other = electrical.VirtualCurrentSensors(self.identities)
        for time in np.arange(3) / Fixed.current_regulator_rate:
            np.testing.assert_array_equal(self.sensors.sample(np.ones(2), time).adc_codes,
                                          other.sample(np.ones(2), time).adc_codes)

    def test_energy_reference_matches_resistive_equilibrium_and_empty_open_coil(self):
        from methodology.coupling_verification import interval_energy
        driver = self.drivers
        current = driver.current_limit / 2
        voltage = driver.resistance * current
        period = Fixed.adaptive_max_step
        result = interval_energy(current, voltage, 0., driver.resistance, driver.inductance, period)
        expected = voltage * current * period
        self.assertAlmostEqual(result['input_J'], expected, delta=expected * Fixed.motion_verification_tolerance)
        self.assertAlmostEqual(result['resistive_J'], expected, delta=expected * Fixed.motion_verification_tolerance)
        self.assertEqual(result['mechanical_J'], 0.)
        result = interval_energy(0., voltage, 0., driver.resistance, driver.inductance, period, False)
        self.assertTrue(all(value == 0 for value in result.values()))

    def test_driver_mean_current_satisfies_integrated_circuit_equation(self):
        initial = np.array([1., -1.])
        self.drivers.currents = initial.copy()
        self.drivers.voltage = np.array([-1., 1.])
        back_emf = np.array([0.5, -0.5])
        period = 1 / Fixed.current_regulator_rate
        average = self.drivers.advance(back_emf, period)
        expected = (self.drivers.voltage - back_emf
            - self.drivers.inductance * (self.drivers.currents - initial) / period) / self.drivers.resistance
        np.testing.assert_allclose(average, expected, atol=Fixed.motion_verification_tolerance, rtol=0)

    def test_disabled_current_impulse_and_hall_filter_match_independent_quadrature(self):
        initial = np.array([1., -2.])
        self.drivers.currents = initial.copy()
        self.drivers.voltage = np.array([1., -1.])
        enabled = np.zeros(2, dtype=bool)
        voltage = self.drivers.terminal_voltage(enabled)
        back_emf = np.array([0.2, -0.4])
        tau = self.drivers.inductance / self.drivers.resistance
        equilibrium = (voltage - back_emf) / self.drivers.resistance
        stops = tau * np.log((initial - equilibrium) / -equilibrium)
        period = 2 * max(stops)
        filter_state = initial / 2
        filtered = sensing.filtered_coil_current_step(filter_state, initial, voltage, back_emf,
            self.drivers.resistance, self.drivers.inductance, period, enabled)
        average = self.drivers.advance(back_emf, period, enabled)
        np.testing.assert_array_equal(self.drivers.currents, np.zeros(2))
        rate = 2 * np.pi * Fixed.hall_sensor_bandwidth
        for index, stop in enumerate(stops):
            def signal(time):
                return equilibrium[index] + (initial[index] - equilibrium[index]) * np.exp(-time / tau)
            impulse = quad(signal, 0, stop)[0]
            expected_filter = filter_state[index] * np.exp(-rate * period) + quad(
                lambda time: rate * np.exp(-rate * (period-time)) * signal(time), 0, stop)[0]
            self.assertAlmostEqual(average[index] * period, impulse, delta=Fixed.motion_verification_tolerance)
            self.assertAlmostEqual(filtered[index], expected_filter, delta=Fixed.motion_verification_tolerance)
        self.drivers.currents = initial.copy()
        before = self.drivers.currents.copy()
        self.drivers.terminal_voltage(enabled)
        np.testing.assert_array_equal(before, self.drivers.currents)
        half_average = self.drivers.advance(back_emf, period / 2, enabled)
        second_average = self.drivers.advance(back_emf, period / 2, enabled)
        np.testing.assert_allclose((half_average + second_average) / 2, average,
            atol=Fixed.motion_verification_tolerance, rtol=0)

    def test_driver_rejects_unknown_switch_states_and_unsupported_off_state_emf(self):
        for mask in ([1, 0], [True], [True, False, True]):
            with self.assertRaisesRegex(ValueError, 'mask'):
                self.drivers.terminal_voltage(mask)
        with self.assertRaisesRegex(ValueError, 're-conduction'):
            self.drivers.advance(np.full(2, Fixed.driver_peak_current * self.drivers.voltage_limit),
                Fixed.adaptive_reference_step, enabled=False)

    def test_euler_rate_conversion_agrees_with_rotational_increment(self):
        pose = np.array([0, 0, 0, .1, -.2, .3])
        rates = np.array([.4, -.5, .6])
        period = Fixed.motion_time_step
        first = Rotation.from_euler('xyz', pose[3:] - rates * period / 2)
        last = Rotation.from_euler('xyz', pose[3:] + rates * period / 2)
        expected = (last * first.inv()).as_rotvec() / period
        np.testing.assert_allclose(airborne.euler_rates_to_world(pose, rates), expected, atol=Fixed.hall_fit_tolerance, rtol=0)


class VariableReferenceTests(unittest.TestCase):
    def test_uncoupled_reference_reduces_to_ballistic_motion_and_exact_rl_extinction(self):
        from methodology.coupling_verification import variable_coupling_reference, variable_energy_residual
        duration = Fixed.coupling_variable_duration
        initial = np.array([0., 1., Fixed.qualification_angle_error, 1., 1., 0.])
        trial = SimpleNamespace(mass=1., inertia=1., span=1., current=1., duration=duration,
            initial=initial, vertical_coupling=0., angular_coupling=0.,
            plant=SimpleNamespace(geometry=SimpleNamespace(gravity=Constants.gravity)),
            drivers=SimpleNamespace(resistance=1., inductance=duration, voltage=np.array([1.])))
        times = np.linspace(0., duration, len(Fixed.coupling_refinements))
        for switched in (False, True):
            trial.switch_time = duration / 2 if switched else duration
            result, extinction = variable_coupling_reference(trial, times, 'DOP853', 1)
            np.testing.assert_allclose(result[:, 0], times - trial.plant.geometry.gravity * times ** 2 / 2,
                rtol=0, atol=Fixed.motion_verification_tolerance)
            np.testing.assert_allclose(result[:, 2], initial[2] + times, rtol=0, atol=Fixed.motion_verification_tolerance)
            np.testing.assert_allclose(variable_energy_residual(trial, result), 0., rtol=0, atol=Fixed.motion_verification_tolerance)
            if switched:
                clamp = Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction
                exact = trial.switch_time + trial.drivers.inductance * np.log1p(initial[4] / clamp)
                self.assertAlmostEqual(extinction, exact, delta=Fixed.motion_verification_tolerance)
                self.assertEqual(result[-1, 4], 0.)
            else:
                self.assertIsNone(extinction)
                np.testing.assert_allclose(result[:, 4], initial[4], rtol=0, atol=Fixed.motion_verification_tolerance)


if __name__ == '__main__':
    unittest.main()

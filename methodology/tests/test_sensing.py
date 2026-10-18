from dataclasses import replace
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.integrate import quad, solve_ivp

import levitation_sim
from model import Fixed
from methodology import electrical, motion, sensing
from methodology.cad import magnetics



class JointRecordingTests(unittest.TestCase):
    def test_recording_preserves_results_and_dense_coefficients(self):
        from methodology.joint_airborne import FilteredTrajectory
        from methodology.numerical_accuracy import JointTrajectoryRecorder
        from scipy.integrate._ivp.rk import RkDenseOutput
        solution = solve_ivp(lambda time, state: state, (0., 1.), [1.], dense_output=True)
        filtered = solution.sol
        solution.sol = FilteredTrajectory(filtered, filtered)
        integrate = Mock(return_value=[solution])
        packet = object()
        encode = Mock(return_value=packet)
        simulation = SimpleNamespace(dynamics=SimpleNamespace(integrate=integrate), sensors=SimpleNamespace(encode_signal=encode))
        recorder = JointTrajectoryRecorder(simulation)
        self.assertIs(recorder.integrate('argument')[0], solution)
        integrate.assert_called_once_with('argument')
        signal = np.array([[1., 2.]])
        self.assertIs(recorder.encode_signal(signal, 0.), packet)
        self.assertIs(encode.call_args.args[0], signal)
        signal[:] = 0
        self.assertEqual(recorder.scans[0]['samples_T'], [[1., 2.]])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'dense.npz'
            recorder.save_dense(path)
            with np.load(path, allow_pickle=False) as saved:
                np.testing.assert_array_equal(saved['segment_bounds_s'], [[0., 1.]])
                for component in ('physical', 'filtered'):
                    self.assertEqual(saved[component + '_segment_counts'].tolist(), [len(filtered.interpolants)])
                    restored = RkDenseOutput(saved[component + '_t_old'][-1], saved[component + '_t'][-1],
                        saved[component + '_y_old'][-1], saved[component + '_Q'][-1])
                    np.testing.assert_array_equal(restored(solution.t[-1]), filtered(solution.t[-1]))

class AcceleratedSensingBudgetTests(unittest.TestCase):
    def test_opposite_component_errors_cannot_cancel(self):
        from methodology.numerical_accuracy import accelerated_sensor_comparison
        zero = np.zeros((2, 1))
        opposite = np.array([[1.], [-1.]])
        result = accelerated_sensor_comparison(np.zeros(1),
            dict(direct=[zero, zero], atlas=[opposite, opposite], refined=[zero, zero]),
            np.zeros(1), dict(limit_T=1.))
        self.assertEqual(result['combined_bound_T'], [2.])
        self.assertFalse(result['passed'])

    def test_refinement_filter_and_spatial_errors_add(self):
        from methodology.numerical_accuracy import accelerated_sensor_comparison
        zero = np.zeros((2, 1))
        nominal = np.array([[1.], [0.]])
        atlas = np.array([[2.], [0.]])
        result = accelerated_sensor_comparison(np.array([3.]),
            dict(direct=[nominal, nominal], atlas=[atlas, atlas], refined=[zero, zero]),
            np.array([1.]), dict(limit_T=4.))
        self.assertEqual(result['combined_bound_T'], [4.])
        self.assertTrue(result['passed'])

class HallFilterTests(unittest.TestCase):
    def test_independent_scan_reference_matches_analytic_power_up_and_component_split(self):
        from methodology.numerical_accuracy import reference_filtered_scan
        components = Fixed.hall_output_noise * np.array([1., -1., -1., 1.])
        channels = len(components) // 2
        samples, _, _ = sensing.hall_readout()
        times = Fixed.hall_power_settle_time + np.arange(channels * samples).reshape(channels, samples) / Fixed.hall_adc_sample_rate
        response = (1 - np.exp(-2 * np.pi * Fixed.hall_sensor_bandwidth * times)).mean(axis=1)
        expected = components.reshape(2, channels) * response
        for method in ('DOP853', 'Radau'):
            result = reference_filtered_scan(lambda time: components, 0., channels, [], method)
            np.testing.assert_allclose(result, expected, rtol=Fixed.coupling_relative_tolerance, atol=0)

    def test_independent_scan_reference_preserves_decay_after_input_extinction(self):
        from methodology.numerical_accuracy import reference_filtered_scan
        amplitude = Fixed.hall_output_noise
        rate = 2 * np.pi * Fixed.hall_sensor_bandwidth
        tau = Fixed.qualification_refinement_factor / rate
        stop = Fixed.hall_power_settle_time / 2
        samples, _, _ = sensing.hall_readout()
        times = Fixed.hall_power_settle_time + np.arange(samples) / Fixed.hall_adc_sample_rate
        at_stop = amplitude * rate / (rate - 1 / tau) * (np.exp(-stop / tau) - np.exp(-rate * stop))
        expected = at_stop * np.exp(-rate * (times - stop)).mean()
        for method in ('DOP853', 'Radau'):
            result = reference_filtered_scan(lambda time: np.array([0., amplitude * np.exp(-time / tau) if time < stop else 0.]),
                0., 1, [stop], method)
            self.assertEqual(result[0, 0], 0.)
            self.assertAlmostEqual(result[1, 0] / expected, 1., delta=Fixed.coupling_relative_tolerance)

    def test_converter_precision_preserves_scan_timing_and_noise_correlation(self):
        reference_samples, reference_correlation, reference_quantum = sensing.hall_readout()
        reference_bits = Fixed.hall_adc_native_bits
        for bits in Fixed.hall_adc_precision_candidates:
            with patch.object(Fixed, 'hall_adc_native_bits', bits):
                samples, correlation, quantum = sensing.hall_readout()
            self.assertEqual(samples, reference_samples)
            np.testing.assert_array_equal(correlation, reference_correlation)
            self.assertEqual(quantum, reference_quantum * 2 ** (reference_bits - bits))

    def test_exponential_input_matches_independent_convolution_and_refinement(self):
        rate = 2 * np.pi * Fixed.hall_sensor_bandwidth
        duration = 1 / Fixed.pose_feedback_rate
        start = Fixed.hall_power_settle_time
        samples = sensing.hall_readout()[0]
        sample_times = start + Fixed.hall_power_settle_time + np.arange(samples) / Fixed.hall_adc_sample_rate
        for time_constant in (1 / rate, Fixed.qualification_refinement_factor / rate):
            expected = np.array([quad(lambda time: rate * np.exp(-rate * (end - time))
                * (1 - np.exp(-time / time_constant)), start, end)[0] for end in sample_times])
            for segments in (1, Fixed.qualification_refinement_factor):
                times = np.linspace(0, duration, segments + 1)
                fields = (1 - np.exp(-times / time_constant))[:, None]
                actual = sensing.filtered_scan(times, fields, start, time_constant)[0]
                np.testing.assert_allclose(actual, expected, atol=Fixed.motion_verification_tolerance, rtol=0)

    def test_exponential_reconstruction_rejects_invalid_time_constant(self):
        for time_constant in (0, -1, np.nan, np.inf):
            with self.assertRaisesRegex(ValueError, 'time constant'):
                sensing.filtered_scan(np.array([0., 1.]), np.ones((2, 1)), 0, time_constant)

class HallInterfaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.construction = magnetics.ConstructionModel()
        cls.plant = motion.MotionPlant(cls.construction)
        cls.fields = sensing.HallFieldModel(cls.construction)

    def setUp(self):
        self.plant.reset()
        self.sensors = sensing.VirtualHallArray(self.fields)
        self.estimator = sensing.HallPoseEstimator(self.fields)

    def test_sensor_plane_uses_single_cad_winding_layer(self):
        expected = (-self.plant.geometry.gap - self.construction.configuration.coil_height / 1000
                    - self.plant.geometry.pcb_thickness - self.plant.geometry.hall_standoff)
        np.testing.assert_allclose(self.fields.points[:, 2], expected, atol=Fixed.motion_verification_tolerance, rtol=0)
        self.assertEqual(set(self.fields.identities), self.plant.physical_identities)

    def test_adc_quantization_and_sequential_scan_timestamps(self):
        self.sensors.offset[:] = 0
        self.sensors.gain[:] = 1
        field = self.fields.magnet_field(np.zeros(6))
        with patch.object(Fixed, 'hall_output_noise', 0):
            packet = self.sensors.digitize(field, 0)
        samples, _, quantum = sensing.hall_readout()
        decoded = (packet.adc_codes.mean(axis=1) + 0.5) * quantum - Fixed.hall_supply_voltage / 2 / Fixed.hall_sensitivity
        self.assertLessEqual(np.max(np.abs(decoded - field)), quantum / 2)
        self.assertEqual(packet.adc_codes.shape, (len(field), samples))
        np.testing.assert_allclose(np.diff(packet.sample_times_s.ravel()), 1 / Fixed.hall_adc_sample_rate,
                                   atol=Fixed.motion_verification_tolerance, rtol=0)
        self.assertAlmostEqual(packet.available_time_s - packet.sample_times_s.max(),
                               Fixed.qualification_sensor_latency + 1 / Fixed.hall_adc_sample_rate)
        self.assertLess(np.max(np.abs(decoded)), Fixed.hall_linear_range - quantum)

    def test_coil_field_is_added_before_adc_and_can_be_subtracted(self):
        self.sensors.offset[:] = 0
        self.sensors.gain[:] = 1
        self.sensors.actual_coil_field = self.fields.coil_field.copy()
        currents = {identity: Fixed.driver_channel_current / 2 for identity in self.fields.identities}
        with patch.object(Fixed, 'hall_output_noise', 0):
            packet = self.sensors.sample_static(self.plant, currents)
        _, _, quantum = sensing.hall_readout()
        decoded = (packet.adc_codes.mean(axis=1) + 0.5) * quantum - Fixed.hall_supply_voltage / 2 / Fixed.hall_sensitivity
        corrected = decoded - self.fields.coil_field @ self.fields.current_vector(currents)
        np.testing.assert_allclose(corrected, self.fields.magnet_field(np.zeros(6)), atol=quantum / 2, rtol=0)

    def test_measured_current_history_removes_changing_coil_field(self):
        self.sensors.offset[:] = 0
        self.sensors.gain[:] = 1
        self.sensors.actual_coil_field = self.fields.coil_field.copy()
        count = len(self.fields.identities)
        current_sensors = electrical.VirtualCurrentSensors(self.fields.identities)
        current_sensors.offset[:] = 0
        current_sensors.gain[:] = 1
        times = np.arange(round(Fixed.qualification_sensor_latency * Fixed.current_regulator_rate) + 1) / Fixed.current_regulator_rate
        currents = np.linspace(-Fixed.driver_channel_current / 2, Fixed.driver_channel_current / 2, len(times))[:, None] * np.ones((1, count))
        magnet_field = self.fields.magnet_field(np.zeros(6))
        fields = magnet_field + currents @ self.fields.coil_field.T
        with patch.object(Fixed, 'hall_output_noise', 0), patch.object(Fixed, 'current_feedback_noise', 0):
            packet = self.sensors.sample_moving(times, fields, 0)
            readouts = [current_sensors.sample(current, time) for current, time in zip(currents, times)]
        history = electrical.CurrentReadings(self.fields.identities, times, np.vstack([r.adc_codes for r in readouts]), times[-1])
        corrected_field, _, _ = sensing.measured_coil_fields(self.fields, history, packet)
        samples, _, quantum = sensing.hall_readout()
        decoded = (packet.adc_codes.mean(axis=1) + .5) * quantum - Fixed.hall_supply_voltage / 2 / Fixed.hall_sensitivity
        expected = sensing.filtered_scan(times, np.tile(magnet_field, (len(times), 1)), 0).mean(axis=1)
        bound = quantum / 2 + np.abs(self.fields.coil_field).sum(axis=1) * electrical.current_quantum() / 2
        self.assertTrue(np.all(np.abs(decoded - corrected_field - expected) <= bound))
        snapshot = self.fields.coil_field @ currents[-1]
        self.assertGreater(np.linalg.norm(decoded - snapshot - expected), np.linalg.norm(bound))

    def test_current_history_rejects_unavailable_or_missing_samples(self):
        packet = self.sensors.sample_static(self.plant, {})
        samples = np.arange(round(Fixed.qualification_sensor_latency * Fixed.current_regulator_rate) + 1) / Fixed.current_regulator_rate
        sensor = electrical.VirtualCurrentSensors(self.fields.identities)
        codes = np.vstack([sensor.sample(np.zeros(len(self.fields.identities)), time).adc_codes for time in samples])
        history = electrical.CurrentReadings(self.fields.identities, samples, codes, samples[-1])
        for invalid in (replace(history, available_time_s=packet.available_time_s + Fixed.qualification_sensor_latency),
                        replace(history, sample_times_s=samples[::2], adc_codes=codes[::2]),
                        replace(history, sample_times_s=samples[1:], adc_codes=codes[1:])):
            with self.assertRaises(ValueError):
                sensing.measured_coil_fields(self.fields, invalid, packet)

    def test_initializer_recovers_unseeded_six_axis_pose_without_mutating_plant(self):
        pose = np.array([0.002, -0.001, 0.0005, *np.radians([1, -1, -120])])
        self.plant.reset(position=pose[:3] + [0, 0, self.fields.geometry.center_of_mass_height],
                         rotation=Rotation.from_euler('xyz', pose[3:]))
        before = self.plant.data.qpos.copy()
        packet = self.sensors.sample_static(self.plant, {})
        estimate = self.estimator.estimate(packet, {})
        self.assertTrue(estimate['accepted'], estimate)
        error = np.asarray(estimate['pose']) - pose
        self.assertLess(np.linalg.norm(error[:3]), Fixed.qualification_position_error)
        self.assertLess((Rotation.from_euler('xyz', estimate['pose'][3:]) * Rotation.from_euler('xyz', pose[3:]).inv()).magnitude(),
                        Fixed.qualification_angle_error)
        np.testing.assert_array_equal(self.plant.data.qpos, before)
        self.assertEqual(estimate['observability']['rank6'], 6)

    def test_saturation_and_adc_rails_do_not_produce_a_pose(self):
        packet = self.sensors.digitize(np.full(len(self.fields.points), 2 * Fixed.hall_linear_range), 0)
        self.assertIsNone(self.estimator.estimate(packet, {})['pose'])
        self.assertEqual(self.estimator.estimate(packet, {})['reason'], 'saturated')
        rail_packet = replace(packet, adc_codes=np.zeros_like(packet.adc_codes))
        self.assertFalse(self.estimator.estimate(rail_packet, {})['accepted'])

    def test_pose_insensitive_readings_are_rejected(self):
        field = np.zeros(len(self.fields.points))
        packet = self.sensors.digitize(field, 0)
        with patch.object(self.fields, 'magnet_field', return_value=field):
            estimate = self.estimator.estimate(packet, {})
        self.assertFalse(estimate['accepted'])
        self.assertIsNone(estimate['pose'])
        self.assertEqual(estimate['reason'], 'unobservable')

    def test_invalid_packets_and_unknown_coils_fail_loudly(self):
        packet = self.sensors.sample_static(self.plant, {})
        with self.assertRaisesRegex(ValueError, 'timestamps'):
            self.estimator.estimate(replace(packet, available_time_s=-1), {})
        with self.assertRaisesRegex(ValueError, 'ADC'):
            self.estimator.estimate(replace(packet, adc_codes=np.full_like(packet.adc_codes, -1)), {})
        with self.assertRaisesRegex(ValueError, 'outside'):
            self.fields.current_vector({('missing',): 0})
        with self.assertRaisesRegex(ValueError, 'rating'):
            self.fields.current_vector({self.fields.identities[0]: 2 * Fixed.driver_channel_current})

    def test_virtual_errors_and_noise_replay_with_fixed_seed(self):
        second = sensing.VirtualHallArray(self.fields)
        first_packet = self.sensors.sample_static(self.plant, {})
        second_packet = second.sample_static(self.plant, {})
        np.testing.assert_array_equal(first_packet.adc_codes, second_packet.adc_codes)
        _, correlation, _ = sensing.hall_readout()
        self.assertGreater(correlation.mean(), 1 / len(correlation))

    def test_batched_body_frame_fields_match_individually_rotated_magnets(self):
        poses = np.tile(np.r_[0.002, -0.001, 0.0005, np.radians(Fixed.motion_check_orientation_deg)], (len(self.fields.points), 1))
        expected = []
        for pose, point in zip(poses, self.fields.points):
            levitation_sim.place_piece(self.fields.magnets, *pose)
            expected.append(self.fields.magnets.collection.getB(point)[2])
        actual = self.fields.magnet_field(poses)
        np.testing.assert_allclose(actual, expected, atol=Fixed.motion_verification_tolerance, rtol=0)

    def test_analog_filter_matches_step_and_ramp_under_time_refinement(self):
        period = Fixed.hall_power_settle_time
        rate = 2 * np.pi * Fixed.hall_sensor_bandwidth
        step = sensing.hall_filter_step(0, 1, 1, period)
        self.assertAlmostEqual(step, 1 - np.exp(-rate * period))
        ramp = sensing.hall_filter_step(0, 0, 1, period)
        refined = sensing.hall_filter_step(sensing.hall_filter_step(0, 0, 0.5, period / 2), 0.5, 1, period / 2)
        self.assertAlmostEqual(ramp, 1 - (1 - np.exp(-rate * period)) / (rate * period))
        self.assertAlmostEqual(ramp, refined)

    def test_moving_adc_samples_a_filtered_ramp_at_each_channel_time(self):
        self.sensors.offset[:] = 0
        self.sensors.gain[:] = 1
        times = np.array([0, 1 / Fixed.pose_feedback_rate])
        slope = Fixed.hall_linear_range * Fixed.pose_controller_frequency
        fields = np.repeat((times * slope)[:, None], len(self.fields.points), axis=1)
        with patch.object(Fixed, 'hall_output_noise', 0):
            packet = self.sensors.sample_moving(times, fields, 0)
        rate = 2 * np.pi * Fixed.hall_sensor_bandwidth
        expected = slope * (packet.sample_times_s - (1 - np.exp(-rate * packet.sample_times_s)) / rate)
        _, _, quantum = sensing.hall_readout()
        decoded = (packet.adc_codes + 0.5) * quantum - Fixed.hall_supply_voltage / 2 / Fixed.hall_sensitivity
        np.testing.assert_allclose(decoded, expected, atol=quantum / 2, rtol=0)
        self.assertGreater(decoded[-1].mean(), decoded[0].mean())
        with self.assertRaisesRegex(ValueError, 'trace'):
            self.sensors.sample_moving(times, fields, times[-1])

    def test_tracker_never_reads_a_packet_before_delivery_or_after_expiry(self):
        estimator = Mock(field_model=self.fields)
        tracker = sensing.HallTracker(estimator)
        packet = self.sensors.sample_static(self.plant, {})
        with self.assertRaisesRegex(ValueError, 'premature'):
            tracker.consume(packet, {}, packet.sample_times_s.mean())
        self.assertEqual(tracker.consume(packet, {}, packet.available_time_s + 2 * Fixed.hall_tracking_max_scan_gap)['reason'], 'late_packet')
        estimator.estimate.assert_not_called()

    def test_observer_predicts_to_delivery_using_only_measured_pose_history(self):
        estimator = Mock(field_model=self.fields)
        tracker = sensing.HallTracker(estimator)
        velocity = np.array(Fixed.hall_tracking_velocity)
        packet = self.sensors.sample_static(self.plant, {})
        for index in range(round(Fixed.qualification_recovery_duration * Fixed.pose_feedback_rate)):
            start = index / Fixed.pose_feedback_rate
            frame = replace(packet, sample_times_s=packet.sample_times_s + start, available_time_s=packet.available_time_s + start)
            pose = velocity * frame.sample_times_s.mean()
            estimator.estimate.return_value = dict(accepted=True, pose=pose.tolist())
            result = tracker.consume(frame, {}, frame.available_time_s)
            if index == round(Fixed.hall_tracking_duration * Fixed.pose_feedback_rate) - 1:
                error = np.array(result['pose']) - velocity * frame.available_time_s
                self.assertLessEqual(np.linalg.norm(error[:3]), Fixed.qualification_position_error)
                self.assertLessEqual(np.linalg.norm(error[3:]), Fixed.qualification_angle_error)
        self.assertTrue(result['valid'])
        np.testing.assert_allclose(result['pose'], velocity * frame.available_time_s, atol=Fixed.motion_verification_tolerance, rtol=0)

    def test_tracker_loses_stale_or_rejected_measurements_and_reacquires(self):
        estimator = Mock(field_model=self.fields)
        estimator.estimate.return_value = dict(accepted=True, pose=[0] * 6)
        tracker = sensing.HallTracker(estimator)
        packet = self.sensors.sample_static(self.plant, {})
        tracker.consume(packet, {}, packet.available_time_s)
        late = packet.available_time_s + 2 * Fixed.hall_tracking_max_scan_gap
        self.assertEqual(tracker.predict(late)['reason'], 'stale')
        next_packet = replace(packet, sample_times_s=packet.sample_times_s + late, available_time_s=packet.available_time_s + late)
        self.assertEqual(tracker.consume(next_packet, {}, next_packet.available_time_s)['reason'], 'acquiring')
        self.assertIsNone(estimator.estimate.call_args.args[2])
        estimator.estimate.return_value = dict(accepted=False, reason='saturated')
        rejected = replace(next_packet, sample_times_s=next_packet.sample_times_s + late, available_time_s=next_packet.available_time_s + late)
        self.assertEqual(tracker.consume(rejected, {}, rejected.available_time_s)['reason'], 'saturated')
        self.assertFalse(tracker.predict(rejected.available_time_s)['valid'])

    def test_contact_prediction_uses_cad_surface_without_contact_truth(self):
        tracker = sensing.HallTracker(self.estimator)
        pose = np.zeros(6)
        pose[2] = -self.fields.geometry.gap
        pose[3:] = np.radians(Fixed.motion_check_orientation_deg)
        velocity = np.array(Fixed.hall_tracking_velocity)
        velocity[2] = -1
        constrained, corrected_velocity = tracker.constrain_contact(pose.copy(), velocity.copy())
        self.plant.reset(position=constrained[:3] + [0, 0, self.fields.geometry.center_of_mass_height],
                         rotation=Rotation.from_euler('xyz', constrained[3:]))
        skin = self.plant.model.geom('Piece_bottom_skin').id
        axis_z = self.plant.data.geom_xmat[skin].reshape(3, 3)[2, 2]
        bottom = (self.plant.data.geom_xpos[skin, 2] - self.plant.model.geom_size[skin, 1] * abs(axis_z)
                  - self.plant.model.geom_size[skin, 0] * np.sqrt(1 - axis_z ** 2))
        self.assertAlmostEqual(bottom, self.plant.metadata['board_top_m'])
        self.assertEqual(corrected_velocity[2], 0)
        np.testing.assert_array_equal(corrected_velocity[[0, 1, 3, 4, 5]], velocity[[0, 1, 3, 4, 5]])


if __name__ == '__main__':
    unittest.main()

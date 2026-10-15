import json
from copy import deepcopy
from contextlib import nullcontext
from datetime import datetime, timezone
from pathlib import Path
import tempfile
from threading import Event
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

from model import Fixed
from methodology import airborne, airborne_diagnostics, airborne_verification, sensing, qualification
from methodology.cad import magnetics


class AirborneReplayTests(unittest.TestCase):
    def test_single_count_intervention_preserves_clock_and_original_frame(self):
        codes = np.array([[100, 101], [102, 103]])
        changed = codes.copy()
        changed[1, 0] += 1
        difference = airborne_diagnostics.single_count_difference(codes, changed)
        times = np.array([[0., 1.], [2., 3.]])
        frame = sensing.HallFrame(codes, times, 4.)
        result = airborne_diagnostics.changed_count_frame(frame, codes, difference)
        np.testing.assert_array_equal(result.adc_codes, changed)
        np.testing.assert_array_equal(frame.adc_codes, codes)
        self.assertIs(result.sample_times_s, times)
        self.assertEqual(result.available_time_s, frame.available_time_s)
        with self.assertRaisesRegex(ValueError, 'recorded packet'):
            airborne_diagnostics.changed_count_frame(frame, changed, difference)

    def test_count_intervention_rejects_multiple_samples_or_counts(self):
        original = np.zeros((2, 2), dtype=int)
        for changed in (original, np.ones((2, 2), dtype=int), np.array([[2, 0], [0, 0]])):
            with self.assertRaisesRegex(ValueError, 'exactly one'):
                airborne_diagnostics.single_count_difference(original, changed)

    def test_current_attribution_separates_force_change_from_null_current(self):
        geometry = patch.object(airborne_diagnostics.levitation_sim, 'G', SimpleNamespace(settings=Fixed), create=True)
        geometry.start()
        self.addCleanup(geometry.stop)
        matrix = np.c_[np.eye(6), np.eye(6)]
        null = np.r_[np.ones(6), -np.ones(6)]
        result = airborne_diagnostics.current_wrench_components(matrix, null)
        np.testing.assert_allclose(result['wrench'], 0, atol=Fixed.motion_verification_tolerance)
        self.assertAlmostEqual(result['active_current_norm_A'], 0)
        active = np.ones(12)
        combined = airborne_diagnostics.current_wrench_components(matrix, null + active)
        self.assertAlmostEqual(combined['active_current_norm_A'], np.linalg.norm(active))
        self.assertAlmostEqual(combined['null_current_norm_A'], np.linalg.norm(null))
        scaled = airborne_diagnostics.current_wrench_components(matrix * np.arange(1, 7)[:, None], null + active)
        self.assertAlmostEqual(scaled['active_current_norm_A'], combined['active_current_norm_A'])
        self.assertAlmostEqual(scaled['null_current_norm_A'], combined['null_current_norm_A'])

    def test_adaptive_replay_uses_attempt_starts_not_half_steps_or_pending_endpoint(self):
        random = np.random.default_rng(Fixed.hall_virtual_seed)
        period = 1 / Fixed.current_regulator_rate
        starts = [0., period, period + period / 2]
        simulation = SimpleNamespace(identities=(0,), fields=SimpleNamespace(points=[0]),
            sensors=SimpleNamespace(random=random))
        state = dict(adaptive=dict(clock=2 * period, adaptation=dict(intervals=[dict(start_s=value) for value in starts])),
            field_times=[0., period / 2, period, period + period / 4, starts[-1], 2 * period],
            field_values=[[0.]] * 6, trace=[dict(time_s=0.)],
            readouts=[dict(sample_times_s=[value], adc_codes=[[index]], available_time_s=value)
                      for index, value in enumerate(starts[:2])],
            sensors=dict(random=deepcopy(random.bit_generator.state)))
        with patch.object(sensing, 'hall_available_time', return_value=Fixed.airborne_hover_duration):
            events = list(airborne_diagnostics.recorded_events(simulation, state))
        self.assertEqual([row[1] for row in events], starts)
        self.assertEqual([row[-1].adc_codes.item() for row in events], [0, 1, 1])

    def test_pending_checkpoint_step_does_not_invent_an_unrecorded_current_sample(self):
        random = np.random.default_rng(Fixed.hall_virtual_seed)
        period = 1 / Fixed.current_regulator_rate / Fixed.qualification_integration_substeps
        simulation = SimpleNamespace(period=period, regulator_tick=Fixed.qualification_integration_substeps,
            identities=(0,), scan_delay=Fixed.qualification_sensor_latency,
            sensors=SimpleNamespace(random=random))
        count = simulation.regulator_tick
        state = dict(step=count, field_times=(np.arange(count) * period).tolist(), field_values=[[0.]] * count,
            readouts=[dict(sample_times_s=[0.], adc_codes=[[1]], available_time_s=0.)],
            sensors=dict(random=deepcopy(random.bit_generator.state)))
        events = list(airborne_diagnostics.recorded_events(simulation, state))
        self.assertEqual(len(events), count)
        self.assertEqual(events[-1][1], state['field_times'][-1])
        state['field_times'].append(count * period)
        state['field_values'].append([0.])
        state['readouts'].append(dict(sample_times_s=[count * period], adc_codes=[[1]], available_time_s=count * period))
        self.assertEqual(len(list(airborne_diagnostics.recorded_events(simulation, state))), count + 1)


class AirborneCheckpointTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.construction = magnetics.ConstructionModel()

    def substitute_magnetics(self, simulation):
        simulation.plant.magnetic_matrix = Mock(return_value=np.zeros((6, len(simulation.identities))))
        simulation.controller.nominal_matrix = np.zeros((6, len(simulation.identities)))
        simulation.controller.command = Mock(return_value=dict(accepted=True, required_current_A=0.,
                                                               currents=np.full(len(simulation.identities), Fixed.max_current_command_error)))

    def test_coupled_restart_exactly_replays_pending_scan_and_electrical_state(self):
        original = airborne.AirborneSimulation(self.construction, Fixed.airborne_check_duration, disturbed=True)
        self.substitute_magnetics(original)
        delivery_step = int(np.ceil(original.scan_delay / original.period))
        for _ in range(delivery_step):
            original.advance()
        self.assertEqual(original.delivered_scans, 0)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'checkpoint.json'
            original.save_checkpoint(path)
            resumed = airborne.AirborneSimulation(self.construction, Fixed.airborne_check_duration, disturbed=True, initialize=False)
            self.substitute_magnetics(resumed)
            resumed.load_checkpoint(path)
            self.assertEqual(original.checkpoint_state(), resumed.checkpoint_state())
            for _ in range(Fixed.qualification_integration_substeps):
                original.advance()
                resumed.advance()
            self.assertEqual(original.delivered_scans, 1)
            self.assertEqual(original.checkpoint_state(), resumed.checkpoint_state())
            state = json.loads(path.read_text())
            state['source_sha256']['model.py'] = 'modified'
            path.write_text(json.dumps(state))
            with self.assertRaises(ValueError):
                resumed.load_checkpoint(path)

    def test_allocation_failure_is_retained_without_applying_a_fallback(self):
        simulation = airborne.AirborneSimulation(self.construction, Fixed.airborne_check_duration)
        simulation.controller.command = Mock(return_value=dict(accepted=False, required_current_A=2 * Fixed.driver_channel_current, currents=None))
        simulation.plant.advance_wrench = Mock(side_effect=AssertionError('Failed control must not advance the plant'))
        simulation.advance()
        self.assertTrue(simulation.finished)
        self.assertFalse(simulation.report()['passed'])
        self.assertEqual(simulation.stop_reason, 'current_allocation_exceeds_reserved_budget')
        self.assertEqual(len(simulation.trace), 1)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'checkpoint.json'
            simulation.save_checkpoint(path)
            resumed = airborne.AirborneSimulation(self.construction, Fixed.airborne_check_duration, initialize=False)
            resumed.load_checkpoint(path)
            self.assertEqual(simulation.checkpoint_state(), resumed.checkpoint_state())
            with self.assertRaises(ValueError):
                resumed.advance()

    def test_refinement_keeps_hardware_clocks_and_nominal_controller_unchanged(self):
        baseline = airborne.AirborneSimulation(self.construction, Fixed.airborne_check_duration)
        refined = airborne.AirborneSimulation(self.construction, Fixed.airborne_check_duration, initialize=False,
            time_refinement=Fixed.qualification_refinement_factor, magnetic_refinement=max(Fixed.current_demand_mesh_refinements))
        self.assertEqual(refined.period * Fixed.qualification_refinement_factor, baseline.period)
        for tick in ('command_tick', 'regulator_tick', 'pose_tick'):
            self.assertEqual(getattr(refined, tick) * refined.period, getattr(baseline, tick) * baseline.period)
        self.assertGreater(len(refined.plant.array.conductor_vertices), len(baseline.plant.array.conductor_vertices))
        self.assertEqual(refined.plant.array.force_order,
            baseline.plant.array.force_order * Fixed.qualification_refinement_factor)
        self.assertEqual(refined.controller.array.force_order, baseline.controller.array.force_order)
        self.assertEqual(refined.report()['force_quadrature_orders']['physical'], refined.plant.array.force_order)
        np.testing.assert_array_equal(refined.controller.array.conductor_vertices, baseline.controller.array.conductor_vertices)
        np.testing.assert_array_equal(refined.tracker.estimator.field_model.coil_field,
                                      baseline.tracker.estimator.field_model.coil_field)
        self.assertFalse(np.array_equal(refined.fields.coil_field, baseline.fields.coil_field))
        np.testing.assert_array_equal(refined.sensors.gain, baseline.sensors.gain)
        np.testing.assert_array_equal(refined.sensors.offset, baseline.sensors.offset)
        np.testing.assert_allclose(refined.sensors.actual_coil_field * baseline.fields.coil_field,
                                   baseline.sensors.actual_coil_field * refined.fields.coil_field,
                                   atol=0, rtol=Fixed.matrix_rank_tolerance)
        np.testing.assert_array_equal(refined.current_sensors.offset, baseline.current_sensors.offset)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'checkpoint.json'
            baseline.save_checkpoint(path)
            with self.assertRaises(ValueError):
                refined.load_checkpoint(path)


class AirborneVerificationTests(unittest.TestCase):
    def report(self):
        return dict(passed=True, finished=True, stop_reason=None, duration_completed_s=1, elapsed_s=1,
                    hardware_rates_Hz=dict(command=4000, regulator=20000, feedback=1000),
                    peaks=dict(current_A=1, required_current_A=1, voltage_V=1, copper_loss_W=1),
                    trace=[dict(time_s=t, pose=[0] * 6, estimated_pose=[0] * 6, currents_A=[1, -1]) for t in (0, 1)])

    def test_comparison_checks_rotations_currents_and_observer_not_just_position(self):
        baseline = self.report()
        self.assertTrue(airborne_verification.compare_trials(baseline, baseline)['converged'])
        partial = deepcopy(baseline)
        partial['passed'] = False
        partial['peaks']['current_A'] = 0
        self.assertTrue(airborne_verification.compare_trials(baseline, partial, trace_only=True)['converged'])
        self.assertFalse(airborne_verification.compare_trials(baseline, partial)['converged'])
        for field, index, difference in [('pose', 3, Fixed.qualification_angle_error),
                                         ('estimated_pose', 0, Fixed.qualification_position_error),
                                         ('currents_A', 0, Fixed.driver_channel_current)]:
            refined = deepcopy(baseline)
            refined['trace'][-1][field][index] += difference
            self.assertFalse(airborne_verification.compare_trials(baseline, refined)['converged'])
        refined = deepcopy(baseline)
        refined['hardware_rates_Hz']['feedback'] *= 2
        with self.assertRaises(ValueError):
            airborne_verification.compare_trials(baseline, refined)

    def test_deadline_saves_unfinished_state_without_claiming_success(self):
        simulation = Mock(finished=False)
        simulation.report.return_value = dict(finished=False, passed=False)
        with tempfile.TemporaryDirectory() as folder:
            output, checkpoint = Path(folder) / 'report.json', Path(folder) / 'checkpoint.json'
            result = airborne.run_simulation(simulation, output, checkpoint, deadline=0)
            simulation.advance.assert_not_called()
            simulation.save_checkpoint.assert_called_once_with(checkpoint)
            self.assertFalse(result['passed'])
            self.assertEqual(json.loads(output.read_text()), result)

    def test_batch_gates_long_hover_and_renders_without_manual_followup(self):
        for converged, extended_hover, screen_only, expected_count, expected_status in (
                (True, True, False, 9, 'completed'), (False, True, False, 2, 'numerical_comparison_failed'),
                (True, False, False, 8, 'completed'), (False, False, False, 2, 'numerical_comparison_failed'),
                (True, True, True, 4, 'completed'), (False, True, True, 2, 'numerical_comparison_failed')):
            with self.subTest(converged=converged), tempfile.TemporaryDirectory() as folder:
                simulation = SimpleNamespace(source={})
                with patch.object(airborne_verification.magnetics, 'ConstructionModel'), \
                     patch.object(airborne, 'AirborneSimulation', return_value=simulation) as constructor, \
                     patch.object(airborne, 'run_simulation', side_effect=lambda *a, **kw: self.report()) as run, \
                     patch.object(airborne_verification, 'compare_trials', side_effect=lambda *a, **kw: dict(converged=converged, checks={})), \
                     patch.object(airborne_verification, 'render_report', return_value={}) as render, \
                     patch('builtins.print'):
                    result = airborne_verification.run_batch(Path(folder), datetime.max.replace(tzinfo=timezone.utc),
                        None if screen_only else Path(folder) / 'video.mp4', extended_hover=extended_hover, screen_only=screen_only)
                self.assertEqual(constructor.call_count, expected_count)
                self.assertEqual(result['status'], expected_status)
                checkpoints = [call.args[2] for call in run.call_args_list]
                self.assertEqual(len(set(checkpoints)), expected_count)
                self.assertTrue(all(path.parent == Path(folder) for path in checkpoints))
                if screen_only:
                    self.assertTrue(all(call.args[1] == Fixed.airborne_refinement_screen_duration for call in constructor.call_args_list))
                    render.assert_not_called()
                else:
                    render.assert_called_once()

    def test_screen_stops_on_trace_failure_before_the_longer_batch(self):
        calls = []

        def run(simulation, output, checkpoint, deadline, progress):
            calls.append(output.stem)
            report = self.report()
            if output.stem == 'screen_magnetic_refined':
                report.update(finished=False, passed=False, duration_completed_s=0.)
                report['trace'] = report['trace'][:1]
                report['trace'][0]['currents_A'][0] += Fixed.driver_channel_current
                progress(report)
                self.assertTrue(simulation.finished)
                self.assertEqual(simulation.stop_reason, 'numerical_comparison_failed')
                report.update(finished=simulation.finished, stop_reason=simulation.stop_reason)
            return report

        with tempfile.TemporaryDirectory() as folder, \
             patch.object(airborne_verification.magnetics, 'ConstructionModel'), \
             patch.object(airborne, 'AirborneSimulation', side_effect=lambda *a, **kw: SimpleNamespace(source={})), \
             patch.object(airborne, 'run_simulation', side_effect=run), \
             patch.object(airborne_verification, 'render_report', return_value={}), patch('builtins.print'):
            result = airborne_verification.run_batch(Path(folder), datetime.max.replace(tzinfo=timezone.utc), Path(folder) / 'video.mp4', magnetic_first=True)
        self.assertEqual(calls, ['screen_baseline', 'screen_repeat', 'screen_magnetic_refined'])
        self.assertEqual(result['status'], 'numerical_comparison_failed')
        self.assertFalse(result['comparisons']['screen_magnetic_refined']['checks']['current_trace_A'])

    def test_guard_stop_is_saved_before_run_returns(self):
        simulation = SimpleNamespace(finished=False, stop_reason=None, step=0, checkpoint_tick=1, checkpoint_due=lambda: True,
            plant=SimpleNamespace(data=SimpleNamespace(time=0.)), peaks={'current_A': 0.})
        simulation.advance = lambda: setattr(simulation, 'step', simulation.step + 1)
        simulation.report = lambda: dict(finished=simulation.finished, stop_reason=simulation.stop_reason, passed=False)
        simulation.save_checkpoint = lambda path: path.write_text(json.dumps(simulation.report()))

        def stop(report):
            simulation.finished = True
            simulation.stop_reason = 'numerical_comparison_failed'

        with tempfile.TemporaryDirectory() as folder, patch('builtins.print'):
            output, checkpoint = Path(folder) / 'report.json', Path(folder) / 'checkpoint.json'
            report = airborne.run_simulation(simulation, output, checkpoint, progress=stop)
            self.assertEqual(report, json.loads(output.read_text()))
            self.assertEqual(report, json.loads(checkpoint.read_text()))
            self.assertEqual(report['stop_reason'], 'numerical_comparison_failed')

    def test_repeat_mismatch_stops_before_refinement(self):
        baseline = self.report()
        changed = deepcopy(baseline)
        changed['trace'][-1]['pose'][0] = np.finfo(float).eps
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(airborne_verification.magnetics, 'ConstructionModel'), \
             patch.object(airborne, 'AirborneSimulation', return_value=SimpleNamespace(source={})), \
             patch.object(airborne, 'run_simulation', side_effect=[baseline, changed]) as run, \
             patch.object(airborne_verification, 'render_report', return_value={}), patch('builtins.print'):
            result = airborne_verification.run_batch(Path(folder), datetime.max.replace(tzinfo=timezone.utc), Path(folder) / 'video.mp4')
        self.assertEqual(run.call_count, 2)
        self.assertEqual(result['status'], 'repeatability_failed')

    def test_remaining_case_failure_cancels_peer_and_preserves_both_reports(self):
        baseline = self.report()
        baseline.update(source_sha256={}, mechanical_properties={}, intended_duration_s=Fixed.airborne_refinement_duration,
                        magnetic_refinement=min(Fixed.current_demand_mesh_refinements), disturbed=True)
        cancelled = Event()

        def construct(*args, **kwargs):
            simulation = SimpleNamespace(source={}, finished=False, stop_reason=None, step=0, checkpoint_tick=1, checkpoint_due=lambda: True,
                plant=SimpleNamespace(data=SimpleNamespace(time=0.)), peaks={'current_A': 0.})
            simulation.advance = lambda: setattr(simulation, 'step', simulation.step + 1)

            def report():
                result = deepcopy(baseline)
                result.update(finished=simulation.finished, passed=False, stop_reason=simulation.stop_reason)
                result['trace'] = result['trace'][:1]
                if kwargs['magnetic_refinement'] != baseline['magnetic_refinement']:
                    result['trace'][0]['currents_A'][0] += Fixed.driver_channel_current
                return result

            simulation.report = report
            simulation.save_checkpoint = lambda path: path.write_text(json.dumps(report()))
            return simulation

        with tempfile.TemporaryDirectory() as folder, \
             patch.object(airborne_verification, 'recorded_construction'), \
             patch.object(airborne, 'AirborneSimulation', side_effect=construct), patch('builtins.print'):
            output = Path(folder)
            failed = airborne_verification.remaining_case(baseline, output, 'magnetic_refined', float('inf'), cancelled)
            self.assertTrue(cancelled.is_set())
            self.assertEqual(failed['report']['stop_reason'], 'numerical_comparison_failed')
            peer = airborne_verification.remaining_case(baseline, output, 'time_refined', float('inf'), cancelled)
            self.assertEqual(peer['report']['stop_reason'], 'peer_trial_failed')
            for role, result in (('magnetic_refined', failed), ('time_refined', peer)):
                self.assertEqual(json.loads((output / (role + '.json')).read_text()), result['report'])
                self.assertEqual(json.loads((output / (role + '_checkpoint.json')).read_text()), result['report'])
                self.assertFalse(result['report']['passed'])

    def test_saved_trial_requires_matching_passed_checkpoint(self):
        report = self.report()
        report['source_sha256'] = {}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'baseline.json'
            checkpoint = Path(folder) / 'baseline_checkpoint.json'
            path.write_text(json.dumps(report))
            checkpoint.write_text(json.dumps(report))
            self.assertEqual(airborne_verification.verified_saved_trial(path), report)
            changed = deepcopy(report)
            changed['trace'][0]['currents_A'][0] += Fixed.driver_channel_current
            checkpoint.write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                airborne_verification.verified_saved_trial(path)
            report['finished'] = False
            path.write_text(json.dumps(report))
            checkpoint.write_text(json.dumps(report))
            with self.assertRaises(ValueError):
                airborne_verification.verified_saved_trial(path)

    def test_reused_baseline_repeat_failure_prevents_parallel_launch(self):
        report = self.report()
        report.update(source_sha256={}, intended_duration_s=Fixed.airborne_refinement_duration,
                      time_refinement=1, magnetic_refinement=min(Fixed.current_demand_mesh_refinements), disturbed=True)
        screen = dict(status='completed', cases={name: {} for name in
            ('screen_baseline', 'screen_repeat', 'screen_time_refined', 'screen_magnetic_refined')},
            comparisons={name: dict(converged=True) for name in ('screen_repeat', 'screen_time_refined', 'screen_magnetic_refined')})
        short = dict(report, intended_duration_s=Fixed.airborne_refinement_screen_duration)

        def fail_repeat(baseline, output, role, deadline, cancelled):
            self.assertEqual(role, 'repeat')
            cancelled.set()
            return dict(report=dict(report, passed=False, stop_reason='repeatability_failed'),
                        comparison=dict(converged=False))

        context = SimpleNamespace(Manager=lambda: nullcontext(SimpleNamespace(Event=Event)))
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(airborne_verification, 'verified_saved_trial', side_effect=[report, short, short, short, short]), \
             patch.object(airborne_verification, 'source_fingerprints', return_value={}), \
             patch.object(airborne_verification.multiprocessing, 'get_context', return_value=context), \
             patch.object(airborne_verification, 'remaining_case', side_effect=fail_repeat), \
             patch.object(airborne_verification, 'ProcessPoolExecutor') as executor, patch('builtins.print'):
            screen_path = Path(folder) / 'screen.json'
            screen_path.write_text(json.dumps(screen))
            result = airborne_verification.run_remaining(Path(folder) / 'baseline.json', screen_path,
                                                        datetime.max.replace(tzinfo=timezone.utc))
            executor.assert_not_called()
            self.assertEqual(result['status'], 'verification_failed')
            self.assertEqual(result['active_cases'], [])
            self.assertFalse(result['comparisons']['repeat']['converged'])


class AirbornePredictionTests(unittest.TestCase):
    def test_acceleration_history_clips_at_measurement_time_and_ignores_future_inputs(self):
        tracker = sensing.HallTracker(None)
        tracker.record_acceleration(0, np.ones(6))
        tracker.record_acceleration(0.0007, -np.ones(6))
        tracker.record_acceleration(0.002, np.full(6, 100))
        values, periods = tracker.acceleration_intervals(0.0002, 0.0011)
        np.testing.assert_allclose(periods, [0.0005, 0.0004])
        position, velocity = qualification.propagate_estimate(np.zeros(6), np.zeros(6), values, periods)
        np.testing.assert_allclose(velocity, 0.0001)
        np.testing.assert_allclose(position, 0.0005 ** 2 / 2 + 0.0005 * 0.0004 - 0.0004 ** 2 / 2)
        observed, rate, disturbance = qualification.observer_update(
            np.zeros(6), np.zeros(6), np.zeros(6), position, periods.sum(), values, periods)
        np.testing.assert_allclose(observed, position)
        np.testing.assert_allclose(rate, velocity)
        np.testing.assert_allclose(disturbance, 0)
        with self.assertRaises(ValueError):
            tracker.record_acceleration(0.002, np.zeros(6))
        with self.assertRaises(ValueError):
            tracker.acceleration_intervals(1, 0)

    def test_measured_input_uses_rigid_body_and_euler_acceleration_consistently(self):
        controller = airborne.AirborneController.__new__(airborne.AirborneController)
        controller.identities = tuple(range(6))
        controller.geometry = SimpleNamespace(weight=2, gravity=10, inertias=np.array([0.01, 0.02, 0.03]))
        controller.nominal_matrix = np.eye(6)
        pose = np.array([0, 0, 0, 0.1, -0.2, 0.3])
        velocity = np.array([0, 0, 0, 0.4, -0.3, 0.2])
        acceleration = np.array([1, -2, 3, 0.3, -0.2, 0.1])
        step = Fixed.angle_derivative_step
        plus = airborne.euler_rates_to_world(pose + velocity * step, velocity[3:] + acceleration[3:] * step)
        minus = airborne.euler_rates_to_world(pose - velocity * step, velocity[3:] - acceleration[3:] * step)
        world_acceleration = (plus - minus) / (2 * step)
        omega = airborne.euler_rates_to_world(pose, velocity[3:])
        rotation = Rotation.from_euler('xyz', pose[3:]).as_matrix()
        inertia = rotation @ np.diag(controller.geometry.inertias) @ rotation.T
        wrench = np.r_[(acceleration[:3] + [0, 0, 10]) * 0.2,
                       inertia @ world_acceleration + np.cross(omega, inertia @ omega)]
        readings = SimpleNamespace(identities=controller.identities, sample_times_s=np.array([0]),
                                   decode=Mock(return_value=wrench[None, :]))
        estimate = dict(valid=True, pose=pose, velocity=velocity)
        actual = controller.measured_acceleration(estimate, readings, 0)
        np.testing.assert_allclose(actual, acceleration, atol=Fixed.motion_verification_tolerance, rtol=0)
        readings.decode.assert_called_once_with(0)
        with self.assertRaises(ValueError):
            controller.measured_acceleration(estimate, readings, 1)


if __name__ == '__main__':
    unittest.main()

from copy import deepcopy
from types import SimpleNamespace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from model import Fixed
from methodology.adaptive_airborne import AdaptiveAirborneSimulation
from methodology.cad import magnetics
from methodology.magnetic_atlas import AtlasAirborneSimulation


class AdaptiveAirborneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.simulation = AdaptiveAirborneSimulation(magnetics.ConstructionModel(), Fixed.airborne_check_duration)
        cls.folder = tempfile.TemporaryDirectory()
        cls.initial = Path(cls.folder.name) / 'initial.json'
        cls.simulation.save_checkpoint(cls.initial)

    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()

    def setUp(self):
        self.sim = self.simulation
        self.sim.load_checkpoint(self.initial)
        self.patches = [patch.object(self.sim.plant, 'magnetic_matrix', return_value=np.zeros((6, len(self.sim.identities)))),
            patch.object(self.sim.controller, 'command', return_value=dict(accepted=True, required_current_A=0.,
                currents=np.zeros(len(self.sim.identities)))),
            patch.object(self.sim.controller, 'measured_acceleration', return_value=np.zeros(6))]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)

    def test_rejection_restores_physics_without_resampling_or_retaining_trial_history(self):
        def injected_error(*args):
            return {'position': 2. if self.sim.adaptation['rejected'] == 0 else 0.5}
        with patch.object(self.sim, 'step_error', side_effect=injected_error):
            self.sim.advance()
        rejected = self.sim.checkpoint_state()
        self.assertGreaterEqual(self.sim.adaptation['rejected'], 1)
        accepted_period = self.sim.clock
        self.assertLess(accepted_period, Fixed.adaptive_max_step)
        self.assertEqual(len(self.sim.readouts), 1)
        self.assertEqual(len(self.sim.field_times), 3)
        self.sim.load_checkpoint(self.initial)
        self.sim.proposed_step = accepted_period
        with patch.object(self.sim, 'step_error', return_value={'position': 0.5}):
            self.sim.advance()
        direct = self.sim.checkpoint_state()
        for key in ('plant', 'drivers', 'tracker', 'sensors', 'current_sensors', 'trace', 'field_times', 'field_values', 'coil_field_values', 'peaks'):
            self.assertEqual(rejected[key], direct[key], key)

    def test_exact_events_and_restart_across_pending_hall_delivery(self):
        while self.sim.clock < self.sim.scan_delay - 1 / Fixed.current_regulator_rate:
            self.sim.advance()
        path = Path(self.folder.name) / 'pending.json'
        self.sim.save_checkpoint(path)
        before = deepcopy(self.sim.checkpoint_state())
        while self.sim.delivered_scans == 0 and not self.sim.finished:
            self.sim.advance()
        expected = deepcopy(self.sim.checkpoint_state())
        self.assertEqual(self.sim.tracker.last_delivery, self.sim.scan_delay)
        self.sim.load_checkpoint(path)
        self.assertEqual(self.sim.checkpoint_state(), before)
        while self.sim.delivered_scans == 0 and not self.sim.finished:
            self.sim.advance()
        self.assertEqual(self.sim.checkpoint_state(), expected)
        times = [row.sample_times_s[0] for row in self.sim.readouts]
        self.assertEqual(times, [index / Fixed.current_regulator_rate for index in range(len(times))])
        self.assertTrue(np.all(np.diff(self.sim.field_times) > 0))

    def test_minimum_step_failure_saves_unadvanced_physics(self):
        initial = self.sim.plant.integration_state().copy()
        with patch.object(self.sim, 'step_error', return_value={'field': 2.}):
            self.sim.advance()
        self.assertEqual(self.sim.stop_reason, 'adaptive_accuracy_unresolved')
        self.assertFalse(self.sim.report()['passed'])
        self.assertEqual(self.sim.clock, 0.)
        np.testing.assert_array_equal(self.sim.plant.integration_state(), initial)
        self.assertEqual(len(self.sim.field_times), 1)
        path = Path(self.folder.name) / 'failed.json'
        self.sim.save_checkpoint(path)
        expected = deepcopy(self.sim.checkpoint_state())
        self.sim.load_checkpoint(path)
        self.assertEqual(self.sim.checkpoint_state(), expected)

    def test_atlas_backend_resume_identity_and_instance_isolation(self):
        atlas = SimpleNamespace(identity='test-atlas', field=lambda source, points: np.zeros_like(points),
            evaluator_identity=dict(method='scipy'), evaluator_sources={})
        construction = magnetics.ConstructionModel()
        simulation = AtlasAirborneSimulation(construction, atlas, 'atlas_forces', Fixed.airborne_check_duration)
        self.assertIs(simulation.controller.field_evaluator, simulation.plant.field_evaluator)
        np.testing.assert_array_equal(simulation.plant.field_evaluator(None, np.ones((1, 3))), np.zeros((1, 3)))
        self.assertIsNone(self.sim.controller.field_evaluator)
        self.assertIsNone(self.sim.plant.field_evaluator)
        path = Path(self.folder.name) / 'atlas.json'
        simulation.save_checkpoint(path)
        state = deepcopy(simulation.checkpoint_state())
        with patch.object(simulation.controller, 'command', return_value=dict(accepted=True, required_current_A=0.,
                currents=np.zeros(len(simulation.identities)))), patch.object(simulation.controller, 'measured_acceleration', return_value=np.zeros(6)):
            simulation.advance()
            expected = deepcopy(simulation.checkpoint_state())
            simulation.load_checkpoint(path)
            self.assertEqual(simulation.checkpoint_state(), state)
            simulation.advance()
            self.assertEqual(simulation.checkpoint_state(), expected)
        for changed in (dict(mode='direct', identity=None), dict(mode='atlas_forces', identity='different-atlas')):
            simulation.atlas_configuration = changed
            with self.assertRaisesRegex(ValueError, 'backend or atlas'):
                simulation.load_checkpoint(path)
        with self.assertRaisesRegex(ValueError, 'schema'):
            self.sim.load_checkpoint(path)

    def test_error_detects_velocity_and_analogue_path_even_when_end_positions_match(self):
        initial = self.sim.endpoint()
        midpoint, coarse, fine = (deepcopy(initial) for _ in range(3))
        fine['truth']['linear_velocity_m_per_s'][0] += 2 * Fixed.adaptive_velocity_tolerance
        errors = self.sim.step_error(initial, coarse, midpoint, fine, Fixed.adaptive_max_step)
        self.assertEqual(errors['position'], 0.)
        self.assertGreater(errors['velocity'], 1)
        fine = deepcopy(initial)
        midpoint['total'][0] += 2 * Fixed.adaptive_field_tolerance / self.sim.sensors.gain[0]
        errors = self.sim.step_error(initial, coarse, midpoint, fine, Fixed.adaptive_max_step)
        self.assertGreater(errors['field'], 1)

    def test_free_fall_step_doubling_error_matches_analytic_difference(self):
        self.sim.drivers.currents[:] = 0
        self.sim.drivers.voltage[:] = 0
        self.sim.drivers.filtered_command[:] = 0
        self.sim.drivers.command[:] = 0
        start = self.sim.physical_snapshot()
        period = Fixed.adaptive_max_step
        matrix = np.zeros((6, len(self.sim.identities)))
        initial = self.sim.endpoint()
        self.sim.advance_physics(period, matrix)
        coarse = self.sim.endpoint()
        self.sim.restore_physics(start)
        self.sim.advance_physics(period / 2, matrix)
        midpoint = self.sim.endpoint()
        self.sim.advance_physics(period / 2, matrix)
        fine = self.sim.endpoint()
        expected = self.sim.plant.geometry.gravity * period ** 2 / 4
        self.assertAlmostEqual(abs(fine['truth']['com_position_m'][2] - coarse['truth']['com_position_m'][2]), expected, places=14)
        errors = self.sim.step_error(initial, coarse, midpoint, fine, period)
        self.assertAlmostEqual(errors['position'], expected / Fixed.adaptive_position_tolerance, places=6)

    def test_event_roundoff_never_creates_subminimum_steps_or_changes_noise_draw_count(self):
        states = []
        for mode in ('adaptive', 'reference', 'tighter'):
            self.sim.fixed_step = None
            self.sim.tolerance_scale = 1
            self.sim.load_checkpoint(self.initial)
            if mode == 'reference':
                self.sim.fixed_step = Fixed.adaptive_reference_step
            if mode == 'tighter':
                self.sim.tolerance_scale = 1 / Fixed.qualification_refinement_factor
            while not self.sim.finished:
                self.sim.advance()
            self.assertTrue(self.sim.report()['passed'])
            self.assertTrue(all(row['interval_s'] >= Fixed.adaptive_min_step for row in self.sim.adaptation['intervals']),
                [(mode, row) for row in self.sim.adaptation['intervals'] if row['interval_s'] < Fixed.adaptive_min_step])
            times = [row.sample_times_s[0] for row in self.sim.readouts]
            self.assertEqual(times, [index / Fixed.current_regulator_rate for index in range(len(times))])
            self.assertTrue(np.all(np.diff(self.sim.field_times) > 0))
            states.append(deepcopy((self.sim.sensors.random.bit_generator.state, self.sim.current_sensors.random.bit_generator.state)))
        self.assertEqual(states[0], states[1])
        self.assertEqual(states[0], states[2])
        self.sim.fixed_step = None
        self.sim.tolerance_scale = 1

    def test_coincident_checkpoint_and_hardware_clocks_do_not_create_duplicate_endpoints(self):
        simulation = self.sim
        self.addCleanup(setattr, simulation, 'duration', Fixed.airborne_check_duration)
        self.addCleanup(setattr, simulation, 'fixed_step', None)
        for fixed_step in (None, Fixed.adaptive_reference_step):
            simulation.fixed_step = None
            simulation.duration = Fixed.airborne_check_duration
            simulation.load_checkpoint(self.initial)
            simulation.fixed_step = fixed_step
            simulation.duration = Fixed.adaptive_sensitivity_duration
            simulation.next_checkpoint = 9
            boundary = simulation.next_checkpoint * Fixed.airborne_checkpoint_interval
            regulator_index = round(boundary * Fixed.current_regulator_rate)
            simulation.next_regulator = regulator_index - 1
            simulation.next_command = round(boundary * Fixed.current_command_rate)
            simulation.clock = simulation.next_regulator / Fixed.current_regulator_rate
            simulation.next_scan = int(simulation.clock * Fixed.pose_feedback_rate)
            simulation.plant.data.time = simulation.clock
            endpoint = simulation.endpoint()
            simulation.field_times = [simulation.clock]
            simulation.field_values = [endpoint['total']]
            simulation.coil_field_values = [endpoint['coil']]
            with patch.object(simulation, 'process_events') as events:
                while simulation.clock < (regulator_index + 1) / Fixed.current_regulator_rate - Fixed.adaptive_clock_tolerance:
                    simulation.advance()
            self.assertTrue(np.all(np.diff(simulation.field_times) > 0))
            self.assertTrue(all(row['interval_s'] >= Fixed.adaptive_min_step for row in simulation.adaptation['intervals']))
            self.assertEqual(sum(call.args[2] for call in events.call_args_list), 1)
            self.assertEqual(simulation.next_checkpoint, 10)
        simulation.fixed_step = None
        simulation.duration = Fixed.airborne_check_duration

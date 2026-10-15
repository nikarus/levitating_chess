import unittest
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock, patch

import levitation_sim
import numpy as np
from scipy.spatial.transform import Rotation

from model import Inputs, Fixed
from methodology import coil_layouts, prototype_coils
from methodology.cad import magnetics
from methodology.airborne import AirborneSimulation
from methodology.physics_verification import AdaptiveCoilHandoffTrial, CoilHandoffTrial, moving_handoff_scenario, gap_handoff_scenarios, tilted_handoff_scenarios, AdaptivePrescribedTrial, PrescribedPhysicalTrial, handoff_poses, matrix_refinement_metrics, moving_comparison, run_prescribed_trial


class PhysicalMatrixTests(unittest.TestCase):
    def test_gap_bounds_accept_exact_endpoints_and_reject_adjacent_outside_floats(self):
        trial = PrescribedPhysicalTrial.__new__(PrescribedPhysicalTrial)
        geometry = SimpleNamespace(center_of_mass_height=0.014742337541608922,
            gap=Inputs.magnet_to_coil_distance / 1000, max_flight_gap=Inputs.max_flight_gap / 1000)
        truth = dict(com_position_m=np.zeros(3), rotation=Rotation.identity())
        trial.plant = SimpleNamespace(geometry=geometry, data=SimpleNamespace(ncon=0), truth=lambda: truth,
            metadata=dict(board_top_m=0.), integration_state=lambda: np.zeros(1))
        trial.fields = SimpleNamespace(base_radius=0.)
        trial.drivers = SimpleNamespace(currents=np.zeros(1), current_limit=Fixed.driver_channel_current)
        trial.drivers.voltage = np.zeros(1)
        trial.drivers.copper_resistance = 0.
        trial.peaks = dict(position_error_m=0., angle_error_rad=0., current_A=0., voltage_V=0., copper_loss_W=0.)
        trial.reference = np.zeros(6)
        trial.last_magnet = trial.last_coil = np.zeros(1)
        minimum = geometry.center_of_mass_height
        maximum = minimum + (geometry.max_flight_gap - geometry.gap)
        with patch('methodology.physics_verification.motion.bottom_film_clearance', return_value=geometry.gap):
            for height, failure in ((minimum, None), (maximum, None),
                    (np.nextafter(minimum, -np.inf), 'outside_sampled_gap_range'),
                    (np.nextafter(maximum, np.inf), 'outside_sampled_gap_range')):
                with self.subTest(height=height):
                    truth['com_position_m'][2] = height
                    trial.reference[2] = height - minimum
                    for observer in (trial.observe, lambda: AirborneSimulation.observe_physics(trial, truth)):
                        trial.stop_reason = None
                        observer()
                        self.assertEqual(trial.stop_reason, failure)
            truth['com_position_m'][2] = minimum
            trial.reference[:] = 0
            for current, failure in ((Fixed.driver_channel_current, None),
                    (np.nextafter(Fixed.driver_channel_current, np.inf), 'current_limit_exceeded'),
                    (-np.nextafter(Fixed.driver_channel_current, np.inf), 'current_limit_exceeded'),
                    (np.nan, 'nonfinite_physical_state')):
                with self.subTest(current=current):
                    trial.drivers.currents[:] = current
                    for observer in (trial.observe, lambda: AirborneSimulation.observe_physics(trial, truth)):
                        trial.stop_reason = None
                        observer()
                        self.assertEqual(trial.stop_reason, failure)

    def test_contact_and_nonfinite_state_prevent_airborne_acceptance(self):
        from methodology.motion import airborne_observation
        geometry = SimpleNamespace(center_of_mass_height=Inputs.magnet_to_coil_distance / 1000,
            gap=Inputs.magnet_to_coil_distance / 1000, max_flight_gap=Inputs.max_flight_gap / 1000)
        truth = dict(com_position_m=np.array([0., 0., geometry.center_of_mass_height]), rotation=Rotation.identity())
        plant = SimpleNamespace(geometry=geometry, data=SimpleNamespace(ncon=0),
            metadata=dict(board_top_m=0.), integration_state=lambda: np.zeros(1))
        drivers = SimpleNamespace(currents=np.zeros(1), current_limit=Fixed.driver_channel_current)
        with patch('methodology.motion.bottom_film_clearance', return_value=geometry.gap):
            plant.data.ncon = 1
            self.assertEqual(airborne_observation(plant, truth, drivers, 0.)[0], 'contact')
            plant.data.ncon = 0
            plant.integration_state = lambda: np.array([np.nan])
            self.assertEqual(airborne_observation(plant, truth, drivers, 0.)[0], 'nonfinite_physical_state')
        plant.integration_state = lambda: np.zeros(1)
        with patch('methodology.motion.bottom_film_clearance', return_value=0.):
            self.assertEqual(airborne_observation(plant, truth, drivers, 0.)[0], 'contact')

    def test_prescribed_runner_stops_at_first_failed_observation(self):
        initial = dict(time_s=0., pose=[0.] * 6, currents_A=[0.], world_velocity=[0.] * 6,
            wrench=[0.] * 6, analogue_field_T=[0.])
        reference = dict(input_identity='same', hardware_rates_Hz={}, trace=[initial,
            dict(initial,time_s=1 / Fixed.current_regulator_rate)])
        trial = SimpleNamespace(scenario=dict(duration_s=Fixed.airborne_check_duration), input_identity='same',
            stop_reason=None,trace=[deepcopy(initial)],save=Mock())
        def advance(index):
            trial.trace.append(dict(initial,time_s=(index+1)/Fixed.current_regulator_rate,
                currents_A=[Fixed.driver_channel_current]))
        trial.advance_interval = Mock(side_effect=advance)
        trial.report = lambda: dict(hardware_rates_Hz={},trace=trial.trace)
        comparison = run_prescribed_trial(trial,reference,None,'candidate')
        self.assertFalse(comparison['converged'])
        self.assertEqual(trial.stop_reason,'physical_comparison_failed')
        trial.advance_interval.assert_called_once_with(0)

    def test_adaptive_accepted_fields_preserve_full_filter_memory(self):
        trial = AdaptivePrescribedTrial.__new__(AdaptivePrescribedTrial)
        trial.drivers = SimpleNamespace(inductance=1., resistance=1.)
        trial.last_magnet, trial.last_coil = np.array([0.01]), np.array([-0.02])
        trial.filtered_magnet, trial.filtered_coil = trial.last_magnet.copy(), trial.last_coil.copy()
        trial.raw_history = [dict(time_s=0.)]
        period = Fixed.adaptive_max_step
        midpoint = dict(total=np.array([0.04]), coil=np.array([0.01]))
        endpoint = dict(total=np.array([0.06]), coil=np.array([0.02]))
        expected = deepcopy(trial)
        expected.accept_fields(period / 2, np.array([0.03]), np.array([0.01]), period / 2)
        expected.accept_fields(period, np.array([0.04]), np.array([0.02]), period / 2)
        trial.append_endpoint(period / 2, midpoint)
        trial.append_endpoint(period, endpoint)
        np.testing.assert_allclose(trial.filtered_magnet, expected.filtered_magnet, rtol=Fixed.matrix_rank_tolerance, atol=0)
        np.testing.assert_array_equal(trial.filtered_coil, expected.filtered_coil)
        self.assertEqual([row['time_s'] for row in trial.raw_history], [0., period / 2, period])

    def test_adaptive_events_apply_only_prescribed_voltages_without_resetting_current(self):
        trial = AdaptivePrescribedTrial.__new__(AdaptivePrescribedTrial)
        trial.inputs = dict(base_voltage_V=[1.], pulse_voltage_V=[0.1])
        trial.drivers = SimpleNamespace(currents=np.array([0.3]))
        trial.fields = SimpleNamespace(points=[0])
        trial.trace = [dict(time_s=0.)]
        trial.observe, trial.record, trial.force_matrix = Mock(), Mock(), Mock()
        trial.next_scan, trial.next_regulator, trial.stop_reason = 0, 0, None
        trial.process_events(0., True, True, False)
        np.testing.assert_allclose(trial.drivers.voltage, [1.1])
        trial.next_regulator = 1
        trial.process_events(1 / Fixed.current_regulator_rate, True, False, False)
        np.testing.assert_allclose(trial.drivers.voltage, [0.9])
        np.testing.assert_array_equal(trial.drivers.currents, [0.3])
        self.assertFalse(hasattr(trial, 'controller'))

    def test_prescribed_pulses_preserve_currents_and_land_on_event_boundaries(self):
        trial = PrescribedPhysicalTrial.__new__(PrescribedPhysicalTrial)
        trial.fixed_step = Fixed.adaptive_reference_step
        trial.inputs = dict(base_voltage_V=[1.], pulse_voltage_V=[0.1])
        trial.drivers = SimpleNamespace(currents=np.array([0.3]), resistance=1., inductance=1.)
        trial.reference = np.zeros(6)
        trial.matrix = np.zeros((6, 1))
        trial.plant = SimpleNamespace(data=SimpleNamespace(time=0.), magnetic_matrix=Mock(return_value=trial.matrix))
        trial.last_magnet = trial.last_coil = trial.filtered_magnet = trial.filtered_coil = np.zeros(1)
        trial.physical_fields = Mock(return_value=(np.zeros(1), np.zeros(1)))
        trial.record, trial.observe = Mock(), Mock()
        trial.raw_history, trial.stop_reason = [], None
        voltages = []
        def advance(period, matrix):
            voltages.append(float(trial.drivers.voltage[0]))
            trial.drivers.currents += trial.drivers.voltage * period
        trial.advance_physics = advance
        trial.advance_interval(0)
        trial.advance_interval(1)
        count = round(1 / Fixed.current_regulator_rate / trial.fixed_step)
        np.testing.assert_allclose(voltages, [1.1] * count + [0.9] * count)
        self.assertAlmostEqual(trial.drivers.currents[0], 0.3 + 2 / Fixed.current_regulator_rate)
        self.assertAlmostEqual(trial.plant.data.time, 2 / Fixed.current_regulator_rate)
        self.assertTrue(np.all(np.diff([row['time_s'] for row in trial.raw_history]) > 0))
        self.assertFalse(hasattr(trial, 'controller'))

    def test_moving_comparison_rejects_drift_and_separates_unbudgeted_fields(self):
        row = dict(time_s=0., pose=[0.] * 6, currents_A=[0.], world_velocity=[0.] * 6,
            wrench=[0.] * 6, analogue_field_T=[0.])
        reference = dict(hardware_rates_Hz={}, trace=[row])
        candidate = deepcopy(reference)
        candidate['trace'][0]['analogue_field_T'][0] = Fixed.adaptive_field_tolerance
        result = moving_comparison(reference, candidate)
        self.assertTrue(result['converged'])
        self.assertEqual(result['analogue_field_T_maximum_difference'], Fixed.adaptive_field_tolerance)
        candidate['trace'][0]['pose'][0] = Fixed.qualification_position_error
        self.assertFalse(moving_comparison(reference, candidate)['converged'])

    def test_handoff_probes_change_coil_identity_in_rotated_lattice(self):
        coil_layouts.reference_state()
        array = prototype_coils.prototype_array(Inputs.cad_coil_layout)
        groups = {}
        for case in handoff_poses(array):
            array.build(center=case['pose'][:2])
            groups.setdefault(case['handoff_group'], []).append(set(array.identities))
        self.assertEqual(len(groups), 4)
        for group in groups.values():
            self.assertEqual(len(group), 2)
            self.assertNotEqual(group[0], group[1])

    def test_absolute_bound_covers_simultaneous_signed_channel_errors(self):
        refined = np.c_[np.eye(6), np.eye(6)]
        original = refined * (1 - Fixed.qualification_convergence_fraction)
        result = matrix_refinement_metrics(original, refined, Fixed.driver_channel_current)
        difference = (refined - original) @ np.full(refined.shape[1], Fixed.driver_channel_current)
        self.assertLessEqual(np.linalg.norm(difference[:3]), result['force_error_bound_at_channel_limits'])
        self.assertLessEqual(np.linalg.norm(difference[3:]), result['torque_error_bound_at_channel_limits'])
        self.assertAlmostEqual(result['force_relative_column_error'], Fixed.qualification_convergence_fraction)

    def test_identical_matrices_have_zero_error_and_missing_authority_fails(self):
        result = matrix_refinement_metrics(np.eye(6), np.eye(6), Fixed.driver_channel_current)
        self.assertTrue(all(value == 0 for value in result.values()))
        with self.assertRaisesRegex(ValueError, 'authority'):
            matrix_refinement_metrics(np.zeros((6, 6)), np.zeros((6, 6)), Fixed.driver_channel_current)
        with self.assertRaisesRegex(ValueError, 'matching'):
            matrix_refinement_metrics(np.eye(6), np.eye(3), Fixed.driver_channel_current)


class MovingHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        construction = magnetics.ConstructionModel()
        refinement = min(Fixed.current_demand_mesh_refinements)
        geometry, array, _ = construction(Inputs.cad_coil_layout, refinement)
        scenario = moving_handoff_scenario(geometry, array)
        atlas = SimpleNamespace(field=None, identity=None)
        fixed = CoilHandoffTrial(construction, atlas, scenario, Fixed.adaptive_reference_step, refinement)
        cls.trial = AdaptiveCoilHandoffTrial(construction, atlas, fixed.report())
        cls.initial = cls.trial.physical_snapshot()
        cls.initial_history = deepcopy(cls.trial.raw_history)
        cls.initial_trace = deepcopy(cls.trial.trace)

    def setUp(self):
        self.trial.restore_physics(self.initial)
        self.trial.initialize_adaptation()
        self.trial.apply_prescribed_switch(0.)
        self.trial.raw_history = deepcopy(self.initial_history)
        self.trial.trace = deepcopy(self.initial_trace)

    def test_tilted_handoffs_cover_both_axes_signs_and_gap_endpoints(self):
        geometry = self.trial.plant.geometry
        array = prototype_coils.prototype_array(Inputs.cad_coil_layout)
        layout = levitation_sim.magnet_layout_from_geometry()
        endpoints = gap_handoff_scenarios(geometry, array)
        scenarios = tilted_handoff_scenarios(geometry, array, layout)
        self.assertEqual({(row['initial_gap_m'], row['tilt_axis'], row['tilt_sign']) for row in scenarios},
            {(gap, axis, sign) for gap in (geometry.gap, geometry.max_flight_gap)
                for axis in range(2) for sign in (-1, 1)})
        for scenario in scenarios:
            endpoint = next(row for row in endpoints if row['initial_gap_m'] == scenario['initial_gap_m'])
            self.assertEqual(scenario['pose'][3 + scenario['tilt_axis']],
                scenario['tilt_sign'] * levitation_sim.rim_tilt_limit(layout, scenario['initial_gap_m']))
            for key in ('selected_before', 'selected_after', 'switch_time_s', 'duration_s', 'world_velocity'):
                self.assertEqual(scenario[key], endpoint[key])
            self.assertEqual(scenario['pose'][:3], endpoint['pose'][:3])

    def test_longer_handoff_has_one_scheduled_coil_exchange(self):
        geometry = self.trial.plant.geometry
        array = prototype_coils.prototype_array(Inputs.cad_coil_layout)
        for scenario in gap_handoff_scenarios(geometry, array, Fixed.airborne_check_duration):
            self.assertEqual(scenario['duration_s'], Fixed.airborne_check_duration)
            self.assertEqual(scenario['switch_time_s'], Fixed.airborne_check_duration / 2)
            for fraction, selection in ((0., 'selected_before'), (0.25, 'selected_before'),
                    (0.75, 'selected_after'), (1., 'selected_after')):
                pose = np.array(scenario['pose']) + fraction * scenario['duration_s'] * np.array(scenario['world_velocity'])
                array.build(center=pose[:2])
                self.assertEqual(set(array.identities), set(map(tuple, scenario[selection])))

    def test_gap_cases_start_at_opposite_bounds_and_move_inward_without_changing_handoff(self):
        trial = self.trial
        geometry = trial.plant.geometry
        array = prototype_coils.prototype_array(Inputs.cad_coil_layout)
        lower, upper = gap_handoff_scenarios(geometry, array)
        self.assertEqual(lower['pose'][2], 0.)
        self.assertEqual(upper['pose'][2], geometry.max_flight_gap - geometry.gap)
        self.assertGreater(lower['world_velocity'][2], 0.)
        self.assertEqual(upper['world_velocity'][2], -lower['world_velocity'][2])
        for key in ('selected_before', 'selected_after', 'switch_time_s', 'duration_s'):
            self.assertEqual(lower[key], upper[key])
        self.assertNotEqual(lower['selected_before'], lower['selected_after'])

    def test_switch_preserves_current_and_changes_both_incoming_and_outgoing(self):
        trial = self.trial
        initial = trial.drivers.currents.copy()
        before = trial.enabled.copy()
        self.assertFalse(before[trial.incoming].any())
        self.assertTrue(before[trial.outgoing].all())
        trial.apply_prescribed_switch(trial.inputs['switch_time_s'])
        self.assertTrue(trial.enabled[trial.incoming].all())
        self.assertFalse(trial.enabled[trial.outgoing].any())
        np.testing.assert_array_equal(trial.drivers.currents, initial)
        self.assertTrue(np.any(trial.drivers.voltage[trial.incoming] != 0))

    def test_rejected_switched_step_restores_current_and_all_hall_filter_memory(self):
        trial = self.trial
        trial.apply_prescribed_switch(trial.inputs['switch_time_s'])
        start = trial.physical_snapshot()
        period = Fixed.adaptive_max_step / 2
        matrix = trial.matrix.copy()
        with patch.object(trial.plant, 'magnetic_matrix', return_value=matrix), \
                patch.object(trial.fields, 'magnet_field', return_value=trial.last_magnet.copy()), \
                patch.object(trial, 'apply_prescribed_switch'):
            trial.advance_physics(period / 2, matrix)
            trial.advance_physics(period / 2, matrix)
            expected = trial.physical_snapshot()
            trial.restore_physics(start)
            history_length = len(trial.raw_history)
            with patch.object(trial, 'step_error', side_effect=[{'field': 2.}, {'field': 0.5}]):
                trial.advance()
            actual = trial.physical_snapshot()
        self.assertEqual(trial.adaptation['rejected'], 1)
        self.assertEqual(trial.clock, period)
        self.assertEqual(len(trial.raw_history), history_length + 2)
        for key in ('plant', 'currents', 'filtered_command'):
            np.testing.assert_allclose(actual[key], expected[key], rtol=Fixed.matrix_rank_tolerance, atol=Fixed.matrix_rank_tolerance)
        for key in expected['fields']:
            np.testing.assert_array_equal(actual['fields'][key], expected['fields'][key])
        self.assertTrue(np.all(actual['currents'][trial.outgoing] == 0))
        self.assertTrue(np.any(actual['currents'][trial.incoming] != 0))
        trial.restore_physics(start)
        trial.initialize_adaptation()

    def test_clipped_midpoint_and_full_filter_state_enter_adaptive_field_error(self):
        initial = dict(total=np.array([1.]), coil=np.array([1.]))
        coarse = dict(total=np.array([0.]), coil=np.array([0.]), midpoint_coil=np.array([0.]), filtered=np.array([0.1]))
        midpoint = dict(total=np.array([0.]))
        fine = deepcopy(coarse)
        self.assertEqual(self.trial.field_error(initial, coarse, midpoint, fine, Fixed.adaptive_max_step), 0)
        fine['filtered'] += Fixed.adaptive_field_tolerance
        self.assertAlmostEqual(self.trial.field_error(initial, coarse, midpoint, fine, Fixed.adaptive_max_step),
            Fixed.adaptive_field_tolerance)


if __name__ == '__main__':
    unittest.main()

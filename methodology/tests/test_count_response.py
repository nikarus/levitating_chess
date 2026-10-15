from copy import deepcopy
from types import SimpleNamespace
import unittest

from model import Fixed
from methodology.count_response import continue_response


class CountResponseTests(unittest.TestCase):
    def simulation(self, reason):
        peaks = dict(position_error_m=0., angle_error_rad=0., estimated_position_error_m=0.,
            estimated_angle_error_rad=0., current_A=0.)
        simulation = SimpleNamespace(stop_reason=reason, finished=True, source={},
            drivers=SimpleNamespace(current_limit=Fixed.driver_channel_current), peaks=peaks,
            clock=Fixed.airborne_refinement_screen_duration, tracker={'history': [1, 2]},
            sensors={'random': {'state': 123}}, fixed_step=Fixed.adaptive_reference_step)
        simulation.report = lambda: dict(peaks=simulation.peaks)
        return simulation

    def test_diagnostic_stop_is_cleared_without_resetting_coupled_state(self):
        simulation = self.simulation('count_response_comparison_failed')
        before = deepcopy((simulation.clock, simulation.tracker, simulation.sensors, simulation.fixed_step))
        continue_response(simulation)
        self.assertIsNone(simulation.stop_reason)
        self.assertFalse(simulation.finished)
        self.assertEqual(before, (simulation.clock, simulation.tracker, simulation.sensors, simulation.fixed_step))
        self.assertEqual(simulation.checkpoint_schema, 'airborne-count-response-v1')

    def test_operating_failure_cannot_be_cleared(self):
        for reason in ('contact', 'tracking_error_budget_exceeded', 'current_limit_exceeded', None):
            simulation = self.simulation(reason)
            with self.assertRaisesRegex(ValueError, 'operating failure'):
                continue_response(simulation)
            self.assertEqual(simulation.stop_reason, reason)

    def test_comparison_stop_does_not_hide_excessive_tracking_or_current(self):
        for key, limit in (('position_error_m', Fixed.qualification_position_error),
                           ('estimated_angle_error_rad', Fixed.qualification_angle_error),
                           ('current_A', Fixed.driver_channel_current)):
            simulation = self.simulation('peer_trial_failed')
            simulation.peaks[key] = limit * Fixed.qualification_refinement_factor
            with self.assertRaisesRegex(ValueError, 'operating limits'):
                continue_response(simulation)


if __name__ == '__main__':
    unittest.main()

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

from model import Fixed
from methodology.hover import NumericalGate, check_operating_limits, compare_prefixes, comparison_snapshot


def report():
    return dict(passed=False, finished=False, stop_reason=None,
        duration_completed_s=Fixed.airborne_checkpoint_interval,
        hardware_rates_Hz=dict(command=Fixed.current_command_rate, regulator=Fixed.current_regulator_rate, feedback=Fixed.pose_feedback_rate),
        peaks=dict(current_A=1, required_current_A=1, voltage_V=1, copper_loss_W=1,
            estimated_position_error_m=0, estimated_angle_error_rad=0),
        trace=[dict(time_s=0, pose=[0] * 6, estimated_pose=[0] * 6, currents_A=[0])])


class HoverGateTests(unittest.TestCase):
    def test_unfinished_prefix_is_not_reported_as_completed_control(self):
        original = report()
        reference = deepcopy(original)
        comparison = compare_prefixes(original, reference)
        self.assertTrue(comparison['converged'])
        self.assertFalse(original['passed'])
        reference['peaks']['estimated_position_error_m'] = 2 * Fixed.qualification_position_error
        self.assertFalse(compare_prefixes(original, reference)['converged'])

    def test_snapshot_adds_endpoint_without_changing_stored_trace_or_hardware_state(self):
        original = report()
        simulation = SimpleNamespace(report=lambda: original, clock=Fixed.airborne_checkpoint_interval,
            plant=SimpleNamespace(geometry=SimpleNamespace(center_of_mass_height=0),
                truth=lambda: dict(com_position_m=np.zeros(3), rotation=Rotation.identity())),
            tracker=SimpleNamespace(predict=lambda now: dict(valid=True, pose=[0] * 6)),
            drivers=SimpleNamespace(currents=np.zeros(1)))
        original_trace = original['trace']
        snapshot = comparison_snapshot(simulation)
        self.assertEqual(len(original_trace), 1)
        self.assertEqual(snapshot['trace'][-1]['time_s'], simulation.clock)

    def test_failed_comparison_stops_both_workers_and_preserves_failed_window(self):
        baseline, reference = report(), report()
        reference['trace'][0]['currents_A'] = [Fixed.driver_channel_current]
        with tempfile.TemporaryDirectory() as folder:
            simulation = SimpleNamespace(clock=Fixed.airborne_checkpoint_interval, stop_reason=None, finished=False,
                drivers=SimpleNamespace(current_limit=Fixed.driver_channel_current))
            summary = dict(active_window=dict(name='initial_reference', end_s=Fixed.airborne_refinement_duration),
                windows=dict(initial_reference={}))
            gate = NumericalGate(simulation, {}, None, Path(folder), summary, float('inf'))
            connection = Mock()
            connection.recv.side_effect = [dict(kind='progress', report=reference),
                dict(kind='complete', reached_end=False, stop_reason='peer_trial_failed')]
            gate.connection = connection
            gate.process = Mock(exitcode=0)
            with patch('methodology.hover.comparison_snapshot', return_value=baseline):
                gate.progress(baseline)
            self.assertEqual(simulation.stop_reason, 'numerical_comparison_failed')
            self.assertTrue(simulation.finished)
            connection.send.assert_called_once_with(False)
            self.assertFalse(summary['windows']['initial_reference']['comparison']['converged'])
            self.assertEqual(summary['status'], 'failed')

    def test_deadline_keeps_unfinished_reference_window_resumable(self):
        simulation = SimpleNamespace(stop_reason=None, finished=False)
        summary = dict(active_window=dict(name='initial_reference'), windows=dict(initial_reference={}))
        gate = NumericalGate(simulation, {}, None, None, summary, 0)
        gate.connection = Mock()
        gate.process = Mock(exitcode=0)
        gate.finish(dict(kind='complete', reached_end=False, stop_reason=None))
        self.assertEqual(summary['active_window']['name'], 'initial_reference')
        self.assertFalse(simulation.finished)

    def test_operating_limits_stop_even_without_a_reference_running(self):
        for key, value in (('estimated_position_error_m', 2 * Fixed.qualification_position_error),
                           ('current_A', 2 * Fixed.driver_channel_current)):
            simulation = SimpleNamespace(stop_reason=None, finished=False,
                drivers=SimpleNamespace(current_limit=Fixed.driver_channel_current))
            current = report()
            current['peaks'][key] = value
            check_operating_limits(simulation, current)
            self.assertTrue(simulation.finished)
            self.assertIsNotNone(simulation.stop_reason)


if __name__ == '__main__':
    unittest.main()

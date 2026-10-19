from copy import deepcopy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

from model import Fixed
from methodology import airborne, airborne_inputs, electrical


class SharedInputTests(unittest.TestCase):
    def test_shared_joint_drive_preserves_physical_filter_and_does_not_call_feedback(self):
        driver = SimpleNamespace(command=np.zeros(1), currents=np.zeros(1), filtered_command=np.ones(1),
            current_limit=Fixed.driver_channel_current, regulate=Mock(side_effect=AssertionError('Unexpected regulator')))
        simulation = SimpleNamespace(drivers=driver, commands={0.: np.array([2.])},
            instructions={0.: dict(voltage_V=[3.], requested_voltage_V=[4.])},
            next_scan=0, fields=SimpleNamespace(points=[0]), observe_physics=Mock(),
            plant=SimpleNamespace(truth=Mock(return_value={})), stop_reason=None,
            trace=[], physical_row=Mock(return_value=dict(time_s=0.)))
        airborne_inputs.SharedJointInputSimulation.process_events(simulation, 0., True, True, False)
        np.testing.assert_array_equal(driver.command, [2.])
        np.testing.assert_array_equal(driver.voltage, [3.])
        np.testing.assert_array_equal(driver.filtered_command, [1.])
        driver.regulate.assert_not_called()
        self.assertFalse(simulation.finished)
        simulation.commands.clear()
        simulation.instructions.clear()
        airborne_inputs.SharedJointInputSimulation.process_events(simulation, 0., True, True, True)
        self.assertTrue(simulation.finished)

    def driver(self):
        driver = electrical.CoilDrivers.__new__(electrical.CoilDrivers)
        driver.identities = (0,)
        driver.resistance = 1.
        driver.inductance = 1 / Fixed.current_regulator_rate
        driver.current_limit = Fixed.driver_channel_current
        driver.voltage_limit = 1.
        for name in airborne.DRIVER_STATE:
            setattr(driver, name, np.zeros(1))
        return driver

    def test_reconstruction_preserves_control_but_does_not_substitute_physical_current(self):
        driver = self.driver()
        initial = dict(drivers={name: getattr(driver, name).tolist() for name in airborne.DRIVER_STATE})
        period = 1 / Fixed.current_regulator_rate
        intervals, readouts, commands = [], [], []
        for index in range(Fixed.qualification_integration_substeps):
            now = index * period
            command = [Fixed.max_current_command_error * Fixed.qualification_refinement_factor]
            commands.append(dict(time_s=now, command_A=command))
            reading = electrical.CurrentReadings((0,), np.array([now]),
                np.array([[2 ** (Fixed.current_feedback_bits - 1)]]), now)
            driver.set_command(command)
            driver.regulate(reading, now)
            driver.advance(np.array([Fixed.max_current_command_error]), period)
            intervals.append(dict(start_s=now, interval_s=period))
            readouts.append(dict(sample_times_s=[now], adc_codes=reading.adc_codes.tolist(), available_time_s=now))
        checkpoint = dict(drivers={name: getattr(driver, name).tolist() for name in airborne.DRIVER_STATE},
            readouts=readouts, adaptive=dict(adaptation=dict(intervals=intervals)))
        reconstructed = self.driver()
        instructions = airborne_inputs.reconstruct_instructions(reconstructed, initial, checkpoint, commands)
        self.assertEqual(len(instructions), len(readouts))
        self.assertNotEqual(reconstructed.currents.tolist(), checkpoint['drivers']['currents'])
        self.assertNotIn('currents', instructions[0.])
        checkpoint['drivers']['integral'][0] += Fixed.max_current_command_error
        with self.assertRaisesRegex(ValueError, 'controller state differs'):
            airborne_inputs.reconstruct_instructions(reconstructed, initial, checkpoint, commands)

    def test_event_replay_never_uses_sensors_or_changes_physical_current(self):
        simulation = airborne_inputs.SharedInputSimulation.__new__(airborne_inputs.SharedInputSimulation)
        simulation.clock = 0.
        simulation.field_times = [0.]
        simulation.next_scan = 0
        simulation.fields = SimpleNamespace(points=[0])
        simulation.drivers = self.driver()
        simulation.drivers.currents[:] = Fixed.max_current_command_error
        simulation.instructions = {0.: dict(voltage=[Fixed.max_current_command_error])}
        simulation.plant = SimpleNamespace(truth=lambda: dict(com_position_m=np.zeros(3), rotation=Rotation.identity()),
            geometry=SimpleNamespace(center_of_mass_height=0))
        simulation.sensors, simulation.current_sensors = Mock(), Mock()
        simulation.observe_physics = Mock()
        simulation.stop_reason = None
        simulation.trace = []
        with patch.object(airborne_inputs.sensing, 'hall_available_time', return_value=Fixed.airborne_hover_duration):
            simulation.process_events(0., True, True, False)
        self.assertEqual(simulation.drivers.currents.tolist(), [Fixed.max_current_command_error])
        self.assertEqual(simulation.drivers.voltage.tolist(), [Fixed.max_current_command_error])
        simulation.sensors.sample_moving.assert_not_called()
        simulation.current_sensors.sample.assert_not_called()

    def test_physical_comparison_uses_existing_limits_without_claiming_observer_accuracy(self):
        report = dict(hardware_rates_Hz={}, trace=[dict(time_s=0., pose=[0.] * 6, currents_A=[0.])])
        candidate = deepcopy(report)
        result = airborne_inputs.physical_comparison(report, candidate)
        self.assertTrue(result['converged'])
        self.assertFalse(any(name.startswith('estimated') for name in result['checks']))
        candidate['trace'][0]['currents_A'][0] = Fixed.driver_channel_current
        self.assertFalse(airborne_inputs.physical_comparison(report, candidate)['converged'])


if __name__ == '__main__':
    unittest.main()

from types import SimpleNamespace
import unittest

import numpy as np

from model import Fixed
from methodology.command_precision import disable_rounding, set_unrounded_command
from methodology.electrical import CoilDrivers


class CommandPrecisionTests(unittest.TestCase):
    def test_only_rounding_is_removed_and_clipping_is_retained(self):
        limit = Fixed.driver_channel_current
        driver = SimpleNamespace(currents=np.zeros(3), current_limit=limit)
        command = np.array([Fixed.max_current_command_error / Fixed.qualification_refinement_factor, 2 * limit, -2 * limit])
        CoilDrivers.set_command(driver, command)
        self.assertNotEqual(driver.command[0], command[0])
        set_unrounded_command(driver, command)
        np.testing.assert_array_equal(driver.command, [command[0], limit, -limit])
        command[0] = 0.
        self.assertNotEqual(driver.command[0], command[0])

    def test_invalid_commands_still_fail(self):
        driver = SimpleNamespace(currents=np.zeros(1), current_limit=Fixed.driver_channel_current)
        for command in ([np.nan], [np.inf], [0., 0.]):
            with self.assertRaises(ValueError):
                set_unrounded_command(driver, command)

    def test_intervention_does_not_change_the_production_driver(self):
        command = [Fixed.max_current_command_error / Fixed.qualification_refinement_factor]
        driver = CoilDrivers.__new__(CoilDrivers)
        driver.currents = np.zeros(1)
        driver.current_limit = Fixed.driver_channel_current
        simulation = SimpleNamespace(drivers=driver, source={})
        disable_rounding(simulation)
        simulation.drivers.set_command(command)
        self.assertEqual(driver.command.tolist(), command)
        original = CoilDrivers.__new__(CoilDrivers)
        original.currents = np.zeros(1)
        original.current_limit = driver.current_limit
        original.set_command(command)
        self.assertNotEqual(original.command.tolist(), command)
        self.assertEqual(simulation.checkpoint_schema, 'airborne-unrounded-command-v1')


if __name__ == '__main__':
    unittest.main()

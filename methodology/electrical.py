from dataclasses import dataclass

import numpy as np

from model import Inputs, Fixed
from methodology import design, qualification


def current_quantum():
    return 2 * Fixed.driver_peak_current / 2 ** Fixed.current_feedback_bits


@dataclass(frozen=True)
class CurrentReadings:
    identities: tuple
    sample_times_s: np.ndarray
    adc_codes: np.ndarray
    available_time_s: float

    def decode(self, now):
        codes = self.adc_codes
        if (codes.shape != (len(self.sample_times_s), len(self.identities)) or not len(self.sample_times_s)
                or len(set(self.identities)) != len(self.identities) or not np.isfinite(codes).all()
                or np.any(codes != np.floor(codes)) or np.any(codes <= 0)
                or np.any(codes >= 2 ** Fixed.current_feedback_bits - 1)):
            raise ValueError('Current readout is malformed or saturated')
        if (not np.isfinite(self.sample_times_s).all() or np.any(np.diff(self.sample_times_s) <= 0)
                or not np.isfinite(self.available_time_s) or self.available_time_s < self.sample_times_s[-1]
                or not np.isfinite(now) or now < self.available_time_s):
            raise ValueError('Current readout timing is invalid or not yet available')
        return (codes + 0.5) * current_quantum() - Fixed.driver_peak_current


class VirtualCurrentSensors:
    def __init__(self, identities):
        self.identities = tuple(identities)
        self.random = np.random.default_rng(Fixed.current_feedback_seed)
        self.offset = self.random.uniform(-1, 1, len(identities)) * Fixed.current_sense_offset_residual / Fixed.current_sense_resistance
        self.gain = self.random.uniform(1 - Fixed.current_feedback_gain_bound, 1 + Fixed.current_feedback_gain_bound, len(identities))

    def sample(self, currents, time):
        currents = np.asarray(currents)
        if currents.shape != self.offset.shape or not np.isfinite(currents).all() or not np.isfinite(time):
            raise ValueError('Invalid physical current sample')
        observed = currents * self.gain + self.offset + self.random.normal(0, Fixed.current_feedback_noise, len(currents))
        codes = np.floor((observed + Fixed.driver_peak_current) / current_quantum())
        codes = np.clip(codes, 0, 2 ** Fixed.current_feedback_bits - 1).astype(int)
        return CurrentReadings(self.identities, np.array([time]), codes[None, :], time)


class CoilDrivers:
    def __init__(self, construction, identities):
        self.identities = tuple(identities)
        self.copper_resistance = construction.configuration.resistance
        self.resistance = self.copper_resistance + Fixed.driver_hot_resistance + Fixed.current_sense_resistance
        self.inductance = design.winding_inductance(construction.coil, construction.configuration)
        self.voltage_limit = Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction * Fixed.usable_bus_voltage_fraction
        self.current_limit = min(Fixed.driver_channel_current, self.voltage_limit / self.resistance)
        self.currents = np.zeros(len(identities))
        self.command = self.currents.copy()
        self.filtered_command = self.currents.copy()
        self.integral = self.currents.copy()
        self.voltage = self.currents.copy()
        self.requested_voltage = self.currents.copy()

    def set_command(self, command):
        command = np.asarray(command, dtype=float)
        if command.shape != self.currents.shape or not np.isfinite(command).all():
            raise ValueError('Invalid driver current command')
        resolution = 2 * Fixed.max_current_command_error
        self.command = np.clip(np.round(np.clip(command, -self.current_limit, self.current_limit) / resolution)
                               * resolution, -self.current_limit, self.current_limit)

    def regulate(self, readings, now):
        if readings.identities != self.identities or readings.sample_times_s[-1] != now:
            raise ValueError('Current regulator requires a fresh readout of its own channels')
        measured = readings.decode(now)[-1]
        self.voltage, self.integral, self.requested_voltage = qualification.current_regulator(
            self.filtered_command - measured, self.integral, self.resistance, self.inductance, self.voltage_limit)

    def terminal_voltage(self, enabled=True):
        enabled = np.asarray(enabled)
        if enabled.dtype.kind != 'b' or enabled.shape not in ((), self.currents.shape):
            raise ValueError('Driver enabled mask must be boolean and match physical channels')
        return np.where(enabled, self.voltage,
            -Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction * np.sign(self.currents))

    def advance(self, back_emf, period, enabled=True):
        back_emf = np.asarray(back_emf)
        if back_emf.shape != self.currents.shape or not np.isfinite(back_emf).all() or not np.isfinite(period) or period <= 0:
            raise ValueError('Invalid electrical integration inputs')
        voltage = self.terminal_voltage(enabled)
        if np.any(~np.asarray(enabled) & (np.abs(back_emf) >= Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction)):
            raise ValueError('Disabled-channel back-EMF reaches the rail; diode re-conduction is outside this model')
        next_currents, average = qualification.rl_interval(self.currents, voltage, back_emf,
            self.resistance, self.inductance, period, enabled)
        if np.max(np.abs(next_currents)) > Fixed.driver_peak_current:
            raise ValueError('Instantaneous driver protection rating exceeded')
        self.currents = next_currents
        self.filtered_command = self.command + (self.filtered_command - self.command) * np.exp(-2 * np.pi * Fixed.setpoint_filter_cutoff * period)
        return average

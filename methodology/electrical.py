from dataclasses import dataclass

import numpy as np

from model import Inputs, Fixed
from methodology import design, qualification


def current_quantum():
    return 2 * Fixed.driver_peak_current / 2 ** Fixed.current_feedback_bits


def hardware_interface_requirements(current_channels, hall_channels):
    if any(not isinstance(count, int) or isinstance(count, bool) or count <= 0
           for count in (current_channels, hall_channels)):
        raise ValueError('Hardware interface channel counts must be positive integers')
    return dict(
        ratings=dict(full_bus_V=Inputs.selected_bus_voltage,
            usable_terminal_voltage_V=Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction * Fixed.usable_bus_voltage_fraction,
            regulated_channel_current_A=Fixed.driver_channel_current,
            instantaneous_current_limit_A=Fixed.driver_peak_current,
            allowed_ripple_peak_deviation_A=Fixed.driver_max_ripple_current),
        rates_Hz=dict(command=Fixed.current_command_rate, current_regulator=Fixed.current_regulator_rate,
            pose_feedback=Fixed.pose_feedback_rate, pwm_carrier=Fixed.driver_pwm_frequency),
        dynamics_Hz=dict(current_loop_bandwidth=Fixed.current_loop_bandwidth,
            command_filter_cutoff=Fixed.setpoint_filter_cutoff,
            pose_controller=Fixed.pose_controller_frequency,
            pose_observer=Fixed.pose_controller_frequency * Fixed.pose_observer_frequency_ratio),
        current_acquisition=dict(channels=current_channels, nominal_converter_bits=Fixed.current_feedback_bits,
            bipolar_range_A=[-Fixed.driver_peak_current, Fixed.driver_peak_current],
            quantum_A=current_quantum(), noise_rms_A=Fixed.current_feedback_noise,
            residual_offset_bound_A=Fixed.current_sense_offset_residual / Fixed.current_sense_resistance,
            residual_gain_fraction=Fixed.current_feedback_gain_bound,
            shunt_ohm=Fixed.current_sense_resistance,
            shunt_input_peak_V=Fixed.driver_peak_current * Fixed.current_sense_resistance,
            input_referred_noise_rms_V=Fixed.current_feedback_noise * Fixed.current_sense_resistance,
            command_quantum_A=2 * Fixed.max_current_command_error,
            aggregate_samples_per_s=current_channels * Fixed.current_regulator_rate,
            raw_sample_payload_bits_per_s=current_channels * Fixed.current_regulator_rate * Fixed.current_feedback_bits,
            timing='All channels sampled at the regulator event and available immediately; no acquisition skew or conversion/transport delay is modeled.',
            consumers=['Current regulation', 'Hall coil-field subtraction', 'Observer acceleration prediction']),
        hall_acquisition=dict(channels=hall_channels, nominal_converter_bits=Fixed.hall_adc_native_bits,
            samples_per_sensor=Fixed.hall_samples_per_sensor,
            configured_conversion_rate_per_scan_engine=Fixed.hall_adc_sample_rate,
            aggregate_samples_per_s=hall_channels * Fixed.hall_samples_per_sensor * Fixed.pose_feedback_rate,
            raw_sample_payload_bits_per_s=hall_channels * Fixed.hall_samples_per_sensor * Fixed.pose_feedback_rate * Fixed.hall_adc_native_bits),
        limitations=[
            'Configured simulation assumptions, not component specifications or validated worst-case product requirements.',
            'Raw payload excludes addressing, timestamps, framing, retries and scheduling overhead; local processing may avoid transmitting it all.',
            'Nominal converter bits do not establish effective resolution, total accuracy or settling performance.',
            'Counts describe the supplied local observation/drive window, not a selected PCB tile or the installed whole-board population.',
            'The averaged electrical simulation does not qualify the PWM carrier, ripple, dead time, switching losses or bus dynamics.'])


def export_hardware_requirements(evidence_directory, output):
    import hashlib
    import json
    from zipfile import ZipFile, ZIP_DEFLATED
    import model
    from methodology import source_fingerprints

    if output.exists():
        raise ValueError('Preserve existing electronics requirements evidence')
    filenames = ('summary.json', 'verification.json', 'initial_checkpoint.json')
    evidence = {name: json.loads((evidence_directory / name).read_text()) for name in filenames}
    summary, verification, checkpoint = (evidence[name] for name in filenames)
    if not summary['finished'] or summary['stop_reason'] is not None or not verification['passed']:
        raise ValueError('Requirements reference must be a completed, verified bounded flight case')
    requirements = hardware_interface_requirements(len(checkpoint['drivers']['currents']), len(checkpoint['joint']['hall_filter']))
    if summary['hardware_rates_Hz'] != {name: requirements['rates_Hz'][key]
            for name, key in (('command', 'command'), ('regulator', 'current_regulator'), ('feedback', 'pose_feedback'))}:
        raise ValueError('Reference flight timing differs from current requirements')
    for key, value in (('native_bits', Fixed.current_feedback_bits), ('sample_rate_Hz', Fixed.current_regulator_rate),
                       ('quantum_A', current_quantum()), ('sensor_noise_rms_A', Fixed.current_feedback_noise),
                       ('command_step_A', 2 * Fixed.max_current_command_error)):
        if summary['current_conversion'][key] != value:
            raise ValueError(f'Reference current-conversion assumption differs: {key}')
    report = dict(status='provisional_interface_audit_not_hardware_qualified',
        source_sha256=source_fingerprints(__file__, model.__file__),
        reference_directory=str(evidence_directory.resolve()),
        reference_sha256={name: hashlib.sha256((evidence_directory / name).read_bytes()).hexdigest() for name in filenames},
        reference_source_sha256=summary['source_sha256'],
        reference=dict(duration_s=summary['duration_completed_s'], scenario=summary['motion_scenario'],
            verification_passed=verification['passed'], observed_peaks=summary['peaks'],
            limitations=summary['limitations']),
        requirements=requirements,
        unresolved=['Choose a realizable sampled-current sensing/regulation architecture; the legacy comparator/delta-sigma BOM is a historical reference only.',
            'Check channel acquisition throughput, skew, latency, reference accuracy, noise and PWM sample timing using selected circuits.',
            'Recalculate installed channel and sensor counts for the square coil bed before scaling the old BOM.',
            'Derive time/location-dependent losses from workload currents and selected components; no whole-board temperature or supply sizing follows from this bounded flight.',
            'Qualify simultaneous sliding-reset, local dwell and repeated-move workloads; preserve existing thermal failures and uncertainty.',
            'Account for every repeated and shared circuit component, PCB construction, interconnect, cooling, manufacturing and production testing.'])
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in report['source_sha256']:
            archive.write(name, name)
        for name in filenames:
            archive.write(evidence_directory / name, f'reference/{name}')
    (output / 'requirements.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    return report


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


if __name__ == '__main__':
    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser(description='Export provisional electronics requirements from a completed bounded flight.')
    parser.add_argument('--flight-evidence', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    export_hardware_requirements(arguments.flight_evidence, arguments.output)

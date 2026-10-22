import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np

import model
from model import Fixed, Constants
from methodology import qualification, source_fingerprints
from methodology.electronics import current_loop, logic


def save_report(report,output):
    if output.exists():
        raise ValueError('Preserve existing protection evidence')
    serialized=json.dumps(report,indent=Fixed.report_precision,allow_nan=False)+'\n'
    output.mkdir(parents=True)
    with ZipFile(output/'sources.zip','x',compression=ZIP_DEFLATED) as archive:
        for name in report['source_sha256']:
            archive.write(name,name)
    (output/'summary.json').write_text(serialized)
    return report


def held_voltage_limit_time(current, voltage, resistance, inductance, limit):
    if resistance <= 0 or inductance <= 0 or limit <= 0:
        raise ValueError('Positive RL parameters and current limit are required')
    if abs(current) >= limit:
        return 0.
    equilibrium = voltage / resistance
    if abs(equilibrium) <= limit:
        return np.inf
    boundary = np.copysign(limit, equilibrium)
    return float(-inductance / resistance * np.log((boundary-equilibrium)/(current-equilibrium)))


def full_voltage_fault_screen():
    circuit=current_loop.nominal_circuit()
    inductance=circuit['inductance']*min(Fixed.qualification_inductance_factors)
    voltage=circuit['bus_high']
    resistance=circuit['resistance']
    delay=Fixed.electronics_drv8962_enable_delay
    current=Fixed.driver_channel_current
    peak=qualification.rl_step(current,voltage,0.,resistance,inductance,delay)
    equilibrium=voltage/resistance
    threshold=equilibrium+(Fixed.driver_peak_current-equilibrium)*np.exp(resistance*delay/inductance)
    return dict(status='instant_detection_full_bus_command_fault_bound',
        initial_current_A=current,bus_voltage_V=voltage,inductance_H=inductance,resistance_ohm=resistance,
        allowed_peak_current_A=Fixed.driver_peak_current,driver_disable_delay_bound_s=delay,
        time_to_operating_peak_s=held_voltage_limit_time(current,voltage,resistance,inductance,Fixed.driver_peak_current),
        current_at_maximum_disable_delay_A=float(peak),
        latest_ideal_trip_current_for_peak_limit_A=float(threshold),
        meets_operating_peak_limit=bool(peak<=Fixed.driver_peak_current),
        source='https://www.ti.com/lit/ds/symlink/drv8962.pdf',
        limitations=[
            'An erroneous full-bus PWM command starts at the normal regulated current. Detection is instantaneous and EN is lowered immediately, but output disable may take its specified maximum. This is a command fault with an otherwise functional driver, not a physically shorted transistor.',
            'Uses the existing minimum screened inductance, nominal cold path resistance, and maximum operating bus; no sensor, comparator, logic or wiring delay is charged. Extra delay or lower resistance worsens this bound.',
            'This checks whether the existing operating peak can also be guaranteed during this fault. It does not predict that the device will always use its maximum delay, establish damage at the computed current, or qualify a separate transient fault-current allowance.',
            'The native sheet disables internal current regulation. This screen does not include a redesigned independent current limiter or faster isolation stage; the missing-pulse watchdog covers a different fault.'])


def fault_screen(output):
    report=full_voltage_fault_screen()
    report['source_sha256']=source_fingerprints(__file__,model.__file__,qualification.__file__,current_loop.__file__,
        current_loop.power_stage.__file__,current_loop.power_stage.design.__file__,current_loop.power_stage.prototype_coils.__file__)
    return save_report(report,output)


def frozen_command_trial(trace, configuration, ripple_bound, schedule):
    nominal = current_loop.nominal_circuit()
    resistance = nominal['resistance'] * configuration.get('resistance_factor', 1.)
    inductance = nominal['inductance'] * configuration.get('inductance_factor', 1.)
    period = 1 / Fixed.current_regulator_rate
    actuation = schedule['actuation_s']
    np.testing.assert_allclose(trace[:, 0], (np.arange(len(trace))+1)*period,
        rtol=0., atol=np.finfo(float).eps*trace[-1, 0])
    previous_currents = np.r_[0., trace[:-1, 3]]
    previous_voltages = np.r_[0., trace[:-1, 5]]
    update_currents = qualification.rl_step(previous_currents, previous_voltages, 0., resistance, inductance, actuation)
    reconstructed = qualification.rl_step(update_currents, trace[:, 5], 0., resistance, inductance, period-actuation)
    reconstruction_error = float(np.max(np.abs(reconstructed-trace[:, 3])))
    if reconstruction_error > Fixed.electronics_spice_current_tolerance:
        raise ValueError('Recorded trajectory does not reconstruct with its stated averaged RL circuit')
    next_update_currents = qualification.rl_step(trace[:, 3], trace[:, 5], 0., resistance, inductance, actuation)
    limit = Fixed.driver_peak_current-ripple_bound
    delays = np.array([held_voltage_limit_time(current, voltage, resistance, inductance, limit)
        for current, voltage in zip(next_update_currents, trace[:, 5])])
    finite = np.flatnonzero(np.isfinite(delays))
    if not len(finite):
        return dict(reconstruction_error_A=reconstruction_error, finite_crossing_intervals=0,
            minimum_hold_after_missed_update_s=None)
    index = int(finite[np.argmin(delays[finite])])
    return dict(reconstruction_error_A=reconstruction_error, finite_crossing_intervals=len(finite),
        minimum_hold_after_missed_update_s=float(delays[index]), row=index,
        missed_update_time_s=float(trace[index, 0]+actuation),
        current_at_missed_update_A=float(next_update_currents[index]),
        stale_effective_voltage_V=float(trace[index, 5]), resistance_ohm=resistance, inductance_H=inductance,
        ripple_bound_A=ripple_bound, average_current_limit_A=limit,
        interval_has_following_recorded_update=index+1<len(trace))


def run(recording, output):
    if output.exists():
        raise ValueError('Preserve existing protection evidence')
    report_path = recording/'summary.json'
    trace_path = recording/'traces.npz'
    recorded = json.loads(report_path.read_text())
    if not recorded['passed'] or not recorded['single_precision_regulator_and_integer_PWM']:
        raise ValueError('Use a completed passing digital current-control recording')
    for filename, digest in recorded['source_sha256'].items():
        if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != digest:
            raise ValueError(f'Recorded source no longer matches the diagnostic configuration: {filename}')
    traces = np.load(trace_path)
    trials = []
    for index, row in enumerate(recorded['trials']):
        name = f'case_{index//2}_noise_{int(row["quantization_and_noise"])}'
        trial = frozen_command_trial(traces[name], row['configuration'], row['ripple_bound_A'], recorded['schedule'])
        trials.append(dict(trace=name, configuration=row['configuration'], **trial))
    limiting = min((row for row in trials if row['minimum_hold_after_missed_update_s'] is not None),
        key=lambda row: row['minimum_hold_after_missed_update_s'])
    deadline = limiting['minimum_hold_after_missed_update_s']
    channel_interval = logic.controller_schedule()['carrier_ticks']/Fixed.electronics_logic_clock
    report = dict(status='recorded_averaged_frozen_command_protection_screen',
        source_sha256=source_fingerprints(__file__, model.__file__, qualification.__file__, current_loop.__file__, logic.__file__),
        input_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (report_path, trace_path)},
        trial_count=len(trials), trials=trials, limiting_case=limiting,
        group_successful_update_interval_s=channel_interval,
        maximum_driver_enable_delay_s=Fixed.electronics_drv8962_enable_delay,
        maximum_remaining_detection_and_gating_time_s=deadline-Fixed.electronics_drv8962_enable_delay,
        per_channel_periodic_watchdog_is_fast_enough=deadline>=1/Fixed.current_regulator_rate+Fixed.electronics_drv8962_enable_delay,
        group_periodic_watchdog_has_nominal_timing_room=deadline>=channel_interval+Fixed.electronics_drv8962_enable_delay,
        hardware_protection_qualified=False,
        limitations=[
            'Exact RL continuation of recorded effective averaged voltage with the existing conservative ripple reserve. It is not a resolved switching fault or manufacturer protection model.',
            'A CPU failure may leave the last valid voltage active after its next expected replacement. The minimum is over every recorded held-voltage interval; monotonic RL motion makes the endpoint the shortest remaining crossing time.',
            'The diagnostic covers stale valid commands only. Corrupt PWM, rail faults, shoot-through, frozen high outputs, back-EMF, coupling and pending command corruption require separate protection.',
            'No existing watchdog or circuit is assumed. Heartbeats must be tied to successful validated control updates, not free-running peripherals. A group heartbeat cannot detect one skipped coil unless the complete sequence is validated.',
            'The EN propagation delay uses the datasheet maximum. Detection, component tolerance, gate delay and commutation still need qualification.',
            'Tested resistance/filter factors are exploratory sensitivities. This screen does not establish exhaustive operating-corner or fault coverage.'])
    return save_report(report,output)


def watchdog_timing():
    tolerance, tcr = Fixed.electronics_protection_resistor_accuracy
    excursion = max(abs(value-Fixed.electronics_protection_reference_temperature)
        for value in Fixed.electronics_protection_temperature)
    resistor_error = (1+tolerance)*(1+tcr*excursion)-1
    resistor = Fixed.electronics_watchdog_set_resistance
    resistance_interval = np.array([1-resistor_error, 1+resistor_error])*resistor
    low, high = Constants.ltc6993_set_resistance_range
    if resistance_interval[0] < low or resistance_interval[1] > high:
        raise ValueError('Watchdog timing resistor leaves the specified range')
    nominal = resistor*Constants.ltc6993_time_per_resistance*Constants.ltc6993_divisor
    interval = nominal*np.array([(1-resistor_error)*(1-Fixed.electronics_watchdog_pulse_error),
        (1+resistor_error)*(1+Fixed.electronics_watchdog_pulse_error)])
    top, bottom = Fixed.electronics_watchdog_divider_top, Fixed.electronics_watchdog_divider_bottom
    ratios = [bottom*(1+sign*resistor_error)/(top*(1-sign*resistor_error)+bottom*(1+sign*resistor_error))
        for sign in (-1,1)]
    impedance = top*bottom/(top+bottom)*(1+resistor_error)
    leakage = Constants.ltc6993_divider_leakage*impedance/(Fixed.electronics_logic_supply*(1-Fixed.electronics_supply_tolerance))
    code_center = (Constants.ltc6993_divcode+0.5)/Constants.ltc6993_divcode_levels
    divider_pass = max(abs(ratio-code_center)+leakage for ratio in ratios) <= Constants.ltc6993_divcode_margin
    if not divider_pass or impedance > Constants.ltc6993_divider_impedance_limit:
        raise ValueError('Watchdog divider is not guaranteed to select its intended code')
    return dict(nominal_timeout_s=nominal, minimum_timeout_s=float(interval[0]), maximum_timeout_s=float(interval[1]),
        resistor_fractional_envelope=resistor_error, divider_ratio_interval=ratios,
        divider_leakage_ratio_bound=leakage, divider_code_pass=divider_pass,
        typical_startup_s=nominal/Constants.ltc6993_divisor*Constants.ltc6993_startup_master_cycles,
        logic_shutdown_delay_bound_s=sum(Fixed.electronics_protection_logic_delays),
        sources=['https://www.analog.com/media/en/technical-documentation/data-sheets/LTC6993-6993-1-6993-2-6993-3-6993-4.pdf',
            'https://www.ti.com/lit/ds/symlink/sn74lvc1g11.pdf',
            'https://www.ti.com/lit/ds/symlink/sn74lvc1g74.pdf',
            'https://www.ti.com/lit/ds/symlink/sn74lvc2g08.pdf'],
        scope='Datasheet timing-resistor/divider tolerance and logic propagation screen; timer output loading, jitter, trigger delay, supply ramp and resistor aging are not qualified')


class IndependentShutdown:
    def __init__(self, timeout):
        if timeout <= 0:
            raise ValueError('Watchdog timeout must be positive')
        self.timeout = timeout
        self.now = 0.
        self.expiry = 0.
        self.pulse = False
        self.arm = False
        self.safe = False

    def advance(self, now, pulse, arm, power_healthy, driver_healthy, timer_ready=True):
        if now < self.now:
            raise ValueError('Protection events must be chronological')
        if now >= self.expiry or not power_healthy or not driver_healthy or not timer_ready:
            self.safe = False
        if not timer_ready:
            self.expiry = now
        elif pulse and not self.pulse:
            self.expiry = now+self.timeout
        clear_released = timer_ready and now < self.expiry and power_healthy and driver_healthy
        if clear_released and arm and not self.arm:
            self.safe = True
        self.now, self.pulse, self.arm = now, pulse, arm
        return self.safe

    def enables(self, requests):
        return tuple(self.safe and request for request in requests)


def watchdog_screen(recording, output):
    if output.exists():
        raise ValueError('Preserve existing watchdog evidence')
    evidence = json.loads(recording.read_text())
    if evidence['status'] != 'recorded_averaged_frozen_command_protection_screen':
        raise ValueError('Use the recorded frozen-command diagnostic')
    timing = watchdog_timing()
    interval = evidence['group_successful_update_interval_s']
    shutdown = timing['maximum_timeout_s']+timing['logic_shutdown_delay_bound_s']+Fixed.electronics_drv8962_enable_delay
    trials = []
    for timeout in (timing['minimum_timeout_s'], timing['maximum_timeout_s']):
        protection = IndependentShutdown(timeout)
        startup_off = not protection.advance(0., False, False, True, True, timer_ready=False)
        protection.advance(interval, True, False, True, True)
        armed = protection.advance(interval+interval/2, False, True, True, True)
        maintained = all(protection.advance(index*interval, True, True, True, True)
            and protection.advance((index+0.5)*interval, False, True, True, True)
            for index in range(2, Fixed.electronics_acquisition_sar[2]+1))
        expiry = protection.expiry
        disabled = not protection.advance(expiry, False, True, True, True)
        no_automatic_rearm = not protection.advance(expiry+interval, True, True, True, True)
        protection.advance(expiry+interval+interval/2, False, False, True, True)
        explicit_rearm = protection.advance(expiry+2*interval, True, True, True, True)
        fault_latched = not protection.advance(expiry+2.5*interval, False, True, True, False)
        fault_does_not_rearm = not protection.advance(expiry+3*interval, True, True, True, True)
        trials.append(dict(timeout_s=timeout, startup_off=startup_off, armed=armed, maintained=maintained,
            missing_pulse_disabled=disabled, no_automatic_rearm=no_automatic_rearm,
            explicit_rearm=explicit_rearm, driver_fault_latched=fault_latched,
            fault_clear_does_not_rearm=fault_does_not_rearm))
    report = dict(status='hardware_watchdog_logic_and_recorded_timing_screen', timing=timing, trials=trials,
        heartbeat_interval_s=interval, allowed_heartbeat_interval_variation_s=timing['minimum_timeout_s']-interval,
        shutdown_after_last_pulse_using_maximum_driver_delay_s=shutdown,
        recorded_current_limit_margin_s=evidence['limiting_case']['minimum_hold_after_missed_update_s']-shutdown,
        passed=timing['minimum_timeout_s']>interval and shutdown<evidence['limiting_case']['minimum_hold_after_missed_update_s']
            and all(all(value for key,value in row.items() if key!='timeout_s') for row in trials),
        hardware_protection_qualified=False,
        source_sha256=source_fingerprints(__file__,model.__file__),
        input_sha256={str(recording):hashlib.sha256(recording.read_bytes()).hexdigest()},
        limitations=[
            'Behavioural timer/AND/asynchronous-latch model, not a transistor or pad-level simulation. Logic results omit propagation; the separate timing calculation adds datasheet logic bounds.',
            'Only successfully validated, sequential channel commits may trigger the heartbeat. Silent missed channels, bad PWM values and a heartbeat peripheral running by itself are outside this hardware timer protection.',
            'DRV8962 enable delay uses its specified maximum. Remaining timing margin must cover timer jitter/output loading, trigger recognition and PCB effects; it cannot establish a fault-current guarantee.',
            'POWER_FAULT_N must be driven by independent rail supervision; its source remains unfinished. This logic does not limit arbitrary excess current or a stuck-on bridge.',
            'Explicit arming requires clear recovery/setup time and a valid clock pulse. Coincident asynchronous fault/arm edges and supply ramps need implementation-level verification.',
            'Recorded RL timing evidence retains its own source identity; adding protection does not rerun or requalify the earlier current trajectories.'])
    return save_report(report,output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--watchdog', action='store_true')
    mode.add_argument('--full-voltage-fault', action='store_true')
    parser.add_argument('--recording', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.full_voltage_fault:
        report=fault_screen(arguments.output)
    else:
        if arguments.recording is None:
            parser.error('--recording is required for recorded-command and watchdog screens')
        report=(watchdog_screen if arguments.watchdog else run)(arguments.recording, arguments.output)
    print(json.dumps(report if arguments.watchdog or arguments.full_voltage_fault else report['limiting_case'], indent=Fixed.report_precision))

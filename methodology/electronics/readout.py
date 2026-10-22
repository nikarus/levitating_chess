import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np

import model
from model import Inputs, Fixed
from methodology import coil_layouts, electrical, source_fingerprints
from methodology.electronics import power_stage


def edge_intervals(mean_voltage, bus_voltage, frequency):
    if not np.isfinite(frequency) or frequency <= 0:
        raise ValueError('Carrier frequency must be positive and finite')
    duties = power_stage.unipolar_leg_duties(mean_voltage, bus_voltage)
    if any(duty in (0., 1.) for duty in duties):
        raise ValueError('Static bridge legs have no periodic edge-spacing measurement')
    return [[duty / frequency, (1 - duty) / frequency] for duty in duties]


def readout_metrics(gain, shunt, input_noise_density, supply_current):
    if not np.isfinite([gain, shunt, input_noise_density, supply_current]).all() or min(
            gain, shunt, input_noise_density, supply_current) <= 0:
        raise ValueError('Readout parameters must be positive and finite')
    sensitivity = gain * shunt
    excursion = Fixed.driver_peak_current * sensitivity
    rails = [Fixed.electronics_current_reference - excursion,
             Fixed.electronics_current_reference + excursion]
    max_enbw = (Fixed.current_feedback_noise * shunt / input_noise_density) ** 2
    return dict(gain_V_per_V=gain, shunt_ohm=shunt, sensitivity_V_per_A=sensitivity,
        nominal_output_range_V=rails,
        ideal_supply_rail_check=0 < rails[0] and rails[1] < Fixed.electronics_analog_supply,
        peak_shunt_loss_W=Fixed.driver_peak_current ** 2 * shunt,
        amplifier_quiescent_upper_W=Fixed.electronics_analog_supply * supply_current,
        full_supply_adc_quantum_A=Fixed.electronics_analog_supply / sensitivity / 2 ** Fixed.current_feedback_bits,
        minimum_ideal_full_supply_adc_bits=math.ceil(math.log2(
            Fixed.electronics_analog_supply / sensitivity / electrical.current_quantum())),
        typical_white_noise_only_max_ENBW_Hz=max_enbw,
        corresponding_single_pole_cutoff_Hz=max_enbw * 2 / np.pi,
        total_readout_qualified=False)


def screen():
    case = next(row for row in power_stage.topology_cases()
                if row['topology'] == 'unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    limit = case['terminal_voltage_limit_V']
    samples = []
    for bus in (case['bus_low_V'], case['bus_nominal_V'], case['bus_high_V']):
        for voltage in np.linspace(-limit, limit, Fixed.electronics_pwm_screen_points):
            intervals = edge_intervals(float(voltage), bus, Fixed.driver_pwm_frequency)
            shortest = min(min(leg) for leg in intervals)
            samples.append(dict(mean_voltage_V=float(voltage), bus_V=bus,
                shortest_common_mode_edge_interval_s=shortest,
                ina241_documented_edge_spacing_met=shortest >= Fixed.electronics_ina241_edge_spacing))
    candidates = []
    for name, gain, noise, current in Fixed.electronics_readout_specs:
        shunts = (Fixed.current_sense_resistance,) if name == 'INA241A1' else (
            Fixed.current_sense_resistance, Fixed.electronics_readout_trial_shunt)
        for shunt in shunts:
            row = dict(name=name, **readout_metrics(gain, shunt, noise, current))
            if name == 'MAX49925':
                row['within_published_gain_error_test_excursion'] = (
                    Fixed.driver_peak_current * gain * shunt <= Fixed.electronics_max49925_gain_test_excursion)
            candidates.append(row)
    return dict(status='analytical_candidate_screen_not_component_qualification',
        required_current_quantum_A=electrical.current_quantum(),
        required_current_noise_rms_A=Fixed.current_feedback_noise,
        pwm_carrier_Hz=Fixed.driver_pwm_frequency,
        shortest_interval_case=min(samples, key=lambda row: row['shortest_common_mode_edge_interval_s']),
        ina241_documented_edge_spacing_covers_command_envelope=all(
            row['ina241_documented_edge_spacing_met'] for row in samples),
        ina241_edge_spacing_voltage_interval_V=[
            -case['bus_low_V'] * (1 - 2 * Fixed.driver_pwm_frequency * Fixed.electronics_ina241_edge_spacing),
            case['bus_low_V'] * (1 - 2 * Fixed.driver_pwm_frequency * Fixed.electronics_ina241_edge_spacing)],
        samples=samples, candidates=candidates,
        datasheets={
            'INA241A1': 'https://www.ti.com/lit/ds/symlink/ina241a.pdf',
            'AD8410A': 'https://www.analog.com/media/en/technical-documentation/data-sheets/ad8410a.pdf',
            'MAX49925': 'https://www.analog.com/media/en/technical-documentation/data-sheets/max49925.pdf',
            'TPA132A1': 'https://static.3peak.com/res/doc/ds/Datasheet_TPA132.pdf',
            'INA181A1': 'https://www.ti.com/lit/ds/symlink/ina181.pdf'},
        limitations=[
            'Ideal centered unipolar leg timing; finite edge times, dead time, parasitics and movement are not simulated here.',
            'Meeting an edge-spacing condition does not establish sampling accuracy; violating it removes that documented rejection condition, not all amplifier functionality.',
            'Noise uses typical spot input-noise densities as white-noise estimates and assigns the entire existing RMS budget to the amplifier. Flicker, spectral variation, reference, ADC, shunt noise, aliasing and PWM residue require separate analysis.',
            'The illustrative single-pole noise bandwidth is not a selected filter or a verified delay budget.',
            'ADC bits assume a full-supply input span and ideal quantization. A narrower converter range changes this calculation; bits alone do not establish accuracy or noise.',
            'Output checks only screen ideal supply rails; actual output swing, gain/offset variation and ADC loading remain unqualified.',
            'Shunt alternatives are screening cases only; no schematic, controller or flight setting has been changed.',
            'INA181A1 is a low-side architecture candidate, not a qualified replacement on the switching coil terminal; nominal gain requires calibration against the existing residual-error budget.',
            'INA241A vendor macro-model omits enhanced PWM rejection. This screen does not execute or qualify manufacturer circuit models.'])


def export_screen(output):
    if output.exists():
        raise ValueError('Preserve existing readout evidence')
    result = screen()
    result['source_sha256'] = source_fingerprints(__file__, model.__file__, electrical.__file__,
        power_stage.__file__, power_stage.design.__file__, power_stage.prototype_coils.__file__,
        power_stage.qualification.__file__)
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for filename in result['source_sha256']:
            archive.write(filename, filename)
    (output / 'summary.json').write_text(json.dumps(result, indent=Fixed.report_precision) + '\n')
    return result


def adoption_cost_screen(candidate='AD8410A'):
    board = power_stage.design.BoardGeometry()
    array = power_stage.prototype_coils.prototype_array(Inputs.cad_coil_layout)
    count = board.motor_area / 1e6 * coil_layouts.channel_density(array, 'independent')
    _, gain, _, quiescent = next(row for row in Fixed.electronics_readout_specs if row[0] == candidate)
    return dict(
        area_equivalent_channels=count,
        production_boards=Inputs.production_boards,
        area_equivalent_production_amplifiers=count * Inputs.production_boards,
        amplifier_all_powered_quiescent_upper_W=count * Fixed.electronics_analog_supply * quiescent,
        signal_scale_V_per_A=gain * Fixed.electronics_readout_trial_shunt,
        price_observation_date='2026-10-20',
        price_scenarios=[dict(description=name, catalog_quantity=quantity, unit_USD=price,
            amplifier_only_USD_per_board=count * price, source=url)
            for part, name, quantity, price, url in Fixed.electronics_readout_price_observations if part == candidate],
        limitations=[
            'Area times existing independent-channel density, not a finalized edge count or module BOM.',
            'Catalogue prices are extrapolated at their published tiers; no production-volume quotation or guaranteed discount exists.',
            'Only amplifier IC cost is included; shunts, ADC/reference, bridge, control, assembly and cooling are additional.',
            'One amplifier per coil is a normalization only; a low-side architecture may need additional sensing channels and shunts.',
            'All-powered quiescent ceiling excludes shunts and dynamic losses; selective power gating and wake-up behaviour are not designed.'])


def export_sourcing_screen(output):
    if output.exists():
        raise ValueError('Preserve existing sourcing evidence')
    names = tuple(dict.fromkeys(row[0] for row in Fixed.electronics_readout_price_observations))
    technical = screen()
    report = dict(status='sourcing_and_analytical_shortlist_not_hardware_selection',
        candidates=[dict(name=name, cost=adoption_cost_screen(name),
            signal_screen=next(row for row in technical['candidates'] if row['name'] == name
                              and row['shunt_ohm'] == Fixed.electronics_readout_trial_shunt),
            datasheet=technical['datasheets'][name]) for name in names],
        limitations=technical['limitations'],
        sourcing_scope=[
            'Use the configured production-board count times installed demand for quotation; do not equate the published catalogue break with the production-order price.',
            'No supplier quotation has been requested or received; catalogue stock is not a volume-supply commitment.',
            'JLCPCB C45341909 lists TPA132A1-SO1R-S for assembly but exposed no usable price; catalogue search redirected to sign-in. No JLCPCB price is assumed.',
            'The LCSC TPA price is for TSSOP-8, not the SOP-8 JLCPCB listing; package pinouts differ.',
            'TI INA181A1IDBVR is distinct from listings with a third-party manufacturer and similar name.',
            'Manufacturer datasheets support a technical shortlist, not an assertion of acceptable PWM residue, sampled noise, delay or complete BOM savings.'],
        source_sha256=source_fingerprints(__file__, model.__file__, electrical.__file__, power_stage.__file__,
            power_stage.design.__file__, power_stage.prototype_coils.__file__, power_stage.qualification.__file__,
            coil_layouts.__file__))
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for filename in report['source_sha256']:
            archive.write(filename, filename)
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    return report


def vendor_test_netlist(bus, current, command, refinement):
    gain = next(row[1] for row in Fixed.electronics_readout_specs if row[0] == 'AD8410A')
    shunt = Fixed.electronics_readout_trial_shunt
    period = 1 / Fixed.driver_pwm_frequency
    step = period / (Fixed.electronics_spice_steps_per_cycle * refinement)
    stop = Fixed.electronics_spice_cycles * period
    edge = Fixed.driver_switching_time
    duty = power_stage.unipolar_leg_duties(command, bus)[0]
    if min(duty, 1 - duty) * period <= edge:
        raise ValueError('Readout test pulse cannot fit its finite switching edges')
    return f"""AD8410A isolated macromodel check; not a complete coil channel
.include ad8410a.cir
Vs supply 0 {Fixed.electronics_analog_supply:.17g}
Vref reference 0 {Fixed.electronics_current_reference:.17g}
Vcommon common 0 PULSE(0 {bus:.17g} {(1-duty)*period/2:.17g} {edge:.17g} {edge:.17g} {duty*period-edge:.17g} {period:.17g})
Vdiff differential 0 {current*shunt:.17g}
Eplus plus common differential 0 0.5
Eminus minus common differential 0 -0.5
Xamp minus 0 reference output supply reference plus AD8410A
.options reltol={Fixed.electronics_spice_relative_tolerance:.17g} abstol={Fixed.electronics_spice_absolute_tolerance:.17g}
.control
set wr_singlescale
set wr_vecnames
set numdgt=17
dc Vdiff {-Fixed.driver_peak_current*shunt:.17g} {Fixed.driver_peak_current*shunt:.17g} {2*Fixed.driver_peak_current*shunt/(Fixed.electronics_pwm_screen_points-1):.17g}
wrdata dc.dat v(output)
tran {step:.17g} {stop:.17g} 0 {step:.17g}
let reported_current = (v(output)-{Fixed.electronics_current_reference:.17g})/{gain*shunt:.17g}
wrdata transient.dat v(common) reported_current
quit
.endc
.end
"""


def prepare_vendor_evidence(output, vendor_model, executable, interface):
    if output.exists():
        raise ValueError('Preserve existing vendor-model evidence')
    content = vendor_model.read_bytes()
    if interface not in content:
        raise ValueError('Unexpected manufacturer model pin interface')
    executable = str(Path(shutil.which(executable)).resolve())
    output.mkdir(parents=True)
    fingerprints = source_fingerprints(__file__, model.__file__, electrical.__file__, power_stage.__file__,
        power_stage.design.__file__, power_stage.prototype_coils.__file__, power_stage.qualification.__file__,
        coil_layouts.__file__)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for filename in fingerprints:
            archive.write(filename, filename)
    return content, executable, fingerprints


def execute_spice_case(folder, executable, netlist, model_name, compatibility='ltpsa', timeout=Fixed.electronics_spice_case_timeout):
    folder.mkdir()
    (folder / model_name).symlink_to('../' + model_name)
    (folder / '.spiceinit').write_text(f'set ngbehavior={compatibility}\n')
    (folder / 'test.cir').write_text(netlist)
    process = subprocess.run(['timeout', str(timeout),
        executable, '-b', 'test.cir'], cwd=folder,
        capture_output=True, text=True)
    log = process.stdout + process.stderr
    (folder / 'ngspice.log').write_text(log)
    process.check_returncode()
    if any(token in log.lower() for token in ('error', 'aborted', 'timestep too small')):
        raise ValueError(f'ngspice reported an error; inspect {folder}')


def execute_vendor_case(folder, executable, netlist, model_name, compatibility='ltpsa'):
    execute_spice_case(folder, executable, netlist, model_name, compatibility)
    dc = np.loadtxt(folder / 'dc.dat', skiprows=1)
    trace = np.loadtxt(folder / 'transient.dat', skiprows=1)
    validate_vendor_transient(trace)
    if not np.isfinite(dc).all():
        raise ValueError('Nonfinite vendor-model DC results')
    return dc, trace


def validate_vendor_transient(trace):
    stop = Fixed.electronics_spice_cycles / Fixed.driver_pwm_frequency
    if (trace.ndim != 2 or trace.shape[0] < 2 or trace.shape[1] != 3
            or not np.isfinite(trace).all() or np.any(np.diff(trace[:, 0]) <= 0)
            or trace[0, 0] != 0 or abs(trace[-1, 0] - stop) > stop * Fixed.electronics_spice_relative_tolerance):
        raise ValueError('Vendor transient must be finite, ordered and cover the complete requested interval')


def run_vendor_model(output, vendor_model, executable):
    content, executable, fingerprints = prepare_vendor_evidence(
        output, vendor_model, executable, b'.SUBCKT AD8410A 1 2 3 5 6 7 8')
    (output / 'ad8410a.cir').write_bytes(content)
    case = next(row for row in power_stage.topology_cases()
                if row['topology'] == 'unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    limit = case['terminal_voltage_limit_V']
    trials = [(command, 0.) for command in (-limit, 0., limit)]
    trials.extend((0., current) for current in (-Fixed.driver_peak_current, Fixed.driver_peak_current))
    rows = []
    for index, (command, current) in enumerate(trials):
        traces, checks = [], []
        for refinement in (1, 2):
            folder = output / f'case_{index}_refinement_{refinement}'
            dc, trace = execute_vendor_case(folder, executable,
                vendor_test_netlist(case['bus_high_V'], current, command, refinement), 'ad8410a.cir')
            gain = next(row[1] for row in Fixed.electronics_readout_specs if row[0] == 'AD8410A')
            dc_error = (dc[:, 1] - Fixed.electronics_current_reference - gain * dc[:, 0]) / (
                gain * Fixed.electronics_readout_trial_shunt)
            period = 1 / Fixed.driver_pwm_frequency
            end = Fixed.electronics_spice_cycles * period
            times = np.linspace(end - period, end, Fixed.electronics_spice_steps_per_cycle + 1)
            errors = np.interp(times, trace[:, 0], trace[:, 2]) - current
            selected = trace[:, 0] >= end - period
            checks.append(dict(refinement=refinement, max_dc_current_error_A=float(np.max(np.abs(dc_error))),
                last_period_peak_readout_error_A=float(np.max(np.abs(trace[selected, 2] - current))),
                last_period_rms_readout_error_A=float(np.sqrt(np.trapezoid(errors**2, times) / period))))
            traces.append(errors)
        difference = float(np.max(np.abs(traces[0] - traces[1])))
        rows.append(dict(command_V=command, imposed_current_A=current, checks=checks,
            refinement_difference_A=difference,
            refinement_pass=difference <= Fixed.electronics_spice_current_tolerance))
    report = dict(status='macromodel_diagnostic_not_hardware_qualification',
        source_sha256=fingerprints, vendor_sha256=hashlib.sha256(content).hexdigest(),
        vendor_url='https://www.analog.com/media/en/simulation-models/spice-models/ad8410a.cir',
        ngspice_version=subprocess.run([executable, '--version'], capture_output=True, text=True, check=True).stdout,
        compatibility='ngspice ltpsa; unchanged manufacturer model',
        bus_V=case['bus_high_V'], switching_edge_s=Fixed.driver_switching_time,
        trials=rows, cost=adoption_cost_screen(), hardware_qualified=False,
        limitations=[
            'Vendor model was tested by its author in LTspice; these ngspice checks do not establish cross-simulator equivalence.',
            'Vendor header limits common-mode step response to symmetric responses and a 10 mV peak floor; it cannot establish the actual residual at our required accuracy.',
            'Temperature drift, distortion, overload recovery, PSRR and supply/load parameter variation are explicitly omitted.',
            'Noise flatband is limited to 100 kHz by the vendor; ordinary transient runs here do not inject its small-signal noise.',
            'Ideal imposed differential voltage and finite common-mode edges isolate amplifier behaviour; no physical shunt, coil, filter, ADC, driver parasitics or regulation loop is simulated.',
            'DC sweep uses zero common-mode voltage. Transient cases cover a few duties and constant current extremes at the upper supply, not all operating conditions.',
            'Continuous waveform peak/RMS error is not sampled-current error, random noise, or a settled-window acceptance test.'])
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    return report


def low_side_window(high_duty, deadtime):
    if not np.isfinite([high_duty, deadtime]).all() or not 0 < high_duty < 1 or deadtime < 0:
        raise ValueError('Low-side window requires a switching duty and nonnegative dead time')
    period = 1 / Fixed.driver_pwm_frequency
    low_duration = (1 - high_duty) * period
    return dict(ideal_low_interval_s=low_duration,
        settled_input_to_sample_s=low_duration - deadtime - Fixed.driver_switching_time
            - Fixed.electronics_readout_sample_guard,
        sample_phase_s=(1 - high_duty) * period / 2 - Fixed.electronics_readout_sample_guard)


def return_leg_blank_interval(high_duty, signed_current, deadtime):
    low_side_window(high_duty, deadtime)
    if not np.isfinite(signed_current):
        raise ValueError('Leg current must be finite')
    period = 1 / Fixed.driver_pwm_frequency
    sourcing = signed_current < 0
    start = (1-high_duty)*period/2 + (deadtime if sourcing else 0)
    width = high_duty*period + (-deadtime if sourcing else deadtime) - Fixed.driver_switching_time
    if width <= 0 or width + 2*Fixed.driver_switching_time >= period:
        raise ValueError('Return-pair pulse cannot represent the configured conduction interval')
    return start, width


def low_side_test_netlist(high_duty, current, deadtime, refinement, return_pair=False):
    window = low_side_window(high_duty, deadtime)
    if window['settled_input_to_sample_s'] <= 0:
        raise ValueError('No low-side sampling window remains after timing guards')
    gain = next(row[1] for row in Fixed.electronics_readout_specs if row[0] == 'INA181A1')
    period = 1 / Fixed.driver_pwm_frequency
    edge = Fixed.driver_switching_time
    shunt = Fixed.electronics_readout_trial_shunt
    step = period / (Fixed.electronics_low_side_spice_steps_per_cycle * refinement)
    drive = f'Vshunt shunt 0 PULSE({current*shunt:.17g} 0 {(1-high_duty)*period/2:.17g} {edge:.17g} {edge:.17g} {high_duty*period+deadtime-edge:.17g} {period:.17g})'
    inputs = 'shunt 0'
    probe = 'v(shunt)'
    scan = f'dc Vshunt {-Fixed.driver_peak_current*shunt:.17g} {Fixed.driver_peak_current*shunt:.17g} {2*Fixed.driver_peak_current*shunt/(Fixed.electronics_pwm_screen_points-1):.17g}'
    if return_pair:
        shunt /= 2
        drives = [f'Vcurrent imposed_current 0 {current:.17g}']
        for leg, duty, polarity in (('A', high_duty, -1), ('B', 1-high_duty, 1)):
            start, width = return_leg_blank_interval(duty, polarity*current, deadtime)
            drives.extend((
                f'Vgate{leg} gate{leg} 0 PULSE(1 0 {start:.17g} {edge:.17g} {edge:.17g} {width:.17g} {period:.17g})',
                f'Bshunt{leg} shunt{leg} 0 V={polarity*shunt:.17g}*v(imposed_current)*v(gate{leg})'))
        drive = '\n'.join(drives)
        inputs = 'shuntB shuntA'
        probe = 'v(shuntB)-v(shuntA)'
        scan = f'dc Vcurrent {-Fixed.driver_peak_current:.17g} {Fixed.driver_peak_current:.17g} {2*Fixed.driver_peak_current/(Fixed.electronics_pwm_screen_points-1):.17g}'
    return f"""INA181A1 low-side acquisition diagnostic; imposed shunt current, not a complete bridge
.include ina181a1.lib
Vs supply 0 {Fixed.electronics_analog_supply:.17g}
Vref reference 0 {Fixed.electronics_current_reference:.17g}
{drive}
Xamp {inputs} supply 0 reference output INA181A1
Rload output 0 {Fixed.electronics_readout_test_load:.17g}
.options {Fixed.electronics_readout_spice_solver} method={Fixed.electronics_readout_spice_method} maxord={Fixed.electronics_readout_spice_order} reltol={Fixed.electronics_readout_spice_relative_tolerance:.17g} abstol={Fixed.electronics_spice_absolute_tolerance:.17g}
.control
set wr_singlescale
set wr_vecnames
set numdgt=17
{scan}
wrdata dc.dat v(output)
tran {step:.17g} {Fixed.electronics_spice_cycles*period:.17g} 0 {step:.17g}
let reported_current = (v(output)-{Fixed.electronics_current_reference:.17g})/{gain*shunt:.17g}
let differential_input = {probe}
wrdata transient.dat differential_input reported_current
quit
.endc
.end
"""


def low_side_screen():
    case = next(row for row in power_stage.topology_cases()
                if row['topology'] == 'unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    cost = adoption_cost_screen('INA181A1')
    gain = next(row[1] for row in Fixed.electronics_readout_specs if row[0] == 'INA181A1')
    rows = []
    for command in (-case['terminal_voltage_limit_V'], 0., case['terminal_voltage_limit_V']):
        for leg, duty in enumerate(power_stage.unipolar_leg_duties(command, case['bus_low_V'])):
            window = low_side_window(duty, Fixed.electronics_drv8962_deadtime)
            remaining = np.exp(-2 * np.pi * Fixed.electronics_ina181_bandwidth
                * max(0., window['settled_input_to_sample_s']))
            rows.append(dict(command_V=command, leg=leg, high_duty=duty, **window,
                assumed_single_pole_full_step_residue_fraction=float(remaining),
                assumed_single_pole_full_step_residue_at_peak_A=float(remaining * Fixed.driver_peak_current)))
    return dict(status='analytical_low_side_architecture_screen_not_hardware_qualification',
        candidate='INA181A1IDBVR', vendor_model_verification_complete=False,
        current_readout_schematic_changed=False,
        bus_low_V=case['bus_low_V'], command_limit_V=case['terminal_voltage_limit_V'],
        shunt_ohm=Fixed.electronics_readout_trial_shunt,
        peak_shunt_drop_V=Fixed.driver_peak_current * Fixed.electronics_readout_trial_shunt,
        recommended_driver_shunt_drop_limit_V=Fixed.electronics_drv8962_shunt_drop_limit,
        signal_scale_V_per_A=gain * Fixed.electronics_readout_trial_shunt,
        assumed_single_pole_time_to_existing_gain_fraction_s=
            float(-np.log(Fixed.current_feedback_gain_bound) / (2 * np.pi * Fixed.electronics_ina181_bandwidth)),
        two_leg_total_shunt_loss_at_constant_peak_A_W=Fixed.driver_peak_current ** 2 * Fixed.electronics_readout_trial_shunt,
        cost_one_amplifier_per_coil=cost,
        amplifier_only_cost_options=[dict(amplifiers_per_coil=count, shunts_per_coil=count,
            unit_price_USD=price['unit_USD'],
            area_equivalent_production_amplifiers=count*cost['area_equivalent_production_amplifiers'],
            amplifier_only_USD_per_board=count*price['amplifier_only_USD_per_board'],
            all_powered_amplifier_quiescent_upper_W=count*cost['amplifier_all_powered_quiescent_upper_W'])
            for count in (1, 2) for price in cost['price_scenarios']],
        samples=rows,
        datasheets=screen()['datasheets'],
        driver_datasheet='https://www.ti.com/lit/ds/symlink/drv8962.pdf',
        limitations=[
            'Two separate PGND shunts support selection of the longer conducting leg; one shared return shunt cancels recirculation current when both low sides conduct.',
            'One fixed leg must tolerate both extreme duties; two legs still have only a half-cycle low-side interval at zero voltage command.',
            'Single-pole settling based on typical bandwidth is only a screening heuristic. Slew limiting, overshoot, model/part variation, noise and actual switching paths are not represented. It neither qualifies nor conclusively rejects the component.',
            'Dead time is a typical datasheet value, with provisional edge and sampling guards. Body-diode conduction can alter the measured pulse. Actual limits, ADC aperture and jitter require verification.',
            'The existing gain-error allowance is compared with settling alone. Calibration, gain/offset drift, shunt tolerance, ADC/reference, filtering and timing also need budgets; limits are unchanged.',
            'Two-shunt loss assumes ideal complementary leg duties and constant coil current. The summed low-side duty is one; it is not two continuously conducting shunts. Ripple, dead time, temperature and workload change losses.',
            'Amplifier prices are published catalogue tiers, not production quotations or a complete BOM. Neither acquisition architecture is adopted.',
            'Duty-dependent sampling does not implement simultaneous current feedback or establish ripple reconstruction and Hall-subtraction timing.'])


def export_low_side_screen(output):
    if output.exists():
        raise ValueError('Preserve existing low-side architecture evidence')
    report = low_side_screen()
    report['source_sha256'] = source_fingerprints(__file__, model.__file__, electrical.__file__, power_stage.__file__,
        power_stage.design.__file__, power_stage.prototype_coils.__file__, power_stage.qualification.__file__,
        coil_layouts.__file__)
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for filename in report['source_sha256']:
            archive.write(filename, filename)
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    return report


def run_low_side_model(output, vendor_model, executable):
    content, executable, fingerprints = prepare_vendor_evidence(
        output, vendor_model, executable, b'.SUBCKT INA181A1 INP INN VCC GND REF OUT')
    (output / 'ina181a1.lib').write_bytes(content)
    case = next(row for row in power_stage.topology_cases()
                if row['topology'] == 'unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    gain = next(row[1] for row in Fixed.electronics_readout_specs if row[0] == 'INA181A1')
    scale = gain * Fixed.electronics_readout_trial_shunt
    duties = (*power_stage.unipolar_leg_duties(case['terminal_voltage_limit_V'], case['bus_low_V']), 0.5)
    rows = []
    for duty in sorted(duties):
        for deadtime in (0., Fixed.electronics_drv8962_deadtime):
            for current in (-Fixed.driver_peak_current, Fixed.driver_peak_current):
                window = low_side_window(duty, deadtime)
                sample_time = (Fixed.electronics_spice_cycles - 1) / Fixed.driver_pwm_frequency + window['sample_phase_s']
                checks, traces = [], []
                for refinement in (1, 2):
                    folder = output / f'case_{len(rows)}_refinement_{refinement}'
                    dc, trace = execute_vendor_case(folder, executable,
                        low_side_test_netlist(duty, current, deadtime, refinement), 'ina181a1.lib',
                        Fixed.electronics_readout_spice_compatibility)
                    dc_reported = (dc[:, 1] - Fixed.electronics_current_reference) / scale
                    dc_reference = float(np.interp(current * Fixed.electronics_readout_trial_shunt, dc[:, 0], dc_reported))
                    sample = float(np.interp(sample_time, trace[:, 0], trace[:, 2]))
                    previous_sample = float(np.interp(sample_time - 1 / Fixed.driver_pwm_frequency, trace[:, 0], trace[:, 2]))
                    times = np.linspace((Fixed.electronics_spice_cycles - 1) / Fixed.driver_pwm_frequency,
                        Fixed.electronics_spice_cycles / Fixed.driver_pwm_frequency,
                        Fixed.electronics_low_side_spice_steps_per_cycle + 1)
                    traces.append(np.interp(times, trace[:, 0], trace[:, 2]))
                    checks.append(dict(refinement=refinement, sample_A=sample,
                        sample_error_A=sample-current, dc_error_A=dc_reference-current,
                        switching_residue_after_ideal_dc_correction_A=sample-dc_reference,
                        previous_cycle_sample_difference_A=sample-previous_sample,
                        max_dc_error_A=float(np.max(np.abs(
                            dc_reported - dc[:, 0] / Fixed.electronics_readout_trial_shunt)))))
                discrepancy = float(np.max(np.abs(traces[0] - traces[1])))
                rows.append(dict(high_duty=duty, imposed_shunt_current_A=current,
                    assumed_deadtime_s=deadtime, **window, sample_time_s=sample_time, checks=checks,
                    waveform_refinement_difference_A=discrepancy,
                    refinement_pass=discrepancy <= Fixed.electronics_spice_current_tolerance,
                    cycle_settled=abs(checks[-1]['previous_cycle_sample_difference_A']) <= Fixed.electronics_spice_current_tolerance,
                    within_entire_existing_gain_budget=abs(
                        checks[-1]['switching_residue_after_ideal_dc_correction_A']) <= abs(current) * Fixed.current_feedback_gain_bound))
    report = dict(low_side_screen(), status='low_side_acquisition_diagnostic_not_hardware_qualification',
        source_sha256=fingerprints, vendor_sha256=hashlib.sha256(content).hexdigest(),
        vendor_url='https://www.ti.com/lit/zip/SBOMBB7',
        ngspice_version=subprocess.run([executable, '--version'], capture_output=True, text=True, check=True).stdout,
        compatibility=f'ngspice {Fixed.electronics_readout_spice_compatibility}, {Fixed.electronics_readout_spice_solver}; unchanged TI PSpice model',
        bus_low_V=case['bus_low_V'], command_limit_V=case['terminal_voltage_limit_V'],
        candidate='INA181A1IDBVR', shunt_ohm=Fixed.electronics_readout_trial_shunt,
        peak_shunt_drop_V=Fixed.driver_peak_current * Fixed.electronics_readout_trial_shunt,
        recommended_driver_shunt_drop_limit_V=Fixed.electronics_drv8962_shunt_drop_limit,
        signal_scale_V_per_A=scale, trials=rows, hardware_qualified=False,
        vendor_model_verification_complete=all(
            row['refinement_pass'] and row['cycle_settled'] for row in rows),
        limitations=[
            'A separate PGND shunt measures signed leg current when its low-side path conducts. Two shunts permit selecting the longer low-side interval; a common shunt under both legs cancels the recirculating current when both low sides conduct.',
            'One fixed leg must tolerate all tested duties. Two legs do not lengthen the window at zero commanded voltage, where both duties are one half.',
            'Imposed constant current pulses isolate acquisition settling, with the resistive output load from the vendor reference test. No coil ripple, actual bridge transistor/diode current, shunt inductance, ADC load, output filter, noise or closed-loop regulation is simulated.',
            'Zero-dead-time cases are optimistic diagnostics; typical-dead-time cases delay the return of measurable current and are not a transistor-level prediction. Body-diode conduction can change that waveform. Timing limits and parasitics remain unqualified.',
            'Sampling just before the next edge is duty dependent, not simultaneous at the regulator clock. Acquisition skew, ripple reconstruction and Hall alignment remain unmodeled.',
            'Ideal DC correction is a diagnostic subtraction, not a selected calibration method. The entire residual-gain budget comparison leaves no allocation for shunt drift, gain uncertainty, ADC or reference errors; exceeding it rejects a simple settled-window interpretation, passing it is not qualification.',
            'Vendor header covers gain, bandwidth, CMRR, PSRR, voltage noise, offset, input bias, quiescent current, startup and output impedance. This nominal model does not establish production variation or temperature corners.',
            'Ordinary transient runs do not inject the vendor small-signal noise. Noise estimates remain in the separate analytical readout screen.',
            'Cross-simulator equivalence to PSpice is not established. Source model, logs, DC curves and refinement traces are retained.',
            'Amplifier cost excludes shunts, ADC/reference, calibration, assembly and cooling; actual module count and production price are unresolved.'])
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    return report


def run_return_pair_model(output, vendor_model, executable):
    content, executable, fingerprints = prepare_vendor_evidence(
        output, vendor_model, executable, b'.SUBCKT INA181A1 INP INN VCC GND REF OUT')
    (output / 'ina181a1.lib').write_bytes(content)
    case = next(row for row in power_stage.topology_cases()
                if row['topology'] == 'unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    gain = next(row[1] for row in Fixed.electronics_readout_specs if row[0] == 'INA181A1')
    shunt = Fixed.electronics_readout_trial_shunt / 2
    scale = gain * shunt
    period = 1 / Fixed.driver_pwm_frequency
    stop = Fixed.electronics_spice_cycles * period
    times = np.linspace(stop-period, stop, Fixed.electronics_low_side_spice_steps_per_cycle + 1)
    rows = []
    for command in (-case['terminal_voltage_limit_V'], 0., case['terminal_voltage_limit_V']):
        duty = power_stage.unipolar_leg_duties(command, case['bus_low_V'])[0]
        for deadtime in (0., Fixed.electronics_drv8962_deadtime):
            for current in (-Fixed.driver_peak_current, Fixed.driver_peak_current):
                traces, checks = [], []
                for refinement in (1, 2):
                    folder = output / f'case_{len(rows)}_refinement_{refinement}'
                    dc, trace = execute_vendor_case(folder, executable,
                        low_side_test_netlist(duty, current, deadtime, refinement, return_pair=True),
                        'ina181a1.lib', Fixed.electronics_readout_spice_compatibility)
                    sampled = np.interp(times, trace[:, 0], trace[:, 2])
                    previous = np.interp(times-period, trace[:, 0], trace[:, 2])
                    mean = float(np.trapezoid(sampled, times) / period)
                    dc_reference = float((np.interp(current/2, dc[:, 0], dc[:, 1])
                        - Fixed.electronics_current_reference) / scale)
                    input_mean = float(np.trapezoid(np.interp(times, trace[:, 0], trace[:, 1]), times) / period / shunt)
                    checks.append(dict(refinement=refinement, carrier_mean_A=mean,
                        raw_mean_error_A=mean-current,
                        mean_error_after_ideal_dc_correction_A=mean-dc_reference,
                        ideal_input_mean_error_A=input_mean-current,
                        previous_cycle_mean_difference_A=float(mean-np.trapezoid(previous, times)/period),
                        raw_carrier_ripple_rms_A=float(np.sqrt(np.trapezoid((sampled-mean)**2, times)/period)),
                        output_range_V=[float(Fixed.electronics_current_reference+scale*np.min(sampled)),
                                        float(Fixed.electronics_current_reference+scale*np.max(sampled))]))
                    traces.append(sampled)
                waveform_difference = float(np.max(np.abs(traces[0]-traces[1])))
                mean_difference = abs(checks[0]['carrier_mean_A']-checks[1]['carrier_mean_A'])
                rows.append(dict(command_V=command, current_A=current, assumed_deadtime_s=deadtime,
                    checks=checks, waveform_refinement_difference_A=waveform_difference,
                    waveform_refinement_pass=waveform_difference <= Fixed.electronics_spice_current_tolerance,
                    carrier_mean_refinement_difference_A=mean_difference,
                    carrier_mean_refinement_pass=mean_difference <= Fixed.electronics_spice_current_tolerance,
                    cycle_mean_settled=abs(checks[-1]['previous_cycle_mean_difference_A']) <= Fixed.electronics_spice_current_tolerance))
    cost = adoption_cost_screen('INA181A1')
    noise = next(row[2] for row in Fixed.electronics_readout_specs if row[0] == 'INA181A1')
    report = dict(status='one_amplifier_two_shunt_mean_acquisition_screen_not_qualified',
        source_sha256=fingerprints, vendor_sha256=hashlib.sha256(content).hexdigest(),
        vendor_url='https://www.ti.com/lit/zip/SBOMBB7',
        compatibility=f'ngspice {Fixed.electronics_readout_spice_compatibility}, {Fixed.electronics_readout_spice_solver}; unchanged TI model',
        ngspice_version=subprocess.run([executable, '--version'], capture_output=True, text=True, check=True).stdout,
        amplifiers_per_coil=1, shunts_per_coil=2, shunt_ohm=shunt,
        mean_signal_scale_V_per_A=scale, peak_shunt_drop_V=Fixed.driver_peak_current*shunt,
        ideal_constant_peak_total_shunt_loss_W=Fixed.driver_peak_current**2*shunt,
        amplifier_cost=cost,
        ideal_full_supply_adc_bits_for_mean=math.ceil(math.log2(
            Fixed.electronics_analog_supply / scale / electrical.current_quantum())),
        typical_amplifier_only_max_ENBW_Hz=(Fixed.current_feedback_noise*shunt/noise)**2,
        trials=rows, hardware_qualified=False,
        limitations=[
            'One amplifier measures PGND_B minus PGND_A across two separate shunts. Imposed signed currents represent each conducting low-side path, not a transistor-level bridge.',
            'For constant current and complementary ideal leg duties, the differential input averages to current times one shunt. Both low sides produce twice that signal; both high sides produce zero.',
            'The shunts are each half the previous trial value to preserve peak amplifier output headroom. Mean sensitivity is halved, increasing ADC-resolution and noise-filtering demands.',
            'Dead-time cases assume equal delays: the sourcing-leg body diode extends its conducting interval while the sinking-leg interval shortens. Unequal delays, reverse recovery, output capacitance, shunt mismatch and parasitics are not covered.',
            'Carrier integration is an ideal recorded-waveform diagnostic, not a selected ADC or filter. No analog filtering, aliasing, acquisition aperture, digital delay or current reconstruction is implemented.',
            'Constant imposed currents exclude coil ripple, changing commands, current zero crossings and mutual coupling. Carrier mean is not instantaneous current for Hall-field subtraction.',
            'Ideal DC correction is diagnostic; calibration and temperature drift are not qualified. Ordinary transient simulations do not inject the manufacturer small-signal noise.',
            'Carrier-mean and full-waveform numerical checks use the same existing discrepancy limit and are reported separately. A mean pass does not erase a waveform failure or qualify hardware.',
            'Cost covers one amplifier only; the two shunts, ADC, reference, filtering, control and assembly remain additional.'])
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    return report


def ramped_readout_supplies(netlist):
    supply = f'Vs supply 0 {Fixed.electronics_analog_supply:.17g}'
    reference = f'Vref reference 0 {Fixed.electronics_current_reference:.17g}'
    if netlist.count(supply)!=1 or netlist.count(reference)!=1:
        raise ValueError('Startup fixture must have the reviewed supply and reference sources')
    netlist = netlist.replace(supply, f'Vs supply 0 PWL(0 0 {Fixed.electronics_readout_supply_rise:.17g} {Fixed.electronics_analog_supply:.17g})')
    netlist = netlist.replace(reference, f'Eref reference 0 supply 0 {Fixed.electronics_current_reference/Fixed.electronics_analog_supply:.17g}')
    return '\n'.join(line+' uic' if line.startswith('tran ') else line for line in netlist.splitlines())+'\n'


def run_startup_check(output, vendor_model, executable):
    content, executable, fingerprints = prepare_vendor_evidence(output, vendor_model, executable,
        b'.SUBCKT INA181A1 INP INN VCC GND REF OUT')
    (output/'ina181a1.lib').write_bytes(content)
    rows=[]
    for ramped in (False, True):
        folder=output/('power_ramp' if ramped else 'dc_start')
        folder.mkdir()
        (folder/'ina181a1.lib').symlink_to('../ina181a1.lib')
        (folder/'.spiceinit').write_text(f'set ngbehavior={Fixed.electronics_readout_spice_compatibility}\n')
        netlist=low_side_test_netlist(0.5, 0., 0., 1)
        netlist='\n'.join(line for line in netlist.splitlines() if not line.startswith(('dc ', 'wrdata dc.dat')))+'\n'
        if ramped:
            netlist=ramped_readout_supplies(netlist)
        (folder/'test.cir').write_text(netlist)
        process=subprocess.run(['timeout',str(Fixed.electronics_spice_case_timeout),executable,'-b','test.cir'],
            cwd=folder,capture_output=True,text=True)
        log=process.stdout+process.stderr
        (folder/'ngspice.log').write_text(log)
        completed=process.returncode==0 and not any(token in log.lower() for token in ('error','aborted','timestep too small'))
        row=dict(power_ramped=ramped,returncode=process.returncode,completed=completed)
        if completed:
            trace=np.loadtxt(folder/'transient.dat',skiprows=1)
            row['last_current_A']=float(trace[-1,2])
            row['end_time_s']=float(trace[-1,0])
        rows.append(row)
    report=dict(status='isolated_zero_input_startup_diagnostic',source_sha256=fingerprints,
        vendor_sha256=hashlib.sha256(content).hexdigest(),trials=rows,hardware_qualified=False)
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    return report


def interval_means(trace, start, stop):
    if not trace[0,0]<=start<stop<=trace[-1,0]:
        raise ValueError('Averaging window must be covered by the recorded trace')
    inside = (trace[:,0]>start)&(trace[:,0]<stop)
    time = np.r_[start,trace[inside,0],stop]
    values = np.vstack((np.array([np.interp(start,trace[:,0],trace[:,column]) for column in range(1,trace.shape[1])]),
                        trace[inside,1:],
                        np.array([np.interp(stop,trace[:,0],trace[:,column]) for column in range(1,trace.shape[1])])))
    return np.trapezoid(values,time,axis=0)/(stop-start)


def run_return_paths(output, vendor_model, executable, physical_only=False):
    content, executable, fingerprints = prepare_vendor_evidence(output, vendor_model, executable,
        b'.SUBCKT INA181A1 INP INN VCC GND REF OUT')
    (output/'ina181a1.lib').write_bytes(content)
    case = next(row for row in power_stage.topology_cases()
                if row['topology']=='unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    period = 1 / Fixed.driver_pwm_frequency
    capacitance=min(Fixed.electronics_bridge_output_capacitances)
    trials = [(current, 0., 'pgndA', capacitance) for current in (-Fixed.driver_channel_current, 0.,
               Fixed.driver_channel_current/2, Fixed.driver_channel_current)]
    trials.extend(((Fixed.driver_channel_current, Fixed.electronics_acquisition_leg_skew, 'pgndA', capacitance),
                   (Fixed.driver_channel_current, 0., 'ground', capacitance)))
    trials.extend((current,0.,'pgndA',max(Fixed.electronics_bridge_output_capacitances))
                  for current in (0.,Fixed.driver_channel_current))
    rows = []
    for index, (request, skew, bulk_return, capacitance) in enumerate(trials):
        checks, signals = [], []
        for refinement in (1,2):
            netlist, settings = power_stage.return_path_netlist(case, request, skew, bulk_return, refinement, physical_only, capacitance)
            if not physical_only:
                netlist=ramped_readout_supplies(netlist)
            folder = output/f'case_{index}_refinement_{refinement}'
            execute_spice_case(folder, executable, netlist, 'ina181a1.lib', Fixed.electronics_readout_spice_compatibility)
            trace = np.loadtxt(folder/'transient.dat', skiprows=1)
            if (trace.ndim!=2 or trace.shape[1]!=12 or not np.isfinite(trace).all() or
                np.any(np.diff(trace[:,0])<=0) or trace[0,0]>settings['stop_s']-2*period or
                abs(trace[-1,0]-settings['stop_s'])>settings['stop_s']*Fixed.electronics_spice_relative_tolerance):
                raise ValueError(f'Incomplete or invalid physical return-path trace: {folder}')
            time = np.linspace(settings['stop_s']-period, settings['stop_s'],
                               Fixed.electronics_low_side_spice_steps_per_cycle+1)
            sampled = np.array([np.interp(time,trace[:,0],trace[:,column]) for column in range(1,12)]).T
            means = interval_means(trace,settings['stop_s']-period,settings['stop_s'])
            previous_means = interval_means(trace[:,:4],settings['stop_s']-2*period,settings['stop_s']-period)
            checks.append(dict(refinement=refinement, mean_actual_current_A=float(means[0]),
                mean_reported_current_A=float(means[1]), mean_shunt_difference_current_A=float(means[2]),
                mean_readout_error_A=float(means[1]-means[0]),
                mean_return_path_error_A=float(means[2]-means[0]),
                peak_current_A=float(np.max(np.abs(sampled[:,0]))),
                peak_ripple_A=float(np.max(np.abs(sampled[:,0]-means[0]))),
                peak_pgnd_voltage_V=float(np.max(np.abs(sampled[:,3:5]))),
                maximum_kcl_residual_A=float(np.max(np.abs(trace[:,6]))),
                previous_cycle_mean_difference_A=float(np.max(np.abs(means[:3]-previous_means))),
                mean_capacitor_difference_current_A=float(means[8]),
                peak_capacitor_difference_current_A=float(np.max(np.abs(sampled[:,8])))))
            signals.append(sampled)
        means_difference = max(abs(checks[0][key]-checks[1][key]) for key in
            ('mean_actual_current_A','mean_reported_current_A','mean_shunt_difference_current_A','mean_readout_error_A'))
        waveform_difference = float(np.max(np.abs(signals[0][:,:3]-signals[1][:,:3])))
        stable = checks[-1]['previous_cycle_mean_difference_A']<=Fixed.electronics_spice_current_tolerance
        row = dict(settings=settings, checks=checks, carrier_mean_refinement_difference_A=means_difference,
                   carrier_mean_refinement_pass=means_difference<=Fixed.electronics_spice_current_tolerance,
                   waveform_refinement_difference_A=waveform_difference,
                   waveform_refinement_pass=waveform_difference<=Fixed.electronics_spice_current_tolerance,
                   cycle_means_settled=stable,
                   kcl_pass=checks[-1]['maximum_kcl_residual_A']<=Fixed.electronics_spice_current_tolerance)
        rows.append(row)
        (output/'partial.json').write_text(json.dumps(rows,indent=Fixed.report_precision)+'\n')
        if not row['carrier_mean_refinement_pass'] or not stable or not row['kcl_pass']:
            raise ValueError('Return-path carrier means are unresolved; preserved partial evidence')
    endpoints = [rows[index]['checks'][-1] for index in (0,3)]
    slope = (endpoints[1]['mean_reported_current_A']-endpoints[0]['mean_reported_current_A'])/(endpoints[1]['mean_actual_current_A']-endpoints[0]['mean_actual_current_A'])
    intercept = endpoints[0]['mean_reported_current_A']-slope*endpoints[0]['mean_actual_current_A']
    for row in rows:
        check = row['checks'][-1]
        row['residual_after_nominal_two_point_calibration_A'] = (check['mean_reported_current_A']-intercept)/slope-check['mean_actual_current_A']
    report = dict(status='physical_return_paths_diagnostic_not_qualified',
        amplifier_model='ideal_gain_only' if physical_only else 'TI_INA181_unchanged',
        source_sha256=fingerprints, vendor_sha256=hashlib.sha256(content).hexdigest(),
        vendor_url='https://www.ti.com/lit/zip/SBOMBB7', driver_datasheet='https://www.ti.com/lit/ds/symlink/drv8962.pdf',
        ngspice_version=subprocess.run([executable,'--version'],capture_output=True,text=True,check=True).stdout,
        trials=rows, calibration=dict(gain=slope,offset_A=intercept,
            meaning='Ideal calibration from simulated actual carrier means at nominal signed endpoints; no physical reference or procedure qualified.'),
        hardware_qualified=False, schematic_changed=False,
        limitations=[
            'Physical coil, separate PGND resistors and bypass-capacitor branches satisfy circuit KCL. No imposed shunt-voltage sources. The amplifier_model field distinguishes ideal gain isolation from the unchanged TI model; ideal gain has no loading, offset, noise or bandwidth.',
            'DRV8962 is represented by bidirectional finite-conductance switches and piecewise-linear body diodes, not a manufacturer transistor model. Diode knee/slope and skew are exploratory; the configured switching time drives a conductance ramp, not a calibrated output-voltage edge.',
            'Typical input delay, dead time and cold on-resistance are not guaranteed worst-case timing bounds. No adequate manufacturer driver transient model was located in the checked product resources.',
            'Open-loop voltage includes estimated resistive and dead-time drops; this feedforward is approximate and there is no current regulator. Actual current, not requested current, is the measurement reference.',
            'Local VM capacitors return to the individual PGND nodes. Bulk return is compared above and below the shunts; neither placement is qualified without the shared bus/layout.',
            'Constant FET output capacitance is an exploratory sensitivity parameter, not a known DRV8962 bound. Motor capacitors start precharged; amplifier supply/reference ramp together from zero when its vendor model is used.',
            'Ideal stiff supply and references; no PCB/package inductance, capacitor ESR, voltage-dependent MOSFET output capacitance, diode recovery, temperature, ADC, external analog filter or random noise.',
            'Carrier-mean convergence does not establish waveform or sampled acquisition accuracy. Two-point calibration cannot qualify duty-, current-, timing- and temperature-dependent errors.',
            'This candidate SPICE netlist is not the current KiCad inline-sense design; no schematic or hardware adoption is claimed.'])
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    (output/'partial.json').unlink()
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Screen current-readout timing, range, quantization and noise without claiming circuit qualification.')
    parser.add_argument('--output', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--vendor-model', type=Path)
    mode.add_argument('--low-side-vendor-model', type=Path)
    mode.add_argument('--startup-vendor-model', type=Path)
    mode.add_argument('--return-path-vendor-model', type=Path)
    mode.add_argument('--return-pair-vendor-model', type=Path)
    mode.add_argument('--low-side-screen', action='store_true')
    mode.add_argument('--sourcing', action='store_true')
    parser.add_argument('--physical-return-only', action='store_true')
    parser.add_argument('--ngspice', default='ngspice')
    arguments = parser.parse_args()
    if arguments.physical_return_only and not arguments.return_path_vendor_model:
        parser.error('--physical-return-only requires --return-path-vendor-model')
    if arguments.startup_vendor_model:
        run_startup_check(arguments.output, arguments.startup_vendor_model, arguments.ngspice)
    elif arguments.return_path_vendor_model:
        run_return_paths(arguments.output, arguments.return_path_vendor_model, arguments.ngspice, arguments.physical_return_only)
    elif arguments.return_pair_vendor_model:
        run_return_pair_model(arguments.output, arguments.return_pair_vendor_model, arguments.ngspice)
    elif arguments.low_side_screen:
        export_low_side_screen(arguments.output)
    elif arguments.low_side_vendor_model:
        run_low_side_model(arguments.output, arguments.low_side_vendor_model, arguments.ngspice)
    elif arguments.vendor_model:
        run_vendor_model(arguments.output, arguments.vendor_model, arguments.ngspice)
    elif arguments.sourcing:
        export_sourcing_screen(arguments.output)
    else:
        export_screen(arguments.output)

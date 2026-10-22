import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np

import model
from model import Fixed
from methodology import source_fingerprints
from methodology.electronics import acquisition, power_stage, readout


def high_gate_average(local_time, step, duty, edge_delay=0.):
    period = 1 / Fixed.driver_pwm_frequency
    rise = (1 - duty) * period / 2 + edge_delay
    fall = rise + duty * period
    edge = Fixed.driver_switching_time
    if np.any(rise < 0) or np.any(fall + edge > period):
        raise ValueError('Finite gate edges must fit within a carrier period')

    def primitive(time):
        return (np.maximum(time - rise, 0.) ** 2 - np.maximum(time - rise - edge, 0.) ** 2
                - np.maximum(time - fall, 0.) ** 2 + np.maximum(time - fall - edge, 0.) ** 2) / (2 * edge)

    return np.clip((primitive(local_time + step) - primitive(local_time)) / step, 0., 1.)


def periodic_delay(samples, delay, frequency):
    frequencies = np.fft.rfftfreq(len(samples), 1 / (len(samples) * frequency))
    return np.fft.irfft(np.fft.rfft(samples) * np.exp(-2j * np.pi * frequencies * delay), n=len(samples))


def causal_burst_average(samples, count, frequency, last_sample_skew=0., quantize=None):
    spacing = len(samples) * frequency / Fixed.driver_pwm_frequency / count
    if not float(spacing).is_integer():
        raise ValueError('Sample spacing must align with the uniform screening grid')
    delayed = periodic_delay(samples, -last_sample_skew, frequency) if last_sample_skew else samples
    if quantize is not None:
        delayed, samples = quantize(delayed), quantize(samples)
    return (delayed + sum(np.roll(samples, int(spacing) * index) for index in range(1, count))) / count


def bridge_waveforms(case, changing, polarity, leg_skew, refinement):
    frequency = Fixed.current_loop_bandwidth if changing else Fixed.driver_pwm_frequency
    carriers = Fixed.driver_pwm_frequency / frequency
    regulator_periods = Fixed.current_regulator_rate / frequency
    if not float(carriers).is_integer() or changing and not float(regulator_periods).is_integer():
        raise ValueError('Periodic test frequencies must align with the carrier and regulator')
    steps = Fixed.electronics_low_side_spice_steps_per_cycle * refinement
    count = int(carriers) * steps
    step = 1 / (Fixed.driver_pwm_frequency * steps)
    time = (np.arange(count) + .5) * step
    update_time = np.floor(np.arange(count) * step * Fixed.current_regulator_rate) / Fixed.current_regulator_rate
    resistance, inductance = case['coil_R_ohm'], case['minimum_coil_L_H']
    command = np.full(count, polarity * Fixed.driver_channel_current * resistance)
    if changing:
        phase = 2 * np.pi * frequency * update_time
        command = Fixed.driver_channel_current * (resistance * np.sin(phase) +
                    inductance * 2 * np.pi * frequency * np.cos(phase))
    if np.max(np.abs(command)) > case['terminal_voltage_limit_V']:
        raise ValueError('Diagnostic exceeds the configured command-voltage bound')
    duty_a, duty_b = power_stage.unipolar_leg_duties(float(command[0]), case['bus_nominal_V'])
    duties_a = np.full(count, duty_a)
    duties_b = np.full(count, duty_b)
    if changing:
        duties_a = (1 + command / case['bus_nominal_V']) / 2
        duties_b = 1 - duties_a
    local_time = np.tile(np.arange(steps) * step, int(carriers))
    high_a = high_gate_average(local_time, step, duties_a)
    high_b = high_gate_average(local_time, step, duties_b, leg_skew)
    voltage = case['bus_nominal_V'] * (high_a - high_b)
    frequencies = np.fft.rfftfreq(count, step)
    current = np.fft.irfft(np.fft.rfft(voltage) / (resistance + 2j * np.pi * frequencies * inductance), n=count)
    if np.max(np.abs(current)) > Fixed.driver_peak_current:
        raise ValueError('Diagnostic exceeds the configured peak-current bound')
    return dict(time=time, current=current, low_a=1-high_a, low_b=1-high_b,
                frequency=frequency, voltage=voltage, command=command)


def perturbations():
    cases = [dict(name='nominal', shunt_error=0., leg_skew_s=0., sample_skew_s=0.)]
    cases.extend(dict(name=f'opposed_shunts_{error:g}', shunt_error=error, leg_skew_s=0., sample_skew_s=0.)
                 for error in Fixed.electronics_acquisition_shunt_mismatch)
    for sign in (-1, 1):
        cases.append(dict(name=f'leg_skew_{sign:+d}', shunt_error=0.,
                          leg_skew_s=sign*Fixed.electronics_acquisition_leg_skew, sample_skew_s=0.))
        cases.extend(dict(name=f'sample_skew_{sign*skew:g}', shunt_error=0., leg_skew_s=0., sample_skew_s=sign*skew)
                     for skew in Fixed.electronics_acquisition_sample_skews)
        cases.append(dict(name=f'combined_{sign:+d}', shunt_error=sign*max(Fixed.electronics_acquisition_shunt_mismatch),
                          leg_skew_s=sign*Fixed.electronics_acquisition_leg_skew,
                          sample_skew_s=sign*max(Fixed.electronics_acquisition_sample_skews)))
    return cases


def screen():
    case = next(row for row in power_stage.topology_cases()
                if row['topology']=='unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    cutoff, order = min(Fixed.electronics_acquisition_cutoffs), min(Fixed.electronics_acquisition_orders)
    gain = next(row[1] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')
    sensitivity = gain * Fixed.electronics_readout_trial_shunt / 2
    rows, saved = [], {}
    for label, changing, polarity in (('steady_positive', False, 1), ('steady_negative', False, -1),
                                     ('changing_current_and_reversal', True, 1)):
        for perturbation in perturbations():
            previous = {}
            for refinement in (1, 2):
                wave = bridge_waveforms(case, changing, polarity, perturbation['leg_skew_s'], refinement)
                current, frequency = wave['current'], wave['frequency']
                modulation = ((1+perturbation['shunt_error'])*wave['low_a'] +
                              (1-perturbation['shunt_error'])*wave['low_b'])
                signal = acquisition.filter_periodic(acquisition.filter_periodic(current * modulation,
                    Fixed.electronics_ina181_bandwidth, 1, frequency), cutoff, order, frequency)
                reference = acquisition.filter_periodic(acquisition.filter_periodic(current,
                    Fixed.electronics_ina181_bandwidth, 1, frequency), cutoff, order, frequency)
                for samples in Fixed.electronics_acquisition_samples_per_carrier:
                    if samples == 1:
                        continue
                    reported = causal_burst_average(signal, samples, frequency, perturbation['sample_skew_s'])
                    delayed_reference = causal_burst_average(reference, samples, frequency)
                    error = reported - delayed_reference
                    if refinement == 1:
                        previous[samples] = error
                        continue
                    coarse_time = (np.arange(len(error)//2)+.5)*2/(len(error)*frequency)
                    fine_aligned = np.interp(coarse_time, wave['time'], error)
                    difference = float(np.max(np.abs(fine_aligned - previous[samples])))
                    bias = float(error.mean())
                    rows.append(dict(trajectory=label, perturbation=perturbation, samples_per_carrier=samples,
                        peak_current_A=float(np.max(np.abs(current))), peak_command_V=float(np.max(np.abs(wave['command']))),
                        mean_measurement_bias_A=bias, peak_distortion_excluding_matched_filter_delay_A=float(np.max(np.abs(error))),
                        RMS_distortion_excluding_matched_filter_delay_A=float(np.sqrt(np.mean(error**2))),
                        peak_distortion_after_own_mean_removed_A=float(np.max(np.abs(error-bias))),
                        peak_total_error_against_instantaneous_current_A=float(np.max(np.abs(reported-current))),
                        numerical_refinement_difference_A=difference,
                        numerical_refinement_pass=difference<=Fixed.electronics_spice_current_tolerance,
                        quantizers=[dict(bits=bits, maximum_quantization_shift_A=float(np.max(np.abs(
                            causal_burst_average(signal,samples,frequency,perturbation['sample_skew_s'],
                                lambda values: acquisition.current_quantizer(values,sensitivity,bits))-reported))))
                            for bits in Fixed.electronics_acquisition_adc_bits]))
                    if samples == 2 and perturbation['name'] in ('nominal','combined_+1'):
                        key=f"{label}_{perturbation['name']}"
                        saved[key]=np.column_stack((wave['time'],current,reported,delayed_reference))
    return dict(status='ideal_bridge_acquisition_sensitivity_not_vendor_or_hardware_qualification',
        filter_cutoff_Hz=cutoff, filter_order=order, assumed_amplifier_single_pole_Hz=Fixed.electronics_ina181_bandwidth,
        coil_resistance_ohm=case['coil_R_ohm'], coil_inductance_H=case['minimum_coil_L_H'],
        current_motion_frequency_Hz=Fixed.current_loop_bandwidth,
        numerical_limit_A=Fixed.electronics_spice_current_tolerance, trials=rows, hardware_qualified=False,
        limitations=[
            'Ideal finite-edge complementary voltage-source bridge and periodic linear RL solution. No transistor/diode, deadtime, bypass routing, back EMF, mutual coupling, parasitics or thermal effects are modeled.',
            'Changing current uses sampled feed-forward voltage from the existing coil R/L at the current-loop bandwidth. It is an excitation for diagnosis, not a closed-loop current-controller test.',
            'The amplifier is a typical-bandwidth single-pole approximation, not the vendor model. These results can expose a mechanism but cannot qualify INA181 transient accuracy or calibrate away its recorded residual.',
            'Opposing shunt errors and timing offsets are exploratory values, not selected parts or guaranteed production bounds. Combined cases are examples, not exhaustive corner qualification.',
            'Matched filtering and averaging of actual coil current provide the reference for distortion. The separate instantaneous-current comparison includes ordinary delay and must not be called sensor distortion.',
            'All sample phases on the grids are evaluated. No phase is optimized independently for each test. Numerical refinement is separate from physical accuracy.',
            'The ideal quantizer is evaluated on the waveform at displaced sample times before averaging. No claim of extra ENOB, ADC settling, stochastic noise or real converter behavior is made.',
            'Mean removal is diagnostic per trial, not a realizable automatic calibration across changing commands.']), saved


def export(output):
    if output.exists():
        raise ValueError('Preserve existing sensitivity evidence')
    result, traces=screen()
    result['source_sha256']=source_fingerprints(__file__,model.__file__,acquisition.__file__,power_stage.__file__,
        power_stage.design.__file__,power_stage.prototype_coils.__file__,power_stage.qualification.__file__)
    output.mkdir(parents=True)
    with ZipFile(output/'sources.zip','x',compression=ZIP_DEFLATED) as archive:
        for filename in result['source_sha256']:
            archive.write(filename,filename)
    np.savez_compressed(output/'traces.npz',**traces)
    (output/'summary.json').write_text(json.dumps(result,indent=Fixed.report_precision)+'\n')
    return result


def vendor_ripple_netlist(case, leg_skew, refinement):
    wave = bridge_waveforms(case, False, 1, leg_skew, refinement)
    period = 1 / Fixed.driver_pwm_frequency
    count = len(wave['current']) * Fixed.electronics_spice_cycles
    times = np.linspace(0., Fixed.electronics_spice_cycles*period, count+1)
    currents = np.interp(times % period, wave['time'], wave['current'], period=period)
    source = f'Vcurrent imposed_current 0 DC {Fixed.driver_channel_current:.17g} PWL(\n' + '\n'.join(
        f'+ {time:.17g} {current:.17g}' for time,current in zip(times,currents)) + '\n+ )'
    duty = power_stage.unipolar_leg_duties(case['coil_R_ohm']*Fixed.driver_channel_current,
                                         case['bus_nominal_V'])[0]
    original = readout.low_side_test_netlist(duty,Fixed.driver_channel_current,0.,refinement,return_pair=True)
    lines = []
    for line in original.splitlines():
        if line.startswith('Vcurrent '):
            line = source
        elif line.startswith('VgateB '):
            start,width = readout.return_leg_blank_interval(1-duty,Fixed.driver_channel_current,0.)
            line = (f'VgateB gateB 0 PULSE(1 0 {start+leg_skew:.17g} '
                    f'{Fixed.driver_switching_time:.17g} {Fixed.driver_switching_time:.17g} {width:.17g} {period:.17g})')
        lines.append(line)
    return '\n'.join(lines)+'\n'


def run_vendor_ripple(output, vendor_model, executable):
    content, executable, fingerprints = readout.prepare_vendor_evidence(output,vendor_model,executable,
        b'.SUBCKT INA181A1 INP INN VCC GND REF OUT')
    extras = source_fingerprints(__file__,acquisition.__file__)
    with ZipFile(output/'sources.zip','a',compression=ZIP_DEFLATED) as archive:
        for filename in extras.keys()-fingerprints.keys():
            archive.write(filename,filename)
    fingerprints.update(extras)
    (output/'ina181a1.lib').write_bytes(content)
    case = next(row for row in power_stage.topology_cases()
                if row['topology']=='unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    period = 1 / Fixed.driver_pwm_frequency
    cutoff,order = min(Fixed.electronics_acquisition_cutoffs),min(Fixed.electronics_acquisition_orders)
    grid = Fixed.electronics_low_side_spice_steps_per_cycle * Fixed.electronics_acquisition_grid_refinement
    sensitivity = next(row[1] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1') * Fixed.electronics_readout_trial_shunt/2
    rows = []
    for leg_skew in (0.,Fixed.electronics_acquisition_leg_skew):
        checks,filtered = [],[]
        for refinement in (1,2):
            folder = output/f'case_{len(rows)}_refinement_{refinement}'
            netlist = vendor_ripple_netlist(case,leg_skew,refinement)
            dc,trace = readout.execute_vendor_case(folder,executable,netlist,'ina181a1.lib',
                Fixed.electronics_readout_spice_compatibility)
            time = np.linspace((Fixed.electronics_spice_cycles-1)*period,Fixed.electronics_spice_cycles*period,grid+1)
            sampled = np.interp(time,trace[:,0],trace[:,2])
            previous = np.interp(time-period,trace[:,0],trace[:,2])
            mean = float(np.trapezoid(sampled,time)/period)
            reference = float((np.interp(Fixed.driver_channel_current/2,dc[:,0],dc[:,1])-
                Fixed.electronics_current_reference)/sensitivity)
            checks.append(dict(refinement=refinement, carrier_mean_A=mean,
                mean_error_after_ideal_dc_correction_A=mean-reference,
                previous_cycle_mean_difference_A=mean-float(np.trapezoid(previous,time)/period)))
            filtered.append(acquisition.filter_periodic(sampled[:-1]-(reference-Fixed.driver_channel_current),
                cutoff,order,Fixed.driver_pwm_frequency))
        mean_difference=abs(checks[1]['mean_error_after_ideal_dc_correction_A']-
                            checks[0]['mean_error_after_ideal_dc_correction_A'])
        rows.append(dict(relative_leg_delay_s=leg_skew, checks=checks,
            carrier_mean_refinement_difference_A=mean_difference,
            carrier_mean_refinement_pass=mean_difference<=Fixed.electronics_spice_current_tolerance,
            cycle_mean_settled=abs(checks[-1]['previous_cycle_mean_difference_A'])<=Fixed.electronics_spice_current_tolerance,
            sample_refinement=[dict(samples_per_carrier=samples,
                max_refinement_difference_A=float(np.max(np.abs(acquisition.phase_sample_means(filtered[0],samples)-
                    acquisition.phase_sample_means(filtered[1],samples)))))
                for samples in Fixed.electronics_acquisition_samples_per_carrier if samples>1]))
    report=dict(status='vendor_amplifier_imposed_RL_ripple_and_leg_delay_diagnostic_not_complete_bridge',
        source_sha256=fingerprints,vendor_sha256=hashlib.sha256(content).hexdigest(),
        vendor_url='https://www.ti.com/lit/zip/SBOMBB7', trials=rows,
        measured_incremental_mean_bias_A=rows[1]['checks'][-1]['mean_error_after_ideal_dc_correction_A']-
            rows[0]['checks'][-1]['mean_error_after_ideal_dc_correction_A'],hardware_qualified=False,
        limitations=[
            'Unchanged vendor amplifier model with imposed current from the ideal periodic RL bridge. Shunt terminal voltages are controlled sources, not physical PGND current paths or a transistor model.',
            'The imposed current and the delayed leg use the same voltage-waveform timing. Input waveform density and SPICE step are refined together.',
            'The leg delay is exploratory, with zero deadtime and no shunt mismatch. It is not a DRV8962 guaranteed propagation tolerance.',
            'Ideal DC correction uses a separate static curve; real calibration, component temperature and noise are not established.',
            'The output filter is applied in periodic postprocessing, not as an ADC/RC load on the amplifier. Current-changing waveform and controller stability qualification remain open.'])
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--vendor-model',type=Path)
    parser.add_argument('--ngspice',default='ngspice')
    args=parser.parse_args()
    result=run_vendor_ripple(args.output,args.vendor_model,args.ngspice) if args.vendor_model else export(args.output)
    print(json.dumps(dict(status=result['status'],trials=len(result['trials']))))

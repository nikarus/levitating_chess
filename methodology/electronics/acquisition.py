import argparse
import hashlib
import json
import math
import subprocess
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np
from scipy.linalg import expm, solve_continuous_lyapunov

import model
from model import Fixed, Inputs
from methodology import electrical, source_fingerprints
from methodology.electronics import readout


def rc_noise_covariance(cutoff, order, lag):
    if cutoff <= 0 or order < 1 or int(order) != order or np.any(np.asarray(lag) < 0):
        raise ValueError('Positive cutoff, positive integer pole count and nonnegative lags required')
    time_constant = 1 / (2 * np.pi * cutoff)
    normalized_lag = np.asarray(lag) / time_constant
    polynomial = sum(math.comb(order - 1, k) * normalized_lag ** (order - 1 - k)
                     * math.factorial(order + k - 1) / 2 ** (order + k)
                     for k in range(order))
    return np.exp(-normalized_lag) * polynomial / (
        2 * time_constant * math.factorial(order - 1) ** 2)


def acquisition_response(cutoff, order, sample_rate, sample_count, frequency):
    if sample_rate <= 0 or sample_count < 1 or int(sample_count) != sample_count:
        raise ValueError('Positive sample rate and positive integer sample count required')
    return weighted_acquisition_response(cutoff, order, sample_rate, np.ones(sample_count) / sample_count, frequency)


def weighted_acquisition_response(cutoff, order, sample_rate, weights, frequency):
    weights = np.asarray(weights)
    if sample_rate <= 0 or weights.ndim != 1 or not len(weights) or not np.isfinite(weights).all():
        raise ValueError('Finite one-dimensional filter weights and positive sample rate required')
    lags = np.arange(len(weights)) / sample_rate
    covariance = rc_noise_covariance(cutoff, order, lags)
    weight_covariance = np.correlate(weights, weights, mode='full')[len(weights) - 1:]
    enbw = weight_covariance[0] * covariance[0] + 2 * np.dot(weight_covariance[1:], covariance[1:])
    transfer = (1 + 1j * frequency / cutoff) ** -order * np.dot(
        weights, np.exp(-2j * np.pi * frequency * lags))
    return dict(white_noise_ENBW_Hz=float(enbw),
        low_frequency_group_delay_s=order / (2 * np.pi * cutoff) + float(np.dot(weights, lags) / weights.sum()),
        loop_bandwidth_magnitude=float(abs(transfer)),
        loop_bandwidth_phase_deg=float(np.angle(transfer, deg=True)),
        first_to_last_sample_s=float(lags[-1]))


def filter_periodic(samples, cutoff, order, carrier):
    frequencies = np.fft.rfftfreq(len(samples), 1 / (len(samples) * carrier))
    transfer = (1 + 1j * frequencies / cutoff) ** -order
    return np.fft.irfft(np.fft.rfft(samples) * transfer, n=len(samples))


def phase_sample_means(samples, samples_per_carrier):
    if len(samples) % samples_per_carrier:
        raise ValueError('Uniform phase grid must be divisible by samples per carrier')
    return samples.reshape(samples_per_carrier, -1).mean(axis=0)


def current_codes(samples, sensitivity, bits, span):
    voltage = Fixed.electronics_current_reference + np.asarray(samples) * sensitivity
    if not np.isfinite(voltage).all() or np.min(voltage) < 0 or np.max(voltage) >= span:
        raise ValueError('ADC input exceeds its screened full-supply range')
    return np.floor(voltage / (span / 2 ** bits)).astype(int)


def current_quantizer(samples, sensitivity, bits, span=Fixed.electronics_analog_supply):
    codes = current_codes(samples,sensitivity,bits,span)
    return ((codes + .5) * span / 2 ** bits - Fixed.electronics_current_reference) / sensitivity


def load_period(folder, grid_count):
    trace = np.loadtxt(folder / 'transient.dat', skiprows=1)
    readout.validate_vendor_transient(trace)
    period = 1 / Fixed.driver_pwm_frequency
    times = np.linspace((Fixed.electronics_spice_cycles - 1) * period,
                        Fixed.electronics_spice_cycles * period, grid_count, endpoint=False)
    return np.interp(times, trace[:, 0], trace[:, 2])


def simultaneous_adc_screen(candidates, evidence, current_noise_density):
    modulator_rate = Fixed.current_regulator_rate * Fixed.electronics_acquisition_adc_osr
    samples_per_carrier = modulator_rate / Fixed.driver_pwm_frequency
    if not samples_per_carrier.is_integer():
        raise ValueError('Periodic ADC comparison requires an integer modulator-to-carrier ratio')
    weights = np.ones(1)
    for _ in range(Fixed.electronics_acquisition_adc_sinc_order):
        weights = np.convolve(weights, np.ones(Fixed.electronics_acquisition_adc_osr) /
                              Fixed.electronics_acquisition_adc_osr)
    sensitivity = evidence['mean_signal_scale_V_per_A']
    price_quantity, price, url = Fixed.electronics_acquisition_adc_price
    channel_count = evidence['amplifier_cost']['area_equivalent_channels']
    frame_bits = Fixed.electronics_acquisition_adc_resolution * Fixed.electronics_acquisition_adc_frame_words
    frame_time = frame_bits / Fixed.electronics_acquisition_adc_max_sclk
    amplifier_price = evidence['amplifier_cost']['price_scenarios'][0]['unit_USD']
    variants = []
    for candidate in candidates:
        if candidate['samples_per_carrier'] != samples_per_carrier:
            continue
        response = weighted_acquisition_response(candidate['cutoff_per_pole_Hz'], candidate['poles'],
            modulator_rate, weights, Fixed.current_loop_bandwidth)
        amplifier_noise = current_noise_density * np.sqrt(response['white_noise_ENBW_Hz'])
        adc_noise = Fixed.electronics_acquisition_adc_noise / sensitivity
        remaining_variance = Fixed.current_feedback_noise ** 2 - amplifier_noise ** 2 - adc_noise ** 2
        variants.append(dict(cutoff_per_pole_Hz=candidate['cutoff_per_pole_Hz'], poles=candidate['poles'],
            **response, readout_delay_lower_estimate_s=response['low_frequency_group_delay_s'] + frame_time,
            phase_including_full_frame_at_loop_bandwidth_deg=response['loop_bandwidth_phase_deg'] -
                360 * Fixed.current_loop_bandwidth * frame_time,
            amplifier_typical_white_noise_rms_A=float(amplifier_noise),
            ADC_typical_noise_reference_A=adc_noise,
            combined_typical_noise_estimate_A=float(np.hypot(amplifier_noise, adc_noise)),
            remaining_independent_noise_estimate_A=float(np.sqrt(max(0., remaining_variance))),
            max_differential_ADC_input_V=max(row['filtered_output_deviation_peak_V'] for row in candidate['trials']),
            all_recorded_inputs_inside_differential_range=all(
                row['filtered_output_deviation_peak_V'] < Fixed.electronics_acquisition_adc_reference
                for row in candidate['trials']),
            acquisition_numerical_refinement_pass=candidate['acquisition_numerical_refinement_pass'],
            max_phase_residue_from_carrier_mean_A=max(
                row['max_phase_residue_from_carrier_mean_A'] for row in candidate['trials']),
            max_error_after_ideal_dc_correction_A=max(
                row['max_error_after_ideal_dc_correction_A'] for row in candidate['trials'])))
    if not variants:
        raise ValueError('No recorded sampling candidate matches the ADC modulator schedule')
    return dict(part=Fixed.electronics_acquisition_adc,
        datasheet='https://www.ti.com/lit/ds/symlink/ads131m08.pdf',
        channels=Fixed.electronics_acquisition_adc_channels, output_rate_Hz=Fixed.current_regulator_rate,
        modulator_rate_Hz=modulator_rate,
        proposed_external_clock_Hz=modulator_rate * Fixed.electronics_acquisition_adc_clock_divisor,
        noise_table_external_clock_Hz=Fixed.electronics_acquisition_adc_nominal_clock,
        nominal_current_code_quantum_A=2 * Fixed.electronics_acquisition_adc_reference /
            2 ** Fixed.electronics_acquisition_adc_resolution / sensitivity,
        proposed_zero_bias_V=Fixed.electronics_acquisition_adc_supply / 2,
        startup_resynchronization_settling_s=Fixed.electronics_acquisition_adc_startup_clocks /
            (modulator_rate * Fixed.electronics_acquisition_adc_clock_divisor),
        default_frame_bits=frame_bits, full_frame_transfer_lower_bound_s=frame_time,
        continuous_serial_payload_per_chip_bits_per_s=frame_bits * Fixed.current_regulator_rate,
        board_aggregate_raw_payload_bits_per_s=channel_count / Fixed.electronics_acquisition_adc_channels *
            frame_bits * Fixed.current_regulator_rate,
        price_observation=dict(date='2026-10-20', quantity=price_quantity, chip_USD=price, source=url),
        ADC_only_USD_per_coil=price / Fixed.electronics_acquisition_adc_channels,
        amplifier_and_ADC_only_USD_per_coil=amplifier_price + price / Fixed.electronics_acquisition_adc_channels,
        amplifier_and_ADC_only_USD_per_board=channel_count * (
            amplifier_price + price / Fixed.electronics_acquisition_adc_channels),
        two_amplifiers_two_simultaneous_ADC_inputs_only_USD_per_coil=2 * (
            amplifier_price + price / Fixed.electronics_acquisition_adc_channels),
        variants=variants,
        limitations=[
            'This is a component comparison, not a selected implementation or complete BOM. Shunts, filters/buffers, reference-bias distribution, decoupling, clocks, supplies, local processors, interconnect, PCB and assembly remain unpriced.',
            'The two-amplifier cost uses two dedicated simultaneous converter inputs. A switched-input alternative has different settling and switching costs and is not ruled out.',
            'The proposed ADC clock is within the published high-resolution-mode range but differs from the noise-table clock. The listed typical ADC noise is a reference estimate, not guaranteed performance.',
            'The ADC requires a lower zero bias or level translation relative to the executed INA181 traces. Here the signal is only recentered algebraically; this does not verify the new amplifier operating point or ADC loading.',
            'At this synchronized clock, the sinc3 filter averages an integer number of carrier periods. Its periodic result equals the uniform modulator-phase mean; this does not validate changing-current or clock-mismatch behavior.',
            'Only the published steady-state digital filter is modeled. Modulator analog response, clock jitter, additional pipeline delay, startup fast-filter samples and overload recovery are not modeled.',
            'SPI transfer time assumes one dedicated bus at its maximum clock with no CS/setup overhead. Board-wide scheduling, fan-out and control deadlines are unqualified.',
            'Internal reference noise is already represented in the cited ADC noise measurement; the external amplifier-bias reference and its correlation with ADC input subtraction remain separate.',
            'All recorded inputs fit only in the stated periodic cases; startup, faults and unfiltered transients still require range/protection checks.'])


def screen(recording):
    evidence = json.loads((recording / 'summary.json').read_text())
    if evidence['status'] != 'one_amplifier_two_shunt_mean_acquisition_screen_not_qualified':
        raise ValueError('Expected return-pair vendor evidence')
    sensitivity = evidence['mean_signal_scale_V_per_A']
    noise_density = next(row[2] for row in Fixed.electronics_readout_specs if row[0] == 'INA181A1')
    current_noise_density = noise_density / evidence['shunt_ohm']
    cycles_per_update = Fixed.driver_pwm_frequency / Fixed.current_regulator_rate
    if not cycles_per_update.is_integer():
        raise ValueError('This screen requires an integer carrier-to-regulator ratio')
    grid = Fixed.electronics_low_side_spice_steps_per_cycle * Fixed.electronics_acquisition_grid_refinement
    recordings = []
    for index, trial in enumerate(evidence['trials']):
        recordings.append([load_period(recording / f'case_{index}_refinement_{refinement}', grid * density)
                           for refinement, density in ((1, 1), (2, 1), (2, 2))])
    candidates = []
    for cutoff in Fixed.electronics_acquisition_cutoffs:
        for order in Fixed.electronics_acquisition_orders:
            filtered = [[filter_periodic(trace, cutoff, order, Fixed.driver_pwm_frequency)
                         for trace in traces] for traces in recordings]
            for samples_per_carrier in Fixed.electronics_acquisition_samples_per_carrier:
                sample_rate = samples_per_carrier * Fixed.driver_pwm_frequency
                timings = []
                for periods in sorted({1, int(cycles_per_update)}):
                    count = samples_per_carrier * periods
                    response = acquisition_response(cutoff, order, sample_rate, count, Fixed.current_loop_bandwidth)
                    noise = current_noise_density * np.sqrt(response['white_noise_ENBW_Hz'])
                    timings.append(dict(averaged_periods=periods, samples_per_update=count,
                        average_ADC_conversions_per_channel_per_s=count * Fixed.current_regulator_rate,
                        amplifier_typical_white_noise_rms_A=float(noise),
                        remaining_independent_noise_rms_A=float(np.sqrt(max(
                            0., Fixed.current_feedback_noise ** 2 - noise ** 2))),
                        amplifier_alone_exceeds_noise_budget=bool(noise > Fixed.current_feedback_noise),
                        **response))
                rows = []
                for trial, traces in zip(evidence['trials'], filtered):
                    sampled = [phase_sample_means(trace, samples_per_carrier) for trace in traces]
                    refined = sampled[-1]
                    dc_reference = trial['checks'][-1]['carrier_mean_A'] - trial['checks'][-1][
                        'mean_error_after_ideal_dc_correction_A']
                    rows.append(dict(command_V=trial['command_V'], current_A=trial['current_A'],
                        assumed_deadtime_s=trial['assumed_deadtime_s'],
                        phase_grid_count=len(refined),
                        filtered_output_deviation_peak_V=float(np.max(np.abs(traces[-1])) * sensitivity),
                        max_phase_residue_from_carrier_mean_A=float(np.max(np.abs(refined - traces[-1].mean()))),
                        max_error_after_ideal_dc_correction_A=float(np.max(np.abs(refined - dc_reference))),
                        vendor_refinement_max_difference_A=float(np.max(np.abs(sampled[1] - sampled[0]))),
                        resampling_refinement_max_difference_A=float(np.max(np.abs(refined[::2] - sampled[1]))),
                        quantizers=[dict(bits=bits, native_quantum_A=Fixed.electronics_analog_supply / 2 ** bits / sensitivity,
                            max_quantization_shift_A=float(np.max(np.abs(phase_sample_means(
                                current_quantizer(traces[-1], sensitivity, bits), samples_per_carrier) - refined))))
                            for bits in Fixed.electronics_acquisition_adc_bits]))
                candidates.append(dict(cutoff_per_pole_Hz=cutoff, poles=order,
                    samples_per_carrier=samples_per_carrier, ADC_within_burst_sample_rate_Hz=sample_rate,
                    timing_and_typical_noise=timings, trials=rows,
                    acquisition_numerical_refinement_pass=all(
                        max(row['vendor_refinement_max_difference_A'], row['resampling_refinement_max_difference_A'])
                        <= Fixed.electronics_spice_current_tolerance for row in rows)))
    cost = evidence['amplifier_cost']
    price = cost['price_scenarios'][0]['unit_USD']
    return dict(status='periodic_recorded_waveform_acquisition_screen_not_hardware_qualification',
        sensitivity_V_per_A=sensitivity,
        required_current_quantum_A=electrical.current_quantum(),
        current_noise_budget_rms_A=Fixed.current_feedback_noise,
        numerical_discrepancy_limit_A=Fixed.electronics_spice_current_tolerance,
        loop_bandwidth_Hz=Fixed.current_loop_bandwidth,
        candidates=candidates,
        simultaneous_ADC_comparison=simultaneous_adc_screen(candidates, evidence, current_noise_density),
        cost_boundary=dict(area_equivalent_channels=cost['area_equivalent_channels'],
            eliminated_amplifier_USD_per_coil=price,
            eliminated_amplifier_USD_per_board=price * cost['area_equivalent_channels'],
            shared_incremental_cost_USD_per_channel_break_even=price),
        hardware_qualified=False,
        limitations=[
            'Ideal unity-gain buffered RC cascades applied to the last recorded PWM cycle in periodic steady state. No physical RC loading, amplifier feedback, buffer noise, component tolerance or startup is modeled.',
            'Recorded inputs are constant imposed coil currents. Carrier means and delayed filtered measurements are not instantaneous current for regulation or Hall subtraction.',
            'Numerical comparisons cover the filtered acquisition only; the retained raw-waveform refinement failure is not repaired.',
            'All grid phases are screened, not all continuous phases; doubled grids bound observed resampling sensitivity only.',
            'Noise uses a flat typical INA181 input density through the ideal RC filter and exact sampled-average autocovariance. This includes correlation and folding of that white noise, not flicker, reference, ADC, shunt, EMI or model uncertainty.',
            'Repeated identical carrier samples do not reduce deterministic quantization or switching bias; multi-period averaging affects the noise and delay calculation only.',
            'Complementary constant-current pulses are nearly antisymmetric about their mean a half-cycle apart. Even sample counts exploit this symmetry; unequal shunts, edges and changing currents can remove it.',
            'Burst sample spacing and average conversions per update are distinct. Sharing a SAR requires a realizable channel schedule and its additional skew/age, not only enough average throughput.',
            'Quantizers are ideal full-supply bins decoded at their centers. Finer averaged code increments are not claimed as extra effective bits.',
            'Signal transfer estimates assume small-signal baseband through the filter and sampler; time-varying bridge modulation and amplifier dynamics require separate verification.',
            'Group delay excludes conversion, communication, processing, actuation and the amplifier response. No loop phase-margin acceptance criterion has been invented.',
            'The cost boundary is the maximum extra acquisition expense versus an otherwise equivalent two-amplifier chain, not a complete BOM or a qualified two-amplifier alternative.'])


def sar_filter_pole():
    order=Fixed.electronics_acquisition_sar_filter_order
    return Fixed.electronics_acquisition_sar[-1]/np.sqrt(2**(1/order)-1)


def sensing_filter():
    resistance=Fixed.electronics_acquisition_filter_resistance
    load=Fixed.electronics_acquisition_sar_input_resistance
    gain=load/(load+resistance)
    sensitivity=next(row[1] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')*Fixed.electronics_readout_trial_shunt/2
    poles=[Fixed.electronics_ina181_bandwidth,(1/resistance+1/load)/(2*np.pi*Fixed.electronics_acquisition_filter_capacitance)]
    poles.extend([sar_filter_pole()]*Fixed.electronics_acquisition_sar_filter_order)
    return dict(poles_Hz=poles,load_gain=gain,
        load_offset_A=(1-gain)*(Fixed.electronics_acquisition_sar_input_bias-Fixed.electronics_current_reference)/sensitivity,
        sensitivity_V_per_A=sensitivity)


def sar_schedule():
    _,_,channels,rate,_,_=Fixed.electronics_acquisition_sar
    period=1/Fixed.driver_pwm_frequency
    sample_count=Fixed.electronics_regulation_samples_per_carrier
    if sample_count!=2 or Fixed.electronics_acquisition_sar_serial_clock>Fixed.electronics_acquisition_sar_max_sclk:
        raise ValueError("Unsupported burst size or excessive ADC serial clock")
    spacing=period/sample_count
    serial=Fixed.electronics_acquisition_sar_frame_bits/Fixed.electronics_acquisition_sar_serial_clock+Fixed.electronics_acquisition_sar_frame_guard
    frame=max(1/rate,serial)
    if frame>spacing or channels*sample_count*Fixed.current_regulator_rate>1/frame:
        raise ValueError('The shared ADC cannot service the proposed two-sample bursts')
    computed=spacing+frame+Fixed.electronics_regulation_compute_time
    latch=np.ceil(computed/period)*period
    return dict(samples_per_carrier=sample_count,sample_spacing_s=spacing,frame_s=frame,
        result_ready_s=spacing+frame,computation_ready_s=computed,pwm_latch_s=float(latch),
        actuation_s=float(latch+Fixed.electronics_drv8962_input_delay),
        serial_utilization=channels*sample_count*Fixed.current_regulator_rate*serial,
        computation_utilization=channels*Fixed.current_regulator_rate*Fixed.electronics_regulation_compute_time,
        extra_age_if_all_channels_update_together_s=(channels-1)*period)


def cascade_noise_covariance(poles, lag):
    poles=np.asarray(poles,dtype=float)
    if poles.ndim!=1 or not len(poles) or np.any(poles<=0) or lag<0:
        raise ValueError('Positive filter poles and nonnegative lag required')
    scale=float(np.min(poles))
    rates=2*np.pi*poles/scale
    matrix=-np.diag(rates)+np.diag(rates[1:],-1)
    forcing=np.zeros_like(matrix)
    forcing[0,0]=-rates[0]**2/2
    stationary=solve_continuous_lyapunov(matrix,forcing)
    return float((expm(matrix*lag*scale)@stationary)[-1,-1]*scale)


def sar_noise_budget():
    sensor=sensing_filter()
    lag=sar_schedule()['sample_spacing_s']
    covariance=[cascade_noise_covariance(sensor['poles_Hz'],delay) for delay in (0.,lag)]
    bandwidth=sum(covariance)/2
    density=next(row[2] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')
    amplifier=density/(Fixed.electronics_readout_trial_shunt/2)*np.sqrt(bandwidth)
    sine_rms=Fixed.electronics_acquisition_sar[4]/(2*np.sqrt(2))*10**(Fixed.electronics_acquisition_sar_test_dbfs/20)
    rows=[]
    for specification,snr in zip(('typical','minimum_specified_SNR'),Fixed.electronics_acquisition_sar_snr,strict=True):
        converter=sine_rms*10**(-snr/20)/(sensor['sensitivity_V_per_A']*sensor['load_gain'])
        combined=float(np.hypot(amplifier,converter))
        remaining=Fixed.current_feedback_noise**2-combined**2
        rows.append(dict(specification=specification,SNR_dB=snr,converter_input_noise_rms_A=float(converter),
            combined_noise_rms_A=combined,within_existing_budget=remaining>=0,
            remaining_independent_noise_rms_A=float(np.sqrt(max(0.,remaining)))))
    return dict(status='candidate_current_acquisition_component_noise_budget',
        current_noise_budget_rms_A=Fixed.current_feedback_noise,
        amplifier_typical_white_noise_rms_A=float(amplifier),
        paired_sample_ENBW_Hz=float(bandwidth),amplifier_pair_correlation=covariance[1]/covariance[0],
        amplifier_regulator_period_correlation=cascade_noise_covariance(sensor['poles_Hz'],1/Fixed.current_regulator_rate)/covariance[0],
        converter_samples_averaging_credit=False,scenarios=rows,hardware_qualified=False,
        sources=['https://www.ti.com/lit/ds/symlink/ina181.pdf','https://www.ti.com/lit/ds/symlink/ads8698.pdf'],
        limitations=[
            'Amplifier spot noise is typical and treated as white through the same approximate amplifier/RC/ADC poles used by regulation. Flicker and maximum amplifier noise remain unbounded.',
            'Stationary covariance includes correlation between paired samples. No independent-sample averaging benefit is assumed for the converter.',
            'Converter noise is inferred from the selected range SNR at its stated sine amplitude. Its quantization and internal-reference contributions are already included; do not add them twice.',
            'This is an allocation screen, not a stochastic switched-circuit simulation. The minimum-SNR case still uses typical amplifier noise and filter bandwidth.',
            'External bias-reference, shunt/filter thermal noise, supply coupling, interference and clock jitter must fit the reported remaining allowance. Deterministic gain, offset and switching distortion require a separate accuracy budget.'])


def accuracy_budget():
    sensor=sensing_filter()
    sensitivity=sensor['sensitivity_V_per_A']
    shunt=Fixed.electronics_readout_trial_shunt/2
    shunt_tolerance,shunt_drift=Fixed.electronics_readout_shunt_accuracy
    amplifier_offset,amplifier_offset_drift,amplifier_gain,amplifier_gain_drift=Fixed.electronics_ina181_accuracy
    adc_offset,adc_offset_drift,adc_gain,adc_gain_drift,adc_inl=Fixed.electronics_sar_accuracy
    bias_error,bias_drift=Fixed.electronics_bias_reference_accuracy
    span=Fixed.electronics_acquisition_sar[4]
    bias=Fixed.electronics_current_reference
    drift_gain=shunt_drift+amplifier_gain_drift+adc_gain_drift
    gain_headroom=Fixed.current_feedback_gain_bound-Fixed.electronics_calibration_gain_allocation
    if gain_headroom<=0:
        raise ValueError('Calibration allocation consumes the entire gain-error allowance')
    delta=gain_headroom/drift_gain
    terms=dict(amplifier_input=amplifier_offset_drift/shunt,
        adc_offset=adc_offset_drift*span/sensitivity,
        adc_gain_at_zero_bias=adc_gain_drift*bias/sensitivity,
        bias_reference=bias_drift*bias/sensitivity)
    offset_limit=Fixed.current_sense_offset_residual/Fixed.current_sense_resistance
    inl=adc_inl*span/2**Fixed.electronics_acquisition_sar[1]/sensitivity
    return dict(status='component_accuracy_allocation_not_calibration_or_thermal_qualification',
        uncalibrated_static_gain_bound_fraction=shunt_tolerance+amplifier_gain+adc_gain,
        uncalibrated_zero_error_bound_A=amplifier_offset/shunt+(adc_offset+(bias_error+adc_gain)*bias)/sensitivity,
        existing_gain_allowance_fraction=Fixed.current_feedback_gain_bound,
        calibration_gain_allocation_fraction=Fixed.electronics_calibration_gain_allocation,
        gain_drift_per_K=drift_gain,maximum_common_temperature_change_from_calibration_K=delta,
        offset_drift_terms_A_per_K=terms,offset_drift_at_gain_temperature_boundary_A=delta*sum(terms.values()),
        adc_endpoint_INL_bound_A=inl,existing_offset_allowance_A=offset_limit,
        offset_remaining_for_calibration_model_error_and_other_sources_A=offset_limit-inl-delta*sum(terms.values()),
        hardware_qualified=False,
        sources=['https://www.vishay.com/docs/30100/wsl.pdf','https://www.ti.com/lit/ds/symlink/ina181.pdf',
            'https://www.ti.com/lit/ds/symlink/ads8698.pdf','https://www.ti.com/lit/ds/symlink/ref50.pdf'],
        limitations=[
            'Absolute worst-case sums for the static mean-current gain model. This is a thermal/calibration design constraint, not proof that the component temperatures meet it.',
            'Each component temperature change is bounded by the reported common envelope; opposing shunt mismatch, switching bias, parasitic inductance and thermal EMF require separate waveform analysis.',
            'ADC system gain/offset specifications use the internal reference per the datasheet. Do not add its separate reference drift again; gain drift also shifts the biased zero-current code.',
            'Calibration gain accuracy is an allocated requirement, not an achieved measurement. Calibration-reference error, hysteresis, aging, supply/reference loading and model discrepancy must fit remaining allocations.',
            'Repeated zero calibration can remove offset change after supplies settle but cannot remove gain drift. Temperature compensation requires characterized coefficients and representative component temperature sensing.'])


def sar_reference():
    part,bits,channels,rate,span,bandwidth = Fixed.electronics_acquisition_sar
    quantity,price,url = Fixed.electronics_acquisition_sar_price
    cost = readout.adoption_cost_screen('INA181A1')
    coil_count = cost['area_equivalent_channels']
    amplifier_price = cost['price_scenarios'][0]['unit_USD']
    gain = next(row[1] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')
    sensitivity = gain*Fixed.electronics_readout_trial_shunt/2
    return dict(status='SAR_precision_throughput_and_catalog_cost_reference_not_circuit_qualification',
        part=part,datasheet='https://www.ti.com/lit/ds/symlink/ads8698.pdf',
        nominal_current_quantum_A=span/2**bits/sensitivity,
        required_current_quantum_A=electrical.current_quantum(),
        nominal_quantum_pass=span/2**bits/sensitivity<=electrical.current_quantum(),
        typical_analog_input_bandwidth_Hz=bandwidth,
        total_converter_throughput_Hz=rate,
        burst_options=[dict(samples_per_coil_per_update=count,
            within_burst_sample_spacing_s=1/(count*Fixed.driver_pwm_frequency),
            converter_minimum_period_s=1/rate,
            burst_spacing_possible=rate>=count*Fixed.driver_pwm_frequency,
            aggregate_required_conversions_per_s=channels*count*Fixed.current_regulator_rate,
            aggregate_rate_possible=rate>=channels*count*Fixed.current_regulator_rate)
            for count in Fixed.electronics_acquisition_samples_per_carrier if count>1],
        sequential_one_carrier_burst_per_channel_schedule=dict(
            duration_s=channels/Fixed.driver_pwm_frequency,
            regulator_period_s=1/Fixed.current_regulator_rate,
            oldest_channel_additional_age_if_all_updates_simultaneous_s=(channels-1)/Fixed.driver_pwm_frequency,
            needs_staggered_regulation_or_explicit_sample_age=True),
        production_volume=dict(boards=Inputs.production_boards,
            area_equivalent_amplifiers=coil_count*Inputs.production_boards,
            area_equivalent_ADC_chips=coil_count*Inputs.production_boards/channels),
        catalog_reference=dict(observation_date='2026-10-20',quantity=quantity,chip_USD=price,source=url,
            ADC_only_USD_per_coil=price/channels,
            amplifier_and_ADC_only_USD_per_board=coil_count*(amplifier_price+price/channels)),
        hardware_qualified=False,
        limitations=[
            'Nominal code spacing and aggregate throughput are necessary conditions, not noise, accuracy, channel-settling, SPI scheduling or control-stability verification.',
            'Two conversions per carrier fit the throughput ceiling; four do not. A staggered eight-channel schedule requires per-coil update phases or modeled extra sample age, unlike the existing simultaneous instantaneous-feedback interface.',
            'The internal analog filter contributes delay. Its small-signal bandwidth alone does not establish its group delay or transient response.',
            'Neither this converter nor oversampling removes bias caused by coil-ripple interaction with bridge-leg timing.',
            'The price is a published 100-piece tier for a technical comparison, not the production-volume price and not a replacement for the cheaper ADS131M08 cost reference. No RFQ has been sent.',
            'Area-equivalent quantities exclude final module rounding, spares and yield. A complete acquisition BOM still needs shunts, filtering/bias, clock/supply support, local control, interconnect, PCB and assembly.'])


def physical_screen(recording):
    evidence=json.loads((recording/'summary.json').read_text())
    loaded=evidence['status']=='physical_output_filter_vendor_amplifier_verification'
    if evidence['status'] not in ('state_space_bridge_and_vendor_amplifier_carrier_mean_verification',
                                  'physical_output_filter_vendor_amplifier_verification'):
        raise ValueError('Expected independently solved bridge and vendor amplifier evidence')
    if not all(row['numerical_mean_pass'] for row in evidence['trials']):
        raise ValueError('Carrier-mean verification must pass before acquisition screening')
    period=1/Fixed.driver_pwm_frequency
    grid=Fixed.electronics_low_side_spice_steps_per_cycle*Fixed.electronics_acquisition_grid_refinement
    recordings=[]
    for index,trial in enumerate(evidence['trials']):
        variants=[]
        for refinement,density in ((1,1),(2,1),(2,2)):
            trace=np.loadtxt(recording/f'case_{index}_refinement_{refinement}/transient.dat',skiprows=1)
            if not np.isfinite(trace).all() or trace[-1,0]-trace[0,0]<2*period:
                raise ValueError('Incomplete physical amplifier recording')
            stop=trace[-1,0]
            time=stop-period+np.arange(grid*density)*period/(grid*density)
            signal=np.interp(time,trace[:,0],trace[:,1])
            native_mean=readout.interval_means(trace,stop-period,stop)[0]
            if abs(signal.mean()-native_mean)>Fixed.electronics_spice_current_tolerance:
                raise ValueError('Uniform resampling failed the native integral check')
            variants.append(signal)
        recordings.append(variants)
    currents=np.array([trial['checks'][-1]['actual_mean_A'] for trial in evidence['trials']])
    low,high=int(np.argmin(currents)),int(np.argmax(currents))
    gain=next(row[1] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')
    sensitivity=gain*Fixed.electronics_readout_trial_shunt/2
    part,bits,channels,rate,span,bandwidth=Fixed.electronics_acquisition_sar
    adc_order=Fixed.electronics_acquisition_sar_filter_order
    adc_pole=sar_filter_pole()
    candidates=[]
    cutoffs=Fixed.electronics_acquisition_cutoffs
    orders=Fixed.electronics_acquisition_orders
    if loaded:
        circuit=evidence['external_filter']
        cutoffs=((1/circuit['resistance_ohm']+1/circuit['load_ohm'])/(2*np.pi*circuit['capacitance_F']),)
        orders=(1,)
    for cutoff in cutoffs:
        for order in orders:
            filtered=[[filter_periodic(signal if loaded else filter_periodic(signal,cutoff,order,Fixed.driver_pwm_frequency),
                adc_pole,adc_order,Fixed.driver_pwm_frequency) for signal in variants] for variants in recordings]
            for count in Fixed.electronics_acquisition_samples_per_carrier:
                if count*Fixed.driver_pwm_frequency>rate or channels*count*Fixed.current_regulator_rate>rate:
                    continue
                sampled=[[phase_sample_means(signal,count) for signal in variants] for variants in filtered]
                measured=np.array([variants[-1][0] for variants in sampled])
                slope=(measured[high]-measured[low])/(currents[high]-currents[low])
                intercept=measured[low]-slope*currents[low]
                response=acquisition_response(cutoff,order,count*Fixed.driver_pwm_frequency,count,Fixed.current_loop_bandwidth)
                transfer=(1+1j*Fixed.current_loop_bandwidth/adc_pole)**-adc_order
                rows=[]
                for index,(variants,samples) in enumerate(zip(filtered,sampled)):
                    quantized=phase_sample_means(current_quantizer(variants[-1],sensitivity,bits,span),count)
                    rows.append(dict(actual_mean_A=float(currents[index]),
                        uncalibrated_phase_max_error_A=float(np.max(np.abs(samples[-1]-currents[index]))),
                        calibrated_phase_max_error_A=float(np.max(np.abs((samples[-1]-intercept)/slope-currents[index]))),
                        calibrated_fixed_phase_error_A=float((samples[-1][0]-intercept)/slope-currents[index]),
                        quantized_calibrated_phase_max_error_A=float(np.max(np.abs((quantized-intercept)/slope-currents[index]))),
                        vendor_refinement_max_difference_A=float(np.max(np.abs(samples[1]-samples[0]))),
                        resampling_refinement_max_difference_A=float(np.max(np.abs(samples[-1][::2]-samples[1])))))
                candidates.append(dict(cutoff_per_pole_Hz=cutoff,poles=order,samples_per_carrier=count,
                    calibration=dict(gain=slope,offset_A=intercept,reference_current_A=currents[[low,high]].tolist(),
                        sample_phase_fraction=0.,method='One two-point fit shared by all trials and all screened phases; ideal known-current calibration references.'),
                    low_frequency_group_delay_without_conversion_s=response['low_frequency_group_delay_s']+adc_order/(2*np.pi*adc_pole),
                    loop_bandwidth_phase_without_conversion_deg=response['loop_bandwidth_phase_deg']+float(np.angle(transfer,deg=True)),
                    numerical_pass=all(max(row['vendor_refinement_max_difference_A'],row['resampling_refinement_max_difference_A'])<=Fixed.electronics_spice_current_tolerance for row in rows),
                    trials=rows))
    return dict(status='physical_bridge_recorded_SAR_acquisition_screen',
        converter=part,external_filter_load_simulated=loaded,nominal_code_A=span/2**bits/sensitivity,
        numerical_limit_A=Fixed.electronics_spice_current_tolerance,candidates=candidates,
        hardware_qualified=False,limitations=[
            ('Recorded physical bridge and unchanged vendor amplifier with a physical passive output filter and typical resistive ADC load; internal ADC input bias and impedance variation remain unmodeled.' if loaded else
             'Recorded physical bridge and unchanged vendor amplifier signals, with ideal external RC filtering. The filter has not yet loaded the amplifier output.'),
            'The ADC internal filter uses equal real poles matching its published typical order and -3 dB bandwidth. This is a screening approximation, not a vendor dynamic model or a guaranteed timing bound.',
            'The same two-point gain/offset correction is applied to every current and phase. Reference-current accuracy, calibration implementation, temperature drift and component tolerances remain unqualified.',
            'Quantization uses the selected nominal input span. ADC noise, reference error, INL/DNL, loading, communication and current-loop stability are not qualified.',
            'Only the saved periodic currents are tested; this does not qualify changing-current distortion or arbitrary switching patterns.'])


def export(recording, output, mode='recorded',pioasm=None):
    if output.exists():
        raise ValueError('Preserve existing acquisition evidence')
    from methodology.electronics import logic

    actions={'recorded':lambda:screen(recording),'physical':lambda:physical_screen(recording),
        'sar_reference':sar_reference,'noise':sar_noise_budget,'accuracy':accuracy_budget,
        'controller':logic.controller_schedule,'protocol':logic.sar_protocol_screen,'stream':logic.sar_stream_screen}
    result=actions[mode]()
    if pioasm is not None and mode!='protocol':
        raise ValueError('PIO assembly applies only to the serial protocol screen')
    result['source_sha256'] = source_fingerprints(__file__, logic.__file__, model.__file__, readout.__file__, electrical.__file__,
        readout.power_stage.__file__,readout.power_stage.design.__file__,readout.power_stage.prototype_coils.__file__,
        readout.power_stage.qualification.__file__,readout.coil_layouts.__file__)
    inputs = [] if mode not in ('recorded','physical') else [recording / 'summary.json', *sorted(recording.glob('case_*/transient.dat'))]
    result['recorded_input_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for filename in result['source_sha256']:
            archive.write(filename, filename)
    if mode=='protocol':
        program,timing=logic.sar_pio_program()
        source=output/'acquisition.pio'
        source.write_text(program)
        result['pio_timing']=timing
        if pioasm is not None:
            executable=Path(pioasm).resolve()
            logs=[]
            for output_format,suffix in (('json','json'),('c-sdk','pio.h')):
                destination=output/f'acquisition.{suffix}'
                process=subprocess.run([str(executable),'-o',output_format,str(source),str(destination)],
                    capture_output=True,text=True,check=True)
                logs.append(dict(format=output_format,stdout=process.stdout,stderr=process.stderr,
                    sha256=hashlib.sha256(destination.read_bytes()).hexdigest()))
            version=subprocess.run([str(executable),'--version'],capture_output=True,text=True,check=True)
            result['pio_assembler']=dict(version=version.stdout.strip(),executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),outputs=logs)
    (output / 'summary.json').write_text(json.dumps(result, indent=Fixed.report_precision) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--recording', type=Path)
    selection.add_argument('--sar-reference', action='store_true')
    selection.add_argument('--sar-noise', action='store_true')
    selection.add_argument('--accuracy-budget', action='store_true')
    selection.add_argument('--controller-schedule', action='store_true')
    selection.add_argument('--serial-protocol', action='store_true')
    selection.add_argument('--acquisition-stream', action='store_true')
    parser.add_argument('--physical-bridge', action='store_true')
    parser.add_argument('--pioasm',type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.physical_bridge and (args.sar_reference or args.sar_noise or args.accuracy_budget or args.controller_schedule or args.serial_protocol or args.acquisition_stream):
        parser.error("Physical-bridge screening requires recordings")
    modes={'sar_reference':args.sar_reference,'noise':args.sar_noise,'accuracy':args.accuracy_budget,
        'controller':args.controller_schedule,'protocol':args.serial_protocol,'stream':args.acquisition_stream,
        'physical':args.physical_bridge,'recorded':args.recording is not None and not args.physical_bridge}
    mode=next(name for name,selected in modes.items() if selected)
    result = export(args.recording, args.output,mode,args.pioasm)
    print(json.dumps(dict(status=result['status'])))

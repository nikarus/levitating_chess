import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np
from scipy.linalg import expm
from scipy.optimize import brentq

import model
from model import Inputs, Fixed
from methodology import coil_layouts, source_fingerprints
from methodology.electronics import current_loop, power_stage, acquisition_stress, readout, logic


class SharedMotorBus:
    def __init__(self,source_voltage,bulk_count):
        _,capacitance,tolerance,_,impedance,_=Fixed.electronics_shared_bulk_candidate
        if source_voltage<=0 or not isinstance(bulk_count,int) or bulk_count<=0:
            raise ValueError('A positive motor source and explicit bulk count are required')
        self.source_voltage=source_voltage
        self.capacitance=bulk_count*capacitance*(1-tolerance)
        self.capacitor_resistance=impedance/bulk_count
        self.feed_resistance=Fixed.electronics_shared_feed_resistance
        self.current_limit=Fixed.electronics_shared_source_current

    def operating_point(self,capacitor_voltage,load_current):
        source_current=np.clip((self.source_voltage-capacitor_voltage+self.capacitor_resistance*load_current)/
            (self.feed_resistance+self.capacitor_resistance),0.,self.current_limit)
        capacitor_current=source_current-load_current
        voltage=capacitor_voltage+self.capacitor_resistance*capacitor_current
        return dict(voltage_V=float(voltage),source_current_A=float(source_current),
            capacitor_current_A=float(capacitor_current),capacitor_rate_V_s=float(capacitor_current/self.capacitance),
            distribution_loss_W=float(self.feed_resistance*source_current**2),
            current_limit_loss_W=float(max(0.,(self.source_voltage-voltage-self.feed_resistance*source_current)*source_current)),
            capacitor_loss_W=float(self.capacitor_resistance*capacitor_current**2))

    def voltage_rate(self,capacitor_voltage,load_current,capacitor_rate,load_rate):
        source=self.operating_point(capacitor_voltage,load_current)['source_current_A']
        source_rate=(-capacitor_rate+self.capacitor_resistance*load_rate)/(self.feed_resistance+self.capacitor_resistance) if 0<source<self.current_limit else 0.
        return capacitor_rate+self.capacitor_resistance*(source_rate-load_rate)


def absorbed_energy_voltage(capacitance, initial_voltage, energy):
    if capacitance<=0 or initial_voltage<0 or energy<0:
        raise ValueError('Energy absorption requires positive capacitance and nonnegative voltage and energy')
    return float(np.sqrt(initial_voltage**2+2*energy/capacitance))


def required_absorption_capacitance(initial_voltage, maximum_voltage, energy):
    if energy<0 or initial_voltage<0 or maximum_voltage<=initial_voltage:
        raise ValueError('Energy absorption requires a positive voltage headroom')
    return 2*energy/(maximum_voltage**2-initial_voltage**2)


def disabled_return(channels, current, inductance, resistance, capacitance, voltage, diode_voltage=0., capacitor_resistance=0.):
    if not isinstance(channels,int) or channels<=0 or min(inductance,capacitance,voltage)<=0 or min(current,resistance,diode_voltage,capacitor_resistance)<0:
        raise ValueError('Disabled return requires identical nonnegative branch currents and positive L, C and bus voltage')
    matrix=np.array([[-(resistance+channels*capacitor_resistance)/inductance,-1/inductance],
        [channels/capacitance,0.]])
    initial=np.array([current,voltage+2*diode_voltage])
    def state(time):
        return expm(matrix*time)@initial
    if current==0:
        end=0.
    else:
        bracket=inductance*current/(voltage+2*diode_voltage)
        frequency_squared=channels/(inductance*capacitance)-(matrix[0,0]/2)**2
        if frequency_squared>0:
            bracket=min(bracket,np.pi/(2*np.sqrt(frequency_squared)))
        end=brentq(lambda time:state(time)[0],0.,bracket,xtol=np.spacing(bracket))
    def terminal(time):
        branch,shifted_voltage=state(time)
        return shifted_voltage-2*diode_voltage+channels*capacitor_resistance*branch
    def terminal_rate(time):
        rates=matrix@state(time)
        return rates[1]+channels*capacitor_resistance*rates[0]
    extrema=[0.,end]
    if end>0 and terminal_rate(0.)*terminal_rate(end)<0:
        extrema.append(brentq(terminal_rate,0.,end,xtol=np.spacing(end)))
    final_voltage=float(state(end)[1]-2*diode_voltage)
    initial_energy=channels*inductance*current**2/2
    stored=capacitance*(final_voltage**2-voltage**2)/2
    return dict(decay_time_s=end,final_capacitor_voltage_V=final_voltage,
        maximum_terminal_voltage_V=float(max(map(terminal,extrema))),
        initial_return_current_A=channels*current,initial_coil_energy_J=initial_energy,
        capacitor_energy_increase_J=stored,dissipated_energy_J=initial_energy-stored,
        lossless_voltage_bound_V=absorbed_energy_voltage(capacitance,voltage,initial_energy))


def regeneration_screen(parts):
    envelope=group_power_envelope(parts)
    nominal=current_loop.nominal_circuit()
    mpn,capacitance,tolerance,rating,impedance,_=Fixed.electronics_shared_bulk_candidate
    trials=[]
    for count in Fixed.electronics_shared_bulk_counts:
        effective=count*capacitance*(1-tolerance) if count else envelope['local_nominal_motor_capacitance_F']
        for initial_voltage in (nominal['bus_nominal'],nominal['bus_high']):
            for capacitor_resistance in ((0.,impedance/count) if count else (0.,)):
                result=disabled_return(envelope['channels'],Fixed.driver_peak_current,envelope['inductance_upper_H'],
                    nominal['coil_resistance']+Fixed.electronics_readout_trial_shunt/2+2*Fixed.electronics_bridge_diode[1],
                    effective,initial_voltage,Fixed.electronics_bridge_diode[0],capacitor_resistance)
                trials.append(dict(bulk_count=count,bulk_MPN=mpn if count else None,
                    assumed_capacitance_F=effective,initial_voltage_V=initial_voltage,
                    assumed_capacitor_resistance_ohm=capacitor_resistance,**result,
                    within_operating_voltage=result['maximum_terminal_voltage_V']<=nominal['bus_high']))
    return dict(trials=trials,bulk_voltage_rating_V=rating,
        status='piecewise_exact_disabled_inductive_return_screen_not_supply_qualification',
        limitations=[
            'Identical stationary branches begin at the physical peak and maximum screened inductance. Both current polarities return energy through two body diodes. The branch is disconnected at its first zero-current crossing, preventing unphysical reverse current.',
            'The matrix exponential solves the conducting RLC interval without time stepping. Coil/shunt/diode resistance and capacitor resistance dissipate energy; the independent lossless bound remains applicable.',
            'The source is disconnected and cannot absorb energy. Motion-induced voltage, externally injected energy, wiring inductance, source dynamics, clamp action and switching-node ringing are absent.',
            'Bulk cases use minimum nameplate capacitance and give no credit for local ceramics. The no-bulk case uses nominal local capacitance. Aging, temperature and frequency dependence still need qualification.',
            'The nonzero capacitor-resistance case uses the published 100 kHz/20 C impedance as an exploratory constant resistance, not a broadband or cold-temperature ESR bound. Neither capacitor pulse-current capability nor lifetime is qualified.',
            'A voltage pass for this isolated event does not qualify a complete supply. An initial bus at the operating ceiling leaves no regenerative headroom; source tolerance and an absorbing clamp remain necessary design decisions.'])


def group_bus_waveform(currents,voltages,grid,phase_fractions=None):
    currents,voltages=np.asarray(currents),np.asarray(voltages)
    if currents.shape!=voltages.shape or currents.ndim!=1 or not np.isfinite(currents).all() or not np.isfinite(voltages).all():
        raise ValueError('One finite held current and voltage are required per bridge')
    phases=np.zeros(len(currents)) if phase_fractions is None else np.asarray(phase_fractions)
    if phases.shape!=currents.shape:
        raise ValueError('One explicit carrier phase is required per bridge')
    period=1/Fixed.driver_pwm_frequency
    step=period/grid
    time=np.arange(grid)*step
    waveform=np.zeros(grid)
    nominal=current_loop.nominal_circuit()['bus_nominal']
    for current,voltage,phase in zip(currents,voltages,phases):
        if current==0:
            continue
        legs=power_stage.timer_leg_duties(float(voltage),nominal)
        for duty,polarity in zip(legs,(-1,1)):
            start,width=readout.return_leg_blank_interval(duty,polarity*current,Fixed.electronics_drv8962_deadtime)
            pulse_duty=(width+Fixed.driver_switching_time)/period
            shift=start-(1-pulse_duty)*period/2
            pulse=acquisition_stress.high_gate_average(time,step,pulse_duty,shift)
            if phase:
                pulse=acquisition_stress.periodic_delay(pulse,phase*period,Fixed.driver_pwm_frequency)
            waveform-=polarity*current*pulse
    return waveform


def capacitor_ripple(waveform,bulk_count,local_capacitance,source_conductance):
    if bulk_count<=0 or local_capacitance<0 or source_conductance<0:
        raise ValueError('Explicit positive bulk count and nonnegative passive branches required')
    mpn,capacitance,tolerance,_,impedance,rating=Fixed.electronics_shared_bulk_candidate
    frequency=np.fft.rfftfreq(len(waveform),1/(len(waveform)*Fixed.driver_pwm_frequency))[1:]
    bulk_admittance=1/(impedance/bulk_count+1/(2j*np.pi*frequency*bulk_count*capacitance*(1-tolerance)))
    total=bulk_admittance+2j*np.pi*frequency*local_capacitance+source_conductance
    spectrum=np.fft.rfft(waveform)
    bulk_spectrum=np.r_[0.,spectrum[1:]*bulk_admittance/total]
    ripple=np.fft.irfft(bulk_spectrum,n=len(waveform))
    rms=float(np.sqrt(np.mean(ripple**2)))
    return dict(MPN=mpn,total_bus_AC_RMS_A=float(np.std(waveform)),bulk_bank_RMS_A=rms,
        each_bulk_RMS_A=rms/bulk_count,each_bulk_rated_RMS_A=rating,
        bulk_bank_ESR_loss_W=rms**2*impedance/bulk_count,bulk_count=bulk_count,
        local_nominal_capacitance_F=local_capacitance,source_conductance_S=source_conductance,
        within_component_ripple_rating=rms/bulk_count<=rating)


def ripple_screen(recording,output,interleave=False):
    if output.exists():
        raise ValueError('Preserve existing capacitor ripple evidence')
    evidence=json.loads((recording/'summary.json').read_text())
    if evidence['status']!='compiled_eight_coil_feedback_and_power_sequence_screen' or not evidence['passed']:
        raise ValueError('Use a completed passing integrated electrical screen')
    output.mkdir(parents=True)
    channels=Fixed.electronics_acquisition_sar[2]
    grid=Fixed.electronics_low_side_spice_steps_per_cycle*Fixed.electronics_acquisition_grid_refinement
    local=channels*Fixed.electronics_channel_bus_capacitance
    recorded_phases=-np.array([row['pwm_phase_advance_ticks'] for row in evidence['controller_schedule']['channels']])/evidence['controller_schedule']['carrier_ticks']
    phases=recorded_phases if interleave else np.zeros(channels)
    trials=[]
    inputs={}
    for index,trial in enumerate(evidence['trials']):
        path=recording/f'case_{index}.npz'
        inputs[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        saved=np.load(path)
        updates=saved['updates']
        trajectory=saved['trajectory']
        worst=None
        for end in range(channels,len(updates)+1,channels):
            block=updates[end-channels:end]
            if not np.array_equal(block[:,1],np.arange(channels)):
                raise ValueError('Recorded updates do not contain a complete sequential channel round')
            time=max(row[0]+evidence['controller_schedule']['channels'][int(row[1])]['actuation_s']-logic_commit_time() for row in block)
            if time>trajectory[-1,0]:
                continue
            currents=np.array([np.interp(time,trajectory[:,0],trajectory[:,channel+1]) for channel in range(channels)])
            waveform=group_bus_waveform(currents,block[:,5],grid,phases)
            rms=float(np.std(waveform))
            if worst is None or rms>worst['rms']:
                worst=dict(rms=rms,time=time,currents=currents,voltages=block[:,5].copy(),waveform=waveform)
        if worst is None:
            raise ValueError('No completed current/PWM snapshot found')
        rows=[]
        for count in (value for value in Fixed.electronics_shared_bulk_counts if value):
            for local_capacitance in (0.,local):
                for source_conductance in (0.,1/Fixed.electronics_shared_feed_resistance):
                    row=capacitor_ripple(worst['waveform'],count,local_capacitance,source_conductance)
                    refined=group_bus_waveform(worst['currents'],worst['voltages'],grid*Fixed.qualification_refinement_factor,phases)
                    fine=capacitor_ripple(refined,count,local_capacitance,source_conductance)
                    row['refinement_RMS_difference_A']=abs(row['each_bulk_RMS_A']-fine['each_bulk_RMS_A'])
                    row['numerical_pass']=row['refinement_RMS_difference_A']<=Fixed.electronics_spice_current_tolerance
                    rows.append(row)
        trials.append(dict(recorded_case=index,fault=trial['fault'],snapshot_time_s=worst['time'],
            currents_A=worst['currents'].tolist(),PWM_voltages_V=worst['voltages'].tolist(),ripple=rows))
    inputs[str(recording/'summary.json')]=hashlib.sha256((recording/'summary.json').read_bytes()).hexdigest()
    fingerprints=source_fingerprints(__file__,model.__file__,current_loop.__file__,power_stage.__file__,
        acquisition_stress.__file__,readout.__file__,logic.__file__)
    report=dict(phase_fractions=phases.tolist(),phase_interleaving_matches_recorded_schedule=bool(np.array_equal(phases,recorded_phases)),
        status='recorded_PWM_capacitor_ripple_sizing_screen',source_sha256=fingerprints,input_sha256=inputs,
        trials=trials,hardware_qualified=False,complete_electronics_simulation_ready=False,
        numerical_pass=all(row['numerical_pass'] for trial in trials for row in trial['ripple']),
        limitations=[
            'Frozen common-time physical currents and the most recently committed PWM values reconstruct synchronized carrier pulses, including current-polarity dead-time conduction. This is a sizing diagnostic, not a new coupled switching trajectory.',
            'The selected snapshot maximizes total AC bus current among complete recorded channel rounds; it does not prove the worst bulk-current spectrum among all possible duties and motions.',
            'The source is compared as AC-isolated and as a pure branch resistance. Local ceramic capacitance is compared as absent and as its nominal ideal value. Wiring/part inductance, DC-bias derating and actual source frequency response remain unqualified.',
            'Bulk impedance at one datasheet frequency is used as a constant ESR; equal sharing is assumed. Compare per-capacitor RMS with the datasheet rating before accepting the candidate; a passing current screen would still require temperature/lifetime and impedance qualification.',
            'Selected phases are reported against the recorded controller schedule. This reconstructed waveform does not qualify hardware counter synchronization or shared bus switching transients.'])
    with ZipFile(output/'sources.zip','x',compression=ZIP_DEFLATED) as archive:
        for path in fingerprints:
            archive.write(path,path)
        archive.write(recording/'summary.json','inputs/summary.json')
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision,allow_nan=False)+'\n')
    return report


def logic_commit_time():
    schedule=logic.controller_schedule()
    _,pio=logic.sar_pio_program()
    return (pio['next_sample_tick']+pio['rx_fifo_complete_tick']+schedule['allocated_compute_ticks'])/Fixed.electronics_logic_clock


def group_power_envelope(parts):
    counts=Counter(row['mpn'] for row in parts)
    channels=counts['INA181A1IDBVR']
    drivers=counts[Fixed.electronics_integrated_driver[0]]
    converters=counts[Fixed.electronics_acquisition_sar[0]]
    if channels!=Fixed.electronics_acquisition_sar[2] or drivers*Fixed.electronics_integrated_driver[2]!=channels or converters!=1:
        raise ValueError('Supply screen requires the complete reviewed independent-coil group')
    nominal=current_loop.nominal_circuit()
    inductance=nominal['inductance']*max(Fixed.qualification_inductance_factors)
    energy=channels*inductance*Fixed.driver_peak_current**2/2
    capacitors=[row for row in parts if row['model_parameter']=='electronics_channel_bus_capacitance']
    capacitance=sum(float(row['value']) for row in capacitors)
    analog_voltage=Fixed.electronics_analog_supply
    amplifier_current=next(row[3] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')
    fixed_analog_current=channels*amplifier_current+counts['REF5025IDR']*Fixed.electronics_reference_quiescent_current
    typical_awake,maximum_awake,sleep_current=Fixed.electronics_drv8962_supply_currents
    conversion,standby,power_down=Fixed.electronics_sar_analog_currents
    modes={name:dict(driver_VM_W=drivers*nominal['bus_nominal']*driver_current,
        analog_5V_W=analog_voltage*(fixed_analog_current+converters*adc_current))
        for name,driver_current,adc_current in (
            ('all_awake_converting',maximum_awake,conversion),
            ('drivers_asleep_ADC_standby',sleep_current,standby),
            ('drivers_asleep_ADC_power_down',sleep_current,power_down))}
    for mode in modes.values():
        mode['partial_no_load_W']=sum(mode.values())
    board=power_stage.design.BoardGeometry()
    array=power_stage.prototype_coils.prototype_array(Inputs.cad_coil_layout)
    channel_count=board.motor_area/1e6*coil_layouts.channel_density(array,'independent')
    return dict(channels=channels,driver_packages=drivers,converter_packages=converters,
        operating_bus_V=nominal['bus_nominal'],operating_bus_ceiling_V=nominal['bus_high'],
        inductance_upper_H=inductance,simultaneous_disabled_coil_energy_bound_J=energy,
        local_nominal_motor_capacitance_F=capacitance,
        lossless_bus_voltage_after_disable_from_nominal_V=absorbed_energy_voltage(capacitance,nominal['bus_nominal'],energy),
        minimum_effective_capacitance_from_nominal_F=required_absorption_capacitance(nominal['bus_nominal'],nominal['bus_high'],energy),
        local_nominal_capacitance_passes_operating_envelope=absorbed_energy_voltage(capacitance,nominal['bus_nominal'],energy)<=nominal['bus_high'],
        driver_typical_awake_no_load_W=drivers*nominal['bus_nominal']*typical_awake,
        no_load_modes=modes,area_equivalent_installed_channels=channel_count,
        area_equivalent_whole_board_partial_no_load_W={name:values['partial_no_load_W']*channel_count/channels for name,values in modes.items()},
        wake_waits_s=dict(driver=Fixed.electronics_drv8962_wake_time,driver_cold_start=Fixed.electronics_drv8962_turn_on_time,
            ADC_from_standby=Fixed.electronics_sar_wake_times[0],ADC_from_power_down=Fixed.electronics_sar_wake_times[1]),
        continuous_all_channel_coil_copper_loss_W=channels*nominal['coil_resistance']*Fixed.driver_channel_current**2)


def run(review,output):
    if output.exists():
        raise ValueError('Preserve existing supply evidence')
    schematic=json.loads(review.read_text())
    for filename,digest in schematic['source_sha256'].items():
        if hashlib.sha256(Path(filename).read_bytes()).hexdigest()!=digest:
            raise ValueError(f'Schematic review source changed: {filename}')
    report=dict(status='schematic_based_partial_power_and_inductive_return_envelope',
        **group_power_envelope(schematic['components']),
        disabled_return=regeneration_screen(schematic['components']),
        source_sha256=source_fingerprints(__file__,model.__file__,current_loop.__file__,power_stage.__file__,
            power_stage.design.__file__,power_stage.prototype_coils.__file__,coil_layouts.__file__),
        input_sha256={str(review):hashlib.sha256(review.read_bytes()).hexdigest()},
        complete_supply_or_thermal_qualification=False,
        sources=['https://www.ti.com/lit/ds/symlink/drv8962.pdf','https://www.ti.com/lit/ds/symlink/ads8698.pdf',
            'https://www.ti.com/lit/ds/symlink/ina181.pdf','https://www.ti.com/lit/ds/symlink/ref50.pdf',
            'https://na.industrial.panasonic.com/products/capacitors/aluminum-electrolytic-capacitors/aluminum-electrolytic-capacitors-radial-lead-type/series/83367/model/83735'],
        limitations=[
            'Worst-case stored coil energy at the existing physical peak and inductance upper sensitivity bound, returned without dissipation or source absorption. Lossless energy conservation bounds this isolated inductive return, not externally supplied mechanical energy.',
            'Local capacitance is nameplate capacitance, before DC bias, tolerance, temperature, ESR, ESL and distribution inductance. The calculation does not qualify a bulk capacitor or clamp circuit.',
            'A capacitor alone cannot keep the bus below its operating ceiling if it starts at that ceiling. An explicit supply headroom and/or absorbing clamp is required; no supply is assumed to sink current.',
            'No-load modes sum specified maximum driver VM, converter AVDD, amplifier and reference supply currents at their stated voltages. MCU, digital I/O, reference output load, watchdog, Hall sensing, other logic, conversion losses and output switching are excluded. These are partial scenarios, not whole-board upper bounds.',
            'All-powered installed-channel scaling is area-equivalent, before edge and module rounding. Selective sleep and wake sequencing are required design functions, not implemented by this inventory calculation.',
            'The all-channel copper-loss value is a continuous group stress envelope at cold resistance, not an actual chess workload. Full-board workload, hot resistance, driver losses, motion regeneration, bus wiring and thermal paths remain unresolved.'])
    output.mkdir(parents=True)
    with ZipFile(output/'sources.zip','x',compression=ZIP_DEFLATED) as archive:
        for name in report['source_sha256']:
            archive.write(name,name)
        archive.write(review,str(Path('inputs')/review.name))
        archive.write(review.with_name('sources.zip'),'inputs/schematic_sources.zip')
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--review',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    arguments=parser.parse_args()
    print(json.dumps(run(arguments.review,arguments.output),indent=Fixed.report_precision))

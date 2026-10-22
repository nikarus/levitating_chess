import argparse
import heapq
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np
from scipy.integrate import solve_ivp

import model
from model import Constants, Fixed
from methodology import qualification, source_fingerprints
from methodology.electronics import acquisition, current_loop, firmware, logic, power_stage, protection, supply


class ControlGroupPlant:
    def __init__(self, refinement=1, resistance_factor=1., inductance_factor=1., filter_factor=1., source_factor=None, bulk_count=None):
        self.loop=current_loop.SampledCurrentLoop(resistance_factor,inductance_factor,filter_factor)
        self.channels=Fixed.electronics_acquisition_sar[2]
        self.state=np.zeros((self.channels,self.loop.physical_count))
        self.voltage=np.zeros(self.channels)
        self.enabled=False
        self.time=0.
        self.refinement=refinement
        self.peak=0.
        self.evaluations=0
        self.resistance=self.loop.nominal['resistance']*resistance_factor
        self.inductance=self.loop.nominal['inductance']*inductance_factor
        self.disabled_resistance=(self.loop.nominal['coil_resistance']+Fixed.electronics_readout_trial_shunt/2+
            2*Fixed.electronics_bridge_diode[1])*resistance_factor
        if (source_factor is None)!=(bulk_count is None):
            raise ValueError('A dynamic bus requires both source factor and bulk count')
        self.bus=None if source_factor is None else supply.SharedMotorBus(self.loop.nominal['bus_nominal']*source_factor,bulk_count)
        self.capacitor_voltage=self.loop.nominal['bus_nominal'] if self.bus is None else self.bus.source_voltage
        self.minimum_bus_voltage=self.capacitor_voltage
        self.maximum_bus_voltage=self.capacitor_voltage
        self.energy=np.zeros(4)
        self.disabled_voltage=self.loop.nominal['bus_nominal']+2*Fixed.electronics_bridge_diode[0]

    def bus_point(self,state,capacitor_voltage):
        current=state[:,0]
        if self.enabled:
            deadtime_fraction=2*Fixed.electronics_drv8962_deadtime*Fixed.driver_pwm_frequency
            polarity=np.tanh(current/Fixed.electronics_deadtime_compensation_current)
            modulation=self.voltage/self.loop.nominal['bus_nominal']-deadtime_fraction*polarity
        else:
            modulation=-np.sign(current)
        load=float(modulation@current)
        point=self.bus.operating_point(capacitor_voltage,load)
        return point,modulation

    def advance(self,time):
        if time<self.time:
            raise ValueError('Electrical events must be chronological')
        if time==self.time:
            return
        if self.bus is not None:
            self.advance_with_bus(time)
            return
        initial=self.state[:,0].copy()
        start=self.time
        matrix=self.loop.physical_generator
        def disabled_current(elapsed):
            ratio=self.disabled_voltage/self.disabled_resistance
            return np.sign(initial)*np.maximum((np.abs(initial)+ratio)*np.exp(-elapsed*self.disabled_resistance/self.inductance)-ratio,0.)
        def derivative(now,flat):
            state=flat.reshape(self.state.shape).copy()
            if not self.enabled:
                state[:,0]=disabled_current(now-start)
            result=state@matrix[:self.loop.physical_count,:self.loop.physical_count].T
            if self.enabled:
                delivered=self.voltage-current_loop.deadtime_feedforward(state[:,0],self.loop.nominal['bus_nominal'])
                result[:,0]+=delivered/self.inductance
            else:
                result[:,0]=-np.sign(state[:,0])*(self.disabled_voltage+self.disabled_resistance*np.abs(state[:,0]))/self.inductance
            return result.ravel()
        solution=solve_ivp(derivative,(start,time),self.state.ravel(),method='DOP853',
            rtol=Fixed.electronics_spice_relative_tolerance/self.refinement,
            atol=Fixed.electronics_spice_absolute_tolerance/self.refinement,
            max_step=1/(Fixed.driver_pwm_frequency*self.refinement))
        if not solution.success:
            raise ValueError(solution.message)
        self.state=solution.y[:,-1].reshape(self.state.shape)
        if not self.enabled:
            self.state[:,0]=disabled_current(time-start)
        self.peak=max(self.peak,float(np.max(np.abs(solution.y[::self.loop.physical_count]))))
        self.evaluations+=solution.nfev
        self.time=time

    def advance_with_bus(self,time):
        count=self.state.size
        shape=self.state.shape
        initial_sign=np.sign(self.state[:,0])
        matrix=self.loop.physical_generator
        def physical_state(flat):
            state=flat[:count].reshape(shape).copy()
            if not self.enabled:
                state[:,0]=initial_sign*np.maximum(initial_sign*state[:,0],0.)
            return state
        def derivative(now,flat):
            state=physical_state(flat)
            current=state[:,0]
            point,modulation=self.bus_point(state,flat[count])
            voltage=point['voltage_V']
            if voltage<=0:
                raise ValueError('Motor supply collapsed outside the modeled powered domain')
            result=state@matrix[:self.loop.physical_count,:self.loop.physical_count].T
            if self.enabled:
                deadtime_fraction=2*Fixed.electronics_drv8962_deadtime*Fixed.driver_pwm_frequency
                diode_loss=2*Fixed.electronics_bridge_diode[0]*deadtime_fraction*np.tanh(current/Fixed.electronics_deadtime_compensation_current)
                resistance=self.resistance
            else:
                diode_loss=2*Fixed.electronics_bridge_diode[0]*np.sign(current)
                resistance=self.disabled_resistance
            result[:,0]=(modulation*voltage-diode_loss-resistance*current)/self.inductance
            losses=float(resistance*(current@current)+diode_loss@current)
            return np.r_[result.ravel(),point['capacitor_rate_V_s'],
                self.bus.source_voltage*point['source_current_A'],losses,point['distribution_loss_W']+point['current_limit_loss_W'],point['capacitor_loss_W']]
        def bus_extremum(now,flat):
            state=physical_state(flat)
            current=state[:,0]
            rates=derivative(now,flat)
            current_rates=rates[:count].reshape(shape)[:,0]
            if self.enabled:
                fraction=2*Fixed.electronics_drv8962_deadtime*Fixed.driver_pwm_frequency
                scaled=current/Fixed.electronics_deadtime_compensation_current
                polarity=np.tanh(scaled)
                load_gradient=self.voltage/self.loop.nominal['bus_nominal']-fraction*(polarity+scaled*(1-polarity**2))
            else:
                load_gradient=-np.sign(current)
            point,_=self.bus_point(state,flat[count])
            load=point['source_current_A']-point['capacitor_current_A']
            return self.bus.voltage_rate(flat[count],load,rates[count],float(load_gradient@current_rates))
        initial=np.r_[self.state.ravel(),self.capacitor_voltage,self.energy]
        solution=solve_ivp(derivative,(self.time,time),initial,method='DOP853',events=bus_extremum,
            rtol=Fixed.electronics_spice_relative_tolerance/self.refinement,
            atol=Fixed.electronics_spice_absolute_tolerance/self.refinement,
            max_step=1/(Fixed.driver_pwm_frequency*self.refinement))
        if not solution.success:
            raise ValueError(solution.message)
        self.state=physical_state(solution.y[:,-1])
        self.capacitor_voltage=solution.y[count,-1]
        self.energy=solution.y[count+1:,-1]
        for flat in np.vstack((solution.y.T,solution.y_events[0].reshape(-1,len(initial)))):
            state=physical_state(flat)
            voltage=self.bus_point(state,flat[count])[0]['voltage_V']
            self.minimum_bus_voltage=min(self.minimum_bus_voltage,voltage)
            self.maximum_bus_voltage=max(self.maximum_bus_voltage,voltage)
            self.peak=max(self.peak,float(np.max(np.abs(state[:,0]))))
        self.evaluations+=solution.nfev
        self.time=time



def simulate(backend, refinement=1, noisy=False, fault=None, simultaneous=False, gain_direction=1., **plant_parameters):
    if fault not in (None,'missing_commit','invalid_frame','power_loss'):
        raise ValueError('Unsupported integrated fault scenario')
    schedule=logic.controller_schedule()
    _,pio=logic.sar_pio_program()
    clock=Fixed.electronics_logic_clock
    channels=Fixed.electronics_acquisition_sar[2]
    fractions=np.ones(channels) if simultaneous else np.asarray(Fixed.electronics_group_command_fractions)
    if fractions.shape!=(channels,) or np.max(np.abs(fractions))>1:
        raise ValueError('One bounded independent command fraction is required per coil')
    plant=ControlGroupPlant(refinement=refinement,**plant_parameters)
    sequencer=logic.ControlGroupPower()
    ready=sequencer.wake(0)
    interface,configuration_ticks=logic.configured_acquisition()
    epoch=ready+configuration_ticks
    stream=sequencer.begin_acquisition(epoch,interface)
    runtime=firmware.CompiledRuntime(backend,epoch,configuration_verified=True)
    if runtime.begin_zero_calibration()!=0:
        raise ValueError('Idle zero calibration was rejected')
    timing=protection.watchdog_timing()
    gate=protection.IndependentShutdown(timing['maximum_timeout_s'])
    hold_ticks=round(Fixed.electronics_regulation_hold*clock)
    command_ticks=round(clock/Fixed.current_command_rate)
    duration_ticks=hold_ticks*len(Fixed.electronics_regulation_commands)
    end_tick=duration_ticks+round(Fixed.electronics_regulation_settling_window*clock)
    if command_ticks*Fixed.current_command_rate!=clock or hold_ticks%command_ticks:
        raise ValueError('Integrated commands must align with the existing command clock')
    fault_tick=hold_ticks*2 if fault else None
    random=np.random.default_rng(Fixed.current_feedback_seed)
    offsets=fractions*Fixed.current_sense_offset_residual/Fixed.current_sense_resistance if noisy else np.zeros(channels)
    if gain_direction not in (-1.,1.):
        raise ValueError('Gain-error corner must select one of the configured bound directions')
    gains=1+gain_direction*fractions*Fixed.current_feedback_gain_bound if noisy else np.ones(channels)
    commands=np.zeros(channels)
    events=[]
    serial=0
    def queue(tick,kind,payload=None):
        nonlocal serial
        serial+=1
        heapq.heappush(events,(tick,serial,kind,payload))
    for tick in range(0,duration_ticks,command_ticks):
        queue(tick,'command')
    total_frames=duration_ticks//pio['next_sample_tick']
    for index in range(total_frames):
        queue(index*pio['next_sample_tick'],'sample',index)
    if fault=='power_loss':
        queue(fault_tick,'power_loss')
    queue(duration_ticks,'finish')
    queue(end_tick,'end')
    words=[]
    frames=0
    results=[]
    trajectory=[]
    stopped=None
    shutdown_tick=None
    fault_seen=False
    service_stopped=False
    armed_tick=None
    partial=Fixed.electronics_acquisition_sar_frame_bits-Constants.rp2350_pio_word_bits
    sensitivity=acquisition.sensing_filter()['sensitivity_V_per_A']
    adc_span=Fixed.electronics_acquisition_sar[4]
    def inhibit(tick):
        nonlocal shutdown_tick
        if shutdown_tick is None:
            sequencer.disable(epoch+int(np.ceil(tick)))
            shutdown_tick=tick+clock*(timing['logic_shutdown_delay_bound_s']+Fixed.electronics_drv8962_enable_delay)
            queue(shutdown_tick,'disable')
    while events:
        tick,_,kind,payload=heapq.heappop(events)
        if tick>end_tick:
            break
        plant.advance(tick/clock)
        trajectory.append(np.r_[tick/clock,plant.state[:,0]])
        if plant.bus is not None and (plant.maximum_bus_voltage>plant.loop.nominal['bus_high'] or
                plant.minimum_bus_voltage<plant.loop.nominal['bus_nominal']*(1-Fixed.electronics_supply_tolerance)):
            stopped='operating_bus_voltage'
            break
        if plant.peak+plant.loop.ripple_bound>Fixed.driver_peak_current:
            stopped='operating_peak_current'
            break
        if kind=='command':
            if fault_seen or service_stopped:
                continue
            commands=fractions*Fixed.driver_channel_current*Fixed.electronics_regulation_commands[tick//hold_ticks]
            for channel,current in enumerate(commands):
                if runtime.command(channel,float(current)):
                    raise ValueError('Compiled runtime rejected an authorized bounded command')
        elif kind=='sample':
            if fault_seen or service_stopped:
                continue
            index=payload
            channel=index//Fixed.electronics_regulation_samples_per_carrier%channels
            sample=plant.state[:,-1]*acquisition.sensing_filter()['load_gain']*gains+offsets
            if noisy:
                sample+=random.normal(0.,Fixed.current_feedback_noise*np.sqrt(Fixed.electronics_regulation_samples_per_carrier),channels)
            codes=acquisition.current_codes(sample,sensitivity,Fixed.electronics_acquisition_sar[1],adc_span)
            following=channel if index%2==0 else (channel+1)%channels
            frame=interface.exchange(logic.sar_manual_command(following),codes)
            words.extend([frame>>partial,frame&((1<<partial)-1)])
            frames+=1
            if index%2:
                queue(tick+pio['rx_fifo_complete_tick'],'receive',(index-1,words.copy()))
                words.clear()
        elif kind=='receive':
            if fault_seen or service_stopped:
                continue
            first,buffer=payload
            if fault=='invalid_frame' and tick>=fault_tick:
                buffer[-1]|=1
                fault_seen=True
            expected=None
            for index in range(2):
                expected=stream.receive(first+index,buffer[index*2:index*2+2],epoch+int(tick))
            result=runtime.receive_pair(first,buffer,epoch+int(tick))
            if result['status']!=1:
                if expected is not None or not fault_seen:
                    raise ValueError('C and independent acquisition models disagree on frame validity')
                inhibit(tick)
                continue
            if expected is None or expected['channel']!=result['channel'] or expected['latch_tick']!=result['latch_tick']:
                raise ValueError('C and independent acquisition models disagree on channel or timing')
            queue(tick+schedule['allocated_compute_ticks'],'commit',result)
        elif kind=='commit':
            if fault_seen or service_stopped:
                continue
            if fault=='missing_commit' and tick>=fault_tick:
                fault_seen=True
                continue
            if not sequencer.commit(epoch+int(tick),payload):
                raise ValueError('Power sequencer rejected a timely channel update')
            arm=sequencer.commits>=channels
            safe=gate.advance(tick/clock,True,arm,True,True)
            queue(tick+int(np.ceil(Fixed.electronics_watchdog_trigger_duration*clock)),'pulse_low')
            queue(gate.expiry*clock,'watchdog_expiry',gate.expiry)
            if arm and armed_tick is None:
                if runtime.calibration_remaining:
                    raise ValueError('Hardware cannot arm before all zero measurements are complete')
                sequencer.arm(epoch+int(tick),safe)
                armed_tick=tick
                queue(tick+Fixed.electronics_drv8962_enable_delay*clock,'enable')
            compare=payload['pwm_compare']
            half=schedule['pwm_TOP']+1
            mask=(1<<Constants.rp2350_pwm_compare_bits)-1
            voltage=((compare&mask)-(compare>>Constants.rp2350_pwm_compare_bits))*plant.loop.nominal['bus_nominal']/half
            queue(payload['latch_tick']-epoch+Fixed.electronics_drv8962_input_delay*clock,
                'actuate',(payload['channel'],voltage))
            channel=payload['channel']
            results.append([tick/clock,channel,commands[channel],plant.state[channel,0],payload['measured_A'],voltage])
        elif kind=='pulse_low':
            gate.advance(tick/clock,False,gate.arm,True,True)
        elif kind=='watchdog_expiry':
            if payload==gate.expiry:
                gate.advance(payload,False,gate.arm,True,True)
                inhibit(tick)
        elif kind=='power_loss':
            fault_seen=True
            gate.advance(tick/clock,gate.pulse,gate.arm,False,True)
            inhibit(tick)
        elif kind=='disable':
            plant.enabled=False
        elif kind=='enable':
            if shutdown_tick is None:
                plant.enabled=True
        elif kind=='actuate':
            channel,voltage=payload
            plant.voltage[channel]=voltage
        elif kind=='finish':
            service_stopped=True
            inhibit(tick)
        elif kind=='end':
            sequencer.sleep(epoch+int(tick),'power_down')
    trace=np.asarray(results)
    trajectory=np.asarray(trajectory)
    allowance=(Fixed.qualification_current_transient_allowance+Fixed.current_sense_offset_residual/Fixed.current_sense_resistance+
        Fixed.driver_channel_current*Fixed.current_feedback_gain_bound)
    error=None
    if fault is None and stopped is None:
        remainder=np.remainder(trace[:,0],Fixed.electronics_regulation_hold)
        mask=(trace[:,0]<duration_ticks/clock) & (remainder>=Fixed.electronics_regulation_hold-Fixed.electronics_regulation_settling_window)
        error=float(np.max(np.abs(trace[mask,3]-trace[mask,2])))
    passed=stopped is None and armed_tick is not None
    if fault:
        passed=passed and fault_seen and shutdown_tick is not None and not plant.enabled and np.max(np.abs(plant.state[:,0]))<=Fixed.electronics_spice_current_tolerance
    else:
        passed=passed and error is not None and error<=allowance and not plant.enabled and sequencer.mode=='power_down'
    energy_error=None
    energy_allowance=None
    if plant.bus is not None:
        stored_change=(plant.bus.capacitance*(plant.capacitor_voltage**2-plant.bus.source_voltage**2)/2+
            plant.inductance*np.sum(plant.state[:,0]**2)/2)
        energy_error=float(stored_change-plant.energy[0]+sum(plant.energy[1:]))
        energy_allowance=(channels*plant.inductance*Fixed.driver_peak_current*Fixed.electronics_spice_current_tolerance+
            plant.bus.capacitance*plant.loop.nominal['bus_high']*Fixed.electronics_bridge_voltage_tolerance)
        passed=passed and abs(energy_error)<=energy_allowance
    report=dict(energy_balance_error_J=energy_error,energy_balance_allowance_J=energy_allowance,
        bus_voltage_range_V=[plant.minimum_bus_voltage,plant.maximum_bus_voltage],
        measurement_gain_direction=gain_direction,simultaneous_commands=simultaneous,final_power_mode=sequencer.mode,passed=bool(passed),fault=fault,noise_and_calibration=noisy,plant_parameters=plant_parameters,
        stopped=stopped,frames=frames,committed_pairs=len(trace),armed_after_calibrated_round=armed_tick is not None,
        armed_time_s=None if armed_tick is None else armed_tick/clock,
        shutdown_time_s=None if shutdown_tick is None else shutdown_tick/clock,
        peak_average_current_A=plant.peak,peak_with_conservative_ripple_A=plant.peak+plant.loop.ripple_bound,
        plateau_current_error_A=error,existing_tracking_allowance_A=allowance,
        final_currents_A=plant.state[:,0].tolist(),derivative_evaluations=plant.evaluations)
    return trace,trajectory,report


def run(output,host_compiler,arm_compiler,configurations=None):
    if output.exists():
        raise ValueError('Preserve existing integrated electronics evidence')
    output.mkdir(parents=True)
    backend,approximation,build_sources=firmware.build(output/'compiled',host_compiler,arm_compiler)
    fingerprints=source_fingerprints(__file__,model.__file__,current_loop.__file__,firmware.__file__,logic.__file__,
        acquisition.__file__,protection.__file__,supply.__file__,power_stage.__file__,power_stage.design.__file__,
        power_stage.prototype_coils.__file__,qualification.__file__,Path(firmware.__file__).with_name('regulator.c'),*build_sources)
    with ZipFile(output/'sources.zip','x',compression=ZIP_DEFLATED) as archive:
        for filename in fingerprints:
            archive.write(filename,filename)
    trials=[]
    if configurations is None:
        configurations=[dict(noisy=noisy) for noisy in (False,True)]
        configurations.extend(dict(noisy=True,fault=fault) for fault in ('missing_commit','invalid_frame','power_loss'))
        for source_factor in (1-Fixed.electronics_shared_source_tolerance,1+Fixed.electronics_shared_source_tolerance):
            configurations.append(dict(noisy=True,simultaneous=True,source_factor=source_factor,
                bulk_count=max(Fixed.electronics_shared_bulk_counts),inductance_factor=max(Fixed.qualification_inductance_factors)))
        for source in (1-Fixed.electronics_shared_source_tolerance,1+Fixed.electronics_shared_source_tolerance):
            configurations.extend([
                dict(noisy=True,simultaneous=True,source_factor=source,bulk_count=max(Fixed.electronics_shared_bulk_counts),
                    inductance_factor=max(Fixed.qualification_inductance_factors),filter_factor=min(Fixed.electronics_regulation_filter_factors),gain_direction=-1.),
                dict(noisy=True,simultaneous=True,source_factor=source,bulk_count=max(Fixed.electronics_shared_bulk_counts),
                    inductance_factor=min(Fixed.qualification_inductance_factors),filter_factor=max(Fixed.electronics_regulation_filter_factors))])
        configurations.extend(dict(noisy=True,simultaneous=True,source_factor=1+Fixed.electronics_shared_source_tolerance,
            bulk_count=max(Fixed.electronics_shared_bulk_counts),inductance_factor=max(Fixed.qualification_inductance_factors),fault=fault)
            for fault in ('missing_commit','invalid_frame','power_loss'))
    for index,configuration in enumerate(configurations):
        trace,trajectory,result=simulate(backend,**configuration)
        refined_trace,refined_trajectory,refined=simulate(backend,refinement=Fixed.qualification_refinement_factor,**configuration)
        same_events=trace.shape==refined_trace.shape and np.array_equal(trace[:,:2],refined_trace[:,:2])
        difference=float(np.max(np.abs(trace[:,3:5]-refined_trace[:,3:5]))) if same_events else None
        bus_difference=float(np.max(np.abs(np.array(result['bus_voltage_range_V'])-refined['bus_voltage_range_V'])))
        result.update(numerical_bus_difference_V=bus_difference,numerical_difference_A=difference,numerical_pass=same_events and difference<=Fixed.electronics_spice_current_tolerance and
                bus_difference<=current_loop.nominal_circuit()['resistance']*Fixed.electronics_spice_current_tolerance,
            refined_passed=refined['passed'])
        np.savez_compressed(output/f'case_{index}.npz',updates=trace,trajectory=trajectory,refined_updates=refined_trace,refined_trajectory=refined_trajectory)
        trials.append(result)
        (output/'summary.json').write_text(json.dumps(dict(status='incomplete_integrated_electronics_screen',trials=trials),indent=Fixed.report_precision)+'\n')
        if not result['passed'] or not result['numerical_pass'] or not result['refined_passed']:
            break
    report=dict(controller_schedule=logic.controller_schedule(),complete_electronics_simulation_ready=False,status='compiled_eight_coil_feedback_and_power_sequence_screen',source_sha256=fingerprints,
        passed=len(trials)==len(configurations) and all(row['passed'] and row['numerical_pass'] and row['refined_passed'] for row in trials),
        trials=trials,feedforward_approximation=approximation,hardware_qualified=False,
        trace_columns=['time_s','channel','command_A','physical_current_A','reported_current_A','PWM_voltage_V'],
        limitations=[
            'Eight physical averaged RL/filter plants generate every ADC code. Tagged serial frames feed the compiled C runtime; packed PWM compares return through scheduled latch and driver delays. No recorded feedback or true-current controller input is used.',
            'Zero calibration uses the first validated disabled-channel pair; injected offsets are never supplied directly to the controller. Noise is an independent complete-budget approximation, not a complete component noise spectrum.',
            'Current-dependent smooth dead-time loss is a provisional averaged approximation. Carrier commutation and readout distortion require the existing separate switching/vendor-model checks.',
            'Dynamic-bus cases include a non-sinking current-limited source, resistive distribution, minimum-nameplate bulk capacitance and exploratory ESR. Other cases retain a stiff rail. The source boundary and bulk count are candidates, not selected or qualified supply hardware; source-loop dynamics, wiring inductance, auxiliary rails and thermal behavior remain open.',
            'Counter preloads and per-channel zero-crossing latches define interleaved carrier timing. The C reference predicts each channel to its own actuation instant. Phase-dependent switched readout distortion remains to be qualified. Watchdog and enable logic are behavioral with separately added delay bounds. GPIO/DMA/CPU execution time, physical rail startup and asynchronous hardware races are not qualified.',
            'Fault cases test missing commits, malformed frames and an injected supervisor fault signal. They do not qualify arbitrary overcurrent, a failed power transistor, a supervisor circuit or motion/Hall interfaces.'])
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision,allow_nan=False)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--host-compiler',type=Path,required=True)
    parser.add_argument('--arm-compiler',type=Path,required=True)
    args=parser.parse_args()
    report=run(args.output,args.host_compiler,args.arm_compiler)
    print(json.dumps(dict(passed=report['passed'],trials=report['trials']),indent=Fixed.report_precision))

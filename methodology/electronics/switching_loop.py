import argparse
import ast
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import CubicHermiteSpline

import model
from model import Fixed
from methodology import qualification, source_fingerprints
from methodology.electronics import acquisition, current_loop, power_stage, logic


def coupled_equations(case,settings,enabled=True,phase=0.):
    physical,physical_jacobian=power_stage.return_path_equations(case,settings,enabled)
    sensor=acquisition.sensing_filter()
    rates=2*np.pi*np.asarray(sensor['poles_Hz'])
    shunt=Fixed.electronics_readout_trial_shunt/2
    count=5+len(rates)

    def derivative(time,state):
        result=np.empty(count)
        result[:5]=physical(time+phase,state)[:5]
        result[5]=rates[0]*((state[2]-state[1])/shunt-state[5])
        result[6]=rates[1]*(sensor['load_gain']*state[5]+sensor['load_offset_A']-state[6])
        result[7:]=rates[2:]*(state[6:-1]-state[7:])
        return result

    def jacobian(time,state):
        matrix=np.zeros((count,count))
        matrix[:5,:5]=physical_jacobian(time+phase,state)[:5,:5]
        matrix[5,1:3]=rates[0]*np.array([-1.,1.])/shunt
        matrix[5,5]=-rates[0]
        matrix[6,5:7]=rates[1]*np.array([sensor['load_gain'],-1.])
        for index,rate in enumerate(rates[2:],7):
            matrix[index,index-1:index+1]=np.array([rate,-rate])
        return matrix

    return derivative,jacobian


def periodic_initial(recording,refinement,initial_fraction=1.,phase=0.):
    evidence=json.loads((recording/'summary.json').read_text())
    currents=np.array([row['checks'][-1]['actual_mean_A'] for row in evidence['trials']])
    requested=np.array([row['settings']['requested_current_A'] for row in evidence['trials']])
    matches=np.flatnonzero(requested==initial_fraction*Fixed.driver_channel_current)
    if len(matches)!=1:
        raise ValueError('Exactly one recorded initial-current case is required')
    trial_index=int(matches[0])
    trial=evidence['trials'][trial_index]
    if evidence['status']!='state_space_bridge_and_vendor_amplifier_carrier_mean_verification' or not trial['numerical_mean_pass']:
        raise ValueError('A verified periodic bridge is required')
    path=recording/f'case_{trial_index}_refinement_{refinement}/plant.npz'
    with np.load(path) as saved:
        time=saved['time']
        states=saved['states']
    period=1/Fixed.driver_pwm_frequency
    if abs(time[-1]-period)>period*Fixed.electronics_spice_relative_tolerance:
        raise ValueError('Recorded bridge carrier differs from the simulation carrier')
    grid=Fixed.electronics_low_side_spice_steps_per_cycle*Fixed.electronics_acquisition_grid_refinement*refinement
    samples=np.arange(grid)*period/grid
    signal=np.interp(samples,time,(states[:,2]-states[:,1])/(Fixed.electronics_readout_trial_shunt/2))
    sensor=acquisition.sensing_filter()
    if not 0<=phase<period:
        raise ValueError('Periodic initialization phase must be inside the recorded carrier')
    initial=[np.interp(phase,time,states[:,index]) for index in range(5)]
    for index,pole in enumerate(sensor['poles_Hz']):
        if index==1:
            signal=sensor['load_gain']*signal+sensor['load_offset_A']
        signal=acquisition.filter_periodic(signal,pole,1,Fixed.driver_pwm_frequency)
        initial.append(np.interp(phase,np.r_[samples,period],np.r_[signal,signal[0]]))
    return np.asarray(initial),trial['settings'],float(currents[trial_index]),path


def simulate(recording,refinement,output,release=False,scenario=None,digital=False,regulator=current_loop.regulator_voltage,reference_factory=current_loop.CurrentReference,channel=None):
    initial_fraction,target_fraction,enabled=(1.,0. if release else -1.,True) if scenario is None else scenario
    phase=0.
    if channel is not None:
        if not digital or not 0<=channel<Fixed.electronics_acquisition_sar[2]:
            raise ValueError('A phased switching test requires a valid digital control channel')
        row=logic.controller_schedule()['channels'][channel]
        phase=(1/(2*Fixed.driver_pwm_frequency)+row['pwm_phase_advance_ticks']/Fixed.electronics_logic_clock)%(1/Fixed.driver_pwm_frequency)
    state,settings,initial_current,source=periodic_initial(recording,refinement,initial_fraction,phase)
    case=next(row for row in power_stage.topology_cases()
        if row['topology']=='unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    nominal=current_loop.nominal_circuit()
    sensor=acquisition.sensing_filter()
    commanded_current=target_fraction*Fixed.driver_channel_current
    reference=reference_factory(initial_current,np.float32 if digital else float)
    reference.state[-1]=commanded_current
    schedule=acquisition.sar_schedule()
    if channel is not None:
        schedule={**schedule,'actuation_s':row['actuation_s'],'pwm_latch_s':row['latch_offset_ticks']/Fixed.electronics_logic_clock}
    period=1/Fixed.driver_pwm_frequency
    duration=Fixed.electronics_regulation_switching_duration if scenario is None else Fixed.electronics_regulation_hold
    cycles=duration/period
    carriers_per_update=Fixed.driver_pwm_frequency/Fixed.current_regulator_rate
    if not all(float(value).is_integer() for value in (cycles,carriers_per_update)):
        raise ValueError('The switching run must align with carrier and regulator clocks')
    cycles,carriers_per_update=map(int,(cycles,carriers_per_update))
    integral=settings['command_V']-current_loop.deadtime_feedforward(initial_current,case['bus_nominal_V'])-current_loop.reference_feedforward(initial_current,0.,nominal)
    queued={}
    native_time=[np.array([0.])]
    native_states=[state[None,:].copy()]
    events=[]
    rng=np.random.default_rng(Fixed.current_feedback_seed)
    count=len(state)
    tolerance=np.r_[Fixed.electronics_spice_absolute_tolerance,
        np.full(4,Fixed.electronics_bridge_voltage_tolerance),
        np.full(count-5,Fixed.electronics_spice_absolute_tolerance)]/refinement
    stopped=False
    conducting=True
    for cycle in range(cycles):
        begin,end=cycle*period,(cycle+1)*period
        first=state[-1]
        sample_time=begin+schedule['sample_spacing_s']
        points={begin,sample_time,end}
        points.update(time for time in queued if begin<=time<=end)
        if not enabled and cycle==0:
            points.add(Fixed.electronics_drv8962_enable_delay)
        ordered=sorted(points)
        for start,stop in zip(ordered[:-1],ordered[1:]):
            if start in queued:
                settings={**settings,'command_V':queued.pop(start)}
            if not enabled and start>=Fixed.electronics_drv8962_enable_delay:
                conducting=False
            derivative,jacobian=coupled_equations(case,settings,conducting,phase)
            solution=solve_ivp(derivative,(start,stop),state,method='Radau',jac=jacobian,
                rtol=Fixed.electronics_spice_relative_tolerance/refinement,atol=tolerance,
                max_step=period/Fixed.electronics_spice_steps_per_cycle/refinement)
            state=solution.y[:,-1]
            native_time.append(solution.t[1:])
            native_states.append(solution.y[:,1:].T)
            if not solution.success:
                np.savez_compressed(output/f'refinement_{refinement}_failed.npz',time=np.concatenate(native_time),
                    states=np.concatenate(native_states),events=np.asarray(events))
                (output/'failure.json').write_text(json.dumps(dict(refinement=refinement,cycle=cycle,message=solution.message)))
                raise ValueError(solution.message)
            if np.max(np.abs(solution.y[0]))>Fixed.driver_peak_current:
                stopped=True
                break
            if stop==sample_time and cycle%carriers_per_update==0:
                raw=acquisition.current_quantizer(np.array([first,state[-1]]),sensor['sensitivity_V_per_A'],
                    Fixed.electronics_acquisition_sar[1],Fixed.electronics_acquisition_sar[4])
                reading=(float(raw.mean())-sensor['load_offset_A'])/sensor['load_gain']+rng.normal(0.,Fixed.current_feedback_noise)
                computed=begin+schedule['computation_ready_s']
                reference_start=0. if digital else begin
                target,target_rate=reference.current_and_rate(reference_start+schedule['actuation_s'])
                desired_reading=reference.paired_reading(reference_start)
                voltage,integral,requested=regulator(desired_reading-reading,integral,
                    target,target_rate,nominal,digital)
                if digital:
                    reference.state=reference.at(1/Fixed.current_regulator_rate)
                if enabled:
                    queued[begin+schedule['actuation_s']]=float(voltage)
                events.append([begin,computed,begin+schedule['pwm_latch_s'],reading,target,voltage,requested])
        if stopped:
            break
    time=np.concatenate(native_time)
    states=np.concatenate(native_states)
    events=np.asarray(events)
    np.savez_compressed(output/f'refinement_{refinement}.npz',time=time,states=states,events=events)
    window=1/Fixed.current_regulator_rate
    last=time[-1]
    trace=np.column_stack((time,states[:,0]))
    from methodology.electronics.readout import interval_means
    final_mean=float(interval_means(trace,last-window,last)[0]) if last>=window else None
    allowance=Fixed.qualification_current_transient_allowance+Fixed.current_sense_offset_residual/Fixed.current_sense_resistance+Fixed.driver_channel_current*Fixed.current_feedback_gain_bound
    hold_start=duration-Fixed.electronics_regulation_settling_window
    means=[float(interval_means(trace,start,start+window)[0]) for start in
        np.arange(hold_start,duration-window/2,window)] if not stopped and scenario is not None else []
    plateau_error=max(abs(value-commanded_current) for value in means) if means else None
    return dict(channel=channel,physical_gate_phase_shift_s=phase,actuation_delay_s=schedule['actuation_s'],
        refinement=refinement,stopped=stopped,completed_time_s=float(last),commanded_current_A=commanded_current,
        bridge_enabled=enabled,settled_period_means_A=means,plateau_error_A=plateau_error,
        plateau_tracking_pass=bool(not stopped and plateau_error is not None and plateau_error<=allowance) if scenario is not None else None,
        final_mean_tracking_allowance_A=allowance,
        final_mean_tracking_pass=bool(not stopped and final_mean is not None and abs(final_mean-commanded_current)<=allowance),
        peak_current_A=float(np.max(np.abs(states[:,0]))),final_regulator_period_mean_A=final_mean,
        initial_mean_A=initial_current,initial_plant_path=str(source),
        initial_plant_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        controller_samples=len(events),native_points=len(time))


def run(recording,output,release=False,scenario=None,digital=False,compiled_library=None,channel=None):
    if output.exists():
        raise ValueError('Preserve existing switching-loop evidence')
    output.mkdir(parents=True)
    regulator=current_loop.regulator_voltage
    reference_factory=current_loop.CurrentReference
    extras=()
    if compiled_library is not None:
        from methodology.electronics import firmware

        if not digital:
            raise ValueError('Compiled regulation requires digital clocks and PWM')
        regulator=firmware.CompiledRegulator(compiled_library)
        reference_factory=lambda initial,dtype:firmware.CompiledReference(initial,dtype,regulator.library,0 if channel is None else channel)
        extras=(firmware.__file__,Path(firmware.__file__).with_name('regulator.c'),compiled_library,
            compiled_library.with_name('build.json'),compiled_library.with_name('regulator_settings.h'))
    fingerprints=source_fingerprints(__file__,model.__file__,qualification.__file__,acquisition.__file__,logic.__file__,current_loop.__file__,
        power_stage.__file__,power_stage.design.__file__,power_stage.prototype_coils.__file__,*extras)
    with ZipFile(output/'sources.zip','x',compression=ZIP_DEFLATED) as archive:
        for name in fingerprints:
            archive.write(name,name)
    rows=[]
    for refinement in (1,Fixed.qualification_refinement_factor):
        rows.append(simulate(recording,refinement,output,release,scenario,digital,regulator,reference_factory,channel))
        (output/'partial.json').write_text(json.dumps(rows,indent=Fixed.report_precision)+'\n')
        if rows[-1]['stopped']:
            raise ValueError('Physical switching run stopped at the unchanged peak-current limit')
        if not rows[-1]['final_mean_tracking_pass']:
            raise ValueError('Current failed to settle inside the existing tracking allowance; coarse evidence preserved')
        if scenario is not None and not rows[-1]['plateau_tracking_pass']:
            raise ValueError('A settled current plateau exceeded the existing allowance; coarse evidence preserved')
    comparison=compare_saved_traces(output)
    report=dict(status='resolved_switching_bridge_with_sampled_regulator_and_reduced_readout',
        source_sha256=fingerprints,trials=rows,**comparison,
        selected_channel=channel,lifecycle_scenario=scenario,single_precision_regulator_and_integer_PWM=digital,
        compiled_reference_and_regulator=compiled_library is not None,
        plateau_tracking_pass=all(row['plateau_tracking_pass'] for row in rows) if scenario is not None else None,
        final_mean_tracking_pass=all(row['final_mean_tracking_pass'] for row in rows),
        hardware_qualified=False,limitations=[
            'Physical bridge circuit and passive output filter are resolved with a stiff integrator and exact derivatives. The amplifier and ADC internal response remain reduced typical-bandwidth models.',
            'Controller receives only quantized, filtered, delayed sensing with the existing noise budget; actual coil current is used only for plant integration and diagnostics.',
            'Initial bridge comes from a verified powered periodic state. Electronic states use the corresponding periodic reduced-model response; the integral starts at its known holding command minus dead-time and nominal resistive feedforward. Enable tests begin after supplies and bias settle; they do not qualify cold power-on. Disabled gates turn off after the modeled propagation delay, with diode and capacitor paths retained.',
            'Dead-time feedforward uses the smoothed requested current at actuation and typical timing/diode estimates; this is a prediction to avoid delayed-sign correction, not a true-current measurement. Reference feedforward supplies nominal R*i+L*di/dt; feedback compares the measured signal with the nominally filtered paired reference. Saturation and anti-windup remain inside the shared regulator.',
            'Two-point load correction removes the known nominal resistor-divider gain and bias only. Manufacturer-model replay must verify the reduced readout during this reversal.',
            'No amplifier input loading feedback, guaranteed timing/temperature bounds, shared supply dynamics, EMI, back-EMF, mutual coupling or fault lifecycle is qualified.'])
    if digital:
        report['controller_schedule']=logic.controller_schedule()
        report['limitations'].append('Reference states and PI arguments/results use single precision and both leg duties use integer PWM compares. This is not compiled firmware or a worst-case execution-time qualification.')
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    (output/'partial.json').unlink()
    return report


def compare_trace_samples(first,second,case):
    current_differences=[]
    linear_differences=[]
    sensor_differences=[]
    for sampled,reference in ((first,second),(second,first)):
        if sampled['time'][0]!=reference['time'][0] or sampled['time'][-1]!=reference['time'][-1]:
            raise ValueError('Numerical traces must cover the same physical interval')
        interpolator=CubicHermiteSpline(reference['time'],reference['states'][:,0],
            power_stage.winding_current_rate(reference['states'],case),extrapolate=False)
        current_differences.append(float(np.max(np.abs(interpolator(sampled['time'])-sampled['states'][:,0]))))
        linear_differences.append(float(np.max(np.abs(np.interp(sampled['time'],reference['time'],
            reference['states'][:,0])-sampled['states'][:,0]))))
        sensor_differences.append(float(np.max(np.abs(np.interp(sampled['time'],reference['time'],
            reference['states'][:,-1])-sampled['states'][:,-1]))))
    if first['events'].shape!=second['events'].shape or not np.array_equal(first['events'][:,:3],second['events'][:,:3]):
        raise ValueError('Controller events must have matching clocks for a numerical comparison')
    current=max(current_differences)
    sensor=max(sensor_differences)
    return dict(peak_current_refinement_difference_A=current,
        peak_linear_interpolation_current_difference_A=max(linear_differences),
        peak_analog_sensor_refinement_difference_A=sensor,
        peak_sampled_feedback_difference_A=float(np.max(np.abs(first['events'][:,3]-second['events'][:,3]))),
        numerical_pass=max(current,sensor)<=Fixed.electronics_spice_current_tolerance,
        current_comparison='Both native time grids, cubic Hermite reconstruction using winding voltage/current slopes; linear interpolation retained as a diagnostic')


def compare_saved_traces(recording):
    case=next(row for row in power_stage.topology_cases()
        if row['topology']=='unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    with np.load(recording/'refinement_1.npz') as first,np.load(recording/f'refinement_{Fixed.qualification_refinement_factor}.npz') as second:
        return compare_trace_samples(first,second,case)


def recheck_sampling(recording,output):
    if output.exists():
        raise ValueError('Preserve existing recorded-trace verification')
    original=json.loads((recording/'summary.json').read_text())
    if hashlib.sha256(Path(model.__file__).read_bytes()).hexdigest()!=original['source_sha256']['model.py']:
        raise ValueError('Recorded-trace verification requires the exact executed model settings')
    output.mkdir(parents=True)
    fingerprints=source_fingerprints(__file__,model.__file__,power_stage.__file__)
    report=dict(status='recorded_switching_trace_resampling_diagnosis',**compare_saved_traces(recording),
        original_numerical_pass=original['numerical_pass'],source_sha256=fingerprints,
        input_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in
            (recording/'summary.json',recording/'refinement_1.npz',recording/f'refinement_{Fixed.qualification_refinement_factor}.npz')},
        hardware_qualified=False,limitations=[
            'Reconstructs saved continuous winding current from native states and the winding differential equation; no controller commands or physical states are changed.',
            'Checks both native grids against the opposite reconstruction with the unchanged current tolerance. This is a recorded numerical comparison, not additional hardware or component-corner qualification.',
            'The original linear-interpolation failure remains preserved. Filtered analog readout is still conservatively compared with linear interpolation.'])
    with ZipFile(output/'sources.zip','x',compression=ZIP_DEFLATED) as archive:
        for name in fingerprints:
            archive.write(name,name)
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    return report


def lifecycle_suite(recording,output):
    if output.exists():
        raise ValueError('Preserve existing lifecycle-suite evidence')
    output.mkdir(parents=True)
    stages=[]
    for name,initial,target,enabled in Fixed.electronics_regulation_lifecycle_cases:
        stages.append(dict(name=name,status='pending'))
    report=dict(status='running',stages=stages,passed=False,hardware_qualified=False)
    path=output/'summary.json'
    path.write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    for stage,(_,initial,target,enabled) in zip(stages,Fixed.electronics_regulation_lifecycle_cases,strict=True):
        stage['status']='running'
        path.write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
        folder=output/stage['name']
        with (output/f"{stage['name']}.log").open('x') as log:
            process=subprocess.run([sys.executable,'-W','error','-m',__name__ if __name__!='__main__' else 'methodology.electronics.switching_loop',
                '--recording',str(recording),'--output',str(folder),'--scenario',stage['name']],stdout=log,stderr=subprocess.STDOUT)
        stage['exit_code']=process.returncode
        if process.returncode!=0:
            stage['status']='failed'
            report['status']='failed'
            path.write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
            raise ValueError(f"Lifecycle stage failed: {stage['name']}; inspect its retained log and partial evidence")
        result=json.loads((folder/'summary.json').read_text())
        stage['passed']=result['numerical_pass'] and result['final_mean_tracking_pass'] and result['plateau_tracking_pass']
        stage['status']='passed' if stage['passed'] else 'failed'
        if not stage['passed']:
            report['status']='failed'
        path.write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
        if not stage['passed']:
            raise ValueError(f"Lifecycle stage failed: {stage['name']}")
    report['status']='completed'
    report['passed']=True
    path.write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    return report


def replay_window(time,states,events,duration):
    if not 0.<duration<=time[-1] or time[0]!=0. or np.any(np.diff(time)<=0):
        raise ValueError('Replay window must lie within the recorded trajectory')
    end=np.searchsorted(time,duration)
    endpoint=np.array([np.interp(duration,time,column) for column in states.T])
    return np.r_[time[:end],duration],np.vstack((states[:end],endpoint)),events[
        events[:,0]+acquisition.sar_schedule()['sample_spacing_s']<=duration]


def vendor_replay(recording,output,vendor_model,executable,reuse_coarse=None,transition_only=False):
    from scipy.signal import lsim
    from methodology.electronics import bridge_readout,readout

    evidence=json.loads((recording/'summary.json').read_text())
    if evidence['status']!='resolved_switching_bridge_with_sampled_regulator_and_reduced_readout' or not evidence['numerical_pass']:
        raise ValueError('A numerically verified switching reversal is required')
    with ZipFile(recording/'sources.zip') as archive:
        recorded_model=archive.read('model.py')
    if hashlib.sha256(recorded_model).hexdigest()!=evidence['source_sha256']['model.py']:
        raise ValueError('Recorded model source archive differs from its fingerprint')
    def physical_model_tree(content):
        tree=ast.parse(content)
        for node in tree.body:
            if isinstance(node,ast.ClassDef) and node.name=='Fixed':
                node.body=[field for field in node.body if not (isinstance(field,ast.Assign) and any(
                    isinstance(target,ast.Name) and target.id in ('electronics_replay_compression_fraction','electronics_spice_case_timeout') for target in field.targets))]
        return ast.dump(tree)
    if physical_model_tree(recorded_model)!=physical_model_tree(Path(model.__file__).read_bytes()):
        raise ValueError('Model settings differ from the executed switching reversal; only replay compression and wall-time allowance may differ')
    content,executable,fingerprints=readout.prepare_vendor_evidence(output,vendor_model,executable,
        b'.SUBCKT INA181A1 INP INN VCC GND REF OUT')
    extras=source_fingerprints(__file__,current_loop.__file__,acquisition.__file__,logic.__file__,bridge_readout.__file__)
    with ZipFile(output/'sources.zip','a',compression=ZIP_DEFLATED) as archive:
        for name in extras.keys()-fingerprints.keys():
            archive.write(name,name)
    fingerprints.update(extras)
    (output/'ina181a1.lib').write_bytes(content)
    period=1/Fixed.driver_pwm_frequency
    prefix=Fixed.electronics_acquisition_filter_cycles*period
    inputs={str(recording/'summary.json'):hashlib.sha256((recording/'summary.json').read_bytes()).hexdigest()}
    rows=[]
    results=[]
    for trial in evidence['trials']:
        refinement=trial['refinement']
        initial=Path(trial['initial_plant_path'])
        if hashlib.sha256(initial.read_bytes()).hexdigest()!=trial['initial_plant_sha256']:
            raise ValueError('Initial periodic bridge has changed')
        source=recording/f'refinement_{refinement}.npz'
        inputs[str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
        inputs[str(initial)]=trial['initial_plant_sha256']
        with np.load(initial) as saved:
            initial_time,initial_values=bridge_readout.shifted_periodic_waveform(
                saved['time'],saved['states'][:,1:3],trial['physical_gate_phase_shift_s'])
            initial_time,initial_values=bridge_readout.compact_piecewise_linear(initial_time,initial_values,
                bridge_readout.replay_voltage_tolerance(refinement))
        with np.load(source) as saved:
            switch_time,switch_states,events=saved['time'],saved['states'],saved['events']
        switch_time,indices=np.unique(switch_time,return_index=True)
        switch_states=switch_states[indices]
        if transition_only:
            switch_time,switch_states,events=replay_window(switch_time,switch_states,events,
                Fixed.electronics_regulation_switching_duration)
        compact_time,compact_values=bridge_readout.compact_piecewise_linear(switch_time,switch_states[:,1:3],
            bridge_readout.replay_voltage_tolerance(refinement))
        time=np.concatenate([initial_time[:-1]+index*period for index in range(Fixed.electronics_acquisition_filter_cycles)]+[prefix+compact_time])
        values=np.concatenate([initial_values[:-1] for _ in range(Fixed.electronics_acquisition_filter_cycles)]+[compact_values])
        time,indices=np.unique(time,return_index=True)
        values=values[indices]
        folder=output/f'case_0_refinement_{refinement}'
        netlist=bridge_readout.amplifier_waveform_netlist(time,values,refinement,True)
        reused_case=None
        if reuse_coarse is not None and refinement==1:
            reused_case=reuse_coarse/'case_0_refinement_1'
            if (reused_case/'test.cir').read_text()!=netlist or (reuse_coarse/'ina181a1.lib').read_bytes()!=content:
                raise ValueError('Reused replay circuit or manufacturer model differs')
            prior=json.loads((reuse_coarse/'partial.json').read_text())
            if not any(row['refinement']==1 for row in prior):
                raise ValueError('Requested coarse replay was not completed')
            folder.mkdir()
            for name in ('test.cir','ngspice.log','transient.dat','.spiceinit'):
                source_path=reused_case/name
                inputs[str(source_path)]=hashlib.sha256(source_path.read_bytes()).hexdigest()
                os.link(source_path,folder/name)
            inputs[str(reuse_coarse/'sources.zip')]=hashlib.sha256((reuse_coarse/'sources.zip').read_bytes()).hexdigest()
            (folder/'ina181a1.lib').symlink_to('../ina181a1.lib')
        else:
            readout.execute_spice_case(folder,executable,netlist,'ina181a1.lib',Fixed.electronics_readout_spice_compatibility,
                timeout=Fixed.electronics_spice_case_timeout*time[-1]/prefix*refinement)
        trace=np.loadtxt(folder/'transient.dat',skiprows=1)
        if not np.isfinite(trace).all() or abs(trace[-1,0]-time[-1])>period*Fixed.electronics_spice_relative_tolerance:
            raise ValueError('Incomplete transient manufacturer replay')
        count=int(round(time[-1]/period*Fixed.electronics_low_side_spice_steps_per_cycle*refinement))
        uniform=np.linspace(0.,time[-1],count+1)
        signal=np.interp(uniform,trace[:,0],trace[:,1])
        pole=2*np.pi*acquisition.sar_filter_pole()
        if Fixed.electronics_acquisition_sar_filter_order!=2:
            raise ValueError('Transient SAR screen requires the specified two-pole candidate')
        _,filtered,_=lsim(([pole*pole],[1.,2*pole,pole*pole]),signal,uniform)
        selected=uniform>=prefix
        motion_time=uniform[selected]-prefix
        measured=filtered[selected]
        reduced=np.interp(motion_time,switch_time,switch_states[:,-1])
        actual=np.interp(motion_time,switch_time,switch_states[:,0])
        sensor=acquisition.sensing_filter()
        difference=(measured-reduced)/sensor['load_gain']
        allowance=Fixed.current_sense_offset_residual/Fixed.current_sense_resistance+Fixed.current_feedback_gain_bound*np.abs(actual)
        pair_time=np.column_stack((events[:,0],events[:,0]+acquisition.sar_schedule()['sample_spacing_s']))
        pairs=np.interp(prefix+pair_time,uniform,filtered)
        quantized=acquisition.current_quantizer(pairs,sensor['sensitivity_V_per_A'],Fixed.electronics_acquisition_sar[1],Fixed.electronics_acquisition_sar[4])
        feedback=(quantized.mean(axis=1)-sensor['load_offset_A'])/sensor['load_gain']
        noise=np.random.default_rng(Fixed.current_feedback_seed).normal(0.,Fixed.current_feedback_noise,len(events))
        feedback_difference=feedback+noise-events[:,3]
        np.savez_compressed(folder/'comparison.npz',time=motion_time,actual_current=actual,
            vendor_readout=measured,reduced_readout=reduced,sample_time=events[:,0],vendor_feedback=feedback+noise,
            original_feedback=events[:,3])
        rows.append(dict(physical_gate_phase_shift_s=trial['physical_gate_phase_shift_s'],
            refinement=refinement,reused_case_from=str(reused_case) if reused_case is not None else None,
            replayed_duration_s=float(switch_time[-1]),controller_samples=len(events),
            replay_voltage_error_bound_V=bridge_readout.replay_voltage_tolerance(refinement),
            peak_reduced_model_discrepancy_A=float(np.max(np.abs(difference))),
            rms_deterministic_discrepancy_A=float(np.sqrt(np.mean(difference**2))),
            peak_feedback_discrepancy_A=float(np.max(np.abs(feedback_difference))),
            within_existing_gain_offset_envelope=bool(np.all(np.abs(difference)<=allowance)),
            peak_input_loading_A=float(np.max(np.abs(trace[:,2:])))))
        results.append((motion_time,measured))
        (output/'partial.json').write_text(json.dumps(rows,indent=Fixed.report_precision)+'\n')
    numerical=float(np.max(np.abs(np.interp(results[0][0],results[1][0],results[1][1])-results[0][1])))
    report=dict(status='manufacturer_amplifier_replay_of_resolved_switching_reversal',source_sha256=fingerprints,
        transition_only=transition_only,
        recorded_input_sha256=inputs,vendor_sha256=hashlib.sha256(content).hexdigest(),trials=rows,
        peak_numerical_discrepancy_A=numerical,numerical_pass=numerical<=Fixed.electronics_spice_current_tolerance,
        hardware_qualified=False,limitations=[
            'Exact recorded bridge voltages drive the unchanged TI amplifier and physical output filter; converter internal dynamics remain the same typical two-pole approximation.',
            'The replay compares the reduced sensor model during one closed-loop reversal. It does not feed the manufacturer output back into a new bridge trajectory.',
            'The existing gain/offset envelope is checked without changing it. Agreement within that envelope does not establish an error allocation for every remaining component.',
            'RMS discrepancy is deterministic model error, not random noise. Input loading is measured but not coupled back; supply/reference noise, ADC INL/DNL and temperature remain unqualified.'])
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    (output/'partial.json').unlink()
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--recording',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--vendor-model',type=Path)
    parser.add_argument('--ngspice',default='ngspice')
    parser.add_argument('--reuse-coarse-replay',type=Path)
    parser.add_argument('--transition-only',action='store_true')
    parser.add_argument('--release',action='store_true')
    parser.add_argument('--digital',action='store_true')
    parser.add_argument('--compiled-regulator',type=Path)
    parser.add_argument('--channel',type=int)
    parser.add_argument('--recheck-sampling',action='store_true')
    parser.add_argument('--lifecycle-suite',action='store_true')
    parser.add_argument('--scenario',choices=[row[0] for row in Fixed.electronics_regulation_lifecycle_cases])
    args=parser.parse_args()
    if args.channel is not None and (not args.digital or args.vendor_model or args.lifecycle_suite or args.recheck_sampling):
        parser.error('Channel phase selects one new digital switching trajectory')
    if args.compiled_regulator is not None and (not args.digital or args.vendor_model or args.lifecycle_suite or args.recheck_sampling):
        parser.error('Compiled regulation requires one new digital switching trajectory')
    if args.recheck_sampling:
        if args.vendor_model or args.digital or args.scenario or args.lifecycle_suite or args.release or args.transition_only:
            parser.error('Recorded resampling verification must run separately from simulation and manufacturer replay')
        print(json.dumps(recheck_sampling(args.recording,args.output)))
        sys.exit(0)
    if args.digital and (args.vendor_model or args.lifecycle_suite):
        parser.error('Digital selects one new controlled trajectory; it does not change recorded replay or the lifecycle batch')
    if args.reuse_coarse_replay is not None and args.vendor_model is None:
        parser.error('Coarse replay reuse requires the vendor replay mode')
    if args.release and args.vendor_model:
        parser.error('Release selects a new controlled bridge trajectory, not a replay')
    if args.lifecycle_suite and (args.release or args.vendor_model):
        parser.error('Lifecycle suite must run separately from release or vendor replay')
    if args.scenario and (args.lifecycle_suite or args.release or args.vendor_model):
        parser.error('One lifecycle scenario must run separately from other modes')
    if args.lifecycle_suite:
        result=lifecycle_suite(args.recording,args.output)
        print(json.dumps(result))
    else:
        scenario=next(row[1:] for row in Fixed.electronics_regulation_lifecycle_cases if row[0]==args.scenario) if args.scenario else None
        if args.transition_only and not args.vendor_model:
            parser.error('Transition-only replay requires a manufacturer model')
        result=vendor_replay(args.recording,args.output,args.vendor_model,args.ngspice,args.reuse_coarse_replay,args.transition_only) if args.vendor_model else run(args.recording,args.output,args.release,scenario,args.digital,args.compiled_regulator,args.channel)
        print(json.dumps(dict(numerical_pass=result['numerical_pass'],trials=result['trials'])))

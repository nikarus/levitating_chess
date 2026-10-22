import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

import numpy as np
from scipy.optimize import root

from model import Fixed
from methodology import source_fingerprints
from methodology.electronics import power_stage, readout


def periodic_bridge(case, settings, refinement, initial=None):
    period=1/Fixed.driver_pwm_frequency
    shunt=Fixed.electronics_readout_trial_shunt/2
    scale=np.array([Fixed.driver_peak_current,Fixed.driver_peak_current*shunt,
                    Fixed.driver_peak_current*shunt,case['bus_nominal_V'],case['bus_nominal_V']])
    if initial is None:
        current=settings['requested_current_A']
        initial=np.array([current,-current*shunt,current*shunt,
            -current*(shunt+Fixed.electronics_drv8962_on_resistance),current*(shunt+Fixed.electronics_drv8962_on_resistance)])

    def closure(scaled):
        state=scaled*scale
        solved=power_stage.independent_return_period(case,settings,state,0.,np.array([period]),refinement)
        return (solved['final_state']-state)/scale

    result=root(closure,initial/scale,options=dict(xtol=Fixed.electronics_spice_relative_tolerance,
        eps=Fixed.electronics_spice_relative_tolerance,maxfev=Fixed.electronics_bridge_periodic_evaluations))
    state=result.x*scale
    solved=power_stage.independent_return_period(case,settings,state,0.,np.array([0.,period]),refinement)
    closure_current=np.max(np.abs(solved['final_state']-state)/np.array([1.,shunt,shunt,
        Fixed.electronics_drv8962_on_resistance,Fixed.electronics_drv8962_on_resistance]))
    if not np.isfinite(state).all() or closure_current>Fixed.electronics_spice_current_tolerance:
        raise ValueError(f'Periodic bridge state did not close: {closure_current} A, {result.message}')
    return dict(time=solved['native_time'],states=solved['native_states'],initial=state,
        means=solved['carrier_means_A'],closure_A=float(closure_current),root_success=bool(result.success),
        root_message=str(result.message),root_evaluations=result.nfev)


def compact_piecewise_linear(time, values, tolerance):
    time=np.asarray(time)
    values=np.asarray(values)
    if time.ndim!=1 or values.shape[0]!=len(time) or values.ndim!=2 or len(time)<2 or tolerance<=0 or not np.isfinite(tolerance) or not np.isfinite(time).all() or not np.isfinite(values).all() or np.any(np.diff(time)<=0):
        raise ValueError('PWL reduction requires increasing finite times and finite columns')
    keep={0,len(time)-1}
    pending=[(0,len(time)-1)]
    while pending:
        start,stop=pending.pop()
        if stop-start<=1:
            continue
        fraction=(time[start+1:stop]-time[start])/(time[stop]-time[start])
        line=values[start]+fraction[:,None]*(values[stop]-values[start])
        errors=np.max(np.abs(values[start+1:stop]-line),axis=1)
        offset=int(np.argmax(errors))
        if errors[offset]>tolerance:
            split=start+1+offset
            keep.add(split)
            pending.extend(((start,split),(split,stop)))
    indices=np.array(sorted(keep))
    return time[indices],values[indices]


def shifted_periodic_waveform(time,values,phase):
    time,values=np.asarray(time),np.asarray(values)
    if time.ndim!=1 or values.ndim!=2 or len(time)!=len(values) or len(time)<2 or time[0]!=0. or np.any(np.diff(time)<=0) or not np.isfinite(values).all() or not np.isfinite(time).all() or not 0<=phase<time[-1]:
        raise ValueError('Periodic replay requires finite increasing times and a phase within one period')
    period=time[-1]
    shifted=np.unique(np.r_[0.,(time[:-1]-phase)%period,period])
    queries=(shifted+phase)%period
    samples=np.column_stack([np.interp(queries,time,column) for column in values.T])
    samples[-1]=samples[0]
    return shifted,samples


def replay_voltage_tolerance(refinement):
    return Fixed.electronics_replay_compression_fraction*Fixed.electronics_spice_current_tolerance*Fixed.electronics_readout_trial_shunt/2/refinement


def amplifier_replay_netlist(plant, refinement, output_filter=False):
    period=1/Fixed.driver_pwm_frequency
    cycles=Fixed.electronics_acquisition_filter_cycles if output_filter else Fixed.electronics_readout_startup_cycles
    stop=cycles*period
    native_time,native_values=compact_piecewise_linear(plant['time'],plant['states'][:,1:3],
        replay_voltage_tolerance(refinement))
    time=np.concatenate([native_time[:-1]+cycle*period for cycle in range(cycles)]+[np.array([stop])])
    values=np.concatenate([native_values[:-1] for _ in range(cycles)]+[native_values[-1:]])
    time,indices=np.unique(time,return_index=True)
    values=values[indices]
    return amplifier_waveform_netlist(time,values,refinement,output_filter)


def amplifier_waveform_netlist(time,values,refinement,output_filter):
    period=1/Fixed.driver_pwm_frequency
    stop=float(time[-1])
    sources=[]
    for column,leg in enumerate(('A','B')):
        sources.append(f'Vpgnd{leg} pgnd{leg} 0 PWL(\n'+'\n'.join(f'+ {t:.17g} {v:.17g}' for t,v in zip(time,values[:,column]))+'\n+ )')
    gain=next(row[1] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')
    sensitivity=gain*Fixed.electronics_readout_trial_shunt/2
    step=period/Fixed.electronics_low_side_spice_steps_per_cycle/refinement
    output_load=f'Rload output 0 {Fixed.electronics_readout_test_load:.17g}'
    output_node='output'
    if output_filter:
        output_load=f"""Rfilter output filtered {Fixed.electronics_acquisition_filter_resistance:.17g}
Cfilter filtered 0 {Fixed.electronics_acquisition_filter_capacitance:.17g}
Vadc_bias adc_bias 0 {Fixed.electronics_acquisition_sar_input_bias:.17g}
Radc filtered adc_bias {Fixed.electronics_acquisition_sar_input_resistance:.17g}"""
        output_node='filtered'
    netlist=f'''Physical shunt waveform replay into unchanged TI INA181; input loading reported separately
.include ina181a1.lib
Vs supply 0 {Fixed.electronics_analog_supply:.17g}
Vref reference 0 {Fixed.electronics_current_reference:.17g}
{chr(10).join(sources)}
Xamp pgndB pgndA supply 0 reference output INA181A1
{output_load}
.options {Fixed.electronics_readout_spice_solver} method={Fixed.electronics_readout_spice_method} maxord={Fixed.electronics_readout_spice_order} reltol={Fixed.electronics_readout_spice_relative_tolerance:.17g} abstol={Fixed.electronics_spice_absolute_tolerance:.17g}
.control
set wr_singlescale
set wr_vecnames
set numdgt=17
tran {step:.17g} {stop:.17g} 0 {step:.17g}
let reported_current=(v({output_node})-{Fixed.electronics_current_reference:.17g})/{sensitivity:.17g}
wrdata transient.dat reported_current i(VpgndA) i(VpgndB)
quit
.endc
.end
'''
    return readout.ramped_readout_supplies(netlist)


def run(output, vendor_model, executable):
    content,executable,fingerprints=readout.prepare_vendor_evidence(output,vendor_model,executable,
        b'.SUBCKT INA181A1 INP INN VCC GND REF OUT')
    extras=source_fingerprints(__file__)
    with ZipFile(output/'sources.zip','a',compression=ZIP_DEFLATED) as archive:
        for name in extras.keys()-fingerprints.keys():
            archive.write(name,name)
    fingerprints.update(extras)
    (output/'ina181a1.lib').write_bytes(content)
    case=next(row for row in power_stage.topology_cases() if row['topology']=='unipolar_full_bridge'
              and row['ripple_pass'] and row['voltage_headroom_pass'])
    period=1/Fixed.driver_pwm_frequency
    rows=[]
    for current in (-Fixed.driver_channel_current,0.,Fixed.driver_channel_current/2,Fixed.driver_channel_current):
        _,settings=power_stage.return_path_netlist(case,current,0.,'pgndA',1,True)
        plants=[]
        checks=[]
        for refinement in (1,2):
            plant=periodic_bridge(case,settings,refinement,plants[0]['initial'] if plants else None)
            plants.append(plant)
            folder=output/f'case_{len(rows)}_refinement_{refinement}'
            readout.execute_spice_case(folder,executable,amplifier_replay_netlist(plant,refinement),
                'ina181a1.lib',Fixed.electronics_readout_spice_compatibility)
            np.savez_compressed(folder/'plant.npz',time=plant['time'],states=plant['states'],initial=plant['initial'])
            trace=np.loadtxt(folder/'transient.dat',skiprows=1)
            stop=Fixed.electronics_readout_startup_cycles*period
            if not np.isfinite(trace).all() or abs(trace[-1,0]-stop)>stop*Fixed.electronics_spice_relative_tolerance:
                raise ValueError('Incomplete amplifier waveform')
            stop=float(trace[-1,0])
            means=readout.interval_means(trace,stop-period,stop)
            previous=readout.interval_means(trace,stop-2*period,stop-period)
            checks.append(dict(refinement=refinement,actual_mean_A=float(plant['means'][0]),
                shunt_mean_A=float(plant['means'][1]),reported_mean_A=float(means[0]),
                reported_error_A=float(means[0]-plant['means'][0]),
                peak_amplifier_input_current_A=float(np.max(np.abs(trace[:,2:]))),
                periodic_closure_A=plant['closure_A'],root_success=plant['root_success'],
                root_evaluations=plant['root_evaluations'],adjacent_reported_mean_difference_A=float(abs(means[0]-previous[0]))))
        delta=max(abs(checks[1][name]-checks[0][name]) for name in ('actual_mean_A','shunt_mean_A','reported_mean_A','reported_error_A'))
        rows.append(dict(settings=settings,checks=checks,numerical_mean_difference_A=delta,
            numerical_mean_pass=delta<=Fixed.electronics_spice_current_tolerance))
        (output/'partial.json').write_text(json.dumps(rows,indent=Fixed.report_precision)+'\n')
        if not rows[-1]['numerical_mean_pass'] or checks[-1]['adjacent_reported_mean_difference_A']>Fixed.electronics_spice_current_tolerance:
            raise ValueError('Coupled circuit mean verification is unresolved')
    report=dict(status='state_space_bridge_and_vendor_amplifier_carrier_mean_verification',
        source_sha256=fingerprints,vendor_sha256=hashlib.sha256(content).hexdigest(),trials=rows,
        hardware_qualified=False,limitations=[
            'Behavioral driver, constant exploratory output capacitance, stiff supply and reference; not a manufacturer driver qualification.',
            'Physical shunt voltages are solved by Radau and replayed through the unchanged TI amplifier model. Input loading is recorded but not fed back to the bridge in this diagnostic.',
            'Mean verification does not qualify raw waveform accuracy, sampled ADC accuracy, noise, calibration, thermal behavior or current-loop stability.'])
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    (output/'partial.json').unlink()
    return report


def run_output_filter(output, recording, executable):
    evidence=json.loads((recording/'summary.json').read_text())
    if evidence['status']!='state_space_bridge_and_vendor_amplifier_carrier_mean_verification' or not all(
            row['numerical_mean_pass'] for row in evidence['trials']):
        raise ValueError('Loaded output requires verified physical bridge evidence')
    content,executable,fingerprints=readout.prepare_vendor_evidence(output,recording/'ina181a1.lib',executable,
        b'.SUBCKT INA181A1 INP INN VCC GND REF OUT')
    if hashlib.sha256(content).hexdigest()!=evidence['vendor_sha256']:
        raise ValueError('Manufacturer model differs from the bridge recording')
    extras=source_fingerprints(__file__)
    with ZipFile(output/'sources.zip','a',compression=ZIP_DEFLATED) as archive:
        for name in extras.keys()-fingerprints.keys():
            archive.write(name,name)
    fingerprints.update(extras)
    (output/'ina181a1.lib').write_bytes(content)
    inputs={str(recording/'summary.json'):hashlib.sha256((recording/'summary.json').read_bytes()).hexdigest()}
    rows=[]
    period=1/Fixed.driver_pwm_frequency
    for index,trial in enumerate(evidence['trials']):
        checks=[]
        for refinement in (1,2):
            path=recording/f'case_{index}_refinement_{refinement}/plant.npz'
            inputs[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
            with np.load(path) as saved:
                plant=dict(time=saved['time'],states=saved['states'])
            if abs(plant['time'][0])>period*Fixed.electronics_spice_relative_tolerance or abs(plant['time'][-1]-period)>period*Fixed.electronics_spice_relative_tolerance:
                raise ValueError('Saved bridge period differs from the acquisition carrier')
            folder=output/f'case_{index}_refinement_{refinement}'
            readout.execute_spice_case(folder,executable,amplifier_replay_netlist(plant,refinement,True),
                'ina181a1.lib',Fixed.electronics_readout_spice_compatibility)
            trace=np.loadtxt(folder/'transient.dat',skiprows=1)
            expected=Fixed.electronics_acquisition_filter_cycles*period
            if not np.isfinite(trace).all() or abs(trace[-1,0]-expected)>period*Fixed.electronics_spice_relative_tolerance:
                raise ValueError('Incomplete loaded amplifier waveform')
            stop=trace[-1,0]
            mean=readout.interval_means(trace,stop-period,stop)[0]
            previous=readout.interval_means(trace,stop-2*period,stop-period)[0]
            checks.append(dict(refinement=refinement,reported_mean_A=float(mean),
                actual_mean_A=trial['checks'][refinement-1]['actual_mean_A'],
                adjacent_reported_mean_difference_A=float(abs(mean-previous)),
                peak_amplifier_input_current_A=float(np.max(np.abs(trace[:,2:])))))
        difference=abs(checks[1]['reported_mean_A']-checks[0]['reported_mean_A'])
        rows.append(dict(checks=checks,numerical_mean_difference_A=difference,
            numerical_mean_pass=difference<=Fixed.electronics_spice_current_tolerance))
        (output/'partial.json').write_text(json.dumps(rows,indent=Fixed.report_precision)+'\n')
        if not rows[-1]['numerical_mean_pass'] or checks[-1]['adjacent_reported_mean_difference_A']>Fixed.electronics_spice_current_tolerance:
            raise ValueError('Loaded amplifier settling or mean refinement failed')
    report=dict(status='physical_output_filter_vendor_amplifier_verification',
        source_sha256=fingerprints,recorded_input_sha256=inputs,vendor_sha256=hashlib.sha256(content).hexdigest(),
        trials=rows,external_filter=dict(resistance_ohm=Fixed.electronics_acquisition_filter_resistance,
            capacitance_F=Fixed.electronics_acquisition_filter_capacitance,
            load_ohm=Fixed.electronics_acquisition_sar_input_resistance),hardware_qualified=False,
        limitations=[
            'Actual passive output filter loads the unchanged amplifier model. The ADC load is represented by its typical input resistance to ground, omitting internal bias and impedance variation.',
            'Physical bridge recordings are reused without amplifier-input loading feedback. ADC internal filtering, quantization, noise, conversion timing and current regulation remain separate.',
            'Component tolerances, bias/reference impedance, shared supply dynamics, thermal drift and protection remain unqualified.'])
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    (output/'partial.json').unlink()
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    selection=parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--vendor-model',type=Path)
    selection.add_argument('--output-filter-recording',type=Path)
    parser.add_argument('--ngspice',required=True)
    args=parser.parse_args()
    if args.output_filter_recording:
        run_output_filter(args.output,args.output_filter_recording,args.ngspice)
    else:
        run(args.output,args.vendor_model,args.ngspice)

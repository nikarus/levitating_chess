import argparse
import json
from itertools import product
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np
from scipy.linalg import expm

import model
from model import Fixed
from methodology import qualification, source_fingerprints
from methodology.electronics import acquisition, power_stage, logic


def nominal_circuit():
    case=next(row for row in power_stage.topology_cases()
        if row['topology']=='unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
    return dict(resistance=case['coil_R_ohm']+2*Fixed.electronics_drv8962_on_resistance+Fixed.electronics_readout_trial_shunt/2,
        inductance=case['minimum_coil_L_H']/min(Fixed.qualification_inductance_factors),
        voltage_limit=case['terminal_voltage_limit_V'],coil_resistance=case['coil_R_ohm'],
        bus_high=case['bus_high_V'],bus_nominal=case['bus_nominal_V'],ripple_frequency=case['ripple_frequency_Hz'])

def deadtime_feedforward(expected_current,bus_voltage):
    return power_stage.deadtime_voltage_drop(bus_voltage)*np.tanh(expected_current/Fixed.electronics_deadtime_compensation_current)


def reference_feedforward(target, target_rate, circuit):
    return circuit['resistance']*target+circuit['inductance']*target_rate


def regulator_voltage(error, integral, target, target_rate, circuit, digital=False, compensate_deadtime=True):
    feedforward=reference_feedforward(target,target_rate,circuit)
    if compensate_deadtime:
        feedforward+=deadtime_feedforward(target,circuit['bus_nominal'])
    arguments=(error,integral,circuit['resistance'],circuit['inductance'],circuit['voltage_limit'],
        Fixed.electronics_regulation_integral_gain_ratio,feedforward)
    voltage,integral,requested=qualification.current_regulator(*(tuple(map(np.float32,arguments)) if digital else arguments))
    if digital:
        positive,negative=power_stage.timer_leg_duties(float(voltage),circuit['bus_nominal'])
        voltage=(positive-negative)*circuit['bus_nominal']
    return voltage,integral,requested


class CurrentReference:
    def __init__(self,initial_current,dtype=float):
        self.rates=np.asarray(2*np.pi*np.r_[Fixed.setpoint_filter_cutoff,acquisition.sensing_filter()['poles_Hz']],dtype=dtype)
        self.matrix=np.zeros((len(self.rates)+1,len(self.rates)+1),dtype=dtype)
        for index,rate in enumerate(self.rates):
            self.matrix[index,index]=-rate
            self.matrix[index,index-1]=rate
        self.state=np.full(len(self.rates)+1,initial_current,dtype=dtype)
        self.transitions={}

    def transition(self,elapsed):
        if elapsed not in self.transitions:
            self.transitions[elapsed]=expm(self.matrix*elapsed)
        return self.transitions[elapsed]

    def at(self,elapsed):
        return self.transition(elapsed)@self.state

    def paired_reading(self,start):
        spacing=acquisition.sar_schedule()['sample_spacing_s']
        return (self.at(start)[-2]+self.at(start+spacing)[-2])/2

    def current_and_rate(self,elapsed):
        state=self.at(elapsed)
        return state[0],self.rates[0]*(state[-1]-state[0])


class SampledCurrentLoop:
    def __init__(self, resistance_factor=1.,inductance_factor=1.,filter_factor=1.,gain=1.,extra_age=0.,refinement=1,digital=False,regulator=regulator_voltage):
        self.regulator=regulator
        self.digital=digital
        self.nominal=nominal_circuit()
        self.ripple_bound=power_stage.design.split_rail_ripple_peak(self.nominal['coil_resistance'],
            self.nominal['inductance']*inductance_factor,self.nominal['bus_high']/2,self.nominal['ripple_frequency'])
        self.schedule=acquisition.sar_schedule()
        self.period=1/Fixed.current_regulator_rate
        self.spacing=self.schedule['sample_spacing_s']
        delay=self.schedule['actuation_s']+extra_age
        self.delayed_periods=int(np.floor(delay/self.period))
        self.actuation=delay-self.delayed_periods*self.period
        if not self.spacing<self.actuation<self.period:
            raise ValueError('Sampling must precede the update phase in the lifted cycle')
        poles=acquisition.sensing_filter()['poles_Hz']
        self.physical_count=len(poles)+1
        self.gain=gain
        matrix=np.zeros((self.physical_count+2,self.physical_count+2))
        matrix[0,0]=-self.nominal['resistance']*resistance_factor/(self.nominal['inductance']*inductance_factor)
        matrix[0,-2:]=np.array([1.,-1.])/(self.nominal['inductance']*inductance_factor)
        for index,pole in enumerate(poles,1):
            matrix[index,index]=-2*np.pi*pole*filter_factor
            matrix[index,index-1]=2*np.pi*pole*filter_factor
        self.physical_generator=matrix
        self.transitions=[]
        for duration in (self.spacing,self.actuation-self.spacing,self.period-self.actuation):
            self.transitions.append(np.linalg.matrix_power(expm(matrix*duration/refinement),refinement))
        self.state_size=self.physical_count+2+self.delayed_periods

    def advance(self,state,target,back_emf=0.,noise=0.,offset=0.,quantize=False,target_rate=0.,reference_reading=None):
        count=self.physical_count
        physical=np.r_[state[:count],state[count+1],back_emf]
        first=physical[count-1]*self.gain+offset
        physical=self.transitions[0]@physical
        second=physical[count-1]*self.gain+offset
        samples=np.array([first,second])
        if quantize:
            gain=next(row[1] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')
            samples=acquisition.current_quantizer(samples,gain*Fixed.electronics_readout_trial_shunt/2,
                Fixed.electronics_acquisition_sar[1],Fixed.electronics_acquisition_sar[4])
        reading=float(samples.mean()+noise)
        delayed=state[count+2] if self.delayed_periods else reading
        desired=target if reference_reading is None else reference_reading
        voltage,integral,requested=self.regulator(desired-delayed,state[count],target,target_rate,
            self.nominal,self.digital,compensate_deadtime=self.digital)
        if self.digital:
            voltage-=deadtime_feedforward(target,self.nominal['bus_nominal'])
        sample_current=physical[0]
        physical=self.transitions[1]@physical
        update_current=physical[0]
        physical[-2]=voltage
        physical=self.transitions[2]@physical
        queue=np.r_[state[count+3:],reading] if self.delayed_periods else np.empty(0)
        result=np.r_[physical[:count],integral,voltage,queue]
        peak=max(abs(state[0]),abs(sample_current),abs(update_current),abs(physical[0]))
        return result,dict(measured_A=delayed,requested_voltage_V=float(requested),
            voltage_V=float(voltage),peak_average_current_A=float(peak))

    def linear_map(self):
        if self.digital:
            raise ValueError('Timer quantization has no linear state map')
        return np.column_stack([self.advance(state,0.)[0] for state in np.eye(self.state_size)])


def simulate(loop,quantize,noise_rms=0.,offset=0.,reference_factory=CurrentReference):
    ticks=Fixed.electronics_regulation_hold*Fixed.current_regulator_rate
    settling=Fixed.electronics_regulation_settling_window*Fixed.current_regulator_rate
    if not float(ticks).is_integer() or not float(settling).is_integer() or not 0<settling<ticks:
        raise ValueError('Current-regulation test windows must be integral positive update counts')
    ticks,settling=int(ticks),int(settling)
    state=np.zeros(loop.state_size)
    reference=reference_factory(0.,np.float32 if loop.digital else float)
    random=np.random.default_rng(Fixed.current_feedback_seed)
    noise=random.normal(0.,noise_rms,len(Fixed.electronics_regulation_commands)*ticks)
    records=[]
    peak=0.
    for fraction in Fixed.electronics_regulation_commands:
        command=fraction*Fixed.driver_channel_current
        reference.state[-1]=command
        for _ in range(ticks):
            filtered,target_rate=reference.current_and_rate(loop.actuation)
            state,metrics=loop.advance(state,filtered,noise=noise[len(records)],offset=offset,quantize=quantize,
                target_rate=target_rate,reference_reading=reference.paired_reading(0.))
            reference.state=reference.at(loop.period)
            peak=max(peak,metrics['peak_average_current_A'])
            records.append([(len(records)+1)*loop.period,command,filtered,state[0],metrics['measured_A'],metrics['voltage_V']])
            if peak>Fixed.driver_peak_current:
                trace=np.asarray(records)
                return trace,dict(stopped=True,reason='Average current exceeds the physical peak-device limit',peak_average_current_A=peak,passed=False)
    trace=np.asarray(records)
    plateau_error=max(float(np.max(np.abs(trace[end-settling:end,3]-trace[end-settling:end,1]))) for end in range(ticks,len(Fixed.electronics_regulation_commands)*ticks+1,ticks))
    allowance=Fixed.qualification_current_transient_allowance+Fixed.current_sense_offset_residual/Fixed.current_sense_resistance+Fixed.driver_channel_current*Fixed.current_feedback_gain_bound
    peak_with_ripple=peak+loop.ripple_bound
    return trace,dict(stopped=False,peak_average_current_A=peak,peak_with_conservative_ripple_A=peak_with_ripple,
        ripple_bound_A=float(loop.ripple_bound),
        plateau_current_error_A=plateau_error,existing_tracking_allowance_A=allowance,
        peak_voltage_V=float(np.max(np.abs(trace[:,5]))),
        passed=bool(peak_with_ripple<=Fixed.driver_peak_current and plateau_error<=allowance))


def run(output,digital=False,regulator=regulator_voltage,reference_factory=CurrentReference,extra_sources=()):
    if output.exists():
        raise ValueError('Preserve existing sampled-regulation evidence')
    output.mkdir(parents=True)
    fingerprints=source_fingerprints(__file__,model.__file__,qualification.__file__,acquisition.__file__,logic.__file__,power_stage.__file__,
        power_stage.design.__file__,power_stage.prototype_coils.__file__,*extra_sources)
    with ZipFile(output/'sources.zip','x',compression=ZIP_DEFLATED) as archive:
        for name in fingerprints:
            archive.write(name,name)
    schedule=acquisition.sar_schedule()
    stability=[]
    for resistance,inductance,filter_factor,gain,extra_age in product(Fixed.electronics_regulation_resistance_factors,
            sorted({1.,*Fixed.qualification_inductance_factors}),Fixed.electronics_regulation_filter_factors,
            (1-Fixed.current_feedback_gain_bound,1+Fixed.current_feedback_gain_bound),
            (0.,schedule['extra_age_if_all_channels_update_together_s'])):
        loop=SampledCurrentLoop(resistance,inductance,filter_factor,gain,extra_age)
        eigenvalues=np.linalg.eigvals(loop.linear_map())
        radius=float(np.max(np.abs(eigenvalues)))
        stability.append(dict(resistance_factor=resistance,inductance_factor=inductance,filter_factor=filter_factor,
            measurement_gain=gain,extra_age_s=extra_age,spectral_radius=radius,stable=radius<1))
    trials=[]
    traces={}
    for index,parameters in enumerate([None,*[row for row in stability if row['extra_age_s']==0.]]):
        configuration={} if parameters is None else dict(resistance_factor=parameters['resistance_factor'],
            inductance_factor=parameters['inductance_factor'],filter_factor=parameters['filter_factor'],gain=parameters['measurement_gain'])
        for stochastic in (False,True):
            loop=SampledCurrentLoop(**configuration,digital=digital,regulator=regulator)
            trace,metrics=simulate(loop,stochastic,Fixed.current_feedback_noise if stochastic else 0.,
                Fixed.current_sense_offset_residual/Fixed.current_sense_resistance if stochastic else 0.,reference_factory=reference_factory)
            fine=SampledCurrentLoop(**configuration,refinement=Fixed.qualification_refinement_factor,digital=digital,regulator=regulator)
            refined,refined_metrics=simulate(fine,stochastic,Fixed.current_feedback_noise if stochastic else 0.,
                Fixed.current_sense_offset_residual/Fixed.current_sense_resistance if stochastic else 0.,reference_factory=reference_factory)
            if trace.shape!=refined.shape:
                raise ValueError('Refined trajectory stopped at a different event')
            difference=float(np.max(np.abs(trace[:,3:5]-refined[:,3:5])))
            trials.append(dict(configuration=configuration,quantization_and_noise=stochastic,**metrics,
                numerical_difference_A=difference,numerical_pass=difference<=Fixed.electronics_spice_current_tolerance,
                refined_passed=refined_metrics['passed']))
            traces[f'case_{index}_noise_{int(stochastic)}']=trace
    np.savez_compressed(output/'traces.npz',**traces)
    report=dict(status='sampled_current_loop_timing_and_averaged_plant_screen',source_sha256=fingerprints,
        schedule=schedule,stability=stability,trials=trials,hardware_qualified=False,
        single_precision_regulator_and_integer_PWM=digital,
        passed=all(row['stable'] for row in stability if row['extra_age_s']==0.) and all(row['passed'] and row['numerical_pass'] and row['refined_passed'] for row in trials),
        limitations=[
            'Exact matrix exponentials advance an averaged linear RL plant and approximate amplifier/RC/ADC filter cascade. The existing PI regulator uses a candidate integral-gain scale, nominal R*i+L*di/dt feedforward and a reference passed through the nominal sensor filter; its flight default is unchanged.',
            'Paired samples, serial-frame completion, provisional computation time, PWM latching and driver propagation delay are explicit. No processor or timer implementation is qualified.',
            'Physical PWM commutation, dead-time distortion, capacitor return currents, changing-current readout distortion, back-EMF and shared bus behavior are not yet coupled in these trajectories.',
            'Resistance and filter factors are exploratory sensitivities, not guaranteed component bounds. Coil inductance uses the existing geometry-uncertainty interval.',
            'The stochastic check injects the existing complete per-update noise budget and a positive residual-offset bound; it does not establish the component noise budget or exhaustive stochastic robustness.',
            'Carrier ripple uses the shared all-duty bound with each trial inductance, high bus voltage and cold coil resistance without credit for extra driver resistance. It is not a resolved switching waveform.'])
    if digital:
        report['controller_schedule']=logic.controller_schedule()
        report['limitations'].extend([
            'The digital candidate rounds reference-filter states and PI arguments/results to single precision and rounds both symmetric leg duties to integer phase-correct PWM compares. Physical plant integration remains double precision.',
            'The eigenvalue table is the unquantized linear reference; digital pass/fail comes from finite-arithmetic trajectories, unchanged tracking/ripple bounds and integration refinement.',
            'Nominal requested-current dead-time compensation is added before PWM quantization and removed in the averaged plant. Actual commutation/sign errors still require the resolved switching circuit.',
            'This algorithm/clock-feasibility screen does not establish instruction timing or qualify the selected microcontroller; a supplied compiled backend must record its separate build evidence.'])
    (output/'summary.json').write_text(json.dumps(report,indent=Fixed.report_precision)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--digital',action='store_true')
    args=parser.parse_args()
    report=run(args.output,args.digital)
    print(json.dumps(dict(passed=report['passed'],trial_count=len(report['trials']))))

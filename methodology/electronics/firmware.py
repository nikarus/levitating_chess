import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

import model
from model import Fixed, Constants
from methodology import source_fingerprints
from methodology.electronics import current_loop,logic,power_stage


def regulator_settings():
    circuit=current_loop.nominal_circuit()
    deadtime=power_stage.deadtime_voltage_drop(circuit['bus_nominal'])
    voltage_budget=circuit['resistance']*Fixed.electronics_spice_current_tolerance
    rounding_bound=8*np.finfo(np.float32).eps*deadtime
    interpolation_budget=voltage_budget/2-rounding_bound
    if interpolation_budget<=0:
        raise ValueError('Single-precision feedforward cannot meet the allocated comparison tolerance')
    domain=float(np.arctanh(1-voltage_budget/(2*deadtime)))
    second_derivative_bound=4/(3*np.sqrt(3))
    intervals=int(np.ceil(domain/np.sqrt(8*interpolation_budget/(deadtime*second_derivative_bound))))
    arguments=np.linspace(0,domain,intervals+1)
    table=np.tanh(arguments).astype(np.float32)
    values=dict(COIL_PATH_RESISTANCE=circuit['resistance'],COIL_INDUCTANCE=circuit['inductance'],
        DEADTIME_VOLTAGE=deadtime,DEADTIME_CURRENT_SCALE=Fixed.electronics_deadtime_compensation_current,
        DEADTIME_DOMAIN=domain,DEADTIME_INDEX_SCALE=intervals/domain,
        LOOP_ANGULAR_FREQUENCY=2*np.pi*Fixed.current_loop_bandwidth,
        VOLTAGE_LIMIT=circuit['voltage_limit'],BUS_VOLTAGE=circuit['bus_nominal'],
        INTEGRAL_RATIO=Fixed.electronics_regulation_integral_gain_ratio,REGULATION_RATE=Fixed.current_regulator_rate,
        PWM_HALF_COUNTS=Fixed.electronics_logic_clock/(2*Fixed.driver_pwm_frequency))
    literal=lambda value:float(np.float32(value)).hex()+'f'
    header=''.join(f'#define {name} {literal(value)}\n' for name,value in values.items())
    header+=f'#define DEADTIME_INTERVALS {intervals}u\n'
    header+='static const float deadtime_table[] = {'+','.join(literal(value) for value in table)+'};\n'
    reference=current_loop.CurrentReference(0.,np.float32)
    schedule=current_loop.acquisition.sar_schedule()
    target_row=reference.transition(schedule['actuation_s'])[0]
    pair_row=(reference.transition(0.)[-2]+reference.transition(schedule['sample_spacing_s'])[-2])/2
    transition=reference.transition(1/Fixed.current_regulator_rate)
    def expression(row):
        terms=[f'{literal(value)} * state[{index}]' for index,value in enumerate(row) if value!=0]
        if not terms:
            return '0.0f'
        return ' + '.join(terms)
    header+='void reference_step(const float *state, float *targets, float *advanced) {\n'
    header+=f'targets[0] = {expression(target_row)};\n'
    header+=f'targets[1] = {literal(reference.rates[0])} * (state[{len(reference.state)-1}] - targets[0]);\n'
    header+=f'targets[2] = {expression(pair_row)};\n'
    header+=''.join(f'advanced[{index}] = {expression(row)};\n' for index,row in enumerate(transition))
    header+='}\n'
    _,pio=logic.sar_pio_program()
    controller=logic.controller_schedule()
    integers=dict(PWM_COMPARE_BITS=Constants.rp2350_pwm_compare_bits,CONTROL_CHANNELS=Fixed.electronics_acquisition_sar[2],REFERENCE_STATES=len(reference.state),
        FRAME_BITS=Fixed.electronics_acquisition_sar_frame_bits,ADC_BITS=Fixed.electronics_acquisition_sar[1],
        ADC_COMMAND_BITS=Constants.ads8698_command_bits,ADC_TAG_BITS=Constants.ads8698_channel_tag_bits,
        FIFO_WORD_BITS=Constants.rp2350_pio_word_bits,SAMPLES_PER_PAIR=Fixed.electronics_regulation_samples_per_carrier,
        CARRIER_TICKS=controller['carrier_ticks'],SAMPLE_TICKS=pio['next_sample_tick'],
        FIFO_READY_TICKS=pio['rx_fifo_complete_tick'],COMPUTE_TICKS=controller['allocated_compute_ticks'])
    if integers['SAMPLES_PER_PAIR']!=2:
        raise ValueError('Compiled acquisition runtime implements the qualified paired-sample protocol')
    header+=''.join(f'#define {name} {value}u\n' for name,value in integers.items())
    header+='static const unsigned channel_latch_ticks[CONTROL_CHANNELS] = {'+','.join(str(row['latch_offset_ticks'])+'u' for row in controller['channels'])+'};\n'
    rows=[reference.transition(row['actuation_s'])[0] for row in controller['channels']]
    header+='static const float channel_target_weights[CONTROL_CHANNELS][REFERENCE_STATES] = {'+','.join('{'+','.join(literal(value) for value in row)+'}' for row in rows)+'};\n'
    header+='void channel_reference_step(unsigned channel, const float *state, float *targets, float *advanced) {\n'
    header+='reference_step(state, targets, advanced);\n'
    header+='targets[0] = 0.0f;\nfor (unsigned index = 0; index < REFERENCE_STATES; ++index) targets[0] += channel_target_weights[channel][index] * state[index];\n'
    header+=f'targets[1] = {literal(reference.rates[0])} * (state[REFERENCE_STATES-1u] - targets[0]);\n}}\n'
    amplifier_gain=next(row[1] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')
    sensitivity=amplifier_gain*Fixed.electronics_readout_trial_shunt/2
    header+=f'#define ADC_CURRENT_SCALE {literal(Fixed.electronics_acquisition_sar[4]/(1<<Fixed.electronics_acquisition_sar[1])/sensitivity)}\n'
    header+=f'#define ADC_CURRENT_BIAS {literal(Fixed.electronics_current_reference/sensitivity)}\n'
    header+=f'#define COMMAND_CURRENT_LIMIT {literal(Fixed.driver_channel_current)}\n'
    bound=deadtime*second_derivative_bound*(domain/intervals)**2/8+rounding_bound
    return header,dict(table_entries=len(table),table_bytes=table.nbytes,argument_domain=domain,
        interpolation_and_rounding_bound_V=bound,saturated_tail_bound_V=voltage_budget/2,
        comparison_voltage_budget_V=voltage_budget,
        method='Linear interpolation uses max abs(tanh second derivative)=4/(3 sqrt(3)); saturation tail is explicit. Rounding reserves eight float32 relative epsilons times the voltage scale.',
        scope='Feedforward voltage before PWM quantization; not a closed-loop trajectory-equivalence bound')


class CompiledRegulator:
    def __init__(self,library):
        build=json.loads(library.with_name('build.json').read_text())
        for filename,digest in build['source_sha256'].items():
            if hashlib.sha256((Path(model.__file__).parent/filename).read_bytes()).hexdigest()!=digest:
                raise ValueError(f'Compiled regulator source changed: {filename}')
        if hashlib.sha256(library.read_bytes()).hexdigest()!=build['binaries'][library.name]:
            raise ValueError('Compiled regulator binary differs from its build evidence')
        header,_=regulator_settings()
        if library.with_name('regulator_settings.h').read_text()!=header:
            raise ValueError('Compiled regulator settings differ from the active model')
        self.library=ctypes.CDLL(str(library.resolve()))
        self.function=self.library.regulate_current
        self.function.argtypes=[ctypes.c_float]*4+[ctypes.POINTER(ctypes.c_float)]
        self.function.restype=None
        self.nominal=current_loop.nominal_circuit()

    def __call__(self,error,integral,target,target_rate,circuit,digital=False,compensate_deadtime=True):
        if not digital or not compensate_deadtime or circuit!=self.nominal:
            raise ValueError('Compiled regulator requires the generated digital nominal circuit configuration')
        result=(ctypes.c_float*3)()
        self.function(error,integral,target,target_rate,result)
        return tuple(result)



class CompiledRuntime:
    def __init__(self,backend,epoch_tick=0,configuration_verified=False):
        self.library=backend.library
        self.library.current_runtime_size.argtypes=[]
        self.library.current_runtime_size.restype=ctypes.c_uint
        size=self.library.current_runtime_size()
        self.state=(ctypes.c_uint64*((size+ctypes.sizeof(ctypes.c_uint64)-1)//ctypes.sizeof(ctypes.c_uint64)))()
        self.library.current_runtime_initialize.argtypes=[ctypes.c_void_p,ctypes.c_uint64,ctypes.c_uint]
        self.library.current_runtime_initialize.restype=None
        self.library.current_runtime_initialize(self.state,epoch_tick,int(configuration_verified))
        self.library.current_runtime_begin_zero_calibration.argtypes=[ctypes.c_void_p]
        self.library.current_runtime_begin_zero_calibration.restype=ctypes.c_int
        self.library.current_runtime_calibration_remaining.argtypes=[ctypes.c_void_p]
        self.library.current_runtime_calibration_remaining.restype=ctypes.c_uint
        self.library.current_runtime_fault.argtypes=[ctypes.c_void_p]
        self.library.current_runtime_fault.restype=ctypes.c_uint
        self.library.current_runtime_command.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_float]
        self.library.current_runtime_command.restype=ctypes.c_int
        self.library.current_runtime_check_deadline.argtypes=[ctypes.c_void_p,ctypes.c_uint64]
        self.library.current_runtime_check_deadline.restype=ctypes.c_int
        self.function=self.library.current_runtime_frame
        self.function.argtypes=[ctypes.c_void_p,ctypes.c_uint64,ctypes.c_uint,ctypes.c_uint,ctypes.c_uint64,ctypes.c_uint,
            ctypes.POINTER(ctypes.c_uint),ctypes.POINTER(ctypes.c_uint),ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_float),ctypes.POINTER(ctypes.c_float)]
        self.function.restype=ctypes.c_int
        self.pair_function=self.library.current_runtime_pair
        self.pair_function.argtypes=[ctypes.c_void_p,ctypes.c_uint64,ctypes.POINTER(ctypes.c_uint),ctypes.c_uint64,ctypes.c_uint,
            ctypes.POINTER(ctypes.c_uint),ctypes.POINTER(ctypes.c_uint),ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_float),ctypes.POINTER(ctypes.c_float)]
        self.pair_function.restype=ctypes.c_int

    @property
    def fault(self):
        return self.library.current_runtime_fault(self.state)

    def begin_zero_calibration(self):
        return self.library.current_runtime_begin_zero_calibration(self.state)

    @property
    def calibration_remaining(self):
        return self.library.current_runtime_calibration_remaining(self.state)

    def command(self,channel,current):
        return self.library.current_runtime_command(self.state,channel,current)

    def check_deadline(self,tick):
        return self.library.current_runtime_check_deadline(self.state,tick)

    def receive(self,frame_index,words,available_tick,transfer_error=False):
        if len(words)!=2 or any(not isinstance(word,int) or not 0<=word<1<<Constants.rp2350_pio_word_bits for word in words):
            raise ValueError('Runtime API requires two unsigned FIFO words')
        channel,pwm=ctypes.c_uint(),ctypes.c_uint()
        latch=ctypes.c_uint64()
        measurement,voltage=ctypes.c_float(),ctypes.c_float()
        status=self.function(self.state,frame_index,*words,available_tick,int(transfer_error),
            ctypes.byref(channel),ctypes.byref(pwm),ctypes.byref(latch),ctypes.byref(measurement),ctypes.byref(voltage))
        return dict(status=status,channel=channel.value,pwm_compare=pwm.value,latch_tick=latch.value,
            measured_A=measurement.value,voltage_V=voltage.value,fault=self.fault)

    def receive_pair(self,first_frame,words,available_tick,transfer_error=False):
        if len(words)!=4 or any(not isinstance(word,int) or not 0<=word<1<<Constants.rp2350_pio_word_bits for word in words):
            raise ValueError('Runtime DMA service requires four unsigned FIFO words')
        buffer=(ctypes.c_uint*len(words))(*words)
        channel,pwm=ctypes.c_uint(),ctypes.c_uint()
        latch=ctypes.c_uint64()
        measurement,voltage=ctypes.c_float(),ctypes.c_float()
        status=self.pair_function(self.state,first_frame,buffer,available_tick,int(transfer_error),
            ctypes.byref(channel),ctypes.byref(pwm),ctypes.byref(latch),ctypes.byref(measurement),ctypes.byref(voltage))
        return dict(status=status,channel=channel.value,pwm_compare=pwm.value,latch_tick=latch.value,
            measured_A=measurement.value,voltage_V=voltage.value,fault=self.fault)


class CompiledReference:
    def __init__(self,initial_current,dtype,library,channel=0):
        if dtype!=np.float32:
            raise ValueError('Compiled reference requires single-precision state')
        self.state=np.full(len(current_loop.CurrentReference(0.).state),initial_current,dtype=np.float32)
        self.library=library
        self.channel=channel
        self.function=library.channel_reference_step
        self.function.argtypes=[ctypes.c_uint]+[ctypes.POINTER(ctypes.c_float)]*3
        self.function.restype=None
        self.targets=np.zeros(3,dtype=np.float32)
        self.advanced=np.zeros_like(self.state)
        self.sampled=False

    def current_and_rate(self,elapsed):
        if elapsed!=logic.controller_schedule()['channels'][self.channel]['actuation_s']:
            raise ValueError('Compiled reference requires the generated actuation phase')
        self.function(self.channel,*(array.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            for array in (self.state,self.targets,self.advanced)))
        self.sampled=True
        return self.targets[0],self.targets[1]

    def paired_reading(self,start):
        if not self.sampled or start!=0:
            raise ValueError('Compiled reference pair must belong to the current regulator sample')
        return self.targets[2]

    def at(self,elapsed):
        if not self.sampled or elapsed!=1/Fixed.current_regulator_rate:
            raise ValueError('Compiled reference advances exactly one sampled regulator interval')
        self.sampled=False
        return self.advanced.copy()


def build(output,host_compiler,arm_compiler):
    if output.exists():
        raise ValueError('Preserve existing compiled-regulator evidence')
    output.mkdir(parents=True)
    source=Path(__file__).with_name('regulator.c')
    header=output/'regulator_settings.h'
    header_text,approximation=regulator_settings()
    header.write_text(header_text)
    library=output/'regulator.so'
    commands=[
        [str(host_compiler),'-O2','-std=c11','-fno-fast-math','-ffp-contract=off','-fPIC','-shared',
         '-I',str(output),str(source),'-o',str(library)],
        [str(arm_compiler),'--target=arm-none-eabi','-mcpu=cortex-m33','-mfpu=fpv5-sp-d16','-mfloat-abi=hard',
         '-O2','-std=c11','-fno-fast-math','-ffp-contract=off','-I',str(output),'-S',str(source),'-o',str(output/'regulator.arm.s')],
        [str(arm_compiler),'--target=arm-none-eabi','-mcpu=cortex-m33','-mfpu=fpv5-sp-d16','-mfloat-abi=hard',
         '-O2','-std=c11','-fno-fast-math','-ffp-contract=off','-I',str(output),'-c',str(source),'-o',str(output/'regulator.arm.o')],
    ]
    logs=[]
    for command in commands:
        result=subprocess.run(command,capture_output=True,text=True,check=True)
        logs.append(dict(command=command,stdout=result.stdout,stderr=result.stderr))
    fingerprints=source_fingerprints(__file__,source,header,model.__file__)
    (output/'build.json').write_text(json.dumps(dict(source_sha256=fingerprints,commands=logs,
        binaries={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in (library,output/'regulator.arm.o')},
        toolchains={str(path):subprocess.run([str(path),'--version'],capture_output=True,text=True,check=True).stdout
            for path in (host_compiler,arm_compiler)}),indent=Fixed.report_precision)+'\n')
    return CompiledRegulator(library),approximation,fingerprints


def runtime_screen(backend, trace):
    schedule=logic.controller_schedule()
    _,pio=logic.sar_pio_program()
    channels=Fixed.electronics_acquisition_sar[2]
    interface,configuration_ticks=logic.configured_acquisition()
    power=logic.ControlGroupPower()
    ready_tick=power.wake(1<<Constants.rp2350_pio_word_bits)
    epoch=ready_tick+configuration_ticks
    stream=power.begin_acquisition(epoch,interface)
    runtime=CompiledRuntime(backend,epoch,configuration_verified=True)
    references=[CompiledReference(0.,np.float32,backend.library,channel) for channel in range(channels)]
    integrals=np.zeros(channels)
    amplifier_gain=next(row[1] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')
    sensitivity=amplifier_gain*Fixed.electronics_readout_trial_shunt/2
    scale=np.float32(Fixed.electronics_acquisition_sar[4]/(1<<Fixed.electronics_acquisition_sar[1])/sensitivity)
    bias=np.float32(Fixed.electronics_current_reference/sensitivity)
    frame_index=0
    maximum_measurement_difference=0.
    for row in trace:
        for channel in range(channels):
            if runtime.command(channel,float(row[1])):
                raise ValueError('Valid current command was rejected by the compiled runtime')
            reference=references[channel]
            reference.state[-1]=row[1]
            code=int(current_loop.acquisition.current_codes(row[4],sensitivity,Fixed.electronics_acquisition_sar[1],Fixed.electronics_acquisition_sar[4]))
            dma_words=[]
            pair_start=frame_index
            ready=epoch+(pair_start+Fixed.electronics_regulation_samples_per_carrier-1)*pio['next_sample_tick']+pio['rx_fifo_complete_tick']
            for sample in range(Fixed.electronics_regulation_samples_per_carrier):
                following=channel if sample==0 else (channel+1)%channels
                frame=interface.exchange(logic.sar_manual_command(following),np.full(channels,code,dtype=int))
                partial=Fixed.electronics_acquisition_sar_frame_bits-Constants.rp2350_pio_word_bits
                words=[frame>>partial,frame&((1<<partial)-1)]
                dma_words.extend(words)
                expected=stream.receive(frame_index,words,ready)
                frame_index+=1
                if sample==0:
                    if expected is not None:
                        raise ValueError('Regulation occurred before the paired acquisition completed')
                    continue
                result=runtime.receive_pair(pair_start,dma_words,ready)
                if result['status']!=1 or result['channel']!=expected['channel'] or result['latch_tick']!=expected['latch_tick']:
                    raise ValueError('Compiled acquisition sequence or actuation phase differs from the independent protocol model')
                measurement=float(np.float32(np.float32(code+.5)*scale-bias))
                maximum_measurement_difference=max(maximum_measurement_difference,abs(measurement-result['measured_A']))
                target,rate=reference.current_and_rate(schedule['channels'][channel]['actuation_s'])
                voltage,integral,_=backend(reference.paired_reading(0.)-measurement,integrals[channel],target,rate,backend.nominal,digital=True)
                reference.state=reference.at(1/Fixed.current_regulator_rate)
                integrals[channel]=integral
                half=schedule['pwm_TOP']+1
                expected_positive,expected_negative=power_stage.timer_leg_duties(voltage,backend.nominal['bus_nominal'])
                compare=(round(expected_negative*half)<<Constants.rp2350_pwm_compare_bits)|round(expected_positive*half)
                if result['pwm_compare']!=compare or result['voltage_V']!=voltage:
                    raise ValueError('Compiled acquisition-to-PWM path differs from the qualified compiled kernel')
                commit_tick=ready+schedule['allocated_compute_ticks']
                if not power.commit(commit_tick,result):
                    raise ValueError('Power sequencer rejected a valid compiled channel commit')
                if power.commits==channels:
                    power.arm(commit_tick,hardware_safe=True)
    if maximum_measurement_difference>Fixed.electronics_spice_current_tolerance:
        raise ValueError('Compiled conversion scaling differs from the reference')
    return dict(passed=True,frames_checked=frame_index,independent_channels=channels,clock_epoch_ticks=epoch,
        maximum_measurement_difference_A=maximum_measurement_difference,
        cold_start_recovery_ticks=ready_tick-(1<<Constants.rp2350_pio_word_bits),
        acquisition_enabled_after_complete_channel_round=power.enabled,
        hardware_execution_time_qualified=False,
        scope='Recorded-feedback replay through C frame validation, paired conversion scaling, per-channel reference/PI state and packed PWM compares; exact agreement with the independent serial state machine and compiled kernel. Behavioral wake/readback/fresh-channel sequencing wraps the runtime. Hardware arming is an injected healthy input; not a new physical trajectory, cold-rail transient or GPIO/DMA deployment.')


def run(output,host_compiler,arm_compiler):
    backend,approximation,fingerprints=build(output,host_compiler,arm_compiler)
    reference_factory=lambda initial,dtype:CompiledReference(initial,dtype,backend.library)
    report=current_loop.run(output/'current_control',digital=True,regulator=backend,reference_factory=reference_factory,extra_sources=tuple(fingerprints))
    trace=np.load(output/'current_control/traces.npz')['case_0_noise_0']
    runtime=runtime_screen(backend,trace)
    result=dict(runtime=runtime,status='compiled_current_regulation_averaged_screen',passed=report['passed'],
        trial_count=len(report['trials']),feedforward_approximation=approximation,host_execution_verified=True,arm_object_compiled=True,
        worst_case_execution_time_qualified=False,hardware_qualified=False,schedule=logic.controller_schedule(),
        limitations=[
            'Runs C reference filtering and the PI/feedforward/PWM kernel inside the existing averaged plant. C acquisition-to-PWM runtime is separately checked by recorded-feedback replay; GPIO/DMA deployment and execution time remain unverified.',
            'The same source compiles for Cortex-M33, but only the host binary is executed here. ARM library linkage, runtime and cycle timing are not established.',
            'The bounded interpolation removes the transcendental library call. Kernel, reference filtering, acquisition and PWM writes still need processor cycle-time verification.',
            'No interrupts, DMA contention, flash stalls or processor faults are simulated by this compilation check.'])
    (output/'summary.json').write_text(json.dumps(result,indent=Fixed.report_precision)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='Build and screen the embedded current-regulation kernel.')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--host-compiler',type=Path,required=True)
    parser.add_argument('--arm-compiler',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(run(args.output,args.host_compiler,args.arm_compiler)))

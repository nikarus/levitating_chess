import numpy as np

from model import Fixed, Constants
from methodology.electronics import acquisition


def controller_schedule():
    clock=Fixed.electronics_logic_clock
    schedule=acquisition.sar_schedule()
    clocks_per_bit=clock/Fixed.electronics_acquisition_sar_serial_clock
    carrier_ticks=clock/Fixed.driver_pwm_frequency
    regulator_ticks=clock/Fixed.current_regulator_rate
    if clock>Fixed.electronics_logic_clock_max or not all(float(value).is_integer() for value in
            (clocks_per_bit,carrier_ticks/2,regulator_ticks)):
        raise ValueError('Controller clocks require integer phase-correct PWM and serial timing within the rated clock')
    guard_ticks=int(np.ceil(clock*Fixed.electronics_acquisition_sar_frame_guard))
    frame_ticks=int(clocks_per_bit*Fixed.electronics_acquisition_sar_frame_bits+guard_ticks)
    sample_ticks=int(carrier_ticks/Fixed.electronics_regulation_samples_per_carrier)
    conversion_ticks=int(np.ceil(clock/Fixed.electronics_acquisition_sar[3]))
    compute_ticks=int(np.ceil(clock*Fixed.electronics_regulation_compute_time))
    if max(frame_ticks,conversion_ticks)>sample_ticks:
        raise ValueError('Integer-clock acquisition cannot finish before the next sample')
    result_ticks=sample_ticks+frame_ticks
    latch_ticks=int(np.ceil((result_ticks+compute_ticks)/carrier_ticks)*carrier_ticks)
    if latch_ticks/clock!=schedule['pwm_latch_s']:
        raise ValueError('Realizable clock schedule changes the already screened actuation phase')
    channels=Fixed.electronics_acquisition_sar[2]
    if channels*carrier_ticks>regulator_ticks or compute_ticks>carrier_ticks:
        raise ValueError('Acquisition or compute jobs overlap the next channel interval')
    preloads=np.arange(channels)*carrier_ticks*Fixed.electronics_pwm_interleave_span/channels
    if np.any(preloads!=np.floor(preloads)) or np.any(preloads>=carrier_ticks/2):
        raise ValueError('Phase-correct startup requires integer upward counter preloads below TOP')
    preloads=preloads.astype(int)
    latches=(np.ceil((result_ticks+compute_ticks+preloads)/carrier_ticks)*carrier_ticks-preloads).astype(int)
    return dict(status='integer_clock_controller_schedule_not_firmware_WCET_qualification',
        clock_Hz=clock,carrier_ticks=int(carrier_ticks),pwm_TOP=int(carrier_ticks/2)-1,
        serial_clock_ticks=int(clocks_per_bit),frame_guard_ticks=guard_ticks,
        frame_ticks=frame_ticks,conversion_minimum_ticks=conversion_ticks,
        allocated_compute_ticks=compute_ticks,
        maximum_compute_ticks_before_next_job_or_latch=int(min(carrier_ticks,np.min(latches)-result_ticks)),
        compute_utilization=channels*compute_ticks/regulator_ticks,
        channels=[dict(channel=index,sample_ticks=[int(index*carrier_ticks),int(index*carrier_ticks+sample_ticks)],
            result_ready_ticks=int(index*carrier_ticks+result_ticks),
            compute_ready_ticks=int(index*carrier_ticks+result_ticks+compute_ticks),
            latch_ticks=int(index*carrier_ticks+latches[index]),latch_offset_ticks=int(latches[index]),
            pwm_counter_initial=int(preloads[index]),pwm_phase_advance_ticks=int(preloads[index]),
            actuation_s=float(latches[index]/clock+Fixed.electronics_drv8962_input_delay)) for index in range(channels)],
        preserves_screened_actuation_phase=bool(np.all(latches==latch_ticks)),hardware_qualified=False,
        source='https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf',
        limitations=[
            'RP2350-family clock and PWM candidate only; board-level clock tolerance, pin assignment, firmware and processor worst-case execution time are not qualified.',
            'Phase-correct TOP and compare latches follow the datasheet; initialize upward counters with the recorded preloads and start all slices with one global enable. A carrier is assigned to each paired channel readout.',
            'The SPI guard is rounded upward to whole system clocks. Conversion, transfer and compute precede the same carrier latch used by the analog control screen.',
            'The allocated compute duration is a design assumption. The maximum cycle count is a scheduling deadline, not evidence that code meets it.',
            'Shared command transport, fault handling, calibration and Hall sensing need separate timing/resource allocations.'])


def sar_manual_command(channel):
    if not isinstance(channel,int) or not 0<=channel<Fixed.electronics_acquisition_sar[2]:
        raise ValueError('Manual ADC channel is outside the selected device')
    return Constants.ads8698_manual_command | channel<<Constants.ads8698_manual_channel_shift


class ControlGroupPower:
    def __init__(self):
        self.mode='cold'
        self.now=0
        self.ready_tick=None
        self.disabled_tick=0
        self.sleep_tick=0
        self.acquisition_epoch=None
        self.last_latch=None
        self.commits=0
        self.driver_awake=False
        self.enabled=False

    def advance(self,tick,power_healthy=True,driver_healthy=True):
        if not isinstance(tick,int) or tick<self.now:
            raise ValueError('Power sequencing requires monotonic integer controller ticks')
        self.now=tick
        if not power_healthy or not driver_healthy:
            self.disable(tick)
            self.mode='fault'

    def wake(self,tick):
        self.advance(tick)
        if self.mode not in ('cold','standby','power_down'):
            raise ValueError('Wake requires an inhibited cold or sleeping group; faults require explicit reset')
        if self.mode!='cold' and tick<self.sleep_tick+int(np.ceil(Fixed.electronics_drv8962_sleep_time*Fixed.electronics_logic_clock)):
            raise ValueError('Driver nSLEEP must remain low for its specified minimum duration')
        driver_wait=Fixed.electronics_drv8962_turn_on_time if self.mode=='cold' else Fixed.electronics_drv8962_wake_time
        adc_wait=Fixed.electronics_sar_wake_times[self.mode!='standby']
        self.ready_tick=tick+int(np.ceil(max(driver_wait,adc_wait)*Fixed.electronics_logic_clock))
        self.mode='waking'
        self.driver_awake=True
        self.commits=0
        self.last_latch=None
        return self.ready_tick

    def begin_acquisition(self,tick,interface):
        self.advance(tick)
        if self.mode!='waking' or tick<self.ready_tick:
            raise ValueError('Driver and converter recovery must complete before acquisition')
        stream=SarAcquisitionStream(interface,tick)
        self.acquisition_epoch=tick
        self.mode='acquiring'
        return stream

    def commit(self,tick,result):
        self.advance(tick)
        schedule=controller_schedule()
        if (self.mode not in ('acquiring','active') or result['status']!=1 or result['fault'] or
                result['channel']!=self.commits%Fixed.electronics_acquisition_sar[2] or tick>=result['latch_tick'] or
                result['latch_tick']!=self.acquisition_epoch+self.commits*schedule['carrier_ticks']+schedule['channels'][self.commits%Fixed.electronics_acquisition_sar[2]]['latch_offset_ticks']):
            self.disable(tick)
            self.mode='fault'
            return False
        self.last_latch=result['latch_tick']
        self.commits+=1
        return True

    def arm(self,tick,hardware_safe):
        self.advance(tick)
        if (self.mode!='acquiring' or self.commits<Fixed.electronics_acquisition_sar[2] or
                not hardware_safe or tick>=self.last_latch):
            raise ValueError('Arming requires fresh feedback from every channel and an independently armed hardware gate')
        self.mode='active'
        self.enabled=True

    def disable(self,tick):
        self.advance(tick)
        if self.enabled:
            self.disabled_tick=tick
        self.enabled=False
        if self.mode in ('active','acquiring'):
            self.mode='inhibited'

    def sleep(self,tick,mode):
        self.advance(tick)
        wait=int(np.ceil(Fixed.electronics_drv8962_enable_delay*Fixed.electronics_logic_clock))
        if (mode not in ('standby','power_down') or self.enabled or self.mode not in ('inhibited','waking') or
                tick<self.disabled_tick+wait):
            raise ValueError('Disable all bridges and allow EN propagation before sleeping the group')
        self.mode=mode
        self.sleep_tick=tick
        self.driver_awake=False
        self.ready_tick=None
        self.last_latch=None
        self.commits=0

    def reset_fault(self,tick,power_healthy,driver_healthy):
        self.advance(tick,power_healthy,driver_healthy)
        if self.mode!='fault' or not power_healthy or not driver_healthy:
            raise ValueError('Fault reset requires an inhibited group and healthy independent fault inputs')
        self.mode='power_down'
        self.sleep_tick=tick
        self.driver_awake=False


def configured_acquisition():
    channels=Fixed.electronics_acquisition_sar[2]
    interface=SarSerialInterface()
    sequence=sar_protocol_screen()['initialization']
    for row in sequence:
        if interface.exchange(int(row['command_hex'],16),[0]*channels)!=int(row['reply_hex'],16):
            raise ValueError('ADC readback failed')
    interface.exchange(sar_manual_command(0),[0]*channels)
    return interface,(len(sequence)+1)*controller_schedule()['frame_ticks']



def sar_register_command(address,value,write=True):
    if not 0<address<1<<(Constants.ads8698_command_bits-Constants.ads8698_address_shift-1):
        raise ValueError('Program register address is outside the non-command range')
    if not 0<=value<1<<Constants.ads8698_register_bits:
        raise ValueError('Program register value exceeds its data word')
    return address<<Constants.ads8698_address_shift | int(write)<<Constants.ads8698_write_bit | value


def sar_frame_valid(frame,expected_channel):
    frame_bits=Fixed.electronics_acquisition_sar_frame_bits
    trailing=frame_bits-Constants.ads8698_command_bits-Fixed.electronics_acquisition_sar[1]-Constants.ads8698_channel_tag_bits
    if not isinstance(frame,int) or trailing<0 or not 0<=frame<1<<frame_bits:
        return False
    channel=(frame>>trailing)&((1<<Constants.ads8698_channel_tag_bits)-1)
    return not (frame>>(frame_bits-Constants.ads8698_command_bits) or frame&((1<<trailing)-1) or channel!=expected_channel)


def sar_decode_frame(frame,expected_channel):
    if not sar_frame_valid(frame,expected_channel):
        raise ValueError('Malformed or stale-channel ADC frame; do not update the regulator')
    trailing=Fixed.electronics_acquisition_sar_frame_bits-Constants.ads8698_command_bits-Fixed.electronics_acquisition_sar[1]
    return (frame>>trailing)&((1<<Fixed.electronics_acquisition_sar[1])-1)


class SarAcquisitionStream:
    def __init__(self,interface,epoch_tick):
        registers={Constants.ads8698_feature_register:Constants.ads8698_channel_tag_format,
            **{Constants.ads8698_first_range_register+index:Constants.ads8698_unipolar_range_code
               for index in range(Fixed.electronics_acquisition_sar[2])}}
        if interface.registers!=registers or interface.selected_channel!=0 or not isinstance(epoch_tick,int):
            raise ValueError('Configure, read back and prime channel zero before starting acquisition')
        self.schedule=controller_schedule()
        _,self.timing=sar_pio_program()
        self.epoch_tick=epoch_tick
        self.frame_index=0
        self.paired_codes=[]
        self.fault=None
        self.last_service_tick=epoch_tick

    @property
    def regulation_allowed(self):
        return self.fault is None

    def deadline_tick(self):
        pair=self.frame_index//Fixed.electronics_regulation_samples_per_carrier
        return (self.epoch_tick+pair*self.schedule['carrier_ticks']+
            self.schedule['channels'][pair%Fixed.electronics_acquisition_sar[2]]['latch_offset_ticks']-self.schedule['allocated_compute_ticks'])

    def inhibit(self,reason):
        self.fault=reason
        self.paired_codes.clear()
        return None

    def check_deadline(self,now_tick):
        if self.fault is None and now_tick>self.deadline_tick():
            self.inhibit('acquisition_deadline_missed')
        return self.regulation_allowed

    def receive(self,frame_index,words,available_tick,pio_stalled=False,dma_error=False):
        if self.fault is not None:
            return None
        if pio_stalled or dma_error:
            return self.inhibit('pio_stall' if pio_stalled else 'dma_error')
        if frame_index!=self.frame_index:
            return self.inhibit('missing_or_repeated_frame')
        if available_tick<self.last_service_tick:
            return self.inhibit('service_clock_reversed')
        earliest=self.epoch_tick+frame_index*self.timing['next_sample_tick']+self.timing['rx_fifo_complete_tick']
        if available_tick<earliest:
            return self.inhibit('incomplete_conversion')
        if not self.check_deadline(available_tick):
            return None
        partial_bits=Fixed.electronics_acquisition_sar_frame_bits-Constants.rp2350_pio_word_bits
        if len(words)!=2 or any(not isinstance(word,int) for word in words) or not (
                0<=words[0]<1<<Constants.rp2350_pio_word_bits and 0<=words[1]<1<<partial_bits):
            return self.inhibit('invalid_fifo_frame')
        frame=(words[0]<<partial_bits)|words[1]
        samples=Fixed.electronics_regulation_samples_per_carrier
        channel=(frame_index//samples)%Fixed.electronics_acquisition_sar[2]
        if not sar_frame_valid(frame,channel):
            return self.inhibit('invalid_channel_or_frame')
        self.paired_codes.append(sar_decode_frame(frame,channel))
        self.last_service_tick=available_tick
        self.frame_index+=1
        if len(self.paired_codes)<samples:
            return None
        pair_start=self.epoch_tick+(frame_index//samples)*self.schedule['carrier_ticks']
        result=dict(channel=channel,mean_code=sum(self.paired_codes)/samples,
            sample_ticks=[pair_start+i*self.timing['next_sample_tick'] for i in range(samples)],
            latch_tick=pair_start+self.schedule['channels'][channel]['latch_offset_ticks'])
        self.paired_codes.clear()
        return result


class SarSerialInterface:
    def __init__(self):
        self.selected_channel=None
        self.registers={Constants.ads8698_feature_register:0,
            **{Constants.ads8698_first_range_register+index:0 for index in range(Fixed.electronics_acquisition_sar[2])}}

    def exchange(self,command,codes):
        if not isinstance(command,int) or not 0<=command<1<<Constants.ads8698_command_bits:
            raise ValueError('Invalid serial command word')
        frame_bits=Fixed.electronics_acquisition_sar_frame_bits
        if not command&(1<<(Constants.ads8698_command_bits-1)):
            address=command>>Constants.ads8698_address_shift
            if address not in self.registers:
                raise ValueError('Program register is outside the implemented acquisition configuration')
            if command&(1<<Constants.ads8698_write_bit):
                self.registers[address]=command&((1<<Constants.ads8698_register_bits)-1)
            return self.registers[address]<<(frame_bits-Constants.ads8698_command_bits-Constants.ads8698_register_bits)
        channel=(command-Constants.ads8698_manual_command)>>Constants.ads8698_manual_channel_shift
        if command!=sar_manual_command(channel):
            raise ValueError('Unsupported ADC operating command')
        if self.registers[Constants.ads8698_feature_register]!=Constants.ads8698_channel_tag_format or any(
                self.registers[Constants.ads8698_first_range_register+index]!=Constants.ads8698_unipolar_range_code
                for index in range(Fixed.electronics_acquisition_sar[2])):
            raise ValueError('ADC range and tagged-output configuration must be verified before acquisition')
        bits=Fixed.electronics_acquisition_sar[1]
        codes=np.asarray(codes)
        if codes.shape!=(Fixed.electronics_acquisition_sar[2],) or not np.issubdtype(codes.dtype,np.integer) or np.any(codes<0) or np.any(codes>=1<<bits):
            raise ValueError('Converter input must provide one valid code per physical channel')
        previous=self.selected_channel
        self.selected_channel=channel
        if previous is None:
            return None
        trailing=frame_bits-Constants.ads8698_command_bits-bits-Constants.ads8698_channel_tag_bits
        if trailing<0:
            raise ValueError('Configured frame truncates the ADC channel tag')
        return (int(codes[previous])<<Constants.ads8698_channel_tag_bits | previous)<<trailing


def sar_pio_program():
    schedule=controller_schedule()
    bit_ticks=schedule['serial_clock_ticks']
    frame_bits=Fixed.electronics_acquisition_sar_frame_bits
    side_count=len(('SCLK','CS'))
    max_delay=(1<<(Constants.rp2350_pio_delay_side_bits-side_count))-1
    immediate_max=(1<<Constants.rp2350_pio_immediate_bits)-1
    half=bit_ticks//2
    if bit_ticks%2 or not 3<=half<=max_delay+1 or frame_bits%2 or not 0<=frame_bits//2-1<=immediate_max:
        raise ValueError('Serial format does not fit the two-bit PIO loop and instruction delays')
    used_ticks=1+frame_bits*bit_ticks+2
    idle_ticks=schedule['carrier_ticks']//Fixed.electronics_regulation_samples_per_carrier-used_ticks
    idle=None
    for stride in range(1,max_delay+2):
        iterations=min(immediate_max+1,(idle_ticks-1)//stride)
        setup=idle_ticks-iterations*stride
        if iterations>0 and 1<=setup<=max_delay+1:
            idle=(iterations,setup,stride)
            break
    if idle is None:
        raise ValueError('Inter-conversion delay does not fit the PIO loop')
    iterations,setup,stride=idle
    lines=['.program flying_chess_acquisition',f'.side_set {side_count}', '.wrap_target',
        f'set x, {frame_bits//2-1} side 0','bits:',
        f'out pins, 1 side 1 [{half-1}]',f'nop side 0 [{half-3}]','in pins, 1 side 0','nop side 0',
        f'out pins, 1 side 1 [{half-1}]',f'nop side 0 [{half-3}]','in pins, 1 side 0','jmp x-- bits side 0',
        'push block side 2','pull block side 2',f'set y, {iterations-1} side 2 [{setup-1}]',
        'idle:',f'jmp y-- idle side 2 [{stride-1}]','.wrap']
    clock=Fixed.electronics_logic_clock
    falling=1+half+np.arange(frame_bits)*bit_ticks
    cs_high=1+frame_bits*bit_ticks
    next_cs=used_ticks+idle_ticks
    available=np.array([falling[0],cs_high-falling[-1],next_cs-cs_high,half,half,
        falling[Constants.ads8698_command_bits-1],next_cs-falling[Constants.ads8698_command_bits-1]])/clock
    limits=np.array(Constants.ads8698_serial_timing)
    if np.any(available<limits):
        raise ValueError('PIO instruction timing violates converter timing requirements')
    details=dict(instruction_count=sum(not line.startswith('.') and not line.endswith(':') for line in lines),
        first_falling_clock_tick=int(falling[0]),last_falling_clock_tick=int(falling[-1]),
        cs_high_tick=cs_high,next_sample_tick=next_cs,serial_sample_after_clock_assignment_ticks=half-2,
        rx_fifo_complete_tick=cs_high+1,allocated_frame_ticks=schedule['frame_ticks'],
        datasheet_timing_available_s=available.tolist(),datasheet_timing_minimum_s=limits.tolist(),
        configuration=dict(shift_left=True,autopull=False,autopush=True,push_threshold=Constants.rp2350_pio_word_bits,
            initial_osr_command_shift=Constants.rp2350_pio_word_bits-Constants.ads8698_command_bits),
        limitations=[
            'Requires initial OSR preloading, empty ISR, prefetched TX data and continuous RX draining. FIFO stalls invalidate the timing and must inhibit regulation.',
            'Checks instruction-space edge timing, not pad/PCB delay, input synchronizer timing, oscillator tolerance or initial phase synchronization to PWM.',
            'Each complete frame produces one full RX word and one partial word; DMA/CPU must combine them before decoding. No compiled current-regulator execution time is established.'])
    return '\n'.join(lines)+'\n',details


def sar_protocol_screen():
    interface=SarSerialInterface()
    channels=Fixed.electronics_acquisition_sar[2]
    bits=Fixed.electronics_acquisition_sar[1]
    codes=np.rint(np.linspace(0,(1<<bits)-1,channels)).astype(int)
    initialization=[]
    for address,value in [(Constants.ads8698_feature_register,Constants.ads8698_channel_tag_format),
            *[(Constants.ads8698_first_range_register+index,Constants.ads8698_unipolar_range_code) for index in range(channels)]]:
        for write in (True,False):
            command=sar_register_command(address,value if write else 0,write)
            reply=interface.exchange(command,codes)
            expected=value<<(Fixed.electronics_acquisition_sar_frame_bits-Constants.ads8698_command_bits-Constants.ads8698_register_bits)
            if reply!=expected:
                raise ValueError('ADC configuration readback differs from the required value')
            initialization.append(dict(command_hex=f'{command:04X}',reply_hex=f'{reply:010X}',write=write))
    if interface.exchange(sar_manual_command(0),codes) is not None:
        raise ValueError('First manual frame must be discarded during pipeline priming')
    schedule=controller_schedule()
    frames=[]
    for channel in schedule['channels']:
        index=channel['channel']
        for sample,tick in enumerate(channel['sample_ticks']):
            next_channel=index if sample==0 else (index+1)%channels
            command=sar_manual_command(next_channel)
            frame=interface.exchange(command,codes)
            code=sar_decode_frame(frame,index)
            if code!=codes[index]:
                raise ValueError('Received ADC data belongs to another physical input')
            frames.append(dict(sample_tick=tick,channel=index,command_hex=f'{command:04X}',
                received_hex=f'{frame:010X}',code=code,next_selected_channel=next_channel))
            codes=(1<<bits)-1-codes
    wrap=interface.exchange(sar_manual_command(0),codes)
    if sar_decode_frame(wrap,0)!=codes[0]:
        raise ValueError('ADC pipeline failed across the regulator-period boundary')
    return dict(status='ADS8698_manual_pipeline_and_register_protocol_screen',passed=True,
        initialization=initialization,discarded_priming_command_hex=f'{sar_manual_command(0):04X}',frames=frames,
        schedule=schedule,hardware_qualified=False,
        source='https://www.ti.com/lit/ds/symlink/ads8698.pdf',limitations=[
            'Behavioral model of the used manual-conversion and configuration-register subset. Converter analog physics and register electrical timing are not simulated here.',
            'CS falling samples the previously selected channel. The command in that frame chooses the next conversion. Both members of a pair must therefore be scheduled explicitly.',
            'The host captures SDO on falling clocks: the conversion occupies capture clocks 17 through 34 and the channel tag follows. Configuration writes are echoed and independently read back before priming.',
            'Channel tags reject wrong-channel frames but are not a CRC; corrupted conversion bits can remain undetected. Board signal integrity, PIO/DMA code, interrupt handling and execution time remain unqualified.'])




def sar_stream_screen():
    protocol=sar_protocol_screen()
    def initialized():
        interface=SarSerialInterface()
        codes=[0]*Fixed.electronics_acquisition_sar[2]
        for row in protocol['initialization']:
            interface.exchange(int(row['command_hex'],16),codes)
        interface.exchange(sar_manual_command(0),codes)
        return SarAcquisitionStream(interface,0)
    stream=initialized()
    _,timing=sar_pio_program()
    partial=Fixed.electronics_acquisition_sar_frame_bits-Constants.rp2350_pio_word_bits
    outputs=[]
    for index,row in enumerate(protocol['frames']):
        frame=int(row['received_hex'],16)
        result=stream.receive(index,[frame>>partial,frame&((1<<partial)-1)],row['sample_tick']+timing['rx_fifo_complete_tick'])
        if result is not None:
            outputs.append(result)
    if len(outputs)!=Fixed.electronics_acquisition_sar[2] or not stream.regulation_allowed:
        raise ValueError('Valid acquisition stream did not issue one measurement per coil')
    first=int(protocol['frames'][0]['received_hex'],16)
    words=[first>>partial,first&((1<<partial)-1)]
    available=timing['rx_fifo_complete_tick']
    cases={
        'pio_stall':dict(frame_index=0,words=words,available_tick=available,pio_stalled=True),
        'dma_error':dict(frame_index=0,words=words,available_tick=available,dma_error=True),
        'missing_or_repeated_frame':dict(frame_index=1,words=words,available_tick=available),
        'incomplete_conversion':dict(frame_index=0,words=words,available_tick=available-1),
        'invalid_fifo_frame':dict(frame_index=0,words=words[:1],available_tick=available),
        'invalid_channel_or_frame':dict(frame_index=0,words=[words[0],words[1]|1],available_tick=available),
        'acquisition_deadline_missed':dict(frame_index=0,words=words,available_tick=initialized().deadline_tick()+1),
    }
    failures=[]
    for reason,arguments in cases.items():
        candidate=initialized()
        if candidate.receive(**arguments) is not None or candidate.fault!=reason:
            raise ValueError('Acquisition failure did not inhibit regulation')
        if candidate.receive(0,words,available) is not None or candidate.regulation_allowed:
            raise ValueError('Acquisition fault cleared without explicit reinitialization')
        failures.append(dict(injected=reason,latched_fault=candidate.fault,regulation_allowed=candidate.regulation_allowed))
    silent=initialized()
    if silent.check_deadline(silent.deadline_tick()+1):
        raise ValueError('Silent acquisition loss did not inhibit regulation')
    return dict(status='behavioral_acquisition_stream_fault_screen',passed=True,measurements=outputs,
        injected_failures=failures,silent_loss_detected=True,hardware_qualified=False,
        limitations=[
            'Tests host-side frame assembly and latched inhibition semantics, not a deployed DMA/interrupt implementation or independent hardware shutdown.',
            'Frame sequence and service timestamps must come from trustworthy DMA counters and the hardware clock; PIO stall and DMA error flags must be observed.',
            'Deadline expiration needs an independent scheduled watchdog call; a stopped processor cannot enforce this Python state machine.',
            'Valid-looking conversion-bit corruption is not detected by channel tags. Analog alarms, signal integrity and disturbance rejection remain separate.',
            'A fault prevents new regulation results. Existing PWM must be disabled by the separate hardware enable gate; its response time is not established here.'])

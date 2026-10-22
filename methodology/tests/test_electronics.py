from pathlib import Path
import json
import subprocess
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from xml.etree import ElementTree

import numpy as np

from model import Fixed
from methodology import electrical
from methodology.electronics.acquisition import (
    acquisition_response, cascade_noise_covariance, filter_periodic, phase_sample_means, rc_noise_covariance,
    weighted_acquisition_response)
from methodology.electronics.acquisition_stress import (
    causal_burst_average, high_gate_average)
from methodology.electronics.power_stage import unipolar_leg_duties
from methodology.electronics.readout import (
    edge_intervals, execute_vendor_case, low_side_window, readout_metrics,
    return_leg_blank_interval, validate_vendor_transient)
from methodology.electronics.review import validate_integrated_channel


class IntegratedCircuitConnectivityTests(unittest.TestCase):
    def test_controller_pin_map_and_regulator_polarity(self):
        from methodology.electronics.review import validate_controller

        evidence=Path(__file__).resolve().parents[1]/'results/cad/electronics/local_controller/netlist.xml'
        netlist=ElementTree.parse(evidence).getroot()
        self.assertTrue(validate_controller(netlist)['passed'])
        pins=[node for node in netlist.findall('nets/net/node') if node.attrib['ref']=='L20']
        for pin in pins:
            pin.attrib['pin']='2' if pin.attrib['pin']=='1' else '1'
        with self.assertRaises(ValueError):
            validate_controller(netlist)

    def test_group_feedback_and_pwm_are_independent(self):
        from methodology.electronics.review import validate_control_group

        evidence=Path(__file__).resolve().parents[1]/'results/cad/electronics/control_group/netlist.xml'
        netlist=ElementTree.parse(evidence).getroot()
        self.assertTrue(validate_control_group(netlist)['passed'])
        pin=next(node for node in netlist.findall('nets/net/node')
            if node.attrib['ref']=='U620' and node.attrib['pin']=='77')
        pin.attrib['pin']='79'
        with self.assertRaises((ValueError,KeyError)):
            validate_control_group(netlist)

    def test_shared_adc_internal_reference_and_channel_map(self):
        from methodology.electronics.review import validate_acquisition

        evidence=Path(__file__).resolve().parents[1]/'results/cad/electronics/shared_acquisition/netlist.xml'
        netlist=ElementTree.parse(evidence).getroot()
        self.assertTrue(validate_acquisition(netlist)['passed'])
        pin=next(node for node in netlist.findall('nets/net/node')
            if node.attrib['ref']=='U10' and node.attrib['pin']=='4')
        pin.attrib['pin']='5'
        with self.assertRaises(ValueError):
            validate_acquisition(netlist)

    def setUp(self):
        evidence = Path(__file__).resolve().parents[1] / 'results/cad/electronics/low_side_channel/netlist.xml'
        self.netlist = ElementTree.parse(evidence).getroot()

    def pin(self, reference, number):
        return next(node for node in self.netlist.findall('nets/net/node')
                    if node.attrib['ref'] == reference and node.attrib['pin'] == number)

    def test_reviewed_kicad_export_matches_ddv_and_sense_connections(self):
        self.assertTrue(validate_integrated_channel(self.netlist)['passed'])

    def test_bottom_cooled_package_pin_swap_is_rejected(self):
        ground = self.pin('U1', '1')
        pump = self.pin('U1', '22')
        ground.attrib['pin'], pump.attrib['pin'] = '22', '1'
        with self.assertRaises(ValueError):
            validate_integrated_channel(self.netlist)

    def test_reversed_current_readout_is_rejected(self):
        positive = self.pin('U2', '3')
        negative = self.pin('U2', '4')
        positive.attrib['pin'], negative.attrib['pin'] = '1', '8'
        with self.assertRaises(ValueError):
            validate_integrated_channel(self.netlist)

    def test_missing_charge_pump_connection_is_rejected(self):
        for net in self.netlist.findall('nets/net'):
            for node in list(net.findall('node')):
                if node.attrib['ref'] == 'C7' and node.attrib['pin'] == '2':
                    net.remove(node)
        with self.assertRaises(ValueError):
            validate_integrated_channel(self.netlist)

    def test_incomplete_orderable_amplifier_identity_is_rejected(self):
        amplifier = self.netlist.find("components/comp[@ref='U2']")
        amplifier.find("fields/field[@name='MPN']").text = 'INA181'
        with self.assertRaises(ValueError):
            validate_integrated_channel(self.netlist)

    def test_driver_reference_cannot_be_tied_to_five_volt_dvdd(self):
        node=self.pin('U1','34')
        parent=next(net for net in self.netlist.findall('nets/net') if node in list(net))
        parent.remove(node)
        next(net for net in self.netlist.findall('nets/net') if net.attrib['name']=='/DVDD').append(node)
        with self.assertRaises(ValueError):
            validate_integrated_channel(self.netlist)

    def test_native_passives_match_model_including_split_filter_and_shunts(self):
        from methodology.electronics.review import native_component_parameters

        schematic=Path(__file__).resolve().parents[1]/'electronics/channel.kicad_sch'
        parameters=native_component_parameters(schematic)
        self.assertEqual(parameters['R5'],('electronics_readout_trial_shunt',2.))
        self.assertEqual(parameters['C12'],('electronics_acquisition_filter_capacitance',2.))


class ReadoutScreenTests(unittest.TestCase):
    def test_lifecycle_initial_reference_matches_its_physical_recording(self):
        import json
        from methodology.electronics.switching_loop import periodic_initial

        folder=Path(__file__).resolve().parents[1]/'results/electronics/low_side_readout/physical_return_paths/settled_bridge_readout'
        evidence=json.loads((folder/'summary.json').read_text())
        for fraction in (-1.,0.,1.):
            _,settings,mean,path=periodic_initial(folder,1,fraction)
            expected=next(row for row in evidence['trials'] if row['settings']['requested_current_A']==fraction*Fixed.driver_channel_current)
            self.assertEqual(settings,expected['settings'])
            self.assertEqual(mean,expected['checks'][-1]['actual_mean_A'])
            self.assertTrue(path.is_file())

    def test_arbitrary_filter_covariance_matches_analytic_repeated_poles(self):
        for order in (1,2,4):
            for lag in (0.,3.125e-6,50e-6):
                actual=cascade_noise_covariance([8000.]*order,lag)
                expected=rc_noise_covariance(8000.,order,lag)
                self.assertAlmostEqual(actual,float(expected),places=7)

    def test_leg_duties_reproduce_signed_terminal_voltage(self):
        for voltage in (-18., 0., 18.):
            positive, negative = unipolar_leg_duties(voltage, 24.)
            self.assertAlmostEqual((positive - negative) * 24., voltage)
            self.assertAlmostEqual(positive + negative, 1.)
            self.assertEqual(unipolar_leg_duties(-voltage, 24.), (negative, positive))

    def test_edge_spacing_counts_carrier_edges_not_differential_ripple(self):
        intervals = edge_intervals(0., 24., 160000.)
        self.assertEqual(intervals[0], intervals[1])
        self.assertAlmostEqual(intervals[0][0], 3.125e-6)
        intervals = edge_intervals(18., 24., 160000.)
        self.assertAlmostEqual(min(intervals[0]), 0.78125e-6)
        self.assertAlmostEqual(sum(intervals[0]), 1 / 160000.)

    def test_invalid_or_static_edges_are_not_assumed_to_be_valid_samples(self):
        for voltage, bus, rate in ((24., 24., 160000.), (25., 24., 160000.), (0., 24., 0.)):
            with self.assertRaises(ValueError):
                edge_intervals(voltage, bus, rate)

    def test_lower_shunt_and_higher_gain_keep_range_but_change_heat_and_noise(self):
        original = readout_metrics(10., 0.02, 38e-9, 0.01)
        alternative = readout_metrics(20., 0.01, 38e-9, 0.01)
        self.assertEqual(original['nominal_output_range_V'], alternative['nominal_output_range_V'])
        self.assertEqual(original['minimum_ideal_full_supply_adc_bits'],
                         alternative['minimum_ideal_full_supply_adc_bits'])
        self.assertAlmostEqual(alternative['peak_shunt_loss_W'], original['peak_shunt_loss_W'] / 2)
        self.assertAlmostEqual(alternative['typical_white_noise_only_max_ENBW_Hz'],
                               original['typical_white_noise_only_max_ENBW_Hz'] / 4)

    def test_converter_bits_are_sufficient_and_minimal_for_assumed_full_span(self):
        result = readout_metrics(10., 0.02, 62e-9, 0.0032)
        bits = result['minimum_ideal_full_supply_adc_bits']
        quantum = Fixed.electronics_analog_supply / result['sensitivity_V_per_A'] / 2 ** bits
        self.assertLessEqual(quantum, electrical.current_quantum())
        self.assertGreater(2 * quantum, electrical.current_quantum())
        self.assertFalse(result['total_readout_qualified'])

    def test_low_side_window_accounts_for_each_timing_loss(self):
        window = low_side_window(0.5, Fixed.electronics_drv8962_deadtime)
        self.assertAlmostEqual(window['ideal_low_interval_s'], 1 / (2 * Fixed.driver_pwm_frequency))
        self.assertAlmostEqual(window['settled_input_to_sample_s'],
            window['ideal_low_interval_s'] - Fixed.electronics_drv8962_deadtime
            - Fixed.driver_switching_time - Fixed.electronics_readout_sample_guard)

    def test_second_leg_improves_extreme_window_but_not_zero_command(self):
        for voltage in (-18., 18.):
            windows = [low_side_window(duty, 0.)['ideal_low_interval_s']
                       for duty in unipolar_leg_duties(voltage, 24.)]
            self.assertGreater(max(windows), 1 / (2 * Fixed.driver_pwm_frequency))
            self.assertLess(min(windows), 1 / (2 * Fixed.driver_pwm_frequency))
        zero_windows = [low_side_window(duty, 0.)
                        for duty in unipolar_leg_duties(0., 24.)]
        self.assertEqual(zero_windows[0], zero_windows[1])

    def test_invalid_low_side_timing_is_rejected(self):
        for duty, deadtime in ((0., 0.), (1., 0.), (0.5, -1.)):
            with self.assertRaises(ValueError):
                low_side_window(duty, deadtime)

    def test_partial_vendor_transient_is_rejected_even_if_process_exits_successfully(self):
        stop = Fixed.electronics_spice_cycles / Fixed.driver_pwm_frequency
        complete = np.array([[0., 0., 0.], [stop, 0., 0.]])
        validate_vendor_transient(complete)
        partial = complete.copy()
        partial[-1, 0] = stop / 2
        with self.assertRaises(ValueError):
            validate_vendor_transient(partial)
        nonfinite = complete.copy()
        nonfinite[-1, -1] = np.nan
        with self.assertRaises(ValueError):
            validate_vendor_transient(nonfinite)

    def test_ngspice_abort_with_zero_exit_status_is_preserved_and_rejected(self):
        aborted = subprocess.CompletedProcess(['ngspice'], 0, 'tran simulation(s) aborted\n', '')
        with TemporaryDirectory() as directory:
            folder = Path(directory) / 'case'
            with patch('methodology.electronics.readout.subprocess.run', return_value=aborted):
                with self.assertRaises(ValueError):
                    execute_vendor_case(folder, 'ngspice', 'test circuit', 'vendor.lib')
            self.assertIn('aborted', (folder / 'ngspice.log').read_text())

    def test_return_pair_conduction_areas_sum_to_one_carrier_in_both_directions(self):
        period = 1 / Fixed.driver_pwm_frequency
        for voltage in (-18., 0., 18.):
            for current in (-Fixed.driver_peak_current, Fixed.driver_peak_current):
                duties = unipolar_leg_duties(voltage, 24.)
                widths = [return_leg_blank_interval(duty, polarity*current, Fixed.electronics_drv8962_deadtime)[1]
                          for duty, polarity in zip(duties, (-1, 1))]
                integrated_conduction = sum(period-width-Fixed.driver_switching_time for width in widths)
                self.assertAlmostEqual(integrated_conduction / period, 1.)

    def test_equal_deadtime_extends_source_conduction_and_shortens_sink_conduction(self):
        duty = .5
        source_start, source_width = return_leg_blank_interval(duty, -1., Fixed.electronics_drv8962_deadtime)
        sink_start, sink_width = return_leg_blank_interval(duty, 1., Fixed.electronics_drv8962_deadtime)
        self.assertAlmostEqual(source_start-sink_start, Fixed.electronics_drv8962_deadtime)
        self.assertAlmostEqual(sink_width-source_width, 2*Fixed.electronics_drv8962_deadtime)


class AcquisitionTests(unittest.TestCase):
    def test_ADC_codes_and_midpoint_decode_share_the_straight_binary_transfer(self):
        from methodology.electronics import acquisition

        span=Fixed.electronics_acquisition_sar[4]
        bits=Fixed.electronics_acquisition_sar[1]
        sensitivity=acquisition.sensing_filter()['sensitivity_V_per_A']
        quantum=span/(1<<bits)
        voltage=quantum*np.array([.25,.75,1.25,1.75,(1<<bits)-.25])
        current=(voltage-Fixed.electronics_current_reference)/sensitivity
        codes=acquisition.current_codes(current,sensitivity,bits,span)
        np.testing.assert_array_equal(codes,[0,0,1,1,(1<<bits)-1])
        decoded=acquisition.current_quantizer(current,sensitivity,bits,span)
        np.testing.assert_allclose(decoded,((codes+.5)*quantum-Fixed.electronics_current_reference)/sensitivity)
        self.assertLessEqual(np.max(np.abs(decoded-current)),quantum/(2*sensitivity))

    def test_finite_gate_edges_and_leg_delay_preserve_commanded_duty(self):
        count = 2048
        step = 1/(Fixed.driver_pwm_frequency*count)
        time = np.arange(count)*step
        for duty in (.1,.5,.9):
            for skew in (-50e-9,0.,50e-9):
                self.assertAlmostEqual(high_gate_average(time,step,duty,skew).mean(),duty,places=10)

    def test_causal_burst_preserves_baseband_phase_and_rejects_carrier(self):
        frequency = Fixed.driver_pwm_frequency/8
        count = 8192
        time = np.arange(count)/(count*frequency)
        signal = np.cos(2*np.pi*frequency*time)+np.cos(2*np.pi*Fixed.driver_pwm_frequency*time)
        reported = causal_burst_average(signal,2,frequency)
        expected = (np.cos(2*np.pi*frequency*time)+np.cos(2*np.pi*frequency*(time-1/(2*Fixed.driver_pwm_frequency))))/2
        np.testing.assert_allclose(reported,expected,atol=1e-12)

    def test_quantization_is_applied_at_displaced_aperture_before_averaging(self):
        count = 2048
        phase = np.arange(count)/count
        signal = 1.3*np.sin(2*np.pi*phase)+.17
        skew = .013/Fixed.driver_pwm_frequency
        reported = causal_burst_average(signal,2,Fixed.driver_pwm_frequency,skew,np.floor)
        expected = (np.floor(1.3*np.sin(2*np.pi*(phase+.013))+.17)+
                    np.roll(np.floor(signal),count//2))/2
        np.testing.assert_array_equal(reported,expected)

    def test_sinc_filter_delay_and_response_match_closed_form(self):
        osr, order, sample_rate, frequency, cutoff = 128, 3, 2560000., 2000., 8000.
        weights = np.ones(1)
        for _ in range(order):
            weights = np.convolve(weights, np.ones(osr)/osr)
        response = weighted_acquisition_response(cutoff, 1, sample_rate, weights, frequency)
        ratio = frequency/sample_rate
        expected = (np.sinc(osr*ratio)/np.sinc(ratio) *
                    np.exp(-1j*np.pi*ratio*(osr-1)))**order / (1 + 1j*frequency/cutoff)
        self.assertAlmostEqual(response['loop_bandwidth_magnitude'], abs(expected))
        self.assertAlmostEqual(response['loop_bandwidth_phase_deg'], np.angle(expected, deg=True))
        self.assertAlmostEqual(response['low_frequency_group_delay_s'],
                               1/(2*np.pi*cutoff) + order*(osr-1)/(2*sample_rate))

    def test_noise_covariance_matches_independent_spectral_integral(self):
        from scipy.integrate import quad
        cutoff = 1000.
        for order in (1, 2, 3):
            for lag in (0., .0001, .0005):
                integral = quad(lambda frequency: (1 + (frequency/cutoff)**2)**-order,
                                0., np.inf, weight='cos', wvar=2*np.pi*lag)[0]
                self.assertAlmostEqual(float(rc_noise_covariance(cutoff, order, lag)), integral, places=5)

    def test_correlated_samples_do_not_get_independent_noise_reduction(self):
        enbw = float(rc_noise_covariance(1000., 1, 0.))
        response = acquisition_response(1000., 1, 1e9, 8, 0.)
        self.assertGreater(response['white_noise_ENBW_Hz'], .99*enbw)
        self.assertAlmostEqual(response['loop_bandwidth_magnitude'], 1.)
        sparse = acquisition_response(1000., 1, 100., 8, 0.)
        self.assertAlmostEqual(sparse['white_noise_ENBW_Hz'], enbw/8)

    def test_periodic_filter_preserves_dc_and_matches_sine_gain_and_phase(self):
        phase = np.arange(1024)/1024
        samples = 3. + 2*np.cos(2*np.pi*phase)
        transfer = (1 + 1j*10000./2000.)**-2
        expected = 3. + 2*np.real(transfer*np.exp(2j*np.pi*phase))
        np.testing.assert_allclose(filter_periodic(samples, 2000., 2, 10000.), expected, atol=1e-12)

    def test_uniform_sampling_rejects_nonaliased_harmonics_but_not_aliases(self):
        phase = np.arange(1024)/1024
        samples = 3. + np.cos(2*np.pi*phase) + .5*np.cos(2*np.pi*4*phase)
        np.testing.assert_allclose(phase_sample_means(samples, 4),
                                   3. + .5*np.cos(2*np.pi*4*phase[:256]), atol=1e-12)


class PhysicalReadoutIntegrationTests(unittest.TestCase):
    def test_native_mesh_preserves_narrow_pulse_area(self):
        from methodology.electronics.readout import interval_means

        trace = np.array([[0.,0.],[0.49,0.],[0.5,1.],[0.51,0.],[1.,0.]])
        self.assertAlmostEqual(interval_means(trace,0.,1.)[0],0.01)
        self.assertAlmostEqual(interval_means(trace,0.495,0.505)[0],0.75)

    def test_incomplete_recording_cannot_silently_extrapolate_mean(self):
        from methodology.electronics.readout import interval_means

        trace = np.array([[0.,0.],[1.,1.]])
        with self.assertRaises(ValueError):
            interval_means(trace,0.,2.)


class ReplayWaveformReductionTests(unittest.TestCase):
    def test_adc_loading_follows_the_datasheet_biased_input_impedance(self):
        from methodology.electronics.acquisition import sensing_filter

        for bias in (0.,Fixed.electronics_acquisition_sar_input_bias):
            with patch.object(Fixed,'electronics_acquisition_sar_input_bias',bias):
                sensor=sensing_filter()
            for current in (-Fixed.driver_channel_current,0.,Fixed.driver_channel_current):
                source=Fixed.electronics_current_reference+sensor['sensitivity_V_per_A']*current
                series=Fixed.electronics_acquisition_filter_resistance
                load=Fixed.electronics_acquisition_sar_input_resistance
                node=(source/series+bias/load)/(1/series+1/load)
                predicted=Fixed.electronics_current_reference+sensor['sensitivity_V_per_A']*(
                    sensor['load_gain']*current+sensor['load_offset_A'])
                self.assertAlmostEqual(node,predicted)

    def test_adc_bias_change_is_a_constant_filter_state_translation_before_quantization(self):
        from methodology.electronics import acquisition, power_stage, switching_loop

        recording=Path(__file__).resolve().parents[1]/'results/electronics/low_side_readout/physical_return_paths/settled_bridge_readout'
        case=next(row for row in power_stage.topology_cases() if row['topology']=='unipolar_full_bridge'
            and row['ripple_pass'] and row['voltage_headroom_pass'])
        with patch.object(Fixed,'electronics_acquisition_sar_input_bias',0.):
            old,settings,_,_=switching_loop.periodic_initial(recording,1)
            derivative,_=switching_loop.coupled_equations(case,settings)
            expected=derivative(0.,old)
            old_sensor=acquisition.sensing_filter()
        updated,settings,_,_=switching_loop.periodic_initial(recording,1)
        derivative,_=switching_loop.coupled_equations(case,settings)
        np.testing.assert_allclose(derivative(0.,updated),expected,atol=1e-7)
        new_sensor=acquisition.sensing_filter()
        self.assertAlmostEqual(old[-1]-old_sensor['load_offset_A'],updated[-1]-new_sensor['load_offset_A'])

    def test_transition_window_interpolates_boundary_and_excludes_incomplete_sample_pair(self):
        from methodology.electronics import acquisition, switching_loop

        spacing=acquisition.sar_schedule()['sample_spacing_s']
        time=np.array([0.,2*spacing,4*spacing])
        values=np.column_stack((time,-time))
        events=np.array([[0.,1.],[2*spacing,2.],[3*spacing,3.]])
        clipped,states,retained=switching_loop.replay_window(time,values,events,3*spacing)
        np.testing.assert_allclose(clipped,[0.,2*spacing,3*spacing])
        np.testing.assert_allclose(states[-1],[3*spacing,-3*spacing])
        np.testing.assert_array_equal(retained,events[:2])
        with self.assertRaises(ValueError):
            switching_loop.replay_window(time,values,events,5*spacing)

    def test_periodic_replay_phase_preserves_breakpoints_pulses_and_integral(self):
        from methodology.electronics.bridge_readout import shifted_periodic_waveform

        time=np.array([0.,.1,.15,.2,.9,1.])
        values=np.column_stack(([0.,0.,3.,0.,0.,0.],[1.,2.,4.,2.,-1.,1.]))
        for phase in (0.,.15,.36,.95):
            shifted,samples=shifted_periodic_waveform(time,values,phase)
            query=np.linspace(0.,1.,1001)
            actual=np.column_stack([np.interp(query,shifted,column) for column in samples.T])
            expected=np.column_stack([np.interp((query+phase)%1.,time,column) for column in values.T])
            np.testing.assert_allclose(actual,expected,atol=1e-13)
            np.testing.assert_allclose(np.trapezoid(samples,shifted,axis=0),np.trapezoid(values,time,axis=0),atol=1e-14)
            self.assertAlmostEqual(samples[:,0].max(),3.)
            np.testing.assert_array_equal(samples[0],samples[-1])

    def test_reduction_bounds_error_in_every_input_column_and_retains_pulses(self):
        from methodology.electronics.bridge_readout import compact_piecewise_linear

        time=np.linspace(0,1,10001)
        values=np.column_stack((np.sin(2*np.pi*time), np.maximum(0,1-np.abs(time-.5)/.001)))
        reduced_time,reduced_values=compact_piecewise_linear(time,values,1e-5)
        reconstructed=np.column_stack([np.interp(time,reduced_time,column) for column in reduced_values.T])
        self.assertLess(len(reduced_time),len(time)/2)
        self.assertLessEqual(np.max(np.abs(values-reconstructed)),1e-5)
        self.assertEqual(reduced_values[:,1].max(),1.)

    def test_invalid_waveforms_are_rejected(self):
        from methodology.electronics.bridge_readout import compact_piecewise_linear

        with self.assertRaises(ValueError):
            compact_piecewise_linear(np.array([0.,0.]),np.zeros((2,2)),1e-5)


class SwitchingTraceComparisonTests(unittest.TestCase):
    def test_winding_equation_slopes_remove_linear_resampling_error_without_hiding_state_error(self):
        from methodology.electronics.switching_loop import compare_trace_samples

        case=dict(coil_R_ohm=2.,minimum_coil_L_H=3.)
        def recording(time):
            current=time**3
            states=np.zeros((len(time),6))
            states[:,0]=current
            states[:,3]=case['coil_R_ohm']*current+case['minimum_coil_L_H']*3*time**2
            return dict(time=time,states=states,events=np.zeros((1,7)))
        first=recording(np.linspace(0,1,11))
        second=recording(np.linspace(0,1,16))
        result=compare_trace_samples(first,second,case)
        self.assertGreater(result['peak_linear_interpolation_current_difference_A'],Fixed.electronics_spice_current_tolerance)
        self.assertLess(result['peak_current_refinement_difference_A'],1e-14)
        self.assertTrue(result['numerical_pass'])
        second['states'][:,0]+=2*Fixed.electronics_spice_current_tolerance
        self.assertFalse(compare_trace_samples(first,second,case)['numerical_pass'])

    def test_resampling_rejects_changed_physical_interval_or_controller_clock(self):
        from methodology.electronics.switching_loop import compare_trace_samples

        trace=dict(time=np.array([0.,1.]),states=np.zeros((2,6)),events=np.zeros((1,7)))
        other={key:value.copy() for key,value in trace.items()}
        case=dict(coil_R_ohm=1.,minimum_coil_L_H=1.)
        other['time'][-1]=2.
        with self.assertRaises(ValueError):
            compare_trace_samples(trace,other,case)
        other['time'][-1]=1.
        other['events'][0,0]=.1
        with self.assertRaises(ValueError):
            compare_trace_samples(trace,other,case)


class SarSerialProtocolTests(unittest.TestCase):
    def test_stream_pairs_wrap_without_using_a_future_conversion(self):
        from methodology.electronics import logic
        from methodology.electronics.logic import SarSerialInterface,SarAcquisitionStream,sar_register_command,sar_manual_command

        interface=SarSerialInterface()
        for address,value in [(3,1),*[(5+index,6) for index in range(8)]]:
            interface.exchange(sar_register_command(address,value),[0]*8)
        interface.exchange(sar_manual_command(0),[0]*8)
        stream=SarAcquisitionStream(interface,0)
        for index in range(34):
            channel=(index//2)%8
            frame=((index+100)<<6)|(channel<<2)
            result=stream.receive(index,[frame>>8,frame&255],index*400+322)
            if index%2:
                self.assertEqual(result['mean_code'],index+99.5)
                self.assertEqual(result['channel'],channel)
                self.assertEqual(result['sample_ticks'],[(index-1)*400,index*400])
                self.assertEqual(result['latch_tick'],(index-1)*logic.controller_schedule()['carrier_ticks']//2+
                    logic.controller_schedule()['channels'][result['channel']]['latch_offset_ticks'])
            else:
                self.assertIsNone(result)
        self.assertTrue(stream.regulation_allowed)
        self.assertIsNone(stream.receive(33,[0,0],34*400+322))
        self.assertFalse(stream.regulation_allowed)
        self.assertIsNone(stream.receive(34,[0,0],34*400+322))

    def test_acquisition_failures_latch_and_silent_loss_expires(self):
        from methodology.electronics.logic import sar_stream_screen,SarSerialInterface,SarAcquisitionStream

        report=sar_stream_screen()
        self.assertEqual(len(report['injected_failures']),7)
        self.assertTrue(report['silent_loss_detected'])
        self.assertTrue(all(not case['regulation_allowed'] for case in report['injected_failures']))
        with self.assertRaises(ValueError):
            SarAcquisitionStream(SarSerialInterface(),0)

    def test_datasheet_commands_and_falling_edge_word_positions(self):
        from methodology.electronics.logic import sar_manual_command,sar_register_command,sar_decode_frame

        self.assertEqual([sar_manual_command(index) for index in range(8)],
            [0xC000,0xC400,0xC800,0xCC00,0xD000,0xD400,0xD800,0xDC00])
        self.assertEqual(sar_register_command(5,6),0x0B06)
        self.assertEqual(sar_register_command(3,1),0x0701)
        self.assertEqual(sar_register_command(5,0,False),0x0A00)
        captures=[0]*16+[int(bit) for bit in f'{0x2ABCD:018b}']+[0,1,1,1]+[0,0]
        frame=int(''.join(map(str,captures)),2)
        self.assertEqual(sar_decode_frame(frame,7),0x2ABCD)
        with self.assertRaises(ValueError):
            sar_decode_frame(frame,6)
        with self.assertRaises(ValueError):
            sar_decode_frame(frame|1,7)

    def test_assembled_pio_bitstream_and_tick_schedule(self):
        from methodology.electronics.logic import sar_pio_program,sar_decode_frame

        folder=Path(__file__).resolve().parents[1]/'results/electronics/current_loop/pio_acquisition'
        source,timing=sar_pio_program()
        self.assertEqual((folder/'acquisition.pio').read_text(),source)
        program=json.loads((folder/'acquisition.json').read_text())['programs'][0]
        instructions=[int(row['hex'],16) for row in program['instructions']]
        received_frame=(0x2ABCD<<6)|(7<<2)
        sampled=[int(bit) for bit in f'{received_frame:040b}']*2
        commands=[0xC000,0xDC00,0xC400]
        osr=commands[0]<<16
        tx=[word<<16 for word in commands[1:]]
        isr=shift_count=pc=tick=sample_index=0
        counters={1:0,2:0}
        rx=[]
        sent=[]
        sample_ticks=[]
        edge_ticks=[]
        frames=[]
        previous_side=2
        word_mask=(1<<32)-1
        for _ in range(2*timing['next_sample_tick']):
            word=instructions[pc]
            opcode=word>>13
            side=(word>>11)&3
            delay=(word>>8)&7
            if previous_side&2 and not side&2:
                sample_ticks.append(tick)
            if previous_side&1 and not side&1:
                edge_ticks.append(tick)
            previous_side=side
            next_pc=program['wrapTarget'] if pc==program['wrap'] else pc+1
            if opcode==7:
                counters[(word>>5)&7]=word&31
            elif opcode==3:
                self.assertEqual(word&255,1)
                sent.append(osr>>31)
                osr=(osr<<1)&word_mask
            elif opcode==2:
                self.assertEqual(word&255,1)
                isr=((isr<<1)|sampled[sample_index])&word_mask
                sample_index+=1
                shift_count+=1
                if shift_count==32:
                    rx.append(isr)
                    isr=shift_count=0
            elif opcode==4:
                if word&128:
                    osr=tx.pop(0)
                else:
                    rx.append(isr)
                    isr=shift_count=0
            elif opcode==5:
                self.assertEqual(word&255,0x42)
            elif opcode==0:
                condition=(word>>5)&7
                self.assertIn(condition,(2,4))
                register=condition//2
                if counters[register]:
                    next_pc=word&31
                counters[register]=(counters[register]-1)&word_mask
            else:
                self.fail('Unreviewed PIO instruction in acquisition binary')
            tick+=1+delay
            pc=next_pc
            if pc==program['wrapTarget']:
                frames.append(tick)
                if len(frames)==2:
                    break
        self.assertEqual(sample_ticks,[0,400])
        self.assertEqual(frames,[400,800])
        self.assertEqual(edge_ticks[:40],list(range(5,318,8)))
        self.assertEqual(sent,[int(bit) for command in commands[:2] for bit in f'{command:016b}'+'0'*24])
        self.assertEqual(len(rx),4)
        for first,second in zip(rx[::2],rx[1::2],strict=True):
            self.assertEqual(sar_decode_frame((first<<8)|second,7),0x2ABCD)

    def test_configuration_pair_pipeline_and_period_wrap(self):
        from methodology.electronics.logic import sar_protocol_screen,SarSerialInterface

        report=sar_protocol_screen()
        self.assertTrue(report['passed'])
        self.assertEqual([row['channel'] for row in report['frames']],np.repeat(np.arange(8),2).tolist())
        self.assertEqual([row['next_selected_channel'] for row in report['frames']],
            [0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,0])
        interface=SarSerialInterface()
        with self.assertRaises(ValueError):
            interface.exchange(0xC000,np.zeros(8,dtype=int))


class SampledRegulationTests(unittest.TestCase):
    def test_phase_correct_counter_reproduces_rounded_leg_duties_and_bounds_voltage_error(self):
        from methodology.electronics.power_stage import timer_leg_duties
        from methodology.electronics.current_loop import nominal_circuit

        bus=nominal_circuit()['bus_nominal']
        counts=int(Fixed.electronics_logic_clock/(2*Fixed.driver_pwm_frequency))
        counter=np.r_[np.arange(counts),np.arange(counts)[::-1]]
        for voltage in np.linspace(-bus,bus,101):
            positive,negative=timer_leg_duties(voltage,bus)
            levels=np.rint(np.array([positive,negative])*counts)
            waveform=bus*((counter<levels[0]).astype(int)-(counter<levels[1]).astype(int))
            self.assertAlmostEqual(waveform.mean(),bus*(positive-negative))
            self.assertLessEqual(abs(waveform.mean()-voltage),bus/counts+1e-12)
            self.assertAlmostEqual(positive+negative,1.)

    def test_integer_clock_schedule_finishes_before_each_selected_carrier_latch(self):
        from methodology.electronics.logic import controller_schedule

        schedule=controller_schedule()
        for channel in schedule['channels']:
            self.assertLess(channel['result_ready_ticks'],channel['compute_ready_ticks'])
            self.assertLessEqual(channel['compute_ready_ticks'],channel['latch_ticks'])
        with patch.object(Fixed,'electronics_regulation_compute_time',1/Fixed.current_regulator_rate):
            with self.assertRaises(ValueError):
                controller_schedule()

    def test_phased_latches_match_counter_zero_transitions(self):
        from methodology.electronics import logic

        schedule=logic.controller_schedule()
        half=schedule['pwm_TOP']+1
        counter=np.r_[np.arange(half),np.arange(half-1,-1,-1)]
        for row in schedule['channels']:
            shifted=np.roll(counter,-row['pwm_counter_initial'])
            transition=np.flatnonzero((shifted==0)&(np.roll(shifted,1)==0))
            self.assertEqual(len(transition),1)
            latch=row['latch_offset_ticks']
            self.assertEqual(latch%len(counter),transition[0])
            commit=row['compute_ready_ticks']-row['channel']*schedule['carrier_ticks']
            self.assertGreater(latch,commit)
            self.assertLess(latch-len(counter),commit)
        self.assertEqual(len({row['pwm_counter_initial'] for row in schedule['channels']}),Fixed.electronics_acquisition_sar[2])

    def test_each_inductance_uses_a_bound_on_its_own_switching_ripple(self):
        from methodology.electronics.current_loop import SampledCurrentLoop
        from methodology.electronics.power_stage import periodic_cycle

        for factor in (.5,1.,2.):
            loop=SampledCurrentLoop(inductance_factor=factor)
            for duty in (.2,.5,.8):
                cycle=periodic_cycle(loop.nominal['coil_resistance'],loop.nominal['inductance']*factor,
                    0.,loop.nominal['bus_high'],duty,loop.nominal['ripple_frequency'])
                self.assertLessEqual(cycle['peak_deviation_A'],loop.ripple_bound+1e-12)

    def test_reference_filters_preserve_constant_current_and_exact_command_pole(self):
        from methodology.electronics.current_loop import CurrentReference

        reference=CurrentReference(2.)
        np.testing.assert_allclose(reference.at(.001),reference.state,rtol=0,atol=1e-12)
        reference.state[-1]=-1.
        current,rate=reference.current_and_rate(.0001)
        expected=-1.+3*np.exp(-2*np.pi*Fixed.setpoint_filter_cutoff*.0001)
        self.assertAlmostEqual(current,expected,places=12)
        self.assertAlmostEqual(rate,2*np.pi*Fixed.setpoint_filter_cutoff*(-1.-expected),places=8)
        self.assertGreater(reference.paired_reading(.0001),current)

    def test_reference_feedforward_holds_nominal_current_without_integral_bias(self):
        from methodology.electronics.current_loop import SampledCurrentLoop

        loop=SampledCurrentLoop()
        state=np.r_[np.full(loop.physical_count,2.),0.,2.*loop.nominal['resistance']]
        actual,_=loop.advance(state,2.)
        np.testing.assert_allclose(actual,state,rtol=0,atol=1e-12)

    def test_held_voltage_changes_only_after_the_scheduled_actuation(self):
        from methodology import qualification
        from methodology.electronics.current_loop import SampledCurrentLoop

        loop=SampledCurrentLoop(gain=0.)
        state=np.zeros(loop.state_size)
        state[loop.physical_count+1]=1.
        actual,metrics=loop.advance(state,0.)
        intermediate=qualification.rl_step(0.,1.,0.,loop.nominal['resistance'],loop.nominal['inductance'],loop.actuation)
        expected=qualification.rl_step(intermediate,0.,0.,loop.nominal['resistance'],loop.nominal['inductance'],loop.period-loop.actuation)
        self.assertAlmostEqual(actual[0],expected,places=12)
        self.assertEqual(metrics['measured_A'],0.)

    def test_new_command_cannot_change_the_samples_that_precede_it(self):
        from methodology.electronics.current_loop import SampledCurrentLoop

        loop=SampledCurrentLoop()
        state,metrics=loop.advance(np.zeros(loop.state_size),1.)
        self.assertEqual(metrics['measured_A'],0.)
        self.assertGreater(state[0],0.)

    def test_shared_schedule_respects_each_frame_and_compute_deadline(self):
        from methodology.electronics.acquisition import sar_schedule

        schedule=sar_schedule()
        self.assertLess(schedule['frame_s'],schedule['sample_spacing_s'])
        self.assertLess(schedule['serial_utilization'],1.)
        self.assertLess(schedule['computation_utilization'],1.)
        self.assertGreaterEqual(schedule['pwm_latch_s'],schedule['computation_ready_s'])

    def test_candidate_integral_gain_does_not_change_default_regulator(self):
        from methodology import qualification

        arguments=(1.,0.,1.,1e-4,20.)
        original=qualification.current_regulator(*arguments)
        scaled=qualification.current_regulator(*arguments,integral_gain_ratio=.5)
        self.assertEqual(original[0],scaled[0])
        self.assertEqual(original[2],scaled[2])
        self.assertEqual(original[1]*.5,scaled[1])


class ProtectionTests(unittest.TestCase):
    def test_full_voltage_fault_uses_unchanged_peak_and_exact_shutdown_bound(self):
        from methodology.electronics.protection import full_voltage_fault_screen
        from methodology.qualification import rl_step

        report=full_voltage_fault_screen()
        self.assertEqual(report['allowed_peak_current_A'],Fixed.driver_peak_current)
        self.assertAlmostEqual(rl_step(report['latest_ideal_trip_current_for_peak_limit_A'],report['bus_voltage_V'],0.,
            report['resistance_ohm'],report['inductance_H'],report['driver_disable_delay_bound_s']),Fixed.driver_peak_current)
        with patch.object(Fixed,'electronics_drv8962_enable_delay',0.):
            immediate=full_voltage_fault_screen()
            self.assertEqual(immediate['current_at_maximum_disable_delay_A'],Fixed.driver_channel_current)
            self.assertTrue(immediate['meets_operating_peak_limit'])

    def test_frozen_command_crossing_matches_rl_solution(self):
        from methodology.electronics.protection import held_voltage_limit_time
        from methodology.qualification import rl_step

        for sign in (-1,1):
            current,voltage,resistance,inductance,limit=sign*3.,sign*12.,1.,1e-4,6.
            elapsed=held_voltage_limit_time(current,voltage,resistance,inductance,limit)
            self.assertAlmostEqual(abs(rl_step(current,voltage,0.,resistance,inductance,elapsed)),limit)
            self.assertTrue(np.isinf(held_voltage_limit_time(current,sign*4.,resistance,inductance,limit)))
        self.assertEqual(held_voltage_limit_time(7.,0.,1.,1e-4,6.),0.)

    def test_stuck_high_watchdog_and_fault_clear_require_rearm(self):
        from methodology.electronics.protection import IndependentShutdown,watchdog_timing

        duration=watchdog_timing()['minimum_timeout_s']
        shutdown=IndependentShutdown(duration)
        self.assertFalse(shutdown.advance(0.,False,False,True,True,timer_ready=False))
        self.assertFalse(shutdown.advance(duration,True,False,True,True))
        self.assertTrue(shutdown.advance(duration*1.5,True,True,True,True))
        self.assertEqual(shutdown.enables((True,False)),(True,False))
        self.assertFalse(shutdown.advance(duration*2,True,True,True,True))
        self.assertFalse(shutdown.advance(duration*3,False,True,True,True))
        self.assertFalse(shutdown.advance(duration*3.5,True,True,True,True))
        shutdown.advance(duration*3.6,False,False,True,True)
        self.assertTrue(shutdown.advance(duration*4,True,True,True,True))
        self.assertFalse(shutdown.advance(duration*4.1,False,True,False,True))
        self.assertFalse(shutdown.advance(duration*4.2,True,True,True,True))
        self.assertEqual(shutdown.enables((True,True)),(False,False))

    def test_returned_energy_capacitance_balance(self):
        from methodology.electronics.supply import absorbed_energy_voltage,required_absorption_capacitance

        for energy in (0.,0.01,1.):
            capacitance=required_absorption_capacitance(24.,25.2,energy)
            if energy:
                self.assertAlmostEqual(absorbed_energy_voltage(capacitance,24.,energy),25.2)
            else:
                self.assertEqual(capacitance,0.)
        with self.assertRaises(ValueError):
            required_absorption_capacitance(25.2,25.2,0.01)

    def test_shutdown_netlist_rejects_bypassed_gate(self):
        from methodology.electronics.review import validate_protection

        evidence=Path(__file__).resolve().parents[1]/'results/cad/electronics/protection/netlist.xml'
        netlist=ElementTree.parse(evidence).getroot()
        self.assertTrue(validate_protection(netlist)['passed'])
        gate=next(node for node in netlist.findall('nets/net/node')
            if node.attrib['ref']=='U33' and node.attrib['pin']=='2')
        gate.attrib['pin']='1'
        with self.assertRaises(ValueError):
            validate_protection(netlist)

    def test_disabled_return_matches_lossless_energy_and_stops_at_zero(self):
        from methodology.electronics.supply import disabled_return,absorbed_energy_voltage

        for capacitance in (1e-12,1e-6,1e-3):
            result=disabled_return(8,6.5,1e-4,0.,capacitance,24.)
            self.assertAlmostEqual(result['final_capacitor_voltage_V']/absorbed_energy_voltage(capacitance,24.,8*1e-4*6.5**2/2),1.)
            self.assertAlmostEqual(result['dissipated_energy_J'],0.)
            self.assertGreater(result['decay_time_s'],0.)
        result=disabled_return(8,0.,1e-4,.5,1e-3,24.)
        self.assertEqual(result['decay_time_s'],0.)
        self.assertEqual(result['maximum_terminal_voltage_V'],24.)

    def test_disabled_return_agrees_with_independent_integration_and_loss_accounting(self):
        from scipy.integrate import solve_ivp
        from methodology.electronics.supply import disabled_return

        count,current,inductance,resistance,capacitance,voltage,diode,esr=8,6.5,1e-4,.5,.001,24.,.8,.016
        exact=disabled_return(count,current,inductance,resistance,capacitance,voltage,diode,esr)
        def equations(time,state):
            branch,capacitor,loss=state
            return [-(capacitor+2*diode+(resistance+count*esr)*branch)/inductance,
                count*branch/capacitance,count*resistance*branch**2+count*2*diode*branch+esr*(count*branch)**2]
        numerical=solve_ivp(equations,(0,exact['decay_time_s']),[current,voltage,0.],
            rtol=Fixed.electronics_spice_relative_tolerance,atol=Fixed.electronics_spice_absolute_tolerance,
            dense_output=True)
        self.assertTrue(numerical.success)
        self.assertAlmostEqual(numerical.y[0,-1],0.,places=6)
        self.assertAlmostEqual(numerical.y[1,-1],exact['final_capacitor_voltage_V'],places=6)
        self.assertAlmostEqual(numerical.y[2,-1],exact['dissipated_energy_J'],places=8)
        states=numerical.sol(np.linspace(0,exact['decay_time_s'],Fixed.electronics_pwm_screen_points))
        self.assertLessEqual(np.max(states[1]+count*esr*states[0]),exact['maximum_terminal_voltage_V']+Fixed.electronics_bridge_voltage_tolerance)


class GroupPowerSequenceTests(unittest.TestCase):
    def configured_interface(self):
        from methodology.electronics import logic

        interface=logic.SarSerialInterface()
        codes=[0]*Fixed.electronics_acquisition_sar[2]
        for row in logic.sar_protocol_screen()['initialization']:
            interface.exchange(int(row['command_hex'],16),codes)
        interface.exchange(logic.sar_manual_command(0),codes)
        return interface

    def test_cold_start_fresh_round_disable_sleep_and_wake(self):
        from methodology.electronics import logic

        power=logic.ControlGroupPower()
        ready=power.wake(0)
        with self.assertRaises(ValueError):
            power.begin_acquisition(ready-1,self.configured_interface())
        stream=power.begin_acquisition(ready,self.configured_interface())
        self.assertTrue(stream.regulation_allowed)
        with self.assertRaises(ValueError):
            power.arm(ready,True)
        schedule=logic.controller_schedule()
        for channel in range(Fixed.electronics_acquisition_sar[2]):
            latch=ready+schedule['channels'][channel]['latch_ticks']
            tick=latch-1
            self.assertTrue(power.commit(tick,dict(status=1,fault=0,channel=channel,latch_tick=latch)))
            self.assertFalse(power.enabled)
        with self.assertRaises(ValueError):
            power.arm(tick,False)
        power.arm(tick,True)
        self.assertTrue(power.enabled)
        power.disable(tick)
        with self.assertRaises(ValueError):
            power.sleep(tick,'standby')
        slept=tick+int(np.ceil(Fixed.electronics_drv8962_enable_delay*Fixed.electronics_logic_clock))
        power.sleep(slept,'standby')
        self.assertFalse(power.driver_awake)
        wake=slept+int(np.ceil(Fixed.electronics_drv8962_sleep_time*Fixed.electronics_logic_clock))
        with self.assertRaises(ValueError):
            power.wake(wake-1)
        recovered=power.wake(wake)
        self.assertEqual(recovered-wake,int(np.ceil(max(Fixed.electronics_drv8962_wake_time,Fixed.electronics_sar_wake_times[0])*Fixed.electronics_logic_clock)))
        self.assertEqual(power.commits,0)
        self.assertFalse(power.enabled)

    def test_faults_latch_and_old_or_missing_commits_cannot_arm(self):
        from methodology.electronics import logic

        for kind in ('wrong_channel','late','driver_fault','power_fault'):
            power=logic.ControlGroupPower()
            ready=power.wake(0)
            power.begin_acquisition(ready,self.configured_interface())
            latch=ready+logic.controller_schedule()['channels'][0]['latch_ticks']
            if kind in ('wrong_channel','late'):
                self.assertFalse(power.commit(latch if kind=='late' else latch-1,
                    dict(status=1,fault=0,channel=int(kind=='wrong_channel'),latch_tick=latch)))
            else:
                power.advance(latch,kind!='power_fault',kind!='driver_fault')
            self.assertEqual(power.mode,'fault')
            self.assertFalse(power.enabled)
            with self.assertRaises(ValueError):
                power.wake(latch)
            with self.assertRaises(ValueError):
                power.reset_fault(latch,False,True)
            power.reset_fault(latch,True,True)
            self.assertEqual(power.mode,'power_down')
            self.assertFalse(power.driver_awake)


class CompiledRegulatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from methodology.electronics import firmware

        folder=Path('methodology/results/electronics/current_loop/compiled_regulator')
        build=json.loads((folder/'build.json').read_text())
        temporary=TemporaryDirectory(dir=folder.parent)
        cls.addClassCleanup(temporary.cleanup)
        cls.backend,_,_=firmware.build(Path(temporary.name)/'compiled',
            Path(build['commands'][0]['command'][0]),Path(build['commands'][1]['command'][0]))

    def test_compiled_deadtime_bound_reference_and_saturation(self):
        import ctypes
        from methodology.electronics import current_loop,firmware

        backend=self.backend
        function=backend.library.deadtime_voltage
        function.argtypes=[ctypes.c_float]
        function.restype=ctypes.c_float
        _,settings=firmware.regulator_settings()
        domain=settings['argument_domain']*Fixed.electronics_deadtime_compensation_current
        samples=np.r_[np.linspace(-domain,domain,32*settings['table_entries']),
            -Fixed.driver_channel_current,Fixed.driver_channel_current]
        actual=np.array([function(float(value)) for value in samples])
        expected=current_loop.deadtime_feedforward(samples,backend.nominal['bus_nominal'])
        self.assertLessEqual(np.max(np.abs(actual-expected)),settings['comparison_voltage_budget_V'])
        np.testing.assert_array_equal(actual,np.array([-function(float(-value)) for value in samples]))
        for error,integral in ((10.,100.),(-10.,-100.)):
            result=backend(error,integral,0.,0.,backend.nominal,digital=True)
            self.assertEqual(result[1],integral)
            self.assertLessEqual(abs(result[0]),backend.nominal['voltage_limit'])
        reference=current_loop.CurrentReference(0.,np.float32)
        compiled=firmware.CompiledReference(0.,np.float32,backend.library)
        elapsed=current_loop.acquisition.sar_schedule()['actuation_s']
        for command in (Fixed.driver_channel_current,-Fixed.driver_channel_current,0.):
            reference.state[-1]=compiled.state[-1]=command
            for _ in range(20):
                actual_target,actual_rate=compiled.current_and_rate(elapsed)
                expected_target,expected_rate=reference.current_and_rate(elapsed)
                self.assertAlmostEqual(float(actual_target),float(expected_target),delta=Fixed.electronics_spice_current_tolerance)
                self.assertAlmostEqual(float(actual_rate),float(expected_rate),delta=Fixed.electronics_spice_current_tolerance*reference.rates[0])
                self.assertAlmostEqual(float(compiled.paired_reading(0.)),float(reference.paired_reading(0.)),delta=Fixed.electronics_spice_current_tolerance)
                reference.state=reference.at(1/Fixed.current_regulator_rate)
                compiled.state=compiled.at(1/Fixed.current_regulator_rate)
                np.testing.assert_allclose(compiled.state,reference.state,atol=Fixed.electronics_spice_current_tolerance)

    def test_compiled_acquisition_to_pwm_replays_all_channels(self):
        from methodology.electronics import firmware

        recording=Path('methodology/results/electronics/current_loop/compiled_regulator/current_control/traces.npz')
        trace=np.load(recording)['case_0_noise_0']
        result=firmware.runtime_screen(self.backend,trace)
        self.assertTrue(result['passed'])
        self.assertEqual(result['frames_checked'],len(trace)*Fixed.electronics_acquisition_sar[2]*Fixed.electronics_regulation_samples_per_carrier)

    def test_integrated_feedback_uses_independent_commands_and_idle_zero_calibration(self):
        from methodology.electronics import system

        trace,_,report=system.simulate(self.backend,noisy=True)
        self.assertTrue(report['passed'],report)
        self.assertEqual(report['final_power_mode'],'power_down')
        self.assertTrue(report['armed_after_calibrated_round'])
        for channel,fraction in enumerate(Fixed.electronics_group_command_fractions):
            plateau=trace[(trace[:,1]==channel)&(trace[:,0]>2*Fixed.electronics_regulation_hold-Fixed.electronics_regulation_settling_window)&
                (trace[:,0]<2*Fixed.electronics_regulation_hold)]
            self.assertGreater(len(plateau),0)
            self.assertLessEqual(np.max(np.abs(plateau[:,3]-fraction*Fixed.driver_channel_current)),report['existing_tracking_allowance_A'])

    def test_compiled_zero_calibration_rejects_energized_commands(self):
        from methodology.electronics import firmware

        runtime=firmware.CompiledRuntime(self.backend,configuration_verified=True)
        self.assertEqual(runtime.begin_zero_calibration(),0)
        self.assertEqual(runtime.calibration_remaining,Fixed.electronics_acquisition_sar[2])
        self.assertEqual(runtime.command(0,Fixed.driver_channel_current),-1)
        self.assertNotEqual(runtime.fault,0)
        runtime=firmware.CompiledRuntime(self.backend,configuration_verified=True)
        self.assertEqual(runtime.command(0,Fixed.driver_channel_current),0)
        self.assertEqual(runtime.begin_zero_calibration(),-1)

    def test_compiled_runtime_faults_stop_pwm_updates(self):
        from methodology.electronics import firmware,logic

        _,timing=logic.sar_pio_program()
        schedule=logic.controller_schedule()
        ready=timing['rx_fifo_complete_tick']
        for frame,words,tick,error in (
            (1,[0,0],ready,False),
            (0,[0,0],ready,True),
            (0,[0,0],ready-1,False),
            (0,[0,0],schedule['channels'][0]['latch_ticks'],False),
            (0,[0,1],ready,False),
            (0,[1<<31,0],ready,False),
            (0,[0,1<<8],ready,False),
            (0,[0,0],ready,False)):
            runtime=firmware.CompiledRuntime(self.backend,configuration_verified=True)
            result=runtime.receive(frame,words,tick,error)
            self.assertEqual(result['status'],-1)
            self.assertNotEqual(result['fault'],0)
            self.assertEqual(runtime.receive(0,[0,0],ready)['status'],-1)
        unconfigured=firmware.CompiledRuntime(self.backend)
        self.assertEqual(unconfigured.receive(0,[0,0],ready)['status'],-1)
        for invalid in (float('nan'),float('inf'),Fixed.driver_channel_current*2):
            runtime=firmware.CompiledRuntime(self.backend,configuration_verified=True)
            self.assertEqual(runtime.command(0,invalid),-1)
        runtime=firmware.CompiledRuntime(self.backend,configuration_verified=True)
        self.assertEqual(runtime.check_deadline(schedule['channels'][0]['latch_ticks']),-1)



class CoupledSwitchingEquationTests(unittest.TestCase):
    def test_filter_and_bridge_derivatives_match_directional_differences(self):
        from methodology.electronics import power_stage,switching_loop

        case=next(row for row in power_stage.topology_cases()
            if row['topology']=='unipolar_full_bridge' and row['ripple_pass'] and row['voltage_headroom_pass'])
        _,settings=power_stage.return_path_netlist(case,2.,0.,'pgndA',1,True)
        derivative,jacobian=switching_loop.coupled_equations(case,settings)
        state=np.array([2.,-.01,.01,1.,23.,2.,2.,2.,2.])
        directions=np.random.default_rng(0).normal(size=(4,len(state)))
        time=1/Fixed.driver_pwm_frequency/2
        for direction in directions:
            epsilon=1e-5
            measured=(derivative(time,state+epsilon*direction)-derivative(time,state-epsilon*direction))/(2*epsilon)
            np.testing.assert_allclose(measured,jacobian(time,state)@direction,rtol=1e-7,atol=1e-6)


class DeadtimeCompensationTests(unittest.TestCase):
    def test_compensation_is_odd_bounded_and_continuous_at_zero(self):
        from methodology.electronics.current_loop import deadtime_feedforward
        from methodology.electronics.power_stage import deadtime_voltage_drop

        current=np.linspace(-6,6,121)
        value=deadtime_feedforward(current,24.)
        np.testing.assert_allclose(value,-deadtime_feedforward(-current,24.))
        self.assertLessEqual(np.max(np.abs(value)),deadtime_voltage_drop(24.))
        self.assertEqual(deadtime_feedforward(0.,24.),0.)

    def test_final_saturation_and_antiwindup_include_feedforward(self):
        from methodology.qualification import current_regulator

        voltage,integral,requested=current_regulator(1.,0.,1.,1e-4,1.,feedforward_voltage=2.)
        self.assertEqual(voltage,1.)
        self.assertGreater(requested,1.)
        self.assertEqual(integral,0.)


class IntegratedPlantTests(unittest.TestCase):
    def test_disabled_return_matches_analytical_decay_without_current_reversal(self):
        from methodology.electronics.system import ControlGroupPlant

        plant=ControlGroupPlant()
        initial=np.array(Fixed.electronics_group_command_fractions)*Fixed.driver_channel_current
        plant.state[:,0]=initial
        duration=1/Fixed.driver_pwm_frequency
        ratio=plant.disabled_voltage/plant.disabled_resistance
        expected=np.sign(initial)*np.maximum((np.abs(initial)+ratio)*np.exp(-duration*plant.disabled_resistance/plant.inductance)-ratio,0.)
        plant.advance(duration)
        np.testing.assert_allclose(plant.state[:,0],expected,atol=Fixed.electronics_spice_absolute_tolerance)
        plant.advance(Fixed.electronics_regulation_settling_window)
        np.testing.assert_array_equal(plant.state[:,0],np.zeros_like(initial))


class SharedBusTests(unittest.TestCase):
    def test_source_never_sinks_and_obeys_current_limit_and_kirchhoff(self):
        from methodology.electronics import current_loop,supply

        nominal=current_loop.nominal_circuit()['bus_nominal']
        bus=supply.SharedMotorBus(nominal,max(Fixed.electronics_shared_bulk_counts))
        for voltage,load in ((nominal,0.),(nominal,10.),(nominal,-10.),(nominal/2,20.)):
            point=bus.operating_point(voltage,load)
            source=point['source_current_A']
            self.assertGreaterEqual(source,0.)
            self.assertLessEqual(source,bus.current_limit)
            self.assertAlmostEqual(point['capacitor_current_A'],source-load)
            self.assertAlmostEqual(point['voltage_V'],voltage+bus.capacitor_resistance*point['capacitor_current_A'])
            dissipated=point['distribution_loss_W']+point['current_limit_loss_W']
            self.assertAlmostEqual(nominal*source,point['voltage_V']*source+dissipated)
        self.assertEqual(bus.operating_point(nominal,-10.)['source_current_A'],0.)

    def test_dynamic_bus_accepts_an_interval_without_voltage_extrema(self):
        from methodology.electronics import system

        plant=system.ControlGroupPlant(source_factor=1.,bulk_count=max(Fixed.electronics_shared_bulk_counts))
        plant.enabled=True
        plant.state[:,0]=Fixed.driver_channel_current
        plant.voltage[:]=plant.resistance*Fixed.driver_channel_current
        plant.advance(1/Fixed.electronics_logic_clock)
        self.assertTrue(np.isfinite(plant.state).all())
        self.assertLess(plant.capacitor_voltage,plant.bus.source_voltage)

    def test_coupled_disabled_bus_agrees_with_exact_inductive_return(self):
        from methodology.electronics import supply,system

        plant=system.ControlGroupPlant(source_factor=1.,bulk_count=max(Fixed.electronics_shared_bulk_counts),
            inductance_factor=max(Fixed.qualification_inductance_factors))
        plant.state[:,0]=Fixed.driver_peak_current
        expected=supply.disabled_return(plant.channels,Fixed.driver_peak_current,plant.inductance,plant.disabled_resistance,
            plant.bus.capacitance,plant.bus.source_voltage,Fixed.electronics_bridge_diode[0],plant.bus.capacitor_resistance)
        plant.advance(Fixed.electronics_regulation_settling_window)
        self.assertAlmostEqual(plant.capacitor_voltage,expected['final_capacitor_voltage_V'],delta=Fixed.electronics_bridge_voltage_tolerance)
        self.assertAlmostEqual(plant.maximum_bus_voltage,expected['maximum_terminal_voltage_V'],delta=Fixed.electronics_bridge_voltage_tolerance)
        np.testing.assert_array_equal(plant.state[:,0],np.zeros(plant.channels))


class CarrierSupplyTests(unittest.TestCase):
    def test_frozen_bridge_input_power_matches_deadtime_and_diode_balance(self):
        from methodology.electronics import current_loop,power_stage,supply

        nominal=current_loop.nominal_circuit()
        current=Fixed.driver_channel_current
        command=nominal['resistance']*current+current_loop.deadtime_feedforward(current,nominal['bus_nominal'])
        grid=Fixed.electronics_low_side_spice_steps_per_cycle*Fixed.electronics_acquisition_grid_refinement
        waveform=supply.group_bus_waveform([current],[command],grid)
        positive,negative=power_stage.timer_leg_duties(command,nominal['bus_nominal'])
        effective_duty=positive-negative-2*Fixed.electronics_drv8962_deadtime*Fixed.driver_pwm_frequency
        self.assertAlmostEqual(waveform.mean(),effective_duty*current,delta=Fixed.electronics_spice_current_tolerance)
        negative_waveform=supply.group_bus_waveform([-current],[-command],grid)
        np.testing.assert_allclose(negative_waveform,waveform,atol=Fixed.electronics_spice_absolute_tolerance)

    def test_capacitor_only_branch_carries_all_AC_current_with_equal_sharing(self):
        from methodology.electronics import supply

        grid=Fixed.electronics_low_side_spice_steps_per_cycle
        wave=Fixed.driver_channel_current*np.sin(2*np.pi*np.arange(grid)/grid)
        count=max(Fixed.electronics_shared_bulk_counts)
        result=supply.capacitor_ripple(wave,count,0.,0.)
        self.assertAlmostEqual(result['each_bulk_RMS_A'],Fixed.driver_channel_current/np.sqrt(2)/count)

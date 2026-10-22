import unittest
from unittest.mock import patch

import numpy as np

from model import Fixed
from methodology import design, qualification
from methodology.electronics.power_stage import periodic_cycle, bridge_gate_edges


class SwitchingReferenceTests(unittest.TestCase):
    def test_cycle_closes_and_mean_obeys_integrated_circuit_equation(self):
        for duty in (0., 0.1, 0.5, 0.9, 1.):
            result = periodic_cycle(2., 0.01, -5., 12., duty, 1000., 0.7)
            high, first_average = qualification.rl_interval(result['low_A'], 12., 0.7, 2., 0.01, duty / 1000)
            low, second_average = qualification.rl_interval(high, -5., 0.7, 2., 0.01, (1 - duty) / 1000)
            self.assertAlmostEqual(low, result['low_A'])
            self.assertAlmostEqual(first_average * duty + second_average * (1 - duty), result['mean_A'])

    def test_existing_global_bound_encloses_general_two_level_cycles(self):
        bound = design.split_rail_ripple_peak(2., 0.01, (12. + 5.) / 2, 1000.)
        for duty in np.linspace(0, 1, Fixed.electronics_pwm_screen_points):
            result = periodic_cycle(2., 0.01, -5., 12., duty, 1000., 0.7)
            self.assertLessEqual(result['peak_deviation_A'], bound + Fixed.allocator_residual_tolerance)

    def test_emf_shifts_mean_without_changing_ripple(self):
        first = periodic_cycle(2., 0.01, 0., 24., 0.4, 1000.)
        second = periodic_cycle(2., 0.01, 0., 24., 0.4, 1000., 3.)
        self.assertAlmostEqual(first['mean_A'] - second['mean_A'], 1.5)
        self.assertAlmostEqual(first['peak_deviation_A'], second['peak_deviation_A'])


class ReturnPathTimingTests(unittest.TestCase):
    def test_finite_gate_transitions_leave_deadtime_in_both_directions(self):
        period, edge = 1/Fixed.driver_pwm_frequency, Fixed.driver_switching_time
        for duty in (0.125, 0.5, 0.875):
            high_start, high_width, low_start, low_width = bridge_gate_edges(duty, 0., Fixed.electronics_drv8962_deadtime)
            self.assertAlmostEqual(high_start-low_start-edge, Fixed.electronics_drv8962_deadtime)
            self.assertAlmostEqual(low_start+edge+low_width-(high_start+high_width+2*edge), Fixed.electronics_drv8962_deadtime)
            self.assertLess(low_width+2*edge, period)

    def test_zero_deadtime_and_edge_recover_complementary_duty(self):
        period = 1/Fixed.driver_pwm_frequency
        with patch.object(Fixed, 'driver_switching_time', 0.):
            for duty in (0.125, 0.5, 0.875):
                _, high_width, _, low_width = bridge_gate_edges(duty, 0., 0.)
                self.assertAlmostEqual(high_width/period, duty)
                self.assertAlmostEqual((period-low_width)/period, 1-duty)


class PhysicalBridgeEquationTests(unittest.TestCase):
    def setUp(self):
        from methodology.electronics import power_stage

        self.case=next(case for case in power_stage.topology_cases()
            if case['topology']=='unipolar_full_bridge' and case['ripple_pass'] and case['voltage_headroom_pass'])
        _,self.settings=power_stage.return_path_netlist(self.case,Fixed.driver_channel_current,0.,'pgndA',1,True)
        self.derivative,self.jacobian=power_stage.return_path_equations(self.case,self.settings)

    def test_exact_derivatives_match_independent_directional_differences(self):
        states=([2.,-.01,.01,1.,23.,0.,0.], [2.,-.01,.01,-2.,26.,0.,0.])
        directions=np.random.default_rng(0).normal(size=(5,7))
        for state in map(np.array,states):
            for time in np.linspace(0,1/Fixed.driver_pwm_frequency,7):
                matrix=self.jacobian(time,state)
                for direction in directions:
                    displacement=1e-5*direction
                    measured=(self.derivative(time,state+displacement)-self.derivative(time,state-displacement))/2e-5
                    expected=matrix@direction
                    np.testing.assert_allclose(measured,expected,rtol=1e-7,atol=1e-6)

    def test_disabled_bridge_retains_diode_decay_without_pwm_dependence(self):
        from methodology.electronics import power_stage

        derivative,jacobian=power_stage.return_path_equations(self.case,self.settings,False)
        forward,_=Fixed.electronics_bridge_diode
        for sign in (-1.,1.):
            outputs=[-forward-0.1,self.case['bus_nominal_V']+forward+0.1]
            state=np.array([sign*2.,0.,0.,*outputs[::int(sign)],0.,0.])
            self.assertLess(sign*derivative(0.,state)[0],0.)
            np.testing.assert_array_equal(derivative(0.,state),derivative(0.3/Fixed.driver_pwm_frequency,state))
            direction=np.arange(1.,8.)
            displacement=1e-5*direction
            measured=(derivative(0.,state+displacement)-derivative(0.,state-displacement))/2e-5
            np.testing.assert_allclose(measured,jacobian(0.,state)@direction,rtol=1e-7,atol=1e-6)

    def test_storage_and_dissipation_equal_bus_power(self):
        from methodology.electronics.power_stage import unipolar_leg_duties,bridge_gate_edges,periodic_gate

        bus=self.case['bus_nominal_V']
        shunt=Fixed.electronics_readout_trial_shunt/2
        capacitance=self.settings['fet_output_capacitance_F']
        bypass=np.array([Fixed.electronics_integrated_bus_bypass+Fixed.electronics_channel_bus_capacitance,
                         Fixed.electronics_integrated_bus_bypass])
        duties=unipolar_leg_duties(self.settings['command_V'],bus)
        gates=[bridge_gate_edges(duty,Fixed.electronics_drv8962_input_delay,Fixed.electronics_drv8962_deadtime) for duty in duties]
        off=1/Fixed.electronics_bridge_off_resistance
        forward,resistance=Fixed.electronics_bridge_diode
        for state in map(np.array,([2.,-.01,.01,1.,23.,0.,0.],[2.,-.01,.01,-2.,26.,0.,0.])):
            for time in np.linspace(0,1/Fixed.driver_pwm_frequency,7):
                rates=self.derivative(time,state)
                storage=self.case['minimum_coil_L_H']*state[0]*rates[0]
                loss=self.case['coil_R_ohm']*state[0]**2
                supplied=0.
                for index,(high_start,high_width,low_start,low_width) in enumerate(gates):
                    ground,output=state[1+index],state[3+index]
                    ground_rate,output_rate=rates[1+index],rates[3+index]
                    high_gate=periodic_gate(time,high_start,high_width)
                    low_gate=1-periodic_gate(time,low_start,low_width)
                    high=(2*off+(1/Fixed.electronics_drv8962_on_resistance-off)*high_gate)*(bus-output)-max(output-bus-forward,0)*(1/resistance-off)
                    low=(2*off+(1/Fixed.electronics_drv8962_on_resistance-off)*low_gate)*(output-ground)-max(ground-output-forward,0)*(1/resistance-off)
                    storage+=capacitance*((output-bus)*output_rate+(output-ground)*(output_rate-ground_rate))+bypass[index]*(ground-bus)*ground_rate
                    loss+=high*(bus-output)+low*(output-ground)+ground**2/shunt
                    supplied+=bus*(high-capacitance*output_rate-bypass[index]*ground_rate)
                self.assertAlmostEqual(storage+loss,supplied,places=7)

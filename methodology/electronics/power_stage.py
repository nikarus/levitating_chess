import argparse
import json
from pathlib import Path
import shutil
import subprocess
from zipfile import ZIP_DEFLATED, ZipFile

import numpy as np

import model
from model import Inputs, Fixed
from methodology import design, prototype_coils, qualification, source_fingerprints


def unipolar_leg_duties(mean_voltage, bus_voltage):
    if bus_voltage <= 0 or not np.isfinite([mean_voltage, bus_voltage]).all() or abs(mean_voltage) > bus_voltage:
        raise ValueError('Unipolar bridge voltage must be within a positive finite bus')
    modulation = mean_voltage / bus_voltage
    return (1 + modulation) / 2, (1 - modulation) / 2


def timer_leg_duties(mean_voltage,bus_voltage):
    counts=Fixed.electronics_logic_clock/(2*Fixed.driver_pwm_frequency)
    if not counts.is_integer():
        raise ValueError('Phase-correct PWM requires an integer half-period count')
    duties=unipolar_leg_duties(mean_voltage,bus_voltage)
    return tuple(np.rint(np.asarray(duties)*counts)/counts)


def periodic_cycle(resistance, inductance, low_voltage, high_voltage, duty, frequency, back_emf=0.):
    if resistance <= 0 or inductance <= 0 or frequency <= 0 or not 0 <= duty <= 1 or low_voltage >= high_voltage:
        raise ValueError('Invalid independent RL switching cycle')
    period = 1 / frequency
    on_duration, off_duration = duty * period, (1 - duty) * period
    after_on = qualification.rl_step(0., high_voltage, back_emf, resistance, inductance, on_duration)
    after_off = qualification.rl_step(after_on, low_voltage, back_emf, resistance, inductance, off_duration)
    low_current = after_off / -np.expm1(-resistance * period / inductance)
    high_current = qualification.rl_step(low_current, high_voltage, back_emf, resistance, inductance, on_duration)
    mean = (duty * high_voltage + (1 - duty) * low_voltage - back_emf) / resistance
    return dict(low_A=float(low_current), high_A=float(high_current), mean_A=float(mean),
        peak_deviation_A=float(max(abs(low_current - mean), abs(high_current - mean))))


def topology_cases():
    _, coil, winding = prototype_coils.prototype_geometry(Inputs.cad_coil_layout)
    inductance = design.winding_inductance(coil, winding) * min(Fixed.qualification_inductance_factors)
    voltage_limit = Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction * Fixed.usable_bus_voltage_fraction
    candidates = [('split_rail_half_bridge', Inputs.selected_bus_voltage)]
    candidates.extend(('unipolar_full_bridge', voltage) for voltage in Inputs.electronics_full_bridge_buses)
    cases = []
    for topology, bus in candidates:
        upper_bus = bus * (1 + Fixed.electronics_supply_tolerance)
        lower_bus = bus * (1 - Fixed.electronics_supply_tolerance)
        frequency = Fixed.driver_pwm_frequency * (2 if topology == 'unipolar_full_bridge' else 1)
        low_voltage = -upper_bus / 2 if topology == 'split_rail_half_bridge' else 0.
        high_voltage = upper_bus / 2 if topology == 'split_rail_half_bridge' else upper_bus
        available = lower_bus * Fixed.usable_bus_voltage_fraction * (0.5 if topology == 'split_rail_half_bridge' else 1.)
        samples = []
        for mean_voltage in np.linspace(0., voltage_limit, Fixed.electronics_pwm_screen_points):
            duty = (mean_voltage - low_voltage) / (high_voltage - low_voltage)
            cycle = periodic_cycle(winding.resistance, inductance, low_voltage, high_voltage, duty, frequency)
            samples.append(dict(mean_voltage_V=float(mean_voltage), duty=float(duty), **cycle))
        worst = max(samples, key=lambda row: row['peak_deviation_A'])
        bound = design.split_rail_ripple_peak(winding.resistance, inductance, (high_voltage - low_voltage) / 2, frequency)
        cases.append(dict(topology=topology, bus_nominal_V=bus, bus_high_V=upper_bus, bus_low_V=lower_bus,
            terminal_voltage_limit_V=voltage_limit, guaranteed_usable_voltage_V=available,
            voltage_headroom_pass=available >= voltage_limit,
            coil_R_ohm=winding.resistance, minimum_coil_L_H=inductance,
            switching_levels_V=[low_voltage, high_voltage], ripple_frequency_Hz=frequency,
            per_leg_carrier_Hz=Fixed.driver_pwm_frequency,
            worst=worst, samples=samples, all_duty_ripple_bound_A=bound,
            ripple_pass=bound <= Fixed.driver_max_ripple_current,
            conservative_switch_peak_A=Fixed.driver_channel_current + bound,
            switch_peak_pass=Fixed.driver_channel_current + bound <= Fixed.driver_peak_current))
    return cases


def spice_reference(case, output, executable, refinement):
    period = 1 / case['ripple_frequency_Hz']
    edge = period * Fixed.electronics_spice_edge_fraction
    high_duration = case['worst']['duty'] * period
    if not edge < high_duration < period - edge:
        raise ValueError('SPICE pulse requires a non-degenerate duty cycle')
    step = period / (Fixed.electronics_spice_steps_per_cycle * refinement)
    stop = Fixed.electronics_spice_cycles * period
    name = f"{case['topology']}_{case['bus_nominal_V']:g}V_{refinement}"
    low, high = case['switching_levels_V']
    initial = case['worst']['low_A']
    supplies = f'Vdrive drive 0 PULSE({low:.17g} {high:.17g} 0 {edge:.17g} {edge:.17g} {high_duration-edge:.17g} {period:.17g})'
    return_node = '0'
    if case['topology'] == 'unipolar_full_bridge':
        carrier_period = 2 * period
        duty = case['worst']['duty']
        initial = qualification.rl_step(case['worst']['high_A'], low, 0., case['coil_R_ohm'],
            case['minimum_coil_L_H'], (1 - duty) * period / 2)
        leg_duties = unipolar_leg_duties(duty * high, high)
        supplies = '\n'.join(
            f'Vleg{leg} {node} 0 PULSE(0 {high:.17g} {(1-leg_duty)*carrier_period/2:.17g} '
            f'{edge:.17g} {edge:.17g} {leg_duty*carrier_period-edge:.17g} {carrier_period:.17g})'
            for leg, node, leg_duty in zip(('A', 'B'), ('drive', 'return'), leg_duties))
        return_node = 'return'
    voltage_expression = 'v(drive)' if return_node == '0' else 'v(drive)-v(return)'
    circuit = f"""Independent periodic RL reference; ideal differential drive, not a transistor model
{supplies}
Rcoil drive winding {case['coil_R_ohm']:.17g}
Lcoil winding {return_node} {case['minimum_coil_L_H']:.17g} IC={initial:.17g}
.options reltol={Fixed.electronics_spice_relative_tolerance:.17g} abstol={Fixed.electronics_spice_absolute_tolerance:.17g}
.control
set wr_singlescale
set wr_vecnames
set numdgt=17
tran {step:.17g} {stop:.17g} 0 {step:.17g} uic
let differential_voltage = {voltage_expression}
wrdata {name}.dat i(Lcoil) differential_voltage
quit
.endc
.end
"""
    netlist = output / f'{name}.cir'
    netlist.write_text(circuit)
    result = subprocess.run([executable, '-b', netlist.name], cwd=output, capture_output=True, text=True, check=True)
    (output / f'{name}.log').write_text(result.stdout + result.stderr)
    if 'error' in result.stderr.lower():
        raise ValueError('ngspice reported an error; inspect the saved log')
    waveform = np.loadtxt(output / f'{name}.dat', skiprows=1)
    start = stop - period
    selected = (waveform[:, 0] > start) & (waveform[:, 0] < stop)
    times = np.r_[start, waveform[selected, 0], stop]
    currents = np.interp(times, waveform[:, 0], waveform[:, 1])
    mean = float(np.trapezoid(currents, times) / period)
    peak = float(np.max(np.abs(currents - mean)))
    analytic = case['worst']
    deviation = max(abs(mean - analytic['mean_A']), abs(peak - analytic['peak_deviation_A']))
    return dict(netlist=netlist.name, refinement=refinement, mean_A=mean,
        peak_deviation_A=peak, rms_A=float(np.sqrt(np.trapezoid(currents ** 2, times) / period)),
        maximum_analytic_difference_A=deviation, passed=deviation <= Fixed.electronics_spice_current_tolerance)



def bridge_gate_edges(duty, delay, deadtime):
    period, edge = 1 / Fixed.driver_pwm_frequency, Fixed.driver_switching_time
    rise, fall = (1-duty)*period/2+delay, (1+duty)*period/2+delay
    high_start, high_width = rise+edge+deadtime, fall-rise-deadtime-2*edge
    low_start, low_width = rise, fall-rise+deadtime
    if high_width <= 0 or low_width+2*edge >= period or min(high_start, low_start) < 0:
        raise ValueError('Dead time and switching edges leave no valid gate interval')
    return high_start, high_width, low_start, low_width


def deadtime_voltage_drop(bus_voltage):
    return 2*Fixed.electronics_drv8962_deadtime*Fixed.driver_pwm_frequency*(bus_voltage+2*Fixed.electronics_bridge_diode[0])


def return_path_netlist(case, current_request, leg_skew, bulk_return, refinement, physical_only=False, output_capacitance=Fixed.electronics_bridge_output_capacitances[0]):
    if bulk_return not in ('ground', 'pgndA'):
        raise ValueError('Unreviewed bulk-capacitor return')
    period, edge = 1 / Fixed.driver_pwm_frequency, Fixed.driver_switching_time
    shunt = Fixed.electronics_readout_trial_shunt / 2
    on_resistance = Fixed.electronics_drv8962_on_resistance
    forward, diode_resistance = Fixed.electronics_bridge_diode
    duty_command = current_request*(case['coil_R_ohm']+2*on_resistance+shunt) + np.sign(current_request)*deadtime_voltage_drop(case['bus_nominal_V'])
    if abs(duty_command) > case['terminal_voltage_limit_V']:
        raise ValueError('Diagnostic command exceeds the existing voltage limit')
    duties = unipolar_leg_duties(duty_command, case['bus_nominal_V'])
    settling_cycles = int(np.ceil(Fixed.electronics_bridge_settling_constants * case['minimum_coil_L_H'] /
                                 case['coil_R_ohm'] / period))
    start = settling_cycles * period
    stop = start + Fixed.electronics_spice_cycles * period
    step = period / (Fixed.electronics_bridge_spice_steps_per_cycle * refinement)
    relative_tolerance = Fixed.electronics_spice_relative_tolerance if physical_only else Fixed.electronics_readout_spice_relative_tolerance
    lines = []
    for leg, duty, skew in zip(('A','B'), duties, (0., leg_skew)):
        high_start, high_width, low_start, low_width = bridge_gate_edges(duty,
            Fixed.electronics_drv8962_input_delay+skew, Fixed.electronics_drv8962_deadtime)
        lines.extend((
            f'Vhigh{leg} high{leg} 0 PULSE(0 1 {high_start:.17g} {edge:.17g} {edge:.17g} {high_width:.17g} {period:.17g})',
            f'Vlow{leg} low{leg} 0 PULSE(1 0 {low_start:.17g} {edge:.17g} {edge:.17g} {low_width:.17g} {period:.17g})',
            f'Rshunt{leg} pgnd{leg} 0 {shunt:.17g}',
            f'Vcap{leg} bus cap{leg} 0',
            f'Cvm{leg} cap{leg} pgnd{leg} {Fixed.electronics_integrated_bus_bypass:.17g} IC={case["bus_nominal_V"]:.17g}',
            f'Vreturn{leg} out{leg} lowterminal{leg} 0'))
        for side, drain, source in (('high','bus',f'out{leg}'),('low',f'lowterminal{leg}',f'pgnd{leg}')):
            lines.extend((
                f'Bfet{side}{leg} {drain} {source} I=V({drain},{source})*(1/{Fixed.electronics_bridge_off_resistance:.17g}+(1/{on_resistance:.17g}-1/{Fixed.electronics_bridge_off_resistance:.17g})*V({side}{leg}))',
                f'Adiode{side}{leg} {source} {drain} body_diode',
                f'Coss{side}{leg} {drain} {source} {output_capacitance:.17g} IC={case["bus_nominal_V"] if side=="high" else 0.:.17g}'))
    bulk_node = '0' if bulk_return == 'ground' else bulk_return
    gain = next(row[1] for row in Fixed.electronics_readout_specs if row[0]=='INA181A1')
    amplifier = f'Ereadout output reference pgndB pgndA {gain:.17g}' if physical_only else 'Xamp pgndB pgndA supply 0 reference output INA181A1'
    netlist = f"""Representative DRV8962 current paths; not a vendor driver model
.include ina181a1.lib
.model body_diode sidiode(Ron={diode_resistance:.17g} Roff={Fixed.electronics_bridge_off_resistance:.17g} Vfwd={forward:.17g})
Vbus bus 0 {case['bus_nominal_V']:.17g}
{chr(10).join(lines)}
Vcapbulk bus capbulk 0
Cbulk capbulk {bulk_node} {Fixed.electronics_channel_bus_capacitance:.17g} IC={case['bus_nominal_V']:.17g}
Rcoil outA winding {case['coil_R_ohm']:.17g}
Lcoil winding outB {case['minimum_coil_L_H']:.17g}
Vs supply 0 {Fixed.electronics_analog_supply:.17g}
Vref reference 0 {Fixed.electronics_current_reference:.17g}
{amplifier}
Rload output 0 {Fixed.electronics_readout_test_load:.17g}
.options {Fixed.electronics_bridge_spice_solver} method={Fixed.electronics_readout_spice_method} maxord={Fixed.electronics_readout_spice_order} reltol={relative_tolerance:.17g} vntol={Fixed.electronics_bridge_voltage_tolerance:.17g} chgtol={Fixed.electronics_bridge_charge_tolerance:.17g} abstol={Fixed.electronics_bridge_absolute_tolerance/refinement:.17g}
.control
set wr_singlescale
set wr_vecnames
set numdgt=17
tran {step:.17g} {stop:.17g} {start:.17g} {step:.17g}
let reported_current = (v(output)-{Fixed.electronics_current_reference:.17g})/{gain*shunt:.17g}
let shunt_difference_current = (v(pgndB)-v(pgndA))/{shunt:.17g}
let source_low_current = -i(VreturnA)
let sink_low_current = i(VreturnB)
let capacitor_difference_current = i(VcapB)-i(VcapA){'-i(Vcapbulk)' if bulk_return=='pgndA' else ''}
let kcl_error = shunt_difference_current-source_low_current-sink_low_current-capacitor_difference_current
wrdata transient.dat i(Lcoil) reported_current shunt_difference_current v(pgndA) v(pgndB) kcl_error source_low_current sink_low_current capacitor_difference_current v(outA) v(outB)
quit
.endc
.end
"""
    return netlist, dict(start_s=start, stop_s=stop, command_V=duty_command,
        maximum_step_s=step, on_resistance_ohm=on_resistance,
        requested_current_A=current_request, relative_leg_delay_s=leg_skew, bulk_return=bulk_return,
        fet_output_capacitance_F=output_capacitance)


def periodic_gate(time, start, width):
    phase=(time-start)%(1/Fixed.driver_pwm_frequency)
    edge=Fixed.driver_switching_time
    return min(max(phase/edge,0.),1.)-min(max((phase-edge-width)/edge,0.),1.)


def winding_current_rate(state, case):
    values=np.asarray(state)
    return (values[...,3]-values[...,4]-case['coil_R_ohm']*values[...,0])/case['minimum_coil_L_H']


def return_path_equations(case, settings, enabled=True):
    shunt=Fixed.electronics_readout_trial_shunt/2
    duties=unipolar_leg_duties(settings['command_V'],case['bus_nominal_V'])
    gates=[bridge_gate_edges(duty,Fixed.electronics_drv8962_input_delay+delay,Fixed.electronics_drv8962_deadtime)
           for duty,delay in zip(duties,(0.,settings['relative_leg_delay_s']))]
    capacitances=np.full(2,Fixed.electronics_integrated_bus_bypass)
    if settings['bulk_return']=='pgndA':
        capacitances[0]+=Fixed.electronics_channel_bus_capacitance

    output_capacitance=settings['fet_output_capacitance_F']
    inverses=[np.linalg.inv(np.array([[2*output_capacitance,-output_capacitance],
                       [-output_capacitance,output_capacitance+capacitance]])) for capacitance in capacitances]

    off=1/Fixed.electronics_bridge_off_resistance
    forward,resistance=Fixed.electronics_bridge_diode
    diode_conductance=1/resistance-off

    def conductances(time, index):
        if not enabled:
            return 2*off,2*off
        high_start,high_width,low_start,low_width=gates[index]
        high=periodic_gate(time,high_start,high_width)
        low=1-periodic_gate(time,low_start,low_width)
        return (2*off+(1/Fixed.electronics_drv8962_on_resistance-off)*high,
                2*off+(1/Fixed.electronics_drv8962_on_resistance-off)*low)

    def derivative(time, state):
        rates=[]
        for index,(current,ground,output) in enumerate(zip((state[0],-state[0]),state[1:3],state[3:5])):
            high_conductance,low_conductance=conductances(time,index)
            low_current=low_conductance*(output-ground)-max(ground-output-forward,0)*diode_conductance
            high_current=high_conductance*(case['bus_nominal_V']-output)-max(output-case['bus_nominal_V']-forward,0)*diode_conductance
            rates.append(inverses[index] @ [high_current-low_current-current,low_current-ground/shunt])
        return np.r_[winding_current_rate(state,case),
            rates[0][1],rates[1][1],rates[0][0],rates[1][0],state[0],(state[2]-state[1])/shunt]

    def jacobian(time, state):
        matrix=np.zeros((7,7))
        matrix[0,[0,3,4]]=np.array([-case['coil_R_ohm'],1.,-1.])/case['minimum_coil_L_H']
        for index,(ground,output) in enumerate(zip(state[1:3],state[3:5])):
            high,low=conductances(time,index)
            high+=diode_conductance*(output>case['bus_nominal_V']+forward)
            low+=diode_conductance*(ground>output+forward)
            columns=inverses[index] @ np.array([[-(1-2*index),low,-high-low],
                                                [0.,-low-1/shunt,low]])
            matrix[np.ix_([3+index,1+index],[0,1+index,3+index])]=columns
        matrix[5,0]=1.
        matrix[6,1:3]=np.array([-1.,1.])/shunt
        return matrix

    return derivative,jacobian


def independent_return_period(case, settings, initial, start, times, refinement):
    from scipy.integrate import solve_ivp

    period=1/Fixed.driver_pwm_frequency
    derivative,jacobian=return_path_equations(case,settings)
    initial=np.r_[initial,0.,0.]
    tolerance=np.array([Fixed.electronics_spice_absolute_tolerance,
        Fixed.electronics_bridge_voltage_tolerance,Fixed.electronics_bridge_voltage_tolerance,
        Fixed.electronics_bridge_voltage_tolerance,Fixed.electronics_bridge_voltage_tolerance,
        Fixed.electronics_spice_absolute_tolerance*period,Fixed.electronics_spice_absolute_tolerance*period])/refinement
    solution=solve_ivp(derivative,(start,start+period),initial,method='Radau',jac=jacobian,
        rtol=Fixed.electronics_spice_relative_tolerance/refinement,atol=tolerance,
        max_step=period/Fixed.electronics_spice_steps_per_cycle/refinement,dense_output=True)
    if not solution.success:
        raise ValueError(solution.message)
    return dict(time=times,states=solution.sol(times).T,
        native_time=solution.t,native_states=solution.y.T,
        final_state=solution.y[:5,-1],
        carrier_means_A=solution.y[5:,-1]/period,
        final_current_difference_A=float(solution.y[0,-1]-initial[0]),
        evaluations=solution.nfev)


def run(output, executable):
    if output.exists():
        raise ValueError('Preserve existing power-stage evidence')
    executable = str(Path(shutil.which(executable)).resolve())
    cases = topology_cases()
    output.mkdir(parents=True)
    fingerprints = source_fingerprints(__file__, model.__file__, prototype_coils.__file__, design.__file__, qualification.__file__)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for filename in fingerprints:
            archive.write(filename, filename)
    for case in cases:
        checks = [spice_reference(case, output, executable, refinement) for refinement in (1, 2)]
        difference = max(abs(checks[0][key] - checks[1][key]) for key in ('mean_A', 'peak_deviation_A', 'rms_A'))
        case['independent_spice'] = dict(checks=checks, refinement_difference_A=difference,
            passed=all(check['passed'] for check in checks) and difference <= Fixed.electronics_spice_current_tolerance)
    report = dict(status='ideal_switching_topology_screen_not_complete_circuit_validation',
        source_sha256=fingerprints,
        ngspice_version=subprocess.run([executable, '--version'], capture_output=True, text=True, check=True).stdout,
        bounds=dict(allowed_peak_ripple_A=Fixed.driver_max_ripple_current, peak_device_current_A=Fixed.driver_peak_current,
            cycle_average_current_A=Fixed.driver_channel_current, supply_tolerance_fraction=Fixed.electronics_supply_tolerance),
        cases=cases, independent_spice_passed=all(case['independent_spice']['passed'] for case in cases),
        limitations=[
            'Independent linear RL coil with constant back-EMF during a carrier period; no mutual inductance or magnetic saturation.',
            'Self-inductance remains an unmeasured geometry estimate with the configured lower-bound factor.',
            'Worst ripple is screened over the configured terminal-voltage range; back-EMF shifts mean current without changing periodic ripple.',
            'Full-bridge result requires center-aligned unipolar modulation with two equal differential pulses per carrier cycle; ordinary sign-magnitude PWM has a different ripple frequency.',
            'No MOSFET, gate-driver, dead-time, parasitic, power-distribution or current-acquisition model is included.',
            'A periodic ripple bound does not qualify transient regulation or current reconstruction.',
            'Voltage and current extrema are independent conservative requirements, not an asserted steady physical workload.',
            'No flight simulator settings, source files in the other checkout or original requirements have been changed.'])
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    if not report['independent_spice_passed']:
        raise ValueError('Independent SPICE or refinement check failed; retain evidence')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Screen power-stage ripple against an independent ngspice RL reference.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ngspice', required=True)
    arguments = parser.parse_args()
    run(arguments.output, arguments.ngspice)

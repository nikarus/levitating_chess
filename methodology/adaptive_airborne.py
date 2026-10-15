import argparse
from datetime import datetime
import json
from pathlib import Path

import numpy as np

from model import Fixed
from methodology import airborne, airborne_verification, sensing, source_fingerprints
from methodology.cad import magnetics


def initial_adaptation():
    return dict(accepted=0, rejected=0, force_evaluations=0, maximum_accepted_error=0., intervals=[],
        signed_energy_defect_J=0., absolute_energy_defect_J=0.)


def integration_energy_budget(plant):
    span = plant.geometry.max_flight_gap - plant.geometry.gap
    budget = plant.model.body_mass[plant.king_id] * np.linalg.norm(plant.model.opt.gravity) * span * Fixed.adaptive_energy_relative_tolerance
    if not np.isfinite(budget) or budget <= 0:
        raise ValueError('Airborne energy accuracy requires a positive gravity/gap energy scale')
    return float(budget)


class AdaptiveAirborneSimulation(airborne.AirborneSimulation):
    checkpoint_schema = 'airborne-adaptive-v1'

    def __init__(self, construction, duration, disturbed=True, initialize=True, fixed_step=None, tolerance_scale=1, magnetic_refinement=None):
        super().__init__(construction, duration, disturbed=disturbed, initialize=initialize, magnetic_refinement=magnetic_refinement)
        if tolerance_scale <= 0 or not np.isfinite(tolerance_scale):
            raise ValueError('Adaptive tolerance scale must be finite and positive')
        if fixed_step is not None and (fixed_step <= 0 or not np.isfinite(fixed_step)):
            raise ValueError('Fixed reference step must be finite and positive')
        self.fixed_step = fixed_step
        self.tolerance_scale = tolerance_scale
        self.clock = 0.
        self.next_command = 0
        self.next_regulator = 0
        self.next_checkpoint = 1
        self.proposed_step = Fixed.adaptive_max_step
        self.last_checkpoint_due = False
        self.adaptation = initial_adaptation()
        self.source.update(source_fingerprints(__file__))

    def checkpoint_due(self):
        return self.last_checkpoint_due

    def physical_snapshot(self):
        return dict(plant=self.plant.integration_state(), period=self.plant.model.opt.timestep,
                    currents=self.drivers.currents.copy(), filtered_command=self.drivers.filtered_command.copy(),
                    peaks=self.peaks.copy(), stop_reason=self.stop_reason)

    def restore_physics(self, state):
        self.plant.model.opt.timestep = state['period']
        self.plant.restore_integration_state(state['plant'])
        self.drivers.currents = state['currents'].copy()
        self.drivers.filtered_command = state['filtered_command'].copy()
        self.peaks = state['peaks'].copy()
        self.stop_reason = state['stop_reason']

    def force_matrix(self):
        self.adaptation['force_evaluations'] += 1
        return self.plant.magnetic_matrix(window_center=self.reference[:2])

    def endpoint(self):
        total, coil = self.physical_fields()
        return dict(truth=self.plant.truth(), currents=self.drivers.currents.copy(), total=total, coil=coil)

    def field_error(self, initial, coarse, midpoint, fine, period):
        tau = self.drivers.inductance / self.drivers.resistance
        weight = -np.expm1(-period / 2 / tau) / -np.expm1(-period / tau)
        coarse_mid_coil = initial['coil'] + weight * (coarse['coil'] - initial['coil'])
        coarse_mid_magnet = (initial['total'] - initial['coil'] + coarse['total'] - coarse['coil']) / 2

        def filtered(start, end, interval, state):
            magnet_state, coil_state = state
            return (sensing.hall_filter_step(magnet_state, start['total'] - start['coil'], end['total'] - end['coil'], interval),
                    sensing.hall_filter_step(coil_state, start['coil'], end['coil'], interval, tau))

        zero = (np.zeros_like(initial['total']), np.zeros_like(initial['coil']))
        coarse_filtered = sum(filtered(initial, coarse, period, zero))
        fine_filtered = sum(filtered(midpoint, fine, period / 2, filtered(initial, midpoint, period / 2, zero)))
        return float(max(np.max(np.abs((fine['total'] - coarse['total']) * self.sensors.gain)),
            np.max(np.abs((midpoint['total'] - coarse_mid_coil - coarse_mid_magnet) * self.sensors.gain)),
            np.max(np.abs((fine_filtered - coarse_filtered) * self.sensors.gain))))

    def step_error(self, initial, coarse, midpoint, fine, period):
        left, right = coarse['truth'], fine['truth']
        differences = dict(
            position=float(np.linalg.norm(right['com_position_m'] - left['com_position_m'])),
            angle=float((right['rotation'] * left['rotation'].inv()).magnitude()),
            velocity=float(np.linalg.norm(right['linear_velocity_m_per_s'] - left['linear_velocity_m_per_s'])),
            angular_velocity=float(np.linalg.norm(right['angular_velocity_rad_per_s'] - left['angular_velocity_rad_per_s'])),
            current=float(np.max(np.abs(fine['currents'] - coarse['currents']))),
            field=self.field_error(initial, coarse, midpoint, fine, period))
        limits = dict(position=Fixed.adaptive_position_tolerance, angle=Fixed.adaptive_angle_tolerance,
            velocity=Fixed.adaptive_velocity_tolerance, angular_velocity=Fixed.adaptive_angular_velocity_tolerance,
            current=Fixed.adaptive_current_tolerance, field=Fixed.adaptive_field_tolerance)
        errors = {key: value / limits[key] / self.tolerance_scale for key, value in differences.items()}
        if not all(np.isfinite(value) for value in errors.values()):
            raise ValueError('Adaptive error estimate is nonfinite')
        return errors

    def append_endpoint(self, time, endpoint):
        self.field_times.append(time)
        self.field_values.append(endpoint['total'])
        self.coil_field_values.append(endpoint['coil'])

    def advance(self):
        if self.finished:
            raise ValueError('Cannot advance a finished adaptive trial')
        now = self.clock
        command_due = abs(now - self.next_command / Fixed.current_command_rate) <= Fixed.adaptive_clock_tolerance
        regulator_due = abs(now - self.next_regulator / Fixed.current_regulator_rate) <= Fixed.adaptive_clock_tolerance
        self.process_events(now, regulator_due, command_due, abs(now - self.duration) <= Fixed.adaptive_clock_tolerance)
        self.next_command += int(command_due)
        self.next_regulator += int(regulator_due)
        self.last_checkpoint_due = False
        if self.finished:
            return
        boundary = min(self.next_command / Fixed.current_command_rate, self.next_regulator / Fixed.current_regulator_rate,
            sensing.hall_available_time(self.next_scan / Fixed.pose_feedback_rate, len(self.fields.points)),
            self.next_checkpoint * Fixed.airborne_checkpoint_interval, self.duration)
        requested_period = self.proposed_step if self.fixed_step is None else self.fixed_step
        period = min(requested_period, boundary - now)
        boundary_clipped = boundary - now < requested_period - Fixed.adaptive_clock_tolerance
        if boundary - (now + period) <= Fixed.adaptive_clock_tolerance:
            period = boundary - now
        if period <= 0:
            raise ValueError('Adaptive scheduler failed to advance time')
        energy_budget = integration_energy_budget(self.plant) * self.tolerance_scale
        self.adaptation['energy_budget_J'] = energy_budget
        initial = self.endpoint()
        matrix = self.force_matrix()
        start = self.physical_snapshot()
        errors = {}
        for attempt in range(Fixed.adaptive_max_retries + 1):
            self.advance_physics(period, matrix)
            if self.fixed_step is not None:
                energy_defects = [self.integration_energy_defect]
                endpoint = self.endpoint()
                self.observe_physics(endpoint['truth'])
                accepted = [(now + period, endpoint)]
                break
            coarse = self.endpoint()
            self.restore_physics(start)
            self.advance_physics(period / 2, matrix)
            energy_defects = [self.integration_energy_defect]
            midpoint = self.endpoint()
            self.observe_physics(midpoint['truth'])
            if self.stop_reason is not None:
                period /= 2
                accepted = [(now + period, midpoint)]
                break
            self.advance_physics(period / 2, self.force_matrix())
            energy_defects.append(self.integration_energy_defect)
            fine = self.endpoint()
            self.observe_physics(fine['truth'])
            errors = self.step_error(initial, coarse, midpoint, fine, period)
            errors['energy'] = float(np.sum(np.abs(energy_defects)) / (energy_budget * period / self.duration))
            if not np.isfinite(errors['energy']):
                raise ValueError('Adaptive energy defect is nonfinite')
            maximum = max(errors.values())
            if maximum <= 1 or self.stop_reason is not None:
                accepted = [(now + period / 2, midpoint), (now + period, fine)]
                if self.stop_reason is None:
                    self.adaptation['maximum_accepted_error'] = max(self.adaptation['maximum_accepted_error'], maximum)
                break
            self.adaptation['rejected'] += 1
            self.restore_physics(start)
            if attempt == Fixed.adaptive_max_retries or period / 2 < Fixed.adaptive_min_step:
                self.stop_reason = 'adaptive_accuracy_unresolved'
                self.finished = True
                self.adaptation['failed_attempt'] = dict(time_s=now, attempted_step_s=period, errors=errors)
                return
            period /= 2
        self.clock = boundary if period == boundary - now else now + period
        self.plant.data.time = self.clock
        accepted[-1] = (self.clock, accepted[-1][1])
        for time, endpoint in accepted:
            self.append_endpoint(time, endpoint)
        self.step += 1
        self.adaptation['accepted'] += 1
        signed_energy, absolute_energy = float(np.sum(energy_defects)), float(np.sum(np.abs(energy_defects)))
        self.adaptation['signed_energy_defect_J'] += signed_energy
        self.adaptation['absolute_energy_defect_J'] += absolute_energy
        self.adaptation['intervals'].append(dict(start_s=now, interval_s=period, errors=errors,
            signed_energy_defect_J=signed_energy, absolute_energy_defect_J=absolute_energy))
        self.proposed_step = min(Fixed.adaptive_max_step, requested_period / 2 ** attempt
            * (2 if errors and max(errors.values()) < Fixed.adaptive_growth_threshold else 1))
        if boundary_clipped and attempt == 0:
            self.proposed_step = requested_period
        if abs(self.clock - self.next_checkpoint * Fixed.airborne_checkpoint_interval) <= Fixed.adaptive_clock_tolerance:
            self.last_checkpoint_due = True
            self.next_checkpoint += 1
        if self.stop_reason is not None:
            self.process_events(self.clock, False, False, False)

    def checkpoint_state(self):
        state = super().checkpoint_state()
        state['schema'] = self.checkpoint_schema
        state['adaptive'] = {name: getattr(self, name) for name in ('fixed_step', 'tolerance_scale', 'clock',
            'next_command', 'next_regulator', 'next_checkpoint', 'proposed_step', 'last_checkpoint_due', 'adaptation')}
        return state

    def load_checkpoint(self, path):
        state = json.loads(path.read_text())
        self.validate_checkpoint(state)
        self.restore_checkpoint(state)

    def validate_checkpoint(self, state):
        if state['schema'] != self.checkpoint_schema or state['source_sha256'] != self.source:
            raise ValueError('Adaptive checkpoint schema or source differs')
        if (state['duration_s'] != self.duration or state['disturbed'] != self.disturbed
                or state['magnetic_refinement'] != self.magnetic_refinement
                or state['adaptive']['fixed_step'] != self.fixed_step or state['adaptive']['tolerance_scale'] != self.tolerance_scale):
            raise ValueError('Adaptive checkpoint scenario differs')

    def restore_checkpoint(self, state):
        self.plant.model.opt.timestep = state['plant']['time_step_s']
        super().restore_checkpoint(state)
        for name, value in state['adaptive'].items():
            setattr(self, name, value)

    def report(self):
        report = super().report()
        report.update(physics_time_step_s=None, integration_mode='adaptive' if self.fixed_step is None else 'fixed_event_aligned',
                      fixed_reference_step_s=self.fixed_step, tolerance_scale=self.tolerance_scale, adaptation=self.adaptation)
        report['limitations'].append('Step doubling estimates local state/field error. Absolute mechanical-work defects are budgeted over the requested duration; this does not bound global state or magnetic error. Hardware events occur at exact scheduled times. The energy criterion excludes contact and unaccounted external work.')
        return report


def benchmark(output, deadline):
    if output.exists():
        raise ValueError('Adaptive benchmark output already exists; preserve its evidence')
    output.mkdir(parents=True)
    construction = magnetics.ConstructionModel()
    reports = {}
    summary = dict(scope='Short adaptive-step accuracy and cost benchmark; not hover qualification',
                   status='running', cases={}, comparisons={}, source_sha256=source_fingerprints(__file__, airborne_verification.__file__))
    for name, fixed_step, scale in (('adaptive', None, 1), ('reference', Fixed.adaptive_reference_step, 1),
                                    ('tighter', None, 1 / Fixed.qualification_refinement_factor)):
        summary['active_case'] = name
        airborne_verification.save_summary(output / 'summary.json', summary)
        simulation = AdaptiveAirborneSimulation(construction, Fixed.adaptive_benchmark_duration,
                                                fixed_step=fixed_step, tolerance_scale=scale)
        report = airborne.run_simulation(simulation, output / (name + '.json'), output / (name + '_checkpoint.json'), deadline=deadline)
        reports[name] = report
        summary['cases'][name] = {key: report[key] for key in ('passed', 'finished', 'stop_reason', 'elapsed_s', 'peaks')}
        summary['cases'][name]['adaptation'] = {key: value for key, value in report['adaptation'].items() if key != 'intervals'}
        if not report['passed']:
            summary['status'] = 'trial_failed'
            break
        if name != 'adaptive':
            summary['comparisons'][name] = airborne_verification.compare_trials(reports['adaptive'], report)
            if not summary['comparisons'][name]['converged']:
                summary['status'] = 'numerical_comparison_failed'
                break
    else:
        summary['status'] = 'completed'
    summary['active_case'] = None
    airborne_verification.save_summary(output / 'summary.json', summary)
    print(json.dumps(summary), flush=True)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Benchmark adaptive physics with exact hardware event timing.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--deadline', type=datetime.fromisoformat, required=True)
    arguments = parser.parse_args()
    if arguments.deadline.tzinfo is None:
        parser.error('Deadline must include a timezone')
    benchmark(arguments.output, arguments.deadline.timestamp())

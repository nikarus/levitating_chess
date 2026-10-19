import argparse
from pathlib import Path
from time import perf_counter
from types import SimpleNamespace

import magpylib as magpy
import numpy as np
from numpy.polynomial.legendre import leggauss
from scipy.integrate import OdeSolution, quad, solve_ivp
from scipy.linalg import expm
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Inputs, Fixed, Constants
from methodology import adaptive_airborne, airborne, airborne_verification, electrical, magnetic_atlas, motion, physics_verification, sensing, source_fingerprints
from methodology.cad import magnetics


def magnet_surface_quadrature(source, order):
    nodes, weights = leggauss(order)
    coordinates = np.stack(np.meshgrid(nodes, nodes, indexing='ij'), axis=-1).reshape(-1, 2)
    area_weights = np.outer(weights, weights).ravel()
    points, charges = [], []
    for magnet in source.sources_all:
        if not isinstance(magnet, magpy.magnet.Cuboid):
            raise ValueError('Magnet surface quadrature requires uniformly polarized cuboids')
        for axis in np.flatnonzero(magnet.polarization):
            tangents = np.delete(np.arange(3), axis)
            face_weights = area_weights * np.prod(magnet.dimension[tangents]) / 4
            for sign in (-1, 1):
                local = np.zeros((len(coordinates), 3))
                local[:, axis] = sign * magnet.dimension[axis] / 2
                local[:, tangents] = coordinates * magnet.dimension[tangents] / 2
                points.append(magnet.orientation.apply(local) + magnet.position)
                charges.append(sign * magnet.polarization[axis] / Constants.vacuum_permeability * face_weights)
    if not points:
        raise ValueError('Magnet surface quadrature requires nonzero magnetization')
    return np.concatenate(points), np.concatenate(charges)


def magnet_surface_wrench(points, charges, center, fields):
    if (points.ndim != 2 or points.shape[1] != 3 or fields.shape != points.shape
            or charges.shape != (len(points),) or np.shape(center) != (3,)
            or not all(np.isfinite(value).all() for value in (points, charges, center, fields))):
        raise ValueError('Magnet surface wrench requires finite matching samples and a three-axis torque anchor')
    forces = charges[:, None] * fields
    return np.r_[forces.sum(axis=0), np.cross(points - center, forces).sum(axis=0)]


def surface_flux(source, wire, order):
    vertices = wire.orientation.apply(wire.vertices) + wire.position
    if not np.array_equal(vertices[0], vertices[-1]):
        raise ValueError('Flux reference requires a closed winding')
    nodes, weights = leggauss(order)
    nodes, weights = (nodes + 1) / 2, weights / 2
    u, v = np.meshgrid(nodes, nodes, indexing='ij')
    weights = np.outer(weights, weights) * (1 - u)
    origin = vertices[0]
    edges = vertices[1:-2] - origin
    next_edges = vertices[2:-1] - origin
    points = origin + u[None, :, :, None] * edges[:, None, None, :] + ((1 - u) * v)[None, :, :, None] * next_edges[:, None, None, :]
    fields = levitation_sim.magnetic_field(source, points.reshape(-1, 3)).reshape(points.shape)
    oriented_area = np.cross(edges, next_edges)
    return float(wire.current * np.sum(np.einsum('tuvc,tc->tuv', fields, oriented_area) * weights))


def curved_surface_flux(source, points, elements, anchor, order):
    nodes, weights = leggauss(order)
    nodes, weights = (nodes + 1) / 2, weights / 2
    directions = points - anchor
    samples = anchor + nodes[:, None, None] * directions[None, :, :]
    fields = levitation_sim.magnetic_field(source, samples.reshape(-1, 3)).reshape(samples.shape)
    return float(np.sum(np.einsum('upc,pc->up', fields, np.cross(directions, elements)) * (nodes * weights)[:, None]))


def flux_wrench(source, wire, center, order, refinement, flux_evaluator=surface_flux):
    derivatives = []
    for axis in range(6):
        step = (Fixed.hall_position_derivative_step if axis < 3 else Fixed.angle_derivative_step) / refinement
        fluxes = []
        for sign in (-1, 1):
            moved = source.copy()
            direction = np.eye(3)[axis % 3] * sign * step
            if axis < 3:
                moved.move(direction)
            else:
                moved.rotate(Rotation.from_rotvec(direction), anchor=center)
            fluxes.append(flux_evaluator(moved, wire, order))
        derivatives.append((fluxes[1] - fluxes[0]) / (2 * step))
    return np.array(derivatives)


def verify_flux(construction):
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    layout = levitation_sim.magnet_layout_from_geometry()
    angles = np.radians(Fixed.motion_check_orientation_deg)
    _, center = levitation_sim.place_piece(layout, z=(geometry.max_flight_gap - geometry.gap) / 2, **dict(zip(('roll', 'pitch', 'yaw'), angles)))
    half_width, half_length = array.short / 2, array.long / 2
    vertices = np.array([[-half_width, -half_length, 0], [half_width, -half_length, 0],
        [half_width, half_length, 0], [-half_width, half_length, 0], [-half_width, -half_length, 0]])
    filament = array.coils[0][0]
    plane = (filament.orientation.apply(filament.vertices) + filament.position)[0, 2]
    position = np.array([array.short_pitch, array.short_pitch, 0]) * Fixed.pose_phase_fractions[1]
    position[2] = plane
    wire = magpy.current.Polyline(vertices=vertices, position=position, current=1., meshing=Fixed.nominal_force_mesh)
    rows = []
    for refinement in Fixed.coupling_refinements:
        wire.meshing = Fixed.nominal_force_mesh * refinement
        matrix = levitation_sim.CoilForceQuadrature([[wire]]).matrix(layout.collection, center)[:, 0]
        fluxes = [flux_wrench(layout.collection, wire, center, order, refinement) for order in Fixed.coupling_flux_orders]
        metrics = physics_verification.matrix_refinement_metrics(matrix[:, None], fluxes[-1][:, None], Fixed.driver_channel_current)
        surface_metrics = physics_verification.matrix_refinement_metrics(fluxes[-2][:, None], fluxes[-1][:, None], Fixed.driver_channel_current)
        rows.append(dict(refinement=refinement, line_wrench_per_A=matrix.tolist(), flux_wrenches_per_A=[value.tolist() for value in fluxes],
            metrics=metrics, surface_refinement=surface_metrics))
    keys = ('force_relative_column_error', 'torque_relative_column_error')
    passed = all(rows[-1][group][key] <= Fixed.coupling_relative_tolerance for group in ('metrics', 'surface_refinement') for key in keys)
    return dict(scope='Closed rectangular test winding and rotated permanent-magnet array; flux surface integral versus production Lorentz reaction',
        independent_of='Force-map assembly, Lorentz line quadrature and torque anchor; the permanent-magnet field library is shared.',
        center_m=center.tolist(), wire_vertices_m=(wire.orientation.apply(wire.vertices) + wire.position).tolist(),
        source_state=magnetic_atlas.local_source_state(layout.collection).tolist(),
        relative_limit=Fixed.coupling_relative_tolerance, cases=rows, passed=bool(passed))


def interval_energy(initial, voltage, emf, resistance, inductance, period, enabled=True):
    equilibrium = (voltage - emf) / resistance
    stop = period
    if not enabled:
        if initial == 0:
            stop = 0.
        elif initial * equilibrium < 0:
            stop = min(period, inductance / resistance * np.log1p(-initial / equilibrium))
    def current(time):
        return equilibrium + (initial - equilibrium) * np.exp(-resistance * time / inductance)
    impulse = quad(current, 0, stop, epsabs=Fixed.motion_verification_tolerance, epsrel=Fixed.motion_verification_tolerance)[0]
    loss = resistance * quad(lambda time: current(time) ** 2, 0, stop,
        epsabs=Fixed.motion_verification_tolerance, epsrel=Fixed.motion_verification_tolerance)[0]
    return dict(input_J=voltage * impulse, resistive_J=loss, mechanical_J=emf * impulse, conduction_s=float(stop))


def verify_circuit_energy(construction):
    rows = []
    for enabled in (True, False):
        for sign in (-1, 1):
            driver = electrical.CoilDrivers(construction, ('reference',))
            initial = sign * driver.current_limit * Fixed.coupling_excitation_fraction
            driver.currents[:] = initial
            driver.voltage[:] = driver.resistance * initial
            emf = -driver.resistance * initial * Fixed.coupling_excitation_fraction
            voltage = float(driver.terminal_voltage(enabled)[0])
            energy = interval_energy(initial, voltage, emf, driver.resistance, driver.inductance, Fixed.adaptive_max_step, enabled)
            driver.advance(np.array([emf]), Fixed.adaptive_max_step, enabled)
            storage = float(driver.inductance * (driver.currents[0] ** 2 - initial ** 2) / 2)
            residual = storage - energy['input_J'] + energy['resistive_J'] + energy['mechanical_J']
            scale = abs(storage) + sum(abs(energy[key]) for key in ('input_J', 'resistive_J', 'mechanical_J'))
            rows.append(dict(enabled=enabled, sign=sign, energy=energy, storage_change_J=storage, balance_residual_J=residual,
                relative_residual=abs(residual) / scale, final_current_A=float(driver.currents[0])))
    return dict(scope='Held back-EMF winding energy including signed terminal work and ideal-clamp extinction', cases=rows,
        passed=all(row['relative_residual'] <= Fixed.motion_verification_tolerance and
            (row['enabled'] or row['final_current_A'] == 0) for row in rows))


def verify_smooth_coupling(construction):
    plant = motion.MotionPlant(construction)
    geometry = plant.geometry
    identities = tuple(plant.array.identities)
    mass = float(plant.model.body_mass[plant.king_id])
    gravity = geometry.gravity
    coupling = geometry.weight / Fixed.driver_channel_current
    matrix = np.zeros((6, len(identities)))
    matrix[2, 0] = coupling
    z = geometry.center_of_mass_height + (geometry.max_flight_gap - geometry.gap) / 2
    velocity = physics_verification.prescribed_vertical_speed(geometry)
    duration = Fixed.airborne_check_duration
    rows = []
    for refinement in Fixed.coupling_refinements:
        driver = electrical.CoilDrivers(construction, identities)
        initial_current = driver.current_limit * Fixed.coupling_excitation_fraction
        driver.currents[0] = initial_current
        voltage = driver.resistance * initial_current + coupling * velocity
        driver.voltage[0] = voltage
        plant.reset(position=np.array([0, 0, z]), velocity=np.array([0, 0, velocity, 0, 0, 0]))
        simulation = SimpleNamespace(plant=plant, drivers=driver, identities=identities, reference=np.zeros(6), peaks={'back_emf_V': 0.})
        generator = np.array([[0, 1, 0, 0], [0, 0, coupling / mass, -gravity],
            [0, -coupling / driver.inductance, -driver.resistance / driver.inductance, voltage / driver.inductance], [0, 0, 0, 0]])
        initial = np.array([z, velocity, initial_current, 1.])
        period = Fixed.adaptive_reference_step / refinement
        count = round(duration / period)
        if abs(count * period - duration) > Fixed.adaptive_clock_tolerance:
            raise ValueError('Coupling reference duration must align with the integration step')
        errors = np.zeros(3)
        electrical_work = 0.
        for index in range(count):
            before = plant.truth()
            energy = interval_energy(driver.currents[0], voltage, coupling * before['linear_velocity_m_per_s'][2],
                driver.resistance, driver.inductance, period)
            electrical_work += energy['input_J'] - energy['resistive_J']
            airborne.AirborneSimulation.advance_physics(simulation, period, matrix)
            if plant.data.ncon:
                raise ValueError('Smooth coupling benchmark reached contact')
            truth = plant.truth()
            actual = np.array([truth['com_position_m'][2], truth['linear_velocity_m_per_s'][2], driver.currents[0]])
            exact = expm(generator * ((index + 1) * period)) @ initial
            errors = np.maximum(errors, np.abs(actual - exact[:3]))
        storage = mass * (actual[1] ** 2 - velocity ** 2) / 2 + mass * gravity * (actual[0] - z) + driver.inductance * (actual[2] ** 2 - initial_current ** 2) / 2
        scales = np.array([gravity * duration ** 2, gravity * duration, initial_current])
        energy_scale = mass * gravity ** 2 * duration ** 2 + driver.inductance * initial_current ** 2 / 2
        rows.append(dict(refinement=refinement, step_s=period, maximum_errors=errors.tolist(), relative_errors=(errors / scales).tolist(),
            input_minus_resistive_J=electrical_work, storage_change_J=float(storage), energy_residual_J=float(storage - electrical_work),
            relative_energy_residual=float(abs(storage - electrical_work) / energy_scale)))
    values = np.array([row['relative_errors'] + [row['relative_energy_residual']] for row in rows])
    orders = np.log(values[:-1] / values[1:]) / np.log(np.array(Fixed.coupling_refinements[1:]) / Fixed.coupling_refinements[:-1])[:, None]
    passed = np.all(values[-1] <= Fixed.coupling_relative_tolerance) and np.all((orders >= Fixed.coupling_order_bounds[0]) & (orders <= Fixed.coupling_order_bounds[1]))
    return dict(scope='Manufactured constant vertical coupling, production MuJoCo/RL advance, exact linear-system exponential; no flight claim',
        equations='dz/dt=v; m*dv/dt=k*i-m*g; L*di/dt=V-R*i-k*v',
        mass_kg=mass, coupling_N_per_A=coupling, resistance_ohm=driver.resistance, inductance_H=driver.inductance,
        voltage_V=voltage, initial_state=initial.tolist(), duration_s=duration, error_components=['position','velocity','current','energy'],
        relative_limit=Fixed.coupling_relative_tolerance, order_bounds=Fixed.coupling_order_bounds,
        cases=rows, observed_orders=orders.tolist(), passed=bool(passed))


class VariableCouplingTrial(adaptive_airborne.AdaptiveAirborneSimulation):
    def __init__(self, construction, plant, switched, fixed_step=None, tolerance_scale=1):
        self.plant = plant
        self.identities = tuple(plant.array.identities)
        self.drivers = electrical.CoilDrivers(construction, self.identities)
        geometry = plant.geometry
        self.span = geometry.max_flight_gap - geometry.gap
        self.height = geometry.center_of_mass_height + self.span / 2
        self.mass = float(plant.model.body_mass[plant.king_id])
        self.inertia = float(plant.model.body_inertia[plant.king_id, 2])
        self.current = self.drivers.current_limit * Fixed.coupling_excitation_fraction
        self.vertical_coupling = geometry.weight / self.current
        self.angular_coupling = self.inertia * (2 * np.pi * Fixed.pose_controller_frequency) ** 2 / self.current
        velocity = physics_verification.prescribed_vertical_speed(geometry) * Fixed.coupling_excitation_fraction
        self.initial = np.array([0., velocity, Fixed.qualification_angle_error, 0., self.current, 0.])
        self.scenario = dict(duration_s=Fixed.coupling_variable_duration)
        self.fields = SimpleNamespace(points=[0])
        self.source = {}
        physics_verification.AdaptivePrescribedTrial.initialize_adaptation(self)
        self.fixed_step, self.tolerance_scale = fixed_step, tolerance_scale
        self.switch_time = self.duration / 2 if switched else self.duration
        if abs(self.switch_time * Fixed.current_regulator_rate - round(self.switch_time * Fixed.current_regulator_rate)) > Fixed.adaptive_clock_tolerance * Fixed.current_regulator_rate:
            raise ValueError('Manufactured switch must coincide with a regulator event')
        self.reference = np.zeros(6)
        self.peaks = dict(back_emf_V=0.)
        self.stop_reason = None
        self.work = 0.
        self.enabled = True
        self.drivers.currents[0] = self.current
        self.drivers.voltage[0] = self.drivers.resistance * self.current + self.vertical_coupling * velocity
        if abs(self.drivers.voltage[0]) > self.drivers.voltage_limit:
            raise ValueError('Manufactured voltage exceeds the driver rating')
        self.plant.reset(position=np.array([0., 0., self.height]), rotation=Rotation.from_rotvec([0., 0., self.initial[2]]),
            velocity=np.array([0., 0., velocity, 0., 0., 0.]))
        self.trace = [dict(time_s=0., state=self.state().tolist())]

    def state(self):
        truth = self.plant.truth()
        return np.array([truth['com_position_m'][2] - self.height, truth['linear_velocity_m_per_s'][2],
            truth['rotation'].as_rotvec()[2], truth['angular_velocity_rad_per_s'][2], self.drivers.currents[0], self.work])

    def force_matrix(self):
        self.adaptation['force_evaluations'] += 1
        return self.coupling_matrix(self.plant)

    def coupling_matrix(self, plant):
        truth = plant.truth()
        matrix = np.zeros((6, len(self.identities)))
        matrix[2, 0] = self.vertical_coupling * (1 + Fixed.coupling_excitation_fraction * (truth['com_position_m'][2] - self.height) / self.span)
        matrix[5, 0] = self.angular_coupling * np.sin(truth['rotation'].as_rotvec()[2])
        return matrix

    def advance_physics(self, period, matrix):
        truth = self.plant.truth()
        emf = float(matrix[:, 0] @ np.r_[truth['linear_velocity_m_per_s'], truth['angular_velocity_rad_per_s']])
        energy = interval_energy(self.drivers.currents[0], self.drivers.terminal_voltage(self.enabled)[0], emf,
            self.drivers.resistance, self.drivers.inductance, period, self.enabled)
        airborne.AirborneSimulation.advance_physics(self, period, matrix, self.enabled)
        self.work += energy['input_J'] - energy['resistive_J']

    def physical_snapshot(self):
        state = super().physical_snapshot()
        state['work'] = self.work
        return state

    def restore_physics(self, state):
        super().restore_physics(state)
        self.work = state['work']

    def endpoint(self):
        return dict(truth=self.plant.truth(), currents=self.drivers.currents.copy(), total=np.zeros(1), coil=np.zeros(1), state=self.state())

    def append_endpoint(self, time, endpoint):
        self.trace.append(dict(time_s=time, state=endpoint['state'].tolist()))

    def observe_physics(self, truth):
        if self.plant.data.ncon or abs(truth['com_position_m'][2] - self.height) >= self.span / 2:
            self.stop_reason = 'manufactured_motion_outside_airborne_domain'
        if np.max(np.abs(self.drivers.currents)) > self.drivers.current_limit:
            self.stop_reason = 'manufactured_current_limit'

    def process_events(self, now, regulator_due, command_due, complete):
        if now >= sensing.hall_available_time(self.next_scan / Fixed.pose_feedback_rate, len(self.fields.points)):
            self.next_scan += 1
        if regulator_due and now >= self.switch_time:
            self.enabled = False
        self.observe_physics(self.plant.truth())
        self.finished = bool(complete or self.stop_reason)


def reference_phase(derivative, interval, initial, method, tolerance, event, event_aligned):
    start, end = interval
    boundaries = [end]
    if event_aligned:
        ticks = np.arange(np.floor(start * Fixed.current_regulator_rate) + 1, np.ceil(end * Fixed.current_regulator_rate))
        boundaries = [*(ticks / Fixed.current_regulator_rate), end]
    segments = []
    for boundary in boundaries:
        segment = solve_ivp(derivative, (start, boundary), initial, method=method,
            rtol=tolerance, atol=tolerance, dense_output=True, events=event)
        if not segment.success:
            raise ValueError('Independent coupling integration failed: ' + segment.message)
        segments.append(segment)
        if segment.status == 1:
            break
        start, initial = boundary, segment.y[:, -1]
    times = np.concatenate([segments[0].t, *[segment.t[1:] for segment in segments[1:]]])
    return SimpleNamespace(t=times, y=np.concatenate([segments[0].y, *[segment.y[:, 1:] for segment in segments[1:]]], axis=1),
        sol=OdeSolution(times, [interpolant for segment in segments for interpolant in segment.sol.interpolants]),
        t_events=segments[-1].t_events, success=True,
        metrics=dict(rhs_evaluations=sum(segment.nfev for segment in segments),
            accepted_steps=len(times) - 1, integration_calls=len(segments),
            minimum_step_s=float(np.min(np.diff(times))), maximum_step_s=float(np.max(np.diff(times)))))


def variable_coupling_solution(trial, method, refinement, event_aligned=False):
    mass, inertia = trial.mass, trial.inertia
    resistance, inductance = trial.drivers.resistance, trial.drivers.inductance
    gravity = trial.plant.geometry.gravity
    length, force, torque = trial.span, trial.vertical_coupling, trial.angular_coupling
    scales = np.array([length, length / trial.duration, Fixed.qualification_angle_error,
        Fixed.qualification_angle_error / trial.duration, trial.current, mass * gravity * length])
    tolerance = Fixed.coupling_reference_tolerance / refinement
    voltage = trial.drivers.voltage[0]
    clamp = -Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction

    def derivative(terminal, conducting):
        def evaluate(time, scaled):
            z, speed, angle, angular_speed, current, work = scaled * scales
            current = current if conducting else 0.
            vertical = force + force * Fixed.coupling_excitation_fraction / length * z
            angular = torque * np.sin(angle)
            result = np.array([speed, (vertical * current / mass) - gravity, angular_speed,
                angular * current / inertia,
                (terminal - resistance * current - vertical * speed - angular * angular_speed) / inductance if conducting else 0.,
                terminal * current - resistance * current ** 2])
            return result / scales
        return evaluate

    def extinction(time, scaled):
        return scaled[4]
    extinction.terminal, extinction.direction = True, -1
    powered = reference_phase(derivative(voltage, True), (0., trial.switch_time), trial.initial / scales,
        method, tolerance, None, event_aligned)
    if not powered.success or powered.t[-1] != trial.switch_time:
        raise ValueError('Independent powered reference failed')
    event_time = None
    metrics = dict(powered=powered.metrics)
    if trial.switch_time < trial.duration:
        if powered.y[4, -1] <= 0:
            raise ValueError('Extinction benchmark requires positive current before disabling')
        decay = reference_phase(derivative(clamp, True), (trial.switch_time, trial.duration), powered.y[:, -1],
            method, tolerance, extinction, event_aligned)
        if not decay.success or len(decay.t_events[0]) != 1:
            raise ValueError('Independent switched reference did not extinguish')
        event_time = float(decay.t[-1])
        released = decay.y[:, -1] * scales
        released[4] = 0.
        metrics['decay'] = decay.metrics

    def sample(times):
        times = np.atleast_1d(times)
        if not np.isfinite(times).all() or np.any(times < 0) or np.any(times > trial.duration):
            raise ValueError('Reference samples must lie inside the resolved interval')
        result = np.empty((len(times), len(scales)))
        before = times <= trial.switch_time
        if np.any(before):
            result[before] = powered.sol(times[before]).T * scales
        if event_time is not None:
            during = (times > trial.switch_time) & (times <= event_time)
            if np.any(during):
                result[during] = decay.sol(times[during]).T * scales
            after = times > event_time
            elapsed = times[after] - event_time
            result[after] = released
            result[after, 0] += released[1] * elapsed - gravity * elapsed ** 2 / 2
            result[after, 1] -= gravity * elapsed
            result[after, 2] += released[3] * elapsed
        return result
    return SimpleNamespace(sample=sample, extinction_s=event_time, metrics=metrics)


def variable_coupling_reference(trial, times, method, refinement):
    solution = variable_coupling_solution(trial, method, refinement)
    return solution.sample(times), solution.extinction_s


def variable_energy_residual(trial, states):
    initial = trial.initial
    return (trial.mass * (states[:, 1] ** 2 - initial[1] ** 2) / 2 + trial.mass * trial.plant.geometry.gravity * states[:, 0]
        + trial.inertia * (states[:, 3] ** 2 - initial[3] ** 2) / 2
        + trial.drivers.inductance * (states[:, 4] ** 2 - initial[4] ** 2) / 2 - states[:, 5])


def verify_variable_coupling(construction):
    plant = motion.MotionPlant(construction)
    rows = []
    for switched in (False, True):
        for step, scale in ((Fixed.adaptive_reference_step, 1),
                (Fixed.adaptive_reference_step / Fixed.qualification_refinement_factor, 1), (None, 1),
                (None, 1 / Fixed.qualification_refinement_factor)):
            trial = VariableCouplingTrial(construction, plant, switched, step, scale)
            while not trial.finished:
                trial.advance()
            times = np.array([row['time_s'] for row in trial.trace])
            actual = np.array([row['state'] for row in trial.trace])
            reference, extinction = variable_coupling_reference(trial, times, 'DOP853', 1)
            independent, independent_extinction = variable_coupling_reference(trial, times, 'Radau', Fixed.qualification_refinement_factor)
            errors = np.max(np.abs(actual - independent), axis=0)
            reference_error = np.max(np.abs(reference - independent), axis=0)
            limits = np.array([Fixed.qualification_position_error * Fixed.qualification_convergence_fraction,
                Fixed.qualification_angle_error * Fixed.qualification_convergence_fraction,
                Fixed.driver_channel_current * Fixed.qualification_current_reserve_fraction * Fixed.qualification_convergence_fraction])
            indices = [0, 2, 4]
            energy_scale = trial.mass * trial.plant.geometry.gravity * trial.span + trial.drivers.inductance * trial.current ** 2 / 2
            energy = float(np.max(np.abs(variable_energy_residual(trial, actual))) / energy_scale)
            reference_energy = float(np.max(np.abs(variable_energy_residual(trial, independent))) / energy_scale)
            energy_ledger_difference = abs(float(variable_energy_residual(trial, actual)[-1]) - trial.adaptation['signed_energy_defect_J'])
            ledger_passed = energy_ledger_difference <= energy_scale * Fixed.coupling_relative_tolerance ** 2
            budget_passed = trial.adaptation['absolute_energy_defect_J'] <= trial.adaptation['energy_budget_J']
            reference_passed = bool(np.all(reference_error[indices] <= limits * Fixed.coupling_relative_tolerance)
                and reference_energy <= Fixed.coupling_relative_tolerance ** 2)
            passed = bool(trial.stop_reason is None and np.all(errors[indices] <= limits)
                and energy <= Fixed.coupling_relative_tolerance and reference_passed and ledger_passed and (step is not None or budget_passed))
            rows.append(dict(switched=switched, fixed_step_s=step, tolerance_scale=scale, passed=passed,
                stop_reason=trial.stop_reason, duration_s=trial.duration, initial_state=trial.initial.tolist(),
                parameters=dict(mass_kg=trial.mass, inertia_kg_m2=trial.inertia, resistance_ohm=trial.drivers.resistance,
                    inductance_H=trial.drivers.inductance, force_coefficient=trial.vertical_coupling, torque_coefficient=trial.angular_coupling,
                    length_m=trial.span, voltage_V=float(trial.drivers.voltage[0]), switch_time_s=trial.switch_time),
                maximum_errors=errors.tolist(), reference_disagreement=reference_error.tolist(), reference_passed=reference_passed,
                position_angle_current_limits=limits.tolist(), relative_energy_residual=energy, relative_reference_energy_residual=reference_energy,
                relative_energy_limit=Fixed.coupling_relative_tolerance, reference_extinction_s=extinction,
                energy_ledger_difference_J=energy_ledger_difference, energy_ledger_passed=bool(ledger_passed),
                accumulated_energy_budget_passed=bool(budget_passed),
                accumulated_energy_budget_enforced=step is None,
                independent_extinction_s=independent_extinction, adaptation=trial.adaptation, trace=trial.trace,
                reference_states=independent.tolist()))
            if not passed:
                return dict(passed=False, cases=rows)
    return dict(passed=True, cases=rows)


def review_joint_solvers(construction):
    plant = motion.MotionPlant(construction)
    rows = []
    times = np.linspace(0., Fixed.coupling_variable_duration,
        round(Fixed.coupling_variable_duration / Fixed.adaptive_reference_step) + 1)
    limits = np.array([Fixed.qualification_position_error * Fixed.qualification_convergence_fraction,
        Fixed.qualification_angle_error * Fixed.qualification_convergence_fraction,
        Fixed.driver_channel_current * Fixed.qualification_current_reserve_fraction * Fixed.qualification_convergence_fraction])
    for switched in (False, True):
        trial = VariableCouplingTrial(construction, plant, switched)
        independent = variable_coupling_solution(trial, 'Radau', Fixed.qualification_refinement_factor)
        reference = independent.sample(times)
        energy_scale = trial.mass * trial.plant.geometry.gravity * trial.span + trial.drivers.inductance * trial.current ** 2 / 2
        reference_energy = float(np.max(np.abs(variable_energy_residual(trial, reference))) / energy_scale)
        for method in ('RK45', 'DOP853'):
            for event_aligned in (False, True):
                started = perf_counter()
                solution = variable_coupling_solution(trial, method, 1, event_aligned)
                states = solution.sample(times)
                elapsed = perf_counter() - started
                errors = np.max(np.abs(states - reference), axis=0)
                energy = float(np.max(np.abs(variable_energy_residual(trial, states))) / energy_scale)
                extinction_difference = None if not switched else abs(solution.extinction_s - independent.extinction_s)
                passed = bool(np.all(errors[[0, 2, 4]] <= limits * Fixed.coupling_relative_tolerance)
                    and energy <= Fixed.coupling_relative_tolerance ** 2
                    and reference_energy <= Fixed.coupling_relative_tolerance ** 2
                    and (not switched or extinction_difference <= Fixed.adaptive_clock_tolerance))
                rows.append(dict(switched=switched, method=method, event_aligned=event_aligned, passed=passed,
                    elapsed_s=elapsed, metrics=solution.metrics, maximum_reference_disagreement=errors.tolist(),
                    relative_energy_residual=energy, relative_reference_energy_residual=reference_energy,
                    extinction_difference_s=extinction_difference))
                if not passed:
                    return dict(passed=False, cases=rows)
    return dict(passed=True, duration_s=Fixed.coupling_variable_duration,
        local_scaled_reference_tolerance=Fixed.coupling_reference_tolerance,
        comparison_position_angle_current_limits=(limits * Fixed.coupling_relative_tolerance).tolist(),
        relative_energy_limit=Fixed.coupling_relative_tolerance ** 2,
        scope='Joint ODE methods on existing manufactured coupling; not a production MuJoCo/controller benchmark',
        cost_scope='Event-aligned cases restart at regulator ticks without changing voltage; passive post-extinction motion is analytical. RHS counts include dense output, but no CAD fields, Hall filter, ADC, controller or checkpoint work.',
        cases=rows)


def verify(output, variable=False, solver_review=False):
    if output.exists():
        raise ValueError('Coupling verification output exists; preserve its evidence')
    output.mkdir(parents=True)
    construction = magnetics.ConstructionModel()
    summary = dict(scope='Independent coupling references; not full physical-envelope qualification', status='running',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, motion.__file__, airborne.__file__, electrical.__file__,
            electrical.qualification.__file__, physics_verification.__file__, adaptive_airborne.__file__, sensing.__file__)),
        physical_envelope_qualified=False, checks={},
        limitations=['The manufactured variable-coupling check verifies integration, not the CAD magnetic map or the controller.',
            'Its passive sensor fields are zero; no Hall-history accuracy is claimed by that check.',
            'Short reference agreement does not certify global accuracy over an arbitrary long unstable trajectory.'])
    started = perf_counter()
    checks = (('joint_solver_review', review_joint_solvers),) if solver_review else (('variable_coupling', verify_variable_coupling),) if variable else (
        ('flux', verify_flux), ('circuit_energy', verify_circuit_energy), ('smooth_coupling', verify_smooth_coupling))
    for name, check in checks:
        summary['active_check'] = name
        airborne_verification.save_summary(output / 'summary.json', summary)
        result = check(construction)
        summary['checks'][name] = result
        if not result['passed']:
            summary.update(status='failed', active_check=None, elapsed_s=perf_counter() - started)
            airborne_verification.save_summary(output / 'summary.json', summary)
            return summary
    summary.update(status='completed', active_check=None, elapsed_s=perf_counter() - started)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check magnetic flux reciprocity and coupled energy independently.')
    parser.add_argument('--output', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--variable', action='store_true')
    mode.add_argument('--solver-review', action='store_true')
    args = parser.parse_args()
    verify(args.output, args.variable, args.solver_review)

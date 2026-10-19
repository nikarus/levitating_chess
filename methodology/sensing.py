import argparse
from dataclasses import dataclass
import json
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.optimize import least_squares
from scipy.special import exprel
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Inputs, Fixed
from methodology import electrical, motion, qualification, source_fingerprints
from methodology.cad import magnetics


@dataclass(frozen=True)
class HallFrame:
    adc_codes: np.ndarray
    sample_times_s: np.ndarray
    available_time_s: float


class HallFieldModel:
    def __init__(self, construction, magnetic_refinement=None):
        self.magnetic_refinement = Fixed.hall_conductor_refinement if magnetic_refinement is None else magnetic_refinement
        if not np.isfinite(self.magnetic_refinement) or self.magnetic_refinement < 1:
            raise ValueError('Hall conductor refinement must be finite and cannot coarsen the base wire path')
        self.geometry, array, _ = construction(Inputs.cad_coil_layout, self.magnetic_refinement)
        levitation_sim.use_geometry(self.geometry)
        self.magnets = levitation_sim.magnet_layout_from_geometry()
        self.base_radius = construction.piece_properties['bottom_film_radius_m']
        drivers = electrical.CoilDrivers(construction, ())
        self.coil_time_constant = drivers.inductance / drivers.resistance
        self.pitch = min(Fixed.hall_pitch_candidates) / 1000
        plane = -self.geometry.gap - array.height - self.geometry.pcb_thickness - self.geometry.hall_standoff
        self.points = levitation_sim.local_hall_points(self.pitch, Fixed.hall_observation_window_side, plane)
        array.window = Fixed.cad_patch_cells_per_side * array.short_pitch
        array.build()
        local_points = array.rotation.inv().apply(self.points)
        if np.max(np.abs(local_points[:, :2])) > (array.window - array.spacing) / 2:
            raise ValueError('Hall points extend beyond the physical CAD patch')
        self.identities = tuple(array.identities)
        self.coil_field = np.array([np.sum([wire.getB(self.points)[:, 2] for wire in coil], axis=0)
                                    for coil in array.coils]).T

    def magnet_field(self, pose, points=None):
        return levitation_sim.hall_field(self.magnets, self.points if points is None else points, pose)

    def current_vector(self, currents):
        if not set(currents) <= set(self.identities):
            raise ValueError('Hall field requested for a coil outside the CAD patch')
        values = np.array([currents.get(identity, 0) for identity in self.identities])
        if not np.isfinite(values).all() or np.max(np.abs(values)) > Fixed.driver_channel_current:
            raise ValueError('Hall field currents are nonfinite or exceed the channel rating')
        return values


def hall_readout():
    samples = Fixed.hall_samples_per_sensor
    times = np.arange(samples) / Fixed.hall_adc_sample_rate
    correlation = np.exp(-2 * np.pi * Fixed.hall_sensor_bandwidth * np.abs(times[:, None] - times))
    quantization = Fixed.hall_supply_voltage / 2 ** Fixed.hall_adc_native_bits / Fixed.hall_sensitivity
    return samples, correlation, quantization


def hall_filter_step(state, initial_field, final_field, period, time_constant=None):
    rate = 2 * np.pi * Fixed.hall_sensor_bandwidth
    decay = np.exp(-rate * period)
    if time_constant is None:
        weight = 1 - (1 - decay) / (rate * period)
    else:
        sensor_exponent, input_exponent = rate * period, period / time_constant
        convolution = sensor_exponent * np.exp(-min(sensor_exponent, input_exponent)) * exprel(-abs(sensor_exponent - input_exponent))
        weight = (-np.expm1(-sensor_exponent) - convolution) / -np.expm1(-input_exponent)
    return decay * state + (1 - decay) * initial_field + (final_field - initial_field) * weight


def hall_available_time(start_time, channel_count):
    samples, _, _ = hall_readout()
    return start_time + Fixed.hall_power_settle_time + channel_count * samples / Fixed.hall_adc_sample_rate + Fixed.qualification_sensor_latency


def filtered_coil_current_step(state, current, voltage, back_emf, resistance, inductance, period, enabled):
    state, current, voltage, back_emf, enabled = (np.asarray(value) for value in (state, current, voltage, back_emf, enabled))
    if (state.ndim != 1 or any(value.shape != state.shape for value in (current, voltage, back_emf, enabled))
            or enabled.dtype.kind != 'b' or not np.isfinite(np.r_[state, current, voltage, back_emf]).all()
            or not np.isfinite([period, resistance, inductance]).all() or period <= 0 or resistance <= 0 or inductance <= 0):
        raise ValueError('Filtered winding currents require matching finite channel vectors and positive circuit values')
    tau = inductance / resistance
    equilibrium = (voltage - back_emf) / resistance
    conducting = np.minimum(period, tau * qualification.rl_stopping_exponent(current, equilibrium, enabled))
    result = state.copy()
    for index, interval in enumerate(conducting):
        if interval > 0:
            final = qualification.rl_step(current[index], voltage[index], back_emf[index],
                resistance, inductance, interval, enabled[index])
            result[index] = hall_filter_step(state[index], current[index], final, interval, tau)
        result[index] *= np.exp(-2 * np.pi * Fixed.hall_sensor_bandwidth * (period - interval))
    return result


class VirtualHallArray:
    def __init__(self, field_model):
        self.field_model = field_model
        self.random = np.random.default_rng(Fixed.hall_virtual_seed)
        count = len(field_model.points)
        self.offset = self.random.uniform(-Fixed.hall_calibrated_offset_bound, Fixed.hall_calibrated_offset_bound, count)
        self.gain = self.random.uniform(1 - Fixed.hall_calibrated_gain_bound, 1 + Fixed.hall_calibrated_gain_bound, count)
        self.actual_coil_field = field_model.coil_field * self.random.uniform(
            1 - Fixed.coil_field_subtraction_error, 1 + Fixed.coil_field_subtraction_error, field_model.coil_field.shape)

    def digitize(self, field, start_time):
        return self.encode_signal(self.gain[:, None] * np.asarray(field)[:, None] + self.offset[:, None], start_time)

    def encode_signal(self, signal, start_time):
        samples, correlation, _ = hall_readout()
        noise = self.random.standard_normal((len(self.gain), samples)) @ np.linalg.cholesky(correlation).T * Fixed.hall_output_noise
        signal = signal + noise
        voltage = Fixed.hall_supply_voltage / 2 + Fixed.hall_sensitivity * np.clip(signal, -Fixed.hall_linear_range, Fixed.hall_linear_range)
        codes = np.floor(voltage / Fixed.hall_supply_voltage * 2 ** Fixed.hall_adc_native_bits)
        codes = np.clip(codes, 0, 2 ** Fixed.hall_adc_native_bits - 1).astype(int)
        times = start_time + Fixed.hall_power_settle_time + np.arange(codes.size).reshape(codes.shape) / Fixed.hall_adc_sample_rate
        available = hall_available_time(start_time, len(self.gain))
        return HallFrame(codes, times, available)

    def sample_static(self, plant, currents):
        truth = plant.truth()
        translation = truth['com_position_m'] - [0, 0, self.field_model.geometry.center_of_mass_height]
        pose = np.r_[translation, truth['rotation'].as_euler('xyz')]
        field = self.field_model.magnet_field(pose) + self.actual_coil_field @ self.field_model.current_vector(currents)
        return self.digitize(field, truth['time_s'])

    def sample_moving(self, field_times, fields, start_time, coil_fields=None, coil_time_constant=None):
        return self.encode_signal(self.moving_signal(field_times, fields, start_time, coil_fields, coil_time_constant), start_time)

    def moving_signal(self, field_times, fields, start_time, coil_fields=None, coil_time_constant=None):
        if (coil_fields is None) != (coil_time_constant is None):
            raise ValueError('Physical coil reconstruction requires both field history and its time constant')
        if coil_fields is None:
            samples = filtered_scan(field_times, fields * self.gain + self.offset, start_time)
        else:
            if coil_fields.shape != fields.shape:
                raise ValueError('Physical coil and total field histories must have matching shapes')
            samples = filtered_scan(field_times, (fields - coil_fields) * self.gain + self.offset, start_time)
            samples += filtered_scan(field_times, coil_fields * self.gain, start_time, coil_time_constant)
        return samples


def filtered_scan(field_times, fields, start_time, time_constant=None):
    samples, _, _ = hall_readout()
    count = fields.shape[1]
    adc_times = start_time + Fixed.hall_power_settle_time + np.arange(count * samples) / Fixed.hall_adc_sample_rate
    if (not np.isfinite(field_times).all() or start_time < field_times[0] or adc_times[-1] > field_times[-1]
            or np.any(np.diff(field_times) <= 0)):
        raise ValueError('Moving scan is outside the supplied chronological field trace')
    if fields.shape != (len(field_times), count) or not np.isfinite(fields).all():
        raise ValueError('Moving field trace has invalid shape or values')
    events = np.unique(np.r_[start_time, adc_times, field_times[(field_times > start_time) & (field_times < adc_times[-1])]])
    if time_constant is None:
        inputs = np.array([np.interp(events, field_times, fields[:, channel]) for channel in range(count)]).T
    else:
        if not np.isfinite(time_constant) or time_constant <= 0:
            raise ValueError('Exponential field interpolation requires a positive finite time constant')
        indices = np.minimum(np.searchsorted(field_times, events, side='right') - 1, len(field_times) - 2)
        intervals = field_times[indices + 1] - field_times[indices]
        weights = -np.expm1(-(events - field_times[indices]) / time_constant) / -np.expm1(-intervals / time_constant)
        inputs = fields[indices] + weights[:, None] * (fields[indices + 1] - fields[indices])
    filtered = np.zeros_like(inputs)
    for index in range(1, len(events)):
        filtered[index] = hall_filter_step(filtered[index - 1], inputs[index - 1], inputs[index], events[index] - events[index - 1], time_constant)
    sampled = filtered[np.searchsorted(events, adc_times), np.repeat(np.arange(count), samples)]
    return sampled.reshape(count, samples)


def measured_coil_fields(field_model, readings, frame):
    if isinstance(readings, electrical.CurrentReadings):
        currents = readings.decode(frame.available_time_s)
        if set(readings.identities) - set(field_model.identities):
            raise ValueError('Current readout contains a coil outside the Hall map')
        if np.any(np.diff(readings.sample_times_s) > 1 / Fixed.current_regulator_rate + Fixed.motion_verification_tolerance):
            raise ValueError('Current samples are missing within the Hall scan history')
        indices = [field_model.identities.index(identity) for identity in readings.identities]
        matrix = field_model.coil_field[:, indices]
        start = frame.sample_times_s[0, 0] - Fixed.hall_power_settle_time
        field = filtered_scan(readings.sample_times_s, currents @ matrix.T, start, field_model.coil_time_constant).mean(axis=1)
        magnitude = np.max(np.abs(currents), axis=0)
        error = (Fixed.current_sense_offset_residual / Fixed.current_sense_resistance
                 + Fixed.current_feedback_gain_bound * magnitude + electrical.current_quantum() / 2)
        uncertainty = np.sqrt((np.abs(matrix) @ error) ** 2 / 3 + Fixed.current_feedback_noise ** 2 * np.sum(matrix ** 2, axis=1))
        return field, np.abs(matrix) @ magnitude, uncertainty
    currents = field_model.current_vector(readings)
    return field_model.coil_field @ currents, np.abs(field_model.coil_field) @ np.abs(currents), np.zeros(len(field_model.points))


class HallPoseEstimator:
    def __init__(self, field_model):
        self.field_model = field_model
        geometry = field_model.geometry
        tilt = np.radians(Fixed.hall_pose_tilt_bound_deg)
        self.lower = np.array([-field_model.pitch / 2, -field_model.pitch / 2, geometry.surface_stack - geometry.gap, -tilt, -tilt, -np.pi])
        self.upper = np.array([field_model.pitch / 2, field_model.pitch / 2, geometry.max_flight_gap - geometry.gap, tilt, tilt, np.pi])
        self.scale = np.array([field_model.pitch] * 3 + [1] * 3)

    def estimate(self, frame, measured_currents, prediction=None):
        samples, correlation, quantization = hall_readout()
        shape = (len(self.field_model.points), samples)
        if frame.adc_codes.shape != shape or frame.sample_times_s.shape != shape:
            raise ValueError('Hall packet shape differs from the configured scan')
        if not np.isfinite(frame.adc_codes).all() or np.any(frame.adc_codes != np.floor(frame.adc_codes)):
            raise ValueError('Hall packet contains invalid ADC codes')
        if np.any(frame.adc_codes < 0) or np.any(frame.adc_codes >= 2 ** Fixed.hall_adc_native_bits):
            raise ValueError('Hall packet ADC codes are out of range')
        if not np.isfinite(frame.sample_times_s).all() or not np.isfinite(frame.available_time_s):
            raise ValueError('Hall packet timestamps are nonfinite')
        if np.any(np.diff(frame.sample_times_s.ravel()) <= 0) or frame.available_time_s < frame.sample_times_s.max():
            raise ValueError('Hall packet timestamps are inconsistent')
        sample_fields = (frame.adc_codes + 0.5) * quantization - Fixed.hall_supply_voltage / 2 / Fixed.hall_sensitivity
        if np.max(np.abs(sample_fields)) >= Fixed.hall_linear_range - quantization:
            return dict(accepted=False, reason='saturated', pose=None)
        coil_field, coil_magnitude, current_uncertainty = measured_coil_fields(self.field_model, measured_currents, frame)
        decoded = sample_fields.mean(axis=1)
        measured = decoded - coil_field
        uncertainty = np.sqrt(Fixed.hall_output_noise ** 2 * correlation.mean() + quantization ** 2 / 12
                              + Fixed.hall_calibrated_offset_bound ** 2 / 3
                              + (Fixed.hall_calibrated_gain_bound * decoded) ** 2 / 3
                              + (Fixed.coil_field_subtraction_error * coil_magnitude) ** 2 / 3 + current_uncertainty ** 2)
        midpoint = float(frame.sample_times_s.mean())
        if prediction is not None:
            predicted_pose, velocity = (np.asarray(value) for value in prediction)
            if predicted_pose.shape != (6,) or velocity.shape != (6,) or not np.isfinite(prediction).all():
                raise ValueError('Pose prediction must contain finite six-axis pose and velocity')
            offsets = frame.sample_times_s.ravel() - midpoint - 1 / (2 * np.pi * Fixed.hall_sensor_bandwidth)
            sample_points = np.repeat(self.field_model.points, samples, axis=0)

        def residual(scaled_pose):
            pose = scaled_pose * self.scale
            if prediction is None:
                field = self.field_model.magnet_field(pose)
            else:
                moving_poses = pose + offsets[:, None] * velocity
                field = self.field_model.magnet_field(moving_poses, sample_points).reshape(shape).mean(axis=1)
            return (field - measured) / uncertainty

        if prediction is None:
            seeds = [np.array([x * self.upper[0], y * self.upper[1], 0, 0, 0, np.radians(yaw)]) / self.scale
                     for x in Fixed.hall_initial_xy_fractions for y in Fixed.hall_initial_xy_fractions for yaw in Fixed.hall_initial_yaws_deg]
            seeds.sort(key=lambda seed: np.linalg.norm(residual(seed)))
        else:
            seed = predicted_pose.copy()
            seed[3:] = np.arctan2(np.sin(seed[3:]), np.cos(seed[3:]))
            seeds = [np.clip(seed, self.lower, self.upper) / self.scale]
        steps = np.array([Fixed.hall_position_derivative_step] * 3 + [Fixed.angle_derivative_step] * 3) / self.scale
        fits = [least_squares(residual, seed, jac=lambda point: levitation_sim.centered_jacobian(residual, point, steps),
                             bounds=(self.lower / self.scale, self.upper / self.scale),
                             max_nfev=Fixed.hall_fit_max_evaluations, ftol=Fixed.hall_fit_tolerance,
                             xtol=Fixed.hall_fit_tolerance, gtol=Fixed.hall_fit_tolerance)
                for seed in seeds[:Fixed.hall_initial_fit_count]]
        fits.sort(key=lambda fit: np.linalg.norm(fit.fun))
        best = fits[0]
        pose = best.x * self.scale
        rms = float(np.sqrt(np.mean(best.fun ** 2)))
        if not np.any(best.jac):
            return dict(accepted=False, reason='unobservable', pose=None, best_candidate=pose.tolist(),
                        normalized_residual_rms=rms, observability=dict(rank6=0, cond6=None))
        rivals = []
        for fit in fits[1:]:
            alternative = fit.x * self.scale
            rotation_error = (Rotation.from_euler('xyz', alternative[3:]) * Rotation.from_euler('xyz', pose[3:]).inv()).magnitude()
            distinct = (np.linalg.norm(alternative[:3] - pose[:3]) > Fixed.position_error_gap_fraction * self.field_model.geometry.gap
                        or rotation_error > Fixed.qualification_angle_error)
            if distinct and np.sqrt(np.mean(fit.fun ** 2)) <= rms + Fixed.hall_fit_ambiguity_margin:
                rivals.append(alternative.tolist())
        levitation_sim.use_geometry(self.field_model.geometry)
        observability = levitation_sim.hall_metrics(best.jac * uncertainty[:, None] / self.scale)
        accepted = bool(best.success and rms <= Fixed.hall_fit_residual_limit and not rivals and observability['rank6'] == 6)
        return dict(accepted=accepted, reason='accepted' if accepted else 'ambiguous_or_poor_fit',
                    pose=pose.tolist() if accepted else None, best_candidate=pose.tolist(), normalized_residual_rms=rms,
                    competing_poses=rivals, converged=bool(best.success), refined_seeds=len(fits),
                    observability=observability,
                    sample_midpoint_s=midpoint, available_time_s=frame.available_time_s,
                    age_at_delivery_s=float(frame.available_time_s - midpoint))


class HallTracker:
    def __init__(self, estimator):
        self.estimator = estimator
        self.position = None
        self.last_delivery = None
        self.measurement_time = None
        self.acceleration_times = []
        self.accelerations = []

    def record_acceleration(self, time, acceleration):
        acceleration = np.asarray(acceleration, dtype=float)
        if (not np.isfinite(time) or acceleration.shape != (6,) or not np.isfinite(acceleration).all()
                or (self.acceleration_times and time <= self.acceleration_times[-1])):
            raise ValueError('Nominal acceleration inputs must be finite and chronological')
        self.acceleration_times.append(float(time))
        self.accelerations.append(acceleration.copy())

    def acceleration_intervals(self, start, end):
        if end < start:
            raise ValueError('Observer propagation must advance time')
        times = np.array(self.acceleration_times)
        boundaries = np.unique(np.r_[start, times[(times > start) & (times < end)], end])
        indices = np.searchsorted(times, boundaries[:-1], side='right') - 1
        values = np.zeros((len(indices), 6))
        available = indices >= 0
        if np.any(available):
            values[available] = np.array(self.accelerations)[indices[available]]
        return values, np.diff(boundaries)

    def propagate(self, time):
        accelerations, periods = self.acceleration_intervals(self.measurement_time, time)
        return self.constrain_contact(*qualification.propagate_estimate(
            self.position, self.velocity, accelerations + self.disturbance, periods))

    def constrain_contact(self, pose, velocity):
        fields = self.estimator.field_model
        geometry = fields.geometry
        clearance = motion.bottom_film_clearance(pose[:3] + [0, 0, geometry.center_of_mass_height],
            Rotation.from_euler('xyz', pose[3:]), geometry.center_of_mass_height, fields.base_radius,
            geometry.surface_stack - geometry.gap - Fixed.piece_bottom_skin / 1000)
        if clearance < 0:
            pose[2] -= clearance
            velocity[2] = max(0, velocity[2])
        return pose, velocity

    def consume(self, frame, measured_currents, now):
        if now < frame.available_time_s or (self.last_delivery is not None and now <= self.last_delivery):
            raise ValueError('Hall packet is premature or delivered out of order')
        midpoint = float(frame.sample_times_s.mean())
        if self.measurement_time is not None and midpoint <= self.measurement_time:
            raise ValueError('Hall sample times must advance')
        if now - frame.available_time_s > Fixed.hall_tracking_max_scan_gap + Fixed.motion_verification_tolerance:
            self.position = None
            self.last_delivery = now
            return dict(valid=False, reason='late_packet')
        if self.last_delivery is not None and now - self.last_delivery > Fixed.hall_tracking_max_scan_gap + Fixed.motion_verification_tolerance:
            self.position = None
        prediction = None
        if self.position is not None:
            prediction = self.propagate(midpoint)
        estimate = self.estimator.estimate(frame, measured_currents, prediction)
        self.last_delivery = now
        if not estimate['accepted']:
            self.position = None
            return dict(valid=False, reason=estimate['reason'], estimate=estimate)
        measured_pose = np.array(estimate['pose'])
        if self.position is None:
            self.position = measured_pose
            self.velocity = np.zeros(6)
            self.disturbance = np.zeros(6)
            self.acquisition_start = now
        else:
            measured_pose[3:] = prediction[0][3:] + np.arctan2(np.sin(measured_pose[3:] - prediction[0][3:]),
                                                              np.cos(measured_pose[3:] - prediction[0][3:]))
            accelerations, periods = self.acceleration_intervals(self.measurement_time, midpoint)
            self.position, self.velocity, self.disturbance = qualification.observer_update(
                self.position, self.velocity, self.disturbance, measured_pose, midpoint - self.measurement_time, accelerations, periods)
        self.position, self.velocity = self.constrain_contact(self.position, self.velocity)
        self.measurement_time = midpoint
        return dict(**self.predict(now), estimate=estimate)

    def predict(self, now):
        if self.position is None:
            return dict(valid=False, reason='no_pose')
        if now < self.last_delivery:
            raise ValueError('Prediction precedes the last delivered measurement')
        if now - self.last_delivery > Fixed.hall_tracking_max_scan_gap + Fixed.motion_verification_tolerance:
            self.position = None
            return dict(valid=False, reason='stale')
        if now - self.acquisition_start < Fixed.hall_tracking_acquisition_duration:
            return dict(valid=False, reason='acquiring')
        pose, velocity = self.propagate(now)
        return dict(valid=True, reason='tracking', pose=pose.tolist(), velocity=velocity.tolist())


def moving_trajectory(plant, field_model, supported):
    plant.reset(velocity=Fixed.hall_tracking_velocity)
    samples, _, _ = hall_readout()
    duration = (Fixed.hall_tracking_duration + Fixed.hall_power_settle_time
                + len(field_model.points) * samples / Fixed.hall_adc_sample_rate + Fixed.qualification_sensor_latency)
    wrench = np.zeros(6)
    if supported:
        wrench[2] = plant.geometry.weight
    times, poses = [], []
    for _ in range(int(np.ceil(duration / plant.model.opt.timestep)) + 1):
        truth = plant.truth()
        times.append(truth['time_s'])
        poses.append(np.r_[truth['com_position_m'] - [0, 0, plant.geometry.center_of_mass_height], truth['rotation'].as_euler('xyz')])
        plant.advance_wrench(wrench)
    poses = np.asarray(poses)
    fields = []
    for start in range(0, len(poses), Fixed.hall_field_batch_frames):
        batch = poses[start:start + Fixed.hall_field_batch_frames]
        fields.append(field_model.magnet_field(np.repeat(batch, len(field_model.points), axis=0),
                      np.tile(field_model.points, (len(batch), 1))).reshape(len(batch), len(field_model.points)))
    return np.array(times), poses, np.vstack(fields)


def moving_checks(construction, fields):
    summaries, traces = [], {}
    for name, supported in (('supported_translation_yaw_with_dropout', True), ('unpowered_drop_and_contact', False)):
        for refinement in (1, Fixed.qualification_refinement_factor):
            plant = motion.MotionPlant(construction, refinement)
            times, poses, magnetic_fields = moving_trajectory(plant, fields, supported)
            sensors = VirtualHallArray(fields)
            tracker = HallTracker(HallPoseEstimator(fields))
            records = []
            for index in range(round(Fixed.hall_tracking_duration * Fixed.pose_feedback_rate)):
                start = index / Fixed.pose_feedback_rate
                packet = sensors.sample_moving(times, magnetic_fields, start)
                now = packet.available_time_s
                omitted = supported and Fixed.hall_tracking_dropout_interval[0] <= start < Fixed.hall_tracking_dropout_interval[1]
                result = tracker.predict(now) if omitted else tracker.consume(packet, {}, now)
                record = dict(time_s=now, scan_start_s=start, omitted=omitted, valid=result['valid'], reason=result['reason'])
                if result['valid']:
                    true_pose = np.array([np.interp(now, times, poses[:, axis]) for axis in range(6)])
                    position_error = float(np.linalg.norm(np.asarray(result['pose'])[:3] - true_pose[:3]))
                    angle_error = float((Rotation.from_euler('xyz', result['pose'][3:]) * Rotation.from_euler('xyz', true_pose[3:]).inv()).magnitude())
                    record.update(pose=result['pose'], position_error_m=position_error, angle_error_rad=angle_error)
                records.append(record)
                print(name, refinement, index, record['reason'], record.get('position_error_m'), flush=True)
            valid = [row for row in records if row['valid']]
            if not valid:
                raise RuntimeError('Moving Hall diagnostic never acquired a usable pose')
            stale = any(row['reason'] == 'stale' for row in records)
            reacquired = any(row['valid'] and row['scan_start_s'] > Fixed.hall_tracking_dropout_interval[1] for row in records)
            summary = dict(case=name, refinement=refinement, physics_time_step_s=plant.model.opt.timestep,
                           frames=len(records), usable_frames=len(valid), first_usable_time_s=valid[0]['time_s'],
                           status_counts={reason: sum(row['reason'] == reason for row in records) for reason in sorted({row['reason'] for row in records})},
                           worst_position=max(valid, key=lambda row: row['position_error_m']),
                           worst_angle=max(valid, key=lambda row: row['angle_error_rad']),
                           dropout_invalidated=stale, usable_after_dropout=reacquired)
            summary['passed'] = (summary['worst_position']['position_error_m'] <= Fixed.qualification_position_error
                                 and summary['worst_angle']['angle_error_rad'] <= Fixed.qualification_angle_error
                                 and (not supported or (stale and reacquired)))
            summaries.append(summary)
            traces[name, refinement] = records
    comparisons = []
    for name in {row['case'] for row in summaries}:
        coarse, fine = traces[name, 1], traces[name, Fixed.qualification_refinement_factor]
        pairs = [(left, right) for left, right in zip(coarse, fine) if left['valid'] and right['valid']]
        position_change = max(float(np.linalg.norm(np.asarray(left['pose'])[:3] - np.asarray(right['pose'])[:3])) for left, right in pairs)
        angle_change = max(float((Rotation.from_euler('xyz', left['pose'][3:]) * Rotation.from_euler('xyz', right['pose'][3:]).inv()).magnitude())
                           for left, right in pairs)
        statuses_match = all(left['reason'] == right['reason'] for left, right in zip(coarse, fine))
        comparisons.append(dict(case=name, maximum_position_change_m=position_change, maximum_angle_change_rad=angle_change,
                                statuses_match=statuses_match, passed=statuses_match
                                and position_change <= Fixed.qualification_convergence_fraction * Fixed.qualification_position_error
                                and angle_change <= Fixed.qualification_convergence_fraction * Fixed.qualification_angle_error))
    return dict(scope='Sequential Hall scans of MuJoCo motion, sensor-only observer and prediction to packet delivery; no feedback control',
                cases=summaries, refinement=comparisons, passed=all(row['passed'] for row in summaries + comparisons),
                assumptions=['Coils are off. Supported motion uses a prescribed gravity-balancing mechanical wrench, not a levitation controller.',
                             'Each local scan starts with a powered-up single-pole filter at quiescent output; startup transients are provisional.',
                             'Physical fields are interpolated between MuJoCo steps; both motion and field knots are independently halved relative to ADC timing.',
                             'The inverse model uses observer velocity during the scan and the nominal low-frequency filter delay; acceleration during a scan is unmodeled.',
                             'Observer predictions enforce the known CAD bottom-film/board nonpenetration constraint with zero downward speed on projected contact; bounce and unknown surfaces are not modeled.',
                             'No changing-current subtraction, current sensing, pose-window handoff, neighbors, material uncertainty or closed-loop stability qualification.',
                             'Offline compute time is not included in modeled transport latency; embedded estimator execution time remains unqualified.'])


def run(output, moving=False):
    construction = magnetics.ConstructionModel()
    plant = motion.MotionPlant(construction)
    fields = HallFieldModel(construction)
    sensors = VirtualHallArray(fields)
    estimator = HallPoseEstimator(fields)
    tilt = levitation_sim.rim_tilt_limit(fields.magnets, fields.geometry.max_flight_gap) / 2
    phase = fields.pitch * min(value for value in Fixed.pose_phase_fractions if value > 0)
    poses = [('rest', [0, 0, fields.geometry.surface_stack - fields.geometry.gap, 0, 0, 0]),
             ('hover', [0, 0, 0, 0, 0, 0]),
             ('offset_yaw', [phase, -phase, 0, 0, 0, np.radians(max(Fixed.pose_yaws_deg))]),
             ('raised_tilt', [-phase, phase, fields.geometry.max_flight_gap - fields.geometry.gap, tilt, -tilt,
                              np.radians(Fixed.motion_check_orientation_deg[2])])]
    driven = {identity: Fixed.driver_channel_current * (1 - Fixed.qualification_current_reserve_fraction) * (-1) ** index
              for index, identity in enumerate(fields.identities)}
    records = []
    for label, pose in poses:
        for mode, currents in (('off', {}), ('signed_channel_limit', driven)):
            plant.reset(position=np.array(pose[:3]) + [0, 0, fields.geometry.center_of_mass_height],
                        rotation=Rotation.from_euler('xyz', pose[3:]))
            packet = sensors.sample_static(plant, currents)
            started = perf_counter()
            estimate = estimator.estimate(packet, currents)
            position_error, angle_error = None, None
            if estimate['reason'] != 'saturated':
                candidate = np.asarray(estimate['best_candidate'])
                position_error = float(np.linalg.norm(candidate[:3] - pose[:3]))
                angle_error = float((Rotation.from_euler('xyz', candidate[3:]) * Rotation.from_euler('xyz', pose[3:]).inv()).magnitude())
            records.append(dict(case=label, currents=mode, true_pose=pose, estimate=estimate, elapsed_s=perf_counter() - started,
                                position_error_m=position_error, angle_error_rad=angle_error,
                                passed=estimate['accepted'] and position_error <= Fixed.position_error_gap_fraction * fields.geometry.gap
                                and angle_error <= Fixed.qualification_angle_error))
            print(label, mode, records[-1]['passed'], position_error, angle_error, flush=True)
    report = dict(scope='Static CAD/MuJoCo pose-to-Hall-ADC-to-pose initialization screen; no true-pose estimator seed',
                  source_sha256=dict(plant.provenance, **source_fingerprints(__file__, qualification.__file__, electrical.__file__)),
                  sensor_pitch_m=fields.pitch, sensor_points_m=fields.points.tolist(), physical_coils=len(fields.identities),
                  scan_duration_s=Fixed.hall_power_settle_time + len(fields.points) * hall_readout()[0] / Fixed.hall_adc_sample_rate,
                  records=records, passed=all(row['passed'] for row in records),
                  limitations=['Local patch and densest existing pitch candidate only; not a selected full-board sensor layout.',
                               'Static cases hold pose throughout a scan; the optional moving section checks scan skew and analog filtering.',
                               'Current subtraction receives prescribed true currents until virtual current sensing is connected.',
                               'Residual gain/offset/coil-map errors and noise correlation are provisional calibration assumptions.',
                               'Finite multistart search checks sampled local alternatives, not global uniqueness.',
                               'No neighbors, magnet variation, sensor placement tolerance, electrical dynamics or feedback control.'])
    if moving:
        report['moving'] = moving_checks(construction, fields)
        report['static_passed'] = report['passed']
        report['passed'] = report['static_passed'] and report['moving']['passed']
        report['scope'] = 'CAD/MuJoCo static and moving Hall readout/pose tracking screen; no feedback control'
    output.write_text(json.dumps(report, indent=Fixed.report_precision, allow_nan=False) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check CAD-derived virtual Hall readout and local pose initialization.')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('results') / 'sensing.json')
    parser.add_argument('--moving', action='store_true', help='Also check moving scans, observer timing, dropout recovery and time refinement.')
    arguments = parser.parse_args()
    run(arguments.output, arguments.moving)

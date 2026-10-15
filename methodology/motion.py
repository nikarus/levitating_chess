import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from time import perf_counter

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Inputs, Fixed, Constants
from methodology import source_fingerprints
from methodology.cad import magnetics, motion_scene


def airborne_observation(plant, truth, drivers, base_radius):
    minimum_height = plant.geometry.center_of_mass_height
    maximum_height = minimum_height + (plant.geometry.max_flight_gap - plant.geometry.gap)
    clearance = bottom_film_clearance(truth['com_position_m'], truth['rotation'], minimum_height,
        base_radius, plant.metadata['board_top_m'])
    if not np.isfinite(np.r_[plant.integration_state(), drivers.currents, clearance]).all():
        return 'nonfinite_physical_state', clearance
    if plant.data.ncon or clearance <= 0:
        return 'contact', clearance
    if not minimum_height <= truth['com_position_m'][2] <= maximum_height:
        return 'outside_sampled_gap_range', clearance
    if np.max(np.abs(drivers.currents)) > drivers.current_limit:
        return 'current_limit_exceeded', clearance
    return None, clearance


class MotionPlant:
    def __init__(self, construction, refinement=1, magnetic_refinement=None, full_coil_patch=False):
        self.scene, self.metadata = motion_scene.scene_xml(construction, refinement)
        self.model = mujoco.MjModel.from_xml_string(self.scene)
        self.data = mujoco.MjData(self.model)
        self.king_id = self.model.body('king').id
        self.geometry, self.array, self.configuration = construction(Inputs.cad_coil_layout,
            min(Fixed.current_demand_mesh_refinements) if magnetic_refinement is None else magnetic_refinement)
        self.magnets = levitation_sim.magnet_layout_from_geometry()
        self.field_evaluator = None
        self.physical_identities = set(self.metadata['physical_identities'])
        self.full_coil_patch = full_coil_patch
        if full_coil_patch:
            self.array.window = Fixed.cad_patch_cells_per_side * self.array.short_pitch
            self.array.build(center=(0, 0))
        self.provenance = dict(magnetics.fingerprints(), **source_fingerprints(__file__, motion_scene.__file__,
                               Path(__file__).with_name('requirements.txt'), Path(motion_scene.__file__).with_name('environment.lock.txt')))
        self.reset()

    def reset(self, position=None, rotation=None, velocity=None):
        mujoco.mj_resetData(self.model, self.data)
        if position is not None:
            self.data.qpos[:3] = position
        if rotation is not None:
            self.data.qpos[3:] = rotation.as_quat(scalar_first=True)
        if velocity is not None:
            self.data.qvel[:] = velocity
        mujoco.mj_forward(self.model, self.data)

    def truth(self):
        rotation = Rotation.from_quat(self.data.qpos[3:], scalar_first=True)
        return dict(time_s=float(self.data.time), com_position_m=self.data.qpos[:3].copy(), rotation=rotation,
                    linear_velocity_m_per_s=self.data.qvel[:3].copy(), angular_velocity_rad_per_s=rotation.apply(self.data.qvel[3:]))

    def magnetic_matrix(self, window_center=None):
        levitation_sim.use_geometry(self.geometry)
        truth = self.truth()
        center = truth['com_position_m']
        selected_center = center[:2] if window_center is None else window_center
        self.array.build(center=(0, 0) if self.full_coil_patch else selected_center)
        if not set(self.array.identities) <= self.physical_identities:
            raise ValueError('Requested magnetic window extends beyond the physical CAD patch')
        translation = center - [0, 0, self.geometry.center_of_mass_height]
        _, magnet_center = levitation_sim.place_piece(self.magnets, *translation, *truth['rotation'].as_euler('xyz'))
        if not np.allclose(center, magnet_center, atol=Fixed.cad_geometry_tolerance / 1000, rtol=0):
            raise ValueError('Magnetic and mechanical centre-of-mass coordinates disagree')
        return levitation_sim.actuator_matrix(self.magnets, self.array, center, self.field_evaluator)

    def apply_wrench(self, wrench):
        self.data.qfrc_applied[:] = 0
        self.data.xfrc_applied[:] = 0
        wrench = np.asarray(wrench, dtype=float)
        if wrench.shape != (6,) or not np.isfinite(wrench).all():
            raise ValueError('Expected a finite world-frame force and COM torque')
        mujoco.mj_applyFT(self.model, self.data, wrench[:3], wrench[3:], self.data.xipos[self.king_id],
                         self.king_id, self.data.qfrc_applied)

    def mechanical_energy_change(self, before, after):
        mass = self.model.body_mass[self.king_id]
        principal_axes = Rotation.from_quat(self.model.body_iquat[self.king_id], scalar_first=True)
        initial_spin = principal_axes.inv().apply(before['rotation'].inv().apply(before['angular_velocity_rad_per_s']))
        final_spin = principal_axes.inv().apply(after['rotation'].inv().apply(after['angular_velocity_rad_per_s']))
        initial_velocity, final_velocity = before['linear_velocity_m_per_s'], after['linear_velocity_m_per_s']
        return float(mass * np.dot(final_velocity - initial_velocity, final_velocity + initial_velocity) / 2
            + np.dot(self.model.body_inertia[self.king_id], (final_spin - initial_spin) * (final_spin + initial_spin)) / 2
            - mass * np.dot(self.model.opt.gravity, after['com_position_m'] - before['com_position_m']))

    def advance_wrench(self, wrench):
        mujoco.mj_step1(self.model, self.data)
        self.apply_wrench(wrench)
        mujoco.mj_step2(self.model, self.data)
        contact_impulse = self.data.qfrc_constraint[:3].copy() * self.model.opt.timestep
        if np.any(self.data.warning.number):
            raise RuntimeError(f'MuJoCo reported a numerical warning: {self.data.warning.number}')
        mujoco.mj_forward(self.model, self.data)
        return contact_impulse

    def advance_currents(self, currents):
        matrix = self.magnetic_matrix()
        if not set(currents) <= set(self.array.identities):
            raise ValueError('Prescribed currents include a coil outside the evaluated window')
        values = np.array([currents.get(identity, 0) for identity in self.array.identities])
        if not np.isfinite(values).all() or np.max(np.abs(values)) > Fixed.driver_channel_current:
            raise ValueError('Prescribed coil current exceeds the configured channel rating')
        wrench = matrix @ values
        self.advance_wrench(wrench)
        return wrench

    def integration_state(self):
        specification = mujoco.mjtState.mjSTATE_INTEGRATION
        state = np.empty(mujoco.mj_stateSize(self.model, specification))
        mujoco.mj_getState(self.model, self.data, state, specification)
        return state

    def restore_integration_state(self, state):
        mujoco.mj_setState(self.model, self.data, np.asarray(state), mujoco.mjtState.mjSTATE_INTEGRATION)
        mujoco.mj_forward(self.model, self.data)

    def checkpoint_state(self):
        return dict(scope='Mechanical plant state only; excludes controller, sensor and electrical state',
                    mujoco_version=mujoco.__version__, scene_sha256=hashlib.sha256(self.scene.encode()).hexdigest(),
                    time_step_s=self.model.opt.timestep, source_sha256=self.provenance, integration_state=self.integration_state().tolist())

    def restore_checkpoint(self, checkpoint):
        if checkpoint['mujoco_version'] != mujoco.__version__ or checkpoint['source_sha256'] != self.provenance:
            raise ValueError('Checkpoint dependency or source version differs from this plant')
        if checkpoint['scene_sha256'] != hashlib.sha256(self.scene.encode()).hexdigest() or checkpoint['time_step_s'] != self.model.opt.timestep:
            raise ValueError('Checkpoint scene or numerical settings differ from this plant')
        self.restore_integration_state(checkpoint['integration_state'])

    def save_checkpoint(self, path):
        path.write_text(json.dumps(self.checkpoint_state(), indent=Fixed.report_precision) + '\n')

    def load_checkpoint(self, path):
        self.restore_checkpoint(json.loads(path.read_text()))


def mechanical_checks(plant, construction):
    records = []
    trajectories = {}
    for refinement in (1, Fixed.qualification_refinement_factor):
        candidate = plant if refinement == 1 else MotionPlant(construction, refinement)
        initial_position = np.asarray(candidate.metadata['king_com_from_magnet_bottom_m'])
        initial_position[2] += candidate.geometry.max_flight_gap - candidate.geometry.gap
        period = candidate.model.opt.timestep
        steps = round(Fixed.motion_check_duration / period)
        duration = steps * period
        for name, wrench in (('free_fall', np.zeros(6)),
                             ('lateral_force', candidate.geometry.weight * np.array([1, 0, 1, 0, 0, 0]))):
            candidate.reset(position=initial_position)
            acceleration = wrench[:3] / candidate.model.body_mass[candidate.king_id] - [0, 0, Constants.gravity]
            for _ in range(steps):
                candidate.advance_wrench(wrench)
            exact_position = initial_position + acceleration * duration ** 2 / 2
            exact_velocity = acceleration * duration
            position_error = float(np.linalg.norm(candidate.data.qpos[:3] - exact_position))
            velocity_error = float(np.linalg.norm(candidate.data.qvel[:3] - exact_velocity))
            allowed_position_error = np.linalg.norm(acceleration) * duration * period / 2 + Fixed.motion_verification_tolerance
            record = dict(name=name, refinement=refinement, position_error_m=position_error,
                          velocity_error_m_per_s=velocity_error, passed=position_error <= allowed_position_error
                          and velocity_error <= Fixed.motion_verification_tolerance and candidate.data.ncon == 0)
            records.append(record)
            trajectories[name, refinement] = position_error
        angular_acceleration = 4 * np.radians(Inputs.target_yaw_angle_deg) / Inputs.target_yaw_time ** 2
        for axis in range(3):
            candidate.reset(position=initial_position)
            torque = np.eye(3)[axis] * candidate.geometry.inertias[axis] * angular_acceleration
            for _ in range(steps):
                candidate.advance_wrench([0, 0, candidate.geometry.weight, *torque])
            rotation = candidate.truth()['rotation'].as_rotvec()
            exact_rotation = np.eye(3)[axis] * angular_acceleration * duration ** 2 / 2
            angle_error = float(np.linalg.norm(rotation - exact_rotation))
            velocity_error = float(np.linalg.norm(candidate.data.qvel[3:] - np.eye(3)[axis] * angular_acceleration * duration))
            records.append(dict(name=f'principal_torque_{axis}', refinement=refinement, angle_error_rad=angle_error,
                                angular_velocity_error_rad_per_s=velocity_error,
                                passed=angle_error <= angular_acceleration * duration * period / 2 + Fixed.motion_verification_tolerance
                                and velocity_error <= Fixed.motion_verification_tolerance))
            trajectories[f'principal_torque_{axis}', refinement] = angle_error
    for name, refinement in trajectories:
        if refinement == 1:
            records.append(dict(name=f'{name}_time_refinement', passed=trajectories[name, Fixed.qualification_refinement_factor]
                                <= trajectories[name, 1] / Fixed.qualification_refinement_factor + Fixed.motion_verification_tolerance))
    rotation = Rotation.from_euler('xyz', Fixed.motion_check_orientation_deg, degrees=True)
    plant.reset(position=initial_position, rotation=rotation)
    force = plant.geometry.weight * np.array([1, -1, 1])
    lever = rotation.apply([0, plant.metadata['base_radius_m'], 0])
    torque = np.cross(lever, force)
    plant.apply_wrench([*force, *torque])
    generalized = plant.data.qfrc_applied.copy()
    plant.data.qfrc_applied[:] = 0
    mujoco.mj_applyFT(plant.model, plant.data, force, np.zeros(3), plant.data.xipos[plant.king_id] + lever,
                     plant.king_id, plant.data.qfrc_applied)
    offset_error = float(np.max(np.abs(plant.data.qfrc_applied - generalized)))
    mujoco.mj_forward(plant.model, plant.data)
    tensor = rotation.as_matrix() @ np.asarray(plant.metadata['king_inertia_kg_m2']) @ rotation.as_matrix().T
    angular_error = float(np.linalg.norm(rotation.apply(plant.data.qacc[3:]) - np.linalg.solve(tensor, torque)))
    linear_error = float(np.linalg.norm(plant.data.qacc[:3] - (force / plant.model.body_mass[plant.king_id] - [0, 0, Constants.gravity])))
    records.append(dict(name='rotated_body_and_off_centre_force', generalized_force_error=offset_error,
                        linear_acceleration_error_m_per_s2=linear_error, angular_acceleration_error_rad_per_s2=angular_error,
                        passed=max(offset_error, angular_error, linear_error) <= Fixed.motion_verification_tolerance and plant.data.ncon == 0))
    contact_peaks = []
    for contact_plant in (plant, candidate):
        contact_plant.reset()
        penetration = 0
        for _ in range(round(Fixed.qualification_recovery_duration / contact_plant.model.opt.timestep)):
            contact_plant.advance_wrench(np.zeros(6))
            if contact_plant.data.ncon:
                penetration = max(penetration, -float(min(contact_plant.data.contact.dist)))
        support_error = abs(float(contact_plant.data.qfrc_constraint[2]) / contact_plant.geometry.weight - 1)
        speed = float(np.linalg.norm(contact_plant.data.qvel[:3]))
        records.append(dict(name='drop_and_rest', time_step_s=contact_plant.model.opt.timestep,
                            maximum_penetration_m=penetration, final_speed_m_per_s=speed, support_relative_error=support_error,
                            passed=contact_plant.data.ncon > 0 and penetration <= Fixed.motion_contact_penetration_limit
                            and support_error <= Fixed.qualification_convergence_fraction
                            and speed <= Fixed.motion_contact_penetration_limit / Fixed.motion_contact_time_constant))
        contact_peaks.append(penetration)
    contact_change = abs(contact_peaks[-1] / contact_peaks[0] - 1)
    records.append(dict(name='contact_time_refinement', relative_penetration_change=contact_change,
                        passed=contact_change <= Fixed.qualification_convergence_fraction))
    initial_speed = 2 * Fixed.resting_friction_coefficient * Constants.gravity * Fixed.motion_check_duration
    plant.data.qvel[0] = initial_speed
    mujoco.mj_forward(plant.model, plant.data)
    initial_velocity = plant.data.qvel[:3].copy()
    contact_impulse = np.zeros(3)
    steps = round(Fixed.motion_check_duration / plant.model.opt.timestep)
    for _ in range(steps):
        contact_impulse += plant.advance_wrench(np.zeros(6))
    predicted_velocity = initial_velocity + contact_impulse / plant.model.body_mass[plant.king_id]
    predicted_velocity[2] -= Constants.gravity * steps * plant.model.opt.timestep
    impulse_error = float(np.linalg.norm(plant.data.qvel[:3] - predicted_velocity))
    friction_bound = Fixed.resting_friction_coefficient * contact_impulse[2]
    records.append(dict(name='sliding_contact_impulse', velocity_balance_error_m_per_s=impulse_error,
                        final_speed_m_per_s=float(plant.data.qvel[0]), contact_impulse_Ns=contact_impulse.tolist(),
                        scope='Free king can rock and change normal force; check momentum balance and Coulomb bound, not constant-normal-force deceleration.',
                        passed=bool(impulse_error <= Fixed.motion_verification_tolerance and contact_impulse[2] > 0
                        and contact_impulse[0] < 0 and 0 < plant.data.qvel[0] < initial_speed
                        and np.linalg.norm(contact_impulse[:2]) <= friction_bound + Fixed.motion_verification_tolerance)))
    return dict(checks=records, passed=all(row['passed'] for row in records))


def magnetic_check(plant):
    position = np.array(plant.metadata['king_com_from_magnet_bottom_m'])
    position[2] += plant.geometry.max_flight_gap - plant.geometry.gap
    plant.reset(position=position, rotation=Rotation.from_euler('z', Fixed.prototype_path_yaws_deg[1], degrees=True))
    matrix = plant.magnetic_matrix()
    target = np.array([0, 0, plant.geometry.weight, 0, 0, 0])
    peak, initial = levitation_sim.min_peak_current(matrix, target)
    limit = Fixed.driver_channel_current * (1 - Fixed.qualification_current_reserve_fraction)
    if peak > limit:
        raise ValueError('Interface hover-force check exceeds the reserved current budget')
    allocated = levitation_sim.min_loss_current(matrix, target, limit, initial_currents=initial)
    currents = dict(zip(plant.array.identities, allocated))
    started = perf_counter()
    trace = []
    acceleration_residual = 0
    for _ in range(Fixed.motion_magnetic_check_steps):
        wrench = plant.advance_currents(currents)
        acceleration_residual = max(acceleration_residual, float(np.max(np.abs(plant.data.qacc))))
        truth = plant.truth()
        trace.append(dict(time_s=truth['time_s'], com_position_m=truth['com_position_m'].tolist(),
                          rotation_vector_rad=truth['rotation'].as_rotvec().tolist(), magnetic_wrench=wrench.tolist()))
    displacement = float(np.linalg.norm(plant.data.qpos[:3] - position))
    return dict(scope='Constant prescribed currents from one nominal gravity-balancing allocation; interface check, not feedback levitation',
                peak_minimum_rating_A=peak, peak_applied_current_A=float(max(abs(allocated))),
                duration_s=plant.data.time, elapsed_s=perf_counter() - started,
                com_displacement_m=displacement, maximum_acceleration_residual_SI=acceleration_residual, trace=trace,
                passed=displacement <= Fixed.cad_geometry_tolerance / 1000
                and acceleration_residual <= Fixed.motion_verification_tolerance and plant.data.ncon == 0)


def bottom_film_clearance(com_position, rotation, com_height, base_radius, board_top):
    axis_z = rotation.as_matrix()[2, 2]
    half_thickness = Fixed.piece_bottom_skin / 2000
    bottom = (com_position[2] - (com_height + half_thickness) * axis_z - half_thickness * abs(axis_z)
              - base_radius * np.sqrt(max(0, 1 - axis_z ** 2)))
    return bottom - board_top


def preview_camera(plant):
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(plant.model, camera)
    camera.lookat[:] = plant.data.qpos[:3]
    camera.distance /= Fixed.cad_preview_margin
    return camera


def render(plant, path):
    from PIL import Image

    width, height = Fixed.cad_preview_size
    camera = preview_camera(plant)
    with mujoco.Renderer(plant.model, height=height, width=width) as renderer:
        renderer.update_scene(plant.data, camera=camera)
        Image.fromarray(renderer.render()).save(path)


class MotionVideo:
    def __init__(self, plant, path):
        from PIL import ImageFont

        self.plant = plant
        self.width, self.height = Fixed.cad_preview_size
        self.camera = preview_camera(plant)
        self.camera.distance *= Fixed.cad_preview_margin
        self.font = ImageFont.load_default(size=Fixed.motion_video_font_size)
        self.line_height = Fixed.motion_video_font_size * 2
        self.panel_width = self.width // 2
        self.panel_height = self.height - self.line_height * 4
        self.side_height = self.panel_height // len(Fixed.motion_video_side_azimuths)
        self.side_cameras = []
        for azimuth in Fixed.motion_video_side_azimuths:
            camera = mujoco.MjvCamera()
            mujoco.mjv_defaultFreeCamera(plant.model, camera)
            camera.lookat[:] = [0, 0, (plant.metadata['board_top_m'] - Fixed.piece_bottom_skin / 1000) / 2]
            camera.azimuth, camera.elevation = azimuth, 0
            camera.distance = plant.metadata['base_radius_m'] * Fixed.motion_video_gap_distance_radii
            self.side_cameras.append(camera)
        self.surface_id = plant.model.geom('Playing_surface').id
        self.skin_id = plant.model.geom('Piece_bottom_skin').id
        self.surface_alpha = plant.model.geom_rgba[self.surface_id, 3]
        self.skin_alpha = plant.model.geom_rgba[self.skin_id, 3]
        plant.model.geom_rgba[self.skin_id, 3] = 1
        self.initial_position = plant.data.qpos[:3].copy()
        self.command = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pixel_format', 'rgb24',
                       '-video_size', f'{self.width}x{self.height}', '-framerate', str(Fixed.motion_video_frame_rate), '-i', 'pipe:0',
                       '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)]
        self.encoder = subprocess.Popen(self.command, stdin=subprocess.PIPE)
        self.overview = mujoco.Renderer(plant.model, height=self.panel_height, width=self.panel_width)
        self.detail = mujoco.Renderer(plant.model, height=self.side_height, width=self.panel_width)
        self.frames = 0

    def __enter__(self):
        return self

    def frame(self, labels):
        from PIL import Image, ImageDraw

        plant = self.plant
        frame = Image.new('RGB', (self.width, self.height), 'white')
        plant.model.geom_rgba[self.surface_id, 3] = self.surface_alpha
        self.overview.update_scene(plant.data, camera=self.camera)
        frame.paste(Image.fromarray(self.overview.render()), (0, self.line_height * 3))
        plant.model.geom_rgba[self.surface_id, 3] = Fixed.motion_video_surface_opacity
        for index, camera in enumerate(self.side_cameras):
            self.detail.update_scene(plant.data, camera=camera)
            frame.paste(Image.fromarray(self.detail.render()), (self.panel_width, self.line_height * 3 + index * self.side_height))
        drawing = ImageDraw.Draw(frame)
        for line, label in enumerate(labels):
            drawing.text((Fixed.motion_video_font_size, line * self.line_height), label, font=self.font, fill='black')
        displacement = (plant.data.qpos[:3] - self.initial_position) * 1000
        rotation = plant.truth()['rotation']
        tilt = np.degrees(np.arccos(np.clip(rotation.as_matrix()[2, 2], -1, 1)))
        clearance = bottom_film_clearance(plant.data.qpos[:3], rotation, plant.geometry.center_of_mass_height,
                                          plant.metadata['base_radius_m'], plant.metadata['board_top_m']) * 1000
        for index, azimuth in enumerate(Fixed.motion_video_side_azimuths):
            drawing.text((self.panel_width + Fixed.motion_video_font_size, self.line_height * 3 + index * self.side_height),
                         f'Side {azimuth} deg | gap close-up', font=self.font, fill='black')
        drawing.text((Fixed.motion_video_font_size, self.height - self.line_height),
                     f'Displacement: x {displacement[0]:+.3f}, z {displacement[2]:+.3f} mm | tilt {tilt:.3f} deg | surface clearance {clearance:.3f} mm',
                     font=self.font, fill='black')
        return frame

    def write(self, labels):
        frame = self.frame(labels)
        self.encoder.stdin.write(frame.tobytes())
        self.frames += 1
        return frame

    def __exit__(self, exception_type, exception, traceback):
        self.encoder.stdin.close()
        return_code = self.encoder.wait()
        self.overview.close()
        self.detail.close()
        self.plant.model.geom_rgba[self.surface_id, 3] = self.surface_alpha
        self.plant.model.geom_rgba[self.skin_id, 3] = self.skin_alpha
        if return_code:
            raise subprocess.CalledProcessError(return_code, self.command)
        return False


def record_video(plant, path):
    plant.reset()
    phase_steps = round(Fixed.qualification_recovery_duration / plant.model.opt.timestep)
    phase_duration = phase_steps * plant.model.opt.timestep
    frame_intervals = round(phase_duration * Fixed.motion_video_frame_rate * Fixed.motion_video_slowdown)
    with MotionVideo(plant, path) as video:
        for phase in ('Drop and settle', 'Slide after initial push'):
            if phase == 'Slide after initial push':
                plant.data.qvel[0] = 2 * Fixed.resting_friction_coefficient * Constants.gravity * Fixed.motion_check_duration
                mujoco.mj_forward(plant.model, plant.data)
            video.initial_position = plant.data.qpos[:3].copy()
            completed_steps = 0
            for frame_index in range(frame_intervals + 1):
                target_step = round(frame_index * phase_steps / frame_intervals)
                for _ in range(target_step - completed_steps):
                    plant.advance_wrench(np.zeros(6))
                completed_steps = target_step
                video.write([f'{phase} | coils OFF | no controller',
                             f'Simulated phase time: {completed_steps * plant.model.opt.timestep:.4f} s | {Fixed.motion_video_slowdown}x slow motion',
                             'Overview + two gap views | translucent playing surface in side views'])
    return dict(path=str(path), scope='Unpowered drop/settle followed by initial-velocity sliding; no levitation controller',
                frames=video.frames, frame_rate=Fixed.motion_video_frame_rate, playback_slowdown=Fixed.motion_video_slowdown,
                simulated_duration_s=float(plant.data.time), playback_duration_s=video.frames / Fixed.motion_video_frame_rate,
                views=['overview', *[f'side_{angle}_deg_gap' for angle in Fixed.motion_video_side_azimuths]],
                physics_time_step_s=plant.model.opt.timestep)


def run(output, preview, video=False):
    construction = magnetics.ConstructionModel()
    plant = MotionPlant(construction)
    report = dict(scope='CAD/MuJoCo motion and direct magnetic-force interface verification; virtual hardware loop not yet implemented',
                  mujoco_version=mujoco.__version__, source_sha256=plant.provenance, scene=plant.metadata,
                  time_step_s=plant.model.opt.timestep, mechanical_checks=mechanical_checks(plant, construction),
                  magnetic_check=magnetic_check(plant),
                  limitations=['No physical sensor estimator, driver dynamics or closed-loop control in this check.',
                               'Contact parameters are provisional; manufactured winding clearance and material behavior remain unresolved.',
                               'The magnetic circuit retains the provisional local PCB closure and nominal material properties.'])
    scene_path = Path(__file__).with_name('cad') / 'output' / 'motion_scene.xml'
    scene_path.write_text(plant.scene + '\n')
    if preview:
        plant.reset()
        render(plant, scene_path.with_suffix('.png'))
    if video:
        report['video'] = record_video(plant, scene_path.with_suffix('.mp4'))
    output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    if not report['magnetic_check']['passed'] or not report['mechanical_checks']['passed']:
        raise RuntimeError('Motion interface verification failed; see the saved report')
    print(json.dumps(report['magnetic_check'], indent=Fixed.report_precision), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Verify the MuJoCo interface using CAD mass and direct magnetic forces.')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('results') / 'motion.json')
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--video', action='store_true', help='Record slow-motion unpowered drop/settle and sliding diagnostics.')
    arguments = parser.parse_args()
    run(arguments.output, arguments.preview, arguments.video)

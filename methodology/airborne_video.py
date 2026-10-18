import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from model import Fixed
from methodology import motion, source_fingerprints
from methodology.cad import magnetics


def render_report(report_path, output):
    recorded_report = report_path.read_bytes()
    report = json.loads(recorded_report)
    root = Path(__file__).resolve().parent.parent
    if any(hashlib.sha256((root / name).read_bytes()).hexdigest() != value for name, value in report['source_sha256'].items()):
        raise ValueError('Video geometry/source differs from the recorded simulation')
    trace = report['trace']
    times = np.array([row['time_s'] for row in trace])
    poses = np.array([row['pose'] for row in trace])
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ValueError('Video needs at least two chronological physical poses')
    rotations = Slerp(times, Rotation.from_euler('xyz', poses[:, 3:]))
    plant = motion.MotionPlant(magnetics.ConstructionModel())
    duration = times[-1] - times[0]
    slowdown = max(Fixed.motion_video_slowdown, Fixed.airborne_video_min_duration / duration)
    intervals = int(np.ceil(duration * slowdown * Fixed.motion_video_frame_rate))
    plant.reset(position=poses[0, :3] + [0, 0, plant.geometry.center_of_mass_height], rotation=rotations(times[0]))
    status = report['stop_reason'] if report['stop_reason'] is not None else ('completed' if report['finished'] else 'in progress')
    with motion.MotionVideo(plant, output) as video:
        for time in np.linspace(times[0], times[-1], intervals + 1):
            plant.data.qpos[:3] = [np.interp(time, times, poses[:, axis]) for axis in range(3)]
            plant.data.qpos[2] += plant.geometry.center_of_mass_height
            plant.data.qpos[3:] = rotations(time).as_quat(scalar_first=True)
            plant.data.time = time
            mujoco.mj_forward(plant.model, plant.data)
            current = np.interp(time, times, [row['peak_current_A'] for row in trace])
            frame = video.write(['Sensor-controlled airborne trial | NOT QUALIFIED',
                                f'Time {time * 1000:.3f} ms | {slowdown:.0f}x slow motion | current {current:.2f} A | no motion exaggeration',
                                f'Result: {status.replace("_", " ")}'])
        frame.save(output.with_suffix('.png'))
    metadata = dict(path=str(output), frames=video.frames, slowdown=slowdown, simulated_duration_s=duration,
                    start_time_s=float(times[0]), end_time_s=float(times[-1]), trial_finished=report['finished'],
                    trial_stop_reason=report['stop_reason'], trial_passed=report['passed'],
                    views=['overview', *[f'side_{angle}_deg_gap' for angle in Fixed.motion_video_side_azimuths]],
                    source_report_sha256=hashlib.sha256(recorded_report).hexdigest(),
                    simulation_source_sha256=report['source_sha256'], render_source_sha256=source_fingerprints(__file__))
    output.with_suffix('.json').write_text(json.dumps(metadata, indent=Fixed.report_precision) + '\n')
    return metadata


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Render saved airborne trajectories without repeating dynamics.')
    parser.add_argument('--report', type=Path, default=Path(__file__).with_name('results') / 'airborne.json')
    parser.add_argument('--output', type=Path, help='Video path; defaults to beside the input report.')
    args = parser.parse_args()
    print(json.dumps(render_report(args.report, args.output if args.output is not None else args.report.with_name(args.report.stem + '_video.mp4'))))

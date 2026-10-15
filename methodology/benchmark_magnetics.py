import argparse
import json
import platform
from pathlib import Path
from time import perf_counter
from unittest.mock import patch

import magpylib
import numpy as np
from magpylib_force import getFT

import levitation_sim
from model import Inputs, Fixed
from methodology import airborne, source_fingerprints
from methodology.cad import magnetics


def library_actuator_matrix(layout, array, center_of_mass):
    filaments = [filament for coil in array.coils for filament in coil]
    values = np.asarray(getFT(layout.collection, filaments, anchor=center_of_mass, eps=Fixed.force_gradient_step))
    return -values.reshape(len(array.coils), -1, 6).sum(axis=1).T


def benchmark(output):
    construction = magnetics.ConstructionModel()
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    layout = levitation_sim.magnet_layout_from_geometry()
    tilt = levitation_sim.rim_tilt_limit(layout, geometry.max_flight_gap) * min(Fixed.pose_tilt_fractions)
    phase = max(Fixed.pose_phase_fractions) * array.short_pitch
    poses = ([0] * 6, [phase, -phase, geometry.max_flight_gap - geometry.gap, tilt, -tilt, np.radians(max(Fixed.pose_yaws_deg))])
    report = dict(scope='Runtime and numerical equivalence of direct force evaluation; not flight qualification',
                  source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__)),
                  environment=dict(platform=platform.platform(), python=platform.python_version(),
                                   numpy=np.__version__, magpylib=magpylib.__version__),
                  workers=Fixed.force_field_workers, batch_points=Fixed.force_field_batch_points, matrices=[])
    for pose in poses:
        _, center = levitation_sim.place_piece(layout, *pose)
        values, timings = {}, {}
        for name, evaluator, workers in (('cached_serial', levitation_sim.actuator_matrix, 1),
                                         ('cached_parallel', levitation_sim.actuator_matrix, Fixed.force_field_workers)):
            times = []
            for _ in range(Fixed.qualification_refinement_factor):
                with patch.object(Fixed, 'force_field_workers', workers):
                    started = perf_counter()
                    values[name] = evaluator(layout, array, center)
                    times.append(perf_counter() - started)
            timings[name] = times
        differences = {name: float(np.max(np.abs(value - values['cached_serial']))) for name, value in values.items()}
        for value in values.values():
            np.testing.assert_allclose(value, values['cached_serial'], atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
        row = dict(pose=pose, timings_s=timings, max_absolute_matrix_differences=differences,
                   coil_count=len(array.coils), wire_quadrature_points=len(array.force_quadrature.points), passed=True)
        report['matrices'].append(row)
        output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
        print(json.dumps(row), flush=True)
    trajectories = {}
    for name, workers in (('serial', 1), ('parallel', Fixed.force_field_workers)):
        with patch.object(Fixed, 'force_field_workers', workers):
            simulation = airborne.AirborneSimulation(construction, Fixed.airborne_check_duration, disturbed=True)
            started = perf_counter()
            samples = []
            while not simulation.finished:
                simulation.advance()
                samples.append(np.r_[simulation.plant.data.qpos, simulation.plant.data.qvel,
                                     simulation.drivers.currents, simulation.drivers.voltage])
                if simulation.step % simulation.checkpoint_tick == 0 or simulation.finished:
                    print(f'{name}: {simulation.plant.data.time:.6f} s, stop={simulation.stop_reason}', flush=True)
            trajectories[name] = dict(state=simulation.checkpoint_state(), samples=np.array(samples),
                                      elapsed_s=perf_counter() - started)
    reference, actual = trajectories['serial'], trajectories['parallel']
    np.testing.assert_allclose(actual['samples'], reference['samples'],
                               atol=Fixed.motion_verification_tolerance, rtol=Fixed.matrix_rank_tolerance)
    same_state = actual['state'] == reference['state']
    if not same_state:
        raise AssertionError('Direct force optimization changed the short coupled checkpoint state')
    report['coupled'] = dict(duration_s=simulation.duration, disturbed=simulation.disturbed, steps=simulation.step,
                             delivered_scans=simulation.delivered_scans, stop_reason=simulation.stop_reason,
                             physics_time_step_s=simulation.period, states_exactly_equal=same_state,
                             max_absolute_trajectory_difference=float(np.max(np.abs(actual['samples'] - reference['samples']))),
                             timings_s={name: trial['elapsed_s'] for name, trial in trajectories.items()}, passed=True,
                             source_sha256=simulation.source)
    output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    print(json.dumps(report['coupled']), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compare optimized magnetic forces and coupled motion against magpylib-force.')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('results') / 'magnetic_performance.json')
    arguments = parser.parse_args()
    benchmark(arguments.output)

import argparse
import cProfile
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
import platform
import pstats
from pathlib import Path
from time import perf_counter
from unittest.mock import patch

import magpylib
import numpy as np
from magpylib_force import getFT
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Inputs, Fixed
from methodology import airborne, source_fingerprints
from methodology.cad import magnetics
from methodology.joint_airborne import JointAirborneSimulation


def library_actuator_matrix(layout, array, center_of_mass):
    filaments = [filament for coil in array.coils for filament in coil]
    values = np.asarray(getFT(layout.collection, filaments, anchor=center_of_mass, eps=Fixed.force_gradient_step))
    return -values.reshape(len(array.coils), -1, 6).sum(axis=1).T


def worker_candidates():
    return sorted({1, Fixed.force_field_workers,
        min(Fixed.force_field_workers * Fixed.qualification_refinement_factor, len(os.sched_getaffinity(0)))})


def recorded_force_geometry(checkpoint):
    construction = magnetics.ConstructionModel(recorded_piece_properties=checkpoint['construction']['piece_properties'])
    construction.use_cad_mass = checkpoint['construction']['use_cad_mass']
    construction.return_depth = checkpoint['construction']['return_depth']
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    levitation_sim.use_geometry(geometry)
    layout = levitation_sim.magnet_layout_from_geometry()
    array.force_quadrature = array.create_force_quadrature()
    return layout, array


def benchmark_batches(folder, output):
    if output.exists():
        raise ValueError('Preserve existing batching evidence')
    checkpoint = json.loads((folder / 'initial_checkpoint.json').read_text())
    recorded = json.loads((folder / 'summary.json').read_text())
    layout, array = recorded_force_geometry(checkpoint)
    report = dict(scope='Recorded CAD and poses replayed for batching equivalence; not a checkpoint continuation',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__)),
        input_sha256={name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
                      for name in ('summary.json', 'initial_checkpoint.json')},
        workers=Fixed.force_field_workers, batch_candidates=list(Fixed.force_field_batch_candidates),
        wire_points=len(array.force_quadrature.points), probes=[], completed=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    trace = recorded['trace']
    for row in (trace[0], trace[len(trace) // Fixed.qualification_refinement_factor], trace[-1]):
        _, center = levitation_sim.place_piece(layout, *row['pose'])
        reference = None
        probe = dict(time_s=row['time_s'], pose=row['pose'], cases=[])
        report['probes'].append(probe)
        for batch in Fixed.force_field_batch_candidates:
            case = dict(batch_points=batch, timings_s=[], max_absolute_difference=0., exactly_equal=True)
            probe['cases'].append(case)
            for _ in range(Fixed.qualification_refinement_factor):
                with patch.object(Fixed, 'force_field_batch_points', batch):
                    started = perf_counter()
                    matrix = levitation_sim.actuator_matrix(layout, array, center)
                    case['timings_s'].append(perf_counter() - started)
                if reference is None:
                    reference = matrix.copy()
                case['max_absolute_difference'] = max(case['max_absolute_difference'], float(np.max(np.abs(matrix - reference))))
                case['exactly_equal'] = bool(np.array_equal(matrix, reference))
                output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
                np.testing.assert_array_equal(matrix, reference)
    report['median_seconds'] = {str(batch): float(np.median([duration for probe in report['probes']
        for case in probe['cases'] if case['batch_points'] == batch for duration in case['timings_s']]))
        for batch in Fixed.force_field_batch_candidates}
    report['completed'] = True
    output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    print(json.dumps(report['median_seconds']), flush=True)


def benchmark_atlas_workers(folder, output):
    from methodology.magnetic_atlas import MagneticFieldAtlas
    if output.exists():
        raise ValueError('Preserve existing atlas worker evidence')
    atlas = MagneticFieldAtlas.load(folder / 'fields/atlas.npz')
    checkpoint = json.loads((folder / 'integration/initial_checkpoint.json').read_text())
    recorded = json.loads((folder / 'coupled/direct.json').read_text())
    layout, array = recorded_force_geometry(checkpoint)
    workers_to_compare = worker_candidates()
    report = dict(scope='Identical atlas queries split across CPU workers; frozen poses, not coupled qualification',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__)), atlas_identity=atlas.identity,
        input_sha256={name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
            for name in ('integration/initial_checkpoint.json', 'coupled/direct.json')},
        workers=workers_to_compare, wire_points=len(array.force_quadrature.points), probes=[], completed=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    trace = recorded['trace']
    for row in (trace[0], trace[len(trace) // Fixed.qualification_refinement_factor], trace[-1]):
        _, center = levitation_sim.place_piece(layout, *row['pose'])
        reference_field = atlas.field(layout.collection, array.force_quadrature.points)
        reference_matrix = levitation_sim.actuator_matrix(layout, array, center, field_evaluator=atlas.field)
        probe = dict(time_s=row['time_s'], pose=row['pose'], cases=[])
        report['probes'].append(probe)
        for workers in workers_to_compare:
            case = dict(workers=workers, timings_s=[], exactly_equal=True)
            probe['cases'].append(case)
            for _ in range(Fixed.qualification_refinement_factor):
                evaluated = []
                def evaluate(source, points):
                    if workers == 1:
                        fields = atlas.field(source, points)
                    else:
                        with ThreadPoolExecutor(max_workers=workers) as executor:
                            fields = np.concatenate(list(executor.map(lambda batch: atlas.field(source, batch), np.array_split(points, workers))))
                    evaluated.append(fields)
                    return fields
                started = perf_counter()
                matrix = levitation_sim.actuator_matrix(layout, array, center, field_evaluator=evaluate)
                case['timings_s'].append(perf_counter() - started)
                case['exactly_equal'] = bool(np.array_equal(evaluated[0], reference_field) and np.array_equal(matrix, reference_matrix))
                output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
                np.testing.assert_array_equal(evaluated[0], reference_field)
                np.testing.assert_array_equal(matrix, reference_matrix)
    report['median_seconds'] = {str(workers): float(np.median([duration for probe in report['probes']
        for case in probe['cases'] if case['workers'] == workers for duration in case['timings_s']]))
        for workers in workers_to_compare}
    report['completed'] = True
    output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    print(json.dumps(report['median_seconds']), flush=True)


def profile_statistics(profiler):
    statistics = pstats.Stats(profiler)
    functions = [dict(file=key[0], line=key[1], function=key[2], primitive_calls=value[0],
                      calls=value[1], own_seconds=value[2], inclusive_seconds=value[3])
                 for key, value in statistics.stats.items()]
    return sorted(functions, key=lambda row: row['inclusive_seconds'], reverse=True)


def profile_hover(folder, output):
    if output.exists():
        raise ValueError('Preserve existing profiling evidence')
    recorded = json.loads((folder / 'summary.json').read_text())
    started = perf_counter()
    simulation = JointAirborneSimulation.from_checkpoint(folder / 'initial_checkpoint.json')
    setup_seconds = perf_counter() - started
    if recorded['source_sha256'] != simulation.source:
        raise ValueError('Recorded hover source differs from the profiled simulation')
    profiler = cProfile.Profile()
    started = perf_counter()
    profiler.runcall(simulation.advance)
    interval_seconds = perf_counter() - started
    functions = profile_statistics(profiler)
    report = dict(scope='Current-source workload profile and direct-force thread comparison; no flight qualification',
        source_sha256=simulation.source, benchmark_source_sha256=source_fingerprints(__file__),
        input_sha256={name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
                      for name in ('summary.json', 'initial_checkpoint.json')},
        environment=dict(platform=platform.platform(), python=platform.python_version(),
                         logical_cpus=os.cpu_count(), affinity_cpus=len(os.sched_getaffinity(0))),
        setup_seconds=setup_seconds,
        interval=dict(duration_s=simulation.clock, elapsed_s=interval_seconds,
                      integration=simulation.report()['integration'],
                      functions=sorted(functions, key=lambda row: row['inclusive_seconds'], reverse=True)),
        recorded_workload=dict(elapsed_s=recorded['elapsed_s'], duration_s=recorded['duration_completed_s'],
                               integration=recorded['integration']), probes=[])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    workers_to_compare = worker_candidates()
    trace = recorded['trace']
    for row in (trace[0], trace[len(trace) // Fixed.qualification_refinement_factor], trace[-1]):
        pose = np.asarray(row['pose'])
        simulation.plant.reset(position=pose[:3] + [0, 0, simulation.plant.geometry.center_of_mass_height],
                               rotation=Rotation.from_euler('xyz', pose[3:]))
        simulation.plant.magnetic_matrix(window_center=simulation.reference[:2])
        center = simulation.plant.truth()['com_position_m']
        reference = None
        timings = {}
        differences = {}
        for workers in workers_to_compare:
            timings[str(workers)] = []
            for _ in range(Fixed.qualification_refinement_factor):
                with patch.object(Fixed, 'force_field_workers', workers):
                    started = perf_counter()
                    matrix = levitation_sim.actuator_matrix(simulation.plant.magnets, simulation.plant.array, center)
                    timings[str(workers)].append(perf_counter() - started)
                if reference is None:
                    reference = matrix.copy()
                np.testing.assert_array_equal(matrix, reference)
                differences[str(workers)] = float(np.max(np.abs(matrix - reference)))
        field_times = []
        for _ in range(Fixed.qualification_refinement_factor):
            started = perf_counter()
            simulation.field_signal(simulation.plant, np.asarray(row['currents_A']))
            field_times.append(perf_counter() - started)
        report['probes'].append(dict(time_s=row['time_s'], pose=row['pose'], force_timings_s=timings,
            max_absolute_matrix_differences=differences, hall_signal_timings_s=field_times,
            wire_points=len(simulation.plant.array.force_quadrature.points)))
        output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    report['completed'] = True
    output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    print(json.dumps(dict(output=str(output), interval_seconds=interval_seconds, probes=report['probes'])), flush=True)


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
    parser.add_argument('--profile-hover', type=Path)
    parser.add_argument('--batch-hover', type=Path)
    parser.add_argument('--atlas-workers', type=Path)
    arguments = parser.parse_args()
    if arguments.atlas_workers is not None:
        if arguments.batch_hover is not None or arguments.profile_hover is not None:
            parser.error('Atlas worker benchmark requires a single mode')
        benchmark_atlas_workers(arguments.atlas_workers, arguments.output)
    elif arguments.batch_hover is not None:
        benchmark_batches(arguments.batch_hover, arguments.output)
    elif arguments.profile_hover is None:
        benchmark(arguments.output)
    else:
        profile_hover(arguments.profile_hover, arguments.output)

import argparse
import cProfile
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
import platform
import pstats
from pathlib import Path
from time import perf_counter, process_time
from unittest.mock import patch
from zipfile import ZipFile, ZIP_DEFLATED

import magpylib
import numpy as np
from magpylib_force import getFT
from scipy.spatial.transform import Rotation
from threadpoolctl import threadpool_info, threadpool_limits

import levitation_sim
from model import Inputs, Fixed
from methodology import airborne, source_fingerprints
from methodology.cad import magnetics
from methodology.joint_airborne import JointAirborneSimulation
from methodology.magnetic_spline import spline_workers


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


def benchmark_spline(folder, library, output):
    from methodology import magnetic_atlas, magnetic_spline
    if output.exists():
        raise ValueError('Preserve existing spline comparison evidence')
    atlas = magnetic_atlas.MagneticFieldAtlas.load(folder / 'fields/atlas.npz')
    checkpoint = json.loads((folder / 'trial/initial_checkpoint.json').read_text())
    recorded = json.loads((folder / 'trial/summary.json').read_text())
    layout, array = recorded_force_geometry(checkpoint)
    reference = atlas.interpolator
    candidate = magnetic_spline.TensorSplineEvaluator(reference, library)
    report = dict(scope='Same-coefficient native spline evaluation at recorded and domain poses; not flight qualification',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, magnetic_atlas.__file__,
            magnetic_spline.__file__, Path(magnetic_spline.__file__).with_suffix('.c'))),
        input_sha256={name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
            for name in ('trial/summary.json', 'trial/initial_checkpoint.json', 'fields/summary.json')},
        atlas_identity=atlas.identity, native_build=candidate.build, coefficient_bytes=atlas.coefficients.nbytes,
        wire_points=len(array.force_quadrature.points), probes=[], status='running',
        equivalence_tolerances=dict(absolute=Fixed.singular_value_tolerance, relative=Fixed.matrix_rank_tolerance),
        speed_target=Fixed.atlas_minimum_speedup)
    output.parent.mkdir(parents=True, exist_ok=True)

    def save():
        output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')

    random = np.random.default_rng(Fixed.atlas_validation_seed)
    points = random.uniform(*atlas.bounds, size=(Fixed.atlas_validation_points, 3))
    corners = np.array(np.meshgrid(*atlas.bounds.T)).reshape(3, -1).T
    points = np.vstack((points, corners))
    expected = reference(points)
    actual = candidate(points)
    report['domain_field_difference_T'] = magnetic_atlas.vector_error(actual, expected)
    report['domain_equivalent'] = bool(np.allclose(actual, expected,
        atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance))
    save()
    if not report['domain_equivalent']:
        report['status'] = 'domain_failed'
        save()
        return report
    trace = recorded['trace']
    rows = [trace[0], trace[len(trace) // Fixed.qualification_refinement_factor], trace[-1]]
    poses = [dict(kind='recorded_motion', time_s=row['time_s'], pose=row['pose']) for row in rows]
    poses.extend(dict(kind='domain_pose', pose=pose.tolist())
        for pose in magnetic_atlas.validation_poses(levitation_sim.G, array, layout))
    for pose in poses:
        _, center = levitation_sim.place_piece(layout, *pose['pose'])
        source = layout.collection
        points = array.force_quadrature.points
        probe = dict(**pose, timings_s={}, checks={})
        report['probes'].append(probe)
        cases = {}
        for name, evaluator in (('scipy', reference), ('native', candidate)):
            with patch.object(atlas, 'interpolator', evaluator):
                fields = atlas.field(source, points)
                gradient = magnetic_atlas.field_jacobian(atlas.field, source, points[::len(points) // Fixed.atlas_validation_points])
                timings = []
                for _ in range(Fixed.qualification_refinement_factor):
                    started = perf_counter()
                    matrix = levitation_sim.actuator_matrix(layout, array, center, field_evaluator=atlas.field)
                    timings.append(perf_counter() - started)
                cases[name] = dict(field=fields, gradient=gradient, matrix=matrix)
                probe['timings_s'][name] = timings
        for quantity in ('field', 'gradient', 'matrix'):
            actual, expected = cases['native'][quantity], cases['scipy'][quantity]
            probe[quantity + '_max_absolute_difference'] = float(np.max(np.abs(actual - expected)))
            absolute_tolerance = Fixed.singular_value_tolerance / Fixed.hall_position_derivative_step if quantity == 'gradient' else Fixed.singular_value_tolerance
            probe['checks'][quantity] = bool(np.allclose(actual, expected,
                atol=absolute_tolerance, rtol=Fixed.matrix_rank_tolerance))
        probe['checks']['field_budget'] = magnetic_atlas.vector_error(cases['native']['field'], cases['scipy']['field']) <= Fixed.atlas_field_tolerance
        for start, name in ((0, 'force'), (3, 'torque')):
            actual, expected = cases['native']['matrix'][start:start + 3], cases['scipy']['matrix'][start:start + 3]
            discrepancy = magnetic_atlas.vector_error(actual.T, expected.T) / np.max(np.linalg.norm(expected, axis=0))
            probe[name + '_relative_difference'] = float(discrepancy)
            probe['checks'][name] = bool(discrepancy <= Fixed.atlas_matrix_relative_tolerance)
        probe['passed'] = all(probe['checks'].values())
        save()
        if not probe['passed']:
            report['status'] = 'equivalence_failed'
            save()
            return report
    report['median_matrix_s'] = {name: float(np.median([duration for probe in report['probes']
        if probe['kind'] == 'recorded_motion' for duration in probe['timings_s'][name]])) for name in ('scipy', 'native')}
    report['matrix_speedup'] = report['median_matrix_s']['scipy'] / report['median_matrix_s']['native']
    report['status'] = 'passed' if report['matrix_speedup'] >= Fixed.atlas_minimum_speedup else 'speed_target_not_met'
    save()
    return report


def save_spline_diagnostic(simulation, evaluator, path):
    from methodology.airborne_verification import save_summary
    state = simulation.checkpoint_state()
    state.update(schema='spline-evaluator-diagnostic-v1', evaluator=evaluator,
        scope='Diagnostic snapshot; production checkpoint resume is forbidden')
    save_summary(path, state)


def compare_spline_coupled(folder, library, field_screen, output):
    from methodology import airborne_verification, hover, magnetic_atlas, magnetic_spline, numerical_accuracy
    if output.exists():
        raise ValueError('Preserve existing coupled spline evidence')
    gate = json.loads(field_screen.read_text())
    atlas = magnetic_atlas.MagneticFieldAtlas.load(folder / 'fields/atlas.npz')
    native = magnetic_spline.TensorSplineEvaluator(atlas.interpolator, library)
    if gate['status'] != 'passed' or gate['atlas_identity'] != atlas.identity or gate['native_build'] != native.build:
        raise ValueError('Coupled spline comparison requires matching successful field evidence')
    initial_geometry = json.loads((folder / 'trial/initial_checkpoint.json').read_text())['construction']
    construction = magnetics.ConstructionModel(recorded_piece_properties=initial_geometry['piece_properties'])
    construction.use_cad_mass = initial_geometry['use_cad_mass']
    construction.return_depth = initial_geometry['return_depth']
    reference = atlas.interpolator
    started = perf_counter()
    simulation = JointAirborneSimulation(construction, Fixed.airborne_check_duration, atlas=atlas, magnetic_mode='atlas_forces')
    sources = source_fingerprints(__file__, magnetic_spline.__file__, Path(magnetic_spline.__file__).with_suffix('.c'),
        numerical_accuracy.__file__, hover.__file__, airborne_verification.__file__)
    simulation.source.update(sources)
    initial = deepcopy(simulation.checkpoint_state())
    output.mkdir(parents=True)
    summary = dict(scope='Bounded coupled startup with identical spline coefficients; not sustained or moving-flight qualification',
        status='running', source_sha256=simulation.source, native_build=native.build, atlas_identity=atlas.identity,
        input_sha256=source_fingerprints(field_screen, folder / 'trial/initial_checkpoint.json'),
        initialization_s=perf_counter()-started, cases={}, comparisons={})
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in simulation.source:
            archive.write(name, name)
    summary_path = output / 'summary.json'
    airborne_verification.save_summary(summary_path, summary)
    reports, scans = {}, {}
    for name, scale, evaluator in (('scipy', 1, reference), ('native', 1, native),
            ('native_tighter', 1 / Fixed.qualification_refinement_factor, native)):
        case_path = output / name
        case_path.mkdir()
        if name != 'scipy':
            simulation = JointAirborneSimulation(construction, Fixed.airborne_check_duration, initialize=False,
                atlas=atlas, magnetic_mode='atlas_forces')
            simulation.source.update(sources)
            simulation.restore_checkpoint(deepcopy(initial))
        simulation.tolerance_scale = scale
        identity = dict(method=name, native_build_sha256=None if name == 'scipy' else native.build['library_sha256'])
        save_spline_diagnostic(simulation, identity, case_path / 'initial_snapshot.json')
        recorder = numerical_accuracy.JointTrajectoryRecorder(simulation)
        def progress(report):
            hover.check_operating_limits(simulation, report)
            if name != 'scipy' and simulation.stop_reason is None:
                airborne_verification.compare_progress(reports['scipy'], report, simulation, name)
        simulation.started = perf_counter()
        with patch.object(atlas, 'interpolator', evaluator), patch.object(simulation.dynamics, 'integrate', recorder.integrate), \
                patch.object(simulation.sensors, 'encode_signal', recorder.encode_signal), \
                patch.object(simulation, 'save_checkpoint', lambda path: save_spline_diagnostic(simulation, identity, path)):
            report = airborne.run_simulation(simulation, case_path / 'report.json', case_path / 'snapshot.json', progress=progress)
        report.update(evaluator=identity)
        airborne_verification.save_summary(case_path / 'report.json', report)
        evidence = dict(scans=recorder.scans, dense_sha256=recorder.save_dense(case_path / 'dense.npz'))
        airborne_verification.save_summary(case_path / 'evidence.json', evidence)
        reports[name], scans[name] = report, recorder.scans
        summary['cases'][name] = {key: report[key] for key in ('passed', 'elapsed_s', 'duration_completed_s',
            'delivered_moving_scans', 'stop_reason', 'peaks', 'integration')}
        if not report['passed']:
            summary['status'] = name + '_operating_failed'
            airborne_verification.save_summary(summary_path, summary)
            return summary
        if name != 'scipy':
            comparison = airborne_verification.compare_trials(reports['scipy'], report)
            same_times = [s['start_s'] for s in scans[name]] == [s['start_s'] for s in scans['scipy']]
            comparison['checks']['same_scan_times'] = same_times
            if same_times:
                field_difference = float(np.max(np.abs(np.array([s['samples_T'] for s in scans[name]])
                    - np.array([s['samples_T'] for s in scans['scipy']]))))
                comparison.update(max_analogue_sample_difference_T=field_difference, analogue_limit_T=Fixed.atlas_field_tolerance)
                comparison['checks']['analogue_samples'] = field_difference <= Fixed.atlas_field_tolerance
            comparison['converged'] = all(comparison['checks'].values())
            summary['comparisons'][name] = comparison
            if not comparison['converged']:
                summary['status'] = name + '_comparison_failed'
                airborne_verification.save_summary(summary_path, summary)
                return summary
        airborne_verification.save_summary(summary_path, summary)
    summary.update(status='passed', coupled_speedup=reports['scipy']['elapsed_s'] / reports['native']['elapsed_s'])
    airborne_verification.save_summary(summary_path, summary)
    return summary


def initial_physical_state(state):
    state = deepcopy(state)
    for key in ('schema', 'source_sha256', 'atlas'):
        state.pop(key)
    if state.pop('motion_scenario', 'hover') != 'hover':
        raise ValueError('Native hover comparison requires the stationary reference')
    state['plant'].pop('source_sha256')
    return state


def prepare_native_hover(reference_folder, output):
    from methodology import airborne_verification, hover, magnetic_atlas, magnetic_spline, numerical_accuracy
    from scipy.interpolate import NdBSpline
    if output.exists():
        raise ValueError('Preserve existing native hover evidence')
    reference = json.loads((reference_folder / 'summary.json').read_text())
    initial = json.loads((reference_folder / 'initial_checkpoint.json').read_text())
    field_folder = reference_folder.parent / 'fields'
    field_screen = json.loads((field_folder / 'summary.json').read_text())
    if not reference['passed'] or reference['intended_duration_s'] != Fixed.airborne_refinement_duration:
        raise ValueError('Native comparison requires the completed retained hover horizon')
    if field_screen['status'] != 'field_screen_passed' or reference['atlas']['identity'] != field_screen['atlas_identity']:
        raise ValueError('Reference hover and field qualification disagree')
    output.mkdir(parents=True)
    library = magnetic_spline.build(output / 'native')
    layout, array = recorded_force_geometry(initial)
    atlas = magnetic_atlas.MagneticFieldAtlas(layout.collection, field_screen['bounds_m'],
        field_screen['selected']['spacing_m'], field_screen['selected']['spline_order'])
    atlas.save(output / 'atlas.npz')
    with np.load(field_folder / 'atlas.npz', allow_pickle=False) as original:
        metadata = json.loads(str(original['metadata']))
        golden = NdBSpline(tuple(original[f'knots_{axis}'] for axis in range(3)),
            original['coefficients'], original['orders'], extrapolate=False)
        reference_atlas = magnetic_atlas.MagneticFieldAtlas.__new__(magnetic_atlas.MagneticFieldAtlas)
        reference_atlas.provenance = metadata['provenance']
        reference_atlas.bounds, reference_atlas.source_state = original['bounds'], original['source_state']
        reference_atlas.interpolator, reference_atlas.coefficients = golden, golden.c
        if reference_atlas.content_identity() != metadata['identity'] or metadata['identity'] != field_screen['atlas_identity']:
            raise ValueError('Golden reference atlas content or qualification identity differs')
        identical = all(np.array_equal(a, b) for a, b in zip(
            (atlas.bounds, atlas.source_state, atlas.interpolator.k, *atlas.interpolator.t, atlas.coefficients),
            (reference_atlas.bounds, reference_atlas.source_state, golden.k, *golden.t, golden.c)))
    same_grid = all(np.array_equal(a, b) for a, b in zip(
        (atlas.bounds, atlas.interpolator.k, *atlas.interpolator.t), (reference_atlas.bounds, golden.k, *golden.t)))
    equivalent = same_grid and all(np.allclose(a, b, atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
        for a, b in ((atlas.source_state, reference_atlas.source_state), (atlas.coefficients, golden.c)))
    summary = dict(status='running', scope='Fresh atlas and native runtime qualification against retained SciPy hover; no old checkpoint resume',
        source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__, magnetic_atlas.__file__,
            magnetic_spline.__file__, Path(magnetic_spline.__file__).with_suffix('.c'), numerical_accuracy.__file__,
            hover.__file__, airborne_verification.__file__)),
        inputs_sha256=source_fingerprints(reference_folder / 'summary.json', reference_folder / 'initial_checkpoint.json',
            reference_folder / 'comparison.json', field_folder / 'summary.json', field_folder / 'atlas.npz'),
        atlas_identity=atlas.identity, reference_atlas_identity=metadata['identity'],
        atlas_arrays_exactly_equal=identical, atlas_arrays_equivalent=bool(equivalent), same_grid=bool(same_grid),
        coefficient_max_difference=float(np.max(np.abs(atlas.coefficients - golden.c))),
        coefficient_tolerances=dict(absolute=Fixed.singular_value_tolerance, relative=Fixed.matrix_rank_tolerance), probes=[])
    path = output / 'preflight.json'
    airborne_verification.save_summary(path, summary)
    if not equivalent:
        summary['status'] = 'rebuilt_atlas_comparison_failed'
        airborne_verification.save_summary(path, summary)
        return summary
    with magnetic_spline.native_evaluation(atlas, library) as runtime:
        summary['runtime'] = runtime
        random = np.random.default_rng(Fixed.atlas_validation_seed)
        points = random.uniform(*atlas.bounds, size=(Fixed.atlas_validation_points, 3))
        field_difference = magnetic_atlas.vector_error(atlas.interpolator(points), golden(points))
        summary['domain_field_difference_T'] = field_difference
        summary['domain_field_limit_T'] = Fixed.atlas_field_tolerance
        if field_difference > Fixed.atlas_field_tolerance:
            summary['status'] = 'domain_field_failed'
            airborne_verification.save_summary(path, summary)
            return summary
        trace = reference['trace']
        for row in (trace[0], trace[len(trace) // Fixed.qualification_refinement_factor], trace[-1]):
            _, center = levitation_sim.place_piece(layout, *row['pose'])
            expected = levitation_sim.actuator_matrix(layout, array, center, field_evaluator=reference_atlas.field)
            actual = levitation_sim.actuator_matrix(layout, array, center, field_evaluator=atlas.field)
            passed = bool(np.allclose(actual, expected, atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance))
            summary['probes'].append(dict(time_s=row['time_s'], max_matrix_difference=float(np.max(np.abs(actual-expected))), passed=passed))
            airborne_verification.save_summary(path, summary)
            if not passed:
                summary['status'] = 'matrix_comparison_failed'
                airborne_verification.save_summary(path, summary)
                return summary
        construction = magnetics.ConstructionModel(recorded_piece_properties=initial['construction']['piece_properties'])
        construction.use_cad_mass = initial['construction']['use_cad_mass']
        construction.return_depth = initial['construction']['return_depth']
        simulation = JointAirborneSimulation(construction, Fixed.airborne_refinement_duration, atlas=atlas, magnetic_mode='atlas_forces')
        simulation.save_checkpoint(output / 'initial_checkpoint.json')
        summary['initial_physical_state_exact'] = initial_physical_state(simulation.checkpoint_state()) == initial_physical_state(initial)
        summary['source_sha256'].update(simulation.source)
        with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
            for name in summary['source_sha256']:
                archive.write(name, name)
        summary['status'] = 'passed' if summary['initial_physical_state_exact'] else 'initial_state_failed'
    airborne_verification.save_summary(path, summary)
    return summary


def run_native_hover(reference_folder, output):
    from methodology import airborne_verification, hover, magnetic_atlas, magnetic_spline, numerical_accuracy
    gate = json.loads((output / 'preflight.json').read_text())
    if gate['status'] != 'passed':
        raise ValueError('Native hover requires passing preflight evidence')
    for name, digest in dict(gate['source_sha256'], **gate['inputs_sha256']).items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError('Native hover source or reference changed: ' + name)
    trial = output / 'trial'
    if trial.exists():
        raise ValueError('Preserve existing native hover trial')
    reference = json.loads((reference_folder / 'summary.json').read_text())
    reference_scans = json.loads((reference_folder / 'comparison.json').read_text())['scans']
    atlas = magnetic_atlas.MagneticFieldAtlas.load(output / 'atlas.npz')
    with magnetic_spline.native_evaluation(atlas, output / 'native/magnetic_spline.so') as runtime:
        if runtime != gate['runtime'] or atlas.identity != gate['atlas_identity']:
            raise ValueError('Native runtime differs from its passing preflight')
        simulation = JointAirborneSimulation.from_checkpoint(output / 'initial_checkpoint.json', atlas=atlas)
        if simulation.checkpoint_state() != json.loads((output / 'initial_checkpoint.json').read_text()):
            raise ValueError('Native initial checkpoint did not restore exactly')
        trial.mkdir()
        recorder = numerical_accuracy.JointTrajectoryRecorder(simulation)
        evidence = dict(status='running', source_sha256=gate['source_sha256'], runtime=runtime,
            scope='Native evaluator versus retained SciPy 50-ms hover, unchanged physical/noise/acceptance settings',
            comparisons=[], scans=[], initial_checkpoint_exact=True)
        def progress(report):
            hover.check_operating_limits(simulation, report)
            if report['trace'] and simulation.stop_reason is None:
                comparison = airborne_verification.compare_progress(reference, report, simulation, 'native')
                evidence['comparisons'].append(dict(time_s=report['duration_completed_s'], **comparison))
            expected = reference_scans[:len(recorder.scans)]
            same_times = [scan['start_s'] for scan in recorder.scans] == [scan['start_s'] for scan in expected]
            difference = float(np.max(np.abs(np.array([scan['samples_T'] for scan in recorder.scans])
                - np.array([scan['samples_T'] for scan in expected])))) if same_times and expected else 0.
            evidence['analogue'] = dict(same_scan_times=same_times, max_difference_T=difference,
                limit_T=Fixed.atlas_field_tolerance, passed=same_times and difference <= Fixed.atlas_field_tolerance)
            if not evidence['analogue']['passed'] and simulation.stop_reason is None:
                simulation.stop_reason = 'native_analogue_comparison_failed'
                simulation.finished = True
            evidence.update(scans=recorder.scans, dense_sha256=recorder.save_dense(trial / 'dense.npz'),
                completed_duration_s=report['duration_completed_s'], stop_reason=simulation.stop_reason,
                status='failed' if simulation.stop_reason is not None else 'running')
            airborne_verification.save_summary(trial / 'comparison.json', evidence)
        simulation.started = perf_counter()
        with patch.object(simulation.dynamics, 'integrate', recorder.integrate), patch.object(simulation.sensors, 'encode_signal', recorder.encode_signal):
            report = airborne.run_simulation(simulation, trial / 'summary.json', trial / 'checkpoint.json', progress=progress)
        if report['passed']:
            comparison = airborne_verification.compare_trials(reference, report)
            comparison['checks']['same_scan_count'] = len(recorder.scans) == len(reference_scans)
            comparison['checks']['analogue'] = evidence['analogue']['passed']
            comparison['converged'] = all(comparison['checks'].values())
            evidence['final_comparison'] = comparison
            evidence['status'] = 'passed' if comparison['converged'] else 'comparison_failed'
        evidence.update(elapsed_s=report['elapsed_s'], reference_elapsed_s=reference['elapsed_s'],
            complete_run_speedup=reference['elapsed_s'] / report['elapsed_s'])
        airborne_verification.save_summary(trial / 'comparison.json', evidence)
    return evidence


def profile_statistics(profiler):
    statistics = pstats.Stats(profiler)
    functions = [dict(file=key[0], line=key[1], function=key[2], primitive_calls=value[0],
                      calls=value[1], own_seconds=value[2], inclusive_seconds=value[3])
                 for key, value in statistics.stats.items()]
    return sorted(functions, key=lambda row: row['inclusive_seconds'], reverse=True)


def profile_spline(folder, library, output):
    from methodology import airborne_verification, hover, magnetic_atlas, magnetic_spline
    if output.exists():
        raise ValueError('Preserve existing native profiling evidence')
    atlas = magnetic_atlas.MagneticFieldAtlas.load(folder / 'fields/atlas.npz')
    native = magnetic_spline.TensorSplineEvaluator(atlas.interpolator, library)
    checkpoint = json.loads((folder / 'trial/initial_checkpoint.json').read_text())
    recorded = json.loads((folder / 'trial/summary.json').read_text())
    construction = magnetics.ConstructionModel(recorded_piece_properties=checkpoint['construction']['piece_properties'])
    construction.use_cad_mass = checkpoint['construction']['use_cad_mass']
    construction.return_depth = checkpoint['construction']['return_depth']
    started = perf_counter()
    simulation = JointAirborneSimulation(construction, Fixed.airborne_check_duration, atlas=atlas, magnetic_mode='atlas_forces')
    simulation.source.update(source_fingerprints(__file__, magnetic_spline.__file__,
        Path(magnetic_spline.__file__).with_suffix('.c'), hover.__file__, airborne_verification.__file__))
    report = dict(scope='Fresh startup interval profile and recorded-pose native worker screen; not flight qualification',
        status='running', source_sha256=simulation.source, native_build=native.build, atlas_identity=atlas.identity,
        input_sha256=source_fingerprints(folder / 'trial/initial_checkpoint.json', folder / 'trial/summary.json'),
        environment=dict(platform=platform.platform(), python=platform.python_version(),
            affinity_cpus=sorted(os.sched_getaffinity(0)), threadpools=threadpool_info()),
        setup_seconds=perf_counter() - started, probes=[], warm_intervals=[])
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in simulation.source:
            archive.write(name, name)
    summary_path = output / 'summary.json'
    airborne_verification.save_summary(summary_path, report)
    profiler = cProfile.Profile()
    started = perf_counter()
    cpu_started = process_time()
    with patch.object(atlas, 'interpolator', native):
        profiler.runcall(simulation.advance)
    interval = simulation.report()
    hover.check_operating_limits(simulation, interval)
    report['interval'] = dict(duration_s=simulation.clock, elapsed_s=perf_counter() - started,
        cpu_s=process_time() - cpu_started, integration=interval['integration'],
        stop_reason=simulation.stop_reason, functions=profile_statistics(profiler))
    save_spline_diagnostic(simulation, native.build, output / 'snapshot.json')
    if simulation.stop_reason is not None:
        report['status'] = 'operating_failed'
        airborne_verification.save_summary(summary_path, report)
        return report
    warm_initial = deepcopy(simulation.checkpoint_state())
    warm_reference = None
    for blas_threads in (None, min(worker_candidates())):
        for workers in (min(worker_candidates()), max(worker_candidates())):
            simulation.restore_checkpoint(deepcopy(warm_initial))
            profiler = cProfile.Profile()
            with threadpool_limits(limits=blas_threads, user_api='blas'), spline_workers(native, workers) as evaluator:
                started = perf_counter()
                cpu_started = process_time()
                with patch.object(atlas, 'interpolator', evaluator):
                    profiler.runcall(simulation.advance)
                elapsed, cpu = perf_counter() - started, process_time() - cpu_started
            hover.check_operating_limits(simulation, simulation.report())
            state = simulation.checkpoint_state()
            if warm_reference is None:
                warm_reference = deepcopy(state)
            identical = state == warm_reference
            label = f'workers_{workers}_blas_{blas_threads}'
            save_spline_diagnostic(simulation, dict(workers=workers, blas_threads=blas_threads), output / (label + '.json'))
            report['warm_intervals'].append(dict(workers=workers, blas_threads=blas_threads,
                duration_s=simulation.clock - warm_initial['joint']['clock'], elapsed_s=elapsed, cpu_s=cpu,
                states_exactly_equal=identical, stop_reason=simulation.stop_reason,
                functions=profile_statistics(profiler)))
            airborne_verification.save_summary(summary_path, report)
            if not identical or simulation.stop_reason is not None:
                report['status'] = 'warm_interval_failed'
                airborne_verification.save_summary(summary_path, report)
                return report
    layout, array = recorded_force_geometry(checkpoint)
    report['wire_points'] = len(array.force_quadrature.points)
    trace = recorded['trace']
    for row in (trace[0], trace[len(trace) // Fixed.qualification_refinement_factor], trace[-1]):
        _, center = levitation_sim.place_piece(layout, *row['pose'])
        with patch.object(atlas, 'interpolator', native):
            reference_field = atlas.field(layout.collection, array.force_quadrature.points)
            reference_matrix = levitation_sim.actuator_matrix(layout, array, center, field_evaluator=atlas.field)
        probe = dict(time_s=row['time_s'], pose=row['pose'], cases=[])
        report['probes'].append(probe)
        for blas_threads in (None, min(worker_candidates())):
            for workers in worker_candidates():
                case = dict(workers=workers, blas_threads=blas_threads, timings_s=[], cpu_s=[], exactly_equal=True)
                probe['cases'].append(case)
                with threadpool_limits(limits=blas_threads, user_api='blas'), spline_workers(native, workers) as evaluator:
                    with patch.object(atlas, 'interpolator', evaluator):
                        fields = atlas.field(layout.collection, array.force_quadrature.points)
                        for _ in range(Fixed.qualification_refinement_factor):
                            started = perf_counter()
                            cpu_started = process_time()
                            matrix = levitation_sim.actuator_matrix(layout, array, center, field_evaluator=atlas.field)
                            case['timings_s'].append(perf_counter() - started)
                            case['cpu_s'].append(process_time() - cpu_started)
                            case['exactly_equal'] = bool(np.array_equal(fields, reference_field)
                                and np.array_equal(matrix, reference_matrix))
                            airborne_verification.save_summary(summary_path, report)
                            if not case['exactly_equal']:
                                report['status'] = 'worker_equivalence_failed'
                                airborne_verification.save_summary(summary_path, report)
                                return report
    report['median_matrix_s'] = {f'workers_{workers}_blas_{blas_threads}': float(np.median(
        [duration for probe in report['probes'] for case in probe['cases']
            if case['workers'] == workers and case['blas_threads'] == blas_threads for duration in case['timings_s']]))
        for blas_threads in (None, min(worker_candidates())) for workers in worker_candidates()}
    report['status'] = 'passed'
    airborne_verification.save_summary(summary_path, report)
    return report


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
    parser.add_argument('--spline-movement', type=Path)
    parser.add_argument('--spline-library', type=Path)
    parser.add_argument('--spline-coupled', type=Path, help='Passing spline field screen required for the coupled comparison.')
    parser.add_argument('--spline-profile', action='store_true', help='Profile native startup and compare point-query workers.')
    parser.add_argument('--spline-hover', type=Path, help='Retained SciPy hover folder for native production validation.')
    parser.add_argument('--prepare', action='store_true', help='Build and verify native hover inputs before its long run.')
    arguments = parser.parse_args()
    if arguments.spline_hover is not None:
        if any(value is not None for value in (arguments.spline_movement, arguments.spline_library, arguments.spline_coupled, arguments.atlas_workers, arguments.batch_hover, arguments.profile_hover)) or arguments.spline_profile:
            parser.error('Native hover requires a single benchmark mode')
        report = (prepare_native_hover if arguments.prepare else run_native_hover)(arguments.spline_hover, arguments.output)
        print(json.dumps({key: report[key] for key in ('status', 'elapsed_s', 'complete_run_speedup') if key in report}))
    elif arguments.prepare:
        parser.error('--prepare requires --spline-hover')
    elif arguments.spline_movement is not None:
        if arguments.spline_library is None or any(value is not None for value in (arguments.atlas_workers, arguments.batch_hover, arguments.profile_hover)):
            parser.error('Spline comparison requires its library and no other benchmark mode')
        if arguments.spline_profile and arguments.spline_coupled is not None:
            parser.error('Select either spline profiling or coupled comparison')
        if arguments.spline_profile:
            report = profile_spline(arguments.spline_movement, arguments.spline_library, arguments.output)
        elif arguments.spline_coupled is None:
            report = benchmark_spline(arguments.spline_movement, arguments.spline_library, arguments.output)
        else:
            report = compare_spline_coupled(arguments.spline_movement, arguments.spline_library, arguments.spline_coupled, arguments.output)
        print(json.dumps({key: report[key] for key in ('status', 'median_matrix_s', 'matrix_speedup', 'coupled_speedup') if key in report}))
    elif arguments.spline_library is not None or arguments.spline_coupled is not None or arguments.spline_profile:
        parser.error('--spline-library requires --spline-movement')
    elif arguments.atlas_workers is not None:
        if arguments.batch_hover is not None or arguments.profile_hover is not None:
            parser.error('Atlas worker benchmark requires a single mode')
        benchmark_atlas_workers(arguments.atlas_workers, arguments.output)
    elif arguments.batch_hover is not None:
        benchmark_batches(arguments.batch_hover, arguments.output)
    elif arguments.profile_hover is None:
        benchmark(arguments.output)
    else:
        profile_hover(arguments.profile_hover, arguments.output)

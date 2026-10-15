import argparse
import ast
import hashlib
import json
from pathlib import Path
from time import perf_counter

import numpy as np

import levitation_sim
from model import Inputs, Fixed
from methodology import airborne_verification, magnetic_atlas, physics_verification, sensing, source_fingerprints


def direct_references(folder):
    summary = json.loads((folder / 'summary.json').read_text())
    if summary['status'] != 'completed':
        raise ValueError('Atlas comparison requires completed direct gap checks')
    snapshot = folder / 'verification_source.py'
    original = {node.name: ast.dump(node) for node in ast.parse(snapshot.read_text()).body
        if isinstance(node, (ast.FunctionDef, ast.ClassDef))}
    current = {node.name: ast.dump(node) for node in ast.parse(Path(physics_verification.__file__).read_text()).body
        if isinstance(node, (ast.FunctionDef, ast.ClassDef))}
    orchestration = {'moving_handoff_scenario', 'gap_handoff_scenarios', 'verify_moving_handoff', 'verify_gap_handoffs'}
    if any(current[name] != definition for name, definition in original.items() if name not in orchestration):
        raise ValueError('Recorded direct-trial physical calculation changed')
    root = Path(__file__).resolve().parent.parent
    reports = {}
    for name in summary['cases']:
        report = json.loads((folder / name / 'time_refined.json').read_text())
        if report['stop_reason'] or report['duration_completed_s'] != report['scenario']['duration_s'] or report['atlas_identity'] is not None:
            raise ValueError('Reference must be a completed direct-field trial')
        for filename, digest in report['source_sha256'].items():
            source = snapshot if filename == 'methodology/physics_verification.py' else root / filename
            if hashlib.sha256(source.read_bytes()).hexdigest() != digest:
                raise ValueError(f'Direct reference source differs: {filename}')
        reports[name] = report
    return reports


def validation_geometry(construction, reports):
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    scenarios = physics_verification.gap_handoff_scenarios(geometry, array, Fixed.airborne_check_duration)
    poses = [np.array(row['pose']) for report in reports.values() for row in report['trace']]
    for scenario in scenarios:
        for time in (0., scenario['switch_time_s'], scenario['duration_s']):
            pose = np.array(scenario['pose']) + time * np.array(scenario['world_velocity'])
            poses.append(pose)
    array.window = Fixed.cad_patch_cells_per_side * array.short_pitch
    array.build(center=(0, 0))
    quadrature = array.create_force_quadrature()
    layout = levitation_sim.magnet_layout_from_geometry()
    hall = sensing.HallFieldModel(construction)
    points = np.vstack((quadrature.points, hall.points))
    bounds = np.array([[np.inf] * 3, [-np.inf] * 3])
    for pose in poses:
        levitation_sim.place_piece(layout, *pose)
        source = layout.collection
        local = source.orientation.inv().apply(points - source.position)
        bounds[0] = np.minimum(bounds[0], local.min(axis=0))
        bounds[1] = np.maximum(bounds[1], local.max(axis=0))
    radius = np.max(np.linalg.norm(np.array(np.meshgrid(*zip(*bounds))).reshape(3, -1).T, axis=1))
    margin = 2 * Fixed.hall_position_derivative_step + radius * Fixed.qualification_angle_error
    bounds += np.array([-1, 1])[:, None] * margin
    return geometry, layout, hall, quadrature, poses, bounds


def qualify(output, reference_folder, longer=False):
    if output.exists():
        raise ValueError('Handoff atlas output exists; preserve its evidence')
    reports = direct_references(reference_folder)
    construction = airborne_verification.recorded_construction(next(iter(reports.values())))
    geometry, layout, hall, quadrature, poses, bounds = validation_geometry(construction, reports)
    output.mkdir(parents=True)
    summary = dict(scope='Full-patch atlas validation and prescribed handoff comparisons; no feedback qualification',
        status='building', physical_envelope_qualified=False, bounds_m=bounds.tolist(), poses=[pose.tolist() for pose in poses],
        source_sha256=source_fingerprints(__file__, physics_verification.__file__, magnetic_atlas.__file__),
        reused_evidence_sha256=source_fingerprints(reference_folder / 'summary.json', reference_folder / 'verification_source.py',
            *(reference_folder / name / 'time_refined.json' for name in reports)),
        reuse_scope='Physical classes, integrators, filters and comparisons are AST-identical to the recorded source; only scenario duration and backend selection orchestration changed.',
        spacing_m=Fixed.atlas_grid_spacings[-2], spline_order=max(Fixed.atlas_spline_orders),
        domain_scope='Sampled upright short and planned longer handoffs, with derivative and angular margins; every query retains strict domain checks. Not a full six-axis atlas.',
        limits=dict(field_T=Fixed.atlas_field_tolerance, matrix_relative=Fixed.atlas_matrix_relative_tolerance,
            gradient_relative=Fixed.atlas_gradient_relative_tolerance, minimum_speedup=Fixed.atlas_minimum_speedup),
        static=[], short={}, longer={})
    path = output / 'summary.json'
    airborne_verification.save_summary(path, summary)
    levitation_sim.place_piece(layout)
    started = perf_counter()
    atlas = magnetic_atlas.MagneticFieldAtlas(layout.collection, bounds, summary['spacing_m'], summary['spline_order'])
    summary.update(build_s=perf_counter() - started, coefficient_bytes=atlas.coefficients.nbytes, grid_shape=atlas.shape,
        atlas_identity=atlas.identity, status='validating')
    random = np.random.default_rng(Fixed.atlas_validation_seed)
    probes = random.uniform(*bounds, size=(Fixed.atlas_validation_points, 3))
    probes = np.vstack((probes, np.array(np.meshgrid(*zip(*bounds))).reshape(3, -1).T))
    for axis in range(3):
        for side in range(2):
            face = probes.copy()
            face[:, axis] = bounds[side, axis]
            expected = levitation_sim.magnetic_field(layout.collection, face)
            error = magnetic_atlas.vector_error(atlas.field(layout.collection, face), expected)
            summary.setdefault('face_errors_T', []).append(error)
    expected = levitation_sim.magnetic_field(layout.collection, probes)
    summary['probe_error_T'] = magnetic_atlas.vector_error(atlas.field(layout.collection, probes), expected)
    airborne_verification.save_summary(path, summary)
    if max(summary['face_errors_T'] + [summary['probe_error_T']]) > Fixed.atlas_field_tolerance:
        summary['status'] = 'field_validation_failed'
        airborne_verification.save_summary(path, summary)
        return summary
    points = np.vstack((quadrature.points, hall.points))
    for pose in poses:
        _, center = levitation_sim.place_piece(layout, *pose)
        source = layout.collection
        started = perf_counter()
        direct = levitation_sim.magnetic_field(source, points)
        direct_matrix = quadrature.matrix(source, center, lambda sources, query: direct[:len(query)])
        direct_s = perf_counter() - started
        started = perf_counter()
        cached = atlas.field(source, points)
        cached_matrix = quadrature.matrix(source, center, lambda sources, query: cached[:len(query)])
        cached_s = perf_counter() - started
        metrics = physics_verification.matrix_refinement_metrics(cached_matrix, direct_matrix, Fixed.driver_channel_current)
        gradient = magnetic_atlas.field_jacobian(levitation_sim.magnetic_field, source, hall.points)
        cached_gradient = magnetic_atlas.field_jacobian(atlas.field, source, hall.points)
        gradient_error = np.max(np.linalg.norm(cached_gradient - gradient, axis=(1, 2))) / np.max(np.linalg.norm(gradient, axis=(1, 2)))
        field_error = magnetic_atlas.vector_error(cached, direct)
        passed = (field_error <= Fixed.atlas_field_tolerance and gradient_error <= Fixed.atlas_gradient_relative_tolerance
            and all(metrics[key] <= Fixed.atlas_matrix_relative_tolerance for key in ('force_relative_column_error', 'torque_relative_column_error')))
        summary['static'].append(dict(pose=pose.tolist(), field_error_T=field_error, gradient_relative=float(gradient_error),
            metrics=metrics, direct_s=direct_s, cached_s=cached_s, passed=bool(passed)))
        airborne_verification.save_summary(path, summary)
        if not passed:
            summary['status'] = 'static_validation_failed'
            airborne_verification.save_summary(path, summary)
            return summary
    summary['speedup'] = float(np.median([row['direct_s'] for row in summary['static']]) / np.median([row['cached_s'] for row in summary['static']]))
    if summary['speedup'] < Fixed.atlas_minimum_speedup:
        summary['status'] = 'speed_screen_failed'
        airborne_verification.save_summary(path, summary)
        return summary
    atlas.save(output / 'atlas.npz')
    atlas = magnetic_atlas.MagneticFieldAtlas.load(output / 'atlas.npz')
    for name, reference in reports.items():
        folder = output / name
        folder.mkdir()
        construction = airborne_verification.recorded_construction(reference)
        for mode in ('fixed', 'adaptive'):
            summary.update(status='short_comparison', active_case=f'{name}/{mode}')
            airborne_verification.save_summary(path, summary)
            trial = (physics_verification.CoilHandoffTrial(construction, atlas, reference['scenario'], reference['step_s'],
                reference['magnetic_refinement'], reference['inputs']) if mode == 'fixed' else
                physics_verification.AdaptiveCoilHandoffTrial(construction, atlas, reference))
            if trial.input_identity != reference['input_identity'] or json.loads(json.dumps(trial.plant.metadata)) != reference['mechanical_properties']:
                raise ValueError('Atlas replay changed prescribed inputs or mechanical properties')
            run = physics_verification.run_prescribed_trial if mode == 'fixed' else physics_verification.run_adaptive_prescribed_trial
            comparison = run(trial, reference, folder, mode)
            summary['short'][f'{name}/{mode}'] = dict(stop_reason=trial.stop_reason, comparison=comparison, elapsed_s=trial.report()['elapsed_s'])
            if trial.stop_reason:
                summary.update(status='short_comparison_failed', active_case=None)
                airborne_verification.save_summary(path, summary)
                return summary
    if longer:
        geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
        for scenario in physics_verification.gap_handoff_scenarios(geometry, array, Fixed.airborne_check_duration):
            name = scenario['name']
            summary.update(status='longer_comparison', active_case=name)
            airborne_verification.save_summary(path, summary)
            result = physics_verification.verify_moving_handoff(output / 'longer' / name, scenario, atlas)
            summary['longer'][name] = dict(status=result['status'], evidence=f'longer/{name}/summary.json')
            if result['status'] != 'completed':
                summary.update(status='longer_comparison_failed', active_case=None)
                airborne_verification.save_summary(path, summary)
                return summary
    summary.update(status='completed', active_case=None)
    airborne_verification.save_summary(path, summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Validate full-patch magnetic lookup before longer handoff refinement.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--longer', action='store_true')
    args = parser.parse_args()
    qualify(args.output, args.reference, args.longer)

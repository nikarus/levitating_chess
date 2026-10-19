import argparse
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
from time import perf_counter
from zipfile import ZipFile, ZIP_DEFLATED

import numpy as np
from numpy.polynomial.legendre import leggauss
from scipy.interpolate import NdBSpline
from scipy.spatial.transform import Rotation
from threadpoolctl import threadpool_limits

import levitation_sim
from model import Fixed
from methodology import airborne_verification, coil_field, coupling_verification, magnetic_spline, source_fingerprints
from methodology.benchmark_magnetics import recorded_force_geometry


class DiagnosticCoilCache:
    def __init__(self, folder, library, order, require_pass):
        self.report = json.loads((folder / 'summary.json').read_text())
        case = next(row for row in self.report['candidates'] if row['order'] == order)
        if require_pass and not case['passed']:
            raise ValueError('Candidate coil cache requires passing spatial evidence')
        for name, digest in self.report['source_sha256'].items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
                raise ValueError('Diagnostic coil cache source changed: ' + name)
        for name, digest in self.report['input_sha256'].items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
                raise ValueError('Diagnostic coil cache input changed: ' + name)
        path = folder / f'order_{order}' / 'atlas.npz'
        if hashlib.sha256(path.read_bytes()).hexdigest() != case['cache_sha256']:
            raise ValueError('Diagnostic coil cache content changed')
        with np.load(path, allow_pickle=False) as saved:
            metadata = json.loads(str(saved['metadata']))
            if metadata['schema'] != 'coil-field-screen-v1' or metadata['source_sha256'] != self.report['source_sha256']:
                raise ValueError('Diagnostic coil cache metadata differs from its evidence')
            self.bounds = saved['bounds']
            spline = NdBSpline(tuple(saved[f'knots_{axis}'] for axis in range(3)), saved['coefficients'], saved['orders'], extrapolate=False)
        self.native = magnetic_spline.TensorSplineEvaluator(spline, library)
        self.evaluate = self.native
        self.identity = dict(cache_sha256=case['cache_sha256'], order=order, bounds_m=self.bounds.tolist(),
            native_build=self.native.build, production_eligible=False)

    def __call__(self, points):
        points = np.asarray(points)
        if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
                or np.any(points < self.bounds[0]) or np.any(points > self.bounds[1])):
            raise ValueError('Coil field query is outside its declared exterior domain')
        return self.evaluate(points)


def rigid_segment(start, finish, fraction):
    rotation = Rotation.from_euler('xyz', start[3:])
    increment = (Rotation.from_euler('xyz', finish[3:]) * rotation.inv()).as_rotvec()
    pose = np.r_[start[:3] + fraction * (finish[:3] - start[:3]),
        (Rotation.from_rotvec(fraction * increment) * rotation).as_euler('xyz')]
    return pose, np.r_[finish[:3] - start[:3], increment]


def work_integral(evaluate, vertices, order):
    nodes, weights = leggauss(order)
    result = None
    for start, finish in zip(vertices[:-1], vertices[1:]):
        for fraction, weight in zip((nodes + 1) / 2, weights / 2):
            pose, tangent = rigid_segment(start, finish, fraction)
            contribution = weight * (tangent @ evaluate(pose))
            result = contribution if result is None else result + contribution
    return result


def verify(cache_folder, coarse_folder, library, output):
    if output.exists():
        raise ValueError('Preserve existing coil coupling verification evidence')
    candidate_report = json.loads((cache_folder / 'summary.json').read_text())
    selected = min((row for row in candidate_report['candidates'] if row['passed']), key=lambda row: row['median_matrix_s']['candidate'])
    candidate = DiagnosticCoilCache(cache_folder, library, selected['order'], True)
    coarse = DiagnosticCoilCache(coarse_folder, library, selected['order'], False)
    initial_path = next(Path(name) for name in candidate_report['input_sha256'] if name.endswith('initial_checkpoint.json'))
    initial = json.loads(initial_path.read_text())
    layout, array = recorded_force_geometry(initial)
    _, refined_array = recorded_force_geometry(initial, max(Fixed.current_demand_mesh_refinements))
    queries = coil_field.CoilSurfaceQueries(array, min(Fixed.coupling_flux_orders))
    geometry = levitation_sim.G
    energy_budget = geometry.weight * (geometry.max_flight_gap - geometry.gap) * Fixed.adaptive_energy_relative_tolerance
    surface_path = next(Path(name) for name in candidate_report['input_sha256'] if name.endswith('summary.json'))
    surface = json.loads(surface_path.read_text())
    start = np.asarray(surface['probes'][0]['pose'])
    target = np.r_[surface['probes'][-1]['pose'][:3], np.radians(Fixed.motion_check_orientation_deg)]
    finish = start + Fixed.coupling_excitation_fraction * (target - start)
    translated = np.r_[finish[:3], start[3:]]
    rotated = np.r_[start[:3], finish[3:]]
    paths = dict(combined=np.array([start, finish]), closed=np.array([start, translated, finish, rotated, start]))
    report = dict(status='running', scope='Bounded force-map derivatives and work/flux consistency; no coupled flight qualification',
        source_sha256=dict(candidate_report['source_sha256'], **source_fingerprints(__file__)),
        input_sha256=source_fingerprints(cache_folder / 'summary.json', coarse_folder / 'summary.json', surface_path, initial_path),
        candidate=candidate.identity, coarse=coarse.identity,
        limits=dict(derivative_relative=Fixed.atlas_gradient_relative_tolerance, energy_J=energy_budget),
        supported_domain='Every requested coil-local field sample must lie inside the saved exterior box; extrapolation is forbidden.',
        derivative_checks=[], work_checks={}, independent_flux={})
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in report['source_sha256']:
            archive.write(name, name)
    started = perf_counter()

    def save():
        report['elapsed_s'] = perf_counter() - started
        airborne_verification.save_summary(output / 'summary.json', report)

    def matrix(pose, evaluator):
        _, center = levitation_sim.place_piece(layout, *pose)
        return (array.force_quadrature.matrix(layout.collection, center) if evaluator is None
            else queries.matrix(layout.collection, center, evaluator))

    def metrics(actual, expected):
        return {name: float(np.max(np.max(np.linalg.norm((actual - expected)[section], axis=0), axis=0)
            / np.max(np.linalg.norm(expected[section], axis=0), axis=0)))
            for name, section in (('force', slice(0, 3)), ('torque', slice(3, 6)))}

    with ExitStack() as stack:
        stack.enter_context(threadpool_limits(limits=Fixed.native_spline_blas_threads, user_api='blas'))
        for cache in (candidate, coarse):
            cache.evaluate = stack.enter_context(magnetic_spline.spline_workers(cache.native, Fixed.native_spline_workers))
        steps = np.array([Fixed.hall_position_derivative_step] * 3 + [Fixed.angle_derivative_step] * 3)
        for pose in (start, rigid_segment(start, finish, Fixed.coupling_excitation_fraction)[0], finish):
            derivatives = {}
            for name, evaluator in (('direct', None), ('candidate', candidate), ('coarse', coarse)):
                derivatives[name] = [levitation_sim.centered_jacobian(lambda value: matrix(value, evaluator), pose, steps / refinement)
                    for refinement in (1, Fixed.qualification_refinement_factor)]
            reference = derivatives['direct'][-1]
            errors = dict(reference_step_refinement=metrics(derivatives['direct'][0], reference),
                candidate_step_refinement=metrics(*derivatives['candidate']),
                candidate_error=metrics(derivatives['candidate'][-1], reference),
                coarse_error=metrics(derivatives['coarse'][-1], reference))
            passed = all(value <= Fixed.atlas_gradient_relative_tolerance for name, values in errors.items()
                if name != 'coarse_error' for value in values.values())
            passed = passed and all(errors['candidate_error'][key] <= errors['coarse_error'][key] for key in ('force', 'torque'))
            report['derivative_checks'].append(dict(pose=pose.tolist(), errors=errors, passed=passed,
                cache_refinement_improves=all(errors['candidate_error'][key] <= errors['coarse_error'][key] for key in ('force', 'torque'))))
            save()
            if not passed:
                report['status'] = 'derivative_failed'
                save()
                return report
        work_values = {}
        for name, vertices in paths.items():
            work_values[name] = {method: [work_integral(lambda pose: matrix(pose, evaluator), vertices, order)
                for order in Fixed.coupling_flux_orders[:2]]
                for method, evaluator in (('direct', None), ('candidate', candidate), ('coarse', coarse))}
            values = work_values[name]
            discrepancies = {method + '_integration_J': float(Fixed.driver_channel_current * np.sum(np.abs(pair[0] - pair[-1])))
                for method, pair in values.items()}
            discrepancies.update({method + '_error_J': float(Fixed.driver_channel_current * np.sum(np.abs(values[method][-1] - values['direct'][-1])))
                for method in ('candidate', 'coarse')})
            if name == 'closed':
                discrepancies.update({method + '_closure_J': float(Fixed.driver_channel_current * np.sum(np.abs(pair[-1])))
                    for method, pair in values.items()})
            passed = all(value <= energy_budget for key, value in discrepancies.items() if not key.startswith('coarse'))
            passed = passed and discrepancies['candidate_error_J'] <= discrepancies['coarse_error_J']
            report['work_checks'][name] = dict(vertices=vertices.tolist(), work_Wb={method: [row.tolist() for row in pair]
                for method, pair in values.items()}, discrepancies=discrepancies, passed=passed,
                cache_refinement_improves=discrepancies['candidate_error_J'] <= discrepancies['coarse_error_J'])
            save()
            if not passed:
                report['status'] = 'work_failed'
                save()
                return report
        index = int(np.argmax(np.abs(work_values['combined']['direct'][-1])))
        quadrature = refined_array.force_quadrature
        first, last = quadrature.filament_bounds[index:index + 2]
        wire = refined_array.coils[index][0]
        anchor = wire.orientation.apply(wire.vertices[0]) + wire.position
        flux = []
        for order in Fixed.coupling_flux_orders[-2:]:
            endpoints = []
            for pose in (start, finish):
                levitation_sim.place_piece(layout, *pose)
                endpoints.append(coupling_verification.curved_surface_flux(layout.collection,
                    quadrature.points[first:last], quadrature.vectors[first:last], anchor, order))
            flux.append(endpoints)
        differences = np.diff(np.asarray(flux), axis=1).ravel()
        errors = dict(flux_refinement_J=float(Fixed.driver_channel_current * abs(differences[-1] - differences[0])))
        errors.update({method + '_flux_error_J': float(Fixed.driver_channel_current * abs(pair[-1][index] - differences[-1]))
            for method, pair in work_values['combined'].items()})
        passed = all(value <= energy_budget for key, value in errors.items() if not key.startswith('coarse'))
        report['independent_flux'] = dict(coil_identity=refined_array.identities[index], endpoint_flux_Wb=flux,
            errors=errors, passed=passed, formulation='Direct magnet-field flux through the curved winding surface, independent of surface-force assembly and coil-field interpolation.')
        report['status'] = 'passed' if passed else 'flux_failed'
        save()
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Verify diagnostic coil-field force maps before coupled use.')
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--coarse', type=Path, required=True)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    result = verify(arguments.cache, arguments.coarse, arguments.library, arguments.output)
    print(json.dumps(dict(status=result['status'], elapsed_s=result['elapsed_s'])))

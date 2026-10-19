import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
from time import perf_counter
from zipfile import ZipFile, ZIP_DEFLATED

import numpy as np
from threadpoolctl import threadpool_limits

import levitation_sim
from model import Fixed, Inputs
from methodology import airborne_verification, coupling_verification, magnetic_atlas, magnetic_spline, numerical_accuracy, physics_verification, source_fingerprints
from methodology.benchmark_magnetics import recorded_force_geometry
from methodology.cad import magnetics


class CoilSurfaceQueries:
    def __init__(self, array, order):
        self.order = order
        self.rotations = [coil[0].orientation * array.rotation for coil in array.coils]
        self.origins = np.array([coil[0].orientation.apply(array.rotation.apply(
            [placement['x'], placement['y'], placement['top'] - array.height])) + coil[0].position
            for placement, coil in zip(array.placements, array.coils)])
        if array.force_quadrature is None or not array.force_quadrature.matches(array.coils):
            array.force_quadrature = array.create_force_quadrature()
        quadrature = array.force_quadrature
        if any(len(coil) != 1 for coil in array.coils) or not np.all(quadrature.currents == 1):
            raise ValueError('Coil field reuse requires single unit-current CAD windings')
        start, end = quadrature.filament_bounds[:2]
        self.positions = self.rotations[0].inv().apply(quadrature.points[start:end] - self.origins[0])
        self.elements = self.rotations[0].inv().apply(quadrature.vectors[start:end])
        for index, (first, last) in enumerate(zip(quadrature.filament_bounds[:-1], quadrature.filament_bounds[1:])):
            positions = self.rotations[index].inv().apply(quadrature.points[first:last] - self.origins[index])
            elements = self.rotations[index].inv().apply(quadrature.vectors[first:last])
            if (positions.shape != self.positions.shape or not np.allclose(positions, self.positions,
                    atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)
                    or not np.allclose(elements, self.elements, atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)):
                raise ValueError('Coil field reuse requires identical transformed CAD wire quadratures')

    def local_queries(self, points):
        return np.concatenate([rotation.inv().apply(points - origin) for rotation, origin in zip(self.rotations, self.origins)])

    def direct(self, points):
        with ThreadPoolExecutor(max_workers=Fixed.force_field_workers) as executor:
            return np.concatenate(list(executor.map(lambda batch: numerical_accuracy.sampled_conductor_field(
                batch, self.positions, self.elements), np.array_split(points, Fixed.force_field_workers))))

    def matrix(self, source, center, evaluate):
        points, charges = coupling_verification.magnet_surface_quadrature(source, self.order)
        fields = evaluate(self.local_queries(points)).reshape(len(self.origins), len(points), 3)
        return np.asarray([coupling_verification.magnet_surface_wrench(points, charges, center, field @ rotation.as_matrix().T)
            for rotation, field in zip(self.rotations, fields)]).T


def coil_query_bounds(queries, layout, poses):
    corners = np.array(np.meshgrid(*[[-1, 1]] * 3)).reshape(3, -1).T
    bounds = np.array([[np.inf] * 3, [-np.inf] * 3])
    for pose in poses:
        levitation_sim.place_piece(layout, *pose)
        points = np.concatenate([magnet.orientation.apply(corners * magnet.dimension / 2) + magnet.position
            for magnet in layout.collection.sources_all])
        local = queries.local_queries(points)
        bounds[0] = np.minimum(bounds[0], local.min(axis=0))
        bounds[1] = np.maximum(bounds[1], local.max(axis=0))
    return bounds


def coil_grid_axes(queries, bounds, spacing):
    bounds = np.asarray(bounds, dtype=float)
    if bounds.shape != (2, 3) or not np.isfinite(bounds).all() or np.any(bounds[1] <= bounds[0]):
        raise ValueError('Coil cache requires finite increasing three-dimensional bounds')
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError('Coil cache spacing must be positive')
    lower = np.floor(bounds[0] / spacing).astype(int) - Fixed.atlas_padding_cells
    upper = np.ceil(bounds[1] / spacing).astype(int) + Fixed.atlas_padding_cells
    lower[2] = max(lower[2], int(np.floor(queries.positions[:, 2].max() / spacing + .5)))
    axes = [(np.arange(first, last + 1) + .5) * spacing for first, last in zip(lower, upper)]
    if any(not len(axis) or axis[0] > bounds[0, index] or axis[-1] < bounds[1, index] for index, axis in enumerate(axes)):
        raise ValueError('Coil field lattice does not cover its sampled exterior domain')
    storage = int(np.prod([len(axis) for axis in axes])) * 3 * np.dtype(float).itemsize
    if 3 * storage > Fixed.atlas_max_storage_bytes:
        raise ValueError('Coil grid and coefficients exceed the configured memory budget')
    return axes


def sample_coil_grid(queries, axes):
    values = np.empty((*[len(axis) for axis in axes], 3))
    yz = np.stack(np.meshgrid(axes[1], axes[2], indexing='ij'), axis=-1).reshape(-1, 2)
    for index, x in enumerate(axes[0]):
        points = np.column_stack((np.full(len(yz), x), yz))
        values[index] = queries.direct(points).reshape(len(axes[1]), len(axes[2]), 3)
    return values


class CoilFieldCache(magnetic_atlas.MagneticFieldAtlas):
    cache_schema = 'coil-field-cache-v1'

    @classmethod
    def current_provenance(cls):
        provenance = super().current_provenance()
        provenance['source_sha256'].update(magnetics.fingerprints())
        provenance['source_sha256'].update(source_fingerprints(__file__, coupling_verification.__file__, numerical_accuracy.__file__))
        return provenance

    def __init__(self, array, bounds):
        queries = CoilSurfaceQueries(array, min(Fixed.coupling_flux_orders))
        self.bounds = np.asarray(bounds, dtype=float)
        self.source_state = np.column_stack((queries.positions, queries.elements))
        axes = coil_grid_axes(queries, self.bounds, Fixed.coil_cache_grid_spacing)
        values = sample_coil_grid(queries, axes)
        self.interpolator = magnetic_atlas.tensor_product_spline(axes, values, Fixed.coil_cache_spline_order)
        self.coefficients = self.interpolator.c
        self.shape = self.coefficients.shape[:3]
        self.provenance = self.current_provenance()
        self.identity = self.content_identity()

    def field(self, source, points):
        raise ValueError('Coil cache supplies actuator matrices, not moving-magnet fields')

    def __call__(self, points):
        points = np.asarray(points)
        if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
                or np.any(points < self.bounds[0]) or np.any(points > self.bounds[1])):
            raise ValueError('Coil field query is outside its declared exterior domain')
        return self.interpolator(points)

    def bind(self, array):
        queries = CoilSurfaceQueries(array, min(Fixed.coupling_flux_orders))
        state = np.column_stack((queries.positions, queries.elements))
        if (state.shape != self.source_state.shape or not np.allclose(state, self.source_state,
                atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance)):
            raise ValueError('Coil cache winding geometry or quadrature differs')
        quadrature = array.force_quadrature
        def matrix(source, center):
            if array.force_quadrature is not quadrature or not quadrature.matches(array.coils):
                raise ValueError('Coil cache bound window or winding geometry changed')
            return queries.matrix(source, center, self)
        return matrix


def prepare_cache(output):
    if output.exists():
        raise ValueError('Preserve existing coil-cache preparation')
    construction = magnetics.ConstructionModel()
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    layout = levitation_sim.magnet_layout_from_geometry()
    queries = CoilSurfaceQueries(array, min(Fixed.coupling_flux_orders))
    poses = physics_verification.verification_poses(geometry, array, layout)
    bounds = coil_query_bounds(queries, layout, [row['pose'] for row in poses])
    output.mkdir(parents=True)
    started = perf_counter()
    cache = CoilFieldCache(array, bounds)
    cache.save(output / 'atlas.npz')
    source = cache.provenance['source_sha256']
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in source:
            archive.write(name, name)
    points = np.random.default_rng(Fixed.atlas_validation_seed).uniform(*bounds, size=(Fixed.atlas_validation_points, 3))
    field_error = magnetic_atlas.vector_error(cache(points), queries.direct(points))
    report = dict(status='field_screen_passed' if field_error <= Fixed.atlas_field_tolerance else 'field_screen_failed',
        scope='Fresh production coil cache with off-grid field check; not new motion qualification',
        source_sha256=source, atlas_identity=cache.identity, bounds_m=bounds.tolist(),
        field_error_T=field_error, field_limit_T=Fixed.atlas_field_tolerance, elapsed_s=perf_counter() - started)
    airborne_verification.save_summary(output / 'summary.json', report)
    return report


def screen(surface_folder, native_folder, output, spacing):
    if output.exists():
        raise ValueError('Preserve existing coil-field cache evidence')
    surface = json.loads((surface_folder / 'summary.json').read_text())
    if surface['status'] != 'passed':
        raise ValueError('Coil cache requires passing direct surface evidence')
    physical_sources = dict(magnetics.fingerprints(), **source_fingerprints(coupling_verification.__file__, numerical_accuracy.__file__))
    if any(surface['source_sha256'][name] != digest for name, digest in physical_sources.items()):
        raise ValueError('Direct surface geometry or integration source changed')
    checkpoint_path = next(Path(name) for name in surface['input_sha256'] if name.endswith('initial_checkpoint.json'))
    for name, digest in surface['input_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise ValueError('Surface evidence input changed: ' + name)
    layout, array = recorded_force_geometry(json.loads(checkpoint_path.read_text()))
    queries = CoilSurfaceQueries(array, surface['surface_order'])
    domain = physics_verification.verification_poses(levitation_sim.G, array, layout)
    bounds = coil_query_bounds(queries, layout, [row['pose'] for row in domain] + [row['pose'] for row in surface['probes']])
    axes = coil_grid_axes(queries, bounds, spacing)
    shape = tuple(len(axis) for axis in axes)
    storage = int(np.prod(shape)) * 3 * np.dtype(float).itemsize
    report = dict(scope='Sampled coil-field cache accuracy and actuator-matrix timing; no coupled or full-domain qualification',
        status='running', spacing_m=spacing, grid_shape=shape, grid_bytes=storage, bounds_m=bounds.tolist(),
        source_sha256=dict(physical_sources, **source_fingerprints(__file__, magnetic_atlas.__file__, magnetic_spline.__file__,
            Path(magnetic_spline.__file__).with_suffix('.c'), physics_verification.__file__,
            Path(__file__).with_name('benchmark_magnetics.py'))),
        input_sha256=source_fingerprints(surface_folder / 'summary.json', checkpoint_path, native_folder / 'atlas.npz'),
        limits=dict(field_T=Fixed.atlas_field_tolerance, gradient_relative=Fixed.atlas_gradient_relative_tolerance,
            matrix_relative=Fixed.atlas_matrix_relative_tolerance, matrix_speedup=Fixed.atlas_minimum_speedup),
        candidates=[])
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in report['source_sha256']:
            archive.write(name, name)

    def save():
        airborne_verification.save_summary(output / 'summary.json', report)

    save()
    started = perf_counter()
    values = sample_coil_grid(queries, axes)
    report['grid_build_s'] = perf_counter() - started
    random = np.random.default_rng(Fixed.atlas_validation_seed)
    probes = random.uniform(*bounds, size=(Fixed.atlas_validation_points, 3))
    direct_field = queries.direct(probes)
    direct_gradient = magnetic_atlas.field_jacobian(lambda source, points: queries.direct(points), None, probes)
    levitation_sim.place_piece(layout)
    with np.load(native_folder / 'atlas.npz', allow_pickle=False) as saved:
        reference_bounds = saved['bounds']
    for row in surface['probes']:
        levitation_sim.place_piece(layout, *row['pose'])
        source = layout.collection
        local = source.orientation.inv().apply(array.force_quadrature.points - source.position)
        reference_bounds[0] = np.minimum(reference_bounds[0], local.min(axis=0))
        reference_bounds[1] = np.maximum(reference_bounds[1], local.max(axis=0))
    levitation_sim.place_piece(layout)
    started = perf_counter()
    reference = magnetic_atlas.MagneticFieldAtlas(layout.collection, reference_bounds,
        min(Fixed.atlas_grid_spacings), min(Fixed.atlas_spline_orders))
    report['reference_build_s'] = perf_counter() - started
    report['reference_identity'] = reference.identity
    for order in Fixed.atlas_spline_orders:
        started = perf_counter()
        spline = magnetic_atlas.tensor_product_spline(axes, values, order)
        native = magnetic_spline.TensorSplineEvaluator(spline, native_folder / 'native/magnetic_spline.so')
        case = dict(order=order, coefficients_s=perf_counter() - started, native_build=native.build, poses=[], checks={})
        report['candidates'].append(case)
        with threadpool_limits(limits=Fixed.native_spline_blas_threads, user_api='blas'), \
                magnetic_spline.spline_workers(native, Fixed.native_spline_workers) as evaluate, \
                magnetic_spline.native_evaluation(reference, native_folder / 'native/magnetic_spline.so') as runtime:
            case['reference_runtime'] = runtime
            field = evaluate(probes)
            gradient = magnetic_atlas.field_jacobian(lambda source, points: evaluate(points), None, probes)
            case['field_error_T'] = magnetic_atlas.vector_error(field, direct_field)
            case['gradient_relative_error'] = float(np.max(np.linalg.norm(gradient - direct_gradient, axis=1))
                / np.max(np.linalg.norm(direct_gradient, axis=1)))
            case['checks'].update(field=case['field_error_T'] <= Fixed.atlas_field_tolerance,
                gradient=case['gradient_relative_error'] <= Fixed.atlas_gradient_relative_tolerance)
            for row in surface['probes']:
                _, center = levitation_sim.place_piece(layout, *row['pose'])
                timings = dict(reference=[], candidate=[])
                for _ in range(Fixed.qualification_refinement_factor):
                    started = perf_counter()
                    baseline = levitation_sim.actuator_matrix(layout, array, center, field_evaluator=reference.field)
                    timings['reference'].append(perf_counter() - started)
                    started = perf_counter()
                    candidate = queries.matrix(layout.collection, center, evaluate)
                    timings['candidate'].append(perf_counter() - started)
                direct = np.asarray(row['reference_matrix'])
                errors = {name: physics_verification.matrix_refinement_metrics(matrix, direct, Fixed.driver_channel_current)
                    for name, matrix in (('reference', baseline), ('candidate', candidate))}
                passed = all(metrics[key] <= Fixed.atlas_matrix_relative_tolerance for metrics in errors.values()
                    for key in ('force_relative_column_error', 'torque_relative_column_error'))
                case['poses'].append(dict(kind=row['kind'], pose=row['pose'], timings_s=timings, errors=errors,
                    candidate_matrix=candidate.tolist(), reference_matrix=baseline.tolist(), passed=passed))
            case['checks']['matrices'] = all(row['passed'] for row in case['poses'])
        case['median_matrix_s'] = {name: float(np.median([elapsed for row in case['poses'] for elapsed in row['timings_s'][name]]))
            for name in ('reference', 'candidate')}
        case['matrix_speedup'] = case['median_matrix_s']['reference'] / case['median_matrix_s']['candidate']
        case['checks']['speed'] = case['matrix_speedup'] >= Fixed.atlas_minimum_speedup
        case['passed'] = all(case['checks'].values())
        cache_folder = output / f'order_{order}'
        cache_folder.mkdir()
        np.savez(cache_folder / 'atlas.npz', bounds=bounds, coefficients=spline.c, orders=spline.k,
            **{f'knots_{axis}': knots for axis, knots in enumerate(spline.t)},
            metadata=json.dumps(dict(schema='coil-field-screen-v1', source_sha256=report['source_sha256'],
                scope='Diagnostic coefficient cache; production checkpoint use is forbidden')))
        case['cache_sha256'] = hashlib.sha256((cache_folder / 'atlas.npz').read_bytes()).hexdigest()
        save()
    report['status'] = 'passed' if any(case['passed'] for case in report['candidates']) else 'screen_failed'
    save()
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Prepare production coil-field caches or run bounded diagnostic screens.')
    parser.add_argument('--prepare', action='store_true', help='Build a fresh source-checked cache for the normal simulator.')
    parser.add_argument('--surface', type=Path)
    parser.add_argument('--native-reference', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--spacing', type=float, choices=Fixed.atlas_grid_spacings, default=max(Fixed.atlas_grid_spacings))
    arguments = parser.parse_args()
    if arguments.prepare:
        if arguments.surface is not None or arguments.native_reference is not None:
            parser.error('Production preparation does not accept diagnostic inputs')
        result = prepare_cache(arguments.output)
        print(json.dumps(result))
    else:
        if arguments.surface is None or arguments.native_reference is None:
            parser.error('The diagnostic screen requires surface and native reference evidence')
        result = screen(arguments.surface, arguments.native_reference, arguments.output, arguments.spacing)
        print(json.dumps(dict(status=result['status'], grid_build_s=result['grid_build_s'],
            candidates=[dict(order=row['order'], speedup=row['matrix_speedup'], checks=row['checks']) for row in result['candidates']])))

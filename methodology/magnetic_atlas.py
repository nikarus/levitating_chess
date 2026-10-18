import argparse
from datetime import datetime
import hashlib
import json
import platform

import magpylib
import scipy
from pathlib import Path
from time import perf_counter, time

import numpy as np
from scipy.interpolate import NdBSpline, make_interp_spline
from scipy.spatial.transform import Rotation

import levitation_sim
from model import Fixed, Inputs
from methodology import adaptive_airborne, airborne, airborne_verification, sensing, source_fingerprints
from methodology.cad import magnetics


def local_source_state(source):
    inverse = source.orientation.inv()
    return np.array([np.r_[magnet.dimension, magnet.polarization,
        inverse.apply(magnet.position - source.position),
        (inverse * magnet.orientation).as_matrix().ravel()] for magnet in source.children])


class MagneticFieldAtlas:
    evaluator_identity = dict(method='scipy')
    evaluator_sources = {}

    def __init__(self, source, bounds, spacing, order, axes=None, values=None):
        self.source_state = local_source_state(source)
        self.bounds = np.asarray(bounds, dtype=float)
        if self.bounds.shape != (2, 3) or not np.isfinite(self.bounds).all() or np.any(self.bounds[1] <= self.bounds[0]):
            raise ValueError('Atlas requires finite increasing three-dimensional bounds')
        if spacing <= 0 or not np.isfinite(spacing) or order not in Fixed.atlas_spline_orders:
            raise ValueError('Invalid atlas spacing or spline order')
        if axes is None:
            axes, values = self.sample_grid(source, self.bounds, spacing)
        self.shape = tuple(len(axis) for axis in axes)
        coefficients = values
        knots = []
        for axis, coordinates in enumerate(axes):
            spline = make_interp_spline(coordinates, coefficients, k=order, axis=axis)
            coefficients = np.moveaxis(spline.c, 0, axis)
            knots.append(spline.t)
        self.interpolator = NdBSpline(tuple(knots), coefficients, order, extrapolate=False)
        self.coefficients = self.interpolator.c
        self.provenance = dict(source_sha256=source_fingerprints(__file__, levitation_sim.__file__, Path(levitation_sim.__file__).with_name('model.py')),
            numpy=np.__version__, scipy=scipy.__version__, magpylib=magpylib.__version__)
        self.identity = self.content_identity()

    def content_identity(self):
        digest = hashlib.sha256(json.dumps(self.provenance, sort_keys=True).encode())
        for value in (self.bounds, self.source_state, self.interpolator.k, *self.interpolator.t, self.coefficients):
            array = np.ascontiguousarray(value)
            digest.update(str((array.shape, array.dtype.str)).encode())
            digest.update(memoryview(array).cast('B'))
        return digest.hexdigest()

    def save(self, path):
        if path.exists():
            raise ValueError('Atlas cache already exists; preserve its evidence')
        temporary = path.with_suffix('.tmp')
        with temporary.open('wb') as stream:
            np.savez(stream, bounds=self.bounds, source_state=self.source_state, coefficients=self.coefficients,
                orders=self.interpolator.k, **{f'knots_{axis}': knots for axis, knots in enumerate(self.interpolator.t)},
                metadata=json.dumps(dict(schema='magnetic-atlas-v1', identity=self.identity, provenance=self.provenance)))
        temporary.replace(path)

    @classmethod
    def load(cls, path):
        atlas = cls.__new__(cls)
        with np.load(path, allow_pickle=False) as saved:
            metadata = json.loads(str(saved['metadata']))
            if metadata['schema'] != 'magnetic-atlas-v1':
                raise ValueError('Atlas cache schema differs')
            atlas.provenance = metadata['provenance']
            for name in ('bounds', 'source_state', 'coefficients'):
                setattr(atlas, name, saved[name])
            atlas.interpolator = NdBSpline(tuple(saved[f'knots_{axis}'] for axis in range(3)),
                atlas.coefficients, saved['orders'], extrapolate=False)
        atlas.shape = atlas.coefficients.shape[:3]
        atlas.identity = atlas.content_identity()
        if atlas.identity != metadata['identity']:
            raise ValueError('Atlas cache content digest differs')
        if (atlas.provenance['source_sha256'] != source_fingerprints(__file__, levitation_sim.__file__, Path(levitation_sim.__file__).with_name('model.py'))
                or atlas.provenance['numpy'] != np.__version__ or atlas.provenance['scipy'] != scipy.__version__
                or atlas.provenance['magpylib'] != magpylib.__version__):
            raise ValueError('Atlas cache source or dependencies differ')
        return atlas

    @staticmethod
    def sample_grid(source, bounds, spacing):
        lower = np.floor(bounds[0] / spacing).astype(int) - Fixed.atlas_padding_cells
        upper = np.ceil(bounds[1] / spacing).astype(int) + Fixed.atlas_padding_cells
        corners = np.array(np.meshgrid(*[[-1, 1]] * 3)).reshape(3, -1).T
        inverse = source.orientation.inv()
        bottom = min(float(np.min(inverse.apply(magnet.orientation.apply(corners * magnet.dimension / 2)
            + magnet.position - source.position)[:, 2])) for magnet in source.children)
        if bounds[1, 2] >= bottom:
            raise ValueError('Atlas domain must remain below the canonical magnet assembly')
        upper[2] = min(upper[2], int(np.floor(bottom / spacing - 0.5)))
        axes = [(np.arange(first, last + 1) + 0.5) * spacing for first, last in zip(lower, upper)]
        if any(axis[0] > bounds[0, index] or axis[-1] < bounds[1, index] for index, axis in enumerate(axes)):
            raise ValueError('Atlas lattice does not cover its exterior query domain')
        shape = tuple(len(axis) for axis in axes)
        storage = int(np.prod(shape)) * 3 * np.dtype(float).itemsize
        if 3 * storage > Fixed.atlas_max_storage_bytes:
            raise ValueError('Atlas grid and coefficients exceed the configured memory budget')
        values = np.empty((*shape, 3))
        canonical = source.copy()
        canonical.position = (0, 0, 0)
        canonical.orientation = Rotation.identity()
        yz = np.stack(np.meshgrid(axes[1], axes[2], indexing='ij'), axis=-1).reshape(-1, 2)
        for index, x in enumerate(axes[0]):
            points = np.column_stack((np.full(len(yz), x), yz))
            values[index] = levitation_sim.magnetic_field(canonical, points).reshape(*shape[1:], 3)
        if not np.isfinite(values).all():
            raise ValueError('Direct atlas samples are nonfinite')
        return axes, values

    def field(self, source, points):
        state = local_source_state(source)
        if state.shape != self.source_state.shape or not np.allclose(state, self.source_state,
                atol=Fixed.singular_value_tolerance, rtol=Fixed.matrix_rank_tolerance):
            raise ValueError('Atlas magnet geometry or magnetization differs from the query source')
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('Atlas query requires finite three-dimensional points')
        local = source.orientation.inv().apply(points - source.position)
        if np.any(local < self.bounds[0]) or np.any(local > self.bounds[1]):
            raise ValueError('Atlas query is outside its declared domain')
        fields = self.interpolator(local)
        if not np.isfinite(fields).all():
            raise ValueError('Atlas interpolation returned nonfinite fields')
        return source.orientation.apply(fields)


class MagneticBackend:
    def configure_atlas(self, atlas, mode):
        if mode not in ('direct', 'atlas_controller', 'atlas_forces'):
            raise ValueError('Unknown magnetic backend')
        if mode != 'direct' and atlas is None:
            raise ValueError('Accelerated simulation requires an explicit atlas')
        self.atlas_configuration = dict(mode=mode, identity=None if mode == 'direct' else atlas.identity,
            evaluator=None if mode == 'direct' else atlas.evaluator_identity)
        def evaluate(source, points):
            if self.atlas_configuration['evaluator'] != atlas.evaluator_identity:
                raise ValueError('Active atlas evaluator differs from the configured simulation runtime')
            return atlas.field(source, points)
        self.controller.field_evaluator = None if mode == 'direct' else evaluate
        self.plant.field_evaluator = evaluate if mode == 'atlas_forces' else None
        self.source.update(source_fingerprints(__file__))
        if mode != 'direct':
            self.source.update(atlas.evaluator_sources)

    def checkpoint_state(self):
        state = super().checkpoint_state()
        state['atlas'] = self.atlas_configuration
        return state

    def validate_checkpoint(self, state):
        super().validate_checkpoint(state)
        self.validate_atlas(state)

    def validate_atlas(self, state):
        if state['atlas'] != self.atlas_configuration:
            raise ValueError('Checkpoint magnetic backend or atlas content differs')

    def report(self):
        report = super().report()
        report['atlas'] = self.atlas_configuration
        report['limitations'].append('Atlas force interpolation requires independent direct-field validation; Hall sensing and estimation remain direct.')
        return report


class AtlasAirborneSimulation(MagneticBackend, adaptive_airborne.AdaptiveAirborneSimulation):
    checkpoint_schema = 'airborne-atlas-v1'

    def __init__(self, construction, atlas, mode, duration, **settings):
        super().__init__(construction, duration, **settings)
        self.configure_atlas(atlas, mode)


def validation_poses(geometry, array, layout):
    random = np.random.default_rng(Fixed.atlas_validation_seed)
    poses = [np.zeros(6)]
    for yaw in Fixed.pose_yaws_deg:
        for fraction in Fixed.pose_tilt_fractions:
            tilt = levitation_sim.rim_tilt_limit(layout, geometry.max_flight_gap) * fraction / np.sqrt(2)
            phase = max(Fixed.pose_phase_fractions) * array.short_pitch
            poses.append(np.array([phase, -phase, geometry.max_flight_gap - geometry.gap, tilt, -tilt, np.radians(yaw)]))
    for _ in range(Fixed.atlas_validation_poses):
        gap = random.uniform(geometry.gap, geometry.max_flight_gap)
        direction = random.uniform(-np.pi, np.pi)
        tilt = random.uniform(0, levitation_sim.rim_tilt_limit(layout, gap))
        poses.append(np.r_[random.uniform(-array.short_pitch / 2, array.short_pitch / 2, 2), gap - geometry.gap,
                           tilt * np.cos(direction), tilt * np.sin(direction), random.uniform(-np.pi, np.pi)])
    return poses


def vector_error(actual, expected):
    return float(np.max(np.linalg.norm(actual - expected, axis=-1)))


def field_jacobian(evaluator, source, points):
    step = Fixed.hall_position_derivative_step
    return np.stack([(evaluator(source, points + axis * step) - evaluator(source, points - axis * step)) / (2 * step)
                     for axis in np.eye(3)], axis=-1)


def benchmark(output, deadline, finest_only=False, fields_only=False):
    if output.exists():
        raise ValueError('Atlas benchmark output exists; preserve previous evidence')
    output.mkdir(parents=True)
    report = dict(scope='Sampled magnetic atlas accuracy and cost, followed by gated short coupled comparisons; not flight qualification',
        status='running', source_sha256=dict(magnetics.fingerprints(), **source_fingerprints(__file__)),
        interpolation='Separable tensor B-spline, not-a-knot boundaries, exterior samples only',
        environment=dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__, magpylib=magpylib.__version__),
        candidates=[], coupled={})

    def save():
        airborne_verification.save_summary(output / 'summary.json', report)

    save()
    construction = magnetics.ConstructionModel()
    geometry, array, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    layout = levitation_sim.magnet_layout_from_geometry()
    hall = sensing.HallFieldModel(construction)
    quadrature = array.create_force_quadrature()
    array.force_quadrature = quadrature
    poses = validation_poses(geometry, array, layout)
    query_points = np.vstack((quadrature.points, hall.points))
    bounds = np.array([[np.inf] * 3, [-np.inf] * 3])
    references = []
    for pose in poses:
        _, center = levitation_sim.place_piece(layout, *pose)
        source = layout.collection
        local = source.orientation.inv().apply(query_points - source.position)
        bounds[0] = np.minimum(bounds[0], local.min(axis=0))
        bounds[1] = np.maximum(bounds[1], local.max(axis=0))
        started = perf_counter()
        fields = levitation_sim.magnetic_field(source, query_points)
        matrix = quadrature.matrix(source, center, lambda sources, points: fields[:len(points)])
        elapsed = perf_counter() - started
        jacobian = field_jacobian(levitation_sim.magnetic_field, source, hall.points)
        references.append(dict(pose=pose, center=center, fields=fields, matrix=matrix, elapsed_s=elapsed, jacobian=jacobian))
    bounds += np.array([-1, 1])[:, None] * Fixed.hall_position_derivative_step * 2
    levitation_sim.place_piece(layout)
    random = np.random.default_rng(Fixed.atlas_validation_seed)
    probes = []
    for reference in references:
        levitation_sim.place_piece(layout, *reference['pose'])
        points = query_points[random.integers(len(query_points), size=Fixed.atlas_validation_points)]
        local = layout.collection.orientation.inv().apply(points - layout.collection.position)
        probes.append(local)
    probes = np.vstack(probes)
    for axis in range(3):
        for side in range(2):
            point = probes[np.argmin(probes[:, axis]) if side == 0 else np.argmax(probes[:, axis])].copy()
            point[axis] = bounds[side, axis]
            probes = np.vstack((probes, point))
    levitation_sim.place_piece(layout)
    probe_reference = levitation_sim.magnetic_field(layout.collection, probes)
    report.update(bounds_m=bounds.tolist(), poses=[pose.tolist() for pose in poses], wire_points=len(quadrature.points),
        probe_count=len(probes), direct_median_matrix_s=float(np.median([row['elapsed_s'] for row in references])),
        limits=dict(field_T=Fixed.atlas_field_tolerance, matrix_relative=Fixed.atlas_matrix_relative_tolerance,
            gradient_relative=Fixed.atlas_gradient_relative_tolerance, minimum_speedup=Fixed.atlas_minimum_speedup))
    save()
    selected = None
    for spacing in (Fixed.atlas_grid_spacings[-1:] if finest_only else Fixed.atlas_grid_spacings):
        if time() >= deadline:
            report['status'] = 'deadline'
            save()
            return report
        started = perf_counter()
        axes, values = MagneticFieldAtlas.sample_grid(layout.collection, bounds, spacing)
        build_s = perf_counter() - started
        passing = []
        for order in Fixed.atlas_spline_orders:
            started = perf_counter()
            atlas = MagneticFieldAtlas(layout.collection, bounds, spacing, order, axes, values)
            row = dict(spacing_m=spacing, spline_order=order, grid_shape=list(atlas.shape),
                coefficient_bytes=atlas.coefficients.nbytes, grid_build_s=build_s, coefficient_build_s=perf_counter() - started,
                probe_error_T=vector_error(atlas.field(layout.collection, probes), probe_reference), poses=[])
            for reference in references:
                levitation_sim.place_piece(layout, *reference['pose'])
                source = layout.collection
                started = perf_counter()
                fields = atlas.field(source, query_points)
                matrix = quadrature.matrix(source, reference['center'], lambda sources, points: fields[:len(points)])
                elapsed = perf_counter() - started
                expected = reference['matrix']
                errors = [vector_error(matrix[start:start + 3].T, expected[start:start + 3].T)
                          / np.max(np.linalg.norm(expected[start:start + 3], axis=0)) for start in (0, 3)]
                jacobian = field_jacobian(atlas.field, source, hall.points)
                gradient_error = float(np.max(np.linalg.norm(jacobian - reference['jacobian'], axis=(1, 2)))
                    / np.max(np.linalg.norm(reference['jacobian'], axis=(1, 2))))
                row['poses'].append(dict(force_relative=errors[0], torque_relative=errors[1],
                    field_error_T=vector_error(fields, reference['fields']),
                    worst_field_point_m=query_points[np.argmax(np.linalg.norm(fields - reference['fields'], axis=1))].tolist(),
                    gradient_relative=gradient_error, elapsed_s=elapsed))
            levitation_sim.place_piece(layout)
            row['median_matrix_s'] = float(np.median([pose['elapsed_s'] for pose in row['poses']]))
            row['speedup'] = report['direct_median_matrix_s'] / row['median_matrix_s']
            row['checks'] = dict(field=max(row['probe_error_T'], max(pose['field_error_T'] for pose in row['poses'])) <= Fixed.atlas_field_tolerance,
                force=max(pose['force_relative'] for pose in row['poses']) <= Fixed.atlas_matrix_relative_tolerance,
                torque=max(pose['torque_relative'] for pose in row['poses']) <= Fixed.atlas_matrix_relative_tolerance,
                gradient=max(pose['gradient_relative'] for pose in row['poses']) <= Fixed.atlas_gradient_relative_tolerance,
                speed=row['speedup'] >= Fixed.atlas_minimum_speedup)
            row['checks'] = {key: bool(value) for key, value in row['checks'].items()}
            row['passed'] = all(row['checks'].values())
            report['candidates'].append(row)
            save()
            print(json.dumps({key: value for key, value in row.items() if key != 'poses'}), flush=True)
            if row['passed']:
                passing.append((row['median_matrix_s'], atlas, row))
            del atlas
        del values
        if passing:
            _, selected, selected_row = min(passing, key=lambda value: value[0])
            break
    if selected is None:
        report['status'] = 'no_candidate_passed'
        save()
        return report
    report['selected'] = {key: selected_row[key] for key in ('spacing_m', 'spline_order', 'coefficient_bytes', 'speedup')}
    if fields_only:
        selected.save(output / 'atlas.npz')
        report.update(status='field_screen_passed', atlas_identity=selected.identity,
            scope='Sampled current-source field, force, torque, gradient and speed screen; coupled integration and combined error budget not qualified')
        save()
        return report
    save()
    trials = {}
    for name in ('direct', 'atlas_controller', 'atlas_forces'):
        if time() >= deadline:
            report['status'] = 'deadline'
            save()
            return report
        simulation = AtlasAirborneSimulation(construction, selected, name, Fixed.adaptive_benchmark_duration)
        trial = airborne.run_simulation(simulation, output / (name + '.json'), output / (name + '_checkpoint.json'), deadline=deadline)
        trials[name] = trial
        report['coupled'][name] = {key: trial[key] for key in ('passed', 'elapsed_s', 'duration_completed_s', 'stop_reason', 'peaks')}
        if name != 'direct':
            report['coupled'][name]['comparison'] = airborne_verification.compare_trials(trials['direct'], trial)
        save()
        if not trial['passed'] or (name != 'direct' and not report['coupled'][name]['comparison']['converged']):
            report['status'] = 'coupled_failed'
            save()
            return report
    report['status'] = 'completed'
    save()
    return report


def qualify(output, deadline, evidence, resume=False):
    source = dict(magnetics.fingerprints(), **source_fingerprints(__file__, airborne_verification.__file__, adaptive_airborne.__file__, airborne.__file__))
    evidence_identity = hashlib.sha256(evidence.read_bytes()).hexdigest()
    benchmark_report = json.loads(evidence.read_text())
    if benchmark_report['status'] != 'completed':
        raise ValueError('Qualification requires completed atlas feasibility evidence')
    if resume:
        summary = json.loads((output / 'summary.json').read_text())
        if summary['source_sha256'] != source or summary['evidence_sha256'] != evidence_identity:
            raise ValueError('Qualification source or feasibility evidence differs')
        if summary['status'] not in ('running', 'deadline'):
            raise ValueError('Cannot resume completed or failed qualification')
        atlas = MagneticFieldAtlas.load(output / 'atlas.npz')
        construction = airborne_verification.recorded_construction(summary)
    else:
        if output.exists():
            raise ValueError('Qualification output exists; preserve previous evidence')
        output.mkdir(parents=True)
        construction = magnetics.ConstructionModel()
        geometry, _, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
        levitation_sim.use_geometry(geometry)
        layout = levitation_sim.magnet_layout_from_geometry()
        selected = benchmark_report['selected']
        atlas = MagneticFieldAtlas(layout.collection, benchmark_report['bounds_m'], selected['spacing_m'], selected['spline_order'])
        atlas.save(output / 'atlas.npz')
        summary = dict(scope='Bounded atlas/adaptive numerical qualification; not complete flight qualification',
            status='running', source_sha256=source, evidence_sha256=evidence_identity, atlas_identity=atlas.identity,
            selected=selected, cases={}, comparisons={})
    if atlas.identity != summary['atlas_identity']:
        raise ValueError('Qualification atlas identity differs')
    configurations = (
        ('adaptive', 'atlas_forces', {}),
        ('reference', 'atlas_forces', dict(fixed_step=Fixed.adaptive_reference_step)),
        ('tighter', 'atlas_forces', dict(tolerance_scale=1 / Fixed.qualification_refinement_factor)),
        ('direct', 'direct', {}),
        ('magnetic', 'atlas_forces', dict(magnetic_refinement=max(Fixed.current_demand_mesh_refinements))))
    baseline = None
    for name, mode, settings in configurations:
        report_path, checkpoint = output / (name + '.json'), output / (name + '_checkpoint.json')
        if name in summary['cases'] and summary['cases'][name]['passed']:
            report = airborne_verification.verified_saved_trial(report_path)
            expected_backend = dict(mode=mode, identity=None if mode == 'direct' else atlas.identity,
                evaluator=None if mode == 'direct' else atlas.evaluator_identity)
            root = Path(__file__).resolve().parent.parent
            if (report['atlas'] != expected_backend or report['intended_duration_s'] != Fixed.adaptive_sensitivity_duration
                    or any(hashlib.sha256((root / filename).read_bytes()).hexdigest() != digest
                           for filename, digest in report['source_sha256'].items())):
                raise ValueError('Completed qualification case configuration or source differs')
            if name == 'adaptive':
                baseline = report
            continue
        summary.update(status='running', active_case=name)
        simulation = AtlasAirborneSimulation(construction, atlas, mode, Fixed.adaptive_sensitivity_duration,
            initialize=not checkpoint.exists(), **settings)
        summary['mechanical_properties'] = simulation.report()['mechanical_properties']
        airborne_verification.save_summary(output / 'summary.json', summary)
        if checkpoint.exists():
            simulation.load_checkpoint(checkpoint)
        else:
            simulation.save_checkpoint(checkpoint)

        def progress(report):
            if baseline is not None and report['trace'] and report['stop_reason'] is None:
                summary['comparisons'][name] = airborne_verification.compare_progress(baseline, report, simulation, name)
            if simulation.stop_reason is not None:
                summary['status'] = 'trial_failed'
            airborne_verification.save_summary(output / 'summary.json', summary)

        report = airborne.run_simulation(simulation, report_path, checkpoint, deadline=deadline, progress=progress)
        summary['cases'][name] = {key: report[key] for key in ('passed', 'finished', 'stop_reason', 'duration_completed_s', 'elapsed_s', 'peaks')}
        if name == 'adaptive':
            baseline = report
        elif report['finished'] and report['stop_reason'] is None:
            summary['comparisons'][name] = airborne_verification.compare_trials(baseline, report)
        if not report['passed'] or (name != 'adaptive' and not summary['comparisons'][name]['converged']):
            summary['status'] = 'deadline' if not report['finished'] else 'qualification_failed'
            airborne_verification.save_summary(output / 'summary.json', summary)
            return summary
        airborne_verification.save_summary(output / 'summary.json', summary)
    summary.update(status='completed', active_case=None)
    airborne_verification.save_summary(output / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Benchmark a piece-local magnetic atlas against direct fields and forces.')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--deadline', type=datetime.fromisoformat, required=True)
    parser.add_argument('--finest-only', action='store_true', help='Screen only the finest configured grid after retaining coarser evidence.')
    parser.add_argument('--fields-only', action='store_true', help='Save a passing field cache without running legacy coupled trials.')
    parser.add_argument('--qualify', type=Path, help='Qualify the selected atlas using this completed feasibility summary.')
    parser.add_argument('--resume', action='store_true', help='Resume an unfinished qualification with identical sources and atlas.')
    arguments = parser.parse_args()
    if arguments.deadline.tzinfo is None:
        parser.error('Deadline must include a timezone')
    if arguments.resume and arguments.qualify is None:
        parser.error('Resume requires qualification mode')
    if arguments.fields_only and arguments.qualify is not None:
        parser.error('Field-only screening cannot resume coupled qualification')
    if arguments.qualify is None:
        benchmark(arguments.output, arguments.deadline.timestamp(), arguments.finest_only, arguments.fields_only)
    else:
        print(json.dumps(qualify(arguments.output, arguments.deadline.timestamp(), arguments.qualify, arguments.resume)), flush=True)

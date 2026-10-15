import argparse
from functools import partial
import json
from math import ceil
from pathlib import Path
from time import perf_counter

import magpylib as magpy
import numpy as np
from numpy.polynomial.legendre import leggauss
from build123d import PositionMode

import levitation_sim
import model
from model import Inputs, Fixed, Constants
from methodology import coil_layouts, design, prototype_coils, source_fingerprints
from methodology.cad import assembly, piece, winding


def sampled_path(path, segment_length):
    points = []
    for edge in path.edges().sort_by(path):
        count = ceil(edge.length / segment_length)
        points.extend(tuple(edge.position_at(fraction)) for fraction in np.arange(count) / count)
    return np.array([*points, tuple(path @ 1)])


def closed_conductor(path, segment_length, return_depth):
    vertices = sampled_path(path, segment_length)
    end, start = vertices[-1], vertices[0]
    corners = [end, [*end[:2], -return_depth], [*start[:2], -return_depth], start]
    connection = []
    for first, second in zip(corners, corners[1:]):
        count = ceil(np.linalg.norm(np.asarray(second) - first) / segment_length)
        connection.extend(np.linspace(first, second, count + 1)[1:])
    return np.vstack((vertices, connection)) / 1000


def curved_conductor_samples(path, order, return_depth):
    if not isinstance(order, (int, np.integer)) or order < 1:
        raise ValueError('Curved conductor quadrature requires a positive integer order')
    nodes, weights = leggauss(order)
    nodes, weights = (nodes + 1) / 2, weights / 2
    positions, elements = [], []
    for edge in path.edges().sort_by(path):
        lengths = nodes * edge.length
        positions.extend(tuple(edge.position_at(length, PositionMode.LENGTH)) for length in lengths)
        elements.extend(np.asarray(tuple(edge.tangent_at(length, PositionMode.LENGTH))) * edge.length * weight
            for length, weight in zip(lengths, weights))
    start, end = np.asarray(tuple(path @ 0)), np.asarray(tuple(path @ 1))
    corners = (end, np.r_[end[:2], -return_depth], np.r_[start[:2], -return_depth], start)
    for first, second in zip(corners[:-1], corners[1:]):
        positions.extend(first + nodes[:, None] * (second - first))
        elements.extend(weights[:, None] * (second - first))
    return np.asarray(positions) / 1000, np.asarray(elements) / 1000


def curved_force_quadrature(array, samples):
    positions, elements = samples
    if any(len(coil) != 1 for coil in array.coils):
        raise ValueError('CAD curved force quadrature requires one continuous conductor per coil')
    points, vectors = [], []
    for placement, coil in zip(array.placements, array.coils):
        offset = np.array([placement['x'], placement['y'], placement['top'] - array.height])
        filament = coil[0]
        if not np.array_equal(filament.vertices, array.rotation.apply(array.conductor_vertices + offset)):
            raise ValueError('CAD force quadrature cannot reinterpret edited conductor vertices')
        points.append(filament.orientation.apply(array.rotation.apply(positions + offset)) + filament.position)
        vectors.append(filament.orientation.apply(array.rotation.apply(elements)))
    return levitation_sim.CoilForceQuadrature.from_samples(array.coils, points, vectors)


class ConductorArray(coil_layouts.LayoutArray):
    def __init__(self, vertices, coil, configuration, refinement, sample_conductor):
        self.conductor_vertices = vertices
        self.sample_conductor = sample_conductor
        self.force_order = Fixed.winding_force_quadrature_order * refinement
        super().__init__('square', coil, configuration, dict(Inputs.prototype_layouts)[Inputs.cad_coil_layout],
                         refinement, Inputs.prototype_coil_spacing)

    def create_force_quadrature(self):
        return curved_force_quadrature(self, self.sample_conductor())

    def winding(self, orientation, center_x, center_y, z_top):
        if orientation != 'x':
            raise ValueError('The CAD conductor adapter requires the square lattice orientation')
        loop = magpy.current.Polyline(current=1, vertices=self.conductor_vertices + [center_x, center_y, z_top - self.height])
        loop.meshing = 1
        return [loop]


class ConstructionModel:
    def __init__(self):
        self.path, _, reference = winding.continuous_path()
        _, self.coil, _ = prototype_coils.prototype_geometry(Inputs.cad_coil_layout)
        self.configuration = design.WindingGeometry(self.coil, Inputs.prototype_wire_diameter, reference.layers,
                    wire=reference.wire, radial_turns=reference.turns_per_layer, path_length_mm=self.path.length,
                    resistance_per_m=max(Fixed.prototype_wire_resistance_per_m))
        _, reference_piece = coil_layouts.reference_state()
        magnets = assembly.magnet_parts(levitation_sim.magnet_layout_from_geometry())
        _, self.piece_properties = piece.construction_parts(reference_piece, magnets)
        self.use_cad_mass = True
        self.return_depth = 0
        self.vertices = {}
        self.quadrature_samples = {}

    def force_samples(self, order, return_depth):
        key = order, return_depth
        if key not in self.quadrature_samples:
            self.quadrature_samples[key] = curved_conductor_samples(self.path, order, return_depth)
        return self.quadrature_samples[key]

    def __call__(self, name, refinement):
        if name != Inputs.cad_coil_layout:
            raise ValueError('The CAD construction model must use its configured layout')
        geometry, _ = coil_layouts.reference_state()
        if self.use_cad_mass:
            properties = self.piece_properties
            geometry.weight = properties['mass_g'] / 1000 * Constants.gravity
            geometry.center_of_mass_height = properties['center_mm'][2] / 1000
            geometry.inertias = np.diag(properties['inertia_kg_m2']).copy()
            if not np.allclose(properties['center_mm'][:2], 0, atol=Fixed.cad_geometry_tolerance, rtol=0):
                raise ValueError('Simulator requires the CAD centre of mass on the vertical axis')
            tensor = np.asarray(properties['inertia_kg_m2'])
            if np.max(np.abs(tensor - np.diag(geometry.inertias))) > np.max(geometry.inertias) * Fixed.matrix_rank_tolerance:
                raise ValueError('Simulator requires CAD principal inertia axes aligned with piece axes')
        key = refinement, self.return_depth
        if key not in self.vertices:
            self.vertices[key] = closed_conductor(self.path, Fixed.winding_magnetic_segment_length / refinement, self.return_depth)
        sample_conductor = partial(self.force_samples, Fixed.winding_force_quadrature_order * refinement, self.return_depth)
        return geometry, ConductorArray(self.vertices[key], self.coil, self.configuration, refinement, sample_conductor), self.configuration


def fingerprints():
    return source_fingerprints(model.__file__, levitation_sim.__file__, design.__file__, coil_layouts.__file__,
                               prototype_coils.__file__, assembly.__file__, piece.__file__, winding.__file__, __file__)


def run(output):
    started = perf_counter()
    construction = ConstructionModel()
    previous_path = Path(prototype_coils.__file__).with_name('results') / 'prototype_coils.json'
    previous = json.loads(previous_path.read_text())
    rows = [row for row in previous['refined_paths'] if row['name'] == Inputs.cad_coil_layout]
    selections = []
    for metric, trace in (('peak_required_rating_A', 'required_rating_A'), ('voltage_bound_peak_V', 'voltage_bound_V')):
        row = max(rows, key=lambda item: item[metric])
        start = int(np.clip(np.argmax(row[trace]) - 1, 0, row['samples'] - 3))
        fractions = np.arange(start, start + 3) / (row['samples'] - 1)
        selections.append((metric, row, fractions))
    report = dict(scope='Local detailed-conductor comparisons around the prior square handoff current and voltage maxima; prescribed level motion, not global or closed-loop qualification',
                  source_sha256=fingerprints(), selection_evidence_sha256=source_fingerprints(previous_path),
                  king=construction.piece_properties, wire_length_mm=construction.path.length,
                  resistance_20C_ohm=construction.configuration.resistance,
                  return_connection='Local terminal-to-terminal magnetic closure at the PCB top or bottom. Proxy for cancelling paired supply/return wiring, not a physical short or routed driver circuit. PCB resistance is excluded.',
                  limitations=['Nominal geometry only; approximately 6 micrometres minimum enamel clearance has no manufacturing tolerance qualification.',
                               'Filament centreline model excludes finite conductor cross-section and high-frequency current redistribution.',
                               'Self/mutual inductance and PWM ripple remain the earlier uncertain estimates; return-path checks only test magnetic force sensitivity.',
                               'Old trajectory maxima select local neighborhoods; new global worst positions, tilt/yaw dynamics, feedback frequency, heating and reset remain unqualified.'],
                  comparisons=[])
    for metric, row, fractions in selections:
        case = dict(selection_metric=metric, path_angle_deg=row['path_angle_deg'], offset_fraction=row['offset_fraction'],
                    yaw_deg=row['yaw_deg'], fractions=fractions.tolist(), variants=[])
        variants = [('closed_turn_reference', False, 0, min(Fixed.current_demand_mesh_refinements)),
                    ('continuous_wire_only', False, 0, min(Fixed.current_demand_mesh_refinements))]
        variants.extend(('continuous_with_cad_king', True, 0, refinement) for refinement in Fixed.current_demand_mesh_refinements)
        variants.append(('pcb_bottom_return', True, Fixed.pcb_thickness, max(Fixed.current_demand_mesh_refinements)))
        for label, cad_mass, depth, refinement in variants:
            construction.use_cad_mass, construction.return_depth = cad_mass, depth
            job = (row['name'], row['path_angle_deg'], row['offset_fraction'], row['yaw_deg'], len(fractions), refinement)
            before = perf_counter()
            result = prototype_coils.evaluate_path(job, sample_fractions=fractions,
                          physical_model=None if label == 'closed_turn_reference' else construction)
            result.update(variant=label, elapsed_s=perf_counter() - before)
            case['variants'].append(result)
            print(json.dumps({key: result[key] for key in ('variant', 'mesh_refinement', 'peak_required_rating_A',
                              'peak_copper_loss_W', 'voltage_bound_peak_V', 'elapsed_s')}), flush=True)
        detailed = [result for result in case['variants'] if result['variant'] == 'continuous_with_cad_king']
        metrics = ('peak_required_rating_A', 'peak_copper_loss_W', 'voltage_bound_peak_V', 'peak_current_slope_A_per_s')
        changes = {key: abs(detailed[-1][key] / detailed[0][key] - 1) for key in metrics}
        case['mesh_check'] = dict(relative_changes=changes, passed=max(changes.values()) <= Fixed.qualification_convergence_fraction)
        case['return_sensitivity'] = {key: case['variants'][-1][key] / detailed[-1][key] - 1 for key in metrics}
        report['comparisons'].append(case)
        report['elapsed_s'] = perf_counter() - started
        report['segments_per_coil'] = {str(key): len(value) - 1 for key, value in construction.vertices.items()}
        output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')


def check_path(output):
    report = json.loads(output.read_text())
    construction = ConstructionModel()
    paths = sorted(set((row['path_angle_deg'], row['offset_fraction'], row['yaw_deg']) for row in report['comparisons']))
    report['candidate_paths'] = []
    report['candidate_path_source_sha256'] = fingerprints()
    for angle, offset, yaw in paths:
        started = perf_counter()
        job = (Inputs.cad_coil_layout, angle, offset, yaw, Fixed.prototype_path_samples,
               min(Fixed.current_demand_mesh_refinements))
        result = prototype_coils.evaluate_path(job, physical_model=construction)
        result['elapsed_s'] = perf_counter() - started
        result['sampling_status'] = 'Full-path sample spacing has not yet been refined for this construction; local magnetic refinement follows.'
        result['local_mesh_checks'] = []
        report['candidate_paths'].append(result)
        output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
        print(json.dumps({key: result[key] for key in ('peak_required_rating_A', 'peak_copper_loss_W',
                          'voltage_bound_peak_V', 'elapsed_s')}), flush=True)
        for metric, trace in (('peak_required_rating_A', 'required_rating_A'), ('voltage_bound_peak_V', 'voltage_bound_V')):
            start = int(np.clip(np.argmax(result[trace]) - 1, 0, result['samples'] - 3))
            fractions = np.arange(start, start + 3) / (result['samples'] - 1)
            values = []
            for refinement in Fixed.current_demand_mesh_refinements:
                local = prototype_coils.evaluate_path((Inputs.cad_coil_layout, angle, offset, yaw, len(fractions), refinement),
                            sample_fractions=fractions, physical_model=construction)
                values.append({key: local[key] for key in ('peak_required_rating_A', 'peak_copper_loss_W', 'voltage_bound_peak_V')})
            changes = {key: abs(values[-1][key] / values[0][key] - 1) for key in values[0]}
            check = dict(metric=metric, fractions=fractions.tolist(), values=values,
                         relative_changes=changes, passed=max(changes.values()) <= Fixed.qualification_convergence_fraction)
            result['local_mesh_checks'].append(check)
            print(json.dumps(check), flush=True)
            output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compare continuous CAD windings and king properties with the existing motion screen.')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'results' / 'cad_magnetics.json')
    parser.add_argument('--path', action='store_true', help='Check complete previously selected critical paths and their new local extrema.')
    arguments = parser.parse_args()
    if arguments.path:
        check_path(arguments.output)
    else:
        run(arguments.output)

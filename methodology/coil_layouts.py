import argparse
from concurrent.futures import ProcessPoolExecutor
from fractions import Fraction
import json
from math import ceil, lcm
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import levitation_sim
import model
from model import Inputs, Fixed, Constants
from methodology import design, source_fingerprints


class LayoutArray(levitation_sim.CoilArray):
    def __init__(self, pattern, coil, winding, angle, refinement, spacing=0):
        self.pattern = pattern
        self.spacing = spacing / 1000
        self.rotation = Rotation.from_euler('z', angle, degrees=True)
        self.window = Fixed.layout_screen_window_side / 1000
        self.placements = []
        axial = min(Fixed.max_coil_axial_filaments, max(Fixed.minimum_axial_filaments,
                    ceil(winding.coil_height / 1000 / Fixed.axial_filament_spacing)))
        super().__init__(0, coil.outer_width / 1000, coil.outer_length / 1000,
                         coil.winding_radial_width / 1000, winding.coil_height / 1000, winding.turns,
                         Inputs.magnet_to_coil_distance / 1000, winding.coil_height / 1000,
                         Fixed.verification_force_mesh * refinement, Fixed.coil_radial_filaments * refinement,
                         axial * refinement, Inputs.control_cells_per_side, coil.corner_radius / 1000,
                         Fixed.prototype_corner_segments * refinement)

    @property
    def short_pitch(self):
        return self.short + self.spacing

    @property
    def repeat(self):
        if self.pattern == 'herringbone':
            return 4 * self.short_pitch
        if self.pattern == 'square':
            return self.short_pitch
        short, long = Fraction(str(self.short)), Fraction(str(self.long))
        denominator = lcm(short.denominator, long.denominator)
        return lcm(int(short * denominator), int(long * denominator)) / denominator

    @property
    def layers(self):
        return Fixed.herringbone_orientation_families if self.pattern == 'stacked' else 1

    def candidates(self, center):
        radius = ceil((max(abs(center)) + self.window + self.repeat) / self.short)
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                if self.pattern == 'stacked':
                    short_phase = ((self.lattice_cells - 1) / 2) % 1
                    long_count = max(1, round(self.lattice_cells * self.short / self.long))
                    long_phase = ((long_count - 1) / 2) % 1
                    yield ('x', i, j), (i + short_phase) * self.short, (j + long_phase) * self.long, -self.gap
                    yield ('y', i, j), (i + long_phase) * self.long, (j + short_phase) * self.short, -self.gap - self.height
                elif self.pattern == 'square':
                    yield ('x', i, j), i * self.short_pitch, j * self.short_pitch, -self.gap
                elif self.pattern == 'herringbone':
                    if not np.isclose(self.long, 2 * self.short, rtol=0, atol=Fixed.cad_geometry_tolerance / 1000):
                        raise ValueError('This parquet construction requires a two-to-one rectangle')
                    if (i - j) % 4 == 0:
                        yield ('y', i, j), (i + 1) * self.short_pitch, (j + 1 / 2) * self.short_pitch, -self.gap
                    if (i - j) % 4 == 3:
                        yield ('x', i, j), (i + 1 / 2) * self.short_pitch, (j + 1) * self.short_pitch, -self.gap
                else:
                    raise ValueError(f'Unknown coil pattern: {self.pattern}')

    def build(self, center=(0, 0), half_extents=None):
        if half_extents is not None:
            raise ValueError('This layout screen does not qualify board-edge windows')
        center = self.rotation.inv().apply([*center, 0])[:2]
        self.coils, self.identities, self.placements = [], [], []
        for identity, x, y, top in self.candidates(center):
            width, length = (self.short, self.long) if identity[0] == 'x' else (self.long, self.short)
            if np.any(np.abs(np.array([x, y]) - center) + np.array([width, length]) / 2
                      > self.window / 2 + Fixed.cad_geometry_tolerance / 1000):
                continue
            if identity not in self.windings_by_identity:
                filaments = self.winding(identity[0], x, y, top)
                for filament in filaments:
                    filament.vertices = self.rotation.apply(filament.vertices)
                self.windings_by_identity[identity] = filaments
            self.identities.append(identity)
            self.coils.append(self.windings_by_identity[identity])
            self.placements.append(dict(identity=identity, x=x, y=y, top=top, width=width, length=length))


def reference_state():
    board = design.BoardGeometry()
    coil = design.CoilBed(board)
    piece = design.Piece(board, design.HalbachArray(board))
    geometry = levitation_sim.SimGeometry(Inputs, Fixed, Constants, board, coil, piece)
    levitation_sim.use_geometry(geometry)
    return geometry, piece


def candidate_geometry(candidate):
    name, pattern, width, length, wall = candidate
    coil = design.CoilGeometry(width, length, wall)
    if min(width, length) <= 2 * wall or coil.conductor_radial_width <= 0:
        raise ValueError(f'Invalid winding cross-section: {name}')
    winding = design.WindingGeometry(coil, Inputs.selected_conductor_thickness, Inputs.selected_winding_layers)
    return coil, winding


def validate_layout(array):
    for index, first in enumerate(array.placements):
        for second in array.placements[index + 1:]:
            first_center = np.array([first['x'], first['y'], first['top'] - array.height / 2])
            second_center = np.array([second['x'], second['y'], second['top'] - array.height / 2])
            extent = np.array([first['width'] + second['width'], first['length'] + second['length'], 2 * array.height]) / 2
            if np.all(extent - np.abs(first_center - second_center) > Fixed.cad_geometry_tolerance / 1000):
                raise ValueError(f'Overlapping winding envelopes: {first["identity"]}, {second["identity"]}')


def stress_parameters(geometry, winding, hot):
    mass_factor = 1 + Fixed.qualification_mass_fraction if hot else 1
    com_factor = 1 + Fixed.qualification_com_fraction if hot else 1
    br_factor = 1 + Constants.ndfeb_br_tempco * (Fixed.max_touch_temperature - Fixed.material_reference_temperature) if hot else 1
    copper_temperature = Inputs.coil_bed_temp_limit if hot else Fixed.material_reference_temperature
    resistance = winding.resistance * (1 + Constants.copper_resistivity_tempco * (copper_temperature - Fixed.material_reference_temperature))
    return geometry.weight * mass_factor, com_factor, br_factor, resistance


def stressed_wrench(matrix, geometry, com_factor, br_factor):
    wrench = matrix.copy()
    displacement = np.array([0, 0, geometry.center_of_mass_height * (1 - com_factor)])
    wrench[3:] += np.cross(displacement, wrench[:3].T).T
    return wrench * br_factor


def evaluate_layout(job):
    candidate, angle, samples, refinement, yaws = job
    geometry, piece = reference_state()
    coil, winding = candidate_geometry(candidate)
    array = LayoutArray(candidate[1], coil, winding, angle, refinement)
    validate_layout(array)
    magnet = levitation_sim.magnet_layout_from_geometry()
    phases = np.arange(samples) * array.repeat / samples
    cases = []
    ranks = []
    counts = set()
    for gap in (geometry.gap, geometry.max_flight_gap):
        bed = array.layers * array.height
        response = levitation_sim.conductive_sheet_response_at_depths(
            gap + bed + Fixed.pcb_thickness / 2000,
            gap + bed + Fixed.pcb_thickness / 1000 + Fixed.radiator_standoff_below_pcb / 1000)
        drag = sum(levitation_sim.drag_components_at_speed(geometry.flight_speed, response).values())
        for yaw in np.radians(yaws):
            for x in phases:
                for y in phases:
                    world = array.rotation.apply([x, y, 0])
                    array.build(center=world[:2])
                    counts.add(len(array.coils))
                    _, center = levitation_sim.place_piece(magnet, x=world[0], y=world[1], z=gap - geometry.gap, yaw=yaw)
                    matrix = levitation_sim.actuator_matrix(magnet, array, center)
                    normalized = matrix.copy()
                    normalized[3:] /= piece.diameter / 2000
                    ranks.append(levitation_sim.matrix_rank(np.vstack((normalized, np.ones(len(array.coils))))))
                    if ranks[-1] < 7:
                        continue
                    for hot in (False, True):
                        weight, com_factor, br_factor, resistance = stress_parameters(geometry, winding, hot)
                        wrench = stressed_wrench(matrix, geometry, com_factor, br_factor)
                        thrust = weight / geometry.gravity * geometry.cruise_accels[0] + drag * br_factor ** 2
                        targets = np.vstack(([0, 0, weight, 0, 0, 0], levitation_sim.horizontal_wrench_targets(thrust, weight)))
                        for target in targets:
                            peak, initial = levitation_sim.min_peak_current(wrench, target)
                            cases.append(dict(matrix=wrench, target=target, initial=initial, peak_A=peak,
                                              resistance=resistance, hot=hot, gap_mm=gap * 1000,
                                              local_phase_mm=[x * 1000, y * 1000], world_xy_mm=(world[:2] * 1000).tolist(),
                                              yaw_deg=float(np.degrees(yaw)), thrust_N=target[:2].tolist()))
    result = dict(name=candidate[0], pattern=candidate[1], rotation_deg=angle, dimensions_mm=list(candidate[2:]),
                  conductor_width_mm=coil.conductor_radial_width, conductor_thickness_mm=Inputs.selected_conductor_thickness,
                  turns=winding.turns, resistance_20C_ohm=winding.resistance, coil_height_mm=winding.coil_height,
                  bed_height_mm=bed * 1000, coil_counts=sorted(counts), augmented_rank_min=min(ranks),
                  pose_count=len(ranks), demand_count=len(cases), phase_samples=samples, mesh_refinement=refinement, yaws_deg=list(yaws),
                  lattice_repeat_mm=array.repeat * 1000, coil_density_per_m2=array.layers / (array.short * array.long),
                  copper_volume_per_m2_mm3=array.layers / (array.short * array.long) * winding.length_per_winding * winding.cross_section_area * 1000000000)
    if min(ranks) < 7:
        return dict(result, status='Insufficient six-axis plus midpoint-current authority at sampled poses')
    limit = max(Fixed.driver_channel_current, max(case['peak_A'] for case in cases))
    for case in cases:
        currents = levitation_sim.min_loss_current(case['matrix'], case['target'], limit, initial_currents=case['initial'])
        case['loss_W'] = case['resistance'] * float(currents @ currents)
    def describe(case):
        return {key: value for key, value in case.items() if key not in ('matrix', 'initial', 'target', 'resistance')}
    for label, hot in (('nominal', False), ('hot_heavy', True)):
        selected = [case for case in cases if case['hot'] == hot]
        result[label] = dict(peak_current_A=max(case['peak_A'] for case in selected),
                             worst_copper_loss_W=max(case['loss_W'] for case in selected),
                             worst_current_pose=describe(max(selected, key=lambda case: case['peak_A'])),
                             worst_loss_pose=describe(max(selected, key=lambda case: case['loss_W'])))
    result['loss_allocation_limit_A'] = limit
    result['static_current_reserve_pass'] = result['hot_heavy']['peak_current_A'] <= Fixed.driver_channel_current * (1 - Fixed.qualification_current_reserve_fraction)
    result['status'] = 'Static screen only; dynamics, neighbours, edges and thermal acceptance unqualified'
    return result


def run_screen(output, names, samples, refinement, refine):
    candidates = [candidate for candidate in Inputs.coil_layout_candidates if not names or candidate[0] in names]
    if not candidates:
        raise ValueError('No matching layout candidates')
    jobs = [(candidate, angle, samples, refinement, Fixed.pose_yaws_deg) for candidate in candidates
            for angle in ((0,) if candidate[1] == 'stacked' else Fixed.layout_screen_rotations_deg)]
    if refine:
        report = json.loads(output.read_text())
        selected = [next(row for row in report['results'] if row['pattern'] == 'stacked')]
        for pattern in ('herringbone', 'square'):
            passing = [row for row in report['results'] if row['pattern'] == pattern and row.get('static_current_reserve_pass', False)]
            if not passing:
                raise ValueError(f'No passing candidate for refinement: {pattern}')
            selected.append(min(passing, key=lambda row: row['hot_heavy']['worst_copper_loss_W']))
        by_name = {candidate[0]: candidate for candidate in Inputs.coil_layout_candidates}
        jobs = [(by_name[row['name']], row['rotation_deg'], Fixed.layout_refinement_phase_samples,
                 refinement, Fixed.layout_refinement_yaws_deg) for row in selected]
    with ProcessPoolExecutor(max_workers=Fixed.qualification_parallel_workers) as pool:
        results = []
        for result in pool.map(evaluate_layout, jobs):
            results.append(result)
            print(json.dumps(result), flush=True)
    if refine:
        report['refined_results'] = results
        report['refinement_source_sha256'] = source_fingerprints(model.__file__, design.__file__, levitation_sim.__file__, __file__)
        output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
        return
    report = dict(scope='Common local window; level hover and horizontal flight; nominal and hot/heavy king; static allocations with zero-sum currents',
                  window_side_mm=Fixed.layout_screen_window_side, results=results,
                  limitations=['Sampled static screen, not trajectory or complete-board qualification.',
                               'Equal local window area, not equal channel count or equal copper mass.',
                               'Winding envelopes touch; bend radii, terminals and construction clearance remain unresolved.',
                               'Conductor widths are geometric candidates, not supplier-confirmed products.',
                               'Existing approximate conductive-sheet drag model, with depth adjusted for the number of coil layers.',
                               'Loss comparisons use the installed current limit, raised explicitly when the candidate needs more current.',
                               'Existing magnet layout retained; literature magnet arrays are not assumed equivalent.'],
                  source_sha256=source_fingerprints(model.__file__, design.__file__, levitation_sim.__file__, __file__))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')


def check_shortlist(output):
    report = json.loads(output.read_text())
    candidates = {candidate[0]: candidate for candidate in Inputs.coil_layout_candidates}
    for row in report['refined_results']:
        geometry, piece = reference_state()
        candidate = candidates[row['name']]
        coil, winding = candidate_geometry(candidate)
        inductance = design.winding_inductance(coil, winding)
        minimum = inductance * min(Fixed.qualification_inductance_factors)
        ripple = design.split_rail_ripple_peak(winding.resistance, minimum,
                                              Inputs.selected_bus_voltage * Fixed.coil_bus_voltage_fraction, Fixed.driver_pwm_frequency)
        row['electrical_screen'] = dict(estimated_inductance_mH=inductance * 1000, assumed_minimum_mH=minimum * 1000,
                                       pwm_frequency_Hz=Fixed.driver_pwm_frequency, ripple_peak_A=ripple,
                                       ripple_allowance_A=Fixed.driver_max_ripple_current,
                                       ripple_pass=ripple <= Fixed.driver_max_ripple_current,
                                       status='Existing approximate independent-RL screen; actual self/mutual inductance unmeasured')
        checks = []
        for stress in ('nominal', 'hot_heavy'):
            for metric in ('worst_current_pose', 'worst_loss_pose'):
                pose = row[stress][metric]
                values = []
                for refinement in Fixed.current_demand_mesh_refinements:
                    array = LayoutArray(candidate[1], coil, winding, row['rotation_deg'], refinement)
                    array.build(center=np.asarray(pose['world_xy_mm']) / 1000)
                    magnet = levitation_sim.magnet_layout_from_geometry()
                    _, center = levitation_sim.place_piece(magnet, x=pose['world_xy_mm'][0] / 1000,
                                                          y=pose['world_xy_mm'][1] / 1000, z=pose['gap_mm'] / 1000 - geometry.gap,
                                                          yaw=np.radians(pose['yaw_deg']))
                    matrix = levitation_sim.actuator_matrix(magnet, array, center)
                    weight, com_factor, br_factor, resistance = stress_parameters(geometry, winding, pose['hot'])
                    wrench = stressed_wrench(matrix, geometry, com_factor, br_factor)
                    target = [*pose['thrust_N'], weight, 0, 0, 0]
                    peak, initial = levitation_sim.min_peak_current(wrench, target)
                    currents = levitation_sim.min_loss_current(wrench, target, max(row['loss_allocation_limit_A'], peak), initial_currents=initial)
                    values.append(dict(refinement=refinement, peak_A=peak, copper_loss_W=resistance * float(currents @ currents)))
                    if (candidate[1] == 'stacked' and stress == 'hot_heavy' and metric == 'worst_current_pose'
                            and refinement == min(Fixed.current_demand_mesh_refinements)):
                        existing = levitation_sim.verification_coil_array(Inputs.control_cells_per_side, winding.coil_height / 1000)
                        components = levitation_sim.window_components(magnet, existing, np.asarray(pose['world_xy_mm']) / 1000, center)
                        components = {identities: (stressed_wrench(matrix, geometry, com_factor, br_factor) * winding.turns, weight)
                                      for identities, (matrix, weight) in components.items()}
                        allocated, required = levitation_sim.allocate_window_components(components, target, Fixed.driver_channel_current)
                        row['existing_window_at_worst_screen_pose'] = dict(
                            component_coil_counts=[len(identities) for identities in components],
                            union_coil_count=len(allocated), required_component_peak_A=required,
                            allocated_peak_A=max(abs(current) for current in allocated.values()),
                            copper_loss_W=resistance * sum(current ** 2 for current in allocated.values()),
                            status='Same pose with the existing fixed-count blended selection; not a full resweep')
                checks.append(dict(stress=stress, metric=metric, values=values,
                                   peak_change_fraction=abs(values[-1]['peak_A'] / values[0]['peak_A'] - 1),
                                   loss_change_fraction=abs(values[-1]['copper_loss_W'] / values[0]['copper_loss_W'] - 1)))
        row['mesh_checks'] = checks
        row['mesh_check_pass'] = all(max(check['peak_change_fraction'], check['loss_change_fraction'])
                                     <= Fixed.qualification_convergence_fraction for check in checks)
        print(json.dumps(dict(name=row['name'], electrical=row['electrical_screen'], mesh_pass=row['mesh_check_pass'])), flush=True)
    report['check_source_sha256'] = source_fingerprints(model.__file__, design.__file__, levitation_sim.__file__, __file__)
    output.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compare non-overlapping single-layer coils with the stacked baseline.')
    parser.add_argument('--name', action='append')
    parser.add_argument('--refine', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--samples', type=int, default=Fixed.layout_screen_phase_samples)
    parser.add_argument('--refinement', type=int, default=min(Fixed.current_demand_mesh_refinements))
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('results') / 'coil_layouts.json')
    arguments = parser.parse_args()
    if arguments.check:
        check_shortlist(arguments.output)
    else:
        run_screen(arguments.output, arguments.name, arguments.samples, arguments.refinement, arguments.refine)

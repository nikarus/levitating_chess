import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
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


def driver_connection_basis(identities, connection):
    if len(set(identities)) != len(identities):
        raise ValueError('Coil identities must be unique')
    if connection == 'independent':
        return np.eye(len(identities))
    if connection not in ('series_same', 'series_opposed'):
        raise ValueError('Unknown driver connection')
    groups = {}
    for index, (orientation, x, y) in enumerate(identities):
        if orientation != 'x':
            raise ValueError('Paired basket-weave wiring requires the square lattice')
        tile_x, tile_y = x // 2, y // 2
        horizontal = (tile_x + tile_y) % 2 == 0
        key = (tile_x, tile_y, y % 2 if horizontal else x % 2)
        member = x % 2 if horizontal else y % 2
        groups.setdefault(key, {})[member] = index
    complete = [members for _, members in sorted(groups.items()) if len(members) == 2]
    basis = np.zeros((len(identities), len(complete)))
    for column, members in enumerate(complete):
        basis[members[0], column] = 1
        basis[members[1], column] = -1 if connection == 'series_opposed' else 1
    return basis


def channel_repeat(array, connection):
    return array.repeat if connection == 'independent' else 4 * array.short_pitch


def channel_density(array, connection):
    coil_density = 1 / (array.short_pitch ** 2 * (2 if array.pattern == 'herringbone' else 1))
    return coil_density / (1 if connection == 'independent' else 2)


def channel_wrench_targets(geometry, magnet, gap=None):
    weight = geometry.weight
    hover = np.array([0., 0., weight, 0., 0., 0.])
    targets = [('hover', hover)]
    lift = hover.copy()
    lift[2] *= Inputs.force_safety_factor
    targets.append(('lift_reserve', lift))
    thrust = weight / geometry.gravity * geometry.cruise_accels[0]
    targets.extend((f'translation_{index}', target) for index, target in enumerate(
        levitation_sim.horizontal_wrench_targets(thrust, weight)))
    tilt = levitation_sim.rim_tilt_limit(magnet, geometry.max_flight_gap if gap is None else gap)
    accelerations = np.array([4 * tilt / Inputs.target_tilt_time ** 2,
        4 * tilt / Inputs.target_tilt_time ** 2, 4 * np.radians(Inputs.target_yaw_angle_deg) / Inputs.target_yaw_time ** 2])
    rotation = magnet.collection.orientation.as_matrix()
    inertia = rotation @ np.diag(geometry.inertias) @ rotation.T
    for axis, acceleration in enumerate(accelerations):
        for sign in (-1, 1):
            target = hover.copy()
            target[3:] = sign * inertia[:, axis] * acceleration
            targets.append((f'torque_{axis}_{sign}', target))
    return targets


def channel_demands(matrix, targets, resistance, budget):
    demands = []
    for label, target in targets:
        peak, initial = levitation_sim.min_peak_current(matrix, target)
        feasible = bool(peak <= budget)
        current = (levitation_sim.min_loss_current(matrix, target, budget,
            resistances=resistance, initial_currents=initial) if feasible else initial)
        demands.append(dict(action=label, target_wrench=target.tolist(), minimum_peak_current_A=peak,
            copper_loss_W=float(resistance @ current ** 2) if feasible else None, current_reserve_met=feasible,
            residual=float(max(np.max(np.abs(matrix @ current - target)), abs(current.sum())))))
    return demands


def run_channel_pilot(output, window_mm):
    from time import perf_counter
    from zipfile import ZipFile, ZIP_DEFLATED
    from methodology import prototype_coils
    from methodology.cad import magnetics
    if output.exists():
        raise ValueError('Preserve existing channel pilot evidence')
    if window_mm not in Fixed.channel_screen_window_sides:
        raise ValueError('Channel support window must be a configured model candidate')
    started = perf_counter()
    construction = magnetics.ConstructionModel()
    geometry, _, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    magnet = levitation_sim.magnet_layout_from_geometry()
    targets = channel_wrench_targets(geometry, magnet)
    sources = magnetics.fingerprints()
    report = dict(status='incomplete', scope='Preliminary level-pose single-piece channel-count screen', source_sha256=sources, support_window_mm=window_mm,
        piece_properties=construction.piece_properties, results=[],
        limits=dict(channel_current_A=Fixed.driver_channel_current,
            reserved_current_A=Fixed.driver_channel_current * (1 - Fixed.qualification_current_reserve_fraction)),
        limitations=['Every candidate uses the same rounded closed-turn approximation and CAD-derived king mass/inertia.',
            'The production continuous conductor, leads, crossovers and full circuit return are not reproduced.',
            'Nominal mass/remanence/resistance; level pose at maximum flight gap and first configured yaw only.',
            'Individual force/torque requests include gravity; combined extremes, drag and tilted geometry remain untested.',
            'Sparse phase samples have no spatial or magnetic refinement qualification.',
            'Only complete local series pairs are energized; boundary partners outside the common window remain off.',
            'Series groups use one midpoint-return branch per driver; driver currents, not signed winding currents, sum to zero.',
            'Neighbours, edge windows, contact reset, sensing, electrical transients and thermal acceptance are unqualified.',
            'Minimum-loss allocation is evaluated only for targets feasible within the unchanged reserved current budget.',
            'Density excludes board-edge rounding; it is not a BOM saving or a full-board channel count.'])
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(name, name)
    for name, layout_name, angle, spacing, connection in Inputs.channel_layout_candidates:
        candidate, coil, winding = prototype_coils.prototype_geometry(layout_name)
        array = LayoutArray(candidate[1], coil, winding, angle, min(Fixed.current_demand_mesh_refinements), spacing)
        array.window = window_mm / 1000
        paired = connection != 'independent'
        repeat = channel_repeat(array, connection)
        coil_density = channel_density(array, 'independent')
        result = dict(name=name, geometry=layout_name, connection=connection, rotation_deg=angle,
            outer_dimensions_mm=[coil.outer_width, coil.outer_length], spacing_mm=spacing,
            electrical_repeat_mm=repeat * 1000, coils_per_m2=coil_density,
            channels_per_m2=coil_density / (2 if paired else 1), turns=winding.turns,
            winding_resistance_20C_ohm=winding.resistance, rows=[])
        for phase in Fixed.channel_pilot_phase_points:
            position = array.rotation.apply([*(np.array(phase) * repeat), 0])
            array.build(center=position[:2])
            validate_layout(array)
            basis = driver_connection_basis(array.identities, connection)
            _, center = levitation_sim.place_piece(magnet, x=position[0], y=position[1],
                z=geometry.max_flight_gap - geometry.gap, yaw=np.radians(Fixed.pose_yaws_deg[0]))
            matrix = levitation_sim.actuator_matrix(magnet, array, center) @ basis
            normalized = matrix.copy()
            normalized[3:] /= geometry.magnet_lateral_edge * geometry.magnets_per_period / 2
            rank = levitation_sim.matrix_rank(np.vstack((normalized, np.ones(matrix.shape[1]))))
            row = dict(phase=list(phase), world_xy_mm=(position[:2] * 1000).tolist(),
                gap_mm=geometry.max_flight_gap * 1000, yaw_deg=Fixed.pose_yaws_deg[0],
                available_coils=len(array.coils), driven_coils=int(np.count_nonzero(np.any(basis != 0, axis=1))),
                channels=matrix.shape[1], augmented_rank=rank, demands=[])
            if rank == 7:
                resistance = winding.resistance * np.sum(basis ** 2, axis=0)
                row['demands'] = channel_demands(matrix, targets, resistance, report['limits']['reserved_current_A'])
            result['rows'].append(row)
        result['all_sampled_ranks_full'] = all(row['augmented_rank'] == 7 for row in result['rows'])
        demands = [demand for row in result['rows'] for demand in row['demands']]
        result['sampled_current_reserve_met'] = result['all_sampled_ranks_full'] and all(demand['current_reserve_met'] for demand in demands)
        result['peak_required_current_A'] = max((demand['minimum_peak_current_A'] for demand in demands), default=None)
        result['peak_copper_loss_W'] = max(demand['copper_loss_W'] for demand in demands) if result['sampled_current_reserve_met'] else None
        report['results'].append(result)
        report['elapsed_s'] = perf_counter() - started
        (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
        print(json.dumps({key: value for key, value in result.items() if key != 'rows'}), flush=True)
    report['status'] = 'pilot_complete_not_qualified'
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    return report


def channel_pose_case(array, winding, geometry, magnet, phase, gap, yaw, tilt_fraction, tilt_direction, connection='independent'):
    position = array.rotation.apply([*(np.asarray(phase) * channel_repeat(array, connection)), 0])
    array.build(center=position[:2])
    validate_layout(array)
    basis = driver_connection_basis(array.identities, connection)
    limit = levitation_sim.rim_tilt_limit(magnet, gap)
    direction = np.radians(tilt_direction)
    rotation = Rotation.from_rotvec(limit * tilt_fraction * np.array([np.cos(direction), np.sin(direction), 0])) * Rotation.from_euler('z', yaw, degrees=True)
    roll, pitch, yaw_rad = rotation.as_euler('xyz')
    _, center = levitation_sim.place_piece(magnet, x=position[0], y=position[1], z=gap - geometry.gap,
        roll=roll, pitch=pitch, yaw=yaw_rad)
    matrix = levitation_sim.actuator_matrix(magnet, array, center) @ basis
    scaled = matrix.copy()
    scaled[3:] /= geometry.magnet_lateral_edge * geometry.magnets_per_period / 2
    rank = levitation_sim.matrix_rank(np.vstack((scaled, np.ones(matrix.shape[1]))))
    row = dict(phase=list(phase), gap_m=gap, yaw_deg=yaw, tilt_fraction=tilt_fraction,
        tilt_direction_deg=tilt_direction, pose_rad=[roll, pitch, yaw_rad], channels=matrix.shape[1],
        available_coils=len(array.coils), driven_coils=int(np.count_nonzero(np.any(basis != 0, axis=1))),
        world_xy_mm=(position[:2] * 1000).tolist(),
        augmented_rank=rank, demands=[])
    if rank == 7:
        row['demands'] = channel_demands(matrix, channel_wrench_targets(geometry, magnet, gap),
            winding.resistance * np.sum(basis ** 2, axis=0),
            Fixed.driver_channel_current * (1 - Fixed.qualification_current_reserve_fraction))
    return row, matrix


def channel_case_specification(row):
    return tuple(row[key] for key in ('phase', 'gap_m', 'yaw_deg', 'tilt_fraction', 'tilt_direction_deg'))


def channel_critical_cases(rows):
    selected = []
    deficient = [row for row in rows if row['augmented_rank'] < 7]
    if deficient:
        selected.append(('rank_deficiency', deficient[0], None))
    for metric in ('minimum_peak_current_A', 'copper_loss_W'):
        eligible = [(demand[metric], row, demand) for row in rows for demand in row['demands'] if demand[metric] is not None]
        if eligible:
            _, row, demand = max(eligible, key=lambda item: item[0])
            selected.append((metric, row, demand))
    return selected


def channel_extrema(rows):
    selected = []
    for _, row, _ in channel_critical_cases(rows):
        if row not in selected:
            selected.append(row)
    return selected


def channel_pose_collection(report):
    if report['status'] != 'static_comparison_complete_not_flight_qualified':
        raise ValueError('Pose selection requires a completed static comparison')
    poses = {}
    for result in report['results']:
        indexed_rows = [(stage, index, row)
            for stage in ('coarse', 'spatial', 'angular_height_refinement')
            for index, row in enumerate(result[stage])]
        locations = {id(row): (stage, index) for stage, index, row in indexed_rows}
        rows = [row for _, _, row in indexed_rows]
        for scope, subset in (('all_poses', rows), ('level_only', [row for row in rows if row['tilt_fraction'] == 0])):
            for criterion, row, demand in channel_critical_cases(subset):
                phase, gap, yaw, tilt, direction = channel_case_specification(row)
                key = (*phase, gap, yaw, tilt, direction)
                if key not in poses:
                    poses[key] = dict(phase=list(phase), gap_m=gap, yaw_deg=yaw,
                        tilt_fraction=tilt, tilt_direction_deg=direction, origins=[])
                stage, index = locations[id(row)]
                poses[key]['origins'].append(dict(candidate=result['name'], stage=stage, row_index=index,
                    scope=scope, criterion=criterion, action=None if demand is None else demand['action'],
                    observed_value=row['augmented_rank'] if demand is None else demand[criterion]))
    if not poses:
        raise ValueError('Source comparison contains no selectable poses')
    return list(poses.values())


def collect_channel_poses(source_report, output):
    if output.exists():
        raise ValueError('Preserve existing pose collection evidence')
    report = json.loads(source_report.read_text())
    collection = dict(status='screening_seed_not_coverage_certificate',
        source_report_sha256=source_fingerprints(source_report),
        selection_source_sha256=source_fingerprints(__file__),
        support_window_mm=report['support_window_mm'],
        coordinates='Phase is a fraction of the candidate electrical repeat in its lattice axes. '
            'Yaw and tilt-axis azimuth are world-frame angles. Gap is the upright reference '
            'magnet-bottom to top-coil separation; tilt about the centre of mass changes rim clearance. '
            'Tilt fraction multiplies the existing rim-clearance limit.',
        evaluation='Recompute all channel_wrench_targets at every pose for each candidate; '
            'origin actions and observed values explain selection and are not new acceptance limits.',
        limitations=['Selected from recorded independent-square comparisons; other layouts can have different worst poses.',
            'Normalized phase transfers relative placement, not identical physical world coordinates.',
            'A passing seed screen requires further weakness search and finalist verification.',
            'Static authority only; inherited source-report limitations remain applicable.'],
        poses=channel_pose_collection(report))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(collection, indent=Fixed.report_precision) + '\n')
    return collection


def channel_case_summary(rows):
    demands = [demand for row in rows for demand in row['demands']]
    losses = [demand['copper_loss_W'] for demand in demands if demand['copper_loss_W'] is not None]
    return dict(pose_count=len(rows), deficient_pose_count=sum(row['augmented_rank'] < 7 for row in rows),
        over_budget_demand_count=sum(not demand['current_reserve_met'] for demand in demands),
        peak_required_current_A=max((demand['minimum_peak_current_A'] for demand in demands), default=None),
        peak_feasible_copper_loss_W=max(losses, default=None),
        sampled_current_reserve_met=bool(demands) and all(row['augmented_rank'] == 7 for row in rows)
            and all(demand['current_reserve_met'] for demand in demands))


def channel_sweep_candidates():
    candidates = []
    for pattern, sizes in (('square', Inputs.channel_sweep_square_sizes), ('herringbone', Inputs.channel_sweep_parquet_sizes)):
        connections = Inputs.channel_sweep_connections if pattern == 'square' else ('independent',)
        for size in sizes:
            for spacing in Inputs.channel_sweep_spacings:
                for angle in Fixed.layout_screen_rotations_deg:
                    for connection in connections:
                        candidates.append(dict(name=f'{pattern}_{size:g}_gap_{spacing:g}_angle_{angle:g}_{connection}',
                            pattern=pattern, width_mm=size, length_mm=size * (2 if pattern == 'herringbone' else 1),
                            spacing_mm=spacing, rotation_deg=angle, connection=connection))
    names = [candidate['name'] for candidate in candidates]
    if len(set(names)) != len(names):
        raise ValueError('Duplicate sweep configurations')
    return candidates


def screen_channel_candidate(job):
    from time import perf_counter
    from methodology import prototype_coils
    candidate, collection, geometry = job
    started = perf_counter()
    levitation_sim.use_geometry(geometry)
    magnet = levitation_sim.magnet_layout_from_geometry()
    coil, winding = prototype_coils.round_wire_geometry(candidate['width_mm'], candidate['length_mm'])
    array = LayoutArray(candidate['pattern'], coil, winding, candidate['rotation_deg'],
        min(Fixed.current_demand_mesh_refinements), candidate['spacing_mm'])
    array.window = collection['support_window_mm'] / 1000
    result = dict(candidate=candidate, channels_per_m2=channel_density(array, candidate['connection']),
        electrical_repeat_mm=channel_repeat(array, candidate['connection']) * 1000,
        winding_resistance_20C_ohm=winding.resistance, turns=winding.turns, winding_height_mm=winding.coil_height,
        rows=[], status='screen_pass_not_qualified')
    for index, pose in enumerate(collection['poses']):
        row, _ = channel_pose_case(array, winding, geometry, magnet,
            *channel_case_specification(pose), connection=candidate['connection'])
        row['collection_pose_index'] = index
        result['rows'].append(row)
        if row['augmented_rank'] < 7:
            result['status'] = 'rejected_rank'
            break
        if any(not demand['current_reserve_met'] for demand in row['demands']):
            result['status'] = 'rejected_current_budget'
            break
    result.update(summary=channel_case_summary(result['rows']),
        unevaluated_pose_count=len(collection['poses']) - len(result['rows']), elapsed_s=perf_counter() - started)
    return result


def channel_sweep_frontier(results):
    passed = [result for result in results if result['status'] == 'screen_pass_not_qualified']
    def objectives(result):
        return np.array([result['channels_per_m2'], result['summary']['peak_required_current_A'],
            result['summary']['peak_feasible_copper_loss_W']])
    return [result['candidate']['name'] for result in passed if not any(
        np.all(objectives(other) <= objectives(result)) and np.any(objectives(other) < objectives(result))
        for other in passed)]


def run_channel_sweep(collection_path, output):
    from time import perf_counter
    from zipfile import ZipFile, ZIP_DEFLATED
    from methodology import prototype_coils
    from methodology.cad import magnetics
    if output.exists():
        raise ValueError('Preserve existing channel sweep evidence')
    collection = json.loads(collection_path.read_text())
    if collection['status'] != 'screening_seed_not_coverage_certificate':
        raise ValueError('Sweep requires a retained hard-pose collection')
    if collection['support_window_mm'] not in Fixed.channel_screen_window_sides:
        raise ValueError('Use a configured channel support window')
    candidates = channel_sweep_candidates()
    for candidate in candidates:
        coil, winding = prototype_coils.round_wire_geometry(candidate['width_mm'], candidate['length_mm'])
        array = LayoutArray(candidate['pattern'], coil, winding, candidate['rotation_deg'],
            min(Fixed.current_demand_mesh_refinements), candidate['spacing_mm'])
        validate_layout(array)
    sources = magnetics.fingerprints()
    report = dict(status='incomplete', source_sha256=sources, pose_collection_sha256=source_fingerprints(collection_path),
        candidate_count=len(candidates), support_window_mm=collection['support_window_mm'],
        reserved_current_A=Fixed.driver_channel_current * (1 - Fixed.qualification_current_reserve_fraction),
        results=[], limitations=[
            'Seven-pose seed screening is not continuous-domain coverage; passing designs need a weakness search.',
            'Cases use normalized electrical phase; world position differs by pitch, orientation and grouping.',
            'Same nominal CAD king and rounded closed-turn wire model, with fixed wire and turn count for all designs.',
            'Static zero-speed single-piece force/torque requests; no combined extremes, neighbours or dynamic control.',
            'Rejection stops the candidate; reported maxima for rejected candidates cover only the evaluated prefix.',
            'Loss excludes infeasible demands. No heating acceptance limit is invented; loss is a comparison objective.',
            'Density, current and peak feasible copper loss are screening objectives, not manufacturing cost.',
            'Continuous wiring, magnetic refinement, variation, transients, contact, handoffs and thermal workloads remain unqualified.'])
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(name, name)
        archive.write(collection_path, 'hard_poses.json')
    summary_path = output / 'summary.json'
    summary_path.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    started = perf_counter()
    construction = magnetics.ConstructionModel()
    geometry, _, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    report['piece_properties'] = construction.piece_properties
    with ProcessPoolExecutor(max_workers=min(len(candidates), Fixed.qualification_parallel_workers)) as pool:
        futures = [pool.submit(screen_channel_candidate, (candidate, collection, geometry)) for candidate in candidates]
        for future in as_completed(futures):
            result = future.result()
            report['results'].append(result)
            report['elapsed_s'] = perf_counter() - started
            summary_path.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    report['results'].sort(key=lambda result: result['candidate']['name'])
    report.update(status='seed_sweep_complete_not_qualified', elapsed_s=perf_counter() - started,
        nondominated_candidates=channel_sweep_frontier(report['results']))
    summary_path.write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    print(json.dumps(dict(status=report['status'], candidates=len(candidates),
        passed=sum(result['status'] == 'screen_pass_not_qualified' for result in report['results']),
        elapsed_s=report['elapsed_s'])), flush=True)
    return report


def compare_channel_candidate(job):
    from time import perf_counter
    from methodology import prototype_coils, physics_verification
    from methodology.cad import magnetics
    candidate_spec, window_mm = job
    name, layout_name, angle, spacing, connection = candidate_spec
    if connection != 'independent':
        raise ValueError('Broader comparison currently requires independently driven windings')
    started = perf_counter()
    construction = magnetics.ConstructionModel()
    geometry, _, _ = construction(Inputs.cad_coil_layout, min(Fixed.current_demand_mesh_refinements))
    magnet = levitation_sim.magnet_layout_from_geometry()
    candidate, coil, winding = prototype_coils.prototype_geometry(layout_name)
    arrays = {}
    for refinement in Fixed.current_demand_mesh_refinements:
        array = LayoutArray(candidate[1], coil, winding, angle, refinement, spacing)
        array.window = window_mm / 1000
        arrays[refinement] = array
    coarse = arrays[min(arrays)]
    refined = arrays[max(arrays)]
    result = dict(name=name, geometry=layout_name, support_window_mm=window_mm, coarse=[], spatial=[], magnetic=[])
    tilts = [(0., 0.)] + [(max(Fixed.pose_tilt_fractions), direction) for direction in
        np.arange(Fixed.horizontal_force_directions) * 360 / Fixed.horizontal_force_directions]
    for gap in (geometry.gap, geometry.max_flight_gap):
        for yaw in Fixed.channel_screen_yaws_deg:
            for tilt_fraction, direction in tilts:
                for phase in Fixed.channel_pilot_phase_points:
                    row, _ = channel_pose_case(coarse, winding, geometry, magnet, phase, gap, yaw, tilt_fraction, direction)
                    result['coarse'].append(row)
    result['coarse_summary'] = channel_case_summary(result['coarse'])
    phases = np.arange(Fixed.layout_refinement_phase_samples) / Fixed.layout_refinement_phase_samples
    sections = set()
    for worst in channel_extrema(result['coarse']):
        section = channel_case_specification(worst)[1:]
        if section in sections:
            continue
        sections.add(section)
        for x in phases:
            for y in phases:
                row, _ = channel_pose_case(coarse, winding, geometry, magnet, (x, y), *section)
                result['spatial'].append(row)
    result['spatial_summary'] = channel_case_summary(result['spatial'])
    angular = []
    for worst in channel_extrema(result['coarse'] + result['spatial']):
        phase, gap, yaw, tilt_fraction, direction = channel_case_specification(worst)
        yaw_step = (Fixed.channel_screen_yaws_deg[1] - Fixed.channel_screen_yaws_deg[0]) / Fixed.qualification_refinement_factor
        direction_step = 360 / Fixed.horizontal_force_directions / Fixed.qualification_refinement_factor
        for yaw_offset in (-yaw_step, 0., yaw_step):
            for fraction in sorted(set((tilt_fraction, *Fixed.pose_tilt_fractions))):
                for tilt_offset in (-direction_step, 0., direction_step):
                    row, _ = channel_pose_case(coarse, winding, geometry, magnet, phase,
                        (geometry.gap + geometry.max_flight_gap) / 2, (yaw + yaw_offset) % 360,
                        fraction, (direction + tilt_offset) % 360)
                    angular.append(row)
    result['angular_height_refinement'] = angular
    all_rows = result['coarse'] + result['spatial'] + angular
    result['sampled_summary'] = channel_case_summary(all_rows)
    for worst in channel_extrema(all_rows):
        specification = channel_case_specification(worst)
        original, original_matrix = channel_pose_case(coarse, winding, geometry, magnet, *specification)
        fine, fine_matrix = channel_pose_case(refined, winding, geometry, magnet, *specification)
        metrics = physics_verification.matrix_refinement_metrics(original_matrix, fine_matrix, Fixed.driver_channel_current)
        changes = {}
        for metric in ('minimum_peak_current_A', 'copper_loss_W'):
            eligible = [(a[metric], b[metric]) for a, b in zip(original['demands'], fine['demands'])
                if a[metric] is not None and b[metric] is not None]
            changes[metric] = max((abs(a / b - 1) for a, b in eligible), default=None)
        classification_same = (original['augmented_rank'] == fine['augmented_rank'] and
            [d['current_reserve_met'] for d in original['demands']] == [d['current_reserve_met'] for d in fine['demands']])
        errors = [metrics['force_relative_column_error'], metrics['torque_relative_column_error'],
            *[change for change in changes.values() if change is not None]]
        result['magnetic'].append(dict(original=original, refined=fine, matrix_errors=metrics,
            relative_changes=changes, classification_same=classification_same,
            passed=classification_same and max(errors) <= Fixed.qualification_convergence_fraction))
    result['magnetic_checks_passed'] = all(row['passed'] for row in result['magnetic'])
    result['elapsed_s'] = perf_counter() - started
    return result


def run_channel_comparison(output, window_mm):
    from time import perf_counter
    from zipfile import ZipFile, ZIP_DEFLATED
    from methodology import physics_verification
    from methodology.cad import magnetics
    if output.exists():
        raise ValueError('Preserve existing channel comparison evidence')
    if window_mm not in Fixed.channel_screen_window_sides:
        raise ValueError('Use a configured channel support window')
    candidates = [row for row in Inputs.channel_layout_candidates if row[0] in Inputs.channel_comparison_candidates]
    if len(candidates) != len(Inputs.channel_comparison_candidates):
        raise ValueError('Comparison candidate not found')
    sources = dict(magnetics.fingerprints(), **source_fingerprints(physics_verification.__file__))
    report = dict(status='incomplete', scope='Static single-piece comparison across phase, gap, yaw and tilt',
        source_sha256=sources, support_window_mm=window_mm, results=[],
        limitations=['Shared rounded closed-turn geometry approximation; not production continuous-conductor qualification.',
            'Nominal CAD mass, inertia, magnetic strength and winding resistance; no manufacturing or temperature uncertainty.',
            'Static zero-speed demands include world-frame inertia but no gyro torque, drag, sensing, switching or current transients.',
            'Coarse phase samples plus dense slices and local angular/height refinement do not bound the continuous pose domain.',
            'Peak feasible loss excludes out-of-budget demands; current failures remain explicit.',
            'No neighbours, contact reset, board edges, continuous window handoff or thermal acceptance.'])
    output.mkdir(parents=True)
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(name, name)
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    started = perf_counter()
    with ProcessPoolExecutor(max_workers=min(len(candidates), Fixed.qualification_parallel_workers)) as pool:
        for result in pool.map(compare_channel_candidate, [(row, window_mm) for row in candidates]):
            report['results'].append(result)
            (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    report.update(status='static_comparison_complete_not_flight_qualified', elapsed_s=perf_counter() - started)
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compare non-overlapping single-layer coils with the stacked baseline.')
    parser.add_argument('--name', action='append')
    parser.add_argument('--refine', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--compare-channels', action='store_true', help='Compare the shortlisted independent layouts across pose and numerical refinements.')
    parser.add_argument('--sweep-channels', type=Path, help='Screen the configured parameter grid against a retained pose collection; output is a fresh directory.')
    parser.add_argument('--collect-hard-poses', type=Path, help='Select reusable difficult poses from a saved static comparison without rerunning physics.')
    parser.add_argument('--channels', action='store_true', help='Run the isolated round-wire channel-density pilot; output must be a fresh directory.')
    parser.add_argument('--window-mm', type=float, choices=Fixed.channel_screen_window_sides, default=Fixed.layout_screen_window_side)
    parser.add_argument('--samples', type=int, default=Fixed.layout_screen_phase_samples)
    parser.add_argument('--refinement', type=int, default=min(Fixed.current_demand_mesh_refinements))
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('results') / 'coil_layouts.json')
    arguments = parser.parse_args()
    if arguments.sweep_channels:
        if arguments.collect_hard_poses or arguments.compare_channels or arguments.channels or arguments.refine or arguments.check or arguments.name:
            parser.error('Channel sweep owns its candidate grid and uses the supplied pose collection')
        run_channel_sweep(arguments.sweep_channels, arguments.output)
    elif arguments.collect_hard_poses:
        if arguments.compare_channels or arguments.channels or arguments.refine or arguments.check or arguments.name:
            parser.error('Pose collection only selects evidence from its source report')
        collection = collect_channel_poses(arguments.collect_hard_poses, arguments.output)
        print(json.dumps(dict(status=collection['status'], poses=len(collection['poses']))))
    elif arguments.compare_channels:
        if arguments.channels or arguments.refine or arguments.check or arguments.name:
            parser.error('Channel comparison owns its shortlist and sampling')
        run_channel_comparison(arguments.output, arguments.window_mm)
    elif arguments.channels:
        if arguments.refine or arguments.check or arguments.name:
            parser.error('Channel pilot owns its candidate list and samples')
        run_channel_pilot(arguments.output, arguments.window_mm)
    elif arguments.check:
        check_shortlist(arguments.output)
    else:
        run_screen(arguments.output, arguments.name, arguments.samples, arguments.refinement, arguments.refine)

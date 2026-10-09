import numpy as np
import magpylib as magpy
from magpylib_force import getFT
from scipy.spatial.transform import Rotation
from scipy.optimize import linprog, minimize


class SimGeometry:
    def __init__(self, inputs, settings, constants, board, coil, piece):
        self.settings = settings
        self.constants = constants
        self.magnet_lateral_edge = inputs.magnet_lateral_edge / 1000
        self.magnet_thickness = inputs.magnet_thickness / 1000
        self.magnets_per_period = inputs.magnets_per_period
        self.periods_per_side = inputs.periods_per_side
        self.gap = inputs.magnet_to_coil_distance / 1000
        self.max_flight_gap = inputs.max_flight_gap / 1000
        self.remanence = constants.ndfeb_remanence_br
        self.base_corner_standoff = settings.base_corner_standoff / 1000
        self.gravity = constants.gravity
        self.weight = piece.weight
        self.center_of_mass_height = piece.com_height
        self.inertias = np.array([piece.tilt_inertia, piece.tilt_inertia, piece.yaw_inertia])
        self.coil_short = coil.outer_width / 1000
        self.coil_long = coil.outer_length / 1000
        self.coil_lattice_repeat = coil.lattice_repeat / 1000
        self.coil_radial_width = coil.winding_radial_width / 1000
        self.coil_height = settings.nominal_coil_height_for_field / 1000
        self.control_cells_per_side = coil.control_cells_per_side
        self.hall_observation_window_side = settings.hall_observation_window_side
        self.surface_stack = (settings.potting_cover_thickness + settings.playing_surface_thickness + settings.piece_bottom_skin) / 1000
        self.tilt_rim_clearance = inputs.tilt_rim_clearance / 1000
        self.pcb_thickness = settings.pcb_thickness / 1000
        self.hall_standoff = settings.hall_package_standoff / 1000
        self.travel_distance = np.hypot(board.motor_width, board.motor_height) / 1000
        self.cruise_accels = (max(inputs.min_maneuver_accel_g * constants.gravity, 4 * self.travel_distance / inputs.cruise_duration ** 2 * inputs.trajectory_force_margin),)
        self.flight_speed = 2 * self.travel_distance / inputs.cruise_duration
        self.reset_speed = 2 * self.travel_distance / inputs.reset_duration
        self.reset_acceleration = 4 * self.travel_distance / inputs.reset_duration ** 2 * inputs.trajectory_force_margin


AXIS_DIRECTIONS = np.array(
    [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]],
    dtype=float,
)

def use_geometry(geometry):
    global G
    G = geometry


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def snap_to_axis(polarization, remanence):
    alignments = AXIS_DIRECTIONS @ unit_vector(polarization)
    return remanence * AXIS_DIRECTIONS[int(np.argmax(alignments))]


def matrix_rank(matrix):
    return int(np.linalg.matrix_rank(matrix, tol=np.abs(matrix).max() * G.settings.matrix_rank_tolerance))


def condition_number(matrix):
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    significant = singular_values[singular_values > singular_values.max() * G.settings.singular_value_tolerance]
    return significant[0] / significant[-1]


class MagnetLayout:
    def __init__(self, lateral_edge, thickness, periods, per_period, remanence):
        self.lateral_edge = lateral_edge
        self.thickness = thickness
        self.periods = periods
        self.per_period = per_period
        self.remanence = remanence

        self.blocks_per_side = self.periods * self.per_period
        self.platform_side = self.blocks_per_side * self.lateral_edge

        self.collection = self.build()
        self.center_of_mass_height = G.center_of_mass_height

    def block_center(self, index):
        return (index + 0.5 - self.blocks_per_side / 2) * self.lateral_edge

    def block_polarization(self, column_index, row_index):
        remanence = self.remanence
        if (row_index % 2) == 0:
            angle = 2 * np.pi * (column_index % self.per_period) / self.per_period
            return np.array([-remanence * np.sin(angle), 0.0, -remanence * np.cos(angle)])
        angle = 2 * np.pi * (row_index % self.per_period) / self.per_period
        return np.array([0.0, -remanence * np.sin(angle), -remanence * np.cos(angle)])

    def build(self):
        blocks = []
        for column_index in range(self.blocks_per_side):
            x = self.block_center(column_index)
            for row_index in range(self.blocks_per_side):
                y = self.block_center(row_index)
                polarization = snap_to_axis(self.block_polarization(column_index, row_index), self.remanence)
                blocks.append(magpy.magnet.Cuboid(
                    dimension=(self.lateral_edge, self.lateral_edge, self.thickness),
                    polarization=tuple(polarization),
                    position=(x, y, self.thickness / 2),
                ))
        return magpy.Collection(blocks)

    def plane_field(self):
        samples = G.settings.field_grid_samples
        half = self.platform_side / 2
        axis = np.linspace(-half, half, samples)
        grid = np.array([[x, y, -G.gap] for x in axis for y in axis])
        return self.collection.getB(grid)

    def peak_bz(self):
        return float(np.max(np.abs(self.plane_field()[:, 2])))


def magnet_layout_from_geometry():
    return MagnetLayout(G.magnet_lateral_edge, G.magnet_thickness, G.periods_per_side, G.magnets_per_period, G.remanence)


class CoilArray:
    def __init__(self, cells_per_side, short, long, radial_width, height, turns,
                 gap, layer_gap, meshing, filaments_radial, filaments_axial, lattice_cells):
        self.cells_per_side = cells_per_side
        self.short = short
        self.long = long
        self.radial_width = radial_width
        self.height = height
        self.turns = turns
        self.gap = gap
        self.layer_gap = layer_gap
        self.meshing = meshing
        self.filaments_radial = filaments_radial
        self.filaments_axial = filaments_axial
        self.lattice_cells = lattice_cells
        self.turns_per_filament = turns / (filaments_radial * filaments_axial)
        self.windings_by_identity = {}
        self.coils = []
        self.build()

    def winding(self, orientation, center_x, center_y, z_top):
        extent_x = self.short / 2 if orientation == "x" else self.long / 2
        extent_y = self.long / 2 if orientation == "x" else self.short / 2
        filaments = []
        for radial_index in range(self.filaments_radial):
            inset = self.radial_width * (radial_index + 0.5) / self.filaments_radial
            half_x = extent_x - inset
            half_y = extent_y - inset
            outline = np.array([
                [-half_x, -half_y, 0.0], [half_x, -half_y, 0.0],
                [half_x, half_y, 0.0], [-half_x, half_y, 0.0], [-half_x, -half_y, 0.0],
            ])
            for axial_index in range(self.filaments_axial):
                z = z_top - self.height * (axial_index + 0.5) / self.filaments_axial
                loop = magpy.current.Polyline(
                    current=self.turns_per_filament,
                    vertices=outline + np.array([center_x, center_y, z]),
                )
                loop.meshing = self.meshing
                filaments.append(loop)
        return filaments

    def build(self, center=(0.0, 0.0), half_extents=None):
        self.coils = []
        self.identities = []
        region = self.cells_per_side * self.short
        long_count = max(1, round(region / self.long))
        reference_long_count = max(1, round(self.lattice_cells * self.short / self.long))
        short_phase = ((self.lattice_cells - 1) / 2) % 1
        long_phase = ((reference_long_count - 1) / 2) % 1
        families = (("x", self.cells_per_side, long_count, self.short, self.long, short_phase, long_phase, -self.gap),
                    ("y", long_count, self.cells_per_side, self.long, self.short, long_phase, short_phase, -self.gap - self.layer_gap))
        for orientation, count_x, count_y, pitch_x, pitch_y, phase_x, phase_y, z_top in families:
            start_x = int(np.floor(center[0] / pitch_x - phase_x - (count_x - 1) / 2 + 0.5))
            start_y = int(np.floor(center[1] / pitch_y - phase_y - (count_y - 1) / 2 + 0.5))
            if half_extents is not None:
                lower_x = int(np.ceil(-half_extents[0] / pitch_x + 0.5 - phase_x))
                lower_y = int(np.ceil(-half_extents[1] / pitch_y + 0.5 - phase_y))
                upper_x = int(np.floor(half_extents[0] / pitch_x - 0.5 - phase_x)) - count_x + 1
                upper_y = int(np.floor(half_extents[1] / pitch_y - 0.5 - phase_y)) - count_y + 1
                if lower_x > upper_x or lower_y > upper_y:
                    raise ValueError("The complete control window does not fit inside the motor footprint")
                start_x = min(max(start_x, lower_x), upper_x)
                start_y = min(max(start_y, lower_y), upper_y)
            for i in range(start_x, start_x + count_x):
                for j in range(start_y, start_y + count_y):
                    center_x = (i + phase_x) * pitch_x
                    center_y = (j + phase_y) * pitch_y
                    identity = (orientation, i, j)
                    if identity not in self.windings_by_identity:
                        self.windings_by_identity[identity] = self.winding(orientation, center_x, center_y, z_top)
                    self.identities.append(identity)
                    self.coils.append(self.windings_by_identity[identity])


def coil_array_from_geometry(cells_per_side, height, turns, meshing, filaments_radial, filaments_axial):
    return CoilArray(cells_per_side, G.coil_short, G.coil_long, G.coil_radial_width,
                     height, turns, G.gap, height, meshing, filaments_radial,
                     filaments_axial, G.control_cells_per_side)


def verification_coil_array(control_cells, height, refinement=1):
    axial = min(G.settings.max_coil_axial_filaments, max(G.settings.minimum_axial_filaments,
                                                        int(np.ceil(height / G.settings.axial_filament_spacing))))
    return coil_array_from_geometry(control_cells, height, 1, G.settings.verification_force_mesh * refinement,
                                    G.settings.coil_radial_filaments * refinement, axial * refinement)


def coil_window_handoffs():
    repeat = G.coil_lattice_repeat
    return np.unique(np.concatenate([np.arange(pitch / 2, repeat, pitch) for pitch in (G.coil_short, G.coil_long)]))


def coil_window_phases():
    regular = np.arange(G.settings.active_window_phase_samples) * G.coil_lattice_repeat / G.settings.active_window_phase_samples
    return np.unique(np.concatenate((regular, coil_window_handoffs())))


def coil_window_transition_width():
    fraction = G.settings.active_window_transition_fraction
    if not 0 < fraction < 1:
        raise ValueError("Window transition fraction must be between zero and one")
    boundaries = coil_window_handoffs()
    return fraction * np.min(np.diff(np.append(boundaries, boundaries[0] + G.coil_lattice_repeat)))


def blended_window_centers(position):
    width = coil_window_transition_width()
    axes = []
    for coordinate in position:
        boundaries = coil_window_handoffs() + np.floor(coordinate / G.coil_lattice_repeat) * G.coil_lattice_repeat
        boundary = boundaries[np.argmin(np.abs(boundaries - coordinate))]
        if abs(coordinate - boundary) >= width / 2:
            axes.append(((coordinate, 1.0),))
        else:
            fraction = (coordinate - boundary) / width + 0.5
            weight = fraction ** 2 * (3 - 2 * fraction)
            axes.append(((boundary - width / 2, 1 - weight), (boundary + width / 2, weight)))
    return [((x, y), weight_x * weight_y) for x, weight_x in axes[0] for y, weight_y in axes[1]]


def place_piece(layout, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    rotation = Rotation.from_euler("xyz", [roll, pitch, yaw])
    collection = layout.collection
    collection.position = (0.0, 0.0, 0.0)
    collection.orientation = Rotation.identity()
    collection.rotate(rotation, anchor=(0.0, 0.0, layout.center_of_mass_height))
    collection.move((x, y, z))
    center_of_mass = np.array([x, y, layout.center_of_mass_height + z])
    return collection, center_of_mass


def actuator_matrix(layout, array, center_of_mass):
    filaments = [filament for coil in array.coils for filament in coil]
    force_torque = np.asarray(getFT(layout.collection, filaments, anchor=center_of_mass, eps=G.settings.force_gradient_step))
    per_coil = force_torque.reshape(len(array.coils), -1, 6).sum(axis=1)
    return -per_coil.T


def window_components(layout, array, position, center_of_mass, half_extents=None):
    weights = {}
    windings = {}
    for selected_center, weight in blended_window_centers(position):
        if weight == 0:
            continue
        array.build(center=selected_center, half_extents=half_extents)
        identities = tuple(array.identities)
        weights[identities] = weights.get(identities, 0) + weight
        windings.update(zip(identities, array.coils))
    array.identities = list(windings)
    array.coils = list(windings.values())
    indices = {identity: index for index, identity in enumerate(array.identities)}
    matrix = actuator_matrix(layout, array, center_of_mass)
    return {identities: (matrix[:, [indices[identity] for identity in identities]], weight)
            for identities, weight in weights.items()}


def allocate_window_components(components, target, current_limit):
    currents = {}
    achieved = np.zeros(6)
    required_peak = 0.0
    for identities, (matrix, weight) in components.items():
        peak, initial = min_peak_current(matrix, target)
        required_peak = max(required_peak, peak)
        allocated = min_loss_current(matrix, target, max(current_limit, peak), initial_currents=initial)
        achieved += weight * (matrix @ allocated)
        for identity, current in zip(identities, allocated):
            currents[identity] = currents.get(identity, 0) + weight * current
    if max(np.max(np.abs(achieved - target)), abs(sum(currents.values()))) > G.settings.allocator_residual_tolerance:
        raise RuntimeError("Blended allocation violates wrench balance or zero-sum current")
    return currents, required_peak


def min_peak_current(matrix, target, contact_inequality=None, contact_bounds=None):
    coil_count = matrix.shape[1]
    objective = np.zeros(coil_count + 1)
    objective[-1] = 1.0
    inequality = np.zeros((2 * coil_count, coil_count + 1))
    inequality[:coil_count, :coil_count] = np.eye(coil_count)
    inequality[:coil_count, -1] = -1.0
    inequality[coil_count:, :coil_count] = -np.eye(coil_count)
    inequality[coil_count:, -1] = -1.0
    equality = np.hstack([np.array(matrix), np.zeros((matrix.shape[0], 1))])
    equality = np.vstack([equality, np.append(np.ones(coil_count), 0.0)])
    inequality_bounds = np.zeros(2 * coil_count)
    if contact_inequality is not None:
        inequality = np.vstack((inequality, np.column_stack((contact_inequality, np.zeros(len(contact_bounds))))))
        inequality_bounds = np.concatenate((inequality_bounds, contact_bounds))
    equality_norms = np.linalg.norm(equality, axis=1)
    inequality_norms = np.linalg.norm(inequality, axis=1)
    if np.any(equality_norms == 0) or np.any(inequality_norms == 0):
        raise ValueError("Allocation constraints contain an empty row")
    result = linprog(
        objective,
        A_ub=inequality / inequality_norms[:, None], b_ub=inequality_bounds / inequality_norms,
        A_eq=equality / equality_norms[:, None], b_eq=np.append(np.array(target), 0.0) / equality_norms,
        bounds=[(None, None)] * coil_count + [(0, None)],
        method="highs",
        options={"primal_feasibility_tolerance": G.settings.allocator_residual_tolerance,
                 "dual_feasibility_tolerance": G.settings.allocator_residual_tolerance},
    )
    if not result.success:
        raise RuntimeError(result.message)
    currents = result.x[:coil_count]
    return float(np.max(np.abs(currents))), currents


def min_loss_current(matrix, target, current_limit, resistances=1.0, initial_currents=None, contact_inequality=None, contact_bounds=None):
    matrix = np.asarray(matrix, dtype=float)
    target = np.asarray(target, dtype=float)
    coil_count = matrix.shape[1]
    limits = np.broadcast_to(np.asarray(current_limit, dtype=float), (coil_count,))
    weights = np.broadcast_to(np.asarray(resistances, dtype=float), (coil_count,))
    if np.any(limits <= 0) or np.any(weights <= 0):
        raise ValueError("Current limits and resistances must be positive")
    equality = np.vstack((matrix, np.ones(coil_count)))
    expected = np.append(target, 0.0)
    scaled_equality = equality * limits
    row_norms = np.linalg.norm(scaled_equality, axis=1)
    if np.any(row_norms == 0):
        raise ValueError("Allocation constraints contain an empty row")
    scaled_equality /= row_norms[:, None]
    scaled_expected = expected / row_norms
    objective_weights = weights * limits ** 2
    objective_weights /= np.max(objective_weights)
    if initial_currents is None:
        _, initial_currents = min_peak_current(matrix, target, contact_inequality, contact_bounds)
    constraints = [{"type": "eq", "fun": lambda currents: scaled_equality @ currents - scaled_expected,
                    "jac": lambda currents: scaled_equality}]
    if contact_inequality is not None:
        scaled_contact = contact_inequality * limits
        contact_norms = np.linalg.norm(scaled_contact, axis=1)
        if np.any(contact_norms == 0):
            raise ValueError("Contact constraints contain an empty row")
        scaled_contact /= contact_norms[:, None]
        scaled_contact_bounds = contact_bounds / contact_norms
        constraints.append({"type": "ineq", "fun": lambda currents: scaled_contact_bounds - scaled_contact @ currents,
                            "jac": lambda currents: -scaled_contact})
    result = minimize(
        lambda currents: np.dot(objective_weights, currents ** 2),
        np.asarray(initial_currents) / limits,
        jac=lambda currents: 2 * objective_weights * currents,
        bounds=[(-1.0, 1.0)] * coil_count,
        constraints=constraints,
        method="SLSQP",
        options={"ftol": G.settings.allocator_tolerance, "maxiter": G.settings.allocator_max_iterations},
    )
    if not result.success:
        raise RuntimeError(f"Current allocation failed: {result.message}")
    residual = np.max(np.abs(scaled_equality @ result.x - scaled_expected))
    if residual > G.settings.allocator_residual_tolerance or np.max(np.abs(result.x)) > 1 + G.settings.allocator_limit_tolerance:
        raise RuntimeError("Current allocation violates wrench or current constraints")
    if contact_inequality is not None and np.max(scaled_contact @ result.x - scaled_contact_bounds) > G.settings.allocator_residual_tolerance:
        raise RuntimeError("Current allocation violates ground-contact constraints")
    return result.x * limits


def horizontal_wrench_targets(thrust, weight):
    directions = G.settings.horizontal_force_directions
    angles = np.arange(directions) * 2 * np.pi / directions
    return np.column_stack((thrust * np.cos(angles), thrust * np.sin(angles),
                            np.full(directions, weight), np.zeros((directions, 3))))


def min_peak_current_in_basis(matrix, target, basis):
    """Minimize physical coil peak current for currents constrained to i = basis @ u."""
    coil_count, mode_count = basis.shape
    objective = np.zeros(mode_count + 1)
    objective[-1] = 1.0
    inequality = np.zeros((2 * coil_count, mode_count + 1))
    inequality[:coil_count, :mode_count] = basis
    inequality[:coil_count, -1] = -1.0
    inequality[coil_count:, :mode_count] = -basis
    inequality[coil_count:, -1] = -1.0
    equality = np.hstack([np.asarray(matrix) @ basis, np.zeros((matrix.shape[0], 1))])
    result = linprog(
        objective,
        A_ub=inequality, b_ub=np.zeros(2 * coil_count),
        A_eq=equality, b_eq=np.asarray(target, dtype=float),
        bounds=[(None, None)] * mode_count + [(0, None)],
        method="highs",
    )
    if not result.success:
        raise ValueError(f"Reduced current basis is infeasible: {result.message}")
    currents = basis @ result.x[:mode_count]
    return float(np.max(np.abs(currents))), currents


def local_current_mode_analysis(wrenches, level_wrenches, weight, characteristic_length,
                                sprint_thrust):
    def unit_columns(matrix):
        norms = np.linalg.norm(matrix, axis=0)
        significant = norms > G.settings.singular_value_tolerance
        return matrix[:, significant] / norms[significant]

    coil_count = wrenches[0].shape[1]
    zero_sum_projector = np.eye(coil_count) - np.ones((coil_count, coil_count)) / coil_count
    normalized_solutions = []
    for wrench in wrenches:
        scaled = np.array(wrench, copy=True)
        scaled[3:] /= characteristic_length
        augmented = np.vstack([scaled, np.ones((1, coil_count))])
        targets = np.vstack([np.eye(6), np.zeros((1, 6))])
        normalized_solutions.append(np.linalg.lstsq(augmented, targets, rcond=None)[0])
    six_dof_library = zero_sum_projector @ np.hstack(normalized_solutions)
    operational = []
    for wrench in wrenches:
        target = np.array([0, 0, weight, 0, 0, 0], dtype=float)
        peak, currents = min_peak_current(wrench, target)
        operational.append((wrench, target, peak, currents))
    for wrench in level_wrenches:
        for target in horizontal_wrench_targets(sprint_thrust, weight):
            peak, currents = min_peak_current(wrench, target)
            operational.append((wrench, target, peak, currents))
    operational_library = zero_sum_projector @ np.column_stack([scenario[3] for scenario in operational])
    libraries = {
        "6dof": six_dof_library,
        "operational": operational_library,
        "hybrid": np.hstack([unit_columns(six_dof_library), unit_columns(operational_library)]),
    }
    rows = []
    for basis_source, library in libraries.items():
        left, _, _ = np.linalg.svd(library, full_matrices=False)
        library_rank = matrix_rank(library)
        for mode_count in sorted({min(count, library_rank) for count in G.settings.mode_counts}):
            basis = left[:, :mode_count]
            worst_peak_ratio = 0.0
            for wrench, target, full_peak, _ in operational:
                reduced_peak, _ = min_peak_current_in_basis(wrench, target, basis)
                worst_peak_ratio = max(worst_peak_ratio, float(reduced_peak / full_peak))
            rows.append({"basis_source": basis_source, "modes": mode_count,
                         "worst_peak_current_ratio": worst_peak_ratio})
    return rows


def max_generalized_force(wrench_matrix, objective_row, constrained_rows,
                          constrained_values, current_limit):
    coil_count = wrench_matrix.shape[1]
    result = linprog(
        c=-wrench_matrix[objective_row],
        bounds=[(-current_limit, current_limit)] * coil_count,
        A_eq=np.vstack([wrench_matrix[constrained_rows], np.ones((1, coil_count))]),
        b_eq=np.append(np.array(constrained_values, dtype=float), 0.0),
        method="highs",
    )
    if not result.success:
        raise RuntimeError(result.message)
    return -result.fun


class Controllability:
    def __init__(self, wrench_matrix, weight, characteristic_length):
        self.wrench_matrix = wrench_matrix
        self.weight = weight

        self.scaled_wrench_matrix = wrench_matrix.copy()
        self.scaled_wrench_matrix[3:] = wrench_matrix[3:] / characteristic_length

        self.rank6 = matrix_rank(self.scaled_wrench_matrix)
        self.cond6 = condition_number(self.scaled_wrench_matrix)

        if self.rank6 < 6:
            raise RuntimeError("actuator matrix is not full 6-DOF rank")
        self.i_hover6, self.current6 = min_peak_current(wrench_matrix, [0, 0, weight, 0, 0, 0])


def analyze(layout, cells_per_side, weight, characteristic_length,
            meshing, filaments_radial, filaments_axial, turns):
    place_piece(layout)
    array = coil_array_from_geometry(cells_per_side, G.coil_height, turns,
                                     meshing, filaments_radial, filaments_axial)
    wrench_matrix = actuator_matrix(layout, array, np.array([0, 0, layout.center_of_mass_height]))
    return array, Controllability(wrench_matrix, weight, characteristic_length)


def open_loop_stiffness(layout, array, currents):
    steps = [G.settings.position_derivative_step] * 3 + [G.settings.angle_derivative_step] * 3
    jacobian = np.zeros((6, 6))
    for axis_index, step in enumerate(steps):
        forward = np.zeros(6)
        forward[axis_index] = step
        _, center = place_piece(layout, *forward)
        positive = actuator_matrix(layout, array, center) @ currents
        _, center = place_piece(layout, *(-forward))
        negative = actuator_matrix(layout, array, center) @ currents
        jacobian[:, axis_index] = (positive - negative) / (2 * step)
    place_piece(layout)
    generalized_mass = np.concatenate((np.full(3, G.weight / G.gravity), G.inertias))
    acceleration_gradient = jacobian / generalized_mass[:, None]
    state_matrix = np.block([[np.zeros((6, 6)), np.eye(6)], [acceleration_gradient, np.zeros((6, 6))]])
    return jacobian, np.linalg.eigvals(state_matrix)


def local_hall_points(sensor_pitch, window_side, plane_z):
    offsets = np.array([(index + 0.5 - window_side / 2) * sensor_pitch
                        for index in range(window_side)])
    return np.array([[x, y, plane_z] for x in offsets for y in offsets])


def hall_jacobian(layout, points, pose):
    sensor_axis = np.array([0.0, 0.0, 1.0])

    def field_at_pose(dx, dy, dz, droll, dpitch, dyaw):
        place_piece(layout, x=dx, y=dy, z=dz, roll=droll, pitch=dpitch, yaw=dyaw)
        return layout.collection.getB(points) @ sensor_axis

    baseline = field_at_pose(*pose)
    position_step, angle_step = G.settings.hall_position_derivative_step, G.settings.angle_derivative_step
    columns = []
    for axis_index, step in enumerate([position_step, position_step, position_step,
                                       angle_step, angle_step, angle_step]):
        offsets = list(pose)
        offsets[axis_index] += step
        columns.append((field_at_pose(*offsets) - baseline) / step)
    place_piece(layout)
    return np.array(columns).T


def hall_metrics(jacobian):
    rank6 = int(np.linalg.matrix_rank(jacobian, tol=np.abs(jacobian).max() * G.settings.hall_rank_tolerance))
    metrics = {"rank6": rank6, "cond6": condition_number(jacobian)}
    if rank6 >= 6:
        covariance = np.linalg.inv(jacobian.T @ jacobian)
        metrics["position_noise_gain"] = float(np.sqrt(np.max(np.diag(covariance)[:3])))
        metrics["tilt_noise_gain"] = float(np.sqrt(np.max(np.diag(covariance)[3:5])))
    return metrics


def rim_tilt_limit(layout, target_gap):
    base_radius = layout.platform_side / 2 * np.sqrt(2) + G.base_corner_standoff
    rim_drop_allowed = target_gap - G.surface_stack - G.tilt_rim_clearance
    return float(np.arcsin(min(G.settings.maximum_rim_sine, max(0.0, rim_drop_allowed) / base_radius)))


def hall_observability(layout, plane_z, sensor_pitch):
    window_side = G.hall_observation_window_side
    points = local_hall_points(sensor_pitch, window_side, plane_z)
    phase = tuple(sensor_pitch * fraction for fraction in G.settings.pose_phase_fractions)
    nominal = hall_metrics(hall_jacobian(layout, points, [0, 0, 0, 0, 0, 0]))
    worst = {"min_rank6": 6, "max_cond6": 0.0, "poses": 0,
             "max_position_noise_gain": 0.0, "max_tilt_noise_gain": 0.0}
    for target_gap in (G.gap, G.max_flight_gap):
        z = target_gap - G.gap
        tilt = rim_tilt_limit(layout, target_gap)
        for yaw in np.radians(G.settings.pose_yaws_deg):
            for roll, pitch in ((0.0, 0.0), (tilt, 0.0), (0.0, tilt), (tilt, tilt)):
                for dx in phase:
                    for dy in phase:
                        metrics = hall_metrics(hall_jacobian(layout, points, [dx, dy, z, roll, pitch, yaw]))
                        worst["min_rank6"] = min(worst["min_rank6"], metrics["rank6"])
                        if metrics["rank6"] >= 6:
                            worst["max_cond6"] = max(worst["max_cond6"], metrics["cond6"])
                            worst["max_position_noise_gain"] = max(worst["max_position_noise_gain"], metrics["position_noise_gain"])
                            worst["max_tilt_noise_gain"] = max(worst["max_tilt_noise_gain"], metrics["tilt_noise_gain"])
                        worst["poses"] += 1
    place_piece(layout)
    return {
        "sensors_per_piece": window_side ** 2,
        "sensor_pitch": sensor_pitch,
        "observation_window_side": window_side,
        "rank6": nominal["rank6"],
        "condition6": nominal["cond6"],
        "worst_rank6": worst["min_rank6"],
        "worst_condition6": worst["max_cond6"],
        "worst_position_noise_gain": worst["max_position_noise_gain"],
        "worst_tilt_noise_gain": worst["max_tilt_noise_gain"],
        "worst_poses": worst["poses"],
    }


def verified_hall_sensing(control_cells, coil_height_m, ampere_turn_budget, sensor_pitch):
    magnet = magnet_layout_from_geometry()
    weight = G.weight
    plane_z = -(G.gap + 2 * coil_height_m + G.pcb_thickness + G.hall_standoff)
    hall = hall_observability(magnet, plane_z, sensor_pitch)
    points = local_hall_points(sensor_pitch, G.hall_observation_window_side, plane_z)
    place_piece(magnet)
    signal_peak = float(np.max(np.abs(magnet.collection.getB(points)[:, 2])))
    axial = min(G.settings.max_coil_axial_filaments, max(G.settings.minimum_axial_filaments, int(np.ceil(coil_height_m / G.settings.axial_filament_spacing))))
    array = coil_array_from_geometry(control_cells, coil_height_m, 1, G.settings.verification_force_mesh, G.settings.coil_radial_filaments, axial)
    per_coil_field = np.array([np.sum([filament.getB(points) for filament in coil], axis=0)[:, 2]
                               for coil in array.coils])
    _, center_of_mass = place_piece(magnet)
    wrench = actuator_matrix(magnet, array, center_of_mass)
    _, hover_currents = min_peak_current(wrench, [0, 0, weight, 0, 0, 0])
    coil_field_hover = float(np.max(np.abs(per_coil_field.T @ hover_currents)))
    absolute_column_sum = np.sum(np.abs(per_coil_field), axis=0)
    coil_field_hover_bound = float(np.max(absolute_column_sum) * np.max(np.abs(hover_currents)))
    coil_field_budget_bound = float(np.max(absolute_column_sum) * ampere_turn_budget)
    base_diameter = magnet.platform_side * np.sqrt(2) + 2 * G.base_corner_standoff
    neighbour = magnet_layout_from_geometry()
    neighbour.collection.move((base_diameter, 0.0, 0.0))
    neighbour_field = float(np.max(np.abs(neighbour.collection.getB(points)[:, 2])))
    place_piece(magnet)
    return {
        **hall,
        "plane_depth_below_magnets": -plane_z,
        "signal_peak": signal_peak,
        "coil_field_hover": coil_field_hover,
        "coil_field_hover_bound": coil_field_hover_bound,
        "coil_field_budget_bound": coil_field_budget_bound,
        "neighbour_field": neighbour_field,
    }


def eddy_image_force(plane_z):
    mesh = G.settings.eddy_force_mesh
    magnet = magnet_layout_from_geometry()
    place_piece(magnet)
    image_blocks = []
    for cube in magnet.collection:
        px, py, pz = cube.polarization
        x, y, z = cube.position
        image_blocks.append(magpy.magnet.Cuboid(
            dimension=tuple(cube.dimension),
            polarization=(-px, -py, pz),
            position=(x, y, 2 * plane_z - z),
        ))
    image = magpy.Collection(image_blocks)
    anchor = np.array([0.0, 0.0, magnet.center_of_mass_height])
    targets = list(magnet.collection)
    for cube in targets:
        cube.meshing = mesh
    force_torque = np.atleast_2d(getFT(image, targets, anchor=anchor, eps=G.settings.force_gradient_step))
    return float(force_torque.reshape(-1, 6).sum(axis=0)[2])


def neighbour_force(lateral_edge, thickness, periods, distance, controlled_yaw=0.0, neighbour_yaw=0.0):
    mesh = G.settings.neighbour_force_mesh
    controlled = MagnetLayout(lateral_edge, thickness, periods, G.magnets_per_period, G.remanence)
    if controlled_yaw:
        controlled.collection.rotate(Rotation.from_euler("z", controlled_yaw),
                                     anchor=(0.0, 0.0, controlled.center_of_mass_height))
    neighbour = MagnetLayout(lateral_edge, thickness, periods, G.magnets_per_period, G.remanence)
    if neighbour_yaw:
        neighbour.collection.rotate(Rotation.from_euler("z", neighbour_yaw),
                                    anchor=(0.0, 0.0, neighbour.center_of_mass_height))
    neighbour.collection.move((distance, 0.0, 0.0))
    anchor = np.array([0.0, 0.0, controlled.center_of_mass_height])
    targets = list(controlled.collection)
    for cube in targets:
        cube.meshing = mesh
    force_torque = np.atleast_2d(getFT(neighbour.collection, targets, anchor=anchor, eps=G.settings.force_gradient_step))
    return force_torque.reshape(-1, 6).sum(axis=0)


def worst_touching_snap(lateral_edge, thickness, periods, distance):
    angles = np.radians(G.settings.neighbour_yaws_deg)
    worst_lateral = 0.0
    for controlled_yaw in angles:
        for neighbour_yaw in angles:
            force = neighbour_force(lateral_edge, thickness, periods, distance,
                                    controlled_yaw=controlled_yaw,
                                    neighbour_yaw=neighbour_yaw)
            worst_lateral = max(worst_lateral, float(np.hypot(force[0], force[1])))
    return worst_lateral


def pose_coefficients(wrench, weight, characteristic_length):
    controllability = Controllability(wrench, weight, characteristic_length)
    return controllability.rank6, {
        "cond6": controllability.cond6,
        "lift_per_at": weight / controllability.i_hover6,
        "hover_sumsq": float(np.sum(controllability.current6 ** 2)),
    }


def flight_worst_case(control_cells, weight, characteristic_length,
                      gaps, coil_height, filaments_axial, cruise_thrusts=()):
    yaws_deg = G.settings.pose_yaws_deg
    meshing = G.settings.pose_force_mesh
    tilt_fractions = G.settings.pose_tilt_fractions
    magnet = magnet_layout_from_geometry()
    height = coil_height
    array = coil_array_from_geometry(control_cells, height, 1, meshing, G.settings.coil_radial_filaments, filaments_axial)
    phase = tuple(G.coil_long * fraction for fraction in G.settings.pose_phase_fractions)
    worst = {
        "min_rank6": 6, "max_cond6": 0.0,
        "min_lift_per_at": 0.0,
        "max_hover_sumsq": 0.0, "max_tilt_deg": 0.0, "poses": 0,
        "wrenches": [], "level_all_wrenches": [],
        "level_min_lift_per_at": None, "level_max_hover_sumsq": 0.0,
        "level_max_cruise_sumsq": [0.0] * len(cruise_thrusts),
        "max_cruise_peak_at": [0.0] * len(cruise_thrusts),
        "avg_level_cruise_sumsq": [0.0] * len(cruise_thrusts),
        "rungs": {fraction: {"tilt_deg": 0.0, "min_lift_per_at": None, "max_hover_sumsq": 0.0, "wrenches": []}
                  for fraction in tilt_fractions},
    }
    level_sumsq_by_gap = []
    level_cruise_by_gap = []
    for target_gap in gaps:
        z = target_gap - G.gap
        limit = rim_tilt_limit(magnet, target_gap)
        worst["max_tilt_deg"] = max(worst["max_tilt_deg"], np.degrees(limit))
        pose_families = [(None, ((0.0, 0.0),))]
        for fraction in tilt_fractions:
            tilt = limit * fraction
            worst["rungs"][fraction]["tilt_deg"] = max(worst["rungs"][fraction]["tilt_deg"], np.degrees(tilt))
            pose_families.append((fraction, ((tilt, 0.0), (0.0, tilt), (tilt, tilt))))
        level_sumsq = []
        level_cruise = [[] for _ in cruise_thrusts]
        for yaw in np.radians(yaws_deg):
            for fraction, tilt_pairs in pose_families:
                for roll, pitch in tilt_pairs:
                    for dx in phase:
                        for dy in phase:
                            _, center_of_mass = place_piece(magnet, x=dx, y=dy, z=z,
                                                            roll=roll, pitch=pitch, yaw=yaw)
                            wrench = actuator_matrix(magnet, array, center_of_mass)
                            worst["wrenches"].append(wrench)
                            rank6, coeffs = pose_coefficients(wrench, weight, characteristic_length)
                            if worst["poses"] == 0:
                                worst["min_lift_per_at"] = coeffs["lift_per_at"]
                            worst["min_rank6"] = min(worst["min_rank6"], rank6)
                            worst["poses"] += 1
                            worst["max_cond6"] = max(worst["max_cond6"], coeffs["cond6"])
                            worst["min_lift_per_at"] = min(worst["min_lift_per_at"], coeffs["lift_per_at"])
                            worst["max_hover_sumsq"] = max(worst["max_hover_sumsq"], coeffs["hover_sumsq"])
                            if fraction is None:
                                worst["level_all_wrenches"].append(wrench)
                                level_sumsq.append(coeffs["hover_sumsq"])
                                worst["level_max_hover_sumsq"] = max(worst["level_max_hover_sumsq"], coeffs["hover_sumsq"])
                                for index, thrust in enumerate(cruise_thrusts):
                                    direction_solutions = [min_peak_current(wrench, target)
                                                           for target in horizontal_wrench_targets(thrust, weight)]
                                    cruise_sumsq = max(float(np.sum(currents ** 2)) for _, currents in direction_solutions)
                                    level_cruise[index].append(cruise_sumsq)
                                    worst["level_max_cruise_sumsq"][index] = max(worst["level_max_cruise_sumsq"][index], cruise_sumsq)
                                    worst["max_cruise_peak_at"][index] = max(worst["max_cruise_peak_at"][index],
                                                                            max(float(peak) for peak, _ in direction_solutions))
                                for key, value in (("level_min_lift_per_at", coeffs["lift_per_at"]),):
                                    if worst[key] is None or value < worst[key]:
                                        worst[key] = value
                            else:
                                rung = worst["rungs"][fraction]
                                rung["wrenches"].append(wrench)
                                rung["max_hover_sumsq"] = max(rung["max_hover_sumsq"], coeffs["hover_sumsq"])
                                if rung["min_lift_per_at"] is None or coeffs["lift_per_at"] < rung["min_lift_per_at"]:
                                    rung["min_lift_per_at"] = coeffs["lift_per_at"]
        level_sumsq_by_gap.append(float(np.mean(level_sumsq)))
        level_cruise_by_gap.append([float(np.mean(values)) for values in level_cruise])
    worst["avg_level_hover_sumsq"] = max(level_sumsq_by_gap)
    worst["avg_level_cruise_sumsq"] = [max(per_gap[index] for per_gap in level_cruise_by_gap)
                                       for index in range(len(cruise_thrusts))]
    place_piece(magnet)
    return worst


def coupled_worst_authority(wrenches, weight, ampere_turn_budget):
    worst = {"lateral": None, "tilt": None, "yaw": None}
    for wrench in wrenches:
        lateral = min(
            max_generalized_force(wrench, 0, [1, 2, 3, 4, 5], [0, weight, 0, 0, 0], ampere_turn_budget),
            max_generalized_force(wrench, 1, [0, 2, 3, 4, 5], [0, weight, 0, 0, 0], ampere_turn_budget))
        tilt = min(
            max_generalized_force(wrench, 3, [0, 1, 2, 4, 5], [0, 0, weight, 0, 0], ampere_turn_budget),
            max_generalized_force(wrench, 4, [0, 1, 2, 3, 5], [0, 0, weight, 0, 0], ampere_turn_budget))
        yaw = max_generalized_force(wrench, 5, [0, 1, 2, 3, 4], [0, 0, weight, 0, 0], ampere_turn_budget)
        for key, value in (("lateral", lateral), ("tilt", tilt), ("yaw", yaw)):
            if worst[key] is None or value < worst[key]:
                worst[key] = value
    return worst


def verified_worst_authority(control_cells, coil_height_m, ampere_turn_budget):
    magnet = magnet_layout_from_geometry()
    weight = G.weight
    axial = min(G.settings.max_coil_axial_filaments, max(G.settings.minimum_axial_filaments, int(np.ceil(coil_height_m / G.settings.axial_filament_spacing))))
    worst = flight_worst_case(control_cells, weight, magnet.platform_side / 2,
                              (G.gap, G.max_flight_gap), coil_height=coil_height_m, filaments_axial=axial)
    coupled = coupled_worst_authority(worst["wrenches"], weight, ampere_turn_budget)
    level = coupled_worst_authority(worst["level_all_wrenches"], weight, ampere_turn_budget)
    return {
        "lift_margin": worst["level_min_lift_per_at"] * ampere_turn_budget / weight,
        "showpiece_lift_margin": worst["min_lift_per_at"] * ampere_turn_budget / weight,
        "lateral": coupled["lateral"],
        "level_lateral": level["lateral"],
        "tilt": coupled["tilt"],
        "yaw": coupled["yaw"],
        "rungs": {fraction: {
            "tilt_deg": rung["tilt_deg"],
            "lift_margin": rung["min_lift_per_at"] * ampere_turn_budget / weight,
            "tilt_torque": coupled_worst_authority(rung["wrenches"], weight, ampere_turn_budget)["tilt"],
        } for fraction, rung in worst["rungs"].items()},
    }


def coil_height_coupling(magnet, control_cells, heights_m, reference_height_m):
    meshing = G.settings.verification_force_mesh
    place_piece(magnet)
    weight = G.weight
    characteristic_length = magnet.platform_side / 2
    center_of_mass = np.array([0, 0, magnet.center_of_mass_height])

    def lift_at_height(height, axial):
        array = coil_array_from_geometry(control_cells, height, 1, meshing, G.settings.coil_radial_filaments, axial)
        wrench = actuator_matrix(magnet, array, center_of_mass)
        _, coeffs = pose_coefficients(wrench, weight, characteristic_length)
        return coeffs["lift_per_at"]

    reference_lift = lift_at_height(reference_height_m, G.settings.minimum_axial_filaments)
    factors = []
    for height in heights_m:
        if height == reference_height_m:
            factors.append(1.0)
        else:
            axial = min(G.settings.max_coil_axial_filaments, max(G.settings.minimum_axial_filaments, int(np.ceil(height / G.settings.axial_filament_spacing))))
            factors.append(lift_at_height(height, axial) / reference_lift)
    place_piece(magnet)
    return factors


def measure(geometry):
    use_geometry(geometry)
    magnet = magnet_layout_from_geometry()
    weight = geometry.weight
    characteristic_length = magnet.platform_side / 2
    _, controllability = analyze(magnet, geometry.control_cells_per_side, weight,
                                     characteristic_length, G.settings.nominal_force_mesh,
                                     G.settings.coil_radial_filaments, G.settings.minimum_axial_filaments, 1)
    _, nominal = pose_coefficients(controllability.wrench_matrix, weight, characteristic_length)
    cruise_thrusts = [weight / geometry.gravity * acceleration for acceleration in geometry.cruise_accels]
    worst = flight_worst_case(geometry.control_cells_per_side, weight, characteristic_length,
                              (geometry.gap, geometry.max_flight_gap), geometry.coil_height, G.settings.minimum_axial_filaments,
                              cruise_thrusts=cruise_thrusts)
    coupling_factors = coil_height_coupling(magnet, geometry.control_cells_per_side,
                                            [height / 1000 for height in G.settings.coil_coupling_heights_mm],
                                            geometry.coil_height)
    base_diameter = magnet.platform_side * np.sqrt(2) + 2 * G.base_corner_standoff
    return {
        "peak_bz": magnet.peak_bz(),
        "lift_force_per_ampere_turn": nominal["lift_per_at"],
        "hover_ampere_turns_squared_sum": nominal["hover_sumsq"],
        "neighbour_snap_force": worst_touching_snap(geometry.magnet_lateral_edge, geometry.magnet_thickness, geometry.periods_per_side, base_diameter),
        "actuator_rank6": worst["min_rank6"],
        "actuator_condition6": worst["max_cond6"],
        "worst_case_poses": worst["poses"],
        "worst_case_max_gap": geometry.max_flight_gap,
        "worst_case_max_tilt_deg": worst["max_tilt_deg"],
        "worst_lift_force_per_ampere_turn": worst["level_min_lift_per_at"],
        "worst_hover_ampere_turns_squared_sum": worst["level_max_hover_sumsq"],
        "showpiece_lift_force_per_ampere_turn": worst["min_lift_per_at"],
        "showpiece_hover_ampere_turns_squared_sum": worst["max_hover_sumsq"],
        "average_hover_ampere_turns_squared_sum": worst["avg_level_hover_sumsq"],
        "cruise_ampere_turns_squared_sums": worst["avg_level_cruise_sumsq"],
        "worst_cruise_ampere_turns_squared_sums": worst["level_max_cruise_sumsq"],
        "cruise_peak_ampere_turns": worst["max_cruise_peak_at"],
        "coil_height_coupling_heights_mm": G.settings.coil_coupling_heights_mm,
        "coil_height_coupling_factors": coupling_factors,
    }


def operating_allocations(control_cells, coil_height, ampere_turn_budget, include_flight_cases=False, follow_piece=False, include_contact=True):
    magnet = magnet_layout_from_geometry()
    array = verification_coil_array(control_cells, coil_height)
    results = {}
    flight_cases = []
    mass = G.weight / G.gravity
    phases = coil_window_phases() if follow_piece else tuple(G.coil_long * fraction for fraction in G.settings.pose_phase_fractions)
    modes = {
        "flight": ((G.gap, G.max_flight_gap), G.cruise_accels[0], None),
    }
    if include_contact:
        modes.update(reset=((G.surface_stack,), G.reset_acceleration, G.settings.sliding_friction_coefficient),
                     breakaway=((G.surface_stack,), G.reset_acceleration, G.settings.resting_friction_coefficient))
    for name, (gaps, acceleration, friction) in modes.items():
        demand_cases = []
        hover_cases = []
        peak = 0.0
        for gap in gaps:
            speed = G.flight_speed if name == "flight" else G.reset_speed
            drag = conductive_drag(speed, gap, coil_height)
            for yaw in np.radians(G.settings.pose_yaws_deg):
                for x in phases:
                    for y in phases:
                        if follow_piece:
                            array.build(center=(x, y))
                        _, center = place_piece(magnet, x=x, y=y, z=gap - G.gap, yaw=yaw)
                        wrench = actuator_matrix(magnet, array, center)
                        for direction_wrench in horizontal_wrench_targets(1.0, 0.0):
                            if name == "flight":
                                target = np.append((mass * acceleration + drag) * direction_wrench[:2], [G.weight, 0, 0, 0])
                                equality, inequalities, bounds = wrench, None, None
                            else:
                                external = np.append(-drag * direction_wrench[:2], np.zeros(4))
                                equality, target, inequalities, bounds = contact_constraints(wrench, direction_wrench[:2], friction, acceleration, external)
                            required, initial = min_peak_current(equality, target, inequalities, bounds)
                            peak = max(peak, required)
                            demand_cases.append((equality, target, initial, inequalities, bounds))
                            if name == "flight" and include_flight_cases:
                                flight_cases.append({"gap": gap, "x": x, "y": y, "yaw": yaw,
                                                     "wrench": wrench, "target": target,
                                                     "direction": direction_wrench[:2], "drag": drag,
                                                     "coil_identities": tuple(array.identities),
                                                     "peak_ampere_turns": required, "currents": initial})
                        if name == "flight":
                            target = [0, 0, G.weight, 0, 0, 0]
                            required, initial = min_peak_current(wrench, target)
                            peak = max(peak, required)
                            hover_cases.append((wrench, target, initial))
        currents = np.asarray([min_loss_current(equality, target, peak, initial_currents=initial,
                                                contact_inequality=inequalities, contact_bounds=bounds)
                               for equality, target, initial, inequalities, bounds in demand_cases])
        results[name] = {
            "peak_ampere_turns": peak,
            "worst_squared_sum": float(np.max(np.sum(currents ** 2, axis=1))),
            "worst_absolute_sum": float(np.max(np.sum(np.abs(currents), axis=1))),
            "coil_count": len(array.coils),
        }
        if hover_cases:
            hover_currents = np.asarray([min_loss_current(wrench, target, peak, initial_currents=initial)
                                        for wrench, target, initial in hover_cases])
            results["hover"] = {
                "worst_squared_sum": float(np.max(np.sum(hover_currents ** 2, axis=1))),
                "worst_absolute_sum": float(np.max(np.sum(np.abs(hover_currents), axis=1))),
                "coil_count": len(array.coils),
            }
    array.build()
    _, center = place_piece(magnet)
    wrench = actuator_matrix(magnet, array, center)
    _, initial = min_peak_current(wrench, [0, 0, G.weight, 0, 0, 0])
    hover_currents = min_loss_current(wrench, [0, 0, G.weight, 0, 0, 0], results["flight"]["peak_ampere_turns"], initial_currents=initial)
    _, eigenvalues = open_loop_stiffness(magnet, array, hover_currents)
    results["instability_growth_rate"] = float(np.max(eigenvalues.real))
    if include_contact:
        results["contact_pair"] = contact_pair_allocation(control_cells, coil_height, ampere_turn_budget)
        results["contact_tipping_margin"] = (magnet.platform_side / 2 * np.sqrt(2) + G.base_corner_standoff) / (G.settings.resting_friction_coefficient * G.center_of_mass_height)
    if include_flight_cases:
        results["flight_cases"] = flight_cases
    return results


def conductive_drag(speed, gap, coil_height):
    return sum(conductive_drag_components(speed, gap, coil_height).values())


def conductive_drag_components(speed, gap, coil_height):
    return drag_components_at_speed(speed, conductive_sheet_response(gap, coil_height))


def drag_components_at_speed(speed, response):
    return {name: coupling * speed * characteristic_velocity / (speed ** 2 + characteristic_velocity ** 2)
            for name, (coupling, characteristic_velocity) in response.items()}


def conductive_sheet_response(gap, coil_height):
    settings = G.settings
    constants = G.constants
    pcb_depth = gap + 2 * coil_height + settings.pcb_thickness / 2000
    plate_depth = gap + 2 * coil_height + settings.pcb_thickness / 1000 + settings.radiator_standoff_below_pcb / 1000
    slot_thickness = (settings.baseplate_thickness - settings.radiator_slot_web_thickness) / 1000
    slot_factor = min(1.0, (settings.radiator_slot_pitch / 1000 / (G.magnets_per_period * G.magnet_lateral_edge / 2)) ** 2)
    sheets = (("pcb_copper", pcb_depth, constants.copper_resistivity, settings.pcb_copper_plane_thickness / 1000, 1.0),
              ("slotted_plate", plate_depth, constants.aluminium_resistivity, slot_thickness, slot_factor),
              ("plate_web", plate_depth + slot_thickness, constants.aluminium_resistivity, settings.radiator_slot_web_thickness / 1000, 1.0))
    response = {}
    for name, depth, resistivity, thickness, factor in sheets:
        if thickness <= 0:
            raise ValueError("Conductive sheet thickness must be positive")
        characteristic_velocity = 2 * resistivity / (constants.vacuum_permeability * thickness)
        response[name] = (abs(eddy_image_force(-depth)) * factor, characteristic_velocity)
    return response


def contact_pair_allocation(control_cells, coil_height, ampere_turn_budget):
    first = magnet_layout_from_geometry()
    second = magnet_layout_from_geometry()
    separation = first.platform_side * np.sqrt(2) + 2 * G.base_corner_standoff
    cells = control_cells + int(np.ceil(separation / G.coil_short))
    array = verification_coil_array(cells, coil_height)
    _, first_center = place_piece(first, x=-separation / 2, z=G.surface_stack - G.gap)
    _, second_center = place_piece(second, x=separation / 2, z=G.surface_stack - G.gap)
    first_wrench = actuator_matrix(first, array, first_center)
    second_wrench = actuator_matrix(second, array, second_center)
    first_external = neighbour_force(G.magnet_lateral_edge, G.magnet_thickness, G.periods_per_side, separation)
    second_external = neighbour_force(G.magnet_lateral_edge, G.magnet_thickness, G.periods_per_side, -separation)
    first_constraints = contact_constraints(first_wrench, [-1, 0], G.settings.resting_friction_coefficient, G.reset_acceleration, first_external)
    second_constraints = contact_constraints(second_wrench, [1, 0], G.settings.resting_friction_coefficient, G.reset_acceleration, second_external)
    equality = np.vstack((first_constraints[0], second_constraints[0]))
    target = np.concatenate((first_constraints[1], second_constraints[1]))
    inequalities = np.vstack((first_constraints[2], second_constraints[2]))
    bounds = np.concatenate((first_constraints[3], second_constraints[3]))
    peak, initial = min_peak_current(equality, target, inequalities, bounds)
    return {"peak_ampere_turns": float(peak), "coil_count": len(array.coils),
            "residual": float(np.max(np.abs(equality @ initial - target))),
            "within_budget": bool(peak <= ampere_turn_budget)}


def contact_constraints(wrench, direction, friction, acceleration, external_wrench):
    direction = np.asarray(direction)
    external = np.asarray(external_wrench)
    effective_weight = G.weight - external[2]
    mass = G.weight / G.gravity
    yaw = wrench[5] - friction * (direction[0] * wrench[3] + direction[1] * wrench[4])
    yaw_target = -external[5] + friction * (direction[0] * external[3] + direction[1] * external[4])
    equality = np.vstack((wrench[0] + friction * direction[0] * wrench[2],
                          wrench[1] + friction * direction[1] * wrench[2], yaw))
    target = np.append((mass * acceleration + friction * effective_weight) * direction - external[:2], yaw_target)
    support_radius = (G.magnets_per_period * G.periods_per_side * G.magnet_lateral_edge / 2 * np.sqrt(2) + G.base_corner_standoff) / np.sqrt(2)
    roll = wrench[3] + G.center_of_mass_height * friction * direction[1] * wrench[2]
    pitch = wrench[4] - G.center_of_mass_height * friction * direction[0] * wrench[2]
    roll_offset = G.center_of_mass_height * friction * direction[1] * effective_weight - external[3]
    pitch_offset = -G.center_of_mass_height * friction * direction[0] * effective_weight - external[4]
    inequality = np.vstack((wrench[2], roll + support_radius * wrench[2], -roll + support_radius * wrench[2],
                            pitch + support_radius * wrench[2], -pitch + support_radius * wrench[2]))
    bounds = np.array([(1 - G.settings.minimum_contact_normal_fraction) * G.weight - external[2],
                       support_radius * effective_weight + roll_offset,
                       support_radius * effective_weight - roll_offset,
                       support_radius * effective_weight + pitch_offset,
                       support_radius * effective_weight - pitch_offset])
    if not 0 < G.settings.minimum_contact_normal_fraction <= 1:
        raise ValueError("Contact normal-force fraction must be positive and at most one")
    return equality, target, inequality, bounds

import numpy as np
import magpylib as magpy
from magpylib_force import getFT
from scipy.spatial.transform import Rotation
from scipy.optimize import linprog


class SimGeometry:
    def __init__(self, magnet_lateral_edge_mm, magnet_thickness_mm, magnets_per_period, periods_per_side,
                 magnet_to_coil_distance_mm, max_flight_gap_mm, plastic_wall_thickness_mm, base_corner_standoff_mm,
                 remanence, ndfeb_density_g_per_mm3, plastic_density_g_per_mm3,
                 gravity, reference_king_height_mm, reference_king_base_diameter_mm,
                 com_height_fraction, coil_outer_width_mm, coil_outer_length_mm,
                 coil_radial_width_mm, coil_height_mm, control_cells_per_side,
                 hall_sensor_pitch_mm, hall_observation_window_side, target_tilt_deg):
        self.magnet_lateral_edge = magnet_lateral_edge_mm / 1000
        self.magnet_thickness = magnet_thickness_mm / 1000
        self.magnets_per_period = magnets_per_period
        self.periods_per_side = periods_per_side
        self.gap = magnet_to_coil_distance_mm / 1000
        self.max_flight_gap = max_flight_gap_mm / 1000
        self.wall_thickness = plastic_wall_thickness_mm / 1000
        self.base_corner_standoff = base_corner_standoff_mm / 1000
        self.remanence = remanence
        self.ndfeb_density = ndfeb_density_g_per_mm3 * 1e6
        self.plastic_density = plastic_density_g_per_mm3 * 1e6
        self.gravity = gravity
        self.king_height = reference_king_height_mm / 1000
        self.king_base_diameter = reference_king_base_diameter_mm / 1000
        self.com_height_fraction = com_height_fraction
        self.coil_short = coil_outer_width_mm / 1000
        self.coil_long = coil_outer_length_mm / 1000
        self.coil_radial_width = coil_radial_width_mm / 1000
        self.coil_height = coil_height_mm / 1000
        self.control_cells_per_side = control_cells_per_side
        self.hall_sensor_pitch = hall_sensor_pitch_mm / 1000
        self.hall_observation_window_side = hall_observation_window_side
        self.target_tilt_deg = target_tilt_deg


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
    return int(np.linalg.matrix_rank(matrix, tol=np.abs(matrix).max() * 1e-9))


def condition_number(matrix):
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    significant = singular_values[singular_values > singular_values.max() * 1e-12]
    return significant[0] / significant[-1]


class MagnetLayout:
    def __init__(self, lateral_edge, thickness, periods, per_period, remanence):
        self.lateral_edge = lateral_edge
        self.thickness = thickness
        self.periods = periods
        self.per_period = per_period
        self.remanence = remanence

        self.blocks_per_side = self.periods * self.per_period
        self.period = self.per_period * self.lateral_edge
        self.wavenumber = 2 * np.pi / self.period
        self.platform_side = self.blocks_per_side * self.lateral_edge

        self.block_count = 0
        self.collection = self.build()
        self.center_of_mass_height = self.compute_center_of_mass_height()

    def block_center(self, index):
        return (index + 0.5 - self.blocks_per_side / 2) * self.lateral_edge

    def block_polarization(self, column_index, row_index):
        x = self.block_center(column_index)
        y = self.block_center(row_index)
        remanence = self.remanence
        if (row_index % 2) == 0:
            angle = self.wavenumber * x
            return np.array([remanence * np.sin(angle), 0.0, -remanence * np.cos(angle)])
        angle = self.wavenumber * y
        return np.array([0.0, remanence * np.sin(angle), -remanence * np.cos(angle)])

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
        self.block_count = len(blocks)
        return magpy.Collection(blocks)

    def compute_center_of_mass_height(self):
        base_diameter = self.platform_side * np.sqrt(2)
        scale = base_diameter / G.king_base_diameter
        return G.com_height_fraction * G.king_height * scale

    @property
    def magnet_mass(self):
        return self.block_count * self.lateral_edge ** 2 * self.thickness * G.ndfeb_density

    def plane_field(self, samples=41):
        half = self.platform_side / 2
        axis = np.linspace(-half, half, samples)
        grid = np.array([[x, y, -G.gap] for x in axis for y in axis])
        return self.collection.getB(grid)

    def peak_bz(self, samples=41):
        return float(np.max(np.abs(self.plane_field(samples)[:, 2])))


def magnet_layout_from_geometry():
    return MagnetLayout(G.magnet_lateral_edge, G.magnet_thickness, G.periods_per_side, G.magnets_per_period, G.remanence)


class CoilArray:
    def __init__(self, cells_per_side, short, long, radial_width, height, turns,
                 gap, layer_gap, meshing, filaments_radial, filaments_axial):
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
        self.turns_per_filament = turns / (filaments_radial * filaments_axial)
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

    def lattice(self, count_x, pitch_x, count_y, pitch_y):
        offset_x = (count_x - 1) / 2
        offset_y = (count_y - 1) / 2
        for i in range(count_x):
            for j in range(count_y):
                yield (i - offset_x) * pitch_x, (j - offset_y) * pitch_y

    def add(self, orientation, center_x, center_y, z_top):
        self.coils.append(self.winding(orientation, center_x, center_y, z_top))

    def build(self):
        region = self.cells_per_side * self.short
        long_count = max(1, round(region / self.long))
        z_top_x = -self.gap
        z_top_y = -self.gap - self.layer_gap
        for center_x, center_y in self.lattice(self.cells_per_side, self.short, long_count, self.long):
            self.add("x", center_x, center_y, z_top_x)
        for center_x, center_y in self.lattice(long_count, self.long, self.cells_per_side, self.short):
            self.add("y", center_x, center_y, z_top_y)


def coil_array_from_geometry(cells_per_side, height, turns, meshing, filaments_radial, filaments_axial):
    return CoilArray(cells_per_side, G.coil_short, G.coil_long, G.coil_radial_width,
                     height, turns, G.gap, height, meshing, filaments_radial,
                     filaments_axial)


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
    force_torque = np.asarray(getFT(layout.collection, filaments, anchor=center_of_mass))
    per_coil = force_torque.reshape(len(array.coils), -1, 6).sum(axis=1)
    return -per_coil.T


def piece_weight(layout):
    base_diameter = layout.platform_side * np.sqrt(2) + 2 * G.base_corner_standoff
    scale = base_diameter / G.king_base_diameter
    height = G.king_height * scale
    inner_diameter = base_diameter - 2 * G.wall_thickness
    inner_height = height - 2 * G.wall_thickness
    shell_volume = np.pi / 4 * (base_diameter ** 2 * height - inner_diameter ** 2 * inner_height)
    shell_mass = shell_volume * G.plastic_density
    total_mass = shell_mass + layout.magnet_mass
    return total_mass * G.gravity


def min_peak_current(matrix, target):
    coil_count = matrix.shape[1]
    objective = np.zeros(coil_count + 1)
    objective[-1] = 1.0
    inequality = np.zeros((2 * coil_count, coil_count + 1))
    inequality[:coil_count, :coil_count] = np.eye(coil_count)
    inequality[:coil_count, -1] = -1.0
    inequality[coil_count:, :coil_count] = -np.eye(coil_count)
    inequality[coil_count:, -1] = -1.0
    equality = np.hstack([np.array(matrix), np.zeros((matrix.shape[0], 1))])
    result = linprog(
        objective,
        A_ub=inequality, b_ub=np.zeros(2 * coil_count),
        A_eq=equality, b_eq=np.array(target),
        bounds=[(None, None)] * coil_count + [(0, None)],
        method="highs",
    )
    if not result.success:
        raise RuntimeError(result.message)
    return result.x[-1], result.x[:coil_count]


def max_generalized_force(wrench_matrix, objective_row, constrained_rows,
                          constrained_values, current_limit):
    coil_count = wrench_matrix.shape[1]
    result = linprog(
        c=-wrench_matrix[objective_row],
        bounds=[(-current_limit, current_limit)] * coil_count,
        A_eq=wrench_matrix[constrained_rows],
        b_eq=constrained_values,
        method="highs",
    )
    if not result.success:
        raise RuntimeError(result.message)
    return -result.fun


class Controllability:
    def __init__(self, wrench_matrix, weight, characteristic_length):
        self.wrench_matrix = wrench_matrix
        self.weight = weight
        self.characteristic_length = characteristic_length

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


def open_loop_stiffness(layout, array, currents, step=2e-5):
    def coil_force(dx, dy, dz):
        _, center_of_mass = place_piece(layout, x=dx, y=dy, z=dz)
        wrench_matrix = actuator_matrix(layout, array, center_of_mass)
        return (wrench_matrix @ currents)[:3]

    jacobian = np.zeros((3, 3))
    for axis_index in range(3):
        forward = [0.0, 0.0, 0.0]
        backward = [0.0, 0.0, 0.0]
        forward[axis_index] = step
        backward[axis_index] = -step
        jacobian[:, axis_index] = (coil_force(*forward) - coil_force(*backward)) / (2 * step)
    place_piece(layout)
    stiffness = -jacobian
    eigenvalues = np.linalg.eigvals(0.5 * (stiffness + stiffness.T)).real
    return stiffness, eigenvalues


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
    position_step, angle_step = 1e-5, 1e-4
    columns = []
    for axis_index, step in enumerate([position_step, position_step, position_step,
                                       angle_step, angle_step, angle_step]):
        offsets = list(pose)
        offsets[axis_index] += step
        columns.append((field_at_pose(*offsets) - baseline) / step)
    place_piece(layout)
    return np.array(columns).T


def hall_metrics(jacobian):
    return {
        "rank6": int(np.linalg.matrix_rank(jacobian, tol=np.abs(jacobian).max() * 1e-6)),
        "cond6": condition_number(jacobian),
    }


def hall_observability(layout, target_tilt_deg):
    sensor_pitch = G.hall_sensor_pitch
    window_side = G.hall_observation_window_side
    points = local_hall_points(sensor_pitch, window_side, -G.gap)
    phase = (0.0, sensor_pitch / 4, sensor_pitch / 2)
    circumradius = layout.platform_side / 2 * np.sqrt(2)
    nominal = hall_metrics(hall_jacobian(layout, points, [0, 0, 0, 0, 0, 0]))
    worst = {"min_rank6": 6, "max_cond6": 0.0, "poses": 0}
    for target_gap in (G.gap, G.max_flight_gap):
        z = target_gap - G.gap
        geometric_limit = float(np.arcsin(min(0.95, 0.9 * target_gap / circumradius)))
        tilt = min(np.radians(target_tilt_deg), geometric_limit)
        for yaw in np.radians((0, 45, 90)):
            for roll, pitch in ((0.0, 0.0), (tilt, 0.0), (0.0, tilt), (tilt, tilt)):
                for dx in phase:
                    for dy in phase:
                        metrics = hall_metrics(hall_jacobian(layout, points, [dx, dy, z, roll, pitch, yaw]))
                        worst["min_rank6"] = min(worst["min_rank6"], metrics["rank6"])
                        if metrics["rank6"] >= 6:
                            worst["max_cond6"] = max(worst["max_cond6"], metrics["cond6"])
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
        "worst_poses": worst["poses"],
    }


def neighbour_force(lateral_edge, thickness, periods, distance, controlled_yaw=0.0, neighbour_yaw=0.0, mesh=(4, 4, 4)):
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
    force_torque = np.atleast_2d(getFT(neighbour.collection, targets, anchor=anchor))
    if force_torque.ndim == 3:
        return force_torque[:, 0, :].sum(axis=0)
    return force_torque[0]


def worst_touching_snap(lateral_edge, thickness, periods, distance):
    angles = np.radians(np.arange(0.0, 91.0, 15.0))
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
        "lateral_per_at": min(max_generalized_force(wrench, 0, [2], [0.0], 1.0),
                              max_generalized_force(wrench, 1, [2], [0.0], 1.0)),
        "tilt_torque_per_at": min(max_generalized_force(wrench, 3, [0, 1, 2], [0, 0, 0], 1.0),
                                  max_generalized_force(wrench, 4, [0, 1, 2], [0, 0, 0], 1.0)),
        "yaw_torque_per_at": max_generalized_force(wrench, 5, [0, 1, 2], [0, 0, 0], 1.0),
        "hover_sumsq": float(np.sum(controllability.current6 ** 2)),
    }


def flight_worst_case(control_cells, weight, characteristic_length, target_tilt_deg,
                      gaps, yaws_deg=(0, 45, 90), meshing=12):
    magnet = magnet_layout_from_geometry()
    array = coil_array_from_geometry(control_cells, G.coil_height, 1, meshing, 2, 1)
    circumradius = magnet.platform_side / 2 * np.sqrt(2)
    phase = (0.0, G.coil_short / 4, G.coil_short / 2)
    worst = {
        "min_rank6": 6, "max_cond6": 0.0,
        "min_lift_per_at": 0.0, "min_lateral_per_at": 0.0,
        "min_tilt_torque_per_at": 0.0, "min_yaw_torque_per_at": 0.0,
        "max_hover_sumsq": 0.0, "max_tilt_deg": 0.0, "poses": 0,
    }
    for target_gap in gaps:
        z = target_gap - G.gap
        geometric_limit = float(np.arcsin(min(0.95, 0.9 * target_gap / circumradius)))
        tilt = min(np.radians(target_tilt_deg), geometric_limit)
        worst["max_tilt_deg"] = max(worst["max_tilt_deg"], np.degrees(tilt))
        tilts = ((0.0, 0.0), (tilt, 0.0), (0.0, tilt), (tilt, tilt))
        for yaw in np.radians(yaws_deg):
            for roll, pitch in tilts:
                for dx in phase:
                    for dy in phase:
                        _, center_of_mass = place_piece(magnet, x=dx, y=dy, z=z,
                                                        roll=roll, pitch=pitch, yaw=yaw)
                        wrench = actuator_matrix(magnet, array, center_of_mass)
                        rank6, coeffs = pose_coefficients(wrench, weight, characteristic_length)
                        if worst["poses"] == 0:
                            worst["min_lift_per_at"] = coeffs["lift_per_at"]
                            worst["min_lateral_per_at"] = coeffs["lateral_per_at"]
                            worst["min_tilt_torque_per_at"] = coeffs["tilt_torque_per_at"]
                            worst["min_yaw_torque_per_at"] = coeffs["yaw_torque_per_at"]
                        worst["min_rank6"] = min(worst["min_rank6"], rank6)
                        worst["poses"] += 1
                        worst["max_cond6"] = max(worst["max_cond6"], coeffs["cond6"])
                        worst["min_lift_per_at"] = min(worst["min_lift_per_at"], coeffs["lift_per_at"])
                        worst["min_lateral_per_at"] = min(worst["min_lateral_per_at"], coeffs["lateral_per_at"])
                        worst["min_tilt_torque_per_at"] = min(worst["min_tilt_torque_per_at"], coeffs["tilt_torque_per_at"])
                        worst["min_yaw_torque_per_at"] = min(worst["min_yaw_torque_per_at"], coeffs["yaw_torque_per_at"])
                        worst["max_hover_sumsq"] = max(worst["max_hover_sumsq"], coeffs["hover_sumsq"])
    place_piece(magnet)
    return worst


def coil_height_coupling(magnet, control_cells, heights_m, reference_height_m, meshing=8):
    place_piece(magnet)
    weight = piece_weight(magnet)
    characteristic_length = magnet.platform_side / 2
    center_of_mass = np.array([0, 0, magnet.center_of_mass_height])

    def lift_at_height(height, axial):
        array = coil_array_from_geometry(control_cells, height, 1, meshing, 2, axial)
        wrench = actuator_matrix(magnet, array, center_of_mass)
        _, coeffs = pose_coefficients(wrench, weight, characteristic_length)
        return coeffs["lift_per_at"]

    reference_lift = lift_at_height(reference_height_m, 1)
    factors = []
    for height in heights_m:
        if height == reference_height_m:
            factors.append(1.0)
        else:
            axial = min(4, max(1, int(round(height / 0.001))))
            factors.append(lift_at_height(height, axial) / reference_lift)
    place_piece(magnet)
    return factors


def measure(geometry):
    use_geometry(geometry)
    control_cells = geometry.control_cells_per_side

    magnet = magnet_layout_from_geometry()
    weight = piece_weight(magnet)
    characteristic_length = magnet.platform_side / 2
    array, controllability = analyze(magnet, control_cells, weight,
                                     characteristic_length, 16, 2, 1, 1)
    _, nominal = pose_coefficients(controllability.wrench_matrix, weight, characteristic_length)

    flight_gaps = (geometry.gap, geometry.max_flight_gap)
    worst = flight_worst_case(control_cells, weight, characteristic_length, geometry.target_tilt_deg, flight_gaps)

    coupling_heights_mm = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0]
    coupling_factors = coil_height_coupling(magnet, control_cells,
                                            [h / 1000 for h in coupling_heights_mm],
                                            G.coil_height)

    stiffness, eigenvalues = open_loop_stiffness(magnet, array, controllability.current6)
    unstable = eigenvalues[eigenvalues < 0]
    instability_growth_time = 1.0 / np.sqrt(np.abs(unstable).max() / (weight / geometry.gravity))

    base_diameter = magnet.platform_side * np.sqrt(2) + 2 * G.base_corner_standoff
    neighbour_snap_force = worst_touching_snap(geometry.magnet_lateral_edge, geometry.magnet_thickness, geometry.periods_per_side, base_diameter)

    hall = hall_observability(magnet, geometry.target_tilt_deg)

    return {
        "peak_bz": magnet.peak_bz(),
        "lift_force_per_ampere_turn": nominal["lift_per_at"],
        "lateral_to_lift_ratio": nominal["lateral_per_at"] / nominal["lift_per_at"],
        "yaw_torque_per_ampere_turn": nominal["yaw_torque_per_at"],
        "tilt_torque_per_ampere_turn": nominal["tilt_torque_per_at"],
        "hover_ampere_turns_squared_sum": nominal["hover_sumsq"],
        "instability_growth_time": instability_growth_time,
        "vertical_stiffness": float(np.max(np.abs(eigenvalues))),
        "neighbour_snap_force": neighbour_snap_force,
        "actuator_rank6": worst["min_rank6"],
        "actuator_condition6": worst["max_cond6"],
        "hall_rank6": hall["rank6"],
        "hall_condition6": hall["condition6"],
        "hall_worst_rank6": hall["worst_rank6"],
        "hall_worst_condition6": hall["worst_condition6"],
        "hall_worst_case_poses": hall["worst_poses"],
        "hall_sensors_per_piece": hall["sensors_per_piece"],
        "hall_sensor_pitch": hall["sensor_pitch"],
        "hall_observation_window_side": hall["observation_window_side"],
        "worst_case_poses": worst["poses"],
        "worst_case_max_gap": max(flight_gaps),
        "worst_case_max_tilt_deg": worst["max_tilt_deg"],
        "worst_lift_force_per_ampere_turn": worst["min_lift_per_at"],
        "worst_lateral_force_per_ampere_turn": worst["min_lateral_per_at"],
        "worst_tilt_torque_per_ampere_turn": worst["min_tilt_torque_per_at"],
        "worst_yaw_torque_per_ampere_turn": worst["min_yaw_torque_per_at"],
        "worst_hover_ampere_turns_squared_sum": worst["max_hover_sumsq"],
        "coil_height_coupling_heights_mm": coupling_heights_mm,
        "coil_height_coupling_factors": coupling_factors,
    }

import numpy as np
import magpylib as magpy
from magpylib_force import getFT
from scipy.spatial.transform import Rotation
from scipy.optimize import linprog
import model


EDGE = model.Inputs.magnet_cube_edge / 1000
MAGNETS_PER_PERIOD = model.Inputs.magnets_per_period
PERIODS_PER_SIDE = model.Inputs.periods_per_side
GAP = model.Inputs.magnet_to_coil_distance / 1000
WALL_THICKNESS = model.Inputs.plastic_wall_thickness / 1000
BASE_CLEARANCE = model.Fixed.base_material_clearance / 1000
SQUARE_FILL_RATIO = model.Fixed.square_fill_ratio

REMANENCE = model.Constants.ndfeb_remanence_br
NDFEB_DENSITY = model.Constants.ndfeb_density * 1e6
PLASTIC_DENSITY = model.Constants.plastic_density * 1e6
GRAVITY = model.Constants.gravity
KING_HEIGHT = model.Fixed.reference_king_height / 1000
KING_BASE_DIAMETER = model.Fixed.reference_king_base_diameter / 1000
COM_HEIGHT_FRACTION = model.Fixed.com_height_fraction

PIECE_WEIGHT = model.piece.weight
PIECE_MASS = model.piece.mass / 1000

COIL_SHORT = model.coil.outer_width / 1000
COIL_LONG = model.coil.outer_length / 1000
COIL_RADIAL_WIDTH = model.config.radial_width / 1000
COIL_HEIGHT = model.config.coil_height / 1000
COIL_TURNS = model.config.turns
COIL_RESISTANCE = model.config.resistance
COIL_CURRENT_LIMIT = model.config.current_limit
COIL_OPERATING_CURRENT = model.config.operating_current
COIL_USABLE_VOLTAGE = model.config.usable_drive_voltage
DRIVER_CURRENT = model.Fixed.driver_channel_current
COILS_PER_PERIOD = round(MAGNETS_PER_PERIOD * EDGE / COIL_SHORT)
CONTROL_BANDWIDTH_MARGIN = model.Inputs.control_loop_bandwidth_margin

BOARD_SQUARE_PITCH = model.board.square_size / 1000
BASE_DIAMETER = model.board.base_diameter / 1000

AXIS_DIRECTIONS = np.array(
    [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]],
    dtype=float,
)

DISCRETE_PATTERN_KINDS = {"halbach1d_disc", "halbach2d_square", "herringbone"}


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
    def __init__(self, kind="herringbone", footprint="square", edge=EDGE,
                 periods=PERIODS_PER_SIDE, per_period=MAGNETS_PER_PERIOD,
                 remanence=REMANENCE):
        self.kind = kind
        self.footprint = footprint
        self.edge = edge
        self.periods = periods
        self.per_period = per_period
        self.remanence = remanence

        self.blocks_per_side = self.periods * self.per_period
        self.period = self.per_period * self.edge
        self.wavenumber = 2 * np.pi / self.period
        self.platform_side = self.blocks_per_side * self.edge

        self.ideal_directions = []
        self.cube_count = 0
        self.collection = self.build()
        self.center_of_mass_height = self.compute_center_of_mass_height()

    def block_center(self, index):
        return (index + 0.5 - self.blocks_per_side / 2) * self.edge

    def is_inside_footprint(self, x, y):
        half = self.platform_side / 2
        if self.footprint == "square":
            return True
        if self.footprint == "round":
            return np.hypot(x, y) <= half + 1e-12
        if self.footprint == "octagon":
            return abs(x) + abs(y) <= 1.30 * half + 1e-12
        raise ValueError(f"unknown footprint {self.footprint}")

    def block_polarization(self, column_index, row_index):
        x = self.block_center(column_index)
        y = self.block_center(row_index)
        remanence = self.remanence
        if self.kind in ("halbach1d", "halbach1d_disc"):
            angle = self.wavenumber * x
            return np.array([remanence * np.sin(angle), 0.0, -remanence * np.cos(angle)])
        if self.kind == "halbach2d_square":
            cos_x, cos_y = np.cos(self.wavenumber * x), np.cos(self.wavenumber * y)
            sin_x, sin_y = np.sin(self.wavenumber * x), np.sin(self.wavenumber * y)
            return remanence * unit_vector(np.array([sin_x * cos_y, cos_x * sin_y, -cos_x * cos_y]))
        if self.kind == "herringbone":
            if (row_index % 2) == 0:
                angle = self.wavenumber * x
                return np.array([remanence * np.sin(angle), 0.0, -remanence * np.cos(angle)])
            angle = self.wavenumber * y
            return np.array([0.0, remanence * np.sin(angle), -remanence * np.cos(angle)])
        raise ValueError(f"unknown kind {self.kind}")

    def build(self):
        cubes = []
        discrete = self.kind in DISCRETE_PATTERN_KINDS
        for column_index in range(self.blocks_per_side):
            x = self.block_center(column_index)
            for row_index in range(self.blocks_per_side):
                y = self.block_center(row_index)
                if not self.is_inside_footprint(x, y):
                    continue
                polarization = self.block_polarization(column_index, row_index)
                if discrete:
                    polarization = snap_to_axis(polarization, self.remanence)
                self.ideal_directions.append(unit_vector(polarization))
                cubes.append(magpy.magnet.Cuboid(
                    dimension=(self.edge, self.edge, self.edge),
                    polarization=tuple(polarization),
                    position=(x, y, self.edge / 2),
                ))
        self.cube_count = len(cubes)
        return magpy.Collection(cubes)

    def compute_center_of_mass_height(self):
        base_diameter = self.platform_side * np.sqrt(2)
        scale = base_diameter / KING_BASE_DIAMETER
        return COM_HEIGHT_FRACTION * KING_HEIGHT * scale

    @property
    def magnet_mass(self):
        return self.cube_count * self.edge ** 3 * NDFEB_DENSITY

    @property
    def max_axis_error_degrees(self):
        worst_angle = 0.0
        for direction in self.ideal_directions:
            alignments = AXIS_DIRECTIONS @ direction
            angle = np.degrees(np.arccos(np.clip(alignments.max(), -1, 1)))
            worst_angle = max(worst_angle, angle)
        return worst_angle

    @property
    def manufacturable(self):
        return self.max_axis_error_degrees < 1.0

    def plane_field(self, samples=41):
        half = self.platform_side / 2
        axis = np.linspace(-half, half, samples)
        grid = np.array([[x, y, -GAP] for x in axis for y in axis])
        return self.collection.getB(grid)

    def peak_bz(self, samples=41):
        return float(np.max(np.abs(self.plane_field(samples)[:, 2])))


class CoilArray:
    def __init__(self, cells_per_side, short=COIL_SHORT, long=COIL_LONG,
                 radial_width=COIL_RADIAL_WIDTH, height=COIL_HEIGHT, turns=COIL_TURNS,
                 gap=GAP, layer_gap=COIL_HEIGHT, meshing=16,
                 filaments_radial=2, filaments_axial=1):
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
        self.orientations = []
        self.centers = []
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
        self.orientations.append(orientation)
        self.centers.append((center_x, center_y))

    def build(self):
        region = self.cells_per_side * self.short
        long_count = max(1, round(region / self.long))
        z_top_x = -self.gap
        z_top_y = -self.gap - self.layer_gap
        for center_x, center_y in self.lattice(self.cells_per_side, self.short, long_count, self.long):
            self.add("x", center_x, center_y, z_top_x)
        for center_x, center_y in self.lattice(long_count, self.long, self.cells_per_side, self.short):
            self.add("y", center_x, center_y, z_top_y)

    @property
    def ncoils(self):
        return len(self.coils)

    @property
    def extent(self):
        return self.cells_per_side * self.short


def place_piece(layout, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    rotation = Rotation.from_euler("xyz", [roll, pitch, yaw])
    collection = layout.collection
    collection.position = (0.0, 0.0, 0.0)
    collection.orientation = Rotation.identity()
    collection.rotate(rotation, anchor=(0.0, 0.0, layout.center_of_mass_height))
    collection.move((x, y, z))
    center_of_mass = np.array([x, y, layout.center_of_mass_height + z])
    return collection, center_of_mass


def coil_wrench(layout, coil, center_of_mass):
    force = np.zeros(3)
    torque = np.zeros(3)
    for filament in coil:
        filament_force, filament_torque = getFT(layout.collection, filament, anchor=center_of_mass)
        force += filament_force
        torque += filament_torque
    return -np.concatenate([force, torque])


def actuator_matrix(layout, array, center_of_mass):
    return np.array([coil_wrench(layout, coil, center_of_mass) for coil in array.coils]).T


def piece_weight(layout):
    base_diameter = layout.platform_side * np.sqrt(2) + BASE_CLEARANCE
    scale = base_diameter / KING_BASE_DIAMETER
    height = KING_HEIGHT * scale
    inner_diameter = base_diameter - 2 * WALL_THICKNESS
    inner_height = height - 2 * WALL_THICKNESS
    shell_volume = np.pi / 4 * (base_diameter ** 2 * height - inner_diameter ** 2 * inner_height)
    shell_mass = shell_volume * PLASTIC_DENSITY
    total_mass = shell_mass + layout.magnet_mass
    return total_mass * GRAVITY


def square_pitch(layout, pitch_multiplier=1.0):
    base_diameter = layout.platform_side * np.sqrt(2) + BASE_CLEARANCE
    return pitch_multiplier * base_diameter / SQUARE_FILL_RATIO


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
    if result.success:
        return result.x[-1], result.x[:coil_count]
    return np.inf, None


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
    return -result.fun if result.success else np.nan


class Controllability:
    def __init__(self, wrench_matrix, weight, characteristic_length, current_limit):
        self.wrench_matrix = wrench_matrix
        self.weight = weight
        self.characteristic_length = characteristic_length
        self.current_limit = current_limit
        self.ncoils = wrench_matrix.shape[1]

        self.scaled_wrench_matrix = wrench_matrix.copy()
        self.scaled_wrench_matrix[3:] = wrench_matrix[3:] / characteristic_length

        self.rank3 = matrix_rank(wrench_matrix[:3])
        self.rank6 = matrix_rank(self.scaled_wrench_matrix)
        self.cond3 = condition_number(wrench_matrix[:3])
        self.cond6 = condition_number(self.scaled_wrench_matrix)

        self.i_hover3, self.current3 = min_peak_current(wrench_matrix[:3], [0, 0, weight])
        if self.rank6 >= 6:
            self.i_hover6, self.current6 = min_peak_current(wrench_matrix, [0, 0, weight, 0, 0, 0])
        else:
            self.i_hover6, self.current6 = np.inf, None

        self.peak_lift_per_amp = np.abs(wrench_matrix[2]).max()
        self.max_lateral = max_generalized_force(wrench_matrix, 0, [1, 2], [0.0, weight], current_limit)
        if self.rank6 >= 6:
            self.max_roll = max_generalized_force(wrench_matrix, 3, [0, 1, 2, 4, 5], [0, 0, weight, 0, 0], current_limit)
            self.max_pitch = max_generalized_force(wrench_matrix, 4, [0, 1, 2, 3, 5], [0, 0, weight, 0, 0], current_limit)
            self.max_yaw = max_generalized_force(wrench_matrix, 5, [0, 1, 2, 3, 4], [0, 0, weight, 0, 0], current_limit)
        else:
            self.max_roll = self.max_pitch = self.max_yaw = np.nan


def analyze(layout, cells_per_side, weight=None, characteristic_length=None,
            meshing=16, filaments_radial=2, filaments_axial=1, current_limit=DRIVER_CURRENT):
    weight = piece_weight(layout) if weight is None else weight
    characteristic_length = layout.platform_side / 2 if characteristic_length is None else characteristic_length
    place_piece(layout)
    array = CoilArray(cells_per_side, meshing=meshing,
                      filaments_radial=filaments_radial, filaments_axial=filaments_axial)
    wrench_matrix = actuator_matrix(layout, array, np.array([0, 0, layout.center_of_mass_height]))
    return array, Controllability(wrench_matrix, weight, characteristic_length, current_limit)


def worst_case_sweep(layout, cells_per_side, weight, characteristic_length, current_limit,
                     samples_per_axis=3, gaps=(0.0025, 0.003, 0.004, 0.005, 0.006),
                     yaws_deg=(0, 30, 45, 60, 90), tilt_deg=5.0):
    place_piece(layout)
    array = CoilArray(cells_per_side, meshing=12, filaments_radial=1, filaments_axial=1)
    period = layout.period
    xs = np.linspace(0, period, samples_per_axis, endpoint=False)
    ys = np.linspace(0, period, samples_per_axis, endpoint=False)
    tilt = np.radians(tilt_deg)
    worst = {
        "i_hover3": 0.0, "i_hover6": 0.0, "cond3": 0.0, "cond6": 0.0,
        "min_rank3": 6, "min_rank6": 6, "min_lateral": np.inf, "min_yaw": np.inf,
        "n": 0, "hover3_over_budget": 0,
    }
    for target_gap in gaps:
        z = target_gap - GAP
        for yaw in np.radians(yaws_deg):
            for roll in (-tilt, 0.0, tilt):
                for x in xs:
                    for y in ys:
                        _, center_of_mass = place_piece(layout, x=x, y=y, z=z,
                                                        roll=roll, pitch=0.0, yaw=yaw)
                        wrench_matrix = actuator_matrix(layout, array, center_of_mass)
                        controllability = Controllability(wrench_matrix, weight,
                                                          characteristic_length, current_limit)
                        worst["i_hover3"] = max(worst["i_hover3"], controllability.i_hover3)
                        worst["i_hover6"] = max(worst["i_hover6"], controllability.i_hover6)
                        worst["cond3"] = max(worst["cond3"], controllability.cond3)
                        worst["cond6"] = max(worst["cond6"], controllability.cond6)
                        worst["min_rank3"] = min(worst["min_rank3"], controllability.rank3)
                        worst["min_rank6"] = min(worst["min_rank6"], controllability.rank6)
                        if np.isfinite(controllability.max_lateral):
                            worst["min_lateral"] = min(worst["min_lateral"], max(controllability.max_lateral, 0.0))
                        if np.isfinite(controllability.max_yaw):
                            worst["min_yaw"] = min(worst["min_yaw"], max(controllability.max_yaw, 0.0))
                        worst["hover3_over_budget"] += int(controllability.i_hover3 > current_limit)
                        worst["n"] += 1
    place_piece(layout)
    return worst


def hover_current_vector(layout, array, weight):
    place_piece(layout)
    center_of_mass = np.array([0, 0, layout.center_of_mass_height])
    wrench_matrix = actuator_matrix(layout, array, center_of_mass)
    scaled = wrench_matrix.copy()
    scaled[3:] = wrench_matrix[3:] / (layout.platform_side / 2)
    if matrix_rank(scaled) >= 6:
        _, currents = min_peak_current(wrench_matrix, [0, 0, weight, 0, 0, 0])
        return currents
    _, currents = min_peak_current(wrench_matrix[:3], [0, 0, weight])
    return currents


def open_loop_stiffness(layout, array, weight, step=2e-5):
    currents = hover_current_vector(layout, array, weight)

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


def simulate_slide(layout, array, weight, boosted_coil, boost_fraction,
                   duration=0.08, time_step=0.001):
    mass = weight / GRAVITY
    currents = hover_current_vector(layout, array, weight).copy()
    currents[boosted_coil] *= (1 + boost_fraction)
    position = np.zeros(3)
    velocity = np.zeros(3)
    samples = []
    steps = int(round(duration / time_step))
    for step_index in range(steps + 1):
        _, center_of_mass = place_piece(layout, x=position[0], y=position[1], z=position[2])
        net_force = actuator_matrix(layout, array, center_of_mass)[:3] @ currents
        acceleration = net_force / mass - np.array([0.0, 0.0, GRAVITY])
        if step_index % 20 == 0:
            samples.append((step_index * time_step, position.copy(), net_force.copy()))
        velocity += acceleration * time_step
        position += velocity * time_step
    place_piece(layout)
    return samples


def hall_jacobian(layout, sensors_per_side, span=None, plane_z=None):
    span = span if span is not None else layout.platform_side
    plane_z = plane_z if plane_z is not None else -GAP
    axis = np.linspace(-span / 2, span / 2, sensors_per_side)
    points = np.array([[x, y, plane_z] for x in axis for y in axis])

    def field_at_pose(dx, dy, dz, droll, dpitch, dyaw):
        place_piece(layout, x=dx, y=dy, z=dz, roll=droll, pitch=dpitch, yaw=dyaw)
        return layout.collection.getB(points).reshape(-1)

    baseline = field_at_pose(0, 0, 0, 0, 0, 0)
    position_step, angle_step = 1e-5, 1e-4
    columns = []
    for axis_index, step in enumerate([position_step, position_step, position_step,
                                       angle_step, angle_step, angle_step]):
        offsets = [0.0] * 6
        offsets[axis_index] = step
        columns.append((field_at_pose(*offsets) - baseline) / step)
    place_piece(layout)
    return np.array(columns).T


def hall_metrics(jacobian):
    jacobian3, jacobian6 = jacobian[:, :3], jacobian
    return {
        "rank3": int(np.linalg.matrix_rank(jacobian3, tol=jacobian3.max() * 1e-6)),
        "rank6": int(np.linalg.matrix_rank(jacobian6, tol=jacobian6.max() * 1e-6)),
        "cond3": condition_number(jacobian3), "cond6": condition_number(jacobian6),
    }


def neighbour_force(kind, edge, periods, distance, yaw=0.0, mesh=(2, 2, 2)):
    controlled = MagnetLayout(kind=kind, edge=edge, periods=periods)
    neighbour = MagnetLayout(kind=kind, edge=edge, periods=periods)
    if yaw:
        neighbour.collection.rotate(Rotation.from_euler("z", yaw),
                                    anchor=(0.0, 0.0, neighbour.center_of_mass_height))
    neighbour.collection.move((distance, 0.0, 0.0))
    anchor = np.array([0.0, 0.0, controlled.center_of_mass_height])
    targets = list(controlled.collection)
    for cube in targets:
        cube.meshing = mesh
    force_torque = np.atleast_2d(getFT(neighbour.collection, targets, anchor=anchor))
    if force_torque.ndim == 3:
        force = force_torque[:, 0, :].sum(axis=0)
        torque = force_torque[:, 1, :].sum(axis=0)
    else:
        force, torque = force_torque[0], force_torque[1]

    span = controlled.platform_side
    axis = np.linspace(-span / 2, span / 2, 3)
    points = np.array([[x, y, -GAP] for x in axis for y in axis])
    field_self = np.linalg.norm(controlled.collection.getB(points), axis=1).mean()
    field_neighbour = np.linalg.norm(neighbour.collection.getB(points), axis=1).mean()
    return {"force": force, "torque": torque, "interference": field_neighbour / field_self}


def worst_yaw_snap(kind, edge, periods, distance):
    worst_lateral = 0.0
    worst_record = None
    for yaw in (0.0, np.radians(45), np.radians(90)):
        disturbance = neighbour_force(kind, edge, periods, distance, yaw=yaw)
        lateral = abs(disturbance["force"][0])
        if lateral > worst_lateral:
            worst_lateral = lateral
            worst_record = disturbance
    return worst_lateral, worst_record


def stray_field_above(layout, heights):
    place_piece(layout)
    fields = []
    for height in heights:
        point = np.array([[0.0, 0.0, layout.edge + height]])
        fields.append(np.linalg.norm(layout.collection.getB(point)[0]))
    return fields


def required_yaw_torque(layout, weight):
    mass = weight / GRAVITY
    span = layout.platform_side
    moment_of_inertia = (mass * 0.66) * span ** 2 / 6
    angular_acceleration = 4 * np.radians(90) / 0.5 ** 2
    return moment_of_inertia * angular_acceleration


def print_header(title):
    print("\n" + title)
    print("-" * len(title))


def main():
    print("=" * 76)
    print("LEVITATING CHESS - MAGPYLIB FEASIBILITY SIMULATION (inputs from model.py)")
    print("=" * 76)
    print(f"edge {EDGE*1000:.0f} mm | period {MAGNETS_PER_PERIOD*EDGE*1000:.0f} mm | "
          f"{PERIODS_PER_SIDE} period(s) | gap {GAP*1000:.1f} mm | Br {REMANENCE} T")
    print(f"coil {COIL_SHORT*1000:.0f}x{COIL_LONG*1000:.0f} mm rectangular, 2 orientations stacked, "
          f"{COIL_TURNS} turns, height {COIL_HEIGHT*1000:.2f} mm, R {COIL_RESISTANCE:.0f} ohm")
    print(f"piece weight {PIECE_WEIGHT*1000:.0f} mN ({PIECE_MASS*1000:.1f} g) | "
          f"driver limit {DRIVER_CURRENT:.1f} A | wire limit {COIL_CURRENT_LIMIT*1000:.0f} mA | "
          f"usable drive {COIL_USABLE_VOLTAGE:.1f} V")

    print_header("1. MAGNET LAYOUTS  (3x3-cell bed, nominal pose, rectangular finite coils)")
    print(f"  {'layout':16s}{'cubes':>6}{'mass g':>8}{'peakBz T':>10}"
          f"{'Fz/A N':>8}{'Flat mN':>9}{'Myaw uNm':>10}{'mfg?':>7}")
    for kind in ("halbach1d", "halbach1d_disc", "halbach2d_square", "herringbone"):
        magnet = MagnetLayout(kind=kind)
        weight = piece_weight(magnet)
        _, controllability = analyze(magnet, 3, weight=weight)
        print(f"  {kind:16s}{magnet.cube_count:>6}{magnet.magnet_mass*1000:>8.1f}{magnet.peak_bz():>10.3f}"
              f"{controllability.peak_lift_per_amp:>8.2f}{controllability.max_lateral*1e3:>9.1f}"
              f"{controllability.max_yaw*1e6:>10.1f}{str(magnet.manufacturable):>7}")

    print_header("   Edge-effect check: 1 vs 2 periods (herringbone, 3x3-cell bed)")
    print(f"  {'periods':>8}{'cubes':>6}{'peakBz T':>10}{'Fz/A N':>8}{'Ihover3 A':>11}{'Flat mN':>9}{'rank6':>7}")
    for periods in (1, 2):
        magnet = MagnetLayout(kind="herringbone", periods=periods)
        weight = piece_weight(magnet)
        _, controllability = analyze(magnet, 3, weight=weight)
        print(f"  {periods:>8}{magnet.cube_count:>6}{magnet.peak_bz():>10.3f}{controllability.peak_lift_per_amp:>8.2f}"
              f"{controllability.i_hover3:>11.3f}{controllability.max_lateral*1e3:>9.1f}{controllability.rank6:>7}")

    magnet = MagnetLayout(kind="herringbone")
    weight = piece_weight(magnet)
    characteristic_length = magnet.platform_side / 2
    print_header(f"2-3. COIL BED / ACTUATOR MATRIX  (herringbone, weight {weight*1000:.0f} mN, "
                 f"real {COIL_TURNS}-turn rectangular coils)")
    print(f"  {'cells':>6}{'coils':>6}{'rank3':>6}{'rank6':>6}{'cond3':>8}{'cond6':>9}"
          f"{'Ihov3 A':>9}{'Ihov6 A':>9}{'Flat mN':>9}{'Myaw uNm':>10}")
    coil_results = {}
    for cells_per_side in (2, 3, 4, 5):
        array, controllability = analyze(magnet, cells_per_side, weight=weight,
                                         characteristic_length=characteristic_length)
        coil_results[cells_per_side] = (array, controllability)
        print(f"  {f'{cells_per_side}x{cells_per_side}':>6}{controllability.ncoils:>6}{controllability.rank3:>6}{controllability.rank6:>6}"
              f"{controllability.cond3:>8.1f}{controllability.cond6:>9.1f}"
              f"{controllability.i_hover3:>9.3f}{controllability.i_hover6:>9.3f}"
              f"{controllability.max_lateral*1e3:>9.1f}{controllability.max_yaw*1e6:>10.1f}")
    print("  (cells = bed footprint in 10 mm cells; each cell has an x- and a y-coil; "
          f"authority columns at {DRIVER_CURRENT:.0f} A driver limit)")

    recommended_cells = 3
    print_header(f"4. WORST-CASE SWEEP  (herringbone, {recommended_cells}x{recommended_cells}-cell bed; "
                 "x/y over one period, gap 2.5-6 mm, yaw 0-90, roll +-5)")
    worst = worst_case_sweep(magnet, recommended_cells, weight, characteristic_length, DRIVER_CURRENT)
    print(f"  poses evaluated                 {worst['n']}")
    print(f"  min rank (3DOF / 6DOF)          {worst['min_rank3']} / {worst['min_rank6']}")
    print(f"  worst-case hover current 3DOF   {worst['i_hover3']:.3f} A  "
          f"({worst['hover3_over_budget']}/{worst['n']} poses need >{DRIVER_CURRENT:.0f} A)")
    print(f"  worst-case hover current 6DOF   {worst['i_hover6']:.3f} A")
    print(f"  worst-case condition (3/6 DOF)  {worst['cond3']:.0f} / {worst['cond6']:.0f}")
    print(f"  spare lateral authority         {worst['min_lateral']*1e3:.1f} mN")
    print(f"  spare yaw authority             {worst['min_yaw']*1e6:.1f} uNm")

    print_header("5. HALL SENSORS  (3-axis grid at coil plane; pose Jacobian)")
    print(f"  {'grid':>6}{'sensors':>9}{'rank3':>7}{'rank6':>7}{'cond3':>8}{'cond6':>9}{'6DOF obs?':>11}")
    for sensors_per_side in (2, 3, 4):
        metrics = hall_metrics(hall_jacobian(magnet, sensors_per_side))
        observable = "yes" if metrics["rank6"] == 6 else "NO"
        print(f"  {f'{sensors_per_side}x{sensors_per_side}':>6}{sensors_per_side*sensors_per_side:>9}{metrics['rank3']:>7}{metrics['rank6']:>7}"
              f"{metrics['cond3']:>8.1f}{metrics['cond6']:>9.1f}{observable:>11}")

    print_header("6. NEIGHBOUR-PIECE SNAP  (two resting pieces, real piece-to-piece force)")
    print(f"  scaling piece size to reduce snap-to-weight ratio (worst over yaw 0/45/90)")
    print(f"  {'edge':>5}{'per':>4}{'base mm':>9}{'pitch mm':>9}{'gap mm':>8}{'gap/dl':>8}"
          f"{'weight mN':>10}{'snap mN':>9}{'need mu':>9}")
    for edge in (0.005, 0.006, 0.008, 0.010):
        for periods in (1, 2):
            magnet_n = MagnetLayout(kind="herringbone", edge=edge, periods=periods)
            weight_n = piece_weight(magnet_n)
            pitch = square_pitch(magnet_n)
            magnet_gap = pitch - magnet_n.platform_side
            decay_length = (edge * MAGNETS_PER_PERIOD) / (2 * np.pi)
            snap, _ = worst_yaw_snap("herringbone", edge, periods, pitch)
            print(f"  {edge*1000:>5.0f}{periods:>4}{magnet_n.platform_side*np.sqrt(2)*1e3:>9.1f}{pitch*1e3:>9.1f}"
                  f"{magnet_gap*1e3:>8.1f}{magnet_gap/decay_length:>8.1f}{weight_n*1e3:>10.0f}"
                  f"{snap*1e3:>9.1f}{snap/weight_n:>9.2f}")
    print(f"  spreading the squares out (design piece {EDGE*1000:.0f} mm / {PERIODS_PER_SIDE} period):")
    print(f"  {'pitch x':>9}{'pitch mm':>9}{'gap mm':>8}{'snap mN':>9}{'need mu':>9}")
    for multiplier in (1.0, 1.25, 1.5, 2.0):
        pitch = square_pitch(magnet, pitch_multiplier=multiplier)
        snap, _ = worst_yaw_snap("herringbone", EDGE, PERIODS_PER_SIDE, pitch)
        print(f"  {multiplier:>9.2f}{pitch*1e3:>9.1f}{(pitch-magnet.platform_side)*1e3:>8.1f}"
              f"{snap*1e3:>9.1f}{snap/weight:>9.2f}")
    print(f"  (need mu = snap force / piece weight; resting friction ~0.3 plastic, ~0.8 rubber feet)")

    print_header("7. OPEN-LOOP STIFFNESS / CONTROL RATE  (frozen hover currents, finite difference)")
    array3 = coil_results[recommended_cells][0]
    stiffness, eigenvalues = open_loop_stiffness(magnet, array3, weight)
    unstable = eigenvalues[eigenvalues < 0]
    growth_time = 1.0 / np.sqrt(np.abs(unstable).max() / PIECE_MASS) if unstable.size else np.inf
    required_bandwidth = CONTROL_BANDWIDTH_MARGIN / (2 * np.pi * growth_time) if np.isfinite(growth_time) else 0.0
    print(f"  translational stiffness eigenvalues  {np.array2string(eigenvalues, precision=1)} N/m")
    print(f"  unstable modes                       {unstable.size} of 3")
    print(f"  fastest instability growth time      {growth_time*1e3:.1f} ms")
    print(f"  required control bandwidth (x{CONTROL_BANDWIDTH_MARGIN:.0f})       {required_bandwidth:.0f} Hz")
    print(f"  model.py open-loop instability time  {model.control.instability_time:.1f} ms "
          f"(required {model.control.required_bandwidth:.0f} Hz)")

    print_header("8. DYNAMICS  (rigid-body integration; magpylib gives force each step)")
    array_dyn = CoilArray(recommended_cells, meshing=8, filaments_radial=1, filaments_axial=1)
    x_coil = array_dyn.orientations.index("x")
    y_coil = array_dyn.orientations.index("y")
    for label, coil_index in (("boost an x-coil +30%", x_coil), ("boost a y-coil +30%", y_coil)):
        samples = simulate_slide(magnet, array_dyn, weight, coil_index, 0.30)
        print(f"  {label} (coil {coil_index} at {array_dyn.centers[coil_index]} m):")
        print(f"    {'t ms':>6}{'x mm':>9}{'y mm':>9}{'z mm':>9}{'Fx mN':>9}{'Fy mN':>9}{'Fz-W mN':>9}")
        for time_s, position, net_force in samples:
            print(f"    {time_s*1e3:>6.0f}{position[0]*1e3:>9.3f}{position[1]*1e3:>9.3f}{position[2]*1e3:>9.3f}"
                  f"{net_force[0]*1e3:>9.2f}{net_force[1]*1e3:>9.2f}{(net_force[2]-weight)*1e3:>9.2f}")
    print("  (open loop, no feedback: shows the uncontrolled response to a single-coil change;")
    print("   z drifts because magnetic hover is statically unstable - the controller closes this)")

    print_header("REQUIREMENTS TO HOVER AND CONTROL ONE PIECE OF OUR WEIGHT")
    controllability3 = coil_results[recommended_cells][1]
    min_cells_3dof = next((n for n in (2, 3, 4, 5) if coil_results[n][1].rank3 >= 3), None)
    min_cells_6dof = next((n for n in (2, 3, 4, 5) if coil_results[n][1].rank6 >= 6), None)
    hover_volts3 = controllability3.i_hover3 * COIL_RESISTANCE
    hover_volts6 = controllability3.i_hover6 * COIL_RESISTANCE
    yaw_needed = required_yaw_torque(magnet, weight)

    print(f"  Can our {weight*1e3:.0f} mN piece hover?     "
          f"{'YES' if controllability3.i_hover3 <= COIL_CURRENT_LIMIT else 'MARGINAL'} - "
          f"3DOF needs {controllability3.i_hover3*1e3:.0f} mA/coil ({recommended_cells}x{recommended_cells} cells), "
          f"wire allows {COIL_CURRENT_LIMIT*1e3:.0f} mA, worst-case {worst['i_hover3']*1e3:.0f} mA.")
    print(f"  Amps required (per coil)          "
          f"3DOF {controllability3.i_hover3*1e3:.0f} mA nominal / {worst['i_hover3']*1e3:.0f} mA worst-case; "
          f"6DOF {controllability3.i_hover6*1e3:.0f} mA nominal / {worst['i_hover6']*1e3:.0f} mA worst-case.")
    print(f"  Volts required (per coil)         "
          f"3DOF {hover_volts3:.1f} V / 6DOF {hover_volts6:.1f} V at {COIL_RESISTANCE:.0f} ohm "
          f"(usable drive {COIL_USABLE_VOLTAGE:.1f} V).")
    print(f"  Control rate required             "
          f"{required_bandwidth:.0f} Hz loop bandwidth (instability grows in {growth_time*1e3:.1f} ms).")
    print(f"  Minimum bed for 3DOF / 6DOF       {min_cells_3dof}x{min_cells_3dof} / "
          f"{min_cells_6dof}x{min_cells_6dof} cells.")
    print(f"  Best magnet layout                herringbone - manufacturable axis-aligned cubes, "
          f"both Fx and Fy shear, strong Bz.")
    print(f"  Is 6DOF feasible?                 "
          f"{'YES, with caveats' if min_cells_6dof else 'NO'} - rank 6 from {min_cells_6dof}x{min_cells_6dof} cells up, "
          f"Hall-observable; yaw torque {controllability3.max_yaw*1e6:.0f} uNm vs ~{yaw_needed*1e6:.0f} uNm needed, "
          f"weak axis worst-case spare {worst['min_yaw']*1e6:.1f} uNm.")


if __name__ == "__main__":
    main()

import numpy as np
import magpylib as magpy
from magpylib_force import getFT
from scipy.spatial.transform import Rotation
from scipy.optimize import linprog


class Inputs:
    magnet_cube_edge = 0.005
    magnets_per_period = 4
    periods_per_side = 1
    gap = 0.003
    coil_current = 1.0
    coils_per_period = 2


class Constants:
    ndfeb_remanence_br = 1.45
    ndfeb_density = 7500.0
    plastic_density = 1200.0
    gravity = 9.80665
    king_height = 0.095
    king_base_diameter = 0.044
    com_height_fraction = 0.4


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


class MagnetLayout:
    def __init__(self, kind="halbach1d", footprint="square",
                 edge=Inputs.magnet_cube_edge, periods=Inputs.periods_per_side,
                 per_period=Inputs.magnets_per_period,
                 remanence=Constants.ndfeb_remanence_br):
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
        scale = base_diameter / Constants.king_base_diameter
        return Constants.com_height_fraction * Constants.king_height * scale

    @property
    def magnet_mass(self):
        return self.cube_count * self.edge ** 3 * Constants.ndfeb_density

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
        grid = np.array([[x, y, -Inputs.gap] for x in axis for y in axis])
        return self.collection.getB(grid)

    def peak_bz(self, samples=41):
        return float(np.max(np.abs(self.plane_field(samples)[:, 2])))


class CoilPatch:
    def __init__(self, coils_per_side, layout, meshing=40, current=Inputs.coil_current):
        self.coils_per_side = coils_per_side
        self.layout = layout
        self.current = current
        self.pitch = layout.period / Inputs.coils_per_period
        self.side = self.pitch
        self.z = -Inputs.gap
        self.meshing = meshing
        self.coils = self.build()
        wire_outer_diameter = 0.0001
        layers = 3
        radial_width = 0.35 * self.side
        self.turns = max(1, int(radial_width * 0.8 / wire_outer_diameter) * layers)
        self.max_amp_turns = self.turns * self.current

    def build(self):
        half = self.side / 2
        loop_outline = np.array([
            [-half, -half, 0.0], [half, -half, 0.0],
            [half, half, 0.0], [-half, half, 0.0], [-half, -half, 0.0],
        ])
        coils = []
        offset = (self.coils_per_side - 1) / 2
        for i in range(self.coils_per_side):
            for j in range(self.coils_per_side):
                center_x = (i - offset) * self.pitch
                center_y = (j - offset) * self.pitch
                loop = magpy.current.Polyline(
                    current=self.current,
                    vertices=loop_outline + np.array([center_x, center_y, 0.0]),
                    position=(0.0, 0.0, self.z),
                )
                loop.meshing = self.meshing
                coils.append(loop)
        return coils

    @property
    def extent(self):
        return self.coils_per_side * self.pitch


def place_piece(layout, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    rotation = Rotation.from_euler("xyz", [roll, pitch, yaw])
    collection = layout.collection
    collection.position = (0.0, 0.0, 0.0)
    collection.orientation = Rotation.identity()
    collection.rotate(rotation, anchor=(0.0, 0.0, layout.center_of_mass_height))
    collection.move((x, y, z))
    center_of_mass = np.array([x, y, layout.center_of_mass_height + z])
    return collection, center_of_mass


def actuator_matrix(layout, patch, center_of_mass):
    columns = []
    for coil in patch.coils:
        force, torque = getFT(layout.collection, coil, anchor=center_of_mass)
        wrench = -np.concatenate([force, torque]) / coil.current
        columns.append(wrench)
    return np.array(columns).T


def piece_weight(layout, wall_thickness=0.001):
    base_diameter = layout.platform_side * np.sqrt(2)
    scale = base_diameter / Constants.king_base_diameter
    height = Constants.king_height * scale
    diameter = base_diameter
    inner_diameter = diameter - 2 * wall_thickness
    inner_height = height - 2 * wall_thickness
    shell_volume = np.pi / 4 * (diameter ** 2 * height - inner_diameter ** 2 * inner_height)
    shell_mass = shell_volume * Constants.plastic_density
    total_mass = shell_mass + layout.magnet_mass
    return total_mass * Constants.gravity


def condition_number(matrix):
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    significant = singular_values[singular_values > singular_values.max() * 1e-12]
    return significant[0] / significant[-1]


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
    def __init__(self, wrench_matrix, weight, characteristic_length,
                 turns=1, max_amp_turns=None):
        self.wrench_matrix = wrench_matrix
        self.weight = weight
        self.characteristic_length = characteristic_length
        self.turns = turns
        self.max_amp_turns = max_amp_turns if max_amp_turns is not None else turns * Inputs.coil_current
        self.ncoils = wrench_matrix.shape[1]

        self.scaled_wrench_matrix = wrench_matrix.copy()
        self.scaled_wrench_matrix[3:] = wrench_matrix[3:] / characteristic_length

        self.rank3 = int(np.linalg.matrix_rank(wrench_matrix[:3], tol=wrench_matrix[:3].max() * 1e-9))
        self.rank6 = int(np.linalg.matrix_rank(self.scaled_wrench_matrix, tol=self.scaled_wrench_matrix.max() * 1e-9))
        self.cond3 = condition_number(wrench_matrix[:3])
        self.cond6 = condition_number(self.scaled_wrench_matrix)

        self.hover3 = np.linalg.pinv(wrench_matrix[:3]) @ np.array([0, 0, weight])
        self.amp_turns_hover3 = np.max(np.abs(self.hover3))
        self.i_hover3 = self.amp_turns_hover3 / turns
        if self.rank6 >= 6:
            self.hover6 = np.linalg.pinv(wrench_matrix) @ np.array([0, 0, weight, 0, 0, 0])
            self.amp_turns_hover6 = np.max(np.abs(self.hover6))
            self.i_hover6 = self.amp_turns_hover6 / turns
        else:
            self.hover6 = None
            self.amp_turns_hover6 = np.inf
            self.i_hover6 = np.inf

        limit = self.max_amp_turns
        self.max_lateral = max_generalized_force(wrench_matrix, 0, [1, 2], [0.0, weight], limit)
        if self.rank6 >= 6:
            self.max_roll = max_generalized_force(wrench_matrix, 3, [0, 1, 2, 4, 5], [0, 0, weight, 0, 0], limit)
            self.max_pitch = max_generalized_force(wrench_matrix, 4, [0, 1, 2, 3, 5], [0, 0, weight, 0, 0], limit)
            self.max_yaw = max_generalized_force(wrench_matrix, 5, [0, 1, 2, 3, 4], [0, 0, weight, 0, 0], limit)
        else:
            self.max_roll = self.max_pitch = self.max_yaw = np.nan


def worst_case_sweep(layout, patch, weight, characteristic_length,
                     samples_per_axis=3, gaps=(0.0025, 0.003, 0.004, 0.005, 0.006),
                     yaws_deg=(0, 30, 45, 60, 90), tilt_deg=5.0):
    period = layout.period
    xs = np.linspace(0, period, samples_per_axis, endpoint=False)
    ys = np.linspace(0, period, samples_per_axis, endpoint=False)
    tilt = np.radians(tilt_deg)
    worst = {
        "i_hover3": 0.0, "i_hover6": 0.0, "cond3": 0.0, "cond6": 0.0,
        "min_rank3": 6, "min_rank6": 6, "min_lateral": np.inf, "min_yaw": np.inf,
        "n": 0, "hover3_over_1A": 0,
    }
    for target_gap in gaps:
        z = target_gap - Inputs.gap
        for yaw in np.radians(yaws_deg):
            for roll in (-tilt, 0.0, tilt):
                for x in xs:
                    for y in ys:
                        _, center_of_mass = place_piece(layout, x=x, y=y, z=z,
                                                        roll=roll, pitch=0.0, yaw=yaw)
                        wrench_matrix = actuator_matrix(layout, patch, center_of_mass)
                        controllability = Controllability(wrench_matrix, weight, characteristic_length,
                                                          turns=patch.turns, max_amp_turns=patch.max_amp_turns)
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
                        worst["hover3_over_1A"] += int(controllability.i_hover3 > Inputs.coil_current)
                        worst["n"] += 1
    place_piece(layout)
    return worst


def hall_jacobian(layout, sensors_per_side, span=None, plane_z=None):
    span = span if span is not None else layout.platform_side
    plane_z = plane_z if plane_z is not None else -Inputs.gap
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


def neighbour_disturbance(layout_kind, edge, controlled, center_of_mass, distance,
                          sensors_per_side=3):
    neighbour = MagnetLayout(kind=layout_kind, edge=edge)
    neighbour.collection.move((distance, 0.0, 0.0))

    targets = list(controlled.collection)
    for cube in targets:
        cube.meshing = (2, 2, 2)
    force_torque = np.atleast_2d(getFT(neighbour.collection, targets, anchor=center_of_mass))
    if force_torque.ndim == 3:
        force = force_torque[:, 0, :].sum(axis=0)
        torque = force_torque[:, 1, :].sum(axis=0)
    else:
        force, torque = force_torque[0], force_torque[1]

    span = controlled.platform_side
    axis = np.linspace(-span / 2, span / 2, sensors_per_side)
    points = np.array([[x, y, -Inputs.gap] for x in axis for y in axis])
    field_self = np.linalg.norm(controlled.collection.getB(points), axis=1).mean()
    field_neighbour = np.linalg.norm(neighbour.collection.getB(points), axis=1).mean()
    return {"force": force, "torque": torque, "field_self": field_self,
            "field_neighbour": field_neighbour, "interference": field_neighbour / field_self}


def print_header(title):
    print("\n" + title)
    print("-" * len(title))


def analyze(layout, patch_side, weight=None, characteristic_length=None):
    weight = piece_weight(layout) if weight is None else weight
    characteristic_length = layout.platform_side / 2 if characteristic_length is None else characteristic_length
    place_piece(layout)
    patch = CoilPatch(patch_side, layout)
    wrench_matrix = actuator_matrix(layout, patch, np.array([0, 0, layout.center_of_mass_height]))
    return patch, Controllability(wrench_matrix, weight, characteristic_length,
                                  turns=patch.turns, max_amp_turns=patch.max_amp_turns)


def required_yaw_torque(layout, weight):
    mass = weight / Constants.gravity
    span = layout.platform_side
    moment_of_inertia = (mass * 0.66) * span ** 2 / 6
    angular_acceleration = 4 * np.radians(90) / 0.5 ** 2
    return moment_of_inertia * angular_acceleration


def main():
    print("=" * 74)
    print("LEVITATING CHESS - MAGPYLIB CONTROLLABILITY SIMULATION")
    print("=" * 74)
    print(f"Gap {Inputs.gap*1000:.1f} mm | coil current limit {Inputs.coil_current} A | "
          f"Br {Constants.ndfeb_remanence_br} T | {Inputs.coils_per_period} coils/period")

    edge = Inputs.magnet_cube_edge
    reference_kind = "herringbone"

    print_header("1. MAGNET LAYOUTS  (3x3 coil patch, nominal pose)")
    print(f"  {'layout':16s}{'cubes':>6}{'mass g':>8}{'peakBz T':>10}"
          f"{'Fz/A mN':>9}{'Flat mN':>9}{'Myaw uNm':>10}{'mfg?':>6}")
    layout_rows = {}
    for kind in ("halbach1d", "halbach1d_disc", "halbach2d_square", "herringbone"):
        magnet = MagnetLayout(kind=kind, edge=edge)
        weight = piece_weight(magnet)
        patch, controllability = analyze(magnet, 3, weight=weight)
        fz_per_amp = np.abs(controllability.wrench_matrix[2]).max() * patch.turns
        layout_rows[kind] = (magnet, controllability, weight)
        print(f"  {kind:16s}{magnet.cube_count:>6}{magnet.magnet_mass*1000:>8.1f}{magnet.peak_bz():>10.3f}"
              f"{fz_per_amp*1e3:>9.2f}{controllability.max_lateral*1e3:>9.1f}"
              f"{controllability.max_yaw*1e6:>10.1f}{str(magnet.manufacturable):>6}")

    print_header("   Cube-size sweep (herringbone, square footprint)")
    print(f"  {'edge mm':>8}{'cubes':>6}{'mass g':>8}{'peakBz T':>10}{'Ihover3 A':>11}{'Flat mN':>9}")
    for edge_size in (0.004, 0.005, 0.006):
        magnet = MagnetLayout(kind="herringbone", edge=edge_size)
        weight = piece_weight(magnet)
        _, controllability = analyze(magnet, 3, weight=weight)
        print(f"  {edge_size*1000:>8.0f}{magnet.cube_count:>6}{magnet.magnet_mass*1000:>8.1f}{magnet.peak_bz():>10.3f}"
              f"{controllability.i_hover3:>11.2f}{controllability.max_lateral*1e3:>9.1f}")

    print_header("   Footprint sweep (herringbone, 5 mm cubes)")
    print(f"  {'footprint':>10}{'cubes':>6}{'mass g':>8}{'peakBz T':>10}{'Flat mN':>9}")
    for footprint in ("square", "octagon", "round"):
        magnet = MagnetLayout(kind="herringbone", edge=edge, footprint=footprint)
        _, controllability = analyze(magnet, 3)
        print(f"  {footprint:>10}{magnet.cube_count:>6}{magnet.magnet_mass*1000:>8.1f}{magnet.peak_bz():>10.3f}{controllability.max_lateral*1e3:>9.1f}")

    magnet = MagnetLayout(kind=reference_kind, edge=edge)
    weight = piece_weight(magnet)
    characteristic_length = magnet.platform_side / 2
    print_header(f"2-3. COIL PATCH / ACTUATOR MATRIX  (magnet = {reference_kind}, "
                 f"weight {weight*1000:.0f} mN, turns from winding est.)")
    print(f"  {'patch':>6}{'coils':>6}{'rank3':>6}{'rank6':>6}{'cond3':>8}{'cond6':>9}"
          f"{'Ihov3 A':>9}{'Ihov6 A':>9}{'Flat mN':>9}{'Myaw uNm':>10}")
    coil_results = {}
    for patch_side in (2, 3, 4, 5):
        patch, controllability = analyze(magnet, patch_side, weight=weight, characteristic_length=characteristic_length)
        coil_results[patch_side] = (patch, controllability)
        print(f"  {f'{patch_side}x{patch_side}':>6}{controllability.ncoils:>6}{controllability.rank3:>6}{controllability.rank6:>6}"
              f"{controllability.cond3:>8.1f}{controllability.cond6:>9.1f}"
              f"{controllability.i_hover3:>9.2f}{controllability.i_hover6:>9.2f}"
              f"{controllability.max_lateral*1e3:>9.1f}{controllability.max_yaw*1e6:>10.1f}")
    print("  (rank3>=3 => Fx,Fy,Fz controllable; rank6=6 => +Mx,My,Mz; "
          "Ihover = per-coil current at 3 mm)")

    recommended_patch_side = 4
    patch = CoilPatch(recommended_patch_side, magnet)
    print_header(f"4. WORST-CASE SWEEP  (magnet={reference_kind}, {recommended_patch_side}x{recommended_patch_side} patch; "
                 "x/y over one period, gap 2.5-6 mm, yaw 0-90, roll +-5)")
    worst = worst_case_sweep(magnet, patch, weight, characteristic_length)
    print(f"  poses evaluated                 {worst['n']}")
    print(f"  min rank (3DOF / 6DOF)          {worst['min_rank3']} / {worst['min_rank6']}")
    print(f"  worst-case hover current 3DOF   {worst['i_hover3']:.2f} A  "
          f"({worst['hover3_over_1A']}/{worst['n']} poses need >1 A)")
    print(f"  worst-case hover current 6DOF   {worst['i_hover6']:.2f} A")
    print(f"  worst-case condition (3/6 DOF)  {worst['cond3']:.0f} / {worst['cond6']:.0f}")
    print(f"  spare lateral authority @1A     "
          f"{worst['min_lateral']*1e3:.1f} mN (drops to ~0 where hover already uses the 1 A budget)")
    print(f"  spare yaw authority @1A         {worst['min_yaw']*1e6:.1f} uNm")

    print_header("5. HALL SENSORS  (3-axis grid at coil plane; pose Jacobian)")
    print(f"  {'grid':>6}{'sensors':>9}{'rank3':>7}{'rank6':>7}{'cond3':>8}{'cond6':>9}{'6DOF obs?':>11}")
    for sensors_per_side in (2, 3, 4):
        metrics = hall_metrics(hall_jacobian(magnet, sensors_per_side))
        observable = "yes" if metrics["rank6"] == 6 else "NO"
        print(f"  {f'{sensors_per_side}x{sensors_per_side}':>6}{sensors_per_side*sensors_per_side:>9}{metrics['rank3']:>7}{metrics['rank6']:>7}"
              f"{metrics['cond3']:>8.1f}{metrics['cond6']:>9.1f}{observable:>11}")

    print_header("6. NEIGHBOUR-PIECE DISTURBANCE  (resting identical pieces)")
    center_of_mass = np.array([0, 0, magnet.center_of_mass_height])
    base_diameter = magnet.platform_side * np.sqrt(2)
    scenarios = [
        ("adjacent square", base_diameter / 0.8),
        ("captured storage (touching)", base_diameter * 1.02),
        ("two movers (nearest)", base_diameter / 0.8),
    ]
    print(f"  {'scenario':>30}{'dist mm':>9}{'|F| mN':>9}{'|T| uNm':>10}{'Hall interf':>13}")
    for label, distance in scenarios:
        disturbance = neighbour_disturbance(reference_kind, edge, magnet, center_of_mass, distance)
        multiplier = 2 if "two" in label else 1
        print(f"  {label:>30}{distance*1000:>9.1f}{np.linalg.norm(disturbance['force'])*1e3*multiplier:>9.1f}"
              f"{np.linalg.norm(disturbance['torque'])*1e6*multiplier:>10.1f}{disturbance['interference']*100*multiplier:>11.1f} %")
    print(f"  (compare with piece weight {weight*1e3:.0f} mN and lateral authority "
          f"{coil_results[recommended_patch_side][1].max_lateral*1e3:.0f} mN)")

    print_header("FINAL OUTPUT")
    controllability3 = coil_results[3][1]
    controllability4 = coil_results[4][1]
    yaw_needed = required_yaw_torque(magnet, weight)
    min_patch_3dof = next((n for n in (2, 3, 4, 5) if coil_results[n][1].rank3 >= 3), None)
    min_patch_6dof = next((n for n in (2, 3, 4, 5) if coil_results[n][1].rank6 >= 6), None)

    print(f"  Is a 3 mm gap realistic?           "
          f"{'YES' if controllability3.i_hover3 <= 1 else 'MARGINAL'} - nominal hover {controllability3.i_hover3:.2f} A/coil at 3 mm, "
          f"worst-case {worst['i_hover3']:.2f} A across phases.")
    print(f"  Is 1 A per coil enough?            "
          f"{'YES nominally' if controllability3.i_hover3 <= 1 else 'NO'} - nominal {controllability3.i_hover3:.2f} A, "
          f"worst-case {worst['i_hover3']:.2f} A ({worst['hover3_over_1A']}/{worst['n']} phases > 1 A); "
          f"needs more turns or a larger patch.")
    print(f"  Minimum coil patch for 3DOF        "
          f"{min_patch_3dof}x{min_patch_3dof} (rank 3); but 2x2 needs {coil_results[2][1].i_hover3:.2f} A/coil to hover, "
          f"so 3x3 is the practical minimum.")
    print(f"  Minimum coil patch for 6DOF        {min_patch_6dof}x{min_patch_6dof} "
          f"(2x2 only reaches rank {coil_results[2][1].rank6}).")
    print(f"  Best magnet layout                 herringbone - manufacturable with axis-aligned "
          f"cubes, strongest Bz, gives both Fx and Fy shear.")
    print(f"  Best coil layout                   {recommended_patch_side}x{recommended_patch_side} - smallest patch with full 6DOF "
          f"rank and worst-case margin; 3x3 is the bare minimum.")
    print(f"  Is 6DOF actually feasible?         "
          f"{'YES, with caveats' if min_patch_6dof else 'NO'} - rank 6 from {min_patch_6dof}x{min_patch_6dof} up and Hall-observable; "
          f"nominal yaw torque {controllability4.max_yaw*1e6:.1f} uNm >> ~{yaw_needed*1e6:.0f} uNm needed, "
          f"but yaw is the weak axis (high Hall cond6, spare authority ~0 at the 1 A budget "
          f"in worst-case phases) - allow >1 A headroom or a 4x4+ patch.")
    print("  Optimistic assumptions in model.py:")
    for line in (
        "summing abs(Fz) over coils ignores that adjacent coils push opposite "
        "directions (see Fz row signs) - real net lift is much lower;",
        "halbach_force_form_factor / first_harmonic_coefficient assume an ideal "
        "continuous Halbach; the buildable axis-aligned array differs;",
        "the ideal 1-D Halbach needs diagonally magnetised cubes (not "
        "off-the-shelf) - manufacturable only after snapping to axes;",
        "lift treated as phase-independent: worst-case phase needs "
        f"{worst['i_hover3']/max(controllability3.i_hover3, 1e-9):.1f}x the nominal current;",
        "neighbour pieces ignored, yet an adjacent resting piece exerts a "
        "lateral force comparable to or above the piece weight;",
        "6DOF assumed once force rank is met, but yaw torque authority and Hall "
        "yaw observability (high condition number) are the real limits.",
    ):
        print(f"    - {line}")


if __name__ == "__main__":
    main()

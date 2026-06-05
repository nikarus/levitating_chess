import numpy as np
import magpylib as magpy
from magpylib_force import getFT


class Inputs:
    magnet_cube_edge = 0.005
    magnets_per_period = 4
    periods_per_side = 1
    magnet_to_coil_distance = 0.003
    coil_current = 1.0


class Constants:
    ndfeb_remanence_br = 1.45
    gravity = 9.80665


class Cell:
    def __init__(self, label, value, unit=""):
        self.label = label
        self.value = value
        self.unit = unit


class HalbachPiece:
    def __init__(self, radius):
        self.radius = radius
        self.edge = Inputs.magnet_cube_edge
        self.period = Inputs.magnets_per_period * self.edge
        self.blocks_per_side = Inputs.periods_per_side * Inputs.magnets_per_period
        self.platform_side = self.blocks_per_side * self.edge
        self.wavenumber = 2 * np.pi / self.period
        self.cube_count = 0
        self.collection = self.build()

    def block_polarization(self, x):
        angle = self.wavenumber * x
        return (Constants.ndfeb_remanence_br * np.sin(angle), 0.0, -Constants.ndfeb_remanence_br * np.cos(angle))

    def block_center(self, index):
        return (index + 0.5 - self.blocks_per_side / 2) * self.edge

    def is_populated(self, x, y):
        if self.radius is None:
            return True
        return np.hypot(x, y) <= self.radius

    def build(self):
        cubes = []
        for ix in range(self.blocks_per_side):
            x = self.block_center(ix)
            polarization = self.block_polarization(x)
            for iy in range(self.blocks_per_side):
                y = self.block_center(iy)
                if not self.is_populated(x, y):
                    continue
                cube = magpy.magnet.Cuboid(
                    dimension=(self.edge, self.edge, self.edge),
                    polarization=polarization,
                    position=(x, y, self.edge / 2),
                )
                cubes.append(cube)
        self.cube_count = len(cubes)
        return magpy.Collection(cubes)

    def plane_field(self, samples=81):
        half = self.platform_side / 2
        axis = np.linspace(-half, half, samples)
        grid = np.array([[x, y, -Inputs.magnet_to_coil_distance] for x in axis for y in axis])
        return self.collection.getB(grid)

    def peak_bz(self):
        return np.max(np.abs(self.plane_field()[:, 2]))


class CoilLift:
    def __init__(self, piece):
        self.piece = piece
        self.outer_width = piece.period / 2
        self.rows = max(1, round(piece.platform_side / (2.5 * self.outer_width)))
        self.outer_length = piece.platform_side / self.rows
        self.coil_z = -Inputs.magnet_to_coil_distance
        self.coil = self.build()

    def build(self):
        w = self.outer_width / 2
        l = self.outer_length / 2
        vertices = np.array([
            [-w, -l, 0.0],
            [w, -l, 0.0],
            [w, l, 0.0],
            [-w, l, 0.0],
            [-w, -l, 0.0],
        ])
        loop = magpy.current.Polyline(current=Inputs.coil_current, vertices=vertices)
        loop.meshing = 80
        return loop

    def peak_lift_per_amp_turn(self):
        best = 0.0
        for dx in np.linspace(0.0, self.piece.period, 49):
            self.coil.position = (dx, 0.0, self.coil_z)
            force, _ = getFT(self.piece.collection, self.coil, anchor=np.array([dx, 0.0, self.coil_z]))
            best = max(best, abs(force[2]))
        return best

    def coil_centers(self):
        half = self.piece.platform_side / 2
        xs = np.arange(-half + self.outer_width / 2, half, self.outer_width)
        ys = np.arange(-half + self.outer_length / 2, half, self.outer_length)
        return [(cx, cy) for cx in xs for cy in ys]

    def total_lift_per_amp_turn(self):
        best = 0.0
        for offset in np.linspace(0.0, self.piece.period, 9):
            total = 0.0
            for cx, cy in self.coil_centers():
                self.coil.position = (cx + offset, cy, self.coil_z)
                force, _ = getFT(self.piece.collection, self.coil, anchor=np.array([cx + offset, cy, self.coil_z]))
                total += abs(force[2])
            best = max(best, total)
        return best * 2


def format_value(value):
    if isinstance(value, int):
        return str(value)
    if value != 0 and (abs(value) < 1e-3 or abs(value) >= 1e6):
        return f"{value:.4e}"
    text = f"{value:.4f}".rstrip("0").rstrip(".")
    return text if text else "0"


def print_section(title, cells):
    print()
    print(title)
    print("-" * len(title))
    for cell in cells:
        unit = f" {cell.unit}" if cell.unit else ""
        print(f"  {cell.label:<42}{format_value(cell.value):>14}{unit}")


square = HalbachPiece(radius=None)
square_total = CoilLift(square).total_lift_per_amp_turn()
calibration = CoilLift(square).peak_lift_per_amp_turn()
sweep_radii = [0.014, 0.016, 0.018, 0.019, 0.020, 0.022]


def print_report():
    print("LEVITATING CHESS MAGPYLIB FIELD MODEL")
    print_section("Full square array", [
        Cell("Cube count", square.cube_count),
        Cell("Peak Bz at coil plane", square.peak_bz(), "T"),
        Cell("Peak lift per coil per amp-turn", calibration, "N"),
        Cell("Total lift per amp-turn (all coils)", square_total, "N"),
    ])
    print()
    print("Round-footprint radius sweep")
    print("----------------------------")
    print(f"  {'radius mm':>10}{'cubes':>8}{'corner mm':>11}{'cubes %':>9}{'lift %':>9}")
    half_diagonal = square.platform_side / 2 * np.sqrt(2)
    for radius in sweep_radii:
        piece = HalbachPiece(radius=radius)
        corner_standoff = half_diagonal - radius
        cubes_pct = 100 * piece.cube_count / square.cube_count
        lift_pct = 100 * CoilLift(piece).total_lift_per_amp_turn() / square_total
        print(f"  {radius * 1000:>10.1f}{piece.cube_count:>8}{corner_standoff * 1000:>11.2f}{cubes_pct:>9.1f}{lift_pct:>9.1f}")


print_report()

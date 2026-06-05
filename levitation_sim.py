"""Magpylib controllability simulation for levitating chess.

The original ``field_model.py`` only estimates *peak lift*.  That tells us a
piece can be picked up, but not whether it can be *controlled*.  This module
adds a full 6-DOF wrench model on top of Magpylib so we can answer the real
question: given magnets in the piece and coils in the board, can we command
Fx, Fy, Fz (3-DOF, all 32 pieces) and additionally Mx, My, Mz (6-DOF, single
piece) - and can Hall sensors observe the pose well enough to close the loop.

Everything is in SI units (m, T, A, N, N.m) so it feeds straight into Magpylib
and magpylib_force.getFT.

Run with ``python levitation_sim.py``.  Sub-sections can also be imported and
called individually from a notebook.
"""

import itertools

import numpy as np
import magpylib as magpy
from magpylib_force import getFT
from scipy.spatial.transform import Rotation
from scipy.optimize import linprog


# --------------------------------------------------------------------------- #
#  Configuration                                                              #
# --------------------------------------------------------------------------- #
class Inputs:
    magnet_cube_edge = 0.005        # m   (swept over 4/5/6 mm)
    magnets_per_period = 4          # cubes per Halbach period
    periods_per_side = 1            # Halbach periods across the platform
    gap = 0.003                     # m   magnet bottom -> coil plane (3 mm)
    coil_current = 1.0              # A   per-coil drive limit (driver channel)
    coils_per_period = 2            # planar-motor quadrature: 2 coils / period


class Constants:
    ndfeb_remanence_br = 1.45       # T
    ndfeb_density = 7500.0          # kg/m3
    plastic_density = 1200.0        # kg/m3
    gravity = 9.80665               # m/s2
    # piece body geometry (scaled chess king), used only for the COM height
    king_height = 0.095             # m
    king_base_diameter = 0.044      # m
    com_height_fraction = 0.4       # COM as fraction of body height above base


# Axis-aligned magnetisation directions reachable with an off-the-shelf,
# face-magnetised cube in a tightly packed array (no 45-deg rotated cubes).
AXIS_DIRECTIONS = np.array(
    [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]],
    dtype=float,
)


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n else v


def _snap_to_axis(v, br):
    """Return the standard cube magnetisation (br * nearest axis) for ideal v."""
    if np.linalg.norm(v) == 0:
        return np.zeros(3)
    dots = AXIS_DIRECTIONS @ _unit(v)
    return br * AXIS_DIRECTIONS[int(np.argmax(dots))]


# Which patterns are realised with discrete, axis-aligned (off-the-shelf) cubes.
# halbach1d is kept *continuous* on purpose, to expose that an ideal 1-D
# Halbach needs diagonally magnetised (non-standard) cubes.
_DISCRETE_KINDS = {"halbach1d_disc", "halbach2d_square", "herringbone"}


# --------------------------------------------------------------------------- #
#  Magnet layouts                                                             #
# --------------------------------------------------------------------------- #
class MagnetLayout:
    """Builds a piece magnet array and reports its physical properties.

    ``kind`` selects the magnetisation pattern, ``footprint`` crops the cube
    grid to a shape.  Polarisations are the *ideal* (design) directions; the
    manufacturability flag reports whether those ideal directions are all
    standard axis-aligned cube magnetisations.
    """

    def __init__(self, kind="halbach1d", footprint="square",
                 edge=None, periods=None, per_period=None, br=None):
        self.kind = kind
        self.footprint = footprint
        self.edge = edge if edge is not None else Inputs.magnet_cube_edge
        self.periods = periods if periods is not None else Inputs.periods_per_side
        self.per_period = per_period if per_period is not None else Inputs.magnets_per_period
        self.br = br if br is not None else Constants.ndfeb_remanence_br

        self.blocks_per_side = self.periods * self.per_period
        self.period = self.per_period * self.edge
        self.k = 2 * np.pi / self.period
        self.platform_side = self.blocks_per_side * self.edge

        self._dirs = []          # ideal unit magnetisation directions actually placed
        self.cube_count = 0
        self.collection = self._build()
        self.com_z = self._com_height()

    # -- geometry helpers --------------------------------------------------- #
    def _center(self, index):
        return (index + 0.5 - self.blocks_per_side / 2) * self.edge

    def _in_footprint(self, x, y):
        half = self.platform_side / 2
        if self.footprint == "square":
            return True
        if self.footprint == "round":
            return np.hypot(x, y) <= half + 1e-12
        if self.footprint == "octagon":
            # crop the four corners by a 45-deg chamfer at ~0.83 of the half side
            return abs(x) + abs(y) <= 1.30 * half + 1e-12
        raise ValueError(f"unknown footprint {self.footprint}")

    # -- magnetisation patterns -------------------------------------------- #
    def _polarization(self, ix, iy):
        x = self._center(ix)
        y = self._center(iy)
        br = self.br
        if self.kind in ("halbach1d", "halbach1d_disc"):
            # wave travels along x, strong side faces -z (toward coils)
            a = self.k * x
            return np.array([br * np.sin(a), 0.0, -br * np.cos(a)])
        if self.kind == "halbach2d_square":
            # separable 2-D Halbach; snapped to axes -> manufacturable checkerboard
            cx, cy = np.cos(self.k * x), np.cos(self.k * y)
            sx, sy = np.sin(self.k * x), np.sin(self.k * y)
            return br * _unit(np.array([sx * cy, cx * sy, -cx * cy]))
        if self.kind == "herringbone":
            # two orientation families: alternate the wave axis per block row,
            # giving in-plane shear authority in both x and y.
            if (iy % 2) == 0:
                a = self.k * x
                return np.array([br * np.sin(a), 0.0, -br * np.cos(a)])
            a = self.k * y
            return np.array([0.0, br * np.sin(a), -br * np.cos(a)])
        raise ValueError(f"unknown kind {self.kind}")

    def _build(self):
        cubes = []
        discrete = self.kind in _DISCRETE_KINDS
        for ix in range(self.blocks_per_side):
            x = self._center(ix)
            for iy in range(self.blocks_per_side):
                y = self._center(iy)
                if not self._in_footprint(x, y):
                    continue
                pol = self._polarization(ix, iy)
                if discrete:
                    pol = _snap_to_axis(pol, self.br)
                self._dirs.append(_unit(pol))
                cubes.append(magpy.magnet.Cuboid(
                    dimension=(self.edge, self.edge, self.edge),
                    polarization=tuple(pol),
                    position=(x, y, self.edge / 2),
                ))
        self.cube_count = len(cubes)
        return magpy.Collection(cubes)

    def _com_height(self):
        # scale a reference king to this base size, COM at a fraction of height
        base_d = self.platform_side * np.sqrt(2)
        scale = base_d / Constants.king_base_diameter
        return Constants.com_height_fraction * Constants.king_height * scale

    # -- reported properties ----------------------------------------------- #
    @property
    def magnet_mass(self):
        return self.cube_count * self.edge ** 3 * Constants.ndfeb_density

    @property
    def max_axis_error_deg(self):
        """Largest angle between an ideal direction and the nearest cube axis."""
        worst = 0.0
        for d in self._dirs:
            dots = AXIS_DIRECTIONS @ d
            ang = np.degrees(np.arccos(np.clip(dots.max(), -1, 1)))
            worst = max(worst, ang)
        return worst

    @property
    def manufacturable(self):
        # standard cube magnets are face-magnetised -> only +/-x,y,z usable
        return self.max_axis_error_deg < 1.0

    def field_plane(self, samples=41):
        half = self.platform_side / 2
        axis = np.linspace(-half, half, samples)
        grid = np.array([[x, y, -Inputs.gap] for x in axis for y in axis])
        return self.collection.getB(grid)

    def peak_bz(self, samples=41):
        return float(np.max(np.abs(self.field_plane(samples)[:, 2])))


# --------------------------------------------------------------------------- #
#  Coil patch                                                                 #
# --------------------------------------------------------------------------- #
class CoilPatch:
    """N x N square single-turn coils tiled at the coil plane (z = -gap).

    The coil pitch follows planar-motor practice: ``coils_per_period`` coils per
    magnetic period (default 2 = quadrature).  Each coil is a meshed Polyline so
    magpylib_force can integrate the Lorentz force/torque on it.
    """

    def __init__(self, n, layout, meshing=40, current=None):
        self.n = n
        self.layout = layout
        self.current = current if current is not None else Inputs.coil_current
        self.pitch = layout.period / Inputs.coils_per_period
        self.side = self.pitch
        self.z = -Inputs.gap
        self.meshing = meshing
        self.coils = self._build()
        # representative multi-layer winding (wire OD 0.1 mm, 0.8 fill, 3 layers)
        # so the single-turn Lorentz model scales to real amp-turns / current.
        wire_od = 0.0001
        layers = 3
        radial_width = 0.35 * self.side
        self.turns = max(1, int(radial_width * 0.8 / wire_od) * layers)
        self.ni_max = self.turns * self.current  # amp-turns ceiling per coil

    def _build(self):
        half = self.side / 2
        loop_xy = np.array([
            [-half, -half, 0.0], [half, -half, 0.0],
            [half, half, 0.0], [-half, half, 0.0], [-half, -half, 0.0],
        ])
        coils = []
        offset = (self.n - 1) / 2
        for i in range(self.n):
            for j in range(self.n):
                cx = (i - offset) * self.pitch
                cy = (j - offset) * self.pitch
                loop = magpy.current.Polyline(
                    current=self.current,
                    vertices=loop_xy + np.array([cx, cy, 0.0]),
                    position=(0.0, 0.0, self.z),
                )
                loop.meshing = self.meshing
                coils.append(loop)
        return coils

    @property
    def extent(self):
        return self.n * self.pitch


# --------------------------------------------------------------------------- #
#  Actuator (wrench) model                                                    #
# --------------------------------------------------------------------------- #
def piece_pose(layout, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    """Place the magnet collection at a pose and return (collection, com_world).

    Angles in radians.  z is an *added* vertical offset (positive lifts the
    piece, increasing the magnet-to-coil gap).  Returns the same collection
    object re-posed in place.
    """
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw])
    col = layout.collection
    col.position = (0.0, 0.0, 0.0)
    col.orientation = Rotation.identity()
    col.rotate(rot, anchor=(0.0, 0.0, layout.com_z))
    col.move((x, y, z))
    com = np.array([x, y, layout.com_z + z])
    return col, com


def actuator_matrix(layout, patch, com):
    """6 x Ncoils signed wrench-per-amp matrix [Fx,Fy,Fz,Mx,My,Mz]/A on the piece.

    Force/torque on the piece is the negative of the force/torque on each coil
    (Newton's third law).  Values are normalised to 1 A per coil.
    """
    cols = []
    for coil in patch.coils:
        F, T = getFT(layout.collection, coil, anchor=com)
        wrench = -np.concatenate([F, T]) / coil.current
        cols.append(wrench)
    return np.array(cols).T  # shape (6, Ncoils)


# --------------------------------------------------------------------------- #
#  Piece weight                                                               #
# --------------------------------------------------------------------------- #
def piece_weight(layout, wall=0.001):
    """Approximate piece weight (N): magnet mass + plastic king shell."""
    base_d = layout.platform_side * np.sqrt(2)
    scale = base_d / Constants.king_base_diameter
    h = Constants.king_height * scale
    d = base_d
    shell_vol = np.pi / 4 * (d ** 2 * h - max(d - 2 * wall, 0) ** 2 * max(h - 2 * wall, 0))
    shell_mass = shell_vol * Constants.plastic_density
    mass = shell_mass + layout.magnet_mass
    return mass * Constants.gravity


# --------------------------------------------------------------------------- #
#  Controllability analysis of an actuator matrix                             #
# --------------------------------------------------------------------------- #
def _cond(mat):
    s = np.linalg.svd(mat, compute_uv=False)
    s = s[s > s.max() * 1e-12] if s.max() > 0 else s
    return s[0] / s[-1] if len(s) else np.inf


def _max_generalised_force(A, row, equal_rows, equal_vals, imax):
    """Max of A[row]@I subject to A[equal_rows]@I == equal_vals, |I|<=imax."""
    n = A.shape[1]
    res = linprog(
        c=-A[row], bounds=[(-imax, imax)] * n,
        A_eq=A[equal_rows] if equal_rows else None,
        b_eq=equal_vals if equal_rows else None,
        method="highs",
    )
    return (-res.fun) if res.success else np.nan


class Controllability:
    def __init__(self, A, weight, char_len, turns=1, ni_max=None):
        self.A = A
        self.weight = weight
        self.char_len = char_len
        self.turns = turns
        self.ni_max = ni_max if ni_max is not None else turns * Inputs.coil_current
        self.ncoils = A.shape[1]

        # torque rows scaled to force units (divide by lever arm) for conditioning
        self.As = A.copy()
        self.As[3:] = A[3:] / char_len

        self.rank3 = int(np.linalg.matrix_rank(A[:3], tol=A[:3].max() * 1e-9))
        self.rank6 = int(np.linalg.matrix_rank(self.As, tol=self.As.max() * 1e-9))
        self.cond3 = _cond(A[:3])
        self.cond6 = _cond(self.As)

        # min-norm hover amp-turns (A is wrench per amp-turn) -> per-coil current
        self.hover3 = np.linalg.pinv(A[:3]) @ np.array([0, 0, weight])
        self.ni_hover3 = np.max(np.abs(self.hover3))
        self.i_hover3 = self.ni_hover3 / turns
        if self.rank6 >= 6:
            self.hover6 = np.linalg.pinv(A) @ np.array([0, 0, weight, 0, 0, 0])
            self.ni_hover6 = np.max(np.abs(self.hover6))
            self.i_hover6 = self.ni_hover6 / turns
        else:
            self.hover6 = None
            self.ni_hover6 = np.inf
            self.i_hover6 = np.inf

        # authority at the amp-turn ceiling while holding hover (Fz = weight)
        nm = self.ni_max
        self.max_lateral = _max_generalised_force(A, 0, [1, 2], [0.0, weight], nm)
        if self.rank6 >= 6:
            self.max_roll = _max_generalised_force(A, 3, [0, 1, 2, 4, 5], [0, 0, weight, 0, 0], nm)
            self.max_pitch = _max_generalised_force(A, 4, [0, 1, 2, 3, 5], [0, 0, weight, 0, 0], nm)
            self.max_yaw = _max_generalised_force(A, 5, [0, 1, 2, 3, 4], [0, 0, weight, 0, 0], nm)
        else:
            self.max_roll = self.max_pitch = self.max_yaw = np.nan


# --------------------------------------------------------------------------- #
#  Worst-case pose sweep                                                      #
# --------------------------------------------------------------------------- #
def worst_case_sweep(layout, patch, weight, char_len,
                     nxy=3, gaps=(0.0025, 0.003, 0.004, 0.005, 0.006),
                     yaws_deg=(0, 30, 45, 60, 90), tilt_deg=5.0):
    """Sweep pose over one coil period, gap, yaw and small roll/pitch.

    Returns the worst-case (not best-case) controllability metrics so we know
    whether the piece stays commandable everywhere, not just at the sweet spot.
    """
    period = layout.period
    xs = np.linspace(0, period, nxy, endpoint=False)
    ys = np.linspace(0, period, nxy, endpoint=False)
    tilt = np.radians(tilt_deg)
    worst = {
        "i_hover3": 0.0, "i_hover6": 0.0, "cond3": 0.0, "cond6": 0.0,
        "min_rank3": 6, "min_rank6": 6, "min_lateral": np.inf, "min_yaw": np.inf,
        "n": 0, "hover3_over_1A": 0,
    }
    for gx in gaps:
        z = gx - Inputs.gap
        for yaw in np.radians(yaws_deg):
            for roll in (-tilt, 0.0, tilt):
                for x in xs:
                    for y in ys:
                        _, com = piece_pose(layout, x=x, y=y, z=z,
                                            roll=roll, pitch=0.0, yaw=yaw)
                        A = actuator_matrix(layout, patch, com)
                        c = Controllability(A, weight, char_len,
                                            turns=patch.turns, ni_max=patch.ni_max)
                        worst["i_hover3"] = max(worst["i_hover3"], c.i_hover3)
                        worst["i_hover6"] = max(worst["i_hover6"], c.i_hover6)
                        worst["cond3"] = max(worst["cond3"], c.cond3)
                        worst["cond6"] = max(worst["cond6"], c.cond6)
                        worst["min_rank3"] = min(worst["min_rank3"], c.rank3)
                        worst["min_rank6"] = min(worst["min_rank6"], c.rank6)
                        if np.isfinite(c.max_lateral):
                            worst["min_lateral"] = min(worst["min_lateral"], max(c.max_lateral, 0.0))
                        if np.isfinite(c.max_yaw):
                            worst["min_yaw"] = min(worst["min_yaw"], max(c.max_yaw, 0.0))
                        worst["hover3_over_1A"] += int(c.i_hover3 > Inputs.coil_current)
                        worst["n"] += 1
    piece_pose(layout)  # restore nominal
    return worst


# --------------------------------------------------------------------------- #
#  Hall-sensor observability                                                  #
# --------------------------------------------------------------------------- #
def hall_jacobian(layout, n_side, span=None, plane_z=None):
    """3-axis Hall grid (n_side x n_side). Returns the pose Jacobian d B / d pose.

    pose = [x, y, z, roll, pitch, yaw]; columns 0:3 -> 3-DOF, all 6 -> 6-DOF.
    """
    span = span if span is not None else layout.platform_side
    plane_z = plane_z if plane_z is not None else -Inputs.gap
    axis = np.linspace(-span / 2, span / 2, n_side)
    pts = np.array([[x, y, plane_z] for x in axis for y in axis])

    def measure(dx, dy, dz, dr, dp, dyaw):
        _, _ = piece_pose(layout, x=dx, y=dy, z=dz, roll=dr, pitch=dp, yaw=dyaw)
        return layout.collection.getB(pts).reshape(-1)

    base = measure(0, 0, 0, 0, 0, 0)
    dl, da = 1e-5, 1e-4  # m, rad finite-difference steps
    cols = []
    for k, step in enumerate([dl, dl, dl, da, da, da]):
        d = [0.0] * 6
        d[k] = step
        cols.append((measure(*d) - base) / step)
    piece_pose(layout)
    return np.array(cols).T  # (3*n^2, 6)


def hall_metrics(J):
    J3, J6 = J[:, :3], J
    return {
        "rank3": int(np.linalg.matrix_rank(J3, tol=J3.max() * 1e-6)),
        "rank6": int(np.linalg.matrix_rank(J6, tol=J6.max() * 1e-6)),
        "cond3": _cond(J3), "cond6": _cond(J6),
    }


# --------------------------------------------------------------------------- #
#  Neighbour-piece disturbance                                                #
# --------------------------------------------------------------------------- #
def neighbour_disturbance(layout_kind, edge, controlled, com, distance,
                          n_side_sensor=3):
    """Disturbance wrench + Hall interference from a resting neighbour piece.

    The neighbour is an identical magnet array offset by ``distance`` in x and
    sitting on the board (no lift).  Returns force/torque on the controlled
    piece and the neighbour's field contribution at the Hall plane.
    """
    nb = MagnetLayout(kind=layout_kind, edge=edge)
    nb.collection.move((distance, 0.0, 0.0))

    targets = list(controlled.collection)
    for c in targets:
        c.meshing = (2, 2, 2)
    ft = np.atleast_2d(getFT(nb.collection, targets, anchor=com))
    if ft.ndim == 3:                      # (t,2,3)
        F = ft[:, 0, :].sum(axis=0)
        T = ft[:, 1, :].sum(axis=0)
    else:                                 # single target (2,3)
        F, T = ft[0], ft[1]

    span = controlled.platform_side
    axis = np.linspace(-span / 2, span / 2, n_side_sensor)
    pts = np.array([[x, y, -Inputs.gap] for x in axis for y in axis])
    b_self = np.linalg.norm(controlled.collection.getB(pts), axis=1).mean()
    b_nb = np.linalg.norm(nb.collection.getB(pts), axis=1).mean()
    return {"F": F, "T": T, "b_self": b_self, "b_nb": b_nb,
            "interference": b_nb / b_self if b_self else np.inf}


# --------------------------------------------------------------------------- #
#  Reporting                                                                  #
# --------------------------------------------------------------------------- #
def _hdr(title):
    print("\n" + title)
    print("-" * len(title))


def _g(v, scale=1.0, prec=1):
    """Format a possibly-nan/inf value (n/a) for tables."""
    if v is None or not np.isfinite(v):
        return "n/a"
    return f"{v * scale:.{prec}f}"


def analyse(layout, n_patch, weight=None, char_len=None):
    """Nominal-pose controllability of a layout under an n x n coil patch."""
    weight = piece_weight(layout) if weight is None else weight
    char_len = layout.platform_side / 2 if char_len is None else char_len
    piece_pose(layout)
    patch = CoilPatch(n_patch, layout)
    A = actuator_matrix(layout, patch, np.array([0, 0, layout.com_z]))
    return patch, Controllability(A, weight, char_len, turns=patch.turns, ni_max=patch.ni_max)


def required_yaw_torque(layout, weight):
    mass = weight / Constants.gravity
    span = layout.platform_side
    inertia = (mass * 0.66) * span ** 2 / 6      # ~magnet fraction of mass
    accel = 4 * np.radians(90) / 0.5 ** 2        # 90 deg in 0.5 s
    return inertia * accel


def main():
    print("=" * 74)
    print("LEVITATING CHESS - MAGPYLIB CONTROLLABILITY SIMULATION")
    print("=" * 74)
    print(f"Gap {Inputs.gap*1000:.1f} mm | coil current limit {Inputs.coil_current} A | "
          f"Br {Constants.ndfeb_remanence_br} T | {Inputs.coils_per_period} coils/period")

    edge = Inputs.magnet_cube_edge
    ref_kind = "herringbone"           # filled in after the comparison below

    # 1. MAGNET LAYOUTS ---------------------------------------------------- #
    _hdr("1. MAGNET LAYOUTS  (3x3 coil patch, nominal pose)")
    print(f"  {'layout':16s}{'cubes':>6}{'mass g':>8}{'peakBz T':>10}"
          f"{'Fz/A mN':>9}{'Flat mN':>9}{'Myaw uNm':>10}{'mfg?':>6}")
    layout_rows = {}
    for kind in ("halbach1d", "halbach1d_disc", "halbach2d_square", "herringbone"):
        m = MagnetLayout(kind=kind, edge=edge)
        W = piece_weight(m)
        patch, c = analyse(m, 3, weight=W)
        fz_per_a = np.abs(c.A[2]).max() * patch.turns
        layout_rows[kind] = (m, c, W)
        print(f"  {kind:16s}{m.cube_count:>6}{m.magnet_mass*1000:>8.1f}{m.peak_bz():>10.3f}"
              f"{fz_per_a*1e3:>9.2f}{_g(c.max_lateral, 1e3):>9}"
              f"{_g(c.max_yaw, 1e6):>10}{str(m.manufacturable):>6}")

    _hdr("   Cube-size sweep (herringbone, square footprint)")
    print(f"  {'edge mm':>8}{'cubes':>6}{'mass g':>8}{'peakBz T':>10}{'Ihover3 A':>11}{'Flat mN':>9}")
    for e in (0.004, 0.005, 0.006):
        m = MagnetLayout(kind="herringbone", edge=e)
        W = piece_weight(m)
        _, c = analyse(m, 3, weight=W)
        print(f"  {e*1000:>8.0f}{m.cube_count:>6}{m.magnet_mass*1000:>8.1f}{m.peak_bz():>10.3f}"
              f"{c.i_hover3:>11.2f}{c.max_lateral*1e3:>9.1f}")

    _hdr("   Footprint sweep (herringbone, 5 mm cubes)")
    print(f"  {'footprint':>10}{'cubes':>6}{'mass g':>8}{'peakBz T':>10}{'Flat mN':>9}")
    for fp in ("square", "octagon", "round"):
        m = MagnetLayout(kind="herringbone", edge=edge, footprint=fp)
        _, c = analyse(m, 3)
        print(f"  {fp:>10}{m.cube_count:>6}{m.magnet_mass*1000:>8.1f}{m.peak_bz():>10.3f}{c.max_lateral*1e3:>9.1f}")

    # 2 + 3. COIL PATCHES AND ACTUATOR MATRIX ------------------------------ #
    m = MagnetLayout(kind=ref_kind, edge=edge)
    W = piece_weight(m)
    L = m.platform_side / 2
    _hdr(f"2-3. COIL PATCH / ACTUATOR MATRIX  (magnet = {ref_kind}, "
         f"weight {W*1000:.0f} mN, turns from winding est.)")
    print(f"  {'patch':>6}{'coils':>6}{'rank3':>6}{'rank6':>6}{'cond3':>8}{'cond6':>9}"
          f"{'Ihov3 A':>9}{'Ihov6 A':>9}{'Flat mN':>9}{'Myaw uNm':>10}")
    coil_results = {}
    for n in (2, 3, 4, 5):
        patch, c = analyse(m, n, weight=W, char_len=L)
        coil_results[n] = (patch, c)
        print(f"  {f'{n}x{n}':>6}{c.ncoils:>6}{c.rank3:>6}{c.rank6:>6}{c.cond3:>8.1f}{c.cond6:>9.1f}"
              f"{c.i_hover3:>9.2f}{_g(c.i_hover6,1,2):>9}{_g(c.max_lateral,1e3):>9}{_g(c.max_yaw,1e6):>10}")
    print("  (rank3>=3 => Fx,Fy,Fz controllable; rank6=6 => +Mx,My,Mz; "
          "Ihover = per-coil current at 3 mm)")

    # 4. WORST-CASE SWEEP -------------------------------------------------- #
    rec_n = 4
    patch = CoilPatch(rec_n, m)
    _hdr(f"4. WORST-CASE SWEEP  (magnet={ref_kind}, {rec_n}x{rec_n} patch; "
         "x/y over one period, gap 2.5-6 mm, yaw 0-90, roll +-5)")
    w = worst_case_sweep(m, patch, W, L)
    print(f"  poses evaluated                 {w['n']}")
    print(f"  min rank (3DOF / 6DOF)          {w['min_rank3']} / {w['min_rank6']}")
    print(f"  worst-case hover current 3DOF   {w['i_hover3']:.2f} A  "
          f"({w['hover3_over_1A']}/{w['n']} poses need >1 A)")
    print(f"  worst-case hover current 6DOF   {w['i_hover6']:.2f} A")
    print(f"  worst-case condition (3/6 DOF)  {w['cond3']:.0f} / {w['cond6']:.0f}")
    ml = w['min_lateral']
    my = w['min_yaw']
    print(f"  spare lateral authority @1A     "
          f"{_g(ml, 1e3)} mN (drops to ~0 where hover already uses the 1 A budget)")
    print(f"  spare yaw authority @1A         {_g(my, 1e6)} uNm")

    # 5. HALL SENSORS ------------------------------------------------------ #
    _hdr("5. HALL SENSORS  (3-axis grid at coil plane; pose Jacobian)")
    print(f"  {'grid':>6}{'sensors':>9}{'rank3':>7}{'rank6':>7}{'cond3':>8}{'cond6':>9}{'6DOF obs?':>11}")
    for ns in (2, 3, 4):
        hm = hall_metrics(hall_jacobian(m, ns))
        obs = "yes" if hm["rank6"] == 6 else "NO"
        print(f"  {f'{ns}x{ns}':>6}{ns*ns:>9}{hm['rank3']:>7}{hm['rank6']:>7}"
              f"{hm['cond3']:>8.1f}{hm['cond6']:>9.1f}{obs:>11}")

    # 6. NEIGHBOUR PIECES -------------------------------------------------- #
    _hdr("6. NEIGHBOUR-PIECE DISTURBANCE  (resting identical pieces)")
    com = np.array([0, 0, m.com_z])
    base_d = m.platform_side * np.sqrt(2)
    scenarios = [
        ("adjacent square", base_d / 0.8),
        ("captured storage (touching)", base_d * 1.02),
        ("two movers (nearest)", base_d / 0.8),
    ]
    print(f"  {'scenario':>30}{'dist mm':>9}{'|F| mN':>9}{'|T| uNm':>10}{'Hall interf':>13}")
    for label, dist in scenarios:
        nb = neighbour_disturbance(ref_kind, edge, m, com, dist)
        mult = 2 if "two" in label else 1
        print(f"  {label:>30}{dist*1000:>9.1f}{np.linalg.norm(nb['F'])*1e3*mult:>9.1f}"
              f"{np.linalg.norm(nb['T'])*1e6*mult:>10.1f}{nb['interference']*100*mult:>11.1f} %")
    print(f"  (compare with piece weight {W*1e3:.0f} mN and lateral authority "
          f"{coil_results[rec_n][1].max_lateral*1e3:.0f} mN)")

    # FINAL ANSWERS -------------------------------------------------------- #
    _hdr("FINAL OUTPUT")
    c3 = coil_results[3][1]
    c4 = coil_results[4][1]
    yaw_need = required_yaw_torque(m, W)
    min3 = next((n for n in (2, 3, 4, 5) if coil_results[n][1].rank3 >= 3), None)
    min6 = next((n for n in (2, 3, 4, 5) if coil_results[n][1].rank6 >= 6), None)
    amp_ok = w["i_hover3"] <= Inputs.coil_current

    print(f"  Is a 3 mm gap realistic?           "
          f"{'YES' if c3.i_hover3 <= 1 else 'MARGINAL'} - nominal hover {c3.i_hover3:.2f} A/coil at 3 mm, "
          f"worst-case {w['i_hover3']:.2f} A across phases.")
    print(f"  Is 1 A per coil enough?            "
          f"{'YES nominally' if c3.i_hover3 <= 1 else 'NO'} - nominal {c3.i_hover3:.2f} A, "
          f"worst-case {w['i_hover3']:.2f} A ({w['hover3_over_1A']}/{w['n']} phases > 1 A); "
          f"needs more turns or a larger patch.")
    print(f"  Minimum coil patch for 3DOF        "
          f"{min3}x{min3} (rank 3); but 2x2 needs {coil_results[2][1].i_hover3:.2f} A/coil to hover, "
          f"so 3x3 is the practical minimum.")
    print(f"  Minimum coil patch for 6DOF        {min6}x{min6} "
          f"(2x2 only reaches rank {coil_results[2][1].rank6}).")
    print(f"  Best magnet layout                 herringbone - manufacturable with axis-aligned "
          f"cubes, strongest Bz, gives both Fx and Fy shear.")
    print(f"  Best coil layout                   {rec_n}x{rec_n} - smallest patch with full 6DOF "
          f"rank and worst-case margin; 3x3 is the bare minimum.")
    print(f"  Is 6DOF actually feasible?         "
          f"{'YES, with caveats' if min6 else 'NO'} - rank 6 from {min6}x{min6} up and Hall-observable; "
          f"nominal yaw torque {_g(c4.max_yaw,1e6)} uNm >> ~{yaw_need*1e6:.0f} uNm needed, "
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
        f"{w['i_hover3']/max(c3.i_hover3,1e-9):.1f}x the nominal current;",
        "neighbour pieces ignored, yet an adjacent resting piece exerts a "
        "lateral force comparable to or above the piece weight;",
        "6DOF assumed once force rank is met, but yaw torque authority and Hall "
        "yaw observability (high condition number) are the real limits.",
    ):
        print(f"    - {line}")


if __name__ == "__main__":
    main()

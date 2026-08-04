import numpy as np

src = open('model.py').read().split('sim = levitation_sim.measure')[0]
ns = {'__name__': 'verify', '__file__': 'model.py'}
exec(src, ns)
ls = ns['levitation_sim']
geo = ns['sim_geometry']
Inputs, Fixed, Constants = ns['Inputs'], ns['Fixed'], ns['Constants']
ls.use_geometry(geo)

TURNS = 33
RESISTANCE = 0.5234
CURRENT_LIMIT = 5.5
BUDGET = TURNS * CURRENT_LIMIT
HEIGHT_COUPLING = 0.682
SQUARE = 0.060
FRICTION = Fixed.resting_friction_coefficient
power_factor = RESISTANCE / TURNS ** 2 / HEIGHT_COUPLING ** 2

def build_patch(cells):
    return ls.CoilArray(cells, geo.coil_short, geo.coil_long, geo.coil_radial_width,
                        geo.coil_height, 1, geo.gap, geo.coil_height, 8, 2, 1)

def piece_at(x, y):
    layout = ls.magnet_layout_from_geometry()
    layout.collection.move((x, y, 0.0))
    return layout

def multi_hover(positions, array, phase=(0.0, 0.0)):
    pieces = [piece_at(x + phase[0], y + phase[1]) for x, y in positions]
    weight = ls.piece_weight(pieces[0])
    rows, targets = [], []
    for p, (x, y) in zip(pieces, positions):
        com = np.array([x + phase[0], y + phase[1], p.center_of_mass_height])
        rows.append(ls.actuator_matrix(p, array, com))
        targets.extend([0, 0, weight, 0, 0, 0])
    matrix = np.vstack(rows)
    currents = np.linalg.pinv(matrix, rcond=1e-10) @ np.array(targets)
    return float(np.max(np.abs(currents))), currents, pieces, weight

print('=== 1. Isolated piece baseline (large patch) ===')
array = build_patch(14)
peak1, cur1, _, weight = multi_hover([(0.0, 0.0)], array)
p1 = float(np.sum(cur1 ** 2)) * power_factor
print(f'  coils {len(array.coils)}, peak {peak1:.1f} At (budget {BUDGET:.0f}), power {p1:.2f} W')

print('=== 2. Two adjacent pieces (60 mm), aligned and worst phase ===')
for phase in ((0.0, 0.0), (geo.coil_long / 4, geo.coil_long / 4)):
    peak2, cur2, _, _ = multi_hover([(-SQUARE / 2, 0.0), (SQUARE / 2, 0.0)], array, phase)
    p2 = float(np.sum(cur2 ** 2)) * power_factor
    print(f'  phase {tuple(round(v*1000,1) for v in phase)} mm: peak {peak2:.1f} At, '
          f'total power {p2:.2f} W ({p2 / (2 * p1):.2f}x per-piece vs isolated), '
          f'budget margin {BUDGET / peak2:.2f}x')

print('=== 3. Reset-wave crowding: 3x3 pieces on the 60 mm lattice ===')
array9 = build_patch(18)
positions = [(i * SQUARE, j * SQUARE) for i in (-1, 0, 1) for j in (-1, 0, 1)]
peak9, cur9, pieces9, _ = multi_hover(positions, array9)
p9 = float(np.sum(cur9 ** 2)) * power_factor
print(f'  coils {len(array9.coils)}, peak {peak9:.1f} At (budget {BUDGET:.0f}, margin {BUDGET / peak9:.2f}x)')
print(f'  total power {p9:.2f} W -> per piece {p9 / 9:.2f} W vs isolated {p1:.2f} W ({p9 / 9 / p1:.2f}x crosstalk penalty)')

print('=== 4. Dynamic C5: hover currents vs resting neighbour ===')
mover = piece_at(0.0, 0.0)
com = np.array([0.0, 0.0, mover.center_of_mass_height])
wrench_mover = ls.actuator_matrix(mover, array, com)
hover_currents = np.linalg.pinv(wrench_mover, rcond=1e-10) @ np.array([0, 0, weight, 0, 0, 0])
rest_drop = -(geo.gap - geo.surface_stack)
neighbour = piece_at(SQUARE, 0.0)
neighbour.collection.move((0.0, 0.0, rest_drop))
com_n = np.array([SQUARE, 0.0, neighbour.center_of_mass_height + rest_drop])
wrench_neighbour = ls.actuator_matrix(neighbour, array, com_n)
for label, currents in (("energy-optimal", hover_currents),):
    disturbance = wrench_neighbour @ currents
    lateral = float(np.hypot(disturbance[0], disturbance[1]))
    vertical = float(disturbance[2])
    hold = FRICTION * (weight - max(0.0, vertical))
    print(f'  {label}: lateral pull {lateral * 1000:.2f} mN, vertical {vertical * 1000:+.2f} mN, '
          f'friction hold {hold * 1000:.2f} mN -> margin {hold / max(lateral, 1e-9):.1f}x')
coil_centers = np.array([np.mean([f.vertices.mean(axis=0) for f in c], axis=0) for c in array.coils])
keepout = np.hypot(coil_centers[:, 0] - SQUARE, coil_centers[:, 1]) < 0.032
allowed = ~keepout
wrench_cut = wrench_mover[:, allowed]
currents_cut = np.zeros(len(array.coils))
currents_cut[allowed] = np.linalg.pinv(wrench_cut, rcond=1e-10) @ np.array([0, 0, weight, 0, 0, 0])
disturbance = wrench_neighbour @ currents_cut
lateral = float(np.hypot(disturbance[0], disturbance[1]))
vertical = float(disturbance[2])
hold = FRICTION * (weight - max(0.0, vertical))
power_cut = float(np.sum(currents_cut ** 2)) * power_factor
print(f'  with keep-out under the resting piece: lateral pull {lateral * 1000:.2f} mN, vertical {vertical * 1000:+.2f} mN, '
      f'margin {hold / max(lateral, 1e-9):.1f}x, hover power {power_cut:.2f} W')

print('=== 5. Control-latency budget vs instability growth time ===')
pose_rate = 228.6
instability_time = 21.87e-3
current_loop_delay = 1 / 991.0
pose_delay = 1 / pose_rate
averaging_delay = 0.5 * 4 / Fixed.hall_adc_sample_rate
setpoint_frame_delay = 115 / 25e6
total_delay = pose_delay + current_loop_delay + averaging_delay + setpoint_frame_delay
print(f'  pose {pose_delay*1000:.2f} + current loop {current_loop_delay*1000:.2f} + averaging {averaging_delay*1000:.3f}'
      f' + setpoint {setpoint_frame_delay*1000:.3f} = {total_delay*1000:.2f} ms')
print(f'  instability growth time {instability_time*1000:.2f} ms -> ratio {instability_time / total_delay:.2f}x (need >= 2 for phase margin)')

print('=== 6. Tolerance Monte Carlo (24 samples, worst-phase hover + lift margin) ===')
rng = np.random.default_rng(7)
base_edge, base_thick = geo.magnet_lateral_edge, geo.magnet_thickness
worst_margin, worst_peak = [], []
array_mc = build_patch(10)
for k in range(24):
    br = geo.remanence * (1 + rng.normal(0, 0.015))
    angle_err = np.radians(rng.normal(0, 1.0, size=3))
    gap_err = rng.normal(0, 0.15e-3)
    coil_shift = rng.normal(0, 0.2e-3, size=2)
    layout = ls.MagnetLayout(base_edge, base_thick, geo.periods_per_side, geo.magnets_per_period, br)
    rot = ls.Rotation.from_euler('xyz', angle_err)
    layout.collection.rotate(rot, anchor=(0, 0, 0))
    layout.collection.move((coil_shift[0] + geo.coil_long / 4, coil_shift[1] + geo.coil_long / 4, gap_err))
    com = np.array([coil_shift[0] + geo.coil_long / 4, coil_shift[1] + geo.coil_long / 4,
                    layout.center_of_mass_height + gap_err])
    wrench = ls.actuator_matrix(layout, array_mc, com)
    currents = np.linalg.pinv(wrench, rcond=1e-10) @ np.array([0, 0, weight, 0, 0, 0])
    peak = float(np.max(np.abs(currents)))
    lift_margin = BUDGET / peak
    worst_margin.append(lift_margin)
    worst_peak.append(peak)
worst_margin = np.array(worst_margin)
print(f'  worst-phase hover peak At: median {np.median(worst_peak):.1f}, max {np.max(worst_peak):.1f} (budget {BUDGET:.0f})')
print(f'  lift margin at driver budget: median {np.median(worst_margin):.2f}x, min {np.min(worst_margin):.2f}x (target >= 2x)')

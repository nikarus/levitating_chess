import numpy as np
import magpylib as magpy
from magpylib_force import getFT

src = open('model.py').read().split('sim = levitation_sim.measure')[0]
ns = {'__name__': 'audit', '__file__': 'model.py'}
exec(src, ns)
ls = ns['levitation_sim']
geo = ns['sim_geometry']
Inputs, Fixed, Constants = ns['Inputs'], ns['Fixed'], ns['Constants']
board, coil = ns['board'], ns['coil']
ls.use_geometry(geo)

print('=== 1. Halbach pattern (bottom row of blocks, x-family) ===')
magnet = ls.magnet_layout_from_geometry()
names = {(1,0,0):'+x',(-1,0,0):'-x',(0,1,0):'+y',(0,-1,0):'-y',(0,0,1):'+z',(0,0,-1):'-z'}
for r in range(magnet.blocks_per_side):
    row = []
    for c in range(magnet.blocks_per_side):
        p = ls.snap_to_axis(magnet.block_polarization(c, r), 1.0)
        row.append(names[tuple(int(round(v)) for v in p)])
    print('  row', r, row)
net = sum((b.polarization for b in magnet.collection), np.zeros(3))
print('  net polarization sum (should ~0 for closed flux):', np.round(net, 6))

print('=== 2. Strong side + analytic field ===')
half = magnet.platform_side / 2
axis = np.linspace(-half, half, 41)
below = magnet.collection.getB([[x, y, -geo.gap] for x in axis for y in axis])
above = magnet.collection.getB([[x, y, magnet.thickness + geo.gap] for x in axis for y in axis])
pk_below = np.max(np.abs(below[:, 2])); pk_above = np.max(np.abs(above[:, 2]))
print(f'  peak |Bz| below {pk_below:.4f} T   above {pk_above:.4f} T   ratio {pk_below/pk_above:.2f} (must be >1)')
k = magnet.wavenumber
M = magnet.per_period
analytic = geo.remanence * (1 - np.exp(-k * magnet.thickness)) * np.exp(-k * geo.gap) * np.sin(np.pi / M) / (np.pi / M)
print(f'  infinite-array analytic peak Bz {analytic:.4f} T   sim/analytic {pk_below/analytic:.2f} (finite 1-period array: expect 0.4-0.9)')
z2 = geo.gap + 1 / k
below2 = magnet.collection.getB([[x, y, -z2] for x in axis for y in axis])
decay = np.max(np.abs(below2[:, 2])) / pk_below
print(f'  decay over 1/k: sim {decay:.3f}  vs e^-1 = {np.exp(-1):.3f}')

print('=== 3. Br proportionality ===')
m2 = ls.MagnetLayout(magnet.lateral_edge, magnet.thickness, magnet.periods, magnet.per_period, geo.remanence * 2)
print(f'  peak Bz ratio at 2x Br: {m2.peak_bz() / magnet.peak_bz():.4f} (must be 2.0000)')

print('=== 4. Piece weight by hand ===')
w = ls.piece_weight(magnet)
mm = magnet.magnet_mass
base_d = magnet.platform_side * np.sqrt(2) + 2 * geo.base_corner_standoff
print(f'  magnet mass {mm*1000:.1f} g ({magnet.block_count} blocks x {magnet.lateral_edge*1000:.0f}mm cubes x 7.5 g/cc)')
print(f'  total weight {w:.4f} N = {w/geo.gravity*1000:.1f} g,  base diameter {base_d*1000:.2f} mm')

print('=== 5. Wrench matrix + force reciprocity (two independent algorithms) ===')
array = ls.coil_array_from_geometry(geo.control_cells_per_side, geo.coil_height, 1, 16, 2, 1)
ls.place_piece(magnet)
com = np.array([0.0, 0.0, magnet.center_of_mass_height])
wrench = ls.actuator_matrix(magnet, array, com)
lifts = wrench[2, :]
best = int(np.argmax(np.abs(lifts)))
print(f'  coils {len(array.coils)}, best-coupled coil #{best}, lift/At = {lifts[best]:.4e} N/At')
for child in magnet.collection.children:
    child.meshing = (3, 3, 3)
ft_on_magnet = np.asarray(getFT(array.coils[best], list(magnet.collection.children), anchor=com))
f_direct = ft_on_magnet[:, 0, :].sum(axis=0)
t_direct = ft_on_magnet[:, 1, :].sum(axis=0)
f_recip = wrench[:3, best]
t_recip = wrench[3:, best]
print(f'  force on magnet, direct (magnet-as-target): {np.round(f_direct, 6)}')
print(f'  force on magnet, reciprocal (-force on coil): {np.round(f_recip, 6)}')
print(f'  force mismatch: {np.linalg.norm(f_direct - f_recip) / np.linalg.norm(f_recip) * 100:.2f} %')
print(f'  torque mismatch: {np.linalg.norm(t_direct - t_recip) / max(np.linalg.norm(t_recip), 1e-12) * 100:.2f} %')

print('=== 6. Hover solve closes the force balance ===')
target = np.array([0, 0, w, 0, 0, 0])
peak_at, currents = ls.min_peak_current(wrench, target)
residual = wrench @ currents - target
print(f'  peak ampere-turns {peak_at:.2f} At, sum currents {currents.sum():.2e}')
print(f'  wrench residual |F| {np.linalg.norm(residual[:3]):.2e} N (weight {w:.3f} N), |T| {np.linalg.norm(residual[3:]):.2e} Nm')
sumsq = float(np.sum(currents ** 2))
print(f'  hover sum(At^2) {sumsq:.2f}  -> power at 0.746 ohm / 47 turns: {0.746 * sumsq / 47**2:.2f} W (model best-phase ~28.3 W)')

print('=== 7. Coil resistance by hand ===')
rho = Constants.copper_resistivity
mean_perim = 2 * ((coil.outer_length - coil.winding_radial_width) + (coil.outer_width - coil.winding_radial_width)) / 1000
copper_area = (coil.conductor_radial_width - 2 * Fixed.rectangular_wire_film) * (0.05 - 0) * 1e-6
length = 47 * mean_perim
print(f'  mean turn perimeter {mean_perim*1000:.1f} mm, 47 turns -> {length:.2f} m wire')
print(f'  R = rho*L/A = {rho * length / copper_area:.3f} ohm (model 0.746)')

print('=== 8. Thermal RC spot checks ===')
r_area = (2 * 1.0 / 1000 / 2 / Fixed.coil_bed_through_conductivity
          + Fixed.potting_thickness / 1000 / Fixed.potting_thermal_conductivity
          + Fixed.pcb_thickness / 1000 / Fixed.pcb_via_effective_thermal_conductivity
          + Fixed.radiator_standoff_below_pcb / 1000 / Fixed.thermal_pad_conductivity
          + Fixed.baseplate_thickness / 1000 / Fixed.aluminium_thermal_conductivity)
print(f'  stack area resistance {r_area*1e4:.2f} K.cm2/W (hand) ; per piece footprint '
      f'{r_area / (np.pi/4*(base_d)**2):.2f} K/W')
cap_area = 2 * 1.0 / 1000 * Fixed.potting_volumetric_heat_capacity
print(f'  source tau = R*C = {r_area * cap_area:.1f} s (model ~36.5 s)')
rc = ns['RadiatorCooling']
seg = [(10.0, 100.0)]
one = rc.piecewise_peak_rise(rc, seg, 0.5, 1e9)
print(f'  RC adiabatic check: 100 W x 10 s into C=R*tau: rise {one:.4f} K vs P*t/C {100*10/(0.5*1e9):.4f} K')
inf = rc.piecewise_peak_rise(rc, [(1e9, 100.0)], 0.5, 36.0)
print(f'  RC steady check: 100 W forever, R=0.5: rise {inf:.2f} K vs P*R = 50 K')

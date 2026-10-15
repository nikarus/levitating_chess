import build123d as cad
import numpy as np

from model import Inputs, Fixed, Constants
from methodology.cad import assembly


def king_envelope(piece):
    profile = [(radius * piece.diameter, height * piece.box_height) for height, radius in Fixed.cad_king_profile]
    section = cad.Plane.XZ * cad.Polygon((0, 0), *profile, (0, profile[-1][1]), align=None)
    body = cad.revolve(section, axis=cad.Axis.Z)
    crown_bottom = profile[-1][1]
    crown_height = piece.box_height - crown_bottom
    thickness = piece.diameter * Fixed.cad_king_cross_thickness_fraction
    body += assembly.block(thickness, thickness, crown_height, 0, 0, crown_bottom)
    body += assembly.block(piece.diameter * Fixed.cad_king_cross_width_fraction, thickness, thickness, 0, 0,
                           crown_bottom + crown_height * Fixed.cad_king_crossbar_height_fraction - thickness / 2)
    return body



def construction_parts(piece, magnets):
    envelope = king_envelope(piece)
    inner = envelope.solids()[0].offset_3d([], -Inputs.plastic_wall_thickness, tolerance=Fixed.cad_geometry_tolerance)
    roof_bottom = Inputs.magnet_thickness + Fixed.king_magnet_bond_thickness
    roof_top = roof_bottom + Inputs.plastic_wall_thickness
    cavity = inner & assembly.block(piece.diameter, piece.diameter, piece.box_height, 0, 0, roof_top)
    magnet_side = Inputs.magnet_lateral_edge * Inputs.magnets_per_period * Inputs.periods_per_side
    pocket_side = magnet_side + 2 * Fixed.king_magnet_pocket_clearance
    pocket = assembly.block(pocket_side, pocket_side, roof_bottom, 0, 0, 0)
    base_relief = cad.Cylinder(piece.diameter / 2 - Inputs.plastic_wall_thickness, roof_bottom,
                              align=(cad.Align.CENTER, cad.Align.CENTER, cad.Align.MIN))
    pocket_wall_side = pocket_side + 2 * Inputs.plastic_wall_thickness
    base_relief -= assembly.block(pocket_wall_side, pocket_wall_side, roof_bottom, 0, 0, 0)
    shell = envelope - cavity - pocket - base_relief
    adhesive = pocket - assembly.block(magnet_side, magnet_side, Inputs.magnet_thickness, 0, 0, 0)
    skin = cad.Pos(0, 0, -Fixed.piece_bottom_skin) * cad.Cylinder(piece.diameter / 2, Fixed.piece_bottom_skin,
                    align=(cad.Align.CENTER, cad.Align.CENTER, cad.Align.MIN))
    parts = [assembly.labeled_part(shell, 'King_hollow_shell_with_magnet_pocket', 'lightsteelblue', 'King_shell', 6),
             assembly.labeled_part(adhesive, 'Magnet_bond_UNQUALIFIED_material', 'goldenrod', 'Magnet_bond', 6, True),
             assembly.labeled_part(skin, 'Piece_bottom_skin', 'ivory', 'Piece_skin', 6, True)]
    densities = {'King_shell': Constants.plastic_density, 'Magnet_bond': Fixed.king_bond_density,
                 'Piece_skin': Constants.plastic_density, 'Magnets': Constants.ndfeb_density}
    components = []
    for part in parts + magnets:
        density = densities[part.group]
        components.append(dict(label=part.shape.label, mass_g=part.shape.volume * density,
                               center_mm=list(part.shape.center()),
                               inertia_g_mm2=(np.asarray(part.shape.matrix_of_inertia) * density).tolist()))
    mass = sum(component['mass_g'] for component in components)
    center = sum(component['mass_g'] * np.asarray(component['center_mm']) for component in components) / mass
    inertia = np.zeros((3, 3))
    for component in components:
        offset = np.asarray(component['center_mm']) - center
        inertia += np.asarray(component['inertia_g_mm2']) + component['mass_g'] * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))
    report = dict(mass_g=mass, center_mm=center.tolist(), inertia_kg_m2=(inertia / 1000000000).tolist(),
                  bottom_film_radius_m=piece.diameter / 2000,
                  components=components, reference_mass_g=piece.mass, reference_com_height_mm=piece.com_height * 1000,
                  reference_tilt_inertia_kg_m2=piece.tilt_inertia, reference_yaw_inertia_kg_m2=piece.yaw_inertia,
                  retention='Bonded roof and perimeter pocket; bottom film is not a structural fastener. Adhesive strength, peel, temperature and shock are unqualified.',
                  status='CAD-derived candidate properties; existing motion-screen mass/inertia are unchanged pending requalification')
    return parts, report

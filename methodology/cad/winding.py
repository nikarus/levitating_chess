from math import ceil, sqrt

import build123d as cad
import numpy as np
from scipy.spatial import cKDTree

from model import Inputs, Fixed
from methodology import design, prototype_coils
from methodology.cad import assembly


def quarter_arc(start, corner, end):
    start, corner, end = (np.asarray(tuple(point), dtype=float) for point in (start, corner, end))
    center = start + end - corner
    middle = center + (corner - center) / sqrt(2)
    return cad.ThreePointArc(start, middle, end)


def rounded_turn(half_width, half_length, radius, z, seam):
    x, y, r = half_width, half_length, radius
    edges = [cad.Line((seam, -y, z), (x - r, -y, z))]
    corners = [((x-r,-y,z),(x,-y,z),(x,-y+r,z)), ((x,y-r,z),(x,y,z),(x-r,y,z)),
               ((-x+r,y,z),(-x,y,z),(-x,y-r,z)), ((-x,-y+r,z),(-x,-y,z),(-x+r,-y,z))]
    for index, (start, corner, end) in enumerate(corners):
        if index:
            edges.append(cad.Line(edges[-1] @ 1, start))
        edges.append(quarter_arc(start, corner, end))
    edges.append(cad.Line(edges[-1] @ 1, (-seam, -y, z)))
    return edges


def continuous_path():
    _, coil, winding = prototype_coils.prototype_geometry(Inputs.cad_coil_layout)
    seam = Fixed.prototype_lead_separation / 2 + Fixed.winding_lead_bend_radius
    straight_half = coil.outer_width / 2 - coil.corner_radius
    if seam >= straight_half:
        raise ValueError('Winding transition does not fit the straight side')
    turns = []
    for layer in range(winding.layers):
        radial_order = range(winding.turns_per_layer - 1, -1, -1) if layer % 2 == 0 else range(winding.turns_per_layer)
        for radial in radial_order:
            inset = Fixed.coil_winding_clearance / 2 + (radial + 0.5) * winding.wire.radial_pitch
            turns.append(rounded_turn(coil.outer_width / 2 - inset, coil.outer_length / 2 - inset,
                                      coil.corner_radius - inset, (layer + 0.5) * winding.wire.axial_pitch, seam))
    edges = []
    for turn_index, turn in enumerate(turns):
        if edges:
            start, end = edges[-1] @ 1, turn[0] @ 0
            tangent = cad.Vector(2 * seam / 3, 0, 0)
            if abs(end.Z - start.Z) > Fixed.cad_geometry_tolerance:
                direction = -1 if (turn_index // winding.turns_per_layer) % 2 else 1
                bulge = direction * Fixed.winding_layer_transition_bulge * 8 / 5
                controls = [cad.Vector(start.X + (end.X - start.X) * index / 5,
                            start.Y + (bulge if index in (2, 3) else 0),
                            start.Z if index < 3 else end.Z) for index in range(6)]
                edges.append(cad.Bezier(*controls))
            else:
                edges.append(cad.Bezier(start, start + tangent, end - tangent, end))
        edges.extend(turn)
    radius = Fixed.winding_lead_bend_radius
    start, end = edges[0] @ 0, edges[-1] @ 1
    terminal_y = Fixed.prototype_lead_separation
    lead_x = Fixed.prototype_lead_separation / 2
    start_corner = (lead_x, start.Y, start.Z)
    entry = (lead_x, start.Y + radius, start.Z)
    lower = [cad.Line((lead_x, terminal_y, start.Z), entry), quarter_arc(entry, start_corner, start)]
    upper = [quarter_arc(end, (-lead_x, end.Y, end.Z), (-lead_x, end.Y + radius, end.Z)),
             cad.Line((-lead_x, end.Y + radius, end.Z), (-lead_x, -radius, end.Z)),
             quarter_arc((-lead_x, -radius, end.Z), (-lead_x, 0, end.Z), (-lead_x, 0, end.Z - radius)),
             cad.Line((-lead_x, 0, end.Z - radius), (-lead_x, 0, start.Z + radius)),
             quarter_arc((-lead_x, 0, start.Z + radius), (-lead_x, 0, start.Z), (-lead_x, radius, start.Z)),
             cad.Line((-lead_x, radius, start.Z), (-lead_x, terminal_y, start.Z))]
    orientation = cad.Rot(0, 0, 180)
    return orientation * cad.Wire(lower + edges + upper), [[orientation * edge for edge in turn] for turn in turns], winding


def path_clearance(path):
    points, distances, edge_indices = [], [], []
    total = 0
    edges = path.edges().sort_by(path)
    for index, edge in enumerate(edges):
        count = ceil(edge.length / Fixed.winding_path_sample_pitch)
        for fraction in np.arange(count) / count:
            points.append(tuple(edge.position_at(fraction)))
            distances.append(total + fraction * edge.length)
            edge_indices.append(index)
        total += edge.length
    points, distances = np.array(points), np.array(distances)
    pairs = cKDTree(points).query_pairs(Fixed.prototype_wire_outer_diameter + Fixed.winding_path_sample_pitch, output_type='ndarray')
    pairs = pairs[np.abs(distances[pairs[:, 0]] - distances[pairs[:, 1]]) > 2 * Fixed.prototype_wire_outer_diameter]
    exact_pairs = set(tuple(sorted((edge_indices[a], edge_indices[b]))) for a, b in pairs)
    distances_by_pair = [(edges[a].distance_to(edges[b]), a, b) for a,b in exact_pairs if a != b]
    minimum, first, second = min(distances_by_pair)
    return dict(minimum_nonlocal_centerline_distance_mm=minimum,
                minimum_insulated_clearance_mm=minimum - Fixed.prototype_wire_outer_diameter,
                closest_edges=[first,second], screened_points=len(points))


def conductor_solid(path, diameter):
    section = cad.Plane(origin=path @ 0, z_dir=path % 0) * cad.Circle(diameter / 2)
    return cad.sweep(section, path=path, is_frenet=False)


def detailed_winding():
    path, turns, winding = continuous_path()
    clearance = path_clearance(path)
    if clearance['minimum_insulated_clearance_mm'] < -Fixed.cad_geometry_tolerance:
        raise ValueError(f'Continuous winding insulation overlaps: {clearance}')
    copper = conductor_solid(path, Inputs.prototype_wire_diameter)
    stripped_fraction = Fixed.winding_terminal_pad_size[1] / path.length
    insulated = conductor_solid(path.trim(stripped_fraction, 1 - stripped_fraction), Fixed.prototype_wire_outer_diameter)
    coating = insulated - copper
    _, coil, _ = prototype_coils.prototype_geometry(Inputs.cad_coil_layout)
    detailed = design.WindingGeometry(coil, Inputs.prototype_wire_diameter, winding.layers, wire=winding.wire,
                 radial_turns=winding.turns_per_layer, path_length_mm=path.length,
                 resistance_per_m=max(Fixed.prototype_wire_resistance_per_m))
    pad_width, pad_length, pad_thickness = Fixed.winding_terminal_pad_size
    parts = [assembly.labeled_part(copper, f'Continuous_copper_{winding.turns}_turns', 'sienna', 'Detailed_winding', 3),
             assembly.labeled_part(coating, 'Wire_enamel_except_stripped_ends', 'goldenrod', 'Detailed_insulation', 3, True)]
    pads = []
    for index, fraction in enumerate((pad_length / (2 * path.length), 1 - pad_length / (2 * path.length))):
        center = path.position_at(fraction)
        pad = assembly.block(pad_width, pad_length, pad_thickness, center.X, center.Y, -pad_thickness)
        solder = assembly.block(pad_width, pad_length, winding.wire.axial_pitch, center.X, center.Y, 0) - copper
        pads.append(pad)
        parts.extend([assembly.labeled_part(pad, f'Coil_terminal_pad_{index}', 'darkgoldenrod', 'Coil_terminals', 3),
                      assembly.labeled_part(solder, f'Coil_solder_allowance_{index}_NOT_final_fillet', 'silver', 'Coil_terminals', 3)])
    expected_volume = detailed.length_per_winding * detailed.cross_section_area * 1000000000
    volume_error_per_area = abs(copper.volume - expected_volume) / copper.area
    if not copper.is_valid or not coating.is_valid or len(copper.solids()) != 1 or volume_error_per_area > Fixed.cad_geometry_tolerance:
        raise ValueError('Swept copper or enamel failed path-volume validation')
    report = dict(turns=winding.turns, path_length_mm=path.length,
                  resistance_20C_range_ohm=(np.array(Fixed.prototype_wire_resistance_per_m) * detailed.length_per_winding).tolist(),
                  reference_length_mm=winding.length_per_winding * 1000, reference_resistance_ohm=winding.resistance,
                  copper_volume_mm3=copper.volume, volume_error_per_area_mm=volume_error_per_area,
                  clearance=clearance, terminals_mm=[list(path @ 0), list(path @ 1)],
                  scope='Continuous coil and integral leads to pads; PCB routing beyond pads excluded. Nominal geometry, not tolerance or electrical qualification.')
    return parts, pads, path, report

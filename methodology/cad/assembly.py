import argparse
from copy import copy
from dataclasses import dataclass
import json
from pathlib import Path

import build123d as cad
import numpy as np

import levitation_sim
import model
from model import Inputs, Fixed, Constants
from methodology import design, source_fingerprints


@dataclass
class AssemblyPart:
    shape: cad.Shape
    group: str
    exploded_level: int
    overlay: bool


def block(width, length, height, center_x, center_y, bottom):
    return cad.Pos(center_x, center_y, bottom) * cad.Box(width, length, height, align=(cad.Align.CENTER, cad.Align.CENTER, cad.Align.MIN))


def rectangular_ring(width, length, wall, height):
    return block(width, length, height, 0, 0, 0) - block(width - 2 * wall, length - 2 * wall, height, 0, 0, 0)


def labeled_part(shape, label, color, group, exploded_level, overlay=False):
    shape.label = label
    shape.color = cad.Color(color)
    return AssemblyPart(shape, group, exploded_level, overlay)


def reference_geometry():
    board = design.BoardGeometry()
    coil = design.CoilBed(board)
    winding = design.WindingGeometry(coil, Inputs.selected_conductor_thickness, Inputs.selected_winding_layers)
    piece = design.Piece(board, design.HalbachArray(board))
    geometry = levitation_sim.SimGeometry(Inputs, Fixed, Constants, board, coil, piece)
    levitation_sim.use_geometry(geometry)
    array = levitation_sim.verification_coil_array(coil.control_cells_per_side, winding.coil_height / 1000)
    magnets = levitation_sim.magnet_layout_from_geometry()
    return board, coil, winding, piece, array, magnets


def coil_placements(array):
    placements = []
    for identity, filaments in zip(array.identities, array.coils):
        vertices = np.concatenate([filament.vertices for filament in filaments]) * 1000
        center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
        placements.append((identity, float(center[0]), float(center[1]), float(center[2] - array.height * 1000 / 2)))
    return placements


def prototype_parts():
    board, coil, winding, piece, array, magnets = reference_geometry()
    parts = []
    for (orientation, column, row), x, y, bottom in coil_placements(array):
        width, length = (coil.outer_width, coil.outer_length) if orientation == 'x' else (coil.outer_length, coil.outer_width)
        envelope = cad.Pos(x, y, bottom) * rectangular_ring(width, length, winding.radial_width, winding.coil_height)
        parts.append(labeled_part(envelope, f'Coil_{orientation}_{column}_{row}',
                                  'sienna' if orientation == 'x' else 'goldenrod',
                                  'Upper_coils' if orientation == 'x' else 'Lower_coils',
                                  4 if orientation == 'x' else 3))
    coil_bounds = cad.Compound(children=[copy(part.shape) for part in parts]).bounding_box()
    width, length = coil_bounds.size.X, coil_bounds.size.Y
    bed_bottom, bed_top = coil_bounds.min.Z, coil_bounds.max.Z
    bed = block(width, length, bed_top - bed_bottom, 0, 0, bed_bottom)
    potting = bed.cut(*(part.shape for part in parts))
    parts.append(labeled_part(potting, 'Potting_between_winding_envelopes', 'lightgray', 'Potting', 3, True))
    bottom = bed_bottom
    for label, height, color, group, level in (
        ('Potting_bond', Fixed.potting_thickness, 'gainsboro', 'Potting_bond', 3),
        ('PCB_envelope', Fixed.pcb_thickness, 'seagreen', 'PCB', 2),
        ('Thermal_interface', Fixed.radiator_standoff_below_pcb, 'slategray', 'Thermal_interface', 1),
        ('Baseplate_envelope', Fixed.baseplate_thickness, 'silver', 'Baseplate', 0),
    ):
        bottom -= height
        parts.append(labeled_part(block(width, length, height, 0, 0, bottom), label, color, group, level))
    surface_bottom = bed_top
    for label, height, color, level in (
        ('Potting_cover', Fixed.potting_cover_thickness, 'lightgray', 5),
        ('Playing_surface', Fixed.playing_surface_thickness, 'ivory', 6),
    ):
        parts.append(labeled_part(block(width, length, height, 0, 0, surface_bottom), label, color, label, level, True))
        surface_bottom += height
    polarization_colors = {(1, 0, 0): 'firebrick', (-1, 0, 0): 'salmon', (0, 1, 0): 'royalblue',
                           (0, -1, 0): 'skyblue', (0, 0, 1): 'purple', (0, 0, -1): 'mediumpurple'}
    for index, magnet in enumerate(magnets.collection.children):
        x, y, z = magnet.position * 1000
        dx, dy, dz = magnet.dimension * 1000
        direction = tuple(np.rint(magnet.polarization / np.linalg.norm(magnet.polarization)).astype(int))
        axis = ('X', 'Y', 'Z')[int(np.argmax(np.abs(direction)))]
        sign = 'plus' if max(direction) > 0 else 'minus'
        label = f'Magnet_{index}_{sign}_{axis}'
        parts.append(labeled_part(block(dx, dy, dz, x, y, z - dz / 2), label, polarization_colors[direction], 'Magnets', 7))
    skin = cad.Pos(0, 0, -Fixed.piece_bottom_skin) * cad.Cylinder(piece.diameter / 2, Fixed.piece_bottom_skin,
                                                                              align=(cad.Align.CENTER, cad.Align.CENTER, cad.Align.MIN))
    parts.append(labeled_part(skin, 'Piece_bottom_skin', 'ivory', 'Piece_reference', 7, True))
    king = cad.Cylinder(piece.diameter / 2, piece.box_height, align=(cad.Align.CENTER, cad.Align.CENTER, cad.Align.MIN))
    parts.append(labeled_part(king, 'King_clearance_envelope_NOT_material', 'lightsteelblue', 'Piece_reference', 7, True))
    return parts, board, coil, winding, piece


def winding_detail(coil, winding):
    parts = []
    for layer in range(winding.layers):
        for radial_layer in range(winding.turns_per_layer):
            pitch = winding.wire.radial_pitch
            outer_inset = Fixed.coil_winding_clearance / 2 + radial_layer * pitch
            outer_width = coil.outer_width - 2 * outer_inset
            outer_length = coil.outer_length - 2 * outer_inset
            insulated_turn = rectangular_ring(outer_width, outer_length, pitch, winding.wire.axial_pitch)
            conductor_turn = cad.Pos(0, 0, Fixed.rectangular_wire_film) * rectangular_ring(
                outer_width - 2 * Fixed.rectangular_wire_film,
                outer_length - 2 * Fixed.rectangular_wire_film,
                coil.conductor_radial_width, Inputs.selected_conductor_thickness)
            location = cad.Pos(0, 0, layer * winding.wire.axial_pitch)
            parts.append(labeled_part(location * conductor_turn, f'Copper_turn_{layer}_{radial_layer}', 'sienna', 'Copper', 0))
            parts.append(labeled_part(location * (insulated_turn - conductor_turn), f'Insulation_turn_{layer}_{radial_layer}',
                                      'lightgray', 'Insulation', 0, True))
    return parts


def assembly_tree(parts, exploded):
    groups = {}
    for part in parts:
        shape = copy(part.shape)
        if exploded:
            shape = shape.moved(cad.Location((0, 0, part.exploded_level * Fixed.cad_exploded_layer_spacing)))
        groups.setdefault(part.group, []).append(shape)
    return cad.Compound(label='Flying_chess_exploded_DISPLAY_ONLY' if exploded else 'Flying_chess_geometry_reference',
                        children=[cad.Compound(label=name, children=shapes) for name, shapes in groups.items()])


def part_record(part):
    bounds = part.shape.bounding_box()
    return dict(label=part.shape.label, group=part.group, overlay=part.overlay, volume_mm3=part.shape.volume,
                minimum_mm=list(bounds.min), maximum_mm=list(bounds.max), color=list(part.shape.color)[:3])


def validate_parts(parts):
    for part in parts:
        if not part.shape.is_valid or part.shape.volume <= 0:
            raise ValueError(f'Invalid CAD solid: {part.shape.label}')
    physical = [part for part in parts if part.shape.label != 'King_clearance_envelope_NOT_material']
    largest_overlap = 0
    pairs_checked = 0
    for index, first in enumerate(physical):
        a = first.shape.bounding_box()
        for second in physical[index + 1:]:
            b = second.shape.bounding_box()
            overlap = np.minimum(list(a.max), list(b.max)) - np.maximum(list(a.min), list(b.min))
            if np.any(overlap <= Fixed.cad_geometry_tolerance):
                continue
            pairs_checked += 1
            common = first.shape.intersect(second.shape)
            volume = 0 if common is None else sum(shape.volume for shape in common)
            largest_overlap = max(largest_overlap, volume)
            if volume > Fixed.cad_geometry_tolerance ** 3:
                raise ValueError(f'Overlapping CAD material regions: {first.shape.label}, {second.shape.label}: {volume} mm3')
    return dict(valid_solids=True, tested_volume_intersections=pairs_checked, maximum_overlap_mm3=largest_overlap,
                excluded_reference_envelopes=['King_clearance_envelope_NOT_material'])


def export_checked(parts, path, exploded=False):
    assembly = assembly_tree(parts, exploded)
    if not cad.export_step(assembly, path):
        raise RuntimeError(f'STEP export failed: {path}')
    restored = cad.import_step(path)
    expected, actual = assembly.bounding_box(), restored.bounding_box()
    error = max(abs(a - b) for a, b in zip((*expected.min, *expected.max), (*actual.min, *actual.max)))
    volume_error = abs(sum(shape.volume for shape in assembly.solids()) - sum(shape.volume for shape in restored.solids()))
    if not restored.is_valid or error > Fixed.cad_geometry_tolerance or len(assembly.solids()) != len(restored.solids()):
        raise ValueError(f'STEP round-trip geometry changed: {path}')
    if volume_error > Fixed.cad_geometry_tolerance ** 3 * len(assembly.solids()):
        raise ValueError(f'STEP round-trip volume changed: {path}: {volume_error}')
    return dict(file=path.name, solids=len(restored.solids()), bounding_box_error_mm=error, volume_error_mm3=volume_error)


def export_prototype(output):
    output.mkdir(parents=True, exist_ok=True)
    parts, board, coil, winding, piece = prototype_parts()
    detail = winding_detail(coil, winding)
    validation = validate_parts(parts)
    copper_volume = sum(part.shape.volume for part in detail if part.group == 'Copper')
    expected_copper = winding.length_per_winding * winding.cross_section_area * 1000000000
    if abs(copper_volume - expected_copper) > Fixed.cad_geometry_tolerance ** 3 * winding.turns:
        raise ValueError('Detailed copper volume disagrees with the winding electrical model')
    exports = [export_checked(parts, output / 'coil_assembly.step'),
               export_checked(parts, output / 'coil_assembly_exploded.step', exploded=True),
               export_checked(detail, output / 'single_coil.step')]
    report = dict(status='Geometry reference; not manufacturing-ready or thermally qualified',
                  coordinates='millimetres; magnet bottom z=0; board below; exploded view adds display-only offsets',
                  winding=dict(turns=winding.turns, conductor_width_mm=coil.conductor_radial_width,
                               conductor_thickness_mm=Inputs.selected_conductor_thickness, height_mm=winding.coil_height,
                               outer_width_mm=coil.outer_width, outer_length_mm=coil.outer_length,
                               resistance_ohm=winding.resistance, wire_length_m=winding.length_per_winding,
                               copper_volume_mm3=copper_volume),
                  king=dict(mass_g=piece.mass, com_height_mm=piece.com_height * 1000, clearance_envelope_height_mm=piece.box_height,
                            note='Envelope is not a finished shell or CAD-derived mass distribution'),
                  nominal_magnet_gap_mm=Inputs.magnet_to_coil_distance,
                  visible_hover_mm=Inputs.magnet_to_coil_distance - Fixed.potting_cover_thickness - Fixed.playing_surface_thickness - Fixed.piece_bottom_skin,
                  preview=dict(size=Fixed.cad_preview_size, transparency=Fixed.cad_overlay_transparency,
                               margin=Fixed.cad_preview_margin, geometry_tolerance_mm=Fixed.cad_geometry_tolerance),
                  parts=[part_record(part) for part in parts], detail_parts=[part_record(part) for part in detail],
                  validation=validation, exports=exports,
                  limitations=['Sharp rectangular turns reproduce the existing model; bend radii, continuous layer transitions and lead exits are unresolved.',
                               'Assembly coils are winding envelopes including copper, insulation and reserved winding clearance, not solid copper.',
                               'Single-coil view has separate closed turn sections for packing inspection; it is not a connected manufacturable conductor.',
                               'Adjacent winding envelopes and the two orientation layers touch; no additional assembly tolerance or interlayer separator is assumed.',
                               'PCB, interface and baseplate are local envelopes; traces, sensors, heatsink fins, eddy-break slots and mounting are not yet detailed.',
                               'No driver selection, fixture design, temperature solution or new current qualification is claimed by this CAD model.'],
                  source_sha256=source_fingerprints(model.__file__, design.__file__, levitation_sim.__file__, __file__, Path(__file__).with_name('freecad_view.py')))
    (output / 'geometry_report.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    macro = 'from pathlib import Path\n' + f"exec(compile(Path({str(Path(__file__).with_name('freecad_view.py').resolve())!r}).read_text(), 'freecad_view.py', 'exec'), {{'output_directory': {str(output.resolve())!r}}})\n"
    (output / 'open_in_freecad.FCMacro').write_text(macro)
    print(json.dumps(dict(output=str(output), winding=report['winding'], validation=validation, exports=exports), indent=Fixed.report_precision))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Export the existing model geometry for CAD review; no field sweep required.')
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('output'))
    export_prototype(parser.parse_args().output)

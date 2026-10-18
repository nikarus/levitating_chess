import json
from copy import deepcopy
from pathlib import Path

import build123d as cad

import levitation_sim
import model
from model import Inputs, Fixed
from methodology import coil_layouts, design, prototype_coils, source_fingerprints
from methodology.cad import assembly, piece as piece_cad, winding as winding_cad


def square_parts():
    _, piece = coil_layouts.reference_state()
    candidate, coil, winding = prototype_coils.prototype_geometry(Inputs.cad_coil_layout)
    if candidate[1] != 'square' or Fixed.cad_patch_cells_per_side % 2 != 1:
        raise ValueError('The representative patch requires square coils and an odd cell count')
    array = prototype_coils.prototype_array(Inputs.cad_coil_layout)
    array.window = Fixed.cad_patch_cells_per_side * array.short_pitch
    array.build()
    angle = dict(Inputs.prototype_layouts)[Inputs.cad_coil_layout]
    rotation = cad.Rot(0, 0, angle)
    envelope = assembly.rectangular_ring(coil.outer_width, coil.outer_length, coil.winding_radial_width,
                                         winding.coil_height, coil.corner_radius)
    parts = []
    for placement in array.placements:
        identity = '_'.join(str(value) for value in placement['identity'])
        bottom = (placement['top'] - array.height) * 1000
        location = rotation * cad.Pos(placement['x'] * 1000, placement['y'] * 1000, bottom)
        parts.append(assembly.labeled_part(location * deepcopy(envelope), f'Coil_{identity}_winding_envelope',
                                           'sienna', 'Single_layer_coils', 3))
    patch_side = array.window * 1000 - Inputs.prototype_coil_spacing
    bed_bottom = -Inputs.magnet_to_coil_distance - winding.coil_height
    bottom = bed_bottom
    for label, height, color, group, level in (
        ('PCB_envelope_NOT_routed', Fixed.pcb_thickness, 'seagreen', 'PCB', 2),
        ('Thermal_interface_envelope', Fixed.radiator_standoff_below_pcb, 'slategray', 'Thermal_interface', 1),
        ('Baseplate_envelope', Fixed.baseplate_thickness, 'silver', 'Baseplate', 0),
    ):
        bottom -= height
        parts.append(assembly.labeled_part(rotation * assembly.block(patch_side, patch_side, height, 0, 0, bottom),
                                           label, color, group, level))
    surface_bottom = -Inputs.magnet_to_coil_distance
    for label, height, color, level in (
        ('Potting_cover', Fixed.potting_cover_thickness, 'lightgray', 4),
        ('Playing_surface', Fixed.playing_surface_thickness, 'ivory', 5),
    ):
        parts.append(assembly.labeled_part(rotation * assembly.block(patch_side, patch_side, height, 0, 0, surface_bottom),
                                           label, color, label, level, True))
        surface_bottom += height
    parts.extend(assembly.magnet_parts(levitation_sim.magnet_layout_from_geometry()))
    skin = cad.Pos(0, 0, -Fixed.piece_bottom_skin) * cad.Cylinder(piece.diameter / 2, Fixed.piece_bottom_skin,
                    align=(cad.Align.CENTER, cad.Align.CENTER, cad.Align.MIN))
    parts.append(assembly.labeled_part(skin, 'Piece_bottom_skin', 'ivory', 'Piece_skin', 6, True))
    parts.append(assembly.labeled_part(piece_cad.king_envelope(piece), 'King_clearance_envelope_NOT_material',
                                       'lightsteelblue', 'King_envelope', 6, True))
    return parts, array, winding, piece


def construction_parts():
    parts, array, winding, piece = square_parts()
    detail, pads, path, winding_report = winding_cad.detailed_winding()
    identity = ('x', *Fixed.cad_detailed_coil_cell)
    placement = next(row for row in array.placements if row['identity'] == identity)
    location = cad.Rot(0, 0, dict(Inputs.prototype_layouts)[Inputs.cad_coil_layout]) * cad.Pos(
        placement['x'] * 1000, placement['y'] * 1000, (placement['top'] - array.height) * 1000)
    detail_label = 'Coil_' + '_'.join(str(value) for value in identity) + '_winding_envelope'
    parts = [part for part in parts if part.shape.label != detail_label and part.group not in ('Piece_skin', 'King_envelope')]
    pcb = next(part for part in parts if part.group == 'PCB')
    for pad in pads:
        pcb.shape -= location * pad
    pcb.shape.label = 'PCB_envelope_with_terminal_recesses_NOT_routed'
    for part in detail:
        part.shape = location * part.shape
    neighbors = [part for part in parts if part.group == 'Single_layer_coils']
    winding_report['minimum_to_neighbor_envelope_mm'] = assembly.minimum_clearance(
        [part.shape for part in detail if part.group in ('Detailed_winding', 'Detailed_insulation')],
        [neighbor.shape for neighbor in neighbors])
    if winding_report['minimum_to_neighbor_envelope_mm'] < Fixed.cad_geometry_tolerance:
        raise ValueError('Detailed winding touches a neighboring winding envelope')
    parts.extend(detail)
    magnets = [part for part in parts if part.group == 'Magnets']
    shell, piece_report = piece_cad.construction_parts(piece, magnets)
    parts.extend(shell)
    return parts, array, winding, piece, winding_report, piece_report


def export_square(output):
    output.mkdir(parents=True, exist_ok=True)
    parts, array, winding, piece, winding_report, piece_report = construction_parts()
    validation = assembly.validate_parts(parts)
    export = assembly.export_checked(parts, output / 'square_assembly.step')
    report = dict(status='Integrated virtual square-coil candidate; incomplete product detail and physics qualification',
                  coordinates='millimetres; magnet bottom z=0; single coil plane below; no display offsets',
                  geometry=prototype_coils.geometry_record(Inputs.cad_coil_layout),
                  patch=dict(coils=len(array.identities), cells_per_side=Fixed.cad_patch_cells_per_side,
                             side_mm=array.window * 1000 - Inputs.prototype_coil_spacing,
                             active_selection_window_mm=Fixed.layout_screen_window_side,
                             note='Representative physical patch, not full board or simultaneous active-coil count'),
                  king=dict(mass_g=piece.mass, com_height_mm=piece.com_height * 1000, height_mm=piece.box_height,
                            base_diameter_mm=piece.diameter, construction=piece_report,
                            note='Reference values above belong to the previous motion screen; construction gives the new CAD-derived candidate'),
                  nominal_magnet_gap_mm=Inputs.magnet_to_coil_distance,
                  visible_hover_mm=Inputs.magnet_to_coil_distance - Fixed.potting_cover_thickness - Fixed.playing_surface_thickness - Fixed.piece_bottom_skin,
                  preview=dict(size=Fixed.cad_preview_size, transparency=Fixed.cad_overlay_transparency,
                               margin=Fixed.cad_preview_margin, geometry_tolerance_mm=Fixed.cad_geometry_tolerance),
                  parts=[assembly.part_record(part) for part in parts], validation=validation,
                  winding_construction=winding_report, export=export,
                  limitations=['One winding is detailed; other coils use nominal envelopes. Layer-crossing excursions fit the inter-coil space but are not in the closed-turn force approximation.',
                               'Nominal enamel separation is small; packing, positional tolerance and bend/process limits remain unqualified.',
                               'Pads and solder allowances define the connection geometry; PCB routing and final joint shape remain unresolved.',
                               'King shell, bonded pocket and mass properties are now modeled; adhesive strength, shell deformation and temperature limits remain unqualified.',
                               'PCB depth and winding height retain the square motion-screen stack; inter-coil potting, mounting, cooling and electronics need further detail.',
                               'Motion-screen mass and resistance have not been silently replaced; detailed geometry requires a new electrical/magnetic/control check.'],
                  source_sha256=source_fingerprints(model.__file__, design.__file__, levitation_sim.__file__,
                      coil_layouts.__file__, prototype_coils.__file__, assembly.__file__, piece_cad.__file__, winding_cad.__file__, __file__, Path(__file__).with_name('freecad_view.py')))
    (output / 'square_geometry.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    macro = f'''import json
import sys
from pathlib import Path
import FreeCAD as App
from uuid import uuid4
sys.path.insert(0, {str(Path(__file__).parent.resolve())!r})
import importlib
import freecad_view
importlib.reload(freecad_view)
output = Path({str(output.resolve())!r})
for document in list(App.listDocuments().values()):
    document.saveCopy(str(Path('/tmp') / (document.Name + '_before_square_review_' + uuid4().hex + '.FCStd')))
    App.closeDocument(document.Name)
report = json.loads((output / 'square_geometry.json').read_text())
result = freecad_view.create_document(output, 'square_assembly', 'square_assembly.step', report['parts'], report['preview'],
    detail_groups=('Single_layer_coils', 'Magnets', 'PCB', 'Thermal_interface', 'Baseplate'), transparent_groups=('King_shell',))
freecad_view.save_isolated_view(App.activeDocument(), output / 'square_winding.png',
    ('Detailed_winding', 'Coil_terminals'), report['preview'])
if result['imported_solids'] != report['export']['solids']:
    raise ValueError('FreeCAD square-assembly solid count mismatch')
(output / 'square_freecad_validation.json').write_text(json.dumps(dict(version=App.Version(), documents=[result]), indent={Fixed.report_precision}) + '\\n')
'''
    (output / 'open_square.FCMacro').write_text(macro)
    print(json.dumps(dict(patch=report['patch'], king=report['king'], validation=validation, export=export), indent=Fixed.report_precision))
    return report


if __name__ == '__main__':
    export_square(Path(__file__).resolve().parents[1] / 'results' / 'cad')

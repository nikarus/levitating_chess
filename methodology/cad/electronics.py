import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import build123d as cad

import model
from model import Inputs, Fixed
from methodology import design, prototype_coils, source_fingerprints
from methodology.cad import assembly


def export_cad(placement_path, models_directory, output):
    if output.exists():
        raise ValueError('Preserve existing CAD review evidence')
    output.mkdir(parents=True)
    placement = json.loads(placement_path.read_text())
    root = Path(model.__file__).resolve().parent
    board_path = root / 'methodology/electronics/driver_tile.kicad_pcb'
    if hashlib.sha256(board_path.read_bytes()).hexdigest() != placement['source_sha256'][board_path.relative_to(root).as_posix()]:
        raise ValueError('PCB changed after the recorded placement: re-export its placement first')
    manifest_path = models_directory / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    model_records = {record['path']: record for record in manifest['models']}
    models = {}
    for record in model_records.values():
        filename = models_directory / record['path']
        if hashlib.sha256(filename.read_bytes()).hexdigest() != record['sha256']:
            raise ValueError(f'Changed package model: {filename}')
        models[record['path']] = cad.import_step(filename)
    _, coil, winding = prototype_coils.prototype_geometry(Inputs.cad_coil_layout)
    side, other_side = placement['board_size_mm']
    if side != other_side or placement['physical_pcbs_per_coil_tile'] != Fixed.electronics_placement_board_count:
        raise ValueError('Placement and CAD configuration differ')
    pitch = side / Inputs.electronics_placement_cells_per_side
    if abs(pitch - coil.outer_width - Inputs.prototype_coil_spacing) > Fixed.cad_geometry_tolerance:
        raise ValueError('Placement and winding pitch differ')
    parts = []
    for board_index in range(Fixed.electronics_placement_board_count):
        top = -board_index * (Fixed.pcb_thickness + Fixed.cad_exploded_layer_spacing)
        prefix = f'PCB{board_index + 1}'
        parts.append(assembly.labeled_part(
            assembly.block(side, side, Fixed.pcb_thickness, side / 2, -side / 2, top - Fixed.pcb_thickness),
            prefix + '_substrate_UNROUTED', 'seagreen', prefix, 0))
        for part in placement['parts']:
            x, y = part['center_mm']
            if part['role'] == 'driver':
                shape = assembly.block(*Fixed.electronics_driver_package_envelope, 0, 0, 0)
                label = f"{prefix}_{part['reference']}_DDV_CLEARANCE_ENVELOPE"
                color = 'mediumpurple'
                group = prefix + '_driver_envelopes'
            else:
                library, name = part['footprint'].split(':')
                key = f'{library}.3dshapes/{name}.step'
                shape = deepcopy(models[key])
                label = f"{prefix}_{part['reference']}_{part['mpn']}_LIBRARY_PACKAGE"
                color = 'tan' if part['role'] == 'capacitor' else 'dimgray'
                group = prefix + '_library_packages'
            if part['face'] == 'bottom':
                location = cad.Pos(x, -y, top - Fixed.pcb_thickness) * cad.Rot(0, 180, 0)
            elif part['face'] == 'top':
                location = cad.Pos(x, -y, top)
            else:
                raise ValueError(f"Unknown PCB face: {part['face']}")
            shape = location * shape
            parts.append(assembly.labeled_part(shape, label, color, group, 0))
    coil_shape = assembly.rectangular_ring(coil.outer_width, coil.outer_length, coil.winding_radial_width,
                                           winding.coil_height, coil.corner_radius)
    for row in range(Inputs.electronics_placement_cells_per_side):
        for column in range(Inputs.electronics_placement_cells_per_side):
            location = cad.Pos((column + 0.5) * pitch, -(row + 0.5) * pitch, Fixed.cad_exploded_layer_spacing)
            parts.append(assembly.labeled_part(location * deepcopy(coil_shape),
                f'Coil_{column}_{row}_WINDING_ENVELOPE', 'sienna', 'Single_layer_coils', 0))
    validation = assembly.validate_parts(parts)
    exported = assembly.export_checked(parts, output / 'driver_assembly.step')
    board_geometry = design.BoardGeometry()
    report = dict(status='Exploded CAD review; not a manufactured stack or selected production module',
        placement_source=str(placement_path), parts=[assembly.part_record(part) for part in parts],
        preview=dict(size=Fixed.cad_preview_size, transparency=Fixed.cad_overlay_transparency,
                     margin=Fixed.cad_preview_margin, geometry_tolerance_mm=Fixed.cad_geometry_tolerance),
        validation=validation, export=exported,
        scale=dict(study_patch_mm=side, chess_square_mm=board_geometry.square_size,
                   playing_board_mm=board_geometry.board_side,
                   motor_area_mm=[board_geometry.motor_width, board_geometry.motor_height]),
        package_models=manifest,
        limitations=[
            'PCB outlines, package placements and sides come from the recorded native KiCad placement.',
            'KiCad library package models are generic package geometry, not manufacturer-qualified models of each exact MPN.',
            'Purple driver solids are DDV package clearance envelopes, including leads, not detailed vendor geometry.',
            'Copper winding envelopes reuse the existing rounded coil CAD; individual turns, terminations and supports are omitted.',
            'Inter-layer gaps are exploded display spacing, not approved mechanical or thermal dimensions.',
            'Production PCB dimensions, quantity, seam locations, mounting and cooling remain undecided.',
            'No routing, connectors, shared control/sensing circuitry or thermal hardware is included.'
        ],
        driver_package_reference='https://www.ti.com/lit/ds/symlink/tpa3251.pdf#page=40',
        source_sha256=source_fingerprints(__file__, model.__file__, design.__file__,
            prototype_coils.__file__, assembly.__file__, Path(__file__).with_name('freecad_view.py'),
            placement_path, board_path))
    (output / 'geometry.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in report['source_sha256']:
            archive.write(root / name, name)
        archive.write(manifest_path, 'package_models/manifest.json')
        for name in model_records:
            archive.write(models_directory / name, 'package_models/' + name)
    macro = f'''import json
import sys
from pathlib import Path
from uuid import uuid4
import FreeCAD as App
import FreeCADGui as Gui
sys.path.insert(0, {str(Path(__file__).parent.resolve())!r})
import freecad_view
output = Path({str(output.resolve())!r})
for document in list(App.listDocuments().values()):
    document.saveCopy(str(Path('/tmp') / (document.Name + '_before_electronics_' + uuid4().hex + '.FCStd')))
    App.closeDocument(document.Name)
report = json.loads((output / 'geometry.json').read_text())
result = freecad_view.create_document(output, 'driver_assembly', 'driver_assembly.step',
    report['parts'], report['preview'])
if result['imported_solids'] != report['export']['solids']:
    raise ValueError('Electronics STEP import changed the solid count')
document = App.activeDocument()
for obj in document.Objects:
    if 'GeometryStatus' in obj.PropertiesList:
        obj.GeometryStatus = 'Exploded review only; package and winding models remain unqualified'
        if 'CLEARANCE_ENVELOPE' in obj.Label:
            obj.GeometryStatus = 'Provisional DDV package clearance envelope; not detailed vendor CAD'
        if 'LIBRARY_PACKAGE' in obj.Label:
            obj.GeometryStatus = 'Generic KiCad package model; exact MPN mechanical qualification pending'
document.save()
(output / 'freecad_validation.json').write_text(json.dumps(result, indent={Fixed.report_precision}) + '\\n')
'''
    (output / 'open_driver_assembly.FCMacro').write_text(macro)
    print(json.dumps(dict(output=str(output), scale=report['scale'], validation=validation, export=exported),
                     indent=Fixed.report_precision))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Create inspectable electronics CAD from the recorded KiCad placement.')
    parser.add_argument('--placement', type=Path, required=True)
    parser.add_argument('--models', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    export_cad(args.placement, args.models, args.output)

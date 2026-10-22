import argparse
from collections import Counter
import hashlib
import itertools
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PlotPolygon, Patch
import numpy as np
import pcbnew

import model
from model import Inputs, Fixed
from methodology import design, prototype_coils, source_fingerprints
from methodology.electronics import review


COLORS = dict(driver='#374151', shunt='#9b4054', amplifier='#316cba', capacitor='#c89444', pullup='#805c9b')


def polygon_points(courtyard):
    outline = courtyard.Outline(0)
    return [(outline.CPoint(index).x / 1e6, outline.CPoint(index).y / 1e6)
            for index in range(outline.PointCount())]


def courtyard_polygon(points):
    polygon = pcbnew.SHAPE_POLY_SET()
    polygon.NewOutline()
    for x, y in points:
        polygon.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
    return polygon


def placement_check(parts, side):
    polygons = {part['reference']: courtyard_polygon(part['courtyard_mm']) for part in parts}
    overlaps = []
    for first, second in itertools.combinations(parts, 2):
        if first['face'] != second['face']:
            continue
        intersection = pcbnew.SHAPE_POLY_SET()
        intersection.BooleanIntersection(polygons[first['reference']], polygons[second['reference']])
        area = intersection.Area() / pcbnew.FromMM(1)**2
        if area > Fixed.cad_geometry_tolerance ** 2:
            overlaps.append(dict(references=[first['reference'], second['reference']], area_mm2=area))
    outside = [part['reference'] for part in parts if
               any(min(x, y) < -Fixed.cad_geometry_tolerance or max(x, y) > side + Fixed.cad_geometry_tolerance
                   for x, y in part['courtyard_mm'])]
    return dict(courtyard_overlaps=overlaps, outside_outline=outside,
                passed=not overlaps and not outside,
                face_occupancy={face: sum(polygons[p['reference']].Area() / pcbnew.FromMM(1)**2 for p in parts if p['face'] == face)
                                / side ** 2 for face in ('top', 'bottom')})


def build_tile(baseline, library, board_path):
    prior = json.loads(baseline.read_text())
    source_parts = {part['reference']: part for part in prior['components']}
    for part in source_parts.values():
        if part['model_parameter'] and float(part['value']) != getattr(Fixed, part['model_parameter']):
            raise ValueError(f"Stale reference component: {part['reference']}")
    _, coil, _ = prototype_coils.prototype_geometry(Inputs.cad_coil_layout)
    pitch = coil.outer_width + Inputs.prototype_coil_spacing
    count = Inputs.electronics_placement_cells_per_side
    driver_mpn, driver_footprint, driver_channels = Fixed.electronics_integrated_driver
    if count != 4 or Fixed.electronics_placement_board_count != 2 or driver_channels != 2:
        raise ValueError('This placement template requires four-by-four coils and two eight-channel PCBs')
    side = count * pitch
    board = pcbnew.BOARD()
    board.GetDesignSettings().SetBoardThickness(pcbnew.FromMM(Fixed.pcb_thickness))
    edge = pcbnew.PCB_SHAPE(board)
    edge.SetShape(pcbnew.SHAPE_T_RECT)
    edge.SetStart(pcbnew.VECTOR2I(0, 0))
    edge.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(side), pcbnew.FromMM(side)))
    edge.SetLayer(pcbnew.Edge_Cuts)
    board.Add(edge)
    title = pcbnew.TITLE_BLOCK()
    title.SetTitle('Placement study only - eight independent coil channels')
    title.SetComment(0, 'UNROUTED; no electrical netlist; cooling and interconnect not designed')
    board.SetTitleBlock(title)
    parts, paths = [], set()

    def add(reference, specification, role, face, x, y):
        footprint, path = review.load_footprint(library, specification['footprint'])
        paths.add(path)
        footprint.SetReference(reference)
        footprint.SetValue(specification['value'])
        footprint.Reference().SetVisible(False)
        footprint.Value().SetVisible(False)
        for name, value in dict(MPN=specification['mpn'], Role=role,
                                DesignStatus='PLACEMENT ONLY; no electrical connectivity',
                                ModelParameter=specification.get('model_parameter', '')).items():
            field = pcbnew.PCB_FIELD(footprint, footprint.GetNextFieldId(), name)
            field.SetText(value)
            field.SetVisible(False)
            footprint.AddField(field)
        footprint.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
        board.Add(footprint)
        if face == 'bottom':
            footprint.Flip(footprint.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        footprint.BuildCourtyardCaches()
        courtyard = footprint.GetCourtyard(pcbnew.F_CrtYd if face == 'top' else pcbnew.B_CrtYd)
        points = polygon_points(courtyard)
        pads = []
        for pad in footprint.Pads():
            bounds = pad.GetBoundingBox()
            pads.append([bounds.GetX() / 1e6, bounds.GetY() / 1e6,
                         bounds.GetWidth() / 1e6, bounds.GetHeight() / 1e6])
        parts.append(dict(reference=reference, role=role, face=face, center_mm=[x, y],
                          mpn=specification['mpn'], value=specification['value'],
                          footprint=specification['footprint'], courtyard_mm=points,
                          courtyard_area_mm2=courtyard.Area() / pcbnew.FromMM(1)**2, pad_bounds_mm=pads))

    driver = dict(mpn=driver_mpn, footprint=driver_footprint, value=driver_mpn)
    vm_bypass = dict(mpn='C0603C103K5RACTU', footprint=source_parts['C7']['footprint'],
                     value=f'{Fixed.electronics_integrated_bus_bypass:g}',
                     model_parameter='electronics_integrated_bus_bypass')
    support = [source_parts['C7'], vm_bypass, vm_bypass, source_parts['C2'],
               source_parts['C7'], source_parts['C7'], source_parts['C2'], source_parts['C7']]
    capacitor_index = 0
    for column, row in itertools.product(range(count // driver_channels), repeat=2):
        index = column * (count // driver_channels) + row
        origin_x, origin_y = column * driver_channels * pitch, row * driver_channels * pitch
        add(f'U{index + 1}', driver, 'driver', 'top', origin_x + pitch, origin_y + pitch)
        capacitor_index += 1
        add(f'C{capacitor_index}', source_parts['C3'], 'capacitor', 'top',
            side / 2, (index + 0.5) * pitch)
        for channel in range(driver_channels):
            channel_index = index * driver_channels + channel
            for reference, role, source, position in (
                    (f'R{channel_index + 1}', 'shunt', 'R11', Fixed.electronics_sense_placement[0]),
                    (f'U{channel_index + count + 1}', 'amplifier', 'U3', Fixed.electronics_sense_placement[1])):
                add(reference, source_parts[source], role, 'bottom',
                    origin_x + position[0] * driver_channels * pitch,
                    origin_y + (channel + position[1]) * pitch)
            for offset, fraction in enumerate(Fixed.electronics_bypass_placement_x):
                capacitor_index += 1
                specification = support[channel * len(Fixed.electronics_bypass_placement_x) + offset]
                add(f'C{capacitor_index}', specification, 'capacitor', 'bottom',
                    origin_x + fraction * driver_channels * pitch,
                    origin_y + (channel + Fixed.electronics_bypass_placement_y) * pitch)
    add(f'R{count * driver_channels + 1}', source_parts['R12'], 'pullup', 'top', side / 2, side / 2)
    if board_path.exists():
        raise ValueError('Do not overwrite an existing native placement')
    pcbnew.SaveBoard(str(board_path), board)
    reloaded = pcbnew.LoadBoard(str(board_path))
    if len(list(reloaded.GetFootprints())) != len(parts) or list(reloaded.GetTracks()):
        raise ValueError('Native board round-trip does not match the unrouted placement')
    round_trip = {footprint.GetReference(): footprint for footprint in reloaded.GetFootprints()}
    for part in parts:
        footprint = round_trip[part['reference']]
        footprint.BuildCourtyardCaches()
        face = pcbnew.F_Cu if part['face'] == 'top' else pcbnew.B_Cu
        courtyard_layer = pcbnew.F_CrtYd if part['face'] == 'top' else pcbnew.B_CrtYd
        if (footprint.GetLayer() != face or footprint.GetFieldByName('MPN').GetText() != part['mpn']
                or polygon_points(footprint.GetCourtyard(courtyard_layer)) != part['courtyard_mm']):
            raise ValueError(f"Native footprint round-trip changed {part['reference']}")
    return parts, paths, side


def draw_faces(parts, side, output):
    figure, axes = plt.subplots(1, 2, figsize=np.asarray(Fixed.cad_preview_size) / Fixed.layout_plot_dpi, layout="constrained")
    for axis, face in zip(axes, ('top', 'bottom')):
        axis.add_patch(plt.Rectangle((0, 0), side, side, facecolor='#ecf4ef', edgecolor='#37745c'))
        for part in parts:
            if part['face'] != face:
                continue
            axis.add_patch(PlotPolygon(part['courtyard_mm'], facecolor=COLORS[part['role']]))
            for x, y, width, height in part['pad_bounds_mm']:
                axis.add_patch(plt.Rectangle((x, y), width, height, facecolor='#e1be72', linewidth=0))
            if part['role'] in ('driver', 'amplifier', 'shunt'):
                axis.text(*part['center_mm'], part['reference'], ha='center', va='center', color='white')
        axis.set(xlim=(0, side), ylim=(side, 0), aspect='equal', xlabel='mm', ylabel='mm',
                 title='Top — power drivers' if face == 'top' else 'Bottom — precision current sensing')
        if face == 'bottom':
            axis.invert_xaxis()
    figure.suptitle(f'One {side:g} × {side:g} mm PCB controls 8 coils\n'
                    'Actual KiCad courtyards and pad bounds · bottom viewed from underneath\n'
                    'PLACEMENT ONLY · no tracks, connectors, ADC or cooling hardware')
    figure.legend(handles=[Patch(color=color, label=role.capitalize()) for role, color in COLORS.items()],
                  loc='outside lower center', ncol=len(COLORS))
    figure.savefig(output / 'board_faces.png', dpi=Fixed.layout_plot_dpi)
    plt.close(figure)


def export_placement(baseline, library, board_path, output):
    if output.exists():
        raise ValueError('Preserve existing placement evidence')
    output.mkdir(parents=True)
    parts, footprints, side = build_tile(baseline, library, board_path)
    check = placement_check(parts, side)
    total_area = sum(part['courtyard_area_mm2'] for part in parts)
    channels = sum(part['role'] == 'shunt' for part in parts)
    report = dict(status='unrouted_component_placement_only', board_size_mm=[side, side],
        physical_pcbs_per_coil_tile=Fixed.electronics_placement_board_count,
        coils_in_one_unchanged_layer=Inputs.electronics_placement_cells_per_side ** 2,
        independent_channels_per_pcb=channels, parts_per_pcb=len(parts),
        role_counts_per_pcb=dict(Counter(part['role'] for part in parts)),
        included_courtyard_area_per_channel_mm2=total_area / channels,
        one_pcb_for_all_coils_area_ratio=total_area * Fixed.electronics_placement_board_count / (2 * side ** 2),
        placement_check=check, native_footprint_round_trip_passed=True, parts=parts,
        electrical_netlist_present=False, electrical_drc_passed=False, thermal_qualification_passed=False,
        limitations=[
            'This is a lower-bound repeated-circuit inventory, not a complete hardware BOM.',
            'No nets or tracks: placement clearance does not establish electrical functionality or routability.',
            'ADC, references, controller, current-limit latch, supplies, bulk/regeneration, Hall sensing and all connectors/terminations are absent.',
            'Driver support capacitors and shared fault pullup are included; pin-level integration, fast bypass loops and protection configuration remain open.',
            'DDV library footprint is provisional; manufacturer land-pattern and thermal-pad dimensions require verification.',
            'Top-cooled drivers need accessible heatsinks on both physical PCBs; no cooling or spacing solution is implied.',
            'Native board thickness reuses the existing allowance. The separate CAD review owns 3D inspection.',
            'Tile axes follow the coil lattice; this does not change its orientation in the full board.',
            'Individual coil-to-board wiring is unassigned. Two boards split channel ownership; no coil pairing or simultaneous-current restriction is introduced.',
        ],
        references=['https://www.ti.com/lit/ds/symlink/drv8962.pdf',
                    'https://search.kemet.com/component-documentation/download/specsheet/C0603C103K5RACTU'],
        source_sha256=source_fingerprints(__file__, model.__file__, review.__file__,
                                         design.__file__, prototype_coils.__file__, baseline, board_path),
        footprint_sha256={str(path.relative_to(library)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in sorted(footprints)})
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in report['source_sha256']:
            archive.write(name, name)
        for path in sorted(footprints):
            archive.write(path, str(Path('footprints') / path.relative_to(library)))
    draw_faces(parts, side, output)
    if not check['passed']:
        raise ValueError(f'Placement failed: {check}')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Build and visualize an unrouted integrated-driver density study.')
    parser.add_argument('--baseline-review', type=Path, required=True)
    parser.add_argument('--footprints', type=Path, required=True)
    parser.add_argument('--board', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    result = export_placement(arguments.baseline_review, arguments.footprints, arguments.board, arguments.output)
    print(json.dumps({key: result[key] for key in (
        'status', 'board_size_mm', 'physical_pcbs_per_coil_tile', 'independent_channels_per_pcb',
        'parts_per_pcb', 'included_courtyard_area_per_channel_mm2', 'one_pcb_for_all_coils_area_ratio',
        'placement_check')}, indent=Fixed.report_precision))

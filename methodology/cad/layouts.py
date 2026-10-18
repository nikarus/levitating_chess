import json
from pathlib import Path

import build123d as cad
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.patches import PathPatch
from matplotlib.path import Path as PlotPath
import numpy as np

from model import Inputs, Fixed
from methodology import coil_layouts
from methodology.cad import assembly


def layout_parts(candidate, angle):
    coil_layouts.reference_state()
    coil, winding = coil_layouts.candidate_geometry(candidate)
    array = coil_layouts.LayoutArray(candidate[1], coil, winding, angle, min(Fixed.current_demand_mesh_refinements))
    parts = []
    for placement in array.placements:
        orientation, i, j = placement['identity']
        shape = cad.Rot(Z=angle) * cad.Pos(placement['x'] * 1000, placement['y'] * 1000,
                                          (placement['top'] - array.height) * 1000) * assembly.rectangular_ring(
            placement['width'] * 1000, placement['length'] * 1000, coil.winding_radial_width, winding.coil_height)
        parts.append(assembly.labeled_part(shape, f'Coil_{orientation}_{i}_{j}',
                                          'sienna' if orientation == 'x' else 'goldenrod', candidate[0], 0))
    return parts, array


def export_comparison():
    output = Path(__file__).resolve().parents[1] / 'results' / 'cad'
    output.mkdir(parents=True, exist_ok=True)
    report_path = Path(coil_layouts.__file__).with_name('results') / 'coil_layouts.json'
    report = json.loads(report_path.read_text())
    selected = report['refined_results']
    candidates = {row[0]: row for row in Inputs.coil_layout_candidates}
    figure, axes = plt.subplots(1, len(selected), figsize=np.asarray(Fixed.layout_plot_size) / Fixed.layout_plot_dpi,
                               layout='constrained')
    exports = []
    size = max(Fixed.layout_screen_window_side * (abs(np.cos(np.radians(row['rotation_deg'])))
                + abs(np.sin(np.radians(row['rotation_deg'])))) for row in selected) * Fixed.cad_preview_margin / 2
    for axis, row in zip(axes, selected):
        candidate = candidates[row['name']]
        parts, array = layout_parts(candidate, row['rotation_deg'])
        assembly.validate_parts(parts)
        path = output / f'{row["name"]}.step'
        exports.append(assembly.export_checked(parts, path))
        for placement in sorted(array.placements, key=lambda part: part['top']):
            color = 'sienna' if placement['identity'][0] == 'x' else 'goldenrod'
            boundaries = []
            for inset in (0, candidate[-1] / 1000):
                half_x, half_y = placement['width'] / 2 - inset, placement['length'] / 2 - inset
                corners = np.array([[-half_x, -half_y, 0], [half_x, -half_y, 0], [half_x, half_y, 0], [-half_x, half_y, 0]])
                corners += [placement['x'], placement['y'], 0]
                world = array.rotation.apply(corners)[:, :2] * 1000
                boundaries.append(world if inset == 0 else world[::-1])
            vertices = np.concatenate([np.vstack((boundary, boundary[:1])) for boundary in boundaries])
            codes = [PlotPath.MOVETO, PlotPath.LINETO, PlotPath.LINETO, PlotPath.LINETO, PlotPath.CLOSEPOLY] * len(boundaries)
            axis.add_patch(PathPatch(PlotPath(vertices, codes), facecolor=color, edgecolor=color))
        half_magnet = Inputs.magnet_lateral_edge * Inputs.magnets_per_period * Inputs.periods_per_side / 2
        axis.add_patch(Polygon([[-half_magnet, -half_magnet], [half_magnet, -half_magnet],
                                [half_magnet, half_magnet], [-half_magnet, half_magnet]],
                               facecolor='none', edgecolor='royalblue', linestyle='dashed', label='Magnet footprint'))
        axis.set(xlim=(-size, size), ylim=(-size, size), aspect='equal', xlabel='mm', ylabel='mm')
        axis.set_title(f'{row["pattern"].capitalize()} ({row["rotation_deg"]:g}°)\n{candidate[2]:g} × {candidate[3]:g} mm; {array.layers} coil layer(s)')
    figure.suptitle('Coil geometry comparison — same scale\nSingle-layer candidates are not yet qualified hardware')
    figure.legend(*axes[0].get_legend_handles_labels(), loc='outside lower center')
    figure.savefig(report_path.with_name('coil_layouts.png'), dpi=Fixed.layout_plot_dpi)
    plt.close(figure)
    (output / 'layout_exports.json').write_text(json.dumps(exports, indent=Fixed.report_precision) + '\n')


if __name__ == '__main__':
    export_comparison()

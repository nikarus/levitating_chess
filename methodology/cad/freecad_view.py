import json
from pathlib import Path

import FreeCAD as App
import FreeCADGui as Gui
import ImportGui
from pivy import coin


def fit_view(view, shapes, preview):
    camera = view.getCameraNode()
    if not isinstance(camera, coin.SoOrthographicCamera):
        raise ValueError('CAD preview requires an orthographic camera')
    rotation = view.getCameraOrientation()
    inverse = rotation.inverted()
    bounds = App.BoundBox()
    for shape in shapes:
        box = shape.optimalBoundingBox(False, False)
        for x in (box.XMin, box.XMax):
            for y in (box.YMin, box.YMax):
                for z in (box.ZMin, box.ZMax):
                    bounds.add(inverse.multVec(App.Vector(x, y, z)))
    position = inverse.multVec(App.Vector(*camera.position.getValue().getValue()))
    position.x, position.y = bounds.Center.x, bounds.Center.y
    camera.position.setValue(*rotation.multVec(position))
    viewport_width, viewport_height = view.getSize()
    aspect = min(viewport_width / viewport_height, preview['size'][0] / preview['size'][1])
    camera.height.setValue(max(bounds.YLength, bounds.XLength / aspect) * preview['margin'])
    view.redraw()
    Gui.updateGui()


def create_document(output, name, step_filename, records, preview, verify_positions=True, detail_groups=(), visible_overlay_groups=(), transparent_groups=()):
    document = App.newDocument(name)
    ImportGui.insert(str(output / step_filename), document.Name)
    document.recompute()
    objects_by_label = {obj.Label: obj for obj in document.Objects}
    maximum_dimension_error = 0
    maximum_volume_error_per_area = 0
    for record in records:
        obj = objects_by_label[record['label']]
        obj.ViewObject.ShapeColor = tuple(record['color'])
        obj.addProperty('App::PropertyString', 'ModelRole', 'Reference')
        obj.ModelRole = record['group']
        obj.addProperty('App::PropertyString', 'GeometryStatus', 'Reference')
        obj.GeometryStatus = 'Reference geometry; winding manufacture and thermal performance unverified'
        expected_volume = record['volume_mm3']
        volume_error_per_area = abs(obj.Shape.Volume - expected_volume) / obj.Shape.Area
        maximum_volume_error_per_area = max(maximum_volume_error_per_area, volume_error_per_area)
        if volume_error_per_area > preview['geometry_tolerance_mm']:
            raise ValueError(f'FreeCAD import volume mismatch: {obj.Label}')
        box = obj.Shape.optimalBoundingBox(False, False)
        dimensions = (box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax)
        dimension_error = max(abs(actual - expected) for actual, expected in zip(dimensions, record['minimum_mm'] + record['maximum_mm']))
        maximum_dimension_error = max(maximum_dimension_error, dimension_error)
        if verify_positions and dimension_error > preview['geometry_tolerance_mm']:
            raise ValueError(f'FreeCAD import bounding-box mismatch: {obj.Label}')
        if record['overlay'] or record['group'] in transparent_groups:
            obj.ViewObject.Transparency = preview['transparency']
        if record['overlay']:
            obj.ViewObject.Visibility = record['group'] in visible_overlay_groups
    shapes = [objects_by_label[record['label']].Shape for record in records]
    if not all(shape.isValid() for shape in shapes):
        raise ValueError(f'Invalid imported shape in {name}')
    solids = sum(len(shape.Solids) for shape in shapes)
    document.recompute()
    view = Gui.activeDocument().activeView()
    view.setAnimationEnabled(False)
    view.setCameraType('Orthographic')
    view.viewAxonometric()
    Gui.updateGui()
    view.fitAll()
    visible_shapes = [objects_by_label[record['label']].Shape for record in records if not record['overlay'] or record['group'] in visible_overlay_groups]
    fit_view(view, visible_shapes, preview)
    document.saveAs(str(output / f'{name}.FCStd'))
    view.saveImage(str(output / f'{name}.png'), *preview['size'], 'White')
    if detail_groups:
        view.viewFront()
        detail_shapes = [objects_by_label[record['label']].Shape for record in records if record['group'] in detail_groups]
        fit_view(view, detail_shapes, preview)
        view.saveImage(str(output / f'{name}_detail.png'), *preview['size'], 'White')
        view.viewAxonometric()
        fit_view(view, visible_shapes, preview)
    return dict(document=document.Name, source=step_filename, imported_solids=solids, valid=True,
                positions_verified=verify_positions, maximum_dimension_error_mm=maximum_dimension_error if verify_positions else None,
                maximum_volume_error_per_area_mm=maximum_volume_error_per_area, labels=list(objects_by_label))


def save_isolated_view(document, filename, groups, preview):
    objects = [obj for obj in document.Objects if 'ModelRole' in obj.PropertiesList]
    visibility = {obj.Name: obj.ViewObject.Visibility for obj in objects}
    for obj in objects:
        obj.ViewObject.Visibility = obj.ModelRole in groups
    view = Gui.activeDocument().activeView()
    view.viewAxonometric()
    fit_view(view, [obj.Shape for obj in objects if obj.ViewObject.Visibility], preview)
    view.saveImage(str(filename), *preview['size'], 'White')
    for obj in objects:
        obj.ViewObject.Visibility = visibility[obj.Name]
    fit_view(view, [obj.Shape for obj in objects if obj.ViewObject.Visibility], preview)


def open_reference(output_directory):
    output = Path(output_directory)
    report = json.loads((output / 'geometry_report.json').read_text())
    documents = [create_document(output, 'coil_assembly', 'coil_assembly.step', report['parts'], report['preview']),
                 create_document(output, 'single_coil', 'single_coil.step', report['detail_parts'], report['preview']),
                 create_document(output, 'coil_assembly_exploded', 'coil_assembly_exploded.step', report['parts'], report['preview'], verify_positions=False)]
    for result, expected in zip(documents, (report['exports'][0], report['exports'][2], report['exports'][1])):
        if result['imported_solids'] != expected['solids']:
            raise ValueError(f'FreeCAD solid count mismatch: {result["document"]}')
    (output / 'freecad_validation.json').write_text(json.dumps(dict(version=App.Version(), documents=documents), indent=4) + '\n')
    App.Console.PrintMessage('Flying chess CAD references loaded. Overlays can be shown using the model tree.\n')


if "output_directory" in globals():
    open_reference(output_directory)

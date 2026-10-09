import json
from pathlib import Path

import FreeCAD as App
import FreeCADGui as Gui
import ImportGui
from pivy import coin


def create_document(output, name, step_filename, records, preview):
    document = App.newDocument(name)
    ImportGui.insert(str(output / step_filename), document.Name)
    document.recompute()
    objects_by_label = {obj.Label: obj for obj in document.Objects}
    for record in records:
        obj = objects_by_label[record['label']]
        obj.ViewObject.ShapeColor = tuple(record['color'])
        obj.addProperty('App::PropertyString', 'ModelRole', 'Reference')
        obj.ModelRole = record['group']
        obj.addProperty('App::PropertyString', 'GeometryStatus', 'Reference')
        obj.GeometryStatus = 'Reference geometry; winding manufacture and thermal performance unverified'
        expected_volume = record['volume_mm3']
        if abs(obj.Shape.Volume - expected_volume) > preview['geometry_tolerance_mm'] ** 3 * len(obj.Shape.Solids):
            raise ValueError(f'FreeCAD import volume mismatch: {obj.Label}')
        if record['overlay']:
            obj.ViewObject.Transparency = preview['transparency']
            obj.ViewObject.hide()
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
    camera = view.getCameraNode()
    if not isinstance(camera, coin.SoOrthographicCamera):
        raise ValueError('CAD preview requires an orthographic camera')
    rotation = view.getCameraOrientation()
    inverse = rotation.inverted()
    bounds = App.BoundBox()
    for record in records:
        if record['overlay']:
            continue
        box = objects_by_label[record['label']].Shape.BoundBox
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
    document.saveAs(str(output / f'{name}.FCStd'))
    view.saveImage(str(output / f'{name}.png'), *preview['size'], 'White')
    return dict(document=document.Name, source=step_filename, imported_solids=solids, valid=True,
                labels=list(objects_by_label))


output = Path(output_directory)
report = json.loads((output / 'geometry_report.json').read_text())
documents = [create_document(output, 'coil_assembly', 'coil_assembly.step', report['parts'], report['preview']),
             create_document(output, 'single_coil', 'single_coil.step', report['detail_parts'], report['preview']),
             create_document(output, 'coil_assembly_exploded', 'coil_assembly_exploded.step', report['parts'], report['preview'])]
for result, expected in zip(documents, (report['exports'][0], report['exports'][2], report['exports'][1])):
    if result['imported_solids'] != expected['solids']:
        raise ValueError(f'FreeCAD solid count mismatch: {result["document"]}')
(output / 'freecad_validation.json').write_text(json.dumps(dict(version=App.Version(), documents=documents), indent=4) + '\n')
App.Console.PrintMessage('Flying chess CAD references loaded. Overlays can be shown using the model tree.\n')

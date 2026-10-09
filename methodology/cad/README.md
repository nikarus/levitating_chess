# Coil assembly CAD

The first CAD reference reproduces the existing model's local coil window, vertical stack and king magnet array. It is a geometry review, not a manufacturing release or a new thermal/levitation qualification.

The two stacked coil orientations are inherited from the simulation, not an established optimum. See [COIL_LAYOUT_HISTORY.md](../COIL_LAYOUT_HISTORY.md) for the introduction commit, the distinction from a single-layer parquet layout, and the unresolved architecture comparison.

## Inspect in FreeCAD

FreeCAD 1.1.4 is installed for this user and available in the application menu. Open these saved documents:

- [Exploded assembly](output/coil_assembly_exploded.FCStd): separated layers for inspection. The added separation is display-only.
- [Assembled geometry](output/coil_assembly.FCStd): actual modeled positions and gaps.
- [Single winding](output/single_coil.FCStd): individual copper turn sections and insulation.

Each document has named parts and groups. Potting, playing surface, piece bottom skin and the king clearance envelope are initially hidden to expose the winding and magnets. Select an item in the model tree and press Space to toggle visibility. The king envelope is not a solid piece design or a material region. Magnet names identify polarization direction; colors distinguish directions.

Portable STEP versions and PNG previews are alongside the native documents. STEP and FreeCAD files are generated geometry; edit the Python source and regenerate to change dimensions. These are not FreeCAD sketch-based parametric feature trees.

## Source of dimensions

All engineering and display settings live in `Inputs`, `Fixed` or `Constants` in [model.py](../../model.py), with parameter comments. The CAD generator consumes `BoardGeometry`, `CoilBed`, `WindingGeometry` and `Piece` from the existing calculations, and the coil placements and magnet arrangement from `levitation_sim.py`. `WindingGeometry` is also used by the electrical model; CAD does not maintain a second winding calculation or require a magnetic field sweep.

The current reference contains 24 coil envelopes in two orientation layers and 16 magnet blocks. Each winding envelope is 10 × 15 mm, with 57 turns and a 4.218 mm stack height. The modeled copper cross-section is 0.956 × 0.05 mm. These values are generated from the model, not separate CAD requirements. See [geometry_report.json](output/geometry_report.json) for dimensions, resistance, wire length, bounding boxes and source fingerprints.

Coordinates are millimetres. The magnet bottoms are at z = 0; the board is below them. The assembled model retains the nominal 3.5 mm magnet-to-coil gap and the resulting 3.1 mm visible hover gap. The exploded document must not be used to measure physical vertical gaps.

## Regenerate

Run from the repository root:

```sh
methodology/cad/.venv/bin/python -m methodology.cad.assembly
~/.local/bin/freecad methodology/cad/output/open_in_freecad.FCMacro
```

The first command creates STEP files, the geometry report and the opening macro. It checks solid validity, material intersections, copper volume against the electrical model, and STEP export/import preservation. The macro imports those files into FreeCAD, checks individual volumes and total solid counts, and saves native documents, previews and [freecad_validation.json](output/freecad_validation.json). It leaves the exploded assembly active. Close earlier generated tabs before repeated manual imports if you want to avoid duplicate open documents.

Software checks:

```sh
methodology/cad/.venv/bin/python -m unittest discover -s methodology/cad/tests -v
methodology/cad/.venv/bin/python -m unittest discover -s methodology/tests -v
```

The CAD environment uses build123d 0.13.0 plus the existing pinned calculation dependencies. [requirements.txt](requirements.txt) declares direct dependencies; [environment.lock.txt](environment.lock.txt) records the installed environment. The existing root virtual environment was left intact. To create the separate environment on a system with Python's pip available:

```sh
python3 -m venv --without-pip methodology/cad/.venv
python3 -m pip --python methodology/cad/.venv/bin/python install -r methodology/cad/requirements.txt
```

FreeCAD uses its own bundled Python. Its official [1.1.4 Linux AppImage](https://github.com/FreeCAD/FreeCAD/releases/tag/1.1.4) was verified against the release SHA-256, extracted under `~/.local/opt/freecad-1.1.4/`, and linked from `~/.local/bin/freecad`. Downloaded archive SHA-256: `f6dc6ba676e5ac96a565ebc8d657232f94c6158e85b4352141bd1a46f6b43434`.

## What remains to make this buildable

The winding currently has sharp rectangular corners. The single-coil view contains separate closed turn sections to inspect packing; it is not a continuous conductor. Real bend radii, transitions between layers, lead exits, insulation at crossings and terminals still need a construction decision and a supplier/process check. Assembly coil envelopes include insulation and reserved winding clearance, so their volume must not be interpreted as solid copper.

Adjacent winding envelopes and the two orientation layers touch in the existing model. There is no extra separator or assembly tolerance hidden in the CAD. A construction change that increases the stack or gap must feed back into the shared geometry and magnetic calculation.

The PCB, thermal interface and baseplate are local spatial envelopes. They do not yet contain a circuit, sensors, mounting details, cooling fins or eddy-current interruption slots. The heat path is visible, but no spatial thermal solution was run and the existing thermal failures remain open.

The next design task is to resolve a practical winding construction and lead routing, then develop a representative driver/sensing arrangement and measurement fixture around it. Maintain the existing loss/temperature screen while doing that; use detailed thermal or electromagnetic analysis when a specific unresolved decision requires it.

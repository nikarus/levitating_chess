# CAD organization

This directory owns geometry generators, FreeCAD inspection tooling, CAD-specific checks and the active shared Python environment. Geometry and quantitative assumptions reuse the main model. Generated review files belong in `../results/cad/`; numerical studies and motion replays belong with their evidence under `../results/`.

The parent [organization rules](../PROJECT_ORGANIZATION.md) apply here. Keep dependency declarations and the complete environment lock beside the tooling. Keep source geometry authoritative; do not introduce a second design in an exported CAD document. Retain only the current review exports and prune replaced views. Use one FreeCAD instance and one current review document, preserving unsaved work before closing older views.

# Assembly CAD

The current [square-coil assembly](output/square_assembly.FCStd) shows a representative patch of closely packed coils, the Halbach array and the approximate king. This is the starting point for the complete virtual product design. Manufacturing, fixtures and physical testing are deferred; see [NEXT_STEPS.md](../NEXT_STEPS.md).

## Inspect in FreeCAD

Keep one FreeCAD instance and one current review document. Preserve unsaved work before closing earlier views. The generated opening macro saves recovery copies of open documents under `/tmp`, closes them, and loads the updated assembly. Move any recovery copy you need to a durable location; `/tmp` is temporary storage. Reuse the running application:

```sh
~/.local/bin/freecad - --single-instance /home/arseniy/projects/flying_chess/methodology/cad/output/open_square.FCMacro
```

Most brown shapes are nominal winding envelopes in one plane. The foreground coil contains a continuous copper conductor, routed leads, terminal pads and solder allowances. The translucent king is now a hollow shell with a bonded magnet pocket. The green layer is the PCB envelope, with local terminal recesses. Potting cover, playing surface, bottom film, adhesive and enamel are initially hidden. Select an item in the tree and press Space to toggle it.

The [isolated winding preview](output/square_winding.png) shows the detailed conductor and terminals at their actual positions. It is generated from the same document; the macro restores the whole-assembly view afterward.

The [overview](output/square_assembly.png) and [front detail](output/square_assembly_detail.png) show the same physical positions. Coordinates are millimetres; magnet bottoms are at z = 0. No exploded display offsets are applied. [square_geometry.json](output/square_geometry.json) records dimensions, assumptions and checks. The [STEP file](output/square_assembly.step) is portable geometry; neither it nor the FreeCAD import has an editable sketch history. Edit the authoritative Python source to change geometry.

## Shared geometry and remaining detail

The generator reuses the candidate winding, lattice placements, king approximation and magnets from the existing calculations. All quantitative settings live in `Inputs`, `Fixed` or `Constants` in [model.py](../../model.py). The displayed patch is larger than the active calculation window; it does not change channel allocation or imply all visible coils run simultaneously.

Coil tops, PCB depth and baseplate depth match the square motion screen. The stacked-reference potting bond is not silently inserted into this candidate. Any new bond thickness or altered stack must feed back into the electrical, magnetic and thermal calculations.

The shell and winding supply CAD-derived mass/inertia, resistance and a continuous conductor path to the magnetic adapter. It reuses the existing solver without rebuilding solid exports or opening FreeCAD for each pose. The detailed comparison and original simplified screen retain separate numerical evidence. The other winding packs are not solid copper. Tolerance budgets, bond strength, winding bend/process limits, inter-coil potting, mounting, routed PCB and cooling remain unresolved. See [COILS.md](../docs/COILS.md) for the calculated geometry and operating evidence. The current geometry checks do not establish closed-loop stability or temperature limits.

The same geometry now feeds a [MuJoCo motion scene](output/motion_scene.xml), with an [assembly preview](output/motion_scene.png). Its numerical checks and current limitations are maintained in [MODELING_APPROACH.md](../docs/MODELING_APPROACH.md#current-motion-interface). This adds motion/contact evaluation while FreeCAD remains the assembly inspection tool.

Earlier stacked CAD exports remain references for the main model, and layout-comparison exports support the geometry shortlist. Their sharp-corner flat-wire geometry is different from the current rounded round-wire candidate. Do not transfer their current targets or use an exploded view to measure physical gaps.

## Regenerate and verify

From the repository root:

```sh
methodology/cad/.venv/bin/python -m methodology.cad.square_assembly
~/.local/bin/freecad - --single-instance /home/arseniy/projects/flying_chess/methodology/cad/output/open_square.FCMacro
methodology/cad/.venv/bin/python -m unittest discover -s methodology/cad/tests -v
methodology/cad/.venv/bin/python -m unittest discover -s methodology/tests -v
```

Export checks validate solids, physical-region intersections, conductor-path clearance and detailed-coil spacing and STEP round-trip preservation. Reference envelopes are excluded from physical collision claims. FreeCAD separately checks imported dimensions, volumes and solid count and saves the native document and previews. Tests check that the CAD patch preserves force-model placement, one-layer spacing, conductive-stack depths magnet containment in the king outline, continuous winding polarity and the need for crossover detours.

The dedicated CAD environment uses build123d with the existing calculation dependencies. [requirements.txt](requirements.txt) declares direct dependencies; [environment.lock.txt](environment.lock.txt) records the installed environment. FreeCAD uses its own bundled Python and is available through `~/.local/bin/freecad`. Keep temporary execution logs and recovery files outside the repository; retain only the current reproducible review exports.

# Why the model has stacked coils

Repository investigation on 2026-10-09. This records historical evidence and its limits; it does not select a new coil architecture.

## What the history establishes

The explicit stacked geometry first appears in [a252093](https://github.com/nikarus/levitating_chess/commit/a2520931ca8184cbce8bab8ca5569b70aeffb7f2), committed on 2026-06-06 with the message “bugfixing”. That message contains no design rationale.

Before that commit, `levitation_sim.py` used a `CoilPatch` of square, zero-thickness current loops in one plane. It had its own input classes and an assumed round-wire winding to convert ampere-turns into current. It did not implement an alternating rectangular parquet coil layout. Meanwhile, `model.py` already described rectangular coils and counted two orientation families over the same platform area. Thus the two calculations did not represent the same winding geometry.

The commit replaced `CoilPatch` with `CoilArray`, imported the winding dimensions, height, turns, resistance and limits from `model.py`, and represented the winding cross-section with multiple filaments. It explicitly placed one rectangular orientation at `-gap` and the other at `-gap - coil_height`. Its report called this “2 orientations stacked”.

The same commit added minimum-peak-current allocation, enlarged the local control window, revised lateral-force and hover-power accounting, and expanded neighbour-force and control-authority checks. These are consistent with an effort to make the force/current simulation represent the engineering model more faithfully.

## Was this a solution to a thermal failure?

No such justification was found in the reviewed history. The introduction commit changes thermal power accounting to use its revised hover-power estimate, but contains no parquet-versus-stacked thermal comparison, no layout-selection algorithm, and no saved result demonstrating that a single-layer layout failed while stacking passed.

Flat-wire support was introduced later in [2ac0408](https://github.com/nikarus/levitating_chess/commit/2ac0408db7b187dc37c66b2d6dcaa5f65be923e8), on 2026-06-08. The later “Add passive cooling model” commit, [b859347](https://github.com/nikarus/levitating_chess/commit/b859347), dates to 2026-06-15. Basic thermal calculations existed before both; this chronology alone cannot establish the author's motivation.

The most supported interpretation is that stacking was adopted while reconciling the two models and making the winding geometry finite. This is an inference from the code changes, not a recovered statement of intent. The available Git evidence does not establish why stacking was chosen over a properly modeled single-layer parquet alternative, or whether that choice was explicitly approved outside the repository.

The name `herringbone` also needs care: the old simulation used it for a **magnet polarization pattern**. That name does not establish that the coil array underneath was arranged as parquet. The retained `herringbone_orientation_families` parameter similarly does not describe the actual coil placement clearly.

## Consequence for current work

The recent current-demand and dynamic calculations, and the generated CAD, use the stacked geometry with the lower orientation farther from the magnets. They remain evidence for that geometry, not for a single-layer alternative. Stacking has not been demonstrated to be the best thermal or electromagnetic choice.

Before refining fabrication details, compare the existing baseline with a physically valid single-layer layout using the shared model. Recalculate current demand, losses and control authority. Keep the existing frequency targets as starting settings and rerun the dynamic and electrical checks before carrying those targets over. Simply putting both existing arrays at the same height would create conductor intersections.

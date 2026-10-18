# Modeling scope and current evidence

The [simulator reference](SIMULATOR.md) owns architecture and verification rules; the [single-piece checklist](SINGLE_PIECE_TESTS.md) owns behavioral coverage; [NEXT_STEPS.md](../NEXT_STEPS.md) owns execution order. This document records current engineering evidence, its limits and the tool choices still in effect. Quantitative settings remain in the model parameter classes. Complete the virtual design before manufacturing or physical tests.

## Physical-model verification gate

**The bounded implementation-verification gate is complete; whole-product and hardware validation are not.** The [original verification record](../results/physics_verification/implementation_verification/summary.json) and [joint-integrator qualification](../results/physics_verification/joint_passive_filter/qualification.json) support single-piece airborne investigation with CAD geometry, averaged drivers and causal Hall/current sensing. Atlas acceleration has separate evidence below.

The supported local model uses the square-coil patch and CAD-derived king. Runtime guards enforce the configured gap/current/domain bounds, tilted-bottom clearance, finite states and supported current extinction. Contact, reconduction and changing the controlled coil window remain outside this airborne path. Numerical verification does not establish unknown materials, component specifications, controller robustness or every possible trajectory.

| Verified mechanism | Authoritative evidence |
|---|---|
| CAD mechanics, coordinate frames, clocks, causality and replay | Component checks and recorded selections in the [simulator reference](SIMULATOR.md#general-properties-and-existing-checks) and frozen verification records |
| Curved-conductor force/torque and spatial refinement | [Curved force and independent flux derivative](../results/physics_verification/implementation_verification/curved_force/summary.json) |
| Force/back-EMF reciprocity, winding work and coupled dynamics | [Independent coupling references](../results/physics_verification/implementation_verification/independent_coupling/summary.json) |
| Joint temporal integration, switching, passive Hall filter and restart | [Joint qualification](../results/physics_verification/joint_passive_filter/qualification.json) and [combined sensor budget](../results/physics_verification/joint_passive_filter/verified_sensor_budget/summary.json) |
| Atlas field/force accuracy and closed sensor feedback | [Field screen](../results/hover/joint_control/atlas_backend/fields/summary.json), [coupled comparison](../results/hover/joint_control/atlas_backend/coupled/summary.json) and [combined sensing check](../results/hover/joint_control/atlas_backend/sensing/summary.json) |

These are sampled and bounded comparisons, not a rigorous global error certificate. The joint sensing benchmark's largest combined discrepancies are 0.206/0.214 µT against its unchanged 0.566 µT allowance. The atlas startup sensing check reaches 0.18105 µT under that same allowance. Conductor, trajectory, temporal/filter and independent-reference contributions are included without relying on cancellation. Energy consistency supplements state and field checks; it cannot alone establish trajectory accuracy.

Source, dependency and parameter identities in each record belong to that execution. Source archives preserve reproducibility across later edits, including output-path cleanup. Never replace recorded fingerprints or resume an old checkpoint with changed sources. A newly configured run needs its own provenance and compatible cache.

## Current motion interface

The [MuJoCo scene](../results/motion_scene.xml), [preview](../results/motion_scene.png) and [contact demonstration](../results/motion_scene.mp4) reuse the square assembly's mass, centre of mass and full inertia tensor. The [motion interface record](../results/motion.json) checks analytical force/rotation, free fall, contact impulse and a short prescribed-current magnetic case. The video is an unpowered drop/slide diagnostic, not controlled flight.

Contact uses a bottom-film cylinder and an interior-board plane with provisional friction/compliance. Piece-to-piece contact, board edges, deformation and detailed material damping remain unqualified. The [moving Hall screen](../results/sensing.json) retains a failed finer-resolution landing comparison; airborne success does not close this contact limitation. Geometry and construction uncertainties are maintained in [COILS.md](COILS.md).

## Current Hall interface

Sequential Hall sampling, analogue filtering, ADC rounding, calibration errors, noise and delayed delivery are represented. The estimator uses measured currents for coil-field subtraction and nominal geometry for pose estimation. It receives no true pose or hidden plant properties. The architecture and timing contract live in [SIMULATOR.md](SIMULATOR.md#scope-and-information-flow).

The retained [single-count response](../results/hover/count_response_extended/summary.json) shows why digital trace identity is not equivalent to physical tracking: one changed Hall ADC count produced persistent current differences while both trajectories stayed within operating limits over the recorded interval. This is bounded disturbance evidence, not general recovery qualification. Preserve physical noise and hardware quantization during numerical comparisons; do not use numerical error as a substitute for real interference.

## Joint integration review

The [reduced solver comparison](../results/physics_verification/solver_review/summary.json) supports RK45 as the joint integration starting point against independent DOP853/Radau references. It uses manufactured coupling equations and cannot predict full CAD-field performance. The [interior-release diagnosis](../results/hover/interior_release/diagnosis.json) records the duration-dependent cost of the earlier partitioned work-budget method. Those legacy paths remain for their reference checks; they are not the current flight runner.

## Joint integration candidate

The verified [joint method](SIMULATOR.md#joint-integration-candidate) integrates mechanics, winding currents, command filtering and work together, then resolves the passive Hall filter along the dense trajectory. The [qualification record](../results/physics_verification/joint_passive_filter/qualification.json) includes independent temporal/energy references, current extinction, sensor timing and exact restart with pending samples. It replaces the affected partitioned-method checks without relaxing acceptance limits. The controlled hover and local movement below use this method.

## Current airborne interface

| Retained trial | Measured outcome | Scope |
|---|---|---|
| [Direct-field hover](../results/hover/joint_control/summary.json) | 50 ms; all operating/energy limits pass; 49 moving scans; 21.92 h runtime | Direct comparison reference |
| [Accelerated hover comparison](../results/hover/joint_control/atlas_backend/hover/comparison.json) | Same 50 ms; all operating and comparison checks pass; 4.69 h runtime, 4.68× complete-run acceleration | Verified acceleration of this nominal hover |
| [Native hover comparison](../results/hover/native_spline/qualified/trial/comparison.json) | Same 50 ms; all operating, energy, trajectory and analogue checks pass; 61.3 min runtime, 4.59× faster than the SciPy atlas run | Verified native runtime for this nominal hover |
| [Local movement](../results/local_motion/trial/summary.json) | 60 ms including a 0.3 mm sideways move and 0.01 rad yaw; all operating/energy limits pass; 59 moving scans; 6.18 h runtime | First short combined commanded movement |

The local movement reaches peak physical tracking errors of 67.72 µm and 3.067 mrad, peak current 2.431 A, voltage 1.352 V and instantaneous copper loss 8.347 W. Its accumulated absolute total-energy discrepancy is 0.600 nJ against 22.29 nJ. The [tracking plot](../results/local_motion/trial/tracking.png) shows overshoot within limits; endpoint error and velocity remain nonzero, so the short hold does not establish settling. The [multiview replay](../results/local_motion/trial/motion.mp4) uses saved poses with labeled slowdown and no motion exaggeration.

The movement's [field cache verification](../results/local_motion/fields/summary.json), stationary-prefix comparison and [saved-evidence verification](../results/local_motion/trial/verification.json) retain field/force/gradient checks, report/checkpoint agreement, source/archive identity and exact dense current/filter reconstruction. Exact executed sources and raw scans remain with the trial. No noise, control gain, hardware rate or acceptance limit was changed for this movement.

The local controller currently holds a fixed coil/sensor window. Stationary Hall acquisition precedes release, winding currents start at zero, and all subsequent support comes from modeled electromagnetic forces. Sustained control, endpoint settling, large rotations/tilts, handoffs, takeoff/landing, neighbours and full-board movement remain unqualified. The [legacy stacked reference](QUALIFICATION.md) describes a different geometry and must not supply current square-coil hardware ratings.

## Retained diagnostics

Keep numerical failures that explain current safeguards or unresolved limitations. The [force-budget diagnosis](../results/physics_verification/force_budget/summary.json) motivated curved CAD quadrature; the [variable-coupling failure](../results/physics_verification/variable_coupling/summary.json) exposed accumulated work error; the [simultaneous-extinction diagnosis](../results/physics_verification/joint_passive_filter/simultaneous_extinction_diagnosis.json) exposed a missed switching event. The [error-growth analysis](../results/physics_verification/error_growth/summary.json) distinguishes physical open-loop instability from numerical error. Their original sources and outcomes remain unchanged.

Older prescribed-input handoff and fixed/adaptive comparisons remain mechanism evidence under their recorded configurations. They are not all prerequisites for reading the current plan, and they must not be relabeled as current flight qualification. Reproducible obsolete review exports and historical prose are removed; Git retains their history.

## Next performance work

The [native hover comparison](../results/hover/native_spline/qualified/trial/comparison.json) passes the complete retained 50 ms horizon in **3,675 s (61.3 min)**, versus 16,877 s (4.69 h) for the SciPy-atlas reference: **4.59× measured runtime improvement**. The candidate uses four persistent native query workers and one BLAS thread. Both runs use 2,701 accepted steps, 24,094 physical RHS evaluations and 35,428 passive-filter evaluations, delivering 49 moving Hall scans. The gain is implementation performance, not fewer samples or looser solver settings. Timings are observed on this machine, not guaranteed across machines or workloads; the candidate trial timer excludes setup/cache preparation.

All operating, energy, trajectory, peak and raw analogue-scan comparisons pass. Maximum physical position/current differences are 3.31e-17 m and 9.10e-15 A; estimated position/orientation differences are 3.13e-11 m and 1.61e-9 rad. The largest raw analogue difference is 1.16e-14 T against the unchanged 1e-7 T limit. Accumulated absolute electrical–mechanical energy defect is 1.74e-9 J against 2.23e-8 J. Recorded data and dense trajectories support replay without rerunning magnetics. This qualifies the implementation change for this nominal hover, not all movement, disturbances or hardware.

The explicit [native runtime](../magnetic_spline.py) evaluates the same tensor-product spline representation, preserving grid, knots, order, conductor samples and summation. It retains the SciPy evaluator as a reference and records native source/binary identity, worker count, BLAS libraries/thread policy and dependency identity in checkpoints. Native evaluation must be explicitly active when creating or restoring its simulation. Leaving the runtime context restores SciPy and closes workers; the configured simulation rejects a later mismatched evaluator. Numerical settings remain in `Fixed`. No fast-math flags or new environment packages are required. Thirty-five focused atlas, native, joint and adaptive tests pass, including parallel equivalence/failure propagation, exact restart and changed runtime identity rejection.

The [preflight](../results/hover/native_spline/qualified/preflight.json) creates a new cache under current sources rather than bypassing stale-cache guards. Its grid matches the retained cache exactly, but coefficient roundoff differs by up to 7.23e-13 because of the historical magnet-placement sequence. The initial exact-rebuild rejection is preserved in [cache evidence](../results/hover/native_spline/cache_roundoff/preflight.json). The fresh coefficients pass the existing numerical-equivalence tolerances; sampled field and actuator-matrix differences are 4.82e-15 T and 3.12e-16. The actual initial physical, sensor and random-generator state matches the saved hover exactly. Old provenance is unchanged; both the rejected preparation and accepted run retain their executed sources. Existing diagnostic snapshots remain non-resumable as production checkpoints.

Supporting [field/force/gradient screening](../results/local_motion/spline_evaluation/summary.json), [short coupled comparison](../results/local_motion/spline_evaluation/coupled/summary.json), [startup profile](../results/local_motion/spline_evaluation/profile/summary.json) and [worker-policy screen](../results/local_motion/spline_evaluation/worker_policy/summary.json) isolate the implementation changes. The field screen covers recorded and domain poses; the coupled screen also checks tighter integration. The startup profile distinguishes already-cached CAD quadrature work from repeated work. Worker screening shows that limiting competing BLAS pools helps native queries, and that removing inexpensive validation guards is not justified. These narrower timings do not replace the complete-run result above.

A linear extrapolation of the hover result suggests about ten hours per simulated half-second, so further acceleration is now the priority before the longer campaign. Movement can change work; this is not a timing guarantee. The retained short native profile points to force evaluation/assembly, rigid magnet placement and direct Hall evaluation. It is not a full-run attribution, and nested/threaded profile times must not be added as independent percentages.

The next implementation screen should isolate repeated object manipulation and array preparation. `hall_field` already transforms observation points into the magnet frame but resets the magnet collection on every call; its immutable geometry is a candidate for reuse. Magpylib's [functional/core interfaces](https://magpylib.readthedocs.io/en/stable/_pages/user_guide/docs/docs_fieldcomp.html#functional-interface) expose vectorized analytical field evaluation without the full object interface. Compare this direct sensing path on recorded and rotated poses before changing it. Force/torque assembly and rigid transforms are candidates for additional compiled work only if measured costs justify it. The expensive spline evaluator is already C, so a blanket C++ rewrite is not the proposed approach. Retain geometry/domain validation, physical sensor noise and numerical limits.

The larger memory-for-computation candidate is a unit-current field cache for a stationary coil, reused by translating/rotating identical coil geometry. Evaluate its interaction with the finite magnets using independently refined volume or surface integration, rather than repeatedly sampling the magnet field along every wire. This is a research candidate, not a qualified replacement: setup cost, integration resolution, derivatives, return conductors, arbitrary orientations, COM torque and back-EMF consistency must be established. The [virtual-work formulation](https://doc.comsol.com/6.3/doc/com.comsol.help.acdc/acdc_ug_theory.05.78.html) and existing independent flux/work checks provide verification routes. It must preserve finite magnet dimensions; a point-dipole substitution is not implied. Use an explicit memory budget in the model if prototyping proceeds. The current spline coefficients occupy about 213 MB; the inspected environment has ample remaining RAM for a bounded experiment, but a dense table covering every translation and rotation grows rapidly. Exact caching of previous poses has unmeasured hit rates and is not assumed useful.

Screen these candidates using recorded geometry/poses, representative rotations and existing domain boundaries before expensive coupled runs. Estimate complete-loop benefit, memory and cache-build amortization across the campaign; do not promise a speed multiplier from kernel timing alone. Any new spatial approximation needs force, torque, derivative, work and sensing checks plus bounded coupled comparisons. Keep the currently qualified native evaluator as the reference, and carry a selected improvement into the retained movement comparison before extending flight.

## Remaining physical uncertainties

| Area | Modeled now | Still needs evidence |
|---|---|---|
| Sensors | Sequential sampling, filter response, noise, quantization, calibration and measured-current correction | Actual parts, settling/jitter, effective ADC accuracy, reference/supply rejection, transport and computation delay |
| Drivers | Sampled regulator, averaged RL response, current/command quantization and voltage limits | Self/mutual inductance, PWM interference/aliasing, switching and return-path effects, rail dynamics |
| Construction and environment | Nominal CAD king, winding and local patch | Magnet and mass variation, sensor alignment, temperature drift, neighbouring magnets, external fields and vibration |
| Numerical approximation | Independent references, bounded refinement, runtime guards and energy accounting | Application-horizon uncertainty and any new mechanisms or domains |

The coil electrical screen still exceeds the ripple allowance under its low-inductance assumption; averaged-flight results do not remove this failure. Resolve or bound switching/current-readout effects before adopting hardware targets. Use component and circuit evidence to define disturbances; arbitrary white noise is not a model of every interference source. Existing noise assumptions are provisional, including the typical-noise specification in the [DRV5055 datasheet](https://www.ti.com/lit/ds/symlink/drv5055.pdf).

## Full engineering scope

| Area | Questions to resolve | Initial approach and escalation |
|---|---|---|
| Coil construction and packaging | Insulated winding fit, bend radii, transitions, terminals, neighbouring coils and layer placement | build123d continuous conductor and assembly geometry; published wire/process constraints |
| Piece geometry and mass | Magnet positions, mass, centre of mass, inertia and clearances | Current CAD-derived mass, centre of mass and inertia with explicit material/construction uncertainty |
| Magnetic fields, forces and torques | Lift and six-axis authority across positions, orientations and board edges | Existing Magpylib-based calculations; electromagnetic FEM checks at selected difficult configurations |
| Coil electrical behaviour | Resistance, self/mutual inductance, current distribution and frequency-dependent losses | Existing estimates with uncertainty bounds; targeted Elmer electromagnetic analysis where uncertainty affects driver or thermal design |
| Motion and control | Hover recovery, complete lift/fly/land, tilt/yaw, delays, disturbances and coil handoffs | MuJoCo rigid-body dynamics coupled to the existing controller, virtual sensor readings and electrical/magnetic models |
| Neighbours and contact reset | Magnetic interference, shared-coil constraints, friction, tipping and collision-free routing | Existing magnetic calculations and trajectory planning with MuJoCo contact/friction; small groups before full-set reset |
| Temperature distribution | Winding, magnet, electronics and touch-surface hot spots over time | Elmer thermal FEM of a representative assembly; improve the existing workload/thermal model from the results |
| Cooling | Heat rejection under the actual enclosure and quiet operation | Existing heat balance and bounded cooling/contact assumptions; detailed airflow simulation only for a specific unresolved cooling decision |
| Pose sensing | Accuracy, delay, coil-field contamination, neighbour interference and calibration | Existing field/sensing model, real sensor specifications and simulated interference/delay |
| Current drivers | Tracking, ripple, losses, voltage overshoot, component stress and current measurement | KiCad/ngspice circuit simulation of a representative channel |
| Power distribution | Supply size, voltage drop, regenerative absorption, midpoint balance and startup | Existing load/energy accounting plus selected circuit simulations |
| PCB implementation | Current paths, grounding, clearances, sensor interference and mechanical fit | KiCad electrical/layout checks, electrical calculations and signal-integrity checks; STEP assembly exchange |
| Processing and communication | Acquisition, computation and command deadlines | Existing timing budgets, algorithm costs and candidate controller specifications |
| Deformation, vibration and sound | Gap changes from bending/expansion, mechanical resonance and audible operation | CAD and simple calculations; structural/acoustic analysis where necessary |
| Faults and variation | Sensor/channel failure, power interruption, lifted pieces and manufacturing spread | Existing-model scenario/parameter sweeps, with modeled protection and recovery behavior |
| Manufacturing and cost | Purchasable parts, custom parts, quantities, tolerances and assembly costs | KiCad electronics BOM plus mechanical/custom-part records and supplier quotations |

Build123d/FreeCAD, MuJoCo and the existing Python/Magpylib calculations are integrated. Elmer/Gmsh and KiCad/ngspice remain planned tools, introduced only for a specific engineering question. Reuse the existing model before adding another solver; verify material assumptions, boundary conditions and numerical convergence for each new analysis. CAD defines geometry, not magnetic or thermal correctness.

## Thermal scope

The current thermal model uses simplified heat paths and thermal masses. It estimates temperatures and endurance but does not resolve the detailed spatial distribution through the coil assembly.

Begin with windings, insulation/potting, playing surface, PCB, thermal interface and baseplate. Include relevant magnet/piece contact or air-gap conditions for the chosen operating cases. Use a winding-pack approximation with effective material properties initially; resolve individual layers only when the question requires it. Bound uncertain contact resistance and cooling conditions instead of treating them as known facts. Check numerical convergence and sensitivity to uncertain material, contact and cooling properties. Physical comparison belongs to the later validation stage.

This work must explain or resolve the existing thermal failures. It must not replace them with a passing result through unsupported cooling assumptions, relaxed limits or omitted demanding workloads.

## Real components and BOM

Associate each selected electronic part with its manufacturer and exact part number, supplier reference, datasheet, footprint and, where available, a compatible simulation model. KiCad can export the selected component fields and quantities. Combine that electronics BOM with custom coils, magnets, mechanical parts, wiring, assembly and current supplier quotations. [KiCad BOM export](https://docs.kicad.org/9.0/en/eeschema/eeschema.html#generating-a-bill-of-materials).

A purchasable part, schematic symbol, footprint and validated simulation model are different assets. Check pin mapping, package dimensions, model compatibility and modeled operating conditions. Prices and stock require separate sourcing checks. Preserve the current model BOM as a labeled allowance until actual selections replace it; do not count the allowance and selected parts twice.

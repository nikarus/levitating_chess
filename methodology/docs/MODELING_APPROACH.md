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
| [Local movement](../results/local_motion/trial/summary.json) | 60 ms including a 0.3 mm sideways move and 0.01 rad yaw; all operating/energy limits pass; 59 moving scans; 6.18 h runtime | First short combined commanded movement |

The local movement reaches peak physical tracking errors of 67.72 µm and 3.067 mrad, peak current 2.431 A, voltage 1.352 V and instantaneous copper loss 8.347 W. Its accumulated absolute total-energy discrepancy is 0.600 nJ against 22.29 nJ. The [tracking plot](../results/local_motion/trial/tracking.png) shows overshoot within limits; endpoint error and velocity remain nonzero, so the short hold does not establish settling. The [multiview replay](../results/local_motion/trial/motion.mp4) uses saved poses with labeled slowdown and no motion exaggeration.

The movement's [field cache verification](../results/local_motion/fields/summary.json), stationary-prefix comparison and [saved-evidence verification](../results/local_motion/trial/verification.json) retain field/force/gradient checks, report/checkpoint agreement, source/archive identity and exact dense current/filter reconstruction. Exact executed sources and raw scans remain with the trial. No noise, control gain, hardware rate or acceptance limit was changed for this movement.

The local controller currently holds a fixed coil/sensor window. Stationary Hall acquisition precedes release, winding currents start at zero, and all subsequent support comes from modeled electromagnetic forces. Sustained control, endpoint settling, large rotations/tilts, handoffs, takeoff/landing, neighbours and full-board movement remain unqualified. The [legacy stacked reference](QUALIFICATION.md) describes a different geometry and must not supply current square-coil hardware ratings.

## Retained diagnostics

Keep numerical failures that explain current safeguards or unresolved limitations. The [force-budget diagnosis](../results/physics_verification/force_budget/summary.json) motivated curved CAD quadrature; the [variable-coupling failure](../results/physics_verification/variable_coupling/summary.json) exposed accumulated work error; the [simultaneous-extinction diagnosis](../results/physics_verification/joint_passive_filter/simultaneous_extinction_diagnosis.json) exposed a missed switching event. The [error-growth analysis](../results/physics_verification/error_growth/summary.json) distinguishes physical open-loop instability from numerical error. Their original sources and outcomes remain unchanged.

Older prescribed-input handoff and fixed/adaptive comparisons remain mechanism evidence under their recorded configurations. They are not all prerequisites for reading the current plan, and they must not be relabeled as current flight qualification. Reproducible obsolete review exports and historical prose are removed; Git retains their history.

## Next performance work

The accelerated hover and first local movement imply approximately 47–52 wall-clock hours per simulated half-second on this machine, or about 470–520 hours for ten such serial cases. These are linear workload extrapolations from short runs, not runtime guarantees; startup, motion, solver work and machine load can change the cost. Optimize before launching the longer single-piece campaign.

The retained startup profile places 83.6% of measured runtime inside the compiled spline evaluator and 92.5% inside the encompassing force-matrix calculation; these percentages overlap. Each force matrix still queries approximately 228,000 wire points. The latest moving trial uses 30,496 physical RHS evaluations. Existing thread and direct-field batch comparisons do not support simply increasing worker counts or batch sizes. Confirm representative moving-pose costs with the existing benchmark owner before choosing an implementation.

First investigate a specialized evaluation of the same three-dimensional quintic magnetic spline, potentially combining interpolation with force/torque accumulation to reduce generic evaluation work and temporary arrays. [SciPy's current representation](https://docs.scipy.org/doc/scipy-1.17.0/reference/generated/scipy.interpolate.NdBSpline.html) is a tensor-product spline; its measured hot path is already compiled, so merely compiling surrounding Python is not a demonstrated solution. Preserve coefficients, boundary behavior, wire samples and numerical budgets. Reuse saved geometry/poses, then compare fields, gradients, force, torque and back-EMF consistency, followed by a short coupled run and complete-run timing. No additional speedup is established yet.

If that is insufficient, assess whether error-controlled winding integration can reduce field queries while meeting the same spatial-error budget. A larger alternative is to precompute integrated coil response rather than only pointwise magnetic field. Such a response depends on relative position and orientation; general flight cannot use a hover-only map, and a naive full pose grid can be too large. These are separate approximation/representation changes requiring domain, memory, force/torque, work consistency and sensor-feedback verification. Keep the detailed evaluator as the reference. Do not adopt coarser time steps, fewer wire points or a new map simply because the resulting trajectory appears stable.

Use cheap path screening, recorded-data diagnosis and compatible initial-state reuse to avoid redundant expensive work. They reduce campaign cost without constituting physical qualification. Parallel independent cases may improve throughput only if resource measurements support it; they do not by themselves make an individual trajectory faster. GPU work or hardware purchases are not the first step without a suitable measured workload and implementation.

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

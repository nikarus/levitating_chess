# Modeling tools and scope

Proposal recorded after the CAD and simulation discussion on 2026-10-08. Build123d and FreeCAD are now integrated for the first geometry reference; see [cad/README.md](cad/README.md). The other tools below remain proposed. The work sequence is maintained in [NEXT_STEPS.md](NEXT_STEPS.md).

## Recommended approach

Keep the existing Python model as the central system simulation. Add programmable geometry with build123d and spatial thermal analysis with Elmer FEM. Use detailed electromagnetic analysis for specific uncertainties, and KiCad/ngspice for the actual electronics. Start with a representative coil assembly and measurements before scaling to the board.

Build123d supplies geometry, not magnetic, thermal or motion physics. Our existing code already calculates magnetic forces, feedback-controlled motion and simplified heating. The principal immediate addition is a spatial thermal model; replacing the existing magnetic and motion calculations is not justified merely because a more general solver is available.

Prefer the least complex model that answers a design decision. Introduce a new tool or more detail when it resolves a specific uncertainty. Detailed simulation does not establish unknown material properties, supplier capability or actual cooling conditions; measurements remain necessary.

## Full engineering scope

| Area | Questions to resolve | Initial approach and escalation |
|---|---|---|
| Coil construction and packaging | Insulated winding fit, bend radii, transitions, terminals, neighbouring coils and layer placement | build123d geometry; supplier/process review and physical samples |
| Piece geometry and mass | Magnet positions, mass, centre of mass, inertia and clearances | Existing bottom-heavy king approximation; improved CAD/material properties and measurements as the piece design develops |
| Magnetic fields, forces and torques | Lift and six-axis authority across positions, orientations and board edges | Existing Magpylib-based calculations; electromagnetic FEM checks at selected difficult configurations |
| Coil electrical behaviour | Resistance, self/mutual inductance, current distribution and frequency-dependent losses | Measurements and existing estimates; targeted Elmer electromagnetic analysis where uncertainty affects driver or thermal design |
| Motion and control | Hover recovery, complete lift/fly/land, tilt/yaw, delays, disturbances and coil handoffs | Extend the existing dynamic simulation with verified geometry and electrical behaviour |
| Neighbours and contact reset | Magnetic interference, shared-coil constraints, friction, tipping and collision-free routing | Existing magnetic/contact calculations and trajectory planning; validate small groups before full-set reset |
| Temperature distribution | Winding, magnet, electronics and touch-surface hot spots over time | Elmer thermal FEM of a representative assembly; improve the existing workload/thermal model from the results |
| Cooling | Heat rejection under the actual enclosure and quiet operation | Existing heat balance and measured cooling; detailed airflow simulation only for a specific unresolved cooling decision |
| Pose sensing | Accuracy, delay, coil-field contamination, neighbour interference and calibration | Existing field/sensing model, real sensor specifications and prototype measurements |
| Current drivers | Tracking, ripple, losses, voltage overshoot, component stress and current measurement | KiCad/ngspice circuit work and measurements on a representative channel |
| Power distribution | Supply size, voltage drop, regenerative absorption, midpoint balance and startup | Existing load/energy accounting plus selected circuit simulations |
| PCB implementation | Current paths, grounding, clearances, sensor interference and mechanical fit | KiCad electrical/layout checks, calculations and prototype measurements; STEP assembly exchange |
| Processing and communication | Acquisition, computation and command deadlines | Existing timing budgets and benchmarks on candidate controller hardware |
| Deformation, vibration and sound | Gap changes from bending/expansion, mechanical resonance and audible operation | CAD, simple calculations and measurements; structural analysis only where necessary |
| Faults and variation | Sensor/channel failure, power interruption, lifted pieces and manufacturing spread | Existing-model scenario/parameter sweeps, followed by physical protection and recovery checks |
| Manufacturing and cost | Purchasable parts, custom parts, quantities, tolerances and assembly costs | KiCad electronics BOM plus mechanical/custom-part records and supplier quotations |

## Tool roles and automation

- **build123d:** Python-defined geometry, assembly checks and manufacturing exports. CadQuery is a suitable alternative, but adopt one geometry library. FreeCAD can provide visual inspection; generated STEP geometry does not replace the authoritative Python design history. [build123d](https://build123d.readthedocs.io/en/latest/), [exports](https://build123d.readthedocs.io/en/latest/import_export.html), [CadQuery](https://cadquery.readthedocs.io/en/latest/intro.html).
- **Existing Python/Magpylib model:** central force, allocation, motion and workload calculations. Analytical magnetic solutions suit repeated evaluations, but their assumptions about current and magnetization distributions must be respected. AC current redistribution and material effects justify targeted additional analysis. [Magpylib physics and limitations](https://magpylib.readthedocs.io/en/stable/_pages/user_guide/guide_resources_01_physics.html).
- **Elmer FEM with Gmsh:** spatial heat transfer and selected electromagnetic or structural problems. Gmsh prepares the mesh and exposes a Python API; Elmer supplies the physics solvers. Scripts can generate solver inputs and collect results, but the project workflow needs a small integration check before expansion. Verify materials, boundary conditions and mesh convergence. [Elmer capabilities](https://github.com/ElmerCSC/elmerfem), [Gmsh](https://gmsh.info/).
- **KiCad with ngspice:** actual circuit design, circuit simulation, PCB layout and electronics BOM. Use available APIs, command-line operations and documented file formats; verify automation coverage for each task. Do not assume the entire schematic/layout workflow is supported by one API. [KiCad documentation](https://docs.kicad.org/9.0/en/eeschema/eeschema.html), [automation coverage](https://dev-docs.kicad.org/en/apis-and-binding/ipc-api/for-addon-developers/).
- **COMSOL Multiphysics:** an integrated commercial alternative for coupled physics, with programmatic control through its Java API. Consider it if suitable licenses are available and integration effort justifies a change; it is not part of the initial proposed workflow. [COMSOL API](https://www.comsol.com/support/learning-center/article/overview-of-the-comsol-api-107912).

## Connect the models without a large simulation framework

1. Consume the same authoritative dimensions and material settings when constructing geometry and simulation inputs. Reuse existing geometry calculations rather than recreating them in CAD scripts.
2. Use magnetic/control calculations to determine forces, per-coil currents and losses for representative operating cases.
3. Apply those losses to the thermal assembly to obtain spatial temperatures and transient response.
4. Feed temperature-dependent resistance and magnet strength back into the operating calculations when those changes are significant.
5. Reuse verified maps or compact coefficients in the faster system simulation. A full FEM solve at every controller update is not the intended architecture.

CAD assemblies, magnetic-field plots, motion animations and temperature maps can provide visual review of the same design. Visualization does not provide additional validation of the underlying physics.

All new quantitative settings remain in `Inputs`, `Fixed` or `Constants` in [model.py](../model.py), with parameter comments. CAD and solver inputs consume those values. External manufacturer simulation models remain identifiable source assets; adopted design assumptions must be traceable. Proposed CAD/solver work belongs under `methodology/`, with generated artifacts apart from source definitions. Avoid additional root files and duplicate parameter tables.

## Thermal scope

The current thermal model uses simplified heat paths and thermal masses. It estimates temperatures and endurance but does not resolve the detailed spatial distribution through the coil assembly.

Begin with windings, insulation/potting, playing surface, PCB, thermal interface and baseplate. Include relevant magnet/piece contact or air-gap conditions for the chosen operating cases. Use a winding-pack approximation with effective material properties initially; resolve individual layers only when the question requires it. Bound uncertain contact resistance and cooling conditions instead of treating them as known facts. Check numerical convergence and compare predicted temperatures with measurements.

This work must explain or resolve the existing thermal failures. It must not replace them with a passing result through unsupported cooling assumptions, relaxed limits or omitted demanding workloads.

## Real components and BOM

Associate each selected electronic part with its manufacturer and exact part number, supplier reference, datasheet, footprint and, where available, a compatible simulation model. KiCad can export the selected component fields and quantities. Combine that electronics BOM with custom coils, magnets, mechanical parts, wiring, assembly and current supplier quotations. [KiCad BOM export](https://docs.kicad.org/9.0/en/eeschema/eeschema.html#generating-a-bill-of-materials).

A purchasable part, schematic symbol, footprint and validated simulation model are different assets. Check pin mapping, package dimensions, model compatibility and modeled operating conditions. Prices and stock require separate sourcing checks. Preserve the current model BOM as a labeled allowance until actual selections replace it; do not count the allowance and selected parts twice.

# Coils: current virtual design

The single-layer square array is the current candidate for integrated CAD and simulation. It is not yet an adopted replacement for the stacked hardware targets. Quantitative settings belong in `Inputs` and `Fixed` in [model.py](../../model.py). Manufacturing and physical tests follow the completed virtual design review; the work sequence is in [NEXT_STEPS.md](../NEXT_STEPS.md).

## Driver-count investigation

This investigation uses a separate checkout from the flight work. The current square CAD and production simulator settings remain the reference; no new arrangement is adopted. [Study origin](../results/channel_layouts/origin.json) identifies the source checkout and inherited uncommitted files. Quantitative candidate settings live in `Inputs.channel_layout_candidates` and `Fixed`; the existing geometry and constrained current-allocation implementations are reused.

The [current-window pilot](../results/channel_layouts/rated_pilot/summary.json) and [wider-support pilot](../results/channel_layouts/wider_support/summary.json) use the same round wire, rounded closed-turn field approximation and CAD-derived king mass/inertia for every candidate. Each samples eight phase points, a level king at the upper flight gap and one yaw; demands include gravity, lift reserve, horizontal acceleration and separate signed rotation torques. These are static authority screens, not closed-loop trials. They do not reproduce the production continuous conductor, and have no spatial/field refinement, tilted-pose, neighbour, sensing, transient-voltage or thermal qualification.

| Arrangement | Installed channel-density change | Peak required current, wider window | Peak allocated copper loss, wider window |
|---|---:|---:|---:|
| Existing square geometry | reference | 1.73 A | 14.12 W |
| Existing squares, wider spacing | −24.9% | 2.74 A | 28.15 W |
| 10 mm squares | −42.7% | 2.39 A | 26.77 W |
| 12.5 mm squares | −62.9% | 12.38 A; fails current budget | not evaluated |
| Small rectangular parquet | +8.3% | 1.88 A | 18.33 W |
| Larger rectangular parquet | −50.0% | 2.97 A | 48.18 W |
| Adjacent square pairs, same circulation | −50.0% | 4.32 A | 165.63 W |
| Adjacent square pairs, opposed circulation | −50.0% | 9.83 A; fails current budget | not evaluated |

The current budget is unchanged, including its reserve. Peak required current minimizes the maximum branch current; copper loss is separately minimized within that budget and its peak may occur at a different sampled demand. These losses are instantaneous local demand results, not workload averages or temperature predictions. Channel density excludes board edges and is not a BOM percentage. All wider-window cases have full sampled six-axis plus midpoint-current rank; passing rank alone does not establish practical authority.

The existing 45 mm active window leaves some reduced-channel arrangements with only six complete branches, insufficient for arbitrary six-axis control plus midpoint balance. A 60 mm window removes that particular deficiency. It also brings more distant coils into play: neighbour disturbance and simultaneous-reset feasibility must be checked before claiming a usable saving. The 10 mm square changes from failing the sampled current budget in the smaller window to passing it in the wider window. Enlarging the support window does not increase installed density, but may increase the number of simultaneously energized channels.

Paired square windings form fixed adjacent dominoes, horizontal and vertical in alternating two-by-two tiles (basket weave, distinct from physical herringbone rectangles). Each series pair has one driver; opposite circulation reverses the second winding. The allocator balances driver branch currents at the midpoint, not signed winding currents. Incomplete pairs at the selection boundary stay off. Series mutual inductance, routing and actual circuit realization remain unqualified.

The first pilot stopped during minimum-loss optimization outside the retained driver budget; its [failure log](../results/channel_layouts/pilot/failure.log) and source snapshot are preserved. The current screen reports excessive required current and does not optimize an out-of-budget power stage. The broader comparison below supersedes the initial favourable assessment of the 10 mm square. Wider spacing remains a candidate for refinement; larger parquet is secondary, while the tested fixed pairs have a substantial loss penalty. These results do not rule out other sizes, orientations or pair patterns.

### Broader square-coil comparison

The [pose comparison](../results/channel_layouts/pose_comparison/summary.json) evaluates both square sizes with a 60 mm support window. Each has 1,152 initial pose evaluations: eight phases, two flight heights, eight yaw angles over a full turn, and level plus eight rim-clearance-limited tilt directions. Dense phase slices and local angular/intermediate-height checks bring the total to 1,316 evaluations per design. These include repeated poses and do not exhaust the continuous domain. Turning requests use the CAD inertia rotated into world coordinates; the piece has zero angular velocity in these static calculations.

| Geometry | Largest sampled minimum peak current | Largest feasible-demand copper loss | Reserved-current screen |
|---|---:|---:|---|
| Current 7.5 mm squares | 2.289 A | 29.66 W | Meets the budget at all sampled poses |
| 10 mm squares | 5.203 A | 324.60 W | Exceeds the 4.95 A budget in some demands |

Every sampled pose retains full six-axis plus midpoint-current rank. The larger coils' problem is inefficient force/torque production in particular poses, not a complete loss of mathematical controllability. Their required current remains below the provisional 5.5 A channel rating, but fails the retained reserve. Losses are separately optimized under the reserved budget; the table excludes infeasible demands, which stay flagged in the record. Even level-pose rotation raises the larger coils' sampled peak loss to 118.77 W, compared with 18.15 W for the current squares. The much smaller pilot sampled only one yaw and missed this dependence.

Dense phase refinement raises the reference's largest loss by about 1.6% and does not raise the larger coil's sampled maximum. The additional local angular/height samples do not increase either global maximum. Finer winding discretization at the current- and loss-critical poses changes individual current demands by at most 0.72% and feasible losses by at most 3.96%, within the existing 5% screening tolerance; the budget classifications persist. At the larger coil's loss-critical pose, the refined peak is 312.24 W. These are local refinement checks, not a global error guarantee or the production simulator's tighter field-verification gate.

This completes the first broader static comparison, in about 46 minutes. The 10 mm design is not selected for a coupled trial: its nominal channel-density saving is offset by a substantial pose-dependent loss penalty and inadequate current reserve. A less aggressive change in spacing or coil size should be screened before neighbour qualification. All existing limitations on continuous-conductor geometry, material uncertainty, thermal behaviour, contact, sensing and dynamics still apply.

### Compact screening and layout parameters

The [hard-pose collection](../results/channel_layouts/hard_poses.json) selects the worst-current and worst-feasible-loss poses from each design in the broader comparison, both over all recorded poses and over level poses separately. Identical poses are merged, retaining every selection reason. The file records the source report hash and the exact candidate, stage, row and demanding action for each origin. It is derived evidence, not a new parameter source or acceptance standard. Rebuild it with `methodology.coil_layouts --collect-hard-poses <source-report> --output <fresh-json-path>`; no magnetic calculation is needed for selection.

At each selected pose, a candidate must be evaluated against all existing force/torque demands, not just the action that originally made that pose difficult. Positions are fractions of the candidate's repeating electrical cell in lattice coordinates; physical coordinates therefore change with pitch. Yaw and tilt-axis azimuth stay in world coordinates. Independent squares repeat at one pitch, parquet at its physical repeat, and the alternating paired-square wiring at its electrical repeat. A new geometry can have weaknesses elsewhere: this collection is a shared screening seed, not proof of continuous-domain coverage. Search finalists from multiple starting points for new weaknesses, add those cases to the common collection, and retain broader verification for the final shortlist.

Use parameterized families rather than a separate implementation for every candidate. Keep three choices distinct: winding shape and dimensions; spatial arrangement and spacing; electrical grouping. Start with the implemented rounded squares and two-to-one rectangular parquet, varying dimensions and spacing while holding wire, turn packing and winding height fixed for a fair first comparison. Orientation and local grouping can then be varied within their supported families. The implemented sweep reads its sizes, spacings and square connection modes from `Inputs.channel_sweep_*`, reuses `Fixed.layout_screen_rotations_deg`, and evaluates the finite grid against the retained collection. `prototype_coils.round_wire_geometry` owns construction for both named and parameterized windings. The same pose solver handles independent coils and complete adjacent pairs, including their combined field and branch resistance. Each candidate stops at the first rank or reserved-current failure; failed-prefix maxima are not compared as full-screen maxima. All surviving candidates receive the entire collection. The report retains the nondominated trade-offs in channel density, required current and feasible-demand peak loss without inventing a thermal or cost acceptance threshold. Run `methodology.coil_layouts --sweep-channels <pose-collection> --output <fresh-directory>`; the report and source snapshot share that result directory. The initial grid can be refined around promising regions. Quantitative search settings belong in `Inputs`/`Fixed`, using the existing geometry owners. Arbitrary rectangle aspect ratios, racetrack/oval shapes and other lattice families require explicit placement and non-overlap support before inclusion; they are not current implemented candidates. No general-purpose geometry optimizer is needed at this stage.

### Parameter sweep results

The [parameter sweep](../results/channel_layouts/parameter_sweep/summary.json) screened 114 supported size/spacing/orientation/connection combinations in 440 seconds with three workers. Early rejection reduced the work to 381 pose evaluations: 37 designs completed the seven-pose screen, 59 failed the unchanged reserved-current budget and 18 failed augmented control rank. All winding envelopes passed fit/non-overlap checks. Passing the seed screen is not geometry qualification.

For comparisons at the current square lattice orientation and nominal spacing:

| Square winding / connection | Fewer installed channels per area | Peak required current | Peak feasible copper loss |
|---|---:|---:|---:|
| Current 7.5 mm independent | reference | 2.289 A | 29.66 W |
| 8.25 mm independent | 16.8% | 2.596 A | 40.94 W |
| 9 mm independent | 29.7% | 3.023 A | 66.59 W |
| 7.5 mm same-circulation pairs | 50.0% | 3.821 A | 80.88 W |

These are complete seed-screen maxima, not temperature or BOM predictions. The baseline reproduces the broader comparison's recorded maxima. Larger savings exist among seed survivors, but have substantial current/loss penalties or depend on orientation coverage. The report's nondominated candidates are only a screening trade-off set, not recommendations.

An [orientation diagnostic](../results/channel_layouts/parameter_sweep/orientation_check.json) demonstrates that limitation directly. The 10 mm square at zero lattice rotation passes the original seven poses, but rigidly rotating the known failing coil/piece alignment to that lattice gives 5.203 A again, above the unchanged 4.95 A budget. Thus one of the 37 seed survivors is already rejected by a targeted follow-up. The diagnostic retains the transformed pose, all recomputed demands, source/input hashes and reproduction code. Relative coil/magnet alignment must be carried across lattice rotations in the next shared challenge set; raw rotation-dependent gains from this sweep are not established improvements.

Prioritize moderate independent-square size changes for the next weakness search. Keep parquet and local pairing as comparison families rather than selecting them from these sparse results. Search phase, height and orientation from multiple starting points, carry known failures into the shared collection, and refine magnetics/continuous wiring only for survivors. Neighbour and simultaneous-reset checks remain necessary before a channel-density reduction becomes a practical cost reduction.

## Winding geometry

The square candidate uses one plane of side-by-side coils. Each coil itself contains multiple axial layers of wire turns. Round wire permits rounded winding corners without the unresolved edgewise forming of the earlier flat-wire candidate.

The round-wire model uses published IEC dimensions: nominal 0.250 mm bare copper, Grade 2 overall diameter up to 0.297 mm, and resistance of 0.3345–0.3628 Ω/m at the reference temperature. These data bound the assumed wire geometry; they do not qualify a finished winding. [Elektrisola technical data](https://www.elektrisola.com/Attachments/TechnicalDataBySize/ELEKTRISOLA_EnCuWire_IECJIS_Datasheet_eng.pdf).

| Calculated geometry | Parquet footprint | Square footprint |
|---|---:|---:|
| Outside winding size | 5 × 10 mm | 7.5 × 7.5 mm |
| Inside window | 2.504 × 7.504 mm | 5.004 × 5.004 mm |
| Inner / outer corner radius | 1.000 / 2.248 mm | 1.000 / 2.248 mm |
| Turns across wall × axial layers | 4 × 14 | 4 × 14 |
| Total turns / winding height | 56 / 4.298 mm | 56 / 4.298 mm |
| Short lattice pitch | 5.3 mm | 7.8 mm |
| Calculated wire length, including allowances | 1.311 m | 1.311 m |
| Reference-temperature resistance range | 0.439–0.476 Ω | 0.439–0.476 Ω |

Packing includes the maximum insulated diameter plus the configured turn-spacing allowance. Coil spacing is a real gap: the parquet uses a doubled long pitch, so its long-direction envelope gap is twice the short-direction gap. The magnetic calculation retains the actual winding sizes rather than increasing them to fill that space.

The resistance screen uses the manufacturer's upper resistance per metre, includes both free leads, and adds a conservative connector-length allowance between turns. The original motion screen uses rounded closed filaments and retains its original resistance allowances. The CAD comparison below uses the continuous conductor, its path-derived resistance and an explicit provisional circuit closure. Physical verification is deferred.

## Integrated virtual assembly

Open [square_assembly.FCStd](../results/cad/square_assembly.FCStd) in FreeCAD. It contains a closely packed 9 × 9 patch, the existing 16-magnet Halbach array, a hollow king shell and board-layer envelopes. One foreground coil is resolved into continuous copper, enamel, leads, terminal pads and provisional solder regions. The other coils remain nominal winding envelopes to keep the assembly manageable. The lattice rotation, nominal winding dimensions and vertical stack match the square screen. This is a representative physical patch, not the full board or the number of channels energized together.

The continuous conductor traverses all turns in the same circulation direction, alternating inward and outward radial progression on successive layers. Layer changes require a local detour: a direct diagonal transition would overlap neighboring enamel. Outer detours extend into part of the inter-coil gap; inner detours enter the winding window. Thus the original rectangular winding envelope no longer bounds every crossover. The actual detailed coil is checked against neighboring envelopes, and its axial extent stays within the original winding height.

Both leads terminate at separate top-side PCB pads inside the winding window. The top lead descends through that window; enamel is removed at the contact lengths. Pad recesses are included in the otherwise unrouted PCB envelope. Solder shapes reserve occupied joint volume, not a qualified fillet or wetting process. The [winding close-up](../results/cad/square_winding.png) isolates this geometry without creating another FreeCAD document.

The path contains approximately 1.255 m of copper including its integral leads, giving 0.420–0.455 Ω at the reference temperature. This excludes PCB traces beyond the pads. The prior 1.311 m / 0.439–0.476 Ω allowance includes longer free leads and approximate connections, so these values have different connection scopes. The minimum screened enamel-to-enamel clearance is approximately 0.0059 mm with maximum insulated diameter. That establishes nominal non-overlap only: winding placement, enamel variation, bend limits and construction tolerances still need a budget. The closed-turn force calculation does not yet include these crossover and lead paths.

The king now has a hollow shell, a roof and perimeter pocket around the unchanged magnet array, adhesive space and a thin bottom film. Underside relief removes unnecessary material around the pocket. The film is not the structural magnet retainer; the assumed load path is the adhesive into the pocket roof and walls. Bond strength, peel, shock, temperature and shell deformation remain unqualified. No inter-magnet gap is introduced.

With the configured provisional densities, the complete modeled king weighs approximately 22.73 g with its centre of mass 14.74 mm above the magnet bottoms. CAD-derived inertia and component contributions are recorded alongside the previous mass/inertia approximation in [square_geometry.json](../results/cad/square_geometry.json). The CAD magnetic comparison uses these properties; the original motion screen retains its earlier approximation. Both tilt inertias and yaw inertia are transferred to simulator state, but prescribed level translation does not exercise rotational dynamics.

Magnet bottoms remain at z = 0. PCB and baseplate depths retain the square motion-screen stack. Thin surface layers, bottom film, adhesive and enamel are hidden initially; the shell is translucent for inspection. Inter-coil potting, detailed circuitry, mounting and cooling remain to be designed. See the [CAD guide](../cad/README.md) for display and regeneration.

## What the motion screen establishes

[prototype_coils.json](../results/prototype_coils.json) records prescribed accelerating motion across one lattice repeat, at the upper flight gap, with the existing hot/heavy king assumptions. Paths cover the configured directions, transverse offsets and held yaw angles. Position is prescribed: this is an actuator/commutation check, not closed-loop levitation or a new minimum-frequency calculation.

Each channel's allowed current falls smoothly to zero as its entire winding approaches the support-window boundary. The minimum-loss allocation retains physical zero-sum current balance with these unequal channel limits. It therefore does not abruptly discard a coil still carrying a commanded current. Current reserve, copper heating, current slew and averaged RL voltage requirements are reported. Required ratings above the available budget are reported as failures; their currents are not silently clipped.

The voltage check includes hot resistance, motional back-EMF and the existing upper self-inductance uncertainty bound. The conservative voltage bound sums their absolute contributions, avoiding reliance on cancellation. The PWM ripple check remains separate. Mutual inductance, sensing, regulator response, delay and current decay still need integration into the virtual electrical/control model.

Denser spatial samples check the most demanding paths; finer magnetic discretization checks local samples around the refined current/voltage extrema. These checks do not establish a global bound over every pose or qualify tilted/yawing motion, neighbours, edges, reset or temperature limits.

## Continuous CAD conductor comparison

[cad_magnetics.json](../results/cad_magnetics.json) records the detailed candidate checks. The adapter builds the conductor centreline and king properties directly from the CAD sources once, then reuses the existing magnetic force solver and current allocator. Every active coil receives the same continuous 56-turn geometry, at the same physical positions as the CAD patch. Solid copper/enamel exports and FreeCAD are not rebuilt for each pose.

The local comparison separates the effects of winding geometry from the king construction. At the former current-critical location, the original model requires 4.463 A, the continuous winding with the original king requires 4.439 A, and the continuous winding with the CAD king requires 3.208 A after magnetic refinement. The lower centre of mass, rather than the small change in wire coupling, accounts for most of the improvement. The existing heavy/hot and COM uncertainty factors remain applied. These are minimum feasible channel ratings at selected poses, not adopted hardware ratings or peak currents in the minimum-loss waveform. That allocation retains the existing 4.95 A cap and reaches approximately 4.914 A in the refined current-critical neighborhood.

A complete 33-position pass along the formerly hardest handoff path (lattice direction 0°, held king yaw 45°) gives 3.217 A minimum feasible rating, 4.923 A actual peak allocated current, 38.96 W peak total winding loss and a 3.018 V averaged voltage bound. This path took approximately two minutes; finer magnetic checks at its newly located current and voltage peaks also pass, with changes below 0.4%. The whole-path sample spacing still needs refinement, and the other path directions and poses have not yet been rerun with this construction. Copper loss is instantaneous active-patch heating, not board-average dissipation. The voltage estimate still uses uncertain self-inductance and omits mutual inductance and PCB resistance.

The wire ends need an electrical return to form a complete magnetic circuit. The screen uses a short local closure at PCB top or bottom as a proxy for paired supply/return routing whose external fields largely cancel. This is not a proposed physical short, a routed PCB, or a bound on every possible return layout. PCB trace resistance remains excluded. Moving this proxy through the PCB thickness changes required current by less than 0.02% at the two checked neighborhoods.

Halving the magnetic segment length changes local required current by less than 0.2%, copper loss by less than 0.4% and averaged voltage bound by less than 0.2%. The initial comparison took approximately three minutes on this laptop, using one calculation process. This validates the usefulness and local numerical resolution of the detailed adapter, not every operating pose. Wire cross-section effects, winding/assembly tolerances, routed return paths, self/mutual inductance, dynamic stability and thermal behavior remain open. In particular, the earlier inductance and PWM estimates remain provisional.

## Results and candidate choice

**Develop the square candidate in CAD first.** It has the better current margin and lower calculated copper loss in this geometry screen. The complete board layout remains to be qualified.

| Maximum observed across the original and refined paths | Parquet | Square |
|---|---:|---:|
| Minimum required channel rating with handoff taper | 4.988 A | 4.463 A |
| Peak total copper loss in the active patch | 189.1 W | 89.0 W |
| Conservative averaged voltage bound | 4.08 V | 3.30 V |
| Estimated peak PWM ripple at the low-inductance bound | 1.36 A | 1.21 A |
| Current reserve against the existing 5.5 A target | Below the intended 10% | Approximately 19% |

The screen covers 36 paths. The three distinct current/voltage-critical paths were repeated with 65 rather than 33 samples. The square path passes the existing 5% sampling-change criterion, including current slew and voltage. Parquet does not: its current slew changes by as much as 81%, and one path exceeds the 4.95 A reserve budget. That path explicitly raises its allocation cap to demonstrate the required current; the resulting loss comparison is not at an identical cap. Parquet's voltage and loss figures therefore remain unconverged observations, not design bounds.

All local magnetic-mesh comparisons pass the 5% criterion: required-current changes stay below 0.8%, copper-loss changes below 2.7%, and voltage-bound changes below 3.9%. At parquet's critical point, the finer mesh gives 4.949 A rather than 4.988 A, straddling the reserve threshold. Its reserve is therefore numerically marginal, not a settled pass or failure. Square retains substantially more margin. These local mesh checks do not resolve parquet's separate path-sampling failure.

The minimum required rating is a feasibility result. The separate minimum-loss allocation can use up to its permitted channel cap (4.95 A for square), so 4.463 A is not the largest current in that optimized waveform. Heating is summed across the active coils and is instantaneous, not one-coil power or full-board average power. The revised winding, physical spacing and tapered handoffs explain why these values differ from the earlier idealized flat-wire comparison.

Both candidates fail the existing 0.5 A ripple allowance at 160 kHz under the approximate low-inductance assumption. The averaged voltage margin does not remove that failure. Refine self/mutual inductance estimates and simulate circuit alternatives across their uncertainty range before adopting a switching topology or PWM frequency; retain the old feedback/control values only as reference assumptions until this geometry has a closed-loop check.

See the [handoff traces](../results/prototype_coils.png) and [machine-readable results](../results/prototype_coils.json). The saved coarse run retains its original source fingerprints; its conservative voltage bounds were subsequently recomputed from saved waveforms, with separate electrical-calculation fingerprints. Refinement and magnetic-mesh checks also record their own source versions.

## Inspect and reproduce

Use one FreeCAD instance and one current review document. The opening macro preserves open documents as recovery copies under `/tmp` before closing them; move any needed recovery copy to durable storage. The native document and STEP file are generated geometry, not editable sketch history: change the authoritative Python source. Coordinates are millimetres, with magnet bottoms at z = 0 and no display offsets. Toggle hidden covers and films in FreeCAD to inspect them.

The active shared environment is `methodology/cad/.venv`. Direct dependencies are declared in the linked [CAD requirements](../cad/requirements.txt) and [calculation requirements](../requirements.txt); the [environment lock](../cad/environment.lock.txt) records the installed package set. FreeCAD uses its own bundled Python; replay encoding also requires the system FFmpeg executable. Geometry export and FreeCAD import checks compare solids, physical intersections, dimensions and volumes; they do not qualify thermal or closed-loop behavior.


```sh
methodology/cad/.venv/bin/python -m methodology.prototype_coils
methodology/cad/.venv/bin/python -m methodology.prototype_coils --refine
methodology/cad/.venv/bin/python -m methodology.prototype_coils --check
methodology/cad/.venv/bin/python -m methodology.prototype_coils --plot
methodology/cad/.venv/bin/python -m methodology.cad.magnetics
methodology/cad/.venv/bin/python -m methodology.cad.magnetics --path
methodology/cad/.venv/bin/python -m methodology.cad.square_assembly
~/.local/bin/freecad - --single-instance methodology/results/cad/open_square.FCMacro
```

## Supporting geometry comparison

The earlier sharp-corner, flat-wire screen established the shortlist; its current and loss values do not describe the practical round-wire winding above. Its full calculated results, allocation limits and source fingerprints remain in [coil_layouts.json](../results/coil_layouts.json), with a [same-scale layout diagram](../results/coil_layouts.png). Use the round-wire results above as the starting point for the integrated virtual design.

That screen reused the shared force/torque solver and allocators across stacked, parquet and square geometries, with position, gap, yaw and hot/heavy cases. It found that larger coils or wider winding walls did not automatically reduce current demand. The fixed-area selection window also excluded many large stacked coils at some positions; its poor reference result must not be presented as a general percentage improvement over the existing dynamic model. The saved evidence includes a same-pose comparison with the existing selection policy.

Reproduce the supporting screen from the repository root:

```sh
methodology/cad/.venv/bin/python -m methodology.coil_layouts
methodology/cad/.venv/bin/python -m methodology.coil_layouts --refine
methodology/cad/.venv/bin/python -m methodology.coil_layouts --check
methodology/cad/.venv/bin/python -m methodology.cad.layouts
```

## What published designs use

| Geometry | Evidence | Relevance to this project |
|---|---|---|
| Single-plane rectangular herringbone | The TU/e HPPA was built and operated with independently driven coils and position-dependent commutation. A later comparison studied rectangular herringbone and short rounded coils, including the trade-off between end windings and coupling near the magnet-array edge. [Van Lierop, 2008](https://research.tue.nl/en/publications/magnetically-levitated-planar-actuator-with-moving-magnets-2/); [Rovers et al., 2011, author-uploaded paper](https://www.researchgate.net/publication/254848363_Comparison_of_different_topologies_for_a_magnetically_levitated_planar_actuator). | A credible parquet candidate. The published magnet arrays and coil proportions differ from ours; their dimensions cannot be transferred directly. |
| Single-plane square coils | Waterloo built square-coil platforms and validated levitation and motion. Its design evaluates efficiency over relative mover positions rather than only aligning one magnetic pole with one coil. [Xu, 2024, Chapters 5–6](https://uwspace.uwaterloo.ca/items/dccf087c-858c-4033-bd70-2729f8ca67cb). | A simple alternative with fewer orientation-specific winding arrangements. We test it alongside parquet. |
| Rounded coils on a triangular grid | TU/e research proposed this arrangement to improve controllability through arbitrary yaw. [Gajdusek, 2010](https://research.tue.nl/en/publications/controlling-a-contactless-planar-actuator-with-manipulator/). | Relevant to rotation, but not proof of lower losses for our king. It is a researched geometry, not part of this numerical comparison. |
| Multilayer PCB spirals | A fabricated prototype uses independent square coils with winding traces on multiple PCB layers. Later work explicitly studies PCB coil power loss and temperature. [Zou et al., 2023](https://doi.org/10.3390/s23218735); [Zhang et al., 2025](https://doi.org/10.3390/act14010033). | A manufacturing method distinct from the number of overlapping coil arrays: one array plane can still contain many conductor layers. PCB current capacity and cooling require their own design. |
| Multiple conductor orientations stacked vertically | Beckhoff's stator patent describes longitudinal and transverse/oblique conductor layers and their interconnections. [EP3791465B1](https://patents.google.com/patent/EP3791465B1/en). | Stacking is a legitimate approach. A patent describes embodiments; it does not establish the construction of a particular commercial tile or suitability for our dimensions. |
| Optimized nonstandard winding contours | Recent work uses a stream-function method to reduce parasitic force and force harmonics compared with a racetrack coil. [Kuijpers et al., 2025](https://research.tue.nl/en/publications/coil-design-for-a-moving-magnet-planar-motor-using-the-stream-fun/). | Shape optimization can matter, but its published gains concern a particular magnet/coil system. Our first comparison uses simpler windable outlines. |

Commercial modular systems confirm the practicality of six-axis moving-magnet transport, but their public product descriptions are not sufficient to reconstruct the internal winding geometry. See [Planar Motor's system description](https://planarmotor.com/overview). The comparisons here use explicit geometries rather than assumptions about proprietary products.

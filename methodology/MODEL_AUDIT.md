**Model migration audit — 5 October 2026**

Scope: step 1 only. Review `model.py` and `levitation_sim.py` against single-piece levitated movement and simultaneous reset with surface contact. No model, requirement document, or saved simulation report was changed. The pre-existing user edit to `PRODUCT_VISION.md` was preserved. Quantitative requirements belong in `model.py`; this document identifies decisions and changes, not new accepted requirement values.

**Conclusion**

Migrate the existing model. Retain the magnetic geometry, force/torque matrix, current-allocation solvers, sensing calculations, and reusable thermal/electrical accounting. Replace the operating scenarios and their consumers before running a design optimization. Most obsolete code is still actively used: the old reset assumptions even influence which winding is selected.

The parameter audit found 234 class-level parameters: 44 in `Inputs`, 176 in `Fixed`, and 14 in `Constants`. Every parameter has at least one direct code reference. Five have only direct reporting references. There are two uncalled top-level simulation helpers worth reusing, three unused constructor arguments, and twelve stored attributes with no attribute reads in either Python file. These are static repository findings, not claims about external callers.

**Method and limits**

- Read both Python files, the current product documents, and the tracked file inventory; inspect the saved report as historical output only.
- Use Python AST traversal to inventory parameter definitions, direct reads and consuming scopes, uncalled top-level functions, unused constructor arguments, unread stored attributes, and repeated report labels. Manually inspect the resulting candidates and calculation dependencies.
- Execute only isolated existing definitions for three inexpensive probes: zero-deficit buffer construction, winding-width accounting, and movement-duration arithmetic. Do not import or run the complete model: module import executes the simulation, and the normal entry point overwrites `last_run.txt`.
- This is not a new electromagnetic simulation, stability proof, thermal certification, wire-sourcing exercise, or validation of component prices. Static references do not prove a parameter changes an accepted design. Complete numerical sensitivity testing belongs after migration.
- There is no tracked dependency manifest or automated test suite in the current file inventory. References to deleted verification scripts are historical evidence, not reproducible checks in this checkout.

**Current calculation chain**

```text
Inputs / Fixed / Constants
  -> BoardGeometry / CoilBed
  -> SimGeometry -> measure()
       -> magnetic geometry + nominal/pose-dependent force/torque matrices
       -> peak-current allocation, flight powers, nominal stiffness
       -> height-coupling table, neighbour force, mode-compression diagnostic
  -> HalbachArray / Piece / NeighbourSnap
  -> ConfigurationSweep
       -> CoilConfiguration + Propulsion + AttitudeAuthority gates
       -> DiscreteDriver + RadiatorCooling scoring for each candidate
  -> selected winding
       -> WireThermal / DiscreteDriver / RadiatorCooling
       -> CoupledAuthority / EddyDrag
       -> Control -> BoardControl <-> HallSensing pitch selection
  -> DriveMatrix / PowerSupply -> EnergyBuffer
  -> Stability / SurfaceStack / StatusChecks / MassBudget / BillOfMaterials
  -> report
```

The old population parameter is read directly by `CoilBed`, `WireThermal`, `DiscreteDriver`, `RadiatorCooling`, `BoardControl`, and `PowerSupply`. Its effects then reach winding ranking, cooling, electronics, supply, mass, and BOM. Reducing it to one would also reduce the modeled reset population and heated source area to one piece. It would not introduce sliding.

**Changes required before trusting a revised design sweep**

| ID | Evidence in the current code | Consequence | Migration action |
|---|---|---|---|
| A1 | [Population](/home/arseniy/projects/flying_chess/model.py:50), [active coils](/home/arseniy/projects/flying_chess/model.py:346), [reset heat trace](/home/arseniy/projects/flying_chess/model.py:828), [power](/home/arseniy/projects/flying_chess/model.py:1607) | One count stands for airborne pieces, reset population, current channels, thermal footprint, and compute load. | Keep the flight concurrency requirement, change it to the authorized single-piece behaviour, and separately represent reset participation/contact motion. Derive energized coils from actual shared-coil allocations rather than blindly multiplying local windows. |
| A2 | [Reset heat construction](/home/arseniy/projects/flying_chess/model.py:829) | Five lift/cruise/land waves still define reset energy, peak power, and winding thermal ranking. | Replace this trace with contact-reset motion. Remove airborne takeoff-wave timing, airborne landing multipliers, and their checks when their replacement is wired through. Retain T1 repetition and back-to-back-event requirements. |
| A3 | [Existing slide calculation](/home/arseniy/projects/flying_chess/model.py:1221) and [friction usage](/home/arseniy/projects/flying_chess/model.py:440) | “Slide-to-phase” is a half-square motion at flight geometry with eddy drag; friction is used only for the no-snap check. There is no reset contact-force model. | Reuse the magnetic matrix and motion calculations, but add support reaction, static/sliding friction, contact-height fields, and shared-coil constraints. Keep no-snap checks and neighbour disturbance checks distinct. |
| A4 | [Winding geometry](/home/arseniy/projects/flying_chess/model.py:454), [wire construction](/home/arseniy/projects/flying_chess/model.py:281), [sweep](/home/arseniy/projects/flying_chess/model.py:614) | Candidate widths consume the entire radial wall before insulation. The width probe gives 1.024 mm insulated wire in a 1.0 mm wall. Existing window checks do not check that fit. | Calculate turns and dimensions from insulated conductor size, winding allowance, and actual wire candidates. Retain resistance/copper accounting; replace the assumed packing. Do not call the current geometry manufacture-verified. |
| A5 | [Power coefficients](/home/arseniy/projects/flying_chess/levitation_sim.py:694), [flight allocation](/home/arseniy/projects/flying_chess/levitation_sim.py:768), [minimum-loss solver](/home/arseniy/projects/flying_chess/levitation_sim.py:263) | Power is computed from minimum-peak-current solutions. The minimum-loss solver is unused. | Use minimum peak current to establish the required channel rating; use minimum loss subject to that rating for energy/heat. Preserve the actual current vectors for downstream losses, sensing, and shared-coil accounting. |
| A6 | [Control](/home/arseniy/projects/flying_chess/model.py:1285), [stiffness](/home/arseniy/projects/flying_chess/levitation_sim.py:506), [nominal evaluation](/home/arseniy/projects/flying_chess/levitation_sim.py:887) | Control frequency comes from a nominal 3-by-3 translational stiffness calculation at the reference coil height. Rotation is approximated separately. | Retain this as a baseline estimate only. Extend the same model for selected-coil six-axis dynamics, latency and saturation before claiming a minimum control frequency. |
| A7 | [Default move energy](/home/arseniy/projects/flying_chess/model.py:896), [throughput](/home/arseniy/projects/flying_chess/model.py:906), [time truncation](/home/arseniy/projects/flying_chess/model.py:842) | Average heat can be calculated for a movement schedule that one airborne piece cannot execute. Current base-move assumptions occupy 2.5 s versus a 1 s interval at 60 composite moves/minute, before extra capture/blocker activity. | Preserve the testing target, make the movement schedule explicit, and detect infeasibility. Do not silently shorten a cascade's physical dwell with `min(...)` to fit throughput. No timing target is relaxed by this audit. |
| A8 | [Selection](/home/arseniy/projects/flying_chess/model.py:601), [Hall selection](/home/arseniy/projects/flying_chess/model.py:2149), [checks](/home/arseniy/projects/flying_chess/model.py:1769) | Thermal-invalid windings can become diagnostic selections; final Hall selection returns the last failing candidate; status checks are report strings rather than an acceptance gate. | Separate diagnostic results from accepted designs. Require an explicit failed result/exception when no candidate meets the chosen requirements. Retain meaningful feasibility checks and remove the silent Hall fallback. |

**Keep/change/remove map for `model.py`**

“Keep” means retain the implementation or baseline assumption where applicable, not certify its current numerical value. “Review” means a requirement or architecture choice remains unresolved; do not delete it automatically.

| Existing component | Disposition | Specific scope |
|---|---|---|
| `Inputs`, `Fixed`, `Constants` | Keep containers; revise contents | Maintain quantitative requirements here. Full parameter inventory follows. Remove obsolete scenario controls; distinguish requirements, implementation assumptions, physical constants, and numerical settings. |
| `Cell`, `format_value`, `print_section`, BOM printing | Keep | Reporting infrastructure remains useful. Remove obsolete/duplicate output rows with the underlying calculations. |
| `Wire`, `rectangular_wire` | Change | Retain conductor representation; add actual insulated dimensions and packing. Avoid a second parallel winding representation. |
| `BoardGeometry` | Keep baseline; review constraints | Board plus motorized storage geometry remains required. Integer-period docking and the base standoff are design assumptions, not reasons to delete coverage calculations. |
| `CoilBed` | Change | Keep coil geometry and total installed coil count. Replace activity accounting; document split-rail voltage fraction as topology-dependent. Check physical board edges and overlapping control windows. |
| `HalbachArray`, `Piece` | Keep; consolidate duplicated calculations | Same physical quantities are also constructed in `MagnetLayout` and `piece_weight`. Choose one authoritative calculation and preserve unit conversion/reporting. Revisit inertias with a consistent mass model. |
| `NeighbourSnap` | Keep and extend | Required by C5 even though levitation concurrency changes. Static friction alone does not implement reset sliding or demonstrate behaviour as a neighbour is moved away. |
| `CoilConfiguration` | Change | Keep turns/resistance/force/current calculations; correct packing, retain current vectors, replace old reset-flight outputs, verify finalists at actual coil height rather than relying solely on scalar coupling. |
| `ConfigurationSweep` | Keep structure; change evaluation | Retain candidate enumeration and fail reasons. Replace airborne-reset thermal scoring and reconcile preliminary gates with final coupled/control/sensing gates. Keep the selected bus as a baseline initially; reselect only after migrated demands are available. |
| `WireThermal` | Change | Keep copper mass/length and one-piece loss accounting. Replace its fleet-hover multiplier. It is an inventory/power wrapper, not an independent transient thermal solver. |
| `DiscreteDriver` | Change workload; retain topology baseline | Separate current-squared conduction loss, current-dependent switching loss, active-channel gate loss, and fixed electronics loss. Do not scale all of them using one hover-power ratio. Keep installed driver counts separate from concurrently energized counts. |
| `RadiatorCooling` | Substantial change | Keep thermal-stack/RC utilities and material derating concepts. Replace airborne reset scheduling and footprint assumptions. Feed shared movement/current traces into thermal and PSU accounting. Retain local hammer/capture and sustained-use tests. |
| `SurfaceStack` | Keep; align limits and reports | Visible height, contact surface, and touch limits remain required. Avoid treating the calculated temperature as proof of a governor that is only described in report text. |
| `Propulsion` | Consolidate | Keep move-distance/kinematic outputs. Replace unconstrained force-based travel claims and all-coils-at-limit “flight power” with the selected coupled allocation. Retire the alternate magnet-only yaw model once the shared inertia/torque path is used. |
| `AttitudeAuthority` | Keep six-axis requirements; change calculation ownership | Own or consume a consistent inertia/torque demand model. Reconcile torque reference frames with the simulator; review commanded showpiece tilt/yaw requirements separately from stabilization. |
| `CoupledAuthority` | Keep; unify with candidate evaluation | Reuse full constrained force/torque authority. Avoid contradictory preliminary and final acceptance paths. Review automatic selection of a shallower showpiece tilt; it must not silently change the required envelope. |
| `EddyDrag` | Keep physics helper; change scenarios | Conductive layers still matter for motion. Reuse sheet/image calculations, evaluate at contact and flight gaps, and avoid confusing the old “slide” check with frictional reset feasibility. |
| `Control`, `Stability` | Consolidate and extend | One authoritative dynamic model should generate unstable modes and rate requirements. Retain reporting, replace separate translational/rocking proxies as final acceptance evidence. |
| `DriveMatrix` | Simplify | Mostly copies driver/control/architecture attributes and recomputes margins for reports/checks. Keep one owner per calculated quantity; remove copied state where consumers can read its owner directly. |
| `HallSensing` | Keep; change scenarios | Retain field Jacobian, noise, interference and saturation checks. Add actual contact/sliding poses and energized neighbours; present flight/rest estimates as estimates rather than a reset guarantee. |
| `BoardControl` | Change workload | Current sizing scans the whole Hall grid and monitors all installed channels at the pose rate. Single-piece flight permits evaluating local fast scanning, but simultaneous reset and arbitrary hand placement still need coverage/tracking. Remove the always-true coverage assertion. |
| `EnergyBuffer` | Keep pending new traces; make optional in architecture | Remove hardware costs when a no-buffer architecture is selected. Retain deficit/ESR/energy sizing if the new workload still requires storage. Do not assume the component becomes unnecessary merely because pieces slide. |
| `PowerSupply` | Change | Reuse cumulative drawdown/recharge calculations with the actual event trace. Replace fleet-hover capacity and wave policy. Remove one-zone aliases when they add no distinction. Reconsider the fixed 2.5 kW source after migration. |
| `StatusChecks` | Change | Remove airborne-wave checks, retain constraints still applicable, add contact/reset/schedule checks, and distinguish tautologies from independent validation. Centralize accepted quantitative limits in the input classes. |
| `BomItem`, `BillOfMaterials`, `MassBudget` | Keep accounting; refresh dependencies | Quantities must follow selected architecture, including absence of buffer/fans. Avoid using storage capacity as the sole name for total piece count. Parameterize the 100-board order basis and hard-coded unit prices. Do not assume current prices satisfy that volume basis. |
| Module-level construction, `select_hall_pitch`, `run_report`, `print_sweep` | Change orchestration minimally | Keep existing entry point; make calculation callable without import-time full execution or mandatory report overwrite. Share candidate evaluation with report generation instead of recalculating thermal results there. |

**Keep/change/remove map for `levitation_sim.py`**

| Existing component | Disposition | Specific scope |
|---|---|---|
| `SimGeometry`, `use_geometry` | Keep bridge; review implicit state | SI conversion and geometry handoff remain useful. Global `G` is hidden mutable state; at minimum isolate scenario evaluation so different geometries cannot contaminate each other. A wholesale module rewrite is not needed. |
| `unit_vector`, `snap_to_axis`, `AXIS_DIRECTIONS` | Keep | Existing magnet orientation construction. |
| `matrix_rank`, `condition_number` | Keep | Numerical diagnostics; retain explicit rank checks rather than interpreting a finite condition number as sufficient. |
| `MagnetLayout`, `magnet_layout_from_geometry` | Keep | Permanent-magnet construction/field evaluation remains central. Consolidate mass/base/COM formulas shared with `model.py`. |
| `CoilArray`, `coil_array_from_geometry` | Keep; change geometry inputs | Reuse finite coil field/force calculations with buildable dimensions. A centered local patch is not physical-board edge coverage. |
| `place_piece`, `actuator_matrix` | Keep | Foundation for levitated and contact force/torque allocation. Sliding requires extra contact constraints and multi-piece matrices, not a second magnetic engine. |
| `piece_weight` | Consolidate | Duplicates `Piece` shell and magnet mass arithmetic. Preserve agreement between force targets and reported mass. |
| `min_peak_current` | Keep | Required for per-channel demand. It includes a zero-net-current constraint tied to the current split-rail topology. Do not remove that row independently of the hardware model. |
| `min_loss_current` | Keep and connect | Currently uncalled; reuse for heating/energy at an explicit peak-current bound. |
| `horizontal_wrench_targets` | Keep and connect | Currently uncalled; use to cover travel directions instead of duplicating x/y target construction. |
| `min_peak_current_in_basis`, `local_current_mode_analysis` | Defer from default execution; retain as optional analysis | Active mathematical diagnostic, not dead code. It feeds only the mode report and does not select the winding, reduce BOM drivers, or prove implementable routing. Keep available for later channel-topology work. |
| `max_generalized_force`, `Controllability`, `analyze` | Keep; share results | Reuse constrained authority and nominal analysis. Avoid rerunning an identical nominal allocation just to produce another coefficient bundle. |
| `open_loop_stiffness` | Extend | Currently translation-only at fixed currents. Include rotation and selected-coil operating points before using it for frequency acceptance. |
| `local_hall_points`, `hall_jacobian`, `hall_metrics` | Keep | Sensor geometry and local six-axis observability/noise calculations remain useful. |
| `rim_tilt_limit` | Keep; reconcile geometry | Share clearance semantics with `AttitudeAuthority`; check actual lowest rim under combined rotations rather than independently applying single-axis limits. |
| `hall_observability`, `verified_hall_sensing` | Change coverage | Flight pose sweeps plus nominal interference estimates do not cover contact reset. Reuse helpers for rest/contact poses, multiple nearby pieces, and sensor windows at physical edges. |
| `eddy_image_force` | Keep | Reusable conductive-layer estimate; not a contact/friction model. |
| `neighbour_force`, `worst_touching_snap` | Keep and extend | Preserve C5 support; extend scenario coverage as needed for motion, vertical force and torque, and multiple neighbours. Existing scan considers yaw pairs at one touching center separation. |
| `pose_coefficients` | Change | Keep useful coefficient extraction, but separate minimum peak current from minimum-loss energy. Some one-axis authority bounds constrain only part of the six-axis wrench. |
| `flight_worst_case` | Keep flight path; broaden explicit envelope | Retain pose evaluation. Current grid samples three offsets, two heights, limited yaw/tilt poses, and +x/+y travel. Add required directions and boundary cases without labeling a sampled grid a global proof. Contact reset needs its own target/constraint scenario using the same matrix/allocation helpers. |
| `coupled_worst_authority`, `verified_worst_authority` | Keep; consolidate acceptance | Stronger authority calculation than independent-axis proxies. Note `level_wrenches` contains only the first gap, whereas `level_all_wrenches` contains both; transport lateral authority currently uses the first-gap subset. |
| `coil_height_coupling` | Keep as screening estimate | It derives a scalar from nominal lift and applies it broadly downstream. Verify shortlisted coil geometries directly for actual force/torque/current and dynamics. |
| `measure` | Change orchestration/results | Preserve magnetic measurements; separate optional mode diagnostics and scenario outputs. Remove the assumption that the caller always needs two airborne cruise targets. Expose enough information for consistent downstream allocation and dynamics. |

**Confirmed cleanup candidates**

Remove obsolete behaviour only when its replacement is connected; these are not implementation changes in this audit.

| Candidate | Evidence and proposed treatment |
|---|---|
| `Inputs.contention_hover` | Its only reader is `StatusChecks.stagger_fits`. Despite the name/comment, it does not add contention-hover energy. Remove with the wave-fit check. |
| `Inputs.takeoff_waves` | Drives the airborne reset pipeline and report. Remove when the contact-reset trace replaces it. |
| `Inputs.reset_cruise_stretch` | Scales a second airborne thrust target and reset duration. At its current value the two targets are identical. The second target's power/current results are only reported; the thermal reset still uses `sprint_piece_power`. Replace with explicit reset motion constraints, not a renamed airborne multiplier. |
| `Fixed.landing_crowding_factor` | Used in fleet reset landing heat. Remove that use with airborne resets; recompute genuine grounded-neighbour interactions rather than reusing this empirical multiplier. |
| `Fixed.adjacent_hover_crowding_factor` | Only appears in a report sentence saying the multiplier is not used. Remove parameter and stale narrative. |
| `Fixed.reset_choreography_demonstrated` | A manually fixed boolean, not a routing calculation. Replace the old five-wave assertion with evidence for the new reset scenario. Do not merely set it to true. |
| `CoilConfiguration.__init__.halbach`, `DiscreteDriver.__init__.board`, `StatusChecks.__init__.halbach` | No reads of these arguments within the constructor bodies. Remove arguments and update their call sites together. |
| `CoilConfiguration.hover_current`, `.power_per_winding` | Assigned but no attribute reads found in either file. Remove unless the new vector-based reporting explicitly needs them; do not retain a second scalar current definition just for compatibility. |
| `RadiatorCooling.grind_baseline`, `.local_heat_capacity`, `.play_average_power` | Stored attributes have no reads. The `grind_baseline` argument itself is used: only the redundant stored copy is a deletion candidate. |
| `CoupledAuthority.showpiece_lift_margin`, `.tilt_torque` | Stored results have no reads. This does not justify removing the underlying force/torque authority checks or rung results. |
| `EddyDrag.cruise_drag`, `.slide_drag` | Stored values are not read; `drag_at` is called only to populate these two values. Remove this unused path if no migrated scenario consumes it. Retain `sheet`, which also supplies the reported drag and damping used by motion checks. |
| `BomItem.scope` | Stored but not read; grouping already comes from the three item lists. Remove the redundant field/argument if that representation is retained. |
| `MagnetLayout.wavenumber`, `Controllability.characteristic_length` | Stored attributes have no reads. The local `characteristic_length` argument is still used for wrench scaling. |
| `sim['actuator_condition6']` | Produced by `measure` but not consumed by the model. Either expose/use this diagnostic meaningfully or remove the unused output key; keep conditioning analysis where needed. |
| Duplicate thermal row | `RadiatorCooling.cells` emits the identical “Worst-phase takeoff local rise” row twice. Remove one. `Propulsion`'s repeated acceleration label uses different units and is not the same duplicate. |

Five direct report-only parameters are `Fixed.cooling_fan_speed`, `Fixed.cooling_fan_static_pressure`, `Fixed.comparator_input_offset`, `Fixed.supercap_cell_max_voltage`, and `Fixed.adjacent_hover_crowding_factor`. The first four may remain component metadata, but must not be mistaken for checks or active numerical constraints. In particular, supercapacitor maximum voltage is printed but never validates working voltage.

**Duplicated or misleading calculation paths to consolidate**

- **Piece properties:** `Piece`, `piece_weight`, and `MagnetLayout.compute_center_of_mass_height` reconstruct related dimensions/masses; `Propulsion`, `AttitudeAuthority`, and `Stability` use different inertia approximations. Define shared piece properties once and consume them consistently. The simulator's torques are anchored at the COM, while `AttitudeAuthority` adds an `mg*h*sin(angle)` static term; reconcile reference frames before reusing this as a demand constraint.
- **Force authority:** `Propulsion` and parts of `AttitudeAuthority` use partially constrained coefficient maxima, while `CoupledAuthority` later imposes hover and other-axis constraints. Keep fast screens if useful, but require one authoritative coupled acceptance result for the selected candidate.
- **Losses:** `DiscreteDriver` mixes fixed/control and current-dependent losses; `RadiatorCooling` converts the total to a ratio of hover copper loss; `PowerSupply` then scales total driver loss by the sprint/hover ratio. One piece makes this particularly misleading because board electronics do not scale with moving-piece count. Share per-scenario loss accounting between supply and heat calculations.
- **Reset traces:** thermal accounting computes both back-to-back events, retains only the final event's segments, and PSU accounting repeats that final event. This is not the actual original two-event trace. Use the same explicit trace in both consumers.
- **Thermal scope:** fans-off cooling explicitly multiplies reset temperature contributions by zero. A passive-mode result therefore does not establish fanless reset feasibility. The hammer term uses average duty times thermal resistance, not the periodic local peak of a pulsed trace. Retain the workload, replace the assumptions when migrating thermal evaluation.
- **Buffer absence:** the isolated `EnergyBuffer(40, 1, 0, 0)` probe produces zero cells but four protection paths and $56 cost. BOM construction likewise retains buffer management entries. An explicitly selected no-buffer architecture must omit those components as well as the cells.
- **Sensing scope:** rest saturation is extrapolated from a flight field and assumes coils off under the piece. `verified_hall_sensing` checks one nominal neighbour field; it does not validate energized multi-piece contact reset. Observability should remain separate from interference, saturation, and end-to-end timing acceptance.
- **Control wording:** `Control.slew_time` uses `operating_current` while the report says “to Imax.” `measure` reports the largest absolute translational stiffness eigenvalue as `vertical_stiffness`, which is not explicitly the vertical derivative. Preserve the distinction when consolidating dynamics.
- **Validation versus assertions:** `BoardControl.exact_motor_fit = True` cannot verify board coverage. Several driver/buffer count checks compare quantities constructed to be equal or sufficient by definition. They can be arithmetic sanity checks, but do not demonstrate physical coverage, routing, current-loop performance, or reset feasibility.
- **Unsupported report policies:** the report describes a per-cell thermal governor, fault handling and routing policies that are not implemented as such in these two files. Keep these as proposed architecture only until modeled or demonstrated. References to C6 and W2/W3 should be reconciled with the existing T1/T2/T3 document labels.

**Quantitative settings outside the three parameter classes**

Do not move physical identities or unit conversions into `Inputs`. Do move actual design requirements, assumptions, candidate lists, and validation thresholds to a clearly owned location in the existing three classes when their consumer is migrated.

| Setting currently embedded in calculations | Location | Proposed ownership |
|---|---|---|
| Platform side restricted to 20–50 mm; maximum flatness/visible-gap ratio 0.25; maximum tip sensing error 0.001 m | `StatusChecks`; tip error repeated in `select_hall_pitch` | `Inputs` requirements, with one definition per threshold |
| Shunt power derating factor 0.5; half-bridge count and usable split-rail fraction | `StatusChecks`, `CoilBed` | `Fixed` implementation assumptions; voltage fraction should follow the chosen topology |
| Extra acceleration factor 1.05; duplicate full-diagonal trajectory expressions | Module-level `SimGeometry` construction, `EddyDrag` | Named requirement/margin in `Inputs`; one shared derived trajectory |
| Wire and bus candidate lists currently in `Constants` | `Constants.winding_conductor_thicknesses`, `.standard_bus_voltages` | Move/reclassify as design choices in `Inputs`/`Fixed` when replacing the candidate catalogue |
| Coil-height candidate table, pose yaw/tilt samples, direction count, numerical mesh/finite-difference settings | `levitation_sim.measure`, `flight_worst_case`, Hall/force helpers | Pass explicitly from model-owned settings through the existing handoff. Separate required envelope from numerical resolution. No need to import `model` into the simulator. |
| Rank tolerances, solver tolerances and iteration limits | Simulation helpers; thermal loop; eddy-motion search | `Fixed` numerical settings where configurable. These are numerical policy, not product requirements. Check convergence instead of silently accepting an exhausted iteration budget. |
| Reference material temperature 20 C, duplicated copper density 8.96 g/cc | Temperature calculations; `MassBudget` | Reuse a named material reference and `Constants.copper_density` with unit conversion |
| Wire $/kg, plastic allowance, host/carrier prices; production volume 100 only in report text | `BillOfMaterials`, `print_bom` | `Fixed` cost assumptions and `Inputs` production volume, with consistent procurement quantities. Exact prices are not validated here. |
| Fan acoustics | `RadiatorCooling` reports dB(A), no acoustic acceptance gate | An agreed quantitative target belongs in `Inputs`; do not infer a new target from the current fan selection |

Known missing inputs/derived scenario data for the next step: contact-reset duration/acceleration constraints, static and kinetic friction assumptions, normal-force/contact policy, scenario-specific concurrency, a feasible composite-move schedule, contact-position error budget, and a defined dynamic disturbance/settling envelope. Reuse existing mass, gap, trajectory and safety parameters where they express the same concept; do not create parallel copies. This audit does not choose values for those missing quantities.

**Requirement decisions to retain visibly**

1. `min_level_hover_endurance` and `min_showpiece_hover_endurance` impose demonstration behaviours beyond transient levitation. Their absence from the vision does not make them automatically removable: quantitative requirements are allowed in the model. Keep them marked for review until intentionally retained or retired.
2. `target_tilt_time`, `target_yaw_angle_deg`, and `target_yaw_time` define manoeuvres, whereas six-axis stabilization is mandatory. Preserve six-axis authority regardless of the decision on demonstrations.
3. Preserve the 60 composite moves/minute test and report its scheduling conflict with the current per-move duration. Do not reinterpret it as 60 independent simultaneously airborne moves.
4. C2 requires simultaneous reset capability; collision avoidance may require local waiting, but this audit does not authorize replacing the reset with a one-piece-at-a-time reset.
5. Define “A” operationally in model-owned scenario data so the existing T1/T2/T3 tests have clear phase/current traces. Leave the protected testing document unchanged.

**Proposed migration order, using the current classes**

1. Establish one explicit scenario representation for normal flight and contact reset using `Inputs`/`Fixed` values. Preserve baseline geometry/hardware. Add a callable calculation path so targeted checks do not execute the whole model or overwrite historical output.
2. Extend the existing matrix/allocation helpers for contact targets, shared-coil interactions and scenario current vectors. Connect the existing minimum-loss and directional-target helpers. Resolve the winding packing issue before accepting any buildable candidate.
3. Replace `RadiatorCooling`'s wave trace and population-derived footprints with scenario traces. Feed the same traces/losses into `PowerSupply`; update active-channel and compute accounting. Delete wave/landing/airborne-reset parameters and reports in this same change.
4. Consolidate force, piece-property, and dynamic acceptance paths; migrate sensing/contact coverage and fail clearly when no candidate meets requirements. Keep optional mode compression out of the mandatory run path.
5. Update mass/BOM/report consumers, remove unused fields/arguments and stale policies, then run the revised numerical design sweep. Do not optimize around the old thermal scoring while step 3 is unfinished.

Migration completion should mean: single-piece flight and simultaneous contact reset are both represented; no reset lift/hover power remains; all retained requirements have a corresponding check or an explicit unvalidated status; no-buffer scenarios omit buffer hardware; and the saved report clearly distinguishes calculated feasibility from assumptions. It does not mean stability or manufacturability is already proven.

**Verification record**

The isolated probes used AST-extracted, unmodified definitions from `model.py`:

| Probe | Observed result | Interpretation |
|---|---|---|
| `EnergyBuffer` at 40 V, one zone, zero power/energy deficit | 0 cells, 4 protection paths, $56 | A valid no-buffer configuration is not represented completely |
| Existing 1 mm-wide, 0.05 mm-thick wire construction | 1.024 mm insulated radial pitch versus 1.0 mm winding wall | Insulation is not included in radial fit |
| Current base-move phase durations | 0.5 + 1.5 + 0.5 = 2.5 s; requested composite interval = 1 s | Existing assumed full-duration move stream cannot run serially at the stated rate |

Source fingerprints used for this audit: `model.py` SHA-256 `22765da46bd8052187cef72140d135652f70b4fdf0f030ad315c38878d2982bb`; `levitation_sim.py` SHA-256 `cb5377c7b69b26499190f32e3cce384c3aeddb8b0da5e43b7639078ca1d3e333`. Source line references describe this snapshot.

**Complete parameter inventory**

The inventory below lists every parameter with its definition line, migration disposition, and direct consuming scopes. It does not include transitive consumers reached through copied instance attributes. `model setup` is the module-level geometry/construction path. `Keep` preserves a baseline concept, not its numerical validation; `Recalculate` preserves a needed parameter but re-evaluates its value or use; `Replace` retires an old representation once its successor is connected; `Review` requires an explicit requirement/architecture decision; `Metadata` means reporting-only direct use.

**Inputs**

| Parameter | Current value | Disposition | Direct consumers |
|---|---|---|---|
| [magnet_lateral_edge](/home/arseniy/projects/flying_chess/model.py:10) | `5` | Keep baseline geometry; re-evaluate in sweep | model setup, BillOfMaterials, BoardGeometry, HalbachArray |
| [magnet_thickness](/home/arseniy/projects/flying_chess/model.py:11) | `4` | Keep baseline geometry; re-evaluate in sweep | model setup, BillOfMaterials, HalbachArray |
| [magnets_per_period](/home/arseniy/projects/flying_chess/model.py:12) | `4` | Keep baseline geometry; re-evaluate in sweep | model setup, BoardGeometry, HalbachArray |
| [periods_per_side](/home/arseniy/projects/flying_chess/model.py:13) | `1` | Keep baseline geometry; re-evaluate in sweep | model setup, BoardGeometry, CoilBed, HalbachArray |
| [coils_per_period](/home/arseniy/projects/flying_chess/model.py:14) | `2` | Keep baseline geometry; re-evaluate in sweep | CoilBed, HallSensing |
| [coil_outer_length](/home/arseniy/projects/flying_chess/model.py:15) | `15` | Keep baseline geometry; re-evaluate in sweep | CoilBed |
| [winding_radial_width](/home/arseniy/projects/flying_chess/model.py:16) | `1.0` | Keep baseline geometry; re-evaluate in sweep | CoilBed |
| [control_cells_per_side](/home/arseniy/projects/flying_chess/model.py:17) | `4` | Keep baseline geometry; re-evaluate in sweep | CoilBed |
| [magnet_to_coil_distance](/home/arseniy/projects/flying_chess/model.py:18) | `3.5` | Keep baseline geometry; re-evaluate in sweep | model setup, EddyDrag, HallSensing, StatusChecks, SurfaceStack |
| [max_flight_gap](/home/arseniy/projects/flying_chess/model.py:19) | `4.5` | Keep baseline geometry; re-evaluate in sweep | model setup, AttitudeAuthority, StatusChecks, SurfaceStack |
| [plastic_wall_thickness](/home/arseniy/projects/flying_chess/model.py:20) | `1.0` | Keep baseline geometry; re-evaluate in sweep | model setup, Piece, StatusChecks |
| [piece_shell_shape_factor](/home/arseniy/projects/flying_chess/model.py:21) | `0.6` | Keep baseline geometry; re-evaluate in sweep | model setup, Piece |
| [worst_phase_dwell](/home/arseniy/projects/flying_chess/model.py:22) | `0.5` | Change: explicit flight/contact phase semantics | EddyDrag, RadiatorCooling |
| [cruise_duration](/home/arseniy/projects/flying_chess/model.py:23) | `1.5` | Change: explicit flight/contact phase semantics | model setup, EddyDrag, RadiatorCooling |
| [reset_cruise_stretch](/home/arseniy/projects/flying_chess/model.py:24) | `1.0` | Replace: contact-reset motion constraints | model setup, RadiatorCooling |
| [selected_bus_voltage](/home/arseniy/projects/flying_chess/model.py:25) | `40` | Recalculate after scenario migration | ConfigurationSweep, PowerSupply |
| [contention_hover](/home/arseniy/projects/flying_chess/model.py:26) | `2.0` | Remove with airborne reset scheduling | StatusChecks |
| [landing_dwell](/home/arseniy/projects/flying_chess/model.py:27) | `0.5` | Change: explicit flight/contact phase semantics | RadiatorCooling |
| [events_back_to_back](/home/arseniy/projects/flying_chess/model.py:28) | `2` | Keep test requirement; change trace consumer | PowerSupply, RadiatorCooling |
| [sustained_event_period](/home/arseniy/projects/flying_chess/model.py:29) | `300` | Keep test requirement; change trace consumer | PowerSupply, RadiatorCooling |
| [play_move_period](/home/arseniy/projects/flying_chess/model.py:30) | `6` | Keep test requirement; change trace consumer | RadiatorCooling |
| [sustained_moves_per_minute](/home/arseniy/projects/flying_chess/model.py:31) | `60` | Keep test requirement; change trace consumer | RadiatorCooling |
| [replay_capture_fraction](/home/arseniy/projects/flying_chess/model.py:32) | `0.25` | Review movement mix; use feasible schedule | RadiatorCooling |
| [replay_knight_gap_fraction](/home/arseniy/projects/flying_chess/model.py:33) | `0.15` | Review movement mix; use feasible schedule | RadiatorCooling |
| [hammer_visit_period](/home/arseniy/projects/flying_chess/model.py:34) | `5` | Keep test requirement; change trace consumer | RadiatorCooling |
| [hammer_dwell](/home/arseniy/projects/flying_chess/model.py:35) | `1.0` | Keep test requirement; change trace consumer | RadiatorCooling |
| [cascade_exchanges](/home/arseniy/projects/flying_chess/model.py:36) | `10` | Keep test requirement; change trace consumer | RadiatorCooling |
| [coil_bed_temp_limit](/home/arseniy/projects/flying_chess/model.py:37) | `105` | Keep requirement; recheck migrated scenarios | ConfigurationSweep, RadiatorCooling, StatusChecks |
| [force_safety_factor](/home/arseniy/projects/flying_chess/model.py:38) | `1.3` | Keep requirement; recheck migrated scenarios | CoilConfiguration, ConfigurationSweep, CoupledAuthority, StatusChecks |
| [min_maneuver_accel_g](/home/arseniy/projects/flying_chess/model.py:39) | `0.2` | Keep requirement; recheck migrated scenarios | ConfigurationSweep, StatusChecks |
| [min_visible_hover_height](/home/arseniy/projects/flying_chess/model.py:40) | `3` | Keep requirement; recheck migrated scenarios | StatusChecks |
| [min_level_hover_endurance](/home/arseniy/projects/flying_chess/model.py:41) | `30` | Review demonstration/manoeuvre requirement | StatusChecks |
| [min_showpiece_hover_endurance](/home/arseniy/projects/flying_chess/model.py:42) | `3` | Review demonstration/manoeuvre requirement | StatusChecks |
| [tilt_rim_clearance](/home/arseniy/projects/flying_chess/model.py:43) | `1.0` | Keep requirement; recheck migrated scenarios | model setup, AttitudeAuthority |
| [target_tilt_time](/home/arseniy/projects/flying_chess/model.py:44) | `0.3` | Review demonstration/manoeuvre requirement | AttitudeAuthority |
| [target_yaw_angle_deg](/home/arseniy/projects/flying_chess/model.py:45) | `90` | Review demonstration/manoeuvre requirement | AttitudeAuthority |
| [target_yaw_time](/home/arseniy/projects/flying_chess/model.py:46) | `0.5` | Review demonstration/manoeuvre requirement | AttitudeAuthority |
| [ambient_temperature](/home/arseniy/projects/flying_chess/model.py:47) | `35` | Keep requirement; recheck migrated scenarios | RadiatorCooling, SurfaceStack |
| [max_surface_temperature](/home/arseniy/projects/flying_chess/model.py:48) | `50` | Keep requirement; recheck migrated scenarios | ConfigurationSweep, RadiatorCooling, StatusChecks |
| [control_loop_bandwidth_margin](/home/arseniy/projects/flying_chess/model.py:49) | `5` | Recalculate with complete dynamics | Control, StatusChecks |
| [pieces_levitating_simultaneously](/home/arseniy/projects/flying_chess/model.py:50) | `32` | Change to single-piece flight; separate reset load | BoardControl, CoilBed, DiscreteDriver, PowerSupply, RadiatorCooling, WireThermal |
| [drive_look_ahead_factor](/home/arseniy/projects/flying_chess/model.py:51) | `1.5` | Replace heuristic with active-coil demand | CoilBed |
| [takeoff_waves](/home/arseniy/projects/flying_chess/model.py:52) | `5` | Remove with airborne reset scheduling | RadiatorCooling |
| [active_cooling_fans](/home/arseniy/projects/flying_chess/model.py:53) | `2` | Recalculate from migrated thermal workloads | model setup, ConfigurationSweep, print_report, print_sweep |

**Fixed**

| Parameter | Current value | Disposition | Direct consumers |
|---|---|---|---|
| [base_corner_standoff](/home/arseniy/projects/flying_chess/model.py:57) | `8` | Keep baseline geometry/material assumption | model setup, BoardGeometry |
| [square_fill_ratio](/home/arseniy/projects/flying_chess/model.py:58) | `0.8` | Keep baseline geometry/material assumption | BoardGeometry |
| [max_chess_square_size](/home/arseniy/projects/flying_chess/model.py:59) | `60` | Keep quantitative requirement; consider Inputs ownership | StatusChecks |
| [resting_friction_coefficient](/home/arseniy/projects/flying_chess/model.py:60) | `0.4` | Recalculate; distinguish static/sliding contact | NeighbourSnap |
| [captured_pieces_total](/home/arseniy/projects/flying_chess/model.py:61) | `32` | Keep capacity; separate meaning from fleet count | BillOfMaterials, BoardGeometry, MassBudget |
| [captured_side_areas](/home/arseniy/projects/flying_chess/model.py:62) | `2` | Keep baseline geometry/material assumption | BoardGeometry |
| [herringbone_orientation_families](/home/arseniy/projects/flying_chess/model.py:63) | `2` | Keep baseline geometry/material assumption | CoilBed, RadiatorCooling |
| [com_height_fraction](/home/arseniy/projects/flying_chess/model.py:64) | `0.4` | Keep baseline geometry/material assumption | model setup, AttitudeAuthority |
| [reference_king_height](/home/arseniy/projects/flying_chess/model.py:65) | `95` | Keep baseline geometry/material assumption | model setup, Piece |
| [reference_king_base_diameter](/home/arseniy/projects/flying_chess/model.py:66) | `44` | Keep baseline geometry/material assumption | model setup, Piece |
| [rectangular_wire_film](/home/arseniy/projects/flying_chess/model.py:67) | `0.012` | Replace with candidate-specific insulation | ConfigurationSweep, rectangular_wire |
| [turns_per_radial_layer](/home/arseniy/projects/flying_chess/model.py:68) | `1` | Replace with actual packing result | CoilBed, CoilConfiguration |
| [nominal_coil_height_for_field](/home/arseniy/projects/flying_chess/model.py:69) | `1.0` | Keep screening reference; verify actual geometry | model setup |
| [potting_thickness](/home/arseniy/projects/flying_chess/model.py:70) | `0.5` | Keep stack/material model; validate chosen construction | MassBudget, RadiatorCooling |
| [potting_thermal_conductivity](/home/arseniy/projects/flying_chess/model.py:71) | `2.5` | Keep stack/material model; validate chosen construction | RadiatorCooling, SurfaceStack |
| [potting_volumetric_heat_capacity](/home/arseniy/projects/flying_chess/model.py:72) | `2300000.0` | Keep stack/material model; validate chosen construction | RadiatorCooling |
| [coil_bed_through_conductivity](/home/arseniy/projects/flying_chess/model.py:73) | `2.0` | Keep stack/material model; validate chosen construction | RadiatorCooling |
| [potting_cover_thickness](/home/arseniy/projects/flying_chess/model.py:74) | `0.2` | Keep stack/material model; validate chosen construction | model setup, AttitudeAuthority, HallSensing, MassBudget, SurfaceStack |
| [playing_surface_thickness](/home/arseniy/projects/flying_chess/model.py:75) | `0.1` | Keep stack/material model; validate chosen construction | model setup, AttitudeAuthority, HallSensing, SurfaceStack |
| [playing_surface_conductivity](/home/arseniy/projects/flying_chess/model.py:76) | `0.2` | Keep stack/material model; validate chosen construction | SurfaceStack |
| [piece_bottom_skin](/home/arseniy/projects/flying_chess/model.py:77) | `0.1` | Keep stack/material model; validate chosen construction | model setup, AttitudeAuthority, HallSensing, SurfaceStack |
| [surface_flatness_budget](/home/arseniy/projects/flying_chess/model.py:78) | `0.3` | Keep stack/material model; validate chosen construction | SurfaceStack |
| [max_touch_temperature](/home/arseniy/projects/flying_chess/model.py:79) | `77` | Keep limit; correct stale covered-cell explanation | CoilConfiguration, ConfigurationSweep, CoupledAuthority, RadiatorCooling, StatusChecks |
| [prolonged_touch_temperature](/home/arseniy/projects/flying_chess/model.py:80) | `48` | Keep limit; validate all required workloads | StatusChecks |
| [magnet_max_operating_temperature](/home/arseniy/projects/flying_chess/model.py:81) | `150` | Keep material limit; do not clamp away failure | RadiatorCooling, StatusChecks |
| [pcb_via_effective_thermal_conductivity](/home/arseniy/projects/flying_chess/model.py:82) | `10` | Keep stack/material model; validate chosen construction | RadiatorCooling |
| [thermal_pad_conductivity](/home/arseniy/projects/flying_chess/model.py:83) | `5.0` | Keep stack/material model; validate chosen construction | RadiatorCooling |
| [baseplate_thickness](/home/arseniy/projects/flying_chess/model.py:84) | `4.0` | Keep stack/material model; validate chosen construction | EddyDrag, RadiatorCooling |
| [aluminium_thermal_conductivity](/home/arseniy/projects/flying_chess/model.py:85) | `167` | Keep stack/material model; validate chosen construction | RadiatorCooling |
| [aluminium_density](/home/arseniy/projects/flying_chess/model.py:86) | `2700` | Keep stack/material model; validate chosen construction | RadiatorCooling |
| [aluminium_heat_capacity](/home/arseniy/projects/flying_chess/model.py:87) | `900` | Keep stack/material model; validate chosen construction | RadiatorCooling |
| [fin_height](/home/arseniy/projects/flying_chess/model.py:88) | `60` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [fin_thickness](/home/arseniy/projects/flying_chess/model.py:89) | `2` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [fin_channel_width](/home/arseniy/projects/flying_chess/model.py:90) | `8` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [natural_convection_coefficient](/home/arseniy/projects/flying_chess/model.py:91) | `2` | Recalculate cooling geometry/performance/cost | RadiatorCooling, SurfaceStack |
| [forced_convection_coefficient](/home/arseniy/projects/flying_chess/model.py:92) | `8` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [cooling_fan_size](/home/arseniy/projects/flying_chess/model.py:93) | `200` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [cooling_fan_speed](/home/arseniy/projects/flying_chess/model.py:94) | `550` | Metadata: no numerical fan-speed model | RadiatorCooling |
| [cooling_fan_airflow](/home/arseniy/projects/flying_chess/model.py:95) | `100.8` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [cooling_fan_static_pressure](/home/arseniy/projects/flying_chess/model.py:96) | `0.51` | Metadata: no pressure/flow feasibility check | RadiatorCooling |
| [cooling_fan_noise](/home/arseniy/projects/flying_chess/model.py:97) | `10.7` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [cooling_fan_installation_noise](/home/arseniy/projects/flying_chess/model.py:98) | `3` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [cooling_fan_airflow_fraction](/home/arseniy/projects/flying_chess/model.py:99) | `0.25` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [cooling_fan_power](/home/arseniy/projects/flying_chess/model.py:100) | `0.96` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [cooling_fan_mass](/home/arseniy/projects/flying_chess/model.py:101) | `0.37` | Recalculate cooling geometry/performance/cost | RadiatorCooling |
| [cooling_fan_price](/home/arseniy/projects/flying_chess/model.py:102) | `39.95` | Recalculate cooling geometry/performance/cost | BillOfMaterials |
| [cooling_fan_url](/home/arseniy/projects/flying_chess/model.py:103) | `supplier URL` | Recalculate cooling geometry/performance/cost | BillOfMaterials |
| [mosfet_voltage_rating](/home/arseniy/projects/flying_chess/model.py:104) | `60` | Keep driver baseline; resize/revalidate for new loads | CoilConfiguration, ConfigurationSweep, DiscreteDriver, StatusChecks |
| [min_mosfet_voltage_headroom](/home/arseniy/projects/flying_chess/model.py:105) | `1.5` | Keep driver baseline; resize/revalidate for new loads | ConfigurationSweep, StatusChecks |
| [driver_channel_current](/home/arseniy/projects/flying_chess/model.py:106) | `5.5` | Keep driver baseline; resize/revalidate for new loads | CoilConfiguration, DiscreteDriver, StatusChecks |
| [driver_pwm_frequency](/home/arseniy/projects/flying_chess/model.py:107) | `20000` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver, DriveMatrix |
| [driver_switching_time](/home/arseniy/projects/flying_chess/model.py:108) | `1e-07` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [driver_hot_resistance](/home/arseniy/projects/flying_chess/model.py:109) | `0.08` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [driver_mosfet_gate_charge](/home/arseniy/projects/flying_chess/model.py:110) | `3e-08` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [gate_drive_voltage](/home/arseniy/projects/flying_chess/model.py:111) | `10` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [logic_gate_voltage](/home/arseniy/projects/flying_chess/model.py:112) | `5` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [current_command_rate](/home/arseniy/projects/flying_chess/model.py:113) | `500` | Keep driver baseline; resize/revalidate for new loads | BoardControl, DiscreteDriver, DriveMatrix |
| [setpoint_filter_passive_price](/home/arseniy/projects/flying_chess/model.py:114) | `0.003` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [setpoint_filter_cutoff](/home/arseniy/projects/flying_chess/model.py:115) | `1000` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [max_setpoint_bits](/home/arseniy/projects/flying_chess/model.py:116) | `12` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [max_current_command_error](/home/arseniy/projects/flying_chess/model.py:117) | `0.005` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver, StatusChecks |
| [current_loop_bandwidth](/home/arseniy/projects/flying_chess/model.py:118) | `2000` | Keep driver baseline; resize/revalidate for new loads | Control, DiscreteDriver, DriveMatrix |
| [min_current_loop_pwm_cycles](/home/arseniy/projects/flying_chess/model.py:119) | `8` | Keep driver baseline; resize/revalidate for new loads | StatusChecks |
| [min_setpoint_serial_headroom](/home/arseniy/projects/flying_chess/model.py:120) | `1.2` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [current_sense_resistance](/home/arseniy/projects/flying_chess/model.py:121) | `0.02` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver, StatusChecks |
| [comparator_input_offset](/home/arseniy/projects/flying_chess/model.py:122) | `0.005` | Metadata: calibration is assumed, not simulated | DiscreteDriver |
| [current_sense_offset_residual](/home/arseniy/projects/flying_chess/model.py:123) | `0.001` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [max_current_offset_fraction](/home/arseniy/projects/flying_chess/model.py:124) | `0.05` | Keep driver baseline; resize/revalidate for new loads | StatusChecks |
| [current_shunt_price](/home/arseniy/projects/flying_chess/model.py:125) | `0.0418` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [current_shunt_power_rating](/home/arseniy/projects/flying_chess/model.py:126) | `2` | Keep driver baseline; resize/revalidate for new loads | StatusChecks |
| [current_comparator_channels_per_ic](/home/arseniy/projects/flying_chess/model.py:127) | `2` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [current_comparator_price](/home/arseniy/projects/flying_chess/model.py:128) | `0.0198` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [current_frontend_passives_per_channel](/home/arseniy/projects/flying_chess/model.py:129) | `4` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [current_frontend_passive_price](/home/arseniy/projects/flying_chess/model.py:130) | `0.0035` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [driver_gate_passive_price](/home/arseniy/projects/flying_chess/model.py:131) | `0.0012` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [driver_decoupling_price](/home/arseniy/projects/flying_chess/model.py:132) | `0.0197` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [shift_register_outputs](/home/arseniy/projects/flying_chess/model.py:133) | `8` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [gate_driver_half_bridges](/home/arseniy/projects/flying_chess/model.py:134) | `3` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [gate_driver_price](/home/arseniy/projects/flying_chess/model.py:135) | `0.2254` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [power_mosfet_price](/home/arseniy/projects/flying_chess/model.py:136) | `0.075` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [shift_register_clock_rating](/home/arseniy/projects/flying_chess/model.py:137) | `20000000` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [shift_register_power_capacitance](/home/arseniy/projects/flying_chess/model.py:138) | `4.2e-11` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [shift_register_price](/home/arseniy/projects/flying_chess/model.py:139) | `0.028` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [driver_serial_clock](/home/arseniy/projects/flying_chess/model.py:140) | `40000000` | Keep driver baseline; resize/revalidate for new loads | DiscreteDriver |
| [smt_assembly_cost_per_joint](/home/arseniy/projects/flying_chess/model.py:141) | `0.0017` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [max_bus_current](/home/arseniy/projects/flying_chess/model.py:142) | `120` | Keep driver baseline; resize/revalidate for new loads | StatusChecks |
| [bus_distribution_price](/home/arseniy/projects/flying_chess/model.py:143) | `36.96` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [rail_clamp_price](/home/arseniy/projects/flying_chess/model.py:144) | `5.0` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [rail_bulk_capacitor_price](/home/arseniy/projects/flying_chess/model.py:145) | `4.0` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [driver_channels_per_bulk_capacitor](/home/arseniy/projects/flying_chess/model.py:146) | `29` | Keep driver baseline; resize/revalidate for new loads | BoardControl |
| [bulk_capacitor_price](/home/arseniy/projects/flying_chess/model.py:147) | `0.25` | Keep driver baseline; resize/revalidate for new loads | BillOfMaterials |
| [supercap_cell_capacitance](/home/arseniy/projects/flying_chess/model.py:148) | `350` | Review buffer/split-rail hardware after load calculation | EnergyBuffer |
| [supercap_cell_max_voltage](/home/arseniy/projects/flying_chess/model.py:149) | `2.7` | Metadata: add working-voltage validation | EnergyBuffer |
| [supercap_cell_working_voltage](/home/arseniy/projects/flying_chess/model.py:150) | `2.4` | Review buffer/split-rail hardware after load calculation | EnergyBuffer |
| [supercap_cell_esr](/home/arseniy/projects/flying_chess/model.py:151) | `0.0032` | Review buffer/split-rail hardware after load calculation | EnergyBuffer |
| [supercap_eol_capacitance_fraction](/home/arseniy/projects/flying_chess/model.py:152) | `0.8` | Review buffer/split-rail hardware after load calculation | EnergyBuffer |
| [supercap_eol_esr_factor](/home/arseniy/projects/flying_chess/model.py:153) | `2.0` | Review buffer/split-rail hardware after load calculation | EnergyBuffer |
| [supercap_cell_price](/home/arseniy/projects/flying_chess/model.py:154) | `7.65` | Review buffer/split-rail hardware after load calculation | BillOfMaterials, EnergyBuffer |
| [supercap_cell_mass_kg](/home/arseniy/projects/flying_chess/model.py:155) | `0.065` | Review buffer/split-rail hardware after load calculation | EnergyBuffer |
| [supercap_balancer_price_per_cell](/home/arseniy/projects/flying_chess/model.py:156) | `0.15` | Review buffer/split-rail hardware after load calculation | BillOfMaterials, EnergyBuffer |
| [supercap_oring_price](/home/arseniy/projects/flying_chess/model.py:157) | `4.0` | Review buffer/split-rail hardware after load calculation | BillOfMaterials, EnergyBuffer |
| [supercap_management_price_per_rail](/home/arseniy/projects/flying_chess/model.py:158) | `20.0` | Review buffer/split-rail hardware after load calculation | BillOfMaterials, EnergyBuffer |
| [midpoint_balancer_price](/home/arseniy/projects/flying_chess/model.py:159) | `40.0` | Review buffer/split-rail hardware after load calculation | PowerSupply |
| [midpoint_balancer_mass_kg](/home/arseniy/projects/flying_chess/model.py:160) | `0.3` | Review buffer/split-rail hardware after load calculation | PowerSupply |
| [midpoint_balancer_current_rating](/home/arseniy/projects/flying_chess/model.py:161) | `5.0` | Review buffer/split-rail hardware after load calculation | PowerSupply, StatusChecks |
| [reset_choreography_demonstrated](/home/arseniy/projects/flying_chess/model.py:162) | `False` | Replace old manual flag with scenario evidence | StatusChecks |
| [bus_droop_fraction](/home/arseniy/projects/flying_chess/model.py:163) | `0.1` | Recalculate architecture/cooling/BOM assumption | EnergyBuffer, PowerSupply |
| [radiator_aluminium_price_per_kg](/home/arseniy/projects/flying_chess/model.py:164) | `12.0` | Recalculate architecture/cooling/BOM assumption | BillOfMaterials |
| [playing_surface_price](/home/arseniy/projects/flying_chess/model.py:165) | `12.0` | Recalculate architecture/cooling/BOM assumption | BillOfMaterials |
| [motor_pcb_price_per_cm2](/home/arseniy/projects/flying_chess/model.py:166) | `0.01063` | Recalculate architecture/cooling/BOM assumption | BillOfMaterials |
| [usable_bus_voltage_fraction](/home/arseniy/projects/flying_chess/model.py:167) | `0.9` | Recalculate architecture/cooling/BOM assumption | CoilConfiguration |
| [hall_sensor_pitch](/home/arseniy/projects/flying_chess/model.py:168) | `12.5` | Keep one default; avoid duplicate selected-pitch state | model setup, BoardControl |
| [hall_pitch_candidates](/home/arseniy/projects/flying_chess/model.py:169) | `(30.0, 25.0, 24.0, 20.0, 100 / 6, 16.0, 15.0, 100 / 7, 12.5, 12.0, 10.0)` | Recalculate candidates/selection for both modes | select_hall_pitch |
| [hall_observation_window_side](/home/arseniy/projects/flying_chess/model.py:170) | `4` | Keep sensing baseline; re-evaluate mode coverage/cost | model setup, HallSensing |
| [hall_sensor_mux_channels](/home/arseniy/projects/flying_chess/model.py:171) | `16` | Keep sensing baseline; re-evaluate mode coverage/cost | BoardControl |
| [hall_sensor_price](/home/arseniy/projects/flying_chess/model.py:172) | `0.34758` | Keep sensing baseline; re-evaluate mode coverage/cost | BillOfMaterials |
| [hall_sensor_mux_price](/home/arseniy/projects/flying_chess/model.py:173) | `0.2527` | Keep sensing baseline; re-evaluate mode coverage/cost | BillOfMaterials |
| [magnet_cost_per_kg](/home/arseniy/projects/flying_chess/model.py:174) | `384.0` | Keep sensing baseline; re-evaluate mode coverage/cost | BillOfMaterials |
| [hall_adc_sample_rate](/home/arseniy/projects/flying_chess/model.py:175) | `500000` | Keep sensing baseline; re-evaluate mode coverage/cost | BoardControl, HallSensing |
| [hall_adc_native_bits](/home/arseniy/projects/flying_chess/model.py:176) | `12` | Keep sensing baseline; re-evaluate mode coverage/cost | BoardControl, HallSensing |
| [hall_interpolation_bits](/home/arseniy/projects/flying_chess/model.py:177) | `14` | Keep sensing baseline; re-evaluate mode coverage/cost | BoardControl, StatusChecks, select_hall_pitch |
| [hall_package_standoff](/home/arseniy/projects/flying_chess/model.py:178) | `0.6` | Keep sensing baseline; re-evaluate mode coverage/cost | model setup |
| [hall_power_settle_time](/home/arseniy/projects/flying_chess/model.py:179) | `0.0001` | Keep sensing baseline; re-evaluate mode coverage/cost | BoardControl |
| [hall_gate_switch_price](/home/arseniy/projects/flying_chess/model.py:180) | `0.0206` | Keep sensing baseline; re-evaluate mode coverage/cost | BillOfMaterials |
| [thermal_sensor_area_mm2](/home/arseniy/projects/flying_chess/model.py:181) | `5000` | Keep sensing baseline; re-evaluate mode coverage/cost | BoardControl |
| [thermal_sensor_price](/home/arseniy/projects/flying_chess/model.py:182) | `0.0091` | Keep sensing baseline; re-evaluate mode coverage/cost | BillOfMaterials |
| [hall_supply_voltage](/home/arseniy/projects/flying_chess/model.py:183) | `5` | Keep sensing baseline; re-evaluate mode coverage/cost | BoardControl, HallSensing |
| [hall_supply_current](/home/arseniy/projects/flying_chess/model.py:184) | `0.01` | Keep sensing baseline; re-evaluate mode coverage/cost | BoardControl, HallSensing |
| [hall_sensitivity](/home/arseniy/projects/flying_chess/model.py:185) | `12.5` | Keep sensing baseline; re-evaluate mode coverage/cost | HallSensing |
| [hall_output_noise](/home/arseniy/projects/flying_chess/model.py:186) | `1.84e-05` | Keep sensing baseline; re-evaluate mode coverage/cost | HallSensing |
| [hall_linear_range](/home/arseniy/projects/flying_chess/model.py:187) | `0.169` | Keep sensing baseline; re-evaluate mode coverage/cost | HallSensing, StatusChecks, select_hall_pitch |
| [hall_sensor_bandwidth](/home/arseniy/projects/flying_chess/model.py:188) | `20000` | Keep sensing baseline; re-evaluate mode coverage/cost | HallSensing |
| [coil_field_subtraction_error](/home/arseniy/projects/flying_chess/model.py:189) | `0.005` | Keep sensing baseline; re-evaluate mode coverage/cost | HallSensing |
| [position_error_gap_fraction](/home/arseniy/projects/flying_chess/model.py:190) | `0.1` | Keep flight budget; define contact budget separately | HallSensing |
| [flight_position_error_gap_fraction](/home/arseniy/projects/flying_chess/model.py:191) | `0.25` | Keep flight requirement; validate sensing model | HallSensing |
| [pcb_copper_plane_thickness](/home/arseniy/projects/flying_chess/model.py:192) | `0.07` | Keep sensing baseline; re-evaluate mode coverage/cost | EddyDrag |
| [radiator_standoff_below_pcb](/home/arseniy/projects/flying_chess/model.py:193) | `1.3` | Keep stack/construction baseline; re-evaluate geometry/cost | BillOfMaterials, EddyDrag, MassBudget, RadiatorCooling |
| [radiator_slot_pitch](/home/arseniy/projects/flying_chess/model.py:194) | `5` | Keep stack/construction baseline; re-evaluate geometry/cost | EddyDrag |
| [radiator_slot_web_thickness](/home/arseniy/projects/flying_chess/model.py:195) | `0.5` | Keep stack/construction baseline; re-evaluate geometry/cost | EddyDrag |
| [radiator_slotting_price](/home/arseniy/projects/flying_chess/model.py:196) | `200.0` | Keep stack/construction baseline; re-evaluate geometry/cost | BillOfMaterials |
| [potting_epoxy_price](/home/arseniy/projects/flying_chess/model.py:197) | `45.0` | Keep stack/construction baseline; re-evaluate geometry/cost | BillOfMaterials |
| [gap_filler_pad_price](/home/arseniy/projects/flying_chess/model.py:198) | `336.43` | Keep stack/construction baseline; re-evaluate geometry/cost | BillOfMaterials |
| [gap_filler_density](/home/arseniy/projects/flying_chess/model.py:199) | `3.4` | Keep stack/construction baseline; re-evaluate geometry/cost | MassBudget |
| [windings_per_coil_body](/home/arseniy/projects/flying_chess/model.py:200) | `1` | Keep stack/construction baseline; re-evaluate geometry/cost | CoilBed |
| [pcb_thickness](/home/arseniy/projects/flying_chess/model.py:201) | `1.6` | Keep stack/construction baseline; re-evaluate geometry/cost | model setup, EddyDrag, MassBudget, RadiatorCooling |
| [frame_enclosure_mass_kg](/home/arseniy/projects/flying_chess/model.py:202) | `1.0` | Keep inventory/cost assumption; recalculate quantities | MassBudget |
| [board_electronics_mass_kg](/home/arseniy/projects/flying_chess/model.py:203) | `0.3` | Keep inventory/cost assumption; recalculate quantities | MassBudget |
| [blocker_hops_per_gap_move](/home/arseniy/projects/flying_chess/model.py:204) | `4` | Review blocker policy; use feasible move schedule | RadiatorCooling |
| [adjacent_hover_crowding_factor](/home/arseniy/projects/flying_chess/model.py:205) | `2.47` | Remove: obsolete report-only narrative | RadiatorCooling |
| [landing_crowding_factor](/home/arseniy/projects/flying_chess/model.py:206) | `1.83` | Remove with airborne reset landing heat | RadiatorCooling |
| [cruise_heat_spread_factor](/home/arseniy/projects/flying_chess/model.py:207) | `4.0` | Replace or validate from scenario heat footprints | RadiatorCooling |
| [potting_density](/home/arseniy/projects/flying_chess/model.py:208) | `1.8` | Keep inventory/cost assumption; recalculate quantities | MassBudget |
| [bus_distribution_mass_kg](/home/arseniy/projects/flying_chess/model.py:209) | `0.8` | Keep inventory/cost assumption; recalculate quantities | MassBudget |
| [ac_input_price](/home/arseniy/projects/flying_chess/model.py:210) | `8.0` | Keep inventory/cost assumption; recalculate quantities | BillOfMaterials |
| [emi_filter_price](/home/arseniy/projects/flying_chess/model.py:211) | `12.0` | Keep inventory/cost assumption; recalculate quantities | BillOfMaterials |
| [enclosure_price](/home/arseniy/projects/flying_chess/model.py:212) | `40.0` | Keep inventory/cost assumption; recalculate quantities | BillOfMaterials |
| [piece_control_flops](/home/arseniy/projects/flying_chess/model.py:213) | `20000` | Keep inventory/cost assumption; recalculate quantities | BoardControl |
| [setpoint_dma_words_flops](/home/arseniy/projects/flying_chess/model.py:214) | `4` | Keep inventory/cost assumption; recalculate quantities | BoardControl |
| [board_setpoint_fpga_count](/home/arseniy/projects/flying_chess/model.py:215) | `1` | Recalculate control workload, hardware and cost | BoardControl |
| [board_setpoint_fpga_price](/home/arseniy/projects/flying_chess/model.py:216) | `25.0` | Recalculate control workload, hardware and cost | BillOfMaterials |
| [board_setpoint_fpga_power](/home/arseniy/projects/flying_chess/model.py:217) | `1.0` | Recalculate control workload, hardware and cost | PowerSupply |
| [board_setpoint_fpga_solder_joints](/home/arseniy/projects/flying_chess/model.py:218) | `144` | Recalculate control workload, hardware and cost | BillOfMaterials |
| [control_node_adc_engines](/home/arseniy/projects/flying_chess/model.py:219) | `2` | Recalculate control workload, hardware and cost | BoardControl, HallSensing |
| [min_hall_scan_headroom](/home/arseniy/projects/flying_chess/model.py:220) | `1.1` | Recalculate control workload, hardware and cost | BoardControl |
| [min_control_compute_headroom](/home/arseniy/projects/flying_chess/model.py:221) | `2.0` | Recalculate control workload, hardware and cost | BoardControl |
| [control_node_mcu_price](/home/arseniy/projects/flying_chess/model.py:222) | `3.13` | Recalculate control workload, hardware and cost | BillOfMaterials |
| [control_node_mcu_solder_joints](/home/arseniy/projects/flying_chess/model.py:223) | `32` | Recalculate control workload, hardware and cost | BillOfMaterials |
| [node_mcu_throughput_mflops](/home/arseniy/projects/flying_chess/model.py:224) | `170` | Recalculate control workload, hardware and cost | BoardControl |
| [control_node_mcu_power](/home/arseniy/projects/flying_chess/model.py:225) | `0.4` | Recalculate control workload, hardware and cost | PowerSupply |
| [host_power](/home/arseniy/projects/flying_chess/model.py:226) | `8` | Recalculate control workload, hardware and cost | PowerSupply |
| [psu_sizing_margin](/home/arseniy/projects/flying_chess/model.py:227) | `1.1` | Recalculate control workload, hardware and cost | PowerSupply |
| [psu_family](/home/arseniy/projects/flying_chess/model.py:228) | `'Mean Well UHP-2500-36 adjusted to 40V'` | Recalculate supply selection/cost after new load traces | PowerSupply |
| [psu_rating](/home/arseniy/projects/flying_chess/model.py:229) | `2498.4` | Recalculate supply selection/cost after new load traces | PowerSupply |
| [psu_price](/home/arseniy/projects/flying_chess/model.py:230) | `589.0` | Recalculate supply selection/cost after new load traces | PowerSupply |
| [psu_mass_kg](/home/arseniy/projects/flying_chess/model.py:231) | `4.07` | Recalculate supply selection/cost after new load traces | PowerSupply |
| [psu_url](/home/arseniy/projects/flying_chess/model.py:232) | `supplier URL` | Recalculate supply selection/cost after new load traces | PowerSupply |

**Constants**

| Parameter | Current value | Disposition | Direct consumers |
|---|---|---|---|
| [board_squares_per_side](/home/arseniy/projects/flying_chess/model.py:236) | `8` | Keep physical/material constant or board definition | BoardGeometry, EddyDrag, Propulsion |
| [ndfeb_remanence_br](/home/arseniy/projects/flying_chess/model.py:237) | `1.36` | Keep physical/material constant or board definition | model setup |
| [ndfeb_br_tempco](/home/arseniy/projects/flying_chess/model.py:238) | `-0.0011` | Keep physical/material constant or board definition | CoilConfiguration, CoupledAuthority, RadiatorCooling |
| [ndfeb_density](/home/arseniy/projects/flying_chess/model.py:239) | `0.0075` | Keep physical/material constant or board definition | model setup, HalbachArray |
| [plastic_density](/home/arseniy/projects/flying_chess/model.py:240) | `0.0012` | Keep physical/material constant or board definition | model setup, Piece |
| [copper_resistivity](/home/arseniy/projects/flying_chess/model.py:241) | `1.724e-08` | Keep physical/material constant or board definition | CoilConfiguration, EddyDrag |
| [copper_resistivity_tempco](/home/arseniy/projects/flying_chess/model.py:242) | `0.00393` | Keep physical/material constant or board definition | CoilConfiguration, RadiatorCooling |
| [aluminium_resistivity](/home/arseniy/projects/flying_chess/model.py:243) | `2.82e-08` | Keep physical/material constant or board definition | EddyDrag |
| [copper_density](/home/arseniy/projects/flying_chess/model.py:244) | `8960` | Keep physical/material constant or board definition | WireThermal |
| [winding_conductor_thicknesses](/home/arseniy/projects/flying_chess/model.py:245) | `[0.05, 0.07, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5]` | Replace/reclassify as design candidate data | ConfigurationSweep |
| [standard_bus_voltages](/home/arseniy/projects/flying_chess/model.py:246) | `[24, 30, 40, 48]` | Replace/reclassify as design candidate data | ConfigurationSweep |
| [gravity](/home/arseniy/projects/flying_chess/model.py:247) | `9.80665` | Keep physical/material constant or board definition | model setup, AttitudeAuthority, CoupledAuthority, EddyDrag, Piece, Propulsion |
| [vacuum_permeability](/home/arseniy/projects/flying_chess/model.py:248) | `1.25663706e-06` | Keep physical/material constant or board definition | Control, EddyDrag |
| [fr4_density](/home/arseniy/projects/flying_chess/model.py:249) | `0.00185` | Keep physical/material constant or board definition | MassBudget |

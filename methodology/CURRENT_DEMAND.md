**King current-demand investigation — 6 October 2026**

The corrected king model and position-dependent coil allocation substantially reduce the previously reported current demand. The useful result is now a provisional operating region around 3.3–3.5 A per coil for the sampled level-flight and handoff cases, using the current 57-turn winding. This is not a product-wide minimum or a hardware rating: full trajectories, stationary neighbours, hot operation, correction reserve and a supplier-confirmed winding remain outstanding.

`methodology/design.py` uses the corrected mass distribution and retains the all-direction preliminary winding screen as an optional comparison. The adopted winding is explicit in `model.py`. Its default report still uses fixed-window allocation. The moving-window, edge and smooth-handoff policies are reproducible diagnostic candidates in `current_demand.py`, built on the existing simulator and optimizers. They have not been used to reduce the default BOM or to declare the design accepted.

**King mass and inertia**

The user confirmed the king as the reference piece and requested the existing simple geometric approximation, with the magnet weight correctly placed at the bottom. The shell is still the uniformly scaled hollow bounding cylinder; its mass has not been reduced to obtain a better result.

| Quantity | Updated approximation |
|---|---:|
| Bounding height / base diameter | 95.614 / 44.284 mm |
| Plastic shell mass | 11.383 g |
| Bottom magnet mass | 12.000 g |
| Total mass | 23.383 g |
| COM above magnet bottom | 24.299 mm |
| Roll/pitch inertia about COM | 2.6750 × 10⁻⁵ kg·m² |
| Yaw inertia about COM | 5.6389 × 10⁻⁶ kg·m² |

COM is the mass-weighted combination of the shell centroid and magnet-array centroid. Shell inertia is the difference between outer and inner cylinder inertias, scaled by the same shape factor as shell mass. Magnet inertia comes from the contiguous square array, and roll/pitch uses the parallel-axis theorem for both components. `Fixed.com_height_fraction` has been removed. `Piece` remains the single source of mass, COM and inertia for the simulator.

This is an internally consistent approximation, not a measured king silhouette. The full model now estimates a nominal unstable growth rate of 48.043/s and a pose-update screening rate of 240.2 Hz. The latter is still a heuristic, not a validated minimum frequency.

**Current results**

The updated preliminary screen checks all eight configured horizontal directions. It selects 57 turns of nominal 0.956 × 0.05 mm bare rectangular copper, 4.218 mm winding height and 0.9457 Ω per coil. Insulation and winding clearance remain assumed. Acceleration remains 1.9613 m/s², modeled peak speed 1.1538 m/s, and weight 0.22931 N. No motion requirement has been relaxed.

| Calculation | Peak coil current | Coverage |
|---|---:|---|
| Fixed window, hover | 1.758 A | Existing 432 level-flight direction/pose cases, re-optimized for hover |
| Fixed window, acceleration without drag | 4.702 A | Same cases |
| Fixed window, acceleration and drag | 4.855 A | Same cases; default report |
| Nearest moving window, acceleration and drag | 3.260 A | 3,072 cases across the combined lattice period |
| Complete windows shifted inward at motor edges | 3.187 A | 384 corner/edge-midpoint direction/pose cases |
| Smooth overlap handoff probe | 3.508 A | Worst interior case's gap, yaw and force target; both translation axes and every handoff plane |

The fixed-window worst-case minimum-loss solution dissipates about 468 W in copper. The moving-window sweep's worst minimum-loss solution dissipates about 197 W. These are instantaneous sampled operating losses at their respective current limits, not continuous board-power predictions or duty-weighted RMS results. A blended handoff can use additional coils; its power and whole-trajectory heating still need to be included in the workload model.

Increasing segment/radial/axial resolution at the fixed-window worst pose changes 4.8549 A to 4.8488 A, about 0.13%. This does not establish spatial or yaw-grid convergence.

For comparison, the earlier 7.070 A result used the independent 38.246 mm COM assumption, a fixed coil window and a 51-turn winding. In that earlier geometry, correcting COM alone gave 5.040 A at the old worst pose. The new 4.855 A result also includes winding reselection and the all-direction preliminary screen; its entire improvement should not be attributed to COM alone.

**Physical coil selection and handoff**

Coils now have persistent identities consisting of orientation and integer lattice indices. Changing the window centre or size preserves the actual grid, including the larger shared reset window. The two coil pitches have a combined 30 mm repeat. The moving-window diagnostic samples that complete repeat, with handoff planes included. Each interior window contains 24 coils.

A naive edge policy that discards coils outside the motor footprint left only 14–20 coils and reached 5.346 A in the initial probe. The implemented edge policy instead shifts the full window inward while retaining the same physical lattice. All sampled edge windows now contain 24 valid coils, with the 3.187 A maximum above. This is a selected-pose result, not proof of continuous edge coverage or compatibility with neighbouring pieces.

Independently optimized nearest windows can demand an abrupt change in physical-channel current. The candidate handoff blends overlapping window solutions at the same piece pose using smooth weights that sum to one. Because each component solution supplies the same wrench and has zero summed current, the blend preserves both constraints. Its channel current cannot exceed the common component limit. Switching between active coil sets while decoupling force and torque is consistent with established planar-actuator commutation work; see [van Lierop et al., 2008](https://www.jstage.jst.go.jp/article/ieejias/128/12/128_12_1333/_article/-char/en/). The particular blend implemented here is our prototype, not a reproduction or validation of that paper's controller.

| Distance between comparison poses across a handoff | Largest abrupt channel change | Largest blended channel change |
|---|---:|---:|
| 2 mm | 4.117 A | 4.117 A |
| 0.2 mm | 4.185 A | 2.086 A |
| 0.02 mm | 3.508 A | 0.194 A |

These maxima are across the handoff probes at each spacing. The shrinking blended change is evidence of continuity in the tested region; the abrupt change does not shrink. Both policies use the same 3.508 A component-current limit. The blended force/torque and current-sum residuals are below 2 × 10⁻¹³ in their respective units. The probes use up to 28 coils; a separate crossed-handoff geometry regression uses 32 physical coils. Full-path current, power and simultaneous-handoff dynamics remain unverified.

**Why frequency cannot be frozen yet**

The prototype blend width is 1.25 mm. At the modeled peak speed it lasts 1.083 ms. The previous 500 Hz current-command baseline allowed an entire transition between commands. The candidate now uses 4 kHz, giving about 4.33 command updates per transition. A high PWM carrier or a high-rate delta-sigma output does not itself generate the missing intermediate current targets.

At the finest handoff spacing, the largest blended finite-difference current slew is about 11,200 A/s; the abrupt selection gives about 202,400 A/s and grows as the spacing shrinks. These are local waveform diagnostics, not established global slew limits.

The dynamics calculation now distinguishes pose feedback, motion-dependent current-pattern generation, inner current regulation and PWM. `qualification.py` tests local hover recovery and fast flight segments with current lag, voltage saturation, sensing delay/error and correction-current reserve. See `QUALIFICATION.md` and `results/qualification.json` for its scope and results. The 240 Hz pose estimate remains a heuristic; complete trajectories and stationary-neighbour compatibility remain outstanding.

Peak current and update rates drive much of the hardware, but winding turns, resistance, self/mutual inductance, required current slew, simultaneously active channels and duty-weighted loss determine the voltage, drivers, sensing and cooling that can realize them. Those quantities must be selected together.

**Wire availability**

The present exact winding is still unconfirmed. [MWS's shaped-wire manufacturing page](https://mwswire.com/shaped-magnet-wire/) describes custom film-insulated ribbon wire with thicknesses starting around 0.001 inch. This supports the plausibility of thin ribbon as a manufacturing route; it does not confirm our exact width/thickness combination, finished insulation dimensions, tolerances, bend radius, price or availability. Those details are needed before a winding is fixed. No supplier was contacted and no quotation is implied.

**Movement workload**

The legacy 60 composite moves/min figure came from T2 of `PRODUCT_TESTING.md`, introduced in commit `6ef6f97`. The user has now reduced this to 10 composite moves/min. This is board-wide sequential chess-move throughput, including additional flights for captures and clearing/re-closing a knight's path. It is separate from the full-set contact reset.

The existing timing gives 2.5 s per elementary flight and an average 1.85 flights per composite chess move, hence 4.625 s per composite move, or 12.97/min. The new target therefore fits the serial schedule. `Inputs.sustained_moves_per_minute` owns the fan-assisted target; T2 now references the model instead of duplicating the rate. The passive workload already specifies one composite move per 6 s. Individual flight timing, acceleration and peak speed are unchanged. Average fan-assisted movement power drops by a factor of six relative to the old requested workload; peak flight current, reset bursts and handoff timing do not change.

**Next modifications to the existing model**

The user prefers a conservative sufficient hardware specification over optimization of the absolute minimum power or frequency. The next work should therefore validate one generously rated candidate, adjusting it only if a concrete check fails.

1. Specify the candidate using the existing winding and driver baseline: per-channel peak and sustained current, voltage and current slew, active-channel count, pose/command timing and maximum delay, sensing accuracy, and burst/average power. Put any new quantitative assumptions and uncertainty bounds in the three model parameter classes. Confirm the actual wire build so these ratings refer to a realizable winding.
2. Extend the existing six-axis linearization with sampled feedback, current dynamics and moving-window commutation. Check the candidate against difficult routes, handoffs, edges, stationary neighbours, hot operation and bounded mass/COM/sensing uncertainty. Check correction reserve and saturation. This is a bounded feasibility exercise, not a search for the lowest passing frequency. The present sampled results alone do not justify assigning a probability of failure or guaranteeing control from watts and hertz alone.
3. If that candidate passes with margin, use it to size the existing electronics and thermal architecture. Validate coil force, heating, sensing and current response on a small physical coil patch before treating the model's conditional assurance as hardware evidence. The full-set sliding reset remains a separate shared-coil/power requirement.

**Reproduction and verification**

Run `python -m methodology.current_demand` using the environment in `README.md`; it writes `methodology/results/current_demand.json` with source fingerprints and the comparisons above. The saved static diagnostic predates the latest dynamic-controller and PWM-carrier changes; its recorded fingerprints identify that snapshot. Its force/current results remain applicable because the selected winding, piece geometry and individual flight motion are unchanged. Updated dynamic results and switching losses are in `results/qualification.json` and the root `last_run.txt`. `python model.py` writes the default report to `last_run.txt`. Numerical/search settings are in `Inputs`, `Fixed` or `Constants`, including the new phase samples, handoff offsets and blend-width fraction; parameter comments are retained.

A numerical issue at an exactly saturated handoff was traced to the LP objective reporting a peak infinitesimally below its returned physical current vector. The allocator now reports the maximum magnitude of that vector. This lets the existing loss optimizer use a consistent feasible limit; no engineering margin, tolerance relaxation, solver fallback or alternate optimization model was introduced.

The current-demand calculation completed, followed by targeted edge-policy and smooth-handoff recalculation. Regression coverage includes mass limits, physical lattice identity, whole-coil edge placement, crossed-window blending, negative-direction screening, current-limit consistency and parameter ownership. The workload revision changes the timing regression to require the newly approved rate to fit the serial schedule. Thermal, endurance and outstanding physical/dynamic checks still govern product acceptance; numerical regression success does not imply an accepted board design.

After the workload revision, all 32 regression tests pass and the full default model completes. Both throughput checks now pass. The fixed-window fan-assisted sustained movement-power estimate falls from 1,516.4 W to 252.7 W, and its modeled sustained plate temperature falls from 104.3 °C to 44.2 °C. Passive cooling still gives 78.6 °C; local material/touch and hover-endurance failures also remain. These are model predictions using the existing conservative fixed-window workload, not measured performance or a recalculation using blended trajectory losses.

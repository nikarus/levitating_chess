# NEXT PLAN — fix and optimization order

Three kinds of work, in this order:
1. **Model-correctness fixes** — cheap, don't move the design point, but change the numbers. Do these first or every later optimization is tuned against a wrong model.
2. **Design-point movers** — geometry and requirement changes that cascade through everything (tile count, power, thermal, BOM). Batch them, apply once, re-baseline once.
3. **Expensive verification** — multi-piece sims, 6-DoF dynamics, Monte Carlo. Do these last, on the settled design point, so they never run twice.

Dependency chain: `0 -> 1 -> 2 -> {3 || 4} -> 5a -> 6 -> 5`.
REORDERED (cost-first decision): feasibility reds get fixed cheaply (5a), then the cost sweep (Phase 6) runs BEFORE the expensive verification (Phase 5), so verification runs once on the settled cheap design point.
**After each phase: re-run the model and re-evaluate everything — new issues are expected to surface.**

## Phase 0 — Fix the model's honesty (no design change) — DONE

- Hall noise: replace `1/sqrt(oversampling)` with bandwidth-correct averaging (samples within the sensor's 20 kHz bandwidth are correlated).
- Thermal: add the two-layer coil-bed conduction resistance to the source path; make the source node transient (RC, bed/potting heat capacity) instead of steady-state resistance under full pulse power; add a local worst-phase check (one piece at worst grid phase through its own footprint); orientation-realistic convection (fins-down over a table).
- Add the setpoint-stream compute load to the tile-MCU check (placeholder constant until Phase 4 picks the architecture) so the compute gate can actually fail.
- Hygiene: BOM piece count from `captured_pieces_total`; PSU zone texts derived from actual zone count; relabel the tilt cap as conservative (it measures extra flight gap, not physical rim touch — kept by decision, 2.6 deg target stands).

## Phase 1 — Move the design point (geometry) — DONE

- Square = 60 mm = exactly 3 magnet periods, phase-aligned square centers. Check added that square pitch is an integer multiple of the magnet period and both coil pitches (60 = 3x20 = 4x15 = 6x10), so every square center has an identical, best-phase electromagnetic environment.
- Storage stays FULLY MOTORIZED (decision): it is a staging area — at game start all figures fly simultaneously from there to their designated squares (C2). Storage slots are laid on the same phase-aligned 60 mm grid (2 columns x 8 rows per side), so stored pieces rest at best phase and no longer touch.

## Phase 2 — Requirement decisions (product-level) — DONE

- C2 UNCHANGED (decision): all 32 pieces move simultaneously, no staggering, ever.
- C3 UNCHANGED (decision): full 6-DoF for every airborne piece in all cases.
- Added C6 to PRODUCT_VISION.md: firmware thermal governor (two-node RC admission control) replaces the old 2 s hover / 60 s cooldown timers. Choreography rules: slide-to-phase takeoff (0.5 s), hover waits only at lattice points.
- Defined the worst-case sizing scenario ("game-reset stress event"): all 32 lift from adversarial worst-phase positions (0.5 s at worst-phase power), cruise 1.5 s phase-averaged, wait 2 s + land 0.5 s at aligned points; pre-warmed by continuous play (1 move / 6 s); two events back-to-back, then one per 5 min forever.
- Model now sizes thermals from this piecewise event profile and reports the reset burst energy the PSU cannot cover (feeds Phase 3 supercap sizing). Installation constraint (feet + under-board airflow) added to the vision.
- REVISED after worst-case review: sizing reduced to three canonical envelopes matching the hardware time constants — W1 Burst (reset, PSU+buffer), W2 Grind (real-time replay at 60 composite moves/min, radiator; replay lag is NOT acceptable — decision), W3 Hammer (one cell, dwell every 5 s forever + 10-exchange cascade burst). C7 added to the vision. Fan policy: 2 very silent fans, spectate mode only; silent mode (live play) stays fully passive and must pass all checks on its own.

## Phase 3 — Power & thermal architecture (sized on Phase 1+2 numbers) — DONE

- Supercap burst buffer sized from the W1 deficit (shared per-rail banks, ideal-diode ORing across zones); PSU stays 6x UHP-500-12; buffer + recharge checks green.
- HALBACH BUG found during this phase: `snap_to_axis` tie-broke 45-degree ideal angles on float dust — the sim's array was never a Halbach. Fixed to canonical per-block phases; aligned hover power roughly doubled; full physics audit then cross-verified every big link (force reciprocity 0.03%, analytic field, Br linearity, RC limits).
- Magnet grade: N48SH (Br 1.40) over N52 — worst cell soaks resting-piece magnets.
- Thermal limits reworked to external-first (decision): internal coil bed <= 105 C material limit; external surface governs. TOUCH SAFETY (new C7): a hand can lift any piece at any moment, so 77 C brief-touch is a HARD per-cell cap at all times — governor-enforced, model-gated. Cascade modelled RC-drained with serialized (1 s/exchange) hover; worst cell ever 72.7 C, 4.3 K margin. Radiator kept as-is (replay grind owns it).

## Phase 4 — Close the electronics (parallel with Phase 3) — DONE

- Setpoint stream: per-tile FPGA (GW1N-1/iCE40UL class, ~$2.20) runs 115 first-order delta-sigma modulators in fabric; MCU DMA-streams 12-bit setpoints (0.23 Mflop/s vs the 250 Mflop/s software placeholder). Tile compute headroom 12.2x, check green.
- Bipolar sensing: coil legs return to the split-rail midpoint; 0.1% thin-film level-shift dividers put the +/-shunt signal in the LM393 common-mode range; firmware idle zero-calibration removes static offsets, 1 mV drift residual -> 50 mA offset error (limit 275 mA), check green.
- Shunt right-sized 50 -> 20 mR: worst-case dissipation passes 50% derating; fleet sense loss cut 2.5x.

## Phase 5a — Feasibility reds (cheap, analytic, gates the cost sweep) — DONE

- Replaced the flat 0.2 g worst-pose gate with scenario-derived requirements. Sim now exports level-pose (nominal-gap) coupled authority separately from the all-pose worst. Cruise gate: full diagonal in the 1.5 s window at level/nominal-gap pose, margin 1.15x. Slide-to-phase gate: half-square in 0.5 s at any pose, margin 1.98x. Eddy drag charged exactly via closed-form bang-bang-with-linear-drag thrust (drag helps braking; net effect near-neutral). ALL CHECKS GREEN at $4,304.
- TILT REWORK (decision): the 10 deg tilt target removed everywhere; the rule is now "lowest rim point keeps >= 1 mm above the playing surface" (geometric cap 8.0 deg at the showpiece gap). Transport envelopes (reset/cascade/thermal/slide/cruise) priced at level poses only — pieces never fly tilted; tilt is a stationary showpiece with its own rung ladder (100%/50% of cap). Selected rung: 4.0 deg (lift 1.74x, torque 2.25x); 8 deg is geometrically legal but not force-affordable with current coils. Slide margin improved to 3.71x once priced at level poses. ALL CHECKS GREEN at $4,243.

## Phase 6 — Cost-first re-optimization (pulled before verification) — IN PROGRESS

- Sweep gates on ALL checks; minimize BOM subject to green-everything.
- DONE Hall thinning: pitch is now cost-selected (sparsest candidate passing observability, noise, saturation, scan and tilt-resolution gates). Selected 14.29 mm (7/side, 49 sensors/tile vs 64): position error 251 of 350 um budget, condition 1448, scan headroom 1.79x. Saved $208 -> $4,034 all green. Sparser candidates (25/20/16.7 mm) fail the gates.
- REJECTED storage-lite tiles: C3 requires full 6-DoF for all pieces in all cases, storage flights included.
- REJECTED driver-matrix sharing at part level: a back-to-back selector FET pair + drive per coil costs what the shared channel saves; revisit only if channel electronics get expensive again.
- PSU bank already self-optimizes (unit count x buffer trade in PowerSupply); binds on full-fleet single-zone hover capacity.
- NEXT: gate-driver cost (EG2134 $352/board — topology or RFQ), assembly-joint count ($324), radiator slotting + gap filler RFQ realism ($350), cost-model fidelity pass on [TO BE SOURCED] lines.

## Phase 5 — Verification models (expensive, once, LAST, on the settled cheap design) — DONE (verification.py)

- Multi-piece: two adjacent pieces at 60 mm hover with 2.8x budget margin; 3x3 lattice (worst crowding) peak 75 At vs 182 budget (2.42x). Crowding costs 2.33x hover power per fully-surrounded piece — folded into the reset landing segment as a 1.75x fleet average (2x8 home blocks); contention waits are spread by the governor (no crowding). Re-run stays all green at $4,034.
- Dynamic C5: energy-optimal commutation pulls a resting neighbour with ~0.1 mN vs 133 mN friction hold (>1000x margin); earlier scare was a solver artifact (min-peak spreads current under the neighbour; real firmware is minimum-norm).
- Latency stability: total loop delay 5.4 ms vs 21.9 ms instability growth time = 4.1x (>= 2 needed).
- Tolerance Monte Carlo (Br 1.5%, magnetization 1 deg, coil 0.2 mm, gap 0.15 mm; 24 samples): worst-phase hover margin min 3.39x vs 2x target.
- Remaining for hardware phase: derived Hall calibration residual (still an assumed 0.5%), per-zone rail balance on a real trajectory, full 6-DoF closed-loop sim. These are bench-test items on the 3x3 patch (C6 test article).

## Phase 6-old — absorbed into the reordered Phase 6 above

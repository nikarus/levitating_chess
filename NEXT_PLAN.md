# NEXT PLAN — fix and optimization order

Three kinds of work, in this order:
1. **Model-correctness fixes** — cheap, don't move the design point, but change the numbers. Do these first or every later optimization is tuned against a wrong model.
2. **Design-point movers** — geometry and requirement changes that cascade through everything (tile count, power, thermal, BOM). Batch them, apply once, re-baseline once.
3. **Expensive verification** — multi-piece sims, 6-DoF dynamics, Monte Carlo. Do these last, on the settled design point, so they never run twice.

Dependency chain: `0 -> 1 -> 2 -> {3 || 4} -> 5 -> 6`.
The only cross-dependency: Phase 3's magnet-grade change must land before Phase 5's verification runs.
**After each phase: re-run the model and re-evaluate everything — new issues are expected to surface.**

## Phase 0 — Fix the model's honesty (no design change) — DONE

- Hall noise: replace `1/sqrt(oversampling)` with bandwidth-correct averaging (samples within the sensor's 20 kHz bandwidth are correlated).
- Thermal: add the two-layer coil-bed conduction resistance to the source path; make the source node transient (RC, bed/potting heat capacity) instead of steady-state resistance under full pulse power; add a local worst-phase check (one piece at worst grid phase through its own footprint); orientation-realistic convection (fins-down over a table).
- Add the setpoint-stream compute load to the tile-MCU check (placeholder constant until Phase 4 picks the architecture) so the compute gate can actually fail.
- Hygiene: BOM piece count from `captured_pieces_total`; PSU zone texts derived from actual zone count; relabel the tilt cap as conservative (it measures extra flight gap, not physical rim touch — kept by decision, 2.6 deg target stands).

## Phase 1 — Move the design point (geometry, batched)

- Square = 60 mm = exactly 3 magnet periods, phase-aligned square centers; capture buffer laid out on-period. Add a check that square pitch is an integer number of periods.
- De-motorize the storage: narrow transfer lane + passive parking instead of full coil/sensor coverage under buffers (~30% area cut).

## Phase 2 — Requirement decisions (product-level, cheap to apply)

- Amend C2: staggered reset in N waves (`pieces_levitating_simultaneously` = 8 or 16); "all at once" becomes "reset completes in <= X s".
- Amend C3 if desired: full 6-DoF for the hero move; reduced authority during reset.
- Promote the hover-duty rule (max hover per spot + spot cooldown) into PRODUCT_VISION.md as an explicit constraint.

## Phase 3 — Power & thermal architecture (sized on Phase 1+2 numbers)

- Supercap burst buffer + small PSU: worst reset wave energy, cap bank sizing, charge-rate limits, new BOM lines.
- Radiator resized for the realistic duty profile (likely thinner plate as thermal mass, fewer/no fins, cheaper slotting, less gap filler).
- Magnet grade decision (N45SH/N48SH vs N52) from corrected hotspot temperature; update Br and re-run.

## Phase 4 — Close the electronics (parallel with Phase 3)

- Bipolar current sensing: real topology (midpoint-referenced shunt or bidirectional sense amp), schematic, updated BOM costs, offset error and loss formulas.
- Setpoint stream: LUT+DMA on MCU vs small CPLD vs PWM-DAC parts; replace placeholder compute constant; update BOM.

## Phase 5 — Verification models (expensive, once, on the settled design)

- Multi-piece simulation: global wrench for two adjacent pieces, four across a tile boundary, one reset wave; active-coil forces on neighbouring resting pieces (dynamic C5); per-zone rail balance on a real trajectory.
- 6-DoF dynamics: 6x6 stiffness with the selected coil height; current-loop + Hall-gating + setpoint latencies in one stability statement.
- Sensing validation: multi-piece interference, full-thrust coil fields, derived (not assumed) calibration residual.
- Tolerance Monte Carlo (Br, magnetization direction, coil placement, gap, flatness, current error). Targets after tolerances: >= 1.5x coupled lateral, >= 2x lift.

## Phase 6 — Re-optimize and re-issue the BOM

- Sweep gates on ALL checks (coupled authority, sensing, stability, thermal); Pareto frontier over BOM / mass / peak PSU / heat / tolerance margin instead of minimum electrical power.
- Optional branches if Phase 5 allows: Hall density/cost reduction, driver-sharing matrix, ferrite back-plane variant.

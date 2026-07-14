# GRAND PLAN

Ordering rule: anything that can still move power or authority numbers goes before cost work;
cost work goes before integration debts that do not feed back into physics.

## Phase 1 - close the physics that still moves numbers

1. [DONE 2026-07-14] Split-rail zero-net-current constraint (#6)
   - Zero-net-current equality row added to both commutation LPs (hover + authority), all margins re-verified.
   - Midpoint carries only the comparator-offset residual (RSS ~2.8A); balancer re-specced as a 10A
     synchronous half-bridge midpoint regulator [TO BE SOURCED] instead of the $46 battery-equalizer placeholder.
   - Cost of the constraint: verified lift 1.81 -> 1.63x, coupled lateral 0.231 -> 0.221g (still >= 0.2),
     tilt 5.5 -> 3.7x; hover power unchanged; board $3,018 -> $2,983. All 50 checks OK.
   - Still open from #6: boot-time per-channel offset calibration is assumed for the residual estimate
     (RSS of uncalibrated offsets is the conservative bound used).
2. [DONE 2026-07-14] Hall sensing rebuild (#5)
   - Sensor plane modeled at its real location: 12.36mm below magnets (gap + 2 coil layers + PCB + package).
     Signal 18.7mT, worst-case field 22.5mT << 100mT range - no saturation, rank 6 at all 216 poses.
   - Full error chain: sensor noise + ADC quantization through worst-pose Jacobian = 68um noise;
     coil field at sensors (3.1mT) + neighbour piece (2.3mT) subtracted by firmware feed-forward,
     0.5% residual after Hall-array self-calibration (idle-board coil pulsing calibrates coil map
     + channel offsets) = 132um bias. Total 200um.
   - Spec DERIVED: position error budget = gap/10 = 300um (replaced arbitrary 5um input, which is
     unreachable with analog Halls at real depth and was never required by control or gameplay).
     Tilt from measured Jacobian: 1.45mrad -> 139um tip resolution (needs <= 1mm).
   - Hall array supply power (2,240 sensors, 67.2W) added to PSU load; bank unchanged (load 0.75).
   - Reserve lever if margin thins: 10mm sensor pitch -> ~89um total, +$250 board.
   - Gate open: Hall cost cuts (clone sensors) may now be evaluated against this noise budget.
3. [DONE 2026-07-14] Eddy-current drag (Reitz thin-sheet + magpylib superconductor-image force)
   - Solid 4mm Al radiator gave 15.4mN drag = 21% of coupled lateral thrust at full traverse speed
     (0.92 m/s) - FAILED the 10% budget. PCB copper planes negligible (1.2mN). Damping credit
     negligible (0.5/s vs 36/s instability) - no free stabilization.
   - USER CHOSE the hardware fix (keep it fast): crosshatch eddy-break kerfs, 5mm island pitch,
     cut from the top, 0.5mm solid web left at the bottom. Islands modeled with loop-area factor
     (pitch/(lambda/2))^2 = 0.25; web modeled honestly as its own deeper thin sheet.
   - Result: 5.0mN = 6.8% at full 0.92 m/s - PASSES. Vertical thermal path unaffected.
     +$15 slotting pass [TO BE SOURCED]. Board $2,998.24, 52 checks OK.

## Phase 2 - cost reduction on frozen physics (path toward $1k)

Order revised 2026-07-14 after Phase 1 review: a stack inconsistency was found (thermal pad 0.5mm vs
radiator standoff 2.5mm for Hall clearance), and Hall power gating was identified as a new lever that
must precede the PSU audit. Standing caution: coupled lateral margin is ~10%; every change is
auto-re-verified by the model's real-geometry checks.

4. [DONE 2026-07-14] Hall SMD package + radiator stack reconciliation
   - SOT-23 49E-class clone ($0.05 [TO BE SOURCED], must meet 14 V/T + 50uT rms budget): -$351 board.
   - Stack made consistent: deleted separate thermal_pad_thickness; the gap-filler pad IS the radiator
     standoff (single constant, 1.5mm) used by both the thermal chain and the eddy/Hall geometry.
   - Consequences all re-verified: plate 1mm closer -> eddy drag 6.3mN = 8.6% (still <= 10%);
     sensor element at 12.16mm -> position error 199um <= 300um; cyclic baseplate 46.0C <= 50;
     coil selection unchanged. 52 checks OK. Board $2,998.24 -> $2,647.23.
5. [DONE 2026-07-14] Hall power gating
   - Per-mux-group high-side switch, powered only during its scan burst (100us settle + 512us burst
     per update cycle): duty 9.35%, supply 67.2W -> 6.3W. Electronics overhead 89 -> 28W.
   - Scan headroom honestly includes settle overhead: 2.67x (was 3.2x without settle accounting).
   - +4 gate switches/tile (+$2.80 board). Total peak load 1734.6W, required 2168W (+margin):
     still 168W above a 4-unit bank - the 6->4 decision lands in the item-6 margin audit.
6. [DONE 2026-07-14] PSU margin-stack audit + re-sizing
   - psu_sizing_margin 1.25 -> 1.1: the load is already a governed worst-case peak (phase-averaged
     fleet hover x 1.5 look-ahead applied to ALL 32 pieces + C2 firmware governor capping fleet
     current at the PSU rating). 1.25 on top was prudence-on-prudence; 1.1 covers line/aging.
   - 6 -> 4x UHP-500-12 ($565 -> $377). Load fraction 0.953; worst-phase capacity 22.3 -> 13.7
     pieces (transient, governor-staggered - still ample).
   - Also added two thermal BOM items the model consumed but never priced: potting epoxy ($45)
     and 1.5mm gap-filler pad ($35), both [TO BE SOURCED]. Board $2,650.03 -> $2,541.63.
7. [DONE 2026-07-14] Driver right-sizing at 5A
   - FETs: 60V discrete pair -> 40V dual N-MOSFET SOP-8, one package per half-bridge (108/tile
     instead of 216 discretes): -$9.09/tile. Voltage rating check 24V <= 40V holds.
   - Shunts: 3W -> 2W 2512 50mR: -$3.77/tile; new shunt-dissipation check (worst 0.69W <= 1W derated).
   - Gate drivers kept (EG2134 3-HB is already the cheap consolidation).
   - Tile $51.20 -> $38.70; board $2,541.63 -> $2,104.28 (-$437). 53 checks OK; selection unchanged.
8. [DROPPED 2026-07-14] Volume tier 100 -> 1k: user decision - committing to 1k boards is a business
   risk; all pricing stays on the 100-board basis. PHASE 2 COMPLETE at $2,104.28.

## Phase 3 - system-level gaps (required before "product"; 9-10 can still move numbers)

Revised 2026-07-14 after the sourcing pass (BOM_SOURCING.md): its three architecture findings fold
into the items below. Item 9 becomes FULL physical stack closure (top: potting cover + playing
surface + visible hover; bottom: radiator construction + gap filler + slot geometry). New item 9b:
PSU zoning + regenerative clamp. Dual-FET selection stays a sourcing RFQ (no model change).

9. [DONE 2026-07-14, reworked for >=3mm visible] Full physical stack closure (top + bottom)
   - USER SPEC: visible hover >= 3mm (2.1mm draft rejected). Two levers applied together:
     ultra-thin integrated surface (UV print + clear wear topcoat directly on the potting, no
     separate sheet; piece magnets flush with 0.1mm conformal coat; stack 0.4mm total) AND
     magnet-to-coil gap 3.0 -> 3.5mm (max flight 4.5mm). Visible hover 3.1mm nominal / 4.1mm max.
   - Price of the weaker field at 3.5mm (all re-verified): hover 28.4 -> 36.3 W/piece; fins 15 ->
     30mm (fanless dissipation); PCB thermal-via farm 5 -> 10 W/mK effective + premium 5 W/mK
     dispensed filler (conduction path); driver 5 -> 5.5A (coupled lateral 0.197 -> 0.229g);
     PSU back to 6 units. Touch temps IMPROVED (hotspot 68.7C <= 77, idle 43.0C <= 48).
   - Sweep infeasibility diagnostics added: first_failed_gate tally in the crash message.
   - Board $3,095.34 -> $3,367.49 (+$272 = the cost of the 3mm visible-hover spec). 58 checks OK.
10. [DONE 2026-07-14] PSU zoning + regenerative energy clamp
    - Two independent isolated +/-12V zones (one per board half); per-zone governor. Honest capacity
      metric: zone hovers 28.7 pieces vs 16 at reset formation (all-32-in-one-zone was over-
      conservative: reset formation is symmetric; pathological clustering is refused by the governor).
    - Balancer deleted (independent isolated supplies deliver unequal rail currents; rail imbalance
      2.77A << 83A unit rating, checked). Added rail regen clamps + bulk capacitance (4x each,
      RFQ budgets) - PSUs cannot sink braking energy.
    - Dual-FET selection remains a pure sourcing RFQ (BOM line unchanged).
11. Move choreography / planner requirement (knight jumps, blocked paths at low altitude).
12. Standby power (largely resolved by Hall gating in Phase 2), enclosure BOM, magnet safety compliance (CPSC).

## Status

- Done before this plan: mesh convergence verified; coupled-authority factorization bug fixed
  (real-geometry post-selection verification); driver limit 3.5 -> 5A; all checks OK.
- Current: Phase 3, item 11 (move choreography / planner). Board $3,367.49, 58 checks OK,
  visible hover 3.1mm (>= 3mm spec).

# Bill of Materials — Levitating Chess Board

2026-10-01 update: the $4,647.45 baseline below remains the reference estimate. The new fixed-weight minimum-loss study finds 13/1,200 flight targets infeasible for its 24-coil neighborhood. An 80-coil neighborhood using existing hardware passes the sampled level-flight targets and projects $4,226.25 with no supercap energy deficit, but multi-piece, boundary, per-coil thermal and trajectory validation remain open. No candidate savings are adopted here. See `CURRENT_ALLOCATION.md` and `current_allocation_results.json`; 77/78 below counts only the older model gates.

Snapshot of the one-logical-board model run of 2026-08-08 (`last_run.txt`: 77 of 78 checks pass; reset choreography is not yet demonstrated).
Prices are public catalog benchmarks (100-board lot) where linked; `[TO BE SOURCED]` marks lines needing an RFQ or a design decision before quoting.
Source of truth for quantities is `model.py`; regenerate `last_run.txt` after any design change and update this file.

**Candidate board total: $4,647.45** = one logical motor/control bed at $2,294.58 + 32 pieces x $6.01 + shared $2,160.61
**Candidate board mass: 34.01 kg**; whole set including pieces is **34.76 kg**. The 54-cell buffer is 3.51 kg.

This remains a feasibility-stage estimate, not a production quote. The 40 independent smart tiles have been removed from the current architecture. The model now covers the exact 720 x 480 mm motor with one logical continuous board, one provisional board-wide setpoint FPGA, 17 parallel output lanes, and ten regional MCU/ADC nodes derived from throughput. Physical manufacture as one PCB, strips, or panels is deliberately deferred. Magnetic physics and action A are unchanged. The power architecture is now one 40 V full-bus source, a controlled +/-20 V midpoint, and 9s3p supercap banks on both rails. The former 174 A failure is reduced to 108.45 A against the 120 A limit.

## Current one-logical-board BOM

### A. Continuous motor/control bed

| Part | Qty/board | Line cost | Note |
|---|---:|---:|---|
| Driver power MOSFET | 4,608 | $345.60 | One local half-bridge package per coil; >=60 V RFQ target |
| Gate-driver IC | 1,536 | $346.21 | Three half-bridges per EG2134-class IC |
| Current-setpoint latch | 578 | $16.18 | GR74HC595 chains across 17 parallel lanes |
| Setpoint RC filter pairs | 4,608 | $13.82 | One filtered command per channel |
| Board-wide setpoint FPGA | 1 | $25.00 | Allowance only; sourcing and synthesis are CRITICAL open risks |
| Current shunts | 4,608 | $192.61 | 20 mOhm midpoint-return sense |
| Dual current comparators | 2,304 | $45.62 | One comparator channel per driver |
| Current front-end passives | 18,432 | $64.51 | Estimate; idle zero-calibrated |
| Distributed bulk capacitors | 159 | $39.75 | Retains one per approximately 29 drivers |
| Distributed thermal sensors | 70 | $0.64 | Retains former sensor-area density without tiles |
| Gate passives | 9,216 | $11.06 | Two per half-bridge |
| Driver decoupling | 4,418 | $87.03 | Logic bypass capacitors |
| SMT assembly | 205,327 joints | $349.06 | Exact board-wide component count |
| Magnet wire | 3.039 kg | $56.95 | 4,608 32-turn windings |
| Hall position sensors | 1,734 | $602.70 | 51 x 34 grid at 14.2857 mm pitch |
| Hall gate switches | 109 | $2.25 | One per mux group |
| 16-channel Hall muxes | 109 | $27.54 | Scanned by 20 ADC engines |
| Motor/control PCB area | 3,456 cm2 | $36.74 | Area allowance; physical partition not fixed |
| Regional MCU/ADC nodes | 10 | $31.30 | Two shared Hall/current ADC engines per node |
| **Motor/control-bed subtotal** | | **$2,294.58** | |

### B. Pieces

| Part | Qty/piece | Unit cost | Set cost |
|---|---:|---:|---:|
| N48SH 5 x 5 x 4 mm magnet blocks | 16 | $0.288 | $147.46 |
| PETG/PC body, inserts and finish | 1 | $1.40 | $44.80 |
| **Pieces subtotal** | | | **$192.26** |

### C. Board-shared

| Block | Cost |
|---|---:|
| Compute module + mainboard | $86.96 |
| 2.498 kW PSU, active midpoint, bus, rail clamps and bulk capacitance | $683.96 |
| 54-cell supercap system, balancing, management and protection | $477.20 |
| Radiator aluminium and eddy-break slotting | $379.16 |
| Potting and thermal gap filler | $381.43 |
| Playing surface, AC input, EMI filter and enclosure | $72.00 |
| Two low-noise radiator fans | $79.90 |
| **Board-shared subtotal** | **$2,160.61** |

### Roll-up and status

| Block | Cost |
|---|---:|
| Motor/control bed | $2,294.58 |
| Pieces | $192.26 |
| Board-shared | $2,160.61 |
| **Candidate board total** | **$4,647.45** |

Seventy-seven of 78 modeled gates pass. The 40 V source/buffer refactor removes the former rail-current failure while preserving the assumed action-A/C2 electrical workload: reset peak load is 3.904 kW, cumulative two-reset buffer drawdown is 1.740 kJ, installed usable buffer energy is 3.640 kJ at end-of-life assumptions, and rail current is 108.45 A. The new failed gate records that `reset_choreography.py` has not demonstrated collision-free five-wave routes from nontrivial adversarial placements. The power result is therefore conditional on finding a valid choreography or replacing its load trace. Force, 378-pose authority, Hall error, thermal, hover-endurance, and 1.017x hot action-A results remain materially unchanged. See `RESET_CHOREOGRAPHY.md`, `POWER_ARCHITECTURE.md`, and `RISKS.md`.

## Superseded 40-smart-tile snapshot (2026-08-07)

### A. Per tile (10x10 cm coil-array module)

| Part name | Part code | Qty/tile | Purpose | Unit price | Link |
|---|---|---:|---|---:|---|
| Driver power MOSFET | >=60 V dual N-MOSFET SOP-8/PDFN | 116 | One half-bridge per coil; **TO BE SOURCED — former LCSC C20539695 link was rejected because that part is only 40 V** | $0.075 (RFQ target) | [TO BE SOURCED] |
| Gate driver | EG Micro EG2134 (LCSC C480661) | 39 | Drives 3 half-bridges per IC | $0.225 | [lcsc.com](https://www.lcsc.com/product-detail/C480661.html) |
| Setpoint latch | Gcore GR74HC595 (LCSC C18164493) | 15 | Serial delta-sigma setpoint stream latched to 116 channels | $0.028 | [lcsc.com](https://www.lcsc.com/product-detail/C18164493.html) |
| Setpoint RC filter | 15.8 kΩ 1 % + 10 nF X7R 0603 (LCSC C155689 + C519406) | 116 | 1.007 kHz low-pass per delta-sigma bitstream | $0.003/pair | [lcsc.com](https://www.lcsc.com/product-detail/C519406.html) |
| Setpoint FPGA | GOWIN GW1NZ-LV1QN48C6 (LCSC C5799569) + 1.2 V LDO | 1 | Delta-sigma modulator fabric; **must be time-multiplexed through BRAM — 116 parallel accumulators exceed the 864 FFs; synthesis proof pending** | $4.05 ($3.90 @100 + LDO) | [lcsc.com](https://www.lcsc.com/product-detail/C5799569.html) |
| Current shunt | Milliohm HoJLR2512-2W-20mR-1% 75 ppm (LCSC C2924538) | 116 | Midpoint-return current sense; 0.61 W at the 5.5 A channel limit | $0.0418 @4k (full-reel RFQ pending) | [lcsc.com](https://www.lcsc.com/product-detail/C2924538.html) |
| Current comparator | MSKSEMI LM393 (LCSC C5252905) | 58 | Bang-bang current loop per 2 channels | $0.020 | [lcsc.com](https://www.lcsc.com/product-detail/C5252905.html) |
| Current front-end passives | 0603 1 % + matched-pair arrays | 464 | Sense filter + midpoint level-shift; **0.1 % not required — firmware idle zero-cal removes static divider error by design; only tracking/drift matters** | $0.0035 (est.) | [TO BE SOURCED] |
| Driver gate passives | 0603 1 % (LCSC C54531144 class) | 232 | Gate pull resistors | $0.0012 | [lcsc.com](https://www.lcsc.com/product-detail/C54531144.html) |
| Driver decoupling | 100 nF 50 V X7R 0603 (LCSC C14663 class) | 112 | Logic bypass | $0.0197 | [lcsc.com](https://www.lcsc.com/product-detail/C14663.html) |
| Tile bulk capacitance | 330–470 µF 35 V polymer | 4 | Local power decoupling for 116 half-bridges at 20 kHz; 35 V rating covers the current 12 V half-rail and transient margin | $0.25 (est.) | [TO BE SOURCED] |
| Thermal sensor | 10 kΩ 1 % 0402 NTC | 2 | Ground truth for the thermal-governor energy observer + hardware overtemp cutback (added per system review) | $0.0091 (est.) | [TO BE SOURCED] |
| SMT assembly | JLCPCB assembly joints | 5,323 | Automated placement/soldering; count includes Hall sensors, muxes, mux gates, MCU, connector and coil terminations | $0.0017/joint | [jlcpcb.com](https://jlcpcb.com/help/article/pcb-assembly-faqs) |
| Magnet wire | 1×0.05 mm flat self-bonding enameled copper | 76.4 g | Candidate coil: 116 windings × 32 turns | $18.74/kg (RFQ budget) | [enameledwires.com](https://enameledwires.com/products/enameled-copper-wire/self-bonding-rectangular.html) |
| Hall position sensor | TI DRV5055A4QDBZR | 49 | 6-DoF pose sensing, 14.29 mm grid; exact run gives 285 µm docking error against the 350 µm budget | $0.348 | [digikey.com](https://www.digikey.com/en/products/detail/texas-instruments/DRV5055A4QDBZR/8567410) |
| Hall group gate switch | GOODWORK AO3401A (LCSC C2938368) | 4 | Powers each mux group only during scan burst | $0.0206 | [lcsc.com](https://www.lcsc.com/product-detail/MOSFETs_GOODWORK-AO3401A_C2938368.html) |
| Hall readout mux | TI CD74HC4067SM96 (LCSC C98457) | 4 | 16-channel analog mux into the MCU ADC | $0.253 | [lcsc.com](https://www.lcsc.com/product-detail/C98457.html) |
| Tile PCB | JLCPCB 4-layer FR4 100×100 mm | 1 | Drivers, sensors, coil terminations | $1.06/tile | [jlcpcb.com](https://jlcpcb.com/news/discount-on-quality-4-layer-pcbs) |
| Tile control MCU | STM32G431KBT6 | 1 | Pose estimation + control loops (12.0× headroom) | $3.13 | [digikey.com](https://www.digikey.com/en/products/detail/stmicroelectronics/STM32G431KBT6/10231564) |
| Backplane connector | ZHOURI 2×10 2.54 mm header (LCSC C5116480) | 1 | Tile-to-mainboard power + serial | $0.0724 | [lcsc.com](https://www.lcsc.com/product-detail/C5116480.html) |
| **Per-tile subtotal** | | | | **$66.30** | |

### B. Per piece — 32 pieces per board

| Part name | Part code | Qty/piece | Purpose | Unit price | Link |
|---|---|---:|---|---:|---|
| NdFeB magnet block | N48SH 5.00×5.00×4.00 mm, through-thickness magnetized, Ni-Cu-Ni | 16 | 12 g Halbach array; exact model uses the 1.36 T guaranteed-grade floor at 20 °C plus temperature derating | $0.288 (scaled, unverified) | [TO BE SOURCED] ([Mainrich N48SH](https://www.mainrichinternational.com/magnets/n48sh)) |
| Piece body | PETG/PC print + inserts + finish | 1 | 11.38 g shell CAD target: 60% of the old hollow king bounding-cylinder volume; 23.38 g total worst-case piece. PLA remains ruled out | $1.40 (est.) | [jlc3dp.com](https://jlc3dp.com/blog/3d-printing-cost) |
| **Per-piece subtotal** | | | | **$6.01** | |

### C. Board-shared — one set per board

| Part name | Part code | Qty | Purpose | Unit price | Link |
|---|---|---:|---|---:|---|
| Compute module | Raspberry Pi CM5 2GB Lite (SC1556) | 1 | Game logic, choreography, thermal governor, replay | $61.96 | [digikey.com](https://www.digikey.com/en/products/detail/raspberry-pi/SC1556/25805567) |
| Mainboard | Custom 4-layer carrier | 1 | CM5 + tile backplane + PSU/buffer interconnect | $25.00 (quote basis) | [jlcpcb.com](https://jlcpcb.com/quote) |
| Tile interconnect | HDGC 2×10 2.54 mm socket (LCSC C19725277) | 40 | Mainboard sockets for the tiles | $0.1125 | [lcsc.com](https://www.lcsc.com/product-detail/C19725277.html) |
| Bus power supply | Mean Well UHP-500-12 | 2 | One isolated ±12 V split-rail zone (series pair). No credit is taken for unsupported PSU current sharing; 1.00 kW nominal passes the 362 W localized sustained requirement | $83.30 | [digikey.com](https://www.digikey.com/en/products/detail/mean-well-usa-inc/UHP-500-12/8324034) |
| Bus distribution | Copper 110 flat busbar + zone cabling | 1 | Low-drop zone rail distribution | $36.96 (allowance) | [ebay.com](https://www.ebay.com/itm/304578689563) |
| Rail regen clamp | Active MOSFET dump clamp + TVS for spikes | 2 | Absorbs braking energy; thresholds and pulse energy remain unsourced | $5.00 (est.) | [TO BE SOURCED] |
| Rail bulk capacitance | Zone-level low-ESR electrolytic | 2 | Zone rail stiffening; sizing must follow from ripple/inductance spec | $4.00 (est.) | [TO BE SOURCED] |
| Supercap burst buffer | Maxwell BCAP0350-P270-S18, 5s15p per rail | 150 | 11.79 kJ usable at EOL against an 11.60 kJ two-event deficit. The energy and recharge gates pass, but 9.75 kg / $1,148 in cells remain excessive and the 175 A rail burst fails distribution | $7.65 @1k | [digikey.com](https://www.digikey.com/en/products/detail/maxwell-technologies/BCAP0350-P270-S18/11673891) |
| Supercap balancing | Active balancing network; topology pending | 150 | Voltage equalization | $0.15 (est.) | [TO BE SOURCED] |
| Buffer charge/protection | 5-series supercap monitor + precharge/charge path + fuse + disconnect, per rail bank | 2 | **TO BE SOURCED. The formerly cited BQ33100 is not retained as the source.** | $20.00 (est.) | [TO BE SOURCED] |
| Buffer ideal-diode fan-out | Protected high-current path per rail per zone | 4 | Buffer feed protection; present 175 A rail demand fails the 120 A distribution gate | $4.00 (est.) | [TO BE SOURCED] |
| Radiator | 6063-T5/T6 integral-fin extrusion 720×480, 4 mm base, 60 mm fins | 14.93 kg | Current thermal concept passes the modelled C6/C7 workloads, but tooling and freight remain excluded | $12.00/kg (material only) | [TO BE SOURCED] ([550 mm capability ref](https://sinoextrud.com/what-is-the-maximum-heatsink-size-we-can-produce/)) |
| Radiator eddy-break slotting | Gang-saw 5 mm crosshatch, 3.5 mm deep, 0.5 mm web | 1 | ~137 m of cut per board; **credible only gang-sawed, not CNC-milled; drawing still needs kerf width, deburr and post-machining flatness spec** | $200.00 (RFQ budget) | [TO BE SOURCED] |
| Coil potting epoxy | Ziitek TIE280-25AB class, 2.5 W/mK | 1 | Coil-bed potting; playing-surface substrate | $45.00 (RFQ budget) | [ziitek.com](https://www.ziitek.com/epoxy-potting-compound) |
| Thermal gap filler | Laird Tputty SF560, 5.6 W/mK, 1.3 mm bond line, 449 cc | 1 | Couples tile PCBs to radiator; bond line set by the 1.1 mm Hall bodies under the PCB. Ten-pail public price; **selective dispensing could cut 67–80 % but requires thermal-model rework (contact coverage)** | $336.43 (10-pail public) | [laird.com](https://www.laird.com/products/thermal-interface-materials/liquid-gap-fillers/tputty-sf560) |
| Playing surface | UV print + flood clear wear coat on potting | 1 | Board graphics ≤0.10 mm total. Print-only ~€10.4/board supports the estimate; **wear-coat qualification (Taber, chemicals, CoF, yellowing) still open** | $12.00 (print-only basis) | [TO BE SOURCED] ([supplied-material ref](https://print-shop.hr/doneseni-materijal)) |
| AC input | IEC inlet + fuse + mains switch + internal AC harness | 1 | Consumer-product mains entry (added per review: was missing entirely) | $8.00 (est.) | [TO BE SOURCED] |
| Mains EMI filter | Conducted-emissions filter | 1 | The 20 kHz half-bridge farm will not pass conducted emissions without one (added per review) | $12.00 (est.) | [TO BE SOURCED] |
| Enclosure / frame | Frame + skirt enclosure allowance | 1 | Physical product shell; 1.0 kg mass already budgeted (added per review) | $40.00 (est.) | [TO BE SOURCED] |
| Radiator fan | Noctua NF-A20 PWM 200 mm @ 550 rpm | 2 | Fan-assisted cooling for grind mode and resets; 16.7 dB(A) installed | $39.95 | [coolerguys.com](https://www.coolerguys.com/products/noctua-nf-a20-pwm-200mm-cooling-fan) |
| **Shared subtotal** | | | | **$2,451.51** | |

### Roll-up

| Block | Cost |
|---|---:|
| Tiles (40 × $66.30) | $2,651.82 |
| Pieces (32 × $6.01) | $192.26 |
| Board-shared | $2,451.51 |
| **Candidate board total** | **$5,295.59** |

### Corrected status at that time

Seventy-five of 77 gates pass. C6/C7 thermal limits, required hover endurance, localized PSU capacity, buffer energy/recharge, Hall position-error budgets, 6-DoF authority and the temperature-derated action-A commutation gate all pass. The hot action-A margin remains narrow at 1.017x. The two open failures are the 800 x 500 mm uniform-tile rectangle exceeding the 720 x 480 mm motor/radiator and 175 A event rail current versus the 120 A distribution limit.

The local current-mode conclusion remains negative after the mass re-optimization. A fixed 12-mode basis needs at least 2.299x peak current and 2.827x power; 20 modes still need 1.110x peak current, exceeding the 1.017x hot action-A margin while eliminating too few channels to change the cost conclusion. The pose-adaptive nine-mode mathematical bound remains an uncosted routing/crosspoint research option; see `MODE_COMPRESSION.md`.

## Superseded historical review log

The entries below explain how earlier estimates were reached. They are retained for traceability only and are superseded by the 2026-08-08 one-logical-board snapshot at the top of this file.

### Status after sourcing review (2026-08-03)

Adopted at public prices: FPGA (+$74/board), shunts (+$78), Maxwell supercaps (+$50), ORing hardware (+$15), buffer charge/protection (+$40, new line), tile bulk capacitance (+$40, new line), gap filler at 10-pail public (+$186). Mass budget corrected: +1.76 kg filler, +0.15 kg cells → 25.12 kg/board.

Not adopted: 0.1 % front-end passives ($225–430/board) — the Phase-4 sensing architecture zero-calibrates static offsets at idle, so 1 % + matched arrays meet the error budget; the 0.1 % premium buys nothing.

At that time the estimate also excluded radiator die/tooling + freight NRE, magnet RFQ variance, regen-clamp design, wear-coat qualification, VAT, freight and duty.

### Status after system-feasibility review (2026-08-04)

Adopted into `model.py` (all re-run, 72 checks green):

- **Temperature-dependent physics (the review's strongest catch).** Copper resistivity (+0.393 %/K) and NdFeB Br (−0.11 %/K, magnets soaked to cell temperature) now feed a self-consistent fixed point; the worst cell carries a 1.38× hot power derate. The old design point failed — counter-moves: fins 30→45 mm, potting bed thinned 1.0→0.5 mm, bus moved 24→30 V. Worst cell 75.6 °C vs the 77 °C hard touch cap; hot-soak lift margin 1.81× vs 1.3× floor.
- **Takeoff stagger (perceptual simultaneity).** Reset lift-offs run in 3 waves of ~11 inside the existing 2 s contention window; flight stays visually simultaneous. Peak reset load drops 5.2→3.2 kW.
- **Per-zone pile-up sizing.** The buffer now covers the adversarial all-32-in-one-zone reset against one zone's 1.0 kW PSU (first event zonal, rematch reset from the spread home formation): 98 cells, 7.68 kJ usable vs 7.34 kJ needed; zone burst 7.33 kW available vs 3.17 kW demand.
- **Bus-current check redefined.** Now gates the buffer-fed burst rail current (105.6 A vs 120 A rating) instead of the meaningless sustained average; this is what pushed the sweep off 24 V.
- **Hall saturation** now includes the neighbour-piece field, plus a new rest-pose check (parked piece: 66.8 mT vs 169 mT range).
- **Thermal governor grounded.** 2 NTC/tile added; fail-safe policy: sensor fault or overtemp de-energizes coils — pieces settle, passive-safe.
- Minor: unused `production_volume` removed, N52 link dropped from the N48SH line, built-vs-used channel count (4,640 vs 4,608) reported explicitly.

Net effect: **+$630 (+14 %)** and **+7.2 kg**, almost entirely honest physics (hot copper/magnets) plus the single-zone pile-up buffer. Rejected as over-engineering: contact-to-hover trajectory sweep (force per amp rises monotonically as the gap closes; 3.5 mm is the worst point), global 32-piece wrench solve (3×3 adjacency solve in `verification.py` bounds the coupling), continuous pose sweeps, and radiator FEM (the 3×3 patch measures it directly).

### Status after second system-feasibility review (2026-08-04, two independent reports)

Adopted into `model.py` (re-run, **76 checks green**, `verification.py` restored and passing):

- **Honest cruise power (the round's strongest catch).** Cruise was billed at hover×1.5 while the authority model showed the full-diagonal reset move needs ~87 % of maximum lateral force. Now `levitation_sim.py` runs a min-peak-current LP at the actual required thrust for every worst-case pose: sprint tier (full A, 4 m/s² class) gates driver commutation headroom (1.003×, binding); a relaxed tier (reset/replay corridors, stretched 2×) prices the recurring energy. Worst-pose sprint power: 523 W vs 168 W hover.
- **Consequence: 48 V bus.** True corridor power pushed the sweep off 30 V; drivers move to ±24 V split rail with 60 V FETs ($0.075 RFQ target), tile bulk caps to 35 V. Wire re-selected: 1×0.05 mm flat, 56 turns.
- **C7 source-path fix.** The cyclic peak coil/MOSFET temperature (78.5 °C in the old report — above the 77 °C touch cap, unchecked) is now gated, and the governed peak cell temperature takes the worst of the local-hotspot and source+zone-pile-up paths. New worst cell: 67.9 °C quiet / 67.0 °C fans.
- **Pipelined reset choreography.** The unused 2.33× crowding factor is now moot by design: the reset runs 5 takeoff waves 0.5 s apart, each wave cruising while the next lifts, colored so no two adjacent pieces hover simultaneously — no crowded hover hold at all. **C2 interpretation: lift-offs are staggered across ~2.5 s but flight overlaps, so the reset still reads as one simultaneous event** (extends the previously approved takeoff-only stagger to landings).
- **Reset fan policy (behavior change).** Resets spin the fans up (10.7 dB(A) at low rpm — still C4-silent by the fan-mode budget); the fans-off thermal budget covers live play and the T3 hammer. Quiet-mode thermal math excludes reset events accordingly.
- **Cross-zone PSU ORing.** All six rail halves and the buffer are cross-tied through ideal-diode ORing (12 ORing paths), so a single-zone pile-up draws on the whole 3 kW bank. This shrank the supercap buffer 98 → 20 cells (10s1p per rail, 1.57 kJ usable vs 0.64 kW × event deficit), −$597 in cells.
- **Buffer sized at end-of-life:** 80 % capacitance, 2× ESR — the review's aging catch.
- **Zonal thermal node** added: the pile-up zone's lift energy raises the local plate above board average (+~1 K), folded into the governed peak.
- **T1 rewritten** in `PRODUCT_VISION.md`: "twice back-to-back" is physically impossible from the same start; now adversarial reset + immediate rematch reset from the home formation.
- **T3 on its own baseline + combination policy.** The hammer test is evaluated per C6 (tests separate); the NTC-grounded governor throttles any user-combined workload to keep C7.
- **Hall sensing at flight currents.** Coil-field subtraction bias is now evaluated at sprint currents (1.79× hover). Two explicit budgets: docking (hover currents, 10 % of gap — landing precision) and in-transit (25 % of gap — corridors clear of neighbours); grid densified 14.29 → 12.5 mm (49 → 64 sensors/tile). Rest-pose saturation with a parked piece re-checked.
- **Endurance made an input, not an output:** level parked-hover ≥30 s (32.6 s worst) and showpiece-tilt ≥3 s (5.3 s at the design rung) checks added.
- **Consumer-product lines added:** AC inlet ($8), mains EMI filter ($12), enclosure/frame ($40).
- **Honesty fixes:** serial-rate check now derives the required delta-sigma bit rate (2.8× real headroom, not tautological); hall supply current 10 mA (was optimistic); gate-drive power on peak driven windings; SMT joint count completed (+584/tile); potting epoxy (4.5 kg) and bus distribution (0.8 kg) added to the mass budget; burst rail current evaluated at full droop (84 A vs 120 A rating); PLA → PETG; deleted `verification.py` restored.

Historical net effect: **−$109 (−2.1 %) and +5.0 kg** vs the preceding revision. The cross-zone PSU-current-sharing assumption behind that result has now been rejected.

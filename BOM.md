# Bill of Materials — Levitating Chess Board

Snapshot of the cost model run of 2026-08-04 (`last_run.txt`, all 76 checks green), updated after the sourcing review and two system-feasibility reviews.
Prices are public catalog benchmarks (100-board lot) where linked; `[TO BE SOURCED]` marks lines needing an RFQ or a design decision before quoting.
Source of truth for quantities is `model.py`; regenerate `last_run.txt` after any design change and update this file.

**Board total: $5,039.37** = 40 tiles x $72.65 + 32 pieces x $7.16 + shared $1,904.11
**Board mass: 37.34 kg** (60 mm fin radiator, 20-cell EOL-sized buffer, gap filler, potting and bus distribution included; whole set incl. pieces 38.43 kg)

## A. Per tile (10x10 cm coil-array module) — 40 tiles per board

| Part name | Part code | Qty/tile | Purpose | Unit price | Link |
|---|---|---:|---|---:|---|
| Driver power MOSFET | 60 V dual N-MOSFET SOP-8/PDFN (LCSC C20539695 class) | 116 | One half-bridge per coil; bipolar drive from the ±24 V split rail (48 V bus forced by honest cruise power) | $0.075 (RFQ target) | [lcsc.com](https://www.lcsc.com/product-detail/C20539695.html) |
| Gate driver | EG Micro EG2134 (LCSC C480661) | 39 | Drives 3 half-bridges per IC | $0.225 | [lcsc.com](https://www.lcsc.com/product-detail/C480661.html) |
| Setpoint latch | Gcore GR74HC595 (LCSC C18164493) | 15 | Serial delta-sigma setpoint stream latched to 116 channels | $0.028 | [lcsc.com](https://www.lcsc.com/product-detail/C18164493.html) |
| Setpoint RC filter | 15.8 kΩ 1 % + 10 nF X7R 0603 (LCSC C155689 + C519406) | 116 | 1.007 kHz low-pass per delta-sigma bitstream | $0.003/pair | [lcsc.com](https://www.lcsc.com/product-detail/C519406.html) |
| Setpoint FPGA | GOWIN GW1NZ-LV1QN48C6 (LCSC C5799569) + 1.2 V LDO | 1 | Delta-sigma modulator fabric; **must be time-multiplexed through BRAM — 116 parallel accumulators exceed the 864 FFs; synthesis proof pending** | $4.05 ($3.90 @100 + LDO) | [lcsc.com](https://www.lcsc.com/product-detail/C5799569.html) |
| Current shunt | Milliohm HoJLR2512-2W-20mR-1% 75 ppm (LCSC C2924538) | 116 | Midpoint-return current sense; 0.44 W worst-case | $0.0418 @4k (full-reel RFQ pending) | [lcsc.com](https://www.lcsc.com/product-detail/C2924538.html) |
| Current comparator | MSKSEMI LM393 (LCSC C5252905) | 58 | Bang-bang current loop per 2 channels | $0.020 | [lcsc.com](https://www.lcsc.com/product-detail/C5252905.html) |
| Current front-end passives | 0603 1 % + matched-pair arrays | 464 | Sense filter + midpoint level-shift; **0.1 % not required — firmware idle zero-cal removes static divider error by design; only tracking/drift matters** | $0.0035 (est.) | [TO BE SOURCED] |
| Driver gate passives | 0603 1 % (LCSC C54531144 class) | 232 | Gate pull resistors | $0.0012 | [lcsc.com](https://www.lcsc.com/product-detail/C54531144.html) |
| Driver decoupling | 100 nF 50 V X7R 0603 (LCSC C14663 class) | 112 | Logic bypass | $0.0197 | [lcsc.com](https://www.lcsc.com/product-detail/C14663.html) |
| Tile bulk capacitance | 330–470 µF 35 V polymer | 4 | Local power decoupling for 116 half-bridges at 20 kHz — zone caps are electrically too far; 35 V rating covers the 24 V rail (16 V parts on the old rail were flagged in review) | $0.25 (est.) | [TO BE SOURCED] |
| Thermal sensor | 10 kΩ 1 % 0402 NTC | 2 | Ground truth for the thermal-governor energy observer + hardware overtemp cutback (added per system review) | $0.0091 (est.) | [TO BE SOURCED] |
| SMT assembly | JLCPCB assembly joints | 5,368 | Automated placement/soldering — count now includes Hall sensors, muxes, mux gates, MCU, connector and coil terminations (review: joints were undercounted) | $0.0017/joint | [jlcpcb.com](https://jlcpcb.com/help/article/pcb-assembly-faqs) |
| Magnet wire | 1×0.05 mm flat self-bonding enameled copper | 132.9 g | The coil bed: 116 windings × 56 turns | $18.74/kg (RFQ budget) | [enameledwires.com](https://enameledwires.com/products/enameled-copper-wire/self-bonding-rectangular.html) |
| Hall position sensor | TI DRV5055A4QDBZR | 64 | 6-DoF pose sensing, 12.5 mm grid (densified to hold the docking error budget with sprint-current coil bias subtracted) | $0.348 | [digikey.com](https://www.digikey.com/en/products/detail/texas-instruments/DRV5055A4QDBZR/8567410) |
| Hall group gate switch | GOODWORK AO3401A (LCSC C2938368) | 4 | Powers each mux group only during scan burst | $0.0206 | [lcsc.com](https://www.lcsc.com/product-detail/MOSFETs_GOODWORK-AO3401A_C2938368.html) |
| Hall readout mux | TI CD74HC4067SM96 (LCSC C98457) | 4 | 16-channel analog mux into the MCU ADC | $0.253 | [lcsc.com](https://www.lcsc.com/product-detail/C98457.html) |
| Tile PCB | JLCPCB 4-layer FR4 100×100 mm | 1 | Drivers, sensors, coil terminations | $1.06/tile | [jlcpcb.com](https://jlcpcb.com/news/discount-on-quality-4-layer-pcbs) |
| Tile control MCU | STM32G431KBT6 | 1 | Pose estimation + control loops (12.2× headroom) | $3.13 | [digikey.com](https://www.digikey.com/en/products/detail/stmicroelectronics/STM32G431KBT6/10231564) |
| Backplane connector | ZHOURI 2×10 2.54 mm header (LCSC C5116480) | 1 | Tile-to-mainboard power + serial | $0.0724 | [lcsc.com](https://www.lcsc.com/product-detail/C5116480.html) |
| **Per-tile subtotal** | | | | **$72.65** | |

## B. Per piece — 32 pieces per board

| Part name | Part code | Qty/piece | Purpose | Unit price | Link |
|---|---|---:|---|---:|---|
| NdFeB magnet cube | N48SH 5.00×5.00×5.00 mm ±0.05, Hcj ≥1,592 kA/m, through-thickness magnetized, Ni-Cu-Ni, flux-sorted | 16 | Halbach array in the piece base; SH grade survives the hot-cell soak (design cap 77 °C). **RFQ must guarantee flux: model uses Br 1.40 T, catalog N48SH floor is ~1.36 T** — spec sorted lots or re-run the model at the guaranteed minimum. RFQ ~53k pcs (incl. 3.5 % yield) to Mainrich + 2 others | $0.36 (plausible, unverified) | [TO BE SOURCED] ([Mainrich N48SH](https://www.mainrichinternational.com/magnets/n48sh)) |
| Piece body | PETG/PC print + inserts + finish | 1 | Hollow shell over the magnet base; 34 g piece. PLA (Tg ≈60 °C) ruled out on a 77 °C-capable surface | $1.40 (est.) | [jlc3dp.com](https://jlc3dp.com/blog/3d-printing-cost) |
| **Per-piece subtotal** | | | | **$7.16** | |

## C. Board-shared — one set per board

| Part name | Part code | Qty | Purpose | Unit price | Link |
|---|---|---:|---|---:|---|
| Compute module | Raspberry Pi CM5 2GB Lite (SC1556) | 1 | Game logic, choreography, thermal governor, replay | $61.96 | [digikey.com](https://www.digikey.com/en/products/detail/raspberry-pi/SC1556/25805567) |
| Mainboard | Custom 4-layer carrier | 1 | CM5 + tile backplane + PSU/buffer interconnect | $25.00 (quote basis) | [jlcpcb.com](https://jlcpcb.com/quote) |
| Tile interconnect | HDGC 2×10 2.54 mm socket (LCSC C19725277) | 40 | Mainboard sockets for the tiles | $0.1125 | [lcsc.com](https://www.lcsc.com/product-detail/C19725277.html) |
| Bus power supply | Mean Well UHP-500-24 | 6 | Three isolated ±24 V split-rail zones (series pairs); 48 V bus forced by honest cruise power. All rails cross-tied through ideal-diode ORing, so the whole 3 kW bank plus buffer serves a single-zone pile-up | $94.20 | [digikey.com](https://www.digikey.com/en/products/detail/mean-well-usa-inc/UHP-500-24/8324036) |
| Bus distribution | Copper 110 flat busbar + zone cabling | 1 | Low-drop zone rail distribution | $36.96 (allowance) | [ebay.com](https://www.ebay.com/itm/304578689563) |
| Rail regen clamp | Active MOSFET dump clamp + TVS for spikes | 6 | Absorbs braking energy; **“TVS + dump resistor” rejected: a rail TVS knees too close to the 60 V FET rating. Needs clamp thresholds, per-event energy, pulse spec before quoting** | $5.00 (est.) | [TO BE SOURCED] |
| Rail bulk capacitance | Zone-level low-ESR electrolytic (Rubycon 25RXA4700 class candidate) | 6 | Zone rail stiffening; **sizing must follow from ripple/inductance spec; tile-local bulk added separately** | $4.00 (est.) | [TO BE SOURCED] |
| Supercap burst buffer | Maxwell BCAP0350-P270-S18, 10s1p per rail | 20 | Covers the 0.64 kW single-zone pile-up deficit above the cross-tied 3 kW PSU bank (1.57 kJ usable); **sized at end-of-life: 80 % capacitance, 2× ESR** | $7.65 @1k | [digikey.com](https://www.digikey.com/en/products/detail/maxwell-technologies/BCAP0350-P270-S18/11673891) |
| Supercap balancing | Active balancing network; **topology decision pending** | 20 | Voltage equalization; Maxwell recommends active balancing for this duty cycle | $0.15 (est.) | [TO BE SOURCED] |
| Buffer charge/protection | BQ33100-class monitor + precharge/charge path + fuse + disconnect, per rail bank | 2 | **Added per sourcing review — was missing entirely.** An ideal-diode discharge path cannot recharge the bank; a controlled charge path, current limit and cell protection are mandatory | $20.00 (est.) | [TO BE SOURCED] ([TI BQ33100](https://www.ti.com/product/BQ33100)) |
| Buffer ideal-diode ORing | TI LM74800-Q1 + paralleled N-FET pairs, per rail per zone (both polarities) | 12 | Cross-ties PSU rails and buffer into every zone without back-feed; **~84 A worst-zone burst rail path — parallel FETs, busbar attach, transient thermal validation required** | $4.00 (est.) | [TO BE SOURCED] ([LM74800 DS](https://www.ti.com/lit/ds/symlink/lm7480.pdf)) |
| Radiator | 6063-T5/T6 integral-fin extrusion 720×480, 4 mm base, 60 mm fins | 14.93 kg | Passive heatsink for silent mode; fins grown 45→60 mm to absorb the honest reset-corridor heat. **480 mm-wide cross-section needs a 4,500–7,500 t press; quote die/tooling + freight separately; also quote two joined 240 mm sections** | $12.00/kg (material only) | [TO BE SOURCED] ([550 mm capability ref](https://sinoextrud.com/what-is-the-maximum-heatsink-size-we-can-produce/)) |
| Radiator eddy-break slotting | Gang-saw 5 mm crosshatch, 3.5 mm deep, 0.5 mm web | 1 | ~137 m of cut per board; **credible only gang-sawed, not CNC-milled; drawing still needs kerf width, deburr and post-machining flatness spec** | $200.00 (RFQ budget) | [TO BE SOURCED] |
| Coil potting epoxy | Ziitek TIE280-25AB class, 2.5 W/mK | 1 | Coil-bed potting; playing-surface substrate | $45.00 (RFQ budget) | [ziitek.com](https://www.ziitek.com/epoxy-potting-compound) |
| Thermal gap filler | Laird Tputty SF560, 5.6 W/mK, 1.3 mm bond line, 449 cc | 1 | Couples tile PCBs to radiator; bond line set by the 1.1 mm Hall bodies under the PCB. Ten-pail public price; **selective dispensing could cut 67–80 % but requires thermal-model rework (contact coverage)** | $336.43 (10-pail public) | [laird.com](https://www.laird.com/products/thermal-interface-materials/liquid-gap-fillers/tputty-sf560) |
| Playing surface | UV print + flood clear wear coat on potting | 1 | Board graphics ≤0.10 mm total. Print-only ~€10.4/board supports the estimate; **wear-coat qualification (Taber, chemicals, CoF, yellowing) still open** | $12.00 (print-only basis) | [TO BE SOURCED] ([supplied-material ref](https://print-shop.hr/doneseni-materijal)) |
| AC input | IEC inlet + fuse + mains switch + internal AC harness | 1 | Consumer-product mains entry (added per review: was missing entirely) | $8.00 (est.) | [TO BE SOURCED] |
| Mains EMI filter | Conducted-emissions filter | 1 | The 20 kHz half-bridge farm will not pass conducted emissions without one (added per review) | $12.00 (est.) | [TO BE SOURCED] |
| Enclosure / frame | Frame + skirt enclosure allowance | 1 | Physical product shell; 1.0 kg mass already budgeted (added per review) | $40.00 (est.) | [TO BE SOURCED] |
| Radiator fan | Noctua NF-A20 PWM 200 mm @ 550 rpm | 2 | Fan-assisted cooling for grind mode and resets; 16.7 dB(A) installed | $39.95 | [coolerguys.com](https://www.coolerguys.com/products/noctua-nf-a20-pwm-200mm-cooling-fan) |
| **Shared subtotal** | | | | **$1,904.11** | |

## Roll-up

| Block | Cost |
|---|---:|
| Tiles (40 × $72.65) | $2,906.14 |
| Pieces (32 × $7.16) | $229.12 |
| Board-shared | $1,904.11 |
| **Board total** | **$5,039.37** |

## Status after sourcing review (2026-08-03)

Adopted at public prices: FPGA (+$74/board), shunts (+$78), Maxwell supercaps (+$50), ORing hardware (+$15), buffer charge/protection (+$40, new line), tile bulk capacitance (+$40, new line), gap filler at 10-pail public (+$186). Mass budget corrected: +1.76 kg filler, +0.15 kg cells → 25.12 kg/board.

Not adopted: 0.1 % front-end passives ($225–430/board) — the Phase-4 sensing architecture zero-calibrates static offsets at idle, so 1 % + matched arrays meet the error budget; the 0.1 % premium buys nothing.

This total still excludes: radiator die/tooling + freight NRE, magnet RFQ variance (Br guarantee), regen clamp final design, wear-coat qualification, VAT/freight/duty. Treat **$5,148 as a public-price floor**, not a finished production estimate.

## Status after system-feasibility review (2026-08-04)

Adopted into `model.py` (all re-run, 72 checks green):

- **Temperature-dependent physics (the review's strongest catch).** Copper resistivity (+0.393 %/K) and NdFeB Br (−0.11 %/K, magnets soaked to cell temperature) now feed a self-consistent fixed point; the worst cell carries a 1.38× hot power derate. The old design point failed — counter-moves: fins 30→45 mm, potting bed thinned 1.0→0.5 mm, bus moved 24→30 V. Worst cell 75.6 °C vs the 77 °C hard touch cap; hot-soak lift margin 1.81× vs 1.3× floor.
- **Takeoff stagger (perceptual simultaneity).** Reset lift-offs run in 3 waves of ~11 inside the existing 2 s contention window; flight stays visually simultaneous. Peak reset load drops 5.2→3.2 kW.
- **Per-zone pile-up sizing.** The buffer now covers the adversarial all-32-in-one-zone reset against one zone's 1.0 kW PSU (first event zonal, rematch reset from the spread home formation): 98 cells, 7.68 kJ usable vs 7.34 kJ needed; zone burst 7.33 kW available vs 3.17 kW demand.
- **Bus-current check redefined.** Now gates the buffer-fed burst rail current (105.6 A vs 120 A rating) instead of the meaningless sustained average; this is what pushed the sweep off 24 V.
- **Hall saturation** now includes the neighbour-piece field, plus a new rest-pose check (parked piece: 66.8 mT vs 169 mT range).
- **Thermal governor grounded.** 2 NTC/tile added; fail-safe policy: sensor fault or overtemp de-energizes coils — pieces settle, passive-safe.
- Minor: unused `production_volume` removed, N52 link dropped from the N48SH line, built-vs-used channel count (4,640 vs 4,608) reported explicitly.

Net effect: **+$630 (+14 %)** and **+7.2 kg**, almost entirely honest physics (hot copper/magnets) plus the single-zone pile-up buffer. Rejected as over-engineering: contact-to-hover trajectory sweep (force per amp rises monotonically as the gap closes; 3.5 mm is the worst point), global 32-piece wrench solve (3×3 adjacency solve in `verification.py` bounds the coupling), continuous pose sweeps, and radiator FEM (the 3×3 patch measures it directly).

## Status after second system-feasibility review (2026-08-04, two independent reports)

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

Net effect: **−$109 (−2.1 %) and +5.0 kg** vs the previous revision — the buffer savings from cross-zone ORing paid for the 48 V PSUs, denser Hall grid, bigger radiator and consumer lines. Treat **$5,039 as a public-price floor**, not a finished production estimate (same exclusions as above).

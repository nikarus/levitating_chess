# Bill of Materials — Levitating Chess Board

Snapshot of the cost model run of 2026-08-03 (`last_run.txt`, all 66 checks green), updated after sourcing review.
Prices are public catalog benchmarks (100-board lot) where linked; `[TO BE SOURCED]` marks lines needing an RFQ or a design decision before quoting.
Source of truth for quantities is `model.py`; regenerate `last_run.txt` after any design change and update this file.

**Board total: $4,517.80** = 40 tiles x $63.08 + 32 pieces x $7.16 + shared $1,765.53
**Board mass: 25.12 kg** (gap filler and distributor-listed supercap mass now included)

## A. Per tile (10x10 cm coil-array module) — 40 tiles per board

| Part name | Part code | Qty/tile | Purpose | Unit price | Link |
|---|---|---:|---|---:|---|
| Driver power MOSFET | 40 V dual N-MOSFET SOP-8/PDFN (LCSC C20539695 class) | 116 | One half-bridge per coil; bipolar drive from the ±12 V split rail | $0.055 (RFQ target) | [lcsc.com](https://www.lcsc.com/product-detail/C20539695.html) |
| Gate driver | EG Micro EG2134 (LCSC C480661) | 39 | Drives 3 half-bridges per IC | $0.225 | [lcsc.com](https://www.lcsc.com/product-detail/C480661.html) |
| Setpoint latch | Gcore GR74HC595 (LCSC C18164493) | 15 | Serial delta-sigma setpoint stream latched to 116 channels | $0.028 | [lcsc.com](https://www.lcsc.com/product-detail/C18164493.html) |
| Setpoint RC filter | 15.8 kΩ 1 % + 10 nF X7R 0603 (LCSC C155689 + C519406) | 116 | 1.007 kHz low-pass per delta-sigma bitstream | $0.003/pair | [lcsc.com](https://www.lcsc.com/product-detail/C519406.html) |
| Setpoint FPGA | GOWIN GW1NZ-LV1QN48C6 (LCSC C5799569) + 1.2 V LDO | 1 | Delta-sigma modulator fabric; **must be time-multiplexed through BRAM — 116 parallel accumulators exceed the 864 FFs; synthesis proof pending** | $4.05 ($3.90 @100 + LDO) | [lcsc.com](https://www.lcsc.com/product-detail/C5799569.html) |
| Current shunt | Milliohm HoJLR2512-2W-20mR-1% 75 ppm (LCSC C2924538) | 116 | Midpoint-return current sense; 0.44 W worst-case | $0.0418 @4k (full-reel RFQ pending) | [lcsc.com](https://www.lcsc.com/product-detail/C2924538.html) |
| Current comparator | MSKSEMI LM393 (LCSC C5252905) | 58 | Bang-bang current loop per 2 channels | $0.020 | [lcsc.com](https://www.lcsc.com/product-detail/C5252905.html) |
| Current front-end passives | 0603 1 % + matched-pair arrays | 464 | Sense filter + midpoint level-shift; **0.1 % not required — firmware idle zero-cal removes static divider error by design; only tracking/drift matters** | $0.0035 (est.) | [TO BE SOURCED] |
| Driver gate passives | 0603 1 % (LCSC C54531144 class) | 232 | Gate pull resistors | $0.0012 | [lcsc.com](https://www.lcsc.com/product-detail/C54531144.html) |
| Driver decoupling | 100 nF 50 V X7R 0603 (LCSC C14663 class) | 112 | Logic bypass | $0.0197 | [lcsc.com](https://www.lcsc.com/product-detail/C14663.html) |
| Tile bulk capacitance | 330–470 µF 16 V polymer | 4 | Local power decoupling for 116 half-bridges at 20 kHz — zone caps are electrically too far (added per sourcing review) | $0.25 (est.) | [TO BE SOURCED] |
| SMT assembly | JLCPCB assembly joints | 4,780 | Automated placement/soldering | $0.0017/joint | [jlcpcb.com](https://jlcpcb.com/help/article/pcb-assembly-faqs) |
| Magnet wire | 1×0.05 mm flat self-bonding enameled copper | 78.3 g | The coil bed: 116 windings × 33 turns | $18.74/kg (RFQ budget) | [enameledwires.com](https://enameledwires.com/products/enameled-copper-wire/self-bonding-rectangular.html) |
| Hall position sensor | TI DRV5055A4QDBZR | 49 | 6-DoF pose sensing, 14.29 mm cost-selected grid | $0.348 | [digikey.com](https://www.digikey.com/en/products/detail/texas-instruments/DRV5055A4QDBZR/8567410) |
| Hall group gate switch | GOODWORK AO3401A (LCSC C2938368) | 4 | Powers each mux group only during scan burst | $0.0206 | [lcsc.com](https://www.lcsc.com/product-detail/MOSFETs_GOODWORK-AO3401A_C2938368.html) |
| Hall readout mux | TI CD74HC4067SM96 (LCSC C98457) | 4 | 16-channel analog mux into the MCU ADC | $0.253 | [lcsc.com](https://www.lcsc.com/product-detail/C98457.html) |
| Tile PCB | JLCPCB 4-layer FR4 100×100 mm | 1 | Drivers, sensors, coil terminations | $1.06/tile | [jlcpcb.com](https://jlcpcb.com/news/discount-on-quality-4-layer-pcbs) |
| Tile control MCU | STM32G431KBT6 | 1 | Pose estimation + control loops (12.2× headroom) | $3.13 | [digikey.com](https://www.digikey.com/en/products/detail/stmicroelectronics/STM32G431KBT6/10231564) |
| Backplane connector | ZHOURI 2×10 2.54 mm header (LCSC C5116480) | 1 | Tile-to-mainboard power + serial | $0.0724 | [lcsc.com](https://www.lcsc.com/product-detail/C5116480.html) |
| **Per-tile subtotal** | | | | **$63.08** | |

## B. Per piece — 32 pieces per board

| Part name | Part code | Qty/piece | Purpose | Unit price | Link |
|---|---|---:|---|---:|---|
| NdFeB magnet cube | N48SH 5.00×5.00×5.00 mm ±0.05, Hcj ≥1,592 kA/m, through-thickness magnetized, Ni-Cu-Ni, flux-sorted | 16 | Halbach array in the piece base; SH grade survives the 72 °C hot-cell soak. **RFQ must guarantee flux: model uses Br 1.40 T, catalog N48SH floor is ~1.36 T** — spec sorted lots or re-run the model at the guaranteed minimum. RFQ ~53k pcs (incl. 3.5 % yield) to Mainrich + 2 others | $0.36 (plausible, unverified) | [TO BE SOURCED] ([Mainrich N48SH](https://www.mainrichinternational.com/magnets/n48sh)) |
| Piece body | PLA/resin print + inserts + finish | 1 | Hollow shell over the magnet base; 34 g piece | $1.40 (est.) | [jlc3dp.com](https://jlc3dp.com/blog/3d-printing-cost) |
| **Per-piece subtotal** | | | | **$7.16** | |

## C. Board-shared — one set per board

| Part name | Part code | Qty | Purpose | Unit price | Link |
|---|---|---:|---|---:|---|
| Compute module | Raspberry Pi CM5 2GB Lite (SC1556) | 1 | Game logic, choreography, thermal governor, replay | $61.96 | [digikey.com](https://www.digikey.com/en/products/detail/raspberry-pi/SC1556/25805567) |
| Mainboard | Custom 4-layer carrier | 1 | CM5 + tile backplane + PSU/buffer interconnect | $25.00 (quote basis) | [jlcpcb.com](https://jlcpcb.com/quote) |
| Tile interconnect | HDGC 2×10 2.54 mm socket (LCSC C19725277) | 40 | Mainboard sockets for the tiles | $0.1125 | [lcsc.com](https://www.lcsc.com/product-detail/C19725277.html) |
| Bus power supply | Mean Well UHP-500-12 | 6 | Three isolated ±12 V split-rail zones; sized by full-fleet single-zone hover | $83.30 | [digikey.com](https://www.digikey.com/en/products/detail/mean-well-usa-inc/UHP-500-12/8324034) |
| Bus distribution | Copper 110 flat busbar + zone cabling | 1 | Low-drop zone rail distribution | $36.96 (allowance) | [ebay.com](https://www.ebay.com/itm/304578689563) |
| Rail regen clamp | Active MOSFET dump clamp + TVS for spikes | 6 | Absorbs braking energy; **"TVS + dump resistor" rejected: a 12 V TVS knees near 20 V vs the 40 V FET rating. Needs clamp thresholds, per-event energy, pulse spec before quoting** | $5.00 (est.) | [TO BE SOURCED] |
| Rail bulk capacitance | Zone-level low-ESR electrolytic (Rubycon 25RXA4700 class candidate) | 6 | Zone rail stiffening; **sizing must follow from ripple/inductance spec; tile-local bulk added separately** | $4.00 (est.) | [TO BE SOURCED] |
| Supercap burst buffer | Maxwell BCAP0350-P270-S18, 5s3p per rail | 30 | Covers the 2.2 kJ reset-burst deficit (W1 double reset); 3,238 in distributor stock — allocation advised | $7.65 @1k | [digikey.com](https://www.digikey.com/en/products/detail/maxwell-technologies/BCAP0350-P270-S18/11673891) |
| Supercap balancing | Active balancing network; **topology decision pending (5s3p node balancing vs independent strings)** | 30 | Voltage equalization; Maxwell recommends active balancing for this duty cycle | $0.15 (est.) | [TO BE SOURCED] |
| Buffer charge/protection | BQ33100-class monitor + precharge/charge path + fuse + disconnect, per rail bank | 2 | **Added per sourcing review — was missing entirely.** An ideal-diode discharge path cannot recharge the bank; a controlled charge path, current limit and cell protection are mandatory | $20.00 (est.) | [TO BE SOURCED] ([TI BQ33100](https://www.ti.com/product/BQ33100)) |
| Buffer ideal-diode ORing | TI LM74800-Q1 + paralleled 40 V N-FET pairs, per rail per zone | 6 | Buffer feeds every zone without back-feed; **~103 A deficit path ⇒ ~16 W/pair hot — parallel FETs, busbar attach, transient thermal validation required** | $4.00 (est.) | [TO BE SOURCED] ([LM74800 DS](https://www.ti.com/lit/ds/symlink/lm7480.pdf)) |
| Radiator | 6063-T5/T6 integral-fin extrusion 720×480, 4 mm base, 30 mm fins | 9.33 kg | Passive heatsink for silent mode. **480 mm-wide cross-section needs a 4,500–7,500 t press; quote die/tooling + freight separately; also quote two joined 240 mm sections** | $12.00/kg (material only) | [TO BE SOURCED] ([550 mm capability ref](https://sinoextrud.com/what-is-the-maximum-heatsink-size-we-can-produce/)) |
| Radiator eddy-break slotting | Gang-saw 5 mm crosshatch, 3.5 mm deep, 0.5 mm web | 1 | ~137 m of cut per board; **credible only gang-sawed, not CNC-milled; drawing still needs kerf width, deburr and post-machining flatness spec** | $200.00 (RFQ budget) | [TO BE SOURCED] |
| Coil potting epoxy | Ziitek TIE280-25AB class, 2.5 W/mK | 1 | Coil-bed potting; playing-surface substrate | $45.00 (RFQ budget) | [ziitek.com](https://www.ziitek.com/epoxy-potting-compound) |
| Thermal gap filler | Laird Tputty SF560, 5.6 W/mK, 1.5 mm bond line, 518 cc | 1 | Couples tile PCBs to radiator. Ten-pail public price; **selective dispensing at 0.3–0.5 mm could cut 67–80 % but requires thermal-model rework (contact coverage)** | $336.43 (10-pail public) | [laird.com](https://www.laird.com/products/thermal-interface-materials/liquid-gap-fillers/tputty-sf560) |
| Playing surface | UV print + flood clear wear coat on potting | 1 | Board graphics ≤0.10 mm total. Print-only ~€10.4/board supports the estimate; **wear-coat qualification (Taber, chemicals, CoF, yellowing) still open** | $12.00 (print-only basis) | [TO BE SOURCED] ([supplied-material ref](https://print-shop.hr/doneseni-materijal)) |
| Radiator fan | Noctua NF-A20 PWM 200 mm @ 550 rpm | 2 | Spectate-mode-only cooling; 16.7 dB(A) installed | $39.95 | [coolerguys.com](https://www.coolerguys.com/products/noctua-nf-a20-pwm-200mm-cooling-fan) |
| **Shared subtotal** | | | | **$1,765.53** | |

## Roll-up

| Block | Cost |
|---|---:|
| Tiles (40 × $63.08) | $2,523.15 |
| Pieces (32 × $7.16) | $229.12 |
| Board-shared | $1,765.53 |
| **Board total** | **$4,517.80** |

## Status after sourcing review (2026-08-03)

Adopted at public prices: FPGA (+$74/board), shunts (+$78), Maxwell supercaps (+$50), ORing hardware (+$15), buffer charge/protection (+$40, new line), tile bulk capacitance (+$40, new line), gap filler at 10-pail public (+$186). Mass budget corrected: +1.76 kg filler, +0.15 kg cells → 25.12 kg/board.

Not adopted: 0.1 % front-end passives ($225–430/board) — the Phase-4 sensing architecture zero-calibrates static offsets at idle, so 1 % + matched arrays meet the error budget; the 0.1 % premium buys nothing.

This total still excludes: radiator die/tooling + freight NRE, magnet RFQ variance (Br guarantee), regen clamp final design, wear-coat qualification, VAT/freight/duty. Treat **$4,518 as a public-price floor**, not a finished production estimate.

# BOM sourcing review

Checked 2026-07-14 against the current `model.py` and `last_run.txt`. Prices are
public catalogue prices excluding tax, freight and tooling unless explicitly marked
as an RFQ budget. The reference build is 100 boards.

## Reconciliation with the current model

The review pushbacks are valid:

- 20N06 is retired. The live architecture is one 40 V dual N-channel SOP-8/PDFN
  half-bridge package per coil channel: 108 per tile, 35 tiles, **3,780 packages per
  board**. The remaining task is selecting and RFQing that dual part.
- DRV5055A4 Hall noise is a qualification task, not a present CRITICAL finding. Its
  typical 130 nT/sqrt(Hz) density is about 18.4 uT rms over 20 kHz, leaving 2.7x
  margin to the 50 uT rms budget. A maximum is not guaranteed, so samples still need
  characterization over temperature, supply and lots.
- The current BOM baseline is **$3,095.34 per board**. The obsolete 20N06 scenario and
  previous $3,513-$4,116 ranges no longer apply.
- The live thermal interface is 1.5 mm of dispensed 3 W/mK gap filler, not full-area
  premium sheet stock.
- The live radiator is an integral-fin extrusion with its 4 mm base crosshatch-kerfed
  from the top, leaving a 0.5 mm continuous bottom web and the fins below it.

## Dual half-bridge MOSFET candidates

The model's $0.055/package is still an RFQ target, not a sourced price. Public stock
is also below the full 378,000-package production requirement for every low-cost
candidate.

| Candidate | Package / electrical fit | Resistance at temperature | Public price and board cost | Assessment |
|---|---|---|---:|---|
| **AOS AO4882** | Active dual N, SOIC-8, 40 V, 8 A, 12 nC max gate charge | 19 mohm max at 25 C and **29 mohm max at 125 C**, Vgs=10 V | $0.28134 at 15k DigiKey; **$1,063.47/board** | Best documented SOP-8 electrical fit and approximately 359k DigiKey stock, close to one production lot. Use as the technical/reference candidate and request a 378k authorized-distributor quote. Catalogue cost is $855.57 above the model allowance. |
| **Slkor SL4884A** | Dual N, SOP-8, 40 V, 10 A, 23 nC typical gate charge | 17 mohm max at 25 C; normalized curve suggests about 29 mohm typical at 150 C, but no hot maximum | $0.0638 at 6k LCSC; **$241.16/board** | Closest catalog-priced candidate to the budget. Requires hot-Rds, SOA, switching, traceability and lot qualification. Only about 1.9k catalog stock; production is RFQ-only. |
| **HL 4882** | Dual N, SOP-8, 40 V, 8 A, 12 nC typical gate charge | 22 mohm max at 25 C; no guaranteed hot maximum | $0.0470 at 6k LCSC; **$177.66/board** | Meets the cost target and has about 29k catalog stock. Do not nominate until hot resistance and reliability are characterized; the public datasheet does not guarantee <=35 mohm hot. |
| **Diotec DI048N04PQ2** | Dual half bridge, PowerQFN 5x6, 40 V, 48 A, 48 nC gate charge | 9.6 mohm max at 25 C, providing ample hot-resistance margin | $0.39463 at 10k DigiKey; **$1,491.70/board** | Strong branded PDFN fallback, but expensive and its 48 nC charge exceeds the model's 30 nC switching assumption. Re-run switching/gate loss if selected. |

Recommended RFQ set: AO4882 as the reference/authorized-source part, SL4884A and HL
4882 as cost-down candidates, all at 378,000 plus 2-3% attrition. Require 40 V, 5 A
per leg, Rds(on) <=35 mohm at the specified hot junction temperature and 10 V gate,
dual independent N channels or internally connected half bridge, package thermal data,
UIS/SOA evidence, lot traceability, PCN control, production lead time and samples.

**Severity: MAJOR sourcing/cost risk, not CRITICAL architecture risk.** Suitable parts
exist; the unresolved issue is that the $0.055 price and hot performance have not both
been secured in one production-qualified part.

## Hall sensor comparison against the actual 50 uT rms budget

| Candidate | Sensitivity / range | Noise result | Public price and board cost | Decision |
|---|---|---|---:|---|
| **TI DRV5055A4QDBZR** | 12.5 V/T nominal, +/-169 mT | **18.4 uT rms typical** over 20 kHz; 2.7x below budget | $0.34758 target tier; **$778.58/board** | Current selection. Characterize 30 units from each of 3 lots at minimum/nominal/maximum supply and cold/ambient/hot conditions. Test the real analog chain and scan/settling sequence, not only a static sensor. |
| **TI DRV5055Z4QDBZR** | Same sensitivity and range | Same noise family | $0.70267 at 9k; **$1,573.98/board** | No cost advantage; it also gives up the A-version magnet temperature compensation. Reject as cost-down. |
| **Diodes AH49ENTR-G1** | 16 V/T, +/-100 mT | Datasheet only says low noise; **no numeric density or rms maximum**, so it cannot yet be compared with 50 uT | $0.208 at 1k; **$465.92/board** | Worth a sample characterization because it could save about $313/board. Range is exactly at the requirement with no headroom, which also needs testing against the model's worst combined field. |
| **JSMSEMI OH49E** | 25 V/T, only +/-60 mT | No numeric noise specification | $0.1296 at 6k; **$290.30/board** | Reject: it fails the +/-100 mT range before noise is considered. |

The DRV5055A1/A2/A3/A8 sensitivity options do not meet the required +/-100 mT range;
A4/Z4 are the only members of that family with adequate range. Hall noise is therefore
a **qualification task**, while the AH49E test is a meaningful cost-down experiment.

## Dispensed thermal gap filler

The model requires a nominal volume of:

`639.98 mm x 442.84 mm x 1.5 mm = 425.1 cm3 per board`

That is **42.5 litres / about 127.5 kg per 100 boards** at 3.0 g/cm3, before dispensing
waste. RFQ approximately 47 litres including 10% process allowance.

| Candidate | Specification | Public catalogue indication | Consequence |
|---|---|---:|---|
| **Electrolube GF300** | Two-part dispensable silicone, 3.0 W/mK, 3.0 g/cm3, 55 Shore 00, UL94 V-0 | RFQ only | Exact technical baseline. Request pail/drum pricing, static-mixer loss, cure schedule and automated-dispense trial. The current **$90/board is an RFQ target**, equivalent to about $70.60/kg. |
| **Bergquist TGF3000SF** | Two-part silicone-free filler, 3.0 W/mK, 400 ml cartridge | $282 each at 27 Mouser | About **$300/board nominal** before waste. Useful public ceiling/reference, not the intended production packaging. |
| **Parker THERM-A-GAP GEL 30** | Fully cured dispensable gel, 3.5 W/mK, 300 cc cartridge | $274.90 each at 24 DigiKey | About **$389/board nominal** before waste. Technically suitable but poor at cartridge pricing. |

**Severity: MAJOR BOM qualification risk.** The design decision is sound, but no public
price supports $90/board. A 42.5-47 litre production quote may reduce cost substantially;
until received, $90 must remain labelled as a target rather than an honest sourced price.

## Integral-fin radiator and crosshatch machining

The current physical definition is now consistent with the thermal model: approximately
5.3565 kg of integral-fin 6063-class extrusion per board, plus top-side kerfs through
3.5 mm of the 4 mm base. At 100 boards the extrusion lot is approximately **536 kg**.

- The model's **$12/kg = $64.28/board** is a plausible RFQ budget, but not a public
  quote. AlumHome advertises 6063 custom heat-sink extrusion plus CNC machining with a
  500 kg starting MOQ, which the production lot satisfies. COOLTEC advertises more than
  500 existing extrusion profiles and in-house CNC, offering a second RFQ path that may
  avoid a custom die.
- The **$200/board crosshatch machining allowance** is now the correct model baseline,
  but also remains unquoted. The drawing represents about 112 m of kerf per board.
- The approximately 443-640 mm transverse envelope may exceed a single conventional
  extrusion profile. The RFQ must explicitly permit or reject two or more mechanically
  joined extrusion sections; otherwise suppliers cannot quote the real architecture.

RFQ drawing requirements: alloy/temper and thermal conductivity, finished 639.98 x
442.84 mm envelope, 4.0 mm base, fin height/thickness/pitch, extrusion direction,
5.0 mm crosshatch pitch, kerf width, 3.5 mm cut depth, 0.5 mm remaining web, post-cut
flatness, deburring, finish, section joining policy, tooling ownership, 10/100/500 unit
pricing and tooling separated from recurring unit cost.

**Severity: MAJOR sourcing risk, no longer a thermal-model contradiction.** The concept
has credible suppliers, but neither $12/kg nor $200 machining is yet a quote.

## Other sourcing entries that remain applicable

| Item | Current basis | Status |
|---|---:|---|
| Hall group P-FET | GOODWORK AO3401A, $0.0206, $2.88/board | Electrically suitable. Still require a 5 V-tolerant open-drain/level-shifted control and gate-source pull-up. |
| Setpoint RC pair | 15.8 kohm + 10 nF X7R, $0.003/pair, $11.34/board | Correct 1.007 kHz values and corrected pair count; production stock/RFQ remains open. |
| Flat 1.00 x 0.05 mm wire | $18.74/kg budget, about $68.59/board | Dimensional capability confirmed, exact-size price remains RFQ-only. |
| N52 5 mm cubes | $0.26 each public price, $133.12/board | Real catalogue reference; request a 51,200-piece quote and retain finished-product magnet safety requirements. |
| Potting | Ziitek TIE280-25AB-class, $45/board budget | 2.5 W/mK material exists, price remains RFQ-only. |
| UHP-500-12 supplies | Four at $83.3004, $333.20/board | Model now treats them as two independent isolated +/-12 V zones and separately includes rail bulk capacitance and regen clamps. Written series-operation confirmation remains prudent. |

## Evidence links

- AO4882 official data sheet: https://www.aosmd.com/sites/default/files/res/data_sheets/AO4882.pdf
- AO4882 DigiKey listing: https://www.digikey.com/en/products/detail/alpha-omega-semiconductor-inc/AO4882/3060989
- SL4884A LCSC listing: https://www.lcsc.com/product-detail/C20539695.html
- HL 4882 LCSC listing: https://www.lcsc.com/product-detail/C7543833.html
- DI048N04PQ2 product data: https://diotec.com/en/product/DI048N04PQ2.html
- DI048N04PQ2 DigiKey listing: https://www.digikey.com/en/products/detail/diotec-semiconductor/DI048N04PQ2/22192358
- DRV5055 data sheet: https://www.ti.com/lit/ds/symlink/drv5055.pdf
- AH49ENTR-G1 listing: https://www.lcsc.com/product-detail/C314698.html
- OH49E listing: https://www.lcsc.com/product-detail/C49021369.html
- Electrolube GF300 data sheet: https://electrolube.com/app/uploads/2020/02/GF300_wa75g6.pdf
- Bergquist TGF3000SF listing: https://www.mouser.com/c/thermal-management/thermal-interface-products/?series=TGF+3000SF
- Parker GEL 30 data sheet: https://www.parker.com/content/dam/Parker-com/Literature/Chomerics/datasheets/Parker-Chomerics-THERM-A-GAP-GEL-30-Datasheet-CHODS1115.pdf
- Parker GEL 30 catalogue listing: https://www.digikey.com/en/product-highlight/p/parker-chomerics/therm-a-gap-gel-30-high-performance-fully-cured-dispensable-gel
- Custom extrusion/CNC RFQ supplier: https://www.alumhome.com/
- Existing-profile European extrusion/CNC supplier: https://cooltec.de/en/produkte/luftk%C3%BChlk%C3%B6rper/extrufin-stranggepresste-kuhlkorper/


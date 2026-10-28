# 13: Purchasing Calendar (Oct 2026 → Apr 2027)

Every row in [`bom.csv`](bom.csv) has a `buy_window`. Run `python3 tools/bom_total.py --windows` to see spend per window.
Dates marked *(typ.)* follow the usual pattern but weren't confirmed for this season, so check a week before.

## Sale & shutdown calendar

| Date | Event | Where | Use it for | Notes |
|---|---|---|---|---|
| **1-7 Oct 2026** | China **Golden Week** (National Day) | AliExpress sellers | - | **Shipping pauses.** Orders placed now ship ~8 Oct. Order anyway so you're first in the queue |
| **6-7 Oct 2026** | **Amazon Prime Big Deal Days** (Canada) | Amazon.ca | **Tools**, PETG filament, grease | Prime only (free trial works). [Confirmed dates](https://www.aboutamazon.ca/pbdd-2026) |
| 8-10 Nov → **11-19 Nov 2026** | **AliExpress 11.11 Singles' Day** (warm-up from 8 Nov) | AliExpress, JLCPCB coupons *(typ.)* | Wrist belts, slip ring, the D42 sensing parts, pogo pins, gripper parts (the spring is deferred, D41) | Biggest AliExpress sale of the year. Collect store coupons during warm-up. **Arrives late Nov-mid Dec**, so nothing the J1-J4 build needs can wait for it |
| **27 Nov 2026** (Black Friday) → **30 Nov** (Cyber Monday) | Black Friday / Cyber Monday | Amazon.ca, Bambu/Prusa filament stores, AliExpress *(typ.)* | ASA, TPU, optional PETG-CF, any tools still missing | Canadian retailers match US dates |
| ~**12 Dec 2026** *(typ.)* | AliExpress **12.12** | AliExpress | Top-ups after assembly finds gaps | Smaller than 11.11 |
| **26 Dec 2026 →** | **Boxing Day / Boxing Week** | Amazon.ca, Canada Computers, Best Buy | Tools, a webcam, Raspberry Pi accessories | Canadian sale |
| **~10 Jan 2027** | **Last safe AliExpress + PCB order** | AliExpress, JLCPCB | Everything still missing for the core build | Leaves 3-4 weeks before the New Year slowdown |
| ~25 Jan → ~20 Feb 2027 | **Chinese New Year** (6 Feb) slowdown | AliExpress, JLCPCB | **Don't depend on orders here** | Factories and sellers close ~1-3 weeks. Logistics back up |
| ~**mid-late Mar 2027** *(typ.)* | **AliExpress Anniversary Sale** (2026 ran 16-25 Mar, warm-up 13 Mar) | AliExpress | **Post-core parts**: octopus gripper, Amazing Hand servos | Second-biggest AliExpress sale. Good fit for post-April work |
| mid-Jul 2027 *(typ.)* | Amazon Prime Day | Amazon.ca | Summer v2 work | - |

## Order waves (fastest route that still catches the sales)

| Wave | When | What | ~Cost | Why then |
|---|---|---|---|---|
| **W1** | **Placed 7-8 Oct** | **All 7 motors** (3× NEMA17 60 mm, 1× NEMA23 76 mm, 2× NEMA17 40 mm, 1× NEMA17 20 mm pancake: option B. All seven were bought 8 Oct from Spool3D in Canada, because StepperOnline wanted ~253 to ship from China) **+ 7 MKS drivers (CAN versions)**, PSU, E-stop + fuses, ESP32-S3s, CAN modules, **USB-CAN adapter**, buck converters, hall sensors, **output bearings (6810 ×8, 6806 ×6)**, **HTD-3M belts + pulleys**, **2 output encoders**, bearing/pin/fastener assortments, SimpleFOC kit, logic analyzer | ~USD 605 (incl. ~35 shipping) | Motor sizing is known from the torque budget; the gearbox *type* doesn't change it. Bearings and belts are chosen up front so the gearboxes are designed around them and the J1-J4 build isn't waiting on an 11.11 parcel. Waiting until 11.11 would save ~USD 30-50 but cost ~3 weeks. With 30 h/week, time is worth more |
| **W2** | **6-7 Oct** (Prime Days) | Tools you don't have, PETG ×5, grease | ~USD 350 incl. ~225 of tools | Confirmed sale, 1-2 day delivery |
| **W3** | **mid-late Oct** (decided 12 Oct) | JLCPCB order #1 (power/E-stop + base controller; encoder board C if drawn in time, D33) | ~USD 33 | Gearbox CAD still has priority ([09](09_decisions_open_questions.md) Q7) |
| **W4** | **11-19 Nov** (11.11) | Wrist belts, slip ring, pogo pins; the D42 sensing parts (Ø6 rods, MR126, diametric magnets, hall switches and magnets); the 3-prong gripper's N20, 693ZZ, pins and tool-board modules; J1's dowel pins and inserts. No spring (D41) | ~USD 75 (12 Oct estimate, before the D42 and gripper parts) | Needs the J1-J4 and wrist CAD far enough along to size them |
| **W5** | **27-30 Nov** (BF/CM) | ASA (friend's printer), TPU, optional PETG-CF, missing tools | ~USD 50-85 | No depth camera: the iPhone 14 Pro covers it |
| **W6** | **by ~10 Jan** | PCB order #2 + anything still missing | ~USD 33+ | Before Chinese New Year |
| **W7** | **mid-late Mar** (Anniversary) | Octopus, hand | ~USD 220 | Post-core and outside the USD 1,000 cap, so it can wait for the sale |

## Rules of thumb
- **AliExpress:** buy from official brand stores (StepperOnline, Makerbase/MKS), use "Choice" items to get free
  shipping, and stack store + platform coupons during sales. Budget **2-4 weeks** to Kelowna.
- **Amazon.ca:** use it for things you need within days, plus tools and filament during Prime Days / Black Friday.
- **Price-check high-value items** (motors, drivers, camera) a couple of weeks before a sale, because some listings raise
  prices before "discounting".
- **Keep receipts:** mark rows `bought` in `bom.csv` so `bom_total.py` tracks what's been spent.

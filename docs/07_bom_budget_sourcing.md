# 07: BOM, Budget & Sourcing

**The source of truth is [`docs/bom.csv`](bom.csv).** Run `python3 tools/bom_total.py` for totals in USD and CAD.
Prices are AliExpress/Amazon.ca estimates (Sept 2026) at **1 USD ≈ 1.40 CAD**, with estimated tax
(Ali ~7% incl. GST + occasional duty, Amazon.ca/local 12% GST+PST).

**Budget:** ≤ **USD 1,000** (≈ CAD 1,400) for everything that goes into the robot: motors, drivers, bought boards,
**and your custom PCB orders**. **Tools are tracked separately** (scope `tools`).

## Current estimate

| Scope | USD | CAD | Contents |
|---|---|---|---|
| **Core build** | **~1,031** | ~1,444 | 7 motors + 7 CAN drivers (7-DOF), output bearings, belts/pulleys, pins/fasteners, slip ring, 5 kg PETG + ASA + TPU, base board, PSU/E-stop, ESP32s + CAN, buck converters, output encoders, 2 PCB orders, SimpleFOC testbed, 3-prong gripper, **an estimated USD 15 of shipping**. Up from ~891: the motors came from Spool3D (Canada), and the 8 Oct AliExpress and Amazon orders cost more than the estimates once variants and shipping were real (drivers C$35.79 each, isolated CANable, bearings from Amazon). About USD 565 is spent |
| Core + 10% contingency | ~1,134 | ~1,588 | USD 134 over the 1,000 cap (104 on 8 Oct; 20 Oct: CAD-counted pins, encoders, the gripper and its tool board). Dropping the SimpleFOC testbed (USD 25) is the first cut |
| **Headroom under USD 1k** | **−31** | −44 | None: the core alone is over the cap, so the octopus gripper, hand and any upgrade go further over |
| Upgrades | ~81 | ~113 | Desk webcam, iPhone apps, PETG-CF bench test, link stiffener rods. No depth camera: the iPhone 14 Pro's LiDAR does that job |
| Later (post-core) | ~219 | ~307 | Octopus (+ suction), Amazing Hand |
| **Tools (separate)** | ~258 | ~361 | If you bought *every* tool. Tick off what you have |

**Why the core went up from ~778 (revision of 1 Oct):** USD 35 shipping line that was missing, two more kilos of PETG
(3 kg left no room for supports and reprints), 14 output bearings bought by size instead of a 10-piece placeholder,
wider HTD belts for the pre-stages, buck converters and fuses. 10% contingency is thin for a first build of anything;
if money is tight, the cheapest things to defer are TPU and ASA (not needed before December/January) and PCB order 1.
Option B in [15](15_mass_and_design_options.md) (lighter wrist motors, accepted 7 Oct) is in the BOM; at StepperOnline's
6 Oct prices it costs the same as the wrist it replaced, so the totals did not move.

## Tick-off: tools
Edit the `status` column in `bom.csv` (`need` → `have` / `makerspace`) and totals update automatically.

| Tool | Est. CAD | Have? | Notes |
|---|---|---|---|
| Digital calipers | 25 | ☐ | **Required** |
| Metric hex keys + ball-end drivers | 20 | ☐ | |
| Soldering iron (Pinecil-class) + heat-set insert tips | 55 | ☐ | The makerspace has one. Buy for home builds |
| Multimeter | 30 | ☐ | |
| Wire strippers + flush cutters | 30 | ☐ | |
| Crimper (XT30 / JST-XH / Dupont) | 30 | ☐ | Buy **pre-crimped JST-GH leads** rather than a GH crimper |
| USB logic analyzer (24 MHz, sigrok) | 17 | ☐ | Decodes CAN/I²C/SPI. **Highly recommended** |
| USB-CAN adapter (CANable 2.0 class) | 17 | ☐ | Known-good reference while the ESP32-S3 bridge firmware is untested. **Highly recommended** |
| Solder, flux, wick, fine tweezers | 35 | ☐ | |
| Taps M3/M4/M5 + deburring tool | 25 | ☐ | |
| Threadlocker 243 + heat-shrink kit | 20 | ☐ | |
| Luggage scale | 10 | ☐ | Torque tests |
| Dial indicator + magnetic base | - | makerspace | Backlash tests |
| Oscilloscope, bench PSU, hot air, laser cutter, saw, lathe | - | makerspace | Available |

## Things you already have (not counted)
- **Raspberry Pi 4 B (2 GB)**: later the onboard controller / `ros2_control` host (headless). A Pi 5 only if needed.
- **iPhone 14 Pro**: the first camera (DroidCam/Iriun over Wi-Fi), the Viser UI in Safari, a **6-DoF teleop controller**
  (ZIG SIM Pro streams ARKit pose), and the **depth camera** (LiDAR through Record3D), which removes the USD 150
  depth camera from the list. See [05](05_software_kinematics.md).
- **Free PLA at the makerspace**: all fit tests, jigs, test coupons, and prototype gearboxes.

## Where the money goes and how to save
| Lever | Saves |
|---|---|
| Prototype every gearbox in **free makerspace PLA** before printing PETG/CF | ~USD 20-40 of filament |
| 11.11 / Prime Days / Black Friday timing (see [13](13_purchasing_calendar.md)) | ~USD 40-80 |
| iPhone 14 Pro LiDAR instead of a depth camera | USD 150 (already taken out of the upgrades total) |
| Borrow makerspace tools rather than buying duplicates | up to USD 100 |
| Student funding: department, engineering society, robotics club | Unknown |

## Sourcing notes (Kelowna, BC)
- **AliExpress:** motors (StepperOnline store), drivers (Makerbase store: pick the **CAN** variant of the SERVO42D/57D),
  bearings, pins, electronics. 2-4 weeks. Save each motor's datasheet (phase resistance and inductance) when you order:
  `arm.toml` needs them for the torque-speed curve.
- **Amazon.ca:** filament, tools, fast items. Buy during Prime Days (6-7 Oct) and Black Friday.
- **Local:** steel shaft/rod offcuts if the lathe is needed. Links are printed, so no metal stock is required.
- **Taxes/duties:** GST on imports. BC generally doesn't require marketplaces to collect PST on goods shipped from
  outside Canada. Courier shipments (DHL/UPS) can add brokerage fees, so prefer AliExpress standard shipping.
- **JLCPCB:** batch designs into one order per wave (2 orders planned).

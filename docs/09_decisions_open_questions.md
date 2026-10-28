# 09: Decision Log & Open Questions

## Answered

| Topic | Answer |
|---|---|
| DOF | **7-DOF** (SRS, upper-arm roll J3). Brought up as 6-DOF first |
| Structure | **Fully printed**, no metal tube |
| Printers | Bambu **A1 mini** (own). Friend's **Prusa CORE One L** (enclosed, 300×300×330, **hardened nozzle**), unlimited prints if you supply filament. **Free PLA** at the makerspace |
| Budget | **≤ USD 1,000** (can flex) for motors, drivers, bought boards, and custom PCBs. **Tools tracked separately** |
| CAD | **Fusion 360** |
| Reach / payload | PAROL6/Faze4-based: ~440 mm to the flange, 1 kg at full reach. Arm mass 9.77 kg from the CAD (~8.0 estimated before it) |
| Location / sourcing | Kelowna, BC. AliExpress + Amazon.ca |
| Makerspace | Laser cutter, saw, lathe, oscilloscope, hot air, bench PSU |
| PCB experience | PCB layout + SMD (layouts "not the neatest") |
| Computers | **Windows** laptop, **Raspberry Pi 4 B (2 GB)**, **iPhone 14 Pro** (LiDAR) |
| Licences | **Decide at the very end**, before the repo goes public. No LICENSE files until then |
| Time | **~30 h/week**. Exams are not a scheduling factor |
| Mounting | **Base board**, nobody near the arm |
| Grippers | 3-prong → octopus → **humanoid hand last** (but planned now) |
| Repo | Likely **public**. Own designs **inspired by** PAROL6/Faze4/etc. |
| Interests | **Controls** (career), vision, planning, modelling |

## Decisions

| # | Date | Decision | Status | Rationale |
|---|---|---|---|---|
| D1 | 10-01 | **7-DOF SRS layout**: J1 yaw, J2 shoulder pitch, **J3 upper-arm roll**, J4 elbow, J5 forearm roll, J6 wrist pitch, J7 tool roll. 6-DOF mode = J3 locked | **Accepted** | Joint lock with full pose, null space, full-arm mirroring. See [01](01_requirements.md) |
| D2 | 09-30 | Closed-loop steppers on CAN (MKS SERVO42D/57D) for all 7 joints | Proposed | Critical path. See [10](10_drivers_and_custom_pcbs.md) |
| D3 | 10-01 | Cycloidal J1 (25:1); **J2, J3, J4 = 2:1 belt + 24:1 cycloid (48:1)**, J3 + J4 sharing one actuator design; planetary J5-J7 with **J5 + J6 sharing one ~20:1 design**. To be confirmed by the bench | Proposed (rev. 2) | Faze4-proven layout; ratios from the mass audit (D21) |
| D4 | 09-30 | 24 V + CAN, 4 wires through hollow joints | Accepted | |
| D5 | 09-30 | ISO 9409-1-31.5-4-M5 flange + kinematic coupling + 6-pin pogo | Proposed | |
| D6 | 10-01 | Gripper order: 3-prong → octopus → hand (after April). The tool board supports the hand's servo bus from rev B | Accepted | |
| D7 | 10-01 | **Fully printed**. Big parts one-piece on the CORE One L, small parts on the A1 mini, split-plane fallback in CAD, PLA prototypes at the makerspace | **Accepted** | 60×60 mm PETG link ≈ 25×25 mm Al tube stiffness |
| D8 | 09-30 | Buy joint drivers, design support boards. Custom FOC driver = controls side project | Proposed | |
| D9 | 09-30 | Own Python digital twin (Viser + MuJoCo + CAN), ROS 2 wrapper later | Proposed | |
| D10 | 10-01 | PAROL6-class spec: 190/180/70 mm links, 1 kg at full reach. J2 NEMA23 76 mm 48:1 (design 29.1 N·m, 43 available at standstill), J3/J4 NEMA17 60 mm 48:1 (design 12.9, 19.3 available), J5/J6 NEMA17 40 mm 22.2:1 (design 4.0, 5.2 available), J7 NEMA17 20 mm pancake 15:1 (design 0.8, 1.5 available) | Proposed (rev. 3: rev. 2 + D26) | `tools/torque_budget.py` |
| D11 | 10-01 | J7 continuous via 8-wire 12.5 mm capsule slip ring | Proposed | Quiet, 4 A for tools |
| D12 | 09-30 | SimpleFOC testbed in the first order | Proposed | |
| D13 | - | *(merged into D1: it was the 6- vs 7-DOF decision in earlier revisions)* | - | |
| D14 | 10-01 | Order all 7 motors + drivers now (W1) | Proposed | ~3 weeks faster |
| D15 | 10-01 | ESP32-S3 base controller speaks slcan (Windows + Pi) | Proposed | |
| D16 | 10-01 | **All PETG; no CF required.** ASA for motor-adjacent parts on the CORE One L. PETG-CF only as an optional bench experiment (possible: the friend's printer has a hardened nozzle) | **Accepted** | PAROL6 is PETG. CF is brittle, abrasive, and 1.5-2× the cost |
| D17 | 10-01 | **Pi 4 B 2 GB runs headless** (CAN, safety, ros2_control). Laptop runs UI, sim, vision, MoveIt | Accepted | No purchase needed |
| D18 | 10-01 | **iPhone 14 Pro:** ZIG SIM Pro for ARKit pose teleop; Record3D for LiDAR RGB-D. No separate depth camera | **Accepted** | iOS has no WebXR AR; the phone has LiDAR |
| D19 | 10-01 | **500×500×18 mm plywood base board** carrying arm + electronics, plus a ~5 kg weight or C-clamps | Accepted | Tipping margin |
| D20 | 10-01 | **Licensing:** original designs inspired by others, **credited in `CREDITS.md`**. Proposed licenses: code **MIT**, hardware **CERN-OHL-P-2.0**, docs **CC BY 4.0** | **Deferred** to the end of the project | See the licensing notes below |
| D21 | 10-01 | **One mass model.** `arm.toml` `[[masses]]` (motor + driver + gearbox per joint, links, tool) is the only place masses live; the sim, the spring sizing and `tools/torque_budget.py` all read it. First itemised estimate: 8.5 kg total, 4.8 kg past the shoulder (8.1 / 4.4 kg after D26) | Implemented | The first pass had two mass models that disagreed, and counted the three wrist actuators as 0.9 kg (their motors alone are ~1.0 kg) |
| D22 | 10-01 | **Joint speeds are power-limited:** J2 55 °/s, J3/J4 75 °/s, J5/J6 180 °/s, J7 270 °/s, J1 90 °/s. Torque limits in the sim follow the stepper torque-speed curve at 24 V (the MKS drivers' maximum), planned at 75% of the ideal envelope | Proposed | [03](03_actuators_gearboxes.md) §1b, [14](14_math_guide.md) §11 |
| D23 | 10-01 | **Bearings first:** gearboxes are designed around 6810 (J1-J4) and 6806 (J5-J7) output bearings so they ship with W1. Two output encoders and the pre-stage belts also move to W1 | Proposed | The 11.11 order would have arrived after the build phase it feeds, and the bench test needs the encoders |
| D24 | 10-01 | **Motors beside the axis on J2, J3, J4, J7** (belt-driven), so the harness or slip ring passes through the hollow gearbox; J1, J5, J6 use service loops | Proposed | A coaxial motor blocks the hollow shaft. [03](03_actuators_gearboxes.md) |
| D25 | 10-01 | **Counterbalance geometry:** anchor 100 mm above J2, attached 150 mm along the arm, ~565 N/m (123 N peak, re-sized 7 Oct for the light wrist; 574 N/m, 125 N since the 209 mm shoulder, 20 Oct). **Superseded by D40 (20 Oct): a = 70, b = 98 mm, 1499 N/m, 219 N** | Proposed | Same torque as 80/120 mm at ~883 N/m, with less force on the printed anchor |
| D26 | 10-07 | **Option B, lighter wrist motors:** J5/J6 on a 39-40 mm ~45 N·cm motor (still 20:1), J7 on a 20 mm pancake at 15:1 (3:1 belt + ~5:1 planetary). −0.4 kg at the wrist, J5/J6 margin ~1.2 instead of 1.56. Bought 8 Oct as LDO motors (D29) | **Accepted** (7 Oct), applied to `arm.toml`, BOM and docs | Shorter, lighter wrist; the motor is now weaker than the printed gears, so it cannot strip them (6 Oct check). [15](15_mass_and_design_options.md) |
| D42 | 10-20 | **Joint sensing as drawn ([21](21_checks_and_review.md) §11): hall homing on J1-J7; output encoders on J2, J3 and J4 by a Ø6 rod through the hollow input shaft to an MR126 and board C in a cap at the input end; none on J5-J7. One bus harness with a service loop round each joint housing; the gripper's 6 wires through J7's hollow sun (slip ring). Base electronics in a back strip and the E-stop in a front corner, as software keep-outs (and, later that night, drawn as boxes in `DA_B_Spaces`). Gripper: the docs/06 §2 3-prong design (option A), not the 2-finger one first drawn** | **Decided by Josh** ("do those, gripper (simple one) and routing ... designed for them and have the right spacing", 20 Oct) | Supersedes D33's J3 tube + two 36T pulleys (the rod is simpler and the harness doesn't need J3's centre) and settles D33's open J5/J6 (motor encoder + hall; a pickup can be added) |
| D41 | 10-20 | **Build without the counterbalance spring for now; leave the tower and pulley off. Set `[counterbalance] stiffness = 0` in `arm.toml` the day the arm runs without it** | **Decided by Josh** ("we might not be able to do the spring, but that should be fine?", 20 Oct) | The CAD has no room for it: 118 mm of travel and 219 N can't fit the tower's 179 mm, and the printed lug and tower fail at that load ([21](21_checks_and_review.md) §1). The budget is sized without the spring anyway (snapped-cable rule): J2 keeps 1.35×. The cost is heat and speed on J2. Revisit with a parallel pair along the turntable, a 2:1 purchase or a spring in the base, plus an aluminium lug |
| D40 | 10-20 | **Full-arm CAD layout ([20](20_cad_full_arm.md)): J4 range ±120° (was ±140°); J3 28 mm lower on the upper arm (z 245), J5 40 mm lower on the forearm (z 390, its motor straddles the elbow axis), J7 turned 90° (motor along the J6 axis); counterbalance a = 70, b = 98 mm, 1499 N/m with the CAD masses (1252 with the hand estimates)** | **Decided by Josh** (20 Oct); applied to the CAD and `arm.toml` | Measured in Fusion, not estimated: with J3 at z 273 the forearm hit the upper arm at 96°; lowering J3 and J5 (both roll joints, so the kinematics are unchanged) gives 120° clear, contact from 122°. ±140° would need J3 ~50 mm lower, which puts the counterbalance lug inside link 3's sweep. ±120° is what the KUKA iiwa elbow has. The short a, b keep the lug under link 3 and off the pulley; k·a·b (the balancing torque) is unchanged, the cable tension rises 125 → 219 N. J2 ±120°, J6 ±120° and the rest stay as they were and are clear in the CAD |
| D39 | 10-20 | **Housings stay PETG, with metal wherever a motor face bolts on** (aluminium motor plates or spacers on J2 and J3/J4), and the drivers run **SR_vFOC** by default. J1's motor hangs in open air under the board (36 °C in vFOC by 19 §10.2), so its plate is printed PETG; re-torque after the first warm-up. ASA only for the J7 housing (pancake, the worst heat density) and for the forearm if the J5/J6 motors end up enclosed | **Decided by Josh** (one of the four [19](19_cad_review.md) §12 decisions, 20 Oct) | The heat model (19 §10.2) keeps every housing under 55 °C in vFOC even enclosed, so the driver mode, not the plastic, is what matters; metal in the bolt load path stops preload creep. ASA needs the friend's enclosed CORE One L for every reprint; PETG prints on Josh's own A1 mini. Measure the case temperatures on the bench (19 §10.3) |
| D38 | 10-20 | **J3 goes into the upper arm (the arm stays 7-DOF), J5 goes at the elbow end of the forearm**, so the J6 motor rides in the forearm and turns with J5. Forearm J4 → wrist centre stays **180 mm**, wrist centre → flange **70 mm**; the CAD changes, not `arm.toml` | **Decided by Josh** (20 Oct) | The J3 motor is bought, and the IK, budget and D33 sensing all assume J3. With J5 at the wrist (the 19 Oct CAD) the J6 motor would have to drive through the J5 roll joint. The CAD's 208 mm forearm drops J3/J4 from 1.50 to 1.35 and J2 from 1.48 to 1.41 (`torque_budget.py`, 20 Oct); 190 mm gives 1.44 / 1.46. The 35 mm wrist leaves no room for J7 |
| D37 | 10-20 | **J1 motor coaxial under the base, through a ~Ø64 hole in the base board, with the board on ~70 mm feet**; the harness comes up a slot at the back of the base and loops ±170° in a trough on the base top to the turntable (the 03 plan, made concrete in [19](19_cad_review.md) §14). The base bolts down from under the board | **Decided by Josh** (20 Oct) | Keeps J1 at its exact 25:1 with no extra belt, pulleys, hollow eccentric or bearings. The motor and driver hang in open air under the board (cooler than a PETG pocket) and come out from below. A belt-offset motor needs a hollow input and puts a motor tower beside the base where the upper arm passes past J2 = 120°. The harness is only 4 wires (D4), so a loop is easy. The base frame origin stays the top of the board |
| D36 | 10-20 | **Keep the CAD's shoulder stack; it comes to 209 mm** (`arm.toml` J1 `d` 0.130 → 0.209). First set to 196 (the 19 Oct CAD), then 209 the same day when the J1 rebuild needed a 61 mm base for two 6810s ([19](19_cad_review.md) §14) | **Decided by Josh** (20 Oct); applied to `arm.toml` and the CAD | The NEMA23 sits 92.5 mm below J2 to use the bought 276 mm belt, only 14 mm above the cheek's foot, so neither 130 nor a shorter cheek works without moving the J2 motor and redrawing the cheeks and the counterbalance. Height changes no gravity torque and no tipping number (`torque_budget.py`: identical margins at 130, 196 and 209); it lifts the workspace, and the spring re-sizes 565 → 574 N/m because more low poses are reachable |
| D35 | 10-13 | **Motor direction lives in `arm.toml` (`motor_sign` per joint), not in the driver** | **Forced by the manual** | The MKS Dir setting only applies to the pulse interface (CAN manual V1.0.9 §5.2.7); the earlier plan to set it was wrong. Found on the bench with `python -m deskarm.drivers --jog` |
| D34 | 10-13 | **Joint-angle CAN frame and where the switches go:** output encoders send ID `0x10` + driver id, 4 bytes (14-bit angle, flags, sequence); the host does the coarse + fine sum. The bench MT6701 hangs off the base controller (GPIO10 CSN, 12 CLK, 13 DO) with the same frame. Hall switches go to their joint's MKS driver IN_1, read with 34H | **Proposed**; Josh to confirm, as it puts three pins on board B | One frame format from the bench to board C; no extra wires through the joints for the halls. Firmware README, [16](16_packaging_and_routing.md) §3.3 and §3.8 |
| D33 | 10-13 | **Joint sensing (was Q9):** through-rod + MT6701 on J2/J4 (their harness becomes an external loop), through-tube + two 36T GT2 pulleys + MT6701 on J3, no output encoder on J1/J7 (motor encoder + saved angle + hall switch), J5/J6 decided in the wrist CAD. Parts: MT6701 over SSI, Ø8 × 2.5 mm diametric magnets, board C = ESP32-C3 + MT6701 + SN65HVD230 + buck, A3144-type hall switches | **Accepted** (13 Oct) | A 1:1 pickup on the big outputs needs pulleys as big as the joint; on-axis readings need no side-shaft chip; the motor encoder is absolute within a turn, so a coarse reading restores the exact angle. [16](16_packaging_and_routing.md) §3 |
| D32 | 10-08 | **E-stop stays a hardware power cut; ordinary stops use the MKS "emergency stop" command (F7H)** so the motors stop and keep holding. `deskarm.mks` builds the frames | **Accepted**; untested on hardware | By the efficiency numbers the arm can backdrive unpowered ([04](04_electronics_control.md)), so a power cut may let it sag. Measure it on the bench |
| D31 | 10-08 | **6804 eccentric bearing on J3/J4 as well as J2** (was 6802) | **Accepted** | A 12 mm hollow input leaves 0.4 mm of wall in a 6802 and 2.9 mm in a 6804; about half the shaft stress; one bearing type. Costs web thickness in the disc (4.4 mm against 6.5 mm) |
| D30 | 10-08 | **Wrist planetary 22.2:1** (two stages of sun 14 / planet 19 / ring 52, 3 planets, module 1.0), was 20:1 | **Accepted**, applied to `arm.toml` and docs | J5/J6 margin 1.16 → 1.29, still inside 65 mm (62.5 mm ring OD). Three planets, so the output stage gets an 18 mm face |
| D29 | 10-08 | **Motors from Spool3D (Canada), LDO brand:** 3× LDO-42STH60-2004AC (J1/J3/J4), 2× LDO-42STH40-1684AC (J5/J6), 1× LDO-42STH20-1004ASH (J7), and the J2 NEMA23 LKD57HS76-2804-14 (8 mm D-shaft). StepperOnline direct quoted ~253 shipping; Amazon.ca was ~C$214 | **Bought** (8 Oct, C$192 + tax). The NEMA23's rear shaft end for the MKS magnet is unchecked | Domestic, free shipping, and stronger: J3/J4 margin 1.14 → 1.50 |
| D28 | 10-06 | **Firmware is built with the Arduino IDE.** `firmware/base_controller` is an Arduino sketch (`base_controller.ino` + `src/`); PlatformIO still builds the same `src/` | **Accepted** | Closer to what Josh knows; one less toolchain |
| D27 | 10-01 | **`open_bus` verifies the bridge:** it waits for a firmware-version reply and re-opens the channel, instead of trusting that the commands arrived | Implemented | The ESP32-S3's USB port can reset the chip when the PC opens it, and python-can checks no replies |

## Licensing notes (not legal advice)
- **Ideas, layouts, gear ratios, and dimensions** you learn from reading other projects are not copyrighted. **Files are**
  (STEP/STL/code). Drawing your own geometry from scratch in Fusion, *inspired by* PAROL6/Faze4, is your own work.
- Don't import their STEP/STL as a starting point or copy their code unless you accept their license:
  **PAROL6 = GPLv3** (anything built from its code must also be GPLv3), **Thor = CC BY-SA** (share-alike),
  **Amazing Hand = Apache-2.0 code / CC BY 4.0 mechanics** (permissive: just keep attribution).
- Crediting inspirations (even when not legally required) is good practice and looks good in a portfolio.

## Answered since the first plan

| # | Question | Answer (1 Oct 2026 unless the row says otherwise) |
|---|---|---|
| Q1 | Which iPhone model? | **iPhone 14 Pro**: it has LiDAR, so Record3D makes it the depth camera. The USD 150 depth camera is off the list (D18) |
| Q2 | Does the friend's CORE One L have a hardened nozzle? | **Yes.** The PETG-CF bench comparison is possible, and stays optional (D16) |
| Q3 | Which licences? | **Deferred to the very end** of the project, before the repo goes public (D20). Until then there are no LICENSE files, which legally means "all rights reserved" |
| Q4 | Ready to place **W1**? | **Placed** 7-8 Oct: motors from Spool3D, drivers + CANable + J2 bench parts from AliExpress, bearings and the PSU cart from Amazon.ca. The 11.11 list was set up on 12 Oct |
| Q5 | Which motor list goes into W1? | **(c): rev. 2 + option B** (7 Oct, D26): 3× NEMA17 60 mm, 1× NEMA23 76 mm, 2× NEMA17 39-40 mm (J5, J6), 1× NEMA17 20 mm pancake (J7). All bought from Spool3D on 8 Oct (D29) |
| Q6 | Is ~8 kg and 55-75 °/s on the big joints acceptable? | **Yes, with the light wrist** (7 Oct). Motor placement (option C) and link length (F) are left to CAD |
| Q7 | PCB order 1 in mid-October, or folded into the January order? | **Mid-late October** (12 Oct). Boards A and B, and board C now that Q9 is settled (D33) if it is drawn in time; otherwise C goes in the January order |
| Q9 | How to read each joint? | **D33** (13 Oct): the plan in [16](16_packaging_and_routing.md) §3.7 |
| Q8 (half) | Which supply voltages do the MKS drivers allow? | **12-24 V** (MKS manual). More voltage is not a route to more speed |

## Still open

| # | Question | Why it matters | Default |
|---|---|---|---|
| Q8 | Does the **MKS driver accept streamed setpoints** smoothly? | Decides the control architecture (stream positions, or stream speeds and close the position loop in Python) | Answer on the bench with the first motor. The manual (V1.0.9) says the absolute-position command F5 accepts a new target mid-move (firmware ≥ V1.0.5), which is the behaviour we want |

## Risks

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| J2 torque insufficient / printed gearbox fails | Medium | High | Bench-test the real J2 actuator first (belt + 24:1 cycloid), counterbalance. 43 N·m at standstill against a 32.7 N·m design torque (CAD masses) |
| **Masses are from the CAD and the slicer, never weighed** (9.77 kg; 8.0 estimated before the CAD) | Medium | High | Itemised list in `arm.toml`; weigh the first printed actuator and update; J5/J6 have the least margin (J5 1.18, J6 1.24 with the CAD masses and the 3-prong gripper; 1.29× with the estimates, D30); J3/J4 1.44-1.51× |
| **Wrist planetary teeth** (module 1.0 forced by the 65 mm wrist) | Medium | Medium | ~15 MPa at design torque is about the fatigue level estimated for printed PETG: 15-18 mm face or 5 planets on the output stage; measure on the bench ([03](03_actuators_gearboxes.md) §4) |
| **Arm slower than hoped** (power-limited steppers) | High | Low-Medium | 55-75 °/s on the big joints is already in the plan; time-optimal profiles; the counterbalance; a lighter wrist (D26). More supply voltage is ruled out: the drivers are 12-24 V |
| **Streamed setpoints don't suit the MKS drivers** | Medium | High | First bench test; fallback = speed-mode commands with the position loop in Python |
| **Mechanical design takes longer than the roadmap allows** | High | High | Bearings and belts chosen before CAD; one cycloid profile for three joints; shared actuators (J3/J4, J5/J6); minimum-viable-arm milestones in [08](08_roadmap.md) |
| Budget overrun | **High** | Medium | Core is USD 1,031 and core + 10% is USD 1,134 against the 1,000 cap (20 Oct: up from 1,003 with the CAD-counted pins, output encoders, 3-prong gripper and its tool board) (1,003 / 1,104 on 8 Oct, after the first orders at real prices). Staged orders, 11.11 sale, borrow tools, track `bom.csv` |
| AliExpress delays / CNY shutdown | Medium | High | Waves W1-W6, last one by ~10 Jan (see [13](13_purchasing_calendar.md)) |
| A1 mini print-time bottleneck | Medium | Medium | Freeze-and-print continuously. Friend's printer as overflow |
| Custom PCB doesn't work | Medium | Low | Dev-board + module fallback for every custom board |
| Base-controller firmware fails on the real board (never built on target) | Medium | Low | CANable adapter as a known-good bridge; `open_bus` reports a silent bridge instead of hanging; host tests + compile checks against the real header shapes |
| Backlash + wind-up → poor repeatability and accuracy | **High** | Medium | Published printed drives have ~0.5-1° lost motion: at 450 mm reach that is ~4 mm per joint at J1/J2, ~14 mm worst-case stacked. Gear wind-up adds sag under load: ~20 mm at the tool, stretched out with 1 kg, with the placeholder stiffnesses (`deskarm` sim numbers). The ≤ 0.5 mm target needs **output encoders + dual-loop** (or same-direction approach moves), not just a good disc. Also 2-disc cycloidal |
| Motor heat softens PETG | Medium | Medium | Lower holding current, standoffs, temperature logging |
| Scope creep | High | Medium | Phase exits. Octopus/ROS 2 cut first if behind |

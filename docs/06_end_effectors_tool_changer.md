# 06: End Effectors & Tool Changer

The goal: **one standard interface on J7** that every gripper plugs into. Swapping tools should take
under 30 seconds by hand, and the arm should know which tool is attached.

---

## 1. Tool interface spec (v1 draft)

### Mechanical
| Item | Spec |
|---|---|
| J7 output → changer (arm side) | **ISO 9409-1-31.5-4-M5** bolt pattern (31.5 mm pitch circle, 4 × M5), Ø6 locating pin |
| Location | **Kinematic coupling**: 3 steel balls (Ø6 mm) on the tool plate seat into 3 V-grooves (pairs of Ø4 mm steel dowels pressed into the arm plate) at 120°. The tool lands in the same place every time with no play |
| Retention | **Quarter-turn bayonet** with a spring-loaded detent (v1, manual) + 2-4 N52 magnets to hold the tool while it is being latched |
| Stretch (v3) | Servo-actuated or passive latch that the robot operates itself, with a tool rack on the desk (auto tool change) |
| Load rating | 1 kg tool + payload, 5 N·m bending |
| Material | Printed PETG plates with the steel balls/dowels pressed in, and heat-set inserts |

### Electrical: pogo-pin block (6 pins, 2.54 mm pitch, spring-loaded on the arm side, flat pads on the tool)
| Pin | Signal |
|---|---|
| 1 | +24 V (fused, ~3 A) |
| 2 | GND |
| 3 | CAN-H |
| 4 | CAN-L |
| 5 | TOOL_DETECT (tied to GND on the tool, so the arm sees the tool is present before powering it) |
| 6 | Spare (e.g. a thermistor, or a second GND) |

Put the pins in the middle of the bayonet so they only make contact once the tool is fully seated.
Power the pins only after TOOL_DETECT reads present.

### Through the J7 slip ring (continuous rotation)
Use an **8-wire, 12.5 mm capsule slip ring** (gold-on-gold contacts, ~2 A per circuit):
2 circuits for +24 V and 2 for GND (4 A total, ~96 W, plenty for grippers), plus CAN-H, CAN-L, TOOL_DETECT, and a spare.
- **Noise:** mechanically, these rings are near silent at J7 speeds (well under their ~250-300 rpm rating) and add
  only a little drag torque. The steppers and printed gears will be far louder. Electrically, gold capsule rings
  are commonly used for CAN. Keep the pair twisted right up to the ring.
- Mount it through the hollow J7 gearbox: the stator is fixed to the forearm/wrist, and the rotor turns with the flange.
  **Drawn 21 Oct** ([21](21_checks_and_review.md) §11): body in an arm on link 6 under the module, rotor turned by a
  tube up the hollow sun that is glued in the output flange.

### Pneumatic (optional, for suction on the octopus gripper)
Easiest option: put the mini vacuum pump + solenoid **on the tool itself** and power it from 24 V. This avoids running an air line through the arm.

### Software contract (every tool implements this over CAN)
```
TOOL_ID           → {type, version, tcp_offset, mass, com}
OPEN(pos%)        CLOSE(pos%, force%)        STOP
STATUS            → {position, current/force, error}
+ tool-specific commands (e.g. finger-by-finger for the hand, per-tentacle for the octopus)
```
Each tool has its own small MCU (ESP32-C3 + CAN transceiver) and its own actuator drivers, so the
arm never needs a firmware change for a new tool.

---

## 2. Gripper #1: "Dummy" 3-prong gripper (the standard tool, build first)

Three fingers spaced 120° apart that close toward the center, like a lathe chuck or a Robotiq 3-finger gripper.

| Design option | Pros | Cons |
|---|---|---|
| **A. One motor, central lead screw + linkages (recommended v1)** | Self-centering, all fingers synchronized, one actuator, lead screw holds position when power is off | Fingers can't move independently |
| B. Three serial-bus servos (Feetech STS3215 class) | Each finger controlled separately, position and load feedback | 3× the cost and weight, servo gear backlash |
| C. Underactuated 2-phalanx fingers (tendon or linkage) | Wraps around irregular shapes like a real hand | More parts |

Plan: **A** with **Fin Ray TPU fingertips** (compliance for free), aperture 0-80 mm, grip force ≈ 15-20 N.

**Drawn 20 Oct (option A, [21](21_checks_and_review.md) §12):**
- **Fingers and drive:** three pivoting fingers, driven through a pin-in-slot on each finger's tail by a carrier on
  an N20's M3 lead-screw shaft. A 693ZZ takes the thrust.
- **Aperture:** 6-80 mm (the fingertips can't meet).
- **Tips:** 123 mm from the J7 flange, with the TCP at 110 mm.
- **Mass:** 0.119 kg with the pads.
- **Tool board:** a 40 × 36 mm space between two fingers.
- **Pads:** 2 mm TPU pads, glued (21 Oct); the Fin Ray tips come later.
First plan (6 Oct): a small NEMA14/NEMA11 stepper or N20 gear motor + TR8 lead screw, under 300 g. As drawn: the N20 on
its own M3 lead screw, 0.119 kg. Read the motor current for rough force sensing.

---

## 3. Gripper #2: Octopus-inspired soft gripper

Based on **SpiRobs** (logarithmic-spiral tentacles, 2 tendons each) and the Fab Academy
2026 "Octopus Arm" (3 TPU tentacles + servo-driven Dyneema tendons as a UR3 end effector).

| Element | v1 plan |
|---|---|
| Tentacles | 3 tentacles at 120°, **logarithmic-spiral tapered** segments printed in TPU (85A-95A) or as a rigid-segment chain |
| Actuation | 1 servo per tentacle, 2 antagonistic tendons (Dyneema/PE braided line, 0.3-0.5 mm) wound on one pulley, which lets it curl and uncurl |
| Grasp | Close all tentacles to wrap objects. Small taper angle gives a tighter spiral (per the Harvard results) |
| v2 add-on | Small silicone **suction cups** near the tips + mini vacuum pump (the Harvard/Beihang approach) |
| v3 idea | 2 tendon pairs per tentacle for bending in 3D (4 tendons), or 2 segments per tentacle |

**First step:** print one tentacle on the bench, drive it with one hobby servo, and tune the taper,
segment spacing, and tendon routing before building a full gripper.

---

## 4. Gripper #3: Human-like hand

| Path | Cost | Notes |
|---|---|---|
| **Adapt the Amazing Hand (Pollen)** (recommended) | < €200 | 4 fingers, 8 DOF, 8 hobby servos, 400 g, open CAD + Python. Needs a tool-changer adapter plate and our CAN tool MCU (or a bridge to its servo bus) |
| Single-actuator soft hand (BPI SoftHand style) | ~$50 | Human-like wrapping grasps, but only one open/close motion |
| Tendon-driven 5-finger (RUKA / ORCA ideas) | $500+ | Research grade. A long-term goal |

**Payload impact:** a 400 g hand is heavy. The 1 kg rated payload is on top of the gripper, and the mass list now has the
drawn 3-prong gripper (~0.12 kg), so a hand uses ~0.3 kg of the payload. Add its entries to `arm.toml` `[[masses]]`
and re-run `tools/torque_budget.py`.

### Humanoid hand plan (built last, after April, but designed for now)

**Base design:** Pollen's **Amazing Hand**: 4 fingers × 2 DOF = 8 DOF, 2 **Feetech SCS0009** serial-bus servos per
finger (6 V, ~2.3 kg·cm, 10-bit magnetic encoders), rigid printed bones with TPU shells, ~400 g, and a 5 V / 2 A supply
is enough. **License: code Apache-2.0, mechanical CC BY 4.0**, so you may modify it and publish your own version
**with attribution**. The plan is to make it *yours*: a new palm and wrist mount for our tool plate, a thumb
layout you design, and your own firmware on the tool MCU.

**Decisions to make now, so the hand drops in later without rework:**
| Where | Requirement for the hand | Status |
|---|---|---|
| Tool board (PCB rev B, Jan) | **5-6 V / ≥ 3 A buck** from 24 V + **half-duplex UART** (Feetech SCS bus: one data line, 1 Mbit) driven by the ESP32 | Put both on the tool-board design, even if unpopulated for the 3-prong gripper |
| Power path | 8 servos, ~2 A typical at 5 V ≈ 10-15 W → < 1 A at 24 V through the slip ring (rated 4 A) | Fits |
| Tool protocol | Tool reports `type=hand`, `dof=8`, and accepts `SET_JOINTS[8]` + `READ_JOINTS` + grasp primitives (power, pinch, point) | Add to the CAN tool protocol spec |
| Wrist torque | 400 g hand + held object on J5 and J6 | Covered while the hand plus the object stay within the gripper + 1 kg the budget assumes (~1.1 kg): J5 design 4.4 N·m, 5.2 N·m available with the 40 mm motors at 22.2:1 |
| Software | **MediaPipe Hands → `dex-retargeting` → 8 servo angles**, running on the laptop, on the same pipeline as hand mirroring | Build in Phase 6, test in sim (MuJoCo hand model) before the hardware exists |

**Build steps (post-April, ~3-4 weeks):**
1. Print the stock Amazing Hand fingers (PETG bones + TPU shells), bench-test with a USB-serial adapter and their Python code.
2. Design your own palm + tool-plate mount + cable path in Fusion (credit Pollen in `CREDITS.md`).
3. Port control to the tool MCU (CAN ↔ SCS bus bridge).
4. Retarget your hand in real time: MediaPipe → dex-retargeting → hand, with the arm following your wrist.
5. Stretch: fingertip force estimation from servo load readings, then tactile pads.

---

## 5. Build order for tools
1. Tool-changer plates + pogo block + tool MCU (test with a "dummy load" tool that just reports its ID)
2. 3-prong gripper
3. One octopus tentacle on the bench, then the full octopus gripper
4. Hand adapter

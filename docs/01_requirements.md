# 01: Requirements & Target Specs

These are **v1 targets**, benchmarked against **PAROL6** and **Faze4** (both by PCrnjak).
They will change once the actuator test bench produces real numbers. Torques and masses below come from
`python3 tools/torque_budget.py`, which reads the mass list and drive data in
[`software/deskarm/data/arm.toml`](../software/deskarm/data/arm.toml). The masses were hand estimates until 20 Oct
(revised 1 Oct after a mass audit: the first pass counted the three wrist actuators as 0.9 kg, less than their motors
alone) and come from the CAD and the slicer since; the numbers here are from 26 Oct.

## Reference arms

| | PAROL6 | Faze4 | **This arm (v1 target)** |
|---|---|---|---|
| Reach | 400 mm | Not stated in the README (larger arm) | **~440 mm** (J2 axis → flange) |
| Payload | 1 kg | - | **1 kg at full reach** |
| Repeatability | 0.08 mm (claimed) | - | ≤ 0.5 mm (stretch 0.2 mm) |
| Mass | 5.5 kg | ~14-15 kg | **9.77 kg** from the CAD (5.53 kg past the shoulder) |
| DOF | 6 | 6 | **7** (6 with J3 locked) |
| Motors | 6× NEMA17 (3× 65 N·cm, 2× 45 N·cm, 1× 16 N·cm pancake) | 3× NEMA23, 2× NEMA17, 1× NEMA14 | 1× NEMA23, 6× NEMA17 |
| Reducers | Precision planetary + belts | Printed cycloidal + belts (J1-J5), planetary (J6) | Printed cycloidal (J1), belt + cycloidal (J2-J4), planetary (J5-J7) |
| Material | PETG | Printed | PETG (ASA next to hot motors) |
| Structure | Printed PETG | Fully printed | **Printed** (PETG box-section links) |
| Cost | Kit sold | ~$1,000-1,500 | ~USD 1,031 core |

Being PAROL6-sized keeps it desk-friendly. The Faze4-style printed cycloidal drives are what keep it cheap.

## Goals

1. A desk-side **7-DOF** (redundant, SRS) arm that can pick and place objects up to 1 kg anywhere in a
   ~440 mm hemisphere (PAROL6-class), and can run as 6-DOF with one joint locked.
2. Built mostly from 3D prints. Only motors, electronics, bearings, fasteners, and
   filament are bought.
3. A **swappable end effector** with a standard mechanical and electrical interface. Planned tools:
   - "Dummy"-style 3-prong gripper (the standard tool, built first)
   - Octopus-inspired soft/tentacle gripper
   - Anthropomorphic (human-like) hand
4. A learning platform covering kinematics, gear design, motor control, and ROS 2, with room to
   grow into vision and learning-based control.

### What the joints are (J1-J7)
Joints are numbered from the base outward. J1 = base swivel (like turning at the waist), J2 = shoulder,
J4 = elbow, J5 = forearm twist, J6 = wrist bend, **J7 = the last joint, which spins the gripper about its
own axis** (like turning a doorknob or a screwdriver). "Continuous J7" means it can keep spinning in one
direction forever. That needs a slip ring to pass power + CAN to the tool without the cable twisting.

## Non-goals (for v1)

- Industrial precision (< 0.1 mm), heavy payloads (> 1 kg), high speed.
- Automatic tool changing. v1 swaps tools by hand. An auto-swap rack is a stretch goal.
- Force/torque sensing at the wrist.

## Kinematic layout

Industrial "anthropomorphic" arm with a **spherical wrist** (the J5, J6, and J7 axes intersect at one point):

```
        J7 roll  ─┐
        J6 pitch  ├─ spherical wrist (axes intersect → analytic IK)
        J5 roll  ─┘
          │  forearm  (~180 mm)
        J4 pitch (elbow)
          │  upper arm (~190 mm)
        J3 roll (upper-arm roll, 7th DOF)
        J2 pitch (shoulder)
        J1 yaw (base)
      ═══════════════  desk clamp / weighted base
```

Why this layout:
- Inverse kinematics splits into "position from J1-J4" and "orientation from J5-J7"
  (Pieper's method), so it has a closed-form solution, which is much easier than numerical IK.
- It matches PAROL6, Thor, AR4, and most commercial arms, so their docs and URDFs are
  useful references.

## Target specs

| Spec | v1 target | Stretch | Notes |
|---|---|---|---|
| Reach (J2 axis → flange) | **~440 mm** | - | 190 + 180 + 70 mm links. PAROL6 is 400 mm |
| Payload (beyond gripper) | **1.0 kg at full reach** (PAROL6-class) | 1.5 kg, moved slowly (every joint still holds it: ≥ 1.67× its static load) | From `tools/torque_budget.py`, with the drawn 3-prong gripper (~0.12 kg with its motor and board). A ~0.4 kg human-like hand would use ~0.3 kg of the payload |
| Repeatability | ≤ 0.5 mm | ≤ 0.2 mm | Depends on backlash and stiffness. Output encoders help |
| Max joint speed | J1 90 °/s, **J2 55 °/s, J3/J4 75 °/s**, J5/J6 180 °/s, J7 270 °/s | - | Power-limited: at 24 V a stepper delivers ~11-22 W at the joint whatever the ratio, and lifting 15 N·m at 55 °/s already takes 14 W. The MKS drivers accept 12-24 V, so more voltage is not available. See [03](03_actuators_gearboxes.md) §1b |
| Arm mass | **9.77 kg** from the CAD (5.53 kg past the shoulder; 8.0 and 4.4 estimated before the CAD) | < 7.5 kg | PAROL6 is 5.5 kg (6-DOF, smaller motors, no drivers on the arm). Seven steppers + drivers alone are ~3.7 kg. Where it is, what it costs and what each fix buys: [15](15_mass_and_design_options.md) |
| Base footprint | ≤ 200 mm dia | - | Clamps to a desk edge or bolts to a weighted board |
| Print volume | Multi-part assemblies, **most parts ≤ 170 mm** | One-piece variants ≤ 280 mm | A1 mini by default. Friend's Prusa CORE One L (300×300×330, enclosed) for big parts and ASA, see [12](12_printing_fabrication.md) |
| Supply | 24 V DC | - | One PSU, e-stop cuts motor power |
| Wires through arm | 4 (24 V, GND, CAN-H, CAN-L) | - | Everything else is local to each joint |
| Budget | **≤ USD $1,000** (≈ CAD 1,400) incl. tax, **tools tracked separately** | Can flex higher | Core ≈ USD 1,031 (≈ 1,134 with 10% contingency, USD 134 over the cap; 104 on 8 Oct, before the CAD-counted pins, encoders and gripper), see [07](07_bom_budget_sourcing.md) |
| Schedule | Feature-complete **early March 2027** (~30 h/week) | Mid-Feb | See [08](08_roadmap.md) |
| J7 rotation | **Continuous** (slip ring) | - | Lets the gripper spin/screw without winding up its cable |

## Joint table (revision 2, after the mass audit)

From `tools/torque_budget.py` (26 Oct), 1 kg payload at the TCP plus the drawn 3-prong gripper. **Static** is the worst-case
gravity load over all poses; **design** = static × 1.5 (acceleration) × 1.3 (safety). **Available** is what the drive
delivers near standstill (0.75 × holding torque × ratio × efficiency) and at the joint's speed limit, where a stepper
has less. A joint passes if *available at standstill ≥ design* and *available at full speed ≥ static × 1.3*.

| Joint | Motion | Range (target) | Static / design | Drive | Available (standstill / full speed) |
|---|---|---|---|---|---|
| J1 | Base yaw | ±170° | 0 / 2.8 N·m (inertia only) | NEMA17 60 mm + cycloidal 25:1 (ratio chosen for stiffness and resolution, not torque) | 11.2 / 11.0 N·m |
| J2 | Shoulder pitch | ±120° | **16.8 / 32.7 N·m** | NEMA23 76 mm + **2:1 belt + 24:1 cycloidal = 48:1**, plus counterbalance spring | 43.1 / 27.3 N·m at 55 °/s |
| J3 | Upper-arm roll (7th DOF) | ±170° | 6.9 / 13.4 N·m (worst case: forearm held sideways) | **Same actuator as J4**: NEMA17 60 mm + 2:1 belt + 24:1 cycloid = 48:1, **hollow** for the harness | 19.3 / 13.2 N·m at 75 °/s |
| J4 | Elbow pitch | **±120°** (was ±140°: the CAD folds to 120°, contact from 122°, [20](20_cad_full_arm.md)) | 6.6 / 12.8 N·m | NEMA17 60 mm + 2:1 belt + 24:1 cycloidal = 48:1. In the CAD the motor rides on the upper arm's side of the joint, alongside the forearm | 19.3 / 13.2 N·m at 75 °/s |
| J5 | Forearm roll | ±180° | **2.2 / 4.4 N·m** (worst case: wrist bent, tool held sideways) | **Same actuator as J6**: NEMA17 40 mm + 2-stage planetary 22.2:1, at the elbow end of the forearm | 5.2 / 3.8 N·m at 180 °/s |
| J6 | Wrist pitch | ±120° | 2.1 / 4.2 N·m | NEMA17 40 mm + 2-stage planetary 22.2:1. Folding the arm doesn't relieve the wrist | 5.2 / 3.8 N·m at 180 °/s |
| J7 | Tool roll | **Continuous** (±360° in software for now) | 0.4 / 0.7 N·m (payload 30 mm off-axis) | NEMA17 20 mm pancake + 3:1 belt + ~5:1 planetary = 15:1, motor **beside** the axis so a 12.5 mm 8-wire slip ring passes through | 1.5 / 0.9 N·m |

Motors (bought 8 Oct from Spool3D, Canada): LDO-42STH60-2004AC (60 mm, 0.85 N·m) on J1/J3/J4, LDO-42STH40-1684AC
(40 mm, 0.43-0.45 N·m) on J5/J6 and LDO-42STH20-1004ASH (20 mm pancake, 0.18 N·m) on J7, with their listed resistance
and inductance in `arm.toml`. J2 is the NEMA23 LKD57HS76-2804-14 (76 mm, 1.9 N·m, 8 mm D-shaft), bought in the same
order. The light wrist is **option B**, accepted on 7 Oct ([15](15_mass_and_design_options.md), D26):
0.4 kg less at the wrist than the rev. 2 48 mm motors. With the estimated masses every margin was ≥ 1.29 (1.16 before the wrist
went from 20:1 to 22.2:1 on 8 Oct); with the CAD masses and the 3-prong gripper the lowest is J5 at 1.18.

What changed from the first pass, and why:
- **J2: 30:1 → 48:1.** Realistic actuator masses raise its design torque from ~26 to ~32 N·m. The belt pre-stage also moves
  the motor off the joint axis, which the hollow-shaft harness route needs anyway.
- **J3/J4: 40:1 → 48:1.** At 40:1 the drive delivered ~10.7 N·m against a 12.6 N·m design torque even with the old masses.
- **J5: 10:1 → 20:1 with the J6 motor.** With the wrist bent 90°, J5 carries the same tool moment as J6. The first pass
  only gave it an off-axis payload (0.8 N·m), and at 10:1 it could not hold a 1 kg payload sideways.
- **J5/J6 are still the thinnest margin** (1.29× design with the estimated masses and the conservative 0.43 N·m, after
  the wrist planetary went from 20:1 to 22.2:1 on 8 Oct, D30, from 1.16×; J5 1.18 and J6 1.24 with the CAD masses). J3/J4 went from 1.14× to 1.50× with the 0.85 N·m LDO 60 mm motors.

The shoulder (J2) is the hardest joint. There are three ways to reduce its load:
1. A **spring counterbalance** (more than halves the static load stretched out with 1 kg: 16.6 → 6.3 N·m at 1499 N/m,
   `python -m deskarm.counterbalance`). Deferred on 20 Oct (D41): no room in the CAD for its travel and load.
   The budget above is deliberately *without* it, so a snapped cable can't drop the arm.
2. **Move the J4 motor down** toward the shoulder and drive the elbow through a longer belt.
3. Keep the wrist light (short motors, printed parts with low infill where stiffness allows).

## 6 DOF or 7? → **7-DOF (SRS) with a 6-DOF fallback**

**Decision (accepted 1 Oct 2026): build a 7-DOF arm, and bring it up first as a 6-DOF arm by locking J3.**
The 7th joint is an **upper-arm roll ("J3")** right after the shoulder. That gives the
**S-R-S layout** (Spherical shoulder, Revolute elbow, Spherical wrist) used by the human arm, KUKA iiwa,
Franka, and OpenArm.

| | 6-DOF | **7-DOF SRS (chosen)** |
|---|---|---|
| Lock one joint and **still reach any position + orientation** | No: only 5 DOF left | Yes: 6 DOF left |
| Move the elbow while the tool stays still (**null-space motion**) | No | Yes: steers clear of joint limits, obstacles and singularities |
| Mirror your **whole arm** (MediaPipe Pose gives shoulder/elbow/wrist) | No: hand pose only | Yes: elbow swivel maps 1:1 |
| IK | Closed-form | Closed-form with an **arm-angle** parameter, or numerical + null-space tasks (a strong controls topic) |
| Cost / mass / time | Baseline | **+~USD 50**, +0.95 kg 60 mm from the shoulder (**+0.6 N·m static on J2, ~+1.1 N·m design**), +1-2 weeks |

**Why the risk is small:**
- **J3 reuses the elbow actuator design** (NEMA17 60 mm + 2:1 belt + 24:1 cycloid). In the worst case it holds the
  whole forearm sideways, so it needs the same ~14 N·m as the elbow. That's one gearbox design, printed twice.
- **Fallback built in:** with J3 locked at 0°, the arm *is* the PAROL6-class 6-DOF arm, with closed-form IK. Bring-up,
  the tool changer, and vision can all run in 6-DOF mode, then J3 is unlocked for redundancy work.
- Packaging: a pure SRS arm has zero offsets, which keeps the arm-angle IK closed-form. If CAD needs small
  offsets (Franka has them), numerical IK still works fine.

**Joint numbering (final):** **J1** base yaw, **J2** shoulder pitch, **J3** upper-arm roll, **J4** elbow pitch, **J5** forearm roll, **J6** wrist pitch, **J7** tool roll. "6-DOF mode" means J3 is locked at 0°.

## Base board & mounting

Nobody works near the arm, so it sits on its own **base board**, which also carries the electronics:

- **500 × 500 mm, 18 mm plywood** (cut with the makerspace saw), on **~70 mm feet** with a hole under the base for the
  J1 motor and its driver, which hang below the board (D37, 20 Oct; was rubber feet), the arm bolted at the center through
  threaded inserts or T-nuts.
- PSU, E-stop, base-controller PCB, and the Pi are mounted on the board behind the arm, with the E-stop at the front edge.
- **Tipping check** (printed by `tools/torque_budget.py`): at full reach the J2 moment is ~16 N·m static (~24 N·m
  accelerating). With the arm at the center, the board edge is 0.25 m away, so you need ≥ 6.8 kg total standing still,
  ≥ 10.1 kg while accelerating, and ~13.2 kg for a 1.3× margin. The arm (9.77 kg from the CAD with the drawn gripper,
  26 Oct), board (~2.7 kg), and PSU (~1 kg) come to ~13.5 kg, just enough. **Add two C-clamps to the desk, or a ~5 kg paving stone / steel plate
  under the board for margin** (5% over the need today).
- No people nearby means faster test speeds are OK, but the **E-stop, current limits, and soft limits still always apply**.

## Safety requirements

- Hardware **E-stop** that cuts motor power (24 V), not just software.
- Current limits set in every driver. Start at low current while testing.
- Soft joint limits in software and hard stops printed into the joints.
- Homing on every power-up (hall sensors or output encoders).
- Keep the first powered tests slow (≤ 20 °/s) and stand outside the reach envelope.

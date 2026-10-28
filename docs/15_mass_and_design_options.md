# 15: Mass and Design Options: what "accepting the design" means

This is the long answer to Q5 and Q6 in [09](09_decisions_open_questions.md): is an ~8 kg arm that moves its big
joints at 55-75 °/s acceptable, and which motors should be ordered? Every number comes from
`python3 tools/design_options.py` and `python3 tools/torque_budget.py`, which read
[`arm.toml`](../software/deskarm/data/arm.toml). Motors are catalogue values. **§1-3 and §6 use the 7-8 Oct hand
estimates (7.98 kg), which framed the decision; §4 and §5 are re-run with the CAD and slicer masses** (9.77 kg on 26 Oct,
after the edge treatment). The shares in §1-2 moved little: the CAD put 1.8 kg more on the arm, mostly in the
printed housings.

> **Decided 7 Oct 2026:** the layout, reach and 1 kg rating are accepted, and so is **option B, the light wrist**
> (D26): a 39-40 mm ~45 N·cm motor on J5/J6 at 20:1 (22.2:1 since 8 Oct, D30) and a 20 mm pancake on J7 at 15:1. It is now the baseline in
> `arm.toml`, the BOM and every doc. The rev. 2 wrist it replaced is kept below as option B for comparison.
> **8 Oct:** the NEMA17s were bought as LDO motors from Spool3D (D29): 0.85 N·m on J1/J3/J4, 0.43-0.45 N·m on J5/J6,
> 0.18 N·m on J7, and the J2 NEMA23 is an LKD57HS76-2804-14 (1.9 N·m, 1.1 kg). The tables below use them.

## Short version

- **The mass is not the problem it looks like.** 3.6 kg of the 8.0 kg sits on the base and turntable, where no joint
  has to lift it. The shoulder lifts 4.4 kg, and **48% of its load is the payload and gripper themselves**.
- **The design works as specified.** Every joint passes its torque budget with 1 kg at full reach. The price is speed:
  J2 at 55 °/s, J3/J4 at 75 °/s, because 24 V steppers are power-limited ([03](03_actuators_gearboxes.md) §1b), and the
  MKS drivers only accept 12-24 V.
- **No single change transforms it.** Motors moved toward the shoulder, a shorter arm: each buys 5-15% more margin and
  speed. Together about 20%.
- **What was taken:** the light wrist, because it was the only option that changed what gets ordered. Motor placement
  (C) and link length (F) are decided in CAD.

## 1. Where the mass is

| Group | Mass | Does a joint lift it? |
|---|---|---|
| Base housing + J1 motor, driver, gearbox | 1.40 kg | No (bolted to the board) |
| J2 motor (NEMA23, 1.1 kg) + driver + belt stage + cycloid + turntable | 2.19 kg | No (turns with J1 about a vertical axis) |
| J3 actuator, upper-arm link, J4 actuator | 2.25 kg | J2 |
| J5 actuator, forearm link, J6 actuator | 1.34 kg | J2, J4 (and J3) |
| J7 actuator, tool changer, gripper | 0.80 kg | J2, J4, J6 (and J3, J5) |
| **Total** | **7.98 kg** | **4.39 kg is lifted by the shoulder** |

Seven motors and seven drivers are 3.7 kg of the total (3× 0.50, 1.1, 2× 0.28, 0.13, plus ~0.4 kg of driver boards).
The rest is printed gearboxes and housings with their bearings and pins (~3.2 kg, the least certain numbers), the two
printed links (0.65 kg) and the tool changer and gripper (0.43 kg). Weigh the motors when they arrive.

## 2. Where the shoulder torque comes from

Arm stretched out horizontally with 1 kg: J2 holds 14.9 N·m (figure in [14](14_math_guide.md) §10).

| Contribution | N·m | Share |
|---|---|---|
| Payload, 1 kg at 520 mm | 5.10 | 34% |
| Gripper + tool changer | 2.03 | 14% |
| J6 + J7 actuators (at the wrist) | 3.30 | 22% |
| J5 actuator + forearm link | 2.02 | 14% |
| J4 actuator + upper-arm link | 1.91 | 13% |
| J3 actuator (60 mm from the axis) | 0.56 | 4% |

Two things follow. The payload spec is the biggest single lever: rating the arm at 0.5 kg takes 2.5 N·m off J2. And
among the parts you design, **only the wrist actuators matter much**: a gram at the wrist costs J2 six times what a
gram at the J3 actuator does. That is why the light wrist was worth taking (it took 1.3 N·m off J2).

## 3. Why it is heavier than PAROL6

PAROL6 is 5.5 kg, 1 kg payload, 400 mm reach. Its bill of materials lists six NEMA17 motors: three 65 N·cm
(J1-J3), two 45 N·cm (J4, J5) and one 16 N·cm pancake (J6), with metal planetary gearboxes of 10:1 to 25:1 and belts.

| Difference | Effect on this arm |
|---|---|
| A seventh joint | +0.95 kg (J3 actuator) |
| NEMA23 on J2 instead of a NEMA17 | +0.6 kg, on the turntable. Needed because J2's design torque here is ~29 N·m |
| One driver board on every motor (CAN) instead of one controller in the base | +0.4 kg spread along the arm, in exchange for four wires through the joints |
| Wrist motors | Now about PAROL6's sizes (2× 43-45 + 1× 18 N·cm). The rev. 2 wrist (2× 59 + 1× 45 N·cm) was 0.4 kg heavier |
| Longer: 520 mm to the tool tip against 400 mm | Torque scales with reach × mass, so every joint needs more |
| Printed cycloid + belt stages instead of compact metal planetaries | Bulkier housings and more bearings |

So this arm is a class up from PAROL6 in reach and joint count, and pays for it in mass. That is a consequence of the
7-DOF and reach decisions (D1, D10), not a design error.

## 4. What the mass costs, and what it doesn't

| Consequence | Size | Handled by |
|---|---|---|
| Torque on J2, J3, J4 | Design torques 32.7 / 13.4 / 12.8 N·m with the CAD masses and the 3-prong gripper, 26 Oct (32.8 on 21 Oct) (31.9 / 12.7 / 12.1 with the 0.08 TCP; 29.1 / 12.9 / 12.9 with the hand estimates) | Ratios raised to 48:1 (rev. 2). Margins J2 1.32×, J3/J4 1.44× / 1.51× with the 0.85 N·m LDO motors, J5/J6 1.18× / 1.24× (J5 the thinnest since the 3-prong gripper; on 20 Oct, before it, J2 was the thinnest at 1.35×) |
| **Speed under load** | J2 can lift the rated load at up to ~71 °/s (limit set to 55), J3/J4 at up to ~124 °/s (limit 75) | Nothing: this is the motors' power. A lighter arm is proportionally faster, a higher ratio is not |
| Ringing after fast moves | First mode ~3.6 Hz with 1 kg (21 Oct, CAD masses; 3.7 Hz with the hand estimates, 5.3 Hz empty), from the linearised MuJoCo model with placeholder gear stiffness | S-curve profiles now; dual-loop control and measured stiffness later |
| Sag under load | ~20 mm at the tool, stretched out with 1 kg and the spring (gear wind-up + half the backlash) | Repeatable, so it can be calibrated out; output encoders measure it |
| Tipping | Needs ~13.2 kg total with margin (TCP 0.11, 21 Oct); arm (9.77 kg, CAD) + board + PSU ≈ 13.5 kg | Just enough; C-clamps or a paver under the board for margin |
| Energy in a collision | 5.5 kg moving at up to ~0.5 m/s at the tool | E-stop, current limits, nobody inside the reach (already required) |
| Budget | None: mass is mostly motors already in the BOM | - |
| Base mass (3.7 kg) | Helps stability; costs nothing in torque | - |

What 55-75 °/s means in use: a 90° swing of the shoulder takes about 2 s, a typical 300 mm pick-and-place move
1.5-2.5 s. Slow next to an industrial arm, normal for a printed stepper arm, and the wrist joints (180-270 °/s) are not
the limit.

## 5. The options

`python3 tools/design_options.py`, 1 kg payload. *Margin* = torque at standstill ÷ design torque (≥ 1 passes).
*Speed* = fastest the joint turns while still holding 1.3× its static load, without the counterbalance.

| Option | Total kg | Lifted kg | J2 static | J2 margin | J4 margin | J6 margin | J2 °/s | J4 °/s | USD |
|---|---|---|---|---|---|---|---|---|---|
| **A** baseline (`arm.toml`: the CAD masses of 26 Oct, 3-prong gripper, TCP 0.11) | 9.77 | 5.53 | 16.8 | 1.32 | 1.51 | 1.24 | 69 | 117 | 0 |
| **B** rev. 2 heavy wrist (dropped 7 Oct) | 10.16 | 5.92 | 18.0 | 1.23 | 1.40 | 1.48 | 64 | 108 | +3 |
| **C** J4 and J6 motors moved back | 9.77 | 5.53 | 14.7 | 1.50 | 1.67 | 1.24 | 79 | 130 | +6 |
| **E** 6-DOF (no J3 actuator) | 8.71 | 4.47 | 16.2 | 1.37 | 1.51 | 1.24 | 72 | 117 | −50 |
| **F** shorter arm (460 mm reach) | 9.77 | 5.53 | 14.9 | 1.48 | 1.70 | 1.40 | 78 | 132 | 0 |
| **G** C + F | 9.77 | 5.53 | 13.0 | 1.70 | 1.89 | 1.40 | 89 | 147 | +6 |

(Re-run 26 Oct after the edge treatment: masses −0.02 kg, J6 1.23 → 1.24, G's J2 static 13.1 → 13.0; 21 Oct with the slip ring, pads and link 6's holder arm; 20 Oct late night with the 3-prong gripper and the TCP at 0.11 m (the weakest joint is now J5 in A, C, E,
F and G; B's is J2); before that, 20 Oct night, with the masses read from the CAD and the slicer, [20](20_cad_full_arm.md) §9, [21](21_checks_and_review.md) §4; the worst poses are now searched
between the ±90° grid points, since the CAD's lumps sit off the centreline. With the hand estimates, A was 7.98 kg,
14.5 N·m, 1.53 / 1.54 / 1.29 that evening and 14.9 N·m, 1.48 / 1.50 / 1.29 before J3 and J5 moved down.)

What each one is, and what it asks of you:

- **A, the baseline.** J5 and J6 use the LDO 40 mm motor (LDO-42STH40-1684AC, 0.43-0.45 N·m, 0.28 kg) at 22.2:1; J7
  uses the LDO 20 mm pancake (LDO-42STH20-1004ASH, 18 N·cm, 0.13 kg) at 15:1, from a 3:1 belt and a ~5:1 planetary;
  J1/J3/J4 use the LDO 60 mm (0.85 N·m). J5/J6 hold 1.7× the static load with a 1.5 kg payload. Two side effects found
  in the 6 Oct check: the wrist motor is now the torque limit that protects the printed wrist gears
  (~23 MPa at the output-stage sun at full motor current, against ~31 MPa with the 48 mm motor), and J7's 3:1 belt
  stage takes back some of the length the pancake saves.
- **B, the rev. 2 wrist,** kept for comparison: 48 mm motors on J5/J6 at 20:1, the 39 mm motor on J7 at 10:1. More wrist
  margin (1.56), 0.37 kg more at the far end of the arm. If the bench shows the light wrist is too weak, the 48 mm
  motors fit the same mounts and drivers (about USD 20 and three weeks).
- **C, motors moved back.** J4's motor sits 80 mm closer to the shoulder with a longer belt; J6's motor sits at the
  elbow end of the forearm and drives the wrist through a ~100 mm belt (PAROL6 does this). No mass saved, but it is
  moved to where it costs less. Purely a CAD decision: same parts, two longer belts.
- **E, 6-DOF.** Dropping J3 saves 0.8 kg and ~USD 50, but that mass is 60 mm from the shoulder axis, so it barely
  changes the torque (15.0 → 14.5 N·m). It would give up joint locking with full pose, the elbow swivel and full-arm
  mirroring. Not worth it for mass.
- **F, shorter arm.** Scaling the links to PAROL6's reach (168 / 159 / 62 mm) helps more than C, because torque scales
  with length. It costs workspace: 460 mm instead of 520 mm to the tool tip. A CAD decision (the links are printed),
  reversible until the links are drawn.
- **G** is the fallback if the real masses come in above the estimates.

## 6. What does not help

- **A higher gear ratio** raises torque at standstill but not speed under load: above its corner speed a stepper
  delivers constant power, and the load needs torque × speed ([14](14_math_guide.md) §11).
- **More supply voltage** would raise that power in proportion, but the MKS SERVO42D/57D are specified for 12-24 V.
- **Bigger motors on J3/J4** (a NEMA23 at the elbow) add 0.7 kg exactly where it hurts J2.
- **BLDC actuators** would change everything, including the budget and the schedule (D2, D8). That is the v2 path.
- **The counterbalance spring** already more than halves J2's worst-case static load (14.9 → 6.5 N·m), which in
  practice gives J2 its speed back. The budget ignores it on purpose so a snapped cable can't drop the arm, but
  day-to-day motion benefits from it.

## 7. Decision (7 Oct 2026)

1. **Kept:** the 7-DOF layout, the 520 mm reach and the 1 kg rating.
2. **Taken:** the light wrist (A above, D26), bought as 2× LDO-42STH40-1684AC (40 mm) + 1× LDO-42STH20-1004ASH (pancake).
3. **Left to CAD:** C (motor placement) and F (link length), when the forearm and wrist are drawn. They need no
   different parts.
4. **Watch item: J5/J6.** They pass at 1.29× (with the conservative 0.43 N·m) since the planetary went from 20:1 to
   22.2:1 on 8 Oct. When the first wrist actuator is built, weigh it and measure its torque. J3/J4 stopped being
   the watch item when the 0.85 N·m LDO motors replaced the 0.65 N·m ones (1.14× → 1.50×).
5. **Watch item: the wrist planetary teeth.** Module 1.0 is forced by the 65 mm wrist; give the output stage a
   15-18 mm face or 5 planets ([03](03_actuators_gearboxes.md) §4) and measure it on the bench.
6. **Re-run both tools** whenever a mass or a motor changes: `tools/torque_budget.py`, `tools/design_options.py`.

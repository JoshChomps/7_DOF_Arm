# 21: Checks on the full CAD, and how to review them cheaply

The checks run on 20 Oct 2026 (night) after the full arm was built ([20](20_cad_full_arm.md)). Each section gives what
was checked, the command that re-runs it, what it found, and what is still open. **Nothing here was printed or
measured. These are model checks.**

## 0. For a reviewing agent: how to work here at low cost

1. **Read only this file** and the one source file a section names. Don't read docs/14 or docs/17 in full: they're
   long.
2. **Re-run a check** with its command from the repo root on Windows, then compare the output with "Expect".
   Python is `software/.venv/Scripts/python.exe`. The Fusion commands need Autodesk Fusion running with the MCP
   add-in on port 58846, plus the hub file registry in `tools/fusion_files.json`.
3. **Report** in this form, one line per check: `§n: same | differs: <what> | can't run: <why>`. Don't edit CAD,
   `arm.toml` or the docs. Hand the list back to the main agent (the session that wrote this), which fixes
   and re-runs.
4. **Pipe long output** through `| tail -20`. Every command below prints at most ~20 lines.
5. **Rules:**
   - Don't commit or push.
   - Don't save or close Josh's own Fusion documents. The tools open the `DA_` files hidden or as a new tab and close
     them unsaved, unless `--save` is given.

| § | Check | Needs Fusion | Time |
|---|---|---|---|
| 1 | Spring packaging | no | read only |
| 2 | Fastener and pin count vs the BOM | to refresh the hole dump | 1 min |
| 3 | Slicing (print time, filament, fit) | for the STLs | ~40 min |
| 4 | Masses from the CAD | to refresh the body dump | 1 min |
| 5 | Self-collision capsules vs the CAD | yes | ~10 min |
| 6 | Link strength (beam screening) | no | 1 min |
| 7 | Simulator numbers | no | 1 min |
| 8 | Joint sweeps, hole probe, kinematics | yes | 5 min |
| 9 | Tests | no | ~2 min |
| 10 | Not modelled yet | read only | - |
| 11 | Joint sensing (hall + encoders) | module checks | 5 min |
| 12 | 3-prong gripper | yes (`gripcheck`) | 2 min |
| 13 | Cable routing (bus, loops, ties) | read only | - |
| 14 | Electronics spacing (keep-out boxes, `DA_B_Spaces`) | the §8 sweep | read |

## 1. The counterbalance spring: deferred, and the arm doesn't need it

**Finding.** The CAD has no room for the spring.
- The cable between the lug and the pulley runs 28-146 mm over J2's ±120°. A zero-length spring has to stretch that
  far: 118 mm of travel, 219 N at the top (`arm.toml` 1499 N/m).
- The tower has ~179 mm between the pulley (z 279) and the anchor (z 100).
- An extension spring that stretches ~120 mm is itself at least ~100 mm long, plus hooks. That's more than 179 mm.
- **The printed lug and tower also fail as drawn** at that load: 50-120 MPa in the 4.5 mm lug riser, ~35 MPa in the
  tower. So the spring, lug and tower have to be designed together.

**Why that is fine.** `tools/torque_budget.py` sizes every joint **without** the spring, so a snapped cable can't
drop the arm. With the CAD masses:
- J2 still has 1.35× margin at standstill and 1.29× at full speed (§4).
- J2's motor holds the worst static load at ~30% of its holding torque (16.4 N·m / 48 / 0.63 = 0.54 of 1.9 N·m).

What the arm loses without it:
- J2 runs warmer, on more holding current;
- J2 is slower under load;
- the contact detector reads larger loads on J2.

**Decision (D41):**
- Build without the spring.
- Leave the tower and pulley off. They exist only for the spring. The lug on link 2 can stay; it does nothing alone.
- **Set `[counterbalance] stiffness = 0` in `arm.toml` the day the arm runs without it.** The software otherwise
  expects the help. It stays at 1499 for now so the docs and tests describe the planned arm.
- Options for later:
  - two springs in parallel along the turntable;
  - a 2:1 cable purchase (half the travel, twice the force);
  - a spring inside the base.

  Each needs a stronger lug, ideally an aluminium bracket.

## 2. Fasteners and pins: counted from the CAD's holes

```
software/.venv/Scripts/python.exe tools/fusion_run.py sweep inventory    # Fusion: every hole -> tools/cad_holes.json
python tools/cad_bom.py                                                  # no Fusion: counts
```

Each axis that holes share is one item. An insert hole means an insert plus its screw; a clearance hole alone means a
screw into a motor or a nut; a pin hole means a steel pin. **Expect** (20 Oct late night, with the D42 sensing parts
and the 3-prong gripper; the first count is in brackets):

| Item | Count | Note |
|---|---|---|
| M3 insert + screw | 74 (68) | +6: the gripper on the tool plate's inserts |
| M3 screw, no insert | 26 (19) | +6 encoder-cap posts (nuts on the motor plate); +1 is the gripper's lead screw in the carrier, not a screw. Plus 24 M3 × 6 motor screws |
| M4 insert + screw | 46 | |
| M4 screw, no insert | 14 | with nuts |
| M5 insert + screw | 4 | J7 flange |
| pin Ø5 | 100 | 76 ring pins Ø5 × 25 (J1 26, J3/J4 50); 16 output pins Ø5 × 25; ~8 dowels |
| pin Ø6 | 48 (42) | 25 ring pins Ø6 × 30 (J2); 16 output pins Ø6 × 25 (J2, J1); +6 are the three encoder rods, counted at both ends |
| pin Ø3 | 27 (15) | planets: 6 × 16 and 6 × 24 (J5/J6), 3 × 12 (J7). The gripper's 6 pins (Ø3 × 12) show up as 12 items, because the patterned holes split along each pin |
| pin Ø4 | 6 | cover dowels, 4 × 8 |
| magnet / ball seat | 13 (6) | +7 hall-homing magnets (J1-J7) |
| M3 insert, screw from a bought part | 1 | the slip ring's grub screw in link 6 |

The gripper's tool board takes 4 M2 self-tapping screws, which the counter doesn't classify. Run the inventory only
when nothing else is driving `DA_ARM`: a dump taken during the §5 sweep read a posed arm.

`docs/bom.csv` now carries these counts. **The BOM had ~60 Ø5 pins; the CAD needs ~100.** The Ø6 × 25 and
Ø4 × 8 pins aren't bought yet (the bought 6 × 30 and 4 × 20 are too long). Pin lengths come from the design, not
from hole depth, because output pins pass through larger holes in the discs.

## 3. Slicing: print time, filament, printer fit

```
software/.venv/Scripts/python.exe tools/fusion_run.py sweep stl     # Fusion: printed parts -> stl/da/*.stl (re-run if it times out)
python tools/slice_parts.py                                         # Bambu Studio 2.04 CLI -> stl/da/slice.json
```

Settings as docs/19 §9: 6 walls, 40% gyroid, 6 top/bottom layers, tree supports at 40°, PETG HF, textured PEI.
The stock Bambu profiles inherit from parent files and crash the CLI, so the script flattens them into
`stl/da/profiles/` first.

**Expect:**
- **52 part files sliced; the whole arm is ~2.9 kg of PETG, ~126 h** (the J3/J4 and J5/J6 module parts print
  twice; one of each file is ~2.3 kg, ~98 h; 26 Oct, with the edge treatment), with the 3-prong gripper and encoder caps. The three fingers are
  one file.
- **Too big for the A1 mini (180 mm):**
  - Left stand: 164 × 217 mm, 152 g, 4.3 h.
  - Right tower: 218 mm tall, 81 g. Not needed without the spring (§1).
  - These were sliced on the H2D profile as a stand-in for the CORE One L.
- **J2 housing** (60 × 132 × 144 mm, 294 g, 10.5 h) failed on the A1 mini in both orientations tried and was sliced
  on the H2D profile. It should fit the A1 mini when laid flat; check by hand in Bambu Studio.
- **Link 5** needed its modelled orientation; auto-orient pushed it outside the A1 mini's volume.
- Every part has some supports.
- Biggest parts: J3/J4 housings 139 g, 5.7 h each; J5/J6 housings 139 g, 5.8 h; link 2 127 g, 5.1 h.
- **Grams per CAD cm³:** housings and links 0.79-0.99, median ~0.9. Small and solid parts run 1.0-1.5.

## 4. Masses from the CAD

```
software/.venv/Scripts/python.exe tools/fusion_run.py sweep bodies    # Fusion: tools/cad_bodies.json
python tools/cad_masses.py                                            # prints the arm.toml [[masses]] block
```

How the masses are built:
- Printed parts use the slicer's grams (§3).
- Unsliced parts (J1) use 0.90 g/cm³.
- Motors and drivers are catalogue values on the CAD's boxes.
- Bearings, pins and fasteners are point masses on each module's axis.
- The gripper is drawn (§12).

**Expect:** 9.77 kg in all, 5.53 kg past the shoulder (9.79 and 5.54 before the 26 Oct fillet pass); the 3-prong gripper drawn at 0.119 kg (21 Oct). The hand estimates said 7.98 and 4.39. The
`arm.toml` block is the script's output pasted in; the totals are in its last line. The budget, spring and docs/14
numbers follow from it (§9).

## 5. Self-collision capsules against the CAD

```
software/.venv/Scripts/python.exe tools/collision_vs_cad.py 48 1     # Fusion: random + folded poses -> tools/collision_vs_cad.json
software/.venv/Scripts/python.exe tools/collision_vs_cad.py 48 2     # a second random set
# Fusion's answers are cached in tools/collision_vs_cad_cache.json: once cached, a re-run only re-scores the capsules.
# Delete the cache after the CAD changes.
```

`software/deskarm/collision.py` keeps its centreline capsules (base, shoulder, upper arm, elbow, forearm, wrist, tool)
and now adds CAD-derived capsules (`ATTACHED`):
- the J2 module and motor;
- the right tower;
- the J3 motor, J4 module and motor, J6 module, J7 motor;
- four spokes for the turntable flange.

Each rides on its link. Neighbours that the joint limits already cover are skipped. Motor radii sit between the box's
half-width and half-diagonal, and the planner's 5 mm margin covers the corners. Truth for the comparison is Fusion's
interference check on `DA_ARM` (overlap over 1 mm³). Every capsule is also checked against the desk plane, the
board top at z 0, like `ik.workspace_violations`.

**Expect:** 0 missed in both sets.

| Set | CAD hits | Caught | Missed | False alarms |
|---|---|---|---|---|
| seed 1 (56 poses, 21 Oct: final CAD with the slip ring and pads; same as 20 Oct late night) | 14 | 14 | 0 | 3 |
| seed 2 (56 poses, same) | 15 | 15 | 0 | 4 |

Earlier runs:
- The 2-finger gripper, caps and encoders: 13/13 and 13/13 caught, with 3 and 4 false alarms.
- Before the board, gripper and caps were drawn: 7/7 and 11/11 caught, with 9 and 6 false alarms.

The new hits are poses that reach the electronics boxes (8 in seed 2). Each one is caught by `KEEP_OUT` or the desk check.

How it got there:
- The first version missed link 3's U-collar swinging into the J1 base, and the J4/J6 motors dipping into the board.
  Capsules for link 3's collar and arm-plate edges, and the desk check, fixed both.
- **False alarms are conservative on purpose:**
  - The desk is modelled as the plane z = 0. The real board is 500 mm square, and the desk under it sits 88 mm
    lower (the feet). So at J2 ±120° the planner refuses poses that clear the CAD.
  - The tool capsule (r 50 mm, to the TCP) covers the 3-prong gripper fully open, so it is fat when the gripper is
    closed. Against the desk the tool is a flat-ended cylinder that stops at the fingertips (`tool_tip`, 13 mm past
    the TCP). Otherwise the capsule's round end would keep a tool-down pick 50 mm off the board.
- 25% of random joint space is flagged (19% before the tool capsule grew to r 50; the test bound is 30%).

Not checked: poses near the joint limits of two adjacent joints at once. Those neighbour pairs are skipped by design.

## 6. Link strength: beam screening

```
python tools/link_strength.py
```

For each printed link, the gravity moment of everything it carries (the `arm.toml` masses plus 1 kg) about its root
is taken, worst over 3000 random poses, times 1.5. It is applied as bending at the weakest section. The allowable
is 15 MPa for printed PETG across layers (about half its in-layer strength). This is beam theory, with no stress
concentration and no FEA.

**Expect:** every link at 15 MPa or less.

| Member | Stress (MPa) |
|---|---|
| link 2 side plate | 11.8 |
| link 3 arm plate | 11.8 |
| link 3 U-collar | 10.6 |
| forearm side plate | 6.7 |
| link 5 arm | 8.6 |
| link 5 collar | 9.4 |
| link 6 side plate | 3.9 |

Two members were changed because of this check:
- **Link 2 side plate** widened ±50 → ±60 mm (was 15.5 MPa).
- **Link 3 arm plate** thickened 8 → 12 mm outside the J3 flange's bolt circle (was 27.7 MPa). Inside the bolt
  circle the plate stays 8 mm, so the elbow still clears ±120°.

Not screened:
- the counterbalance lug and tower (§1: fail as drawn);
- screw pull-out from inserts;
- the gearbox internals;
- stiffness and deflection.

## 7. Simulator numbers (MuJoCo)

```
software/.venv/Scripts/python.exe tools/sim_modes.py
```

The test runs with motors only and 1 kg. It holds the pose and reads the steady link error, then steps J2 by 0.3°
and takes the ringing from the log decrement. **Expect** (stretched, TCP 0.11): J2 error ~0.86°, J4 ~1.51°; rings at
~3.64 Hz, ζ ~0.060 (3.87 Hz with the 0.08 TCP).

docs/14 quoted 0.858°, 1.522°, 3.82 Hz and 0.054 from the hand-estimated masses. The ceiling 2ζω ≈ 2.9/s, so the
dual loop's k_i = 1 keeps its margin. The home-pose number (7.0 Hz) doesn't match docs/14's 5.1 Hz and may be a
different mode; don't use it until a reviewer checks the method.

## 8. Joint sweeps, hole probe, kinematics (from docs/20)

```
software/.venv/Scripts/python.exe tools/fusion_run.py sweep '[{"J2":120},{"J2":-120},{"J4":120},{"J4":-120},{"J6":120},{"J6":-120}]'
software/.venv/Scripts/python.exe tools/fusion_run.py sweep holes
software/.venv/Scripts/python.exe tools/fusion_run.py sweep tool '[{"J2":30,"J4":-60,"J6":45,"J1":20,"J3":15,"J5":25}]'
```

**Expect:**
- **Sweeps:** all `clear` except J2 −120°: link 5 reaches down into the electronics space (`DA_B_Spaces`, §14). That
  is expected; `KEEP_OUT` keeps the planner off it. (J1 ±170, J3 ±170, J5 ±180 are clear too.)
- **Hole probe:** 131 axes (21 Oct, with the slip ring's grub screw). The only 5 without a partner are:
  - the spring anchor and the cable clevis (no spring, §1);
  - the 3 ball seats (the balls aren't drawn). They are now 5.24 mm deep: a 6 mm ball touches the 4 mm groove's
    edges 2.24 mm above its centre, so the faces meet as the balls seat. The old 3 mm seats would have held the
    tool plate 2.2 mm off the receiver.
- **Tool point:** `[-13.68, -28.0, 608.03]` (CAD mm), which matches the FK.

## 9. Tests

```
cd software && .venv/Scripts/python.exe -m pytest -q
CXX="zig c++" sh firmware/base_controller/test/host/run.sh
```

**Expect:**
- **pytest:** 189 passed, 3 skipped (~140 s; ~30 s of it is the budget's pose search).
- **Firmware:** slcan and mt6701 host tests pass; `main.cpp` and `joint_encoder` compile.

## 10. Not modelled yet (design work, not review)

- **Board C** is a 44 × 34 × 6 mm space (21 Oct; it was 25 × 25, too small for the schematic's ESP32-C3-WROOM-02,
  buck, USB-C and two CAN connectors). The MT6701 sits on the axis, 1 mm from the magnet. The cap's M2 standoffs are
  at (±19, ±13). Lay the PCB out to that, or tell the CAD the real size.
- **Drawn on 21 Oct (were here):**
  - the J7 slip ring's seat (§11);
  - the gripper's TPU pads (§12).
- **Fasteners as bodies,** screw lengths and tool access at assembly.
- **The harness itself:** only its tie points are drawn (§13).
- **Counterbalance:** §1.

## 11. Joint sensing (D42, supersedes parts of D33)

What is drawn:

**Homing on every joint, J1-J7.** An A3144 hall switch sits in a 5 × 4 pocket in the fixed side, with a wire
groove out to the edge. A 6 × 3 magnet sits in a pocket in the output, across the existing 1 mm gap.

| Joint | Sensor in | Magnet in | Radius, angle |
|---|---|---|---|
| J1 | base top (groove into the harness trough) | turntable flange | R 58, 90° |
| J2 | housing top | flange | R 52.5, 36° |
| J3, J4 | housing top, through the 2 mm retainer | flange | R 37, 36° |
| J5, J6 | housing top | flange | R 28, 30° |
| J7 | housing top | flange (now Ø60, was Ø50) | R 26, 90° |

Each hall switch and its joint's driver are on the same, fixed side of the joint, so hall wires never cross a joint.

**J7 slip ring (21 Oct).** The 8-wire, 12.5 mm capsule ring sits on J7's axis under the module:
- **Stator:** its body is held in an arm on link 6 (`DA_L_Link6`, under the 80T pulley, 3 mm clear of it). It
  pushes up into the arm's bore against a lip and is held by an M3 grub screw in an insert. Its stator wires leave
  from below to the J7 driver side.
- **Rotor:** its nose is pressed into `DA_P7_Sliptube`. The tube runs up the hollow sun and carrier (1.5 mm clear of
  the sun, which turns at a different speed) and is glued into the output flange's bore under the receiver. So the
  rotor turns with the tool. Its 8 wires run up inside the tube to the pogo pins.
- **Envelope:** `DA_P_SlipRing` (body Ø12.5 × 22 mm and a Ø7 nose; check the bought ring's length).

**Output encoders on J2, J3, J4.** Each one is:
- a Ø6 steel rod pressed 5 mm into the link on the output (link 2, link 3, the forearm);
- running back through the flange (Ø26), the carrier and the hollow input shaft (Ø10.5, already there);
- to an MR126 in a new printed encoder cap under the input shaft end.

The cap hangs on two M3 posts screwed through the aluminium motor plate (two new holes, nuts on top). The posts pass
the 40T pulley on the side away from the belt, 3.5 mm clear. A 6 × 2.5 diametric magnet on the rod end faces the
MT6701 on board C, 1 mm away. Rods are 134 mm (J2) and 116 mm (J3, J4) long. The rod, MR126, magnet and board are
one bought envelope per module (`DA_P_Encoder_C2/_C34`).

**Changes from D33:**
- J3 uses the same rod instead of the through-tube and two 36T pulleys. The harness goes round J3 as a service
  loop, §13.
- J5-J7 get no output encoder: motor encoder, saved angle and the hall switch. A 1:1 GT2 pickup can be added later
  if the wrist's 0.8° backlash matters.

**Check:**

```
software/.venv/Scripts/python.exe tools/fusion_run.py check C34 0 mates     # and C34 90, C2 0 mates, C2 90, P56 0 mates, P7 0 mates
```

**Expect:**
- All `clear`, with the encoder envelopes in the interference set.
- P56: 28 `ok`. The old false flag `planet2_0 / carrier1: 0.037 (want 1.0)` is fixed (21 Oct). Carrier 1 and sun 2
  are one body, so the measured minimum was sun 2's mesh with the planet. The 1 mm axial gap is now checked from the
  stage heights.
- P7: 32 `ok` at 0° (with the slip ring parts). At other motor angles a planet's flank can come within ~0.01 mm of
  the ring gear's: that's the mesh, not an overlap, and the interference check stays `clear`.

## 12. Gripper: the 3-prong "Dummy" gripper (docs/06 §2, option A)

Superseded on 20 Oct (night): the first drawing was a 2-finger parallel gripper. docs/06 §2 asks for three fingers,
so it was redrawn. Builders: `gripper_*` in `tools/fusion_arm_links.py`.

**Parts:**
- `DA_G_Body`: base disc Ø76 on the tool plate (all six M3 inserts), the motor sleeve, three finger lugs, and the
  tool-board wall.
- `DA_G_Fingers`: three fingers in one file.
- `DA_G_Carrier`: a hub with three forks, on the drive nut.
- `DA_P_Gripper` (bought), with the tool board drawn as a space:
  - an N20 gearmotor with encoder and an M3 lead-screw shaft;
  - a 693ZZ bearing and a printed spacer;
  - three M3 nuts (two lock the screw to the bearing, one drives the carrier);
  - six Ø3 × 12 pins (three pivots, three carrier pins);
  - the tool board's space.

**How it works:**
- **Fingers:** three at 120° (azimuth 90, 210, 330), each pivoting on a pin in its lug, at R 29 and 55 mm above the
  tool plate.
- **Drive:** each finger's inner tail has a slot, and a pin in one of the carrier's forks rides in it, 14 mm inside
  the pivot. The N20 turns the lead screw, which moves the carrier along the axis. Carrier down closes the fingers;
  the forks also keep the carrier from turning.
- **Bearing:** the 693ZZ sits in the sleeve top, locked to the screw by two nuts. The grip load pushes the carrier
  up, and the sleeve's lip takes it. So the N20's gearbox carries no thrust.

**Numbers:**
- **Aperture** (between the pads, 21 Oct): 6-80 mm, drawn at 40. The carrier travels 11.3 mm between closed and
  open.
- **Pads:** `DA_G_Pads` are 2 mm TPU pads glued on the upper 25 mm of each finger. They're flat for now; a Fin Ray
  tip (docs/06 §2) can replace them.
- **Speed:** with a 300 rpm N20 that's ~23 turns, about 4.5 s end to end.
- **Tips** are 123 mm from the J7 flange. `arm.toml` `tcp_offset_z` is now 0.11 m (it was 0.08), the middle of the
  grip zone.
- **Envelope:** fully open, the tips reach R 49.4 (with the pads), so the collision model's tool capsule is now r 50 mm (was 35).
  Separate capsules for the old gripper's body and motor were removed.
- **Mass:** 0.119 kg in `arm.toml` (body 69 g with screws, fingers 12 g, pads 3 g, carrier 3 g, bought parts and board 32 g),
  against docs/06's < 300 g target.
- **Grip force:** each finger's tip is 50 mm from its pivot and its pin 14 mm, so a finger pushing with F needs
  3.6 F on its pin. docs/06's 15-20 N total grip (~6 N a finger) puts ~64 N axial on the screw. That needs
  ~0.02 N·m at the screw (M3 × 0.5, about 30% efficient), well inside an N20's stall torque. **Limit the current**,
  or it can crush things. Not measured.
- **Tool board:** a 40 × 36 × 7 mm space on the wall at −y, between the fingers at 210° and 330°. It holds the
  ESP32-C3, the CAN transceiver, the DRV8833 and the buck converter of docs/06 §1, on four M2 standoffs. The pogo
  wires come up the tool plate's bore and through a passage in the sleeve wall; the N20's wires exit its bottom
  into the same bore.

**Check:**

```
software/.venv/Scripts/python.exe tools/fusion_run.py gripcheck      # tool plate + gripper in memory at 6, 80, 40 mm (or list apertures)
```

**Expect:** `clear` at every aperture (checked at 6, 15, 40, 60 and 80 mm).

**Not done:**
- **Fin Ray tips** (docs/06 §2): the pads are flat TPU for now.
- **The tool board's PCB:** this is the space only. It isn't one of Kairo boards A-C yet.
- **Thrust and grip force:** the 693ZZ's thrust and the grip force are computed, not measured.

## 13. Cable routing

There is one harness: the 4-wire bus, daisy-chained base → J2 driver → … → J7 driver. It carries 24 V + GND in
18 AWG and CAN in a twisted pair. Boards C sit on the bus at J2-J4. Every hall switch, encoder and driver is on its
joint's fixed side, so **every joint carries only the bus**, except J7: the gripper's six wires pass through its
hollow sun, on the slip ring once that's seated.

Each joint gets a **service loop wrapped loosely round its housing**, tied at both ends to the tie slots (`tie()` in
`tools/fusion_arm_links.py`, two 2 × 4.5 slots for a 3.6 mm zip tie). Loop length ≈ R × the joint's range + 50 mm:

| Joint | Range | Loop round | R (mm) | Loop (m) | Fixed end tie | Moving end tie |
|---|---|---|---|---|---|---|
| J1 | ±170° | the base-top trough (R 66-76) | 71 | 0.47 | base slot | left stand, low |
| J2 | ±120° | J2 housing | 70 | 0.34 | left stand, top | link 2 side plate |
| J3 | ±170° | J3 housing | 55 | 0.38 | link 2 | link 3 U-collar |
| J4 | ±120° | J4 housing | 55 | 0.28 | link 3 U-collar | forearm side plate |
| J5 | ±180° | J5 housing | 46 | 0.34 | forearm | link 5 collar |
| J6 | ±120° | J6 housing | 46 | 0.24 | link 5 collar | link 6 side plate |
| J7 | ±360° | through the hollow sun (slip ring) | - | - | link 6 bridge | gripper body |

Including the runs along the links and ~0.1 m drops to each driver, that's **~3.5 m of each bus conductor**, plus
~0.3 m of 6-core for the gripper. The minimum bend radius is not an issue: silicone wire bends at ~5 mm, and the
loops are R 46-71.

Not checked: the loops' own clearance in motion. They are loose coils, not modelled. Watch them on the first slow
sweep of each joint.

## 14. Electronics on the base board: spacing, not placement

The base electronics don't need to be modelled; they need space. The check:

- **The arm's base sweep doesn't constrain the board.** The lowest things J1 swings are the turntable flange
  (R 65, z 62-74) and the stand and tower feet (z 74-84). The J2 motor sweeps R 156 mm, but at z 85-142, above
  anything on the board.
- **The arm reaching down does.** The planner's floor is the board top plus 5 mm, so tall items on the board need a
  keep-out. `KEEP_OUT` in `software/deskarm/collision.py` reserves two areas (base frame, x forward):

  | Area | x (mm) | y (mm) | Height (mm) | For |
  |---|---|---|---|---|
  | **electronics** (back strip) | −250 to −130 | −250 to +250 | 45 | the PSU (Mean Well LRS-350-24, 215 × 115 × 30, along the back edge), boards A (80 × 60) and B (70 × 30), the Pi 4 (86 × 57), the CANable |
  | **E-stop** (front corner) | +180 to +250 | +180 to +250 | 80 | the 22 mm button in its box, reachable by hand |

  Every capsule is checked against both boxes, like the desk. In normal poses the arm stays 113 mm or more away from
  the strip.
- **Wiring:** the base's harness slot faces +x (the front), so the bus runs ~0.2 m round the base on the board from
  the back strip.
- **Arm-mounted electronics are already in the CAD:** the MKS drivers as boxes on every motor's back, and boards C on
  the encoder caps.

If an item ends up taller than 45 mm or outside the strip, change `KEEP_OUT` and re-run the tests.

**Drawn as spaces (20 Oct night).** `DA_B_Spaces` (`spaces()` in `tools/fusion_arm_links.py`) puts one box per item
on the board in `DA_ARM`, so Fusion's sweeps see them. They aren't parts: a box is the room the item needs. CAD
x, y = `arm.toml` (y, −x), so the back strip is CAD y +130 to +250.

| Box | CAD x (mm) | CAD y (mm) | Height (mm) |
|---|---|---|---|
| PSU, LRS-350-24 (215 × 115 × 30) | −245 to −30 | 133 to 248 | 30 |
| Board A, base controller (80 × 60) | −20 to 60 | 188 to 248 | 20 |
| Board B, power (70 × 30) | −20 to 50 | 150 to 180 | 15 |
| Raspberry Pi 4 (86 × 57) | 70 to 156 | 191 to 248 | 25 |
| CANable (60 × 20) | 70 to 130 | 160 to 180 | 12 |
| E-stop box (70 × 70) | 180 to 250 | −250 to −180 | 80 |

Everything fits the strip, with 90 mm spare at the +x end (for the harness and the USB/CAN cables). The tallest box
(30 mm) is under `KEEP_OUT`'s 45 mm.

**Check:** the §8 sweep. **Expect:** only J2 −120° reaches a box (link 5 into the PSU's space). The planner's
keep-out flags that pose; the §5 validation scores it.

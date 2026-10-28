# 03: Actuators & Gearboxes

This doc answers **"cycloidal or planetary?"**, covers motor choice, and explains **how to build**
each gearbox type.

---

## 1. Motor choice: steppers vs BLDC

| | Closed-loop stepper (recommended v1) | BLDC + FOC (v2 / advanced) |
|---|---|---|
| Example | NEMA17/23 + MKS SERVO42D/57D (CAN) | 5010/8108 drone/gimbal motor + moteus-c1 / ODrive S1 / SimpleFOC board |
| Torque at low speed | High (0.4-2 N·m with no gearbox) | Low (0.1-1 N·m), so it needs more ratio or a bigger motor |
| Cost per joint (motor + driver) | ~$35-60 | ~$70-200 (moteus-c1 is $69 and ODrive S1 is $149 for the driver alone) |
| Control difficulty | Easy: position commands, built-in encoder | Harder: tuning current/velocity/position loops, encoder calibration |
| Backdrivable / compliant | No (with a high-ratio gearbox) | **Yes** (with a low-ratio gearbox), which makes it safer and allows force control |
| Holds pose when unpowered | Mostly, with high-ratio gearing | No, so the arm falls when power is cut. Needs brakes or a counterbalance |
| Heat / efficiency | Draws holding current constantly, so it runs warm | Efficient |
| Who uses it | PAROL6, Thor, AR4, Arctos, Dummy | CM6, OpenArm, Mini Cheetah-style actuators |

**Recommendation:** start with **closed-loop steppers**. They're cheaper, more forgiving, and
nearly every successful hobby 6-DOF arm uses them. Design the joint housings so a v2 BLDC actuator
could bolt into the same place if you want to explore compliant control later.

**Closed-loop, not open-loop:** an MKS SERVO42D/57D-style driver adds an encoder and stall
detection to each motor and talks **CAN**, so one 4-wire bus runs the whole arm. It costs about $10-20 more per
joint than a bare TMC2209 driver and saves a lot of trouble (missed steps can make the arm crash into itself or the desk).

### 1b. Stepper torque falls with speed (and what that does to the ratios)

Holding torque is a standstill number. As the motor spins, back-EMF and winding inductance use up the 24 V, the driver
can no longer push rated current, and torque falls: flat up to a **corner speed**, then roughly as 1/speed. Above the
corner the motor delivers about **constant power**, set by the supply voltage and the winding, not by the gearbox.
`deskarm.motor` models this from four datasheet numbers (holding torque, current, phase resistance, phase inductance):

| Motor (catalogue values; the LDO ones are Spool3D's listing) | Corner speed at 24 V (torque 1% below holding) | Ideal shaft power above it | Planned with (× 0.75, × gearbox efficiency) |
|---|---|---|---|
| LDO-42STH60-2004AC: NEMA17 60 mm, 0.85 N·m, 2.0 A, 2.0 Ω, 3.3 mH (LDO drawing Rev A; the listing said 2.1 Ω, 4.1 mH), shaft Ø5 × 24 with a 15 mm flat (J1, J3, J4; bought) | ~370 rpm | ~37 W | ~17 W at the joint |
| LDO-42STH40-1684AC: NEMA17 40 mm, 0.43 N·m, 1.68 A, 1.65 Ω, 3.6 mH (J5, J6; bought) | ~480 rpm | ~22 W | ~12 W |
| LDO-42STH20-1004ASH: NEMA17 20 mm pancake, 0.18 N·m, 1.0 A, 7.2 Ω, 7.0 mH (J7; bought) | ~370 rpm | ~8 W | ~4 W |
| LKD57HS76-2804-14: NEMA23 76 mm, 1.9 N·m, 2.8 A, 1.13 Ω, 3.6 mH, 8 mm shaft (J2; bought) | ~250 rpm | ~56 W | ~26 W |

(26 Oct: re-run from `deskarm.motor` with the values above. Until then the LDO 60 mm row said ~350 rpm and ~31 W,
from the listing's 2.1 Ω and 4.1 mH; the drawing's 3.3 mH gives more power. The other corners moved by 4-11% for the
same reason or from rounding.)

Consequences:
- **A higher ratio buys torque near standstill, not speed under load.** Lifting J2's 16 N·m static load at 55 °/s takes
  16 W at the joint whatever the ratio. That is why J2 is limited to 55 °/s and J3/J4 to 75 °/s.
- The model is an **upper bound** (no iron losses, resonance or pull-out margin), so everything is planned at 75% of it
  (`[drive] torque_derate` in `arm.toml`). Compare with the pull-out curve in your motor's datasheet, and measure it on the bench.
- **Inductance matters as much as holding torque** when you pick a motor: of two motors with the same torque, the one
  with the lower inductance (usually the higher current rating) is faster. Record R and L of the motors you buy in `arm.toml`.
- More supply voltage would raise the power in proportion, but the **MKS SERVO42D/57D are specified for 12-24 V**, so
  with these drivers 24 V is the ceiling. The counterbalance spring is what gives J2 speed back in practice: it halves
  the load the motor has to carry.

The derivation and the curves for these four motors are in the [math guide](14_math_guide.md) §11.

The sim, the torque guard and the time-optimal planner all use this curve, so a move that only fails at speed is caught.

### 1c. Mass budget and how to cut it

`arm.toml` lists every lump of mass (run `python3 tools/torque_budget.py` to print it): **9.77 kg total, 5.53 kg past the
shoulder**, from the CAD and the slicer since 20 Oct (the hand estimates said ~8.0 and 4.4). Motors and drivers are ~3.7 kg of that. The full analysis, with each option run through the budget
(`python3 tools/design_options.py`), is in [15](15_mass_and_design_options.md). In order of payoff:
1. **Wrist first:** every 100 g at the wrist costs J2 ~0.4 N·m static. Option B (40 mm motors on J5/J6, a 20 mm pancake
   on J7), accepted on 7 Oct, already took 0.37 kg off; printed planetary housings with thinner walls save more.
2. **Slide the J4 motor toward the shoulder** (longer belt): 0.55 kg moved 80 mm saves J2 ~0.4 N·m.
3. **J6 motor inside the forearm** with a belt to the wrist axis (PAROL6 does this) instead of at the wrist.
4. A stronger J3/J4 pre-stage (2.5:1 → 60:1) if their margin (1.50× design with the LDO 60 mm motors) disappears.

---

## 2. Gearbox options compared (3D-printed versions)

| Property | Cycloidal | Planetary (1-stage) | Planetary (2-stage / compound) | Harmonic (strain wave) | Belt / capstan |
|---|---|---|---|---|---|
| Ratio per stage | **10-50:1** | 3-10:1 | 15-100:1 | 20-100:1 | 2-6:1 |
| Backlash (printed) | Low (many lobes in contact) | Low-medium (~0.6° reported) | Higher (~2.6° reported for compound) | **~Zero** | ~Zero (if tensioned) |
| Efficiency (printed) | Medium (60-80%) | **High (85-90%)** | Medium | Low-medium | **High** |
| Shock-load tolerance | **High** (load spread over many lobes) | Medium (few teeth in contact) | Medium | Low (flexspline fatigue) | Medium (can skip teeth) |
| Backdrivable | No at > ~20:1 | **Yes** | Partly | No | Yes |
| Axial length | **Short** | Short | Longer | Short | Needs space for pulleys |
| Hollow shaft for cables | Possible (output-pin design) | Possible | Harder | Possible | Easy |
| Print difficulty | Medium: needs an accurate disc profile and a good eccentric bearing | **Easy**: gear generators exist in every CAD tool | Medium: assembly constraints | **Hard**: flexspline material and wear | Easy |
| Vibration | Wobbles unless you use **2 discs 180° apart** | Smooth | Smooth | Smooth | Smooth |
| Wear life (printed) | Good if pins roll (bearings as rollers) | Good | OK | Poor-OK | Belt life is fine |

### Verdict for this arm

| Joint | Choice | Reason |
|---|---|---|
| J1 base | **Cycloidal 25:1** | Stiffness and resolution matter more than torque, and cycloidal is flat, which suits the base |
| J2 shoulder | **2:1 belt + cycloidal 24:1 = 48:1** + counterbalance | Highest torque and shock load, where cycloidal is strongest. Needs ~33 N·m design torque |
| J3 upper-arm roll (7th DOF) | **Same actuator as J4**: 2:1 belt + cycloidal 24:1, **hollow** | Elbow-class torque in the worst case, and one design printed twice |
| J4 elbow | **2:1 belt + cycloidal 24:1 = 48:1** | Needs ~13 N·m design torque |
| J5 forearm roll | **Same actuator as J6**: planetary 2-stage 22.2:1 | With the wrist bent, J5 holds the tool sideways: the same ~4.0 N·m as J6 |
| J6 wrist pitch | **Planetary 2-stage 22.2:1** (two identical stages: sun 14, planet 19, ring 52, 3 planets, module 1.0, 62.5 mm ring OD) *or* small cycloidal / printed harmonic (HD20-style) | Holds the whole tool. Test which gives less backlash. At 65 mm OD only module 1.0 fits: give the output stage an **18 mm face** (3 planets; see the build checklist). Planets run on 623 bearings and 3 mm pins |
| J7 tool roll | **3:1 belt + planetary ~5:1 = 15:1**, 20 mm pancake motor beside the axis | Light load, and should be short so the tool sits close to the wrist |

One 24:1 cycloid profile (25 ring pins) serves J2, J3 and J4 in two sizes: R = 50 mm with 6 mm pins for J2, R = 36 mm
with 5 mm pins for J3/J4 (`tools/cycloid.py`, commands in the [README](../README.md)). **Both use the 6804 eccentric
bearing** (20 × 32 × 7 mm; D31, 8 Oct): in the J3/J4 disc that leaves 4.4 / 4.7 mm webs with the output pins on a 24 mm
radius.

### Hollow shafts and where the motors go

The harness (24 V, GND, CAN-H, CAN-L) runs through the centre of each gearbox. A motor bolted straight onto the cycloid's
input sits exactly where the wires need to go, so:

| Joint | Motor position | Harness route |
|---|---|---|
| J1 | Coaxial, under the base, hanging through a hole in the base board (board on ~70 mm feet; D37, 20 Oct) | The base electronics are on the fixed side; the harness goes up **beside** the motor through a slot in the turntable with a ±170° service loop (or offset the motor with a belt if the loop is in the way) |
| J2 | **Beside the axis**, belt-driven | Through the hollow cycloid input, or an external loop: a pitch joint with ±120° is easy either way |
| J3, J4 | **Beside the axis**, belt-driven | Through the hollow input shaft. This is why these joints have a belt stage at all. Hollow input 12 mm, on a **6804** eccentric bearing as on J2 (2.9 mm wall beside the bore; decided 8 Oct, D31). The 6802 first planned has a 15 mm bore, which leaves 0.4 mm |
| J5, J6 | Coaxial planetary, hollow sun not practical | External loop along the forearm (J5 ±180°) and across the wrist pitch axis (J6 ±120°) |
| J7 | **Beside the axis**, belt- or gear-driven | Slip ring through the hollow output |

**Belt sizing:** size the belt for the motor's *holding* torque, not its running torque. On a 20-tooth HTD-3M pulley
(19.1 mm pitch diameter) a NEMA23 at 1.9 N·m pulls ~200 N (~300 N on a 20-tooth GT2), which a 6 mm GT2 belt cannot
carry: use **HTD-3M, 15 mm wide** on J2 and **10 mm** on J3/J4 (9 mm is not stocked; 10 and 15 mm are). The stage is
**20T on the motor, 40T printed on the cycloid input**. Belt length follows from the centre distance C:
L ≈ 2C + 90 + 91/C mm (225 mm ↔ 66.8, 228 ↔ 68.3, 252 ↔ 80.4, 270 ↔ 89.5, 282 ↔ 95.5). With the motor body beside the
housing, C must exceed housing radius + half the motor width: about 88 mm on J2 (≈120 mm housing, 57 mm motor) and 66 mm
on J3/J4 (≈89 mm housing, 42 mm motor). The motor position is free in CAD, so **design the centre distance to the belt
you bought**: 276 mm on J2 (C = 92.5 mm) and 231 mm on J3/J4 (C = 69.8 mm), with ±3 mm of slot on the motor plate to fit
and tension it. Rating check against the SDP/SI handbook's HTD 3 mm table (6 mm base width, × 2.97 for 15 mm): a 20-groove
pulley is good for ~2.4 N·m at low speed on a 15 mm belt (J2 motor: 1.9 N·m holding, 1.42 planned) and ~1.5 N·m on a
10 mm belt (J3/J4: 0.85 holding). The same handbook's working-tension table is lower (285 N per inch of width, ~168 N
for 15 mm, against J2's 149 N planned and 199 N at full holding torque), so J2 is adequate, not generous: a 24T motor
pulley with a 48T printed one cuts the pull by 17% if the bench shows skipping. The same pull loads the motor's front
bearing (J2 ~150 N, J3/J4 ~67 N at the planned motor torque): keep the pulley close to the motor face.
The eccentric cam must **clamp** on the motor shaft (split hub or two set screws on ground flats); a printed bore on a
5 mm shaft with one set screw will slip.

**Neither type is simply better**, and the printed results depend heavily on your printer
and tolerances. That's why the roadmap starts with a **test bench** ([08](08_roadmap.md), Phase 1): build
one of each with the same motor and measure backlash, torque, and efficiency. Let those measurements decide.
Record the results in [`tests/actuator_bench.md`](../tests/actuator_bench.md).

---

## 3. How a cycloidal drive works and how to build one

### Principle

```
 motor shaft ──► eccentric cam (offset e) ──► cycloidal disc wobbles inside a ring of N pins
                                               │ disc has N−1 lobes
                                               ▼
                           disc rotates backwards by 1 lobe per motor revolution
                                               │
             output pins pass through holes in the disc (hole Ø = pin Ø + 2e)
                                               ▼
                                     output flange (slow, high torque)

 Ratio = (N − 1) : 1        e.g. 26 pins, 25 lobes → 25:1
```

### Key parameters

| Symbol | Meaning | Typical printed starting value |
|---|---|---|
| `R` | Pin-circle radius | 50 mm (J2), 36 mm (J3/J4), 35-45 mm (J1) |
| `Rr` | Pin (roller) radius | 2.5-4 mm (use steel dowel pins or small bearings) |
| `N` | Number of pins | 21-31 (ratio = N − 1). This arm: 25 (24:1) on J2-J4, 26 (25:1) on J1 |
| `e` | Eccentricity | 0.8-1.6 mm (1.6 on J2's disc, 1.1 on J3/J4's). Must satisfy **e < R / N** (otherwise the profile forms cusps) and **e < Rr** (half the pin diameter) |
| Output pins | Usually 6-10 | Hole in disc = output pin Ø + 2e |

### Disc profile (parametric curve, t from 0 to 2π)

This is the standard curve from Omar Younis's SOLIDWORKS tutorial. It works in Fusion or Onshape
equation-driven curves, in a Python script that exports DXF, or you can use **cycloidgen**.

```
ψ(t) = atan( sin((1−N)·t) / ( R/(e·N) − cos((1−N)·t) ) )
x(t) =  R·cos(t) − Rr·cos(t + ψ) − e·cos(N·t)
y(t) = −R·sin(t) + Rr·sin(t + ψ) + e·sin(N·t)
```

### Build checklist (per cycloidal joint)

1. **Two discs, 180° out of phase** on a double eccentric cam, which cancels the wobble.
2. **Eccentric bearing(s)** between cam and disc (e.g. 6802/6804/6806 depending on size).
3. **Rolling, not sliding, pins:** steel dowel pins inside printed holes work. Small bearings
   (e.g. 623/MR series) or **steel sleeves on pins** as rollers are better.
4. **Output bearing that can take moments:** two thin-section deep-groove bearings
   (61810/61812 family) spaced apart, or a printed crossed-roller bearing with steel pins as rollers.
5. **Hollow center** (≥ 10-12 mm) so the 4-wire CAN/power harness can pass through.
6. Tolerances: print a **test coupon** first (pins in holes, bearing seats) and tune the clearance
   (usually +0.1-0.2 mm on holes for PETG). Heat-set inserts for every screw.
7. Grease (lithium or PTFE grease that is safe for plastics).
8. Print the disc **flat**, 100% infill or 6+ walls, in PETG (decision D16; PETG-CF only as a bench experiment). Use PLA for fit tests only.

### Common failure modes
- Disc cracking at the output holes: add material or use more output pins.
- Too much friction: pins are sliding instead of rolling, or the disc profile is offset wrong. Add ~0.1 mm profile clearance.
- Wobble or noise: only one disc is fitted, or the eccentric cam has play.

---

## 4. How a planetary gearbox works and how to build one

### Principle and ratio (ring fixed, sun input, carrier output)

```
Ratio = 1 + Z_ring / Z_sun
Geometry:  Z_ring = Z_sun + 2·Z_planet
Assembly:  (Z_ring + Z_sun) / number_of_planets  must be an integer

Example: sun 12, planet 18, ring 48, 3 planets → 1 + 48/12 = 5:1
Two stages of 5:1 → 25:1
```

### Build checklist
1. **Module ≥ 1.0 (better 1.25-1.5)** for printed teeth. Smaller teeth shear or wear quickly.
   The J5/J6 wrist (2-stage 22.2:1 inside ~65 mm) only fits at module 1.0. A Lewis bending estimate for its output-stage
   sun (12 mm face, 4 planets, load-share factor 1.5) gives ~15 MPa at design torque, about the fatigue level estimated
   for printed PETG, and ~23 MPa if the 40 mm motor pushes at full current (PETG ~40-50 MPa static). So give that stage
   an **18 mm face** (the 22.2:1 set chosen on 8 Oct has 3 planets, so each tooth carries a third more than in this
   4-planet estimate), and treat the bench result as the real number.
2. **Herringbone teeth** print well, run smoothly, and cancel axial thrust.
3. Use **bearings in every planet** (e.g. 623 or 624 on steel pins).
4. The carrier should be supported by a bearing on both sides. The output bearing takes the joint moment (same as cycloidal).
5. Gear generators: Fusion "Spur Gear" add-in, Onshape "Gear Generator" FeatureScript, FreeCAD Gear workbench.
6. **Split-ring compound planetary** (RR1-style) gives high ratio in one stage but adds backlash.
   See RR1 and the COMPAct paper.

---

## 5. Other options (for reference)

- **Belt drive (GT2/HTD3M):** cheap, near-zero backlash when tensioned. Use it as a **pre-stage**
  (1.5-3:1) in front of a cycloidal, or to move a motor closer to the base (J4 driven from near J2).
- **Printed harmonic drive (HD20, Open HD):** consider it for J6 if planetary backlash is poor.
  The flexspline wears out, so plan on reprinting it as a replaceable part.
- **Capstan drive (Dyneema cable on a drum):** zero backlash and backdrivable, popular in
  research arms. Low ratio (≤ 10:1), bulky. An interesting v2 experiment.
- **Worm gear:** self-locking, but inefficient and wears fast when printed. Avoid for joints,
  though it's fine for gripper jaws.

---

## 6. Materials

| Use | Material | Why |
|---|---|---|
| Fit tests, prototypes | PLA | Cheap, accurate, prints fast |
| Gears, discs, housings | **PETG** (default). ASA next to hot motors. PETG-CF only as an optional bench experiment (D16) | Tougher, less brittle under shock, better creep resistance than PLA |
| Harmonic flexspline | PETG / nylon | Needs to flex without cracking |
| Soft grippers, fingertips | TPU 95A (or 85A for tentacles) | Flexible and grippy |
| Highly loaded pins, shafts | **Steel** (dowel pins, shafts) | Don't print these |

Printed parts usually fail at layer boundaries first, so **orient prints so loads run along
the layers**, and use more perimeters instead of more infill.

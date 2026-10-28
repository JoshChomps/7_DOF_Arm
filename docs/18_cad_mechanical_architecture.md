# 18: CAD and mechanical architecture (19 Oct)

> **Read with [19](19_cad_review.md) (review, 19 Oct).** Several statements below don't match the models in Fusion:
> the J1 bearing stack, the ring-pin flutes, the upper-arm "box section", the counterbalance axle, the wrist yoke and
> bearing seats, the tool-changer details and the "0 clashes" reading. 19 §11 lists each one. This file is kept as
> written, as the record of the design intent.

The parts modelled in Fusion on 19 Oct (`Admin Project / DeskArm_Parts`): their dimensions, tolerances, the measures
against heat creep, bearing fits, fastener margins and the sub-assemblies. All of them were replaced on 20 Oct by the
scripted parts of [19](19_cad_review.md) §14 and [20](20_cad_full_arm.md).

---

## 1. Constraints and material

* **PETG throughout**: designed for Bambu Lab A1 Mini (180 × 180 × 180 mm) and Prusa CORE One L (300 × 300 × 330 mm).
* **Thermal Creep Rule**: PETG glass transition is Tg ≈ 75-80°C. Continuous holding current keeps closed-loop steppers at 50-65°C. 
  * *Countermeasure*: motor mounts sit on **standoff bosses with a 3 mm air gap**, which cut the heat conducted into the
    housing (claimed then as more than 60%; not calculated or measured).
  * *Countermeasure*: bearing seats don't rely on press fits alone under sustained hoop stress; bearings are held axially by
    bolted clamping rings and shoulders.
  * *Countermeasure*: flat faces next to bearing seats have **recessed seal reliefs** (e.g. the Ø56 mm undercut in the J1
    base) so the plastic doesn't rub the turning inner ring or the rubber seal.
* **FDM Wall Margin Rule**: Minimum solid wall margin around any fastener hole, heat-set insert pocket, or belt tension slot is **≥ 2.5-3.5 mm** (6-8 perimeter loops with a 0.4 mm nozzle) so tightening a screw doesn't split the layers.

---

## 2. Parts (`Admin Project / DeskArm_Parts`)

| File | Rev | Size (mm) | Features and interfaces | Min edge margin |
|---|---|---|---|---|
| `DeskArm_J1_Base_Housing` | v3 | Ø160 × 48 | 26 dowel flutes (PCD 80), 6810 seat (Ø65.05), Ø56 seal relief, Ø24 pilot clearance, 4× M6 desk holes (PCD 142) | 5.75 mm |
| `DeskArm_J1_Turntable_Spindle` | v2 | Ø130 × 37 | 6810 journal (Ø49.95 h7), Ø53.5 seal step, Ø26 conduit, 8× M4 underside counterbores at X = ± 47.0 | 4.80 mm |
| `DeskArm_J1_Cycloid_Disc` | v1 | Ø77.2 × 6.0 | 25:1 cycloid (N=26, R=40, Rr=2.6, e=1.2), 6804 bore (Ø32.05), 8× Ø8.50 output holes (PCD 52.2) | 5.90 mm |
| `DeskArm_J2_Shoulder_Left_Cheek` | v2 | 110 × 170 × 24 | Foot interface at X = -47.0, J2 6810 seat (Ø65.05) at Y=135, NEMA23 M5 slots with 13.3 mm bottom margin | 13.3 mm |
| `DeskArm_J2_Shoulder_Right_Cheek` | v2 | 110 × 235 × 24 | Dual 608 counterbalance tower (Y=235), 16 mm internal cavity, Ø8.2 axle hole, matching J2 seat & foot | 10.9 mm |
| `DeskArm_J2_Cycloid_Disc` | v1 | Ø96.8 × 6.0 | 24:1 cycloid (N=25, R=50, Rr=3.1, e=1.5), 6804 bore (Ø32.05), 6× Ø11.10 output holes (PCD 61.4) | 9.20 mm |
| `DeskArm_Upper_Arm_Link_Half` | v3 | 80 × 265 × 30 | 190 mm axis-to-axis, 42 mm internal cavity, 9 mm sidewalls, 6× M3 clamping holes, Ø50 shoulder bore, spring lug | 2.80 mm |
| `DeskArm_Forearm_Link_Half` | v2 | 55 × 220 × 27.5 | 180 mm axis-to-axis, J4 Ø30 elbow bore, J5 6806 bearing seat (Ø42.05), 6× external M3 clamping ears | 4.30 mm |
| `DeskArm_Wrist_Pitch_Yoke` | v3 | 58 × 56 × 58 | J5 Ø29.95 journal + Ø34 shoulder, Ø16 harness conduit, J6 pitch ears with dual 6802 seats (Ø24.05 × 5.5) | 9.00 mm |
| `DeskArm_J7_Tool_Flange` | v1 | Ø65 × 18 | ISO 9409-1 4× M5 (PCD 31.5), rear-entry insert pockets (10 mm solid shoulder), 3× kinematic ball seats | 6.00 mm |
| `DeskArm_Shoulder_Assembly` | v1 | 160 × 160 × 296 | Subassembly: Base, Spindle, Cheeks, Upper Arm, Dual J2 Cycloid Discs (180° out of phase) | - |
| `DeskArm_Forearm_Assembly` | v1 | 65 × 245 × 55 | Subassembly: Clamshell Forearm halves, Wrist Pitch Yoke, J7 Tool Flange | - |

---

## 3. Part geometry and interfaces

### 3.1 J1 Base Housing & Turntable Spindle
* **Base Housing Outer Dimensions**: Ø160 mm outer cylinder, 48 mm total height.
  * **Stepped internal bore**, printable without supports: starts at Ø56.0 mm (bearing relief), expands to Ø65.05 mm (6810 bearing seat, depth 7.5 mm), and opens to Ø84.0 mm for the cycloid ring cavity.
  * **26 Dowel Flutes**: Formed on PCD 80.0 mm (R = 40.0 mm), sized at Ø5.05 mm for 5.0 mm hardened steel dowel pins. Flutes are backed by the solid outer rim spanning from R = 42 mm to R = 80 mm (38 mm of solid PETG).
  * **4× NEMA17 Thermal Standoff Bosses**: Raised 3.0 mm above the base floor on PCD 43.84 mm (31.0 × 31.0 mm square pattern) with Ø3.4 mm M3 through-holes. The 3 mm air gap isolates the motor face from the plastic housing.
  * **Central Clearance Bore**: Sized to Ø24.0 mm to clear the NEMA17 Ø22.0 mm pilot boss even after 0.2-0.3 mm PETG thermal shrinkage.
  * **4× M6 Desk Mounting Holes**: Positioned at (± 50.2, ± 50.2) mm on PCD 142.0 mm. Features Ø6.5 mm through-holes with Ø11.0 mm × 6.0 mm counterbores for socket head cap screws. Outer edge margin is 5.75 mm.
* **Turntable Spindle**: Ø130 mm flange, 37 mm total height.
  * **Bearing Journal**: Ø49.95 mm (h7 fit for 6810 bearing inner ring, ID 50.0 mm).
  * **Seal Abutment Step**: Ø53.5 mm shoulder, 2.0 mm high. Sized within SKF/NSK 52.0-54.5 mm abutment limits to clear 2RS rubber contact seals.
  * **Central Harness Conduit**: Ø26.0 mm continuous through-bore for the CAN bus and 24 V power harness with ± 170° service loop.
  * **Stanchion Mounting Pattern**: Sits at X = ± 47.0 mm with 4× M4 clearance holes (Ø4.5 mm) and 2× dowel holes (Ø5.0 mm) per side at Y = ± 10, ± 20, ± 30 mm.
  * **Underside Counterbores**: 8× Ø8.0 mm × 4.2 mm counterbores on the underside allow M4 socket head screws to sit completely flush, preventing head collision with the Base Housing.

### 3.2 J2 Shoulder Stanchions & Counterbalance Tower
* **Left Cheek (Motor & Reducer Stanchion)**: 110 mm (width) × 170 mm (height) × 24 mm (thickness).
  * **Foot Interface**: 4× M4 heat-set insert pockets (Ø5.8 mm × 9.0 mm deep) and 2× dowel alignment holes (Ø5.05 mm) centered at Z = 12.0 mm.
  * **J2 Shoulder Bearing Seat**: Concentric at (0, 135.0) mm from foot base. Sized to Ø65.05 mm × 7.5 mm depth (for 6810 bearing) with Ø54.0 mm through-bore.
  * **NEMA23 Belt Tensioning Slots**: 4× slotted holes sized at 5.4 mm × 12.0 mm travel for M5 bolts. Provides ± 3.0 mm center-distance adjustment for the 15 mm HTD-3M belt. Solid margins: 13.3 mm above foot base, 30.8 mm below J2 bearing seat.
* **Right Cheek (Counterbalance Tower)**: 110 mm (width) × 235 mm (height) × 24 mm (thickness).
  * **Foot and J2 bearing seat**: the same as the left cheek, so both J2 seats share one axis.
  * **Spring Tower**: Extends upward to Y = 235.0 mm with a 16.0 mm wide internal cavity (X in [-8.0, +8.0] mm) to house dual 608-2RS pulley bearings.
  * **M8 Shoulder Axle Through-Hole**: Ø8.2 mm hole at (X = -12.0, Y = 220.0) mm supporting the fixed counterbalance pulley axle. Minimum top margin is 10.9 mm; cheek wall thickness beside the cavity is 4.0 mm solid PETG per side.

### 3.3 Cycloidal Reducer Discs (J1 & J2)
* **J2 Cycloidal Disc (24:1 Ratio)**:
  * Profile from the Younis/Lehmann parametric curve: N = 25 pins, R = 50.0 mm, Rr = 3.1 mm (3.0 mm pin + 0.10 mm printed clearance), e = 1.5 mm.
  * Minimum radius of curvature: ρ = 6.35 mm > Rr, so the profile has no undercut or cusp.
  * Major diameter: 96.8 mm; minor diameter: 90.8 mm.
  * Eccentric Bearing Bore: Ø32.05 mm (6804 bearing: 20 × 32 × 7 mm).
  * 6× Output Pin Drive Holes: D(hole) = D(pin) + 2e = 8.0 + 3.0 = **11.00 mm** (modeled at Ø11.10 mm for FDM fit) on PCD 61.40 mm (R = 30.70 mm).
  * Web Thickness: 9.20 mm bore-to-hole, 9.20 mm hole-to-tooth, 19.70 mm hole-to-hole.
  * Chamfers: 0.6 mm on all hole and perimeter edges for lead-in engagement.
* **J1 Cycloidal Disc (25:1 Ratio)**:
  * N = 26 pins, R = 40.0 mm, Rr = 2.6 mm (2.5 mm pin + 0.10 mm clearance), e = 1.2 mm.
  * Major diameter: 77.2 mm; minor diameter: 72.4 mm.
  * Eccentric Bearing Bore: Ø32.05 mm (6804 bearing).
  * 8× Output Pin Drive Holes: D(hole) = 6.0 + 2.4 = **8.40 mm** (modeled at Ø8.50 mm) on PCD 52.20 mm (R = 26.10 mm).
  * Web Thickness: 5.90 mm bore-to-hole, 5.90 mm hole-to-tooth, 11.58 mm hole-to-hole.
  * Chamfers: 0.5 mm lead-in chamfers.

### 3.4 Upper Arm & Forearm Links
* **Upper Arm Link Half**: 80 mm (width) × 265 mm (total length) × 30 mm (half depth).
  * Axis-to-axis length: 190.0 mm (J2 shoulder to J4 elbow).
  * Clamshell: two halves joined along the centre plane form a 60.0 mm box section.
  * Internal Cavity: 42.0 mm wide (X in [-21.0, +21.0] mm), leaving 9.0 mm solid sidewalls.
  * Clamping Fasteners: 6× M3 through-holes at X = ± 25.5 mm (Y = 45, 95, 145 mm). Edge margin is 2.80 mm to the cavity and 14.5 mm to the outer edge.
  * Shoulder Hub Bore: Ø50.0 mm through-bore (15.0 mm solid PETG wall).
  * Elbow Hub Bore: Ø30.0 mm through-bore (20.0 mm solid PETG wall).
  * Counterbalance Spring Anchor Lug: External tab at (X = 36.0, Y = 150.0) mm with Ø5.2 mm hole for M5 clevis pin (3.4 mm solid perimeter margin).
* **Forearm Link Half**: 55 mm (width) × 220 mm (total length) × 27.5 mm (half depth).
  * Axis-to-axis length: 180.0 mm (J4 elbow to J5 wrist roll).
  * Clamshell: two halves joined along Z = 0 form a 55.0 mm deep link.
  * Internal Cavity: 43.0 mm wide, packaging the proximal J5/J6 motors and 9 mm GT2 drive belt.
  * Elbow Hub: Ø65.0 mm OD with Ø30.0 mm bore (17.50 mm solid wall).
  * Wrist Hub: Ø60.0 mm OD with stepped Ø42.05 mm 6806 bearing seat (8.98 mm solid backing wall, 6.02 mm radial thrust shoulder).
  * 6× Clamping Ear Tabs: External semicircular tabs (R = 6.0 mm, Ø12.0 mm boss) with Ø3.4 mm M3 holes at (± 27.5, Y = 40, 90, 140) mm. Perimeter solid margin is **4.30 mm**.

### 3.5 Wrist Pitch Yoke & J7 Tool Flange
* **Wrist Pitch Yoke (J6 Pitch Bracket)**: 58 mm (base OD) × 56 mm (ear width) × 58 mm (height).
  * Proximal Stem: Ø29.95 mm (h7) journal seating into the Forearm 6806 bearing; Ø34.0 mm seal-clearing thrust shoulder (3 mm high); Ø16.0 mm central harness conduit.
  * Pitch Hinge Ears: Two 10.0 mm thick uprights (X in [-28.0, -18.0] mm and [18.0, 28.0] mm) with rounded tops (R = 15.0 mm).
  * J6 Pitch Axis: Located at Z = 43.0 mm above the stem base (21.0 mm above flange).
  * Bearing Seats: Dual 6802 bearing seats (Ø24.05 mm × 5.5 mm depth) with Ø15.5 mm through-bore. Minimum solid wall around bearing seats is **9.0 mm**.
* **J7 Tool Flange (ISO 9409-1-50-4-M5)**: Ø65.0 mm OD × 18.0 mm height.
  * Bolt Pattern: 4× M5 on PCD 31.5 mm (X = ± 15.75, Y = 0 and X = 0, Y = ± 15.75 mm).
  * Rear-Entry Heat-Set Pockets: Sized at Ø7.2 mm × 8.0 mm deep entering from the *rear* (backside), leaving a **10.0 mm solid PETG shoulder**. Tightening the tool bolts pulls the inserts *against* that shoulder instead of out of the part.
  * Kinematic Ball Seats: 3× conical seats (Ø7.0 mm at R = 23.0 mm, 120° apart) with 6.0 mm outer edge margin.
### 3.6 Internal Actuator & Tool Changer Components
* **DeskArm_J1_Output_Pin_Carrier**: Ø70.0 mm OD × 12.0 mm height.
  * Pin Pockets: 8× Ø6.05 mm pockets (depth 8.0 mm) on PCD 52.20 mm (R = 26.10 mm), receiving Ø6.0 mm steel dowel rollers with 5.9 mm outer perimeter margin and 10.1 mm inner margin.
  * Spindle Interface: 4× M4 counterbored holes on PCD 40.0 mm (R = 20.0 mm) bolting directly to the underside of the Turntable Spindle.
  * Central Conduit: Ø26.0 mm through-bore continuous with the spindle harness path.
* **DeskArm_J2_Eccentric_Shaft**: Concentric Ø20.0 mm support journals × 59.0 mm overall length.
  * Dual Eccentric Lobes: Two 180° out-of-phase lobes with eccentricity e = 1.50 mm (X = ± 1.50 mm, Ø20.0 mm × 7.5 mm width), separated by a concentric Ø24.0 mm × 1.0 mm divider ring.
  * Central Encoder Bore: Concentric Ø8.0 mm through-hole running full length for the MT6701 magnetic encoder through-rod. Minimum solid wall between encoder bore and eccentric lobe is **4.50 mm**.
  * Pulley Extension: Ø14.0 mm × 16.0 mm shaft extension with dual D-flats for the 30T HTD-3M 15 mm belt pulley.
* **DeskArm_Tool_Changer_Receiver**: Ø65.0 mm OD × 10.0 mm thickness (Arm-side ISO 9409 interface).
  * Mounting: 4× M5 counterbored holes on PCD 31.5 mm matching the J7 Tool Flange.
  * Kinematic V-Grooves: 3 pairs of Ø4.05 mm dowel pin pockets at 120° on R = 23.0 mm (dowel center-to-center spacing 5.0 mm).
  * Latch Assist & Electrical: 4× N52 magnet pockets (Ø6.1 mm × 3.0 mm depth at R = 28.0 mm); 6-pin 2.54 mm pitch spring-loaded pogo-pin block pocket (16.0 × 6.0 × 6.0 mm).
* **DeskArm_Tool_Changer_Tool_Plate**: Ø65.0 mm OD × 10.0 mm thickness (Tool-side kinematic mate).
  * Kinematic Ball Seats: 3× Ø6.0 mm spherical seats on R = 23.0 mm (120° apart) that seat without play in the receiver V-grooves.
  * Retention: 4× N52 magnet pockets mating to receiver magnets; central Ø16.0 mm wiring conduit; 3× M4 counterbored holes on PCD 46.0 mm bolting to end-effector tools (e.g. 3-prong gripper base).

---

## 4. Sub-assemblies and clearances

The assemblies were split into sub-assemblies and one top-level assembly to keep Fusion responsive. Fusion's
interference check (`design.analyzeInterference`) reported **0 clashes** in each (the review found that this did not
mean they assembled, [19](19_cad_review.md) §0):

### 4.1 `DeskArm_Shoulder_Assembly`
```
DeskArm_Shoulder_Assembly
├── DeskArm_J1_Base_Housing v3:1        [Grounded at Origin]
├── DeskArm_J1_Turntable_Spindle v2:1   [Trans: (0, 0, 53.0 mm)]
├── DeskArm_J2_Shoulder_Left_Cheek v2:1 [Trans: (-59.0, 0, 61.0 mm), R0: [0, 0, 1], R1: [1, 0, 0], R2: [0, 1, 0]]
├── DeskArm_J2_Shoulder_Right_Cheek v2:1[Trans: (59.0, 0, 61.0 mm),  R0: [0, 0, -1], R1: [-1, 0, 0], R2: [0, 1, 0]]
├── DeskArm_Upper_Arm_Link_Half v4:1    [Trans: (0, 0, 196.0 mm), Halves mated at X = 0, Left half X in [-30, 0]]
├── DeskArm_Upper_Arm_Link_Half v4:2    [Trans: (0, 0, 196.0 mm), Halves mated at X = 0, Right half X in [0, 30]]
├── DeskArm_J2_Cycloid_Disc v2:1        [Trans: (-66.0, 0, 196.0 mm), Shoulder Axis Outer Stage 1]
└── DeskArm_J2_Cycloid_Disc v2:2        [Trans: (-72.0, 0, 196.0 mm), Shoulder Axis Outer Stage 2]
```
* **Foot alignment**: Cheek foot holes at local Z = 12.0 mm with offsets ± 59.0 mm place holes at X (world) = ± 47.0000 mm → **0.0000 mm mismatch** with Turntable Spindle holes.
* **Running Clearances**: Stanchion inner gap is 70.00 mm (X in [-35.0, +35.0] mm). Upper Arm link spans X in [-30.0, +30.0] mm, leaving **5.00 mm running clearance** on each side.
* **Interference Result**: **0 clashes** across all 8 components.

### 4.2 `DeskArm_Forearm_Assembly`
```
DeskArm_Forearm_Assembly
├── DeskArm_Forearm_Link_Half v5:1     [Instance 1 (Right): Z in [0, 27.5 mm], Grounded]
├── DeskArm_Forearm_Link_Half v5:2     [Instance 2 (Left):  Z in [-27.5, 0 mm], Rotated 180° around Y]
├── DeskArm_Wrist_Pitch_Yoke v1:1      [Trans: (0, 165.0, 0 mm), Rotated +90° around X, stem in 6806 seat]
└── DeskArm_J7_Tool_Flange v1:1        [Trans: (0, 225.0, 0 mm), Rotated +90° around X, pitch axis clearance]
```
* **Clamshell Split Alignment**: Halves mate face-to-face along Z = 0 with 0.000 mm mismatch across all 6 clamping ear pairs.
* **Axial Bearing Seat & Conduit**: Distal face terminated flat at Y = 180.0 mm with semi-cylindrical 6806 bearing cradles (Ø42.05 mm × 7.5 mm depth) clamping the Yoke's Ø29.95 mm stem, and Ø30.0 mm wire conduit opening into the hollow bay.
* **Interference Result**: **0 clashes** across all 4 components.

### 4.3 `DeskArm_Full_Assembly` (Top-Level Arm)
```
DeskArm_Full_Assembly
├── DeskArm_Shoulder_Assembly:1        [Grounded at World Origin]
└── DeskArm_Forearm_Assembly:1         [Mated at J4 Elbow Axis: Trans: (0, 190.0, 196.0 mm), R0: [0, 0, 1], R1: [0, 1, 0], R2: [-1, 0, 0]]
```
* **Elbow Hub Stepped Pocket**: In `DeskArm_Forearm_Link_Half`, the 15.0 mm hub step cut is extended along +Y to Y (local) = 38.0 mm (Y (world) = 228.0 mm) on plane Z = 15.0 mm. This keeps the hub width at 30.0 mm (Z in [-15.0, +15.0] mm) until well past the Upper Arm fork ears (Y (world) = 225.0 mm), leaving **2.00 mm running clearance** on each side (X in [-17.0, -15.0] and [+15.0, +17.0]) and **3.00 mm longitudinal running clearance**.
* **Interference Result**: **0 clashes** across all 12 bodies in the full robot arm (0.000 mm³ interference volume).

---

## 5. Print checklist (PETG)

1. **Orientation**:
   * *Base Housing & Turntable Spindle*: Print flat on bed (circular face down). Layer lines are orthogonal to vertical bending loads.
   * *Cheeks*: Print flat on their outer face (Z = 0 down). Bending loads from belt tension and arm weight act in the XY print plane (along continuous filament extrusions).
   * *Upper Arm & Forearm Halves*: Print flat with clamshell split face facing UP. Zero internal supports needed.
   * *Cycloid Discs*: Print flat on bed at 0.15 mm layer height, 100% perimeter loops (no infill pattern in tooth zone).
2. **Slicer Settings**:
   * Perimeters: 6-8 wall loops (minimum 2.5 mm solid perimeter thickness).
   * Infill: 35-45% Gyroid (about equally strong in every direction).
   * Top/Bottom Solid Layers: 6 layers (1.2 mm solid skin).
   * Dimensional Compensation: Set Slicer X-Y Hole Compensation to `+0.10 mm` to offset PETG thermal shrinkage on precision bearing journals and pin bores.

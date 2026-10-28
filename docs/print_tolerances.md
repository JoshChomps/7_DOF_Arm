# Print tolerances (fill in after printing the coupons)

Generate: `python3 tools/coupons.py` → `stl/coupons/*.stl`; `python3 tools/print_pack.py coupons rings` slices them
into `stl/plates/` ([24](24_first_prints_and_j2_bench.md)). Print each coupon **with the same material, nozzle, layer
height and walls as the real parts**. Record the column that fits as wanted. Seats, journals and pins go into `FITS`
in `tools/fusion_arm.py` and `tools/fusion_j1.py`; the insert pockets are plain numbers in the builders (4.2, 5.6, 6.4).

The coupons were changed on 26 Oct to the sizes the CAD actually uses (they had tested 623/608 seats and 3/8 mm pegs,
and had no column at the CAD's +0.05 pin fit or its 6.4 M5 pocket). Each one has a column at the CAD's current guess:

| Fit | Coupon | Want | CAD now | A1 mini PLA | A1 mini PETG | CORE One L PETG | CORE One L ASA |
|---|---|---|---|---|---|---|---|
| Dowel pin, sliding (cycloid ring pins) | pins | pin turns by hand, no wobble | +0.05 | | | | |
| Dowel pin, press (output pins) | pins | press in with a vise | 0 | | | | |
| 6 mm steel ball seat (tool changer) | pins (6 mm row) | ball seats, no rattle | +0.2 (6.2 hole) | | | | |
| 623 seat (10 mm, wrist planets) | bearings | firm press by thumb | +0.05 | | | | |
| 6802 seat (24 mm, input shafts) | bearings | firm press | +0.05 | | | | |
| 6804 seat (32 mm, cycloid discs) | bearings | firm press | +0.05 | | | | |
| 6810 seat (65 mm, J1-J4 outputs) | rings_6810 | firm press | +0.05 | | | | |
| 6806 seat (42 mm, wrist outputs) | rings_6806 | firm press | +0.05 | | | | |
| 6810 journal (50 mm, output flange stems) | rings_6810 cups | slides on with a firm push, no wobble | −0.05 | | | | |
| 6806 journal (30 mm) | rings_6806 cups | | −0.05 | | | | |
| 6802 journal (15 mm, input shaft) | pegs (15 row) | | −0.05 | | | | |
| 6804 journal (20 mm, eccentric lobes) | pegs (20 row) | | −0.05 | | | | |
| M3 heat-set insert | inserts (4.2 row) | goes in straight, no bulging | 4.2 | | | | |
| M4 heat-set insert | inserts (5.6 row) | | 5.6 | | | | |
| M5 heat-set insert | inserts (6.4 row) | | 6.4 | | | | |

Not on a coupon: the 608 (counterbalance pulley, deferred with the spring), the MR126 (encoder caps) and the 693
(gripper). Start them from the 6802 result.

---

## What the CAD assumes until then

The current builders take every fit from one table, `FITS` in `tools/fusion_arm.py` and `tools/fusion_j1.py`, which
is also written into each part as Fusion user parameters: `seat_fit` +0.05 mm (bearing outer ring in a printed bore),
`journal_fit` −0.05 mm (inner ring on a printed journal), `pin_fit` +0.05 mm (dowel in a hole), `press_fit` 0 and
`clear_fit` +0.2 mm. When the coupons give real numbers, change that table and rebuild. The insert pockets and the
tool changer's 6.2 mm ball seats are separate numbers in the builders and are changed by hand.

The 19 Oct parts (replaced on 20 Oct, [19](19_cad_review.md)) used the same fits; their table is kept below:

| Feature / Hardware | Nominal Size | CAD Model Hole / Feature | Diametral Clearance | Notes |
|---|---|---|---|---|
| **6810 Bearing Seat** (J1 Base, J2 Cheeks) | OD 65.00 mm, W 7.0 mm | **Ø65.05 mm** × 7.50 mm depth | +0.05 mm | Stepped seat with Ø56 mm inner ring relief |
| **6810 Bearing Journal** (J1 Spindle) | ID 50.00 mm | **Ø49.95 mm** | -0.05 mm (h7) | Abuts against Ø53.5 mm seal-clearing step |
| **6806 Bearing Seat** (J5 Forearm Hub) | OD 42.00 mm, W 7.0 mm | **Ø42.05 mm** × 7.50 mm depth | +0.05 mm | Stepped seat with Ø30 mm through-bore |
| **6806 Bearing Journal** (Wrist Yoke Stem) | ID 30.00 mm | **Ø29.95 mm** | -0.05 mm (h7) | Abuts against Ø34.0 mm thrust shoulder |
| **6804 Eccentric Bearing Seat** (J1/J2 Cycloid) | OD 32.00 mm, W 7.0 mm | **Ø32.05 mm** × 6.00 mm depth | +0.05 mm | Through bore in cycloidal discs |
| **6802 Bearing Seat** (J6 Pitch Ears) | OD 24.00 mm, W 5.0 mm | **Ø24.05 mm** × 5.50 mm depth | +0.05 mm | Through-bore Ø15.5 mm for wiring/axle |
| **608-2RS Bearing Cavity** (Counterbalance) | OD 22.00 mm, W 7.0 mm | **16.00 mm** slot width | +2.0 mm (pair) | Houses dual 608 bearings (14 mm combined) |
| **5.0 mm Dowel Pin Flutes** (J1 Base) | OD 5.00 mm | **Ø5.05 mm** flutes | +0.05 mm | Sliding/press fit into cycloid pin ring |
| **6.0 mm Dowel Pin Flutes** (J2 Reducer) | OD 6.00 mm | **Ø6.05 mm** flutes | +0.05 mm | PCD 100.0 mm |
| **M3 Fastener Clearance** (Links, Ears, Standoffs)| M3 (3.0 mm) | **Ø3.40 mm** | +0.40 mm | Free clearance through-hole |
| **M4 Fastener Clearance** (Turntable Spindle) | M4 (4.0 mm) | **Ø4.50 mm** | +0.50 mm | Free clearance with Ø8.0 × 4.2 mm counterbore |
| **M5 Fastener Clearance / Slots** (NEMA23) | M5 (5.0 mm) | **5.40 mm** slot width | +0.40 mm | Slotted belt tensioning travel ±3.0 mm |
| **M6 Fastener Clearance** (Desk Mounting) | M6 (6.0 mm) | **Ø6.50 mm** | +0.50 mm | Free clearance with Ø11.0 × 6.0 mm counterbore |
| **M3 Heat-Set Insert Pocket** | OD 4.2 mm (Voron-style) | **Ø4.00 mm** × 5.50 mm depth | -0.20 mm | 8° taper lead-in |
| **M4 Heat-Set Insert Pocket** (Cheek Foot) | OD 6.0 mm | **Ø5.80 mm** × 9.00 mm depth | -0.20 mm | Deep pocket for high pullout load |
| **M5 Heat-Set Insert Pocket** (Tool Flange) | OD 7.4 mm | **Ø7.20 mm** × 8.00 mm depth | -0.20 mm | **Rear-entry** (10 mm solid retaining shoulder) |

Notes (slicer settings, date, filament brand):
- Slicer XY hole compensation: `+0.10 mm` recommended when printing without reaming.
- Perimeters: 6-8 walls (at least 2.5-3.5 mm of solid wall around every fastener).
- Infill: 35-45% Gyroid.


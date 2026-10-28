# Print readiness: the arm, area by area (25 Oct 2026)

Josh asked for the full assembly broken into its important parts, each area judged on **fit, sense, 3D form,
printability and manufacturability, rounded corners and sound engineering**, so printing can start. This is that
review. **Verdict: the geometry is printable, but do not print the arm yet.** Three things come first (§2).

## 1. What was checked, and how

| Question | Source | Re-run |
|---|---|---|
| Does it fit and mate? | **Re-run in Fusion on 25 Oct** (§1a): every module's mating check, the gripper, the full-arm joint sweeps and the hole probe | `tools/fusion_run.py` (20 §1, 21 §8, §11), `tools/fusion_j1_check.py 0 mates` |
| Is each mesh a sound solid? | `tools/mesh_audit.py` on all 52 `stl/da/*.stl` plus the six J1 hub parts: closed edges, volume, bounding box | `python tools/mesh_audit.py stl/da out.json` |
| Rounded corners? | Same script: length of convex edges turning more than 75° and longer than 5 mm (a filleted corner has none). Also a search of every Fusion builder in `tools/` for fillet or chamfer calls | the same |
| Thin walls | Same script: rays from 250 surface points through the material. A screening number (it can cross small features), not a slicer report | the same |
| Print time, grams, printer | `stl/da/slice.json` (Bambu CLI, 21 Oct) | `tools/slice_parts.py` |

Not checked here: strength beyond the beam screening in [21](21_checks_and_review.md) §6, screw access, how tight any
fit is in real plastic, and the J1 hub parts' grams and hours (never sliced; the six STLs were exported read-only
from Fusion into a scratch folder, nothing in the hub was changed or saved).

### 1a. Fit, re-run 25 Oct

Each check builds the module in memory with envelopes of the bought bearings, pins and motor. It runs Fusion's
interference check, then measures every designed contact (bearing on its shoulder, pin in its socket) and gap (preload,
running clearance). Nothing was saved.

| Check | Result | Same as 21 Oct? |
|---|---|---|
| J1 hub, cam at 0° | clear; 36 contacts and gaps all ok | yes |
| C2 (J2), input 0° | clear; 39 ok | yes |
| C34 (J3/J4), input 0° | clear; 44 ok | yes |
| P56 (J5/J6), motor 0° | clear; 28 ok | yes |
| P7 (J7), motor 0° | clear; 32 ok (with the slip ring) | yes |
| Gripper at 6, 40 and 80 mm | clear | yes |
| `DA_ARM` at J2, J4, J6 = ±120° | clear, except J2 −120°: link 5 enters the electronics keep-out box (1.5 cm³), which `KEEP_OUT` already forbids | yes |
| Hole probe | 131 axes; 5 without a partner: the spring anchor and cable clevis (no spring) and the 3 ball seats (balls not drawn) | yes |

So the parts fit each other **as drawn, at nominal size**. Whether they fit in real plastic depends on how these
printers size holes and shafts, which only the coupons can tell (§2).

## 2. What blocks printing, in order

1. **The tolerance coupons have never been printed** ([print_tolerances.md](print_tolerances.md) is empty). Every
   bearing seat (Ø65.05, Ø42.05, Ø32.05, Ø24.05), pin flute (Ø5.05, Ø6.05) and insert pocket (Ø4.0, Ø5.8, Ø7.2) in
   the CAD is a guess at how PETG shrinks on these printers. Print `stl/coupons/` first, on the A1 mini in PETG with
   the real settings (6 walls, 0.2 mm), and set the offsets before anything with a fit.
2. ~~**There are no fillets or chamfers anywhere in the model.**~~ *Fixed on 25-26 Oct (§5a).* As first found, none of
   the Fusion builders made one, and the audit agreed. The links, stand, tower, housings and gripper carried 1-4 m of
   sharp convex edge each (table below). The round parts (covers, flanges, J1 plate, turntable) had none because they
   are turned shapes. Plate links with square corners and square-cornered pockets concentrate stress, catch on the
   cables and are not good practice in printed PETG. The tables in §3 are from **before** the fillet pass.
3. **Nothing has been printed, so the CAD has never met plastic.** The plan already says print one C34 (J3/J4)
   module first and weigh it against `slice.json`. That is still the right second step.

## 3. Part by part

Printer: **A1M** = Bambu A1 mini (180 mm cube; the slicer script uses 170 mm to be safe); **CORE One L** = the
friend's printer (the slicer used the H2D profile as a stand-in). All parts need supports as modelled. "Thin walls"
means the 5th-percentile wall is under 2.4 mm, below the 6-wall (about 2.5 mm) design rule in
[print_tolerances.md](print_tolerances.md): look at them in the slicer preview before printing.

**J1 hub**

| Part | Size (mm) | cm³ | g / h (slicer) | Printer | Thinnest wall (5th pct / min) | Sharp edges ≥ 5 mm | Notes |
|---|---|---|---|---|---|---|---|
| J1_base | 160 × 159.8 × 61 | 533.7 | not sliced | A1M | 3.59 / 1.51 | 707 mm |  |
| J1_turntable | 130 × 130 × 37.3 | 197.5 | not sliced | A1M | 1.95 / 1.93 | 0 mm | thin walls |
| J1_plate | 99.6 × 99.6 × 6.5 | 44.5 | not sliced | A1M | 3.0 / 0.22 | 0 mm | 26 Oct denser scan: 0.06 mm near (−31, 39); find it before printing |
| J1_carrier | 70 × 70 × 13.5 | 39.8 | not sliced | A1M | 5.89 / 3.05 | 0 mm |  |
| J1_disc | 76.9 × 77.1 × 7 | 22.1 | not sliced | A1M | 7.0 / 5.91 | 0 mm |  |
| J1_cam | 25.4 × 23 × 18.5 | 5.7 | not sliced | A1M | 0.7 / 0.5 | 0 mm | two bodies in one file: print as two parts; thin walls |

**C2 (J2)**

| Part | Size (mm) | cm³ | g / h (slicer) | Printer | Thinnest wall (5th pct / min) | Sharp edges ≥ 5 mm | Notes |
|---|---|---|---|---|---|---|---|
| DA_C2_Carrier | 74 × 74 × 13 | 44.5 | 40 / 1.8 | A1M | 3.41 / 2.74 | 0 mm |  |
| DA_C2_Cover | 121 × 121 × 30 | 99.3 | 91 / 3.7 | A1M | 2.25 / 2.23 | 0 mm | thin walls |
| DA_C2_Disc | 96.2 × 96.2 × 7 | 39.3 | 35 / 1.5 | A1M | 7.0 / 7.0 | 0 mm |  |
| DA_C2_Discb | 97 × 97 × 7 | 39.3 | 35 / 1.5 | A1M | 7.0 / 7.0 | 0 mm |  |
| DA_C2_Enccap | 68 × 31.5 × 30 | 11.6 | 15 / 0.7 | A1M | 1.6 / 1.0 | 0 mm | thin walls |
| DA_C2_Flange | 117 × 117 × 39.3 | 124.9 | 99 / 3.7 | A1M | 8.0 / 1.0 | 0 mm | 0.23 was real: carrier insert pockets 0.18 mm from the 6810 journal; moved to R19 on 26 Oct ([24](24_first_prints_and_j2_bench.md) §5) |
| DA_C2_Housing | 144 × 132.5 × 60 | 396.3 | 294 / 10.3 | H2D | 7.51 / 1.55 | 316 mm | auto-orient failed; H2D, orient 1. **26 Oct: fits the A1 mini** (12.2 h with the shaft), the CLI arrange margin rejected it ([24](24_first_prints_and_j2_bench.md) §1) |
| DA_C2_Pulley40 | 42 × 42 × 24 | 17.4 | 18 / 1.3 | A1M | 0.95 / 0.54 | 0 mm | thin walls |
| DA_C2_Shaft | 26.2 × 23 × 80 | 10.8 | 14 / 1.1 | A1M | 2.21 / 0.5 | 0 mm | thin walls |
| DA_C2_Spacers | 54 × 54 × 16 | 5.1 | 6 / 0.3 | A1M | 1.94 / 1.94 | 0 mm | thin walls |

**C34 (J3, J4: print twice)**

| Part | Size (mm) | cm³ | g / h (slicer) | Printer | Thinnest wall (5th pct / min) | Sharp edges ≥ 5 mm | Notes |
|---|---|---|---|---|---|---|---|
| DA_C34_Carrier | 60 × 60 × 13 | 27.4 | 27 / 1.4 | A1M | 3.8 / 2.3 | 0 mm |  |
| DA_C34_Cover | 90 × 90 × 22 | 54.6 | 54 / 2.5 | A1M | 2.79 / 1.0 | 0 mm |  |
| DA_C34_Disc | 68.5 × 68.5 × 7 | 16.7 | 17 / 0.9 | A1M | 5.58 / 4.47 | 0 mm |  |
| DA_C34_Discb | 69 × 69 × 7 | 16.7 | 17 / 0.9 | A1M | 5.7 / 4.47 | 0 mm |  |
| DA_C34_Enccap | 68 × 31.5 × 25 | 11.2 | 14 / 0.6 | A1M | 1.6 / 1.0 | 0 mm | thin walls |
| DA_C34_Flange | 86 × 86 × 34.8 | 79.3 | 66 / 2.6 | A1M | 3.84 / 1.0 | 0 mm |  |
| DA_C34_Housing | 112 × 101 × 53.5 | 152 | 139 / 5.6 | A1M | 5.67 / 1.64 | 1108 mm |  |
| DA_C34_Pulley40 | 42 × 42 × 19 | 13.1 | 14 / 1.0 | A1M | 1.03 / 0.54 | 0 mm | thin walls |
| DA_C34_Shaft | 25.2 × 23 × 67 | 9.3 | 12 / 1.0 | A1M | 2.22 / 0.5 | 0 mm | thin walls |
| DA_C34_Spacers | 90 × 90 × 19 | 13.8 | 19 / 0.9 | A1M | 1.94 / 0.5 | 0 mm | thin walls |

**P56 (J5, J6: print twice)**

| Part | Size (mm) | cm³ | g / h (slicer) | Printer | Thinnest wall (5th pct / min) | Sharp edges ≥ 5 mm | Notes |
|---|---|---|---|---|---|---|---|
| DA_P56_Base | 72 × 72 × 5 | 18.5 | 18 / 0.8 | A1M | 2.8 / 2.0 | 0 mm |  |
| DA_P56_Carrier1 | 48 × 48 × 23 | 9.7 | 10 / 0.7 | A1M | 1.53 / 1.19 | 0 mm | thin walls |
| DA_P56_Carrier2 | 48 × 48 × 25.8 | 21.4 | 20 / 0.9 | A1M | 5.0 / 1.0 | 0 mm |  |
| DA_P56_Flange | 64 × 64 × 7.5 | 19.1 | 18 / 0.8 | A1M | 3.31 / 1.0 | 0 mm |  |
| DA_P56_Housing | 88 × 88 × 64 | 152.2 | 139 / 5.8 | A1M | 1.47 / 1.02 | 3963 mm | thin walls |
| DA_P56_Planet1 | 20.9 × 21 × 12 | 2.5 | 3 / 0.3 | A1M | 1.32 / 1.17 | 0 mm | thin walls |
| DA_P56_Planet2 | 20.9 × 21 × 18 | 3.8 | 5 / 0.5 | A1M | 1.24 / 1.17 | 0 mm | thin walls |
| DA_P56_Spacer | 33 × 33 × 6 | 0.9 | 1 / 0.2 | A1M | 1.44 / 1.44 | 0 mm | thin walls |
| DA_P56_Sun1 | 16 × 15.7 × 12 | 1.5 | 2 / 0.3 | A1M | 1.34 / 1.19 | 0 mm | thin walls |

**P7 (J7)**

| Part | Size (mm) | cm³ | g / h (slicer) | Printer | Thinnest wall (5th pct / min) | Sharp edges ≥ 5 mm | Notes |
|---|---|---|---|---|---|---|---|
| DA_P7_Base | 66 × 116.6 × 8 | 40.7 | 36 / 1.5 | A1M | 6.0 / 0.5 | 0 mm |  |
| DA_P7_Carrier | 44 × 44 × 22.8 | 15 | 16 / 0.7 | A1M | 2.49 / 1.45 | 0 mm |  |
| DA_P7_Flange | 60 × 60 × 9.5 | 20.1 | 19 / 0.9 | A1M | 3.0 / 0.57 | 0 mm | 26 Oct denser scan: ~0.01 mm sliver near the bore at z 47-48; find it before printing |
| DA_P7_Housing | 90 × 78 × 34 | 65.9 | 63 / 2.7 | A1M | 1.51 / 1.02 | 1325 mm | thin walls |
| DA_P7_Planet | 14 × 14 × 9.5 | 0.9 | 1 / 0.2 | A1M | 1.35 / 1.2 | 0 mm | thin walls |
| DA_P7_Pulley80 | 53.9 × 53.9 × 10.5 | 16.4 | 16 / 0.9 | A1M | 1.5 / 0.7 | 0 mm | thin walls |
| DA_P7_Sliptube | 14.8 × 14.8 × 63.5 | 4.1 | 6 / 0.8 | A1M | 1.2 / 1.19 | 0 mm | thin walls |
| DA_P7_Spacer | 33 × 33 × 4 | 0.6 | 1 / 0.2 | A1M | 1.44 / 1.44 | 0 mm | thin walls |
| DA_P7_Sunshaft | 26 × 26 × 28 | 4.5 | 5 / 0.6 | A1M | 1.16 / 0.5 | 0 mm | thin walls |

**Links and stand**

| Part | Size (mm) | cm³ | g / h (slicer) | Printer | Thinnest wall (5th pct / min) | Sharp edges ≥ 5 mm | Notes |
|---|---|---|---|---|---|---|---|
| DA_L_CBPulley | 8 × 28 × 28 | 2.3 | 3 / 0.3 | A1M | 0.5 / 0.5 | 0 mm | thin walls |
| DA_L_Forearm | 98 × 92 × 92 | 100.5 | 99 / 3.4 | A1M | 5.0 / 0.6 | 1079 mm |  |
| DA_L_Link2 | 129 × 120 × 161 | 145.4 | 131 / 5.2 | A1M | 3.03 / 1.04 | 1411 mm |  |
| DA_L_Link3 | 131.5 × 124 × 97.5 | 132.6 | 113 / 3.9 | A1M | 4.0 / 2.53 | 962 mm |  |
| DA_L_Link5 | 106 × 92 × 159 | 146.9 | 165 / 6.0 | A1M | 6.0 / 1.72 | 996 mm | auto-orient failed; A1M, orient 0 |
| DA_L_Link6 | 77 × 92 × 89 | 79.2 | 77 / 3.1 | A1M | 3.74 / 0.6 | 1192 mm |  |
| DA_L_StandLeft | 69 × 164 × 217 | 172.3 | 152 / 4.3 | H2D | 8.0 / 4.54 | 2295 mm |  |
| DA_L_TowerRight | 44 × 80 × 218 | 94.3 | 81 / 4.3 | H2D | 3.0 / 1.0 | 2298 mm |  |

**Tool changer**

| Part | Size (mm) | cm³ | g / h (slicer) | Printer | Thinnest wall (5th pct / min) | Sharp edges ≥ 5 mm | Notes |
|---|---|---|---|---|---|---|---|
| DA_T_Receiver | 50 × 50 × 13 | 15.5 | 17 / 0.9 | A1M | 2.42 / 1.15 | 20 mm |  |
| DA_T_ToolPlate | 50 × 50 × 8 | 13.2 | 14 / 0.7 | A1M | 2.14 / 2.0 | 60 mm | thin walls |

**Gripper**

| Part | Size (mm) | cm³ | g / h (slicer) | Printer | Thinnest wall (5th pct / min) | Sharp edges ≥ 5 mm | Notes |
|---|---|---|---|---|---|---|---|
| DA_G_Body | 76 × 76 × 61 | 53.6 | 54 / 2.2 | A1M | 1.8 / 1.6 | 1574 mm | thin walls |
| DA_G_Carrier | 38.3 × 33.2 × 7 | 2.6 | 3 / 0.3 | A1M | 2.01 / 2.01 | 448 mm | thin walls |
| DA_G_Fingers | 61.9 × 53.6 × 55 | 9.3 | 12 / 0.9 | A1M | 1.95 / 1.72 | 853 mm | thin walls; lay flat: 6 mm² on the bed as modelled |
| DA_G_Pads | 43.7 × 37.9 × 25.1 | 0.9 | 3 / 0.5 | A1M | 2.0 / 2.0 | 372 mm | thin walls |

All 58 meshes are closed solids (no open edges). The only non-manifold file is the J1 cam, which is the two cam
pieces (v3, 19 §14.7) exported together where they touch: print them as two parts.

## 4. Area by area

| Area | Fits and makes sense | Printable | Rounded corners and engineering | Verdict |
|---|---|---|---|---|
| **J1 hub** (base, turntable, plate, carrier, disc, cam) | Mates (§1a) and swept with the arm (20 §5) | Base is 160 × 160 × 61 mm and ~530 cm³, the largest print: fits the A1 mini, roughly 400-450 g and 15-20 h by the slicer's 0.75-0.85 g/cm³ for housings (not sliced). Turntable walls ~1.9 mm in places | Base has 0.7 m of sharp edge; the rest are turned shapes. The cam is the most stressed small printed part (hand estimate ~13 MPa): decide on a metal core before it carries the arm | Print after the coupons. Slice it first |
| **C2, J2 cycloid** | Mates (§1a) | J2 housing 294 g, 10.3 h, failed auto-orient on the A1 mini and was sliced on the big-printer profile; it should fit the A1 mini laid flat. Check by hand | Housing outside has 0.3 m of sharp edge. Discs, carrier, covers are round. Pulley teeth ~1 mm thick, which is normal for HTD-3M | Print after C34 has worked |
| **C34, J3/J4 cycloid** (×2) | Mates (§1a) | All A1 mini. Housing 139 g, 5.6 h | Housing has 1.1 m of sharp edge, the most of the gearbox housings after P56. Cover and spacer walls 1.9-2.8 mm | **The first module to print** (after coupons) |
| **P56, J5/J6 planetary** (×2) | Mates (§1a: 28 ok) | All A1 mini. Gears are small (planets 21 mm) and print well at 0.2 mm only if the slicer's seam and elephant foot are controlled | Housing has the most sharp edge of any part (4.0 m) and walls down to ~1.5 mm. Gear teeth at module 1.0 run ~15 MPa at design torque, about printed PETG's fatigue level | Print after C34. Expect to reprint gears |
| **P7, J7** | Mates (§1a: 32 ok) | All A1 mini; slip tube and sun shaft have ~1.2 mm walls | Housing 1.3 m of sharp edge | Print with the wrist |
| **Links** (link 2, link 3, forearm, link 5, link 6) | Joint sweeps clear to every limit (20 §5) | All A1 mini. Link 5 needs its modelled orientation (auto-orient failed). Print plates flat so the layers carry the bending in their own plane | 1.0-1.4 m of sharp edge each: square plate corners and pocket corners. Beam screening ≤ 11.8 MPa against 15 allowed (21 §6), with no stress concentration counted, which is exactly what sharp inside corners add | **Fillet before printing** |
| **Left stand** | Clear at J2 ±120° | 164 × 217 mm: CORE One L only | 2.3 m of sharp edge, the most of any link | Fillet; book the big printer |
| **Right tower, CB pulley** | Only for the spring, deferred (D41) | CORE One L | 2.3 m of sharp edge; 21 §6 says the lug and tower fail as drawn under spring load | **Do not print** |
| **Tool changer** (receiver, tool plate) | Ball seats fixed 20 Oct | A1 mini, small | Nearly all round already | Ready after the coupons (6 mm ball seats are on the coupon) |
| **Gripper** (body, carrier, fingers, pads) | Clear at 6-80 mm (21 Oct) | Fingers must be laid flat: as modelled they touch the bed over 6 mm². Pads are TPU | Body 1.6 m, fingers 0.9 m of sharp edge. Grip force not yet worked out | Fillet; print last |

## 5. Fixing the corners

The links, stand, gripper and housing outsides are drawn by scripts (`tools/fusion_arm_links.py`, `fusion_arm.py`,
`fusion_j1.py`), so the fix is a fillet step in each builder, then a rebuild and the usual checks:

- **Outside corners of plates and links:** R 3-5 mm. These only remove material, so they cannot create a clash.
- **Inside corners** (where a plate meets a collar, pocket corners): R 2-3 mm. These add material, so re-run the
  sweeps and the hole probe afterwards.
- **Bed-face edges:** a 0.5 mm × 45° chamfer, so elephant's foot doesn't spoil a bearing seat or a mating face.
- **Leave sharp:** gear teeth, pin flutes, bearing-seat shoulders and mating faces, which need their exact edges.
- Afterwards: `display leaves`, `display print --save`, `display full --save`, `display mesh`, `cad_meshes.py`,
  `sweep stl`, `slice_parts.py`, `mesh_audit.py`.

### 5a. The fillet pass (25 Oct, fixed 26 Oct)

`fusion_lib.soften(comp, bed)` runs at the end of each builder listed in `fusion_arm.SOFT`, and on the J1 base in
`fusion_j1.py`. In order:

1. **Bed rim:** a 0.6 mm × 45° chamfer on the outer loop of the face that goes on the bed. `bed` is that face's
   outward normal, taken from the audit's best orientation.
2. **Straight edges:** every straight edge at least 4 mm long is filleted.
   - Outside corners get R = min(4, w/4) and inside corners R = min(2, w/5), where w is the narrower face's width
     across the edge.
   - Faces under 6 mm wide stay sharp: gear flanks, slots, thin walls.
   - So do arcs and circles (holes, seats), and the faces of features named pocket, window, notch, slot, groove, wire
     or seat (they take a bought part).
   - Convexity comes from face 1's loop direction. It was checked on a pocketed box: 16 convex, 8 concave.
3. **Failures:** an edge set Fusion refuses is retried edge by edge. A fillet that builds but comes out unhealthy is
   deleted. Either way the edge is left sharp and counted.
4. **Guards added on 26 Oct**, after the first pass went wrong (below):
   - An edge on a cylinder of R ≤ 6 mm (a hole, a pin flute, a pin) stays sharp: those faces always mate.
   - A feature that removes or adds more volume than its size allows is deleted and its edges retried one at a time.
     The bound is 2 k r² × total edge length (cm³) plus 0.001, with k = 1 − π/4 for a fillet and 1/2 for a chamfer.
   - The protected faces are read after the bed chamfer, which re-makes the faces it touches. Recess, passage,
     sleeve and bore joined the protected names.
   - If the bed chamfer fails, its rim stays sharp instead of falling through to the fillets.
   - **Printability:** an outside edge with a face pointing down more than 45° gets a 45° chamfer of the same size
     instead of a fillet. A fillet there prints as an overhang; a 45° chamfer supports itself.

**What went wrong on 25 Oct.** The face width is measured from all of a face's vertices, so a big face that wraps
around thin ribs looks wide where it is only a few mm across. On the J1 base the R4 set took the edges at the foot of
the eight shell ribs, and the fillets swallowed every rib whole: 8 × 4.1 cm³, 534.0 → 501.4 cm³, with Fusion calling
the feature healthy. A boolean difference of the body before and after the fillet showed it (eight 4.1 cm³ lumps,
z 0-45). The same fault, smaller, took 0.4 cm³ from the left stand and 0.5 cm³ from the forearm. Separately, 50 of
the C2 and C34 housings' 74 rounded edges, and 6 on the fingers, were on small holes. A second review the same day
found three more faults:
- The protection by feature name failed wherever a protected face touched the bed chamfer (the chamfer gives those
  faces new ids). Link 6's "Pocket for the -x tab", where a module tab seats, had R 1.5 inside fillets.
- Rounded downward edges added overhang in the print orientation (the stand 235 → 471 mm² steeper than 45°, Link 3
  85 → 152, the forearm 271 → 345).
- The gripper carrier's bed chamfer failed, so its bed rim was filleted: 400 → 126 mm² on the bed and 75 → 262 mm²
  of overhang on a 7 mm part.

The 25 Oct counts were 74, 74, 83, 20, 28 and 78 filleted for C2, C34, the stand, the forearm, the fingers and the J1
base. With every guard (final, 26 Oct; saved and exported):

| Part | Filleted | Down-facing chamfered | Bed edges chamfered | Left sharp | Volume change (cm³) |
|---|---|---|---|---|---|
| DA_C2_Housing | 15 | 9 | 8 | 6 | −0.64 |
| DA_C34_Housing | 15 | 9 | 8 | 6 | −0.61 |
| DA_P56_Housing | 0 | 0 | 8 | 0 | −0.04 |
| DA_P7_Housing | 15 | 3 | 8 | 12 | −0.13 |
| DA_L_StandLeft | 65 | 7 | 49 | 56 | −1.49 |
| DA_L_TowerRight | 13 | 0 | 4 | 3 | −0.51 |
| DA_L_Link2 | 34 | 3 | 6 | 6 | −2.54 |
| DA_L_Link3 | 22 | 0 | 8 | 5 | −0.75 |
| DA_L_Forearm | 21 | 0 | 4 | 12 | −0.01 |
| DA_L_Link5 | 7 | 5 | 1 | 4 | −0.97 |
| DA_L_Link6 | 22 | 4 | 20 | 10 | −1.24 |
| DA_G_Body | 30 | 0 | 1 | 6 | −1.71 |
| DA_G_Carrier | 13 | 0 | 0 (rim kept sharp) | 38 | −0.12 |
| DA_G_Fingers | 34 | 0 | none (no bed face set) | 12 | −0.22 |
| J1 base (v9) | 40 | 0 | 12 | 32 | +0.03 (534.0, ribs kept) |

Inside fillets (R ≤ 2) add material, so the net can be near zero, as on the J1 base.

**Printability against the parts before any edge treatment**, in each part's print orientation (`SOFT`), from the
STLs. Overhang counts faces steeper than 45.5°, so the 45° chamfers, which print, are not counted:

| Part | Bed contact (mm²) before / now | Overhang (mm²) before / now |
|---|---|---|
| C2 housing | 8141 / 7899 | 849 / 750 |
| C34 housing | 2938 / 2754 | 897 / 760 |
| P56 housing | 2650 / 2501 | 2193 / 2288 |
| P7 housing | 2003 / 1866 | 635 / 651 |
| Left stand | 13046 / 12327 | 235 / 235 |
| Right tower | 3376 / 3226 | 226 / 226 |
| Link 2 | 12625 / 12339 | 1362 / 1181 |
| Link 3 | 9180 / 8849 | 85 / 152 |
| Forearm | 4752 / 4523 | 271 / 345 |
| Link 5 | 4654 / 4505 | 2595 / 2195 |
| Link 6 | 5250 / 4982 | 662 / 658 |
| Gripper body | 4316 / 4174 | 220 / 220 |
| Gripper carrier | 400 / 400 | 75 / 80 |

Bed contact drops 2-6% to the 0.6 mm chamfer, which is intended. Overhang is the same or lower on 9 of 13 parts.
Link 3 (+67 mm²) and the forearm (+74 mm²) are a few inside fillets under ceilings that already need support.

- **P56 housing:** got only the bed chamfer. Its walls are ~1.5 mm, under the 6 mm rule, so its 4 m of edge is
  still sharp. Thickening those walls (the thin-walls note in §3) comes first.
- **Gripper carrier and fingers:** small, so many edges fall under the width rule.
- **Volumes changed by 2% or less** (Link 2 145.4 → 142.8 cm³ is the largest share). The 25 Oct note said "under
  1%", which was wrong for Link 2 (1.8%), and the J1 base's 6% loss went unnoticed.
- **Feather edges:** a ray scan of the 25 Oct STLs showed 0.02-0.3 mm slivers where fillets ran into the bed
  chamfer on the stand. Below a line width, so the slicer drops them; they don't weaken anything.

**Re-checked with the final edge treatment** (26 Oct; the in-memory module checks and `gripcheck` call `soften` too):

| Check | Result |
|---|---|
| C2 | clear, 39 ok |
| C34 | clear, 44 ok |
| P56 | clear, 28 ok (25 Oct; P56 unchanged since) |
| P7 | clear, 32 ok |
| J1 stack | clear, 36 ok |
| Gripper at 6, 40, 80 mm | clear |
| DA_SA_C2, C34, P7 rebuilt and saved; SA1 on base v9; DA_ARM rebuilt | each clear |
| DA_ARM at J2, J4, J6 = ±120° | clear except J2 −120° (link 5 in the keep-out box, 1399 mm³), as before |
| `sweep holes` | 131 axes, the 5 expected unpartnered |
| `mesh_audit`, all 58 STLs | closed (the J1 cam is its two bodies) |
| Slicer, 52 parts | 2317 g, 97.9 h |

## 6. Print order

**Changed 26 Oct (late): see [24](24_first_prints_and_j2_bench.md).** The first order arrived without the motors,
and C34's Ø5 pins, 6802s and belts are still on the 11.11 list. So the first module is now C2 (J2), which is two
6802s short. The coupons were remade to the CAD's sizes, ring coupons for the 6810 and 6806 replace the "test rings"
of step 2, and `tools/print_pack.py` slices it all into Bambu Studio projects (12 plates, ~40 h, ~1.06 kg with a
bench stand and lever). The 25 Oct plan, kept for the record:

1. **Coupons** (`stl/coupons/`: pins, bearings, inserts, pegs) on the A1 mini, PETG, real settings. Fill in
   [print_tolerances.md](print_tolerances.md). About an afternoon.
2. **Test rings:** a 3 mm slice of each bearing seat the coupon doesn't cover (6810, 6806, 6802), as that file says.
3. **One C34 module:** housing, cover, flange, carrier, shaft, discs A and B, spacers, pulley, encoder cap, about 0.4 kg
   and 18 h. Assemble it with the bought bearings and pins, turn it by hand, check backlash, weigh it.
4. Then the J1 hub, C2, the second C34, P56 ×2, P7, the links, the tool changer, and the gripper last. The fillet
   pass is done (§5a). The right tower waits for the spring decision.

## 7. Bought-part sizes checked online (25 Oct)

| Part | CAD | Found | Action |
|---|---|---|---|
| J7 motor (LDO-42STH20 class NEMA 17) | 5 mm D-shaft, 42.3 mm square | Matches the listed drawing (printedsolid.com, LDO-42STH20 page) | none |
| 12.5 mm slip ring, 8 wires | 22 mm body, no flange | Common capsule rings are ~15 mm long, with a ~24 mm mounting flange (ato.com 12.5 mm series) | When ordering, pick a flangeless unit ≤ 22 mm long, or redraw the P7 sliptube to the bought part |
| MKS servo driver boards | depth assumed | Makerbase's GitHub has schematics only, no mechanical drawing | Measure on arrival |
| NEMA 23 (J2) shaft length | assumed | Varies by seller | Measure on arrival |
| N20 gear motor + encoder (gripper) | 34 mm pocket | ~12 × 10 × 26 mm plus ~8 mm encoder (robotshop, robu.in listings): fits | Confirm the lead-screw shaft variant when ordering |

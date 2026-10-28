# 12: Printing & Fabrication Plan

## Three printers, three jobs

| | **Makerspace** (free PLA) | **Bambu A1 mini** (yours) | **Friend's Prusa CORE One L** |
|---|---|---|---|
| Volume | Varies | 180 × 180 × 180 mm | **300 × 300 × 330 mm** |
| Enclosure | - | Open frame | **Enclosed, heated chamber (up to 60 °C)** |
| Materials | **PLA (free)** | PLA, PETG, TPU | + **ASA, PC**, and PETG-CF / PA-CF (it has a hardened nozzle) |
| Use for | **Fit tests, tolerance coupons, jigs, prototype gearboxes, test-bench parts** | Most final PETG parts incl. gearbox discs and gears, TPU fingertips/tentacles | **Housings near hot motors (ASA)**, one-piece big parts, the optional PETG-CF bench experiment |

Workflow per part: **PLA prototype (free) → check fit → final print in PETG (A1 mini) or ASA (friend)**. Per D16 everything
structural is PETG; carbon-fibre filament is only an optional bench comparison.

## Design rule: big parts one-piece on the CORE One L, everything else on the A1 mini
Your friend can print as much as needed (you supply filament), so:
- **Big parts** (base, J2 housing, upper-arm and forearm links) are designed **one-piece, ≤ 280 mm**, for the CORE One L.
  That means fewer joints, a stiffer arm, and less assembly.
- **Everything ≤ 170 mm** prints on your A1 mini, so you're not waiting on the friend for small parts.
- **Fallback:** each big part gets a **planned split plane** (bolted flanges + dowel-pin alignment), kept as a
  suppressed feature at the end of the Fusion timeline. If the friend becomes unavailable, un-suppress it and print
  the halves on the A1 mini.
- **Printed links (the PAROL6 / Faze4 / Thor approach):** PAROL6 is printed PETG, and Faze4 and Thor are fully
  printed. Only AR4 uses aluminum (CNC/sheet), which is why it costs ~$2,000. Links are **hollow box or oval shells**
  with the CAN harness running inside, printed as **two clamshell halves** on the A1 mini, or in one piece on the
  CORE One L.

### How stiff do printed links need to be?
Bending of the upper arm (190 mm) at full reach, carrying the forearm, wrist and a 1 kg payload (about 30 N and
6.5 N·m at the elbow), cantilever beam theory with E ≈ 2 GPa for printed PETG (3.6 GPa for PETG-CF, 69 GPa for
aluminium). The tool droop is the elbow's slope times the ~360 mm from the elbow to the TCP:

| Link cross-section | Droop at elbow | Tool droop from upper-arm bending |
|---|---|---|
| PETG box 50×50 mm, 3 mm walls | 0.45 mm | ~1.5 mm |
| **PETG box 60×60 mm, 4 mm walls** | **0.20 mm** | **~0.7 mm** |
| PETG-CF box 60×60 mm, 4 mm walls | 0.11 mm | ~0.4 mm |
| Aluminum tube 25×25×1.5 mm (for comparison) | 0.21 mm | ~0.7 mm |

**A chunky printed link (~60 mm section, 4+ perimeters) is as stiff as a small aluminum tube.**
Droop is also *repeatable* (same pose + same load = same droop), so it hurts accuracy, not repeatability,
and it can be calibrated out. Backlash matters more. Rules:
- Links ≥ 60 × 60 mm section (or an equivalent oval), 4-6 perimeters, internal ribs every ~50 mm.
- Orient prints so bending stress runs **along** the layers, never pulling layers apart.
- If the measured droop is too high, press **carbon-fiber rods** into channels along the link as cheap stiffeners.

## Do you need carbon fiber? No.
- **PAROL6** (the closest reference) is printed in plain **PETG**, and most hobby arms use PETG or PLA.
  Research arms that aren't printed (OpenArm, Franka) use aluminum and commercial actuators, not printed CF.
- CF-filled filament (PETG-CF, PA-CF) is **~50-100% stiffer** and creeps less. But it's **more brittle** (bad for
  shock loads on gear teeth), **abrasive** (needs a hardened nozzle, e.g. Prusa's ObXidian, ~USD 53, and brass
  nozzles wear out fast), and costs ~1.5-2× PETG. PA (nylon) also has to be dried before printing.
- **Plan:** print **everything in PETG**. On the test bench, optionally try **one PETG-CF cycloid disc vs one PETG
  disc** (your friend's printer has the hardened nozzle this needs) and let the measured backlash/wear decide. For
  parts touching motors, the enclosed CORE One L can print **ASA** (no special nozzle), which handles heat better than PETG.

## Materials
| Part | Material | Printer | Settings (starting point) |
|---|---|---|---|
| Cycloid discs, planetary gears, ring gears | **PETG** (optional bench test: PETG-CF) | A1 mini / friend | 0.15-0.2 mm, 5-6 walls, 50-60% gyroid, print flat |
| Motor mounts / housings touching motors | **ASA** (enclosed CORE One L) or PETG + standoffs | Friend / A1 mini | Motor cases can reach 50-70 °C. PETG softens around 70-80 °C, PLA at ~55-60 °C |
| Housings, links, brackets | PETG | A1 mini | 4 walls, 25-40% infill, loads along layers |
| Fit tests, jigs, prototypes | **PLA (free)** | Makerspace | Draft profile |
| Fingertips, tentacles | TPU 95A | A1 mini | Slow, minimal retraction |

**Never use PLA for final load-bearing or motor-adjacent parts** (creep and heat).
Nylon (PA-CF) absorbs moisture, so ask your friend to dry it before printing.

## Fusion 360 workflow

The plan (1 Oct) was top-down design by hand: a skeleton driven by User Parameters, one design per subsystem and a
jointed top-level assembly. What happened (13-26 Oct):
- The skeleton came from `tools/fusion_skeleton.py` (13 Oct). The 19 Oct hand-drawn parts didn't assemble
  ([19](19_cad_review.md)), so every part was rebuilt **by script** from the `arm.toml` numbers: J1 by
  `tools/fusion_j1.py`, everything from J2 out by `tools/fusion_arm*.py` ([20](20_cad_full_arm.md)). Module
  sub-assemblies and `DA_ARM` are jointed, swept and hole-checked in Fusion.
- Print STLs go to `stl/da/` and `stl/j1/` (`fusion_run.py sweep stl`); the slicer's grams and hours are in
  `stl/da/slice.json` ([21](21_checks_and_review.md) §3). Printer and orientation per part:
  [22](22_print_readiness.md) §3.
- Cycloidal profiles come from `tools/cycloid.py`; the URDF from `deskarm.urdf`, drawn with the CAD's meshes
  ([20](20_cad_full_arm.md) §10); masses and centres of mass from `tools/cad_masses.py`.

## Print-time estimate
The slicer gives **~2.9 kg of PETG and ~126 h** for the whole arm (52 part files, the J3/J4 and J5/J6 module parts
printed twice; [21](21_checks_and_review.md) §3), out of the 9.77 kg arm. The first plan guessed 2-2.5 kg and
80-120 h. Add PLA prototypes at the makerspace in parallel. The BOM carries 5 kg of PETG to cover supports and reprints. Sending gearbox parts to your friend takes ~20% of the load off. **Keep the A1 mini printing continuously**
from late October onward.

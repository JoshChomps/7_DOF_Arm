# 7-DOF Desk Arm

A from-scratch, mostly 3D-printed, 7-axis (redundant) desktop robot arm with a **swappable end-effector**
(3-finger gripper, octopus-style soft gripper, and eventually a human-like hand).
Built on a student budget: print everything that can be printed and buy only motors,
bearings, fasteners, and electronics.

> **Status (26 Oct 2026):** Phase 0. All seven motors, the drivers, the CANable and the big bearings are bought (8 Oct).
> The whole arm is modelled in Fusion by script, mated, swept and hole-checked, with print-ready STLs in `stl/`
> ([20](docs/20_cad_full_arm.md), [22](docs/22_print_readiness.md)), but **nothing has been printed yet**: the tolerance
> coupons come first. The **software digital twin runs in simulation** (see below). Masses come from the CAD and the
> slicer; stiffness, backlash and efficiency are still **estimates** until the test bench measures them.
> Target: feature-complete **early March 2027**, budget **≤ USD 1,000** (tools separate), PAROL6-class reach and payload,
> designed in Fusion 360, printed on a Bambu A1 mini + a friend's Prusa CORE One L.

## The plan in one screen

| Topic | Current recommendation | Why | Doc |
|---|---|---|---|
| Arm layout | **7-DOF SRS**: shoulder yaw-pitch-roll (J1-J3), elbow pitch (J4), **spherical wrist** roll-pitch-roll (J5-J7) | Human-arm layout (KUKA iiwa, Franka, OpenArm). With J3 locked it's the PAROL6/Thor 6-DOF layout, with closed-form IK | [01](docs/01_requirements.md) |
| Size | **PAROL6-class reach and payload:** ~440 mm reach, **1 kg at full reach**. Arm mass **9.77 kg** from the CAD (5.53 kg of it moves with the shoulder) | Desk-sized. **Fully printed** multi-part assemblies (PAROL6/Faze4 approach). A 7th joint, a NEMA23 shoulder and a driver on every motor make it heavier than PAROL6 | [01](docs/01_requirements.md), [15](docs/15_mass_and_design_options.md) |
| DOF | **7-DOF SRS** (upper-arm roll added), run as 6-DOF first | Lock any joint and still reach full pose; null-space control; full-arm mirroring. The extra joint reuses the elbow actuator | [01](docs/01_requirements.md) |
| Motors | **Closed-loop steppers** (1× NEMA23 + 6× NEMA17, MKS SERVO42D/57D CAN drivers) | Cheap, lots of torque, forgiving, CAN daisy chain | [03](docs/03_actuators_gearboxes.md) |
| Big joints J1-J4 | **3D-printed cycloidal**: J1 25:1; **J2, J3, J4 = 2:1 belt + 24:1 cycloid (48:1)** | High ratio, low backlash when printed, tolerates shock loads. The belt puts the motor beside the axis, which leaves the centre free for the harness | [03](docs/03_actuators_gearboxes.md) |
| Wrist J5-J7 | **Planetary**: J5 and J6 share a 2-stage 22.2:1 on 40 mm NEMA17s; J7 a 20 mm pancake at 15:1 (3:1 belt + planetary) | J5 holds the tool sideways, so it needs the same torque as J6. The light wrist (option B) was accepted on 7 Oct | [03](docs/03_actuators_gearboxes.md), [15](docs/15_mass_and_design_options.md) |
| First real build | An **actuator test bench**: cycloidal vs planetary, same motor, measured | Answers "cycloidal or planetary?" with measurements from your own printer | [08](docs/08_roadmap.md) |
| Wiring | 24 V + CAN bus, **4 wires** through hollow joints, slip ring on J7 | Much simpler routing and much less chance of tangling | [04](docs/04_electronics_control.md) |
| Custom circuits | Buy joint drivers; **design** power/E-stop, base controller, encoder and tool boards; custom FOC driver as a side project | Real PCB work that can't block April | [10](docs/10_drivers_and_custom_pcbs.md) |
| Software | **Own digital twin** (Python + Viser + MuJoCo) mirroring the real arm; joint lock; hand mirroring; ROS 2 wrapper later | Fast, and you write the controls yourself | [05](docs/05_software_kinematics.md) |
| Controls track | FOC testbed → dual-loop → TOPP-RA → visual servoing → MPC / impedance | Career direction, and each step produces a portfolio piece | [11](docs/11_controls_vision_track.md) |
| Tool changer | ISO 9409-style flange, kinematic coupling, pogo-pin CAN + 24 V, one MCU per tool | Every gripper plugs into the same connector and reports its own ID | [06](docs/06_end_effectors_tool_changer.md) |
| Budget | **~USD 1,031** core (≈ 1,134 with 10% contingency), tools separate | Tracked in `docs/bom.csv` with a tick-off status column. With contingency it is ~USD 134 over the cap | [07](docs/07_bom_budget_sourcing.md) |
| Buying | 7 order waves timed to Prime Days, 11.11, Black Friday, and before Chinese New Year | Catches the sales without stalling the build | [13](docs/13_purchasing_calendar.md) |

## Docs

1. [Requirements & target specs](docs/01_requirements.md)
2. [Prior art and research](docs/02_prior_art_research.md): open-source arms, actuators, hands, soft grippers
3. [Actuators & gearboxes](docs/03_actuators_gearboxes.md): cycloidal vs planetary vs harmonic vs belt/QDD, and how to build them
4. [Electronics & control hardware](docs/04_electronics_control.md)
5. [Software & kinematics](docs/05_software_kinematics.md)
6. [End effectors & tool changer](docs/06_end_effectors_tool_changer.md)
7. [BOM, budget & sourcing](docs/07_bom_budget_sourcing.md)
8. [Roadmap & milestones](docs/08_roadmap.md)
9. [Decision log & open questions](docs/09_decisions_open_questions.md)
10. [Choosing drivers & custom PCBs](docs/10_drivers_and_custom_pcbs.md)
11. [Controls & vision track](docs/11_controls_vision_track.md)
12. [Printing & fabrication (A1 mini, CORE One L, makerspace)](docs/12_printing_fabrication.md)
13. [Purchasing calendar: sales & order waves](docs/13_purchasing_calendar.md)
14. [Math guide](docs/14_math_guide.md): every module's math with pictures, equations and runnable examples
15. [Mass and design options](docs/15_mass_and_design_options.md): where the weight is, what it costs, what each fix buys
16. [Packaging and routing](docs/16_packaging_and_routing.md): where the electronics live, how the harness runs, how to model it in Fusion
17. [Project history and research notes](docs/17_project_history.md): what was done each day, what was learned, and which conclusions turned out wrong
18. [CAD and mechanical architecture](docs/18_cad_mechanical_architecture.md): the 19 Oct Fusion parts as first described (read with 19)
19. [CAD review, 19 Oct](docs/19_cad_review.md): the Fusion models checked: dimensions, holes and bores, assemblies, range of motion, print run, PETG and heat
20. [The full arm in Fusion, 20 Oct](docs/20_cad_full_arm.md): every part from J2 out by script, sub-assemblies and `DA_ARM`, mated, ranges swept, holes probed, originality check
21. [Checks on the full CAD, 20 Oct](docs/21_checks_and_review.md): spring, BOM from the holes, slicing, masses, capsules vs CAD, link strength, sim numbers; written for a low-token review
22. [Print readiness, 25-26 Oct](docs/22_print_readiness.md): every part judged for fit, form and printing; the edge treatment and its checks; print order
23. [Singularities and cyclic coordinate descent, 26 Oct](docs/23_singularities_and_ccd.md): which singularities the arm really has with seven joints, damping, and whether CCD beats DLS (it doesn't here)
24. [First prints and the J2 bench build, 26 Oct](docs/24_first_prints_and_j2_bench.md): what to print while the motors are on the way, the J2 hardware list, assembly order and hand tests

Also: [print tolerances](docs/print_tolerances.md) (to fill in), [actuator bench procedure and results](tests/actuator_bench.md)
(to fill in), [BOM](docs/bom.csv).

## Software (runs today, no hardware needed)

![Digital twin](docs/img/twin.png)

A browser-based digital twin of the 7-DOF arm: drag the tool, lock joints, constrain the tool (vertical, plane,
translate-only), swivel the elbow, run straight-line moves and recorded programs with trapezoid, jerk-limited S-curve
or **time-optimal (TOPP-RA)** timing, with desk and self-collision guards, on a kinematic sim or a **MuJoCo
physics sim with your motors' torque limits (which fall with speed, like a real stepper's), an itemised mass model
(motor + driver + gearbox per joint), springy/backlashy printed gearboxes (placeholder values until the bench tests) and
the J2 counterbalance spring**. The same UI drives the real arm through `CanBackend` (MKS drivers over CAN; written
12 Oct, so far run only against simulated drivers: `--backend can-sim`). Kinematics, closed-form and numerical IK,
trajectories, masses, drives and a URDF are all generated from one config file. See [software/README.md](software/README.md).

```
cd software && pip install -e ".[twin,sim,dev]" && python -m deskarm.twin                  # http://localhost:8080
python -m deskarm.twin --backend physics --payload-kg 0.5                                      # with gravity + torque limits
python -m deskarm.twin --backend can-sim                                                       # real-arm code, simulated MKS drivers
```

## Tools

- `tools/torque_budget.py`: what each joint must hold (worst pose, rated payload) against what its motor and
  ratio deliver at standstill and at full speed, plus a tipping check. It reads the **same** `arm.toml` as the
  simulator, so the budget and the sim can't disagree. Edit the `[[masses]]` list there as the design changes.

  ```
  python3 tools/torque_budget.py
  python3 tools/torque_budget.py --payload-kg 0.5
  ```
- `tools/design_options.py`: the same budget for a set of what-if designs (lighter wrist motors, motors moved
  toward the shoulder, 6-DOF, a shorter arm), side by side. Discussion in [docs/15](docs/15_mass_and_design_options.md).
- `tools/math_figures.py`: redraws the figures of the [math guide](docs/14_math_guide.md) from the real code.
- `tools/cycloid.py`: cycloidal disc generator. Checks eccentricity/undercut/web thickness and writes a
  DXF (disc, ring pins, output holes/pins, bearing) for Fusion's *Insert DXF*.

  ```
  python3 tools/cycloid.py --pins 25 --R 50 --Rr 3 --e 1.6 --out cad/bench/cycloid_j2_24to1.dxf   # J2 stage
  python3 tools/cycloid.py --pins 25 --R 36 --Rr 2.5 --e 1.1 --out-circle 24 --out-pin-dia 5 --out cad/j34_24to1.dxf   # J3/J4, 6804 bearing
  ```
  (`cad/bench/cycloid_j2_30to1.dxf` is the earlier single-stage 30:1 disc, kept as a bench comparison.)
- `tools/gears.py`: planetary gearbox calculator (valid tooth counts for a ratio, 1 or 2 stages) and
  involute gear generator: meshed DXF layout for Fusion + herringbone sun/planet/ring STLs.

  ```
  python3 tools/gears.py search --ratio 25 --module 1.25 --max-od 90 --stages 2
  python3 tools/gears.py make --sun 12 --planet 18 --ring 48 --planets 3 --out cad/bench/planetary_5to1
  ```
- `tools/coupons.py`: tolerance test coupons (pins, bearing seats, heat-set inserts, shaft pegs) as STLs
  in `stl/coupons/`. Record results in [docs/print_tolerances.md](docs/print_tolerances.md).
- `tools/print_pack.py`: the first print jobs (coupons, rings, the J2 module, a bench stand and lever) oriented, placed
  and sliced into Bambu Studio projects in `stl/plates/`, plus the aluminium motor plates as DXF/SVG. See
  [docs/24](docs/24_first_prints_and_j2_bench.md).
- `tools/fusion_skeleton.py`: run inside Fusion 360 (Scripts and Add-Ins) in a new design: the CAD skeleton (link-length
  and limit parameters, joint axes, reference points) from the `arm.toml` numbers. See [docs/16](docs/16_packaging_and_routing.md) §4.
- `tools/bom_total.py`: budget totals in USD + CAD (incl. estimated tax) from `docs/bom.csv`.
  Set a row's `status` to `have` / `makerspace` / `bought` to tick it off.

  ```
  python3 tools/bom_total.py            # totals
  python3 tools/bom_total.py --windows  # spend per purchase window
  ```

## Firmware

[`firmware/base_controller`](firmware/base_controller/README.md): ESP32-S3 USB↔CAN bridge (SLCAN, works with
python-can on Windows and the Pi) + E-stop/bus-health status frames. The protocol core is unit-tested and verified
end-to-end against python-can on a PC, and the ESP32 glue is compile-checked against stubs that mirror the real
ESP-IDF and Arduino headers. Since 13 Oct it also gets a real ESP32-S3 build (`arduino-cli`, locally and in CI for esp32
cores 2.0.17 and 3.3.12), with no warnings. **It has still never run on an ESP32**: the first upload is the real test.

The tools need `pip install -e "software[tools]"` (numpy + manifold3d; `torque_budget.py` imports `deskarm`). CI runs the
firmware host tests and all Python tests (`.github/workflows/tests.yml`).

## Credits

Original design, inspired by PAROL6, Faze4, Thor, OpenArm, Amazing Hand, and others. See [CREDITS.md](CREDITS.md).

## Repo layout

```
docs/        plans, decisions, BOM
tools/       sizing calculators, scripts
cad/         generated bench DXF/STL (the arm itself lives in Fusion: Admin Project / DeskArm_Parts)
stl/         print-ready exports: coupons/, da/ (every printed part from J2 out), j1/ (the J1 hub)
firmware/    base controller and joint encoder; later tool MCU firmware and any custom driver code
hardware/    KiCad schematics for boards A, B and C, BOM for the first PCB order
software/    Python control, URDF; later ROS 2 packages
tests/       actuator bench results (template: tests/actuator_bench.md), later calibration logs
```

Not created yet, referenced by the plan: `stl/print_list.md` (material + printer per part, starts with the first
real part), `docs/log/` (controls log with plots), `software/analysis/` (plotting notebooks), `software/tools/<id>.yaml`
(per-tool TCP and mass), and LICENSE files (deliberately left to the very end of the project, Q3 in
[09](docs/09_decisions_open_questions.md)).

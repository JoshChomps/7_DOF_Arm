# 08: Roadmap: October 2026 → April 2027 (~30 h/week)

**Targets:**
- **Feature-complete: ~6 March 2027**, stretching to mid-February if shipping cooperates.
- **March:** stretch goals (octopus gripper, null-space showcase or Amazing Hand).
- **April:** buffer, polish, demo video.

**"Done" demo:** 7-DOF arm (with joint-lock / 6-DOF mode) + tool changer + 3-prong gripper, vision-guided pick-and-place, live digital
twin, joint-lock modes, and hand/phone-mirror teleop.

At 30 h/week (~750 h to April) shipping and print time are bottlenecks, and so was **mechanical design**: on
1 Oct there was no CAD at all (the full arm was modelled by script on 20-26 Oct, [20](20_cad_full_arm.md), [22](22_print_readiness.md)), the software is months ahead, and four distinct actuator designs (J1; J2; J3/J4; J5/J6,
plus J7) have to be drawn, prototyped and frozen by one person. Expect 3-5 print iterations per gearbox. Treat the
6 March date as optimistic by roughly 4-8 weeks and April as the real buffer. The plan keeps the printer busy at all
times and keeps you on CAD first, then software and PCBs, while parts are in transit.
Order dates come from [13, Purchasing calendar](13_purchasing_calendar.md).

**Minimum viable arm (protect this if the schedule slips):** J1-J4 moving from the twin by mid-December, the full arm
in 6-DOF mode (J3 locked) by the end of January, the 3-prong gripper by the end of February.

```
            Oct         Nov         Dec         Jan         Feb         Mar         Apr
Orders   W1 W2 W3        W4 W5                   W6 ▲CNY                 W7
Bench    ███████                                                                         (PLA protos → real)
Twin/SW  ████████████████████████████████████████████████████████████████████████
PCB      ██████(A)               ████(B)
J1-J4             ██████████████
Wrist                         ███████████
Tools/Grip                                ████████
Vision/Tele                                     ██████████
Stretch                                                       ███████████
Buffer                                                                    ░░░░░░░░
```

## Weeks 1-3 (30 Sep: 20 Oct): Foundations, sim, PLA prototypes
- [x] **Decide the motor list** (Q5/Q6 in [09](09_decisions_open_questions.md), analysis in [15](15_mass_and_design_options.md)): **option B, the light wrist**, accepted 7 Oct and applied to `arm.toml`, the BOM and the docs
- [x] **W1 order** (placed 7-8 Oct; parcels due ~late Oct): all motors + drivers (**CAN versions**), PSU, E-stop + fuses, ESP32s, CAN modules, **USB-CAN adapter**,
      buck converters, assortments, **output bearings (6810 ×8, 6806 ×6: the gearboxes are designed around them)**,
      **HTD-3M belts + pulleys**, **2 output encoders for the bench**, SimpleFOC kit, logic analyzer
- [ ] Mass budget: `python3 tools/torque_budget.py`. Re-run it every time a mass in `arm.toml` changes
- [ ] **W2 (6-7 Oct, Prime Days):** tools, PETG ×5, grease (TPU and ASA wait for Black Friday)
- [ ] Tolerance coupons in **free makerspace PLA** → `docs/print_tolerances.md` (per printer). STLs ready: `stl/coupons/` (`tools/coupons.py`)
- [x] Fusion **skeleton** with user parameters (PAROL6-class: 190/180/70 mm, 7-DOF SRS frames), workspace check.
      Decide motor positions and the harness route per joint ([03](03_actuators_gearboxes.md), "Hollow shafts") (skeleton 13 Oct; motor positions and routing in the 20 Oct CAD)
- [x] Read the MKS CAN manual and write `deskarm.backends.CanBackend`, tested against our own simulated drivers (`deskarm.mks.SimBus`) (done 12 Oct; not yet run on a driver)
- [x] **Digital twin v0 in sim:** Viser + simple 7-DOF URDF + FK, analytic IK (J3 locked) + numerical DLS IK, joint locks, elbow swivel, pytest round-trip tests → [`software/`](../software/README.md) (done 1 Oct)
- [ ] Design and PLA-print test-bench gearboxes for the NEMA23: **2:1 belt + 24:1 cycloidal** (the J2 actuator) and a 2-stage planetary ~25:1 as the comparison. Disc profile: `python3 tools/cycloid.py` → `cad/bench/cycloid_j2_24to1.dxf` generated (the earlier single-stage `cycloid_j2_30to1.dxf` is kept as a second comparison)
- [ ] KiCad: PCB A (power/E-stop) + B (ESP32-S3 base controller) → **W3: JLC order #1 (mid-late Oct, decided 12 Oct)**. Schematics A, B (and C) drawn 14-15 Oct, layout still to do. Base-controller firmware written + host-tested: [`firmware/base_controller`](../firmware/base_controller/README.md), re-checked 1 Oct against the ESP-IDF and Arduino headers (first real build when the ESP32-S3 arrives)
- **Exit:** twin moves a simulated arm to targets; PLA gearboxes assemble; PCB #1 ordered.

## Weeks 4-6 (21 Oct: 10 Nov): Actuator bench + J1-J4 CAD
- [ ] W1 parts arrive (~late Oct, after Golden Week) → flash the base controller (Arduino IDE, settings in the firmware README), confirm
      `deskarm.bridge.open_bus` reports the firmware version, then the first motor moving from Python over CAN
- [ ] **First bench test: streaming.** Send the driver a new position target every 10-20 ms and watch what it does
      ([04](04_electronics_control.md), "Streaming setpoints"). Everything in the twin assumes this works
- [ ] Bench: PLA prototype → **PETG** version (PETG-CF only as the optional experiment, D16). Measure backlash, stiffness,
      torque vs speed, backdrive, efficiency, heat → [`tests/actuator_bench.md`](../tests/actuator_bench.md) → **gearbox decision**.
      Put the measured numbers into `arm.toml` (`gear_stiffness`, `backlash_deg`, `efficiency`, `[drive] torque_derate`) so the physics sim matches
- [ ] SimpleFOC testbed: current/velocity/position loops, first Bode plot
- [x] CAD J1, J2, **J3 upper-arm roll** + printed upper-arm link, J4 (belt + cycloid); **J3 and J4 share one actuator design** (done 20 Oct, docs/19 §14 and docs/20). The counterbalance was sized at ~1.5 N/mm with the CAD's 70 / 98 mm geometry and deferred (D41)
- [x] Copy CAD masses and centres of mass into `arm.toml` `[[masses]]`; re-run `tools/torque_budget.py` and the spring sizing (20 Oct, `tools/cad_masses.py`; re-run after each CAD change)
- **Exit:** gearbox types decided; J1-J4 CAD frozen; **W4 (11.11) order** of wrist belts, slip ring, spring, remaining encoders, pogo pins, gripper parts.

## Weeks 7-10 (11 Nov: 8 Dec): Build J1, J2, J3, J4
- [ ] Print J1-J4 (A1 mini running continuously, discs/gears at the friend's)
- [ ] PCB rev A bring-up (current-limited supply first). Twin shows commanded vs measured
- [ ] Assemble J1-J4 as parts arrive, wire the 4-wire CAN harness, homing, soft limits, **E-stop test**
- [ ] **W5 (Black Friday):** ASA and TPU (and PETG-CF if you want the optional bench comparison)
- [x] Wrist CAD: spherical wrist, **hollow J7 for the 8-wire slip ring**, ISO 9409-31.5 flange (done early, 20-21 Oct)
- **Exit:** 4-axis arm (J1, J2, J3, J4) moving from the twin under load.

## Weeks 11-14 (9 Dec: 5 Jan): Wrist + full 7-axis
- [ ] Print + assemble J5-J7, slip ring, flange
- [ ] Full arm moves from the twin: **6-DOF mode first** (J3 locked, analytic IK), then **7-DOF mode** (DLS + null space). First repeatability test
- [ ] Output encoders + dual-loop control
- [ ] **W6 (by ~10 Jan):** PCB order #2 (rev B + tool boards) + anything missing, **before Chinese New Year**
- **Exit:** the flange goes where the twin says, repeatably.

## Weeks 15-18 (6 Jan: 2 Feb): Tool changer + 3-prong gripper + control quality
- [ ] Kinematic-coupling plates, bayonet, pogo block, tool MCU firmware (ID + open/close)
- [ ] 3-prong self-centering gripper with Fin Ray TPU tips
- [x] S-curve trajectories, TOPP-RA (done early in sim, 1 Oct, along with straight-line moves, teach & replay, MuJoCo physics + torque check, gearbox compliance/backlash, self-collision check: `software/deskarm`). Still to do on hardware: run them on the real arm
- **Exit:** tool swap < 30 s with auto-detect; pick-and-place from the twin.

## Weeks 19-22 (3 Feb: 6 Mar): Vision, teleop, joint lock → feature-complete
- [ ] Camera (iPhone 14 Pro first; its LiDAR through Record3D for depth) calibration + hand-eye; AprilTag pick-and-place
- [ ] **Hand mirroring** (MediaPipe) and/or **phone 6-DoF controller**
- [x] **Joint-lock / constrained IK** modes in the UI (done early in sim: locks, tool-vertical, translate-only, plane, desk guard)
- [ ] (Optional) Pi as onboard controller + `ros2_control`/MoveIt wrapper
- **Exit: feature-complete demo.**

## Weeks 23-27 (7 Mar: 10 Apr): Stretch
Pick what interests you most:
- [ ] Octopus gripper (SpiRobs-style tentacles; suction add-on)
- [ ] Null-space control showcase: elbow swivel, joint-limit avoidance, full-arm mirroring (MediaPipe Pose)
- [ ] Amazing Hand adapter
- [ ] MPC / visual servoing write-ups
- [ ] **W7 (Anniversary sale, ~mid-late Mar):** parts for whichever you pick (these are outside the USD 1,000 core budget)

## April: buffer, polish, demo video, write-up, and the licence decision (Q3: LICENSE files before the repo goes public)
## After April: humanoid hand (see the plan in [06](06_end_effectors_tool_changer.md)), custom FOC driver, BLDC v2 joint, LeRobot

## Speedrun rules
1. **Never wait on shipping idle**: CAD, software, and PCBs always have work queued.
2. **The printer is the bottleneck**: prototype in free PLA at the makerspace, keep the A1 mini on final parts, and send gears to your friend.
3. **Buy, don't build, anything on the critical path** (joint drivers). Custom boards always have a dev-board fallback.
4. **Sim first**: every feature works in the twin before it touches hardware.
5. **If behind, cut in this order:** ROS 2 wrapper → octopus → hand mirroring polish. **Never cut:** E-stop, homing, limits.

# 17: Project history and research notes

What was done, when, why, and what was learned on the way. The other docs say what the plan *is*; this one says how
it got there, including the mistakes, so the reasoning can be studied later and nobody re-derives a settled result.

How to use it:

- **One entry per working day**, newest at the bottom. Each entry: what was asked, what was done, what was learned
  (with the source), what was decided (decision numbers from [09](09_decisions_open_questions.md)), and what is still
  unverified. Link to where the result lives rather than repeating it.
- **Corrections stay visible.** When a later day finds an earlier conclusion wrong, add it to the *Corrections* table
  at the end and say what changed; don't rewrite the old entry.
- Commits are named by their short hash so `git show <hash>` brings back the exact state.

## 30 Sep 2026: the plan, three times over

Commits `4e3e928` → `f498554`.

- **Asked:** a plan for a 3D-printed desk arm on a student budget.
- **Done:** requirements, prior-art survey, gearbox comparison, CAN electronics, software plan, tool changer, BOM and
  roadmap ([01](01_requirements.md), [13](13_purchasing_calendar.md)). Then three revisions the same day: budget in CAD
  then USD with a purchasing calendar of sales; the spec benchmarked on PAROL6/Faze4 (~440 mm, 1 kg at full reach);
  the switch from 6-DOF to **7-DOF SRS** with an upper-arm roll that reuses the elbow actuator; printed box-section
  links instead of aluminium tube.
- **Learned:** a 60 × 60 mm PETG box link is about as stiff as a 25 × 25 mm aluminium tube, so "fully printed" is not a
  stiffness compromise at this size ([09](09_decisions_open_questions.md) D7). PAROL6, Faze4, AR4 and Atlas
  ([02](02_prior_art_research.md)) set the reference points; Atlas already runs hollow cycloids with one CAN bus.
- **Decided:** D1 (7-DOF SRS), D7 (fully printed), budget ≤ USD 1,000 with tools tracked separately.

## 1 Oct: the software, ahead of the hardware

Commits `156240a` → `525e32f`.

- **Done:** `deskarm` (DH model, FK, Jacobian, closed-form and damped-least-squares IK, null space and elbow swivel),
  the Viser digital twin, the ESP32-S3 USB↔CAN bridge firmware with host tests, the cycloid and planetary generators,
  print-tolerance coupons, the MuJoCo physics backend, straight-line moves, teach and replay, gearbox compliance and
  backlash, the J2 spring, S-curve and time-optimal timing, self-collision checks.
- **Learned:** an **audit of the simulator's own numbers** found two of them wrong. The J2 spring sized to cancel one
  pose (917 N/m) over-balanced the folded arm; sizing it to minimise the worst case over the whole workspace gave
  623 N/m and showed the real benefit is ~50%, not the ~80% first claimed. Backlash was set from published
  measurements of printed drives (0.5° cycloid, 0.6-0.8° planetary), which raised backlash to a high risk (~4 mm of
  play at the tool per base joint).
- **Decided:** D3, D10, D16-D25, D27 (masses live only in `arm.toml`, joint speeds are power-limited, bearings chosen
  before CAD, motors beside the axis on belt-driven joints).

## 1-4 Oct: evaluation and handoff

- **Done:** a review of the whole repo and the first project log (4 Oct), so the work can be picked up from one
  file after a break.
- **Learned:** Josh wants the math code kept compact and explained in a separate guide with figures (which became
  [14](14_math_guide.md)), and every doc kept as an accurate record.

## 6 Oct: firmware route, MKS manual, option B checked

- **Done:** the firmware folder became an Arduino sketch (D28: Arduino IDE, PlatformIO kept as a second route). The
  light-wrist "option B" was checked in depth before Josh decided (table in the project log §11).
- **Learned (MKS SERVO42D/57D CAN manual V1.0.9):** the drivers take **12-24 V only**, so more voltage is not a route to
  speed; the absolute-position command F5 accepts a new target mid-move (good for streamed setpoints, Q8); **on a stall
  the driver releases the shaft**; third-party motors must be 1.8°, under 10 Ω, with a magnet on the rear shaft. The
  printed wrist gears, not the motors, are the weak point in both wrist options (module 1.0 forced by the 65 mm wrist).
- **Learned:** Kairo, Josh's EDA MCP, was only scaffolded, so the first PCBs are drawn by hand.

## 7 Oct: option B accepted, packaging plan

- **Done:** option B applied everywhere (D26: lighter J5/J6 motors, J7 pancake at 15:1); the packaging and routing
  plan ([16](16_packaging_and_routing.md)); an Amazon.ca order (includes the 24 V PSU, not itemised).
- **Learned:** SolidWorks was considered for routing and rejected: four wires don't need a routing add-in, and the schedule risk is mechanical
  design time.

## 8 Oct: motors bought, motor model, design check

Commits `30f3b11`, `92f8656`.

- **Done:** all seven motors from Spool3D (D29: LDO NEMA17s and an LKD NEMA23; StepperOnline direct wanted ~253 in
  shipping), MKS drivers, CANable and J2 bench
  parts on AliExpress, 6810/6806 bearings on Amazon.ca. A stepper torque-speed model (`deskarm.motor`,
  [14](14_math_guide.md) §11) now feeds the budget, the simulator and the planner. The math guide was written. A design
  check of every joint. Later the same day: D30 (wrist 22.2:1), D31 (6804 on J3/J4), D32 (stop policy) and
  `deskarm.mks` for the F7H stop frame.
- **Learned:**
  - The **LDO 60 mm motor's torque falls off at ~350 rpm** (higher inductance), so J1 runs near its corner speed; the
    time-optimal planner overshot the torque margin by ~7% until it took the limit at the fast end of each grid cell.
  - **MKS current setting** (manual V1.0.5): "working current" is not stated as peak or RMS. A bench test with a
    luggage scale decides it. In CLOSE mode the holding current is at most 90% (default 50%), so use SR_vFOC.
  - **The arm can backdrive unpowered** by its own efficiency numbers, so the E-stop (a power cut) may let it sag;
    hence D32: ordinary stops use the driver's stop-and-hold.
  - J3/J4's 12 mm hollow input does not fit a 6802 (0.4 mm wall), hence D31. Belt length depends on where the motor
    sits. The printed hollow eccentric shaft is the most stressed printed part.
  - **AliExpress prices:** "welcome deal" prices apply to one item per order and "free shipping" on a listing is not
    reliable; early lists were ~C$100 low. Quote regular prices and compare against Amazon.ca.
- **Unverified:** nothing has run on hardware; the NEMA23's rear shaft end for the MKS magnet is unchecked.

## 12 Oct: committed, pushed, how to read hollow joints

Commits `92f8656`, `e7896ae` and a one-line follow-up recording the CI result.

- **Asked:** commit and push, sweep stale docs, expand how to read the hollow joints, and make the docs work as a
  history and learning record (this file).
- **Done:** the late 8 Oct work committed on its own, then pushed with today's docs, so CI runs for the first time
  since 1 Oct (the last pushed commit was `525e32f`). It passed: 126 tests on Ubuntu, including the three
  python-can end-to-end tests that Windows skips. Josh set up the 11.11 order list; the BOM waits until he orders. Stale lines fixed (W1 is placed, J5/J6
  margin 1.29, contingency USD 1,104 as `bom_total.py` prints it). [16](16_packaging_and_routing.md) §3 rewritten.
- **Learned (sources: MKS CAN manual V1.0.9 §5.1; MT6701 datasheet Rev 0.1; MA732 datasheet Rev 1.0; K&J
  Magnetics' diametric ring catalogue):**
  - The MKS motor encoder is **14 bits, absolute within one motor turn**; after a power cut only the turn count is
    lost. So a coarse joint reading better than ±180°/R (±3.75° on the 48:1 joints) plus the motor encoder gives the
    exact joint angle without homing. The coarse error budget on J2 is ~3.1° uncorrected, ~1.7° after subtracting the
    predicted gear wind-up.
  - MT6701: INL ±0.5° typical, Ø8 × 2.5 mm magnet, centred within 0.3 mm, no off-axis mode described. MA732: INL
    0.7-1.1°, supports "side-shaft" reading beside a ring magnet with a trim register. Large-bore diametric rings are
    not stock parts.
  - INL is systematic, so it hurts accuracy, not repeatability; the 0.5 mm target is repeatability.
- **Proposed (Q9):** through-rod + MT6701 on J2/J4, through-tube + small pickup on J3, no output encoder on J1/J7,
  the wrist decided in its CAD. Try the through-rod on the J2 bench gearbox first.
- **Unverified:** the motor encoder's power-up behaviour (bench check in [16](16_packaging_and_routing.md) §3.2).

## 12 Oct (later): CanBackend

- **Asked:** write `CanBackend`; PCB order 1 goes in mid-late October; Josh pointed out that the motor angles are
  computed apart from the gearbox they drive, to keep in mind.
- **Done:** `deskarm.mks` (F5 move, replies, acceleration byte, `SimBus` simulated drivers), `CanBackend` (enable +
  heartbeat, one absolute move per joint per step, position polls, faults stop and hold), `absolute_angle` for the
  start-up with its math in [14](14_math_guide.md) §16 and a figure, `--backend can` / `can-sim` in the twin, 15 new
  tests (frames against the manual's worked examples, the twin driving simulated drivers in real time). Q7 answered.
- **Learned:**
  - On Josh's point: the drivers measure the **motor**, behind the belt and the reducer. So (1) counts become joint
    angles only through the whole drive's ratio, which must be exact: the wrist's 22.22 in `arm.toml` became
    (1 + 52/14)² = 22.2245, since rounding would cost 0.04° over 180° on every move; (2) backlash and wind-up sit
    between motor and link, so `CanBackend.read()` is the angle the motors imply, not the link's. The output encoders
    measure the link.
  - MKS units: 16384 counts per motor turn, F5 speed in rpm at MStep 16/32/64, acceleration byte = 1 rpm per
    (256 − acc) × 50 µs, coordinate 0 at power-up (manual V1.0.9 §1.11, §9.1, §11.4).
  - A wrong turn count from a bad coarse reading can't be caught by comparing it with the result: the residual still
    looks small. Only the error budget protects against it.
- **Unverified:** all driver behaviour; whether streamed F5 moves are smooth (Q8).
- **Also:** Josh asked for the sensor choice (MT6701 everywhere over SSI, Ø8 × 2.5 mm magnets, board C on an ESP32-C3,
  hall switches on J1/J7: [16](16_packaging_and_routing.md) §3.7) and for the full to-do list, now the project log §10.

## 13 Oct: joint sensing decided

- **Decided:** Josh accepted the sensing plan as **D33**: through-rod + MT6701 on J2/J4 (their harness loops outside),
  through-tube + a 36T GT2 pickup on J3, no output encoder on J1/J7 (motor encoder + saved angle + hall switch), J5/J6
  in the wrist CAD. Parts: MT6701 over SSI, Ø8 × 2.5 mm diametric magnets, board C = ESP32-C3 + MT6701 + SN65HVD230
  + buck, A3144-type hall switches. Fallback if time runs short: hall homing everywhere, with the rod seats kept in the
  CAD so the encoders drop in later.
- **Why** (from the 12 Oct research, [16](16_packaging_and_routing.md) §3): every chosen route puts the magnet on the
  axis, so no side-shaft chip is needed; the MT6701 gives 4× the AS5600's resolution for the same price; an ESP32-C3
  keeps board C on the same Arduino toolchain and CAN peripheral as board B.
- **What it changes:** J4's harness becomes an external loop; board C can join PCB order 1 if drawn in time; the
  11.11 order gains magnets, MR126 bearings, a Ø6 rod and the J3 pickup parts (`bom.csv` waits until it is placed).

- **Also 13 Oct:** Josh said to commit (`c412766`, not pushed) and to put Arduino on D:. `arduino-cli` 1.5.1 went to
  `D:\Arduino` with its data there too, and the base controller got its **first real ESP32-S3 build** (esp32 core
  3.3.12): it compiles with no warnings. A CI job now builds it for cores 2.0.17 and 3.3.12. Found on the way: Arduino IDE
  2.3.8 was already installed on C: with core 3.3.7; moving it to D: is his call.

- **Later 13 Oct:** Josh said yes to moving the Arduino IDE: 2.3.10 now lives in `D:\Arduino\IDE` (official installer,
  hash and signature checked), sharing `D:\Arduino\data` with `arduino-cli`; the C: copy and 7.8 GB of C: data went.
- **The CAD starts (13 Oct).** Josh opened Fusion; the Autodesk Fusion MCP connection let the agent build the skeleton in
  his new design: link-length and joint-limit parameters, level planes, key points and the seven joint axes, checked
  against the software's forward kinematics (shoulder 130, elbow 320, wrist centre 500, flange 570, TCP 650 mm) and
  for parametric behaviour. The pitch axes first came out along −Y; they were flipped to +Y so positive joint angles
  in Fusion agree with the DH frames. Saved as `tools/fusion_skeleton.py`, with a test that ties its numbers to
  `arm.toml`. The design itself is unsaved until Josh saves it.

- **Software list, 13 Oct (evening).** Josh said Fusion waits; do the other items first (the project log §10 F2-F6). Done, all
  against simulated drivers only:
  - `deskarm.drivers`: set up and read back every MKS driver over CAN from one table (`--assign`, `--calibrate`,
    `--write`, `--jog`). Read-back format checked against the manual's example (`00 83` → `83 0C 80` = 3200 mA).
  - **Learned (manual V1.0.9 §5.2.7, §9.2.2):** the driver's Dir setting only applies to the pulse interface, and in a
    bus mode the En pin is ignored. So direction is now `motor_sign` per joint in `arm.toml` (found with `--jog`).
  - `deskarm.bench`: CSV logger through `CanBackend` with a one-joint arm (Q8 `--sine`), and the push test
    (`--push`) with the fit h = 2τ/k + b, docs/14 §17 (new; old §17/18 are now §18/19). The simulated run recovers
    k = 614 for 600 N·m/rad: the MT6701's 14-bit step limits it to a few per cent.
  - Firmware 0.2: the base controller reads a bench MT6701 over SSI (GPIO10 CSN, 12 CLK, 13 DO; off unless
    `BENCH_ENCODER_JOINT` is set) and sends the joint-angle frame board C will use: id 0x10 + driver id, 4 bytes.
    The CRC-6 initial value is assumed (no test vector found), so frames carry a CRC-ok flag. Real ESP32-S3 build passes.
  - `deskarm.startup`: saved angles, start-angle choice, hall check of J1/J7 on the driver's IN_1 (34H), homing sweep.
  - `CanBackend(mode="speed")`: the F6 fallback for Q8. In the ideal sim it tracks a 20°/s ramp to 0.007° against
    0.8° for streamed moves; the sim has no ramp, so that says nothing about the real drivers.
  - Tests: 161 passed, 3 skipped; firmware host tests pass. **Nothing has run on hardware.**

## 14 Oct: drawings for Fusion, and the math guide rewritten with derivations

- **Asked:** specs and drawings of what to model in Fusion; then, finding the first drawing set too general, plans you
  can actually sketch from; then a math guide that derives things instead of stating them, with working LaTeX.
- **Done:**
  - Two pages (private web pages): a 7-sheet *drawing set* for the whole arm (space budget along the
    arm, base board, cycloid and planetary actuators, J7 and the flange) and *J2 bench gearbox plans* (each printed
    part as a numbered profile to sketch on XZ, to-scale drawings, Fusion steps, parts list, assembly order). Every part
    of the J2 plans comes from one set of numbers; an overlap check of the section found no unintended overlaps.
  - The space budget found four places that don't fit on paper yet: the J1 stack under the J2 housing (52 mm), the J5
    actuator on the forearm (~110 needed, 103 free), J7's 37.5 mm, and the J7 driver against the tool or J6's swing.
  - J2 plan choices: Ø15 input-shaft journals with printed sleeves under the 6804s (a bearing has to slide over the
    journal and shift 1.6 mm onto its lobe, so the journal can be at most Ø16.8), a Ø8 bore (only the Ø6 rod passes on
    J2), the MR126 moved into the encoder cap, the motor on a slotted rear plate.
  - `docs/14_math_guide.md` rewritten: every section has a step-by-step derivation and the same math worked with this
    arm's numbers (e.g. the J2 lever table that sums to 14.92 N·m, the IK of a real pose by hand, J2's motor corner at
    267 rpm and its 56 W power plateau, the J2 belt's 92.5 mm centre distance from the exact belt formula). §15 now
    derives every number in the J2 plans.
- **Learned:**
  - GitHub's Markdown removes a backslash in front of punctuation before the math is drawn, even inside `$$`: `\\`
    became `\`, so **every matrix in the guide rendered as one row**, and `\,` `\;` `\_` were mangled; it also
    double-escapes `<`. Checked with GitHub's own renderer (`gh api markdown`): 33 of the 44 display equations were
    broken. The rewrite uses `\cr`, `\thinspace`, `\lbrace`, `\lt`; all 86 display and 857 inline equations now
    reach GitHub unchanged, all typeset in MathJax (GitHub) and KaTeX (VS Code), and a test keeps it so.
  - The J2 printed input shaft: at full holding torque about 20 MPa combined (twist plus belt bending) at the double-D
    under the pulley, 15 MPa at the planned torque, against ~45-50 MPa for PETG. Fine for the bench, not generous.
- **Unverified:** none of the plans has been built or printed; the stress numbers are hand estimates.

## 14 Oct (later): output encoders in the software

- **Asked:** "familiarise yourself with the project and continue wherever it left off". The next agent item on the
  list was the project log §10 F7: output encoders in `CanBackend` and the calibration fit.
- **Done** (all against simulated drivers and a simulated encoder):
  - `deskarm.encoders`: `Calibration` maps a raw MT6701 reading to the link angle (sign, zero, a few harmonics of
    INL), `fit` finds it by least squares from slow sweeps logged against the motor-implied angle, together with the
    lost motion and the gearbox stiffness, and `sweep` drives one joint through `CanBackend` to log one. CLI
    `python -m deskarm.encoders` for the bench; calibrations saved by joint name in `~/.deskarm/encoders.json`.
  - `CanBackend(encoders=...)`: `read()` is now the encoder's link angle on those joints, `read_motor()` stays the motor
    side. They take their start angle from the encoder, and two new faults stop the arm: an encoder silent for
    `patience` steps, and link vs motor apart by more than the gearbox can wind up (`slip_limit`). `SimBus` gained
    `encoder_error` (a zero offset and INL for the simulated encoder).
  - docs/14 §18 (new; the old §18/19 are now §19/20) derives the fit, with a figure, the numbers and a runnable example.
    Software README, `tests/actuator_bench.md` (§1b, the bench calibration) and docs/16 §3.4 updated.
- **Learned:**
  - **One calibration sweep can't separate the sensor's error from the gearbox's wind-up.** J2's load during a sweep
    is a rigid body turning about a horizontal axis, so its gravity torque is exactly a first harmonic of the angle,
    and so is the zero-length spring's. The wind-up τ/k is then indistinguishable from first-harmonic INL (the
    least-squares matrix is singular: smallest singular value 6×10⁻¹⁶ of the largest). Two poses with different loads
    fix it (0.14). In the simulated example, ignoring the load leaves 0.19° of error and a one-pose calibration used at
    another pose 0.12°; the full fit gets 0.012°, about the 14-bit step.
  - The spring cancels most of J2's load: gravity minus spring is only ±1.2 N·m with the elbow straight and up to
    ~4 N·m bent, so J2's wind-up during a calibration is 0.1-0.4° at the placeholder stiffness. Small, but well above the
    MT6701's 0.022° step.
- **Decided:** nothing new from Josh. Design choices made in the code: the encoder joints take their start from the
  encoder (not `q_start`), and the slip limit is the whole backlash + the wind-up under 1.5× the torque limit + 1°.
- **Checked:** 171 passed, 3 skipped (the python-can end-to-end tests, Linux only). No firmware change.
- **Unverified:** everything above on hardware; the real INL shape; the frame-vs-poll timing (5 ms at 5°/s looks like
  0.05° of lost motion).

## 14 Oct (evening): dual-loop control in the physics sim

- **Asked:** do the project log §10 F8 here (the first controls-track piece).
- **Done:** `deskarm.control.DualLoop` wraps any backend with both readings and commands the motor to
  q* + predicted sag + an integral of the link error. The integral is frozen during moves, has a dead zone just over half
  an MT6701 step and is clamped to what the gearbox can wind up. J2-J4 by default (D33). Twin flag `--dual-loop`;
  docs/14 §19 (new; the old §19/20 are now §20/21) with a figure, the stability derivation and an example; tests in
  `test_control.py`.
- **Result in the sim** (stretched, 1 kg, the loop told 0 kg, 14-bit readings one poll late): the motors alone leave J2
  at 0.87° and J4 at 1.52° (~17 mm at the tool); with the loop both are within 0.02° after ~2 s and 0.003° at rest. A
  stiffness 30% off in the model gives the same. After a move, 0.43-0.67° becomes 0.035° in 3 s.
- **Learned:**
  - The integral gain has a hard ceiling set by the arm's first mode: the ringing decays at ζω − ki/2, so ki must stay
    under 2ζω. Measured in the sim with a step: 3.82 Hz, ζ 0.054 stretched with 1 kg, so 2.59 /s; the sim agrees (2 settles,
    3 rings). Lightly damped printed gears make the outer loop slow (ki = 1 /s, a ~1 s time constant).
  - Found while checking the twin: enabling the loop first held every joint at its *link* angle, which made the
    joints outside the loop (J5/J6) sag a second time (1.1° → 2.1° in the twin). Those now hold their motor position;
    a test covers it.
  - A dead zone narrower than half an encoder step can leave no reading "on target", so the integral would hunt by one
    step; 0.013° (> 0.011°) avoids it.
- **Checked:** 179 passed, 3 skipped; the twin runs with `--dual-loop` (enable holds the pose, a joint move leaves only J6, outside the loop, off target).
- **Committed** as `299e23c` (Josh said "commit, don't push"), then **pushed 15 Oct** when he asked, with the three
  13 Oct commits. CI passed: the software job and the first GitHub builds of the firmware on esp32 cores 2.0.17 and 3.3.12.
- **Unverified:** everything on hardware; real gear friction (the sim's backlash has none, so a link floats in its gap);
  the real first mode and damping.

## 14 Oct (night): boards A and B drawn with Kairo

- **Asked:** draft the schematics for PCB order 1 (board A power/E-stop, board B base controller) with Kairo; pins
  only from its library tools, numbers only from its calculations; no layout.
- **Josh decided:** the projects go in `D:\KiCad` (Kairo's only allowed root) and are copied to a new
  [hardware/](../hardware/README.md) folder, which is the master copy. PSU confirmed as the **Mean Well LRS-350-24**.
  He settled two choices: **5 V for the boards only** (2 A; the Pi keeps its own USB-C supply) and the E-stop
  as **option A** (the button's NC contact drives a 24 V contactor coil; the motor current never crosses a PCB relay).
- **Done:** board A (64 parts, two sheets): two parallel AOD4185 P-FETs for reverse polarity, SMCJ26CA TVS on both
  rails, F1 logic and F2 coil fuses, E-stop loop and contactor coil terminals with a DMN6075S software-enable FET
  bypassed by a fitted 0 Ω, contactor main-contact terminal, 2 mΩ shunt + INA226, four blade-fused XT30 branches,
  1000 µF bulk, TPS54560B 5 V buck from TI's design example. Board B (52 parts, one A0 sheet): ESP32-S3-WROOM-1,
  USB-C + USBLC6, SN65HVD230 with a jumpered 120 Ω, E-stop sense on GPIO6, WS2812B on GPIO48, MT6701 header on GPIO10/12/13,
  I²C/ALERT/MOTOR_EN to board A, two hall inputs, AP2112K-3.3 fed through SS14 diodes from USB or board A. Parts list
  in `hardware/bom_order1.csv`.
- **Checked:** KiCad 10.0.6 ERC **clean** on both projects (board A as the joined hierarchy); Kairo's verifier passes
  on every sheet it built; `trace_net` shows the firmware's pins (IO4/5 CAN, IO6 E-stop, IO10/12/13 MT6701, IO48 LED)
  and the nets that cross board A's sheets. Kairo requirements passed: 5 V rail 4.871-5.14 V, buck inductor peak
  2.83 A (7 A rated), switch limit, 3.3 V rail 3.25-3.349 V. Kairo `calc`: each reverse FET 1.23 W at the PSU's 14.6 A
  if shared (4.9 W alone, hence two); gate 16.8 V at the PSU's 33.6 V over-voltage ceiling; 14.6 A needs 7.96 mm on
  1 oz or 3.98 mm on 2 oz (IPC-2221, 20 °C rise).
- **Learned:**
  - Kairo places blocks in one row of A4-height columns and writes one sheet, so a ~60-part board needs more than A0
    (board A asked for 1219 mm). Signal nets are drawn as local labels, power nets as global labels or power
    symbols, so a design can be split only along power nets. Board A was split, each half built and verified by Kairo,
    then joined with a hand-written top sheet; the only ERC finding of the join was a second PWR_FLAG on each shared
    rail, removed from the copy. (`hardware/kairo_tools/`, explained in `hardware/README.md`.)
  - Two of Kairo's global labels on adjacent connector pins overlap (2.54 mm pitch), and Kairo refuses the drawing.
    Naming shared rails after KiCad power symbols (`+24V`, `+3V3`, `+5VD`, `VBUS`) avoids it.
  - WS2812B-2020 (datasheet V1.3): VDD 3.7-5.3 V, but DIN high is ≥ 2.7 V absolute, so it runs from 5 V with 3.3 V data.
  - LRS-350-24 (Mean Well spec 2025-09-12): 14.6 A, ±1 %, trim 21.6-28.8 V, over-voltage protection 28.8-33.6 V. A
    28 V-max buck such as the TPS54202 would not survive the OVP range, hence the 60 V TPS54560B.
  - The INA226's logic high is 0.7 × its supply (SBOS547 p.5), so on 5 V it would not read 3.3 V I²C; it runs from
    board B's 3.3 V through the A↔B cable.
- **Unverified:** 5 V ripple (three output caps), buck input capacitance under DC bias, buck loop stability with RT
  moved from TI's 243 k to 200 k, the AP2112's junction temperature and total load (pull-up currents not simulated;
  the ESP32's 107.9 mA is typical), the AP2112's output-capacitor ESR window, the FETs' junction temperature, footprint
  pad counts against the real parts, the connectors the MKS drivers use, and everything on hardware. Open for Josh:
  contactor part, fuse ratings, connectors, one or two reverse FETs, TVS voltage vs the PSU trim, bulk capacitor
  (list in `hardware/README.md`).

## 15 Oct: board C drawn with Kairo

- **Asked** (by the session that drew boards A and B): draft board C, the joint encoder CAN node of D33, with Kairo;
  same rules (pins from its library tools, numbers from `calc` and datasheets read with its tools); no layout, no
  commit.
- **Done:** `hardware/board_c/` (59 parts, one A0 sheet, built in `D:\KiCad\projects\deskarm\board_c` and copied):
  ESP32-C3-WROOM-02; MT6701CT (SOIC-8) on the board, read over SSI on IO10/6/7, MODE held high; SN65HVD230 on IO4/5
  with the 120 Ω on an open jumper (board C sits mid-bus); USB-C on the C3's own USB Serial/JTAG for flashing; 24 V
  tap → SS16 reverse diode → SMAJ36A → LMR16006YQ5 fixed 5 V buck (L1 10 µH) → SS14 diode-OR with USB → AP2112K-3.3;
  CAN in/out connectors daisy-chained through the board; RESET/BOOT, strapping pull-ups, status LED, AUX header,
  11 test points. Parts list added to `hardware/bom_order1.csv`.
- **Checked:** KiCad 10.0.6 ERC **clean**; Kairo's verifier passes all five layers; `read_schematic` net pin counts
  match the design; Kairo's review has no faults or suggestions; 3.3 V rail 3.25-3.349 V passes. `calc`: L ≥ 9.645 µH
  at 28.8 V and 1785 kHz with 0.24 A ripple; with 8 µH (10 µH −20 %) the ripple is 0.296 A and the peak 0.748 A at
  the 600 mA rating, under L1's 2.4 A saturation (LCSC listing). Python tests 179 passed, 3 skipped (after a
  docstring change).
- **Learned:**
  - Kairo recognises a buck only when the chip's switch pin is typed `power_out`. KiCad types the LMR16006's SW pin
    `output`, so Kairo cannot check that rail, its inductor or the chip's heat, though the design states
    `regulator: switching` and the wiring is right. The TPS54560B on board A has a `power_out` SW pin, which is why
    its checks ran. (A Kairo improvement: accept an `output` switch pin when `regulator` is `switching`.)
  - `trace_net` wants KiCad's net names (`/ENC_CSN`, not `ENC_CSN`) and locks itself for the session after five
    misses in a row, so read the names with `read_schematic` first.
  - MagnTek's datasheet link in KiCad's library returns 404; LCSC hosts v1.9 (2024.05, Chinese) and an English Rev 0.1
    (2020) is mirrored elsewhere. v1.9 gives Idd 10 mA typical, 14 mA max; MODE and CSN have 200 k internal pull-ups
    (MODE high = I²C/SSI); DO changes on CLK's rising edge, sample after the falling one, as `main.cpp` does.
  - **v1.9 gives the MT6701's INL as ±1.0° typical (25 °C, aligned) and ±1.5° max over temperature**, not the ±0.5°
    typical of Rev 0.1. The coarse error budget (docs/16 §3.3) grows to ~3.5° on J2 and ~3.3° on J3/J4 against
    ±3.75°: thinner, still inside. Swept: docs/16 §3.3-3.4, docs/14 §18, `tests/actuator_bench.md`,
    the `encoders.py` docstring.
  - ESP32-C3-WROOM-02 (datasheet v1.7): GPIO8 must be high and GPIO9 low for download boot, GPIO2 should be pulled up;
    28 mA typical with Wi-Fi off at 160 MHz, no maximum published.
  - LMR16006Y-Q1 (SNVSAC1): 4-40 V operating, 65 V absolute; fixed 5 V is 4.85-5.15 V; 1785-2415 kHz; non-synchronous
    (needs a catch diode); 80 ns typical minimum on-time, so at 24-28.8 V it runs close to that limit.
- **Decided for Josh's review (not final):** 24 V → 5 V → 3.3 V so USB alone powers the board on the desk; a
  separate 24 V tap so motor current never crosses board C; USB-C for flashing; the WROOM-02 because it is the only C3
  module in KiCad's library here.
- **Unverified:** the +5 V rail, inductor and buck heat in Kairo (the symbol limit above); buck input capacitance under
  DC bias; the AP2112's junction temperature and load (pull-up currents not simulated, ESP32 current typical only) and
  its ESR window; R5's power; the SMAJ36A's clamp voltage (datasheet download refused); pulse skipping near the
  minimum on-time; footprints against real parts; anything on hardware. `trace_net` was not run pin by pin. Open for
  Josh: order 1 or 2, connectors, programming route, module size, joint-ID scheme, TVS/diode (list in
  `hardware/README.md`).

## 15 Oct (later): board C firmware

- **Asked** (by the session that drew board C): the firmware for board C as a new Arduino sketch for the ESP32-C3,
  sharing (not copying) the base controller's `src/mt6701`, sending the D34 frame on the bus every 5 ms at 500 kbit/s,
  with the joint id compiled in for now; a host compile check and a real `arduino-cli` build. No commit.
- **Done:** [`firmware/joint_encoder/`](../firmware/joint_encoder/README.md): `src/main.cpp` (SSI read, frame to TWAI
  with no wait, bus-off recovery, status LED, a serial line every 500 ms for desk bring-up), `src/config.h` (pins from
  the board C schematic, `JOINT_ID` 1-7, `CAN_BITRATE` limited to the MKS speeds; anything else refuses to build).
  `firmware/base_controller/test/host/run.sh` now also compiles it against the host stubs with three settings, so the
  CI host-test step covers it.
- **Checked:** host tests and both compile checks pass (zig c++); a bad `JOINT_ID` or bitrate fails to compile as
  intended. Real build with esp32 core 3.3.12, `esp32:esp32:esp32c3:CDCOnBoot=cdc --warnings all`: **no warnings**,
  303 KB (23%), 14.5 KB RAM. Also built with `JOINT_ID=3` through the IDE-style route below.
- **Learned:**
  - An Arduino sketch cannot `#include` code outside its folder by a relative path: the build copies the sketch to a
    temporary folder first (tried on a two-file test sketch, 15 Oct: "No such file"). So the shared folder goes in as a
    library: `arduino-cli --library firmware/base_controller/src/mt6701`, or for the IDE, directory junctions in the
    sketchbook's `libraries` folder.
  - Through a junction GCC resolves `mt6701.h`'s `#include "../slcan/slcan.h"` next to the junction, not next to the
    real folder, so the IDE route needs a second junction for `slcan` (the build fails without it, checked).
  - Bus load, by hand: ~98 bits per frame with worst-case stuffing, so ~4% of 500 kbit/s per board at 200 Hz, ~12% for
    J2-J4.
- **Decided for Josh's review:** LED on = fine, 1 Hz blink = sensor problem, 5 Hz = CAN problem; frames are dropped,
  never waited for, when nothing ACKs; received frames are ignored for now.
- **Unverified:** everything on hardware (flashing over the C3's USB Serial/JTAG, the LED's polarity, which is assumed
  active-high, a real MT6701, a real bus); the IDE route in the IDE itself; a core 2.0.x build (CI builds only the base
  controller). The SSI bit-bang is the one piece repeated from the base controller's `main.cpp` (it is glue there, not
  in `src/mt6701`).

## 15 Oct (later still): the SSI read shared, board C in CI

- **Asked** (Josh, "do this task here"): the follow-up left by the board C firmware entry above.
- **Done:** the bit-banged SSI read is now `mt6701::read_word` in `firmware/base_controller/src/mt6701/mt6701.h`, a
  template over the pin operations, so it runs against a simulated sensor on the PC; `mt6701_arduino.h` holds the
  Arduino pins (`ArduinoSsi`). Both sketches use it, so no firmware code is repeated any more. A host test checks the
  bit order, the 24 rising edges and the idle levels after a read. The CI `firmware` job now also builds
  `firmware/joint_encoder` for the ESP32-C3 on cores 2.0.17 and 3.3.12, warnings failing the job as for the S3.
- **Checked:** host tests pass; moving the sample to after the rising edge makes the new test fail (checked on a copy).
  Real builds with core 3.3.12, all warnings: base controller (S3) 307 KB, also with `BENCH_ENCODER_JOINT=2`, and
  board C (C3) 303 KB, all with **no warnings**.
- **Learned:** nothing new about the hardware; the simulated sensor encodes the datasheet's timing (first bit at CSN
  low, the next on each rising edge), so the test is only as right as that reading of the datasheet.
- **Unverified:** the CI run itself (nothing pushed; the 2.0.17 C3 build has never run), and everything on hardware.

## 15 Oct (evening): feeling a touch through the gearbox (physics sim only)

- **Asked** (Josh, through another session): why the arm can't feel contact from the motors, and then a prototype that
  estimates contact from the output-encoder vs motor-encoder difference, in the MuJoCo sim only: external torque on
  J2-J4, backlash and 14-bit handling, a threshold + debounce detector that stops the arm, the smallest force it
  detects, false alarms during moves, latency at 50-100 Hz. No commit.
- **Why not the motors** (the analysis that started it): through the 48:1 printed cycloid a 10 N push at the tool is
  ~0.07 N·m at the motor, the size of the gearbox friction (~0.08) and the detent (0.02-0.07), and the MKS CAN protocol
  has no current read. The gearbox's wind-up is the better signal: with the placeholder 300 N·m/rad, 10 N twists J4
  by ~0.63°, about 0.35 N per MT6701 step.
- **Done:** `deskarm.control.ContactGuard` (a backend wrapper like `DualLoop`; docs/14 §20, new, with a figure, the
  derivation and a runnable example; old §20/21 are now §21/22). Gear torque from the encoder difference with the gap
  taken out, a **momentum observer** for the external torque (gain 10 /s), threshold $k\delta$ + 0.15 N·m (0.38 N·m on
  J2, 0.265 on J3/J4), 0.03 s debounce, then stop and give way along the push (5 /s, at most 1°) until `enable()`;
  `tare()` fits an untold payload. `physics.py`: `MujocoBackend.tool_force` (a vector at the TCP, or `physics.wall`),
  `tcp()`, and `streamed=True` (the default is unchanged). `tests/test_contact.py`, 9 tests. `tools/math_figures.py`:
  `contact.svg`.
- **Results (sim, 1 kg told, 14-bit readings one poll old):** smallest push seen 0.75-2 N where it has a lever on
  J2-J4 (0.75 N stretched straight down; not seen at all along the stretched arm or sideways, which only J1 feels);
  a step push trips after 0.20 s at 1 N, 0.10 s at 2 N, 0.04-0.08 s from 5 N; lowered onto a wall it trips 0.04-0.18 s
  after touching, peak 4-5 N at 20 mm/s, 7-14 N at 50 mm/s, up to 37 N at 100 mm/s into a stiff wall, then gives way
  to 0-1.5 N; 0 false trips in 40 random full-speed moves at 50 Hz (worst 0.85 of the threshold) and at 100 Hz (1.17,
  shorter than the debounce), 0 in 20 empty.
- **Learned:**
  - The static estimate (gravity − spring − gear torque) is exact at rest but reads 1.0-2.3 N·m during a full-speed
    move: the link's own acceleration. Differentiating the encoder twice is ~1 rad/s² of noise. The momentum observer
    needs only one difference and brings it to ≤ 0.23 N·m (De Luca & Mattone 2005; Haddadin et al. 2017).
  - **The sim's drivers chase each new target at full speed and then wait**, so at a 20 ms poll the motor moves
    stop-go at the poll rate and every poll sees the same phase of it: a speed-proportional bias (1.5-1.8 N·m on J2)
    that no observer at the poll rate can remove. With `streamed=True` (speed = distance left / step, `CanBackend`'s
    F5 rule) it goes away. With the chasing drivers every one of 20 moves tripped. So Q8 (do the MKS drivers follow a
    stream smoothly?) also decides whether this detector can work. Earlier sim results (§19's DualLoop) ran with the
    chasing drivers; their at-rest numbers shouldn't depend on it (no speed, no bias), but they were not re-run.
  - 100 Hz is noisier than 50 Hz for this: the speed is one encoder step over a shorter time.
  - **Model error is the real limit.** Standing still, 10% off in the stiffness or the masses, or 0.1° in the backlash,
    is 0.8-2.6 times the threshold. Every `arm.toml` placeholder is worse than that, so on the arm the guard needs
    $k$, $b$ and the masses identified first (push test §17, sweep fit §18, a slow pose sweep), or a much larger
    margin. `tare` can't hide a mass error (it fits 0.70-0.84 kg for a 10% heavy arm, depending on the pose).
- **Decided (for Josh's review):** the guard is a separate wrapper, under `DualLoop` (it takes motor-side targets).
  After a trip it gives way instead of only holding, because the estimate lags the force and holding kept 7 N on a
  wall in the first try. Thresholds and gains are class attributes, not `arm.toml` (they are tuning, not arm data).
- **Unverified:** everything outside the sim. The sim's gearbox is the model the guard believes (exact masses, a clean
  spring with a gap, no friction), the two encoders are read at the same instant (on the arm they come from board C
  and the 31H polls at different times), `CanBackend` + `ContactGuard` has not been run even against `SimBus`, and the
  twin has no switch for it yet.

## 19 Oct 2026: the first Fusion parts and assemblies

- **Asked** (Josh): model the arm parts directly in Fusion through the Fusion MCP (`Admin Project / DeskArm_Parts`), all
  printable in PETG on the A1 mini (180 mm cube) and the CORE One L; audit the screw holes, bearing seats, edge
  distances and tolerances so nothing sits too close to an edge; build sectioned sub-assemblies instead of one large
  assembly file.
- **Done:** 14 parts saved in `Admin Project / DeskArm_Parts`:
  1. `DeskArm_J1_Base_Housing` (v3): Ø160 × 48 mm, 26 dowel flutes on PCD 80, 6810 seat with a Ø56 relief undercut,
     Ø24 NEMA17 pilot bore, 4 standoff bosses for a 3 mm air gap, 4 × M6 desk holes (PCD 142, 5.75 mm from the edge).
  2. `DeskArm_J1_Turntable_Spindle` (v2): Ø130 × 37 mm, 6810 journal Ø49.95 + Ø53.5 seal step, Ø26 wiring conduit,
     mounting pattern at X = ±47 mm with 8 counterbores underneath.
  3. `DeskArm_J1_Cycloid_Disc` (v1): 25:1, 26 pins (R 40, Rr 2.6, e 1.2), Ø32.05 6804 bore, 8 × Ø8.50 output holes on
     PCD 52.2.
  4. `DeskArm_J1_Output_Pin_Carrier` (v1): Ø70 × 12 mm, 8 × Ø6.05 dowel pockets on PCD 52.2, 4 × M4 counterbores on
     PCD 40, Ø26 conduit.
  5. `DeskArm_J2_Shoulder_Left_Cheek` (v2): 110 × 170 × 24 mm, foot at X = −47, J2 6810 seat at Y = 135, NEMA23 M5
     tension slots.
  6. `DeskArm_J2_Shoulder_Right_Cheek` (v2): 110 × 235 × 24 mm, two 608s for the counterbalance at Y = 235, Ø8.2 axle
     hole.
  7. `DeskArm_J2_Cycloid_Disc` (v2): 24:1, 25 pins (R 50, Rr 3.1, e 1.5), Ø32.05 6804 bore, 6 × Ø11.10 output holes on
     PCD 61.4.
  8. `DeskArm_J2_Eccentric_Shaft` (v1): Ø20 journals × 59 mm, two lobes 180° apart (e 1.5), Ø8 bore for the encoder
     rod, D-flat drive end.
  9. `DeskArm_Upper_Arm_Link_Half` (v4): 80 × 265 × 30 mm (190 mm axis to axis), 42 mm cavity, 6 × M3 clamping holes,
     Ø50 shoulder bore, spring lug at (36, 150).
  10. `DeskArm_Forearm_Link_Half` (v5): 55 × 220 × 27.5 mm (180 mm axis to axis), Ø30 elbow hub, axial 6806 cradle
      for J5, 6 × M3 clamping ears.
  11. `DeskArm_Wrist_Pitch_Yoke` (v1): Ø29.95 J5 journal, Ø16 harness conduit, J6 ears with two 6802 seats.
  12. `DeskArm_J7_Tool_Flange` (v1): Ø65 × 18 mm, ISO 9409-1 4 × M5 on PCD 31.5 with rear-entry insert pockets,
      3 kinematic ball seats.
  13. `DeskArm_Tool_Changer_Receiver` (v1): Ø65 × 10 mm, ISO 9409 pattern, 3 V-grooves (pairs of Ø4.05 dowels),
      4 magnet pockets, 6-pin pogo cavity.
  14. `DeskArm_Tool_Changer_Tool_Plate` (v1): Ø65 × 10 mm, 3 × Ø6 ball seats on R 23, 4 magnet pockets, 3 × M4
      counterbores on PCD 46.

  Three assemblies reported 0 interference: `DeskArm_Shoulder_Assembly` (8 bodies), `DeskArm_Forearm_Assembly`
  (4 bodies) and `DeskArm_Full_Assembly` (12 bodies, 2 mm running clearances). Renders and sections were captured and
  the parts described in [18](18_cad_mechanical_architecture.md), which lists each part's edge margins (5.75-13.3 mm)
  and wall thicknesses.
- **Learned** (as written that day; the review below found most of the design did not assemble, so read these with
  [19](19_cad_review.md)):
  - The cheek feet first landed 12 mm inward because their holes sit at Z = 12 mm in part coordinates; placing the
    cheeks at X = ±59 mm lines the holes up at ±47 with the turntable and leaves a 70 mm gap, 5 mm each side of the
    60 mm upper-arm link.
  - A flat bearing seat rubs the inner ring and the rubber seal of a 2RS bearing. A Ø53.5 spindle shoulder and a Ø56
    relief undercut keep the seat clear of it.
  - Front-entry heat-set inserts can pull out under a heavy tool; rear-entry inserts behind a 10 mm PETG shoulder put
    the load in compression instead. (Claimed then as "impossible to pull out"; it is a stronger arrangement, not an
    unbreakable one.)
  - Fusion API details: switching between direct and timeline design; `ConstructionPlane.isVisible` has no setter;
    `occurrences.addByInsert` needs a transform with determinant +1; a cut across an empty sketch offset needs
    `participantBodies` or a sketch on a face.

## 19 Oct (later): review of the Fusion models

- **Asked** (Josh): refamiliarise, open the new models through the Fusion MCP, check designs and dimensions (holes and
  bores especially) and how they combine in the assemblies, run the print scenario, and say whether PETG survives the
  temperatures, with the focus on the housings. "Be thorough as a senior reviewer."
- **Done:** full write-up in [19](19_cad_review.md), with 12 figures in `docs/img/cad_review/`. Read-only through
  Fusion's MCP server, with every file opened invisibly and closed unsaved (another session had Josh's document open):
  cylindrical faces, holes, timelines, parameters, occurrence transforms and references, interference and minimum gaps;
  J2 and J4 swept with B-rep copies; STLs ray-cast for walls, rendered in Blender, sliced with the Bambu Studio 2.04
  CLI (6 walls, 40% gyroid, PETG HF); discs re-checked with `tools/cycloid.py`; motor temperatures from a hand model.
- **Found** (19 §0): the turntable journal sits 13.5 mm above the base's 6810 seat; there is no room for the J1 reducer
  and the pin carrier's holes have no partners; the J1 ring-pin flutes are 0.53 mm deep; the J1 motor hangs below the
  base with no harness path; the 6804s can't be assembled onto the eccentric shaft; shoulder height 196 vs 130 mm,
  elbow to wrist centre 208 vs 180, wrist to flange 35 vs 70, and no J3 joint; the elbow is free only to ~±75°
  (J2 is free over ±120°); the forearm's 6806 cradle is mostly cut away and neither yoke version works; the
  counterbalance axle is perpendicular to J2; the upper arm is two open channels, not a box; the assemblies use mixed,
  unlinked versions. Right: the cheek/turntable pattern (exact), concentric J2 seats, NEMA23 slots and 92.5 mm belt
  centre distance, both disc profiles.
- **Print run:** every part prints, 16 prints, ~70 h and ~1.96 kg PETG, ~40 g of support; the base alone is 19 h
  (mostly a solid ring that carries nothing). The right cheek and both link pairs need the CORE One L (sliced on the H2D
  profile as a stand-in). Print the cheeks inner face down and the upper arm split face down, not as 18 says.
- **Learned (sources in 19 §10):** PETG HDT is 68-69 °C at 0.45 MPa (Prusament PETG sheet; Bambu PETG HF listings),
  ASA ~93 °C. Whether PETG is OK is decided by the driver mode, not the material: CLOSE with HoldMa 90% at full current
  puts the NEMA17 joints at 60-100 °C, vFOC keeps every housing under ~55 °C even enclosed. PETG's expansion (~6× steel)
  loosens a Ø65 seat by ~0.075 mm at +20 K.
- **Decided:** nothing; four decisions for Josh in 19 §12 (shoulder height, where the J1 motor goes, J3 and J5,
  ASA vs PETG + metal for the motor housings).
- **Unverified:** nothing printed or fitted; slicer times are for the stated settings (CORE One L times will differ);
  temperatures are a model; strength and stiffness figures are hand estimates. Only the latest-modified of the
  duplicate assembly files were opened.

## 19 Oct (evening): CAD re-check and a jointed test assembly

- **Asked** (a session working for Josh): with the Fusion MCP server now configured, go through every part and
  assembly again, re-check the review's findings, re-run interference, gap and range-of-motion checks, and build
  sub-assemblies plus a full assembly from linked parts with joints, saving nothing without Josh's approval.
- **Done:** [19](19_cad_review.md) §13. Every part and all 14 assembly files opened invisibly and closed unsaved. A
  test model built in memory in an empty duplicate assembly file (never saved): 17 linked occurrences in seven rigid
  groups, zero pose = `arm.toml`'s, as-built revolute joints at J1, J2, J4, J5, J6 with the `arm.toml` limits;
  J2 = +90° reproduces the saved CAD pose. The build script is `tools/fusion_test_assembly.py`; the Fusion API
  lessons and where the model lives are in the project log §7.
- **Found:** no part changed since the review, so all twelve findings stand. New: the 7 mm 6804s don't fit the 6 mm
  discs and the two J2 discs touch (lobe pitch 8.5 mm vs 6); the J1 pin carrier's pins stand on the face that bolts
  to the turntable; J6 clears only ±23° with the flange where it is (no J6 output part); the eccentric shaft has no
  bearing to run in; the CAD's J2/J4/J6 run along X, the skeleton's along Y; the yoke "v1" file is newer than "v3";
  one `DeskArm_Shoulder_Assembly` file is empty. Ranges: J1 free, J2 ±139°, J4 ±74°, J5 free, J6 ±23°.
- **Learned:** in Fusion, a joint on an occurrence whose children aren't held together moves only one child: each
  sub-assembly needs a rigid group. `JointGeometry.createByPoint` with a custom axis entity silently gives a Z axis;
  `createByCurve` on a sketch line with `ZAxisJointDirection` puts the axis along the line. `Documents.add` can't open
  an invisible document ("not yet implemented"), and a linked insert needs a document in the same hub project.
- **Decided:** nothing. Saving the sub-assemblies and full assembly waits for Josh; so do the yoke choice and the
  four §12 decisions.
- **Unverified:** nothing printed or fitted; interference volumes depend on the placements chosen for parts that don't
  mate (19 §13.4); J1 swept only at four angles; the yoke v3 file not tried in the model.

## 19 Oct (night): the jointed assemblies saved to the hub

- **Asked** (Josh): "save", then finish what was started before Fusion crashed; the new designs and fixes are in the project log.
- **Done:** six new files in `DeskArm_Parts` ([19](19_cad_review.md) §13.8): `DeskArm_SA1_J1_Base_Turntable` (J1),
  `DeskArm_SA2_J2_Shoulder`, `DeskArm_SA3_Upper_Arm`, `DeskArm_SA4_Forearm_Elbow`, `DeskArm_SA5_Wrist_Tool` (J6) and
  `DeskArm_Arm_Assembly` (links the five; J2, J4, J5). All parts linked at their latest versions, rigid groups in
  every group, yoke = the "v1" file. Checked on the saved full assembly: every joint drives (J1 and J6 from inside
  their linked files too), J2 = +90° gives the saved CAD pose, and the zero pose has the same two J1 clashes. No part
  and no existing file was edited; the template file is still v1 and empty.
- **Learned:** `saveAs` on an opened file writes a new file and leaves the original untouched. Inner joints of a
  linked sub-assembly can be driven from the parent with `createForAssemblyContext`. Fusion crashed once on a script
  that built the full assembly and drove those joints in one call; afterwards its MCP add-in came up on port 58846
  instead of 55337, so the configured server failed and the rest ran over raw HTTP.
- **Decided:** the six-file layout and the "v1" yoke (Josh's "save" to that proposal).
- **Unverified:** the §13.5 ranges were measured in the in-memory model, not re-swept in the saved files; the
  duplicate hub files are still there (Josh's to delete).

## 20 Oct: the joint ranges re-swept in the saved assemblies

- **Asked** (Josh): continue where the last session left off, with the Fusion MCP back on (port 58846).
- **Done:** nothing in the hub had changed since the 19 Oct save, so the four §12 decisions are still open. Ran the
  checks [19](19_cad_review.md) §13.7 listed as not done, on the saved `DeskArm_Arm_Assembly` (opened invisibly,
  closed unsaved): J1 in 10° steps, the J2/J4/J5/J6 boundaries, and the yoke "v3" file swapped into `SA5` in memory.
  Results in 19 §13.9: J2 ±139°, J6 ±23°, J5 and J1 free as before; **J4 ±73°**, one degree less; v3 clears the same
  ±23°. New tool `tools/fusion_sweep.py` runs these sweeps over raw HTTP.
- **Learned:** the saved joints carry the `arm.toml` limits, and Fusion clamps a value past a limit silently: the
  first run reported J2 clear to ±160° while it never left 120°. Caught because the measured gap stopped changing
  after 120°; the tool lifts the limits in memory now. The add-in runs a script with `__name__ == "__main__"`.
  At J6 = 120° the flange and tool changer also hit the forearm, so the missing J6 output part must clear it too.
- **Decided:** nothing. Keep the yoke v1 (no reason found to prefer v3).
- **Unverified:** v3 against the forearm; sweeps of two joints at once; no slices or prints. No `arm.toml` number
  changed, so the budget and tests were not re-run.

## 20 Oct (later): the four CAD decisions, and J1 rebuilt

- **Asked** (Josh): "you make the best decision based on our mathematical constraints and parts for the adjustments".
- **Decided** (Josh; [09](09_decisions_open_questions.md) D36-D39): keep the shoulder stack (209 mm);
  J1 motor coaxial, hanging through the base board on ~70 mm feet, harness looped on the base top; J3 into the upper
  arm and J5 at the elbow end, forearm 180 mm and wrist 70 mm (the CAD changes, not `arm.toml`); PETG with metal motor
  plates and SR_vFOC, ASA only for J7.
- **Done:** `torque_budget.py` showed shoulder height changes no torque or tipping number, while the CAD's 208 mm
  forearm would cost J3/J4 1.50 → 1.35. `arm.toml` J1 `d` 0.130 → 0.209 with the masses moved; the spring re-sizes to
  574 N/m; docs/14 §2, §3, §4, §6, §10, §13 and §19 re-worked with the new numbers (the examples run). In Fusion,
  `tools/fusion_j1.py` rebuilt the J1 files (base v4, new motor plate and cam, disc v2 ×2, carrier v2, turntable v4
  with an inner spacer), `tools/fusion_assembly.py` rebuilt `SA1` (v2) and moved `DeskArm_Arm_Assembly` (v2) to it with
  SA2-SA5 13 mm higher. Checks: the stack in memory with the bought parts at three cam angles is clean; the saved
  assembly's zero pose is clear for the first time; J1 ±170°, J2 ±139°, J4 ±73°, J6 ±23° ([19](19_cad_review.md) §14).
- **Learned:** docs/03 already asked for two spaced output bearings; one thin 6810 can't carry the arm's tipping
  moment. With two, the lower one can only go in from below, so the floor has to come off (the motor plate). The
  disc's lobes reach R 39.79 from the axis (r_max + e, not r_max), so ring pins at R40 can't be held by the cavity
  wall: hold them at both ends. The disc turns −t/25 (checked numerically: +t/25 clashes by 2.3 mm). Fusion: scripts
  over ~16 KB are dropped silently; imported modules persist between runs; a fitted spline needs points evenly spaced
  in arc length (0.1 mm error at the seam otherwise).
- **Unverified:** nothing printed; the LDO shaft length (24 mm) and flat are assumed; disc motion checked at three
  angles, not swept; bearing preload depends on printed lengths; masses are still estimates. The other §12 fixes
  (J2 shaft and discs, cheeks, upper arm with J3, forearm, yoke, J6, J7) are not started.

## 20 Oct (final): the motor shaft, and one last full check

- **Asked** (Josh): "check the motor shaft. Do the full check and verification one last time. Go with the option you
  think is best".
- **Found:** LDO's drawing for the LDO-42STH60-2004AC (Rev A): shaft Ø5 × 24 ± 0.5 with the flat on the last 15 mm
  only, pilot Ø22 × 2, M3 threads 4.5 mm deep, 2.0 Ω, 3.3 mH. The cam's full-length D key would not have gone onto the
  round part of the shaft.
- **Decided and done:** cam v2 keys only z 12-22.5 (inside the flat under either reading of the 24 mm), set screw for
  axial hold; M3 × 6 motor screws; `arm.toml` J1/J3/J4 take the drawing's 2.0 Ω and 3.3 mH (J3/J4 at full speed
  1.28 → 1.54; standstill unchanged). `SA1` and the arm assembly moved to cam v2 (`fusion_assembly.py refresh-*`).
  The check now models the real D shaft turning with the cam, so the old "artefact" is gone.
- **Verified:** everything listed in [19](19_cad_review.md) §14.6: tests, firmware (host and real builds), budget,
  spring, links, the J1 stack at five cam angles, the saved assembly at every joint limit and four combined poses.
- **Unverified:** nothing printed or measured; the shaft and R/L are the maker's drawing, not a caliper or meter.

## 20 Oct (last): does the J1 assembly mate?

- **Asked** (Josh): "double check the assembly, are they mating and does the logic make sense, holes align".
- **Done:** a mating mode in `tools/fusion_j1_check.py` (bearings split into inner and outer rings, 36 measured gaps)
  and a hole-axis probe of the saved assembly. Holes all line up. Four faults fixed ([19](19_cad_review.md) §14.7): the
  one-piece cam could not be assembled (two pieces now), the ring stack had no preload gap (stem 0.2 mm short), the cam
  shoulders were not centred on their rings, the spacer was at the seal limit. Saved cam v3, turntable v5, SA1 v4,
  arm v4; all re-checked.
- **Learned:** "no interference" is not "mates": measure the contacts. Fusion cuts reach other components' bodies
  unless participant bodies are set, which had silently damaged the check's own copies of the plate and cam.
  Assembly order decides what can be one piece.
- **Unverified:** nothing printed; ring diameters for the bearing envelopes are approximate (SKF da/Da limits were
  used for the decisions); J2 onward still don't mate.

## 20 Oct (night): the whole arm in Fusion, swept and probed

- **Asked** (Josh): "ensure that the arm isn't copied too closely from any other person's design … make all the 3D
  parts, make the subassemblies, then make the full big assembly. Properly mated, properly sized, all holes align".
- **Done:**
  - Every part from J2 out is generated by `tools/fusion_arm*.py`, each in its own hub file (`DA_` prefix, 62
    files). That covers the cycloid modules C2 (J2) and C34 (J3/J4), the planetary P56 (J5/J6), the J7 module P7, the
    bought-part envelopes, four module sub-assemblies, ten links and tool parts, and the full assembly `DA_ARM`
    (118 occurrences, 16 rigid joints plus the modules' own revolute joints).
  - Every module passes an in-memory mating check: interference at several input angles plus measured bearing, pin
    and preload contacts.
  - Every joint was swept to its limits in `DA_ARM`, along with combined poses.
  - A whole-arm hole probe: 101 hole axes, 11 without a partner, all expected.
  - The originality check is written up in [20](20_cad_full_arm.md) §7.
- **Decided (D40, delegated):** the elbow is ±120° (was ±140°). J3 sits 28 mm lower and J5 40 mm lower on their
  links, J7 is turned 90°, and the counterbalance is a = 70, b = 98 mm with 1252 N/m. The full reasons are in
  [20](20_cad_full_arm.md) §4.
- **Learned:**
  - The elbow's fold limit is set by how far below the elbow the upper arm's top sits and how wide the forearm is
    near the elbow. Moving roll joints along their links is free kinematically and buys range.
  - A counterbalance lug must stay out of the next link's sweep.
  - Python module caching inside Fusion built one link from stale placement data; the CLI now reloads in dependency
    order.
  - Coincident features are easy to miss without a probe: two gripper holes sat on the coupling's groove and magnet.
- **Re-run:**
  - torque budget: J2 1.53, J3/J4 1.54, J5/J6 1.29, J7 1.78;
  - `design_options.py`: docs/15 table refreshed;
  - spring: 1252 N/m;
  - tests: 189 passed, 3 skipped. Two tests and a docs/14 example were re-tuned because the J3 and J5 lumps moved:
    - the physics stall test payload went from 2.4 to 2.6 kg;
    - the contact test now accepts J2 or J4 tripping first;
    - the §19 numbers were recomputed.
- **Unverified:**
  - nothing printed;
  - no FEA on the plate links;
  - cables not modelled;
  - the CAD-to-`arm.toml` angle sign mapping is not pinned down;
  - masses are still estimates (only the J3 and J5 lumps were moved);
  - the 3.7 / 3.82 Hz sim modes and the motors-only sim errors in docs/14 are from the earlier masses.

## 20 Oct (night, later): stale files deleted, masses from the CAD

- **Asked** (Josh): "Delete the stale things, yes do that" (the duplicate sub-assembly files, and reading the CAD
  masses into `arm.toml`).
- **Done:**
  - Deleted `DA_SA_P56_dup_old` and `DA_SA_P7_dup_old`, after checking nothing referenced them. Fusion keeps deleted
    files in the project's Deleted items. `tools/fusion_run.py stale [--delete]` does this.
  - Then (Josh: "delete") deleted the 26 superseded 19 Oct `DeskArm_*` files too, assemblies first
    (`old --delete`). The J1 files that `DA_ARM` still uses are kept.
  - One deleted file, an empty `DeskArm_Shoulder_Assembly` v1, was the template the tools opened for new files and
    for in-memory checks. They now start from a new document (`fusion_lib.scratch_doc`). `check C34 0 mates` passes
    again (44 checks).
  - Checked the CAD against the kinematics: driving J1-J6 in `DA_ARM` puts the tool plate within 0.01 mm of the FK,
    so CAD angles are `arm.toml` angles.
  - Read every body's volume and centre (`sweep bodies`); `tools/cad_masses.py` builds the 34 `arm.toml` lumps
    ([20](20_cad_full_arm.md) §9).
  - Re-sized the spring to 1476 N/m.
  - The torque budget now refines its worst poses between the ±90° grid points.
  - docs/14, 15, 01, 05, the BOM and the project log are updated.
- **Learned:**
  - Before deleting, check what your own tools reference, not only what other files reference: the template had no
    parent references but the scripts depended on it.
  - The hand estimates were light: 7.98 → 9.47 kg, 4.39 → 5.33 kg past the shoulder. The printed housings and the
    steel pins (166 g in the J2 module alone) are most of the difference.
  - The pitch actuators sit to one side of the centreline, so the roll joints J3 and J5 carry gravity torque, and the
    arm is no longer symmetric. A ±90° pose grid then misses the worst case: a 3000-pose random search found up to
    4% more, and the budget's own test caught it.
- **Re-run:**
  - torque budget: J2 1.32, J3 1.42, J4 1.48, J5 1.20, J6 1.26, J7 1.76, all pass;
  - tipping: 13.2 kg needed against 13.2 kg, so the clamps are now required;
  - tests: 189 passed, 3 skipped (~75 s);
  - five tests were changed because the CAD's lumps are off the centreline:
    - the budget agreement test;
    - the stretched-arm hand check, which now also checks J3/J5;
    - the J3 feedforward test, since J3 is now loaded;
    - `design_options.py`, for the new lump names;
    - the docs/14 §19 example.
- **Unverified:**
  - The PETG density (0.80 effective) and the hardware allowances are guesses until a module is weighed.
  - The sim-derived numbers in docs/14 (first mode, motors-only error) were not re-run.

## 20 Oct (night, last): checks on the full CAD, in the order Josh set

- **Asked** (Josh): "go ahead in that order ... we might not be able to do the spring, but that should be fine? Run
  all the analysis ... conserve tokens ... put all this into docs so another agent can review with low token usage
  ... Run the tests".
- **Done** (each check has its command and expected output in [21](21_checks_and_review.md)):
  1. **Spring** deferred (D41). The CAD has no room for it and its lug and tower fail as drawn. The budget never
     relied on it.
  2. **BOM from the CAD's holes** (`tools/cad_bom.py`): ~100 Ø5 pins, not ~60; Ø6 × 25 and Ø4 × 8 to buy; insert
     and screw counts.
  3. **All 45 printed parts sliced** (`tools/slice_parts.py`): 2.8 kg PETG, 122 h. The stand and tower need the
     big printer.
  4. **Masses** now use the slicer's grams: 9.83 kg, 5.60 kg past the shoulder.
  5. **Self-collision capsules** extended with the CAD's off-centre parts and a desk check. Validated against
     Fusion on 112 poses: 0 missed.
  6. **Link strength screened** (`tools/link_strength.py`). Link 2's side plate was widened and link 3's arm plate
     thickened; all ≤ 13 MPa.
  7. **Sim numbers re-measured** (`tools/sim_modes.py`): 3.65 Hz, ζ 0.059.
  8. **Base board and the J1 motor** added to `DA_ARM`. The motor's shaft flat needed a y-flip, not an x-flip.
- **Learned:**
  - The Bambu CLI crashes on inheriting profiles: flatten them, keep their names, set the plate type.
  - The Fusion add-in can stop returning `print` output. The runner now reads results from a file.
  - A capsule model has to include off-centre parts, and the desk, or it misses real hits. Score it against the CAD.
- **Re-run:**
  - torque budget: J2 1.27, J3 1.38, J4 1.44, J5 1.19, J6 1.25, J7 1.74, all pass;
  - tipping: needs the clamps (13.7 kg needed, 13.5 kg);
  - tests: 189 passed, 3 skipped;
  - firmware host tests: pass.
- **Unverified:** nothing printed or weighed. The slicer grams, the PETG strength allowable and the
  capsule-against-CAD comparison all stand in for measurements.

## 20 Oct (night, final): sensing, a simple gripper, routing, electronics spacing

- **Asked** (Josh): "do those, gripper (simple one) and routing. We don't need to place the electronics in the arm do
  we? we just [need] that we've designed for them and have the right spacing".
- **Done** (D42; [21](21_checks_and_review.md) §11-14):
  - **Sensing:** hall homing pockets on J1-J7; output encoders on J2-J4 (a Ø6 rod through the already-hollow input
    shaft to an MR126 and board C in a new encoder cap).
  - **Gripper:** a parallel gripper, an N20 lead-screw motor with one fixed and one moving finger, 0.115 kg.
  - **Routing:** one 4-wire bus with a service loop round each joint, tie slots on every link, a length table.
  - **Electronics:** they aren't modelled, but `KEEP_OUT` boxes reserve the base board's back strip (PSU, boards,
    Pi) and an E-stop corner, so the planner keeps the arm off them.
  - **J7 flange** Ø50 → Ø60 for its magnet.
  - **J1:** base and turntable re-saved through `fusion_j1.py`, and SA1 refreshed (v5).
- **Checked:**
  - Every module mates in memory with the encoder envelopes.
  - `DA_ARM` (129 occurrences): the zero pose is clear and every joint limit is clear.
  - Hole probe: 122 axes. The only new unpartnered ones are the N20's M1.6 screws, because its envelope has no holes.
  - Capsules re-scored against Fusion with the gripper and caps: 0 missed in 112 poses (26 hits caught, 7 false alarms).
  - pytest 189 passed, 3 skipped; firmware host tests pass.
- **Re-run:**
  - torque budget: J2 1.35, J3 1.52, J4 1.60, J5 1.35, J6 1.44, J7 2.07, all pass (the light gripper helps);
  - tipping: just enough;
  - spring: 1499 N/m (deferred);
  - sim: 3.87 Hz;
  - links: ≤ 11.8 MPa;
  - slicing: 49 parts, 2.9 kg, 126 h.
- **Learned:** keep hall switches and encoders on the same side of the joint as their driver, so no signal wire
  crosses a joint and the harness is only the bus.
- **Unverified:**
  - grip force;
  - service-loop clearance in motion (the loops aren't modelled);
  - the slip ring's seat;
  - board C's real size.

## 20 Oct (late night): the 3-prong gripper, clean-up, electronics spaces drawn

- **Asked** (Josh): "clean it up. Ensure no clipping, proper hole alignment, placement spaces make sense, delete
  anything that is redundant to this point. I didn't see/couldn't find the hand, additionally I thought it was going
  to be 3 'fingered' for the basic one?"
- **Done:**
  - **Gripper:** redrawn as the docs/06 §2 3-prong design (option A, [21](21_checks_and_review.md) §12):
    - three fingers at 120°, each on a pivot pin;
    - a pin-in-slot drive from a carrier on an N20's M3 lead screw, with a 693ZZ taking the thrust;
    - aperture 6-80 mm;
    - a 40 × 36 mm tool-board space;
    - parts `DA_G_Body`, `DA_G_Fingers`, `DA_G_Carrier`, `DA_P_Gripper`.
  - **Fusion check:** `gripcheck` (in memory) is clear at 6, 15, 40, 60 and 80 mm.
  - **Knock-ons of the longer tool:**
    - `arm.toml` `tcp_offset_z` 0.08 → 0.11 (tips at 0.123);
    - the tool capsule's r 35 → 50 mm;
    - the desk now sees the tool as a flat-ended cylinder that stops at the fingertips, so a tool-down pick isn't
      held 50 mm off the board by the capsule's round end.
  - **Electronics spaces:** drawn as boxes (`DA_B_Spaces`: PSU, boards A/B, Pi, CANable, E-stop) so the sweeps see
    them ([21](21_checks_and_review.md) §14).
  - **Tool changer:** the receiver's ball seats are deepened to 5.24 mm. A 6 mm ball meets the 4 mm grooves' edges
    2.24 mm above its centre, so the 3 mm seats would have held the tool plate 2.2 mm off the receiver.
  - **Deleted:**
    - `tools/fusion_test_assembly.py` (obsolete since 19 Oct);
    - `tools/.rebuild_log.txt`;
    - the old jaw's STL and slice entry;
    - the stale collision cache;
    - `fusion_assembly.py`'s mode for the deleted `DeskArm_Arm_Assembly`.
  - **Not deleted:** Fusion refused to delete `DA_G_Jaw`, though no file references it now. Josh can delete it by
    hand.
- **Checked:**
  - `DA_ARM` (131 occurrences) is clear in the zero pose.
  - Single-joint limits are clear, except J2 −120°: link 5 reaches the PSU's space, which `KEEP_OUT` blocks.
  - Hole probe: 129 axes, 5 unpartnered (spring anchor, clevis, 3 ball seats).
  - Capsules against Fusion: 0 missed in 112 poses (29 hits caught, 7 false alarms).
- **Re-run:**
  - masses 9.74 kg, gripper 0.117 kg;
  - budget: all pass, J2 1.33, J3 1.45, J4 1.52, J5 1.18, J6 1.24, J7 2.07 (the payload is 30 mm further out);
  - tipping 13.1 kg needed, 13.4 available;
  - sim 3.66 Hz;
  - docs/14 §10, §12, §13, §19, §20 numbers;
  - pytest.
- **Learned:**
  - Re-read a plan before replacing a placeholder. docs/06 §2 specified three fingers, and the first drawing ignored
    it.
  - A capsule's round end is the wrong shape at a desk, because the tool's working end is flat.
  - Don't run two Fusion jobs at once. A hole dump run during the collision sweep read a posed arm.
- **Unverified:**
  - grip force, the 693ZZ's thrust and the finger strength (computed only);
  - the contact-detection table in docs/14 §20 (run with the 0.08 TCP);
  - the tool-board PCB.

## 21 Oct: finishing the CAD

- **Asked** (Josh): "can you fix up and check over whatever it is you need to? kind of just want to finish it".
- **Done:**
  - **P56 check:** fixed its false flag. Carrier 1 and sun 2 are one body, so the measured "axial gap" was sun 2's
    mesh (0.037 mm). The 1 mm gap is now checked from the stage heights. P56 has 28 `ok`.
  - **Board C:** its space on the encoder caps is 44 × 34 × 6 mm, with M2 standoffs at (±19, ±13). The 25 × 25
    space could not hold the schematic's ESP32-C3-WROOM-02, buck, USB-C and two CAN connectors.
  - **J7 slip ring** ([21](21_checks_and_review.md) §11):
    - its body is in a new arm on link 6, under the 80T pulley, with a lip and an M3 grub screw;
    - `DA_P7_Sliptube` turns the rotor with the output, glued in the flange bore, and carries its wires up to the
      pogo pins;
    - P7 has 32 `ok`.
  - **Gripper:** 2 mm TPU pads (`DA_G_Pads`). The aperture (6-80 mm) is now measured between the pads.
- **Checked:**
  - all module checks;
  - `DA_ARM` (134 occurrences) is clear in the zero pose and at every single-joint limit, plus J7 180° and
    J6 60° / J7 90°. J2 −120° reaches the PSU space, which `KEEP_OUT` blocks;
  - hole probe: 131 axes, 5 expected unpartnered;
  - gripper clear at 6, 15, 40, 60 and 80 mm;
  - capsules against Fusion: 0 missed in 112 poses (29 hits caught, 7 false alarms).
- **Re-run:**
  - masses 9.79 kg (5.54 past the shoulder);
  - budget: all pass (J2 1.32, J3 1.44, J4 1.51, J5 1.18, J6 1.23, J7 2.06);
  - tipping 13.2 kg needed / 13.5;
  - sim 3.64 Hz;
  - slicing: 52 files, ~2.9 kg, ~126 h for the arm;
  - docs/14 §10, §12, §13, §19 numbers;
  - pytest 189 passed. The held-push contact test now settles for 6 s instead of 4: the heavier wrist rings
    longer.
- **Learned:** a minimum-distance check between two parts can only test one gap per pair of bodies. When one body
  holds two features (a carrier plate and a sun gear), check the second gap from the numbers.
- **Not drawn, on purpose:** screws as bodies, the harness, the spring (D41).
- **Unverified:** everything physical. Next: the coupons and one J3/J4 module, then the PCB layouts to the drawn
  spaces.

## 21 Oct (evening): a print-only assembly, a display assembly, and the CAD in the twin

- **Asked** (Josh): "can you make two versions, one that is only the parts that are going to be the fully 3d printed
  assembly, and the other which is the full thing just so it can be visually seen and move around (kind of just a fun
  display). We could also then import the model into the simulator, no?"
- **Done** ([20](20_cad_full_arm.md) §10):
  - `DA_ARM_Print` (87 printed parts) and `DA_ARM_Display` (all 113 parts), new hub files built from `DA_ARM` by
    `tools/fusion_arm_display.py`. They are coloured by material, have one rigid group per body and top-level joints
    J1-J7 with the `arm.toml` limits, so they can be dragged or driven in Fusion;
  - `tools/cad_meshes.py` turns the display file into 30 per-link STLs (`software/deskarm/data/meshes`, 14 MB);
  - `to_urdf(meshes=...)`. The twin draws the CAD arm by default (`--primitive` for the old cylinders). MuJoCo is
    unchanged: it already has the CAD masses.
- **Checked:**
  - both files clear at zero;
  - tool plate within 0.01 mm of `DA_ARM`'s at two poses;
  - every part moves with its body (4 × 10⁻¹³ mm);
  - each body moves as the twin's link to 0.001 mm at two poses, one near the limits;
  - the twin runs, and moves the CAD arm with no loose parts (screenshots);
  - new test `test_urdf_with_cad_meshes`.
- **Learned** (Fusion API, by testing; not in its documentation as far as I found):
  - linked parts nested in empty components did not follow joint motion when placed off-identity, even after
    Capture Position;
  - `addExistingComponent` composed the given transform with the first occurrence's;
  - STL export of an occurrence ignores its place at the top level, and misplaces it once nested joints have moved.

  The first build had all three problems. Its renders showed the arm in pieces, and the first mesh export put the
  module parts at the origin. Each was found by a check, not by eye, except the first, which a render showed after the
  numeric checks had passed: they read the joint transforms, not the parts.
- **Decided:** top-level parts in rigid groups (the layout `DA_ARM` already used), and pose checks compare transforms
  instead of exported meshes.
- **Unverified:** how Fusion's UI drag feels on 113 linked parts. Nothing printed.

## 23 Oct: the contact guard in the twin and on `CanBackend`

- **Asked** (Josh): "do the software things", after the next-steps list that named the guard's twin switch and its
  `CanBackend` + `SimBus` run.
- **Done:**
  - `python -m deskarm.twin --backend physics --contact-guard` wraps the sim in `ContactGuard` (it also turns on
    `streamed=True`, or every move false-trips, docs/14 §20). It composes with `--dual-loop`. Not looked at in the
    browser yet.
  - `tests/test_contact_can.py`: the guard on `CanBackend` with output encoders on J2-J4 against `SimBus`. A still
    arm with the right payload does not trip; a 3 N push at the tool does; `enable()` resets it.
  - `ContactGuard.settle()` (new, see below).
- **Learned:** `CanBackend` starts its motor angle at the output encoder's, so a loaded arm's gear wind-up
  (~0.5° on J2-J4) is hidden in the motor offset. The guard then reads gravity as a push (about 2 N·m on J2) and trips
  within 0.04 s of enabling. This could not show in the physics sim, where the motor angle is true. `settle()` puts
  the predicted wind-up back through `CanBackend.set_angle` (call it once, at rest, after `enable()`).
- **Decided:** a guard method, not a change to `CanBackend`'s start-up, which does not know the payload.
- **Unverified:** `SimBus` has no arm dynamics (the test sets the wind-up by hand), so this checks the plumbing, not the
  observer in motion. `settle()` assumes the told payload and `arm.toml`'s `k` and backlash are right: on the arm they
  are placeholders, so the first bench job is the same identification as §20 says. Not run on hardware.

## 25 Oct: print readiness, area by area

- **Asked** (Josh): break the full assembly into its important parts and judge each area for fit, sense, 3D form,
  printability, manufacturability, rounded corners and engineering practice, "then I can start printing them off". Then:
  "you've checked that the parts fit ye?"
- **Done** ([22](22_print_readiness.md)):
  - `tools/mesh_audit.py` (new) on all 52 `stl/da` STLs and the six J1 hub parts (exported read-only from Fusion to a
    scratch folder; they were never in `stl/da`). Every mesh is a closed solid. The J1 cam file is its two pieces.
  - Fit re-run in Fusion: J1, C2, C34, P56 and P7 mating checks, the gripper, the ±120° sweeps and the hole probe.
    Every result is the same as on 21 Oct (22 §1a).
- **Learned:**
  - There are **no fillets or chamfers anywhere in the model**: no builder makes one. The links, stand, tower, gripper
    and housing outsides carry 1-4 m of sharp convex edge each.
  - **The tolerance coupons have never been printed**, so every seat and pocket size is still a guess about these
    printers.
  - The fit checks are against nominal CAD sizes. They say the parts fit each other as drawn, not in real plastic.
- **Decided:** nothing changed in the CAD. The recommended order is coupons, then one C34 module, then a fillet pass
  before the links (22 §5-6).
- **Unverified:** wall thickness is a ray screening, not the slicer's. The J1 hub was never sliced. No strength work
  beyond 21 §6.

## 26 Oct: fillet pass finished, five faults in it found and fixed

- **Asked** (Josh): "Refamiliarise yourself with where the project is and what was being done last time. Continue
  from there, fusion is open." Then, after Fusion went offline twice and was restarted: "continue".
- **Done:**
  - Ran the rest of the 25 Oct list: zero-pose sweep, `display leaves`/`print`/`full`, the twin meshes, `sweep stl`,
    and the six J1 hub STLs to a new `stl/j1/` folder.
  - `mesh_audit` showed the J1 base at 501.6 cm³ against 533.7 before the fillets, a 6% loss. A boolean difference of
    the body before and after its R4 fillet showed eight 4.1 cm³ lumps, z 0-45: **the fillet had swallowed all
    eight shell ribs**, and Fusion called the feature healthy.
  - Fixed `fusion_lib.soften()` with two guards, then re-saved the six parts they change and re-ran everything after
    them: checks, sub-assemblies, DA_ARM, display assemblies, meshes, STLs, sweeps, hole probe, slicing, masses,
    torque budget, tests.
  - The hole probe now skips fillet faces: 131 axes, the 5 expected unpartnered (25 Oct: 162 and 36).
- **Learned:**
  - The face width is taken from all of a face's vertices. A big face that wraps around thin ribs looks wide even
    where it is only a few mm across, and R4 on both sides of a thin rib removes the rib. A healthy feature is not a
    correct one: compare the volume change with what a fillet can remove, (1 − π/4) r² per unit length.
  - The rounding also took edges on small holes, where faces mate: 50 on each cycloid housing, 6 on the fingers.
  - `fusion_j1.py` reports the volume before `soften()` runs, which is why the 25 Oct run looked fine. `soften()`
    now reports its own net change.
  - After Fusion refuses a fillet inside an assembly, `comp.parentDesign` raises that error again, so it is read
    once at the start.
  - A hole that breaks out of an edge (the link 2 cable clevis) has arcs under 180°, so the probe's first filter
    (180°) lost it. Fillets between square faces are 90°, and 100° separates the two.
  - Fusion went offline twice. Once it came back reporting "online" while `findFileById` and `dataProjects.count`
    still failed, until a restart. A client timeout doesn't cancel a script already sent to Fusion.
- **Second review, same day.** Josh: "can you double check any other issues it might have caused or violated design
  principles? Additionally, it needs to be somewhat printable."
  - **Done:** listed every saved fillet by the two named features it joins (read from each part's timeline), and
    audited the STLs from before any edge treatment (git HEAD) against the fixed ones, in each part's print
    orientation.
  - **Found** three more faults in the first fix:
    - Protection by feature name failed wherever a protected face touched the bed chamfer: the chamfer gives those
      faces new ids, and the ids were read before it ran. Link 6's tab pocket, where a module tab seats, had
      R 1.5 inside fillets.
    - Rounded downward edges add overhang (the stand 235 → 471 mm² steeper than 45°).
    - The gripper carrier's bed chamfer failed and its bed rim was filleted instead (bed contact 400 → 126 mm²).
  - **Fixed:** protected faces are read after the chamfer; recess, passage, sleeve and bore are protected names; a
    failed bed chamfer leaves the rim sharp; an outside edge with a face pointing down more than 45° gets a 45°
    chamfer instead of a fillet; chamfers get the same volume guard (k = 1/2). Re-saved 11 parts (J1 base v9) and
    re-ran every check and output. Overhang is now the same or lower than with no edge treatment on 9 of 13 parts.
    Bed contact is back on the carrier.
  - `fusion_j1_check.py` now also writes its result to `tools/.fusion_out.txt`, because the slower edge treatment
    pushed it past the 130 s client timeout.
  - **Learned:** standard practice for FDM is a chamfer on edges that face down and fillets elsewhere; a 45° face
    prints, a fillet's lower half does not. Face ids in Fusion change whenever a feature touches that face, so ids
    have to be read after the last feature that could touch them. The audit's overhang count needs a threshold just
    past 45°, or it counts the 45° chamfers it should pass.

- **Decided:** guard thresholds as above (R ≤ 6 mm cylinders stay sharp; volume bound 2 k r² L, k = 1 − π/4 for fillets and 1/2 for chamfers; 45° chamfers on edges facing down).
  `arm.toml` masses refreshed from the CAD: 9.77 kg, 5.53 kg past the shoulder (was 9.79, 5.54). The docs/14 §19
  example now expects 0.850° and 1.495°.
- **Unverified:** nothing printed. Fits are still at nominal CAD size, and the coupons come first. The J1 hub was
  never sliced. `tools/sim_modes.py` was not re-run for the new masses. The P56 housing still has no edge fillets. The feature map lists neighbours along each fillet's ends too, so it
  shows features a fillet only touches at its end (the Link 2 motor notch, the stand's motor window); those were read
  as fine, not checked one by one.

## 26 Oct (evening): clean-up of every doc and script, math re-check, singularity and CCD study

- **Asked** (Josh): "go through every document and script on the project, everything ... remove any em dashes, AI
  sounding phrases, ai sounding variable variables, ai comments, and do documentation review ... Any sort of AI residual
  files", with a list of phrases to avoid; "Anti-ai the documents and plans as well, keeping the records consistent
  though, and the math sound. Double check all the math as well, and another thing to look into is singularity, and
  possibly using cycloid coordinate descent?" Earlier the same day: Josh commits and pushes himself, in his own name.
- **Done:**
  - Scanned every tracked `.md`, `.py`, `.h`, `.cpp`, `.ino`, `.toml` and `.csv` for the phrase list, em dashes,
    emoji, narrating comments and generic names, then read every doc. The em dashes and nearly all the listed phrases
    were already gone; the hits left were technical ("key", "enable", "core" as in CORE One, "efficiency").
  - Rewrote the 19 Oct CAD record (docs/18 and its docs/17 entry) in plain language, keeping every dimension; removed the
    emoji from seven docs and the twin UI; toned down stock phrases in docs/02 and 10. The local agent config folder is ignored.
  - Brought stale facts up to date in README, the project log and docs/01-16, 21, with
    dated notes wherever a recorded conclusion changed rather than silent edits.
  - Recomputed every worked number in docs/14 against the code and by hand; fixed the ones that had drifted (below).
    Redrew the figures.
  - Wrote `tools/ik_study.py`, docs/14 §21 (old §21-22 are now §22-23) and [23](23_singularities_and_ccd.md).
- **Learned:**
  - The docs/14 derivations were all right; what drifted were worked numbers that depend on data that later changed
    (the TCP offset, the masses, the motor inductance). The runnable examples caught none of it because they assert
    relationships (FK = product of DH matrices), not the printed numbers.
  - With all seven joints free, this arm is singular only at a straight elbow or with J2 and J6 both at 0
    (Kreutz-Delgado, Long and Seraji, 1992, for the general result; measured here on 300 constructed poses per case).
    The wrist and shoulder singularities need J3 locked.
  - CCD's exact per-joint step is $\operatorname{atan2}(C, B)$ for any weighted set of vectors (Wang and Chen, 1991),
    but it converges linearly where DLS converges quadratically, and this arm's closed-form 6-DOF solution already
    gives DLS good restarts.
- **Decided:** nothing in `deskarm` changed. Recommended in docs/23: keep DLS with λ = 0.02 and the closed-form
  restarts; make the twin's singularity warnings mode-aware or σ_min-based (Josh to decide).
- **Unverified:** everything is simulation. The CCD timings are on this PC, not the Pi. `CanBackend` still doesn't
  subtract the predicted wind-up from the coarse start-up reading (docs/16 §3.3).

## 26 Oct (late): first prints and the J2 bench build

- **Asked** (Josh): "most of my first round order came in, save for the motors themselves ... What can we do print
  wise from here", then "generate whatever prints I need so I can begin both testing and assembling".
- **Done:**
  - Compared the BOM with what each module needs. C2 (J2) is the only gearbox nearly complete: it lacks two 6802s, the
    screws and 8 output pins of the right length. C34 and J1 need Ø5 pins, and the wrist needs 623s and Ø3 pins (all on
    the 11.11 list).
  - Remade the coupons to the sizes the CAD uses, and added ring coupons for the 6810 and 6806 (`tools/coupons.py`).
  - Wrote `tools/print_pack.py`: a bench stand and a lever for C2, the aluminium motor plates as 1:1 DXF and SVG, and
    four print jobs (coupons, rings, C2, bench) placed and sliced into Bambu Studio projects in `stl/plates/`, 12
    plates, ~40 h and ~1.06 kg. A test checks that the stand and lever clear the module and that packed parts stay
    apart.
  - Wrote [24](24_first_prints_and_j2_bench.md): print order, J2 hardware list with screw lengths checked against the
    CAD stack, assembly order, and the hand tests that work with no motor. Updated [22](22_print_readiness.md) §6
    (25 Oct plan kept), [print_tolerances](print_tolerances.md), the bench procedure and the README.
- **Learned:**
  - The Bambu Studio CLI's `--arrange` keeps a 24 mm brim and skirt margin round each part (it says
    `brim_skirt_distance 24` in its debug log), so it rejects any part wider than about 130 mm on the 180 mm A1 mini
    bed, and `slice_parts.py` then fell back to the CORE One L profile. Placing the parts in the script and slicing
    with `--arrange 0` works: the C2 housing slices on the A1 mini in about 11.5 h.
  - The coupons date from 1 Oct (`git log`), before the CAD settled its bearings and fits, and were never
    brought up to date. A test fit only means something at the size the part actually uses.
- **Decided:** J2 is the first module to print and build. C34 waits for its parts.
- **Attribution** (Josh, same evening): D36-D42 in [09](09_decisions_open_questions.md) and the 14 Oct 5 V choice were
  recorded as delegated to the coding agent. Josh made them, so they now read "Decided by Josh". Tool references in
  the docs now say "the agent" instead of a product name.
- **Unverified:** everything. Nothing printed yet; the slicer times are estimates; the stand and lever were checked
  only against the CAD meshes and a box for the motor and driver (the MKS board depth is still to be measured).

## 26 Oct (night): STL check and the J2 flange fix

- **Asked** (Josh): whether the shaft's off-centre circles on `c2/plate_1` were wrong, then "just run a check for the
  stl's for everything", then to move the bolt circle and rebuild.
- **Done:**
  - The off-centre circles are the shaft's two eccentric cams, ±1.6 mm (`e=1.6`), 180° apart. Correct.
  - Ran `tools/mesh_audit.py` on all 66 STLs and matched every plate 3MF object to its source by volume. All are
    closed solids (the audit's non-manifold counts on `rings_6810` and `J1_cam` come from its 0.001 mm vertex weld;
    `manifold3d` accepts both, and `J1_cam` is two touching bodies).
  - Found the C2 flange's M4 carrier insert pockets (Ø5.6 on R22) 0.18 mm from the Ø50 stem that is the 6810's
    journal. Moved `out_bolt_r` to 19 for C2 (3.2 mm wall each side), rebuilt and saved `DA_C2_Carrier` and
    `DA_C2_Flange`, ran `check C2 0 mates` (all ok), re-exported both STLs and re-sliced the C2 plates. Rebuilt `DA_SA_C2` (interference clear) and `DA_ARM` (zero
    pose clear); the sweep to every joint limit gives the same result as docs/21 §8, with only the known J2 −120°
    entry into the electronics keep-out. Two extra folded poses (J2 = J4 = ±90°) put the wrist through the desk board,
    which the planner's keep-out already forbids.
- **Learned:** a mate check proves parts fit together, not that each part is strong enough; a thin wall inside one
  part only shows up in a wall scan, and the 0.23 mm minimum had been in the docs/22 table since 21 Oct without a
  note.
- **Decided** (Josh): C2 carrier bolt circle R19.
- **Unverified:** nothing printed. The slivers in `DA_P7_Flange` and `J1_plate` are located only roughly.

## 26 Oct (night, later): the stereographic arm angle

- **Asked** (Josh): read Elias and Wen, *Redundancy parameterization and inverse kinematics of 7-DOF revolute
  manipulators* (arXiv 2307.13122 v2), and see how it applies; then switch `swivel_angle` to its stereographic angle
  everywhere relevant, and check it doesn't affect the CAD.
- **Learned** (from the paper):
  - Our joint layout is their main case, the KUKA iiwa's: shoulder axes meet, wrist axes meet, all offsets zero. It
    has closed-form IK for a given arm angle ψ, by subproblem decomposition.
  - Any arm angle is a tangent field on the sphere of shoulder-to-wrist directions, so by the hairy ball theorem it
    must be singular somewhere, with total order two. The "angle from up" puts that at both poles; their stereographic
    angle puts all of it at one chosen direction.
  - Tested ours: with the elbow at 60°, the old ψ flipped from 180° to 0° at J2 ≈ −29°, where the wrist passes over
    the shoulder, with the elbow standing still.
- **Done:** `swivel_angle` now uses the stereographic SEW angle with the singular direction straight down into the base
  (`E_T`) and `E_R` = −x, so a forward reach still reads ψ = 0 with the elbow up. Three tests: continuous over the top,
  the reference (forward 0, J3 sign, the reading turns with J1), defined everywhere but straight down. New figure
  `sew_over_the_top.svg` and a rewritten Step 2 in [14](14_math_guide.md) §6; notes in [05](05_software_kinematics.md),
  [11](11_controls_vision_track.md) and `PROF_QA.md`. The twin and `swivel` needed no change (the slider wraps at
  ±180°). It is software only: no CAD part, dimension or assembly uses ψ.
- **Decided** (Josh): stereographic ψ, singular straight down.
- **Trade-off:** with J1 turned by θ, an elbow-up pose reads ψ ≈ θ; a reference that read "up" all round would need a
  second singularity straight up.
- **Unverified:** ψ has not been used on hardware.

## 26 Oct (night, latest): closed-form 7-DOF IK

- **Asked** (Josh): write the closed-form 7-DOF IK for a given arm angle, check it against `solve_dls`, and wire it into
  `solve_ik` with tests and docs.
- **Done:** `analytic_ik_7dof` (wrist centre, J4 by the law of cosines, the elbow point on its circle at ψ, J1/J2 from
  the elbow direction, J3 from the wrist seen from link 2, then the §4 Z-Y-Z wrist): up to 8 solutions per ψ, limit
  filtered, FK checked, nearest first. `feasible_psi` lists the ψ that fit the limits. `solve_ik` takes `psi=` and,
  when least squares fails, tries the closed form at ψ moving out from the current one, replacing the restarts from
  the 6-DOF solutions. Four tests; derivation, numbers and a feasible-ψ figure in [14](14_math_guide.md) §6 Step 4.
- **Checked:** 500 random configurations recovered to 5e-12 rad, every solution at the asked ψ. Against least squares
  solved to 1e-9 and clear of the limits, the same joints to 1e-6. Far seeds: least squares alone 154/200, with the
  closed-form fallback 200/200. About 1.6 ms for all 8 solutions.
- **Learned:** where least squares and the closed form disagree it is least squares' 0.1 mm tolerance (near a straight
  elbow that is ~0.01 rad in the joints) or least squares parked on a limit, not a different answer. Near ψ = 0 the
  example and home poses keep fewer branches: J3 is near 0 and the branches that turn it by 180° pass its ±170° limit.
- **Unverified:** not used on hardware; the twin and `cartesian.py` don't pass `psi=` yet (they behave as before unless
  least squares fails).
- **Searched** (Josh: more 7-DOF and singularity work that could help). Leads worth keeping, none adopted yet:
  - choosing ψ along a whole path by dynamic programming over a ψ grid, with real-time adjustment (arXiv 2411.17052),
    which `feasible_psi` makes cheap here;
  - steering ψ away from collisions (Harbin's space-station arm, IJRR), which our capsules (`collision.py`) could score;
  - joint limits as exact ψ intervals (Shimizu 2008; Chou and Liu, *Trans. FAMENA* 48(1), 2024), where we use a grid;
  - a human-like ψ for mirroring (motion-capture studies: the elbow swings so the palm moves toward the head), for
    the MediaPipe mode in [05](05_software_kinematics.md);
  - GeoFIK (arXiv 2503.03992) on the Franka: fixing one joint as the free parameter (our 6-DOF mode, with J3) is less
    robust than an arm angle;
  - a 2026 preprint survey of singularity avoidance (arXiv 2604.13405) recommends manipulability-driven variable
    damping, which [23](23_singularities_and_ccd.md) already tested here with no gain;
  - hardware: Reachy (first generation, 7-DOF printed arm, LGPL software / CC BY-SA CAD) and Berkeley Humanoid Lite
    (printed cycloidal actuators, arXiv 2504.17249) as references for printed joints.

## 26 Oct (night, last): planning the arm angle along a line

- **Asked** (Josh): build the path-wide arm-angle planner (dynamic programming over `feasible_psi` along a Cartesian
  path, scored by joint motion and the collision capsules), used by `cartesian.py`, with tests and docs.
- **Done:** `deskarm/elbow.py`, `plan_arm_angle`: stages every 1 cm / 0.1 rad, a 10° ψ grid anchored at the start's ψ,
  closed-form nodes in the start's branch, node cost from capsule clearance (lazily checked) and joint-limit margin,
  edge cost from joint motion in seconds, Bellman's recursion, then a fill at the 2 mm spacing. `linear_path` and
  `linear_move` take `elbow="plan"` (default) or `"keep"`; `LinearMoveInfo.elbow` says which ran; no plan means the
  old warm-started IK. `analytic_ik_7dof` got `verify=` and builds R02/R04 directly (1.6 → 0.65 ms without the check).
  Three tests (`test_elbow.py`), docs/14 §6 Step 5 with `elbow_plan.svg`.
- **Learned:**
  - A coarse ψ grid fails near J2 = 0 or J6 = 0: there a few degrees of ψ swing J1/J3 (or J5/J7) by tens of degrees,
    so the start's ψ must be a node and the fill must be free to nudge ψ.
  - Checking clearance only on the chain the search picks (then re-running) keeps the result optimal and needs ~15
    checks instead of ~600.
  - On 80 random lines both methods complete 25 (one each the other can't); warm-started IK collided on 2, the plan
    on none. A 15 cm line plans in ~0.4 s against ~0.1 s warm-started.
- **Decided** (Josh): plan the elbow by default for straight-line moves in 7-DOF mode.
- **Unverified:** not run on hardware; the twin's interactive moves now wait ~0.4 s longer per 15 cm; the capsules
  are the 20-21 Oct ones, checked against Fusion then, not since.

## Corrections: earlier conclusions that turned out wrong

| Found | Earlier claim | What is true | Where fixed |
|---|---|---|---|
| 1 Oct | J2 spring 917 N/m cancels the load (~80% less torque) | Over-balances the folded arm; minimax sizing gives ~623 N/m (now 565 with the light wrist) and ~50% | `counterbalance.py`, [14](14_math_guide.md) §13 |
| 30 Sep | J5 only rolls the wrist, so a small motor will do | J5 holds the tool sideways and needs the same torque as J6 | [03](03_actuators_gearboxes.md), `arm.toml` |
| 8 Oct | J3/J4 eccentric on a 6802 | A 12 mm hollow input leaves 0.4 mm of wall; use a 6804 (D31) | [03](03_actuators_gearboxes.md) |
| 8 Oct | Belts are all 225 mm | Length depends on where the motor sits (~270-282 mm on J2) | [03](03_actuators_gearboxes.md) |
| 8 Oct | TOPP respects the torque limits | It overshot by ~7% on a fast J1 swing until it used the fast end of each grid cell | `topp.py` |
| 8 Oct | AliExpress parts lists at listing prices | ~C$100 low: welcome deals and shipping | purchasing rules in this file, 8 Oct |
| 12 Oct | A 1:1 belt pickup reads J1-J4 | 1:1 needs equal pulleys, as big as the joint output; use a through-rod or a small tube instead | [16](16_packaging_and_routing.md) §3.6 |
| 12 Oct | J5/J6 read "on-axis at the far end" | No stationary part reaches that end; decide in the wrist CAD | [16](16_packaging_and_routing.md) §3.6 |
| 12 Oct | A wrong motor-turn count shows up as a large residual \|θ − θc\| | It doesn't: the residual stays small; only the coarse error budget protects | [16](16_packaging_and_routing.md) §3.3, [14](14_math_guide.md) §16 |
| 13 Oct | The Arduino IDE is not installed on this machine | 2.3.8 was installed on C: with esp32 core 3.3.7 | the project log §3, §7 |
| 13 Oct | Set each driver's Dir so positive counts turn the joint positive (software README) | Dir only works on the pulse interface; over CAN the sign is `motor_sign` in `arm.toml` | software README, `backends.py` |
| 14 Oct | docs/14's equations "render on GitHub" | GitHub's Markdown strips `\\` and `\,` before drawing math: every matrix showed as one row (33 of 44 display equations broken) | docs/14 rewritten; `test_math_survives_github_markdown` |
| 14 Oct | docs/14 §9: the TOPP example equals the trapezoid time $d/v+v/a$ | 40° at 90°/s and 180°/s² never reaches full speed: the optimum is the triangle $2\sqrt{d/a}$ = 0.943 s (a loose tolerance hid it) | docs/14 §9 |
| 14 Oct | The J2 printed input shaft sees ~6 MPa (chat, 14 Oct) | That was twist alone at the full section; with belt bending at the flats it is ~20 MPa at full holding torque | docs/14 §15 Step 10, J2 plans |
| 12 Oct | Wrist ratio 22.22 | (1 + 52/14)² = 22.2245; ratios must be exact once counts are converted | `arm.toml` |
| 15 Oct | MT6701 INL ±0.5° typical (12 Oct, from the 2020 Rev 0.1 datasheet) | Datasheet v1.9 (2024.05): ±1.0° typical at 25 °C aligned, ±1.5° max over −40…125 °C; the J2 coarse budget is ~3.5° against ±3.75°, not ~3.1° | [16](16_packaging_and_routing.md) §3.3-3.4, [14](14_math_guide.md) §18, the project log §9 |
| 14 Oct | Kairo at M1.9 "can likely draw boards A/B" | It drew both, but writes one sheet per project: board A (64 parts) had to be split along power nets and joined by hand | `hardware/README.md`, the project log §10 D |
| 19 Oct | Turntable stanchion mounting holes at $X = \pm 35\text{ mm}$ | Left a 46 mm gap, severely colliding with the 60 mm Upper Arm link; shifted to $X = \pm 47\text{ mm}$ for 70 mm gap (5.0 mm clearance) | [18](18_cad_mechanical_architecture.md), `DeskArm_J1_Turntable_Spindle` |
| 19 Oct | Flat bearing seats against 3D-printed housing faces | Outer and inner races rub flat plastic and 2RS rubber seals; added Ø56 mm relief undercut in Base Housing and Ø53.5 mm step on Spindle | [18](18_cad_mechanical_architecture.md), `DeskArm_J1_Base_Housing` |
| 19 Oct | J2 Cycloid Discs placed inside Left Cheek body ($X = -38, -45\text{ mm}$) | Discs (OD 96.8 mm) clashed with Cheek internal cavity (bore Ø54 mm); moved to shoulder axis exterior ($X = -66, -72\text{ mm}$) where they operate as an external reduction stage with 0 clash | [18](18_cad_mechanical_architecture.md), `DeskArm_Shoulder_Assembly` |
| 19 Oct | Forearm distal J5 bearing seat modeled as transverse bore | Solid front wall caused $15,371\text{ mm}^3$ collision with Wrist Pitch Yoke; trimmed front face flat at $Y = 180\text{ mm}$ with axial semi-circular 6806 cradle and stepped Yoke journal | [18](18_cad_mechanical_architecture.md), `DeskArm_Forearm_Assembly` |
| 19 Oct | Forearm elbow hub step pocket ended at $R = 34\text{ mm}$ | Upper Arm fork ears extend to $Y_{world} = 225\text{ mm}$ ($Y_{local} = 35\text{ mm}$), overlapping Forearm wide body by $366.6\text{ mm}^3$; extended hub step cut to $Y_{local} = 38\text{ mm}$ on plane $Z = 15\text{ mm}$, yielding 0 clashes and 2.0 mm running clearance | [18](18_cad_mechanical_architecture.md), `DeskArm_Full_Assembly` |
| 19 Oct | "0 clashes" across the full 7-DOF arm (18 §4) shows the assemblies are verified | True, but the turntable floats 13.5 mm off its bearing, the reducers and shafts aren't modelled, there is no J3, and the full assembly uses unlinked old copies | [19](19_cad_review.md) §3-4 |
| 19 Oct | J1 ring-pin flutes "firmly backed by the solid rim" (18 §3.1) | Cut 0.53 mm into the wall (pin circle R 40, wall R 42): the pins aren't held; a ~Ø78.5 cavity would embed them 70% | [19](19_cad_review.md) §4.3 |
| 19 Oct | Upper arm halves form a "rigid 60 mm box section" (18 §3.4) | The cavity opens outward: two channels, ~20× less torsional stiffness | [19](19_cad_review.md) §6.1 |
| 19 Oct | Yoke has a Ø29.95 journal and dual 6802 seats; counterbalance axle through the cheek thickness (18 §3.2, §3.5) | v3 has no journal and Ø15.5 holes in 4.5 mm ears, v1 has no holes; the axle runs perpendicular to J2 | [19](19_cad_review.md) §5.4, §7 |
| 19 Oct | Elbow cleared by the hub-step fix (row above) | Clear at 0°; the forearm hits the upper arm beyond ~±75° (`arm.toml` ±140°) | [19](19_cad_review.md) §6.3 |
| 19 Oct (evening) | Elbow overlap 3.4 cm³ at 90°, 6.7 cm³ at 150°; contact is the forearm body on the upper arm's side walls (19 §6.3) | Fusion's interference gives 0.67 cm³ at 90° and 9.2 cm³ at 140°; first contact (±75°) is the forearm's hub neck on the Ø72 fork-recess rim, the side walls only later. The ±74° clear range stands | [19](19_cad_review.md) §13.5-13.6 |
| 19 Oct (evening) | J2 first contact "only past ±140°" (19 §6.3) | At ±140° itself (upper arm on the turntable rim); still clear over the ±120° needed | [19](19_cad_review.md) §13.5 |
| 19 Oct (evening) | Yoke v3 is the latest yoke (19 §1, §7) | The separate "v1" file was saved 24 min later; which is meant is Josh's call | [19](19_cad_review.md) §13.1 |

| 20 Oct | J4 clears ±74° (19 §13.5, in-memory model) | ±73° in the saved files: 0.1 mm³ hub-neck touch at ±74° | [19](19_cad_review.md) §13.9 |
| 20 Oct | A Ø78.5 J1 cavity holds the ring pins 70% embedded and clears the disc (19 §4.3) | The disc sits e off the axis, so its lobes reach R 39.79: a Ø78.5 cavity hits them (46 mm³ per disc). Ø81, pins held at both ends | [19](19_cad_review.md) §14.1 |
| 20 Oct | Shoulder height 196 mm, as in the 19 Oct CAD (D36, first version) | 209 mm: two 6810s make the base 61 mm, and the cheeks can't get shorter under the NEMA23 | [09](09_decisions_open_questions.md) D36, `arm.toml` |
| 20 Oct | J1 cam keyed on the shaft flat over its full length (cam v1) | The LDO flat covers only the last 15 mm; the key runs z 12-22.5 (cam v2) | [19](19_cad_review.md) §14.6 |
| 20 Oct | LDO 60 mm: 2.1 Ω, 4.1 mH (Spool3D listing) | 2.0 Ω, 3.3 mH (LDO's drawing) | `arm.toml`, [03](03_actuators_gearboxes.md) |
| 20 Oct | J1 stack "clear" in the in-memory check (19 §14.2, §14.6) | The check's plate and cam had been cut by the disc's features (cross-component cuts); re-run clean after the fix, and clear is not the same as mating (§14.7) | [19](19_cad_review.md) §14.7 |
| 20 Oct | One-piece double cam (cam v1, v2) | Can't be assembled: the lower 6804 is trapped. Two pieces (v3) | [19](19_cad_review.md) §14.7 |
| 20 Oct | The elbow can reach ±140° once the forearm neck is narrowed (19 §8 item 4, `arm.toml`) | With J3 and J5 modules in line, the forearm meets the upper arm at 96°; moving both along their links gives ±120°, and ±140° would push the counterbalance lug into link 3's sweep | [20](20_cad_full_arm.md) §4, D40 |
| 20 Oct | Counterbalance at a = 100, b = 150 mm, 574 N/m (D25) | The CAD has no room for it: the lug must pass under link 3; a = 70, b = 98 mm, 1252 N/m, 183 N peak | [20](20_cad_full_arm.md) §4, `arm.toml` |
| 20 Oct | Arm 7.98 kg, 4.39 kg past the shoulder; J2 margin 1.48-1.53 (hand estimates) | 9.47 kg, 5.33 kg past the shoulder from the CAD; J2 1.32, J5 1.20; tipping needs the clamps | [20](20_cad_full_arm.md) §9, `arm.toml` |
| 20 Oct | P56 check: `planet2_0 / carrier1: 0.037 (want 1.0)` might be a real 1 mm gap error | It measured sun 2's mesh (carrier 1 and sun 2 are one body); the drawn gap is 1.0 mm. Check fixed | [21](21_checks_and_review.md) §11 |
| 20 Oct | Board C fits a 25 × 25 space on the encoder cap | Its schematic needs ~44 × 34; space enlarged | [21](21_checks_and_review.md) §10 |
| 20 Oct | The "simple" gripper is a 2-finger parallel one (D42, first drawing) | docs/06 §2 had already chosen a 3-prong, self-centring gripper (option A); redrawn the same night | [21](21_checks_and_review.md) §12 |
| 20 Oct | The tool plate seats on the receiver (CAD faces touching) | With 3 mm seats the 6 mm balls would hold it 2.2 mm up; seats deepened to 5.24 mm | `receiver()`, [21](21_checks_and_review.md) §8 |
| 20 Oct | Slicing totals: "49 parts, ~2.9 kg, ~126 h" | Still right for the whole arm (now ~2.9 kg, ~124 h); one of each part file is ~2.3 kg, ~96 h | [21](21_checks_and_review.md) §3 |
| 20 Oct | J3 needs a through-tube and two 36T pulleys for its encoder; J5/J6 sensing open (D33) | The hollow input shaft already takes a rod on J3 as on J2/J4; J5/J6 use the motor encoder + hall (D42) | [21](21_checks_and_review.md) §11 |
| 20 Oct | The self-collision capsules (centreline only) protect the arm | They missed the off-centre actuators, link 3's collar and the desk; extended and scored against Fusion: 0 missed in 112 poses | `collision.py`, [21](21_checks_and_review.md) §5 |
| 20 Oct | ~60 Ø5 pins in the BOM; printed PETG at 0.8 g/cm³ | ~100 Ø5 pins; the slicer gives ~0.9 g/cm³ for the housings | `docs/bom.csv`, [21](21_checks_and_review.md) §2-4 |
| 20 Oct | The worst gravity pose is on the −90 / 0 / +90° grid (`torque_budget.py`) | Only while every lump sits on the centreline; the CAD's off-centre actuators put it between, up to 4% worse | `torque_budget.py`, [20](20_cad_full_arm.md) §9 |
| 26 Oct | The 25 Oct fillet pass changed volumes by under 1% and left every mating face alone | It swallowed the J1 base's eight shell ribs (−33 cm³, 6%), took 0.4-0.5 cm³ from the stand and forearm, and rounded edges on small holes on the cycloid housings and fingers; re-saved with two guards | `fusion_lib.soften`, [22](22_print_readiness.md) §5a |
| 26 Oct | The J1 base with fillets is v5 | v5 was the 20 Oct rebuild; the 25 Oct fillet save was v6, and the final fixed one is v9 (v7 and v8 were the morning fixes) | the project log §1e |
| 26 Oct | The 26 Oct morning fix (two guards) left the fillet pass sound | Protection by name still failed on faces touching the bed chamfer, downward edges printed as overhang, and the gripper carrier lost its bed contact; three more fixes the same day | `fusion_lib.soften`, [22](22_print_readiness.md) §5a |
| 26 Oct | docs/14 §2-§3: home TCP (0.350, 0, 0.244), J2 column (0.035, 0, −0.350), 3.43 N·m; the §2 example's comment 0.165 | Those used the 0.080 m TCP (and the comment the 130 mm shoulder); with 0.110 m and 209 mm: (0.365, 0, 0.218), (0.0086, 0, −0.365), 3.58 N·m | [14](14_math_guide.md) §2-§3 |
| 26 Oct | LDO 60 mm motor: corner ~350 rpm, ~31 W (docs/03) | From the listing's 2.1 Ω, 4.1 mH; `arm.toml` has the drawing's 2.0 Ω, 3.3 mH, which give ~370 rpm and ~37 W (the budget already used them) | [03](03_actuators_gearboxes.md) §1b |
| 26 Oct | docs/07: USD ~20 of headroom under the cap | The core alone (1,031) is USD 31 over | [07](07_bom_budget_sourcing.md) |
| 26 Oct | docs/16 §3.3: J2's coarse budget ~3.5°, and the predicted wind-up "already in deskarm.physics" can be subtracted | ~3.65° with the CAD masses; `CanBackend` doesn't subtract it (fine with a calibrated encoder, refused without) | [16](16_packaging_and_routing.md) §3.3 |
| 26 Oct | The twin warns about the wrist, elbow and shoulder singularities (docs/02 §5c) as the arm's singularities | True only with J3 locked; with seven joints the wrist and shoulder cases aren't singular and J2 = J6 = 0 together is | [23](23_singularities_and_ccd.md) |
| 26 Oct | The C2 housing (144 × 132.5 mm) needs the CORE One L: "auto-orient failed" on the A1 mini (`slice.json`, docs/22 §3) | It fits; the CLI's arrange margin (24 mm round each part) rejected it. Sliced with the parts placed by hand: A1 mini, ~11.5 h | [24](24_first_prints_and_j2_bench.md), [22](22_print_readiness.md) |
| 26 Oct | The coupons test the arm's fits (docs/22 §2, docs/print_tolerances) | They tested 623/608 seats, 3/8 mm pegs and inserts round 4.0/5.6/6.4, with no column at the CAD's +0.05 pin fit; remade to the CAD's sizes, plus 6810/6806 rings | `tools/coupons.py`, [print_tolerances](print_tolerances.md) |
| 26 Oct | The C2 flange is printable as built (docs/22 §1 wall table, min 0.23 mm, no note) | Its M4 carrier insert pockets sat 0.18 mm from the 6810 journal; moved to R19 | `tools/fusion_arm.py`, [24](24_first_prints_and_j2_bench.md) §5 |
| 26 Oct | docs/14 §6: ψ from projected +z, "if the arm is vertical the code uses +x" | That keeps it defined but not continuous: ψ flipped 180° when the wrist passed over the shoulder (J2 ≈ −29° with the elbow at 60°); now the stereographic angle | `ik.py`, [14](14_math_guide.md) §6 |
| 26 Oct | D36-D42 (docs/09) and the 5 V rail choice (hardware README) were delegated to the coding agent | Josh decided them; the records were corrected | [09](09_decisions_open_questions.md), [hardware](../hardware/README.md) |

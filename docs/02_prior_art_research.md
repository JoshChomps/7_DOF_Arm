# 02: Prior Art and Research

Open-source and research projects close to this arm, and **what to take from each one**. Researched September 2026. Prices change, so check them before buying.

> **Licensing note:** "open source" varies. PAROL6 is GPLv3, Thor is CC BY-SA 4.0, and
> Arctos hardware plans are a paid download. Designing our own parts *inspired by* these
> is fine. Copying their geometry means following their license terms.

---

## 0. PCrnjak / Source Robotics (the main reference)

[github.com/PCrnjak](https://github.com/PCrnjak) is the most useful single source for this project, because one
developer has iterated through several arms (**Faze4 → CM6 → PAROL6**) and open-sourced the controllers too.

| Repo / product | What it is | Why it matters to us |
|---|---|---|
| **PAROL6** + `PAROL-docs`, `PAROL6-python-API`, `PAROL-Web-Commander-pc`, `Parol6_Better_Command` | Desktop arm, docs, Python API, web commander | The reference for layout, **Python API design**, and a **web-based commander** (similar to our digital twin) |
| `faze4_ros`, `ar3_core` | ROS/MoveIt packages for Faze4 and AR3 | Examples for the later ROS 2 wrapper |
| `CM6_control_software` | Control software for the compliant BLDC arm | The BLDC/compliant route (controls track L4) |
| `s_visual_kinematics` | Python DH FK/IK + trajectory + workspace visualization | A quick start for our kinematics and plots |
| **Spectral Micro BLDC** (`Spectral-BLDC-docs`, `Spectral-BLDC-Python`, `SpectralMicroROS2`, `BLDC-TEST-JIG-CODE`) | Small BLDC FOC driver: 14-bit encoder, inline current sense, CAN, **position/velocity/torque/impedance** modes. Python lib + ROS 2 package | Reference for our custom FOC driver (board E), and a possible off-the-shelf BLDC driver ([Tindie](https://www.tindie.com/products/sourcerobotics/spectral-micro-bldc-controller)) |
| `STEPFOC-docs` | Documentation for a stepper FOC driver (details not published in the README) | Stepper + FOC = torque control on steppers. Watch this one |
| `S-Drive-Firmware`, `BLDC_SVPWM_lookup_table`, `Python_SVPWM_BLDC_Lookup_table` | Small L6234 BLDC driver firmware and SVPWM tools | Learning material for FOC/SVPWM |
| **SSG-48 gripper** (`SSG48-gripper-docs`), `MSG-gripper-docs`, `Compliant-robotic-gripper` | Printed adaptive electric gripper: BLDC, **force control**, CAN 1 Mbit, 0-48 mm stroke, 80 N ([Hackster](https://hackster.io/Source_robotics/ssg-48-3d-printed-adaptive-electric-gripper-73fb3f)) | Reference for gripper mechanics and a force-controlled tool on the same CAN bus |
| `3D-printed-Differential-robotic-wrist` | Belt-driven (HTD5M) differential wrist, STEP + BOM | Option for a lighter wrist (motors moved back into the forearm) |
| `ros-realtime-rpi4-image-source` | ROS 2 + real-time Linux for Raspberry Pi | If the arm later runs standalone on a Pi |
| `real2render2real` | Scaling robot data without dynamics sim or hardware | Learning-based control (post-April) |
| `BetterSerialPlotter`, `SHM_COMM`, `PC_oclock` | Dev tools: serial plotting, shared-memory IPC, timing | Handy for the controls logging setup |

## 1. Complete open-source 6-DOF arms

| Project | Actuation / gearing | Cost (approx.) | What to take from it |
|---|---|---|---|
| **[PAROL6](https://github.com/PCrnjak/PAROL6-Desktop-robot-arm)** (Source Robotics) | Steppers + precision planetary reducers + belts. 400 mm reach, 6 axes, 5.5 kg | Kit sold; BOM in repo | **The closest match to our goal.** Industrial-style desktop arm with a Python API, commander GUI, and ROS 2/MoveIt sim + URDF. Study the joint layout, cable routing, and software stack. Its BOM: six NEMA17 motors (3× 65 N·cm on J1-J3, 2× 45 N·cm on J4/J5, a 16 N·cm pancake on J6) on 10:1-25:1 metal planetary gearboxes, one controller board in the base. Why ours is heavier: [15](15_mass_and_design_options.md) §3. ([docs](https://source-robotics.github.io/PAROL-docs/), [Hackaday](https://hackaday.io/project/191860-parol6-desktop-robotic-arm)) |
| **[Thor](https://github.com/AngelLM/Thor)** (AngelLM) | Steppers + printed reductions, yaw-roll-roll-yaw-roll-yaw, 625 mm tall, 750 g payload | < €350 hardware | Started as a **university final-year project**, which makes it a good model for a student build. All design work in FreeCAD/KiCad, with GRBL/RRF firmware. ([FreeCAD blog](https://blog.freecad.org/2024/03/05/forged-in-freecad-thor-an-opensource-robotic-arm/), [Printables](https://www.printables.com/model/1498024-thor-open-source-3d-printable-robotic-arm)) |
| **AR4 Mk5** (Annin Robotics) | NEMA steppers, CNC aluminum + prints, Teensy controller | ~$2,000 | A well-proven "finished" design. The Mk5 moved to hall-effect homing sensors, which is worth copying. Aluminum makes it too expensive for us. ([Hackster](https://www.hackster.io/news/is-the-annin-ar4-the-workhorse-robot-arm-for-you-6a26487fb517)) |
| **Arctos** | Steppers + belts + **printed cycloidal** boxes. Open-loop version: modified GRBL on an Arduino Mega + CNC shield. **Closed-loop version: MKS SERVO42D/57D over CAN with a CANable adapter**, driven from ROS / their own GUI | ~$400 parts (+€40 plans) | Shows that printed cycloidal drives work in a cheap 6-axis arm, and its closed-loop version is **the closest existing reference for our driver layer** (same drivers, same bus). ([docs](https://arctosrobotics.com/docs/), [Arduino blog](https://blog.arduino.cc/?p=34636), [Hackster](https://hackster.io/news/arctos-robotics-aims-to-design-a-3d-printable-robot-arm-as-easy-as-building-lego-03559e924d70)) |
| **[CM6](https://hackaday.io/project/180588-cm6-compliant-3d-printed-robotic-arm)** | 6 gimbal **BLDC** motors, low-ratio (5-9:1) printed gearboxes, quasi-direct drive | Cheap | **The BLDC/compliant alternative.** The arm is backdrivable and safe to touch. Worth reading before deciding stepper vs BLDC. |
| **[Atlas](https://hackaday.io/project/168259-atlas-6dof-3d-printed-universal-robot)** | 5 printed hollow-shaft cycloidal reducers ("OpenCyRe"), mixed ODrive BLDC + steppers, Teensy nodes on **one CAN bus**, 8 encoders | - | **Hollow-shaft cycloidal + CAN-per-joint** is the architecture we're proposing. ([Hackster writeup](https://www.hackster.io/news/atlas-is-a-3d-printed-6dof-robotic-arm-495bbcf5b9e1)) |
| **[CyBot](https://hackaday.io/project/182821-3d-printed-cybot-cycloidal-6-axis-robot-arm)** | Steppers + printed cycloidal; Arduino Mega (easy) or RPi + STM32 (advanced) | Cheap | Offers a simple and an advanced electronics path for the same mechanics. |
| **[RR1 rev 2](https://hackaday.io/project/196907-rr1-real-robot-one-revision-2/details)** | 7 steppers + printed **split-ring compound planetary** reducers, FreeCAD sources | Cheap | The strongest printed-**planetary** reference. Read it as the counterpoint to cycloidal. |
| **Dummy** (Peng Zhihui / "稚晖君") | Steppers + **commercial harmonic reducers**, custom closed-loop stepper drivers on CAN, STM32 + ESP32, DH kinematics, teach pendant | Higher (CNC aluminum, harmonic drives) | The "Iron Man Dummy" namesake. A polished full-stack design: a closed-loop stepper per joint on CAN is the pattern this arm uses too. ([Hackaday](https://hackaday.com/tag/ahrs/)) |
| **Faze4** (P. Crnjak, PAROL6's predecessor) | Printed cycloidal J1-J5 + planetary J6 | - | Shows cycloidal on the big joints and planetary on the small wrist joint, which is the same split we recommend. ([discussion](https://hackaday.io/project/167247-faze4-robotic-arm/discussion-150371)) |

### Learning-oriented low-cost arms (lighter, servo-based)

| Project | Notes |
|---|---|
| **SO-101 / SO-ARM101** (TheRobotStudio + Hugging Face LeRobot) | ~$130 DIY, 6 DOF using Feetech serial bus servos. Built for **teleoperation and imitation learning** with a leader/follower pair. Too weak for our payload, but its **software ecosystem (LeRobot)** is where learning-based control is happening. Consider building a cheap **leader arm** later to teleoperate our arm. ([Gigazine](https://gigazine.net/gsc_news/en/20250501-hugging-face-robotic-arm), [GitHub topic](https://github.com/topics/so-101)) |
| **OpenArm** (Enactic) | 7-DOF humanoid research arm, backdrivable BLDC, ROS 2 driver, full CAD + BOM open. ~$6,500 bimanual. The leading *open* compliant arm: too expensive to build, but its docs are worth reading. ([GitHub](https://github.com/enactic/openarm)) |

---

## 2. Actuators & gearboxes

| Project / paper | Type | Takeaway |
|---|---|---|
| **OpenCycloid** ([Instructables](https://www.instructables.com/OpenCycloid-3D-printed-Open-Source-Robotic-Actuator/)) | Printed cycloidal actuator | A full build guide, and a good first cycloidal to copy for the test bench. |
| **OpenCyRe** (from Atlas) | Hollow-shaft printed cycloidal | Hollow shaft for cable pass-through. |
| **[Compound Differential Cycloidal Drive](https://github.com/msmSatoLabs/Compound-Differential-Cycloidal-Drive)** | Two-stage differential cycloidal, 121:1, BLDC | Very high ratio in a compact package. 2025-26 build log. Advanced. |
| **cycloidgen** ([PyPI](https://pypi.org/project/cycloidgen/)) and **Hexagon GR3** ([PDF](https://hexagon.de/pdf/gr3_e.pdf)) | Design tools | Generate cycloidal disc profiles and export DXF/STEP/STL, with checks for undercut. |
| **HD20** ([GitHub](https://github.com/robrotics/hd20/releases)) | Printed strain-wave (harmonic) 20:1 for NEMA17 | ~$4 of hardware, tested to 3.3 N·m. Zero-backlash option for the **wrist**. |
| **Open HD** ([Hackaday](https://hackaday.io/project/160378-open-hd)) and [MakerWorld harmonic drive](https://makerworld.com/en/models/402774-harmonic-drive) | Printed harmonic drive | More strain-wave references (NEMA17/23). |
| **Split-ring planetary** ([awesci](https://awesci.com/3d-printed-split-ring-planetary-gearbox/)) | Compound planetary | High ratio in one planetary "stage". Watch backlash and efficiency. |
| **COMPAct** ([arXiv 2510.07197](https://arxiv.org/abs/2510.07197)) | Research, 2025 | Automatically optimizes printed planetary / compound planetary actuators and generates the CAD. Reports **single-stage planetary ≈ 0.59° backlash vs compound ≈ 2.6°**, which is useful data for our choice. |
| **Printed 1:30 compound planetary with printed roller bearings** ([SSSA paper](https://www.iris.santannapisa.it/retrieve/c966b2af-40d9-4f00-ae12-674c9b95831b/md-25-1816.pdf)) | Research | Steel rollers in polymer raceways: a cheap bearing trick. |
| **Quasi-direct-drive (MIT Mini Cheetah lineage)** | BLDC + low-ratio planetary | QDD = ratio < 10:1 with a low-KV BLDC and current control, which gives high bandwidth, backdrivability, and force control. Hobby versions with printed parts and Halbach rotors exist. One open design reports **28 N·m, 0.95 kg, 6:1, < $150 materials**. This is the "v2 / advanced" path. ([MIT thesis](https://dspace.mit.edu/handle/1721.1/127165), [ODrive forum](https://discourse.odriverobotics.com/t/high-torque-actuator-for-legged-robots/776)) |
| **How To Mechatronics: "Best 3D printed drive"** ([article](https://howtomechatronics.com/how-it-works/what-is-the-best-3d-printed-drive-for-your-next-robotic-project/)) | Head-to-head test | Compares printed cycloidal, planetary, and harmonic drives for efficiency, torque, backlash, and durability. **Read this before the test bench.** Also the [cycloidal design tutorial](https://howtomechatronics.com/how-it-works/what-is-cycloidal-driver-designing-3d-printing-and-testing/). |

---

## 3. Human-like hands (for gripper #3)

| Project | DOF / actuation | Cost | Fit for us |
|---|---|---|---|
| **[Amazing Hand](https://huggingface.co/blog/pollen-robotics/amazing-hand)** (Pollen Robotics / Hugging Face, 2025) | 4 fingers, 8 DOF, 2 hobby servos per finger (differential), rigid bones + TPU shells, 400 g | **< €200** | **Best starting point.** Cheap, fully printable, self-contained, with Python/Arduino control. ([Seeed wiki](https://wiki.seeedstudio.com/hand_amazinghand/)) |
| **RUKA** ([arXiv](https://arxiv.org/abs/2504.13165v1)) | 5 fingers, 15 underactuated DOF, tendon-driven | < $1,300 | A research-grade step up. Shows how to do tendon routing well. |
| **ORCA** ([arXiv](https://arxiv.org/abs/2504.04259)) | 17 DOF tendon-driven, tactile sensors, "pop-out" joints that dislocate instead of breaking | < 2,000 CHF | The robustness ideas are worth copying even if we don't build it. |
| **BPI SoftHand** ([ref](https://ro.uow.edu.au/eispapers1/3270)) | **1 actuator**, underactuated, tendon-driven, printed | Low | Human-like grasps from a single motor, the cheapest route to a hand-like grip. |
| **YEAH Hand** | 5-finger tendon-driven, printable | Low | Another affordable option ([humanoid.guide](https://humanoid.guide/product/ruka)). |

---

## 4. Octopus-inspired / soft grippers (for gripper #2)

| Project | Idea | Takeaway |
|---|---|---|
| **SpiRobs** (logarithmic-spiral robots) | Tentacles shaped as a **logarithmic spiral**, driven by **just 2 tendons**, printed in TPU. They curl around objects, adapt to size, and grasp objects that vary by more than 100× in size, holding up to 260× their own weight. | **The basis for the octopus gripper**: printable, and only needs 1 servo per tentacle. ([80.lv](https://80.lv/articles/you-have-to-see-these-robots-that-move-like-octopus-arms/), [related arXiv 2511.01770](https://arxiv.org/pdf/2511.01770), [tapered backbone modeling arXiv 2603.19124](https://arxiv.org/pdf/2603.19124)) |
| **Harvard/Beihang tentacle** | Tapered soft arm with **suction cups**: bending + suction combined | Taper angle controls how tightly it spirals, and adding suction greatly broadens what it can grip. A possible v2 add-on. ([Harvard SEAS](https://seas.harvard.edu/news/tentacle-bot)) |
| **MERL "Cthulhu" gripper** | Multi-tentacle, 12 DOF, printed metamaterial tentacles, RC servos + steel cables | Multi-tentacle layout with servo-and-cable actuation, which is cheap to copy. ([MERL](https://merl.com/publications/TR2021-088)) |
| **Fab Academy 2026 "Octopus Arm"** | 3 TPU tentacles, servo-driven Dyneema tendons, a **modular end effector for a UR3** | Very close to what we want, with full open files, firmware, and CAD. ([project](https://fabacademy.org/2026/labs/ulima/students/andre-mamani/final.html)) |
| **Fin Ray grippers** | Passive compliant fingers in TPU | Useful as **fingertips for the 3-prong gripper**: they conform to objects with no extra actuators. ([example](https://makerworld.com/models/2866227)) |

---

## 5. Tool changers / quick-change interfaces

| Reference | Takeaway |
|---|---|
| **Berkeley open-source tool changer** ([CASE 2018 paper](https://goldberg.berkeley.edu/pubs/CASE2018-ron-tool-changer-submitted.pdf)) | 3D-printed, passive, backlash-free, low cost: exactly the design brief we have. |
| **UPenn ModLab quick-change end effector** ([page](https://www.modlabupenn.org/quick-change-end-effector/)) | **Pogo pins** carry power and a comms bus (CAN) across the interface. |
| **DAKSH v2** ([GitHub](https://github.com/ankurv2k6/daksh-toolchanger-v2)) | Printed servo-less mechanical lock (3D-printer tool changer, same principle). |
| **ISO 9409-1** flange standard | Industrial robots use this flange pattern. Adopting the small size (31.5 mm PCD, 4×M5) keeps our tools compatible with common hobby grippers. |

---

## 5b. Digital twin, visualization & teleoperation

| Reference | Takeaway |
|---|---|
| **[Viser](https://viser.studio)** ([paper](https://arxiv.org/abs/2507.22885v1)) | Python, web-based 3D visualization with URDF, gizmos, and GUI. Built for interactive IK. **Our twin UI** |
| **Seeed reBot Arm web simulator** ([guide](https://wiki.seeedstudio.com/rebot_arm_b601_dm_web_simulator_developer_guide/)) | three.js + URDF + rosbridge with **joint mirroring, control lock, gravity compensation**: close to the feature list here |
| [URDF viewer](https://github.com/qkrdkwl9090/urdf-viewer), [URDF-Threejs-Modifier](https://github.com/robit-man/URDF-Threejs-Modifier) | Browser URDF viewers with live bidirectional control |
| **Vision-based hand shadowing** ([arXiv 2603.11383](https://arxiv.org/abs/2603.11383)) | MediaPipe Hands → depth → DLS IK → SO-ARM101, 90% pick-and-place. **The recipe for hand mirroring** |
| **OAT** (ICRA 2026) ([arXiv 2507.03227](https://arxiv.org/html/2507.03227v1)) | Wrist + hand pose retargeting from a single RGB camera |
| **[dex-retargeting](https://github.com/dexsuite/dex-retargeting)** (AnyTeleop) | Human hand → robot hand retargeting, for the humanoid hand later |
| **SimpleFOC** ([Mini docs](https://docs.simplefoc.com/simplefocmini)) | €7-10 FOC driver + library. Supports BLDC and steppers. Our controls testbed |
| **Steadywin GIM6010-8** ([listing](https://openelab.io/sv/products/steadywin-gim6010-8-robot-motor)) | ~US$72 integrated BLDC + 8:1 planetary + CAN driver, 5 N·m rated, 388 g, 15 arcmin backlash. A cheap QDD option for v2 |

## 5b-2. MKS SERVO42D / 57D over CAN (the driver layer)

| Reference | Takeaway |
|---|---|
| **[makerbase-motor/MKS-SERVO42D-57D](https://github.com/makerbase-motor/MKS-SERVO42D-57D)** | The official repo: user manual with the **CAN command set** (frame layout, checksum, position / speed modes, homing, current and protection settings). `deskarm.backends.CanBackend` must be written from this, against a real driver |
| **[mks_servo_can](https://github.com/ricardodeazambuja/mks_servo_can)** ([write-up](https://ricardodeazambuja.com/hardware/2025/06/21/mks-servo-can-python/)) | Python library on top of `python-can`, async, with a **command-line simulator**: lets you develop and test the CAN layer before the motors arrive |
| **[MKSServoCAN](https://github.com/TheSpaceEgg/MKSServoCAN)** | Arduino/ESP32 (TWAI) library for the same drivers: a second implementation to cross-check frame encoding, and a starting point if joint control ever moves onto the ESP32 |
| Arctos closed-loop arm (above) | A printed cycloidal 6-axis arm already running on these drivers |

## 5c. 7-DOF kinematics references

| Reference | Takeaway |
|---|---|
| **[ros2-udq-arm-handwriting](https://github.com/ferdy-alpadry/ros2-udq-arm-handwriting)** (Apache-2.0) | 7-DOF arm in pure Python + ROS 2: **unit dual quaternions**, product-of-exponentials FK, screw-axis Jacobian, DLS IK with null space, resolved-rate control, and a **quantized, rate-limited stepper plant model** (TMC2209-style). Based on Özgür & Mezouar (2016). We adopted the stepper-quantization idea in `SimBackend`. The dual-quaternion/PoE formulation is a good alternative to our DH model to compare against later |
| [7-DOF arm kinematics & dynamics (RoboCloud Hub, D. Menon, 2026)](https://www.robocloudhub.tech/learn/blog/7-dof-arm-kinematics-dynamics) | Overview of DH FK, the 6×7 Jacobian, pseudoinverse + **DLS**, null-space obstacle avoidance, singularities and RNEA dynamics (Pinocchio), consistent with our `deskarm.ik`. **Watch out:** the matrix it labels "standard" is Craig's **modified** DH (rows `[cosθ, −sinθ, 0, a]`, `[sinθ·cosα, …, −d·sinα]`). We use **standard** DH, so never mix DH tables from different sources without converting. Its three singularities map to our arm as: wrist (J6 ≈ 0, J5 ∥ J7), elbow (J4 ≈ 0, arm straight), shoulder (wrist centre on the J1 axis). The twin warns about all three (26 Oct: with all seven joints free only the elbow, and J2 and J6 at 0 together, are real singularities; [23](23_singularities_and_ccd.md)). Its IK-solver table (KDL, TRAC-IK, IKFast, cuRobo, Pinocchio+QP) suggests **TRAC-IK** for the later MoveIt 2 setup, or an IKFast plugin for 6-DOF mode |
| Shimizu et al. 2008, *Analytical IK for 7-DOF redundant manipulators with joint limits* | The arm-angle (ψ) closed-form IK for SRS arms, planned in [05](05_software_kinematics.md) |

## 6. Summary: what the field tells us

1. **Closed-loop steppers + printed reducers** is the proven hobby recipe (PAROL6, Thor, Arctos, AR4, Dummy).
   **BLDC quasi-direct drive** (CM6, OpenArm, Mini Cheetah-style) is where research is heading
   because it's compliant, backdrivable, and safe, but it costs more and is harder to control.
2. **Cycloidal** is the most common printed reducer for big joints. **Planetary** shows up in
   wrists and in designs optimized for efficiency or backdrivability. Harmonic is excellent but
   the printed flexspline wears out.
3. **One CAN bus with a smart driver per joint** (Atlas, Dummy, OpenArm) keeps wiring simple.
4. **Learning-based control** (LeRobot, SO-101) is where low-cost manipulation research is now. If the arm has a clean Python/ROS 2 interface, it can join that ecosystem later.
5. **Soft/tendon grippers** (SpiRobs, Fin Ray, AmazingHand) are cheap to prototype with a
   TPU-capable printer and a few hobby servos.

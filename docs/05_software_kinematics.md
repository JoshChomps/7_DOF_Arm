# 05: Software, Digital Twin & Kinematics

> The math behind everything implemented here, module by module, with pictures and runnable examples:
> [14: Math guide](14_math_guide.md).

## Decision: build our own digital twin first, add ROS 2 later as a layer

| | Own stack (Python + Viser + MuJoCo) | ROS 2 + RViz + MoveIt 2 from day one |
|---|---|---|
| Time to "sliders move the real arm" | **A weekend** | 1-3 weeks (Ubuntu, colcon, ros2_control plugin in C++) |
| Understanding | You write the IK, trajectories, and control, **which is the point for a controls career** | Much of it is hidden inside MoveIt/KDL |
| Collision-aware planning | You'd write it yourself (done: capsule self-collision + desk check reject bad moves; no planner around obstacles yet) | **Free with MoveIt** |
| Ecosystem / résumé | Viser and MuJoCo are the current research tools | ROS 2 is expected in industry |
| OS | Anything | Ubuntu (dual boot, or WSL2 on Windows) |

**So:** the core is our own Python stack. Once the arm works (Phase 6), we add a **`ros2_control`
hardware interface** that wraps the same CAN layer, which gets us MoveIt and ROS 2 on the résumé without blocking April.
PCrnjak's PAROL6 did the same thing: its own commander/Python API first, then ROS 2/MoveIt support.

## Architecture: one API, two backends

```
           ┌────────────── Apps ───────────────────────────────────────────┐
           │ Viser web UI (sliders, drag-gizmo IK, path drawing, joint lock)│
           │ Hand-mirror teleop (webcam → MediaPipe)                       │
           │ Scripts / pick-and-place / later: ROS 2 bridge, LeRobot       │
           └──────────────────────────┬────────────────────────────────────┘
                                      │  arm.move_joints(), arm.move_pose(), arm.state()
           ┌──────────────────────────▼────────────────────────────────────┐
           │ Core: kinematics (FK/IK), trajectory gen, limits, safety,      │
           │ tool registry (TCP/mass per tool), joint-lock constraints      │
           └───────────────┬───────────────────────────────┬───────────────┘
                           │                               │
             ┌─────────────▼────────────┐    ┌─────────────▼────────────┐
             │ SimBackend (MuJoCo)       │    │ RealBackend (CAN → MKS)  │
             │ physics, contacts, gripper│    │ + output encoders, homing│
             └──────────────────────────┘    └──────────────────────────┘
```

**Mirror modes in the viewer:**
- **Sim only:** develop everything before the hardware exists (starting in October).
- **Twin → real:** drag the gizmo or sliders, preview the motion as a "ghost", **confirm**, and the real arm executes it.
- **Real → twin:** the viewer shows the *measured* state (from encoders) overlaid on the commanded state, so
  you can see tracking error and backlash directly.
- **Live mirror:** stream targets at 50-100 Hz with rate limiting, used for teleop. Always behind the E-stop and soft limits.

**Tools:**
- **[Viser](https://viser.studio)** (Python, runs in the browser, has a built-in URDF loader, gizmos, and GUI
  widgets). It was designed for interactive IK and robot visualization, and it works on a phone on the same network too.
- **MuJoCo** for physics sim (the gripper and contacts matter for the octopus gripper).
- **Fusion 360 → URDF:** `deskarm.urdf` generates the URDF from `arm.toml` (joint frames, masses, limits). Since 21 Oct
  the twin draws the CAD: `display mesh` + `tools/cad_meshes.py` export one STL set per link ([20](20_cad_full_arm.md)
  §10), and `tools/cad_masses.py` copies each body's mass and centre of mass into the `[[masses]]` list.
- Reference: **PCrnjak's `s_visual_kinematics`** (DH-based FK/IK + trajectory plots in matplotlib).
- If you'd rather use JavaScript: `urdf-loader` + three.js, with a websocket to the Python core. The
  Seeed reBot Arm web simulator does this (three.js + rosbridge, joint mirroring, control lock, gravity comp).

## Feature: lock a joint while the others move (implemented: joint locks, tool-constraint modes, desk guard)

With **7 DOF (SRS)**, locking any one joint still leaves 6, so the tool can still reach any position and orientation
(away from singularities). Locking **J3 at 0°** turns the arm into the classic 6-DOF arm with closed-form IK. Locking two
joints drops to 5 DOF, and then the solver has to give up one task dimension. Implementation plan:

1. **Joint-space jog:** trivial. Move the selected joints and hold the rest.
2. **Cartesian moves with a locked joint:** numerical IK with **damped least squares**, removing the locked
   joint's column from the Jacobian. Use **task weights**, so position is high priority and orientation lower, and the
   "lost" DOF gets absorbed where it matters least (usually rotation about the tool axis).
3. **Constraint modes in the UI:** "lock J1", "keep tool vertical", "lock orientation, move position",
   "move only along a plane". The same machinery covers all of them: weighted/prioritized IK
   with null-space projection, a standard robotics topic that makes a good write-up.
4. **Null-space motion (7-DOF only):** hold the tool still and swing the elbow with a slider (the "arm angle"),
   or let secondary objectives run automatically (stay away from joint limits, keep the elbow up, avoid obstacles).

## Feature: mirror your hand, path, and movement

| Level | Input | Pipeline | Needs |
|---|---|---|---|
| 1. Path replay (done) | Record waypoints in Viser (joint or straight-line moves) | `deskarm.program` → time-parameterize → execute, loop | Nothing extra |
| 2. Teach-and-replay (done) | Jog the arm, save poses (JSON) | same | Nothing extra |
| 3. **Hand mirroring** (2D+) | Webcam → **MediaPipe Hands** (21 landmarks) | wrist position + palm orientation + pinch distance → scale into the workspace → **One-Euro filter** → IK → rate-limited stream; pinch = gripper close | Webcam |
| 4. Hand mirroring (true 3D) | Depth camera or 2 webcams (stereo), or single-RGB wrist-pose estimation (OAT, ICRA 2026) | same + depth | Depth or stereo |
| 3b. **iPhone as a 6-DoF controller** | iPhone ARKit pose streamed over Wi-Fi: **ZIG SIM Pro** (sends ARKit data as OSC/JSON over UDP/TCP at up to 60 Hz), or later a small custom ARKit app | move the phone in the air → the tool follows (relative "clutch" mode: hold a button to engage) → IK | Your iPhone (iOS Safari has no WebXR AR). No camera setup, and gives full 6-D pose |
| 3c. **iPhone as a depth camera** | **Record3D** app + its `record3d` Python library streams RGB-D over USB | MediaPipe landmarks + depth → true 3-D hand position | Your **iPhone 14 Pro** has LiDAR, so this replaces the USD 150 depth camera |
| 4b. **Full-arm mirroring** (7-DOF) | Webcam → **MediaPipe Pose** (shoulder, elbow, wrist landmarks) + Hands | wrist pose → IK target; **your elbow direction → the arm angle** (null space), so the robot's elbow copies yours | Webcam/phone |
| 5. Finger retargeting | MediaPipe → **dex-retargeting** | human finger joints → robot hand joints | The humanoid hand gripper |

Evidence it works: a 2026 "vision-based hand shadowing" pipeline did exactly this on an SO-ARM101:
MediaPipe → 3D via depth → damped-least-squares IK → joint commands, with **90% pick-and-place success**.

Your vision and modelling experience applies directly here: camera calibration, hand-eye calibration
(OpenCV `calibrateHandEye`), filtering, and latency handling.

## Your setup: Windows + Raspberry Pi + phone

| Piece | Role | Notes |
|---|---|---|
| **Windows laptop** | Development, Viser UI, MuJoCo, vision (MediaPipe/OpenCV), Fusion 360 | Everything in the Python stack runs natively on Windows |
| **ESP32-S3 base controller** | USB ↔ CAN bridge | Firmware speaks the **slcan** (Lawicel) protocol, so `python-can`'s `slcan` interface works on **Windows and Linux** with no special drivers |
| **Raspberry Pi 4 B (2 GB)** | Later: the arm's onboard brain: CAN link, trajectory streaming, safety watchdog, `ros2_control` hardware interface | **2 GB is enough headless** (no desktop, no RViz). Run MoveIt planning, Viser, MediaPipe, and MuJoCo on the laptop, and let the Pi run the real-time side. Upgrade to a Pi 5 (4-8 GB) only if it struggles. `slcand` turns the ESP32-S3's slcan link into a native `socketcan` interface |
| **WSL2 (Ubuntu) on Windows** | Optional ROS 2 / MoveIt on the laptop | USB passthrough into WSL2 needs `usbipd-win`, so it's easier to keep hardware I/O on Windows or the Pi and let ROS talk over the network |
| **iPhone 14 Pro** | Camera (DroidCam/Iriun for iOS), a second Viser screen (Safari), 6-DoF controller (ZIG SIM Pro), LiDAR depth camera (Record3D) | Free-few $ for apps |

## Kinematics notes

### DH layout: 7-DOF SRS (as in `arm.toml`, matching the CAD since 20 Oct)

Zero-offset SRS layout (same structure as the KUKA iiwa), lengths in mm, 7-DOF numbering:

| i | Joint | θ | d | a | α |
|---|---|---|---|---|---|
| 1 | Base yaw | θ1 | d_bs = 209 (130 before 20 Oct) | 0 | −90° |
| 2 | Shoulder pitch | θ2 | 0 | 0 | +90° |
| 3 | **Upper-arm roll (J3)** | θ3 | d_se = 190 | 0 | −90° |
| 4 | Elbow pitch | θ4 | 0 | 0 | +90° |
| 5 | Forearm roll | θ5 | d_ew = 180 | 0 | −90° |
| 6 | Wrist pitch | θ6 | 0 | 0 | +90° |
| 7 | Tool roll | θ7 | d_wt = 70 + tool | 0 | 0 |

With θ3 locked at 0, joints 2 and 4 are parallel, and this reduces to the 6-DOF anthropomorphic arm below.

### 7-DOF IK options
- **Numerical (first):** damped least squares on the 6×7 Jacobian + null-space projection for secondary tasks.
  This is the general tool, and it also handles CAD offsets.
- **Analytic with arm angle ψ (implemented 26 Oct, `analytic_ik_7dof`):** for a zero-offset SRS arm, the shoulder, elbow, and wrist form a
  triangle with fixed sides. Given the tool pose, the elbow lies on a circle, and ψ picks the point on it. For each ψ the
  joint angles are closed-form, and joint limits map to allowed ψ intervals
  (Shimizu et al., 2008, *"Analytical inverse kinematic computation for 7-DOF redundant manipulators with joint limits"*).
  Elias and Wen (arXiv 2307.13122, v2 2024) solve this joint layout (theirs is the KUKA iiwa; ours has the same zero
  offsets) by subproblem decomposition, and show that any definition of ψ is singular somewhere. Since 26 Oct
  `swivel_angle` uses their stereographic version, singular only with the shoulder-wrist line straight down into the
  base ([14](14_math_guide.md) §6); the old "angle from up" flipped 180° when the wrist passed over the shoulder.
  `analytic_ik_7dof` returns up to 8 solutions per ψ (§6 Step 4), `feasible_psi` the ψ that fit the limits, and
  `solve_ik(..., psi=)` holds the elbow at a chosen ψ; when least squares fails, `solve_ik` now falls back to the
  closed form at the ψ nearest the current one. Straight-line moves (`cartesian.linear_path`) now plan ψ for the
  whole line first (`elbow.plan_arm_angle`: dynamic programming over a ψ grid, scored by joint motion, limit margin
  and capsule clearance, [14](14_math_guide.md) §6 Step 5) and fall back to warm-started IK when no plan exists.

### Analytic IK in 6-DOF mode (J3 locked; spherical wrist, Pieper): implemented
In the 7-DOF numbering used everywhere else (`analytic_ik_6dof`, derivation in [14](14_math_guide.md) §4):
1. Wrist center **p_w = p_flange − d7·z_flange** (after removing the tool offset).
2. **θ1** = atan2(p_w,y, p_w,x) (front/back solutions).
3. **θ2, θ4**: 2-link planar law of cosines (two elbow configurations), with θ3 = 0.
4. **R_4^7 = (R_0^4)ᵀ R_flange** → **θ5, θ6, θ7** as ZYZ Euler angles (wrist flip / no flip). **Singular at θ6 = 0.**
5. Pick the closest valid solution of up to 8.

Both solvers are kept: analytic (fast, exact) for 6-DOF mode and as a source of seeds, numerical DLS for 7-DOF,
joint-lock and constraint modes. FK(IK(x)) = x is tested on random poses (`software/tests/test_ik.py`).

### Tool center point
Each tool reports its ID over CAN, and software loads `software/tools/<id>.yaml` (TCP offset, mass, COM)
so IK and gravity checks use the correct tool. **Not implemented yet:** today the tool is the `[tool]` offset and the
gripper entry in the `[[masses]]` list of `arm.toml`.

## Motion profiles and dynamics in the sim (implemented)

| Feature | Module | What it gives you |
|---|---|---|
| Trapezoid / **S-curve** (jerk-limited double-S) / **time-optimal TOPP-RA** (velocity, acceleration and torque limits) | `trajectory.py`, `topp.py` | Pick a profile per move in the twin. TOPP-RA uses MuJoCo inverse dynamics with the payload and counterbalance, so it slows down exactly where the arm is heavy |
| **Self-collision** (7 capsules, 10 checked pairs) + desk guard | `collision.py`, `ik.py` | Moves that fold the tool into the shoulder or swing the forearm into the base are refused before they run |
| **Itemised mass model** | `kinematics.py`, `arm.toml` `[[masses]]` | One list of lumps (motor + driver + gearbox per joint, links, tool) feeds the URDF, MuJoCo, `gravity_torque`, the spring sizing and `tools/torque_budget.py`. 9.77 kg total, from the CAD and the slicer |
| **Stepper torque-speed curve** | `motor.py`, `physics.py`, `topp.py` | Each joint's torque limit falls with speed (voltage-limited, from datasheet R and L). The physics stalls a joint that is asked to lift too fast, the torque guard checks every sample against the torque left at that speed, and TOPP-RA slows down exactly where the motor runs out |
| **Gearbox compliance + backlash** (two-mass joints) | `physics.py` | Links sag τ/k + backlash/2 behind the motors (~20 mm at the tool, stretched out with 1 kg and the spring, with the placeholder stiffnesses); the first mode is ~3.6 Hz with 1 kg (21 Oct, CAD masses; 3.7 Hz with the 7 Oct estimates, 5.3 Hz empty) at ζ ≈ 0.05, so the arm rings after fast moves. `read()` = output encoder, `read_motor()` = motor encoder: the setup for dual-loop control, which `control.py` (`DualLoop`, 14 Oct) now closes |
| **J2 counterbalance** (zero-length spring) | `counterbalance.py` | Sized by minimax over the reachable workspace and 0-1 kg: ~1500 N/m with the CAD's anchor 70 mm up and lug 98 mm along link 2 and the CAD masses (574 N/m at 100 / 150 mm with the hand estimates) more than halves J2's worst-case holding torque (16.5 → 6.9 N·m with the 26 Oct masses; deferred, D41) |

## Calibration
- Joint zero offsets after homing (printed jig or digital angle gauge).
- Backlash per joint from output encoder vs motor encoder.
- Repeatability: 50 approaches to one point, measured with a dial indicator or camera + AprilTag.
- Camera intrinsics + hand-eye extrinsics (checkerboard/ChArUco).

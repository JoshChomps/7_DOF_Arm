# 11: Controls & Vision Track

You want to go into **controls**, so this arm should double as a controls lab. Each item below
is tied to a phase, produces a plot or number you can show, and builds on the previous one.
Items marked (P) make good portfolio pieces. The math of everything already implemented (IK, trajectories,
TOPP-RA, the motor and gearbox models) is written out in the [math guide](14_math_guide.md).

## Level 1: Motor & joint control (Oct-Dec)

| Exercise | Hardware | Output |
|---|---|---|
| **FOC from scratch** on the SimpleFOC testbed (gimbal motor + AS5600): open-loop → current loop → velocity → position cascade | SimpleFOC Mini (~$25 kit) | Step responses, tuned gains |
| (P) **Frequency response / bandwidth**: chirp input, measure a Bode plot of the velocity loop | Testbed | Bode plot, a bandwidth number |
| (P) **Impedance control**: make the motor behave like a virtual spring-damper you can feel by hand | Testbed | Video + stiffness/damping sweep |
| Stepper joint tuning via the MKS driver parameters (current, PID, accel) | Test bench | Tracking error vs speed |
| (P) **Torque-speed curve**: hang a known load on a drum, raise it at increasing speeds until the driver stalls. Compare with the voltage-limit model in `deskarm.motor` and set `[drive] torque_derate` | Test bench | Measured vs modelled curve, a derate number |
| (P) **Gearbox characterization**: backlash, stiffness, friction (Coulomb + viscous), efficiency, cycloidal vs planetary. The sim already models stiffness + backlash (`deskarm.physics`), so the measured numbers drop straight into `arm.toml` | Test bench + output encoder | The Phase 1 comparison report |

## Level 2: Arm-level control (Dec-Feb)

| Exercise | Output |
|---|---|
| **Dual-loop control**: motor encoder inner loop (MKS) + output encoder outer loop (your code), with backlash compensation. Done in sim, 14 Oct: `deskarm.control.DualLoop` (feed-forward of the gear sag + integral on the link), math in [14](14_math_guide.md) §19; next: on the bench with the MT6701 | Repeatability before/after |
| **Trajectory generation**: trapezoidal → jerk-limited S-curve, multi-axis sync, then (P) **time-optimal path parameterization (TOPP-RA)** with joint velocity/accel/torque limits. Done in sim (`deskarm.trajectory`, `deskarm.topp`); next: validate on the real arm, compare against the `toppra` library | Plots of q, q̇, q̈, time saved |
| **Jacobian tools**: singularity detection (the twin warns at the wrist, elbow and shoulder singularities; in 7-DOF mode only some are real, [23](23_singularities_and_ccd.md)), manipulability ellipsoids drawn in Viser | Visualization |
| **Contact detection from gear wind-up**: external torque on J2-J4 from the motor/link encoder difference (momentum observer). Done in sim 15 Oct: `deskarm.control.ContactGuard`, math in [14](14_math_guide.md) §20; next: identify $k$ and the masses on the bench | Smallest force it detects, false-trip rate |
| (P) **Constrained / weighted IK** (joint lock, keep-tool-vertical), null-space projection | The joint-lock feature from [05](05_software_kinematics.md) |
| (P) **Redundancy resolution (7-DOF)**: null-space joint-limit avoidance, manipulability maximization, elbow-swivel control, **analytic arm-angle IK** (Elias & Wen 2024: stereographic ψ, already in `swivel_angle`) | Plots of ψ vs joint limits, before/after comparisons |
| **Dynamics model**: gravity torques, feasibility checks for trajectories. Started in `deskarm.physics` (MuJoCo inverse dynamics + physics sim with speed-dependent torque limits, masses from `arm.toml` `[[masses]]`); next: fit it to measured masses/currents | Model vs measured motor current |
| **System ID** of link inertias/friction from logged trajectories (least squares) | Identified parameters, validation fit |

## Level 3: Vision + control (Feb-Mar), where your vision background helps

| Exercise | Output |
|---|---|
| Camera intrinsics + (P) **hand-eye calibration** (eye-in-hand wrist cam or eye-to-hand desk cam) | Reprojection error, calibrated transform |
| AprilTag / ArUco pose → pick-and-place | Demo video |
| (P) **Visual servoing**: PBVS first, then IBVS (image-space features → joint velocities through the Jacobian) | Convergence plots |
| **Hand mirroring**: MediaPipe → filter → IK → arm, with latency measurement and filter tuning | Demo, latency budget |
| Object detection (YOLO-class) → grasp selection | Demo |

## Level 4: Advanced / post-April

| Exercise | Notes |
|---|---|
| **MPC** for joint-space tracking (CasADi/acados) running in sim, then on the real arm | Compare against PID + feed-forward |
| **Custom FOC joint driver** (see [10](10_drivers_and_custom_pcbs.md) board E) → torque-controlled joint → (P) **computed-torque / impedance control on a real arm joint** | The biggest controls payoff |
| BLDC QDD v2 actuator (GIM6010-8 or custom) on J2/J4, enabling collision detection from current | Safe to touch, backdrivable |
| **Learning**: LeRobot (ACT / diffusion policy) using teleop demos from hand mirroring | Imitation learning demo |
| Sim-to-real: tune the MuJoCo model to match logged real data | Model-fidelity study |

## Tooling
- Python: `numpy`, `scipy`, `pinocchio`, `mujoco`, `viser`, `python-can`, `opencv-python`, `mediapipe`, `pytest`.
- Logging: every run writes a CSV/Parquet log (commanded, measured, current, temperature) so plots are easy.
  Keep plotting notebooks in `software/analysis/`.
- Keep a **controls log** in `docs/log/` with plots. That becomes your portfolio.

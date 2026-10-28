# Actuator bench: procedure and results

**Status: template. Nothing has been measured yet.** Fill in the tables as the tests are run
(roadmap weeks 4-6, [docs/08](../docs/08_roadmap.md)). Each result replaces a placeholder in
[`software/deskarm/data/arm.toml`](../software/deskarm/data/arm.toml), so the simulator and
`tools/torque_budget.py` stop running on estimates.

## Setup

- Motor + MKS driver on a rigid plate, gearbox under test bolted on, a **lever arm of known length**
  (e.g. 200 mm, hole for a hook) on the output. For J2 (26 Oct): `stl/bench/C2_Stand.stl` holds the module by its
  tabs, output up, and `C2_Lever.stl` is a 105 mm lever with a hook hole at R 88 ([24](../docs/24_first_prints_and_j2_bench.md)).
  Tests done by hand before the motors arrive go in the tables marked "hand, no motor".
- **Output encoder**: the bench MT6701 on the output shaft (through-rod, docs/16 §3.5), wired to the base controller
  (GPIO10/12/13, firmware built with `BENCH_ENCODER_JOINT` = the joint number; firmware README).
- **Driver set up first** with `python -m deskarm.drivers --channel COM5` (`--assign`, `--calibrate` unloaded,
  `--write`, then `--jog` to find `motor_sign`; software README).
- Weights or the luggage scale for torque (torque = force × lever length; 1 kg at 200 mm = 1.96 N·m).
- Dial indicator on the lever as a cross-check of the output encoder.
- Current-limited bench supply for first power-up, then the 24 V PSU.
- Log every run to CSV: time, commanded angle, motor angle (driver), output angle, supply current, motor temperature.

## 0. Driver behaviour (do this first, one motor, no gearbox)

| Question | How | Result |
|---|---|---|
| Does a position command over CAN move the motor? (CANable first, then the ESP32-S3 bridge) | `python -m deskarm.bench --channel COM5 --joint J2 --hold` (a one-joint `CanBackend`), or the frames in `deskarm.mks` by hand | |
| Does the speed field mean motor rpm at MStep 16? | Command 60 rpm for 10 s, count turns from `31H` | |
| **Streamed setpoints:** send a new absolute target every 10 ms / 20 ms along a slow sine. Smooth, or stop-start? | `deskarm.bench --sine 10 --period 4 --dt 0.02 -o sine.csv` (and `--dt 0.01`); look at the motor column's velocity and at the `t` column (Windows timing jitter) | |
| If stop-start: does speed mode with the position loop in Python track the same sine? | `CanBackend(..., mode="speed")` (F6, written 13 Oct, sim only) | |
| Encoder read rate achievable with 1 node / 2 nodes at 500 kbit/s and 1 Mbit/s | Count replies per second | |
| Supply voltage range of the driver (the manual says 12-24 V: confirm on the label) | | |
| What the driver reports after power is cut and restored (position kept or lost?) | Read `30H` and `31H`, power-cycle, turn the shaft by hand, read again. Expect `30H` value = the same angle within the turn (absolute), `31H` back at 0. The coarse + fine start-up (docs/14 §16) depends on the first | |

## 1. Backlash and stiffness (push test)

Motor enabled and holding. Hang torque +τ on the lever, read the output encoder; move the weight to the other
side (−τ), read again. Repeat for 3-4 torque levels up to the joint's static load.

    hysteresis = 2 τ / k + backlash        (fit a line: slope 2/k, intercept = backlash)

This is the same test as `software/tests/test_physics.py::test_backlash_and_stiffness_from_a_push_test`. Run it with
`python -m deskarm.bench --channel COM5 --joint J2 --push 2 4 6 8 -o push.csv`: it prompts for each weight and fits
k and b ([14](../docs/14_math_guide.md) §17). Expect a few per cent on k from the MT6701's 0.022° step.

| Gearbox | Material | Backlash (deg) | Stiffness k (N·m/rad) | `arm.toml` placeholder | Notes |
|---|---|---|---|---|---|
| 2:1 belt + 24:1 cycloid, R = 50 mm (J2) | PLA | | | 0.5°, 600 | |
| same | PETG | | | | |
| 2:1 belt + 24:1 cycloid, R = 36 mm (J3/J4) | PETG | | | 0.5°, 300 | |
| 2-stage planetary ~25:1 (comparison) | PETG | | | | |
| 2-stage planetary 22.2:1 (J5/J6), output stage 18 mm face | PETG | | | 0.8°, 120 | |
| 3:1 belt + planetary ~5:1 = 15:1 (J7) | PETG | | | 0.6°, 60 | |

Repeat the J2 cycloid after ~1,000 load cycles: printed drives loosen with wear.

### 1b. Output encoder calibration (added 14 Oct)

`python -m deskarm.encoders --channel COM5 --joint J2` drives the joint slowly (5°/s) to −115°, up to +115° and back,
logging the MT6701 against the motor-implied angle, and fits the sensor's zero, direction and INL plus the lost motion
([14](../docs/14_math_guide.md) §18). Unloaded on the bench, so no stiffness comes out of it (the push test gives that).
Check: the INL harmonics should be under ~1° (datasheet v1.9: ±1.0° typical aligned, ±1.5° max; much more means the magnet is off-centre or too
far), the lost motion should roughly match the push test's backlash (friction adds a little), and the residual should be near the 0.006° the 14-bit
step alone leaves. A residual that repeats every 7.5° (one motor turn) or 15° (one cycloid input turn) is the drive's
transmission error: worth recording. If the
lever stays on, take it off or keep it level: a load that changes along the sweep goes into the harmonics.

| Gearbox | Zero (deg) | INL 1st / 2nd / 3rd (deg) | Lost motion (deg) | Residual rms (deg) | Notes |
|---|---|---|---|---|---|
| J2 bench, PLA | | | | | |

## 2. Torque vs speed (sets `[drive] torque_derate`)

Lift a known weight on a drum (or the lever through a fixed arc) at increasing speeds until the driver reports a stall.
Record the highest speed that still lifts each load.

| Motor | Load at the joint (N·m) | Max joint speed (°/s) | Motor rpm | Model (`deskarm.motor`, derate 1.0) | Measured / model |
|---|---|---|---|---|---|
| NEMA23 76 mm, 48:1 | 8 | | | | |
| | 16 | | | | |
| | 24 | | | | |
| NEMA17 60 mm, 48:1 | 4 | | | | |
| | 7 | | | | |
| | 10 | | | | |

The lowest "measured / model" ratio is the derate to use (0.75 is assumed today). Also write the motor's datasheet
`resistance` and `inductance_mh` into `arm.toml`: the model depends on them.

## 3. Efficiency and backdriving

| Gearbox | Motor current to lift τ (A) | Current to lower τ (A) | Efficiency estimate | `arm.toml` placeholder | Backdrives unpowered at (N·m) |
|---|---|---|---|---|---|
| belt + cycloid (J2) | | | | 0.63 | |
| belt + cycloid (J3/J4) | | | | 0.63 | |
| planetary 22.2:1 | | | | 0.72 | |

Efficiency ≈ (τ / ratio) / (motor torque from current), using the motor's torque constant; the lift/lower pair
separates friction from load. "Backdrives unpowered" tells you whether the arm holds its pose after an E-stop.

## 4. Heat

Hold the rated static load for 20 minutes at the working current, then at 70% hold current.

| Motor | Hold current | Case temperature after 20 min (°C) | Housing temperature at the mount (°C) |
|---|---|---|---|
| NEMA23 76 mm | 100% | | |
| | 70% | | |
| NEMA17 60 mm | 100% | | |
| | 70% | | |

PETG softens at ~70-80 °C. If the mount goes above ~55 °C, lower the hold current, add standoffs, or print that part in ASA.

## 5. Mass

Weigh each finished actuator (motor + driver + gearbox + fasteners) and replace its lumps in `arm.toml` `[[masses]]`.
"CAD" is the `arm.toml` value since 20 Oct (catalogue motors and drivers, slicer grams for the prints, bearings and
pins as point masses); the hand estimates of 7-8 Oct are kept for comparison.

| Actuator | Hand estimate (kg) | CAD, 26 Oct (kg) | Measured (kg) |
|---|---|---|---|
| J2 motor (LKD57HS76) + driver | 1.19 | 1.19 | |
| J2 belt stage + cycloid module + output side | 1.00 | 1.01 + 0.22 | |
| J3 / J4 actuator (motor + driver, module, output side) | 0.95 | 0.55 + 0.66 + 0.16 = 1.37 | |
| J5 / J6 actuator (LDO 40 mm motor + driver, module, output side) | 0.51 / 0.53 | 0.33 + 0.28 + 0.04 = 0.65 | |
| J7 actuator (LDO 20 mm pancake + driver, module, output side) | 0.37 (with slip ring) | 0.18 + 0.22 + 0.04 = 0.44 | |

Then re-run:

    python3 tools/torque_budget.py
    python -m deskarm.counterbalance

## Decision

| Question | Answer | Evidence |
|---|---|---|
| Cycloidal or planetary for J2-J4? | | |
| Is 48:1 enough for J3/J4, or 2.5:1 belt (60:1)? | | |
| PETG or PETG-CF discs? | | |
| Streamed position or speed-mode control? | | |

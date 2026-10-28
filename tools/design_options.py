#!/usr/bin/env python3
"""
Design options: what the arm's mass costs, and what each way of cutting it buys.

    python3 tools/design_options.py                 # comparison table, rated payload
    python3 tools/design_options.py --payload-kg 0.5

Each option is a small edit of software/deskarm/data/arm.toml (a motor moved toward the shoulder, a
shorter arm, the heavier wrist the baseline replaced...) applied in memory, then run through the same torque budget
as tools/torque_budget.py. Nothing is written back: the table is for deciding, arm.toml stays the
baseline until you change it. The reasoning behind each option is in docs/15_mass_and_design_options.md.

Columns
-------
total / moved   arm mass, and the part of it the shoulder (J2) has to lift
J2, J4 static   worst-case gravity torque with the payload (no counterbalance)
J2, J4 margin   torque at standstill / design torque (design = static x 1.5 x 1.3); >= 1 passes
J2, J4 speed    fastest the joint can turn while still holding static x 1.3 (power-limited)
weakest         the joint with the smallest margin
"""
import argparse
import copy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "software"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import torque_budget as tb  # noqa: E402
from deskarm.kinematics import arm_from_config, load_config  # noqa: E402
from deskarm.motor import stepper_torque  # noqa: E402

# Catalogue motors: holding torque N*m, A, ohm, mH, rotor inertia kg*m^2, mass kg. The LDO ones are the
# motors bought on 8 Oct (Spool3D listing values); the StepperOnline ones are kept for comparisons.
MOTORS = {
    "LDO-42STH60-2004AC (60 mm)": (0.85, 2.0, 2.1, 4.1, 1.02e-5, 0.50),
    "LDO-42STH40-1684AC (40 mm)": (0.43, 1.68, 1.65, 3.6, 5.3e-6, 0.28),
    "LDO-42STH20-1004ASH (20 mm pancake)": (0.18, 1.0, 7.2, 7.0, 2.14e-6, 0.127),
    "17HS24-2104S (60 mm)": (0.65, 2.1, 1.6, 3.0, 8.2e-6, 0.50),
    "17HS19-2004S1 (48 mm)": (0.59, 2.0, 1.4, 3.0, 6.8e-6, 0.40),
    "17HS15-1504S (39 mm)": (0.45, 1.5, 2.3, 4.4, 5.4e-6, 0.28),
    "17HS08-1004S (20 mm pancake)": (0.16, 1.0, 3.7, 4.5, 1.5e-6, 0.15),
}


def joint(cfg, name):
    return next(j for j in cfg["joints"] if j["name"] == name)


def lump(cfg, prefix):
    return next(m for m in cfg["masses"] if m["name"].startswith(prefix))


def set_motor(cfg, joint_name, motor, ratio=None):
    """Swap a joint's motor (and optionally its ratio); the actuator lump changes by the mass difference."""
    hold, amps, ohm, mh, rotor, mass = MOTORS[motor]
    j = joint(cfg, joint_name)
    old_mass = min(MOTORS.values(), key=lambda m: abs(m[0] - j["hold_torque"]))[5]   # nearest catalogue motor
    j.update(hold_torque=hold, rated_current=amps, resistance=ohm, inductance_mh=mh, rotor_inertia=rotor)
    if ratio:
        j["ratio"] = ratio
    lump(cfg, f"{joint_name} motor")["mass"] += mass - old_mass


def split_motor_back(cfg, joint_name, new_z, carrier):
    """Move the joint's motor + driver lump to ``new_z`` on the centreline, carried by ``carrier`` (belt-driven)."""
    mo = lump(cfg, f"{joint_name} motor")
    mo.update(name=mo["name"] + " (moved back, belt-driven)", on=carrier, pos=[0.0, 0.0, new_z])


# ---------------------------------------------------------------- the options ----
def baseline(cfg):
    pass


def heavy_wrist(cfg):
    """The rev. 2 wrist that option B replaced (7 Oct): 48 mm motors on J5/J6 at 20:1, the 39 mm motor on J7 at 10:1."""
    set_motor(cfg, "J5", "17HS19-2004S1 (48 mm)", ratio=20)
    set_motor(cfg, "J6", "17HS19-2004S1 (48 mm)", ratio=20)
    set_motor(cfg, "J7", "17HS15-1504S (39 mm)", ratio=10)


def motors_back(cfg):
    """PAROL6-style: J4's motor near the shoulder and J6's motor at the elbow end of the forearm."""
    split_motor_back(cfg, "J4", 0.22, "J3")           # was beside the forearm (z 0.469)
    split_motor_back(cfg, "J6", 0.39, "J5")           # the 40 mm motor + driver, was beside the wrist (z 0.579)


def six_dof(cfg):
    """No J3 actuator: the joint becomes a plain printed link section (structure only)."""
    lump(cfg, "J3 motor")["mass"] = 0.0
    lump(cfg, "J3 cycloid module")["mass"] = 0.15


def shorter_arm(cfg, scale=0.885):
    """PAROL6 reach: 520 mm -> 460 mm to the TCP (links 168 / 159 / 62 mm)."""
    z0 = joint(cfg, "J1")["d"]
    for name in ("J3", "J5", "J7"):
        joint(cfg, name)["d"] *= scale
    cfg["tool"]["tcp_offset_z"] *= scale
    for m in cfg["masses"]:
        if m["on"] not in ("base", "J1"):
            m["pos"][2] = z0 + (m["pos"][2] - z0) * scale
    cb = cfg.get("counterbalance")
    if cb:
        cb["attach_distance"] *= scale


def back_and_short(cfg):
    motors_back(cfg)
    shorter_arm(cfg)


OPTIONS = [
    ("A  baseline (arm.toml)", baseline, 0),
    ("B  rev. 2 heavy wrist (dropped)", heavy_wrist, +3),
    ("C  J4 + J6 motors moved back", motors_back, +6),
    ("E  6-DOF (no J3 actuator)", six_dof, -50),
    ("F  shorter arm (460 mm reach)", shorter_arm, 0),
    ("G  C + F", back_and_short, +6),
]


def loaded_speed(arm, k, torque):
    """Fastest joint speed (deg/s) at which joint k still delivers ``torque`` (N*m)."""
    j = arm.joints[k]
    v = np.linspace(0.0, np.deg2rad(400.0), 4001)
    t = arm.drive.torque_derate * j.ratio * j.efficiency * stepper_torque(
        v * j.ratio, j.hold_torque, j.rated_current, j.resistance, j.inductance,
        arm.drive.voltage, arm.drive.pole_pairs)
    ok = np.flatnonzero(t >= torque)
    return float(np.rad2deg(v[ok[-1]])) if ok.size else 0.0


def evaluate(cfg, payload):
    arm = arm_from_config(cfg)
    static, design, at0, _, _ = tb.budget(arm, payload)
    margin = at0 / design
    weakest = int(np.argmin(margin[1:])) + 1
    return {
        "total": arm.total_mass,
        "moved": sum(m.mass for m in arm.masses if m.link >= 1),
        "static": static, "design": design, "margin": margin, "weakest": arm.joints[weakest].name,
        "speed": [loaded_speed(arm, k, static[k] * tb.SAFETY_FACTOR) for k in range(arm.n)],
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description="Compare design options against the arm.toml baseline")
    ap.add_argument("--payload-kg", type=float, default=tb.PAYLOAD_KG)
    args = ap.parse_args(argv)
    base = load_config()
    print(f"Payload {args.payload_kg} kg at the TCP. Margin = torque at standstill / design torque "
          f"(static x {tb.DYNAMIC_FACTOR} x {tb.SAFETY_FACTOR}).")
    print("Speed = fastest the joint turns while still holding static x "
          f"{tb.SAFETY_FACTOR} (no counterbalance).\n")
    head = (f"{'Option':<32}{'total':>7}{'moved':>7}  {'J2 stat':>7}{'J4 stat':>8}"
            f"  {'J2 marg':>7}{'J4 marg':>8}{'J6 marg':>8}  {'J2 d/s':>6}{'J4 d/s':>7}{'J6 d/s':>7}"
            f"  {'weakest':>7}{'USD':>6}")
    print(head)
    print("-" * len(head))
    results = []
    for name, edit, cost in OPTIONS:
        cfg = copy.deepcopy(base)
        edit(cfg)
        r = evaluate(cfg, args.payload_kg)
        results.append((name, r))
        s, m, v = r["static"], r["margin"], r["speed"]
        print(f"{name:<32}{r['total']:>6.2f} {r['moved']:>6.2f}  {s[1]:>7.1f}{s[3]:>8.1f}"
              f"  {m[1]:>7.2f}{m[3]:>8.2f}{m[5]:>8.2f}  {v[1]:>6.0f}{v[3]:>7.0f}{v[5]:>7.0f}"
              f"  {r['weakest']:>7}{cost:>+6d}")
    print("\nkg, N*m, deg/s. USD = rough change in the core BOM. A margin below 1.00 means that joint needs a")
    print("higher ratio (or a bigger motor) in that option. Speeds above ~200 deg/s are capped by vmax anyway.")
    return results


if __name__ == "__main__":
    main()

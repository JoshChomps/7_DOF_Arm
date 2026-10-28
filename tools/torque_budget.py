#!/usr/bin/env python3
"""
Torque budget: what every joint has to hold, and what its drive can deliver.

    python3 tools/torque_budget.py                    # rated payload from the CONFIG below
    python3 tools/torque_budget.py --payload-kg 0.5
    python3 tools/torque_budget.py --config my_arm.toml

Everything comes from ONE place, software/deskarm/data/arm.toml: the [[masses]] list (motors,
drivers, gearboxes, printed links, tool), the link lengths, and each joint's motor + ratio. The
physics sim, the counterbalance sizing and this script therefore always agree. Edit arm.toml as
the design changes (replace the mass estimates with CAD numbers), then re-run this.

Needs numpy (pip install -e "software[tools]").

What the columns mean
---------------------
static    worst-case gravity torque on the joint with the payload, over every combination of
          the joints at -90 / 0 / +90 deg (the arm stretched out sideways, forearm held
          horizontal by the roll joints, and so on), then refined one joint at a time (the CAD's
          lumps sit off the centreline, so the worst pose can fall between). No counterbalance:
          a snapped cable must not drop the arm.
design    static x DYNAMIC_FACTOR x SAFETY_FACTOR: torque to accelerate that load with margin.
          J1 carries no gravity load, so its design torque is inertia x its acceleration limit.
at 0      what the drive delivers near standstill: derate x holding torque x ratio x efficiency.
at vmax   what is left at the joint's speed limit (steppers lose torque with speed, deskarm.motor).
A joint passes if   at 0 >= design   and   at vmax >= static x SAFETY_FACTOR
(it can accelerate the load from rest, and still carry it at full speed).

The math behind the numbers: docs/14_math_guide.md §10 (gravity torque, budget) and §11 (torque vs speed).
"""
import argparse
import itertools
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "software"))

import numpy as np  # noqa: E402

from deskarm.counterbalance import spring_torque  # noqa: E402
from deskarm.kinematics import G, load_arm  # noqa: E402

# ---------------------------------------------------------------- CONFIG ----
PAYLOAD_KG = 1.0          # rated payload at FULL horizontal reach (PAROL6-class)
HEAVY_PAYLOAD_KG = 1.5    # stretch payload: margins are reported for it too
DYNAMIC_FACTOR = 1.5      # acceleration headroom on top of static gravity load
SAFETY_FACTOR = 1.3       # printed-part / estimate uncertainty
ROLL_OFFSET_M = 0.030     # payload centre of mass this far off the tool axis (loads J7)
BOARD_HALF_WIDTH_M = 0.25 # arm at the centre of a 500 x 500 mm base board
BOARD_AND_PSU_KG = 3.7    # 18 mm plywood (~2.7 kg) + PSU (~1 kg)
# ---------------------------------------------------------------------------

STRETCHED = np.deg2rad([0, 90, 0, 0, 0, 0, 0])


def worst_static(arm, payload_kg, sweeps=2, steps=19, seeds=6):
    """Largest |gravity torque| per joint, and the pose that causes it: the -90/0/+90 deg grid, then for each joint
    rounds of scanning J2-J6 one at a time (full range, then +-5 and +-1 deg) from its few worst grid poses. The CAD's
    lumps sit off the centreline, so the arm isn't symmetric and the worst pose falls between grid points."""
    choices = []
    for j in arm.joints:
        opts = [a for a in np.deg2rad([-90.0, 0.0, 90.0]) if j.lower - 1e-9 <= a <= j.upper + 1e-9]
        choices.append(opts or [0.0])
    choices[0] = [0.0]                                   # base yaw doesn't change gravity loads
    choices[-1] = [0.0]                                  # nor does spinning the tool
    grid = [np.array(q) for q in itertools.product(*choices)]
    taus = np.array([np.abs(arm.gravity_torque(q, payload_kg)) for q in grid])
    worst, pose = taus.max(axis=0), np.array([grid[i] for i in taus.argmax(axis=0)])
    lo, hi = arm.lower, arm.upper
    for k in range(1, arm.n - 1):
        for start in np.argsort(-taus[:, k])[:seeds]:
            q, best = grid[start].copy(), taus[start, k]
            for r in range(sweeps + 10):                 # full range, then +-5 deg, then +-1 deg
                w = None if r < sweeps else (0.087 if r < sweeps + 6 else 0.0175)
                for i in range(1, arm.n - 1):
                    span = (lo[i], hi[i]) if w is None else (max(lo[i], q[i] - w), min(hi[i], q[i] + w))
                    for a in np.linspace(*span, steps if w is None else 11):
                        trial = q.copy()
                        trial[i] = a
                        t = abs(arm.gravity_torque(trial, payload_kg)[k])
                        if t > best + 1e-12:
                            best, q = t, trial
            if best > worst[k]:
                worst[k], pose[k] = best, q
    return worst, pose


def yaw_inertia(arm, payload_kg):
    """Moment of inertia about the J1 axis with the arm stretched out (kg*m^2), incl. the rotor."""
    L = arm.link_frames(STRETCHED)
    inertia = arm.joints[0].armature
    for (m, com, _), T in zip(arm.link_inertials(), L):
        p = (T @ np.append(com, 1.0))[:3]
        inertia += m * (p[0] ** 2 + p[1] ** 2)
    tcp = arm.fk(STRETCHED)[:3, 3]
    return inertia + payload_kg * (tcp[0] ** 2 + tcp[1] ** 2)


def budget(arm, payload_kg):
    """Per joint: static, design, limit near standstill, limit at vmax (all N*m)."""
    static, pose = worst_static(arm, payload_kg)
    link7 = arm.link_inertials()[-1][0]
    static[-1] = max(static[-1], (link7 + payload_kg) * G * ROLL_OFFSET_M)
    design = static * DYNAMIC_FACTOR * SAFETY_FACTOR
    design[0] = yaw_inertia(arm, payload_kg) * arm.amax[0] * SAFETY_FACTOR
    return static, design, arm.torque_limit(), arm.torque_limit(arm.vmax), pose


def main(argv=None):
    ap = argparse.ArgumentParser(description="Per-joint torque budget from arm.toml")
    ap.add_argument("--config", help="arm TOML (default: software/deskarm/data/arm.toml)")
    ap.add_argument("--payload-kg", type=float, default=PAYLOAD_KG)
    args = ap.parse_args(argv)
    arm = load_arm(args.config)
    payload = args.payload_kg

    print("Masses (arm.toml [[masses]], from the CAD and the slicer):")
    for m in arm.masses:
        on = "base" if m.link < 0 else arm.joints[m.link].name
        print(f"  {m.mass:5.2f} kg  on {on:<4}  z = {m.pos[2] * 1000:4.0f} mm  {m.name}")
    past_j2 = sum(m.mass for m in arm.masses if m.link >= 1)
    reach = float(np.linalg.norm(arm.fk(STRETCHED)[:3, 3] - arm.frames(STRETCHED)[1][:3, 3]))
    print(f"  total {arm.total_mass:.2f} kg, of which {past_j2:.2f} kg is past the shoulder (J2 lifts it)")
    print(f"\nReach J2 -> TCP: {reach * 1000:.0f} mm  |  payload {payload} kg at the TCP  |  "
          f"supply {arm.drive.voltage:.0f} V, torque derate {arm.drive.torque_derate}\n")

    static, design, at0, atv, _ = budget(arm, payload)
    print(f"{'Joint':<27}{'static':>8}{'design':>8}{'at 0':>8}{'at vmax':>9}   "
          f"{'at 0 / design':>13}  {'at vmax / hold':>14}")
    print("-" * 96)
    low = []
    for i, j in enumerate(arm.joints):
        hold = static[i] * SAFETY_FACTOR
        m_design = at0[i] / design[i]
        m_speed = atv[i] / hold if hold > 1e-9 else float("inf")
        ok = m_design >= 1.0 and m_speed >= 1.0
        if not ok:
            low.append(j.name)
        speed = f"{m_speed:14.2f}" if np.isfinite(m_speed) else f"{'-':>14}"
        print(f"{j.name + ' ' + j.description.split(' (')[0]:<27}{static[i]:>8.2f}{design[i]:>8.2f}{at0[i]:>8.2f}"
              f"{atv[i]:>9.2f}   {m_design:>13.2f}  {speed}  {'ok' if ok else 'LOW'}")
    print(f"\n(design = static x {DYNAMIC_FACTOR} x {SAFETY_FACTOR}; J1 design = yaw inertia x its "
          f"acceleration limit x {SAFETY_FACTOR}; hold = static x {SAFETY_FACTOR})")
    print("UNDER-POWERED: " + ", ".join(low) if low else "Every joint passes with the rated payload.")

    print(f"\n{'Drive':<8}{'motor':>9}{'ratio':>7}{'eff':>6}{'vmax':>9}{'motor rpm':>11}"
          f"{'ratio for design':>18}{'power at vmax':>15}")
    for i, j in enumerate(arm.joints):
        rpm = j.vmax * j.ratio * 60 / (2 * np.pi)
        need = design[i] / (j.motor_torque * j.efficiency) if j.motor_torque > 0 else float("nan")
        print(f"{j.name:<8}{j.hold_torque:>7.2f}Nm{j.ratio:>7.0f}{j.efficiency:>6.2f}"
              f"{np.rad2deg(j.vmax):>6.0f}d/s{rpm:>11.0f}{need:>16.1f}:1{atv[i] * j.vmax:>13.1f} W")
    print("  A stepper on a fixed supply makes roughly constant POWER above its corner speed, so a higher")
    print("  ratio buys torque near standstill but not speed under load. More voltage would, but the MKS drivers stop at 24 V.")

    if payload < HEAVY_PAYLOAD_KG:
        s_h, _, _, _, _ = budget(arm, HEAVY_PAYLOAD_KG)
        worst = [(arm.joints[i].name, at0[i] / s_h[i]) for i in range(1, arm.n)]
        name, margin = min(worst, key=lambda kv: kv[1])
        verdict = ("every joint still holds it" if margin >= SAFETY_FACTOR
                   else f"{name} holds it with less than the {SAFETY_FACTOR}x safety factor" if margin >= 1.0
                   else f"{name} cannot hold it")
        print(f"\n{HEAVY_PAYLOAD_KG} kg payload, worst pose: {verdict}. Torque at standstill / static load:")
        print("  " + ", ".join(f"{n} {m:.2f}x" for n, m in worst) +
              f"  (the rated-payload design asks for {DYNAMIC_FACTOR * SAFETY_FACTOR:.2f}x, so move it slowly).")

    j2 = abs(arm.gravity_torque(STRETCHED, payload)[1])
    if arm.counterbalance is not None:
        with_spring = abs(arm.gravity_torque(STRETCHED, payload)[1] - spring_torque(arm, STRETCHED)[1])
        empty = arm.gravity_torque(STRETCHED, 0.0)[1] - spring_torque(arm, STRETCHED)[1]
        print(f"\nJ2 counterbalance (k = {arm.counterbalance.stiffness:.0f} N/m): arm stretched out, "
              f"{payload} kg: {j2:.1f} -> {with_spring:.1f} N*m; empty gripper: {abs(empty):.1f} N*m.")
        print("  Worst case over the whole workspace and how k was chosen: python -m deskarm.counterbalance")
        print("  The budget above is WITHOUT the spring, so a snapped cable can't drop the arm.")

    need_static = j2 / (G * BOARD_HALF_WIDTH_M)
    need_dynamic = need_static * DYNAMIC_FACTOR
    have = arm.total_mass + BOARD_AND_PSU_KG
    print(f"\nTipping (arm at the centre of a {2 * BOARD_HALF_WIDTH_M * 1000:.0f} mm board): needs "
          f">= {need_static:.1f} kg total standing still, >= {need_dynamic:.1f} kg while accelerating, "
          f"{need_dynamic * SAFETY_FACTOR:.1f} kg with margin.")
    print(f"  Arm {arm.total_mass:.1f} kg + board and PSU {BOARD_AND_PSU_KG:.1f} kg = {have:.1f} kg"
          + (": enough." if have >= need_dynamic * SAFETY_FACTOR
             else ": add C-clamps or a weight under the board."))

    print("\nNotes:")
    print(" * Ratios > ~50:1 in one printed stage get lossy and non-backdrivable: use a belt pre-stage.")
    print(" * Ratios < ~10:1 are 'quasi direct drive' territory (BLDC + FOC).")
    return low


if __name__ == "__main__":
    main()

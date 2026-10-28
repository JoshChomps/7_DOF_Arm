#!/usr/bin/env python3
"""Re-measure the sim numbers docs/14 quotes (§11 first mode, §19 Steps 1-4) from the current arm.toml (docs/21 §7).

    python tools/sim_modes.py

Motors only (no dual loop), 1 kg: hold the pose, read the steady link error on J2 and J4; then step J2 by 0.3 deg and
take the ringing frequency and damping from the J2 link angle (log decrement over three periods).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "software"))
from deskarm import load_arm  # noqa: E402
from deskarm.physics import MujocoBackend  # noqa: E402

DT = 0.002


def run(arm, q, payload=1.0, step_deg=0.3, settle=2.0, ring=3.0):
    be = MujocoBackend(arm, q0=q, payload_kg=payload)
    be.enable()
    be.command(q)
    for _ in range(int(settle / DT)):
        be.step(DT)
    err = np.degrees(q - be.read())
    target = q + np.radians(step_deg) * np.eye(arm.n)[1]
    be.command(target)
    th = []
    for _ in range(int(ring / DT)):
        be.step(DT)
        th.append(be.read()[1])
    x = np.array(th) - np.mean(th[len(th) // 2:])
    peaks = [i for i in range(1, len(x) - 1) if x[i] > x[i - 1] and x[i] >= x[i + 1] and x[i] > 0]
    f = (len(peaks[:4]) - 1) / ((peaks[3] - peaks[0]) * DT) if len(peaks) >= 4 else float("nan")
    zeta = np.log(x[peaks[0]] / x[peaks[3]]) / (2 * np.pi * 3) if len(peaks) >= 4 else float("nan")
    return err, f, zeta


def main():
    arm = load_arm()
    for name, q in (("stretched", np.radians([0, 90, 0, 0, 0, 0, 0])), ("home", arm.home)):
        err, f, zeta = run(arm, q)
        print(f"{name:9s} motors-only link error J2 {abs(err[1]):.3f} deg, J4 {abs(err[3]):.3f} deg; "
              f"J2 step rings at {f:.2f} Hz, zeta {zeta:.3f}")


if __name__ == "__main__":
    main()

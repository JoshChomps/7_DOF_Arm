"""Output encoders: raw MT6701 reading -> joint angle, and its calibration.

D33 puts an MT6701 on the output of J2-J4 (docs/16 §3.7). It reads one joint turn in 14 bits
(``bridge.JointAngle.turn``) from an arbitrary zero, either way round, with a slow error (its INL,
~±1°, ±1.5° max) that repeats once per joint turn. A ``Calibration`` maps it to the joint angle:

    s * 2 pi * turn = theta + c(theta)  (mod 2 pi),   c(theta) = c0 + sum_k a_k cos k theta + b_k sin k theta

``fit`` finds s and c from a slow sweep logged against the motor-implied angle, and on the way the lost
motion and the gearbox stiffness the sweep shows. ``sweep`` drives one joint through ``CanBackend`` to log
it; ``save``/``load`` keep calibrations by joint name in JSON. Math: docs/14_math_guide.md §18.

    python -m deskarm.encoders --channel COM5 --joint J2        # bench: sweep, fit, save
    python -m deskarm.encoders --sim --joint J2                 # the same against simulated drivers

Not yet run on hardware.
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

DEFAULT = Path.home() / ".deskarm" / "encoders.json"


def wrap(x):
    """Angle(s) into [-pi, pi)."""
    return (np.asarray(x) + np.pi) % (2 * np.pi) - np.pi


@dataclass
class Calibration:
    sign: int                           # +1 if the reading grows with the joint angle
    c: list[float]                      # c0, a1, b1, a2, b2, ... (rad)
    lost_motion: float = 0.0            # rad: gap between the two sweep directions (backlash when unloaded)
    stiffness: float | None = None      # N*m/rad, when the sweep had two different loads
    rms: float = 0.0                    # rad: what the fit leaves (transmission error, noise)

    def error(self, theta):
        """c(theta): what the reading has on top of the joint angle (rad)."""
        c = np.asarray(self.c, float)
        kt = np.multiply.outer(np.asarray(theta, float), np.arange(1, (len(c) + 1) // 2))
        return c[0] + np.cos(kt) @ c[1::2] + np.sin(kt) @ c[2::2]

    def angle(self, turn, near):
        """Joint angle (rad) from a raw reading (turns), the solution nearest ``near`` (within pi)."""
        x = self.sign * 2 * np.pi * np.asarray(turn, float)
        theta = np.asarray(near, float)
        for _ in range(4):                  # theta = x - c(theta), by fixed-point iteration
            theta = theta + wrap(x - self.error(theta) - theta)
        return theta


def fit(motor, turn, direction, torque=None, harmonics: int = 3) -> Calibration:
    """Calibration from slow sweeps. Per sample: ``motor`` = motor-implied joint angle (rad), ``turn`` = raw
    reading (turns), ``direction`` = +1/-1 the way the joint was moving, ``torque`` = the gearbox torque
    (N*m, holding: gravity minus spring), or None if it doesn't change. A torque that is only a function of
    the angle looks like the first harmonic: sweep at two poses with different loads. docs/14 §18.
    """
    m, x = np.asarray(motor, float), 2 * np.pi * np.asarray(turn, float)
    d = np.asarray(direction, float)
    r = [np.exp(1j * (s * x - m)).mean() for s in (1, -1)]           # right sign -> readings cluster
    sign = 1 if abs(r[0]) >= abs(r[1]) else -1
    c0 = np.angle(r[0] if sign == 1 else r[1])
    y = c0 + wrap(sign * x - m - c0)                                  # = c(theta) - (h/2) d - tau / k
    cols = [np.ones_like(m)] + [f(k * m) for k in range(1, harmonics + 1) for f in (np.cos, np.sin)]
    cols.append(-d)
    loaded = torque is not None and np.ptp(np.asarray(torque, float)) > 1e-6
    if loaded:
        cols.append(-np.asarray(torque, float))
    A = np.column_stack(cols)
    sv = np.linalg.svd(A / np.linalg.norm(A, axis=0), compute_uv=False)
    if sv[-1] < 1e-3 * sv[0]:
        raise ValueError("the sweep can't separate the sensor error from the load: sweep the whole range both "
                         "ways, at two poses with different loads (or pass torque=None)")
    p, *_ = np.linalg.lstsq(A, y, rcond=None)
    n = 1 + 2 * harmonics
    return Calibration(sign, p[:n].tolist(), lost_motion=float(2 * p[n]),
                       stiffness=float(1 / p[n + 1]) if loaded and p[n + 1] > 0 else None,
                       rms=float(np.sqrt(np.mean((y - A @ p) ** 2))))


def gearbox_torque(arm, q, j: int, payload_kg: float = 0.0) -> float:
    """Torque (N*m) joint ``j``'s gearbox carries holding the arm still at ``q``: gravity minus the spring."""
    from .counterbalance import spring_torque

    return float(arm.gravity_torque(q, payload_kg)[j] - spring_torque(arm, q)[j])


def sweep(backend, j: int, speed_deg_s: float = 5.0, margin_deg: float = 5.0, dt: float = 0.02,
          payload_kg: float = 0.0, clock=time.monotonic, sleep=time.sleep) -> dict:
    """Drive joint ``j`` to its lower limit, up to the upper and back (``margin_deg`` inside both) and log
    every new good output reading: {"motor", "turn", "direction", "torque"} as lists, ready for ``fit``.
    The other joints hold where they are. Slowly: the reading and the motor poll are not taken at the
    same instant, and that lag looks like lost motion (5 ms at 5°/s = 0.025° each way).
    """
    jt = backend.arm.joints[j]
    lo, hi = jt.lower + np.radians(margin_deg), jt.upper - np.radians(margin_deg)
    log = {"motor": [], "turn": [], "direction": [], "torque": []}
    v = np.radians(speed_deg_s)
    for goal, record in ((lo, False), (hi, True), (lo, True)):
        start, t0, next_t, last = backend.read_motor()[j], clock(), clock(), None
        sense = 1.0 if goal > start else -1.0
        while True:
            t = clock() - t0
            target = backend.read_motor().copy()
            target[j] = goal if (goal - start) * sense <= v * t else start + sense * v * t
            backend.command(target)
            backend.step(dt)
            if backend.faults:
                raise RuntimeError(f"sweep of {jt.name} stopped: {backend.faults}")
            q, r = backend.read_motor(), backend.output.get(j)
            if record and r is not None and r.good and r.seq != last:
                log["motor"].append(float(q[j]))
                log["turn"].append(r.turn)
                log["direction"].append(sense)
                log["torque"].append(gearbox_torque(backend.arm, q, j, payload_kg))
                last = r.seq
            if abs(q[j] - goal) < np.radians(0.05) or t > abs(goal - start) / v + 5:
                break
            next_t += dt
            sleep(max(0.0, next_t - clock()))
    return log


def load(path: str | Path = DEFAULT) -> dict[str, Calibration]:
    """Saved calibrations by joint name ({} if there is no file yet)."""
    try:
        return {k: Calibration(**v) for k, v in json.loads(Path(path).read_text(encoding="utf-8")).items()}
    except FileNotFoundError:
        return {}


def save(name: str, cal: Calibration, path: str | Path = DEFAULT) -> None:
    """Add or replace joint ``name``'s calibration, keeping the others."""
    path = Path(path)
    data = {k: asdict(v) for k, v in load(path).items()} | {name: asdict(cal)}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=1), encoding="utf-8")


def report(name: str, cal: Calibration) -> str:
    amp = np.hypot(cal.c[1::2], cal.c[2::2])
    return (f"{name}: sign {cal.sign:+d}, zero {np.degrees(cal.c[0]):.3f} deg, harmonics "
            + ", ".join(f"{np.degrees(a):.3f}" for a in amp) + " deg, lost motion "
            f"{np.degrees(cal.lost_motion):.3f} deg, stiffness "
            + ("-" if cal.stiffness is None else f"{cal.stiffness:.0f} N*m/rad")
            + f", residual {np.degrees(cal.rms):.3f} deg rms")


def main(argv=None):
    from . import mks
    from .backends import CanBackend
    from .bench import bench_arm

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--channel", help="the bridge's serial port (COM5, /dev/ttyACM0)")
    src.add_argument("--sim", action="store_true", help="simulated driver and encoder")
    ap.add_argument("--joint", default="J2")
    ap.add_argument("--speed", type=float, default=5.0, help="sweep speed, deg/s")
    ap.add_argument("--harmonics", type=int, default=3)
    ap.add_argument("-o", "--out", default=str(DEFAULT))
    a = ap.parse_args(argv)

    arm = bench_arm(a.joint)
    cid = int(a.joint[1:])
    clock, sleep = time.monotonic, time.sleep
    if a.sim:                                   # an encoder 2 rad off zero with a 0.46 deg INL, on a fake clock
        t = [0.0]
        clock, sleep = (lambda: t[0]), (lambda s: t.__setitem__(0, t[0] + s))
        bus = mks.SimBus([cid], clock=clock, encoders={cid: arm.joints[0].ratio},
                         encoder_error={cid: lambda th: 2.0 + 0.008 * np.sin(th + 0.4)})
    else:
        from .bridge import open_bus
        bus = open_bus(a.channel)
    be = CanBackend(arm, bus, [0.0], ids=[cid])
    be.enable()
    cal = fit(**sweep(be, 0, a.speed, clock=clock, sleep=sleep), harmonics=a.harmonics)
    be.estop()
    save(a.joint, cal, a.out)
    print(report(a.joint, cal), f"\nsaved to {a.out}")


if __name__ == "__main__":
    main()

"""Actuator bench: log one joint to CSV, and the push test for backlash and stiffness.

    python -m deskarm.bench --channel COM5 --joint J2 --sine 10 --period 4 --seconds 20 -o sine.csv
    python -m deskarm.bench --channel COM5 --joint J2 --hold --seconds 60 -o hold.csv
    python -m deskarm.bench --channel COM5 --joint J2 --push 2 4 6 8 -o push.csv
    python -m deskarm.bench --sim --joint J2 --push 2 4 6       # the same against simulated drivers

One driver on the bench, already set up with ``deskarm.drivers`` (its CAN id is the joint's: J2 = 2). The
output angle comes from the bench MT6701 on the base controller (firmware ``BENCH_ENCODER_JOINT`` = the
joint number; it arrives as frame 0x10 + id) and is logged raw: wiring the encoder into the control loop
is a later step. The run goes through ``CanBackend`` with a one-joint arm built from ``arm.toml``, so the
streamed-setpoint question (Q8, ``--sine``) is answered for the code the arm will run.

CSV columns: ``t`` (s), ``command``, ``motor`` (deg; motor encoder / ratio), ``output`` (deg from the output
encoder, relative to its first reading, unwrapped; blank without one), ``output_ok``, ``output_seq``.

``--push``: the motor holds still while you hang each torque on the lever, +τ then -τ. Each level gives one
hysteresis h = |deflection(+τ) - deflection(-τ)|, and the fit h = 2τ/k + b gives the gearbox stiffness k and
backlash b (docs/14_math_guide.md §17). Not yet run on hardware.
"""
from __future__ import annotations

import argparse
import csv
import time

import numpy as np

from . import mks
from .backends import CanBackend
from .kinematics import ArmModel, arm_from_config, load_config


def bench_arm(joint: str) -> ArmModel:
    """A one-joint arm with ``joint``'s drive data from arm.toml (no links, masses or spring)."""
    cfg = load_config()
    found = [j for j in cfg["joints"] if j["name"] == joint]
    if not found:
        raise ValueError(f"no joint {joint} in arm.toml")
    return arm_from_config({k: cfg[k] for k in ("drive", "tool") if k in cfg} | {"joints": found})


def push_fit(torque, hysteresis) -> tuple[float, float]:
    """(stiffness k in N*m/rad, backlash b in rad) from h = 2 tau / k + b, least squares. docs/14 §17."""
    slope, b = np.polyfit(np.asarray(torque, float), np.asarray(hysteresis, float), 1)
    return 2.0 / slope, b


class Output:
    """The output encoder's angle in rad, unwrapped from its first reading (None until a good frame arrives)."""

    def __init__(self):
        self.first = self.last = None
        self.angle = None

    def update(self, reading) -> float | None:
        if reading is None or not reading.good or reading.turn == self.last:
            return self.angle
        if self.first is None:
            self.first, self.angle = reading.turn, 0.0
        else:
            self.angle += ((reading.turn - self.last + 0.5) % 1.0 - 0.5) * 2 * np.pi
        self.last = reading.turn
        return self.angle


def run(backend: CanBackend, command, seconds: float, dt: float, writer=None, clock=time.monotonic,
        sleep=time.sleep, output: Output | None = None):
    """Step ``backend`` every ``dt`` for ``seconds``, commanding ``command(t)`` (rad, or None to hold).

    Returns the rows (t, command, motor, output, ok, seq) and writes them to ``writer`` (a csv.writer) too.
    """
    output = output or Output()
    rows, t0 = [], clock()
    next_t = t0
    while (t := clock() - t0) < seconds:
        c = command(t)
        if c is not None:
            backend.command(np.array([c]))
        backend.step(dt)
        r = backend.output.get(0)
        out = output.update(r)
        row = (round(t, 4), None if c is None else np.degrees(c), float(np.degrees(backend.read_motor()[0])),
               None if out is None else float(np.degrees(out)), bool(r and r.good), r.seq if r else None)
        rows.append(row)
        if writer:
            writer.writerow(row)
        if backend.faults:
            raise RuntimeError(f"stopped: {backend.faults}")
        next_t += dt
        sleep(max(0.0, next_t - clock()))
    return rows


def push_test(backend: CanBackend, levels, ask=lambda text, tau: input(text), settle: float = 1.0,
              average: float = 1.0, dt: float = 0.02, out=print, **run_kw):
    """Push test at each torque in ``levels`` (N*m at the joint). ``ask(text, tau)`` waits for the weight.

    Returns (k, b, rows) with rows = (tau, deflection at +tau, deflection at -tau, hysteresis), all in rad.
    Deflection = output encoder minus the motor-implied angle, so the servo's own give is taken out.
    """
    output, rows = Output(), []

    def deflection():
        run(backend, lambda t: None, settle, dt, output=output, **run_kw)
        got = run(backend, lambda t: None, average, dt, output=output, **run_kw)
        d = [o - m for _, _, m, o, ok, _ in got if o is not None and ok]
        if not d:
            raise RuntimeError("no good output-encoder readings: is the bench MT6701 connected and its frame on?")
        return np.radians(np.mean(d))

    run(backend, lambda t: None, settle, dt, output=output, **run_kw)       # first reading = zero
    for tau in levels:
        ask(f"Hang +{tau:g} N*m on the lever (positive direction), then press Enter", tau)
        plus = deflection()
        ask(f"Move it to -{tau:g} N*m (the other side), then press Enter", -tau)
        minus = deflection()
        rows.append((tau, plus, minus, abs(plus - minus)))
        out(f"  {tau:5.2f} N*m: hysteresis {np.degrees(abs(plus - minus)):.3f} deg")
    k, b = push_fit([r[0] for r in rows], [r[3] for r in rows])
    out(f"stiffness k = {k:.0f} N*m/rad, backlash b = {np.degrees(b):.3f} deg   (arm.toml gear_stiffness, backlash_deg)")
    return k, b, rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--joint", required=True, help="J1..J7: drive data from arm.toml, CAN id = its number")
    ap.add_argument("--channel", help="the bridge's serial port (COM5, /dev/ttyACM0)")
    ap.add_argument("--sim", action="store_true", help="simulated driver and output encoder")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sine", type=float, metavar="DEG", help="sine of this amplitude about the start angle")
    mode.add_argument("--hold", action="store_true", help="hold still and log")
    mode.add_argument("--push", type=float, nargs="+", metavar="NM", help="push test at these torques (N*m)")
    ap.add_argument("--period", type=float, default=4.0, help="sine period (s)")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--dt", type=float, default=0.02, help="step: one move + one poll per dt (s)")
    ap.add_argument("-o", "--out", help="CSV file")
    a = ap.parse_args(argv)
    if not a.sim and not a.channel:
        ap.error("give --channel (or --sim)")
    arm = bench_arm(a.joint)
    cid = int(a.joint[1:])
    if a.sim:
        bus = mks.SimBus([cid], encoders={cid: arm.joints[0].ratio * arm.joints[0].motor_sign})
    else:
        from .bridge import open_bus
        bus = open_bus(a.channel)
    try:
        backend = CanBackend(arm, bus, np.zeros(1), ids=[cid])
        backend.enable()
        f = open(a.out, "w", newline="") if a.out else None
        writer = csv.writer(f) if f else None
        try:
            if a.push:
                if writer:
                    writer.writerow(("tau_nm", "plus_deg", "minus_deg", "hysteresis_deg"))
                ask = lambda text, tau: input(text)                          # noqa: E731
                if a.sim:                                                    # a gearbox with arm.toml's k and b
                    j = arm.joints[0]
                    print(f"simulated gearbox: k = {j.gear_stiffness:g} N*m/rad, b = {np.degrees(j.backlash):g} deg")
                    ask = lambda text, tau: bus.deflection.__setitem__(      # noqa: E731
                        cid, tau / j.gear_stiffness + np.sign(tau) * j.backlash / 2)
                k, b, rows = push_test(backend, a.push, ask)
                for r in rows:
                    if writer:
                        writer.writerow((r[0], *np.degrees(r[1:])))
            else:
                if writer:
                    writer.writerow(("t", "command", "motor", "output", "output_ok", "output_seq"))
                amp, w = np.radians(a.sine or 0.0), 2 * np.pi / a.period
                command = (lambda t: amp * np.sin(w * t)) if a.sine else (lambda t: None)
                rows = run(backend, command, a.seconds, a.dt, writer)
                lag = [abs(c - m) for _, c, m, *_ in rows if c is not None]
                print(f"{len(rows)} rows" + (f", worst |command - motor| {max(lag):.2f} deg" if lag else ""))
        finally:
            if f:
                f.close()
            backend.estop()
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

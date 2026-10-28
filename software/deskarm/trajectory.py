"""Joint-space trajectories: synchronized trapezoidal and jerk-limited (S-curve) profiles.

Every joint starts and stops at the same time (the slowest joint sets the duration), which
is how industrial arms move point-to-point and keeps the tool path predictable. Time-optimal
(torque-aware) timing lives in ``deskarm.topp``; ``joint_move`` picks any of the three.

Math, with plots of both profiles: docs/14_math_guide.md §7.
"""
from __future__ import annotations

import numpy as np


class TrapezoidalTrajectory:
    """Point-to-point move from ``q0`` to ``q1`` within per-joint velocity/acceleration limits."""

    def __init__(self, q0, q1, vmax, amax, speed_scale: float = 1.0):
        self.q0 = np.asarray(q0, dtype=float)
        self.q1 = np.asarray(q1, dtype=float)
        vmax = np.asarray(vmax, dtype=float) * speed_scale
        amax = np.asarray(amax, dtype=float) * speed_scale
        dist = np.abs(self.q1 - self.q0)
        self.sign = np.sign(self.q1 - self.q0)

        # Minimum time per joint (triangular profile if v_max is never reached).
        t_min = np.where(dist * amax >= vmax**2,
                         dist / vmax + vmax / amax,
                         2 * np.sqrt(dist / amax))
        self.duration = float(t_min.max()) if dist.size else 0.0
        T = self.duration

        # Slow every joint down so all finish at T: solve d = v*T - v^2/a for cruise speed v.
        self.a = amax
        if T > 0:
            disc = np.maximum((amax * T) ** 2 - 4 * amax * dist, 0.0)
            self.v = (amax * T - np.sqrt(disc)) / 2
        else:
            self.v = np.zeros_like(dist)
        self.t_acc = np.divide(self.v, amax, out=np.zeros_like(self.v), where=amax > 0)

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """Joint position and velocity at time ``t`` (clamped to [0, duration])."""
        T = self.duration
        t = float(np.clip(t, 0.0, T))
        v, a, ta = self.v, self.a, self.t_acc
        pos = np.where(
            t < ta, 0.5 * a * t**2,
            np.where(t <= T - ta, v * t - 0.5 * v * ta,
                     v * (T - ta) - 0.5 * v * ta + v * (t - (T - ta)) - 0.5 * a * (t - (T - ta)) ** 2))
        vel = np.where(t < ta, a * t, np.where(t <= T - ta, v, v - a * (t - (T - ta))))
        if t >= T:
            return self.q1.copy(), np.zeros_like(self.q1)
        return self.q0 + self.sign * pos, self.sign * vel


class DoubleS:
    """Jerk-limited rest-to-rest profile ("double S", 7 segments) for a path parameter 0 -> h.

    Jerk is +-j or 0, so acceleration ramps instead of jumping. That is gentler on printed gears and
    belts and excites the arm's wobble modes less than a trapezoid (Biagiotti & Melchiorri,
    *Trajectory Planning for Automatic Machines and Robots*, 3.4).
    """

    def __init__(self, h: float, v: float, a: float, j: float):
        self.h = h = float(h)
        if h <= 0:
            self.tj = self.ta = self.tv = 0.0
            self.j = j
            self.duration = 0.0
            self._segs = []
            return
        if v * j >= a * a:                       # acceleration limit is reached
            tj, ta = a / j, a / j + v / a
        else:
            tj = np.sqrt(v / j)
            ta = 2 * tj
        tv = h / v - ta
        if tv < 0:                               # too short to cruise: peak velocity < v
            tv = 0.0
            tj = a / j
            ta = (tj + np.sqrt(tj * tj + 4 * h / a)) / 2
            if ta < 2 * tj:                      # ... and too short to reach full acceleration
                tj = (h / (2 * j)) ** (1 / 3)
                ta = 2 * tj
        self.tj, self.ta, self.tv, self.j = tj, ta, tv, j
        jerks = (j, 0.0, -j, 0.0, -j, 0.0, j)
        durs = (tj, ta - 2 * tj, tj, tv, tj, ta - 2 * tj, tj)
        self._segs = []                          # (t0, dur, jerk, s0, v0, a0)
        t = s = vel = acc = 0.0
        for jk, d in zip(jerks, durs):
            self._segs.append((t, d, jk, s, vel, acc))
            s += vel * d + acc * d * d / 2 + jk * d ** 3 / 6
            vel += acc * d + jk * d * d / 2
            acc += jk * d
            t += d
        self.duration = t
        self._scale = h / s if s > 0 else 1.0    # removes round-off so the profile ends exactly at h

    def sample(self, t: float) -> tuple[float, float, float]:
        """(s, sdot, sddot) at time t."""
        if self.duration <= 0:
            return self.h, 0.0, 0.0
        if t >= self.duration:
            return self.h, 0.0, 0.0
        t = max(t, 0.0)
        for t0, d, jk, s0, v0, a0 in reversed(self._segs):
            if t >= t0:
                tau = t - t0
                s = s0 + v0 * tau + a0 * tau ** 2 / 2 + jk * tau ** 3 / 6
                v = v0 + a0 * tau + jk * tau ** 2 / 2
                acc = a0 + jk * tau
                return s * self._scale, v * self._scale, acc * self._scale
        return 0.0, 0.0, 0.0


class SCurveTrajectory:
    """Point-to-point joint move with velocity, acceleration **and jerk** limits.

    All joints move along the straight line q0 -> q1 in joint space on one double-S profile, so
    they start and stop together. The profile is limited by whichever joint is tightest.
    Same interface as ``TrapezoidalTrajectory``.
    """

    def __init__(self, q0, q1, vmax, amax, jmax, speed_scale: float = 1.0):
        self.q0 = np.asarray(q0, dtype=float)
        self.q1 = np.asarray(q1, dtype=float)
        dq = np.abs(self.q1 - self.q0)
        moving = dq > 1e-12
        if not moving.any():
            self.profile = DoubleS(0.0, 1.0, 1.0, 1.0)
        else:
            lim = lambda x: float(np.min(np.asarray(x, dtype=float)[moving] * speed_scale / dq[moving]))  # noqa: E731
            self.profile = DoubleS(1.0, lim(vmax), lim(amax), lim(jmax))
        self.duration = self.profile.duration

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        s, sd, _ = self.profile.sample(t)
        return self.q0 + s * (self.q1 - self.q0), sd * (self.q1 - self.q0)

    def sample_acc(self, t: float) -> np.ndarray:
        return self.profile.sample(t)[2] * (self.q1 - self.q0)


def joint_move(arm, q0, q1, profile: str = "trapezoid", speed_scale: float = 1.0, torque_model=None):
    """Point-to-point joint move with the chosen profile: "trapezoid", "scurve" or "time-optimal"."""
    if profile == "trapezoid":
        return TrapezoidalTrajectory(q0, q1, arm.vmax, arm.amax, speed_scale=speed_scale)
    if profile == "scurve":
        return SCurveTrajectory(q0, q1, arm.vmax, arm.amax, arm.jmax, speed_scale=speed_scale)
    if profile == "time-optimal":
        from .topp import joint_path, time_optimal
        return time_optimal(arm, joint_path(q0, q1), speed_scale=speed_scale, torque_model=torque_model)[0]
    raise ValueError(f"unknown profile {profile!r}")

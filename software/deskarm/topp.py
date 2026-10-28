"""Time-optimal path parameterization (TOPP-RA): the fastest way to follow a fixed path.

Given a path q(s), s in [0, 1] (a joint move, or the IK solutions along a straight line), find the
fastest timing s(t) that respects every joint's velocity and acceleration limit and, with a
``TorqueModel``, every motor's **torque** limit (gravity, inertia, Coriolis, the payload and the
counterbalance spring included). The result runs at the limit of whichever constraint is active:
fast where the arm is light and folded, slower where it's stretched out holding a payload.

Method: reachability analysis, Pham & Pham, "A New Approach to Time-Optimal Path
Parameterization Based on Reachability Analysis", IEEE T-RO 2018 (https://github.com/hungpham2511/toppra).
With x = sdot^2 and u = sddot, along the path

    qdot = q'(s) sdot          qddot = q'(s) u + q''(s) x          tau = a(s) u + b(s) x + c(s)

so every limit is **linear** in (u, x) at each grid point. A backward pass computes, for each grid
point, the interval of x from which the end is still reachable (a small 2-variable LP per point,
solved here by enumerating vertices), then a forward pass greedily picks the largest feasible u.
No solver dependency; ~100 grid points, ~40 ms per move (~100 ms with torque limits) in Python.
It's exact up to the grid: an acceleration phase shorter than one grid cell costs about one
cell of extra time, and limits are enforced at the grid points, so between them a torque can
exceed the margin by a few percent (hence the default ``torque_margin`` of 0.95).

**Torque that depends on speed.** A stepper's torque falls with speed (``deskarm.motor``), which
makes the torque limit depend on sdot and breaks the "linear in (u, x)" structure. Two things
restore it. First, a speed cap at every grid point: no joint may run faster than the speed at which
its motor can still supply the static load there with ``speed_headroom`` to spare. Second, the
limits are frozen per grid point (at the faster end of each cell) and the problem is solved a few
times: limits taken from a faster profile are too tight (safe), limits from a slower one too loose,
so alternating brackets the answer and the last solve always uses limits from a profile at least as
fast as the result.

Accelerations are bang-bang (that's what time-optimal means), so jerk is unlimited: use the
S-curve profile when smoothness matters more than speed.

Step-by-step derivation, with a plot of a real solution: docs/14_math_guide.md §9.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .cartesian import WaypointTrajectory
from .kinematics import ArmModel


class ToppError(RuntimeError):
    pass


@dataclass
class ToppInfo:
    duration: float
    grid: int
    active: str             # which limit dominates: "velocity", "acceleration", "torque", "tool speed"


def _lp_x(A: np.ndarray, b: np.ndarray, maximize: bool) -> float | None:
    """max (or min) x over {(u, x): A @ [u, x] <= b} by vertex enumeration (None if infeasible)."""
    m = len(b)
    i, j = np.triu_indices(m, 1)
    a1, a2 = A[i], A[j]
    det = a1[:, 0] * a2[:, 1] - a1[:, 1] * a2[:, 0]
    ok = np.abs(det) > 1e-12
    i, j, a1, a2, det = i[ok], j[ok], a1[ok], a2[ok], det[ok]
    u = (b[i] * a2[:, 1] - a1[:, 1] * b[j]) / det
    x = (a1[:, 0] * b[j] - b[i] * a2[:, 0]) / det
    viol = A @ np.vstack([u, x]) - b[:, None]
    tol = 1e-9 * (1 + np.abs(b))[:, None]
    feas = np.all(viol <= tol, axis=0)
    if not feas.any():
        return None
    return float(x[feas].max() if maximize else x[feas].min())


def _u_range(A: np.ndarray, b: np.ndarray, x: float) -> tuple[float, float]:
    """Feasible u interval for fixed x."""
    lo, hi = -np.inf, np.inf
    au, rhs = A[:, 0], b - A[:, 1] * x
    pos, neg, zero = au > 1e-12, au < -1e-12, np.abs(au) <= 1e-12
    if pos.any():
        hi = float(np.min(rhs[pos] / au[pos]))
    if neg.any():
        lo = float(np.max(rhs[neg] / au[neg]))
    if zero.any() and np.any(rhs[zero] < -1e-9):
        return np.inf, -np.inf
    return lo, hi


def time_optimal(arm: ArmModel, path_qs: np.ndarray, *, speed_scale: float = 1.0, torque_model=None,
                 torque_margin: float = 0.95, sd_max: float | None = None, grid: int = 100,
                 accel_limits: bool = True, dt: float = 0.005,
                 speed_headroom: float = 1.15) -> tuple[WaypointTrajectory, ToppInfo]:
    """Fastest rest-to-rest timing along ``path_qs`` (shape (K, n), evenly spaced in s).

    ``torque_model`` (a ``deskarm.physics.TorqueModel``) adds torque limits, using
    ``torque_margin`` of what each motor can deliver **at the speed it is turning**.
    ``sd_max`` caps ds/dt (e.g. tool speed / path length).
    ``accel_limits=False`` drops the joint acceleration limits from ``arm.toml`` (needs a torque
    model): the motors' real torque then decides how hard each joint can accelerate, pose by pose.
    Returns a densely sampled ``WaypointTrajectory`` (every ``dt`` seconds).
    """
    path_qs = np.asarray(path_qs, dtype=float)
    K, n = path_qs.shape
    s_path = np.linspace(0.0, 1.0, K)
    N = max(2, grid)
    s = np.linspace(0.0, 1.0, N + 1)
    q = np.array([np.interp(s, s_path, path_qs[:, k]) for k in range(n)]).T
    if np.max(np.abs(q[-1] - q[0])) < 1e-9 and np.max(np.abs(np.diff(q, axis=0))) < 1e-9:
        return WaypointTrajectory([0.0, 1e-3], [q[0], q[-1]]), ToppInfo(1e-3, N, "none")
    ds = 1.0 / N
    qs = np.gradient(q, ds, axis=0)                       # q'(s)
    qss = np.gradient(qs, ds, axis=0)                     # q''(s)
    if K == 2:                                            # straight joint move: exact derivatives
        qss[:] = 0.0

    if not accel_limits and torque_model is None:
        raise ValueError("accel_limits=False needs a torque_model")
    vmax = arm.vmax * speed_scale
    amax = arm.amax * speed_scale if accel_limits else np.full(n, 1e4)
    big = 1e6

    # velocity bounds on x = sdot^2, and the dynamics tau = a u + b x + c at every grid point
    with np.errstate(divide="ignore"):
        xmax = np.min(np.where(np.abs(qs) > 1e-12, (vmax / np.abs(qs)) ** 2, big), axis=1)
    if sd_max is not None:
        xmax = np.minimum(xmax, sd_max ** 2)
    dyn = None
    limits_at = None
    if torque_model is not None:
        c = np.array([torque_model.inverse(q[i]) for i in range(N + 1)])
        a = np.array([torque_model.inverse(q[i], np.zeros(n), qs[i]) for i in range(N + 1)]) - c
        bq = np.array([torque_model.inverse(q[i], qs[i], qss[i]) for i in range(N + 1)]) - c
        dyn = (a, bq, c)
        fin = np.isfinite(torque_model.limits)
        lim0 = np.tile(torque_model.limits * torque_margin, (N + 1, 1))
        limits_at = getattr(torque_model, "limits_at", None)
        if limits_at is not None:
            # No joint faster than the speed at which its motor still covers the static load here
            # (with headroom to brake). Where it can barely hold, fall back to its corner speed.
            need = np.minimum(np.abs(c) * speed_headroom / torque_margin,
                              torque_model.limits * (1 - 1e-6))
            v_cap = arm.speed_at_torque(need)
            with np.errstate(divide="ignore", invalid="ignore"):
                x_cap = np.where(np.abs(qs) > 1e-12, (v_cap / np.abs(qs)) ** 2, big)
            xmax = np.minimum(xmax, np.min(np.where(np.isfinite(x_cap), x_cap, big), axis=1))

    def solve(tau_lim):
        """One TOPP-RA pass with per-grid-point torque limits. Returns x = sdot^2 on the grid."""
        A_list, b_list = [], []
        for i in range(N + 1):
            rows = [np.c_[qs[i], qss[i]], np.c_[-qs[i], -qss[i]]]
            rhs = [amax, amax]
            if dyn is not None:
                rows += [np.c_[a[i], bq[i]][fin], np.c_[-a[i], -bq[i]][fin]]
                rhs += [(tau_lim[i] - c[i])[fin], (tau_lim[i] + c[i])[fin]]
            A_list.append(np.vstack(rows + [np.array([[0.0, -1.0]]), np.array([[1.0, 0.0], [-1.0, 0.0]])]))
            b_list.append(np.concatenate(rhs + [np.array([0.0]), np.array([big, big])]))

        # backward pass: controllable sets K_i = [lo_i, hi_i]
        lo, hi = np.zeros(N + 1), np.zeros(N + 1)
        lo[N], hi[N] = 0.0, 0.0                               # come to rest at the end
        for i in range(N - 1, -1, -1):
            A, b = A_list[i], b_list[i]
            A2 = np.vstack([A, [[0.0, 1.0]], [[2 * ds, 1.0]], [[-2 * ds, -1.0]]])
            b2 = np.concatenate([b, [xmax[i]], [hi[i + 1]], [-lo[i + 1]]])
            h = _lp_x(A2, b2, maximize=True)
            if h is None:
                raise ToppError(f"no feasible timing at {100 * s[i]:.0f}% of the path "
                                f"(a joint can't hold the arm there even at rest)")
            hi[i] = h
            lo[i] = max(_lp_x(A2, b2, maximize=False) or 0.0, 0.0)
        if lo[0] > 1e-12:
            raise ToppError("can't start from rest on this path")

        # forward pass: greedy max acceleration while staying controllable
        x = np.zeros(N + 1)
        for i in range(N):
            A, b = A_list[i], b_list[i]
            ulo, uhi = _u_range(A, b, x[i])
            uhi = min(uhi, (hi[i + 1] - x[i]) / (2 * ds))
            ulo = max(ulo, (lo[i + 1] - x[i]) / (2 * ds))
            u = uhi if uhi >= ulo - 1e-9 else ulo
            x[i + 1] = float(np.clip(x[i] + 2 * ds * u, lo[i + 1], hi[i + 1]))
        x[-1] = 0.0
        return x

    def limits_for(x):
        """Torque limits at the joint speeds of profile x (limits never above the standstill ones).

        Grid point i governs the cell up to i+1, where the joint is already faster, so use the
        faster end: a motor near its corner speed loses torque within one cell.
        """
        xc = np.maximum(x, np.append(x[1:], x[-1]))
        v = np.abs(qs) * np.sqrt(np.maximum(xc, 0.0))[:, None]
        return np.minimum(limits_at(v) * torque_margin, lim0)

    tau_lim = None if dyn is None else lim0
    x = solve(tau_lim)
    if limits_at is not None:
        fast = x                                              # solved with limits that were too loose
        for last in (False, True):
            tau_lim = limits_for(fast)
            x = solve(tau_lim)                                # limits from a faster profile: safe
            if last or np.max(np.sqrt(fast) - np.sqrt(x)) <= 0.02 * max(np.sqrt(fast).max(), 1e-12):
                break
            fast = solve(limits_for(x))

    # time stamps at grid points (constant u per interval), then sample every dt
    sd = np.sqrt(np.maximum(x, 0.0))
    seg_t = 2 * ds / np.maximum(sd[:-1] + sd[1:], 1e-12)
    t_grid = np.concatenate([[0.0], np.cumsum(seg_t)])
    T = float(t_grid[-1])
    ts = np.linspace(0.0, T, max(2, int(np.ceil(T / dt)) + 1))
    k = np.clip(np.searchsorted(t_grid, ts, side="right") - 1, 0, N - 1)
    tau = ts - t_grid[k]
    u_seg = (x[1:] - x[:-1]) / (2 * ds)
    s_t = np.clip(s[k] + sd[k] * tau + 0.5 * u_seg[k] * tau ** 2, 0.0, 1.0)
    s_t[-1] = 1.0
    q_t = np.array([np.interp(s_t, s, q[:, j]) for j in range(n)]).T

    active = _active(arm, qs, qss, x, u_seg, vmax, amax, sd_max, torque_model, q, tau_lim, xmax)
    return WaypointTrajectory(ts, q_t), ToppInfo(T, N, active)


def _active(arm, qs, qss, x, u_seg, vmax, amax, sd_max, tm, q, tau_lim, xmax) -> str:
    """Which kind of limit the profile spends most of its time on (for the UI)."""
    counts = {"velocity": 0, "acceleration": 0, "torque": 0, "tool speed": 0}
    for i in range(len(u_seg)):
        sd = np.sqrt(x[i])
        if sd_max is not None and sd >= 0.98 * sd_max:
            counts["tool speed"] += 1
        elif np.max(np.abs(qs[i]) * sd / vmax) >= 0.98:
            counts["velocity"] += 1
        elif np.max(np.abs(qs[i] * u_seg[i] + qss[i] * x[i]) / amax) >= 0.98:
            counts["acceleration"] += 1
        elif tm is not None:
            tau = tm.inverse(q[i], qs[i] * sd, qs[i] * u_seg[i] + qss[i] * x[i])
            # at the torque limit, or at the speed where the motor only just carries the load
            if np.max(np.abs(tau) / tau_lim[i]) >= 0.97 or x[i] >= 0.96 * xmax[i]:
                counts["torque"] += 1
    best = max(counts, key=counts.get)
    return best if counts[best] else "none"


def joint_path(q0, q1, samples: int = 2) -> np.ndarray:
    """Straight line in joint space (the path of a joint move)."""
    q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
    return np.array([q0 + s * (q1 - q0) for s in np.linspace(0.0, 1.0, samples)])

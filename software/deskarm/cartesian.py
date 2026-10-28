"""Straight-line (Cartesian) moves: the tool travels along a straight line between two poses.

Position is interpolated linearly and orientation by SLERP. IK is solved every 2 mm / 0.02 rad along
the line (warm-started), using the same modes / locks / constraint tasks as the twin, and the path
is then timed with a trapezoidal, jerk-limited (S-curve) or time-optimal (TOPP-RA) profile. The result is a ``WaypointTrajectory`` (dense joint waypoints) that
plugs into everything that takes a trajectory: execution, the desk guard, the torque check.

In 7-DOF mode with a full pose and nothing locked, the elbow's arm angle along the whole line is chosen first
(``elbow.plan_arm_angle``: dynamic programming over psi, scored by joint motion, joint-limit margin and collision
clearance); otherwise, or if no plan exists, IK is warm-started point to point as before.

The move is re-timed so no joint exceeds its velocity or acceleration limit, and it is rejected
(``CartesianError``) if the line leaves the workspace or passes through a singularity where the
joints would have to jump.

Math: docs/14_math_guide.md §8 (pose interpolation, re-timing), §1 (rotation log / exp), §6 Step 5 (elbow plan).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .elbow import plan_arm_angle, plannable
from .ik import rotation_log, singularity_warnings, solve_ik
from .kinematics import ArmModel
from .trajectory import SCurveTrajectory, TrapezoidalTrajectory


class CartesianError(RuntimeError):
    pass


def rotation_exp(w: np.ndarray) -> np.ndarray:
    """Rotation vector -> rotation matrix (Rodrigues)."""
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def interpolate_pose(T0: np.ndarray, T1: np.ndarray, s: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (1 - s) * T0[:3, 3] + s * T1[:3, 3]
    T[:3, :3] = T0[:3, :3] @ rotation_exp(s * rotation_log(T0[:3, :3].T @ T1[:3, :3]))
    return T


class WaypointTrajectory:
    """Joint trajectory through dense waypoints (piecewise-linear between samples)."""

    def __init__(self, times: np.ndarray, qs: np.ndarray):
        self.times = np.asarray(times, dtype=float)
        self.qs = np.asarray(qs, dtype=float)
        self.duration = float(self.times[-1])

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        t = float(np.clip(t, 0.0, self.duration))
        if self.duration <= 0:
            return self.qs[-1].copy(), np.zeros(self.qs.shape[1])
        k = int(np.clip(np.searchsorted(self.times, t, side="right") - 1, 0, len(self.times) - 2))
        dt = self.times[k + 1] - self.times[k]
        a = (t - self.times[k]) / dt
        q = (1 - a) * self.qs[k] + a * self.qs[k + 1]
        qd = (self.qs[k + 1] - self.qs[k]) / dt
        if t >= self.duration:
            qd = np.zeros_like(qd)
        return q, qd


@dataclass
class LinearMoveInfo:
    length_m: float
    angle_rad: float
    duration: float
    slowed_by: float          # >1 if joint limits forced a slower move than the tool limits
    samples: int
    profile: str = "trapezoid"
    limited_by: str = ""      # time-optimal only: which limit dominates
    elbow: str = ""           # "planned" (arm angle chosen for the whole line) or "warm-started"


PROFILES = ("trapezoid", "scurve", "time-optimal")


def linear_path(arm: ArmModel, q_start: np.ndarray, T_goal: np.ndarray, *, mode: str = "7dof",
                locked=(), task: str = "pose", max_jump: float = 0.15, max_step_m: float = 0.002,
                max_step_rad: float = 0.02, elbow: str = "plan") -> tuple[np.ndarray, float, float, str]:
    """Joint configurations along the straight tool line, evenly spaced in path length.

    Returns (qs, length_m, angle_rad, how). Points are <= ``max_step_m`` / ``max_step_rad`` apart. With
    ``elbow="plan"`` (and 7-DOF mode, full pose, nothing locked) the arm angle is planned for the whole line
    (``how = "planned"``); otherwise, or if that finds no plan, IK is warm-started from the previous point so the arm
    keeps its configuration and elbow swivel (``how = "warm-started"``).
    """
    if elbow not in ("plan", "keep"):
        raise ValueError('elbow must be "plan" or "keep"')
    q_start = np.asarray(q_start, dtype=float)
    T0 = arm.fk(q_start)
    L = float(np.linalg.norm(T_goal[:3, 3] - T0[:3, 3]))
    th = float(np.linalg.norm(rotation_log(T0[:3, :3].T @ T_goal[:3, :3]))) if task == "pose" else 0.0
    n = max(2, int(np.ceil(L / max_step_m)) + 1, int(np.ceil(th / max_step_rad)) + 1)
    if elbow == "plan" and plannable(arm, mode, locked, task):
        plan = plan_arm_angle(arm, q_start, [interpolate_pose(T0, T_goal, s) for s in np.linspace(0.0, 1.0, n)],
                              max_jump=max_jump)
        if plan is not None:
            return plan.qs, L, th, "planned"
    qs = [q_start]
    q = q_start
    for s in np.linspace(0.0, 1.0, n)[1:]:
        T = interpolate_pose(T0, T_goal, s)
        if task != "pose":                               # position along the line, keep goal axis
            T[:3, :3] = T_goal[:3, :3]
        # tight tolerances: with the default 0.1 mm the first tiny steps wouldn't move the joints at all
        res = solve_ik(arm, T, q, mode=mode, locked=locked, task=task, centering=False,
                       pos_tol=1e-6, rot_tol=1e-5)
        if not res.success:
            why = "; ".join(singularity_warnings(arm, res.q)) or "out of reach, at a joint limit or a lock"
            raise CartesianError(f"no IK solution {s * 100:.0f}% along the line "
                                 f"(err {res.pos_err * 1000:.1f} mm): {why}")
        jump = float(np.max(np.abs(res.q - q)))
        if jump > max_jump:
            raise CartesianError(f"joints would jump {np.degrees(jump):.0f}° at {s * 100:.0f}% along "
                                 f"the line (singularity on the path); use a joint move instead")
        q = res.q
        qs.append(q)
    return np.array(qs), L, th, "warm-started"


def linear_move(arm: ArmModel, q_start: np.ndarray, T_goal: np.ndarray, *, v_lin: float = 0.10,
                w_ang: float = 1.0, a_lin: float = 0.4, a_ang: float = 3.0, j_lin: float = 4.0,
                j_ang: float = 30.0, speed_scale: float = 1.0, mode: str = "7dof", locked=(),
                task: str = "pose", profile: str = "trapezoid", torque_model=None, dt: float = 0.01,
                max_jump: float = 0.15, max_step_m: float = 0.002, max_step_rad: float = 0.02,
                elbow: str = "plan") -> tuple[WaypointTrajectory, LinearMoveInfo]:
    """Straight-line move of the TCP from FK(q_start) to ``T_goal``.

    Speeds: ``v_lin`` m/s and ``w_ang`` rad/s (times ``speed_scale``), and the matching
    acceleration (``a_*``) and jerk (``j_*``, S-curve only) limits. ``profile``:

    * ``"trapezoid"``: trapezoidal tool speed, then slowed uniformly until every joint is within
      its velocity / acceleration limits.
    * ``"scurve"``: the same with a jerk-limited (double-S) speed profile.
    * ``"time-optimal"``: TOPP-RA along the line: tool speed cap + joint velocity / acceleration
      limits (+ motor torque limits if ``torque_model`` is given), each enforced pose by pose.

    ``elbow``: "plan" chooses the arm angle for the whole line where it can (see :func:`linear_path`), "keep" always
    warm-starts point to point.

    Raises CartesianError if a point on the line has no IK solution or the joints would jump more
    than ``max_jump`` rad between samples (a singularity / configuration flip on the way).
    """
    if profile not in PROFILES:
        raise ValueError(f"profile must be one of {PROFILES}")
    q_start = np.asarray(q_start, dtype=float)
    T0 = arm.fk(q_start)
    if (np.linalg.norm(T_goal[:3, 3] - T0[:3, 3]) < 1e-6
            and (task != "pose" or np.linalg.norm(rotation_log(T0[:3, :3].T @ T_goal[:3, :3])) < 1e-6)):
        traj = WaypointTrajectory([0.0, 1e-3], [q_start, q_start])
        return traj, LinearMoveInfo(0.0, 0.0, traj.duration, 1.0, 2, profile)
    path, L, th, how = linear_path(arm, q_start, T_goal, mode=mode, locked=locked, task=task, max_jump=max_jump,
                                   max_step_m=max_step_m, max_step_rad=max_step_rad, elbow=elbow)
    eps = 1e-9
    # limits on the path parameter s in [0, 1] from the tool limits
    per = lambda lin, ang: min(lin * speed_scale / max(L, eps), ang * speed_scale / max(th, eps))  # noqa: E731
    vs, as_, js = per(v_lin, w_ang), per(a_lin, a_ang), per(j_lin, j_ang)

    if profile == "time-optimal":
        from .topp import time_optimal
        traj, ti = time_optimal(arm, path, speed_scale=speed_scale, torque_model=torque_model,
                                sd_max=vs, grid=min(max(len(path) - 1, 50), 200), dt=dt)
        return traj, LinearMoveInfo(L, th, traj.duration, 1.0, len(traj.times), profile, ti.active, how)

    prof = (TrapezoidalTrajectory([0.0], [1.0], [vs], [as_]) if profile == "trapezoid"
            else SCurveTrajectory([0.0], [1.0], [vs], [as_], [js]))
    n = max(2, int(np.ceil(prof.duration / dt)) + 1, len(path))
    times = np.linspace(0.0, prof.duration, n)
    s_path = np.linspace(0.0, 1.0, len(path))
    s_t = np.array([float(prof.sample(t)[0][0]) for t in times])
    qs = np.array([np.interp(s_t, s_path, path[:, k]) for k in range(arm.n)]).T

    # Re-time so every joint respects its velocity and acceleration limits.
    dts = np.diff(times)
    vel = np.abs(np.diff(qs, axis=0)) / dts[:, None]
    f_v = float(np.max(vel / (arm.vmax * speed_scale)))
    acc = np.abs(np.diff(np.diff(qs, axis=0) / dts[:, None], axis=0)) / dts[1:, None]
    f_a = float(np.sqrt(np.max(acc / (arm.amax * speed_scale)))) if len(acc) else 0.0
    slow = max(1.0, f_v, f_a)
    times = times * slow
    traj = WaypointTrajectory(times, qs)
    return traj, LinearMoveInfo(L, th, traj.duration, slow, len(times), profile, elbow=how)

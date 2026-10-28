"""Inverse kinematics.

Three solvers that share one API (all return joint vectors of length ``arm.n``):

* :func:`analytic_ik_6dof` - closed-form solution with J3 (upper-arm roll) locked at 0.
  The arm is then a classic 6-DOF anthropomorphic arm with a spherical wrist (Pieper).
* :func:`analytic_ik_7dof` - closed-form solution for all seven joints at a chosen arm angle psi.
* :func:`solve_dls` - damped-least-squares numerical IK. Any set of joints can be locked;
  an optional null-space objective runs in the redundant directions.
* :func:`swivel` - move the elbow around the shoulder-wrist axis (the "arm angle" psi) while
  the tool stays still. Only meaningful with all 7 joints free.

:func:`solve_ik` picks a solver for the twin's two modes ("7dof" and "6dof").

Math, with pictures: docs/14_math_guide.md §1 (rotation error), §4 (closed form), §6 Step 4 (7-DOF closed form), §5 (damped least
squares, locks, null space, tasks), §6 (swivel angle), §3 (singularities).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import ArmModel, rot_x, rot_z

J3 = 2  # index of the upper-arm roll joint (the redundant joint)


# --------------------------------------------------------------------------- helpers -------
def wrap_angle(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def rotation_log(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> rotation vector (axis * angle)."""
    cos = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos)
    vee = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if angle < 1e-9:
        return 0.5 * vee
    if np.pi - angle < 1e-6:
        B = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(B)))
        axis = B[:, i] / np.sqrt(max(B[i, i], 1e-12))
        return axis * angle
    return angle / (2.0 * np.sin(angle)) * vee


def pose_error(T_target: np.ndarray, T: np.ndarray) -> np.ndarray:
    """6-vector [position error; rotation error], both expressed in the base frame."""
    e_p = T_target[:3, 3] - T[:3, 3]
    e_r = rotation_log(T_target[:3, :3] @ T[:3, :3].T)
    return np.concatenate([e_p, e_r])


@dataclass
class IKResult:
    q: np.ndarray
    success: bool
    pos_err: float
    rot_err: float
    iterations: int = 0
    message: str = ""


def _result(arm: ArmModel, q: np.ndarray, T_target: np.ndarray, pos_tol: float, rot_tol: float,
            iterations: int = 0, message: str = "") -> IKResult:
    e = pose_error(T_target, arm.fk(q))
    pe, re = float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))
    return IKResult(q, pe < pos_tol and re < rot_tol, pe, re, iterations, message)


# --------------------------------------------------------------------------- analytic -------
_SRS_ALPHAS = np.deg2rad([-90, 90, -90, 90, -90, 90, 0])


def supports_analytic(arm: ArmModel) -> bool:
    """True if the arm has the zero-offset SRS structure the closed-form solver assumes."""
    if arm.n != 7:
        return False
    j = arm.joints
    ok_alpha = float(np.max(np.abs(np.array([x.alpha for x in j]) - _SRS_ALPHAS))) < 1e-9
    ok_a = all(abs(x.a) < 1e-12 and abs(x.offset) < 1e-12 for x in j)
    ok_d = all(abs(j[i].d) < 1e-12 for i in (1, 3, 5))
    return bool(ok_alpha and ok_a and ok_d)


def _zyz(M: np.ndarray, q5_ref: float) -> list[tuple[float, float, float]]:
    """All (q5, q6, q7) with Rz(q5) Ry(q6) Rz(q7) = M."""
    s6 = np.hypot(M[0, 2], M[1, 2])
    if s6 < 1e-9:  # wrist singular: J5 and J7 aligned, only their sum is defined
        q6 = 0.0 if M[2, 2] > 0 else np.pi
        if M[2, 2] > 0:
            total = np.arctan2(M[1, 0], M[0, 0])
            return [(q5_ref, q6, total - q5_ref)]
        diff = np.arctan2(-M[1, 0], -M[0, 0])  # q5 - q7 when q6 = pi
        return [(q5_ref, q6, q5_ref - diff)]
    q6 = np.arctan2(s6, M[2, 2])
    a = (np.arctan2(M[1, 2], M[0, 2]), q6, np.arctan2(M[2, 1], -M[2, 0]))
    b = (np.arctan2(-M[1, 2], -M[0, 2]), -q6, np.arctan2(-M[2, 1], M[2, 0]))
    return [a, b]


def _nearest_in_limits(arm: ArmModel, q: np.ndarray, q_ref: np.ndarray) -> np.ndarray | None:
    """Shift each angle by multiples of 2*pi to sit inside the limits, nearest to q_ref."""
    out = np.empty_like(q)
    for i, (qi, lo, hi, r) in enumerate(zip(q, arm.lower, arm.upper, q_ref)):
        candidates = [qi + k * 2 * np.pi for k in range(-2, 3)]
        candidates = [c for c in candidates if lo - 1e-9 <= c <= hi + 1e-9]
        if not candidates:
            return None
        out[i] = min(candidates, key=lambda c: abs(c - r))
    return out


def analytic_ik_6dof(arm: ArmModel, T_target: np.ndarray, q_ref: np.ndarray | None = None,
                     tol: float = 1e-6) -> list[np.ndarray]:
    """Closed-form IK with J3 = 0. Returns every valid solution, nearest to ``q_ref`` first.

    Wrist centre -> base yaw -> two-link triangle (law of cosines) -> Z-Y-Z wrist angles, up to
    2 x 2 x 2 = 8 solutions (docs/14_math_guide.md §4).
    """
    if not supports_analytic(arm):
        raise ValueError("arm geometry is not a zero-offset SRS arm; use solve_dls")
    q_ref = arm.home if q_ref is None else np.asarray(q_ref, float)
    d1, L1, L2, d7 = arm.joints[0].d, arm.joints[2].d, arm.joints[4].d, arm.joints[6].d

    T_flange = T_target @ np.linalg.inv(arm.tool)
    R = T_flange[:3, :3]
    p_w = T_flange[:3, 3] - d7 * R[:, 2]          # wrist centre
    v = p_w - np.array([0.0, 0.0, d1])            # shoulder -> wrist

    if np.hypot(v[0], v[1]) < 1e-9:               # wrist straight above/below the shoulder
        q1_options = [q_ref[0]]
    else:
        base = np.arctan2(v[1], v[0])
        q1_options = [base, wrap_angle(base + np.pi)]

    solutions = []
    for q1 in q1_options:
        rho = np.cos(q1) * v[0] + np.sin(q1) * v[1]
        h = v[2]
        D = (rho**2 + h**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(D) > 1 + 1e-9:
            continue                               # out of reach
        D = np.clip(D, -1.0, 1.0)
        for q4 in {np.arccos(D), -np.arccos(D)}:
            q2 = np.arctan2(rho, h) - np.arctan2(L2 * np.sin(q4), L1 + L2 * np.cos(q4))
            q_arm = np.array([q1, q2, 0.0, q4, 0.0, 0.0, 0.0])
            R04 = arm.frames(q_arm)[4][:3, :3]
            for q5, q6, q7 in _zyz(R04.T @ R, q_ref[4]):
                q = wrap_angle(np.array([q1, q2, 0.0, q4, q5, q6, q7]))
                q = _nearest_in_limits(arm, q, q_ref)
                if q is None:
                    continue
                e = pose_error(T_target, arm.fk(q))
                if np.linalg.norm(e) < tol * 10:   # guard against derivation/edge-case errors
                    solutions.append(q)

    unique: list[np.ndarray] = []
    for s in solutions:
        if not any(np.allclose(s, u, atol=1e-7) for u in unique):
            unique.append(s)
    unique.sort(key=lambda s: float(np.sum((s - q_ref) ** 2)))
    return unique


_RXM, _RXP = rot_x(-np.pi / 2), rot_x(np.pi / 2)


def analytic_ik_7dof(arm: ArmModel, T_target: np.ndarray, psi: float, q_ref: np.ndarray | None = None,
                     tol: float = 1e-6, verify: bool = True) -> list[np.ndarray]:
    """Closed-form IK for all seven joints at arm angle ``psi``. Every valid solution, nearest to ``q_ref`` first.

    Wrist centre -> elbow angle (law of cosines) -> elbow point on its circle at psi -> J1, J2 from the elbow
    direction -> J3 from the wrist seen from link 2 -> Z-Y-Z wrist, up to 2 x 2 x 2 = 8 solutions
    (docs/14_math_guide.md §6, Step 4; Elias & Wen 2024 for this layout). Empty when out of reach, when the elbow
    would be straight (psi undefined) or when nothing fits the joint limits. ``verify`` re-checks each solution
    with forward kinematics (a guard; the planner's psi grid skips it).
    """
    if not supports_analytic(arm):
        raise ValueError("arm geometry is not a zero-offset SRS arm; use solve_dls")
    q_ref = arm.home if q_ref is None else np.asarray(q_ref, float)
    d1, L1, L2, d7 = arm.joints[0].d, arm.joints[2].d, arm.joints[4].d, arm.joints[6].d

    T_flange = T_target @ np.linalg.inv(arm.tool)
    R = T_flange[:3, :3]
    S = np.array([0.0, 0.0, d1])
    W = T_flange[:3, 3] - d7 * R[:, 2]
    ell = np.linalg.norm(W - S)
    c4 = (ell**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if ell < 1e-9 or c4 > 1 - 1e-12 or c4 < -1 - 1e-9:
        return []                                   # out of reach, or straight elbow (psi undefined)
    u = (W - S) / ell
    x = sew_reference(u)
    if x is None:
        return []
    xc = (L1**2 - L2**2 + ell**2) / (2 * ell)       # circle centre distance from S, and radius
    r = np.sqrt(max(L1**2 - xc**2, 0.0))
    E = S + xc * u + r * (np.cos(psi) * x + np.sin(psi) * np.cross(u, x))
    z2 = (E - S) / L1                               # = (c1 s2, s1 s2, c2)

    solutions = []
    for sgn in (1.0, -1.0):                         # (q1, q2) and (q1 + pi, -q2) give the same z2
        q2 = sgn * np.arccos(np.clip(z2[2], -1.0, 1.0))
        q1 = np.arctan2(sgn * z2[1], sgn * z2[0]) if np.hypot(z2[0], z2[1]) > 1e-9 else q_ref[0]
        R02 = rot_z(q1) @ _RXM @ rot_z(q2) @ _RXP      # DH frames 1-2 (alphas -90, +90)
        w = R02.T @ (W - S)                         # = (L2 c3 s4, L2 s3 s4, L1 + L2 c4)
        for q4 in (np.arccos(np.clip(c4, -1, 1)), -np.arccos(np.clip(c4, -1, 1))):
            s4 = np.sign(np.sin(q4))
            q3 = np.arctan2(s4 * w[1], s4 * w[0])
            R04 = R02 @ rot_z(q3) @ _RXM @ rot_z(q4) @ _RXP
            for q5, q6, q7 in _zyz(R04.T @ R, q_ref[4]):
                q = _nearest_in_limits(arm, wrap_angle(np.array([q1, q2, q3, q4, q5, q6, q7])), q_ref)
                if q is not None and (not verify or np.linalg.norm(pose_error(T_target, arm.fk(q))) < tol * 10):
                    solutions.append(q)

    unique: list[np.ndarray] = []
    for sol in solutions:
        if not any(float(np.max(np.abs(sol - v))) < 1e-7 for v in unique):
            unique.append(sol)
    unique.sort(key=lambda sol: float(np.sum((sol - q_ref) ** 2)))
    return unique


def feasible_psi(arm: ArmModel, T_target: np.ndarray, step_deg: float = 2.0) -> np.ndarray:
    """Arm angles (rad, on a grid) at which ``T_target`` has a solution inside the joint limits."""
    grid = np.deg2rad(np.arange(-180.0, 180.0, step_deg))
    return np.array([p for p in grid if analytic_ik_7dof(arm, T_target, p)])


# --------------------------------------------------------------------------- numerical -----
def joint_centering(arm: ArmModel, gain: float = 0.005):
    """Null-space objective pushing joints away from their limits."""
    mid = (arm.lower + arm.upper) / 2
    half_range = np.maximum(arm.upper - arm.lower, 1e-6) / 2

    def f(q: np.ndarray) -> np.ndarray:
        return gain * (mid - q) / half_range**2
    return f


TASKS = ("pose", "position", "axis")


def axis_error(z: np.ndarray, z_target: np.ndarray) -> np.ndarray:
    """Rotation vector that turns unit vector ``z`` onto ``z_target`` (shortest arc)."""
    c = float(np.clip(z @ z_target, -1.0, 1.0))
    axis = np.cross(z, z_target)
    s = np.linalg.norm(axis)
    if s < 1e-12:
        if c > 0:
            return np.zeros(3)
        perp = np.cross(z, [1.0, 0.0, 0.0])          # opposite: any perpendicular axis, angle pi
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(z, [0.0, 1.0, 0.0])
        return perp / np.linalg.norm(perp) * np.pi
    return axis / s * np.arctan2(s, c)


def task_error(T_target: np.ndarray, T: np.ndarray, task: str = "pose") -> np.ndarray:
    """Error vector for a task: "pose" (6), "position" (3), "axis" (position + tool z-axis)."""
    e = pose_error(T_target, T)
    if task == "pose":
        return e
    if task == "position":
        return e[:3]
    if task == "axis":                              # orientation only up to spin about the tool axis
        return np.concatenate([e[:3], axis_error(T[:3, 2], T_target[:3, 2])])
    raise ValueError(f"unknown task {task!r}; use one of {TASKS}")


def task_jacobian(arm: ArmModel, q: np.ndarray, task: str = "pose") -> np.ndarray:
    J = arm.jacobian(q)
    if task == "pose":
        return J
    if task == "position":
        return J[:3]
    z = arm.fk(q)[:3, 2]
    P = np.eye(3) - np.outer(z, z)                  # spin about the tool axis doesn't matter
    return np.vstack([J[:3], P @ J[3:]])


def _task_result(arm, q, T_target, task, pos_tol, rot_tol, iterations, message) -> IKResult:
    e = task_error(T_target, arm.fk(q), task)
    pe = float(np.linalg.norm(e[:3]))
    re = float(np.linalg.norm(e[3:])) if e.size > 3 else 0.0
    return IKResult(q, pe < pos_tol and re < rot_tol, pe, re, iterations, message)


def solve_dls(arm: ArmModel, T_target: np.ndarray, q0: np.ndarray, locked=(),
              pos_tol: float = 1e-4, rot_tol: float = 1e-3, max_iter: int = 400,
              damping: float = 0.02, rot_weight: float = 0.3, max_step: float = 0.2,
              nullspace=None, task: str = "pose") -> IKResult:
    """Damped-least-squares IK. ``locked`` joints never move from their value in ``q0``.

    ``task`` picks what must match the target: "pose" (position + orientation), "position"
    (orientation free) or "axis" (position + tool z-axis direction, free spin about it).
    If the free joints can't satisfy the task, the weighted least-squares best effort is
    returned with ``success=False``.

    Each iteration: dq = J^T (J J^T + damping^2 I)^-1 e on the weighted, lock-reduced Jacobian,
    plus the null-space term (I - J# J) n(q) (docs/14_math_guide.md §5).
    """
    q = arm.clamp(np.array(q0, dtype=float))
    free = np.array([i not in set(locked) for i in range(arm.n)])
    prev = np.inf
    for it in range(max_iter):
        e = task_error(T_target, arm.fk(q), task)
        pe = np.linalg.norm(e[:3])
        re = np.linalg.norm(e[3:]) if e.size > 3 else 0.0
        if pe < pos_tol and re < rot_tol:
            return _task_result(arm, q, T_target, task, pos_tol, rot_tol, it, "converged")
        m = e.size
        W = np.diag([1.0] * 3 + [rot_weight] * (m - 3))
        Jw = W @ task_jacobian(arm, q, task)[:, free]
        ew = W @ e
        J_pinv = Jw.T @ np.linalg.solve(Jw @ Jw.T + damping**2 * np.eye(m), np.eye(m))
        dq = J_pinv @ ew
        if nullspace is not None:
            dq += (np.eye(int(free.sum())) - J_pinv @ Jw) @ nullspace(q)[free]
        norm = np.linalg.norm(dq)
        if norm > max_step:
            dq *= max_step / norm
        q[free] += dq
        q = arm.clamp(q)
        cost = float(ew @ ew)
        if abs(prev - cost) < 1e-16 and norm < 1e-10:
            break                                   # stalled (unreachable or at a limit)
        prev = cost
    return _task_result(arm, q, T_target, task, pos_tol, rot_tol, max_iter,
                        "did not converge (best effort)")


# --------------------------------------------------------------------------- workspace -----
def workspace_violations(arm: ArmModel, q: np.ndarray, z_min: float = 0.0,
                         margin: float = 0.03) -> list[str]:
    """Names of joints / the TCP that come within ``margin`` of the desk plane z = z_min.

    The arm is treated as straight segments between joint centres, so checking the joint centres
    and the TCP is enough to keep every segment above the plane (each segment lies between its ends).
    """
    F = arm.frames(q)
    out = []
    for i in range(2, arm.n + 1):                    # frames 0-1 sit on the base
        if F[i][2, 3] < z_min + margin:              # DH frame i sits on joint i+1's axis
            out.append(arm.joints[i].name if i < arm.n else "flange")
    if F[-1][2, 3] < z_min + margin and "TCP" not in out:
        out.append("TCP")
    return out


def trajectory_violation(arm: ArmModel, traj, samples: int = 80, **kw):
    """First (time, names) along a trajectory where the arm would hit the desk, else None."""
    for k in range(samples + 1):
        t = traj.duration * k / samples
        bad = workspace_violations(arm, traj.sample(t)[0], **kw)
        if bad:
            return t, bad
    return None


# --------------------------------------------------------------------------- swivel --------
# Stereographic SEW angle (Elias & Wen 2024): every arm-angle definition is singular somewhere (hairy ball), and this
# one is singular only when the shoulder-wrist line points along E_T, straight down into the base, out of reach.
E_T = np.array([0.0, 0.0, -1.0])
E_R = np.array([-1.0, 0.0, 0.0])                    # makes psi = 0 "elbow up" when reaching forward (+x)


def sew_reference(u: np.ndarray) -> np.ndarray | None:
    """Unit reference x on the elbow circle (perpendicular to the unit shoulder->wrist direction u), or None when u
    points along E_T. psi is measured from x, right-handed about u (docs/14_math_guide.md §6)."""
    k = np.cross(np.cross(u - E_T, E_R), u)
    n = np.linalg.norm(k)
    return None if n < 1e-9 else k / n


def swivel_angle(arm: ArmModel, q: np.ndarray) -> float:
    """Arm angle psi: rotation of the elbow about the shoulder->wrist axis (stereographic SEW angle).

    psi = 0 is elbow up for a forward reach and stays continuous over the top, where the wrist passes above the
    shoulder; measured right-handed about the shoulder->wrist direction. NaN when the elbow is straight or the
    shoulder->wrist line points straight down. Geometry and formula: docs/14_math_guide.md §6.
    """
    S, E, Wc = arm.srs_points(q)
    u = Wc - S
    n = np.linalg.norm(u)
    if n < 1e-9:
        return float("nan")
    u /= n
    x = sew_reference(u)
    e = (E - S) - ((E - S) @ u) * u
    if x is None or np.linalg.norm(e) < 1e-9:
        return float("nan")
    return float(np.arctan2(np.cross(u, x) @ e, x @ e))


def _swivel_gradient(arm: ArmModel, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    g = np.zeros(arm.n)
    for i in range(arm.n):
        dq = np.zeros(arm.n)
        dq[i] = eps
        g[i] = wrap_angle(swivel_angle(arm, q + dq) - swivel_angle(arm, q - dq)) / (2 * eps)
    return g


def swivel(arm: ArmModel, q: np.ndarray, psi_target: float, iters: int = 300,
           gain: float = 0.5, tol: float = 1e-3) -> IKResult:
    """Rotate the elbow to ``psi_target`` while holding the tool pose (null-space motion)."""
    q = np.array(q, dtype=float)
    T_hold = arm.fk(q)
    for _ in range(iters):
        psi = swivel_angle(arm, q)
        if np.isnan(psi):
            break
        d_psi = float(wrap_angle(psi_target - psi))
        if abs(d_psi) < tol:
            break
        g = _swivel_gradient(arm, q)
        dq0 = gain * d_psi * g / (g @ g + 1e-9)
        J = arm.jacobian(q)
        J_pinv = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), np.eye(6))
        dq = J_pinv @ pose_error(T_hold, arm.fk(q)) + (np.eye(arm.n) - J_pinv @ J) @ dq0
        norm = np.linalg.norm(dq)
        if norm > 0.1:
            dq *= 0.1 / norm
        q_new = arm.clamp(q + dq)
        if np.allclose(q_new, q, atol=1e-12):
            break                                   # blocked by a joint limit
        q = q_new
    res = solve_dls(arm, T_hold, q)                 # remove any drift in the tool pose
    psi = swivel_angle(arm, res.q)
    ok = res.success and not np.isnan(psi) and abs(wrap_angle(psi_target - psi)) < 10 * tol
    res.success = bool(ok)
    res.message = "swivel reached" if ok else "swivel limited (joint limits or singular elbow)"
    return res


# --------------------------------------------------------------------------- singularities -
def singularity_warnings(arm: ArmModel, q: np.ndarray, angle_tol_deg: float = 5.0,
                         shoulder_tol: float = 0.02) -> list[str]:
    """Human-readable warnings when the SRS arm is near one of its three classic singularities."""
    tol = np.deg2rad(angle_tol_deg)
    out = []
    if abs(np.sin(q[5])) < np.sin(tol):
        out.append("wrist singularity: J6 ≈ 0°, so J5 and J7 are aligned")
    if abs(q[3]) < tol:
        out.append("elbow singularity: arm almost straight (J4 ≈ 0°)")
    _, _, W = arm.srs_points(q)
    if np.hypot(W[0], W[1]) < shoulder_tol:
        out.append("shoulder singularity: wrist centre on the J1 axis")
    return out


# --------------------------------------------------------------------------- dispatcher ----
def solve_ik(arm: ArmModel, T_target: np.ndarray, q_ref: np.ndarray, mode: str = "7dof",
             locked=(), task: str = "pose", centering: bool = True, pos_tol: float = 1e-4,
             rot_tol: float = 1e-3, psi: float | None = None) -> IKResult:
    """Solve for the twin. ``mode`` is "7dof" (all joints) or "6dof" (J3 locked at 0);
    ``task`` is "pose", "position" or "axis" (see :func:`solve_dls`). ``centering`` adds the
    joint-limit-avoidance null-space objective; turn it off for smooth path following, where the
    minimum-norm joint motion is wanted. ``psi`` (7dof, full pose, nothing locked) asks for that arm angle
    exactly, from the closed form; otherwise, if damped least squares fails, the closed form is tried at arm
    angles moving out from ``q_ref``'s."""
    q_ref = np.asarray(q_ref, dtype=float)
    locked = set(locked)
    tols = {"pos_tol": pos_tol, "rot_tol": rot_tol}
    if mode == "6dof":
        q_seed = q_ref.copy()
        q_seed[J3] = 0.0
        if task == "pose" and supports_analytic(arm) and not (locked - {J3}):
            sols = analytic_ik_6dof(arm, T_target, q_seed)
            if sols:
                return _result(arm, sols[0], T_target, pos_tol, rot_tol, 0, "analytic")
            return _result(arm, q_seed, T_target, pos_tol, rot_tol, 0, "out of reach (analytic)")
        return solve_dls(arm, T_target, q_seed, locked=locked | {J3}, task=task, **tols)

    closed_form = task == "pose" and not locked and supports_analytic(arm)
    if psi is not None:
        if not closed_form:
            raise ValueError("psi needs mode 7dof, task 'pose', no locked joints and a zero-offset SRS arm")
        sols = analytic_ik_7dof(arm, T_target, psi, q_ref)
        if sols:
            return _result(arm, sols[0], T_target, pos_tol, rot_tol, 0, "analytic (psi)")
        return _result(arm, q_ref, T_target, pos_tol, rot_tol, 0, "no solution at this psi (reach, limits or straight elbow)")

    ns = joint_centering(arm) if centering else None
    res = solve_dls(arm, T_target, q_ref, locked=locked, nullspace=ns, task=task, **tols)
    if res.success or not closed_form:
        return res
    # Closed form at arm angles moving out from the current one: keeps the elbow where it was if it can.
    psi0 = swivel_angle(arm, q_ref)
    psi0 = 0.0 if np.isnan(psi0) else psi0
    for k in range(0, 181, 5):
        for p in {psi0 + np.deg2rad(k), psi0 - np.deg2rad(k)}:
            sols = analytic_ik_7dof(arm, T_target, float(wrap_angle(p)), q_ref)
            if sols:
                return _result(arm, sols[0], T_target, pos_tol, rot_tol, 0, "analytic (psi search)")
    return res
    # Restart from the closed-form 6-DOF solutions, which are good seeds far from q_ref.
    for seed in analytic_ik_6dof(arm, T_target, q_ref):
        r = solve_dls(arm, T_target, seed, nullspace=ns, **tols)
        if r.success:
            r.message = "converged (analytic seed)"
            return r
    return res

"""IK study: which singularities this arm really has, damping through them, and cyclic coordinate descent (docs/23).

    software/.venv/Scripts/python.exe tools/ik_study.py            # every part, ~1 min
    software/.venv/Scripts/python.exe tools/ik_study.py ccd        # one part: singular | path | ccd

Nothing here is used by the twin; it compares options for deskarm.ik. Math: docs/14_math_guide.md §21.
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software"))
from deskarm import load_arm, solve_dls                          # noqa: E402
from deskarm.ik import pose_error                                 # noqa: E402

W_ROT = 0.3                         # a radian of orientation counts as 0.3 m, as in solve_dls


def sigma_min(arm, q, locked=()):
    """Smallest singular value of the 6 x n pose Jacobian with the locked joints' columns removed."""
    free = [i for i in range(arm.n) if i not in set(locked)]
    J = np.diag([1, 1, 1, W_ROT, W_ROT, W_ROT]) @ arm.jacobian(q)[:, free]
    return float(np.linalg.svd(J, compute_uv=False)[-1])


# ------------------------------------------------------------------------- singular sets ----
def constructed(arm, rng, n=300):
    """Poses built to sit exactly on each candidate condition; the rest of q random within limits."""
    L1, L2 = arm.joints[2].d, arm.joints[4].d
    cases = {}
    for name in ("random", "J4 = 0", "J6 = 0", "J2 = 0", "J2 = 0 and J6 = 0", "wrist on J1 axis (J3 = 0)",
                 "wrist on J1 axis and J6 = 0 (J3 = 0)"):
        rows = []
        for _ in range(n):
            q = rng.uniform(arm.lower, arm.upper) * 0.9
            if "J4" in name:
                q[3] = 0
            if "J6 = 0" in name:
                q[5] = 0
            if name.startswith("J2 = 0"):
                q[1] = 0
            if "wrist on" in name:                               # L1 sin q2 + L2 sin(q2 + q4) = 0 with J3 = 0
                q[2] = 0
                q[1] = np.arctan2(-L2 * np.sin(q[3]), L1 + L2 * np.cos(q[3]))
            rows.append((sigma_min(arm, q), sigma_min(arm, q, locked={2})))
        cases[name] = np.array(rows)
    return cases


# ------------------------------------------------------------------------- damping -------
def dls(arm, T, q, locked=(), lam=0.02, adaptive=None, iters=400, max_step=0.2, history=None):
    """solve_dls's loop with an optional adaptive damping (eps, lam_max): lam^2 = (1 - (s/eps)^2) lam_max^2 below eps."""
    free = np.array([i not in set(locked) for i in range(arm.n)])
    W = np.diag([1, 1, 1, W_ROT, W_ROT, W_ROT])
    for it in range(iters):
        e = pose_error(T, arm.fk(q))
        if history is not None:
            history.append(np.linalg.norm(e[:3]))
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-3:
            return q, it, True
        Jw = W @ arm.jacobian(q)[:, free]
        l2 = lam ** 2
        if adaptive:
            eps, lmax = adaptive
            s = np.linalg.svd(Jw, compute_uv=False)[-1]
            l2 = 0.0 if s >= eps else (1 - (s / eps) ** 2) * lmax ** 2
        dq = Jw.T @ np.linalg.solve(Jw @ Jw.T + (l2 + 1e-12) * np.eye(6), W @ e)
        if np.linalg.norm(dq) > max_step:
            dq *= max_step / np.linalg.norm(dq)
        q = q.copy()
        q[free] = q[free] + dq
        q = arm.clamp(q)
    return q, iters, False


def path_through_wrist(arm, locked=(2,), steps=120, **kw):
    """Straight tool line whose IK crosses J6 = 0 (6-DOF mode by default): per-step joint jump and tool error."""
    qa = np.radians([10, 40, 0, 70, 30, 12, -20])
    qb = np.radians([10, 40, 0, 70, -30, -12, 20])
    Ta, Tb = arm.fk(qa), arm.fk(qb)
    from deskarm.cartesian import interpolate_pose
    q, jumps, errs, smin = qa.copy(), [], [], []
    for s in np.linspace(0, 1, steps + 1)[1:]:
        T = interpolate_pose(Ta, Tb, s)
        q_new, _, _ = dls(arm, T, q, locked=locked, **kw)
        jumps.append(np.abs(q_new - q).max())
        errs.append(np.linalg.norm(pose_error(T, arm.fk(q_new))[:3]))
        smin.append(sigma_min(arm, q_new, locked))
        q = q_new
    return np.array(jumps), np.array(errs), np.array(smin)


# ------------------------------------------------------------------------- CCD -----------
def ccd_angle(z, o, vecs, targets, weights):
    """Exact best turn about the axis (unit z through point o) for a weighted sum of |R(t) v - target|^2.

    Each vector splits into v_par + v_perp; R(t) v = v_par + cos t v_perp + sin t (z x v), so the cost is
    const - 2 sum w (B cos t + C sin t) with B = target . v_perp, C = target . (z x v): best t = atan2(C, B).
    """
    B = C = 0.0
    for v, t, w in zip(vecs, targets, weights):
        vp = v - z * (z @ v)
        B += w * (t @ vp)
        C += w * (t @ np.cross(z, v))
    return float(np.arctan2(C, B))


def ccd(arm, T, q0, locked=(), sweeps=400, w_rot=W_ROT, tol=(1e-4, 1e-3), history=None):
    """Cyclic coordinate descent for the full pose: joints from the tool inward, one exact turn each.

    Position: the TCP about each axis toward the target point. Orientation: the tool's x, y, z axes toward the
    target's, each weighted w_rot^2 (a radian of orientation error costs about w_rot^2 like a metre of position).
    Each turn is clipped to the joint limits: the cost is a sinusoid with its minimum at the best turn, so the clip
    moves toward it along the shorter arc and still lowers the cost.
    """
    q = arm.clamp(np.array(q0, float))
    pt, Rt = T[:3, 3], T[:3, :3]
    free = [i for i in range(arm.n) if i not in set(locked)]
    for sweep in range(sweeps):
        for i in reversed(free):
            F = arm.frames(q)
            z, o, P = F[i][:3, 2], F[i][:3, 3], F[-1]
            vecs = [P[:3, 3] - o] + [P[:3, k] for k in range(3)]
            targets = [pt - o] + [Rt[:, k] for k in range(3)]
            t = ccd_angle(z, o, vecs, targets, [1.0] + [w_rot ** 2] * 3)
            q[i] = np.clip(q[i] + t, arm.lower[i], arm.upper[i])
        e = pose_error(T, arm.fk(q))
        if history is not None:
            history.append(np.linalg.norm(e[:3]))
        if np.linalg.norm(e[:3]) < tol[0] and np.linalg.norm(e[3:]) < tol[1]:
            return q, sweep + 1, True
    return q, sweeps, False


def compare(arm, rng, n=200, spread=0.6, task="pose"):
    """Random reachable targets, solved from a seed `spread` rad (normal) away by DLS, CCD, and CCD to 5 mm then DLS.
    task "position" frees the orientation: DLS's position task, CCD with the orientation weight at 0."""
    out = {k: [] for k in ("dls", "ccd", "ccd+dls")}
    w = W_ROT if task == "pose" else 0.0
    for _ in range(n):
        q_true = rng.uniform(arm.lower, arm.upper) * 0.8
        seed = arm.clamp(q_true + rng.normal(0, spread, arm.n))
        T = arm.fk(q_true)
        t0 = time.perf_counter()
        r = solve_dls(arm, T, seed, task=task)
        out["dls"].append((r.success, r.iterations, time.perf_counter() - t0, np.abs(r.q - seed).max()))
        t0 = time.perf_counter()
        q, it, ok = ccd(arm, T, seed, w_rot=w, tol=(1e-4, 1e-3 if task == "pose" else np.inf))
        out["ccd"].append((ok, it, time.perf_counter() - t0, np.abs(q - seed).max()))
        t0 = time.perf_counter()
        q1, it1, _ = ccd(arm, T, seed, w_rot=w, tol=(5e-3, 5e-2 if task == "pose" else np.inf), sweeps=60)
        r2 = solve_dls(arm, T, q1, task=task)
        out["ccd+dls"].append((r2.success, it1 + r2.iterations, time.perf_counter() - t0, np.abs(r2.q - seed).max()))
    return {k: np.array(v, dtype=float) for k, v in out.items()}


def main(parts):
    arm = load_arm()
    rng = np.random.default_rng(1)
    if "singular" in parts:
        print("Smallest singular value of the weighted pose Jacobian (median / 5th pct), 7-DOF and J3 locked:")
        for name, a in constructed(arm, rng).items():
            print(f"  {name:38s} 7-DOF {np.median(a[:, 0]):.4f} / {np.percentile(a[:, 0], 5):.4f}"
                  f"   6-DOF {np.median(a[:, 1]):.4f} / {np.percentile(a[:, 1], 5):.4f}")
    if "path" in parts:
        print("Straight line whose IK crosses J6 = 0 (6-DOF mode), 120 steps: max joint jump per step, max tool error:")
        for label, kw in (("fixed lambda 0.02", {}), ("fixed lambda 0.005", {"lam": 0.005}),
                          ("adaptive eps 0.04, lambda_max 0.04", {"adaptive": (0.04, 0.04)})):
            j, e, s = path_through_wrist(arm, **kw)
            print(f"  {label:36s} jump {np.degrees(j.max()):6.2f} deg   error {e.max() * 1e3:6.2f} mm   min sigma {s.min():.4f}")
        j, e, s = path_through_wrist(arm, locked=())
        print(f"  {'7-DOF, fixed lambda 0.02':36s} jump {np.degrees(j.max()):6.2f} deg   error {e.max() * 1e3:6.2f} mm   min sigma {s.min():.4f}")
    if "ccd" in parts:
        for task, spread in (("pose", 0.6), ("pose", 2.0), ("position", 0.6), ("position", 2.0)):
            print(f"200 random reachable targets, {task} task, seed {spread} rad away "
                  "(success, median iterations, median ms, median max |dq|):")
            for k, a in compare(arm, rng, spread=spread, task=task).items():
                ok = a[:, 0] > 0
                print(f"  {k:8s} {ok.mean() * 100:5.1f} %   {np.median(a[ok, 1]):6.0f}   {np.median(a[ok, 2]) * 1e3:7.1f} ms"
                      f"   {np.degrees(np.median(a[ok, 3])):6.1f} deg")


if __name__ == "__main__":
    main(sys.argv[1:] or ["singular", "path", "ccd"])

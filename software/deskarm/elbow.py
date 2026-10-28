"""Choosing the arm angle psi along a whole Cartesian path (7-DOF mode, full pose).

The tool path fixes six of the seven joint freedoms at every point; psi, the elbow's place on its circle, is left.
:func:`plan_arm_angle` picks psi for every point at once by dynamic programming over a psi grid:

* stages every ~1 cm / 0.1 rad along the path; at each, the closed-form IK (``analytic_ik_7dof``) at every grid psi,
  keeping the branch the arm starts in (a branch change needs a singularity, so it can't happen mid-line);
* node cost: joint-limit margin, and clearance from the collision capsules (``collision.min_clearance``), checked
  lazily: only nodes on the chain the search picks are checked, tight ones get their penalty and the search runs
  again, so few of the ~36 x stages nodes ever cost a clearance check;
* edge cost: joint motion between stages, in seconds at full joint speed, squared; steps that would jump are barred;
* the start is the arm's current configuration, the end is free.

The chosen stage psi values are then filled in at the usual 2 mm spacing (psi interpolated, closed form at each point,
nearest the previous one, nudged within one grid step where that would jump). Returns None when no chain or fill
works; ``cartesian.linear_path`` then falls back to warm-started least squares. Math: docs/14_math_guide.md §6, Step 5.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .collision import min_clearance
from .ik import analytic_ik_7dof, supports_analytic, swivel_angle, wrap_angle
from .kinematics import ArmModel


@dataclass
class ElbowPlan:
    qs: np.ndarray            # dense joint path, one row per pose
    psi: np.ndarray           # arm angle at each dense point (rad)
    min_clearance: float      # smallest capsule clearance at the stages (m)
    checks: int               # clearance checks the search needed


def plannable(arm: ArmModel, mode: str, locked, task: str) -> bool:
    return mode == "7dof" and not locked and task == "pose" and supports_analytic(arm)


def plan_arm_angle(arm: ArmModel, q_start: np.ndarray, poses: list[np.ndarray], *, stage_every: int = 5,
                   psi_step_deg: float = 10.0, clear_margin: float = 0.02, limit_margin: float = np.radians(5),
                   w_clear: float = 10.0, w_limit: float = 1.0, max_jump: float = 0.15) -> ElbowPlan | None:
    """Arm angle along ``poses`` (dense, ``poses[0]`` = FK(q_start)); None if no collision-free chain exists.

    Every ``stage_every``-th pose is a stage. Costs (dimensionless, docs/14 §6 Step 5): motion
    sum(((dq / vmax) / step)^2) between stages, plus w_clear (1 - gap/margin)^2 below ``clear_margin`` and
    w_limit (1 - d/margin)^2 within ``limit_margin`` of a joint limit, at each node. ``max_jump`` (rad) is the most
    any joint may move between neighbouring poses, as in ``cartesian.linear_path``.
    """
    q_start = np.asarray(q_start, dtype=float)
    idx = list(range(0, len(poses), stage_every))
    if idx[-1] != len(poses) - 1:
        idx.append(len(poses) - 1)
    psi0 = swivel_angle(arm, q_start)
    psi0 = 0.0 if np.isnan(psi0) else psi0
    grid = wrap_angle(psi0 + np.radians(np.arange(-180.0, 180.0, psi_step_deg)))   # holding psi is always a node

    # nodes: stage 0 is the start; later stages one per grid psi with a solution in the start's branch
    Q, P = [q_start[None, :]], [np.array([psi0])]
    for k in idx[1:]:
        qs, ps = [], []
        for psi in grid:
            sols = analytic_ik_7dof(arm, poses[k], float(psi), q_start, verify=False)
            if sols:
                qs.append(sols[0])
                ps.append(psi)
        if not qs:
            return None
        Q.append(np.array(qs))
        P.append(np.array(ps))

    def limit_cost(q):
        d = np.minimum(q - arm.lower, arm.upper - q)
        return w_limit * np.sum(np.clip(1 - d / limit_margin, 0, None) ** 2, axis=-1)

    base = [limit_cost(q) for q in Q]
    clear = [np.full(len(q), np.nan) for q in Q]       # nan = not checked yet
    clear[0][0] = 0.0
    step = len(idx) - 1
    edges = []
    for k, (a, b) in enumerate(zip(Q[:-1], Q[1:])):
        dq = b[None, :, :] - a[:, None, :]
        c = np.sum((dq / arm.vmax * step) ** 2, axis=-1)
        c[np.max(np.abs(dq), axis=-1) > max_jump * (idx[k + 1] - idx[k])] = np.inf
        edges.append(c)

    checks, gaps = 0, {}
    while True:
        cost = [base[k] + np.nan_to_num(clear[k]) for k in range(len(Q))]
        best, back = cost[0], []
        for k, e in enumerate(edges, start=1):
            with np.errstate(over="ignore", invalid="ignore"):     # inf + inf: barred steps, colliding nodes
                tot = best[:, None] + e
                back.append(np.argmin(tot, axis=0))
                best = tot[back[-1], np.arange(tot.shape[1])] + cost[k]
        if not np.isfinite(best.min()):
            return None
        chain = [int(np.argmin(best))]
        for b in reversed(back):
            chain.append(int(b[chain[-1]]))
        chain.reverse()
        fresh = [(k, i) for k, i in enumerate(chain) if np.isnan(clear[k][i])]
        if not fresh:
            break
        for k, i in fresh:
            g, _ = min_clearance(arm, Q[k][i])
            checks += 1
            gaps[(k, i)] = g
            clear[k][i] = np.inf if g < 0 else w_clear * max(0.0, 1 - g / clear_margin) ** 2
    min_gap = min((gaps[(k, i)] for k, i in enumerate(chain) if (k, i) in gaps), default=np.inf)

    # fill in: psi interpolated between stages (shortest way round), closed form nearest the previous point. Where that
    # would jump (near J2 = 0 or J6 = 0 a small psi change swings J1/J3 or J5/J7 a lot), look within one grid step for
    # the psi with the smallest joint step.
    psi_st = np.unwrap([P[k][i] for k, i in enumerate(chain)])
    psi_all = np.interp(np.arange(len(poses)), idx, psi_st)
    qs, q = [q_start], q_start
    near = np.radians(np.arange(-psi_step_deg, psi_step_deg + 0.5, 1.0))

    def step(n, psi):
        sols = analytic_ik_7dof(arm, poses[n], float(wrap_angle(psi)), q)
        return (float(np.max(np.abs(sols[0] - q))), sols[0], psi) if sols else (np.inf, None, psi)

    for n in range(1, len(poses)):
        jump, q_n, psi_n = step(n, psi_all[n])
        if jump > max_jump / 2:
            jump, q_n, psi_n = min((step(n, psi_n + d) for d in near), key=lambda t: t[0])
        if jump > max_jump:
            return None
        q = q_n
        psi_all[n] = psi_n
        qs.append(q)
    return ElbowPlan(np.array(qs), wrap_angle(psi_all), float(min_gap), checks)

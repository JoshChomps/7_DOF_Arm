"""J2 counterbalance spring: geometry, torque, and sizing.

Layout (side view, J2 axis S into the page)::

        A  pulley / anchor, a metres above S on the J1 turntable (does not pitch with J2)
        |\\
        | \\  spring + cable, pulls B towards A
        |  \\
        S---B----------- upper arm (B is b metres along it)

With a **zero-length spring** (free length 0: in practice a normal spring whose cable runs over a
pulley at A, so the spring's own length sits elsewhere) the tension is k * |AB| and the torque
about S is exactly

    tau = -k * a * b * sin(theta)        theta = angle between "up" and the upper arm

which has the same shape as the gravity moment of a *rigid* arm swinging about J2 (m g l sin theta),
so for one fixed arm shape and payload a single spring cancels the weight at every shoulder angle.
The real arm changes shape (elbow, wrist) and payload, which moves its centre of mass, so no spring
balances everything: ``size_spring`` picks k to minimize the worst-case J2 torque over the reachable
workspace and payload range instead. A spring with free length L0 > 0 is also modelled
(tension k * max(|AB| - L0, 0)); initial tension of real extension springs is not.

    python -m deskarm.counterbalance             # size k for arm.toml (numpy only)
    python -m deskarm.counterbalance --payload-kg 0 0.5 --a 0.07 --b 0.1

Derivation of k a b sin(theta), the minimax sizing and plots: docs/14_math_guide.md §13.
"""
from __future__ import annotations

import argparse

import numpy as np

from .kinematics import ArmModel, Counterbalance, dh_transform


def _frames_to(arm: ArmModel, q, last: int) -> list[np.ndarray]:
    """DH frames 0..last only (the physics sim calls this every 0.5 ms)."""
    T = np.eye(4)
    out = [T]
    for qi, j in zip(q[:last], arm.joints[:last]):
        T = T @ dh_transform(qi + j.offset, j.d, j.a, j.alpha)
        out.append(T)
    return out


def spring_geometry(arm: ArmModel, q, cb: Counterbalance | None = None):
    """Joint centre S, anchor A, attachment B and the joint axis, in the base frame."""
    cb = cb or arm.counterbalance
    j = cb.joint
    F = _frames_to(arm, np.asarray(q, dtype=float), j + 1)
    S = F[j][:3, 3]                      # frames[j] z is this joint's axis
    A = S + np.array([0.0, 0.0, cb.anchor_height])
    B = S + cb.attach_distance * F[j + 1][:3, 2]   # the link points along the next joint's z axis
    return S, A, B, F[j][:3, 2]


def spring_torque(arm: ArmModel, q, cb: Counterbalance | None = None) -> np.ndarray:
    """Joint torques (n,) the spring applies (only the counterbalanced joint is non-zero)."""
    tau = np.zeros(arm.n)
    cb = cb or arm.counterbalance
    if cb is None or cb.stiffness <= 0:
        return tau
    S, A, B, axis = spring_geometry(arm, q, cb)
    d = A - B
    L = float(np.sqrt(d @ d))
    if L < 1e-9:
        return tau
    tension = cb.stiffness * max(L - cb.free_length, 0.0)
    r, f = B - S, tension * d / L
    moment = np.array([r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]])
    tau[cb.joint] = float(moment @ axis)
    return tau


def spring_length(arm: ArmModel, q, cb: Counterbalance | None = None) -> float:
    S, A, B, _ = spring_geometry(arm, q, cb)
    return float(np.linalg.norm(A - B))


def reachable_poses(arm: ArmModel, samples: int = 600, seed: int = 0) -> np.ndarray:
    """Random joint configurations within limits that clear the desk and don't self-collide."""
    from .collision import self_collisions
    from .ik import workspace_violations

    rng = np.random.default_rng(seed)
    out = []
    while len(out) < samples:
        q = rng.uniform(arm.lower, arm.upper)
        if not workspace_violations(arm, q) and not self_collisions(arm, q):
            out.append(q)
    return np.array(out)


def worst_case_torque(arm: ArmModel, poses, anchor_height: float, attach_distance: float,
                      payload_kg=(0.0, 1.0), joint: int = 1, free_length: float = 0.0):
    """Function k -> worst |static joint torque| the motor supplies over ``poses`` x payloads."""
    G = np.concatenate([[arm.gravity_torque(q, payload_kg=m)[joint] for q in poses]
                        for m in payload_kg])
    unit_cb = Counterbalance(joint, anchor_height, attach_distance, 1.0, free_length)
    unit = np.tile([spring_torque(arm, q, unit_cb)[joint] for q in poses], len(payload_kg))

    def worst(k: float) -> float:
        return float(np.max(np.abs(G - k * unit)))

    worst.k_scale = 5 * float(np.max(np.abs(G))) / max(float(np.max(np.abs(unit))), 1e-12)
    return worst


def size_spring(arm: ArmModel, anchor_height: float, attach_distance: float,
                payload_kg: tuple[float, float] = (0.0, 1.0), joint: int = 1, samples: int = 600,
                free_length: float = 0.0) -> dict:
    """Spring rate that minimizes the **worst-case** static torque the joint motor must supply.

    Minimax over reachable, collision-free poses and the payload range. Gravity torque is affine in
    the payload mass, so the two ends of the range cover every payload in between, and the spring
    torque is linear in k, so the worst case is a convex function of k (a 1-D search finds it).

    Balancing a single pose (arm stretched with a nominal payload) over-balances the arm whenever
    the forearm is folded or the gripper is empty; this sizing accounts for that. Returns k plus
    the before/after worst-case torques and the numbers needed to buy the spring.
    """
    Q = reachable_poses(arm, samples)
    worst = worst_case_torque(arm, Q, anchor_height, attach_distance, payload_kg, joint, free_length)
    lo, hi = 0.0, worst.k_scale
    for _ in range(100):                                           # ternary search (convex in k)
        m1, m2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
        lo, hi = (lo, m2) if worst(m1) <= worst(m2) else (m1, hi)
    k = (lo + hi) / 2
    cb = Counterbalance(joint, anchor_height, attach_distance, k, free_length)
    q = np.zeros(arm.n)
    lengths = [spring_length(arm, np.r_[q[:joint], th, q[joint + 1:]], cb)
               for th in np.linspace(arm.joints[joint].lower, arm.joints[joint].upper, 41)]
    return {"k": k, "worst_without": worst(0.0), "worst_with": worst(k),
            "limit": float(arm.torque_limit()[joint]),
            "max_tension": k * max(max(lengths) - free_length, 0.0),
            "min_length": min(lengths), "max_length": max(lengths), "poses": len(Q)}


def main(argv=None) -> None:
    from .kinematics import load_arm

    arm = load_arm()
    base = arm.counterbalance
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--a", type=float, default=base.anchor_height if base else 0.08,
                   help="anchor height above the J2 axis (m)")
    p.add_argument("--b", type=float, default=base.attach_distance if base else 0.12,
                   help="attachment distance along the upper arm (m)")
    p.add_argument("--payload-kg", type=float, nargs=2, default=(0.0, 1.0), metavar=("MIN", "MAX"),
                   help="payload range at the TCP to balance over (default 0 1)")
    p.add_argument("--samples", type=int, default=600, help="random reachable poses to check")
    args = p.parse_args(argv)

    r = size_spring(arm, args.a, args.b, tuple(args.payload_kg), samples=args.samples)
    lim = r["limit"]
    print(f"worst-case J2 static torque over {r['poses']} reachable poses, payload "
          f"{args.payload_kg[0]}-{args.payload_kg[1]} kg:")
    print(f"  without spring {r['worst_without']:.2f} N*m ({100 * r['worst_without'] / lim:.0f} % of {lim:.1f})"
          f"  ->  with spring {r['worst_with']:.2f} N*m ({100 * r['worst_with'] / lim:.0f} %)")
    print(f"zero-length spring rate k = {r['k']:.0f} N/m  (a = {args.a * 1000:.0f} mm, b = {args.b * 1000:.0f} mm)")
    print(f"cable length A-B: {r['min_length'] * 1000:.0f}-{r['max_length'] * 1000:.0f} mm "
          f"-> spring must stretch that far; peak tension {r['max_tension']:.0f} N")
    print(f"moving mass in arm.toml: {arm.moving_mass:.2f} kg (from the CAD since 20 Oct). Set [counterbalance] "
          "stiffness in arm.toml to this k,\nand re-run whenever the [[masses]] list changes. Real extension "
          "springs have initial tension, which this model ignores.")


if __name__ == "__main__":
    main()

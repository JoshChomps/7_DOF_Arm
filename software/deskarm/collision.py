"""Self-collision check: the arm as a chain of capsules (spheres swept along segments).

    base column - shoulder - upper arm - elbow - forearm - wrist - tool

Joint housings are spheres at the shoulder, elbow and wrist centres, and the links are capsules
between them, trimmed so they start at the housing surface (that's how the printed links attach).
Bodies at most one housing apart (forearm and upper arm around the elbow, tool and forearm around
the wrist, upper arm and base around the shoulder) are left to the **joint limits**: how far a joint
can fold before its own links touch depends on CAD details (clevis offsets, cut-outs) that the
limits in ``arm.toml`` should encode. Every other pair is checked, which catches what joint limits
can't see because it depends on several joints at once:

* the tool or wrist reaching back into the base column, the shoulder or the upper arm,
* the elbow or forearm swinging down into the base (J2 and J4 together).

The radii started as placeholders sized from the planned motors and cycloid housings. Since 20 Oct the
CAD's off-centre parts (the pitch actuators and the motors beside the arm) are extra capsules in
``ATTACHED``, each riding on its link, taken from DA_ARM and checked against Fusion's own
interference test on random poses (docs/21 §5, ``tools/collision_vs_cad.py``). Same idea as MoveIt's collision
check, at a fraction of the cost: 10 capsule pairs, well under 1 ms per pose.

Math (capsules, segment-to-segment distance) with a picture: docs/14_math_guide.md §14.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import ArmModel


@dataclass(frozen=True)
class Geometry:
    base_radius: float = 0.075        # J1 base housing (CAD: its top sphere ends at the turntable top, z 84 mm)
    base_clearance: float = 0.200     # column stops this far below the shoulder axis
    shoulder_radius: float = 0.060    # J2/J3 cycloid housing (100 mm disc)
    upper_radius: float = 0.030       # printed upper-arm tube
    elbow_radius: float = 0.045       # J4 housing
    fore_radius: float = 0.028
    wrist_radius: float = 0.035       # J5-J7 spherical wrist
    tool_radius: float = 0.050        # 3-prong gripper: base R 38, fingertips R 49.4 fully open (docs/21 §12)
    tool_tip: float = 0.013           # fingertips past the TCP: the desk sees the tool as a flat-ended cylinder


GEOMETRY = Geometry()

BODIES = ("base", "shoulder", "upper arm", "elbow", "forearm", "wrist", "tool")


@dataclass(frozen=True)
class Capsule:
    name: str
    p0: np.ndarray
    p1: np.ndarray
    radius: float


def capsules(arm: ArmModel, q, geom: Geometry = GEOMETRY) -> list[Capsule]:
    F = arm.frames(q)
    S, E, W = arm.srs_points(q)
    flange, tcp = F[arm.n][:3, 3], F[-1][:3, 3]
    base_top = np.array([0.0, 0.0, max(S[2] - geom.base_clearance, 0.0)])

    def trimmed(a, ra, b, rb):
        u = b - a
        n = float(np.linalg.norm(u))
        if n < ra + rb:
            return a + u / 2, a + u / 2
        u /= n
        return a + u * ra, b - u * rb

    up0, up1 = trimmed(S, geom.shoulder_radius, E, geom.elbow_radius)
    fo0, fo1 = trimmed(E, geom.elbow_radius, W, geom.wrist_radius)
    to0, _ = trimmed(W, geom.wrist_radius, tcp, 0.0)
    if np.linalg.norm(tcp - W) < np.linalg.norm(flange - W):     # TCP inside the wrist: no tool
        to0 = tcp
    return [
        Capsule("base", np.zeros(3), base_top, geom.base_radius),
        Capsule("shoulder", S, S, geom.shoulder_radius),
        Capsule("upper arm", up0, up1, geom.upper_radius),
        Capsule("elbow", E, E, geom.elbow_radius),
        Capsule("forearm", fo0, fo1, geom.fore_radius),
        Capsule("wrist", W, W, geom.wrist_radius),
        Capsule("tool", to0, tcp, geom.tool_radius),
    ]


# Bodies at most one housing apart are handled by the joint limits (see the module docstring).
PAIRS = tuple((i, j) for i in range(len(BODIES)) for j in range(i + 3, len(BODIES)))


@dataclass(frozen=True)
class Attached:
    """A capsule fixed to one link: ``carrier`` = index of the joint that moves it (0 = J1), ends in the base
    frame at the zero pose (m), and the bodies it is never checked against (its neighbours: joint limits)."""
    name: str
    carrier: int
    p0: tuple
    p1: tuple
    radius: float
    skip: frozenset = frozenset()


def _a(name, carrier, p0, p1, r, *skip):
    return Attached(name, carrier, p0, p1, r, frozenset(skip))


# From DA_ARM (docs/20, docs/21 §5): CAD x, y -> base frame (-y, x). Motors are motor + driver boxes: the radius
# sits between the box's half-width and half-diagonal (NEMA17 21 / 30 mm -> 24), so the planner's 5 mm margin
# covers the corners. The turntable flange (z 62-74, R 72) is four spokes.
ATTACHED = (
    *(_a(f"turntable {a}", 0, (-0.066 * np.cos(np.radians(a)), -0.066 * np.sin(np.radians(a)), 0.068),
          (0.066 * np.cos(np.radians(a)), 0.066 * np.sin(np.radians(a)), 0.068), 0.006, "base", "shoulder", "J2 module",
          "J2 motor", "tower", *(f"turntable {b}" for b in (0, 45, 90, 135))) for a in (0, 45, 90, 135)),
    _a("J2 module", 0, (0, -0.150, 0.209), (0, -0.055, 0.209), 0.061, "base", "shoulder", "upper arm", "J2 motor"),
    _a("J2 motor", 0, (0, -0.125, 0.1135), (0, -0.091, 0.1135), 0.034, "base", "shoulder", "J2 module"),
    _a("tower", 0, (0, 0.070, 0.105), (0, 0.070, 0.285), 0.016, "base", "shoulder"),
    _a("J3 motor", 1, (0.0698, 0, 0.255), (0.0698, 0, 0.283), 0.024, "shoulder", "upper arm", "J2 module"),
    _a("link 3 collar", 2, (-0.046, -0.0845, 0.330), (0.046, -0.0845, 0.330), 0.016, "upper arm", "shoulder", "elbow",
       "J3 motor", "J4 module", "J4 motor", "link 3 arm -22", "link 3 arm +22"),
    *(_a(f"link 3 arm {x:+.0f}", 2, (x / 1000, -0.080, 0.3235), (x / 1000, -0.035, 0.3235), 0.008, "upper arm",
         "shoulder", "elbow", "J3 motor", "J4 module", "J4 motor", "link 3 collar", "link 3 arm -22", "link 3 arm +22")
      for x in (-22, 22)),                            # the 12 mm plate, y +-30: its two long edges
    _a("J4 module", 2, (0, -0.1415, 0.399), (0, -0.0605, 0.399), 0.045, "upper arm", "elbow", "forearm", "J4 motor",
       "link 3 collar", "link 3 arm -22", "link 3 arm +22"),
    _a("J4 motor", 2, (0, -0.1145, 0.469), (0, -0.0865, 0.469), 0.024, "upper arm", "elbow", "forearm", "J4 module",
       "link 3 collar", "link 3 arm -22", "link 3 arm +22"),
    _a("J6 module", 4, (0, -0.164, 0.579), (0, -0.064, 0.579), 0.040, "forearm", "wrist", "tool", "J7 motor"),
    _a("J7 motor", 5, (0, 0.0585, 0.606), (0, 0.0585, 0.622), 0.024, "forearm", "wrist", "tool", "J6 module"),
)


# Space kept free on the base board (docs/21 §14), base frame (m): x0, x1, y0, y1, height above the board top.
KEEP_OUT = (
    ("electronics", -0.250, -0.130, -0.250, 0.250, 0.045),   # back strip: PSU 215 x 115 x 30, boards A/B, Pi, CANable
    ("E-stop", 0.180, 0.250, 0.180, 0.250, 0.080),           # front corner, 22 mm button in its box
)


def _box_gap(c: Capsule, box) -> float:
    """Gap from a capsule to a keep-out box (sampled along the capsule's segment, negative = inside)."""
    _, x0, x1, y0, y1, h = box
    lo, hi = np.array([x0, y0, 0.0]), np.array([x1, y1, h])
    pts = [c.p0 + (c.p1 - c.p0) * t for t in np.linspace(0.0, 1.0, 7)]
    return min(float(np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0.0))) for q in pts) - c.radius


def attached_capsules(arm: ArmModel, q, items=ATTACHED) -> list[tuple[Attached, Capsule]]:
    L0, Lq = arm.link_frames(np.zeros(arm.n)), arm.link_frames(q)
    out = []
    for a in items:
        T = Lq[a.carrier] @ np.linalg.inv(L0[a.carrier])
        w = lambda p: (T @ np.append(np.asarray(p, float), 1.0))[:3]
        out.append((a, Capsule(a.name, w(a.p0), w(a.p1), a.radius)))
    return out


def segment_distance(p1, q1, p2, q2) -> float:
    """Shortest distance between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection 5.1.9)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = d1 @ r
        if e <= eps:
            s, t = np.clip(-c / a, 0.0, 1.0), 0.0
        else:
            b = d1 @ d2
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                s, t = np.clip(-c / a, 0.0, 1.0), 0.0
            elif t > 1.0:
                s, t = np.clip((b - c) / a, 0.0, 1.0), 1.0
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def clearances(arm: ArmModel, q, geom: Geometry = GEOMETRY) -> dict[tuple[str, str], float]:
    """Surface-to-surface gap (m, negative = overlap) for every non-adjacent body pair, and every body's height
    above the desk (the base board, z = 0)."""
    caps = capsules(arm, q, geom)
    out = {}
    for i, j in PAIRS:
        a, b = caps[i], caps[j]
        out[(a.name, b.name)] = segment_distance(a.p0, a.p1, b.p0, b.p1) - a.radius - b.radius
    att = attached_capsules(arm, q)
    for c in caps[1:] + [c for a, c in att if a.carrier > 0 or not a.name.startswith("turntable")]:
        out[(c.name, "desk")] = min(c.p0[2], c.p1[2]) - c.radius   # the base board's top is z = 0 (the base frame)
        if c.name == "tool" and np.linalg.norm(c.p1 - c.p0) > 1e-9:  # its round end would keep the tips 50 mm up
            n = (c.p1 - c.p0) / np.linalg.norm(c.p1 - c.p0)
            rim = c.radius * np.sqrt(max(0.0, 1.0 - n[2] ** 2))       # lowest point of the fingertip disc
            out[(c.name, "desk")] = min(c.p0[2] - c.radius, c.p1[2] + n[2] * geom.tool_tip - rim)
        for box in KEEP_OUT:
            out[(c.name, box[0])] = _box_gap(c, box)
    gap = lambda a, b: segment_distance(a.p0, a.p1, b.p0, b.p1) - a.radius - b.radius
    for k, (a, ca) in enumerate(att):
        for c in caps:
            if c.name not in a.skip:
                out[(a.name, c.name)] = gap(ca, c)
        for b, cb in att[k + 1:]:
            if b.carrier != a.carrier and b.name not in a.skip and a.name not in b.skip:
                out[(a.name, b.name)] = gap(ca, cb)
    return out


def self_collisions(arm: ArmModel, q, margin: float = 0.005, geom: Geometry = GEOMETRY) -> list[str]:
    """Human-readable list of body pairs closer than ``margin`` (empty = clear)."""
    return [f"{a} hits {b}" if gap < 0 else f"{a} within {gap * 1000:.0f} mm of {b}"
            for (a, b), gap in clearances(arm, q, geom).items() if gap < margin]


def min_clearance(arm: ArmModel, q, geom: Geometry = GEOMETRY) -> tuple[float, tuple[str, str]]:
    gaps = clearances(arm, q, geom)
    pair = min(gaps, key=gaps.get)
    return gaps[pair], pair


def trajectory_collision(arm: ArmModel, traj, samples: int = 80, **kw):
    """First (time, messages) along a trajectory where the arm would hit itself, else None."""
    for k in range(samples + 1):
        t = traj.duration * k / samples
        hit = self_collisions(arm, traj.sample(t)[0], **kw)
        if hit:
            return t, hit
    return None

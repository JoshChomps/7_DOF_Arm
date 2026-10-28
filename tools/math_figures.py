#!/usr/bin/env python3
"""
Figures for docs/14_math_guide.md, drawn from the real code and arm.toml.

    python3 tools/math_figures.py            # rewrites docs/img/math/*.svg

Every curve and every stick figure here is computed by the same functions the arm uses
(deskarm.kinematics, ik, trajectory, topp, motor, counterbalance, collision; tools/cycloid, gears),
so when the code or arm.toml changes, re-running this keeps the pictures honest. Plain SVG is
written by hand (no plotting library), so numpy is the only dependency. The time-optimal figure
needs MuJoCo (the [sim] extra) and is skipped without it.
"""
import math
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "software"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import cycloid  # noqa: E402
import gears  # noqa: E402
from deskarm import load_arm  # noqa: E402
from deskarm.collision import capsules  # noqa: E402
from deskarm.counterbalance import spring_torque  # noqa: E402
from deskarm.ik import analytic_ik_7dof, swivel, swivel_angle  # noqa: E402
from deskarm.kinematics import G, load_config  # noqa: E402
from deskarm.motor import stepper_torque  # noqa: E402
from deskarm.trajectory import DoubleS, TrapezoidalTrajectory  # noqa: E402

OUT = ROOT / "docs" / "img" / "math"
BLUE, ORANGE, GREEN, RED, GRAY, PURPLE, INK = "#1f6feb", "#d9730d", "#2da44e", "#cf222e", "#6e7781", "#8250df", "#24292f"
FONT = 'font-family="Segoe UI, Helvetica, Arial, sans-serif"'


def esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def nice_ticks(lo, hi, n=6):
    span = (hi - lo) or 1.0
    step = 10 ** math.floor(math.log10(span / n))
    for m in (1, 2, 2.5, 5, 10):
        if span / (step * m) <= n:
            step *= m
            break
    t = math.ceil(lo / step - 1e-9) * step
    out = []
    while t <= hi + 1e-9:
        out.append(0.0 if abs(t) < step * 1e-6 else t)
        t += step
    return out


def fmt(v):
    return f"{v:.3g}" if abs(v) < 1000 else f"{v:.0f}"


class Canvas:
    """An SVG being built: a list of elements plus a world -> pixel mapping."""

    def __init__(self, w, h):
        self.w, self.h, self.items = w, h, []

    def X(self, x):
        return x

    def Y(self, y):
        return y

    def line(self, x0, y0, x1, y1, color=INK, width=1.5, dash=None, cap="butt", opacity=1.0):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.items.append(f'<line x1="{self.X(x0):.1f}" y1="{self.Y(y0):.1f}" x2="{self.X(x1):.1f}" '
                          f'y2="{self.Y(y1):.1f}" stroke="{color}" stroke-width="{width}" '
                          f'stroke-linecap="{cap}" opacity="{opacity}"{d}/>')

    def poly(self, xs, ys, color=BLUE, width=2.0, dash=None, fill="none", close=False, opacity=1.0):
        pts = " ".join(f"{self.X(x):.1f},{self.Y(y):.1f}" for x, y in zip(xs, ys))
        d = f' stroke-dasharray="{dash}"' if dash else ""
        tag = "polygon" if close else "polyline"
        self.items.append(f'<{tag} points="{pts}" fill="{fill}" stroke="{color}" stroke-width="{width}" '
                          f'stroke-linejoin="round" opacity="{opacity}"{d}/>')

    def dot(self, x, y, r=4, color=INK, fill=None, width=1.5, opacity=1.0):
        self.items.append(f'<circle cx="{self.X(x):.1f}" cy="{self.Y(y):.1f}" r="{r}" stroke="{color}" '
                          f'stroke-width="{width}" fill="{fill or color}" opacity="{opacity}"/>')

    def text(self, x, y, s, size=13, color=INK, anchor="start", dx=0, dy=0, bold=False, italic=False):
        style = (' font-weight="600"' if bold else "") + (' font-style="italic"' if italic else "")
        self.items.append(f'<text x="{self.X(x) + dx:.1f}" y="{self.Y(y) + dy:.1f}" font-size="{size}" '
                          f'fill="{color}" text-anchor="{anchor}" {FONT}{style}>{esc(s)}</text>')

    def arrow(self, x0, y0, x1, y1, color=INK, width=2.0, head=9):
        self.line(x0, y0, x1, y1, color, width)
        ax, ay, bx, by = self.X(x0), self.Y(y0), self.X(x1), self.Y(y1)
        ang = math.atan2(by - ay, bx - ax)
        pts = [(bx, by)] + [(bx - head * math.cos(ang + s * 0.4), by - head * math.sin(ang + s * 0.4)) for s in (1, -1)]
        self.items.append('<polygon points="' + " ".join(f"{px:.1f},{py:.1f}" for px, py in pts)
                          + f'" fill="{color}"/>')

    def svg(self):
        return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" width="{self.w}" '
                f'height="{self.h}">\n<rect width="100%" height="100%" fill="#ffffff"/>\n'
                + "\n".join(self.items) + "\n</svg>\n")

    def save(self, name):
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / name).write_text(self.svg(), encoding="utf-8")
        print("wrote", (OUT / name).relative_to(ROOT))


class Plot(Canvas):
    """x-y chart with axes, ticks and a legend."""

    def __init__(self, xlim, ylim, xlabel="", ylabel="", title="", w=680, h=380):
        super().__init__(w, h)
        self.xlim, self.ylim = xlim, ylim
        self.l, self.r, self.t, self.b = 68, 20, 36 if title else 16, 48
        self.legend_items = []
        for t in nice_ticks(*xlim):
            self.line(t, ylim[0], t, ylim[1], "#d8dee4", 1)
            self.text(t, ylim[0], fmt(t), 12, GRAY, "middle", dy=18)
        for t in nice_ticks(*ylim):
            self.line(xlim[0], t, xlim[1], t, "#d8dee4", 1)
            self.text(xlim[0], t, fmt(t), 12, GRAY, "end", dx=-8, dy=4)
        self.poly([xlim[0], xlim[1], xlim[1], xlim[0]], [ylim[0], ylim[0], ylim[1], ylim[1]], INK, 1.2, close=True)
        self.items.append(f'<text x="{(self.l + w - self.r) / 2:.0f}" y="{h - 10}" font-size="13" fill="{INK}" '
                          f'text-anchor="middle" {FONT}>{esc(xlabel)}</text>')
        self.items.append(f'<text transform="translate(16,{(self.t + h - self.b) / 2:.0f}) rotate(-90)" '
                          f'font-size="13" fill="{INK}" text-anchor="middle" {FONT}>{esc(ylabel)}</text>')
        if title:
            self.items.append(f'<text x="{self.l}" y="22" font-size="15" font-weight="600" fill="{INK}" '
                              f'{FONT}>{esc(title)}</text>')

    def X(self, x):
        return self.l + (x - self.xlim[0]) / (self.xlim[1] - self.xlim[0]) * (self.w - self.l - self.r)

    def Y(self, y):
        return self.h - self.b - (y - self.ylim[0]) / (self.ylim[1] - self.ylim[0]) * (self.h - self.t - self.b)

    def curve(self, xs, ys, color=BLUE, label=None, width=2.2, dash=None):
        ys = np.clip(np.asarray(ys, dtype=float), self.ylim[0], self.ylim[1])
        self.poly(xs, ys, color, width, dash)
        if label:
            self.legend_items.append((label, color, dash))

    def legend(self, x_frac=0.60, y_frac=0.06):
        x = self.l + x_frac * (self.w - self.l - self.r)
        y = self.t + y_frac * (self.h - self.t - self.b) + 6
        for k, (label, color, dash) in enumerate(self.legend_items):
            d = f' stroke-dasharray="{dash}"' if dash else ""
            yy = y + 19 * k
            self.items.append(f'<line x1="{x:.0f}" y1="{yy:.0f}" x2="{x + 26:.0f}" y2="{yy:.0f}" stroke="{color}" '
                              f'stroke-width="2.4"{d}/>')
            self.items.append(f'<text x="{x + 33:.0f}" y="{yy + 4:.0f}" font-size="12.5" fill="{INK}" {FONT}>'
                              f'{esc(label)}</text>')


class Sketch(Canvas):
    """Scale drawing: world metres (or mm) -> pixels with equal axes, y up."""

    def __init__(self, xlim, ylim, w=680, pad=24):
        self.xlim, self.ylim, self.pad = xlim, ylim, pad
        self.s = (w - 2 * pad) / (xlim[1] - xlim[0])
        super().__init__(w, int((ylim[1] - ylim[0]) * self.s + 2 * pad))

    def X(self, x):
        return self.pad + (x - self.xlim[0]) * self.s

    def Y(self, y):
        return self.h - self.pad - (y - self.ylim[0]) * self.s


# ------------------------------------------------------------------------------- figures ---
def arm_side_points(arm, q):
    """Base, shoulder, elbow, wrist centre, flange, TCP projected on the x-z plane."""
    F = arm.frames(q)
    S, E, W = arm.srs_points(q)
    pts = [np.zeros(3), S, E, W, F[arm.n][:3, 3], F[-1][:3, 3]]
    return [(p[0], p[2]) for p in pts]


def draw_arm(sk, pts, color=INK, width=5, opacity=1.0, joints=True):
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        sk.line(x0, y0, x1, y1, color, width, cap="round", opacity=opacity)
    if joints:
        for x, y in pts[1:4]:
            sk.dot(x, y, 7, color, "#ffffff", 2.5, opacity)


def fig_arm_frames(arm):
    sk = Sketch((-0.30, 0.62), (-0.04, 0.72))
    sk.line(-0.3, 0, 0.62, 0, GRAY, 1.5)
    sk.text(0.40, 0, "desk (z = 0)", 12, GRAY, dy=16)
    zero = arm_side_points(arm, np.zeros(arm.n))
    draw_arm(sk, zero, GRAY, 4, 0.45)
    sk.text(zero[-1][0], zero[-1][1], "zero pose q = 0 (straight up)", 12, GRAY, dx=10, dy=4)
    home = arm_side_points(arm, arm.home)
    draw_arm(sk, home, BLUE, 6)
    names = ["base", "S  shoulder: J1 yaw, J2 pitch, J3 roll", "E  elbow: J4",
             "W  wrist centre: J5 roll, J6 pitch, J7 roll", "flange", "TCP"]
    offs = [(-12, 16), (-14, 4), (-2, -14), (10, -14), (14, 4), (12, 16)]
    anchors = ["end", "end", "start", "start", "start", "start"]
    for (x, y), n, (dx, dy), a in zip(home, names, offs, anchors):
        sk.text(x, y, n, 12.5, INK, a, dx=dx, dy=dy)
    sk.dot(*home[-1], 4, RED)
    d = [j.d for j in arm.joints]
    mids = [(home[i], home[i + 1]) for i in range(5)]
    labels = [f"d1 = {d[0] * 1000:.0f}", f"d3 = {d[2] * 1000:.0f}", f"d5 = {d[4] * 1000:.0f}",
              f"d7 = {d[6] * 1000:.0f}", f"tool {arm.tool[2, 3] * 1000:.0f}"]
    where = [("end", -12, 4), ("end", -12, -4), ("middle", 0, 20), ("start", 12, 2), ("start", 12, 2)]
    for ((x0, y0), (x1, y1)), lab, (anchor, dx, dy) in zip(mids, labels, where):
        sk.text((x0 + x1) / 2, (y0 + y1) / 2, lab, 11.5, BLUE, anchor, dx=dx, dy=dy, italic=True)
    sk.arrow(-0.27, 0.02, -0.19, 0.02, GRAY, 1.5, 7)
    sk.arrow(-0.27, 0.02, -0.27, 0.09, GRAY, 1.5, 7)
    sk.text(-0.19, 0.02, "x", 12, GRAY, dx=6, dy=4)
    sk.text(-0.27, 0.09, "z", 12, GRAY, dx=6, dy=2)
    sk.text(-0.29, 0.69, "Side view (lengths in mm). Blue: home pose q = [0, 30, 0, 60, 0, 60, 0] deg", 13, INK, bold=True)
    sk.save("arm_frames.svg")


def fig_jacobian(arm):
    sk = Sketch((-0.12, 0.60), (0.02, 0.60))
    q = arm.home
    pts = arm_side_points(arm, q)
    draw_arm(sk, pts, GRAY, 5, 0.6)
    F = arm.frames(q)
    p = F[-1][:3, 3]
    for k, color, name in ((1, BLUE, "J2"), (3, ORANGE, "J4")):
        o, z = F[k][:3, 3], F[k][:3, 2]
        col = np.cross(z, p - o) * 0.35                       # tool velocity for 0.35 rad/s of joint speed
        sk.line(o[0], o[2], p[0], p[2], color, 1.6, dash="6 4")
        sk.dot(o[0], o[2], 7, color, "#ffffff", 2.5)
        sk.dot(o[0], o[2], 2, color)
        sk.arrow(p[0], p[2], p[0] + col[0], p[2] + col[2], color, 2.4)
        sk.text(o[0], o[2], f"{name} axis z (out of the page)", 12, color, "end", dx=-12, dy=-10)
        mid = o + 0.4 * (p - o)
        sk.text(mid[0], mid[2], f"r = p - o  ({name})", 12, color, "middle", dy=18 if k == 1 else -8)
        sk.text(p[0] + col[0], p[2] + col[2], f"z x r  ({name} column)", 12, color, dx=8, dy=4)
    sk.dot(p[0], p[2], 4, RED)
    sk.text(p[0], p[2], "p (TCP)", 12, RED, "end", dx=-8, dy=16)
    sk.text(-0.11, 0.57, "One Jacobian column per joint: tool velocity = (joint axis) x (lever arm) x joint speed",
            13, INK, bold=True)
    sk.save("jacobian_columns.svg")


def fig_ik_triangle(arm):
    L1, L2 = arm.joints[2].d, arm.joints[4].d
    rho, h = 0.27, 0.12
    D = (rho**2 + h**2 - L1**2 - L2**2) / (2 * L1 * L2)
    sk = Sketch((-0.16, 0.44), (-0.16, 0.34))
    S, W = (0.0, 0.0), (rho, h)
    sk.line(-0.16, 0, 0.44, 0, GRAY, 1, dash="4 4")
    sk.line(0, -0.16, 0, 0.34, GRAY, 1, dash="4 4")
    for sign, color, name in ((1, BLUE, "elbow one way (q4 > 0)"), (-1, ORANGE, "elbow the other way (q4 < 0)")):
        q4 = sign * math.acos(D)
        q2 = math.atan2(rho, h) - math.atan2(L2 * math.sin(q4), L1 + L2 * math.cos(q4))
        E = (L1 * math.sin(q2), L1 * math.cos(q2))
        sk.line(*S, *E, color, 5, cap="round")
        sk.line(*E, *W, color, 5, cap="round")
        sk.dot(*E, 6, color, "#ffffff", 2.5)
        sk.text(E[0], E[1], name, 12, color, "start" if sign < 0 else "end", dx=12 if sign < 0 else -12, dy=4)
    sk.line(*S, *W, GRAY, 1.5, dash="6 4")
    sk.dot(*S, 6, INK, "#ffffff", 2.5)
    sk.dot(*W, 5, RED)
    sk.text(*S, "S (shoulder)", 12, INK, "end", dx=-10, dy=18)
    sk.text(*W, "W (wrist centre)", 12, RED, dx=10, dy=4)
    sk.line(rho, 0, rho, h, GRAY, 1.2, dash="3 3")
    sk.text(rho * 0.72, 0, "rho (horizontal distance)", 12, GRAY, "middle", dy=-6)
    sk.text(rho, h / 2, "h", 12, GRAY, dx=6, dy=4)
    sk.text(0, 0.31, "z (up): q2 is measured from here", 12, GRAY, dx=8)
    sk.text(-0.15, -0.13, f"L1 = {L1 * 1000:.0f} mm, L2 = {L2 * 1000:.0f} mm.   cos q4 = (rho^2 + h^2 - L1^2 - L2^2) / (2 L1 L2)",
            12.5, INK)
    sk.save("ik_triangle.svg")


def iso(p):
    """Oblique projection of a 3-D point for the swivel figure."""
    return p[0] * 0.92 - p[1] * 0.55, p[2] + 0.28 * p[0] + 0.42 * p[1]


def fig_swivel(arm):
    sk = Sketch((-0.22, 0.52), (-0.02, 0.62))
    q0 = np.deg2rad([0, 45, 0, 80, 0, 40, 0])
    elbows = []
    for psi in np.deg2rad(np.arange(-150, 151, 30)):
        res = swivel(arm, q0, psi)
        if not res.success:
            continue
        S, E, W = arm.srs_points(res.q)
        F = arm.frames(res.q)
        chain = [np.zeros(3), S, E, W, F[arm.n][:3, 3], F[-1][:3, 3]]
        main = abs(psi) < 1e-9
        xy = [iso(p) for p in chain]
        for (x0, y0), (x1, y1) in zip(xy[:-1], xy[1:]):
            sk.line(x0, y0, x1, y1, BLUE if main else GRAY, 5 if main else 2.5, cap="round", opacity=1 if main else 0.45)
        elbows.append(iso(E))
        sk.dot(*iso(E), 4, ORANGE)
    S, _, W = arm.srs_points(q0)
    sk.line(*iso(S), *iso(W), RED, 1.8, dash="6 4")
    sk.poly([e[0] for e in elbows], [e[1] for e in elbows], ORANGE, 1.5, dash="3 3")
    sk.text(*iso(S), "S", 13, INK, "end", dx=-10, dy=4, bold=True)
    sk.text(*iso(W), "W", 13, INK, dx=10, dy=4, bold=True)
    sk.text(*elbows[len(elbows) // 2], "elbow circle (psi from -150 to 150 deg)", 12, ORANGE, dx=10, dy=-10)
    sk.text(-0.21, 0.59, "Same tool pose, different elbow: the elbow swings on a circle about the S-W axis",
            13, INK, bold=True)
    sk.save("swivel_circle.svg")


def fig_sew(arm):
    """psi as the wrist passes over the shoulder: the old 'projected +z' reference flips 180 deg, the stereographic
    one (docs/14 §6) doesn't."""

    def old_psi(q):                                 # the pre-26 Oct definition, for comparison
        S, E, W = arm.srs_points(q)
        u = (W - S) / np.linalg.norm(W - S)
        v = np.array([0.0, 0, 1]) - u[2] * u
        e = (E - S) - ((E - S) @ u) * u
        return math.atan2(np.cross(v, e) @ u, v @ e)

    q2s = np.arange(-60, 0.01, 0.25)
    qs = [np.deg2rad([0, a, 0, 60, 0, 0, 0]) for a in q2s]
    p = Plot((-60, 0), (-200, 200), "J2 (deg), elbow bent 60 deg, J1 = J3 = 0", "arm angle psi (deg)",
             "Wrist passing over the shoulder: old psi flips 180 deg, stereographic psi stays put")
    old = [math.degrees(old_psi(q)) for q in qs]
    p.curve(q2s, [o + 360 if o < -179 else o for o in old], RED, "old: angle from projected +z")  # -180 = +180
    p.curve(q2s, [math.degrees(swivel_angle(arm, q)) for q in qs], BLUE, "stereographic SEW (singular only straight down)")
    p.line(-29, -200, -29, 200, GRAY, 1, dash="4 4")
    p.text(-29, 170, "wrist straight above the shoulder", 12, GRAY, dx=8)
    p.legend(0.03, 0.72)
    p.save("sew_over_the_top.svg")


def fig_psi_feasible(arm):
    """For three tool poses: how many closed-form solutions fit the joint limits at each arm angle."""
    poses = [("example (0, 45, 0, 80, 0, 40, 0)", [0, 45, 0, 80, 0, 40, 0]),
             ("home (0, 30, 0, 60, 0, 60, 0)", [0, 30, 0, 60, 0, 60, 0]),
             ("long reach (0, 70, 0, 25, 0, 60, 0)", [0, 70, 0, 25, 0, 60, 0])]
    p = Plot((-180, 180), (0, 3.7), "arm angle psi (deg)", "rows: one tool pose each",
             "Which arm angles work: solutions inside the joint limits (bar height 1-8)")
    psis = np.arange(-180, 180, 2.0)
    for row, (label, qd) in enumerate(poses):
        T = arm.fk(np.deg2rad(qd))
        y0 = 2.45 - row * 1.15
        for psi in psis:
            n = len(analytic_ik_7dof(arm, T, math.radians(psi)))
            if n:
                p.line(psi, y0, psi, y0 + 0.08 * n, BLUE, 3.5)
        p.line(-180, y0, 180, y0, GRAY, 1)
        p.text(-178, y0 + 0.72, label, 12, INK)
        psi0 = math.degrees(swivel_angle(arm, np.deg2rad(qd)))
        p.line(psi0, y0 - 0.05, psi0, y0 + 0.68, ORANGE, 1.5, dash="3 3")
    p.text(60, 3.45, "orange: the pose's own psi", 12, ORANGE)
    p.save("psi_feasible.svg")


def fig_elbow_plan(arm):
    """The docs/14 §6 Step 5 example: warm-started IK vs the planned arm angle along one straight line."""
    from deskarm.cartesian import linear_path
    from deskarm.collision import min_clearance

    q0 = np.deg2rad([114.14, 74.19, -64.1, 78.94, -73.63, 1.09, -1.32])
    T1 = arm.fk(q0)
    T1[:3, 3] += [-0.0563, -0.103, 0.0966]
    p = Plot((0, 100), (-40, 60), "along the line (%)", "smallest capsule clearance (mm)",
             "Same straight line, two ways to place the elbow")
    for elbow, color, label in (("keep", RED, "warm-started IK: psi drifts 20 -> 63 deg, J7 motor hits the J2 module"),
                                ("plan", BLUE, "planned psi: 20 -> -40 deg, stays 14 mm clear")):
        qs = linear_path(arm, q0, T1, elbow=elbow)[0]
        xs = np.linspace(0, 100, len(qs))
        p.curve(xs, [1000 * min_clearance(arm, q)[0] for q in qs], color, label)
    p.line(0, 0, 100, 0, GRAY, 1, dash="4 4")
    p.legend(0.03, 0.74)
    p.save("elbow_plan.svg")


def fig_dls():
    lam = 0.02
    s = np.linspace(0.001, 0.2, 400)
    p = Plot((0, 0.2), (0, 60), "singular value sigma of the Jacobian (small = near a singularity)",
             "joint motion per unit of task error", "Why the IK is damped (lambda = 0.02, as in solve_dls)")
    p.curve(s, 1 / s, RED, "plain pseudo-inverse: 1 / sigma (blows up)")
    p.curve(s, s / (s**2 + lam**2), BLUE, "damped: sigma / (sigma^2 + lambda^2)")
    p.line(lam, 0, lam, 60, GRAY, 1, dash="4 4")
    p.text(lam, 52, "sigma = lambda: peak 1 / (2 lambda) = 25", 12, GRAY, dx=8)
    p.legend(0.42, 0.25)
    p.save("dls_damping.svg")


def fig_profiles():
    h, v, a, j = math.radians(60), math.radians(75), math.radians(150), math.radians(1200)
    tr = TrapezoidalTrajectory([0.0], [h], [v], [a])
    ds = DoubleS(h, v, a, j)
    T = max(tr.duration, ds.duration)
    ts = np.linspace(0, T, 600)
    trap = np.array([[tr.sample(t)[0][0], tr.sample(t)[1][0]] for t in ts])
    acc_t = np.gradient(trap[:, 1], ts)
    dbl = np.array([ds.sample(t) for t in ts])
    for col, name, unit, lim in ((0, "position", "deg", (0, 70)), (1, "velocity", "deg/s", (0, 90)),
                                 (2, "acceleration", "deg/s^2", (-180, 180))):
        p = Plot((0, T), lim, "time (s)", f"{name} ({unit})",
                 f"One joint, 60 deg move: {name}  (limits 75 deg/s, 150 deg/s^2, 1200 deg/s^3)", h=300)
        y_trap = trap[:, col] if col < 2 else acc_t
        p.curve(ts, np.degrees(y_trap), ORANGE, f"trapezoid ({tr.duration:.2f} s)")
        p.curve(ts, np.degrees(dbl[:, col]), BLUE, f"S-curve / double-S ({ds.duration:.2f} s)")
        p.legend(0.62, 0.42 if col == 0 else 0.05)
        p.save(f"profile_{name}.svg")


def fig_motor(arm):
    rpm = np.linspace(0, 1500, 400)
    w = rpm * 2 * np.pi / 60
    p = Plot((0, 1500), (0, 2.1), "motor speed (rpm)", "torque at the motor shaft (N m)",
             "Stepper torque vs speed at 24 V (ideal envelope, deskarm.motor)")
    motors = [("NEMA23 76 mm, 1.9 N m (J2)", arm.joints[1], BLUE), ("LDO NEMA17 60 mm, 0.85 N m (J1, J3, J4)", arm.joints[0], ORANGE),
              ("LDO NEMA17 40 mm, 0.43 N m (J5, J6)", arm.joints[4], GREEN), ("LDO 20 mm pancake, 0.18 N m (J7)", arm.joints[6], PURPLE)]
    for label, j, color in motors:
        t = stepper_torque(w, j.hold_torque, j.rated_current, j.resistance, j.inductance, arm.drive.voltage)
        p.curve(rpm, t, color, label)
    j = arm.joints[1]
    t = stepper_torque(w, j.hold_torque, j.rated_current, j.resistance, j.inductance, arm.drive.voltage)
    power = float(np.max(t * w))
    p.curve(rpm[40:], power / w[40:], BLUE, f"constant power, {power:.0f} W", 1.2, "5 4")
    p.legend(0.47, 0.05)
    p.save("motor_torque_speed.svg")

    j = arm.joints[1]
    v = np.linspace(0, 110, 300)
    lim = np.array([j.torque_at(math.radians(x)) for x in v])
    static = abs(arm.gravity_torque(np.deg2rad([0, 90, 0, 0, 0, 0, 0]), 1.0)[1])
    p = Plot((0, 110), (0, 50), "J2 joint speed (deg/s)", "torque at the joint (N m)",
             "J2: what the drive can deliver vs what the load needs (1 kg, arm stretched out)")
    p.curve(v, lim, BLUE, "available: 0.75 x motor x 48:1 x 0.63")
    for y, label, color, dash in ((static * 1.95, "design = static x 1.5 x 1.3", RED, "7 4"),
                                  (static * 1.3, "hold = static x 1.3", ORANGE, "7 4"),
                                  (static, "static load", GREEN, None)):
        p.curve([0, 110], [y, y], color, f"{label} = {y:.1f} N m", 1.8, dash)
    vmax = math.degrees(j.vmax)
    p.line(vmax, 0, vmax, 50, GRAY, 1.2, dash="4 4")
    p.text(vmax, 3, f"vmax = {vmax:.0f} deg/s", 12, GRAY, dx=6)
    p.legend(0.54, 0.02)
    p.save("j2_torque_vs_speed.svg")


def fig_mass(arm):
    cfg = load_config()
    z0 = cfg["joints"][0]["d"]
    sk = Sketch((-0.06, 0.60), (-0.30, 0.14))
    sk.line(0, 0, 0.52, 0, GRAY, 6, cap="round", opacity=0.5)
    sk.dot(0, 0, 8, INK, "#ffffff", 2.5)
    sk.text(0, 0, "J2 axis", 12, INK, "middle", dy=-16)
    rows = [(m["name"].split(" (")[0], m["mass"], m["pos"][2] - z0) for m in cfg["masses"] if m["on"] not in ("base", "J1")]
    rows.append(("payload 1 kg", 1.0, 0.52))
    total = sum(m * G * r for _, m, r in rows)
    for i, (name, m, r) in enumerate(rows):
        color = RED if name.startswith("payload") else (ORANGE if "actuator" in name else BLUE)
        sk.dot(r, 0, 3 + 13 * math.sqrt(m), color, color, 1, 0.75)
        y = -0.045 - 0.025 * i
        tau = m * G * r
        sk.line(0.0, y, tau / total * 0.9, y, color, 9)
        sk.text(tau / total * 0.9, y, f"{tau:.2f} N m  {name} ({m:.2f} kg at {r * 1000:.0f} mm)", 11.5, INK, dx=8, dy=4)
    sk.text(-0.05, 0.11, f"Arm stretched out: every lump pulls on J2 with m g r. Total {total:.1f} N m with 1 kg",
            13, INK, bold=True)
    sk.text(-0.05, 0.085, "circle area = mass; orange = actuators, blue = printed parts and tool, red = payload", 11.5, GRAY)
    sk.save("mass_levers.svg")


def fig_counterbalance(arm):
    cb = arm.counterbalance
    a, b = cb.anchor_height, cb.attach_distance
    th = math.radians(60)
    sk = Sketch((-0.30, 0.42), (-0.07, 0.26), w=620)
    S, A = (0.0, 0.0), (0.0, a)
    Bp = (b * math.sin(th), b * math.cos(th))
    tip = (0.34 * math.sin(th), 0.34 * math.cos(th))
    sk.line(0, 0, 0, 0.22, GRAY, 1, dash="4 4")
    sk.line(*S, *tip, INK, 6, cap="round")
    sk.line(*A, *Bp, GREEN, 2.5, dash="2 3")
    sk.line(*S, *A, GRAY, 3)
    for pt, name, dx, dy in ((S, "S  (J2 axis)", -12, 16), (A, "A  anchor on the turntable", -12, 4), (Bp, "B", 6, 20)):
        sk.dot(*pt, 5, INK, "#ffffff", 2)
        sk.text(*pt, name, 12, INK, "end" if dx < 0 else "start", dx=dx, dy=dy)
    sk.text(-0.012, a / 2, "a", 13, GRAY, "end", italic=True)
    sk.text(Bp[0] / 2, Bp[1] / 2, "b", 13, GRAY, dx=8, dy=16, italic=True)
    sk.text(A[0] + 0.03, A[1] + 0.012, "spring + cable, tension k |AB|", 12, GREEN, dx=0, dy=-8)
    sk.text(0.022, 0.050, "theta", 12, GRAY, italic=True)
    sk.text(tip[0], tip[1], "upper arm", 12, INK, dx=8)
    sk.text(-0.29, -0.055, "torque about S = k a b sin(theta): the same shape as gravity, m g l sin(theta)", 12.5, INK, bold=True)
    sk.save("counterbalance_geometry.svg")

    ang = np.linspace(-120, 120, 241)
    bare = replace(arm, counterbalance=None)

    def grav(payload):
        return np.array([bare.gravity_torque(np.deg2rad([0, x, 0, 0, 0, 0, 0]), payload)[1] for x in ang])

    spring = np.array([spring_torque(arm, np.deg2rad([0, x, 0, 0, 0, 0, 0]))[1] for x in ang])
    p = Plot((-120, 120), (-20, 20), "J2 angle from vertical (deg), arm straight", "torque the J2 motor must supply (N m)",
             f"J2 with a straight arm: gravity, and what is left with the spring (k = {cb.stiffness:.0f} N/m)")
    p.curve(ang, grav(1.0), RED, "no spring, 1 kg payload")
    p.curve(ang, grav(0.0), ORANGE, "no spring, empty gripper")
    p.curve(ang, grav(1.0) - spring, BLUE, "with spring, 1 kg", dash="7 4")
    p.curve(ang, grav(0.0) - spring, GREEN, "with spring, empty", dash="7 4")
    p.legend(0.56, 0.04)
    p.save("counterbalance_torque.svg")


def fig_two_mass(arm):
    c = Canvas(680, 190)
    c.items.append(f'<rect x="40" y="60" width="130" height="70" rx="6" fill="#ddf4ff" stroke="{BLUE}" stroke-width="2"/>')
    c.items.append(f'<rect x="470" y="60" width="170" height="70" rx="6" fill="#fff1e5" stroke="{ORANGE}" stroke-width="2"/>')
    c.text(105, 90, "motor side", 13, INK, "middle", bold=True)
    c.text(105, 110, "rotor inertia x ratio^2", 12, GRAY, "middle")
    c.text(555, 90, "link side", 13, INK, "middle", bold=True)
    c.text(555, 110, "arm inertia + gravity load", 12, GRAY, "middle")
    xs = [170, 200] + [205 + 12 * i for i in range(14)] + [375, 400]
    ys = [95, 95] + [80 if i % 2 == 0 else 110 for i in range(14)] + [95, 95]
    c.poly(xs, ys, GREEN, 2.2)
    c.line(400, 75, 400, 115, GREEN, 2.5)
    c.line(430, 75, 430, 115, ORANGE, 2.5)
    c.line(430, 95, 470, 95, ORANGE, 2.5)
    c.text(285, 68, "gear stiffness k (N m / rad)", 12.5, GREEN, "middle")
    c.text(415, 140, "gap = backlash b", 12.5, INK, "middle")
    c.arrow(60, 40, 150, 40, BLUE, 2)
    c.text(105, 30, "servo torque (limited)", 12, BLUE, "middle")
    c.text(40, 170, "read_motor() measures the left block, read() the right one. Their difference is tau / k + b / 2.", 12.5, INK)
    c.save("two_mass_joint.svg")

    j = arm.joints[1]
    k, b = j.gear_stiffness, math.degrees(j.backlash)
    tau = np.linspace(-20, 20, 401)
    defl = np.degrees(tau / k) + np.sign(tau) * b / 2
    p = Plot((-20, 20), (-2.6, 2.6), "torque on the joint (N m)", "link angle - motor angle (deg)",
             f"J2 gearbox: wind-up and backlash (k = {k:.0f} N m/rad, backlash {b:.1f} deg, placeholders)")
    p.curve(tau, defl, BLUE, "tau / k + sign(tau) b / 2")
    p.curve([0, 0], [-b / 2, b / 2], RED, f"dead band: {b:.1f} deg of free play", 3)
    p.legend(0.05, 0.05)
    p.save("gear_windup.svg")


def fig_capsules(arm):
    sk = Sketch((-0.22, 1.02), (-0.10, 0.62))
    poses = [(arm.home, 0.0, "home pose: every pair clear"),
             (np.deg2rad([0, -30, 0, 120, 0, 90, 0]), 0.62, "tool folded back: tool hits shoulder")]
    for q, dx, label in poses:
        for cap in capsules(arm, q):
            color = RED if cap.name == "tool" else (ORANGE if cap.name in ("shoulder", "elbow", "wrist") else BLUE)
            sk.line(cap.p0[0] + dx, cap.p0[2], cap.p1[0] + dx, cap.p1[2], color, 2 * cap.radius * sk.s, cap="round", opacity=0.45)
            sk.line(cap.p0[0] + dx, cap.p0[2], cap.p1[0] + dx, cap.p1[2], INK, 1.2)
        sk.text(dx - 0.18, 0.59, label, 13, INK, bold=True)
    sk.line(-0.22, 0, 1.02, 0, GRAY, 1.5)
    sk.text(-0.2, -0.07, "A capsule = a segment plus a radius. Two bodies collide when segment distance < r1 + r2.", 12, GRAY)
    sk.save("capsules.svg")


def fig_gears():
    R, Rr, N, e = 50.0, 3.0, 25, 1.6
    sk = Sketch((-58, 178), (-58, 58), w=700)
    pts = cycloid.disc_profile(R, Rr + 0.1, N, e, 1200)
    sk.poly([x for x, _ in pts], [y for _, y in pts], BLUE, 1.6, close=True, fill="#ddf4ff")
    for i in range(N):
        t = 2 * math.pi * i / N
        sk.items.append(f'<circle cx="{sk.X(R * math.cos(t) - e):.1f}" cy="{sk.Y(R * math.sin(t)):.1f}" '
                        f'r="{Rr * sk.s:.1f}" fill="#ffffff" stroke="{ORANGE}" stroke-width="1.5"/>')
    sk.dot(0, 0, 3, BLUE)
    sk.dot(-e, 0, 3, ORANGE)
    sk.text(0, -8, "disc centre, offset e from the housing axis", 11.5, INK, "middle")
    sk.text(-56, 54, "Cycloidal disc: 24 lobes in 25 ring pins = 24:1", 13, INK, bold=True)
    zs, zp, zr, n, m = 12, 18, 48, 3, 1.25
    cx = 118.0
    a = m * (zs + zp) / 2
    sun = gears.gear_outline(zs, m, 20, 0.075)
    sk.poly([cx + x for x, _ in sun], [y for _, y in sun], ORANGE, 1.4, close=True, fill="#fff1e5")
    planet = gears.gear_outline(zp, m, 20, 0.075)
    for phi, ph in gears.planet_phases(zs, zp, n):
        c, s_ = math.cos(ph), math.sin(ph)
        px, py = a * math.cos(phi), a * math.sin(phi)
        sk.poly([cx + px + x * c - y * s_ for x, y in planet], [py + x * s_ + y * c for x, y in planet],
                BLUE, 1.4, close=True, fill="#ddf4ff")
    ring = gears.ring_space_outline(zr, m, 20, 0.075)
    rp = gears.ring_phase(zp, zr)
    c, s_ = math.cos(rp), math.sin(rp)
    sk.poly([cx + x * c - y * s_ for x, y in ring], [x * s_ + y * c for x, y in ring], GREEN, 1.4, close=True)
    sk.text(cx, 54, "Planetary: sun 12, planets 18, ring 48 = 1 + 48/12 = 5:1", 13, INK, "middle", bold=True)
    sk.text(cx, -52, "green outline = the ring gear's tooth spaces", 11.5, GREEN, "middle")
    sk.save("gear_profiles.svg")


def fig_topp(arm):
    try:
        from deskarm.physics import TorqueModel
        from deskarm.topp import joint_path, time_optimal
    except ImportError:
        print("skipped topp_phase_plane.svg (needs mujoco)")
        return
    bare = replace(arm, counterbalance=None)
    fast = replace(bare.joints[1], vmax=math.radians(90))
    arm2 = replace(bare, joints=bare.joints[:1] + (fast,) + bare.joints[2:])
    lo, up = np.deg2rad([0, 115, 0, 0, 0, 0, 0]), np.deg2rad([0, 35, 0, 0, 0, 0, 0])
    p = Plot((0, 1), (0, 100), "position along the path s (0 = arm low, 1 = lifted)", "J2 speed (deg/s)",
             "Time-optimal lift with J2 (speed limit raised to 90 deg/s to show the effect)")
    s_grid = np.linspace(0, 1, 201)
    for payload, color in ((0.0, GREEN), (2.0, BLUE)):
        tm = TorqueModel(arm2, payload)
        need = np.array([abs(tm.inverse(lo + s * (up - lo))[1]) for s in s_grid])
        cap = np.degrees(arm2.speed_at_torque(np.outer(need * 1.15 / 0.95, np.ones(arm2.n)))[:, 1])
        p.curve(s_grid, np.minimum(cap, 90), color, f"speed where the motor still carries the load ({payload:.0f} kg)", 1.4, "6 4")
        tr, _ = time_optimal(arm2, joint_path(lo, up), torque_model=tm)
        s_t = (tr.qs[:, 1] - lo[1]) / (up[1] - lo[1])
        v = np.abs(np.gradient(tr.qs[:, 1], tr.times))
        p.curve(s_t, np.degrees(v), color, f"time-optimal profile, {payload:.0f} kg ({tr.duration:.2f} s)")
    p.legend(0.27, 0.62)
    p.save("topp_phase_plane.svg")


def fig_motor_turns(arm):
    """§16: the grid of joint angles a motor's single-turn reading allows, and the coarse window."""
    R = arm.joints[1].ratio
    step, turn, n_true = 360 / R, 0.62, 4                   # J2; reference (0 deg, 0 turns)
    truth = step * (n_true + turn)
    coarse = truth - 2.1
    sk = Sketch((12, 58), (-6, 22))
    sk.text(13, 20.5, f"J2 (R = {R:g}): after power-up the motor reads {turn:.2f} of a turn, so the joint can only be",
            13, INK, bold=True)
    sk.text(13, 18.6, f"at θ0 + {step:g}°·(n + {turn:.2f}), a grid {step:g}° apart. The coarse reading picks the point.",
            13, INK, bold=True)
    sk.poly([coarse - step / 2, coarse + step / 2, coarse + step / 2, coarse - step / 2], [0.5, 0.5, 13.5, 13.5],
            ORANGE, 1.2, fill=ORANGE, close=True, opacity=0.18)
    sk.text(coarse, 14.3, f"window: coarse ± {step / 2:g}° (half a motor turn)", 12, ORANGE, "middle")
    for n in range(0, 8):
        x = step * (n + turn)
        if 12.5 < x < 57.5:
            hit = n == n_true
            sk.line(x, 1.0, x, 10.0, GREEN if hit else GRAY, 4 if hit else 1.6)
            sk.text(x, 10.0, f"n = {n}", 12, GREEN if hit else GRAY, "middle", dy=-8, bold=hit)
    sk.line(coarse, 1.0, coarse, 12.5, RED, 2, dash="5 4")
    sk.dot(coarse, 6.0, 5, RED)
    sk.text(coarse, 6.0, "coarse θc", 12, RED, "end", dx=-9, dy=4)
    sk.text(truth, 4.0, f"θ = {truth:.2f}°", 12, GREEN, "start", dx=8, bold=True)
    sk.line(12.5, 0, 57.5, 0, INK, 1.2)
    for t in range(15, 58, 5):
        sk.line(t, -0.5, t, 0.5, INK, 1.2)
        sk.text(t, 0, f"{t}°", 11.5, GRAY, "middle", dy=20)
    sk.text(35, -4.5, "joint angle", 12.5, INK, "middle")
    sk.save("motor_turns.svg")


def fig_push_test(arm):
    """§17: push-test hysteresis against torque, and the line that gives stiffness and backlash."""
    from deskarm.bench import push_fit

    j = arm.joints[1]
    k, b = j.gear_stiffness, math.degrees(j.backlash)
    lsb = 360 / 16384                                        # one MT6701 step
    tau = np.array([2.0, 4.0, 6.0, 8.0])
    h = np.round((np.degrees(2 * tau / k) + b) / lsb) * lsb   # what a 14-bit encoder would read
    k_fit, b_fit = push_fit(tau, np.radians(h))
    p = Plot((0, 9), (0, 2.5), "torque hung on the lever, tau (N*m at the joint)",
             "hysteresis h (deg)", f"Push test on J2 (placeholders k = {k:g} N*m/rad, b = {b:g} deg)")
    xs = np.array([0, 9])
    p.curve(xs, np.degrees(2 * xs / k_fit + b_fit), BLUE, f"fit h = 2 tau / k + b: k = {k_fit:.0f}, b = "
            f"{math.degrees(b_fit):.2f} deg")
    for t, y in zip(tau, h):
        p.dot(t, y, 5, RED)
    p.legend_items.append(("measured: +tau and -tau, one point per level", RED, None))
    p.dot(0, math.degrees(b_fit), 5, ORANGE)
    p.text(0, math.degrees(b_fit), "intercept = backlash b", 12, ORANGE, dx=10, dy=18)
    p.text(5.0, np.degrees(2 * 5.0 / k_fit + b_fit), "slope = 2 / k", 12, BLUE, dx=12, dy=26)
    p.legend(0.03, 0.06)
    p.save("push_test.svg")


def fig_encoder_cal(arm):
    """§18: what a J2 calibration sweep sees (reading minus motor angle) and the fitted sensor error."""
    from deskarm.encoders import fit, gearbox_torque

    zero, a1, b2, h, k = math.radians(123), math.radians(0.4), math.radians(0.15), math.radians(0.5), 600.0
    up = np.radians(np.linspace(-115, 115, 231))
    log, bands = {"motor": [], "turn": [], "direction": [], "torque": []}, []
    for elbow in (0, 90):
        for d, sw in ((1, up), (-1, up[::-1])):
            ys = []
            for th in sw:
                q = np.array(arm.home, float)
                q[1], q[3] = th, math.radians(elbow)
                tau = gearbox_torque(arm, q, 1)
                link = th - d * h / 2 - tau / k
                turn = np.round((link + zero + a1 * math.cos(link) + b2 * math.sin(2 * link)) / (2 * np.pi) % 1
                                * 16384) / 16384
                for key, v in zip(log, (th, turn, d, tau)):
                    log[key].append(v)
                ys.append(math.degrees((2 * np.pi * turn - th - zero + np.pi) % (2 * np.pi) - np.pi))
            bands.append((elbow, d, np.degrees(sw), ys))
    cal = fit(**log, harmonics=2)
    p = Plot((-120, 120), (-2.4, 1.2), "motor-implied joint angle (deg)", "reading - motor angle - zero (deg)",
             "J2 calibration sweep: two poses, both directions (simulated, 14-bit readings)")
    for elbow, d, xs, ys in bands:
        p.curve(xs, ys, BLUE if elbow == 0 else ORANGE, f"elbow {elbow} deg, moving {'up' if d > 0 else 'down'}",
                1.6, None if d > 0 else "5 4")
    th = np.radians(np.linspace(-120, 120, 241))
    p.curve(np.degrees(th), np.degrees(cal.error(th) - cal.c[0]), INK,
            f"fitted sensor error c(theta) - c0 (lost motion {math.degrees(cal.lost_motion):.2f} deg, "
            f"k = {cal.stiffness:.0f})", 3.0)
    p.legend(0.02, 0.68)
    p.save("encoder_cal.svg")


def fig_dual_loop(arm):
    """§19: J2's link error holding the stretched arm with 1 kg: motors only, the dual loop, too much gain."""
    try:
        from deskarm.control import DualLoop
        from deskarm.physics import MujocoBackend
    except ImportError:
        print("skipped dual_loop.svg (needs mujoco)")
        return
    q, dt, lsb = np.radians([0, 90, 0, 0, 0, 0, 0]), 0.02, 2 * np.pi / 16384

    class Encoder:                                       # 14 bits, one poll old
        def __init__(self, be):
            self.be, self.last = be, be.read()

        def __getattr__(self, name):
            return getattr(self.be, name)

        def read(self):
            return np.round(self.last / lsb) * lsb

        def step(self, h):
            self.last = self.be.read()
            self.be.step(h)

    p = Plot((0, 6), (-1.5, 1.5), "time after enable (s)", "J2 link error (deg)",
             "Holding the stretched arm, 1 kg, the loop told 0 kg (14-bit encoder, one poll late)")
    for label, color, ki in (("motors only", GRAY, None), ("dual loop, ki = 1 /s", BLUE, 1.0),
                             ("dual loop, ki = 3 /s: above 2 zeta omega, rings", RED, 3.0)):
        be = MujocoBackend(arm, q0=q, payload_kg=1.0)
        c = be
        if ki is not None:
            c = DualLoop(Encoder(be))
            c.ki = ki
        c.enable()
        c.command(q)
        ts, es = [], []
        for k in range(int(6 / dt)):
            c.step(dt)
            ts.append((k + 1) * dt)
            es.append(math.degrees(be.read()[1] - q[1]))
        p.curve(ts, es, color, label, 2.2 if ki == 1.0 else 1.6)
    p.line(0, 0, 6, 0, INK, 1.0)
    p.legend(0.38, 0.72)
    p.save("dual_loop.svg")


def fig_contact(arm):
    """§20: J2's external-torque estimate through a full-speed move with 1 kg, then a 2 N push on the tool."""
    try:
        from deskarm.control import ContactGuard
        from deskarm.physics import MujocoBackend
        from deskarm.trajectory import joint_move
    except ImportError:
        print("skipped contact.svg (needs mujoco)")
        return
    dt, lsb = 0.02, 2 * np.pi / 16384

    class Encoders:                                      # both one poll old, the link in 14-bit steps
        def __init__(self, be):
            self.be, self.last = be, (be.read(), be.read_motor())

        def __getattr__(self, name):
            return getattr(self.be, name)

        def read(self):
            return np.round(self.last[0] / lsb) * lsb

        def read_motor(self):
            return self.last[1]

        def step(self, h):
            self.last = (self.be.read(), self.be.read_motor())
            self.be.step(h)

    be = MujocoBackend(arm, payload_kg=1.0, streamed=True)
    g = ContactGuard(Encoders(be), payload_kg=1.0)
    g.enable()
    tr = joint_move(arm, arm.home, np.radians([40, 70, 20, 80, 0, 30, 0]), "scurve")
    t_push = tr.duration + 1.0
    ts, static, obs, trip = [], [], [], None
    for k in range(int((t_push + 1.0) / dt)):
        t = k * dt
        g.command(tr.sample(min(t, tr.duration))[0])
        be.tool_force = np.array([0.0, 0.0, -2.0 if t >= t_push else 0.0])
        g.step(dt)
        ts.append(t + dt)
        static.append(g.static_external(be.read(), be.read_motor())[1])
        obs.append(g.external[1])
        if g.contact and trip is None:
            trip = (t + dt, g.external[1])
    th = g.threshold[1]
    p = Plot((0, ts[-1]), (-1.5, 1.5), "time (s)", "J2 external torque estimate (N·m)",
             "Full-speed move with 1 kg, then a 2 N push down on the tool (50 Hz, 14-bit, one poll late)")
    p.curve(ts, static, GRAY, "static estimate: gravity - spring - k (motor - link)", 1.6)
    p.curve(ts, obs, BLUE, "momentum observer (ContactGuard.external)", 2.4)
    p.curve([0, ts[-1]], [th, th], RED, f"threshold +-{th:.2f} N·m = k LSB + margin", 1.4, "6 4")
    p.curve([0, ts[-1]], [-th, -th], RED, None, 1.4, "6 4")
    p.line(tr.duration, -1.5, tr.duration, 1.5, INK, 1.0, "3 3")
    p.text(tr.duration, 1.5, "move ends", 12, INK, "middle", dy=14)
    p.line(t_push, -1.5, t_push, 1.5, ORANGE, 1.0, "3 3")
    p.text(t_push, 1.5, "2 N push", 12, ORANGE, "middle", dy=14)
    if trip:
        p.dot(*trip, 5, RED)
        p.text(trip[0], trip[1], f"trips {trip[0] - t_push:.2f} s after the push", 12, RED, "end", dx=-8, dy=-26)
    p.legend(0.30, 0.74)
    p.save("contact.svg")


def fig_ccd(arm):
    import ik_study
    q_true = np.radians([30, 50, 20, 80, -40, 60, 30])
    seed = arm.clamp(q_true + np.radians([25, -20, 30, -25, 35, -30, 40]))
    T = arm.fk(q_true)
    hd, hc = [], []
    ik_study.dls(arm, T, seed, history=hd)
    ik_study.ccd(arm, T, seed, sweeps=200, history=hc)
    p = Plot((0, 200), (-2, 3), "iteration (DLS) or sweep through all 7 joints (CCD)", "log10 of the position error (mm)",
             "Same target, same seed: damped least squares vs cyclic coordinate descent")
    p.curve(np.arange(len(hd)), np.log10(np.maximum(np.array(hd) * 1e3, 1e-3)), BLUE, f"DLS: {len(hd) - 1} iterations")
    p.curve(np.arange(1, len(hc) + 1), np.log10(np.maximum(np.array(hc) * 1e3, 1e-3)), ORANGE,
            f"CCD: {len(hc)} sweeps" + ("" if hc[-1] < 1e-4 else ", not converged"))
    p.line(0, -1, 200, -1, GRAY, 1, dash="4 4")
    p.text(200, -1, "0.1 mm tolerance", 12, GRAY, "end", dy=-6)
    p.legend(0.40, 0.08)
    p.save("ik_ccd.svg")


def main():
    arm = load_arm()
    fig_arm_frames(arm)
    fig_jacobian(arm)
    fig_ik_triangle(arm)
    fig_swivel(arm)
    fig_sew(arm)
    fig_psi_feasible(arm)
    fig_elbow_plan(arm)
    fig_dls()
    fig_profiles()
    fig_motor(arm)
    fig_mass(arm)
    fig_counterbalance(arm)
    fig_two_mass(arm)
    fig_capsules(arm)
    fig_gears()
    fig_topp(arm)
    fig_motor_turns(arm)
    fig_push_test(arm)
    fig_encoder_cal(arm)
    fig_dual_loop(arm)
    fig_contact(arm)
    fig_ccd(arm)


if __name__ == "__main__":
    main()

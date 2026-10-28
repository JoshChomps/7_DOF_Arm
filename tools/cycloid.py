#!/usr/bin/env python3
"""
Cycloidal reducer disc generator: checks the design and writes a DXF for Fusion 360.

    python3 tools/cycloid.py                          # J2-style defaults
    python3 tools/cycloid.py --pins 26 --R 40 --Rr 2.5 --e 1.2 --out cad/cycloid_j1.dxf

Import the DXF in Fusion (Insert > Insert DXF, units = mm). Layers:
    DISC          cycloidal disc profile (closed polyline)
    PINS          ring pins (fixed housing)
    OUTPUT_HOLES  holes in the disc for the output pins (dia = output pin dia + 2e)
    OUTPUT_PINS   output pins (on the output flange)
    BEARING       eccentric bearing bore in the disc
    INPUT_AXIS    small marker on the motor/housing axis

Drawn in the assembled position: the disc (DISC, OUTPUT_HOLES, BEARING) is centred on the
origin, and the housing axis (ring PINS, OUTPUT_PINS, INPUT_AXIS) sits at (-e, 0). Copy the
disc layers into the disc sketch and the pin layers into the housing / output-flange sketches.

Ratio with fixed ring pins and output through the disc holes: (pins - 1) : 1, output turning
opposite to the input. Use two discs 180 deg out of phase to cancel the wobble (docs/03).

No dependencies (pure Python).

The profile equations and the undercut check are explained in docs/14_math_guide.md §15.
"""
import argparse
import math
from pathlib import Path


def disc_profile(R, Rr, N, e, n_points=2000):
    """Points of the disc outline (standard parametric form, Lehmann / Younis)."""
    pts = []
    k = R / (e * N)
    for i in range(n_points):
        t = 2 * math.pi * i / n_points
        psi = math.atan2(math.sin((1 - N) * t), k - math.cos((1 - N) * t))
        x = R * math.cos(t) - Rr * math.cos(t + psi) - e * math.cos(N * t)
        y = -R * math.sin(t) + Rr * math.sin(t + psi) + e * math.sin(N * t)
        pts.append((x, y))
    return pts


def min_convex_radius(R, N, e, n=20000):
    """Smallest radius of curvature on the convex parts of the pitch curve.

    The disc is the pitch curve offset inward by the pin radius; if the pin radius exceeds
    this value the profile self-intersects (undercut / cusps).
    """
    def d1(t):
        return (-R * math.sin(t) + e * N * math.sin(N * t), -R * math.cos(t) + e * N * math.cos(N * t))

    def d2(t):
        return (-R * math.cos(t) + e * N * N * math.cos(N * t), R * math.sin(t) - e * N * N * math.sin(N * t))

    ks, turning = [], 0.0
    for i in range(n):
        t = 2 * math.pi * i / n
        (x1, y1), (x2, y2) = d1(t), d2(t)
        speed = math.hypot(x1, y1)
        k = (x1 * y2 - y1 * x2) / speed**3
        ks.append(k)
        turning += k * speed                       # total turning = integral of k ds
    orientation = math.copysign(1.0, turning)      # convex parts curve the same way as the loop
    convex = [abs(k) for k in ks if k * orientation > 0]
    return 1.0 / max(convex) if convex else math.inf


def dxf(entities):
    out = ["0", "SECTION", "2", "ENTITIES"]
    for layer, kind, data in entities:
        if kind == "polyline":
            out += ["0", "POLYLINE", "8", layer, "66", "1", "70", "1", "10", "0", "20", "0", "30", "0"]
            for x, y in data:
                out += ["0", "VERTEX", "8", layer, "10", f"{x:.5f}", "20", f"{y:.5f}", "30", "0"]
            out += ["0", "SEQEND", "8", layer]
        elif kind == "circle":
            (x, y), r = data
            out += ["0", "CIRCLE", "8", layer, "10", f"{x:.5f}", "20", f"{y:.5f}", "30", "0", "40", f"{r:.5f}"]
    out += ["0", "ENDSEC", "0", "EOF"]
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--pins", type=int, default=31, help="number of ring pins N (ratio = N-1 : 1)")
    ap.add_argument("--R", type=float, default=50.0, help="ring pin circle radius (mm)")
    ap.add_argument("--Rr", type=float, default=3.0, help="ring pin radius (mm)")
    ap.add_argument("--e", type=float, default=1.3, help="eccentricity (mm)")
    ap.add_argument("--clearance", type=float, default=0.10,
                    help="shrinks the disc profile (mm); tune with printed test coupons")
    ap.add_argument("--out-pins", type=int, default=8, help="number of output pins")
    ap.add_argument("--out-pin-dia", type=float, default=6.0, help="output pin (or roller) dia (mm)")
    ap.add_argument("--out-circle", type=float, default=None, help="output pin circle radius (mm)")
    ap.add_argument("--bearing-od", type=float, default=32.0, help="eccentric bearing OD (6804 = 32 mm)")
    ap.add_argument("--out", default="cad/cycloid_disc.dxf")
    a = ap.parse_args()

    N, R, Rr, e = a.pins, a.R, a.Rr, a.e
    Rr_eff = Rr + a.clearance
    problems = []
    if e >= R / N:
        problems.append(f"e = {e} must be < R/N = {R / N:.3f} (profile forms cusps)")
    if e >= Rr:
        problems.append(f"e = {e} must be < pin radius Rr = {Rr}")
    rho = min_convex_radius(R, N, e)
    if Rr_eff >= rho:
        problems.append(f"pin radius + clearance = {Rr_eff:.2f} must be < min radius of curvature "
                        f"{rho:.2f} (undercut): use fewer/smaller pins or a bigger R")

    pts = disc_profile(R, Rr_eff, N, e)
    radii = [math.hypot(x, y) for x, y in pts]
    r_min, r_max = min(radii), max(radii)
    out_hole = a.out_pin_dia + 2 * e
    out_r = a.out_circle if a.out_circle else (a.bearing_od / 2 + r_min) / 2
    web_inner = out_r - out_hole / 2 - a.bearing_od / 2      # material between bore and holes
    web_outer = r_min - (out_r + out_hole / 2)               # material between holes and teeth
    gap = 2 * out_r * math.sin(math.pi / a.out_pins) - out_hole
    for name, val in (("bore-to-hole web", web_inner), ("hole-to-tooth web", web_outer),
                      ("hole-to-hole web", gap)):
        if val < 2.0:
            problems.append(f"{name} is only {val:.2f} mm (want >= 2 mm for a printed disc)")

    print(f"Ratio            : {N - 1}:1 (output reverses direction)")
    print(f"Ring pins        : {N} x dia {2 * Rr:.2f} mm on R = {R} mm")
    print(f"Eccentricity     : {e} mm  (limits: < R/N = {R / N:.3f}, < Rr = {Rr})")
    print(f"Min curvature rad: {rho:.2f} mm (pin radius + clearance = {Rr_eff:.2f})")
    print(f"Disc outline     : r_min {r_min:.2f} mm, r_max {r_max:.2f} mm (dia {2 * r_max:.1f})")
    print(f"Output holes     : {a.out_pins} x dia {out_hole:.2f} mm on r = {out_r:.2f} mm")
    print(f"Eccentric bearing: OD {a.bearing_od} mm, bore centre offset {e} mm")
    print(f"Webs             : bore-hole {web_inner:.2f}, hole-tooth {web_outer:.2f}, hole-hole {gap:.2f} mm")

    ents = [("DISC", "polyline", pts)]
    for i in range(N):
        t = 2 * math.pi * i / N
        ents.append(("PINS", "circle", ((R * math.cos(t) - e, R * math.sin(t)), Rr)))
    for i in range(a.out_pins):
        t = 2 * math.pi * i / a.out_pins
        c = (out_r * math.cos(t), out_r * math.sin(t))
        ents.append(("OUTPUT_HOLES", "circle", (c, out_hole / 2)))
        ents.append(("OUTPUT_PINS", "circle", ((c[0] - e, c[1]), a.out_pin_dia / 2)))
    ents.append(("BEARING", "circle", ((0.0, 0.0), a.bearing_od / 2)))
    ents.append(("INPUT_AXIS", "circle", ((-e, 0.0), 0.5)))

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(dxf(ents))
    print(f"\nWrote {out}  (Fusion: Insert > Insert DXF, units mm)")
    if problems:
        print("\nPROBLEMS:")
        for p in problems:
            print("  -", p)
        raise SystemExit(1)
    print("All checks passed.")


if __name__ == "__main__":
    main()

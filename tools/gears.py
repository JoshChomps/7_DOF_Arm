#!/usr/bin/env python3
"""
Planetary gearbox calculator + involute gear generator (DXF for Fusion, herringbone STL).

Find tooth counts (ring fixed, sun in, carrier out; ratio = 1 + Zr/Zs):
    python3 tools/gears.py search --ratio 25 --module 1.25 --max-od 90
    python3 tools/gears.py search --ratio 25 --module 1.25 --max-od 90 --stages 2

Make the gears of one set (sun, planet, ring, carrier layout):
    python3 tools/gears.py make --sun 12 --planet 18 --ring 48 --planets 3 --module 1.25 \
        --face 12 --helix 30 --out cad/bench/planetary_5to1

Writes <out>_layout.dxf (all gears meshed, for Fusion: Insert > Insert DXF) and herringbone STLs
<out>_sun.stl / _planet.stl / _ring.stl (helix 0 = spur). Profiles are true involutes; backlash
(total, mm, along the pitch circle) is split between the two gears of each mesh.

Rules checked: Zr = Zs + 2 Zp, (Zs + Zr) divisible by the planet count (equal spacing),
planets don't collide, minimum teeth, ring OD. Needs: pip install numpy manifold3d

Ratio and assembly rules and the involute tooth formula: docs/14_math_guide.md §15.
"""
import argparse
import math
from pathlib import Path

from cadio import write_dxf, write_stl


# --------------------------------------------------------------------------- search ----------
def single_stage_sets(module, planets, min_teeth=12, max_teeth=90, max_od=None, wall=None,
                      planet_gap=1.0):
    """Valid simple planetary sets (ring fixed). Yields dicts sorted later by the caller."""
    wall = wall if wall is not None else max(3 * module, 4.0)
    for zs in range(min_teeth, max_teeth + 1):
        for zp in range(min_teeth, max_teeth + 1):
            zr = zs + 2 * zp
            if (zs + zr) % planets:
                continue
            a = module * (zs + zp) / 2                       # sun-planet centre distance
            chord = 2 * a * math.sin(math.pi / planets)      # planet-to-planet centre distance
            if chord <= module * (zp + 2) + planet_gap:      # planet tip circles must not touch
                continue
            ring_od = module * (zr + 2.5) + 2 * wall
            if max_od is not None and ring_od > max_od:
                continue
            yield {"zs": zs, "zp": zp, "zr": zr, "n": planets, "ratio": 1 + zr / zs,
                   "a": a, "ring_od": ring_od}


def search(target, module, planets_options=(3, 4, 5), stages=1, top=10, **kw):
    sets = [s for n in planets_options for s in single_stage_sets(module, n, **kw)]
    if stages == 1:
        sets.sort(key=lambda s: (abs(s["ratio"] - target), s["ring_od"]))
        return [(s["ratio"], [s]) for s in sets[:top]]
    sets = [s for s in sets if 2.5 <= s["ratio"] <= 10]
    pairs = []
    for i, s1 in enumerate(sets):
        for s2 in sets[i:]:                          # unordered pairs: A+B == B+A
            r = s1["ratio"] * s2["ratio"]
            if abs(r - target) / target <= 0.05:
                pairs.append((abs(r - target), max(s1["ring_od"], s2["ring_od"]), r, [s1, s2]))
    pairs.sort(key=lambda p: (round(p[0], 3), p[1]))
    return [(r, st) for _, _, r, st in pairs[:top]]


def identical_stage_options(target, module, planets_options=(3, 4, 5), top=3, **kw):
    """Two identical stages: one design printed twice (ratio = stage ratio squared)."""
    sets = [s for n in planets_options for s in single_stage_sets(module, n, **kw)]
    sets.sort(key=lambda s: (abs(s["ratio"] ** 2 - target), s["ring_od"]))
    return [(s["ratio"] ** 2, [s, s]) for s in sets[:top]]


# --------------------------------------------------------------------------- involutes -------
def _inv(a):
    return math.tan(a) - a


def gear_outline(z, module, pressure_deg=20.0, backlash=0.0, addendum=1.0, dedendum=1.25,
                 flank_points=16, arc_points=6):
    """Closed CCW outline of an external involute gear (list of (x, y), mm).

    ``backlash`` thins every tooth by backlash/2 along the pitch circle (each gear of a pair takes
    half the total). A negative value thickens teeth, which is how ring-gear tooth *spaces* are
    widened (see ring_outline).
    """
    alpha = math.radians(pressure_deg)
    r = module * z / 2
    rb = r * math.cos(alpha)
    ra = r + addendum * module
    rf = r - dedendum * module
    s_half = (math.pi * module / 2 - backlash / 2) / (2 * r)   # half tooth angle at pitch circle

    def theta(rho):  # polar half-angle of the flank at radius rho
        rho = max(rho, rb)
        return s_half + _inv(alpha) - _inv(math.acos(rb / rho))

    if theta(ra) <= 0:  # pointed tooth: cut the tip where the flanks meet
        lo, hi = r, ra
        for _ in range(60):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if theta(mid) > 0 else (lo, mid)
        ra = lo
    r_start = max(rb, rf)
    flank = [r_start + (ra - r_start) * (i / flank_points) ** 1.5 for i in range(flank_points + 1)]
    th_root = theta(r_start)

    pts = []
    pitch = 2 * math.pi / z
    for k in range(z):
        c = k * pitch
        if rf < rb:                                   # radial segment below the base circle
            pts.append((rf * math.cos(c - th_root), rf * math.sin(c - th_root)))
        for rho in flank:                             # leading flank, root -> tip
            a = c - theta(rho)
            pts.append((rho * math.cos(a), rho * math.sin(a)))
        t_tip = theta(ra)
        for i in range(1, arc_points):                # tip land
            a = c - t_tip + 2 * t_tip * i / arc_points
            pts.append((ra * math.cos(a), ra * math.sin(a)))
        for rho in reversed(flank):                   # trailing flank, tip -> root
            a = c + theta(rho)
            pts.append((rho * math.cos(a), rho * math.sin(a)))
        if rf < rb:
            pts.append((rf * math.cos(c + th_root), rf * math.sin(c + th_root)))
        a0, a1 = c + th_root, c + pitch - th_root     # root land to the next tooth
        for i in range(1, arc_points):
            a = a0 + (a1 - a0) * i / arc_points
            pts.append((rf * math.cos(a), rf * math.sin(a)))
    return pts


def ring_space_outline(z, module, pressure_deg=20.0, backlash=0.0, **kw):
    """Outline of the ring's tooth *spaces* (an external gear shape); ring = disc - this."""
    return gear_outline(z, module, pressure_deg, backlash=-backlash, addendum=1.25, dedendum=1.0, **kw)


# --------------------------------------------------------------------------- solids ----------
def _cs(points):
    from manifold3d import CrossSection
    return CrossSection([points])


def _herringbone(cs, face, twist_deg):
    from manifold3d import Manifold  # noqa: F401  (import check)
    half = cs.extrude(face / 2, n_divisions=max(1, int(abs(twist_deg) / 2)), twist_degrees=twist_deg)
    upper = half.mirror((0, 0, 1)).translate((0, 0, face))
    return half + upper


def twist_for(pitch_radius, face, helix_deg):
    """Profile rotation over half the face width for a given helix angle at the pitch circle."""
    return math.degrees((face / 2) * math.tan(math.radians(helix_deg)) / pitch_radius)


def make_set(zs, zp, zr, n, module, face=12.0, helix=30.0, backlash=0.15, pressure=20.0,
             sun_bore=5.0, planet_bore=10.0, ring_od=None):
    """Return dict of manifold solids + 2D layout for one planetary set."""
    from manifold3d import CrossSection

    if zr != zs + 2 * zp:
        raise ValueError("ring teeth must equal sun + 2 * planet")
    rs, rp, rr = module * zs / 2, module * zp / 2, module * zr / 2
    a = rs + rp
    wall = max(3 * module, 4.0)
    ring_od = ring_od or module * (zr + 2.5) + 2 * wall

    sun2d = _cs(gear_outline(zs, module, pressure, backlash / 2))
    planet2d = _cs(gear_outline(zp, module, pressure, backlash / 2))
    ring_space2d = _cs(ring_space_outline(zr, module, pressure, backlash / 2))
    if sun_bore:
        sun2d = sun2d - CrossSection.circle(sun_bore / 2, 64)
    if planet_bore:
        planet2d = planet2d - CrossSection.circle(planet_bore / 2, 64)

    sun = _herringbone(sun2d, face, +twist_for(rs, face, helix))
    planet = _herringbone(planet2d, face, -twist_for(rp, face, helix))
    ring_space = _herringbone(ring_space2d, face, -twist_for(rr, face, helix))
    from manifold3d import Manifold
    ring = Manifold.cylinder(face, ring_od / 2, circular_segments=256) - ring_space

    return {"sun": sun, "planet": planet, "ring": ring, "a": a, "ring_od": ring_od,
            "sun2d": sun2d, "planet2d": planet2d, "ring_space2d": ring_space2d,
            "planets": planet_phases(zs, zp, n), "ring_phase": ring_phase(zp, zr)}


def planet_phases(zs, zp, n):
    """(carrier angle, planet rotation) for each planet, in mesh with a sun that has a tooth on +x.

    Rolling contact at the pitch point: a sun tooth at absolute angle 0 maps to a planet *space*
    at absolute angle phi + pi + phi * zs / zp, where phi is the planet's carrier angle.
    """
    out = []
    for k in range(n):
        phi = 2 * math.pi * k / n
        out.append((phi, phi + math.pi + phi * zs / zp - math.pi / zp))
    return out


def ring_phase(zp, zr):
    """Ring rotation that meshes with planet 0 (internal contact on the +x axis)."""
    return math.pi * (zp - 1) / zr


def layout_entities(zs, zp, zr, n, module, backlash=0.15, pressure=20.0, ring_od=None,
                    sun_bore=5.0, planet_bore=10.0):
    """2D meshed layout (sun, planets, ring) for Fusion."""
    s = make_set(zs, zp, zr, n, module, face=1.0, helix=0.0, backlash=backlash, pressure=pressure,
                 sun_bore=sun_bore, planet_bore=planet_bore, ring_od=ring_od)
    ents = [("SUN", "polyline", gear_outline(zs, module, pressure, backlash / 2))]
    if sun_bore:
        ents.append(("SUN", "circle", ((0.0, 0.0), sun_bore / 2)))
    for ang, phase in s["planets"]:
        cx, cy = s["a"] * math.cos(ang), s["a"] * math.sin(ang)
        pts = gear_outline(zp, module, pressure, backlash / 2)
        c, si = math.cos(phase), math.sin(phase)
        ents.append(("PLANETS", "polyline", [(cx + x * c - y * si, cy + x * si + y * c) for x, y in pts]))
        if planet_bore:
            ents.append(("PLANETS", "circle", ((cx, cy), planet_bore / 2)))
        ents.append(("CARRIER_PINS", "circle", ((cx, cy), 0.5)))
    pts = ring_space_outline(zr, module, pressure, backlash / 2)
    c, si = math.cos(s["ring_phase"]), math.sin(s["ring_phase"])
    ents.append(("RING", "polyline", [(x * c - y * si, x * si + y * c) for x, y in pts]))
    ents.append(("RING", "circle", ((0.0, 0.0), s["ring_od"] / 2)))
    return ents, s


# --------------------------------------------------------------------------- CLI -------------
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("search", help="find tooth counts for a target ratio")
    s.add_argument("--ratio", type=float, required=True)
    s.add_argument("--module", type=float, default=1.25)
    s.add_argument("--planets", type=int, nargs="+", default=[3, 4, 5])
    s.add_argument("--stages", type=int, choices=[1, 2], default=1)
    s.add_argument("--min-teeth", type=int, default=13)
    s.add_argument("--max-od", type=float, default=None, help="max ring outer diameter (mm)")
    s.add_argument("--top", type=int, default=10)
    m = sub.add_parser("make", help="write DXF layout + STLs for one set")
    m.add_argument("--sun", type=int, required=True)
    m.add_argument("--planet", type=int, required=True)
    m.add_argument("--ring", type=int, required=True)
    m.add_argument("--planets", type=int, default=3)
    m.add_argument("--module", type=float, default=1.25)
    m.add_argument("--face", type=float, default=12.0, help="face width (mm)")
    m.add_argument("--helix", type=float, default=30.0, help="herringbone helix angle, 0 = spur")
    m.add_argument("--backlash", type=float, default=0.15, help="total backlash per mesh (mm)")
    m.add_argument("--pressure", type=float, default=20.0)
    m.add_argument("--sun-bore", type=float, default=5.0)
    m.add_argument("--planet-bore", type=float, default=10.0, help="623 bearing OD = 10 mm")
    m.add_argument("--ring-od", type=float, default=None)
    m.add_argument("--out", default="cad/planetary")
    a = ap.parse_args()

    if a.cmd == "search":
        res = search(a.ratio, a.module, tuple(a.planets), a.stages, a.top,
                     min_teeth=a.min_teeth, max_od=a.max_od)
        if not res:
            raise SystemExit("no valid sets: relax --max-od, --min-teeth or the ratio")
        print(f"module {a.module} mm, target {a.ratio}:1, {a.stages} stage(s)\n")
        for ratio, stages in res:
            desc = "  +  ".join(f"Zs{s['zs']} Zp{s['zp']} Zr{s['zr']} x{s['n']} ({s['ratio']:.3f})"
                                for s in stages)
            od = max(s["ring_od"] for s in stages)
            same = "  [identical stages]" if len(stages) == 2 and stages[0] is stages[1] else ""
            print(f"{ratio:8.3f}:1   ring OD {od:5.1f} mm   {desc}{same}")
        if a.stages == 2:
            print("\nIdentical stages (print one design twice):")
            for ratio, stages in identical_stage_options(a.ratio, a.module, tuple(a.planets),
                                                         min_teeth=a.min_teeth, max_od=a.max_od):
                st = stages[0]
                print(f"{ratio:8.3f}:1   ring OD {st['ring_od']:5.1f} mm   "
                      f"2 x Zs{st['zs']} Zp{st['zp']} Zr{st['zr']} x{st['n']} ({st['ratio']:.3f})")
        return

    ents, s = layout_entities(a.sun, a.planet, a.ring, a.planets, a.module, a.backlash, a.pressure,
                              a.ring_od, a.sun_bore, a.planet_bore)
    out = Path(a.out)
    write_dxf(out.with_name(out.name + "_layout.dxf"), ents)
    full = make_set(a.sun, a.planet, a.ring, a.planets, a.module, a.face, a.helix, a.backlash,
                    a.pressure, a.sun_bore, a.planet_bore, a.ring_od)
    for part in ("sun", "planet", "ring"):
        write_stl(out.with_name(f"{out.name}_{part}.stl"), full[part], part)
    ratio = 1 + a.ring / a.sun
    print(f"ratio {ratio:.3f}:1  centre distance {s['a']:.3f} mm  ring OD {s['ring_od']:.1f} mm")
    print(f"wrote {out}_layout.dxf and {out}_sun/_planet/_ring.stl (print {a.planets} planets)")


if __name__ == "__main__":
    main()

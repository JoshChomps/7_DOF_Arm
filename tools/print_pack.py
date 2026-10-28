#!/usr/bin/env python3
"""First prints, as Bambu Studio projects ready to open and send (docs/24).

    python tools/print_pack.py            # bench parts, motor-plate drawings, then every job below
    python tools/print_pack.py c2 bench   # only these jobs

Makes three things:
  stl/bench/C2_Stand.stl, C2_Lever.stl   a stand that holds the J2 module by its three tabs, output up, and a 105 mm
                                         lever on the output flange (pointer, hook hole for weights, Ø6 rod bore)
  stl/bench/<C2|C34>_motor_plate.dxf/.svg the 3 mm aluminium motor plate (D39), 1:1, for cutting or as a drill template
  stl/plates/<job>/plate_N.3mf           each job oriented, placed and sliced on the A1 mini (settings of
                                         slice_parts.py), with stl/plates/summary.json (hours and grams per plate)

The numbers come from tools/fusion_arm.py (imported with Fusion's API stubbed out), so the bench parts follow the CAD.
Orientation: coupons and bench parts print as modelled; the module parts use mesh_audit's least-overhang face,
except where a bearing seat would end up sideways (ORIENT).
"""
import json
import re
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path
from unittest import mock

import numpy as np
from manifold3d import CrossSection, Manifold, Mesh

for _k in ("adsk", "adsk.core", "adsk.fusion"):
    sys.modules.setdefault(_k, mock.MagicMock())
import fusion_arm as FA                                          # noqa: E402  (needs the stub above)
import slice_parts as SP                                         # noqa: E402
from cadio import write_dxf, write_stl                           # noqa: E402
from fusion_lib import circle_pts, polar, stadium                # noqa: E402
from mesh_audit import audit, load                               # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
BENCH, PLATES = ROOT / "stl" / "bench", ROOT / "stl" / "plates"
C2 = [f"DA_C2_{n}" for n in ("Housing", "Cover", "Flange", "Carrier", "Disc", "Discb", "Spacers", "Shaft", "Pulley40",
                             "Enccap")]
JOBS = {   # job -> STLs (paths from the repo root); each is sliced on as few A1 mini plates as fit
    "coupons": ["stl/coupons/pins.stl", "stl/coupons/inserts.stl", "stl/coupons/pegs.stl", "stl/coupons/bearings.stl"],
    "rings": ["stl/coupons/rings_6810.stl", "stl/coupons/rings_6806.stl"],
    "c2": [f"stl/da/{n}.stl" for n in C2],
    "bench": ["stl/bench/C2_Stand.stl", "stl/bench/C2_Lever.stl"],
}
ORIENT = {"DA_C2_Enccap": "Z+"}    # least overhang lays it on its side, which turns the MR126 seat sideways (oval)
BED = 165.0                        # usable A1 mini bed, mm (slice_parts uses 170 for a single part)


# --------------------------------------------------------------------------------------------- bench parts
def stand(m="C2"):
    """Three posts under the housing tabs (M4 inserts at the tab holes), on a T-shaped foot below the encoder cap.
    Above the motor plate (z > -28) the posts stay outside the housing's R 60.5 by 1 mm."""
    s, e = FA.cyc(m), FA.enc(m)
    r_hole = s["tab_r"][1] - 5                                   # tab hole radius (b_housing)
    z_tab = 26.0                                                 # tab underside
    zb = e["board"][0] - 5                                       # foot top, 5 mm under board C
    out = Manifold.cube((2 * r_hole + 12, 24, 8)).translate((-r_hole - 6, -12, zb - 8))
    out += Manifold.cube((24, r_hole + 6, 8)).translate((-12, -r_hole - 6, zb - 8))
    r0 = s["od"] / 2 + 1
    for a in (0, 180, 270):
        low = Manifold.cube((22, 22, -28 - zb)).translate((r_hole - 11, -11, zb))
        up = Manifold.cube((r_hole + 6 - r0, 16, z_tab + 28)).translate((r0, -8, -28))
        pocket = Manifold.cylinder(10, 5.6 / 2, circular_segments=48).translate((r_hole, 0, z_tab - 9))
        out += (low + up - pocket).rotate((0, 0, a))
    for x, y in ((-35, 0), (35, 0), (0, -40)):
        out -= Manifold.cylinder(20, 5.5 / 2, circular_segments=48).translate((x, y, zb - 10))
    return out.translate((0, 0, -(zb - 8)))                      # foot on z = 0


def lever(m="C2"):
    """Ring on the output flange's six M4 inserts, with an 18 mm deep arm: pointer tip at R 105, a Ø6 hook hole at
    R 88 for weights, and a Ø6 press bore on the axis for the encoder rod (D42). Screws: M4 x 12 (6 mm of plastic
    under the head, counterbored where the arm covers a hole)."""
    s = FA.cyc(m)
    fd = s["od"] - 4
    out = Manifold.cylinder(6, fd / 2, circular_segments=160) - Manifold.cylinder(8, 15, circular_segments=96).translate((0, 0, -1))
    arm = CrossSection([[(-24, -12), (93, -12), (105, 0), (93, 12), (-24, 12)]]).extrude(18)
    out += arm
    for x, y in circle_pts(fd / 2 - 6, 6, 0):
        out -= Manifold.cylinder(20, 4.5 / 2, circular_segments=32).translate((x, y, -1))
        out -= Manifold.cylinder(13, 4.0, circular_segments=32).translate((x, y, 6))
    for x, d in ((0, 6.0 + FA.F["press_fit"]), (88, 6.0)):
        out -= Manifold.cylinder(20, d / 2, circular_segments=48).translate((x, 0, -1))
    return out                                                   # sits on the flange face (module z = flange1)


def clash(m="C2"):
    """Interference (mm³) of the stand and lever with the module's parts, a motor-and-driver box and its pulley."""
    s, e = FA.cyc(m), FA.enc(m)
    mo = FA.MOTOR[s["motor"]]
    zb = e["board"][0] - 5
    tris = [load(ROOT / "stl" / "da" / f"DA_{m}_{n}.stl") for n in ("Housing", "Cover", "Flange", "Enccap", "Pulley40")]
    parts = [_mesh(t) for t in tris]
    h = mo["sq"] / 2
    parts.append(Manifold.cube((2 * h, 2 * h, mo["L"] + mo["drv"])).translate((-h, s["C"] - h, s["mface"])))
    half = mo["sq"] / 2 + 5
    plate = CrossSection.circle(s["od"] / 2, 160) + CrossSection.square((2 * half, s["C"] + half)).translate((-half, 0))
    parts.append(plate.extrude(3).translate((0, 0, s["mface"] - 3)))
    st = stand(m).translate((0, 0, zb - 8))
    lv = lever(m).translate((0, 0, s["flange1"]))
    return {name: round(sum((b ^ p).volume() for p in parts), 1) for name, b in (("stand", st), ("lever", lv))}


def _mesh(t):
    """Manifold from an STL's triangle soup (vertices welded at 1 µm)."""
    v, idx = np.unique(np.round(t.reshape(-1, 3), 3), axis=0, return_inverse=True)
    return Manifold(Mesh(vert_properties=v.astype(np.float32), tri_verts=idx.reshape(-1, 3).astype(np.uint32)))


# --------------------------------------------------------------------------------------------- motor plate
def plate_outline(m):
    """(outline, holes) of b_plate in fusion_arm.py: polygons as lists of (x, y), mm."""
    s, e = FA.cyc(m), FA.enc(m)
    mo = FA.MOTOR[s["motor"]]
    half = mo["sq"] / 2 + 5
    shape = CrossSection.circle(s["od"] / 2, 240) + CrossSection.square((2 * half, s["C"] + half)).translate((-half, 0))
    circ = lambda c, d: [polar(d / 2, 360 * k / 72, c) for k in range(72)]
    holes = [circ((0, 0), 21)] + [circ(polar(s["bolt_r"], a), s["screw"]) for a in s["bolt_angles"]]
    holes += [circ(p, 3.4) for p in e["posts"]] + [stadium(0, s["C"], mo["pilot"][0] + 0.4, 3)]
    b = mo["bolt"] / 2
    holes += [stadium(x, s["C"] + y, mo["screw"], 3) for x in (-b, b) for y in (-b, b)]
    return [list(map(tuple, p)) for p in shape.to_polygons()], holes


def write_plate(m):
    outline, holes = plate_outline(m)
    write_dxf(BENCH / f"{m}_motor_plate.dxf", [("OUTLINE", "polyline", p) for p in outline] +
              [("HOLES", "polyline", p) for p in holes])
    pts = [q for p in outline for q in p]
    x0, y0 = min(p[0] for p in pts) - 10, min(p[1] for p in pts) - 25
    x1, y1 = max(p[0] for p in pts) + 10, max(p[1] for p in pts) + 10
    w, h = x1 - x0, y1 - y0
    path = lambda p: "M" + " L".join(f"{x - x0:.2f},{y1 - y:.2f}" for x, y in p) + " Z"
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.1f}mm" height="{h:.1f}mm" viewBox="0 0 {w:.2f} {h:.2f}">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<path d="{" ".join(path(p) for p in outline + holes)}" fill="none" stroke="black" stroke-width="0.2"/>']
    for x, y in [(0, 0), (0, FA.cyc(m)["C"])]:                    # centre marks for the drill
        svg.append(f'<path d="M{x - x0 - 3:.2f},{y1 - y:.2f} h6 M{x - x0:.2f},{y1 - y - 3:.2f} v6" stroke="black" stroke-width="0.2"/>')
    svg.append(f'<path d="M10,{h - 10:.2f} h50" stroke="black" stroke-width="0.5"/>')
    svg.append(f'<text x="10" y="{h - 13:.2f}" font-size="4" font-family="sans-serif">50 mm (check the print scale) · '
               f'{m} motor plate, 3 mm aluminium · tools/print_pack.py</text></svg>')
    (BENCH / f"{m}_motor_plate.svg").write_text("\n".join(svg), encoding="utf-8")


# --------------------------------------------------------------------------------------------- jobs
ROT = {"Z+": np.eye(3), "Z-": np.diag([1, -1, -1]), "X+": np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]]),
       "X-": np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]), "Y+": np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]),
       "Y-": np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])}      # each turns that axis to +z (the build direction)


def oriented(stl):
    """(name, triangles turned so the print face is down and moved to the origin, the face used)."""
    up = ORIENT.get(stl.stem) or ("Z+" if "stl/da" not in stl.as_posix() else audit(str(stl))["best_orient"])
    t = load(stl) @ ROT[up].T
    return stl.stem, t - t.reshape(-1, 3).min(0), up


def pack(items, gap=6.0):
    """Shelf packing on the bed, deepest parts first: [[(item, x, y), ...] per plate]. The CLI's own arrange keeps a
    24 mm brim margin round every part, which rejects anything wider than ~130 mm, so the parts are placed here."""
    size = lambda it: it[1].reshape(-1, 3).max(0)[:2]
    out = []
    for it in sorted(items, key=lambda i: -size(i)[1]):
        w, d = size(it)
        if max(w, d) > BED:
            raise ValueError(f"{it[0]} is {w:.0f} x {d:.0f} mm, larger than the bed")
        for plate in out:
            rows = plate["rows"]                                 # [y0, depth, x cursor]
            row = next((r for r in rows if r[2] + w <= BED and d <= r[1]), None)
            if row is None and rows[-1][0] + rows[-1][1] + gap + d <= BED:
                row = [rows[-1][0] + rows[-1][1] + gap, d, 0.0]
                rows.append(row)
            if row is not None:
                plate["parts"].append((it, row[2], row[0]))
                row[2] += w + gap
                break
        else:
            out.append({"rows": [[0.0, d, w + gap]], "parts": [(it, 0.0, 0.0)]})
    return [p["parts"] for p in out]


def write_soup(path, t):
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
    rec = np.zeros(len(t), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, t
    path.write_bytes(path.stem.encode()[:80].ljust(80, b" ") + np.uint32(len(t)).tobytes() + rec.tobytes())


def slice_plate(stls, out):
    """Slice one plate with the parts where they are; keep the project (settings and parts, no G-code)."""
    m, p, f = SP.profiles("A1M")
    cmd = [str(SP.EXE), "--orient", "0", "--arrange", "0", "--load-settings", f"{m};{p}", "--load-filaments", str(f),
           "--slice", "0", "--export-3mf", out.name, "--outputdir", str(out.parent)] + [str(s) for s in stls]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)
    for g in out.parent.glob("*.gcode"):                         # a failed run leaves loose G-code behind
        g.unlink()
    if not out.exists():
        return {"error": (r.stdout + r.stderr).strip().splitlines()[-1:]}
    with zipfile.ZipFile(out) as z:
        info = z.read("Metadata/slice_info.config").decode()
        keep = [(i, z.read(i)) for i in z.infolist() if ".gcode" not in i.filename]
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:   # drop the G-code: tens of MB, and Studio re-slices
        for i, data in keep:
            z.writestr(i, data)
    get = lambda key: sum(float(x) for x in re.findall(rf'key="{key}" value="([^"]*)"', info))
    warn = [w for w in re.findall(r'warning msg="([^"]+)"', info) if w != "bed_temperature_too_high_than_filament"]
    return {"hours": round(get("prediction") / 3600, 2), "grams": round(get("weight"), 1), "warnings": sorted(set(warn))}


def run_job(name, summary):
    dest = PLATES / name
    dest.mkdir(parents=True, exist_ok=True)
    for old in dest.glob("*.3mf"):
        old.unlink()
    res = []
    for k, plate in enumerate(pack([oriented(ROOT / s) for s in JOBS[name]]), 1):
        with tempfile.TemporaryDirectory() as tmp:
            paths = []
            for (stem, t, _), x, y in plate:
                paths.append(Path(tmp) / f"{stem}.stl")
                write_soup(paths[-1], t + np.array([x + (180 - BED) / 2, y + (180 - BED) / 2, 0]))
            r = slice_plate(paths, dest / f"plate_{k}.3mf")
        res.append(dict(r, parts=[f"{stem} ({up})" for (stem, _, up), _, _ in plate]))
        print(name, res[-1], flush=True)
    summary[name] = res


def main(args):
    BENCH.mkdir(parents=True, exist_ok=True)
    write_stl(BENCH / "C2_Stand.stl", stand("C2"), "C2_Stand")
    write_stl(BENCH / "C2_Lever.stl", lever("C2"), "C2_Lever")
    for m in ("C2", "C34"):
        write_plate(m)
    print("clash with the C2 module (mm3):", clash("C2"))
    path = PLATES / "summary.json"
    summary = json.loads(path.read_text()) if path.exists() else {}
    for name in args or JOBS:
        run_job(name, summary)
        path.write_text(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main(sys.argv[1:])

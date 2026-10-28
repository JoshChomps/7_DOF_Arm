#!/usr/bin/env python3
"""
Tolerance test coupons: print these first, in PETG on the A1 mini with the real settings, and record which fit
is right in docs/print_tolerances.md. The sizes are the ones the CAD uses (FITS in tools/fusion_arm.py and the
insert pockets in the builders), so each coupon has a column at the CAD's current guess.

    python3 tools/coupons.py                 # writes stl/coupons/*.stl
    python3 tools/coupons.py --out stl/coupons --segments 128

Coupons (labels are raised 7-segment digits: column header = offset in mm, row label = nominal):
    pins.stl        through-holes for 3/4/5/6 mm dowel pins and balls, offsets -0.05 ... +0.20 (CAD +0.05, press 0)
    bearings.stl    seats for 623 (10 mm), 6802 (24 mm), 6804 (32 mm), offsets 0 ... +0.15 (CAD +0.05)
    inserts.stl     blind holes for M3/M4/M5 heat-set inserts round the CAD's 4.2 / 5.6 / 6.4 mm pockets
    pegs.stl        journals for the 6802 and 6804 bores (15, 20 mm) on the input shaft, -0.15 ... 0 (CAD -0.05)
    rings_6810.stl  four 6810 seat rings (65 mm, 0 ... +0.15) with a journal cup inside each (50 mm, -0.15 ... 0)
    rings_6806.stl  the same for the 6806 (42 / 30 mm)

Holes are drawn as circumscribed polygons, so the narrowest width equals the stated size.
Print flat, 0.2 mm layers, 6 walls, same settings as the real parts. Needs: pip install manifold3d
"""
import argparse
import math
from pathlib import Path

from manifold3d import Manifold

from cadio import write_stl

SEGMENTS = 96

# 7-segment glyphs (segments a b c d e f g) plus '.', '-', '+'
_SEG = {
    "0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg", "5": "acdfg",
    "6": "acdefg", "7": "abc", "8": "abcdefg", "9": "abcdfg", "-": "g", "+": "g|",
}
GLYPH_W, GLYPH_H, STROKE, GAP, EMBOSS = 2.6, 4.4, 0.7, 0.8, 0.6


def _box(x, y, w, h, z, t):
    return Manifold.cube((w, h, t)).translate((x, y, z))


def text(s, x, y, z, height=GLYPH_H):
    """Raised 7-segment text with its lower-left corner at (x, y), standing on plane z."""
    k = height / GLYPH_H
    w, h, st, gap = GLYPH_W * k, height, STROKE * k, GAP * k
    parts = []
    cx = x
    for ch in s:
        if ch == ".":
            parts.append(_box(cx, y, st, st, z, EMBOSS))
            cx += st + gap
            continue
        segs = _SEG[ch]
        mid = y + (h - st) / 2
        boxes = {
            "a": (cx, y + h - st, w, st), "d": (cx, y, w, st), "g": (cx, mid, w, st),
            "f": (cx, mid, st, h - (mid - y)), "b": (cx + w - st, mid, st, h - (mid - y)),
            "e": (cx, y, st, mid - y + st), "c": (cx + w - st, y, st, mid - y + st),
            "|": (cx + (w - st) / 2, y + h * 0.2, st, h * 0.6),
        }
        for sgm in segs:
            bx, by, bw, bh = boxes[sgm]
            parts.append(_box(bx, by, bw, bh, z, EMBOSS))
        cx += w + gap
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def text_width(s, height=GLYPH_H):
    k = height / GLYPH_H
    return sum((STROKE * k if c == "." else GLYPH_W * k) + GAP * k for c in s) - GAP * k


def hole(d, depth, x, y, z0=-1.0, segments=None):
    n = segments or SEGMENTS
    r = d / 2 / math.cos(math.pi / n)   # circumscribed: flat-to-flat = d
    return Manifold.cylinder(depth, r, circular_segments=n).translate((x, y, z0))


def fmt_offset(o):
    if abs(o) < 1e-9:
        return "0"
    s = f"{abs(o):.2f}".rstrip("0").lstrip("0")
    return ("-" if o < 0 else "+") + s


def grid_coupon(nominals, offsets, thickness, pitch_x, row_pitch, blind_depth=None, labels=True,
                peg_height=None, base=3.0, segments=None):
    """Plate with a grid of holes (or pegs). Rows = nominals, columns = offsets."""
    left = 4 + max(text_width(str(n)) for n in nominals) + 3
    top = GLYPH_H + 4
    width = left + pitch_x * len(offsets)
    row_h = [row_pitch(n) for n in nominals]
    height = sum(row_h) + top
    t = base if peg_height else thickness
    plate = Manifold.cube((width, height, t))
    feats = []
    y = height - top
    for n, rh in zip(nominals, row_h):
        cy = y - rh / 2
        for j, o in enumerate(offsets):
            cx = left + pitch_x * (j + 0.5)
            d = n + o
            if peg_height:
                feats.append(("add", Manifold.cylinder(peg_height, d / 2, circular_segments=segments or SEGMENTS)
                              .translate((cx, cy, t))))
            elif blind_depth:
                feats.append(("sub", hole(d, blind_depth + 1, cx, cy, z0=t - blind_depth, segments=segments)))
            else:
                feats.append(("sub", hole(d, t + 2, cx, cy, segments=segments)))
        if labels:
            feats.append(("add", text(f"{n:g}", 3, cy - GLYPH_H / 2, t)))
        y -= rh
    if labels:
        for j, o in enumerate(offsets):
            s = fmt_offset(o)
            cx = left + pitch_x * (j + 0.5)
            feats.append(("add", text(s, cx - text_width(s) / 2, height - top + 1.5, t)))
    for op, f in feats:
        plate = plate - f if op == "sub" else plate + f
    return plate


def ring_coupon(od, bore, seat_offsets, journal_offsets, wall=3.5, h=4.0, labels=True, segments=None):
    """2 x 2 seat rings (bore od + offset) for a bearing's outer ring, each with a journal cup (od bore + offset)
    for its inner ring standing inside it. The cup's floor is 6 mm wider than the journal, so the bearing stops on
    it. Each piece is labelled on its own top face: the ring on a tab, the cup on its floor."""
    n = segments or SEGMENTS
    pitch = od + max(seat_offsets) + 2 * wall + 6
    out = None
    for k, (so, jo) in enumerate(zip(seat_offsets, journal_offsets)):
        cx, cy = (k % 2 - 0.5) * pitch, (0.5 - k // 2) * pitch
        r_in, r_out = (od + so) / 2, (od + so) / 2 + wall
        ring = Manifold.cylinder(h, r_out, circular_segments=n) - hole(od + so, h + 2, 0, 0, segments=n)
        sy = 1 if cy > 0 else -1                                   # tab points away from the middle
        ty = r_in + 1 if sy > 0 else -(r_out + 6)
        ring = ring + _box(-7, ty, 14, r_out + 5 - r_in, 0, h)
        cup = (Manifold.cylinder(1.5, bore / 2 + 3, circular_segments=n)
               + Manifold.cylinder(h + 1.5, (bore + jo) / 2, circular_segments=n)
               - hole(bore - 6, h + 2, 0, 0, z0=1.5, segments=n))
        if labels:
            st, jt = fmt_offset(so), fmt_offset(jo)
            ring = ring + text(st, -text_width(st, 3.5) / 2, sy * (r_out + 2.5) - 1.75, h, 3.5)
            cup = cup + text(jt, -text_width(jt) / 2, -GLYPH_H / 2, 1.5)
        piece = (ring + cup).translate((cx, cy, 0))
        out = piece if out is None else out + piece
    return out


def make_all(segments=None, labels=True):
    coupons = {
        "pins": grid_coupon([3, 4, 5, 6], [-0.05, 0.0, 0.05, 0.1, 0.15, 0.2], thickness=6, pitch_x=11,
                            row_pitch=lambda n: n + 7, labels=labels, segments=segments),
        "bearings": grid_coupon([10, 24, 32], [0.0, 0.05, 0.1, 0.15], thickness=7, pitch_x=38,
                                row_pitch=lambda n: n + 8, labels=labels, segments=segments),
        # the CAD's pockets: M3 4.2, M4 5.6, M5 6.4 mm (fusion_arm.py, fusion_arm_links.py, fusion_arm_j7.py)
        "inserts": grid_coupon([4.2, 5.6, 6.4], [-0.2, -0.1, 0.0, 0.1, 0.2], thickness=10,
                               blind_depth=7, pitch_x=12, row_pitch=lambda n: n + 8, labels=labels,
                               segments=segments),
        "pegs": grid_coupon([15, 20], [-0.15, -0.1, -0.05, 0.0], thickness=0, pitch_x=26,
                            row_pitch=lambda n: n + 8, peg_height=8, labels=labels, segments=segments),
        "rings_6810": ring_coupon(65, 50, [0.0, 0.05, 0.1, 0.15], [-0.15, -0.1, -0.05, 0.0], labels=labels,
                                  segments=segments),
        "rings_6806": ring_coupon(42, 30, [0.0, 0.05, 0.1, 0.15], [-0.15, -0.1, -0.05, 0.0], labels=labels,
                                  segments=segments),
    }
    return coupons


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default="stl/coupons")
    ap.add_argument("--segments", type=int, default=SEGMENTS, help="facets per hole")
    a = ap.parse_args()
    for name, m in make_all(a.segments).items():
        bb = m.bounding_box()
        size = [bb[i + 3] - bb[i] for i in range(3)]
        path = Path(a.out) / f"{name}.stl"
        write_stl(path, m, name)
        print(f"{path}  {size[0]:.0f} x {size[1]:.0f} x {size[2]:.1f} mm")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Fasteners, inserts and pins the CAD needs, counted from its holes (docs/21 §2).

    python tools/cad_bom.py        # reads tools/cad_holes.json (Fusion: tools/fusion_run.py sweep inventory)

Each axis that holes share is one item: an insert hole (5.6 / 4.2 / 6.4) is an M4 / M3 / M5 heat-set insert and
its screw; a clearance hole alone (4.5 / 3.4 / 5.5) is a screw into something else (a motor, a nut); a pin hole
(3.0-3.2, 4.0-4.1, 5.0-5.1, 6.0-6.1) is a steel pin as long as the holes span. Counterbores and bigger bores are
ignored. The motor screws, bearings and belts are counted elsewhere (docs/bom.csv).
"""
import json
from collections import Counter
from pathlib import Path

CLASS = [  # (lo, hi, name)
    (2.9, 3.25, "pin 3"), (3.3, 3.5, "M3 clearance"), (3.95, 4.15, "pin 4"), (4.15, 4.3, "M3 insert"),
    (4.4, 4.6, "M4 clearance"), (4.95, 5.15, "pin 5"), (5.45, 5.55, "M5 clearance"), (5.55, 5.7, "M4 insert"),
    (5.95, 6.15, "pin 6"), (6.15, 6.25, "magnet / ball seat"), (6.35, 6.45, "M5 insert"),
]


def kind(d):
    return next((n for lo, hi, n in CLASS if lo <= d < hi), None)


def _canon(r):
    """Fusion gives a hole's axis either way round: point it one way, and flip its span along it to match."""
    a = r["axis"]
    if next(v for v in a if abs(v) > 1e-9) < 0:
        r = dict(r, axis=[-v + 0.0 for v in a], t=[-r["t"][1], -r["t"][0]])
    return r


def axes(rows, tol=0.1):
    out = []                                           # [(axis, foot, [rows])]
    for r in map(_canon, rows):
        for a, f, rs in out:
            if a == r["axis"] and max(abs(f[i] - r["foot"][i]) for i in range(3)) < tol:
                rs.append(r)
                break
        else:
            out.append((r["axis"], r["foot"], [r]))
    split = []                                         # holes on one line but apart along it are separate items
    for a, f, rs in out:
        rs.sort(key=lambda r: r["t"][0])
        cur, end = [], None
        for r in rs:
            if cur and r["t"][0] > end + 0.5:
                split.append((a, f, cur))
                cur = []
            cur.append(r)
            end = r["t"][1] if len(cur) == 1 else max(end, r["t"][1])
        split.append((a, f, cur))
    return split


def item(rs):
    ks = {kind(r["d"]) for r in rs} - {None}
    span = round(max(r["t"][1] for r in rs) - min(r["t"][0] for r in rs))
    parts = sorted({r["part"] for r in rs})
    for size in ("M3", "M4", "M5"):
        if f"{size} insert" in ks:
            return f"{size} insert + {size} screw" if f"{size} clearance" in ks else f"{size} insert (screw from a bought part)", span, parts
    for size in ("M3", "M4", "M5"):
        if f"{size} clearance" in ks:
            return f"{size} screw, no insert (into a motor / nut)", span, parts
    pins = sorted(k for k in ks if k.startswith("pin"))
    if pins:
        return f"steel {pins[0]} mm", span, parts
    return (sorted(ks)[0] if ks else None), span, parts


def main():
    rows = json.loads((Path(__file__).parent / "cad_holes.json").read_text(encoding="utf-8"))
    count, lengths, where = Counter(), {}, {}
    for _, _, rs in axes(rows):
        name, span, parts = item(rs)
        if not name:
            continue
        count[name] += 1
        lengths.setdefault(name, Counter())[span] += 1
        where.setdefault(name, Counter()).update(p.replace("DA_", "").replace("DeskArm_", "") for p in parts)
    for name, n in sorted(count.items()):
        ls = ", ".join(f"{n}x{l}" for l, n in sorted(lengths[name].items()))
        top = ", ".join(p for p, _ in where[name].most_common(6))
        print(f"{n:4d}  {name:42s} spans (mm): {ls}\n      in: {top}")


if __name__ == "__main__":
    main()

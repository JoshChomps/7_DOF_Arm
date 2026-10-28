"""Printability audit of every STL in a folder: closed solid, volume, box, sharp convex edges, overhang, wall thickness.

    python tools/mesh_audit.py stl/da out.json        # docs/22 §1

Walls are rays from 250 surface points through the material: a screening number, not the slicer's.
"""
import glob, json, os, struct, sys
import numpy as np

def load(p):
    b = open(p, "rb").read()
    if b[:5] == b"solid" and b"facet" in b[:400]:
        v = np.array([[float(x) for x in l.split()[1:]] for l in b.decode().splitlines() if l.strip().startswith("vertex")])
        return v.reshape(-1, 3, 3)
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return a["v"].astype(float)

def audit(p):
    t = load(p)
    key = lambda v: tuple(np.round(v, 3))
    idx = {}
    f = np.array([[idx.setdefault(key(v), len(idx)) for v in tri] for tri in t])
    nrm = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    area = np.linalg.norm(nrm, axis=1) / 2
    ok = area > 1e-9
    nrm = nrm[ok] / (2 * area[ok])[:, None]; f = f[ok]; t = t[ok]; area = area[ok]
    vol = float(np.sum(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2]))) / 6)
    edges = {}
    for i, tri in enumerate(f):
        for a, b in ((0, 1), (1, 2), (2, 0)):
            u, w = tri[a], tri[b]
            edges.setdefault((min(u, w), max(u, w)), []).append((i, u < w))
    open_e = sum(1 for e in edges.values() if len(e) == 1)
    nonman = sum(1 for e in edges.values() if len(e) > 2)
    sharp_len = tot_len = 0.0
    verts = np.array(list(idx.keys()))
    for (u, w), e in edges.items():
        if len(e) != 2:
            continue
        L = np.linalg.norm(verts[u] - verts[w]); tot_len += L
        (i, _), (j, _) = e
        cosd = np.clip(nrm[i] @ nrm[j], -1, 1)
        if cosd < np.cos(np.radians(60)):                     # > 60 deg turn
            mid = (t[j].mean(0) - t[i].mean(0))
            if mid @ nrm[i] < 0:                              # convex
                sharp_len += L
    lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
    # overhang: area facing down steeper than 45 deg from vertical, per +-axis as "up"
    best = None
    for ax in range(3):
        for s in (1, -1):
            up = np.zeros(3); up[ax] = s
            c = nrm @ up
            h = t @ up                                               # vertex heights, (n, 3)
            on_bed = (c < -0.999) & (h.max(1) - h.min() < 0.05)     # flat, facing down, at the lowest level
            over = area[c < -0.7071].sum()                           # facing down within 45 deg of straight down
            flat = area[on_bed].sum()
            o = over - flat                                          # floating flat faces are overhangs (bridges)
            if best is None or o < best[0]:
                best = (o, flat, "XYZ"[ax] + ("+" if s > 0 else "-"))
    # wall thickness: rays from sampled face centres along inward normal
    rng = np.random.default_rng(0)
    pick = rng.choice(len(t), size=min(250, len(t)), replace=False, p=area / area.sum())
    thick = []
    A, B, C = t[:, 0], t[:, 1], t[:, 2]
    E1, E2 = B - A, C - A
    for k in pick:
        o = t[k].mean(0); d = -nrm[k]; o = o + d * 1e-3
        pv = np.cross(d, E2); det = np.einsum("ij,ij->i", E1, pv)
        m = np.abs(det) > 1e-12
        inv = np.where(m, 1 / np.where(m, det, 1), 0)
        tv = o - A; u = np.einsum("ij,ij->i", tv, pv) * inv
        qv = np.cross(tv, E1); v = (qv @ d) * inv
        dist = np.einsum("ij,ij->i", E2, qv) * inv
        hit = m & (u >= 0) & (v >= 0) & (u + v <= 1) & (dist > 1e-3)
        if hit.any():
            thick.append(dist[hit].min())
    th = np.array(thick)
    return dict(tris=len(t), open_edges=open_e, nonmanifold=nonman, vol_cm3=round(vol / 1000, 1), bbox=[round(x, 1) for x in (hi - lo)],
                sharp_pct=round(100 * sharp_len / max(tot_len, 1e-9), 1), best_orient=best[2], overhang_pct=round(100 * best[0] / area.sum(), 1),
                bed_mm2=round(best[1]), wall_p5=round(float(np.percentile(th, 5)), 2) if len(th) else None,
                wall_min=round(float(th.min()), 2) if len(th) else None)

if __name__ == "__main__":
    out = {}
    for p in sorted(glob.glob(sys.argv[1] + "/*.stl")):
        n = os.path.basename(p)[:-4]
        try:
            out[n] = audit(p)
        except Exception as e:
            out[n] = {"error": repr(e)}
        print(n, out[n], flush=True)
    json.dump(out, open(sys.argv[2], "w"), indent=1)

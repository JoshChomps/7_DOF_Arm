"""Helpers for the arm's Fusion builders (runs inside Fusion; imported by tools/fusion_arm.py).

Geometry is extruded along a world axis between two coordinates, from circles and polygons given in the two other
coordinates: axis "z" takes (x, y), "x" takes (y, z), "y" takes (x, z). Cuts and joins only touch the bodies named
(or the part's own bodies): a cut in one component can otherwise reach bodies in another. Units: mm in, cm inside.
Hub files are tracked in tools/fusion_files.json (name -> lineage id); new files are made by saveAs from an empty
template file, because Documents.add can't make an invisible document.
"""
import json
import math
import os

import adsk.core
import adsk.fusion

LINEAGE = "urn:adsk.wipprod:dm.lineage:"
FOLDER_OF = "hM3fH0x2RJS4Pe9TRhWm1w"         # DeskArm_SA1 (J1): new files go into its folder, DeskArm_Parts
REGISTRY = os.path.join(os.path.dirname(__file__), "fusion_files.json")
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
AXES = {"x": (0, 1, 2), "y": (1, 0, 2), "z": (2, 0, 1)}     # axis index, then the two in-plane coordinates


def app():
    return adsk.core.Application.get()


def registry():
    try:
        with open(REGISTRY, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}


def register(name, lineage):
    reg = registry()
    reg[name] = lineage
    with open(REGISTRY, "w", encoding="utf-8") as f:
        json.dump(reg, f, indent=1, sort_keys=True)


def datafile(name_or_id):
    reg = registry()
    lid = reg.get(name_or_id, name_or_id)
    return app().data.findFileById(LINEAGE + lid)


def P3(v):
    return adsk.core.Point3D.create(v[0] / 10, v[1] / 10, v[2] / 10)


class Part:
    """Builds bodies in one component. Every method takes mm."""

    def __init__(self, comp, design):
        self.c, self.d = comp, design
        self.planes = {}
        self.bodies = {}

    def plane(self, axis, at):
        key = (axis, round(at, 4))
        if key not in self.planes:
            base = {"x": self.c.yZConstructionPlane, "y": self.c.xZConstructionPlane, "z": self.c.xYConstructionPlane}[axis]
            if abs(at) < 1e-9:
                self.planes[key] = base
            else:
                pi = self.c.constructionPlanes.createInput()
                pi.setByOffset(base, adsk.core.ValueInput.createByReal(at / 10))
                self.planes[key] = self.c.constructionPlanes.add(pi)
        return self.planes[key]

    def _world(self, axis, at, u, v):
        i, j, k = AXES[axis]
        w = [0.0, 0.0, 0.0]
        w[i], w[j], w[k] = at, u, v
        return w

    def ext(self, axis, a0, a1, op, circles=(), polys=(), into=None, body=None, name=None, pick=None):
        """Extrude from a0 to a1 along `axis`. circles: (u, v, dia); polys: [(u, v), ...] closed.
        into: body (or list) to join/cut; body: name for a new body; pick(profile) filters profiles."""
        if a1 < a0:
            a0, a1 = a1, a0
        sk = self.c.sketches.add(self.plane(axis, a0))
        if name:
            sk.name = name
        to_sk = lambda u, v: sk.modelToSketchSpace(P3(self._world(axis, a0, u, v)))
        for u, v, dia in circles:
            sk.sketchCurves.sketchCircles.addByCenterRadius(to_sk(u, v), dia / 20)
        lines = sk.sketchCurves.sketchLines
        for pts in polys:
            sp = [to_sk(u, v) for u, v in pts]
            first = prev = lines.addByTwoPoints(sp[0], sp[1])
            for q in sp[2:]:
                prev = lines.addByTwoPoints(prev.endSketchPoint, q)
            lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)
        prof = adsk.core.ObjectCollection.create()
        for p in sk.profiles:
            if pick is None or pick(p):
                prof.add(p)
        if prof.count == 0:
            raise RuntimeError(f"no profile for {name}")
        inp = self.c.features.extrudeFeatures.createInput(prof, op)
        normal = sk.xDirection.crossProduct(sk.yDirection)
        sign = 1 if [normal.x, normal.y, normal.z][AXES[axis][0]] > 0 else -1
        inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(sign * (a1 - a0) / 10))
        if op != NEW:
            tgt = into if into is not None else list(self.c.bRepBodies)
            inp.participantBodies = tgt if isinstance(tgt, list) else [tgt]
        f = self.c.features.extrudeFeatures.add(inp)
        if name:
            f.name = name
        if op == NEW:
            b = f.bodies.item(0)
            if body:
                b.name = body
                self.bodies[body] = b
            return b
        return f

    def spline(self, axis, a0, a1, op, pts, into=None, body=None, name=None):
        """Closed fitted spline through pts (u, v), extruded like ext()."""
        if a1 < a0:
            a0, a1 = a1, a0
        sk = self.c.sketches.add(self.plane(axis, a0))
        col = adsk.core.ObjectCollection.create()
        for u, v in pts:
            col.add(sk.modelToSketchSpace(P3(self._world(axis, a0, u, v))))
        col.add(col.item(0))
        sk.sketchCurves.sketchFittedSplines.add(col)
        return self._extrude_sketch(sk, axis, a0, a1, op, into, body, name, pick=lambda p: True)

    def polyline(self, axis, a0, a1, op, pts, into=None, body=None, name=None, holes=()):
        """One closed polyline (many points, e.g. a gear) plus optional round holes (u, v, dia)."""
        if a1 < a0:
            a0, a1 = a1, a0
        sk = self.c.sketches.add(self.plane(axis, a0))
        sk.isComputeDeferred = True
        sp = [sk.modelToSketchSpace(P3(self._world(axis, a0, u, v))) for u, v in pts]
        lines = sk.sketchCurves.sketchLines
        first = lines.addByTwoPoints(sp[0], sp[1])
        prev = first
        for q in sp[2:]:
            prev = lines.addByTwoPoints(prev.endSketchPoint, q)
        lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)
        for u, v, dia in holes:
            sk.sketchCurves.sketchCircles.addByCenterRadius(
                sk.modelToSketchSpace(P3(self._world(axis, a0, u, v))), dia / 20)
        sk.isComputeDeferred = False
        outer = max(sk.profiles, key=lambda p: p.areaProperties().area)
        return self._extrude_sketch(sk, axis, a0, a1, op, into, body, name, pick=lambda p: p == outer)

    def _extrude_sketch(self, sk, axis, a0, a1, op, into, body, name, pick):
        prof = adsk.core.ObjectCollection.create()
        for p in sk.profiles:
            if pick(p):
                prof.add(p)
        if name:
            sk.name = name
        inp = self.c.features.extrudeFeatures.createInput(prof if prof.count > 1 else prof.item(0), op)
        normal = sk.xDirection.crossProduct(sk.yDirection)
        sign = 1 if [normal.x, normal.y, normal.z][AXES[axis][0]] > 0 else -1
        inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(sign * (a1 - a0) / 10))
        if op != NEW:
            tgt = into if into is not None else list(self.c.bRepBodies)
            inp.participantBodies = tgt if isinstance(tgt, list) else [tgt]
        f = self.c.features.extrudeFeatures.add(inp)
        if name:
            f.name = name
        if op == NEW:
            b = f.bodies.item(0)
            if body:
                b.name = body
                self.bodies[body] = b
            return b
        return f

    # shorthands --------------------------------------------------------------------------------
    def cyl(self, axis, a0, a1, d, at=(0, 0), op=NEW, body=None, into=None, name=None):
        return self.ext(axis, a0, a1, op, [(at[0], at[1], d)], body=body, into=into, name=name)

    def ring(self, axis, a0, a1, od, idia, at=(0, 0), op=NEW, body=None, into=None, name=None):
        return self.ext(axis, a0, a1, op, [(at[0], at[1], od), (at[0], at[1], idia)], body=body, into=into,
                        name=name, pick=lambda p: p.profileLoops.count == 2)

    def pattern_z(self, bodies, n=3, join=None, names=()):
        """n copies of bodies round the z axis (the first is the original); joined into `join`, or named."""
        col = adsk.core.ObjectCollection.create()
        for b in bodies:
            col.add(b)
        inp = self.c.features.circularPatternFeatures.createInput(col, self.c.zConstructionAxis)
        inp.quantity = adsk.core.ValueInput.createByReal(n)
        f = self.c.features.circularPatternFeatures.add(inp)
        new = list(bodies) + [f.bodies.item(i) for i in range(f.bodies.count)]
        for b, nm in zip(new, names):
            b.name = nm
        if join is not None:
            tools = adsk.core.ObjectCollection.create()
            for b in new:
                tools.add(b)
            self.c.features.combineFeatures.add(self.c.features.combineFeatures.createInput(join, tools))
        return new

    def holes(self, axis, a0, a1, pts, d, into=None, name=None):
        return self.ext(axis, a0, a1, CUT, [(u, v, d) for u, v in pts], into=into, name=name)

    def box(self, x0, x1, y0, y1, z0, z1, op=NEW, body=None, into=None, name=None):
        return self.ext("z", z0, z1, op, polys=[[(x0, y0), (x1, y0), (x1, y1), (x0, y1)]], body=body, into=into,
                        name=name)


def polar(r, a_deg, c=(0, 0)):
    return (c[0] + r * math.cos(math.radians(a_deg)), c[1] + r * math.sin(math.radians(a_deg)))


def circle_pts(r, n, start=0.0, c=(0, 0)):
    return [polar(r, start + 360 * k / n, c) for k in range(n)]


def rect(u0, u1, v0, v1):
    return [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]


def rotated_rect(cx, cy, length, width, ang):
    c, s = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    pts = [(-length / 2, -width / 2), (length / 2, -width / 2), (length / 2, width / 2), (-length / 2, width / 2)]
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in pts]


def stadium(cx, cy, d, travel, n=12):
    """Slot outline: a dia-d hole that can move +-travel along v."""
    pts = [(cx + d / 2 * math.cos(math.pi * i / n), cy + travel + d / 2 * math.sin(math.pi * i / n)) for i in range(n + 1)]
    pts += [(cx - d / 2 * math.cos(math.pi * i / n), cy - travel - d / 2 * math.sin(math.pi * i / n)) for i in range(n + 1)]
    return pts


def hull(pts):
    """Convex hull of 2-D points, CCW (monotone chain)."""
    pts = sorted(set((round(u, 6), round(v, 6)) for u, v in pts))

    def half(seq):
        h = []
        for p in seq:
            while len(h) > 1 and ((h[-1][0] - h[-2][0]) * (p[1] - h[-2][1])
                                  - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h[:-1]
    return half(pts) + half(pts[::-1])


# homing hall pair (docs/21 §11): sensor pocket + wire groove in the fixed side, magnet pocket in the output
def hall_sensor(p, b, r, a, z_top, depth, r_out, name="Hall switch"):
    """Sensor pocket (5 x 4) at radius r, angle a, open at z_top, plus a 2 mm wire groove out to r_out."""
    c, s_ = math.cos(math.radians(a)), math.sin(math.radians(a))
    p.ext("z", z_top - depth, z_top, CUT, polys=[rotated_rect(r * c, r * s_, 5, 4, a)], into=b, name=name)
    mid = (r + r_out) / 2
    p.ext("z", z_top - 1.5, z_top, CUT, polys=[rotated_rect(mid * c, mid * s_, r_out - r, 2, a)], into=b,
          name=name + " wire groove")


def hall_magnet(p, b, r, a, z_bottom):
    p.cyl("z", z_bottom, z_bottom + 3, 6.2, at=polar(r, a), op=CUT, into=b, name="Homing magnet 6 x 3")


# edge treatment (docs/22 §5) ----------------------------------------------------------------------
KEEP_SHARP = ("ocket", "indow", "otch", "lot", "roove", "ire", "seat", "Seat", "ecess", "assage", "leeve", "Bore",
              "bore")                                     # features whose faces mate or clear a part


def _edge_info(e):
    """(convex, w1, w2, n1, n2): convexity from face 1's loop direction (face on the left, normal out), each face's width
    across the edge (mm) and its normal there. Checked on a pocketed box: 16 convex, 8 concave (25 Oct)."""
    ev = e.evaluator
    _, t0, t1 = ev.getParameterExtents()
    _, p = ev.getPointAtParameter((t0 + t1) / 2)
    _, t = ev.getFirstDerivative((t0 + t1) / 2)
    f1, f2 = e.faces.item(0), e.faces.item(1)
    _, n1 = f1.evaluator.getNormalAtPoint(p)
    _, n2 = f2.evaluator.getNormalAtPoint(p)
    if [c for c in e.coEdges if c.loop.face == f1][0].isOpposedToEdge:
        t.scaleBy(-1)
    t.normalize()
    widths = []
    for f, n in ((f1, n1), (f2, n2)):
        if f.geometry.objectType != adsk.core.Plane.classType():
            small = f.geometry.objectType == adsk.core.Cylinder.classType() and f.geometry.radius <= 0.6
            widths.append(0.0 if small else 100.0)   # hole, flute or pin (R <= 6): mates, keep sharp (J1 ring pins, 26 Oct)
            continue                                 # a big cylinder (housing OD at a tab root): wide
        d = n.crossProduct(t)
        s = [d.x * v.geometry.x + d.y * v.geometry.y + d.z * v.geometry.z for v in f.vertices]
        widths.append((max(s) - min(s)) * 10)
    return n1.crossProduct(t).dotProduct(n2) < -1e-6, widths[0], widths[1], n1, n2


def soften(comp, bed=None, r_out=4.0, r_in=2.0, chamfer=0.6, min_w=6.0, min_len=4.0, keep=KEEP_SHARP):
    """Round the straight edges of comp's bodies: outside corners R <= r_out, inside corners R <= r_in, each at most a
    quarter (inside: a fifth) of the narrower face, so an 8 mm plate's rim gets R 2 and its corners R 4. Faces under
    min_w wide (pockets, slots, gear flanks), small cylinders (holes, flutes, pins), arcs and circles (seats) and the
    faces of features named in keep stay sharp. bed (the outward normal of the face that goes on the print bed, e.g.
    (0, 0, -1)): that face's outer rim gets a 45 deg chamfer, so it prints without an overhang and elephant's foot
    doesn't flare it, and an outside edge with a face pointing down more than 45 deg gets a 45 deg chamfer of the same
    size instead of a fillet, which would print as an overhang. A feature whose volume change is more than twice what
    its size allows (it swallowed a thin rib) is deleted and retried edge by edge. Edges Fusion refuses are left sharp
    and counted. Returns a one-line report."""
    vol = sum(b.volume for b in comp.bRepBodies)
    up = adsk.core.Vector3D.create(*[-c for c in bed]) if bed is not None else None
    n_ch = n_skip = 0
    rim = set()                                   # bed rim edges: sharp if the chamfer fails, never filleted
    if bed is not None:                           # chamfer first: its 0.6 mm faces are under min_w, so the rim stays a chamfer
        for body in comp.bRepBodies:
            lo = min((f for f in body.faces if f.geometry.objectType == adsk.core.Plane.classType()),
                     key=lambda f: -sum(a * b for a, b in zip(bed, (f.pointOnFace.x, f.pointOnFace.y, f.pointOnFace.z))),
                     default=None)
            if lo is None:
                continue
            ext = sum(a * b for a, b in zip(bed, (lo.pointOnFace.x, lo.pointOnFace.y, lo.pointOnFace.z)))
            col = adsk.core.ObjectCollection.create()
            for f in body.faces:
                if f.geometry.objectType != adsk.core.Plane.classType():
                    continue
                _, n = f.evaluator.getNormalAtPoint(f.pointOnFace)
                on = abs(sum(a * b for a, b in zip(bed, (f.pointOnFace.x, f.pointOnFace.y, f.pointOnFace.z))) - ext) < 1e-4
                if on and n.dotProduct(adsk.core.Vector3D.create(*bed)) > 0.999:
                    for lp in f.loops:
                        if lp.isOuter:
                            for x in lp.edges:
                                col.add(x)
                                rim.add(x.entityToken)
            if col.count:
                try:
                    inp = comp.features.chamferFeatures.createInput2()
                    inp.chamferEdgeSets.addEqualDistanceChamferEdgeSet(col, adsk.core.ValueInput.createByReal(chamfer / 10),
                                                                       True)
                    comp.features.chamferFeatures.add(inp).name = "Bed chamfer"
                    n_ch += col.count
                except Exception:
                    n_skip += col.count
    kept = set()                                  # after the chamfer: it re-makes the faces it touches (new tokens)
    for f in comp.features.extrudeFeatures:
        if any(k in f.name for k in keep):
            kept.update(fa.entityToken for fa in f.faces)
    groups = {}
    for body in comp.bRepBodies:
        for e in body.edges:
            if (e.faces.count != 2 or e.geometry.objectType != adsk.core.Line3D.classType() or e.length * 10 < min_len
                    or e.entityToken in rim or any(fa.entityToken in kept for fa in e.faces)):
                continue
            convex, w1, w2, n1, n2 = _edge_info(e)
            w = min(w1, w2)
            if w < min_w:
                continue
            r = min(r_out, w / 4) if convex else min(r_in, w / 5)
            down = up is not None and convex and min(n1.dotProduct(up), n2.dotProduct(up)) < -0.7071
            if r >= 0.5:
                groups.setdefault(("chamfer" if down else "fillet", round(r * 2) / 2), []).append(e.entityToken)
    done = {"fillet": 0, "chamfer": 0}
    design = comp.parentDesign                    # read once: after a refused fillet, parentDesign re-raises its error
    find = lambda tok: (design.findEntityByToken(tok) or [None])[0]    # earlier features re-make edges
    for (kind, r), toks in sorted(groups.items(), key=lambda g: -g[0][1]):
        edges = [x for x in map(find, toks) if x is not None and x.isValid]

        def add(es):
            col = adsk.core.ObjectCollection.create()
            for x in es:
                col.add(x)
            size = adsk.core.ValueInput.createByReal(r / 10)
            if kind == "fillet":
                inp = comp.features.filletFeatures.createInput()
                inp.addConstantRadiusEdgeSet(col, size, False)
                inp.isRollingBallCorner = True
                per = 1 - math.pi / 4                     # area a fillet takes per r^2 of length
            else:
                inp = comp.features.chamferFeatures.createInput2()
                inp.chamferEdgeSets.addEqualDistanceChamferEdgeSet(col, size, False)
                per = 0.5
            # more than twice that means it swallowed a thin rib whose face only looked wide (the J1 base lost 8 shell
            # ribs, 33 cm3, that way on 25 Oct)
            cap = 2 * per * (r / 10) ** 2 * sum(x.length for x in es) + 1e-3
            v0 = sum(b.volume for b in comp.bRepBodies)
            ft = (comp.features.filletFeatures if kind == "fillet" else comp.features.chamferFeatures).add(inp)
            ft.name = f"Soften {'R' if kind == 'fillet' else 'C'}{r:g}"
            if (ft.healthState != adsk.fusion.FeatureHealthStates.HealthyFeatureHealthState
                    or abs(sum(b.volume for b in comp.bRepBodies) - v0) > cap):          # built but broken: drop it
                ft.deleteMe()
                raise RuntimeError("unhealthy feature")
        try:
            add(edges)
            done[kind] += len(edges)
            n_skip += len(toks) - len(edges)
        except Exception:                                        # one bad edge spoils the set: retry one by one
            for tok in toks:
                x = find(tok)
                try:
                    if x is None or not x.isValid:
                        raise RuntimeError
                    add([x])
                    done[kind] += 1
                except Exception:
                    n_skip += 1
    dv = sum(b.volume for b in comp.bRepBodies) - vol
    return (f"softened: {done['fillet']} edges filleted, {done['chamfer']} down-facing chamfered, {n_ch} bed edges "
            f"chamfered, {n_skip} left sharp, {dv:+.2f} cm3")


def arc_resample(pts, n):
    """Points evenly spaced in arc length along a closed outline (a fitted spline needs that)."""
    s = [0.0]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
    out, j = [], 0
    for i in range(n):
        while s[j + 1] < s[-1] * i / n:
            j += 1
        out.append(pts[j])
    return out


# matrices -----------------------------------------------------------------------------------------
def matrix(R=((1, 0, 0), (0, 1, 0), (0, 0, 1)), t=(0, 0, 0)):
    m = adsk.core.Matrix3D.create()
    m.setWithArray([R[0][0], R[0][1], R[0][2], t[0] / 10, R[1][0], R[1][1], R[1][2], t[1] / 10,
                    R[2][0], R[2][1], R[2][2], t[2] / 10, 0, 0, 0, 1])
    return m


def rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {"x": ((1, 0, 0), (0, c, -s), (0, s, c)), "y": ((c, 0, s), (0, 1, 0), (-s, 0, c)),
            "z": ((c, -s, 0), (s, c, 0), (0, 0, 1))}[axis]


def mul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def apply(R, v):
    return tuple(sum(R[i][k] * v[k] for k in range(3)) for i in range(3))


# documents ----------------------------------------------------------------------------------------
def open_doc(name_or_id):
    return app().documents.open(datafile(name_or_id), False)


def scratch_doc():
    """A new unsaved document (for builds that become new files, and for in-memory checks); close it unsaved."""
    return app().documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)   # (hidden is not supported)


def design_of(doc):
    d = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
    d.designType = adsk.fusion.DesignTypes.ParametricDesignType
    return d


def clear(design):
    tl = design.timeline
    if tl.count:
        tl.markerPosition = 0
        tl.deleteAllAfterMarker()
    for p in list(design.userParameters):
        p.deleteMe()


def save(doc, name, note):
    """New version if the document is already the hub file `name`, else saveAs a new file `name` (from the
    new document). New files get their lineage id later from ids(), once Fusion has uploaded them."""
    if doc.dataFile and doc.dataFile.name == name:
        doc.save(note)
        return f"{name}: new version"
    doc.saveAs(name, parts_folder(), note, "")
    return f"{name}: new file"


def parts_folder():
    return app().data.findFileById(LINEAGE + FOLDER_OF).parentFolder


def folder_files(prefix="DA_"):
    """{name: [DataFile, ...]} of the parts folder (newest first)."""
    files, out = parts_folder().dataFiles, {}
    for i in range(files.count):
        try:
            f = files.item(i)
            if f.name.startswith(prefix) and f.id.startswith("urn"):
                out.setdefault(f.name, []).append(f)
        except Exception:
            pass                                   # files still uploading can throw
    for v in out.values():
        v.sort(key=lambda f: f.dateModified, reverse=True)
    return out


def ids(prefix="DA_"):
    """Register every hub file in the parts folder whose name starts with prefix (the newest of duplicates)."""
    out = []
    for name, fs in folder_files(prefix).items():
        register(name, fs[0].id.split(":")[-1])
        out.append(f"{name} v{fs[0].versionNumber}" + (f" ({len(fs)} files)" if len(fs) > 1 else ""))
    return out


def doc_for(name):
    """Open the hub file `name` (registry, else the folder), or a new hidden document if it doesn't exist yet."""
    if name not in registry():
        ids()
    if name in registry():
        return open_doc(name)
    return scratch_doc()


def report(comp):
    out = []
    for b in comp.bRepBodies:
        bb = b.boundingBox
        out.append(f"  {b.name}: {b.volume:.1f} cm3, x {bb.minPoint.x*10:.1f}..{bb.maxPoint.x*10:.1f}, "
                   f"y {bb.minPoint.y*10:.1f}..{bb.maxPoint.y*10:.1f}, z {bb.minPoint.z*10:.1f}..{bb.maxPoint.z*10:.1f}")
    for f in comp.features:
        if f.healthState != 0:
            out.append(f"  FEATURE {f.name}: health {f.healthState} {f.errorOrWarningMessage}")
    return out


def interference(design, skip=(), detail=False):
    """{"A x B": mm3}; detail=True keys by occurrence path and adds the clash's bounding box (mm)."""
    bodies = adsk.core.ObjectCollection.create()
    for o in design.rootComponent.allOccurrences:
        for b in o.bRepBodies:
            if b.isSolid and b.isVisible:
                bodies.add(b)
    for b in design.rootComponent.bRepBodies:
        bodies.add(b)
    ii = design.createInterferenceInput(bodies)
    ii.areCoincidentFacesIncluded = False
    hits = {}
    for r in design.analyzeInterference(ii):
        a, b = sorted((r.entityOne.parentComponent.name, r.entityTwo.parentComponent.name))
        if (a, b) in skip:
            continue
        k = f"{a} x {b}"
        if detail:
            occ = lambda e: e.assemblyContext.fullPathName if e.assemblyContext else e.parentComponent.name
            bb = r.interferenceBody.boundingBox
            lo, hi = bb.minPoint, bb.maxPoint
            k = f"{occ(r.entityOne)} x {occ(r.entityTwo)}"
            hits[k] = (round(r.interferenceBody.volume * 1000, 1),
                       [round(v * 10) for v in (lo.x, hi.x, lo.y, hi.y, lo.z, hi.z)])
            continue
        hits[k] = round(hits.get(k, 0) + r.interferenceBody.volume * 1000, 2)
    return hits


def stale(delete=False):
    """The parts folder's *_dup_old files and what still references them; delete=True deletes the unreferenced ones
    (Fusion moves a deleted file to the project's Deleted items, where it can be restored)."""
    out = []
    for name, fs in folder_files("DA_").items():
        if not name.endswith("_dup_old"):
            continue
        for f in fs:
            refs = [r.name for r in f.parentReferences] if f.parentReferences else []
            line = f"{name} v{f.versionNumber}: referenced by {refs or 'nothing'}"
            if delete and not refs:
                f.deleteMe()
                line += " -> deleted"
                reg = registry()
                if reg.pop(name, None):
                    with open(REGISTRY, "w", encoding="utf-8") as fh:
                        json.dump(reg, fh, indent=1, sort_keys=True)
            out.append(line)
    return out


def old_files(keep_root="hM3fH0x2RJS4Pe9TRhWm1w", delete=False):
    """The 19 Oct DeskArm_* files: which ones the current arm still uses (everything SA1 references, recursively)."""
    used, todo = set(), [datafile(keep_root)]
    while todo:
        f = todo.pop()
        if f.id in used:
            continue
        used.add(f.id)
        todo += [c for c in (f.childReferences or [])]
    out = []
    for name, fs in sorted(folder_files("DeskArm").items()):
        for f in fs:
            refs = [r.name for r in f.parentReferences] if f.parentReferences else []
            out.append(f"{'USED ' if f.id in used else 'old  '}{name} v{f.versionNumber}  <- {refs or '-'}")
    if delete:                                         # assemblies first: a file still referenced can't go
        for _ in range(6):
            left = [f for fs in folder_files("DeskArm").values() for f in fs if f.id not in used]
            for f in left:
                if not f.parentReferences or f.parentReferences.count == 0:
                    out.append(f"deleted {f.name} v{f.versionNumber}")
                    f.deleteMe()
        out.append(f"old files left: {sum(1 for fs in folder_files('DeskArm').values() for f in fs if f.id not in used)}")
    return out

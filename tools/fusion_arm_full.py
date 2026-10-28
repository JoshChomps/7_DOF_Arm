"""The full arm, DA_ARM (docs/20): every module sub-assembly and link, linked, in the zero pose, chained by rigid
joints; each joint's motion is the revolute joint inside its module (J1 inside the J1 sub-assembly), driven from
here with createForAssemblyContext. sweep() drives them and runs Fusion's interference check.
"""
import json
import math

import adsk.core
import adsk.fusion

import fusion_lib as L
from fusion_arm_asm import MODULES

SA1 = "hM3fH0x2RJS4Pe9TRhWm1w"                   # DeskArm_SA1_J1_Base_Turntable (J1, 20 Oct)
SA = {"J2": "DA_SA_C2", "J3": "DA_SA_C34", "J4": "DA_SA_C34", "J5": "DA_SA_P56", "J6": "DA_SA_P56", "J7": "DA_SA_P7"}
GROUP = {"J2": "C2", "J3": "C34", "J4": "C34", "J5": "P56", "J6": "P56", "J7": "P7"}
LINKS = ["DA_L_StandLeft", "DA_L_TowerRight", "DA_L_CBPulley", "DA_L_Link2", "DA_L_Link3", "DA_L_Forearm",
         "DA_L_Link5", "DA_L_Link6", "DA_T_Receiver", "DA_T_ToolPlate", "DA_G_Body", "DA_G_Fingers", "DA_G_Pads", "DA_G_Carrier",
         "DA_P_Gripper"]
FIXED = [("DA_B_Board", L.rot("z", 0)), ("DA_B_Spaces", L.rot("z", 0)), ("DA_P_N17_60", L.rot("y", 180))]   # body down, shaft flat stays at +y
# (occurrence, attached to): "J2.out" = the J2 module's output group, "SA1.turntable" = J1's rotating group
CHAIN = [("DA_L_StandLeft", "SA1.turntable"), ("DA_L_TowerRight", "SA1.turntable"),
         ("DA_L_CBPulley", "DA_L_TowerRight"), ("J2.fixed", "SA1.turntable"),
         ("DA_L_Link2", "J2.out"), ("J3.fixed", "DA_L_Link2"), ("DA_L_Link3", "J3.out"), ("J4.fixed", "DA_L_Link3"),
         ("DA_L_Forearm", "J4.out"), ("J5.fixed", "DA_L_Forearm"), ("DA_L_Link5", "J5.out"),
         ("J6.fixed", "DA_L_Link5"), ("DA_L_Link6", "J6.out"), ("J7.fixed", "DA_L_Link6"),
         ("DA_T_Receiver", "J7.out"), ("DA_T_ToolPlate", "DA_T_Receiver"), ("DA_G_Body", "DA_T_ToolPlate"),
         ("DA_G_Fingers", "DA_G_Body"), ("DA_G_Pads", "DA_G_Fingers"), ("DA_G_Carrier", "DA_G_Body"), ("DA_P_Gripper", "DA_G_Body")]


def _occ(occ, key):
    if key == "SA1.turntable":
        return occ["SA1"].childOccurrences.itemByName("J1 turntable:1")
    if "." in key:
        j, g = key.split(".")
        return occ[j].childOccurrences.itemByName(f"{GROUP[j]} {g}:1")
    return occ[key]


def build(save=False):
    name = "DA_ARM"
    doc = L.doc_for(name)
    out = [name]
    try:
        d = L.design_of(doc)
        L.clear(d)
        root = d.rootComponent
        occ = {"SA1": root.occurrences.addByInsert(L.datafile(SA1), adsk.core.Matrix3D.create(), True)}
        occ["SA1"].isGrounded = True
        for n in LINKS:
            occ[n] = root.occurrences.addByInsert(L.datafile(n), adsk.core.Matrix3D.create(), True)
        for j, (m, R, t) in MODULES.items():
            occ[j] = root.occurrences.addByInsert(L.datafile(SA[j]), L.matrix(R, t), True)
        for n, R in FIXED:                             # base board and the J1 motor (body down through the board)
            root.occurrences.addByInsert(L.datafile(n), L.matrix(R, (0, 0, 0)), True).isGrounded = True
        for a, b in CHAIN:
            inp = root.asBuiltJoints.createInput(_occ(occ, a), _occ(occ, b), None)
            inp.setAsRigidJointMotion()
            root.asBuiltJoints.add(inp).name = f"{a} on {b}"
        out.append(f"{root.allOccurrences.count} occurrences, {root.asBuiltJoints.count} rigid joints")
        hits = L.interference(d)
        out.append("zero pose: " + (", ".join(f"{k} {v} mm3" for k, v in hits.items()) or "clear"))
        if save:
            out.append(L.save(doc, name, "full arm (tools/fusion_arm_full.py)"))
    finally:
        doc.close(False)
    return "\n".join(out)


def joints(root):
    """J1..J7 -> joint (in the assembly context) of the saved DA_ARM."""
    occ = {o.component.name.split(" v")[0]: o for o in root.occurrences}
    js = {}
    sa1 = [o for o in root.occurrences if o.component.name.startswith("DeskArm_SA1")][0]
    js["J1"] = sa1.component.asBuiltJoints.item(0).createForAssemblyContext(sa1)
    by_name = {}
    for o in root.occurrences:
        by_name.setdefault(o.component.name.split(" v")[0], []).append(o)
    order = {"DA_SA_C34": ["J3", "J4"], "DA_SA_P56": ["J5", "J6"], "DA_SA_C2": ["J2"], "DA_SA_P7": ["J7"]}
    for comp, names in order.items():
        os_ = sorted(by_name.get(comp, []), key=lambda o: o.transform.translation.z)   # J3 below J4, J5 below J6
        for jn, o in zip(names, os_):
            js[jn] = o.component.asBuiltJoints.item(0).createForAssemblyContext(o)
    return js


def sweep(poses, detail=False):
    doc = L.doc_for("DA_ARM")
    res = {}
    try:
        d = L.design_of(doc)
        js = joints(d.rootComponent)
        for pose in poses:
            for n, j in js.items():
                j.jointMotion.rotationValue = math.radians(pose.get(n, 0))
            adsk.doEvents()
            hits = L.interference(d, detail=detail)
            res[json.dumps(pose)] = hits or "clear"
    finally:
        doc.close(False)
    return "\n".join(f"{k} {v}" for k, v in res.items())


def holes(tol=0.05):
    """Every hole (R <= 6) in a link or tool part of the saved DA_ARM at the zero pose -> is a cylinder (hole or
    pin, R <= 12) in another part coaxial with it (axis within tol mm)? Lists the ones without a partner."""
    doc = L.doc_for("DA_ARM")
    out = []
    try:
        root = L.design_of(doc).rootComponent
        cyls = []                                      # (part, body key, origin mm, unit axis, R mm, is hole)
        for o in root.allOccurrences:
            for b in o.bRepBodies:
                for f in b.faces:
                    g = f.geometry
                    if g.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType or g.radius > 1.2 or not _round(f):
                        continue
                    p0, ax = g.origin, g.axis
                    ax.normalize()
                    q = f.pointOnFace
                    _, n = f.evaluator.getNormalAtPoint(q)
                    w = q.vectorTo(p0)
                    w.subtract(_scaled(ax, w.dotProduct(ax)))  # point on face -> axis, perpendicular
                    cyls.append((o.component.name.split(" v")[0], o.fullPathName + b.name,
                                 (p0.x * 10, p0.y * 10, p0.z * 10), ax, g.radius * 10, n.dotProduct(w) > 0, q))
        mine = [c for c in cyls if c[0].startswith(("DA_L_", "DA_T_", "DA_B_", "DA_G_")) and c[5] and c[4] <= 6]
        done = set()
        for c in mine:
            key = (c[1], tuple(round(v, 1) for v in _foot(c)), round(abs(c[3].x), 2), round(abs(c[3].y), 2))
            if key in done:
                continue
            done.add(key)
            mates = [d for d in cyls if d[1] != c[1] and _coax(c, d, tol)]
            if not mates:
                q = c[6]
                out.append(f"{c[0]}: R {c[4]:.2f} at ({q.x * 10:.1f}, {q.y * 10:.1f}, {q.z * 10:.1f}) - no partner")
        out.insert(0, f"{len(done)} link/tool hole axes checked, {len(out)} without a coaxial partner")
    finally:
        doc.close(False)
    return "\n".join(out)


def _round(f):
    """A hole or pin face: a circle or an arc of over 100 deg on its boundary. A fillet (soften, docs/22 §5a) between
    square faces spans 90 deg, so it is skipped. 180 deg was too strict: the link 2 cable clevis hole breaks out of an
    edge and was lost (26 Oct)."""
    for e in f.edges:
        g = e.geometry
        if g.objectType == adsk.core.Circle3D.classType():
            return True
        if g.objectType == adsk.core.Arc3D.classType() and (g.endAngle - g.startAngle) % (2 * math.pi) > math.radians(100):
            return True
    return False


def _scaled(v, k):
    u = v.copy()
    u.scaleBy(k)
    return u


def _foot(c):
    """The axis point nearest the world origin (a key that is the same for every face on one axis)."""
    (x, y, z), a = c[2], c[3]
    t = x * a.x + y * a.y + z * a.z
    return (x - t * a.x, y - t * a.y, z - t * a.z)


def _coax(c, d, tol):
    if abs(abs(c[3].dotProduct(d[3])) - 1) > 1e-6:
        return False
    (x, y, z), a = c[2], c[3]
    dx, dy, dz = d[2][0] - x, d[2][1] - y, d[2][2] - z
    t = dx * a.x + dy * a.y + dz * a.z
    return math.sqrt(max(dx * dx + dy * dy + dz * dz - t * t, 0)) < tol


def bodies(path):
    """Every solid in DA_ARM at the zero pose -> JSON at `path`: top-level occurrence (and its z, which tells J3 from
    J4 and J5 from J6), group, part, body, volume (cm3), centre of volume (mm, CAD world) and the mean of the three
    principal moments per unit mass about it (mm2), for tools/cad_masses.py."""
    doc = L.doc_for("DA_ARM")
    rows = []
    try:
        root = L.design_of(doc).rootComponent
        for o in root.allOccurrences:
            top = o
            while top.assemblyContext:
                top = top.assemblyContext
            names = o.fullPathName.split("+")
            for b in o.bRepBodies:
                if not b.isSolid:
                    continue
                pp = b.physicalProperties
                c = pp.centerOfMass
                _, i1, i2, i3 = pp.getPrincipalMomentsOfInertia()
                rows.append(dict(top=top.component.name.split(" v")[0], top_z=round(top.transform.translation.z * 10, 1),
                                 group=names[1].split(":")[0] if len(names) > 2 else "",
                                 part=o.component.name.split(" v")[0], body=b.name,
                                 volume=round(b.volume, 3), com=[round(v * 10, 2) for v in (c.x, c.y, c.z)],
                                 k2=round((i1 + i2 + i3) / 3 / pp.mass * 100, 1) if pp.mass else 0.0))
    finally:
        doc.close(False)
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"modules": {j: [m, R, t] for j, (m, R, t) in MODULES.items()}, "bodies": rows}, f, indent=0)
    return f"{len(rows)} bodies -> {path}"


def tool_point(poses):
    """Centre of the tool plate (CAD world, mm) at each pose: checks the CAD's joint senses against arm.toml."""
    doc = L.doc_for("DA_ARM")
    out = []
    try:
        d = L.design_of(doc)
        js = joints(d.rootComponent)
        tp = [o for o in d.rootComponent.occurrences if o.component.name.startswith("DA_T_ToolPlate")][0]
        for pose in poses:
            for n, j in js.items():
                j.jointMotion.rotationValue = math.radians(pose.get(n, 0))
            adsk.doEvents()
            c = tp.bRepBodies.item(0).physicalProperties.centerOfMass
            out.append(json.dumps(pose) + " " + json.dumps([round(v * 10, 2) for v in (c.x, c.y, c.z)]))
    finally:
        doc.close(False)
    return "\n".join(out)


def inventory(path):
    """Every small hole in DA_ARM (R <= 3.6 mm), grouped by shared axis (one screw, insert or pin per axis), with the
    diameters on the axis and its length along it -> JSON at `path`, for tools/cad_bom.py."""
    doc = L.doc_for("DA_ARM")
    rows = []
    try:
        root = L.design_of(doc).rootComponent
        for o in root.allOccurrences:
            for b in o.bRepBodies:
                for f in b.faces:
                    g = f.geometry
                    if g.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType or g.radius > 0.36:
                        continue
                    ax = g.axis
                    ax.normalize()
                    q = f.pointOnFace
                    _, n = f.evaluator.getNormalAtPoint(q)
                    w = q.vectorTo(g.origin)
                    w.subtract(_scaled(ax, w.dotProduct(ax)))
                    if n.dotProduct(w) <= 0:                   # a pin or boss, not a hole
                        continue
                    bb = f.boundingBox
                    a = (ax.x, ax.y, ax.z)
                    ts = [sum(p[i] * a[i] for i in range(3)) * 10 for p in
                          ((bb.minPoint.x, bb.minPoint.y, bb.minPoint.z), (bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z))]
                    c = (g.origin.x * 10, g.origin.y * 10, g.origin.z * 10)
                    t = sum(c[i] * a[i] for i in range(3))
                    foot = [round(c[i] - t * a[i], 2) for i in range(3)]
                    rows.append(dict(part=o.component.name.split(" v")[0], d=round(g.radius * 20, 2),
                                     axis=[round(abs(v), 3) for v in a], foot=foot, t=sorted(round(v, 2) for v in ts)))
    finally:
        doc.close(False)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(rows, fh, indent=0)
    return f"{len(rows)} hole faces -> {path}"


def export_stl(folder, skip=("_Plate",), fresh=False, only=()):
    """Every printed DA_ part (not the bought envelopes, sub-assemblies or the aluminium plates) -> folder/<name>.stl."""
    import os
    os.makedirs(folder, exist_ok=True)
    out = []
    for name in sorted(L.registry()):
        if not name.startswith(("DA_C", "DA_P5", "DA_P7", "DA_L_", "DA_T_", "DA_G_")) or name.endswith(skip):
            continue
        if only and name not in only:
            continue
        if os.path.exists(os.path.join(folder, name + ".stl")) and not fresh and not only:
            continue                                   # the add-in's HTTP call times out on a full export
        doc = L.open_doc(name)
        try:
            d = L.design_of(doc)
            em = d.exportManager
            o = em.createSTLExportOptions(d.rootComponent, os.path.join(folder, name + ".stl"))
            o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
            em.execute(o)
            out.append(name)
        finally:
            doc.close(False)
    return f"{len(out)} STLs -> {folder}"

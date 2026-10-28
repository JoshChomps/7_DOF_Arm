"""Two whole-arm files rebuilt flat from DA_ARM's parts (docs/20 §10), and the per-link meshes for the twin.

- DA_ARM_Print: only the printed parts (PETG and the TPU pads), coloured, in their places.
- DA_ARM_Display: every part, coloured, for looking at and moving around.

Every part is a top-level linked insert of its own hub file at its DA_ARM zero-pose place (so a part edit shows up in
both), coloured by material, tagged with its body and material, and held in one rigid group per moving body, "L0 base"
(grounded) ... "L7 tool". Seven top-level revolute joints J1..J7 with the arm.toml limits join the bodies, so the arm
can be dragged or driven in Fusion (DA_ARM's joints sit inside the linked module sub-assemblies). Parts nested in
empty "body" components did not follow their joints when driven (21 Oct), hence the flat layout. From the repo root, Fusion open:

    software/.venv/Scripts/python.exe tools/fusion_run.py display leaves            # DA_ARM's parts -> cad_leaves.json
    software/.venv/Scripts/python.exe tools/fusion_run.py display print|full [--save]
    software/.venv/Scripts/python.exe tools/fusion_run.py display check NAME        # interference, zero pose
    software/.venv/Scripts/python.exe tools/fusion_run.py display frames NAME '<pose>'   # body motions, part spread
    software/.venv/Scripts/python.exe tools/fusion_run.py display mesh NAME <dir>   # STLs for tools/cad_meshes.py
"""
import json
import math
import os
import tomllib

import adsk.core
import adsk.fusion

import fusion_lib as L
from fusion_arm_asm import MODULES
from fusion_arm_full import CHAIN

BODY_NAMES = ["L0 base", "L1 turntable", "L2 shoulder", "L3 upper arm", "L4 elbow", "L5 forearm", "L6 wrist", "L7 tool"]
LOOK = {"tpu": "Rubber - Soft", "motor": "Paint - Enamel Glossy (Black)", "belt": "Rubber - Hard",
        "metal": "Aluminum - Anodized Rough (Grey)", "steel": "Stainless Steel - Satin", "pcb": "Paint - Enamel Glossy (Green)",
        "space": "Glass - Frosted Light"}
PETG = ["Plastic - Matte (Gray)", "Plastic - Glossy (White)", "Plastic - Matte (Blue)"]   # base, then alternating links
ARM_TOML = os.path.join(os.path.dirname(__file__), "..", "software", "deskarm", "data", "arm.toml")
LEAVES = os.path.join(os.path.dirname(__file__), "cad_leaves.json")
# The joint senses: the tool plate must land where DA_ARM's does (fusion_run.py sweep tool, 21 Oct:
# [-13.68, -28.0, 608.03] and [0.0, 209.35, 571.29]).
CHECK = [{"J2": 30, "J4": -60, "J6": 45, "J1": 20, "J3": 15, "J5": 25}, {"J7": 90, "J6": -60, "J2": -20}]


def category(part):
    if part.startswith("DA_P_N"):
        return "motor"
    if part.startswith("DA_P_Belt"):
        return "belt"
    if part.startswith("DA_P_Pulley") or part.endswith("_Plate") or part == "DA_B_Board":
        return "metal"
    if part in ("DA_P_SlipRing", "DA_P_Gripper"):
        return "steel"
    if part.startswith("DA_P_Encoder"):
        return "pcb"
    if part == "DA_B_Spaces":
        return "space"
    return "tpu" if part == "DA_G_Pads" else "printed"


def body_of(key):
    """Moving body (0 base ... 7 tool) of a DA_ARM occurrence key, following CHAIN back to a joint output."""
    if key == "SA1.turntable":
        return 1
    if key.endswith(".out"):
        return int(key[1])
    parent = dict(CHAIN).get(key)
    return body_of(parent) if parent else 0


def leaves():
    """Every part in the saved DA_ARM -> LEAVES: [part, body, world matrix (16 numbers, cm)], zero pose. Its own call:
    opening DA_ARM and building in one call come close to the add-in's 120 s."""
    doc = L.doc_for("DA_ARM")
    rows = []
    try:
        root = L.design_of(doc).rootComponent
        tops = sorted((o for o in root.occurrences if o.component.name.startswith("DA_SA_")),
                      key=lambda o: o.transform.translation.z)          # J3 below J4, J5 below J6
        jname = {}
        for comp, names in {"DA_SA_C2": ["J2"], "DA_SA_C34": ["J3", "J4"], "DA_SA_P56": ["J5", "J6"], "DA_SA_P7": ["J7"]}.items():
            for jn, o in zip(names, [o for o in tops if o.component.name.startswith(comp)]):
                jname[o.name] = jn
        for o in root.allOccurrences:
            if not o.bRepBodies.count:
                continue
            path = o.fullPathName.split("+")
            top = path[0].split(" v")[0]
            if top.startswith("DeskArm_SA1"):
                key = "SA1.turntable" if path[1].startswith("J1 turntable") else "SA1.base"
            elif top.startswith("DA_SA_"):
                key = f"{jname[path[0]]}.{'out' if path[1].split(':')[0].endswith('out') else 'fixed'}"
            else:
                key = top
            rows.append([o.component.name.split(" v")[0], body_of(key), list(o.transform2.asArray())])
    finally:
        doc.close(False)
    with open(LEAVES, "w", encoding="utf-8") as f:
        json.dump(rows, f, indent=0)
    return f"{len(rows)} parts -> {LEAVES}"


def _mul(A, B):
    a, b = A.asArray(), B.asArray()
    m = adsk.core.Matrix3D.create()
    m.setWithArray([sum(a[4 * i + k] * b[4 * k + j] for k in range(4)) for i in range(4) for j in range(4)])
    return m


def _look(design, name):
    a = design.appearances.itemByName(name)
    if not a:
        lib = L.app().materialLibraries.itemByName("Fusion Appearance Library")
        a = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
    return a


def axes():
    """J1..J7 -> (point mm, unit axis) in CAD world: J1 is +z through the origin, the rest their module's local +z."""
    out = {"J1": ((0, 0, 0), (0, 0, 1))}
    for j, (_, R, t) in MODULES.items():
        out[j] = (t, L.apply(R, (0, 0, 1)))
    return out


def _drive(root, pose):
    for j in root.asBuiltJoints:
        j.jointMotion.rotationValue = math.radians(pose.get(j.name, 0))
    adsk.doEvents()


def _close(A, B, tol=1e-6):
    return max(abs(a - b) for a, b in zip(A.asArray(), B.asArray())) < tol


def build(name, printed_only, save=False):
    with open(ARM_TOML, "rb") as f:
        limits = {j["name"]: (j["min_deg"], j["max_deg"]) for j in tomllib.load(f)["joints"]}
    with open(LEAVES, encoding="utf-8") as f:
        rows = [r for r in json.load(f) if not printed_only or category(r[0]) in ("printed", "tpu")]
    j1 = L.folder_files("DeskArm_J1")                  # the J1 parts aren't in the DA_ registry
    doc = L.doc_for(name)
    out = [name]
    try:
        d = L.design_of(doc)
        L.clear(d)
        root = d.rootComponent
        bodies, comps = [[] for _ in BODY_NAMES], {}
        for part, k, m in rows:
            T = adsk.core.Matrix3D.create()
            T.setWithArray(m)
            if part in comps:                          # a repeat: fast, but Fusion may compose T with the first's place
                comp, first = comps[part]
                o = root.occurrences.addExistingComponent(comp, T)
                if not _close(o.transform2, T):
                    o.deleteMe()
                    inv = first.copy()
                    inv.invert()
                    o = root.occurrences.addExistingComponent(comp, _mul(T, inv))
            else:                                      # ~0.9 s per linked insert
                df = j1[part][0] if part.startswith("DeskArm_") else L.datafile(part)
                o = root.occurrences.addByInsert(df, T, True)
                comps[part] = (o.component, T)
            if not _close(o.transform2, T):
                raise RuntimeError(f"{part} misplaced")
            c = category(part)
            o.appearance = _look(d, PETG[0 if k == 0 else 1 + k % 2] if c == "printed" else LOOK[c])
            o.attributes.add("deskarm", "body", str(k))
            o.attributes.add("deskarm", "material", c)
            bodies[k].append(o)
        for k, occs in enumerate(bodies):              # each body moves as one
            col = adsk.core.ObjectCollection.create()
            for o in occs:
                col.add(o)
            root.rigidGroups.add(col, True).name = BODY_NAMES[k]
        bodies[0][0].isGrounded = True
        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "Joint axes"
        for k, (jn, (p, a)) in enumerate(sorted(axes().items()), start=1):
            p0 = adsk.core.Point3D.create(*(v / 10 for v in p))
            p1 = adsk.core.Point3D.create(*(v / 10 + 2 * u for v, u in zip(p, a)))
            geo = adsk.fusion.JointGeometry.createByCurve(sk.sketchCurves.sketchLines.addByTwoPoints(p0, p1),
                                                          adsk.fusion.JointKeyPointTypes.StartKeyPoint)
            inp = root.asBuiltJoints.createInput(bodies[k][0], bodies[k - 1][0], geo)
            inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
            j = root.asBuiltJoints.add(inp)
            j.name = jn
            lo, hi = limits[jn]
            if hi - lo < 360:                          # J7 turns freely
                lim = j.jointMotion.rotationLimits
                lim.isMinimumValueEnabled = lim.isMaximumValueEnabled = True
                lim.minimumValue, lim.maximumValue = math.radians(lo), math.radians(hi)
        sk.isVisible = False
        tp = [o for o in bodies[7] if o.component.name.startswith("DA_T_ToolPlate")][0]
        for pose in CHECK:
            _drive(root, pose)
            c = tp.bRepBodies.item(0).physicalProperties.centerOfMass
            out.append(f"{pose} tool plate {[round(v * 10, 2) for v in (c.x, c.y, c.z)]}")
        _drive(root, {})
        out.append(f"{len(rows)} parts, {root.rigidGroups.count} rigid groups, {root.asBuiltJoints.count} joints")
        if save:
            out.append(L.save(doc, name, "arm by body, top-level joints (tools/fusion_arm_display.py)"))
    finally:
        doc.close(False)
    return "\n".join(out)


def _body(o):
    a = o.attributes.itemByName("deskarm", "body")
    return int(a.value) if a else None


def frames(name, pose):
    """How each body moves from the zero pose to `pose` (16 numbers, cm, CAD world), and the largest difference
    between that and any of the body's parts (mm): tools/cad_meshes.py check compares the motions with the twin's."""
    doc = L.doc_for(name)
    try:
        root = L.design_of(doc).rootComponent
        _drive(root, {})
        occs = [o for o in root.occurrences if _body(o) is not None]
        zero = [o.transform2.copy() for o in occs]
        _drive(root, pose)
        motion, spread = {}, 0.0
        for o, z in zip(occs, zero):
            inv = z.copy()
            inv.invert()
            m = _mul(o.transform2, inv).asArray()
            k = _body(o)
            motion.setdefault(k, m)
            spread = max([spread] + [abs(a - b) * 10 for a, b in zip(m, motion[k])])   # cm -> mm (rotation x 10 too)
        return json.dumps({"motion": [motion[k] for k in range(len(BODY_NAMES))], "spread_mm": spread})
    finally:
        doc.close(False)


def mesh(name, folder):
    """Every part of `name` -> folder/<body>_<material>_<n>.stl in the part's own coordinates, mm (the export ignores
    the occurrence's place), and folder/places.json: file -> its zero-pose place (16 numbers, cm), for
    tools/cad_meshes.py."""
    os.makedirs(folder, exist_ok=True)
    doc = L.doc_for(name)
    places = {}
    try:
        d = L.design_of(doc)
        _drive(d.rootComponent, {})
        em = d.exportManager
        for o in d.rootComponent.occurrences:
            if _body(o) is None:
                continue
            f = f"{_body(o)}_{o.attributes.itemByName('deskarm', 'material').value}_{len(places)}.stl"
            opt = em.createSTLExportOptions(o, os.path.join(folder, f))
            opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            opt.isBinaryFormat = True
            em.execute(opt)
            places[f] = list(o.transform2.asArray())
    finally:
        doc.close(False)
    with open(os.path.join(folder, "places.json"), "w", encoding="utf-8") as fh:
        json.dump(places, fh)
    return f"{len(places)} STLs -> {folder}"


def cli(args):
    names = {"print": ("DA_ARM_Print", True), "full": ("DA_ARM_Display", False)}
    if args[0] == "leaves":
        return leaves()
    if args[0] in names:
        return build(*names[args[0]], save="--save" in args)
    if args[0] == "check":
        doc = L.doc_for(args[1])
        try:
            hits = L.interference(L.design_of(doc))
        finally:
            doc.close(False)
        return ", ".join(f"{k} {v} mm3" for k, v in hits.items()) or "clear"
    if args[0] == "frames":
        return frames(args[1], json.loads(args[2]))
    if args[0] == "mesh":
        return mesh(args[1], args[2])
    if args[0] == "picture":                          # picture NAME '<pose>' PNG: a shaded view (opens a tab briefly)
        doc = L.app().documents.open(L.datafile(args[1]), True)
        try:
            _drive(L.design_of(doc).rootComponent, json.loads(args[2]))
            vp = L.app().activeViewport
            cam = vp.camera
            cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
            cam.isFitView = True
            vp.camera = cam
            vp.visualStyle = adsk.core.VisualStyles.ShadedWithVisibleEdgesOnlyVisualStyle
            for _ in range(5):
                adsk.doEvents()
                vp.refresh()
            vp.saveAsImageFile(args[3], 1400, 1400)
        finally:
            doc.close(False)
        return args[3]
    raise ValueError(args)

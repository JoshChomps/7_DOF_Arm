"""The J1 sub-assembly, DeskArm_SA1_J1_Base_Turntable, from the J1 parts (tools/fusion_j1.py, docs/19 §14). DA_ARM
(tools/fusion_arm_full.py) inserts it.

    software/.venv/Scripts/python.exe tools/fusion_assembly.py sa1 [--save]           # rebuild it
    software/.venv/Scripts/python.exe tools/fusion_assembly.py refresh-sa1 [--save]   # links -> latest

sa1: clears the file and inserts the J1 parts linked at their latest versions, in the base frame (z = 0 on the board):
group "J1 base" (grounded: base, motor plate, cam, disc A at +e, disc B turned 180 deg at -e) and "J1 turntable"
(turntable with its inner spacer, pin carrier), each with a rigid group, and J1 as-built revolute about Z, +-170 deg.
Without --save the document is closed unsaved. (Its 19 Oct "arm" mode, for DeskArm_Arm_Assembly, went with that
file on 20 Oct: DA_ARM replaced it.)
"""
import json
import math

LINEAGE = "urn:adsk.wipprod:dm.lineage:"
SA1 = "hM3fH0x2RJS4Pe9TRhWm1w"
J1 = dict(base="TVG9cvxKQzaYPYpT7r_1dw", plate="jHAj8-sFTBmPFRJhYG8-qw", cam="m00GME05TB2wXTXfOtjtCA",
          disc="zfAhHGfwQay0aE8lArSq0g", carrier="23rBK7akTGKtnOihvYXNug", turntable="DaOv-H-SRu6VqINYPa5xrA")
TARGET, SAVE = "", False                             # filled in by __main__


def run(_context: str):
    import adsk.core, adsk.fusion
    app = adsk.core.Application.get()
    doc = app.documents.open(app.data.findFileById(LINEAGE + SA1), False)
    out = []
    try:
        design = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        root = design.rootComponent
        pt = lambda p: adsk.core.Point3D.create(p[0] / 10, p[1] / 10, p[2] / 10)

        def revolute(name, a, b, o, d, lims, sk):
            line = sk.sketchCurves.sketchLines.addByTwoPoints(pt(o), pt([o[i] + 20 * d[i] for i in range(3)]))
            geo = adsk.fusion.JointGeometry.createByCurve(line, adsk.fusion.JointKeyPointTypes.StartKeyPoint)
            inp = root.asBuiltJoints.createInput(a, b, geo)
            inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
            j = root.asBuiltJoints.add(inp)
            j.name = name
            lim = j.jointMotion.rotationLimits
            lim.isMinimumValueEnabled = lim.isMaximumValueEnabled = True
            lim.minimumValue, lim.maximumValue = math.radians(lims[0]), math.radians(lims[1])

        def matrix(rot_deg, x, y, z):
            m = adsk.core.Matrix3D.create()
            m.setToRotation(math.radians(rot_deg), adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
            m.translation = adsk.core.Vector3D.create(x / 10, y / 10, z / 10)
            return m

        if TARGET.startswith("refresh"):              # a part changed: move the links, keep everything else
            for r in doc.documentReferences:
                if r.isOutOfDate:
                    r.getLatestVersion()
            for j in root.asBuiltJoints:
                out.append(f"joint {j.name}: valid {j.isValid}, {getattr(j, "errorOrWarningMessage", "") or "no error"}")
        elif TARGET == "sa1":
            if design.timeline.count:
                design.timeline.markerPosition = 0
                design.timeline.deleteAllAfterMarker()
            f = {k: app.data.findFileById(LINEAGE + v) for k, v in J1.items()}
            groups = []
            for gname, parts in (("J1 base", [("base", 0, 0, 0, 0), ("plate", 0, 0, 0, 0), ("cam", 0, 0, 0, 0),
                                              ("disc", 0, 1.2, 0, 7.5), ("disc", 180, -1.2, 0, 15.5)]),
                                 ("J1 turntable", [("turntable", 0, 0, 0, 0), ("carrier", 0, 0, 0, 0)])):
                g = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                g.component.name = gname
                for p, r, x, y, z in parts:
                    g.component.occurrences.addByInsert(f[p], matrix(r, x, y, z), True)
                col = adsk.core.ObjectCollection.create()
                for o in g.component.occurrences:
                    col.add(o)
                g.component.rigidGroups.add(col, True).name = gname + " rigid"
                groups.append(g)
            groups[0].isGrounded = True
            sk = root.sketches.add(root.xYConstructionPlane)
            sk.name = "Joint axis (zero pose)"
            revolute("J1", groups[1], groups[0], (0, 0, 0), (0, 0, 1), (-170, 170), sk)
        out.append("refs: " + ", ".join(f"{r.dataFile.name} v{r.dataFile.versionNumber}" for r in doc.documentReferences))
        for o in root.allOccurrences:
            t = o.transform2.translation if hasattr(o, "transform2") else o.transform.translation
            out.append(f"{o.fullPathName}: z {t.z * 10:.1f}")
        out.append("joints: " + ", ".join(j.name for j in root.asBuiltJoints))
        if SAVE:
            doc.save("J1 rebuilt, shoulder at 209 mm (tools/fusion_assembly.py, 20 Oct)")
            out.append("saved")
    finally:
        doc.close(False)
    print("\n".join(out))


if __name__ == "__main__" and "__file__" in globals():   # the add-in runs this with no __file__
    import sys
    from fusion_sweep import post
    args = sys.argv[1:]
    src = open(__file__, encoding="utf-8").read().replace(
        'TARGET, SAVE = "", False', f"TARGET, SAVE = {args[0]!r}, {'--save' in args}", 1)
    sid, _ = post({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "deskarm", "version": "1"}}})
    post({"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)
    _, res = post({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {
        "name": "fusion_mcp_execute", "arguments": {"featureType": "script", "object": {"script": src}}}}, sid)
    reply = json.loads(res["result"]["content"][0]["text"])
    print(reply.get("message") or reply.get("error"))

"""Checks the J1 stack of tools/fusion_j1.py in memory (nothing saved): every part plus envelopes of the bought parts
(two 6810s, two 6804s, 26 ring pins dia 5 x 25, 8 output pins dia 6 x 25, the motor), turned through several cam angles
with the discs and the carrier following the 25:1 kinematics, then Fusion's interference check at each angle.

    software/.venv/Scripts/python.exe tools/fusion_j1_check.py 0 7 45 90 180      # cam angles, deg

Disc A's centre sits at e (cos t, sin t) and the disc turns -t/25 (output reverses); disc B is A turned 180 deg about
the axis (26 pins, so the pin set maps onto itself). The carrier turns with the discs. Press fits drawn line on line
(output pins in the carrier) show as touching faces, not volume.
"""
import json
import math


ANGLES, TOOLS, MATES = [0], "", False
E, N = 1.2, 26
T, S = "turntable:J1_Turntable", "turntable:J1_Inner_Spacer"
CL, CU = "cam:J1_Cam_Lower", "cam:J1_Cam_Upper"
MATES_TABLE = [   # (a, b, expected gap in mm, or ">" for clear of it)
    (T, "6810_up_in", 0), (T, "6810_up_out", ">"), (S, "6810_up_in", 0), (S, "6810_lo_in", 0),
    (S, "6810_up_out", ">"), (S, "6810_lo_out", ">"), ("carrier", "6810_lo_in", 0), ("carrier", "6810_lo_out", ">"),
    (T, "carrier", 0.2), (T, "base", 1.0), ("base", "6810_lo_out", 0), ("base", "6810_up_out", 0),
    ("base", "6810_lo_in", ">"), ("base", "6810_up_in", ">"), (CL, "6804A_in", 0), (CL, "6804A_out", ">"),
    (CU, "6804A_in", 0), (CU, "6804B_in", 0), (CU, "6804A_out", ">"), (CU, "6804B_out", ">"), (CL, CU, 0),
    (CL, "plate", ">"), ("cam", "bought_shaft", 0.05), ("disc", "6804A_out", 0.025), ("discB", "6804B_out", 0.025),
    ("disc", "plate", 1.0), ("disc", "discB", 1.0), ("discB", "carrier", 0.5), ("ring_pins", "plate", 0),
    ("ring_pins", "base", 0), ("ring_pins", "disc", 0.1), ("bought_out_pins", "carrier", 0),
    ("bought_out_pins", "plate", 0.5), ("bought_out_pins", "disc", 0), ("plate", "base", 0), ("motor", "plate", 0)]


def run(_context: str):
    import sys
    import adsk.core, adsk.fusion
    sys.path.insert(0, TOOLS)
    import importlib, fusion_j1
    importlib.reload(fusion_j1)                  # Fusion keeps modules imported between runs
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)   # scratch, never saved
    res = {}
    try:
        design = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = design.rootComponent
        if root.occurrences.count or root.bRepBodies.count:
            raise RuntimeError("template is not empty")
        occ = {}

        def part(name, build):
            o = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            o.component.name = name
            build(o.component)
            occ[name] = o
            return o

        for n in ("base", "plate", "cam", "carrier", "turntable", "disc"):
            part(n, lambda c, n=n: fusion_j1.build(n, c, design if n == "base" else _Fake(design)))
        part("discB", lambda c: fusion_j1.build("disc", c, _Fake(design)))
        import fusion_lib
        importlib.reload(fusion_lib).soften(occ["base"].component, (0, 0, 1))   # as fusion_j1.run saves it

        def cyl(c, x, y, z0, z1, d, d_in=0):
            pi = c.constructionPlanes.createInput()
            pi.setByOffset(c.xYConstructionPlane, adsk.core.ValueInput.createByReal(z0 / 10))
            sk = c.sketches.add(c.constructionPlanes.add(pi) if z0 else c.xYConstructionPlane)
            P = lambda xx, yy: adsk.core.Point3D.create(xx / 10, yy / 10, 0)
            sk.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), d / 20)
            if d_in:
                sk.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), d_in / 20)
            prof = [p for p in sk.profiles if p.profileLoops.count == (2 if d_in else 1)][0]
            c.features.extrudeFeatures.addSimple(prof, adsk.core.ValueInput.createByReal((z1 - z0) / 10),
                                                 adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        def flat(c):                                    # the LDO flat: 4.5 across, last 15 mm of the 24 mm shaft
            pi = c.constructionPlanes.createInput()
            pi.setByOffset(c.xYConstructionPlane, adsk.core.ValueInput.createByReal(1.0))   # worst case: flat from z 10
            sk = c.sketches.add(c.constructionPlanes.add(pi))
            sk.sketchCurves.sketchLines.addTwoPointRectangle(adsk.core.Point3D.create(-0.3, 0.2, 0),
                                                             adsk.core.Point3D.create(0.3, 0.3, 0))
            inp = c.features.extrudeFeatures.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
            inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(1.4))
            inp.participantBodies = list(c.bRepBodies)
            c.features.extrudeFeatures.add(inp)

        pol = lambda r, a: (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
        # bearings as separate inner and outer rings (6810: d1 ~54.9, D2 ~60.5; 6804: d1 ~23.6, D2 ~28.4), so the
        # mates show which ring each printed face bears on
        for nm, z in (("6810_lo", 36.5), ("6810_up", 53.5)):
            part(nm + "_in", lambda c, z=z: cyl(c, 0, 0, z, z + 7, 54.9, 50))
            part(nm + "_out", lambda c, z=z: cyl(c, 0, 0, z, z + 7, 65, 60.5))
        for nm, z in (("6804A", 7.5), ("6804B", 15.5)):
            part(nm + "_in", lambda c, z=z: cyl(c, 0, 0, z, z + 7, 23.6, 20))
            part(nm + "_out", lambda c, z=z: cyl(c, 0, 0, z, z + 7, 32, 28.4))
        part("ring_pins", lambda c: [cyl(c, *pol(40, k * 360 / N), 3, 28, 5) for k in range(N)])
        part("motor", lambda c: [cyl(c, 0, 0, -60, 0, 56), cyl(c, 0, 0, 0, 2, 22)])   # body as a dia 56 can, pilot
        part("bought_shaft", lambda c: [cyl(c, 0, 0, 2, 24, 5), flat(c)])   # LDO D shaft, turns with the cam
        part("bought_out_pins", lambda c: [cyl(c, *pol(26.1, 45 * k), 7, 32, 6) for k in range(8)])

        def place(o, rot_deg, tx, ty, tz=0):
            m = adsk.core.Matrix3D.create()
            m.setToRotation(math.radians(rot_deg), adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
            m.translation = adsk.core.Vector3D.create(tx / 10, ty / 10, tz / 10)
            o.transform = m

        for t in ANGLES:
            phi = -t / (N - 1)
            ax, ay = pol(E, t)
            place(occ["disc"], phi, ax, ay, 7.5)
            place(occ["discB"], phi + 180, -ax, -ay, 15.5)
            for nm in ("6804A_in", "6804A_out"):
                place(occ[nm], 0, ax, ay)
            for nm in ("6804B_in", "6804B_out"):
                place(occ[nm], 0, -ax, -ay)
            place(occ["cam"], t, 0, 0)
            place(occ["bought_shaft"], t, 0, 0)
            for n in ("carrier", "bought_out_pins", "turntable"):
                place(occ[n], phi, 0, 0)
            bodies = adsk.core.ObjectCollection.create()
            for o in root.allOccurrences:
                for b in o.bRepBodies:
                    bodies.add(b)
            ii = design.createInterferenceInput(bodies)
            ii.areCoincidentFacesIncluded = False
            hits = {}
            for r in design.analyzeInterference(ii):
                k = f"{r.entityOne.parentComponent.name} x {r.entityTwo.parentComponent.name}"
                hits[k] = round(hits.get(k, 0) + r.interferenceBody.volume * 1000, 2)     # mm3
            res[f"cam {t} deg"] = hits or "clear"
        if MATES:                                       # at the last angle: gaps that should be 0, or a set value
            mm = app.measureManager

            def bodies(key):
                o, _, b = key.partition(":")
                return [x for x in occ[o].bRepBodies if not b or x.name == b]
            for a, b, want in MATES_TABLE:
                m = min((mm.measureMinimumDistance(x, y) for x in bodies(a) for y in bodies(b)), key=lambda r: r.value)
                got = m.value * 10
                ok = (got > 0.05) if want == ">" else abs(got - want) < 0.03
                at = "" if ok else " between " + " and ".join(
                    "(" + ", ".join(f"{v * 10:.2f}" for v in q.asArray()) + ")" for q in (m.positionOne, m.positionTwo))
                res[f"{a} / {b}"] = f"{got:.3f} mm (want {want}) {'ok' if ok else 'CHECK'}{at}"
    finally:
        doc.close(False)
    import os                                    # also to a file: a run past the client's 130 s loses the reply
    open(os.path.join(TOOLS, ".fusion_out.txt"), "w", encoding="utf-8").write(
        "\n".join(f"{k} {v}" for k, v in res.items()))
    print(json.dumps(res))


class _Fake:
    """Only the base build adds the fit parameters; the other parts reuse them."""
    def __init__(self, design):
        self.userParameters = _NoAdd(design.userParameters)


class _NoAdd:
    def __init__(self, ups):
        self.ups = ups

    def add(self, *a):
        return None


if __name__ == "__main__" and "__file__" in globals():   # the add-in runs this with no __file__
    import sys
    from pathlib import Path
    from fusion_sweep import post
    src = open(__file__, encoding="utf-8").read().replace(
        'ANGLES, TOOLS, MATES = [0], "", False',
        f"ANGLES, TOOLS, MATES = {[float(a) for a in sys.argv[1:] if a != 'mates'] or [0]!r}, "
        f"{str(Path(__file__).parent)!r}, {'mates' in sys.argv}", 1)
    sid, _ = post({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "deskarm", "version": "1"}}})
    post({"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)
    _, res = post({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {
        "name": "fusion_mcp_execute", "arguments": {"featureType": "script", "object": {"script": src}}}}, sid)
    reply = json.loads(res["result"]["content"][0]["text"])
    if not reply.get("success"):
        sys.exit(reply.get("error"))
    for k, v in json.loads(reply["message"]).items():
        print(k, v)

"""J1 actuator in Fusion: rebuilds the J1 part files from the numbers below (docs/19 §14, decisions D36-D39).

    software/.venv/Scripts/python.exe tools/fusion_j1.py base plate cam disc carrier turntable   # build, report, close unsaved
    software/.venv/Scripts/python.exe tools/fusion_j1.py --save base ...                         # same, then save a new version

Coordinates: mm, Z up, the J1 axis on Z, z = 0 the top of the base board (the base frame of arm.toml). The disc is
drawn about its own centre (it sits e = 1.2 mm off the axis); everything else is drawn where it goes.

Stack (z): motor plate 0-6.5 (motor face under it, the motor hangs through the board) | two 7 mm discs at 7.5 and
15.5 on a double cam, 26 ring pins dia 5 x 25 (z 3-28, held by the plate and the base shelf: the lobes reach R 39.79,
so the cavity is dia 81 and can't hold them) | pin carrier 23-36.5 with 8 output pins dia 6 x 25 |
two 6810s at 36.5-43.5 and 53.5-60.5 on a 10 mm land (17 mm apart, so they take the arm's moment) | base top 61 |
turntable flange 62-74 (the cheeks start at 74, as on the 19 Oct turntable, so the shoulder is 74 + 135 = 209).
Inner rings are clamped between the flange step and the carrier through a printed inner spacer; the outer rings sit
on the land, the lower one held up by that preload. The harness is an external loop in a trough on the base top.

Existing hub files are cleared and rebuilt (the old versions stay in Fusion's version history); `plate` and `cam`
are new files, made in a new document (fusion_lib.scratch_doc) and saved into the parts folder. Runs over the
add-in's raw HTTP endpoint like tools/fusion_sweep.py; one part per call keeps each call under the 120 s timeout.
"""
import json
import math

LINEAGE = "urn:adsk.wipprod:dm.lineage:"
FILES = dict(base="TVG9cvxKQzaYPYpT7r_1dw", turntable="DaOv-H-SRu6VqINYPa5xrA", carrier="23rBK7akTGKtnOihvYXNug",
             disc="zfAhHGfwQay0aE8lArSq0g", plate="jHAj8-sFTBmPFRJhYG8-qw", cam="m00GME05TB2wXTXfOtjtCA")
NEW_NAMES = dict(plate="DeskArm_J1_Motor_Plate", cam="DeskArm_J1_Eccentric_Cam")
FOLDER_OF = "hM3fH0x2RJS4Pe9TRhWm1w"         # DeskArm_SA1: new files go into its folder (DeskArm_Parts)
URL = "http://127.0.0.1:58846/mcp"
PARTS, SAVE, TOOLS = [], False, ""         # filled in by __main__ before the source is sent

# Fits (user parameters in every file, so a coupon result is one edit): docs/print_tolerances.md decides them.
FITS = dict(seat_fit="0.05 mm",            # 6810 / 6804 outer ring in a printed bore
            journal_fit="-0.05 mm",        # 6810 inner ring on a printed journal
            pin_fit="0.05 mm",             # dowel pin in a printed hole (ring pins)
            press_fit="0 mm",              # output pins pressed into the carrier
            clear_fit="0.2 mm")            # clearance holes and pilots
DISC_POINTS = 360                          # fit points for the disc spline (the add-in times out with many more)
E = 1.2                                    # eccentricity (tools/cycloid.py --pins 26 --R 40 --Rr 2.5 --e 1.2)


def build(name, comp, design):
    import adsk.core, adsk.fusion
    CUT, JOIN, NEW = (adsk.fusion.FeatureOperations.CutFeatureOperation, adsk.fusion.FeatureOperations.JoinFeatureOperation,
                      adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    for k, v in FITS.items():
        design.userParameters.add(k, adsk.core.ValueInput.createByString(v), "mm", "fit, docs/print_tolerances.md")
    P = lambda x, y: adsk.core.Point3D.create(x / 10, y / 10, 0)
    planes = {}

    def plane(z):
        if z == 0:
            return comp.xYConstructionPlane
        if z not in planes:
            pi = comp.constructionPlanes.createInput()
            pi.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByString(f"{z} mm"))
            planes[z] = comp.constructionPlanes.add(pi)
            planes[z].name = f"z {z}"
        return planes[z]

    def ext(z0, z1, op, circles=(), polys=(), pick=None, fit=None, name=None, into=None):
        """Sketch at z0, extrude to z1. circles: (x, y, dia); polys: point lists; fit: user parameter added to dia."""
        sk = comp.sketches.add(plane(z0))
        for x, y, d in circles:
            c = sk.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), d / 20)
            if fit:
                dim = sk.sketchDimensions.addDiameterDimension(c, P(x + d / 2 + 3, y + 3))
                dim.parameter.expression = f"{d} mm + {fit}"
        for pts in polys:
            for a, b in zip(pts, pts[1:] + pts[:1]):
                sk.sketchCurves.sketchLines.addByTwoPoints(P(*a), P(*b))
        prof = adsk.core.ObjectCollection.create()
        for p in sk.profiles:
            if pick is None or pick(p):
                prof.add(p)
        inp = comp.features.extrudeFeatures.createInput(prof, op)
        inp.setDistanceExtent(False, adsk.core.ValueInput.createByString(f"{z1 - z0} mm"))
        if op != NEW:      # only this part's bodies (or one of them): a cut can reach other components' bodies
            inp.participantBodies = [into] if into else list(comp.bRepBodies)
        f = comp.features.extrudeFeatures.add(inp)
        if name:
            f.name = name
            sk.name = name
        return f

    ring = lambda p: p.profileLoops.count == 2
    polar = lambda r, a: (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
    bolt = lambda r, start, n=4: [polar(r, start + k * 360 / n) for k in range(n)]
    box = lambda x0, y0, x1, y1: [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    PINS = [polar(40, k * 360 / 26) + (5,) for k in range(26)]                 # 26 x dia 5 x 25 ring pins
    PLATE_BOLTS = [polar(46.5, a) for a in (48.46, 131.54, 228.46, 311.54)]    # midway between pins

    def rib(a, r0, r1, w):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        return [(r * c - t * s, r * s + t * c) for r, t in ((r0, -w / 2), (r1, -w / 2), (r1, w / 2), (r0, w / 2))]

    if name == "base":
        ext(0, 61, NEW, [(0, 0, 160)], name="Base body")
        ext(0, 6.5, CUT, [(0, 0, 100)], fit="clear_fit", name="Motor plate recess")
        ext(6.5, 23, CUT, [(0, 0, 81)], name="Disc cavity")     # lobes reach R 39.79 from the axis (e + profile)
        ext(6.5, 28, CUT, PINS, fit="pin_fit", name="Ring pin holes (pins held at both ends)")
        ext(23, 36.5, CUT, [(0, 0, 72)], name="Carrier cavity")
        ext(36.5, 43.5, CUT, [(0, 0, 65)], fit="seat_fit", name="Lower 6810 seat")
        ext(43.5, 53.5, CUT, [(0, 0, 58)], name="Land")
        ext(53.5, 61, CUT, [(0, 0, 65)], fit="seat_fit", name="Upper 6810 seat")
        ext(0, 45, CUT, [(0, 0, 152), (0, 0, 106)], pick=ring, name="Shell void")       # 4 mm outer wall
        ext(0, 45, JOIN, polys=[rib(22.5 + 45 * k, 52.5, 76.5, 4) for k in range(8)], name="Shell ribs")
        ext(0, 45, JOIN, [(x, y, 14) for x, y in bolt(71, 45)], name="M6 insert bosses")
        ext(49, 61, CUT, [(0, 0, 152), (0, 0, 132)], pick=ring, name="Harness trough")
        ext(0, 61, CUT, polys=[box(-5, -81, 5, -75)], name="Harness slot")
        ext(0, 13, CUT, [(x, y, 8) for x, y in bolt(71, 45)], name="M6 board inserts")
        ext(6.5, 15.5, CUT, [(x, y, 5.6) for x, y in PLATE_BOLTS], name="M4 plate inserts")
        ext(58.5, 61, CUT, polys=[box(-2.5, 56, 2.5, 60)], name="Hall switch (homing, D42)")
        ext(59.5, 61, CUT, polys=[box(-1, 60, 1, 67)], name="Hall wire groove to the trough")
        comp.bRepBodies.item(0).name = "J1_Base_Housing"
    elif name == "plate":
        ext(0, 6.5, NEW, [(0, 0, 99.6)], name="Plate body")
        ext(0, 2.2, CUT, [(0, 0, 22)], fit="clear_fit", name="NEMA17 pilot")
        ext(2.2, 6.5, CUT, [(0, 0, 24)], name="Cam passage")
        ext(0, 6.5, CUT, [(x, y, 3.4) for x, y in bolt(21.92, 45)], name="M3 motor holes")
        ext(3, 6.5, CUT, [(x, y, 6.5) for x, y in bolt(21.92, 45)], name="M3 counterbores")
        ext(3, 6.5, CUT, PINS, fit="pin_fit", name="Ring pin sockets")
        ext(0, 6.5, CUT, [(x, y, 4.5) for x, y in PLATE_BOLTS], name="M4 holes")
        ext(0, 4, CUT, [(x, y, 8) for x, y in PLATE_BOLTS], name="M4 counterbores")
        comp.bRepBodies.item(0).name = "J1_Motor_Plate"
    elif name == "cam":
        # Two pieces: one piece with a shoulder under lobe A and a spacer above it would trap the lower 6804 (its 20 mm
        # bore passes neither). Each shoulder is round about its own lobe, so it bears on the whole inner ring. LDO
        # drawing (Rev A): shaft dia 5 x 24 +-0.5 from the body face, flat 4.5 across over the last 15 +-0.5, so the
        # flat starts at z 8-10. Each piece is keyed on the flat (the key sets the 180 deg phase) and held by an M3
        # set screw through its lobe onto the flat, tightened before its bearing goes on.
        def piece(feats, label):
            body = None
            for z0, z1, x, d, fit in feats:
                f = ext(z0, z1, NEW if body is None else JOIN, [(x, 0, d)], fit=fit, into=body, name=f"{label} {z0}")
                body = body or f.bodies.item(0)
            body.name = f"J1_Cam_{label}"
            return body
        lower = piece([(4, 6.8, 0, 23, None), (6.8, 7.5, E, 23, None), (7.5, 14.5, E, 20, "journal_fit")], "Lower")
        upper = piece([(14.5, 15, E, 23, None), (15, 15.5, -E, 23, None), (15.5, 22.5, -E, 20, "journal_fit")], "Upper")
        ext(4, 22.5, CUT, [(0, 0, 5)], fit="clear_fit", name="Shaft bore")
        ext(10.5, 14.5, JOIN, polys=[box(-3, 2.05, 3, 3.0)], into=lower, name="D key, lower")
        ext(14.5, 22.5, JOIN, polys=[box(-3, 2.05, 3, 3.0)], into=upper, name="D key, upper")
        sk = comp.sketches.add(comp.xZConstructionPlane)                 # sketch y = -z on the XZ plane
        for z in (12, 19):
            sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, -z / 10, 0), 0.13)
        prof = adsk.core.ObjectCollection.create()
        for pr in sk.profiles:
            prof.add(pr)
        inp = comp.features.extrudeFeatures.createInput(prof, CUT)
        inp.setSymmetricExtent(adsk.core.ValueInput.createByString("30 mm"), True)
        inp.participantBodies = [lower, upper]
        comp.features.extrudeFeatures.add(inp).name = "M3 set screws (tap), through the lobes onto the flat"
    elif name == "disc":
        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.name = "Disc profile (tools/cycloid.py)"
        import sys
        sys.path.insert(0, TOOLS)
        from cycloid import disc_profile                                  # the same profile as the DXF
        pts = adsk.core.ObjectCollection.create()
        dense = disc_profile(40, 2.5 + 0.10, 26, E, n_points=20000)
        s_ = [0.0]
        for (x0, y0), (x1, y1) in zip(dense, dense[1:] + dense[:1]):
            s_.append(s_[-1] + math.hypot(x1 - x0, y1 - y0))
        n, j = DISC_POINTS, 0
        for i in range(n):                          # resample at equal arc length: even spacing round the lobes
            while s_[j + 1] < s_[-1] * i / n:
                j += 1
            x, y = dense[j]
            pts.add(P(x, y))
        pts.add(pts.item(0))                       # repeat the first point: a spline closed afterwards kinks at the seam
        sk.sketchCurves.sketchFittedSplines.add(pts)
        f = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), adsk.core.ValueInput.createByString("7 mm"), NEW)
        f.name = "Disc 7 mm (6804 width)"
        ext(0, 7, CUT, [(0, 0, 32)], fit="seat_fit", name="6804 seat")
        ext(0, 7, CUT, [polar(26.1, 45 * k) + (6 + 2 * E,) for k in range(8)], name="Output holes")
        f.bodies.item(0).name = "J1_Cycloid_Disc"
    elif name == "carrier":
        ext(23, 35.5, NEW, [(0, 0, 70)], name="Carrier body")
        ext(35.5, 36.5, JOIN, [(0, 0, 54)], name="Inner-ring boss")
        ext(23, 36.5, CUT, [(0, 0, 26)], name="Centre bore (cam, shaft end)")
        ext(23, 32, CUT, [polar(26.1, 45 * k) + (6,) for k in range(8)], fit="press_fit", name="Output pin pockets")
        ext(23, 36.5, CUT, [(x, y, 4.5) for x, y in bolt(20, 22.5)], name="M4 holes")
        ext(23, 27, CUT, [(x, y, 8) for x, y in bolt(20, 22.5)], name="M4 counterbores")
        comp.bRepBodies.item(0).name = "J1_Output_Pin_Carrier"
    elif name == "turntable":
        ext(62, 74, NEW, [(0, 0, 130)], name="Flange")
        ext(60.5, 62, JOIN, [(0, 0, 53.5)], name="Inner-ring step")
        # The stem stops 0.2 mm short of the ring stack (7 + 10 + 7 = 24 mm from 36.5 to 60.5): the carrier's boss
        # then lands on the lower inner ring first, and the M4s preload the rings instead of bottoming on the stem.
        ext(36.7, 60.5, JOIN, [(0, 0, 50)], fit="journal_fit", name="6810 journal (0.2 short: preload)")
        ext(36.7, 70, CUT, [(0, 0, 20)], name="Lightening bore")
        ext(36.7, 44.7, CUT, [(x, y, 5.6) for x, y in bolt(20, 22.5)], name="M4 carrier inserts")
        cheek = [(sx * 47, y) for sx in (-1, 1) for y in (-30, -10, 10, 30)]       # as the 19 Oct turntable
        ext(62, 74, CUT, [(x, y, 4.5) for x, y in cheek], name="Cheek M4 holes")
        ext(62, 74, CUT, [(sx * 47, y, 5) for sx in (-1, 1) for y in (-20, 20)], name="Cheek dowel holes")
        ext(62, 66.2, CUT, [(x, y, 8) for x, y in cheek], name="Cheek M4 counterbores")
        ext(62, 65, CUT, [(0, 58, 6.2)], name="Homing magnet 6 x 3")
        tt = comp.bRepBodies.item(0)
        tt.name = "J1_Turntable"
        ext(43.5, 53.5, NEW, [(0, 0, 54), (0, 0, 50.1)], pick=ring, name="Inner spacer")   # 6810 da 52-55
        comp.bRepBodies.item(1).name = "J1_Inner_Spacer"


def report(comp):
    out = [f"feature {f.name}: health {f.healthState}" for f in comp.features if f.healthState != 0]
    for b in comp.bRepBodies:
        bb = b.boundingBox
        out.append(f"{b.name}: {b.volume:.1f} cm3, x {bb.minPoint.x*10:.1f}..{bb.maxPoint.x*10:.1f}, "
                   f"z {bb.minPoint.z*10:.1f}..{bb.maxPoint.z*10:.1f}, faces {b.faces.count}")
    return out


def run(_context: str):
    import adsk.core, adsk.fusion
    app = adsk.core.Application.get()
    res = {}
    for name in PARTS:
        doc = (app.documents.open(app.data.findFileById(LINEAGE + FILES[name]), False) if FILES[name]
               else app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType))
        try:
            design = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
            design.designType = adsk.fusion.DesignTypes.ParametricDesignType
            if FILES[name] is None and (design.rootComponent.bRepBodies.count or design.rootComponent.occurrences.count):
                raise RuntimeError("template is not empty")
            tl = design.timeline
            if tl.count:
                tl.markerPosition = 0
                tl.deleteAllAfterMarker()
            for p in list(design.userParameters):
                p.deleteMe()
            build(name, design.rootComponent, design)
            res[name] = report(design.rootComponent)
            if name == "base":                                   # prints top down (docs/22 §5)
                import importlib, sys
                sys.path.insert(0, TOOLS)
                import fusion_lib
                res[name].append(importlib.reload(fusion_lib).soften(design.rootComponent, (0, 0, 1)))
            if SAVE:
                if FILES[name]:
                    doc.save("J1 rebuilt for D36-D39 (tools/fusion_j1.py, 20 Oct)")
                else:
                    doc.saveAs(NEW_NAMES[name], app.data.findFileById(LINEAGE + FOLDER_OF).parentFolder,
                               "J1 (tools/fusion_j1.py, 20 Oct)", "")
                res[name].append(f"saved: {doc.dataFile.name} v{doc.dataFile.versionNumber} {doc.dataFile.id}")
        finally:
            doc.close(False)
    print(json.dumps(res))


def post(body, sid=None):
    import urllib.request
    h = {"Content-Type": "application/json", "Accept": "application/json, text/event-stream"}
    if sid:
        h["MCP-Session-Id"] = sid
    r = urllib.request.urlopen(urllib.request.Request(URL, json.dumps(body).encode(), h), timeout=130)
    txt = r.read().decode()
    return r.headers.get("MCP-Session-Id"), json.loads(txt) if txt.strip() else None


if __name__ == "__main__" and "__file__" in globals():   # the add-in runs this with no __file__
    import sys
    from pathlib import Path
    args = sys.argv[1:]
    src = open(__file__, encoding="utf-8").read().replace(   # the add-in drops scripts over ~16 KB: keep it short
        'PARTS, SAVE, TOOLS = [], False, ""',
        f"PARTS, SAVE, TOOLS = {[a for a in args if a != '--save']!r}, {'--save' in args}, {str(Path(__file__).parent)!r}", 1)
    sid, _ = post({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "deskarm", "version": "1"}}})
    post({"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)
    _, res = post({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {
        "name": "fusion_mcp_execute", "arguments": {"featureType": "script", "object": {"script": src}}}}, sid)
    reply = json.loads(res["result"]["content"][0]["text"])
    if not reply.get("success"):
        sys.exit(reply.get("error"))
    try:
        parts = json.loads(reply["message"])
    except ValueError:
        sys.exit("unexpected reply: " + repr(reply)[:3000])
    for part, lines in parts.items():
        print(part)
        for line in lines:
            print("  ", line)

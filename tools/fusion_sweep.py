"""Clash sweep of the saved arm assembly in Fusion (docs/19 §13.9). Read-only: opens, drives, closes unsaved.

From the repo root, with Fusion open and its MCP add-in listening:
    software/.venv/Scripts/python.exe tools/fusion_sweep.py '[{"J2": 140}, {"J4": 74, "J2": 30}]'
Each pose names joint angles in degrees (J1, J2, J4, J5, J6); the rest stay at 0. The joint limits are lifted in
memory, because Fusion silently clamps a value past them. Keep a call under ~60 poses: the add-in times out at 120 s.
Run locally, the file sends its own source to the add-in, which calls run().
"""
import json

ARM = "urn:adsk.wipprod:dm.lineage:AKBQgdJpRGSmL9n61j_puQ"   # DeskArm_Arm_Assembly
URL = "http://127.0.0.1:58846/mcp"
POSES = [{}]


def joints(design):
    root = design.rootComponent
    js = {j.name: j for j in list(root.joints) + list(root.asBuiltJoints)}
    for occ in root.occurrences:      # J1 lives in SA1, J6 in SA5
        for j in list(occ.component.joints) + list(occ.component.asBuiltJoints):
            js[j.name] = j.createForAssemblyContext(occ)
    return {n: j for n, j in js.items() if hasattr(j.jointMotion, "rotationLimits")}


def clashes(design):
    import adsk.core
    bodies = adsk.core.ObjectCollection.create()
    for occ in design.rootComponent.allOccurrences:
        for b in occ.bRepBodies:
            if b.isSolid and b.isVisible:
                bodies.add(b)
    ii = design.createInterferenceInput(bodies)
    ii.areCoincidentFacesIncluded = False
    return sorted((r.entityOne.parentComponent.name, r.entityTwo.parentComponent.name,
                   round(r.interferenceBody.volume, 4)) for r in design.analyzeInterference(ii))


def run(_context: str):
    import adsk.core, adsk.fusion, math
    app = adsk.core.Application.get()
    doc = app.documents.open(app.data.findFileById(ARM), False)
    res = {}
    try:
        design = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        js = joints(design)
        for j in js.values():
            j.jointMotion.rotationLimits.isMinimumValueEnabled = False
            j.jointMotion.rotationLimits.isMaximumValueEnabled = False
        for pose in POSES:
            for n, j in js.items():
                j.jointMotion.rotationValue = math.radians(pose.get(n, 0))
            adsk.doEvents()
            res[json.dumps(pose)] = clashes(design)
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
    src = open(__file__, encoding="utf-8").read().replace("POSES = [{}]", "POSES = " + sys.argv[1], 1)
    sid, _ = post({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "deskarm", "version": "1"}}})
    post({"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)
    _, res = post({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {
        "name": "fusion_mcp_execute", "arguments": {"featureType": "script", "object": {"script": src}}}}, sid)
    reply = json.loads(res["result"]["content"][0]["text"])
    if not reply.get("success"):
        sys.exit(reply.get("error"))
    for pose, hits in json.loads(reply["message"]).items():
        print(pose, hits or "clear")

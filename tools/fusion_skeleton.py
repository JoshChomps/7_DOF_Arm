"""Fusion 360 skeleton for the arm: parameters, joint axes and key points in the zero pose (docs/16 §4 step 1).

Run once in a new, empty parametric design (Z up, mm), from Fusion's Scripts and Add-Ins dialog or through the
Fusion MCP. It adds user parameters for the link lengths, tool offset, spring geometry and joint limits, and a
"Skeleton" component with:

* level planes at the shoulder, elbow, wrist centre, flange and TCP (chained offsets of the parameters),
* construction points where the arm line crosses them, and the base origin,
* the seven joint axes, J1/J3/J5/J7 along +Z and J2/J4/J6 along +Y, as in the DH frames of arm.toml,
* a stick-figure sketch "Arm line (zero pose)" on the XZ plane.

Change a parameter (Modify > Change Parameters) and everything follows. The values below are copied from
software/deskarm/data/arm.toml; software/tests/test_tools.py fails if they drift apart. First run: 13 Oct 2026.
"""
import adsk.core
import adsk.fusion

LENGTHS = [
    ("base_height", 209, "arm.toml J1 d: base board (base frame origin) to the J2 axis (shoulder)"),
    ("upper_arm", 190, "arm.toml J3 d: shoulder to the J4 axis (elbow)"),
    ("forearm", 180, "arm.toml J5 d: elbow to the wrist centre (J5/J6/J7 axes meet)"),
    ("wrist_flange", 70, "arm.toml J7 d: wrist centre to the tool flange"),
    ("tcp_offset", 110, "arm.toml [tool] tcp_offset_z: flange to the tool centre point"),
    ("spring_anchor", 70, "arm.toml [counterbalance] a: spring pulley above the J2 axis"),
    ("spring_attach", 98, "arm.toml [counterbalance] b: spring attachment along the upper arm"),
]
LIMITS = {"J1": (-170, 170), "J2": (-120, 120), "J3": (-170, 170), "J4": (-120, 120),
          "J5": (-180, 180), "J6": (-120, 120), "J7": (-360, 360)}


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    val = adsk.core.ValueInput.createByString
    params = design.userParameters
    for name, mm, comment in LENGTHS:
        params.add(name, val(f"{mm} mm"), "mm", comment)
    for j, (lo, hi) in LIMITS.items():
        params.add(f"{j}_min", val(f"{lo} deg"), "deg", f"arm.toml {j} min_deg")
        params.add(f"{j}_max", val(f"{hi} deg"), "deg", f"arm.toml {j} max_deg")

    comp = design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
    comp.name = "Skeleton"

    def named(collection, setup, name):
        inp = collection.createInput()
        setup(inp)
        item = collection.add(inp)
        item.name = name
        return item

    planes, pts, axes = comp.constructionPlanes, comp.constructionPoints, comp.constructionAxes
    levels, prev = [], comp.xYConstructionPlane
    for expr, name in (("base_height", "Shoulder level (J2 axis)"), ("upper_arm", "Elbow level (J4 axis)"),
                       ("forearm", "Wrist-centre level (J6 axis)"), ("wrist_flange", "Flange level"),
                       ("tcp_offset", "TCP level")):
        prev = named(planes, lambda i, b=prev, e=expr: i.setByOffset(b, val(e)), name)
        levels.append(prev)
    shoulder_pl, elbow_pl, wrist_pl, flange_pl, tcp_pl = levels

    base = named(pts, lambda i: i.setByPoint(comp.originConstructionPoint), "Base origin (desk, under J1)")
    keys = [named(pts, lambda i, p=pl: i.setByEdgePlane(comp.zConstructionAxis, p), name) for pl, name in (
        (shoulder_pl, "Shoulder (J1, J2, J3 axes meet)"), (elbow_pl, "Elbow (J4 axis)"),
        (wrist_pl, "Wrist centre (J5, J6, J7 axes meet)"), (flange_pl, "Flange"), (tcp_pl, "TCP"))]
    shoulder, elbow, wrist, flange, tcp = keys

    for a, b, name in ((base, shoulder, "J1 axis (base yaw)"), (shoulder, elbow, "J3 axis (upper-arm roll)"),
                       (elbow, wrist, "J5 axis (forearm roll)"), (wrist, flange, "J7 axis (tool roll)")):
        named(axes, lambda i, a=a, b=b: i.setByTwoPoints(a, b), name)                       # +Z
    for pl, name in ((shoulder_pl, "J2 axis (shoulder pitch)"), (elbow_pl, "J4 axis (elbow pitch)"),
                     (wrist_pl, "J6 axis (wrist pitch)")):
        named(axes, lambda i, p=pl: i.setByTwoPlanes(p, comp.yZConstructionPlane), name)    # +Y (this order)

    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = "Arm line (zero pose)"
    sp = [sk.project(p).item(0) for p in (base, *keys)]
    for a, b in zip(sp, sp[1:]):
        sk.sketchCurves.sketchLines.addByTwoPoints(a, b)
    app.activeViewport.fit()
    for ax in axes:
        d = ax.geometry.direction
        print(ax.name, [round(v, 3) for v in (d.x, d.y, d.z)])

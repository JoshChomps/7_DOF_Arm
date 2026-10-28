"""Bought-part envelopes, module sub-assemblies, links and the full arm (imported by tools/fusion_arm.py).

Layout (docs/20): world = arm.toml base frame at the zero pose (arm straight up), mm, J1 on Z. The CAD's J2/J4/J6
axes run along X (the arm swings in the y-z plane), a 90 deg J1 offset from the DH frames. Modules are built in their
own frame (axis +z, input at z = 0) and placed by MODULES below; links are drawn directly in world coordinates.
"""
import math

import fusion_lib as L
from fusion_lib import CUT, JOIN, NEW, polar, circle_pts, rect

R_X_SIDE = ((0, 0, 1), (-1, 0, 0), (0, -1, 0))     # local z -> +X, local y (motor) -> -Z, local x -> -Y   (J2)
R_ELBOW = ((0, 0, 1), (1, 0, 0), (0, 1, 0))        # local z -> +X, local y -> +Z, local x -> +Y            (J4, J6)
R_Z180 = ((-1, 0, 0), (0, -1, 0), (0, 0, 1))       # J3: motor (local +y) behind the arm (world -y)
R_Z_M90 = ((0, 1, 0), (-1, 0, 0), (0, 0, 1))       # J7: motor (local +y) -> +X, along the J6 axis, so J6 tips it sideways
R_I = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
J2Z, J4Z, J6Z = 209.0, 399.0, 579.0
MODULES = {  # joint: (module type, rotation, translation)
    "J2": ("C2", R_X_SIDE, (-131.0, 0.0, J2Z)),
    "J3": ("C34", R_Z180, (0.0, 0.0, 245.0)),     # low on the upper arm so the elbow folds past 90 deg
    "J4": ("C34", R_ELBOW, (-124.5, 0.0, J4Z)),
    "J5": ("P56", R_I, (0.0, 0.0, 390.0)),         # low: the elbow folds to 120 deg and J6 has room; motor straddles J4
    "J6": ("P56", R_ELBOW, (-118.0, 0.0, J6Z)),
    "J7": ("P7", R_Z_M90, (0.0, 0.0, 598.0)),
}


def hull2(c1, r1, c2, r2, n=48):
    """Outline of the convex hull of two circles (a belt), CCW, as points."""
    (x1, y1), (x2, y2) = c1, c2
    d = math.hypot(x2 - x1, y2 - y1)
    base = math.atan2(y2 - y1, x2 - x1)
    a = math.acos((r1 - r2) / d)                           # tangent angle from the centre line
    pts = []
    for i in range(n + 1):                                 # big arc on c1, from +a to 2pi - a (the far side)
        t = base + a + (2 * math.pi - 2 * a) * i / n
        pts.append((x1 + r1 * math.cos(t), y1 + r1 * math.sin(t)))
    for i in range(n + 1):                                 # small arc on c2, from -a to +a
        t = base - a + 2 * a * i / n
        pts.append((x2 + r2 * math.cos(t), y2 + r2 * math.sin(t)))
    return pts


def b_motor(p, kind, MOTOR):
    """Motor (+ MKS driver on its back) in its own frame: face at z 0, body +z, shaft -z."""
    mo = MOTOR[kind]
    h = mo["sq"] / 2
    b = p.box(-h, h, -h, h, 0, mo["L"], body="Motor")
    p.box(-h, h, -h, h, mo["L"], mo["L"] + mo["drv"], body="MKS driver")
    p.cyl("z", -mo["pilot"][1], 0, mo["pilot"][0], body="Pilot")
    sh = p.cyl("z", -mo["shaft"][1], -mo["pilot"][1], mo["shaft"][0], body="Shaft")
    p.ext("z", -mo["shaft"][1], -mo["shaft"][1] + 15, CUT, polys=[rect(-3, 3, mo["shaft"][0] / 2 - 0.5, 4)],
          into=sh, name="Flat")


def b_pulley(p, teeth, pitch, width, bore):
    pd = teeth * pitch / math.pi
    b = p.ring("z", 1, width - 1, pd - 0.76, bore, body=f"{teeth}T pulley (bought)")      # tooth tips
    p.ring("z", 0, 1, pd + 3, bore, op=JOIN, into=b, name="Flange")
    p.ring("z", width - 1, width, pd + 3, bore, op=JOIN, into=b, name="Flange")


def b_belt(p, c, r1, r2, z0, z1, t=2.4, inner_off=0.3):
    """Belt envelope: pitch circles r1 at (0, 0), r2 at (0, c); thickness t outside the pitch line, inner face
    inner_off inside it (HTD-3M tooth tips sit 0.38 inside the pitch line, GT2 0.25)."""
    outer = hull2((0, 0), r1 + t, (0, c), r2 + t)
    inner = hull2((0, 0), r1 - inner_off, (0, c), r2 - inner_off)
    b = p.polyline("z", z0, z1, NEW, outer, body="Belt")
    p.polyline("z", z0, z1, CUT, inner, into=b, name="Belt inside")


def b_encoder(p, e, rod):
    """Output encoder (D33/D42), module frame: the 6 mm rod (rod = (z0, z1)), MR126, 6 x 2.5 magnet, board C box."""
    p.cyl("z", *rod, 6, body="Encoder rod 6 (steel)")
    p.ring("z", e["zc1"] - 4, e["zc1"], 12, 6, body="MR126")
    p.cyl("z", *e["mag"], 6, body="Magnet 6 x 2.5 diametric")
    bx, by = e["board_xy"]
    t = p.box(-bx, bx, -by, by, *e["board"], body="Board C (44 x 34, parts both sides)")
    p.ext("z", *e["board"], CUT, [(x, y, 2.2) for x, y in e["m2"]], into=t, name="M2")


def bought_parts(MOTOR, CYC, J7, enc=None, cyc=None):
    """name -> builder(p) of every bought-part envelope file."""
    out = {f"DA_P_{k}": (lambda p, k=k: b_motor(p, k, MOTOR)) for k in MOTOR}
    for m in ("C2", "C34"):
        if enc:
            e, s = enc(m), cyc(m)
            out[f"DA_P_Encoder_{m}"] = (lambda p, e=e, s=s: b_encoder(p, e, (e["mag"][1], s["flange1"] + 5)))
    out["DA_P_Pulley_HTD20_8"] = lambda p: b_pulley(p, 20, 3, 21, 8)
    out["DA_P_Pulley_HTD20_5"] = lambda p: b_pulley(p, 20, 3, 16, 5)
    out["DA_P_Pulley_GT16_5"] = lambda p: b_pulley(p, 16, 2, 9, 5)
    for m in ("C2", "C34"):
        c = CYC[m]
        pw, ext = c["belt_w"] + 4, c.get("ext", 0)
        out[f"DA_P_Belt_{m}"] = (lambda p, c=c, pw=pw, ext=ext: b_belt(p, c["C"], 40 * 3 / math.pi / 2, 20 * 3 / math.pi / 2,
                                                                       -4 - pw - ext + 1.5, -5.5 - ext))
    import fusion_arm_j7
    out["DA_P_SlipRing"] = fusion_arm_j7.slipring
    out["DA_P_Belt_P7"] = lambda p: b_belt(p, J7["C"], 80 * 2 / math.pi / 2, 16 * 2 / math.pi / 2, -8.5, -2.5, t=1.4, inner_off=0.2)
    return out


def module_layout(m, cyc, pg, j7, planet_phases):
    """[(part file, group, rotation, translation)] of module m in its own frame, and the motor/pulley/belt."""
    I = R_I
    if m in ("C2", "C34"):
        s = cyc(m)
        P = f"DA_{m}_"
        fixed = [(P + n, I, (0, 0, 0)) for n in ("Cover", "Housing", "Plate", "Shaft", "Pulley40", "Spacers")]
        fixed += [(P + "Disc", I, (s["e"], 0, 8.5)), (P + "Discb", I, (-s["e"], 0, 16.5))]
        motor = "DA_P_N23_76" if m == "C2" else "DA_P_N17_60"
        pul = "DA_P_Pulley_HTD20_8" if m == "C2" else "DA_P_Pulley_HTD20_5"
        fixed += [(motor, I, (0, s["C"], s["mface"])), (pul, I, (0, s["C"], s["shaft0"])),
                  (f"DA_P_Belt_{m}", I, (0, 0, 0)), (P + "Enccap", I, (0, 0, 0))]
        out = [(P + "Carrier", I, (0, 0, 0)), (P + "Flange", I, (0, 0, 0)), (f"DA_P_Encoder_{m}", I, (0, 0, 0))]
    elif m == "P56":
        s = pg()
        P = "DA_P56_"
        fixed = [(P + n, I, (0, 0, 0)) for n in ("Base", "Housing", "Sun1", "Carrier1", "Spacer")]
        for k, (phi, ph) in enumerate(planet_phases(s["zs"], s["zp"], s["n"])):
            x, y = s["a"] * math.cos(phi), s["a"] * math.sin(phi)
            for st in ("Planet1", "Planet2"):
                fixed.append((P + st, L.rot("z", math.degrees(ph)), (x, y, 0)))
        fixed.append(("DA_P_N17_40", L.rot("y", 180), (0, 0, 0)))      # body down, shaft flat stays at +y
        out = [(P + "Carrier2", I, (0, 0, 0)), (P + "Flange", I, (0, 0, 0))]
    else:
        s = j7()
        P = "DA_P7_"
        fixed = [(P + n, I, (0, 0, 0)) for n in ("Base", "Housing", "Sunshaft", "Pulley80", "Spacer")]
        for k, (phi, ph) in enumerate(planet_phases(s["zs"], s["zp"], s["n"])):
            fixed.append((P + "Planet", L.rot("z", math.degrees(ph)), (s["a"] * math.cos(phi), s["a"] * math.sin(phi), 0)))
        fixed += [("DA_P_N17_20", I, (0, s["C"], 8)), ("DA_P_Pulley_GT16_5", I, (0, s["C"], -10)),
                  ("DA_P_Belt_P7", I, (0, 0, 0))]
        fixed.append(("DA_P_SlipRing", I, (0, 0, 0)))            # body in link 6's arm, which rides with this group
        out = [(P + "Carrier", I, (0, 0, 0)), (P + "Flange", I, (0, 0, 0)), (P + "Sliptube", I, (0, 0, 0))]
    return fixed, out


def build_module_sa(m, layout, save=False):
    """Sub-assembly DA_SA_<m>: groups "<m> fixed" (grounded) and "<m> out", revolute joint about local z."""
    import adsk.core, adsk.fusion
    name = f"DA_SA_{m}"
    reg = L.registry()
    doc = L.doc_for(name)
    res = [name]
    try:
        d = L.design_of(doc)
        L.clear(d)
        root = d.rootComponent
        fixed, out = layout
        groups = []
        for gname, parts in ((f"{m} fixed", fixed), (f"{m} out", out)):
            g = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            g.component.name = gname
            for fname, R, t in parts:
                g.component.occurrences.addByInsert(L.datafile(fname), L.matrix(R, t), True)
            col = adsk.core.ObjectCollection.create()
            for o in g.component.occurrences:
                col.add(o)
            g.component.rigidGroups.add(col, True).name = gname + " rigid"
            groups.append(g)
        groups[0].isGrounded = True
        sk = root.sketches.add(root.xYConstructionPlane)
        sk.name = "Module axis"
        line = sk.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0), adsk.core.Point3D.create(0, 0, 2))
        geo = adsk.fusion.JointGeometry.createByCurve(line, adsk.fusion.JointKeyPointTypes.StartKeyPoint)
        inp = root.asBuiltJoints.createInput(groups[1], groups[0], geo)
        inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        root.asBuiltJoints.add(inp).name = f"{m} output"
        hits = L.interference(d)
        res.append("interference: " + (", ".join(f"{k} {v} mm3" for k, v in hits.items()) or "clear"))
        res.append(f"{root.allOccurrences.count} occurrences")
        if save:
            res.append(L.save(doc, name, f"module {m} (tools/fusion_arm_asm.py)"))
    finally:
        doc.close(False)
    return "\n".join(res)

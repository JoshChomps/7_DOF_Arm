"""Every part of the arm from J2 on, built from the numbers below (docs/20). Runs inside Fusion via tools/fusion_run.py.

Each part is drawn in its own local frame and saved as its own hub file (name prefix DA_). Actuator modules use a
local frame with the joint axis on +z and the input face at z = 0 (input side at z < 0, output towards +z); the
assemblies place them. Fits are user parameters in every file. Numbers that come from a tool say which.
"""
import math

import fusion_lib as L
from fusion_lib import CUT, JOIN, NEW, polar, circle_pts, rect, hall_sensor, hall_magnet

FITS = dict(seat_fit=0.05, journal_fit=-0.05, pin_fit=0.05, press_fit=0.0, clear_fit=0.2)
F = FITS            # numeric copies for geometry (the user parameters record them in each file)

# parts whose edges are rounded after building (fusion_lib.soften, docs/22 §5), with the outward normal of the face
# that goes on the print bed (tools/mesh_audit.py: the orientation with the least overhang). The gears, discs,
# shafts, pulleys and other turned parts are round already; the fingers print one at a time, each on its side.
SOFT = {"DA_C2_Housing": (0, 0, 1), "DA_C34_Housing": (0, 0, 1), "DA_P56_Housing": (0, 0, 1), "DA_P7_Housing": (0, 0, 1),
        "DA_L_StandLeft": (-1, 0, 0), "DA_L_TowerRight": (0, 0, -1), "DA_L_Link2": (-1, 0, 0), "DA_L_Link3": (0, 0, -1),
        "DA_L_Forearm": (0, 0, 1), "DA_L_Link5": (0, 0, -1), "DA_L_Link6": (-1, 0, 0),
        "DA_G_Body": (0, 0, -1), "DA_G_Carrier": (0, 0, 1), "DA_G_Fingers": None}

# --------------------------------------------------------------------------------------------- bought parts
BRG = {"6810": (50, 65, 7, 54.9, 60.5), "6806": (30, 42, 7, 33.7, 38.6), "6804": (20, 32, 7, 23.6, 28.4),
       "6802": (15, 24, 5, 17.5, 21.6), "623": (3, 10, 4, 4.8, 8.2), "608": (8, 22, 7, 12.1, 18.5)}  # d, D, B, d1, D2
MOTOR = {  # square, length, pilot dia x height, bolt square, shaft dia x length, driver depth
    "N17_60": dict(sq=42.3, L=60, pilot=(22, 2), bolt=31, shaft=(5, 24), drv=16, screw=3.4),
    "N17_40": dict(sq=42.3, L=40, pilot=(22, 2), bolt=31, shaft=(5, 24), drv=16, screw=3.4),
    "N17_20": dict(sq=42.3, L=20, pilot=(22, 2), bolt=31, shaft=(5, 20), drv=16, screw=3.4),
    "N23_76": dict(sq=56.4, L=76, pilot=(38.1, 1.6), bolt=47.14, shaft=(8, 21), drv=20, screw=5.5),
}

# --------------------------------------------------------------------------------------------- cycloid modules
# tools/cycloid.py: J2 --pins 25 --R 50 --Rr 3 --e 1.6 (r_max 48.5, out holes 9.2 on r 30.65);
#                   J3/J4 --pins 25 --R 36 --Rr 2.5 --e 1.1 --out-circle 24 --out-pin-dia 5 (r_max 34.5)
CYC = {
    "C2": dict(N=25, R=50, pin_d=6, e=1.6, out_r=30.65, out_pin_d=6, belt_w=15, motor="N23_76", C=95.5, belt=282,
               bolt_r=55, screw=4.5, insert=5.6, out_bolt_r=19, land=16, mode="land", od=121, tab_r=(60.5, 72), ext=22),
    "C34": dict(N=25, R=36, pin_d=5, e=1.1, out_r=24, out_pin_d=5, belt_w=10, motor="N17_60", C=69.8, belt=231,
                bolt_r=40, screw=3.4, insert=4.2, out_bolt_r=18, land=10, mode="spacer", od=90, tab_r=(45, 56), ext=14),
}


def cyc(m):
    """Derived numbers of a cycloid module (all z in the module frame)."""
    s = dict(CYC[m])
    import sys, os
    sys.path.insert(0, os.path.dirname(__file__))
    from cycloid import disc_profile
    prof = disc_profile(s["R"], s["pin_d"] / 2 + 0.10, s["N"], s["e"], n_points=4000)
    s["reach"] = max(math.hypot(x + s["e"], y) for x, y in prof)            # lobes, from the housing axis
    s["cavity"] = 2 * (s["reach"] + 0.6)
    s["carrier_d"] = round(2 * (s["out_r"] + s["out_pin_d"] / 2 + 3.5))
    s["pw"] = s["belt_w"] + 4                                               # 40T pulley width with flanges
    # ext: the input bearing, plate and belt sit ext mm further out on a boss with posts, so the motor (which runs
    # alongside the housing from the plate) ends ext mm short of the output face (J4: clear of the forearm)
    ext = s["ext"]
    s["shaft0"] = -4 - s["pw"] - ext
    s["b1"] = (3, 8) if ext == 0 else (-ext + 1, -ext + 6)      # input bearing (6802) #1
    s["mface"] = -ext                                             # motor face = inner side of the plate
    s["belt_z"] = (s["shaft0"] + 1.5, -5.5 - ext)
    s["S1"] = 37.5
    if s["mode"] == "land":
        s["S2"] = s["S1"] + 7 + s["land"]
        s["top"] = s["S2"] + 7.5
        s["flange0"] = s["S2"] + 8.5
    else:
        s["S2"] = s["S1"] + 7 + s["land"]
        s["top"] = s["S2"] + 7                     # housing top; retainer above it
        s["flange0"] = s["top"] + 3
    s["flange1"] = s["flange0"] + 8
    s["ring_pin_top"] = 33 if s["pin_d"] == 6 else 28
    s["pin_angles"] = [360 * k / s["N"] for k in range(s["N"])]
    s["bolt_angles"] = [180 / s["N"] + 72 * k for k in range(5)]          # midway between pins, every 5th gap
    s["dowel_angles"] = [180 / s["N"] + 360 / s["N"] * 2, 180 / s["N"] + 360 / s["N"] * 14]
    s["profile"] = prof
    return s


def params(design):
    import adsk.core
    for k, v in FITS.items():
        design.userParameters.add(k, adsk.core.ValueInput.createByString(f"{v} mm"), "mm", "fit (docs/print_tolerances.md)")


def b_cover(p, m):
    s = cyc(m)
    b = p.cyl("z", 0, 8, s["od"], body="Cover")
    e0, (z1, _) = -s["ext"], s["b1"]
    if s["ext"]:
        p.cyl("z", e0, 0, 34, op=JOIN, into=b, name="Bearing boss")
        p.ext("z", e0, 0, JOIN, [(*polar(s["bolt_r"], a), 9) for a in s["bolt_angles"]], into=b, name="Posts to the plate")
    p.cyl("z", e0, z1, 21, op=CUT, into=b, name="Hub passage (lip holds bearing 1)")
    p.cyl("z", z1, 8, 24 + F["seat_fit"], op=CUT, into=b, name="6802 seat (inserted from inside)")
    p.holes("z", 3, 8, [polar(s["R"], a) for a in s["pin_angles"]], s["pin_d"] + F["pin_fit"], into=b, name="Ring pin sockets")
    p.holes("z", e0, 8, [polar(s["bolt_r"], a) for a in s["bolt_angles"]], s["screw"], into=b, name="Cover screws")
    p.holes("z", 4, 8, [polar(s["bolt_r"], a) for a in s["dowel_angles"]], 4 + F["pin_fit"], into=b, name="Dowels")


def b_housing(p, m):
    s = cyc(m)
    b = p.cyl("z", 8, s["top"], s["od"], body="Housing")
    for a in (0, 180, 270):                                     # mounting tabs (the motor is at +y)
        r0, r1 = s["tab_r"]
        tab = L.rotated_rect(*polar((r0 + r1) / 2 - 2, a), r1 - r0 + 4, 16, a)
        p.ext("z", 26, 36, JOIN, polys=[tab], into=b, name=f"Tab {a}")
        p.holes("z", 26, 36, [polar(r1 - 5, a)], 4.5, into=b, name=f"Tab hole {a}")
    p.cyl("z", 8, 24, s["cavity"], op=CUT, into=b, name="Disc cavity")
    p.holes("z", 8, s["ring_pin_top"], [polar(s["R"], a) for a in s["pin_angles"]], s["pin_d"] + F["pin_fit"], into=b,
            name="Ring pin holes (held at both ends)")
    p.cyl("z", 24, s["S1"], s["carrier_d"] + 2, op=CUT, into=b, name="Carrier cavity")
    if s["mode"] == "land":
        p.cyl("z", s["S1"], s["S1"] + 7, 65 + F["seat_fit"], op=CUT, into=b, name="Lower 6810 seat")
        p.cyl("z", s["S1"] + 7, s["S2"], 62, op=CUT, into=b, name="Land")
        p.cyl("z", s["S2"], s["top"], 65 + F["seat_fit"], op=CUT, into=b, name="Upper 6810 seat")
    else:
        p.cyl("z", s["S1"], s["top"], 65 + F["seat_fit"], op=CUT, into=b, name="6810 seats + outer spacer")
        p.holes("z", s["top"] - 6, s["top"], circle_pts(39, 5, 0), s["insert"], into=b, name="Retainer inserts")
    p.holes("z", 8, 8 + 9, [polar(s["bolt_r"], a) for a in s["bolt_angles"]], s["insert"], into=b, name="Cover inserts")
    p.holes("z", 8, 12, [polar(s["bolt_r"], a) for a in s["dowel_angles"]], 4 + F["pin_fit"], into=b, name="Dowels")
    hall_sensor(p, b, hall_r(s), 36, s["top"], 2.5 if s["mode"] == "land" else 2, s["od"] / 2 + 1)


def b_shaft(p, m):
    s, e = cyc(m), cyc(m)["e"]
    z0 = s["shaft0"]
    b1 = s["b1"]
    b = p.cyl("z", z0, b1[1], 15 + F["journal_fit"], body="Input shaft")
    p.cyl("z", b1[1], 8.5, 17, op=JOIN, into=b, name="Collar / journal 17")
    p.cyl("z", 8.5, 15.5, 20 + F["journal_fit"], at=(e, 0), op=JOIN, into=b, name="Lobe A")
    p.cyl("z", 15.5, 16, 23, at=(e, 0), op=JOIN, into=b, name="Divider A")
    p.cyl("z", 16, 16.5, 23, at=(-e, 0), op=JOIN, into=b, name="Divider B")
    p.cyl("z", 16.5, 23.5, 20 + F["journal_fit"], at=(-e, 0), op=JOIN, into=b, name="Lobe B")
    p.cyl("z", 23.5, 30, 17, op=JOIN, into=b, name="Journal 17")
    p.cyl("z", 30, 35, 15 + F["journal_fit"], op=JOIN, into=b, name="Bearing 2 journal")
    p.ext("z", z0, -4 - s["ext"], CUT, polys=[rect(-8, 8, 7.0, 8)], into=b, name="Pulley flat")
    p.cyl("z", z0, 35, 10.5, op=CUT, into=b, name="Bore (encoder rod / J3 tube)")


def b_pulley40(p, m):
    s = cyc(m)
    z0, pw = s["shaft0"], s["pw"]
    od = 40 * 3 / math.pi - 0.762                                       # HTD-3M 40T: pitch 38.20, OD 37.44
    t0, t1 = s["belt_z"]
    hub1 = s["b1"][0]
    b = p.cyl("z", t0, t1, od, body="Pulley 40T")
    p.cyl("z", z0, t0, 42, op=JOIN, into=b, name="Flange outer")
    p.cyl("z", t1, t1 + 1.5, 42, op=JOIN, into=b, name="Flange inner")
    p.cyl("z", t1 + 1.5, hub1, 17, op=JOIN, into=b, name="Hub (to bearing 1 inner ring)")
    p.ext("z", t0, t1, CUT, [(*polar(od / 2 - 0.22, 9 * k), 1.9) for k in range(40)], into=b, name="HTD-3M grooves")
    p.cyl("z", z0, hub1, 15.1, op=CUT, into=b, name="Bore")
    p.ext("z", z0, t1 + 1, JOIN, polys=[rect(-3, 3, 7.05, 8.5)], into=b, name="Flat key (on the shaft flat)")
    p.ext("y", 0, 30, CUT, [(0, (t0 + t1) / 2, 3.4)], into=b, name="M3 set screw (tap) onto the flat")   # (x, z)


def b_disc(p, m, second=False):
    """Disc A, or disc B: the second disc runs half a turn of the input later, so its outline is turned by
    -180/(N-1) (half a lobe pitch for an even lobe count) while its output holes stay where disc A's are."""
    s = cyc(m)
    prof = s["profile"]
    if second:
        a = math.radians(-180 / (s["N"] - 1))
        prof = [(x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)) for x, y in prof]
    p.spline("z", 0, 7, NEW, L.arc_resample(prof, 360), body="Disc")
    b = p.bodies["Disc"]
    p.cyl("z", 0, 7, 32 + F["seat_fit"], op=CUT, into=b, name="6804 seat")
    p.holes("z", 0, 7, circle_pts(s["out_r"], 8), s["out_pin_d"] + 2 * s["e"], into=b, name="Output holes")


def b_carrier(p, m):
    s = cyc(m)
    b = p.cyl("z", 24.5, 36.5, s["carrier_d"], body="Carrier")
    p.cyl("z", 36.5, 37.5, 53, op=JOIN, into=b, name="Inner-ring boss")
    p.cyl("z", 24.5, 37.5, 24 + F["seat_fit"], op=CUT, into=b, name="6802 seat (floating)")
    p.holes("z", 24.5, 33.5, circle_pts(s["out_r"], 8), s["out_pin_d"] + F["press_fit"], into=b, name="Output pin pockets")
    bolts = circle_pts(s["out_bolt_r"], 4, 22.5)
    p.holes("z", 24.5, 37.5, bolts, 4.5, into=b, name="M4 to stem")
    p.holes("z", 24.5, 28.5, bolts, 8, into=b, name="M4 counterbores")


def b_flange(p, m):
    s = cyc(m)
    b = p.cyl("z", s["S1"] + 0.2, s["S2"] + 7, 50 + F["journal_fit"], body="Output flange")
    p.cyl("z", s["S2"] + 7, s["flange0"], 53, op=JOIN, into=b, name="Inner-ring step")
    fd = s["od"] - 4
    p.cyl("z", s["flange0"], s["flange1"], fd, op=JOIN, into=b, name="Flange plate")
    p.cyl("z", s["S1"] + 0.2, s["flange1"], 26, op=CUT, into=b, name="Bore")
    p.holes("z", s["S1"] + 0.2, s["S1"] + 8.2, circle_pts(s["out_bolt_r"], 4, 22.5), 5.6, into=b, name="M4 carrier inserts")
    p.holes("z", s["flange1"] - 7, s["flange1"], circle_pts(fd / 2 - 6, 6, 0), 5.6, into=b, name="M4 link inserts")
    hall_magnet(p, b, hall_r(s), 36, s["flange0"])


def b_spacers(p, m):
    s = cyc(m)
    p.ring("z", s["S1"] + 7, s["S2"], 54, 50.1, body="Inner spacer")
    if s["mode"] == "spacer":
        p.ring("z", s["S1"] + 7, s["S2"], 65 - 0.1, 61, body="Outer spacer")
        r = p.ring("z", s["top"], s["top"] + 2, s["od"], 61, body="Retainer")
        p.holes("z", s["top"], s["top"] + 2, circle_pts(39, 5, 0), 3.4, into=r, name="Retainer screws")
        hall_sensor(p, r, hall_r(s), 36, s["top"] + 2, 2, s["od"] / 2 + 1, name="Hall switch (through)")


def b_plate(p, m):
    """Aluminium motor plate, 3 mm (D39): the housing outline plus a lobe to the motor at local +y."""
    s = cyc(m)
    mo = MOTOR[s["motor"]]
    half = mo["sq"] / 2 + 5
    z0, z1 = s["mface"] - 3, s["mface"]
    b = p.cyl("z", z0, z1, s["od"], body="Motor plate (Al 3 mm)")
    p.ext("z", z0, z1, JOIN, polys=[rect(-half, half, 0, s["C"] + half)], into=b, name="Motor lobe")
    p.cyl("z", z0, z1, 21, op=CUT, into=b, name="Hub hole")
    p.holes("z", z0, z1, [polar(s["bolt_r"], a) for a in s["bolt_angles"]], s["screw"], into=b, name="Cover screws")
    p.holes("z", z0, z1, enc(m)["posts"], 3.4, into=b, name="Encoder cap M3")
    slot = lambda cx, cy, d: L.stadium(cx, cy, d, 3)
    pil = mo["pilot"][0] + 0.4
    p.ext("z", z0, z1, CUT, polys=[slot(0, s["C"], pil)], into=b, name="Pilot slot")
    h = mo["bolt"] / 2
    p.ext("z", z0, z1, CUT, polys=[slot(x, s["C"] + y, mo["screw"]) for x in (-h, h) for y in (-h, h)], into=b,
          name="Motor slots (belt tension +-3)")


# --------------------------------------------------------------------------------------------- sensing (D33, D42)
# Homing: an A3144 hall switch (TO-92, 4.1 x 3 x 1.5) in the fixed side, a 6 x 3 magnet in the output, across the
# 1 mm gap. Output angle on J2-J4: a 6 mm rod pressed into the link on the output runs back through the flange and
# the hollow input shaft to an MR126 in the encoder cap; a 6 x 2.5 diametric magnet on its end faces the MT6701 on
# board C (25 x 25), 1 mm away.
def enc(m):
    """Encoder cap numbers (module frame): cap plate z, post positions, board C box."""
    s = cyc(m)
    zc1 = s["shaft0"] - 1                            # cap top, 1 mm below the shaft end
    return dict(zc0=zc1 - 5, zc1=zc1, zp=s["mface"] - 3, posts=[polar(30, a) for a in (200, 340)],
                mag=(zc1 - 5 - 3, zc1 - 5 - 0.5), board=(zc1 - 5 - 10, zc1 - 5 - 4),
                # board C (21 Oct): 44 x 34, room for the ESP32-C3-WROOM-02 (18 x 20), buck, USB-C and two CAN
                # connectors round the MT6701 on the axis; 25 x 25 was too small for the schematic
                board_xy=(22, 17), m2=[(x, y) for x in (-19, 19) for y in (-13, 13)])


def b_enccap(p, m):
    """Encoder cap: holds the rod's MR126 and board C under the input shaft end, on two posts screwed through the
    motor plate (M3, nuts on the plate's top face); the posts pass the 40T pulley on the side away from the belt."""
    e = enc(m)
    b = p.ext("z", e["zc0"], e["zc1"], NEW, polys=[rect(-34, 34, -16, 14)], body="Encoder cap")
    p.ext("z", e["zc1"], e["zp"], JOIN, [(x, y, 8) for x, y in e["posts"]], into=b, name="Posts to the motor plate")
    p.ext("z", e["zc0"], e["zp"], CUT, [(x, y, 3.4) for x, y in e["posts"]], into=b, name="M3 through")
    p.cyl("z", e["zc1"] - 4, e["zc1"], 12 + F["seat_fit"], op=CUT, into=b, name="MR126 seat")
    p.cyl("z", e["zc0"], e["zc1"] - 4, 8, op=CUT, into=b, name="Rod end and magnet")
    p.ext("z", e["board"][1], e["zc0"], JOIN, [(x, y, 5) for x, y in e["m2"]], into=b, name="Board C standoffs")
    p.ext("z", e["board"][1], e["zc0"], CUT, [(x, y, 1.8) for x, y in e["m2"]], into=b, name="M2 self-tapping")


def hall_r(s):
    """Hall pair radius: between the 6810 seat (R 32.5) and the outside, on the flange's screw circle."""
    return (s["od"] - 4) / 2 - 6


def b_discB(p, m):
    b_disc(p, m, second=True)


BUILDERS = {"enccap": b_enccap, "cover": b_cover, "housing": b_housing, "shaft": b_shaft, "pulley40": b_pulley40, "disc": b_disc,
            "discB": b_discB,
            "carrier": b_carrier, "flange": b_flange, "spacers": b_spacers, "plate": b_plate}


def builders(m):
    if m == "P7":
        import importlib, fusion_arm_j7
        importlib.reload(fusion_arm_j7)
        return fusion_arm_j7.builders(F, MOTOR)
    return PG_BUILDERS if m == "P56" else BUILDERS


def part_names(m):
    return {k: f"DA_{m}_{k.capitalize()}" for k in builders(m)}


# --------------------------------------------------------------------------------------------- CLI
def build(m, part, save=False, scratch=False):
    """Build module m's part into its hub file (or the template when it doesn't exist yet). scratch: never save."""
    name = part_names(m)[part]
    reg = L.registry()
    doc = L.doc_for(name)
    out = [name]
    try:
        d = L.design_of(doc)
        if name not in reg and (d.rootComponent.bRepBodies.count or d.rootComponent.occurrences.count):
            raise RuntimeError("template is not empty")
        L.clear(d)
        params(d)
        p = L.Part(d.rootComponent, d)
        builders(m)[part](p, m)
        if name in SOFT:
            out.append(L.soften(d.rootComponent, SOFT[name]))
        out += L.report(d.rootComponent)
        if save and not scratch:
            out.append(L.save(doc, name, f"{part} of {m} (tools/fusion_arm.py)"))
    finally:
        doc.close(False)
    return "\n".join(out)


def cli(args):
    cmd = args[0]
    if cmd == "build":
        return build(args[1], args[2], save="--save" in args)
    if cmd == "check" and args[1] == "P7":
        import importlib, fusion_arm_j7
        importlib.reload(fusion_arm_j7)
        return fusion_arm_j7.check(_scratch, _env, _mates, BRG, MOTOR, F,
                                   [float(a) for a in args[2:] if a != "mates"] or [0], "mates" in args)
    if cmd == "check" and args[1] == "P56":
        return check_pg([float(a) for a in args[2:] if a != "mates"] or [0], "mates" in args)
    if cmd == "check":
        return check_cycloid(args[1], [float(a) for a in args[2:] if a != "mates"] or [0], "mates" in args)
    if cmd == "bought":
        A, J = asm()
        names = list(A.bought_parts(MOTOR, CYC, J.J7, enc, cyc)) if args[1] == "all" else [args[1]]
        return "\n".join(build_bought(n, "--save" in args) for n in names)
    if cmd == "sa":
        return build_sa(args[1], "--save" in args)
    if cmd in ("arm", "sweep"):                       # sweep '[{"J2": 90}]' [detail] | sweep holes
        import importlib, json as _json, fusion_arm_full
        asm()
        importlib.reload(fusion_arm_full)
        if cmd == "arm":
            return fusion_arm_full.build("--save" in args)
        if args[1:2] == ["holes"]:
            return fusion_arm_full.holes()
        if args[1:2] == ["bodies"]:                   # sweep bodies: volumes + centres -> tools/cad_bodies.json
            import os
            return fusion_arm_full.bodies(os.path.join(os.path.dirname(__file__), "cad_bodies.json"))
        if args[1:2] == ["inventory"]:                # sweep inventory: every small hole -> tools/cad_holes.json
            import os
            return fusion_arm_full.inventory(os.path.join(os.path.dirname(__file__), "cad_holes.json"))
        if args[1:2] == ["stl"]:                      # sweep stl: every printed part -> stl/da/
            import os
            return fusion_arm_full.export_stl(os.path.join(os.path.dirname(__file__), "..", "stl", "da"), only=args[2:])
        if args[1:2] == ["tool"]:                     # sweep tool '[{"J2": 30}]': where the tool plate goes
            return fusion_arm_full.tool_point(_json.loads(args[2]))
        return fusion_arm_full.sweep(_json.loads(args[1]), "detail" in args)
    if cmd == "link":
        A, J = asm()                                  # first: fusion_arm_links imports MODULES from fusion_arm_asm
        import importlib, fusion_arm_links
        importlib.reload(fusion_arm_links)
        fns = fusion_arm_links.all_links(cyc, pg, J.j7)
        names = list(fns) if args[1] == "all" else [args[1]]
        out = []
        for name in names:
            doc = L.doc_for(name)
            try:
                d = L.design_of(doc)
                L.clear(d)
                params(d)
                fns[name](L.Part(d.rootComponent, d))
                out.append(name)
                if name in SOFT:
                    out.append(L.soften(d.rootComponent, SOFT[name]))
                out += L.report(d.rootComponent)
                if "--save" in args:
                    out.append(L.save(doc, name, "link (tools/fusion_arm_links.py)"))
            finally:
                doc.close(False)
        return "\n".join(out)
    if cmd == "gripcheck":                            # gripcheck [apertures]: gripper + tool plate, in memory
        import importlib, adsk.core, fusion_arm_links as G
        asm()
        importlib.reload(G)
        out = []
        for ap in [float(a) for a in args[1:]] or G.G_OPEN + (40.0,):
            doc = L.scratch_doc()
            try:
                d = L.design_of(doc)
                for name, fn in (("Tool plate", G.toolplate), ("Body", G.gripper_body),
                                 ("Fingers", lambda p: G.gripper_fingers(p, ap)),
                                 ("Pads", lambda p: G.gripper_pads(p, ap)),
                                 ("Carrier", lambda p: G.gripper_carrier(p, ap)), ("Bought", lambda p: G.gripper_bought(p, ap))):
                    o = d.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                    o.component.name = name
                    fn(L.Part(o.component, d))
                    if f"DA_G_{name}" in SOFT:
                        L.soften(o.component, SOFT[f"DA_G_{name}"])
                hits = L.interference(d, detail=True)
                out.append(f"{ap:g} mm: " + (", ".join(f"{k} {v}" for k, v in hits.items()) or "clear"))
            finally:
                doc.close(False)
        return "\n".join(out)
    if cmd == "rm":                                   # rm NAME...: delete superseded part files
        out = []
        for n in args[1:]:
            f = L.datafile(n)
            refs = [r.name for r in f.parentReferences] if f.parentReferences else []
            if refs:
                out.append(f"{n}: kept, still used by {refs}")
                continue
            try:
                f.deleteMe()
                out.append(f"{n}: deleted")
            except RuntimeError as e:                 # e.g. open in a tab, or in an old assembly version
                out.append(f"{n}: Fusion refused ({e}); open: {[d.name for d in L.app().documents]}")
        return "\n".join(out)
    if cmd == "ids":
        return "\n".join(L.ids())
    if cmd == "stale":                                # stale [--delete]: the *_dup_old files
        return "\n".join(L.stale("--delete" in args))
    if cmd == "old":                                  # the 19 Oct DeskArm_* files, and which the arm still uses
        return "\n".join(L.old_files(delete="--delete" in args))
    if cmd == "display":                              # display print|full [--save] | tool NAME POSES | mesh DIR
        asm()
        import importlib, fusion_arm_full, fusion_arm_display
        importlib.reload(fusion_arm_full)
        importlib.reload(fusion_arm_display)
        return fusion_arm_display.cli(args[1:])
    raise ValueError(cmd)


# --------------------------------------------------------------------------------------------- module check
def _env(p, name, solids):
    """A bought-part envelope component: solids = [(z0, z1, od, id, (x, y))]."""
    for i, (z0, z1, od, idia, at) in enumerate(solids):
        if idia:
            p.ring("z", z0, z1, od, idia, at=at, body=f"{name}{i}")
        else:
            p.cyl("z", z0, z1, od, at=at, body=f"{name}{i}")


def check_cycloid(m, angles=(0,), mates=False):
    """In-memory check of cycloid module m (template doc, never saved): interference at each input angle, and with
    mates=True the contact/gap table at the last angle."""
    import adsk.core
    s = cyc(m)
    doc = L.scratch_doc()
    res = []
    try:
        d = L.design_of(doc)
        root = d.rootComponent
        if root.occurrences.count or root.bRepBodies.count:
            raise RuntimeError("template is not empty")
        params(d)
        occ = {}

        def comp(name, fn):
            o = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            o.component.name = name
            fn(L.Part(o.component, d))
            occ[name] = o

        for part in BUILDERS:
            comp(part, lambda p, part=part: BUILDERS[part](p, m))
        L.soften(occ["housing"].component, SOFT[f"DA_{m}_Housing"])                  # check what gets printed
        mo = MOTOR[s["motor"]]
        b1, b2, b4, b10 = BRG["6802"], BRG["6802"], BRG["6804"], BRG["6810"]
        z1, z2 = s["b1"]
        comp("6802_1_in", lambda p: _env(p, "r", [(z1, z2, b1[3], b1[0], (0, 0))]))
        comp("6802_1_out", lambda p: _env(p, "r", [(z1, z2, b1[1], b1[4], (0, 0))]))
        comp("6802_2_in", lambda p: _env(p, "r", [(30, 35, b2[3], b2[0], (0, 0))]))
        comp("6802_2_out", lambda p: _env(p, "r", [(30, 35, b2[1], b2[4], (0, 0))]))
        for nm, z in (("A", 8.5), ("B", 16.5)):
            comp(f"6804{nm}_in", lambda p, z=z: _env(p, "r", [(z, z + 7, b4[3], b4[0], (0, 0))]))
            comp(f"6804{nm}_out", lambda p, z=z: _env(p, "r", [(z, z + 7, b4[1], b4[4], (0, 0))]))
        for nm, z in (("lo", s["S1"]), ("up", s["S2"])):
            comp(f"6810_{nm}_in", lambda p, z=z: _env(p, "r", [(z, z + 7, b10[3], b10[0], (0, 0))]))
            comp(f"6810_{nm}_out", lambda p, z=z: _env(p, "r", [(z, z + 7, b10[1], b10[4], (0, 0))]))
        comp("ring_pins", lambda p: _env(p, "p", [(3, s["ring_pin_top"], s["pin_d"], 0, polar(s["R"], a)) for a in s["pin_angles"]]))
        comp("out_pins", lambda p: _env(p, "o", [(8.5, 33.5, s["out_pin_d"], 0, xy) for xy in circle_pts(s["out_r"], 8)]))
        e, (A, _) = enc(m), asm()
        comp("encoder", lambda p: A.b_encoder(p, e, (e["mag"][1], s["flange1"])))     # rod stops at the flange face here

        def motor(p):
            h = mo["sq"] / 2
            f = s["mface"]
            p.box(-h, h, s["C"] - h, s["C"] + h, f, f + mo["L"] + mo["drv"], body="motor+driver")
            p.cyl("z", f - mo["pilot"][1], f, mo["pilot"][0], at=(0, s["C"]), body="pilot")
            p.cyl("z", f - mo["shaft"][1], f - mo["pilot"][1], mo["shaft"][0], at=(0, s["C"]), body="shaft")
            p.ring("z", s["shaft0"], s["belt_z"][1] + 1.5, 20 * 3 / math.pi + 4, mo["shaft"][0], at=(0, s["C"]), body="20T pulley")
        comp("motor", motor)

        def place(name, deg, x=0.0, y=0.0):
            occ[name].transform = L.matrix(L.rot("z", deg), (x, y, 0))
            d.snapshots.add() if False else None

        for t in angles:
            phi = -t / (s["N"] - 1)
            ex, ey = polar(s["e"], t)
            place("disc", phi, ex, ey)
            occ["disc"].transform = L.matrix(L.rot("z", phi), (ex, ey, 8.5))
            occ["discB"].transform = L.matrix(L.rot("z", phi), (-ex, -ey, 16.5))
            for nm in ("6804A_in", "6804A_out"):
                place(nm, 0, ex, ey)
            for nm in ("6804B_in", "6804B_out"):
                place(nm, 0, -ex, -ey)
            for nm in ("shaft", "pulley40"):
                place(nm, t)
            for nm in ("carrier", "flange", "out_pins"):
                place(nm, phi)
            hits = L.interference(d)
            res.append(f"input {t} deg: " + (", ".join(f"{k} {v} mm3" for k, v in hits.items()) or "clear"))
        if mates:
            mm = L.app().measureManager
            land = s["mode"] == "land"
            table = [("flange", "6810_up_in", 0), ("flange", "6810_up_out", ">"), ("flange", "carrier", 0.2),
                     ("spacers:Inner spacer", "6810_up_in", 0), ("spacers:Inner spacer", "6810_lo_in", 0),
                     ("spacers:Inner spacer", "6810_lo_out", ">"), ("carrier", "6810_lo_in", 0),
                     ("carrier", "6810_lo_out", ">"), ("housing", "6810_lo_out", 0), ("housing", "6810_up_out", 0 if land else 0.025),
                     ("housing", "6810_lo_in", ">"), ("housing", "flange", ">"), ("shaft", "6802_1_in", 0),
                     ("shaft", "6802_2_in", 0), ("cover", "6802_1_out", 0), ("carrier", "6802_2_out", 0.025),
                     ("shaft", "6804A_in", 0), ("shaft", "6804B_in", 0), ("disc", "6804A_out", 0.025),
                     ("discB", "6804B_out", 0.025), ("disc", "cover", 0.5), ("disc", "discB", 1.0), ("discB", "carrier", 1.0),
                     ("ring_pins", "cover", 0), ("ring_pins", "housing", 0), ("ring_pins", "disc", 0.1),
                     ("out_pins", "carrier", 0), ("out_pins", "cover", 0.5), ("out_pins", "disc", 0),
                     ("pulley40", "shaft", 0.05), ("pulley40", "6802_1_in", 0), ("pulley40", "cover", ">"),
                     ("pulley40", "plate", ">"), ("plate", "cover", 0), ("motor", "plate", 0), ("motor", "housing", ">"),
                     ("motor", "pulley40", ">"), ("shaft", "cover", ">"), ("shaft", "carrier", ">")]
            if not land:
                table += [("spacers:Outer spacer", "6810_lo_out", 0), ("spacers:Outer spacer", "6810_up_out", 0),
                          ("spacers:Retainer", "6810_up_out", 0), ("spacers:Retainer", "flange", ">"),
                          ("spacers:Outer spacer", "6810_lo_in", ">")]

            def bodies(key):
                o, _, b = key.partition(":")
                return [x for x in occ[o].bRepBodies if not b or x.name == b]
            for a, b, want in table:
                r = min((mm.measureMinimumDistance(x, y) for x in bodies(a) for y in bodies(b)), key=lambda q: q.value)
                got = r.value * 10
                ok = got > 0.05 if want == ">" else abs(got - want) < 0.03
                at = "" if ok else " at (" + ", ".join(f"{v * 10:.1f}" for v in r.positionOne.asArray()) + ")"
                res.append(f"{'ok   ' if ok else 'CHECK'} {a} / {b}: {got:.3f} (want {want}){at}")
    finally:
        doc.close(False)
    return "\n".join(res)


# --------------------------------------------------------------------------------------------- wrist planetary (J5, J6)
# D30: two identical stages, sun 14 / planet 19 / ring 52, module 1.0, 3 planets: (1 + 52/14)^2 = 22.22:1.
# Stage 2 has the 18 mm face. Planets on 623 bearings and dia 3 pins (stage 1: 3 x 16, stage 2: 3 x 24).
PG = dict(zs=14, zp=19, zr=52, n=3, m=1.0, backlash=0.15, motor="N17_40", od=72,
          sun1=(6, 18), car1=(19, 23), sun2=(23, 42), pl2=(24, 42), car2=(42.5, 47.5), boss=(47.5, 48.5),
          b_lo=48.5, land=6, flange_t=6, flange_d=64)


def pg():
    s = dict(PG)
    s["a"] = s["m"] * (s["zs"] + s["zp"]) / 2
    s["b_up"] = s["b_lo"] + 7 + s["land"]
    s["top"] = s["b_up"] + 7.5
    s["flange0"] = s["b_up"] + 8.5
    s["flange1"] = s["flange0"] + s["flange_t"]
    s["pins"] = circle_pts(s["a"], s["n"])
    return s


def gear_pts(z, ring=False):
    import sys, os
    sys.path.insert(0, os.path.dirname(__file__))
    import importlib, gears
    importlib.reload(gears)
    s = pg()
    f = gears.ring_space_outline if ring else gears.gear_outline
    return f(z, s["m"], backlash=s["backlash"] / 2, flank_points=6, arc_points=2)


def b_pg_base(p, _):
    s, mo = pg(), MOTOR[PG["motor"]]
    b = p.cyl("z", 0, 5, s["od"], body="Base")
    p.cyl("z", 0, 2.2, mo["pilot"][0] + F["clear_fit"], op=CUT, into=b, name="Pilot")
    p.cyl("z", 2.2, 5, 13, op=CUT, into=b, name="Shaft passage")
    sq = [(x, y) for x in (-15.5, 15.5) for y in (-15.5, 15.5)]
    p.holes("z", 0, 5, sq, 3.4, into=b, name="M3 motor screws")
    p.holes("z", 2, 5, sq, 6.5, into=b, name="M3 counterbores (heads inside)")
    p.holes("z", 0, 5, circle_pts(31.8, 4), 3.4, into=b, name="M3 to housing")


def b_pg_housing(p, _):
    s = pg()
    b = p.cyl("z", 5, s["top"], s["od"], body="Housing")
    p.ring("z", 58, 64, 88, s["od"] - 1, op=JOIN, into=b, name="Mount flange")
    p.holes("z", 58, 64, circle_pts(40, 6, 30), 3.4, into=b, name="Mount holes")
    rs = gear_pts(s["zr"], ring=True)
    ph = math.pi * (s["zp"] - 1) / s["zr"]                                 # gears.ring_phase
    rs = [(x * math.cos(ph) - y * math.sin(ph), x * math.sin(ph) + y * math.cos(ph)) for x, y in rs]
    p.polyline("z", 5, s["car2"][0] + 0.5, CUT, rs, into=b, name="Ring gear 52T (both stages)")
    p.cyl("z", s["car2"][0] + 0.5, s["b_lo"], s["zr"] * s["m"] - 2 + 0.5, op=CUT, into=b, name="Carrier 2 cavity")
    p.cyl("z", s["b_lo"], s["b_lo"] + 7, 42 + F["seat_fit"], op=CUT, into=b, name="Lower 6806 seat")
    p.cyl("z", s["b_lo"] + 7, s["b_up"], 39, op=CUT, into=b, name="Land")
    p.cyl("z", s["b_up"], s["top"], 42 + F["seat_fit"], op=CUT, into=b, name="Upper 6806 seat")
    p.holes("z", 5, 13, circle_pts(31.8, 4), 4.2, into=b, name="M3 inserts (base)")
    hall_sensor(p, b, 28, 30, s["top"], 2.5, s["od"] / 2 + 1)


def b_pg_sun1(p, _):
    s = pg()
    b = p.polyline("z", *s["sun1"], NEW, gear_pts(s["zs"]), body="Sun 1 (14T)")
    p.cyl("z", *s["sun1"], 5 + F["clear_fit"], op=CUT, into=b, name="Shaft bore")
    p.ext("z", 10.5, s["sun1"][1], JOIN, polys=[rect(-2, 2, 2.05, 3)], into=b, name="D key (LDO flat from z 8-10)")


def b_pg_planet(p, stage):
    s = pg()
    z0, z1 = (s["sun1"] if stage == 1 else s["pl2"])
    b = p.polyline("z", z0, z1, NEW, gear_pts(s["zp"]), body=f"Planet {stage} (19T)")
    p.cyl("z", z0, z1, 8, op=CUT, into=b, name="Bore")
    p.cyl("z", z0, z0 + 4, 10 + F["seat_fit"], op=CUT, into=b, name="623 seat")
    p.cyl("z", z1 - 4, z1, 10 + F["seat_fit"], op=CUT, into=b, name="623 seat")


def b_pg_carrier1(p, _):
    s = pg()
    b = p.cyl("z", *s["car1"], 48, body="Carrier 1 + sun 2")
    p.polyline("z", *s["sun2"], JOIN, gear_pts(s["zs"]), into=b, name="Sun 2 (14T)")
    p.holes("z", *s["car1"], s["pins"], 3 + F["press_fit"], into=b, name="Stage-1 pins (3 x 16)")
    p.cyl("z", s["car1"][0], s["car1"][0] + 7, 6.5, op=CUT, into=b, name="Motor shaft tip clearance")


def b_pg_carrier2(p, _):
    s = pg()
    b = p.cyl("z", *s["car2"], 48, body="Carrier 2 (output)")
    p.cyl("z", *s["boss"], 33, op=JOIN, into=b, name="Inner-ring boss")
    p.cyl("z", s["boss"][1], s["b_up"] + 7 - 0.2, 30 + F["journal_fit"], op=JOIN, into=b, name="6806 journal (0.2 short)")
    p.holes("z", *s["car2"], s["pins"], 3 + F["press_fit"], into=b, name="Stage-2 pins (3 x 24)")
    p.cyl("z", s["car2"][0], s["b_up"] + 7, 10, op=CUT, into=b, name="Bore")
    p.holes("z", s["b_up"] + 7 - 8.2, s["b_up"] + 6.8, circle_pts(10.5, 3, 90), 4.2, into=b, name="M3 inserts (flange)")


def b_pg_flange(p, _):
    s = pg()
    b = p.cyl("z", s["b_up"] + 7, s["flange0"], 33, body="Output flange")
    p.cyl("z", s["flange0"], s["flange1"], s["flange_d"], op=JOIN, into=b, name="Plate")
    p.cyl("z", s["b_up"] + 7, s["flange1"], 10, op=CUT, into=b, name="Bore")
    p.holes("z", s["b_up"] + 7, s["flange1"], circle_pts(10.5, 3, 90), 3.4, into=b, name="M3 to carrier stem")
    p.holes("z", s["flange1"] - 3, s["flange1"], circle_pts(10.5, 3, 90), 6.5, into=b, name="Counterbores")
    p.holes("z", s["flange1"] - 5, s["flange1"], circle_pts(27, 6, 0), 4.2, into=b, name="M3 inserts (link)")
    hall_magnet(p, b, 28, 30, s["flange0"])


def b_pg_spacer(p, _):
    s = pg()
    p.ring("z", s["b_lo"] + 7, s["b_up"], 33, 30.1, body="Inner spacer")


PG_BUILDERS = {"base": b_pg_base, "housing": b_pg_housing, "sun1": b_pg_sun1, "planet1": lambda p, m: b_pg_planet(p, 1),
               "carrier1": b_pg_carrier1, "planet2": lambda p, m: b_pg_planet(p, 2), "carrier2": b_pg_carrier2,
               "flange": b_pg_flange, "spacer": b_pg_spacer}


def _mates(occ, table):
    mm, out = L.app().measureManager, []

    def bodies(key):
        o, _, b = key.partition(":")
        return [x for x in occ[o].bRepBodies if not b or x.name == b]
    for a, b, want in table:
        r = min((mm.measureMinimumDistance(x, y) for x in bodies(a) for y in bodies(b)), key=lambda q: q.value)
        got = r.value * 10
        ok = got > 0.02 if want == ">" else abs(got - want) < 0.03
        at = "" if ok else " at (" + ", ".join(f"{v * 10:.1f}" for v in r.positionOne.asArray()) + ")"
        out.append(f"{'ok   ' if ok else 'CHECK'} {a} / {b}: {got:.3f} (want {want}){at}")
    return out


def _scratch():
    import adsk.core
    doc = L.scratch_doc()
    d = L.design_of(doc)
    if d.rootComponent.occurrences.count or d.rootComponent.bRepBodies.count:
        doc.close(False)
        raise RuntimeError("template is not empty")
    params(d)
    occ = {}

    def comp(name, fn):
        o = d.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        o.component.name = name
        fn(L.Part(o.component, d))
        occ[name] = o
    return doc, d, occ, comp


def check_pg(angles=(0,), mates=False):
    s, mo = pg(), MOTOR[PG["motor"]]
    doc, d, occ, comp = _scratch()
    res = []
    try:
        for part, fn in PG_BUILDERS.items():
            if part.startswith("planet"):
                for k in range(s["n"]):
                    comp(f"{part}_{k}", lambda p, fn=fn: fn(p, "P56"))
            else:
                comp(part, lambda p, fn=fn: fn(p, "P56"))
        L.soften(occ["housing"].component, SOFT["DA_P56_Housing"])
        b6 = BRG["6806"]
        for nm, z in (("lo", s["b_lo"]), ("up", s["b_up"])):
            comp(f"6806_{nm}_in", lambda p, z=z: _env(p, "r", [(z, z + 7, b6[3], b6[0], (0, 0))]))
            comp(f"6806_{nm}_out", lambda p, z=z: _env(p, "r", [(z, z + 7, b6[1], b6[4], (0, 0))]))

        def motor(p):
            h = mo["sq"] / 2
            p.box(-h, h, -h, h, -mo["L"] - mo["drv"], 0, body="motor+driver")
            p.cyl("z", 0, mo["pilot"][1], mo["pilot"][0], body="pilot")
            p.cyl("z", mo["pilot"][1], mo["shaft"][1], mo["shaft"][0], body="shaft")
            p.ext("z", 9, mo["shaft"][1], CUT, polys=[rect(-3, 3, 2.0, 3)], name="flat")
        comp("motor", motor)
        zs, zp, zr = s["zs"], s["zp"], s["zr"]
        for t in angles:
            c1 = t * zs / (zs + zr)
            c2 = c1 * zs / (zs + zr)
            occ["motor"].transform = L.matrix(L.rot("z", t))
            occ["sun1"].transform = L.matrix(L.rot("z", t))
            occ["carrier1"].transform = L.matrix(L.rot("z", c1))
            occ["carrier2"].transform = L.matrix(L.rot("z", c2))
            occ["flange"].transform = L.matrix(L.rot("z", c2))
            for k in range(s["n"]):
                phi = 360 * k / s["n"]
                ph = math.degrees(math.radians(phi) + math.pi + math.radians(phi) * zs / zp - math.pi / zp)
                for part, c, sun in (("planet1", c1, t), ("planet2", c2, c1)):
                    spin = ph + c - (sun - c) * zs / zp
                    x, y = polar(s["a"], phi + c)
                    occ[f"{part}_{k}"].transform = L.matrix(L.rot("z", spin), (x, y, 0))
            hits = L.interference(d)
            res.append(f"motor {t} deg: " + (", ".join(f"{k} {v} mm3" for k, v in hits.items()) or "clear"))
        if mates:
            # planet 2 over carrier 1's plate: carrier 1 and sun 2 are one body, so a measured minimum finds the
            # sun 2 mesh instead (0.04 mm, a false CHECK until 21 Oct); the axial gap is checked from the numbers
            g = pg()["pl2"][0] - pg()["car1"][1]
            res.append(f"{'ok   ' if abs(g - 1.0) < 0.03 else 'CHECK'} planet2 / carrier1 plate (axial, from the numbers): {g:.3f} (want 1.0)")
            res += _mates(occ, [
                ("carrier2", "6806_lo_in", 0), ("carrier2", "6806_lo_out", ">"), ("flange", "6806_up_in", 0),
                ("flange", "6806_up_out", ">"), ("flange", "carrier2", 0.2), ("spacer", "6806_lo_in", 0),
                ("spacer", "6806_up_in", 0), ("spacer", "6806_up_out", ">"), ("housing", "6806_lo_out", 0),
                ("housing", "6806_up_out", 0), ("housing", "6806_up_in", ">"), ("housing", "flange", ">"),
                ("base", "housing", 0), ("base", "motor", 0), ("sun1", "motor", 0.05), ("sun1", "base", 1.0),
                ("planet1_0", "base", 1.0), ("planet1_0", "carrier1", 1.0),
                ("planet2_0", "carrier2", 0.5), ("sun1", "carrier1", 1.0), ("sun1", "planet1_0", ">"),
                ("planet1_0", "housing", ">"), ("carrier1", "planet2_0", ">"), ("planet2_0", "housing", ">"),
                ("carrier1", "housing", ">"), ("carrier2", "housing", ">"), ("motor", "carrier1", ">")])
    finally:
        doc.close(False)
    return "\n".join(res)


def asm():
    import importlib, fusion_arm_asm, fusion_arm_j7
    importlib.reload(fusion_arm_asm)
    importlib.reload(fusion_arm_j7)
    return fusion_arm_asm, fusion_arm_j7


def build_bought(name, save=False):
    A, J = asm()
    fn = A.bought_parts(MOTOR, CYC, J.J7, enc, cyc)[name]
    reg = L.registry()
    doc = L.doc_for(name)
    out = [name]
    try:
        d = L.design_of(doc)
        L.clear(d)
        fn(L.Part(d.rootComponent, d))
        out += L.report(d.rootComponent)
        if save:
            out.append(L.save(doc, name, "bought-part envelope (tools/fusion_arm_asm.py)"))
    finally:
        doc.close(False)
    return "\n".join(out)


def build_sa(m, save=False):
    import sys, os
    sys.path.insert(0, os.path.dirname(__file__))
    import gears
    A, J = asm()
    return A.build_module_sa(m, A.module_layout(m, cyc, pg, J.j7, gears.planet_phases), save)

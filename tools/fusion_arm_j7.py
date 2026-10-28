"""J7 tool roll module (imported by tools/fusion_arm.py).

15:1 = GT2 5:1 (16T on the pancake, 80T printed) x planetary 3:1 (sun 24 / planet 12 / ring 48, module 1.0,
3 planets), chosen so the sun can be hollow (dia 15 bore: tool cables now, a slip ring later). Belt GT2 220 mm,
6 mm wide: centre distance 58.45 mm. Flange: ISO 9409-1-31.5-4-M5. Local frame: J7 axis +z, flange face at z 51.
"""
import math

import fusion_lib as L
from fusion_lib import CUT, JOIN, NEW, polar, circle_pts, rect

J7 = dict(zs=24, zp=12, zr=48, n=3, m=1.0, motor="N17_20", od=66, C=58.45, gear=(8, 18),
          car=(18.5, 22.5), b_lo=23.5, land=4, flange_d=60, flange_t=8)   # 60: room for the homing magnet


def j7():
    s = dict(J7)
    s["a"] = s["m"] * (s["zs"] + s["zp"]) / 2
    s["b_up"] = s["b_lo"] + 7 + s["land"]
    s["top"] = s["b_up"] + 7.5
    s["flange0"] = s["b_up"] + 8.5
    s["flange1"] = s["flange0"] + s["flange_t"]
    s["pins"] = circle_pts(s["a"], s["n"])
    return s


def gear(z, ring=False):
    import sys, os, importlib
    sys.path.insert(0, os.path.dirname(__file__))
    import gears
    importlib.reload(gears)
    f = gears.ring_space_outline if ring else gears.gear_outline
    return f(z, 1.0, backlash=0.075, flank_points=6, arc_points=2)


def builders(F, MOTOR):
    def base(p, _):
        s, mo = j7(), MOTOR[J7["motor"]]
        h = mo["sq"] / 2 + 4
        b = p.cyl("z", 0, 8, s["od"], body="Base")
        p.ext("z", 0, 8, JOIN, polys=[rect(-h, h, 0, s["C"] + h)], into=b, name="Motor lobe")
        p.cyl("z", 0.5, 7.5, 32 + F["seat_fit"], op=CUT, into=b, name="6804 seat (open below)")
        p.cyl("z", 0, 0.5, 30, op=CUT, into=b, name="Below seat")
        p.cyl("z", 7.5, 8, 27, op=CUT, into=b, name="Above seat")
        p.cyl("z", 6, 8, mo["pilot"][0] + F["clear_fit"], at=(0, s["C"]), op=CUT, into=b, name="Pilot (motor on top)")
        p.cyl("z", 0, 6, 13, at=(0, s["C"]), op=CUT, into=b, name="Shaft passage")
        sq = [(x, s["C"] + y) for x in (-15.5, 15.5) for y in (-15.5, 15.5)]
        p.holes("z", 0, 8, sq, 3.4, into=b, name="M3 motor screws (heads below)")
        p.holes("z", 0, 8, circle_pts(29.2, 4, 45), 3.4, into=b, name="M3 to housing")

    def housing(p, _):
        s = j7()
        b = p.cyl("z", 8, s["top"], s["od"], body="Housing")
        for a in (0, 180, 270):                                # mounting tabs, away from the motor at +y
            p.ext("z", 26, 32, JOIN, polys=[L.rotated_rect(*polar(38, a), 14, 12, a)], into=b, name=f"Tab {a}")
            p.holes("z", 26, 32, [polar(40, a)], 3.4, into=b, name=f"Tab hole {a}")
        rs = gear(s["zr"], ring=True)
        ph = math.pi * (s["zp"] - 1) / s["zr"]
        rs = [(x * math.cos(ph) - y * math.sin(ph), x * math.sin(ph) + y * math.cos(ph)) for x, y in rs]
        p.polyline("z", 8, 19, CUT, rs, into=b, name="Ring gear 48T")
        p.cyl("z", 19, s["b_lo"], 46, op=CUT, into=b, name="Carrier cavity")
        p.cyl("z", s["b_lo"], s["b_lo"] + 7, 42 + F["seat_fit"], op=CUT, into=b, name="Lower 6806 seat")
        p.cyl("z", s["b_lo"] + 7, s["b_up"], 39, op=CUT, into=b, name="Land")
        p.cyl("z", s["b_up"], s["top"], 42 + F["seat_fit"], op=CUT, into=b, name="Upper 6806 seat")
        L.hall_sensor(p, b, 26, 90, s["top"], 2.5, s["od"] / 2 + 1)
        p.holes("z", 8, 16, circle_pts(29.2, 4, 45), 4.2, into=b, name="M3 inserts (base)")

    def sunshaft(p, _):
        s = j7()
        b = p.polyline("z", *s["gear"], NEW, gear(s["zs"]), body="Sun shaft (24T)")
        p.cyl("z", 7.5, 8, 22, op=JOIN, into=b, name="Shoulder")
        p.cyl("z", 0.5, 7.5, 20 + F["journal_fit"], op=JOIN, into=b, name="6804 journal")
        p.cyl("z", -10, 0.5, 18, op=JOIN, into=b, name="Pulley journal")
        p.ext("z", -10, -1.5, CUT, polys=[rect(-6, 6, 8.5, 10)], into=b, name="Pulley flat")
        p.cyl("z", -10, s["gear"][1], 15, op=CUT, into=b, name="Bore (cables / slip ring)")

    def pulley80(p, _):
        pd = 80 * 2 / math.pi                                   # GT2 80T pitch 50.93
        b = p.cyl("z", -8.5, -2.5, pd - 0.508, body="Pulley 80T")
        p.cyl("z", -10, -8.5, pd + 3, op=JOIN, into=b, name="Flange")
        p.cyl("z", -2.5, -1, pd + 3, op=JOIN, into=b, name="Flange")
        p.cyl("z", -1, 0.5, 22, op=JOIN, into=b, name="Hub (to the 6804 inner ring)")
        p.ext("z", -8.5, -2.5, CUT, [(*polar(pd / 2 - 0.354, 4.5 * k), 1.2) for k in range(80)], into=b,
              name="GT2 grooves")
        p.cyl("z", -10, 0.5, 18.1, op=CUT, into=b, name="Bore")
        p.ext("z", -10, -2, JOIN, polys=[rect(-3, 3, 8.55, 9.5)], into=b, name="Flat key")

    def planet(p, _):
        s = j7()
        z0, z1 = s["gear"][0] + 0.5, s["gear"][1]               # 0.5 clear of the base
        b = p.polyline("z", z0, z1, NEW, gear(s["zp"]), body="Planet (12T)")
        p.cyl("z", z0, z1, 3 + 0.15, op=CUT, into=b, name="Bore (runs on the dia 3 pin)")

    def carrier(p, _):
        s = j7()
        b = p.cyl("z", *s["car"], 44, body="Carrier")
        p.cyl("z", s["car"][1], s["b_lo"], 33, op=JOIN, into=b, name="Inner-ring boss")
        p.cyl("z", s["b_lo"], s["b_up"] + 7 - 0.2, 30 + F["journal_fit"], op=JOIN, into=b,
              name="6806 journal (0.2 short)")
        p.holes("z", *s["car"], s["pins"], 3 + F["press_fit"], into=b, name="Pins (3 x 12)")
        p.cyl("z", s["car"][0], s["b_up"] + 7, 15, op=CUT, into=b, name="Bore")
        p.holes("z", s["b_up"] - 1.2, s["b_up"] + 6.8, circle_pts(11, 3, 60), 4.2, into=b, name="M3 inserts (flange)")

    def flange(p, _):
        s = j7()
        b = p.cyl("z", s["b_up"] + 7, s["flange0"], 33, body="Tool flange")
        p.cyl("z", s["flange0"], s["flange1"], s["flange_d"], op=JOIN, into=b, name="Plate")
        p.cyl("z", s["b_up"] + 7, s["flange1"], 15, op=CUT, into=b, name="Bore")
        p.cyl("z", s["flange1"] - 3, s["flange1"], 20.1, op=CUT, into=b, name="ISO 9409 centring recess 20")
        p.holes("z", s["b_up"] + 7, s["flange1"], circle_pts(11, 3, 60), 3.4, into=b, name="M3 to carrier")
        p.holes("z", s["flange1"] - 6, s["flange1"], circle_pts(11, 3, 60), 6.5, into=b, name="Counterbores")
        p.holes("z", s["flange1"] - 7, s["flange1"], circle_pts(31.5 / 2, 4, 45), 6.4, into=b,
                name="ISO 9409-31.5 M5 inserts")
        p.holes("z", s["flange1"] - 5, s["flange1"], [(31.5 / 2, 0)], 5 + F["pin_fit"], into=b, name="Dowel 5")
        L.hall_magnet(p, b, 26, 90, s["flange0"])

    def spacer(p, _):
        s = j7()
        p.ring("z", s["b_lo"] + 7, s["b_up"], 33, 30.1, body="Inner spacer")

    def sliptube(p, _):
        """Turns the slip ring's rotor with the output: pressed on the rotor's nose, up the hollow sun and carrier,
        glued in the flange bore under the receiver. The ring's 8 rotor wires run up inside it to the pogo pins."""
        p.ring("z", *SLIP["nose"], 9.4, 7, body="Slip ring tube")
        p.ring("z", SLIP["nose"][1], 42, 12, 8, op=JOIN, into=p.bodies["Slip ring tube"], name="Tube")
        p.ring("z", 42, 47.5, 14.8, 8, op=JOIN, into=p.bodies["Slip ring tube"], name="Collar (glued in the flange bore)")

    return {"base": base, "housing": housing, "sunshaft": sunshaft, "pulley80": pulley80, "planet": planet,
            "carrier": carrier, "flange": flange, "spacer": spacer, "sliptube": sliptube}


# J7 slip ring (docs/06 §1): an 8-wire 12.5 mm capsule ring under the module, body held in link 6's arm (fixed), rotor
# nose up into the sliptube (turns with the output). Local z (mm): body, rotor nose (the tube's sleeve).
SLIP = dict(body=(-38, -16), nose=(-16, -11), d=12.5)


def slipring(p):
    p.cyl("z", *SLIP["body"], SLIP["d"], body="Slip ring body (stator)")
    p.cyl("z", SLIP["nose"][0], SLIP["nose"][0] + 4, 7, body="Slip ring rotor nose")
    zg = (SLIP["body"][0] + SLIP["body"][1]) / 2                 # link 6's grub insert (world +y = local -x)
    p.cyl("x", -10, -SLIP["d"] / 2 - 0.05, 3, at=(0, zg), body="M3 x 4 grub screw (steel)")


def check(scratch, env, mates_fn, BRG, MOTOR, F, angles=(0,), mates=False):
    s, mo = j7(), MOTOR[J7["motor"]]
    B = builders(F, MOTOR)
    doc, d, occ, comp = scratch()
    res = []
    try:
        for part, fn in B.items():
            if part == "planet":
                for k in range(s["n"]):
                    comp(f"planet_{k}", lambda p, fn=fn: fn(p, "P7"))
            else:
                comp(part, lambda p, fn=fn: fn(p, "P7"))
        L.soften(occ["housing"].component, (0, 0, 1))                     # as fusion_arm.SOFT["DA_P7_Housing"]
        b6, b4 = BRG["6806"], BRG["6804"]
        for nm, z in (("lo", s["b_lo"]), ("up", s["b_up"])):
            comp(f"6806_{nm}_in", lambda p, z=z: env(p, "r", [(z, z + 7, b6[3], b6[0], (0, 0))]))
            comp(f"6806_{nm}_out", lambda p, z=z: env(p, "r", [(z, z + 7, b6[1], b6[4], (0, 0))]))
        comp("6804_in", lambda p: env(p, "r", [(0.5, 7.5, b4[3], b4[0], (0, 0))]))
        comp("6804_out", lambda p: env(p, "r", [(0.5, 7.5, b4[1], b4[4], (0, 0))]))
        comp("pins", lambda p: env(p, "p", [(8.5, 20.5, 3, 0, xy) for xy in s["pins"]]))

        def motor(p):
            h = mo["sq"] / 2
            p.box(-h, h, s["C"] - h, s["C"] + h, 8, 8 + mo["L"] + mo["drv"], body="pancake+driver")
            p.cyl("z", 6, 8, mo["pilot"][0], at=(0, s["C"]), body="pilot")
            p.cyl("z", 8 - mo["shaft"][1], 6, mo["shaft"][0], at=(0, s["C"]), body="shaft")
            p.ring("z", -10, -1, 16 * 2 / math.pi + 3, mo["shaft"][0], at=(0, s["C"]), body="16T pulley")
        comp("motor", motor)
        comp("slipring", slipring)
        zs, zp, zr = s["zs"], s["zp"], s["zr"]
        for t in angles:
            sun = t * 16 / 80
            c = sun * zs / (zs + zr)
            for nm in ("sunshaft", "pulley80"):
                occ[nm].transform = L.matrix(L.rot("z", sun))
            for nm in ("carrier", "flange", "pins", "sliptube"):
                occ[nm].transform = L.matrix(L.rot("z", c))
            for k in range(s["n"]):
                phi = 360 * k / s["n"]
                ph = math.degrees(math.radians(phi) + math.pi + math.radians(phi) * zs / zp - math.pi / zp)
                x, y = polar(s["a"], phi + c)
                occ[f"planet_{k}"].transform = L.matrix(L.rot("z", ph + c - (sun - c) * zs / zp), (x, y, 0))
            hits = L.interference(d)
            res.append(f"motor {t} deg: " + (", ".join(f"{k} {v} mm3" for k, v in hits.items()) or "clear"))
        if mates:
            res += mates_fn(occ, [
                ("carrier", "6806_lo_in", 0), ("carrier", "6806_lo_out", ">"), ("flange", "6806_up_in", 0),
                ("flange", "6806_up_out", ">"), ("flange", "carrier", 0.2), ("spacer", "6806_lo_in", 0),
                ("spacer", "6806_up_in", 0), ("housing", "6806_lo_out", 0), ("housing", "6806_up_out", 0),
                ("housing", "6806_up_in", ">"), ("base", "housing", 0), ("base", "6804_out", 0),
                ("sunshaft", "6804_in", 0), ("pulley80", "6804_in", 0), ("pulley80", "sunshaft", 0),
                ("pulley80", "base", ">"), ("motor", "base", 0), ("motor", "pulley80", ">"), ("motor", "housing", ">"),
                ("sunshaft", "planet_0", ">"), ("planet_0", "housing", ">"), ("planet_0", "carrier", 0.5),
                ("planet_0", "base", 0.5), ("pins", "carrier", 0), ("pins", "planet_0", 0.075),
                ("sunshaft", "carrier", ">"), ("sunshaft", "housing", ">"), ("sliptube", "sunshaft", ">"),
                ("sliptube", "pulley80", ">"), ("sliptube", "carrier", ">"), ("sliptube", "slipring", 0),
                ("slipring", "pulley80", ">")])
    finally:
        doc.close(False)
    return "\n".join(res)

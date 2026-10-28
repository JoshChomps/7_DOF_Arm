"""Links, stands, counterbalance and tool changer, drawn in world coordinates at the zero pose (docs/20).

Every bolt position is computed from where the module it meets puts its flange inserts or housing tabs
(to_world), so the holes line up by construction; tools/fusion_arm.py checks it in the full assembly.
"""
import math

import fusion_lib as L
from fusion_lib import CUT, JOIN, NEW, polar, circle_pts, rect
from fusion_arm_asm import MODULES, J2Z, J4Z, J6Z
from fusion_arm_j7 import SLIP

CB = dict(a=70.0, b=98.0, plane_x=70.0)      # counterbalance: anchor above J2, lug along link 2, cable plane


def to_world(joint, local):
    _, R, t = MODULES[joint]
    v = L.apply(R, local)
    return tuple(v[i] + t[i] for i in range(3))


def tie(p, b, axis, a0, a1, u, v, name="Harness tie"):
    """A zip-tie anchor (docs/21 §13): two 2 x 4.5 slots 6 mm apart through a plate, at (u, v) in its plane."""
    p.ext(axis, a0, a1, CUT, polys=[rect(u + d - 1, u + d + 1, v - 2.25, v + 2.25) for d in (-3, 3)], into=b, name=name)


def stand_left(p, cyc):
    """Carries the J2 module: a collar plate on the +x face of the housing tabs, and a foot on the turntable's
    cheek holes (x = -47, as the turntable has them). The NEMA23 passes through a window in the collar."""
    s = cyc("C2")
    x0 = to_world("J2", (0, 0, 36))[0]                          # +x face of the tabs: -93
    x1 = x0 + 8
    # inside R 82 of J2 above the axis, a taper below: link 3's U-collar sweeps R >= 108 round J2
    b = p.ext("x", x0, x1, NEW, polys=[L.hull(circle_pts(82, 72, c=(0, J2Z)) + [(-48, 74), (48, 74)])], body="Left stand")
    p.ext("x", x0, x1, CUT, [(0, J2Z, s["od"] + 1)], into=b, name="Housing passage")
    p.ext("x", x0, x1, CUT, polys=[rect(-31, 31, 84, 143.5)], into=b, name="Motor window")
    holes = [to_world("J2", (*polar(s["tab_r"][1] - 5, a), 0))[1:] for a in (0, 180, 270)]
    p.ext("x", x0, x1, CUT, [(y, z, 4.5) for y, z in holes], into=b, name="Tab screws (M4, nuts on the +x side)")
    tie(p, b, "x", x0, x1, -45, 100, "Harness tie (from the turntable)")
    tie(p, b, "x", x0, x1, -35, 280, "Harness tie (J2 loop, fixed end)")
    p.ext("z", 74, 84, JOIN, polys=[rect(x0, -26, -45, 45)], into=b, name="Foot")       # ends clear of the J3 motor
    feet = [(-47, y) for y in (-30, -10, 10, 30)]
    p.ext("z", 74, 83, CUT, [(x, y, 5.8) for x, y in feet], into=b, name="M4 inserts (screws from under the turntable)")
    p.ext("z", 74, 83, CUT, [(-47, y, 5.05) for y in (-20, 20)], into=b, name="Dowels 5")
    for y0, y1 in ((32, 40), (-40, -32)):
        p.ext("y", y0, y1, JOIN, polys=[[(x1, 84), (-55, 84), (x1, 140)]], into=b, name="Gusset")


def tower_right(p):
    """Counterbalance tower: foot on the turntable's +x cheek holes, a fork holding the pulley axle (M8, along X)
    CB a above J2 in the cable plane x = CB plane_x, and the spring anchor low on the web."""
    xc, za = CB["plane_x"], J2Z + CB["a"]
    b = p.ext("z", 74, 84, NEW, polys=[rect(36, xc + 10, -40, 40)], body="Right tower")
    for x0, x1 in ((xc - 7.5, xc - 4.5), (xc + 4.5, xc + 7.5)):
        p.ext("x", x0, x1, JOIN, polys=[[(-12, 84), (12, 84), (10, za + 13), (-10, za + 13)]], into=b,
              name="Fork cheek")
    p.ext("x", xc - 4.5, xc + 4.5, JOIN, polys=[rect(-12, 12, 84, za - 40)], into=b, name="Fork web")
    p.ext("x", xc - 8, xc + 8, CUT, [(0, za, 8.4)], into=b, name="M8 axle")
    p.ext("z", 74, 83, CUT, [(47, y, 5.8) for y in (-30, -10, 10, 30)], into=b, name="M4 inserts")
    p.ext("z", 74, 83, CUT, [(47, y, 5.05) for y in (-20, 20)], into=b, name="Dowels 5")
    p.ext("x", xc - 4.5, xc + 4.5, CUT, [(0, 100, 6)], into=b, name="Spring anchor")


def cb_pulley(p):
    xc, za = CB["plane_x"], J2Z + CB["a"]
    b = p.ext("x", xc - 4, xc + 4, NEW, [(0, za, 28)], body="Counterbalance pulley")
    p.ext("x", xc - 3.5, xc + 3.5, CUT, [(0, za, 22.05)], into=b, name="608 seat")


def link2(p, cyc):
    """J2 output -> J3 module: a side plate on the J2 flange, a collar plate under J3's tabs, the counterbalance lug."""
    s2, s3 = cyc("C2"), cyc("C34")
    xf = to_world("J2", (0, 0, s2["flange1"]))[0]               # J2 flange face: -52
    zc = to_world("J3", (0, 0, 26))[2]                           # underside of J3's tabs: 299
    b = p.ext("x", xf, xf + 8, NEW, [(0, J2Z, 116)], body="Link 2")
    p.ext("x", xf, xf + 8, JOIN, polys=[rect(-60, 60, J2Z, zc)], into=b, name="Side plate")      # +-60: docs/21 §6
    fi = [to_world("J2", (*polar((s2["od"] - 4) / 2 - 6, 60 * k), 0))[1:] for k in range(6)]
    p.ext("x", xf, xf + 8, CUT, [(y, z, 4.5) for y, z in fi], into=b, name="M4 to J2 flange")
    p.ext("x", xf + 4, xf + 8, CUT, [(y, z, 8) for y, z in fi], into=b, name="Counterbores")
    p.ext("x", xf, xf + 5, CUT, [(0, J2Z, 6)], into=b, name="J2 encoder rod (press, D42)")
    tie(p, b, "x", xf, xf + 8, -30, 266, "Harness tie (J2 loop, moving end)")
    p.ext("z", zc - 10, zc, JOIN, polys=[rect(xf, 46, -60, 60)], into=b, name="Collar plate")
    p.ext("z", zc - 10, zc, JOIN, polys=[rect(46, 61.5, -12, 12)], into=b, name="Tongue under the +x tab")
    p.ext("z", zc - 10, zc, CUT, [(0, 0, s3["od"] + 1)], into=b, name="J3 housing passage")
    p.ext("z", zc - 10, zc, CUT, polys=[rect(-23, 23, -95, -46.5)], into=b, name="J3 motor notch")
    th = [to_world("J3", (*polar(s3["tab_r"][1] - 5, a), 0))[:2] for a in (0, 180, 270)]
    p.ext("z", zc - 8, zc, CUT, [(x, y, 5.6) for x, y in th], into=b, name="M4 inserts (J3 tabs)")
    xc, zl = CB["plane_x"], J2Z + CB["b"]
    p.ext("z", zc, zl + 5, JOIN, polys=[rect(57, 61.5, -5, 5)], into=b, name="Lug riser")
    p.ext("z", zl - 5, zl + 5, JOIN, polys=[rect(57, xc + 5, -5, 5)], into=b, name="Lug bridge")
    p.ext("x", 57, xc + 5, CUT, [(0, zl, 5.2)], into=b, name="Cable clevis pin")


def link3(p, cyc):
    """J3 output -> J4 module: a plate on the J3 flange and a U-collar on the +x face of J4's tabs."""
    s = cyc("C34")
    zf = to_world("J3", (0, 0, s["flange1"]))[2]                 # 345.5
    tx = to_world("J4", (0, 0, 36))[0]                           # -88.5
    b = p.ext("z", zf, zf + 8, NEW, [(0, 0, s["od"] - 4)], body="Link 3")
    p.ext("z", zf, zf + 8, JOIN, polys=[rect(tx, 0, -30, 30)], into=b, name="Arm to the elbow")   # narrow: elbow fold
    p.ext("z", zf + 8, zf + 12, JOIN, polys=[rect(tx, -43, -30, 30)], into=b,
          name="Arm thickened to 12 outside the flange bolts (docs/21 §6)")
    fi = [to_world("J3", (*polar((s["od"] - 4) / 2 - 6, 60 * k), 0))[:2] for k in range(6)]
    p.ext("z", zf, zf + 8, CUT, [(x, y, 4.5) for x, y in fi], into=b, name="M4 to J3 flange")
    p.ext("z", zf + 4, zf + 8, CUT, [(x, y, 8) for x, y in fi], into=b, name="Counterbores")
    p.ext("z", zf, zf + 5, CUT, [(0, 0, 6)], into=b, name="J3 encoder rod (press, D42)")
    p.ext("x", tx, tx + 8, JOIN, polys=[rect(-62, 62, zf, J4Z + 16)], into=b, name="U-collar")
    p.ext("x", tx, tx + 8, CUT, [(0, J4Z, s["od"] + 1)], into=b, name="J4 housing passage")
    th = [to_world("J4", (*polar(s["tab_r"][1] - 5, a), 0))[1:] for a in (0, 180, 270)]
    p.ext("x", tx, tx + 8, CUT, [(y, z, 4.5) for y, z in th], into=b, name="Tab screws")
    tie(p, b, "x", tx, tx + 8, -40, 335, "Harness tie (J3 loop, moving end / J4 loop, fixed end)")


def forearm(p, cyc, pg):
    """J4 output -> J5 module: a side plate on the J4 flange and a collar plate under J5's mount flange."""
    s, g = cyc("C34"), pg()
    xf = to_world("J4", (0, 0, s["flange1"]))[0]                 # -52
    zc = to_world("J5", (0, 0, 58))[2]                           # 488
    b = p.ext("x", xf, xf + 10, NEW, [(0, J4Z, s["od"] - 4)], body="Forearm")
    p.ext("x", xf, xf + 10, JOIN, polys=[rect(-43, 43, J4Z, zc)], into=b, name="Side plate")
    fi = [to_world("J4", (*polar((s["od"] - 4) / 2 - 6, 60 * k), 0))[1:] for k in range(6)]
    p.ext("x", xf, xf + 10, CUT, [(y, z, 4.5) for y, z in fi], into=b, name="M4 to J4 flange")
    p.ext("x", xf + 5, xf + 10, CUT, [(y, z, 8) for y, z in fi], into=b, name="Counterbores")
    p.ext("x", xf, xf + 5, CUT, [(0, J4Z, 6)], into=b, name="J4 encoder rod (press, D42)")
    tie(p, b, "x", xf, xf + 10, -30, 440, "Harness tie (J4 loop, moving end)")
    p.ext("z", zc - 6, zc, JOIN, polys=[rect(xf, 46, -46, 46)], into=b, name="Collar plate")
    p.ext("z", zc - 6, zc, CUT, [(0, 0, g["od"] + 1)], into=b, name="J5 housing passage")
    p.ext("z", zc - 6, zc, CUT, [(*polar(40, 30 + 60 * k), 4.2) for k in range(6)], into=b, name="M3 inserts (J5)")
    for y0, y1 in ((38, 44), (-44, -38)):                       # outside the J5 housing (r 36)
        p.ext("y", y0, y1, JOIN, polys=[[(xf + 10, zc - 6), (xf + 10, zc - 50), (xf + 40, zc - 6)]], into=b, name="Gusset")


def link5(p, pg):
    """J5 output -> J6 module: a plate on the J5 flange, a ring riser inside R 40 (the J4 motor, fixed to link 3,
    sits at R >= 62.5 up to z 490; the ring clears the flange screws), then an arm and a collar plate on the -x
    face of J6's mount flange."""
    g = pg()
    zf = to_world("J5", (0, 0, g["flange1"]))[2]                 # 466
    xr = to_world("J6", (0, 0, 58))[0]                           # -60
    za = 494.0                                                   # arm above the J4 motor top (490)
    b = p.ext("z", zf, za + 8, NEW, [(0, 0, 80)], body="Link 5")
    p.ext("z", za, za + 8, JOIN, polys=[rect(xr - 6, 0, -32, 32)], into=b, name="Arm")
    p.ext("z", zf + 8, za + 8, CUT, [(0, 0, 64)], into=b, name="Inside the riser (screw access)")
    p.ext("z", zf, zf + 8, CUT, [(0, 0, 20)], into=b, name="Cable hole")
    p.ext("z", zf, zf + 8, CUT, [(*polar(27, 60 * k), 3.4) for k in range(6)], into=b, name="M3 to J5 flange")
    p.ext("z", zf + 5, zf + 8, CUT, [(*polar(27, 60 * k), 6.5) for k in range(6)], into=b, name="Counterbores")
    p.ext("x", xr - 6, xr, JOIN, polys=[rect(-46, 46, za, J6Z + 46)], into=b, name="Collar plate")
    p.ext("x", xr - 6, xr, CUT, [(0, J6Z, g["od"] + 1)], into=b, name="J6 housing passage")
    mh = [to_world("J6", (*polar(40, 30 + 60 * k), 0))[1:] for k in range(6)]
    p.ext("x", xr - 6, xr, CUT, [(y, z, 4.2) for y, z in mh], into=b, name="M3 inserts (J6)")
    tie(p, b, "x", xr - 6, xr, 0, 515, "Harness tie (J5 loop, moving end / J6 loop, fixed end)")


def link6(p, pg, j7):
    """J6 output -> J7 module: a side plate on the J6 flange (pocketed round J7's -x tab) and a bridge on the
    -y and +y tabs (it stops short of the J7 motor at +x)."""
    g, s7 = pg(), j7()
    xf = to_world("J6", (0, 0, g["flange1"]))[0]                 # -37
    z7 = MODULES["J7"][2][2]                                     # 598
    zt = z7 + 32                                                 # tab tops 630
    b = p.ext("x", xf, xf + 8, NEW, [(0, J6Z, g["flange_d"])], body="Link 6")
    p.ext("x", xf, xf + 8, JOIN, polys=[rect(-32, 32, J6Z, zt + 6)], into=b, name="Side plate")
    fi = [to_world("J6", (*polar(27, 60 * k), 0))[1:] for k in range(6)]
    p.ext("x", xf, xf + 8, CUT, [(y, z, 3.4) for y, z in fi], into=b, name="M3 to J6 flange")
    p.ext("x", xf + 5, xf + 8, CUT, [(y, z, 6.5) for y, z in fi], into=b, name="Counterbores")
    p.ext("z", zt, zt + 6, JOIN, polys=[rect(xf, 35, -46, 46)], into=b, name="Bridge")       # J7 motor from x 37
    tie(p, b, "x", xf, xf + 8, -22, 620, "Harness tie (J6 loop, moving end)")
    p.ext("z", zt, zt + 6, CUT, [(0, 0, s7["od"] + 1)], into=b, name="J7 housing passage")
    p.ext("z", z7 + 25.5, zt + 0.5, CUT, polys=[rect(-46, xf + 8.1, -6.5, 6.5)], into=b, name="Pocket for the -x tab")
    tabs = [to_world("J7", (*polar(40, a), 0))[:2] for a in (0, 180, 270)]
    p.ext("z", zt - 9, zt + 6, CUT, [(x, y, 4.2) for x, y in tabs], into=b,
          name="M3 inserts (J7 tabs)")
    # slip ring holder (J7's hollow axis, fusion_arm_j7.SLIP): an arm from the side plate under the 80T pulley; the
    # ring's body pushes up into the bore against the lip (its rotor nose and the tube pass the lip), an M3 grub
    # in an insert holds it, its stator wires leave from below
    zb0, zb1 = z7 + SLIP["body"][0], z7 + SLIP["body"][1]       # 560, 582
    p.ext("z", zb0 - 4, zb1 + 3, JOIN, polys=[rect(xf + 8, 8, -10, 10)], into=b, name="Slip ring arm")
    p.ext("z", zb0 - 4, zb1, CUT, [(0, 0, SLIP["d"] + 0.1)], into=b, name="Slip ring bore (in from below)")
    p.ext("z", zb1, zb1 + 3, CUT, [(0, 0, 10)], into=b, name="Lip (rotor nose and tube pass)")
    p.ext("y", 6, 10, CUT, [(0, (zb0 + zb1) / 2, 4.2)], into=b, name="M3 insert for the grub screw")


def receiver(p):
    """Tool changer, robot side: ISO 9409-1-31.5-4-M5 to the J7 flange, 3 balls + 3 magnets + pogo pins."""
    z0 = 649.0
    b = p.cyl("z", z0, z0 + 10, 50, body="Receiver")
    p.cyl("z", z0 - 3, z0, 20 - 0.1, op=JOIN, into=b, name="Centring spigot (flange recess 20)")
    p.cyl("z", z0 - 3, z0 + 10, 15, op=CUT, into=b, name="Cable bore")
    p.holes("z", z0, z0 + 10, circle_pts(15.75, 4, 45), 5.5, into=b, name="M5 to flange")
    p.holes("z", z0 + 5, z0 + 10, circle_pts(15.75, 4, 45), 9.5, into=b, name="Counterbores")
    p.holes("z", z0, z0 + 5, [to_world("J7", (15.75, 0, 0))[:2]], 5.05, into=b, name="Dowel 5")
    # a 6 mm ball touches a 4 mm groove's edges 2.24 mm above its centre, so seat it that far below the face: the
    # faces meet as the balls seat (a 3 mm seat left the tool plate floating 2.2 mm up)
    p.holes("z", z0 + 10 - 3 - 2.24, z0 + 10, circle_pts(20, 3, 90), 6.2, into=b, name="Ball seats (6 mm balls)")
    p.holes("z", z0 + 6.8, z0 + 10, circle_pts(20, 3, 30), 6.2, into=b, name="Magnet pockets 6x3")
    p.holes("z", z0, z0 + 10, [polar(10.5, a) for a in (-15, 0, 15, 165, 180, 195)], 1.6, into=b, name="Pogo pins")


def toolplate(p):
    z0 = 659.0
    b = p.cyl("z", z0, z0 + 8, 50, body="Tool plate")
    p.cyl("z", z0, z0 + 8, 15, op=CUT, into=b, name="Cable bore")
    for a in (90, 210, 330):                                     # slots for the balls (kinematic coupling)
        p.ext("z", z0, z0 + 2, CUT, polys=[L.rotated_rect(*polar(20, a), 10, 4, a)], into=b, name=f"Groove {a}")
    p.holes("z", z0, z0 + 3.2, circle_pts(20, 3, 30), 6.2, into=b, name="Magnet pockets 6x3")
    p.holes("z", z0 + 2, z0 + 8, circle_pts(20, 6, 0), 4.2, into=b,         # blind: the coupling face stays whole;
            name="M3 inserts for the gripper (PCD 40, 6 x 60 deg)")          # 30 deg off the grooves and magnets
    p.holes("z", z0, z0 + 1, [polar(10.5, a) for a in (-15, 0, 15, 165, 180, 195)], 2.5, into=b, name="Pogo pads")


def board(p):
    """Base board (D19): 18 mm plywood 500 x 500, top at z 0 (the base frame), a 64 mm hole for the J1 motor (D37),
    M6 from below into the base's inserts (R 71 at 45 + 90k), on four 70 mm feet."""
    b = p.ext("z", -18, 0, NEW, polys=[rect(-250, 250, -250, 250)], body="Base board (plywood)")
    p.ext("z", -18, 0, CUT, [(0, 0, 64)], into=b, name="J1 motor hole")
    p.ext("z", -18, 0, CUT, [(*polar(71, 45 + 90 * k), 6.5) for k in range(4)], into=b, name="M6 to the base")
    p.ext("z", -18, -12, CUT, [(*polar(71, 45 + 90 * k), 12) for k in range(4)], into=b, name="Counterbores")
    for sx in (-1, 1):
        for sy in (-1, 1):
            p.box(sx * 190 - 20, sx * 190 + 20, sy * 190 - 20, sy * 190 + 20, -88, -18, body="Foot")


def spaces(p):
    """Room kept free on the base board for the base electronics (docs/21 §14), drawn as boxes on its top (z 0) so
    the sweep checks them. CAD x, y = arm.toml (y, -x): the back strip is CAD y +130..+250 (KEEP_OUT, collision.py)."""
    for name, (x0, x1, y0, y1, h) in {
            "PSU (LRS-350-24)": (-245, -30, 133, 248, 30), "Board A (base controller)": (-20, 60, 188, 248, 20),
            "Board B (power)": (-20, 50, 150, 180, 15), "Raspberry Pi 4": (70, 156, 191, 248, 25),
            "CANable": (70, 130, 160, 180, 12), "E-stop box": (180, 250, -250, -180, 80)}.items():
        p.box(x0, x1, y0, y1, 0, h, body=name)


# ------------------------------------------------------------------------------------------------- gripper (docs/21 §12)
# The "Dummy" 3-prong gripper of docs/06 §2, option A: three fingers at 120 deg (azimuth 90, 210, 330) pivot on lugs
# at R GR. Each finger's inner tail has a slot; a pin in the carrier's fork rides in it. The carrier is driven along
# the axis by an N20 gearmotor whose output shaft is an M3 lead screw: carrier down = closed. A 693ZZ in the sleeve
# top takes the screw's thrust, so the gearbox doesn't. Aperture (between the inner faces of the fingertips) 6-80 mm,
# drawn at 40. The tool board (ESP32-C3, CAN, DRV8833, buck: docs/06 §1) stands on a wall at -y, between fingers.
GZ, GB = 667.0, 673.0            # tool plate's far face; gripper base top
GR, GP, GL, GD = 29.0, 722.0, 50.0, 14.0   # pivot radius and height, finger length (pivot -> tip), pin lever
G_OPEN = (6.0, 80.0)             # aperture closed / open (mm)


GPAD = 2.0                       # TPU pad on each finger's inner face, upper 25 mm (aperture is between the pads)


def finger_angle(aperture):
    """Finger tilt (deg, + = outward) for an aperture: the pad's tip corner (-4 - GPAD, GL) sits at R aperture / 2."""
    u = 4 + GPAD
    c, R = aperture / 2 - GR, math.hypot(GL, u)
    return math.degrees(math.asin(c / R) + math.atan2(u, GL))


def pin_z(aperture):
    """Carrier pin height: closed GP - 7.0, open GP + 4.3; drawn at 40 mm (tilt -5.8 deg)."""
    return GP + GD * math.tan(math.radians(finger_angle(aperture)))


def _fw(pts, th):
    """Finger frame (u inward -, w up from the pivot) -> world (y, z) for the finger at azimuth 90."""
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    return [(GR + u * c + w * s, GP - u * s + w * c) for u, w in pts]


def gripper_body(p):
    b = p.cyl("z", GZ, GB, 76, body="Gripper body")
    m3 = circle_pts(20, 6, 0)                                    # all six tool-plate inserts
    p.ext("z", GZ, GB, CUT, [(x, y, 3.4) for x, y in m3], into=b, name="M3 to the tool plate")
    p.ext("z", GB - 2.5, GB, CUT, [(x, y, 6.5) for x, y in m3], into=b, name="Counterbores")
    p.ext("z", GB, 709.5, JOIN, polys=[rect(-8, 8, -7, 7)], into=b, name="Motor sleeve")
    p.ext("z", GZ, 701, CUT, polys=[rect(-6.2, 6.2, -5.2, 5.2)], into=b, name="N20 pocket (in from below)")
    p.ext("z", 701, 708, CUT, [(0, 0, 8.1)], into=b, name="693ZZ + spacer bore")
    p.ext("z", 708, 709.5, CUT, [(0, 0, 7.0)], into=b, name="Lip (holds the bearing against the grip load)")
    p.ext("z", GZ, GB + 4, CUT, polys=[rect(-4, 4, -9.5, -5.2)], into=b, name="Wires: tool plate bore -> board")
    lug = p.ext("x", -6.25, 6.25, NEW, polys=[rect(GR - 6, GR + 6, GB, 712)], body="lug")
    for x0, x1 in ((-6.25, -3.25), (3.25, 6.25)):
        p.ext("x", x0, x1, JOIN, polys=[rect(GR - 6, GR + 6, 712, GP + 6)], into=lug, name="Lug cheek")
    p.ext("x", -6.25, 6.25, CUT, [(GR, GP, 3.0)], into=lug, name="Pivot pin 3 (press)")
    p.pattern_z([lug], 3, join=b)
    p.ext("y", -23, -21, JOIN, polys=[rect(-20, 20, GB, 711)], into=b, name="Tool board wall")
    p.ext("y", -23, -21, CUT, polys=[rect(-4, 4, GB, GB + 4)], into=b, name="Wire notch")
    m2 = [(x, z) for x in (-17, 17) for z in (678, 706)]
    p.ext("y", -25, -23, JOIN, [(x, z, 5) for x, z in m2], into=b, name="Board standoffs")
    p.ext("y", -25, -21, CUT, [(x, z, 1.8) for x, z in m2], into=b, name="M2 (self-tapping)")


def gripper_fingers(p, ap=40.0):
    fw = lambda pts: _fw(pts, finger_angle(ap))
    f = p.ext("x", -3, 3, NEW, polys=[fw([(-4, 0), (-4, GL), (2, GL), (4, GL - 3), (4, 0)])], body="Finger 90")
    p.ext("x", -3, 3, JOIN, polys=[fw(rect(-18.6, 0, -3.5, 3.5))], into=f, name="Tail")
    p.ext("x", -3, 3, JOIN, [(*fw([(0, 0)])[0], 10)], into=f, name="Pivot boss")
    p.ext("x", -3, 3, CUT, [(*fw([(0, 0)])[0], 3.2)], into=f, name="Pivot")
    s0, s1 = GD, GD / math.cos(math.radians(max(abs(finger_angle(a)) for a in G_OPEN)))   # pin's travel in the slot
    slot = [(-s0 + 1.6 * math.sin(math.pi * i / 8), -1.6 * math.cos(math.pi * i / 8)) for i in range(9)]
    slot += [(-s1 - 1.6 * math.sin(math.pi * i / 8), 1.6 * math.cos(math.pi * i / 8)) for i in range(9)]
    p.ext("x", -3, 3, CUT, polys=[fw(slot)], into=f, name="Slot for the carrier pin")
    p.pattern_z([f], 3, names=("Finger 90", "Finger 210", "Finger 330"))


def gripper_pads(p, ap=40.0):
    """TPU pads (glued), flat for now: a Fin Ray tip can replace them later (docs/06 §2)."""
    pad = p.ext("x", -3, 3, NEW, polys=[_fw(rect(-4 - GPAD, -4, GL - 25, GL), finger_angle(ap))], body="Pad 90 (TPU)")
    p.pattern_z([pad], 3, names=("Pad 90 (TPU)", "Pad 210 (TPU)", "Pad 330 (TPU)"))


def gripper_carrier(p, ap=40.0):
    GC = pin_z(ap)
    b = p.cyl("z", GC - 3.5, GC + 3.5, 12, body="Carrier")
    p.ext("z", GC - 3.5, GC + 3.5, CUT, [(0, 0, 3.4)], into=b, name="Lead screw clearance")
    hexagon = circle_pts(5.6 / math.sqrt(3), 6, 30)
    p.ext("z", GC - 1.25, GC + 1.25, CUT, polys=[hexagon, L.rotated_rect(*polar(4, 30), 8, 5.6, 30)], into=b,
          name="M3 nut pocket + slot to put it in")
    arm = p.ext("z", GC - 3.5, GC + 3.5, NEW, polys=[rect(-6.25, 6.25, 5, 8.4)], body="arm")
    p.ext("z", GC - 3.5, GC + 3.5, JOIN, polys=[rect(3.25, 6.25, 8.4, 18.5), rect(-6.25, -3.25, 8.4, 18.5)], into=arm,
          name="Fork")
    p.ext("x", -6.25, 6.25, CUT, [(GR - GD, GC, 3.0)], into=arm, name="Pin 3 (press)")
    p.pattern_z([arm], 3, join=b)


def gripper_bought(p, ap=40.0):
    """N20 gearmotor with encoder (12 g) and M3 lead-screw shaft, 693ZZ, nuts, the 6 pins, the tool board's space."""
    p.box(-6, 6, -5, 5, GZ, 701, body="N20 gearmotor + encoder")
    p.cyl("z", 701, 731, 3, body="M3 lead screw (steel)")
    p.ring("z", 701, 704, 8, 7, body="Spacer (printed)")
    p.ring("z", 701.3, 703.7, 6.35, 3, body="Jam nut (steel)")
    p.ring("z", 704, 708, 8, 3, body="693ZZ (steel)")
    p.ring("z", 708, 710.4, 6.35, 3, body="Jam nut 2 (steel)")
    GC = pin_z(ap)
    p.ring("z", GC - 1.2, GC + 1.2, 5.5, 3, body="Drive nut (steel)")
    pins = [p.cyl("x", -6, 6, 3, at=at, body="Pin 3 x 12 (steel)") for at in ((GR, GP), (GR - GD, GC))]
    p.pattern_z(pins, 3, names=["Pin 3 x 12 (steel)"] * 6)
    t = p.box(-20, 20, -32, -25, GB + 1, 710, body="Tool board")              # 40 x 36, parts on both faces
    p.ext("y", -32, -25, CUT, [(x, z, 2.2) for x in (-17, 17) for z in (678, 706)], into=t, name="M2")


def all_links(cyc, pg, j7):
    return {"DA_L_StandLeft": lambda p: stand_left(p, cyc), "DA_L_TowerRight": tower_right,
            "DA_L_CBPulley": cb_pulley, "DA_L_Link2": lambda p: link2(p, cyc), "DA_L_Link3": lambda p: link3(p, cyc),
            "DA_L_Forearm": lambda p: forearm(p, cyc, pg), "DA_L_Link5": lambda p: link5(p, pg),
            "DA_L_Link6": lambda p: link6(p, pg, j7), "DA_T_Receiver": receiver, "DA_T_ToolPlate": toolplate,
            "DA_B_Board": board, "DA_B_Spaces": spaces, "DA_G_Body": gripper_body, "DA_G_Fingers": gripper_fingers, "DA_G_Pads": gripper_pads,
            "DA_G_Carrier": gripper_carrier, "DA_P_Gripper": gripper_bought}

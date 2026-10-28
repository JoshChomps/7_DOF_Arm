#!/usr/bin/env python3
"""arm.toml [[masses]] from the CAD (docs/20 §9): volumes and centres from tools/cad_bodies.json (written in Fusion by
`tools/fusion_run.py sweep bodies`, DA_ARM at the zero pose), times a density per kind of part, plus catalogue
masses for the motors and drivers and point masses for what the CAD doesn't model (bearings, steel pins, fasteners).

    python tools/cad_masses.py            # prints the [[masses]] block to paste into arm.toml

Frames: the CAD world is the arm.toml base frame turned 90 deg about z (arm.toml x = -CAD y, y = CAD x; checked
against the FK to 0.01 mm, docs/20 §1). Each lump's `radius` gives the same mean moment of inertia as its bodies.
"""
import json
import math
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).parent
PETG, SOLID, AL, RUBBER = 0.90, 1.27, 2.70, 1.20      # g/cm3: printed parts not sliced (median of the sliced
# housings and links, 6 walls + 40 % gyroid, docs/21 §4), 100 % infill, Al, belt. Sliced parts use their grams.
SLICED = HERE.parent / "stl" / "da" / "slice.json"
DENSE = ("Disc", "Planet", "Sun", "Cam", "Carrier 1", "Spacer", "Inner spacer", "Outer spacer", "Retainer")
MOTOR = {"N23_76": (1.10, 0.09), "N17_60": (0.50, 0.05), "N17_40": (0.28, 0.05), "N17_20": (0.127, 0.05)}  # kg
PIN = lambda d, l, n: n * math.pi * d * d / 4 * l * 7.85e-6          # n steel pins, mm -> kg
BRG = {"6810": 0.052, "6806": 0.022, "6804": 0.018, "6802": 0.008, "623": 0.0016, "608": 0.012}
# per module, in its own frame (z mm on the axis): (kg, z, side) with side "fixed" or "out"
HW = {
    "C2": [(2 * BRG["6802"], -10, "fixed"), (2 * BRG["6804"], 16, "fixed"), (2 * BRG["6810"], 45, "fixed"),
           (PIN(6, 30, 25), 18, "fixed"), (PIN(6, 25, 8), 21, "out"), (0.04, 20, "fixed")],
    "C34": [(2 * BRG["6802"], -6, "fixed"), (2 * BRG["6804"], 16, "fixed"), (2 * BRG["6810"], 42, "fixed"),
            (PIN(5, 25, 25), 15, "fixed"), (PIN(5, 25, 8), 21, "out"), (0.03, 20, "fixed")],
    "P56": [(2 * BRG["6806"], 55, "fixed"), (12 * BRG["623"] + PIN(3, 16, 3) + PIN(3, 24, 3), 30, "fixed"),
            (0.02, 30, "fixed")],
    "P7": [(2 * BRG["6806"], 30, "fixed"), (BRG["6804"], 4, "fixed"), (PIN(3, 12, 3), 15, "fixed"),
           (0.015, 20, "fixed")],
}
J1_HW = [(2 * BRG["6810"] + 2 * BRG["6804"] + PIN(5, 25, 26) + PIN(6, 25, 8) + 0.03, (0, 0, 30))]
CARRIED = {"J1 base": "base", "DA_P_N17_60": "base", "J1 turntable": "J1", "DA_L_StandLeft": "J1", "DA_L_TowerRight": "J1",
           "DA_L_CBPulley": "J1", "DA_L_Link2": "J2", "DA_L_Link3": "J3", "DA_L_Forearm": "J4", "DA_L_Link5": "J5",
           "DA_L_Link6": "J6", "DA_T_Receiver": "J7", "DA_T_ToolPlate": "J7",
           "DA_G_Body": "J7", "DA_G_Fingers": "J7", "DA_G_Pads": "J7", "DA_G_Carrier": "J7", "DA_P_Gripper": "J7"}
PREV = {"J2": "J1", "J3": "J2", "J4": "J3", "J5": "J4", "J6": "J5", "J7": "J6"}
LINK_FASTENERS = 0.015                 # kg of screws and inserts per link
NO_SCREWS = ("DA_G_Fingers", "DA_G_Pads", "DA_G_Carrier", "DA_P_Gripper", "DA_P_SlipRing")   # pinned, and the pins are in DA_P_Gripper
EXTRA = [  # not in the CAD at all: (on, kg, arm.toml pos m, what)
    ("J1", 0.08, (0.0, 0.07, 0.15), "counterbalance spring, cable, 608, M8 axle (right tower)"),
    ("J7", 0.015, (0.0, 0.0, 0.659), "tool changer balls, magnets, M5 screws, pogo pins"),
]


FIXED_KG = {"N20 gearmotor + encoder": 0.012, "Tool board": 0.012, "Slip ring body (stator)": 0.010}      # catalogue (bought envelopes drawn as boxes)


def density(r):
    p, b = r["part"], r["body"]
    if "steel" in b:
        return 7.85
    if "TPU" in b:
        return RUBBER
    if p.startswith("DA_P_Belt"):
        return RUBBER
    if p.startswith("DA_P_Pulley") or "Al 3 mm" in b:
        return AL
    if any(b.startswith(k) for k in DENSE) or "Cam" in b or "Cycloid_Disc" in p:
        return SOLID
    return PETG


def to_toml(c):                                        # CAD mm -> arm.toml m
    return (-c[1] / 1000, c[0] / 1000, c[2] / 1000)


def joint_of(r, tops):
    """Which arm joint a module occurrence is (J3 below J4, J5 below J6)."""
    zs = sorted(tops[r["top"]])
    order = {"DA_SA_C2": ["J2"], "DA_SA_C34": ["J3", "J4"], "DA_SA_P56": ["J5", "J6"], "DA_SA_P7": ["J7"]}[r["top"]]
    return order[zs.index(r["top_z"])]


def lumps(rows, modules):
    sliced = json.loads(SLICED.read_text()) if SLICED.exists() else {}
    file_vol = defaultdict(dict)                       # part -> {body: cm3}: one copy of the part's file
    for r in rows:
        file_vol[r["part"]][r["body"]] = r["volume"]
    tops = defaultdict(set)
    for r in rows:
        if r["top"].startswith("DA_SA"):
            tops[r["top"]].add(r["top_z"])
    out = defaultdict(list)                            # (on, kind) -> [(kg, pos m, k2 m2)]
    for r in rows:
        if r["top"].startswith("DA_B_"):             # torque_budget.py counts the board and PSU itself
            continue
        kg_body = FIXED_KG.get(r["body"], r["volume"] * density(r) / 1000)
        if "grams" in sliced.get(r["part"], {}):       # the slicer's grams, shared over the file's bodies by volume
            kg_body = sliced[r["part"]]["grams"] / 1000 * r["volume"] / sum(file_vol[r["part"]].values())
        pos, k2 = to_toml(r["com"]), r["k2"] * 1e-6
        if r["part"].startswith("DA_P_N"):             # motor envelope: catalogue mass on the motor / driver box
            kind = r["part"][5:]
            if r["body"] not in ("Motor", "MKS driver"):
                continue
            kg_body = MOTOR[kind][0 if r["body"] == "Motor" else 1]
        if r["top"].startswith("DA_SA"):
            j = joint_of(r, tops)
            on = j if r["group"].endswith("out") else PREV[j]
            what = "motor + driver" if r["part"].startswith("DA_P_N") else ("output" if on == j else "structure")
            out[(on, f"{j} {what}")].append((kg_body, pos, k2))
        else:
            on = CARRIED.get(r["group"] or r["top"], CARRIED.get(r["top"]))
            name = r["top"] if r["top"].startswith("DA_P_N") else "J1 base" if on == "base" else (
                "J1 turntable" if r["group"] == "J1 turntable" else r["top"])
            out[(on, name)].append((kg_body, pos, k2))
    for j, (m, R, t) in modules.items():               # hardware the CAD doesn't model, on each module's axis
        for kg, z, side in HW[m]:
            pos = to_toml([R[i][2] * z + t[i] for i in range(3)])
            out[(j, f"{j} output") if side == "out" else (PREV[j], f"{j} structure")].append((kg, pos, 0.0))
    for kg, p in J1_HW:
        out[("base", "J1 base")].append((kg, to_toml(p), 0.0))
    for name in CARRIED:
        if name.startswith("DA_") and name not in NO_SCREWS:
            key = next(k for k in out if k[1] == name)
            k2s = [x for x in out[key]]
            out[key].append((LINK_FASTENERS, tuple(sum(x[0] * x[1][i] for x in k2s) / sum(x[0] for x in k2s)
                                                    for i in range(3)), 0.0))
    for on, kg, p, what in EXTRA:
        out[(on, what)].append((kg, p, 0.0))
    return out


def combine(items):
    m = sum(x[0] for x in items)
    c = [sum(x[0] * x[1][i] for x in items) / m for i in range(3)]
    i_avg = sum(x[0] * (x[2] + 2 / 3 * sum((x[1][i] - c[i]) ** 2 for i in range(3))) for x in items)
    return m, c, max(math.sqrt(i_avg / (0.4 * m)), 0.01)


NAMES = {"J2 motor + driver": "J2 motor (NEMA23 76 mm LKD57HS76-2804-14) + SERVO57D",
         "J2 structure": "J2 cycloid module (printed, 6810/6804/6802, steel pins)",
         "J3 motor + driver": "J3 motor (LDO NEMA17 60 mm) + SERVO42D",
         "J3 structure": "J3 cycloid module (printed, bearings, steel pins)",
         "J4 motor + driver": "J4 motor (LDO NEMA17 60 mm) + SERVO42D",
         "J4 structure": "J4 cycloid module (printed, bearings, steel pins)",
         "J5 motor + driver": "J5 motor (LDO NEMA17 40 mm) + SERVO42D", "J5 structure": "J5 planetary module",
         "J6 motor + driver": "J6 motor (LDO NEMA17 40 mm) + SERVO42D", "J6 structure": "J6 planetary module",
         "J7 motor + driver": "J7 motor (LDO NEMA17 20 mm pancake) + SERVO42D",
         "J7 structure": "J7 module (belt + planetary)",
         **{f"J{k} output": f"J{k} output side (carrier, flange{', output pins' if k < 5 else ''})" for k in range(2, 8)}, "J1 base": "J1 base (printed housing, cycloid, bearings, pins)",
         "J1 turntable": "J1 turntable and pin carrier",
         "DA_P_N17_60": "J1 motor (LDO NEMA17 60 mm) + SERVO42D, under the board (D37)", "DA_L_StandLeft": "left stand (J2 module)",
         "DA_L_TowerRight": "right tower (counterbalance)", "DA_L_CBPulley": "counterbalance pulley",
         "DA_L_Link2": "link 2", "DA_L_Link3": "link 3", "DA_L_Forearm": "forearm link", "DA_L_Link5": "link 5",
         "DA_L_Link6": "link 6", "DA_T_Receiver": "tool changer, robot side", "DA_T_ToolPlate": "tool changer, tool side",
         "DA_G_Body": "gripper body (base, sleeve, lugs, board wall)", "DA_G_Fingers": "gripper fingers (3)", "DA_G_Pads": "gripper TPU pads (3)",
         "DA_G_Carrier": "gripper carrier", "DA_P_Gripper": "gripper N20, lead screw, 693ZZ, pins, tool board"}
ORDER = ["base", "J1", "J2", "J3", "J4", "J5", "J6", "J7"]


def main():
    dump = json.loads((HERE / "cad_bodies.json").read_text(encoding="utf-8"))
    groups = lumps(dump["bodies"], dump["modules"])
    blocks, total, by_on = [], 0.0, defaultdict(float)
    for key in sorted(groups, key=lambda k: (ORDER.index(k[0]), k[1])):
        m, c, rad = combine(groups[key])
        total += m
        by_on[key[0]] += m
        blocks.append(f'[[masses]]\nname = "{NAMES.get(key[1], key[1])}"\non = "{key[0]}"\nmass = {m:.3f}\n'
                      f'pos = [{c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}]\nradius = {rad:.3f}\n')
    print("\n".join(blocks))
    print(f"# total {total:.2f} kg; by carrying joint: " + ", ".join(f"{k} {by_on[k]:.2f}" for k in ORDER))


if __name__ == "__main__":
    main()

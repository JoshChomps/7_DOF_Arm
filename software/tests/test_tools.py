"""Checks for the standalone design tools in ../tools."""
import importlib.util
import math
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[2] / "tools"


def load(name):
    import sys
    if str(TOOLS) not in sys.path:
        sys.path.insert(0, str(TOOLS))                   # tools import their shared cadio module
    spec = importlib.util.spec_from_file_location(name, TOOLS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_cycloid_disc_meshes_every_pin_with_the_set_clearance():
    cyc = load("cycloid")
    R, Rr, N, e, clearance = 50.0, 3.0, 31, 1.3, 0.1
    pts = cyc.disc_profile(R, Rr + clearance, N, e, 4000)
    gaps = []
    for i in range(N):  # ring pins in the assembled position: housing axis at (-e, 0)
        t = 2 * math.pi * i / N
        px, py = R * math.cos(t) - e, R * math.sin(t)
        gaps.append(min(math.hypot(x - px, y - py) for x, y in pts) - Rr)
    assert min(gaps) > clearance - 0.01 and max(gaps) < clearance + 0.01


def test_cycloid_undercut_check():
    cyc = load("cycloid")
    rho = cyc.min_convex_radius(50.0, 31, 1.3)
    assert 3.1 < rho < 6.1  # default pins pass, 6 mm-radius pins would undercut


def test_torque_budget_and_bom_scripts_run(capsys):
    under_powered = load("torque_budget").main([])
    load("bom_total").main()
    out = capsys.readouterr().out
    assert "J2 shoulder" in out and "core" in out
    assert under_powered == []          # the drives in arm.toml pass their own budget


def test_torque_budget_agrees_with_the_arm_model():
    """The budget reads arm.toml through deskarm, so it can't drift from the sim."""
    import numpy as np
    from deskarm import load_arm
    tb = load("torque_budget")
    arm = load_arm()
    static, design, at0, atv, pose = tb.budget(arm, 1.0)
    stretched = np.abs(arm.gravity_torque(tb.STRETCHED, 1.0))
    for k in (1, 3, 5):                                  # J2, J4, J6: the arm straight out, or a little worse
        assert stretched[k] - 1e-9 <= static[k] <= 1.05 * stretched[k]   # (the CAD's lumps sit off the centreline)
    assert 0.8 * static[3] < static[2] < 1.2 * static[3]     # J3 holds the forearm sideways: elbow-class
    assert 0.8 * static[5] < static[4] < 1.2 * static[5]     # J5 holds the tool sideways: wrist-class
    # a random search can't find a worse pose than the 90-degree grid
    rng = np.random.default_rng(0)
    worst = np.max([np.abs(arm.gravity_torque(rng.uniform(arm.lower, arm.upper), 1.0)) for _ in range(3000)], axis=0)
    assert np.all(worst[:6] <= static[:6] * (1 + 1e-9))
    assert np.allclose(design[1:], static[1:] * tb.DYNAMIC_FACTOR * tb.SAFETY_FACTOR)
    assert np.allclose(at0, arm.torque_limit()) and np.allclose(atv, arm.torque_limit(arm.vmax))


def test_design_options_start_from_the_torque_budget(capsys):
    """Option A is arm.toml untouched, and the heavier rev. 2 wrist can only raise the loads."""
    import numpy as np
    from deskarm import load_arm
    tb, do = load("torque_budget"), load("design_options")
    results = dict(do.main([]))
    assert "baseline" in capsys.readouterr().out
    base = results["A  baseline (arm.toml)"]
    static = tb.budget(load_arm(), tb.PAYLOAD_KG)[0]
    assert np.allclose(base["static"], static) and base["total"] == pytest.approx(load_arm().total_mass)
    heavy = results["B  rev. 2 heavy wrist (dropped)"]
    assert heavy["total"] > base["total"] and np.all(heavy["static"] >= base["static"] - 1e-9)
    assert np.all(base["margin"][1:] >= 1.0)              # the accepted light wrist passes on every joint
    moved = results["C  J4 + J6 motors moved back"]
    assert moved["total"] == pytest.approx(base["total"]) and moved["static"][1] < base["static"][1]


# ---------------------------------------------------------------------------- gears ----------
def _mesh_overlaps(gears, zs, zp, zr, n, m, backlash, planet_phase_error=0.0, grow=0.0, steps=24):
    """Max overlap area (sun-planet, planet-ring) while the sun turns one tooth, carrier fixed."""
    from manifold3d import CrossSection
    sun = gears._cs(gears.gear_outline(zs, m, 20, backlash / 2))
    pl = gears._cs(gears.gear_outline(zp, m, 20, backlash / 2))
    if grow:
        pl = pl.offset(grow)
    ring = CrossSection.circle(m * zr, 256) - gears._cs(gears.ring_space_outline(zr, m, 20, backlash / 2))
    a = m * (zs + zp) / 2
    sp = pr = 0.0
    for k in range(steps):
        t = 2 * math.pi / zs * k / steps
        s = sun.rotate(math.degrees(t))
        r = ring.rotate(math.degrees(gears.ring_phase(zp, zr) - t * zs / zr))
        for phi, ph in gears.planet_phases(zs, zp, n):
            p = pl.rotate(math.degrees(ph + planet_phase_error - t * zs / zp)).translate(
                (a * math.cos(phi), a * math.sin(phi)))
            sp = max(sp, (s ^ p).area())
            pr = max(pr, (p ^ r).area())
    return sp, pr


def test_planetary_sets_mesh_without_interference():
    pytest.importorskip("manifold3d")
    gears = _load_tools_module("gears")
    for zs, zp, zr, n in [(12, 18, 48, 3), (13, 19, 51, 4), (14, 22, 58, 3)]:
        sp, pr = _mesh_overlaps(gears, zs, zp, zr, n, 1.25, backlash=0.15)
        assert sp < 1e-6 and pr < 1e-6, (zs, zp, zr, n, sp, pr)
        # zero backlash: teeth just touch (no overlap) ...
        sp0, pr0 = _mesh_overlaps(gears, zs, zp, zr, n, 1.25, backlash=0.0)
        assert sp0 < 1e-2 and pr0 < 1e-2
        # ... and really are in contact: a 0.05 mm fatter planet collides with both
        spg, prg = _mesh_overlaps(gears, zs, zp, zr, n, 1.25, backlash=0.0, grow=0.05)
        assert spg > 0.01 and prg > 0.01


def test_wrong_planet_phase_is_detected():
    pytest.importorskip("manifold3d")
    gears = _load_tools_module("gears")
    sp, _ = _mesh_overlaps(gears, 12, 18, 48, 3, 1.25, backlash=0.15,
                           planet_phase_error=math.pi / 18, steps=4)
    assert sp > 1.0  # half a tooth off -> large collision, so the test above is meaningful


def test_herringbone_parts_clear_in_3d():
    pytest.importorskip("manifold3d")
    gears = _load_tools_module("gears")
    s = gears.make_set(12, 18, 48, 3, 1.25, face=10, helix=30, backlash=0.15)
    a = s["a"]
    for k in range(3):
        t = 2 * math.pi / 12 * k / 3
        sun = s["sun"].rotate((0, 0, math.degrees(t)))
        ring = s["ring"].rotate((0, 0, math.degrees(s["ring_phase"] - t * 12 / 48)))
        for phi, ph in s["planets"]:
            p = s["planet"].rotate((0, 0, math.degrees(ph - t * 12 / 18))).translate(
                (a * math.cos(phi), a * math.sin(phi), 0))
            assert (sun ^ p).volume() < 1e-6
            assert (p ^ ring).volume() < 1e-6


def test_planetary_search_obeys_rules():
    gears = _load_tools_module("gears")
    for ratio, stages in gears.search(5.0, 1.25, (3, 4), stages=1, top=20, max_od=90):
        st = stages[0]
        assert st["zr"] == st["zs"] + 2 * st["zp"]
        assert (st["zs"] + st["zr"]) % st["n"] == 0
        assert st["ring_od"] <= 90
        assert abs(ratio - (1 + st["zr"] / st["zs"])) < 1e-12
    two = gears.search(25.0, 1.25, (3, 4), stages=2, top=10, max_od=90)
    keys = {tuple(sorted((s["zs"], s["zp"], s["zr"], s["n"]) for s in st)) for _, st in two}
    assert len(keys) == len(two)  # no A+B / B+A duplicates
    for ratio, st in gears.identical_stage_options(25.0, 1.25, (3, 4), max_od=90):
        assert st[0] is st[1] and abs(ratio - st[0]["ratio"] ** 2) < 1e-9


# ---------------------------------------------------------------------------- coupons --------
def test_coupons_have_correct_holes_and_fit_the_printer():
    pytest.importorskip("manifold3d")
    coupons = _load_tools_module("coupons")
    c = coupons.make_all(segments=48)
    assert c["pins"].genus() == 24 and c["bearings"].genus() == 12     # through holes
    assert c["inserts"].genus() == 0 and c["pegs"].genus() == 0        # blind holes / pegs
    for k in ("rings_6810", "rings_6806"):
        assert len(c[k].decompose()) == 8                               # 4 seat rings, 4 loose journal cups
    for m in c.values():
        bb = m.bounding_box()
        assert max(bb[3] - bb[0], bb[4] - bb[1]) <= 170                 # fits the A1 mini


def test_coupon_holes_are_sized_flat_to_flat():
    pytest.importorskip("manifold3d")
    coupons = _load_tools_module("coupons")
    n, t = 48, 6.0
    plain = coupons.grid_coupon([3, 6], [0.0, 0.2], thickness=t, pitch_x=11,
                                row_pitch=lambda d: d + 7, labels=False, segments=n)
    bb = plain.bounding_box()
    box = (bb[3] - bb[0]) * (bb[4] - bb[1]) * t
    removed = 0.0
    for d in (3.0, 3.2, 6.0, 6.2):
        rc = d / 2 / math.cos(math.pi / n)                 # circumscribed polygon radius
        removed += n * rc * rc * math.sin(2 * math.pi / n) / 2 * t
    assert abs(plain.volume() - (box - removed)) / box < 1e-6


def test_bench_parts_clear_the_j2_module_and_plates_fit_the_bed():
    """tools/print_pack.py: the stand and lever touch nothing of C2 but their seats, and packed parts stay apart."""
    pytest.importorskip("manifold3d")
    pp = _load_tools_module("print_pack")
    assert all(v < 1 for v in pp.clash("C2").values())            # mm³
    for plate in pp.pack([pp.oriented(pp.ROOT / s) for s in pp.JOBS["c2"]]):
        boxes = [(x, y, *t.reshape(-1, 3).max(0)[:2]) for (_, t, _), x, y in plate]
        for i, (x, y, w, d) in enumerate(boxes):
            assert x + w <= pp.BED and y + d <= pp.BED
            for x2, y2, w2, d2 in boxes[i + 1:]:
                assert x + w <= x2 or x2 + w2 <= x or y + d <= y2 or y2 + d2 <= y


_load_tools_module = load


def test_fusion_skeleton_matches_arm_toml():
    """tools/fusion_skeleton.py copies numbers from arm.toml (it runs inside Fusion, so it can't import deskarm)."""
    import ast

    from deskarm.kinematics import load_config

    tree = ast.parse((TOOLS / "fusion_skeleton.py").read_text(encoding="utf-8"))
    consts = {t.id: ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name) and t.id in ("LENGTHS", "LIMITS")}
    lengths = {name: mm for name, mm, _ in consts["LENGTHS"]}
    cfg = load_config()
    joints = {j["name"]: j for j in cfg["joints"]}
    assert lengths["base_height"] == round(joints["J1"]["d"] * 1000)
    assert lengths["upper_arm"] == round(joints["J3"]["d"] * 1000)
    assert lengths["forearm"] == round(joints["J5"]["d"] * 1000)
    assert lengths["wrist_flange"] == round(joints["J7"]["d"] * 1000)
    assert lengths["tcp_offset"] == round(cfg["tool"]["tcp_offset_z"] * 1000)
    assert lengths["spring_anchor"] == round(cfg["counterbalance"]["anchor_height"] * 1000)
    assert lengths["spring_attach"] == round(cfg["counterbalance"]["attach_distance"] * 1000)
    assert consts["LIMITS"] == {n: (j["min_deg"], j["max_deg"]) for n, j in joints.items()}
    assert all(j.get("a", 0) == 0 and j.get("offset_deg", 0) == 0 for j in joints.values())   # the skeleton assumes both

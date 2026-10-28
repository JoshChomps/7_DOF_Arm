"""Stepper torque-speed curve and the lumped-mass model (numpy only, no physics engine needed)."""
import tomllib
from dataclasses import replace
from importlib import resources

import numpy as np
import pytest

from deskarm import load_arm
from deskarm.kinematics import G
from deskarm.motor import SQRT2, corner_speed, stepper_torque

ARM = load_arm()
CFG = tomllib.loads(resources.files("deskarm").joinpath("data/arm.toml").read_text())
STRETCHED = np.deg2rad([0, 90, 0, 0, 0, 0, 0])
N17 = dict(hold_torque=0.65, rated_current=2.1, resistance=1.6, inductance=3.0e-3)


# ---------------------------------------------------------------- stepper curve ------------
def test_stepper_curve_is_flat_then_falls_and_obeys_the_power_limit():
    w = np.linspace(0.0, 300.0, 601)                                # 0 .. ~2900 rpm
    t = stepper_torque(w, **N17, voltage=24.0)
    assert t[0] == pytest.approx(N17["hold_torque"])                # full torque at standstill
    assert np.all(np.diff(t) <= 1e-12)                              # never rises with speed
    wc = corner_speed(**N17, voltage=24.0)
    assert np.allclose(t[w < 0.8 * wc], N17["hold_torque"], rtol=0.02)   # current-limited plateau
    # physics: shaft power can never exceed what the supply can push into the two phases
    assert np.all(t * w <= 24.0 * SQRT2 * N17["rated_current"] + 1e-9)
    # well above the corner the motor is voltage-limited: roughly constant power, torque ~ 1/speed
    hi = w > 2 * wc
    p = (t * w)[hi]
    assert p.max() / p.min() < 1.1
    assert t[-1] < 0.25 * N17["hold_torque"]


def test_more_voltage_or_less_inductance_buys_speed_not_holding_torque():
    w = 100.0                                                       # ~950 rpm, past the corner
    base = stepper_torque(w, **N17, voltage=24.0)[0]
    assert stepper_torque(w, **N17, voltage=48.0)[0] > 1.8 * base   # power ~ voltage
    low_l = dict(N17, inductance=1.5e-3)
    assert stepper_torque(w, **low_l, voltage=24.0)[0] > 1.5 * base
    assert stepper_torque(0.0, **N17, voltage=48.0)[0] == pytest.approx(N17["hold_torque"])
    assert np.allclose(stepper_torque([0, 50, 500], **dict(N17, inductance=0.0)), N17["hold_torque"])


def test_joint_limits_follow_the_motor_curve_through_the_gearbox():
    j = ARM.joints[1]                                               # J2
    d = ARM.drive
    for v in (0.0, 0.5 * j.vmax, j.vmax, 1.7 * j.vmax):
        motor = stepper_torque(v * j.ratio, j.hold_torque, j.rated_current, j.resistance, j.inductance,
                               d.voltage, d.pole_pairs)[0]
        expected = d.torque_derate * motor * j.ratio * j.efficiency
        assert j.torque_at(v) == pytest.approx(expected, rel=2e-3)
        qd = np.zeros(ARM.n)
        qd[1] = -v                                                  # direction doesn't matter
        assert ARM.torque_limit(qd)[1] == pytest.approx(expected, rel=2e-3)
    assert np.allclose(ARM.torque_limit(), [x.torque_max for x in ARM.joints])
    assert np.all(ARM.torque_limit(ARM.vmax) <= ARM.torque_limit() + 1e-12)


def test_speed_at_torque_inverts_the_curve():
    lim0 = ARM.torque_limit()
    for frac in (0.9, 0.6):
        v = ARM.speed_at_torque(frac * lim0)
        finite = np.isfinite(v)
        assert finite.any()
        back = ARM.torque_limit(np.where(finite, v, 0.0))
        assert np.allclose(back[finite], (frac * lim0)[finite], rtol=5e-3)
    assert np.all(ARM.speed_at_torque(1.01 * lim0) == 0.0)          # can't deliver it at any speed
    assert np.all(np.isinf(ARM.speed_at_torque(1e-6 * lim0)))       # can at every tabulated speed


def test_flat_limit_without_datasheet_values():
    """A joint given only a usable ``motor_torque`` keeps a speed-independent limit."""
    j = replace(ARM.joints[3], torque_curve=(), motor_torque=0.4)
    assert j.torque_max == pytest.approx(0.4 * j.ratio * j.efficiency)
    assert j.torque_at(5.0) == j.torque_max
    arm = replace(ARM, joints=ARM.joints[:3] + (j,) + ARM.joints[4:])
    assert arm.torque_limit(arm.vmax * 3)[3] == pytest.approx(j.torque_max)


# ---------------------------------------------------------------- mass model ---------------
def test_stretched_arm_gravity_torque_matches_the_mass_list_by_hand():
    """Arm straight out: torque = sum(m g lever), levers read straight from arm.toml. A pitch joint's lever is the
    lump's height above it (an offset along x or y doesn't change it); a roll joint's is the lump's y offset."""
    payload = 1.0
    z_shoulder = ARM.joints[0].d
    z_elbow = z_shoulder + ARM.joints[2].d
    z_wrist = z_elbow + ARM.joints[4].d
    z_tcp = z_wrist + ARM.joints[6].d + CFG["tool"]["tcp_offset_z"]
    names = [j["name"] for j in CFG["joints"]]
    tau = ARM.gravity_torque(STRETCHED, payload)
    for joint, z0 in (("J2", z_shoulder), ("J4", z_elbow), ("J6", z_wrist)):
        k = names.index(joint)
        moment = payload * G * (z_tcp - z0)
        for m in CFG["masses"]:
            if m["on"] != "base" and names.index(m["on"]) >= k:     # carried by this joint
                moment += m["mass"] * G * (m["pos"][2] - z0)
        assert abs(tau[k]) == pytest.approx(moment, rel=1e-9)
    for joint in ("J3", "J5"):                                       # the CAD's J4/J6 modules sit to one side
        k = names.index(joint)
        side = sum(m["mass"] * G * m["pos"][1] for m in CFG["masses"] if m["on"] != "base" and names.index(m["on"]) >= k)
        assert abs(tau[k]) == pytest.approx(abs(side), rel=1e-9)
    assert abs(tau[0]) < 1e-9 and abs(tau[6]) < 0.05                # yaw nothing; the gripper's N20 sits off its axis


def test_masses_add_up_and_gravity_is_linear_in_the_payload():
    assert ARM.total_mass == pytest.approx(sum(m["mass"] for m in CFG["masses"]))
    assert ARM.moving_mass == pytest.approx(sum(m["mass"] for m in CFG["masses"] if m["on"] != "base"))
    assert sum(i[0] for i in ARM.link_inertials()) == pytest.approx(ARM.moving_mass)
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.uniform(ARM.lower, ARM.upper)
        g0, g1, g2 = (ARM.gravity_torque(q, m) for m in (0.0, 1.0, 2.0))
        assert np.allclose(g2 - g1, g1 - g0, atol=1e-12)
        assert ARM.gravity_torque(q)[0] == pytest.approx(0.0, abs=1e-12)   # vertical axis: no load


def test_link_inertia_uses_parallel_axes():
    """Two lumps on one link: inertia about their common centre = sum of own + m d^2 terms."""
    inertials = ARM.link_inertials()
    for k, (mass, com, inertia) in enumerate(inertials):
        lumps = [m for m in ARM.masses if m.link == k]
        assert mass == pytest.approx(sum(m.mass for m in lumps))
        assert np.allclose(inertia, inertia.T) and np.all(np.linalg.eigvalsh(inertia) > 0)
        own = sum(0.4 * m.mass * m.radius**2 for m in lumps)
        zs = np.array([m.pos[2] for m in lumps])
        ms = np.array([m.mass for m in lumps])
        spread = float(ms @ (zs - ms @ zs / ms.sum()) ** 2)         # lumps lie along the arm axis
        # smallest principal inertia is about the arm axis (own terms only), largest adds the spread
        if all(abs(m.pos[0]) < 1e-12 and abs(m.pos[1]) < 1e-12 for m in lumps):
            ev = np.linalg.eigvalsh(inertia)
            assert ev[0] == pytest.approx(own, rel=1e-9)
            assert ev[-1] == pytest.approx(own + spread, rel=1e-9)


def test_numpy_gravity_matches_mujoco():
    mujoco = pytest.importorskip("mujoco")
    from deskarm.urdf import to_urdf
    model = mujoco.MjModel.from_xml_string(to_urdf(ARM, payload_kg=0.7))
    data = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.uniform(ARM.lower, ARM.upper)
        data.qpos[:] = q
        data.qvel[:] = 0
        data.qacc[:] = 0
        mujoco.mj_inverse(model, data)
        assert np.allclose(data.qfrc_inverse, ARM.gravity_torque(q, 0.7), atol=1e-4)

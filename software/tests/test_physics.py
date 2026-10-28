from dataclasses import replace

import numpy as np
import pytest

from deskarm import TrapezoidalTrajectory, load_arm
from deskarm.counterbalance import reachable_poses, size_spring, spring_torque, worst_case_torque

pytest.importorskip("mujoco")
from deskarm.physics import MujocoBackend, TorqueModel  # noqa: E402

FULL = load_arm()                                       # with the J2 spring and gear compliance
ARM = replace(FULL, counterbalance=None)                # no spring: the motors carry everything
STRETCHED = np.deg2rad([0, 90, 0, 0, 0, 0, 0])          # arm straight out horizontally


def run(be, seconds, dt=0.02):
    for _ in range(int(seconds / dt)):
        be.step(dt)


def test_gravity_torque_matches_the_mass_list():
    """MuJoCo's inverse dynamics at rest equals the numpy gravity model built from arm.toml's
    [[masses]] (itself checked against a hand calculation in test_motor_mass.py)."""
    rng = np.random.default_rng(2)
    for payload in (0.0, 1.0):
        tm = TorqueModel(ARM, payload_kg=payload)
        for q in [STRETCHED] + [rng.uniform(ARM.lower, ARM.upper) for _ in range(10)]:
            assert np.allclose(tm.gravity(q), ARM.gravity_torque(q, payload), atol=1e-4)


def test_holds_pose_under_gravity_with_stepper_stiffness():
    be = MujocoBackend(ARM, q0=STRETCHED, payload_kg=1.0, compliant=False)
    be.enable()
    run(be, 1.0)
    assert be.enabled and not be.faults
    assert np.max(np.abs(np.degrees(be.read() - STRETCHED))) < 0.1   # ~1 motor step at the joint
    # each motor supplies exactly the static load: nothing more once the arm has settled
    expect = np.abs(ARM.gravity_torque(STRETCHED, 1.0)) / ARM.torque_limit()
    assert np.allclose(be.torque_utilization, expect, atol=0.01)
    assert 0.3 < expect[1] < 0.6                       # and the rated payload leaves real headroom


@pytest.mark.parametrize("compliant", [False, True])
def test_tracks_a_fast_trajectory(compliant):
    """The motors track like before; the links then settle where the gear model says they must."""
    be = MujocoBackend(FULL, payload_kg=0.5, compliant=compliant)
    be.enable()
    q1 = ARM.home + np.deg2rad([60, -30, 40, 30, 90, -40, 180])
    tr = TrapezoidalTrajectory(ARM.home, q1, ARM.vmax, ARM.amax)
    t, worst = 0.0, 0.0
    while t < tr.duration + 0.5:
        be.command(tr.sample(t)[0])
        be.step(0.01)
        t += 0.01
        worst = max(worst, np.max(np.abs(be.read_motor() - tr.sample(t - 0.01)[0])))
    assert not be.faults
    assert np.degrees(worst) < 2.0                      # motor-side tracking, same bound either way
    run(be, 3.0)                                        # first mode ~5 Hz, zeta ~0.05: let it ring out
    assert np.max(np.abs(np.degrees(be.read_motor() - q1))) < 0.1
    sag = be.read_motor() - be.read()
    if not compliant:
        assert np.max(np.abs(np.degrees(sag))) < 1e-9
        return
    # static prediction: wind-up tau/k, plus anywhere inside the dead band (+-b/2)
    tau = TorqueModel(FULL, payload_kg=0.5).gravity(q1)
    k = np.array([j.gear_stiffness for j in FULL.joints])
    b = np.array([j.backlash for j in FULL.joints])
    assert np.all(np.abs(sag - tau / k) <= b / 2 + np.deg2rad(0.03)), np.degrees(sag - tau / k)


def max_payload(arm, q):
    """Payload (kg) at which the first joint runs out of standstill torque at pose q, and which."""
    g0, g1 = np.abs(arm.gravity_torque(q, 0.0)), np.abs(arm.gravity_torque(q, 1.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        kg = np.where(g1 - g0 > 1e-9, (arm.torque_limit() - g0) / (g1 - g0), np.inf)
    return float(kg.min()), arm.joints[int(np.argmin(kg))].name


def test_overload_stalls_and_disables():
    kg, weakest = max_payload(ARM, STRETCHED)
    be = MujocoBackend(ARM, q0=STRETCHED, payload_kg=1.25 * kg)
    be.enable()
    run(be, 2.0)
    assert not be.enabled
    assert weakest in be.faults                         # the joint the model says is weakest
    ok = MujocoBackend(ARM, q0=STRETCHED, payload_kg=0.8 * kg)
    ok.enable()
    run(ok, 2.0)
    assert ok.enabled and not ok.faults


def test_torque_falls_with_speed_so_a_fast_lift_stalls_where_a_slow_one_works():
    """Lift a heavy payload through horizontal with J2. The motor holds it and lifts it slowly,
    but at full speed the stepper has too little torque left: the guard flags the move and the
    physics stalls. With a speed-independent limit (same standstill torque) both would pass."""
    fast_j2 = replace(ARM.joints[1], vmax=np.deg2rad(90))            # curve covers up to 120 deg/s
    arm = replace(ARM, joints=ARM.joints[:1] + (fast_j2,) + ARM.joints[2:])
    flat = replace(arm, joints=tuple(replace(j, torque_curve=()) for j in arm.joints))
    assert np.allclose(flat.torque_limit(), arm.torque_limit())     # identical at standstill
    low, up = np.deg2rad([0, 115, 0, 0, 0, 0, 0]), np.deg2rad([0, 35, 0, 0, 0, 0, 0])
    payload = 2.4                    # stalls at full speed, lifts at half (2.6 before the TCP moved from 0.08 to 0.11, 20 Oct)
    static = abs(arm.gravity_torque(STRETCHED, payload)[1])
    assert arm.torque_limit(arm.vmax)[1] < static < 0.6 * arm.torque_limit()[1]

    def lift(model, speed_scale):
        tr = TrapezoidalTrajectory(low, up, model.vmax, model.amax, speed_scale=speed_scale)
        peak = TorqueModel(model, payload).trajectory_peak(tr)[0].max()
        be = MujocoBackend(model, q0=low, payload_kg=payload, compliant=False)
        be.enable()
        t = 0.0
        while t < tr.duration + 0.5 and be.enabled:
            be.command(tr.sample(t)[0])
            be.step(0.01)
            t += 0.01
        return peak, be

    peak, be = lift(arm, 1.0)
    assert peak > 1.1 and "J2" in be.faults and not be.enabled      # flagged, and it really stalls
    peak, be = lift(arm, 0.5)
    assert peak < 1.0 and not be.faults                             # same path, half speed: fine
    assert np.degrees(abs(be.read()[1] - up[1])) < 0.1              # (static load alone is ~0.9)
    peak, be = lift(flat, 1.0)
    assert peak < 1.0 and not be.faults                             # the old flat limit hid this


def test_estop_holds_unless_backdrivable():
    held = MujocoBackend(ARM, q0=STRETCHED, payload_kg=1.0)
    start = held.read()                                 # links start sagged onto the gear springs
    run(held, 1.0)                                      # never enabled = powered off
    assert np.allclose(held.read(), start) and np.allclose(held.read_motor(), STRETCHED)
    falls = MujocoBackend(ARM, q0=STRETCHED, payload_kg=1.0, backdrivable=True, compliant=False)
    run(falls, 1.0)
    assert np.degrees(falls.read()[1]) > 100            # dropped toward its joint limit
    falls.enable()                                      # servos back on: holds where it is
    q = falls.read()
    run(falls, 0.5)
    assert np.max(np.abs(np.degrees(falls.read() - q))) < 0.2


def test_trajectory_peak_flags_infeasible_moves():
    tm = TorqueModel(ARM, payload_kg=0.5)
    gentle = TrapezoidalTrajectory(ARM.home, ARM.home + np.deg2rad([20, 10, 0, 10, 0, 0, 0]),
                                   ARM.vmax, ARM.amax, speed_scale=0.5)
    assert tm.trajectory_peak(gentle)[0].max() < 1.0
    heavy = TorqueModel(ARM, payload_kg=5.0)
    swing = TrapezoidalTrajectory(ARM.home, STRETCHED, ARM.vmax, ARM.amax)
    peak, _ = heavy.trajectory_peak(swing)
    assert peak.max() > 1.0


# ---------------------------------------------------------------- gearbox compliance -------
def gear(i):
    return FULL.joints[i].gear_stiffness, FULL.joints[i].backlash


def test_gear_wind_up_matches_static_deflection():
    """Holding the arm out, each loaded link sags by tau / k + backlash / 2 behind its motor."""
    be = MujocoBackend(ARM, q0=STRETCHED, payload_kg=1.0)
    be.enable()
    run(be, 4.0)
    assert be.enabled and not be.faults
    tau = TorqueModel(ARM, payload_kg=1.0).gravity(STRETCHED)
    for i in (1, 3, 5):                                 # J2, J4, J6 carry the weight
        k, b = gear(i)
        expected = tau[i] / k + np.sign(tau[i]) * b / 2
        sag = be.read_motor()[i] - be.read()[i]
        assert abs(sag - expected) < 0.05 * abs(expected), (i, sag, expected)
    assert np.max(np.abs(np.degrees(be.read_motor() - STRETCHED))) < 0.1   # motors are on target


def test_backlash_and_stiffness_from_a_push_test():
    """Bench test on J1 (no gravity load): push +tau, then -tau. Hysteresis = 2 tau / k + backlash."""
    be = MujocoBackend(ARM, payload_kg=0.0)
    be.enable()
    k, b = gear(0)
    tau = 1.0
    be.external_torque[0] = tau
    run(be, 1.5)
    plus = be.read()[0] - be.read_motor()[0]
    be.external_torque[0] = -tau
    run(be, 1.5)
    minus = be.read()[0] - be.read_motor()[0]
    assert abs((plus - minus) - (2 * tau / k + b)) < 0.03 * (2 * tau / k + b)
    be.external_torque[0] = 0.0
    run(be, 1.5)                                        # released: stays inside the dead band
    rest = be.read()[0] - be.read_motor()[0]
    assert abs(rest) <= b / 2 + 1e-4


def test_compliant_arm_starts_in_static_equilibrium():
    """Power-up doesn't drop the links onto slack gear springs: nothing moves when enabled."""
    be = MujocoBackend(ARM, q0=STRETCHED, payload_kg=1.0)
    start = be.read()
    tau = ARM.gravity_torque(STRETCHED, 1.0)
    for i in (1, 3, 5):
        k, b = gear(i)
        assert (STRETCHED - start)[i] == pytest.approx(tau[i] / k + np.sign(tau[i]) * b / 2)
    be.enable()
    peak = np.zeros(ARM.n)
    for _ in range(100):
        be.step(0.01)
        peak = np.maximum(peak, be.torque_utilization)
    util = np.abs(tau) / ARM.torque_limit()
    # the only motion is the servo's own give: one motor step of lag at full torque
    servo_give = 1.8 / np.array([j.ratio for j in ARM.joints]) * util
    # an unloaded joint (J7 here: the tool board puts the gripper's centre 11 mm off its axis, so the others' sag
    # gives it a trace of torque) can drift across its dead band
    slack = np.where(np.abs(tau) < 1e-6, np.degrees([j.backlash for j in ARM.joints]), 0.0)
    assert np.all(np.abs(np.degrees(be.read() - start)) <= servo_give + slack + 0.005)
    assert np.all(peak <= util + 0.02)                  # no overshoot beyond the static load


def test_rigid_and_compliant_agree_on_joint_count():
    be = MujocoBackend(FULL)
    assert be.compliant and be.model.nq == 2 * FULL.n and len(be.read()) == FULL.n


# ---------------------------------------------------------------- counterbalance -----------
def test_zero_length_spring_torque_is_k_a_b_sin():
    cb = FULL.counterbalance
    for deg in (-90, -40, 0, 25, 90, 115):
        q = np.zeros(FULL.n)
        q[1] = np.deg2rad(deg)
        expected = -cb.stiffness * cb.anchor_height * cb.attach_distance * np.sin(q[1])
        assert spring_torque(FULL, q)[1] == pytest.approx(expected, abs=1e-9)
        assert np.count_nonzero(spring_torque(FULL, q)) <= 1


def test_spring_is_sized_for_the_worst_case_not_one_pose():
    cb = FULL.counterbalance
    r = size_spring(FULL, cb.anchor_height, cb.attach_distance, payload_kg=(0.0, 1.0))
    assert r["k"] == pytest.approx(cb.stiffness, rel=0.01)      # arm.toml holds the sized value
    assert r["worst_with"] < r["worst_without"]
    # minimax on fresh poses: k beats both a softer and a stiffer spring ...
    worst = worst_case_torque(FULL, reachable_poses(FULL, 300, seed=7), cb.anchor_height,
                              cb.attach_distance)
    assert worst(r["k"]) <= min(worst(0.7 * r["k"]), worst(1.3 * r["k"]))
    # ... including the "cancel the stretched arm with 0.5 kg" rate, which over-balances elsewhere
    k_one_pose = abs(TorqueModel(ARM, 0.5).gravity(STRETCHED)[1]) / (cb.anchor_height * cb.attach_distance)
    assert k_one_pose > r["k"] and worst(k_one_pose) > worst(r["k"])


def test_counterbalance_unloads_j2_in_the_physics_sim():
    loaded = MujocoBackend(ARM, q0=STRETCHED, payload_kg=0.5, compliant=False)
    helped = MujocoBackend(FULL, q0=STRETCHED, payload_kg=0.5, compliant=False)
    for be in (loaded, helped):
        be.enable()
        run(be, 1.0)
    # the sim's J2 motor torque is gravity minus spring, as the inverse-dynamics model predicts
    expect = abs(TorqueModel(FULL, payload_kg=0.5).gravity(STRETCHED)[1]) / helped.limits[1]
    assert helped.torque_utilization[1] == pytest.approx(expect, rel=0.05)
    assert helped.torque_utilization[1] < loaded.torque_utilization[1]

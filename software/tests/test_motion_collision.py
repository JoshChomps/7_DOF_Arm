"""S-curve and time-optimal (TOPP-RA) timing, and the self-collision check."""
import numpy as np
import pytest

from deskarm import TrapezoidalTrajectory, load_arm
from deskarm.cartesian import linear_move
from deskarm.collision import (PAIRS, capsules, min_clearance, segment_distance, self_collisions,
                               trajectory_collision)
from deskarm.ik import workspace_violations
from deskarm.program import Program
from deskarm.topp import ToppError, joint_path, time_optimal
from deskarm.trajectory import DoubleS, SCurveTrajectory, joint_move

ARM = load_arm()
STRETCHED = np.deg2rad([0, 90, 0, 0, 0, 0, 0])
TUCKED = np.deg2rad([0, -30, 0, 120, 0, 90, 0])      # gripper folded back into the shoulder


# ---------------------------------------------------------------- S-curve ------------------
@pytest.mark.parametrize("h, v, a, j", [(1, 1, 2, 10), (0.05, 1, 2, 10), (1e-3, 1, 2, 10), (10, 1, 2, 100)])
def test_double_s_respects_limits_in_every_regime(h, v, a, j):
    p = DoubleS(h, v, a, j)
    ts = np.linspace(0, p.duration, 20001)
    s = np.array([p.sample(t) for t in ts])
    assert s[-1] == pytest.approx([h, 0, 0])
    assert s[:, 1].max() <= v * (1 + 1e-9)
    assert np.abs(s[:, 2]).max() <= a * (1 + 1e-9)
    assert np.abs(np.diff(s[:, 2]) / np.diff(ts)).max() <= j * 1.001
    assert np.all(np.diff(s[:, 0]) >= -1e-12)                       # never goes backwards


def test_scurve_joint_move_is_synchronized_and_limited():
    q1 = ARM.home + np.deg2rad([60, -30, 40, 30, 90, -40, 180])
    tr = SCurveTrajectory(ARM.home, q1, ARM.vmax, ARM.amax, ARM.jmax)
    ts = np.linspace(0, tr.duration, 4001)
    v = np.array([tr.sample(t)[1] for t in ts])
    acc = np.array([tr.sample_acc(t) for t in ts])
    assert np.all(np.abs(v) <= ARM.vmax * (1 + 1e-9))
    assert np.all(np.abs(acc) <= ARM.amax * (1 + 1e-9))
    assert np.all(np.abs(np.diff(acc, axis=0) / np.diff(ts)[:, None]) <= ARM.jmax * 1.01)
    assert np.allclose(tr.sample(tr.duration)[0], q1)
    s = (np.array([tr.sample(t)[0] for t in ts]) - ARM.home) / (q1 - ARM.home)
    assert np.allclose(s, s[:, :1], atol=1e-9)                       # all joints in lockstep
    assert tr.duration >= TrapezoidalTrajectory(ARM.home, q1, ARM.vmax, ARM.amax).duration


# ---------------------------------------------------------------- TOPP-RA ------------------
@pytest.mark.parametrize("dq_deg", [[90, 0, 0, 0, 0, 0, 0], [5, 0, 0, 0, 0, 0, 0],
                                    [60, -30, 40, 30, 90, -40, 180], [90, 15, 0, 0, 0, 0, 0]])
@pytest.mark.parametrize("a_scale", [1.0, [50, 0.5, 1, 1, 1, 1, 1]])   # 2nd: J1 limits speed, J2 accel
def test_topp_matches_the_analytic_optimum_with_only_kinematic_limits(dq_deg, a_scale):
    """Straight joint path, box limits: optimum is bang-coast-bang in s with
    sdot <= min v_i/|dq_i| and sddot <= min a_i/|dq_i| (the two may come from different joints)."""
    from dataclasses import replace
    arm = replace(ARM, joints=tuple(replace(j, amax=j.amax * f)
                                    for j, f in zip(ARM.joints, np.broadcast_to(a_scale, (ARM.n,)))))
    q1 = arm.home + np.deg2rad(dq_deg)
    d = np.abs(q1 - arm.home)
    m = d > 0
    vs, as_ = np.min(arm.vmax[m] / d[m]), np.min(arm.amax[m] / d[m])
    t_opt = 1 / vs + vs / as_ if vs * vs < as_ else 2 / np.sqrt(as_)
    tr, info = time_optimal(arm, joint_path(arm.home, q1), grid=100)
    # TOPP-RA is exact up to its grid: an acceleration phase shorter than one cell costs ~1 cell
    assert t_opt - 1e-6 <= tr.duration <= t_opt + max(1e-3 * t_opt, 1.5 * (1 / 100) / vs)
    assert np.allclose(tr.sample(tr.duration)[0], q1)
    v = np.diff(tr.qs, axis=0) / np.diff(tr.times)[:, None]
    assert np.all(np.abs(v) <= arm.vmax * 1.001)
    # per-joint synchronized trapezoids leave the straight line, so they can only be as fast or faster
    assert TrapezoidalTrajectory(arm.home, q1, arm.vmax, arm.amax).duration <= tr.duration * (1 + 1e-3)


def test_topp_respects_motor_torque_with_a_heavy_payload():
    pytest.importorskip("mujoco")
    from deskarm.physics import TorqueModel
    tm = TorqueModel(ARM, payload_kg=2.0)
    a, b = np.deg2rad([-90, 90, 0, 0, 0, 0, 0]), np.deg2rad([90, 90, 0, 0, 0, 0, 0])
    fast, info = time_optimal(ARM, joint_path(a, b), torque_model=tm, accel_limits=False)
    assert tm.trajectory_peak(fast)[0].max() < 0.98                 # within the 95 % margin (+ grid)
    kin, _ = time_optimal(ARM, joint_path(a, b))
    assert fast.duration < kin.duration                             # torque allows more than amax here
    # payload at which the weakest joint can no longer hold the stretched arm (spring included)
    g0, g1 = (np.abs(TorqueModel(ARM, m).gravity(STRETCHED)) for m in (0.0, 1.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        kg = np.min(np.where(g1 - g0 > 1e-9, (ARM.torque_limit() - g0) / (g1 - g0), np.inf))
    with pytest.raises(ToppError):                                  # can't even hold that out there
        time_optimal(ARM, joint_path(ARM.home, STRETCHED), torque_model=TorqueModel(ARM, 1.2 * kg))
    time_optimal(ARM, joint_path(ARM.home, STRETCHED), torque_model=TorqueModel(ARM, 0.8 * kg))


def test_topp_slows_down_where_the_motor_runs_out_of_torque_at_speed():
    """A stepper's torque falls with speed. Lifting a heavy payload, the time-optimal profile must
    keep J2 below the speed at which its motor can still carry the load, and stay within the
    speed-dependent limit everywhere. With a flat limit of the same standstill torque it would
    simply run at vmax."""
    pytest.importorskip("mujoco")
    from dataclasses import replace
    from deskarm.physics import TorqueModel
    bare = replace(ARM, counterbalance=None)
    fast_j2 = replace(bare.joints[1], vmax=np.deg2rad(90))
    arm = replace(bare, joints=bare.joints[:1] + (fast_j2,) + bare.joints[2:])
    flat = replace(arm, joints=tuple(replace(j, torque_curve=()) for j in arm.joints))
    low, up = np.deg2rad([0, 115, 0, 0, 0, 0, 0]), np.deg2rad([0, 35, 0, 0, 0, 0, 0])
    payload = 2.3                    # at 2.0 kg the NEMA23 is acceleration-limited (2.4 before the TCP moved to 0.11, 20 Oct)

    tm = TorqueModel(arm, payload)
    tr, info = time_optimal(arm, joint_path(low, up), torque_model=tm)
    assert tm.trajectory_peak(tr)[0].max() < 1.0                    # feasible at every speed it uses
    assert info.active == "torque"
    v = np.abs(np.diff(tr.qs, axis=0) / np.diff(tr.times)[:, None])[:, 1]
    q_mid = (tr.qs[:-1] + tr.qs[1:]) / 2
    load = np.array([abs(arm.gravity_torque(q, payload)[1]) for q in q_mid])
    limit = np.array([arm.joints[1].torque_at(s) for s in v])
    assert np.all(limit >= load)                                    # never faster than the load allows
    assert v.max() < 0.9 * arm.vmax[1]                              # so it can't reach vmax here

    tr_flat, _ = time_optimal(flat, joint_path(low, up), torque_model=TorqueModel(flat, payload))
    v_flat = np.abs(np.diff(tr_flat.qs, axis=0) / np.diff(tr_flat.times)[:, None])[:, 1]
    assert v_flat.max() > 0.99 * flat.vmax[1] and tr_flat.duration < tr.duration
    # and the profile that ignored the curve is not actually executable
    assert tm.trajectory_peak(tr_flat)[0].max() > 1.1


def _line_deviation(traj, p0, p1, samples=200):
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    return max(np.linalg.norm((v := ARM.fk(traj.sample(t)[0])[:3, 3] - p0) - (v @ d) * d)
               for t in np.linspace(0, traj.duration, samples))


@pytest.mark.parametrize("profile", ["scurve", "time-optimal"])
def test_linear_move_profiles_stay_on_the_line(profile):
    T0 = ARM.fk(ARM.home)
    T1 = T0.copy()
    T1[:3, 3] += [0.0, 0.15, -0.05]
    tr, info = linear_move(ARM, ARM.home, T1, v_lin=0.1, profile=profile)
    assert info.profile == profile
    assert _line_deviation(tr, T0[:3, 3], T1[:3, 3]) < 1e-4
    assert np.linalg.norm(ARM.fk(tr.sample(tr.duration)[0])[:3, 3] - T1[:3, 3]) < 1e-4
    v = np.diff(tr.qs, axis=0) / np.diff(tr.times)[:, None]
    assert np.all(np.abs(v) <= ARM.vmax * 1.001)
    trap, _ = linear_move(ARM, ARM.home, T1, v_lin=0.1)
    if profile == "scurve":
        assert trap.duration < tr.duration < 1.3 * trap.duration
    else:
        assert tr.duration <= trap.duration                          # same speed cap, no tool accel cap
        tool_speed = np.linalg.norm(np.diff([ARM.fk(q)[:3, 3] for q in tr.qs], axis=0), axis=1) / np.diff(tr.times)
        assert tool_speed.max() <= 0.1 * 1.02


def test_program_and_joint_move_accept_every_profile():
    prog = Program()
    prog.add(ARM.home + np.deg2rad([20, 0, 0, 0, 0, 0, 0]), "joint", speed=1.0)
    prog.add(ARM.home + np.deg2rad([-15, 5, 0, -5, 0, 0, 0]), "linear", speed=1.0)
    for profile in ("trapezoid", "scurve", "time-optimal"):
        trajs = prog.build(ARM, ARM.home, profile=profile)
        assert np.allclose(trajs[0].sample(trajs[0].duration)[0], prog.waypoints[0].q, atol=1e-9)
        tr = joint_move(ARM, ARM.home, STRETCHED, profile)
        assert np.allclose(tr.sample(tr.duration)[0], STRETCHED, atol=1e-9)
    with pytest.raises(ValueError):
        joint_move(ARM, ARM.home, STRETCHED, "bang-bang")


# ---------------------------------------------------------------- self-collision -----------
def test_segment_distance_matches_brute_force():
    rng = np.random.default_rng(3)
    t = np.linspace(0, 1, 201)
    for _ in range(50):
        p1, q1, p2, q2 = rng.normal(size=(4, 3))
        if rng.random() < 0.2:
            q2 = p2 + (q1 - p1) * rng.random()                      # parallel segments
        A = p1 + t[:, None] * (q1 - p1)
        B = p2 + t[:, None] * (q2 - p2)
        brute = np.min(np.linalg.norm(A[:, None] - B[None], axis=2))
        assert segment_distance(p1, q1, p2, q2) == pytest.approx(brute, abs=5e-3)
        assert segment_distance(p1, q1, p2, q2) <= brute + 1e-12


def test_normal_poses_are_clear():
    for q in (ARM.home, STRETCHED, np.zeros(ARM.n), np.deg2rad([0, 45, 0, 90, 0, 45, 0])):
        assert self_collisions(ARM, q) == [], q
    assert min_clearance(ARM, ARM.home)[0] > 0.02                   # closest: the tower beside the upper arm (CAD, 30 mm)


def test_folding_the_tool_back_is_flagged():
    assert workspace_violations(ARM, TUCKED) == []                  # the desk check can't see this
    hits = self_collisions(ARM, TUCKED)
    assert any("tool" in h and "shoulder" in h for h in hits)
    tr = TrapezoidalTrajectory(ARM.home, TUCKED, ARM.vmax, ARM.amax)
    t, msg = trajectory_collision(ARM, tr)
    assert 0 < t <= tr.duration and msg
    assert len(capsules(ARM, ARM.home)) == 7 and len(PAIRS) == 10


def test_most_of_the_joint_space_is_collision_free():
    rng = np.random.default_rng(0)
    bad = sum(bool(self_collisions(ARM, rng.uniform(ARM.lower, ARM.upper), margin=0.0)) for _ in range(500))
    assert 0 < bad < 0.30 * 500        # flags some, not everything (25% with the 3-prong gripper's r 50 tool capsule; 19% at r 35)

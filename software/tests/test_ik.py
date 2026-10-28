import numpy as np

from deskarm import load_arm
from deskarm.ik import (J3, analytic_ik_6dof, analytic_ik_7dof, feasible_psi, pose_error, solve_dls, solve_ik,
                        supports_analytic, swivel, swivel_angle)

ARM = load_arm()
RNG = np.random.default_rng(1)


def random_q(margin=0.8, j3_zero=False):
    lo, hi = ARM.lower * margin, ARM.upper * margin
    q = lo + (hi - lo) * RNG.random(ARM.n)
    q[6] = np.clip(q[6], -np.pi, np.pi)
    if j3_zero:
        q[J3] = 0.0
    return q


def err(T_target, q):
    e = pose_error(T_target, ARM.fk(q))
    return np.linalg.norm(e[:3]), np.linalg.norm(e[3:])


def test_bundled_arm_supports_analytic():
    assert supports_analytic(ARM)


def test_analytic_6dof_round_trip():
    hits = 0
    for _ in range(300):
        q_true = random_q(j3_zero=True)
        T = ARM.fk(q_true)
        sols = analytic_ik_6dof(ARM, T, q_true)
        assert sols, f"no solution for reachable pose {q_true}"
        for s in sols:
            pe, re = err(T, s)
            assert pe < 1e-6 and re < 1e-6
            assert s[J3] == 0.0 and ARM.within_limits(s)
        hits += np.allclose(sols[0], q_true, atol=1e-6)
    assert hits > 250  # nearest solution is almost always the original configuration


def test_analytic_rejects_out_of_reach():
    T = np.eye(4)
    T[:3, 3] = [1.0, 0, 0.2]  # 1 m away, arm reach is ~0.52 m
    assert analytic_ik_6dof(ARM, T, ARM.home) == []


def test_dls_7dof_round_trip_from_nearby_start():
    for _ in range(50):
        q_true = random_q()
        q0 = ARM.clamp(q_true + RNG.normal(0, 0.15, ARM.n))
        res = solve_dls(ARM, ARM.fk(q_true), q0)
        assert res.success, res
        assert ARM.within_limits(res.q)


def test_solve_ik_7dof_from_home_uses_analytic_seed_when_needed():
    ok = 0
    for _ in range(50):
        T = ARM.fk(random_q())
        res = solve_ik(ARM, T, ARM.home, mode="7dof")
        ok += res.success
    assert ok >= 48


def test_locked_joint_stays_put():
    for _ in range(30):
        q_true = random_q()
        q0 = ARM.clamp(q_true + RNG.normal(0, 0.15, ARM.n))
        q0[0] = q_true[0]                       # J1 locked at its true value
        res = solve_dls(ARM, ARM.fk(q_true), q0, locked={0})
        assert res.q[0] == q0[0]
        assert res.success                      # 6 free joints remain -> full pose reachable


def test_6dof_mode_keeps_j3_zero():
    T = ARM.fk(random_q(j3_zero=True))
    res = solve_ik(ARM, T, ARM.home, mode="6dof")
    assert res.success and res.q[J3] == 0.0 and res.message == "analytic"


def test_swivel_moves_elbow_but_not_tool():
    q = ARM.home.copy()
    T0 = ARM.fk(q)
    psi0 = swivel_angle(ARM, q)
    res = swivel(ARM, q, psi0 + np.deg2rad(30))
    assert res.success, res.message
    pe, re = err(T0, res.q)
    assert pe < 1e-4 and re < 1e-3
    assert abs(swivel_angle(ARM, res.q) - (psi0 + np.deg2rad(30))) < 1e-2


def test_swivel_angle_is_continuous_over_the_top():
    # the wrist passes straight above the shoulder near q2 = -29 deg; the old "projected +z" angle jumped 180 deg there
    psis = [swivel_angle(ARM, np.deg2rad([0, q2, 0, 60, 0, 0, 0])) for q2 in np.arange(-40, -18, 0.25)]
    assert not np.isnan(psis).any()
    assert np.max(np.abs(np.diff(np.unwrap(psis)))) < np.deg2rad(1)
    assert np.max(np.abs(np.diff(psis))) < np.deg2rad(1)          # no wrap either: it stays near 0


def test_swivel_angle_reference():
    # forward reach, elbow up: 0; J3 swings it right-handed about S->W; turning J1 turns the reading with it (the
    # price of a single singular direction, docs/14 §6)
    q = np.deg2rad([0, 45, 0, 80, 0, 40, 0])
    assert abs(swivel_angle(ARM, q)) < 1e-9
    assert swivel_angle(ARM, q + np.deg2rad([0, 0, 30, 0, 0, 0, 0])) > 0
    for q1 in (45, 90, -90):
        assert abs(np.rad2deg(swivel_angle(ARM, q + np.deg2rad([q1, 0, 0, 0, 0, 0, 0]))) - q1) < 1e-6


def test_swivel_angle_singular_only_straight_down():
    from deskarm.ik import E_R, E_T
    for d in RNG.normal(size=(500, 3)):
        u = d / np.linalg.norm(d)
        if u @ E_T > 0.999:
            continue
        k = np.cross(np.cross(u - E_T, E_R), u)
        assert np.linalg.norm(k) > 1e-4                             # reference defined away from E_T


def wrapped(a):
    return np.abs(np.angle(np.exp(1j * np.asarray(a))))


def test_analytic_7dof_recovers_random_configurations():
    for _ in range(100):
        q = random_q(0.9)
        T, psi = ARM.fk(q), swivel_angle(ARM, q)
        sols = analytic_ik_7dof(ARM, T, psi, q)
        assert sols and np.max(wrapped(sols[0] - q)) < 1e-8          # nearest first, and it's the original
        for s in sols:
            assert np.linalg.norm(pose_error(T, ARM.fk(s))) < 1e-6
            assert wrapped(swivel_angle(ARM, s) - psi) < 1e-6
            assert ARM.within_limits(s)


def test_analytic_7dof_matches_dls():
    # whatever DLS finds (solved tight, clear of the limits), the closed form at DLS's arm angle finds the same joints
    margin = np.deg2rad(5)
    n = 0
    for _ in range(40):
        T = ARM.fk(random_q(0.8))
        res = solve_dls(ARM, T, random_q(0.8), pos_tol=1e-9, rot_tol=1e-9, max_iter=2000)
        if not res.success or np.any(res.q < ARM.lower + margin) or np.any(res.q > ARM.upper - margin):
            continue
        sols = analytic_ik_7dof(ARM, T, swivel_angle(ARM, res.q), res.q)
        assert sols and np.max(wrapped(sols[0] - res.q)) < 1e-6
        n += 1
    assert n >= 10


def test_analytic_7dof_edge_cases():
    T = ARM.fk(ARM.home)
    T_far = T.copy()
    T_far[:3, 3] = [2.0, 0.0, 0.5]
    assert analytic_ik_7dof(ARM, T_far, 0.0) == []                  # out of reach
    T_straight = ARM.fk(np.deg2rad([0, 30, 0, 0, 0, 30, 0]))
    assert analytic_ik_7dof(ARM, T_straight, 0.0) == []             # straight elbow: psi undefined
    psis = feasible_psi(ARM, T)
    assert len(psis) and np.min(wrapped(psis - swivel_angle(ARM, ARM.home))) < np.deg2rad(2)


def test_solve_ik_with_psi_and_fallback():
    T = ARM.fk(ARM.home)
    res = solve_ik(ARM, T, ARM.home, psi=np.deg2rad(40))
    assert res.success and res.message == "analytic (psi)"
    assert wrapped(swivel_angle(ARM, res.q) - np.deg2rad(40)) < 1e-6
    try:
        solve_ik(ARM, T, ARM.home, locked={0}, psi=0.0)
        raise AssertionError("psi with a locked joint should raise")
    except ValueError:
        pass
    ok = 0
    for _ in range(30):                                              # far seeds: DLS, then the closed form
        ok += solve_ik(ARM, ARM.fk(random_q(0.9)), random_q(0.9)).success
    assert ok == 30


def test_singularity_warnings():
    from deskarm.ik import singularity_warnings
    assert singularity_warnings(ARM, ARM.home) == []
    q = ARM.home.copy(); q[5] = 0.0
    assert any("wrist" in w for w in singularity_warnings(ARM, q))
    q = ARM.home.copy(); q[3] = 0.0
    assert any("elbow" in w for w in singularity_warnings(ARM, q))
    assert any("shoulder" in w for w in singularity_warnings(ARM, np.zeros(ARM.n)))  # straight up


# ------------------------------------------------------------------ constraint tasks / desk ----
def test_position_task_ignores_orientation():
    from deskarm.ik import solve_ik
    for _ in range(20):
        T = ARM.fk(random_q())
        T_wrong = T.copy()
        T_wrong[:3, :3] = np.eye(3)                 # unreachable orientation, but we don't care
        res = solve_ik(ARM, T_wrong, ARM.home, task="position")
        assert res.success, res
        assert np.linalg.norm(ARM.fk(res.q)[:3, 3] - T[:3, 3]) < 1e-4


def test_axis_task_keeps_tool_vertical_with_free_spin():
    from deskarm.ik import solve_ik
    down = np.array([0.0, 0.0, -1.0])
    # wrist centre ends up 0.15 m above the target; all of these keep it inside the 0.37 m reach
    for x, y, z in [(0.25, 0.0, 0.15), (0.2, 0.15, 0.10), (0.0, -0.22, 0.12), (-0.15, 0.15, 0.18)]:
        T = np.eye(4)
        T[:3, :3] = np.diag([1.0, -1.0, -1.0])      # z axis pointing down; spin left free
        T[:3, 3] = [x, y, z]
        for mode in ("7dof", "6dof"):
            res = solve_ik(ARM, T, ARM.home, mode=mode, task="axis")
            assert res.success, (mode, (x, y, z), res)
            Tq = ARM.fk(res.q)
            assert np.allclose(Tq[:3, 3], [x, y, z], atol=1e-4)
            assert np.degrees(np.arccos(np.clip(Tq[:3, 2] @ down, -1, 1))) < 0.1
    T[:3, 3] = [-0.2, 0.2, ARM.joints[0].d + 0.12]   # wrist centre 0.39 m from the shoulder: too far
    assert not solve_ik(ARM, T, ARM.home, task="axis").success


def test_axis_task_with_locked_joint():
    from deskarm.ik import solve_dls
    q0 = ARM.home.copy()
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = [0.22, 0.05, 0.15]
    res = solve_dls(ARM, T, q0, locked={0}, task="axis")   # J1 fixed: 6 joints for 5 constraints
    assert res.q[0] == q0[0]


def test_workspace_violations_and_trajectory_check():
    from deskarm import TrapezoidalTrajectory
    from deskarm.ik import trajectory_violation, workspace_violations
    assert workspace_violations(ARM, ARM.home) == []
    q_down = ARM.home.copy()
    q_down[1] = np.deg2rad(110)                      # shoulder pitched far forward: arm into desk
    q_down[3] = np.deg2rad(60)
    bad = workspace_violations(ARM, q_down)
    assert "TCP" in bad
    traj = TrapezoidalTrajectory(ARM.home, q_down, ARM.vmax, ARM.amax)
    hit = trajectory_violation(ARM, traj)
    assert hit is not None and 0 < hit[0] <= traj.duration
    safe = TrapezoidalTrajectory(ARM.home, ARM.home + np.deg2rad([30, 0, 0, 0, 0, 0, 0]), ARM.vmax, ARM.amax)
    assert trajectory_violation(ARM, safe) is None

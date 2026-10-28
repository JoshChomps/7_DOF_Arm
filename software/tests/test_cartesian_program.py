import json

import numpy as np
import pytest

from deskarm import load_arm
from deskarm.cartesian import CartesianError, interpolate_pose, linear_move
from deskarm.ik import rotation_log
from deskarm.program import Program, tcp_path

ARM = load_arm()


def _line_deviation(traj, p0, p1, samples=200):
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    worst = 0.0
    for t in np.linspace(0, traj.duration, samples):
        v = ARM.fk(traj.sample(t)[0])[:3, 3] - p0
        worst = max(worst, np.linalg.norm(v - (v @ d) * d))
    return worst


def test_linear_move_is_straight_and_within_limits():
    T0 = ARM.fk(ARM.home)
    T1 = T0.copy()
    T1[:3, 3] += [0.0, 0.15, -0.05]
    tr, info = linear_move(ARM, ARM.home, T1, v_lin=0.1)
    assert _line_deviation(tr, T0[:3, 3], T1[:3, 3]) < 1e-4            # < 0.1 mm off the line
    assert np.linalg.norm(ARM.fk(tr.sample(tr.duration)[0])[:3, 3] - T1[:3, 3]) < 1e-4
    assert np.allclose(ARM.fk(tr.sample(tr.duration)[0])[:3, :3], T1[:3, :3], atol=1e-4)
    assert info.slowed_by == pytest.approx(1.0)                         # tool limits governed
    assert info.duration == pytest.approx(0.158 / 0.1 + 0.1 / 0.4, rel=0.02)
    v = np.diff(tr.qs, axis=0) / np.diff(tr.times)[:, None]
    assert np.all(np.abs(v) <= ARM.vmax + 1e-9)


def test_linear_move_with_rotation_and_6dof_mode():
    T0 = ARM.fk(ARM.home)
    T1 = interpolate_pose(T0, T0, 0.0)
    T1[:3, 3] += [-0.04, -0.05, -0.03]                 # inward: stays inside the reach
    from deskarm.cartesian import rotation_exp
    T1[:3, :3] = T0[:3, :3] @ rotation_exp(np.array([0.0, 0.0, 0.6]))
    for mode in ("7dof", "6dof"):
        tr, _ = linear_move(ARM, ARM.home, T1, mode=mode)
        Te = ARM.fk(tr.sample(tr.duration)[0])
        assert np.linalg.norm(Te[:3, 3] - T1[:3, 3]) < 1e-4
        assert np.linalg.norm(rotation_log(Te[:3, :3].T @ T1[:3, :3])) < 1e-3
        if mode == "6dof":
            assert np.allclose(tr.qs[:, 2], 0.0)                         # J3 stays locked


def test_linear_move_fast_request_is_slowed_by_joint_limits():
    T0 = ARM.fk(ARM.home)
    T1 = T0.copy()
    T1[:3, 3] += [-0.06, 0.12, 0.0]                                    # stays inside the reach
    tr, info = linear_move(ARM, ARM.home, T1, v_lin=5.0, a_lin=50.0)   # absurd tool speed
    assert info.slowed_by > 1.0
    v = np.diff(tr.qs, axis=0) / np.diff(tr.times)[:, None]
    assert np.all(np.abs(v) <= ARM.vmax * 1.001)


def test_linear_move_rejects_unreachable_line_and_says_why():
    T0 = ARM.fk(ARM.home)
    T1 = T0.copy()
    T1[:3, 3] = [0.6, 0.0, ARM.joints[0].d + 0.17]                      # 0.62 m from the shoulder: beyond 0.52 m reach
    with pytest.raises(CartesianError, match="elbow singularity"):
        linear_move(ARM, ARM.home, T1)


def test_program_round_trip_and_build(tmp_path):
    prog = Program(arm_name=ARM.name)
    q1 = ARM.home + np.deg2rad([30, 0, 0, 10, 0, 0, 0])
    q2 = ARM.home + np.deg2rad([-20, 5, 0, -10, 0, 10, 0])
    prog.add(q1, "joint", speed=0.8)
    prog.add(q2, "linear", speed=0.5)
    path = prog.save(tmp_path / "p.json")
    data = json.loads(path.read_text())
    assert data["units"] == "rad" and len(data["waypoints"]) == 2
    loaded = Program.load(path)
    assert [w.move for w in loaded.waypoints] == ["joint", "linear"]
    trajs = loaded.build(ARM, ARM.home)
    assert len(trajs) == 2
    assert np.allclose(trajs[0].sample(trajs[0].duration)[0], q1)
    end = ARM.fk(trajs[1].sample(trajs[1].duration)[0])
    assert np.linalg.norm(end[:3, 3] - ARM.fk(q2)[:3, 3]) < 1e-4       # same pose (7-DOF may differ in q)
    pts = tcp_path(ARM, trajs, samples_per_traj=10)
    assert pts.shape == (22, 3)


def test_waypoint_rejects_bad_move_type():
    with pytest.raises(ValueError):
        Program().add(ARM.home, "teleport")

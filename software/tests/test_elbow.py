import numpy as np
import pytest

from deskarm import load_arm
from deskarm.cartesian import CartesianError, interpolate_pose, linear_move, linear_path
from deskarm.collision import min_clearance
from deskarm.elbow import plan_arm_angle
from deskarm.ik import pose_error, swivel_angle

ARM = load_arm()


def line(q0, d):
    T0 = ARM.fk(q0)
    T1 = T0.copy()
    T1[:3, 3] += d
    return T1


def test_plan_steers_the_elbow_round_a_collision():
    # found in a random sweep (docs/14 §6 Step 5): warm-started IK drives the J7 motor into the J2 module
    q0 = np.deg2rad([114.14, 74.19, -64.1, 78.94, -73.63, 1.09, -1.32])
    T1 = line(q0, [-0.0563, -0.103, 0.0966])
    kept, *_ = linear_path(ARM, q0, T1, elbow="keep")
    assert min(min_clearance(ARM, q)[0] for q in kept) < 0
    planned, _, _, how = linear_path(ARM, q0, T1)
    assert how == "planned"
    assert min(min_clearance(ARM, q)[0] for q in planned) > 0.01
    T0 = ARM.fk(q0)
    s = np.linspace(0, 1, len(planned))
    assert max(np.linalg.norm(pose_error(interpolate_pose(T0, T1, si), ARM.fk(q))) for si, q in zip(s, planned)) < 1e-9
    assert np.max(np.abs(np.diff(planned, axis=0))) <= 0.15


def test_plan_holds_the_elbow_when_nothing_is_in_the_way():
    T1 = line(ARM.home, [0, 0.15, 0])
    qs, _, _, how = linear_path(ARM, ARM.home, T1)
    psi = [swivel_angle(ARM, q) for q in qs]
    assert how == "planned" and np.max(np.abs(np.unwrap(psi) - psi[0])) < np.deg2rad(1)


def test_plan_falls_back_or_refuses():
    T1 = line(ARM.home, [0, 0.1, 0])
    assert linear_path(ARM, ARM.home, T1, mode="6dof")[3] == "warm-started"
    assert linear_path(ARM, ARM.home, T1, locked={0})[3] == "warm-started"
    _, info = linear_move(ARM, ARM.home, T1)
    assert info.elbow == "planned"
    far = line(ARM.home, [0, 0.1, 0.12])                       # runs the elbow straight: out of reach
    n = 40
    assert plan_arm_angle(ARM, ARM.home, [interpolate_pose(ARM.fk(ARM.home), far, s) for s in np.linspace(0, 1, n)]) is None
    with pytest.raises(CartesianError):
        linear_move(ARM, ARM.home, far)

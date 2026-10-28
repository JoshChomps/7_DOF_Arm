import numpy as np
import pytest

from deskarm import load_arm
from deskarm.kinematics import ArmModel

ARM = load_arm()
RNG = np.random.default_rng(0)


def random_q(n=1, margin=0.9):
    lo, hi = ARM.lower * margin, ARM.upper * margin
    return lo + (hi - lo) * RNG.random((n, ARM.n))


def test_zero_pose_points_straight_up():
    T = ARM.fk(np.zeros(ARM.n))
    expected_z = sum(j.d for j in ARM.joints) + ARM.tool[2, 3]
    assert np.allclose(T[:3, 3], [0, 0, expected_z])
    assert np.allclose(T[:3, 2], [0, 0, 1])  # tool axis up


def test_reach_matches_requirements():
    # J2 axis -> flange = 190 + 180 + 70 mm (docs/01_requirements.md)
    assert sum(j.d for j in ARM.joints[2:]) == pytest.approx(0.440)


def test_jacobian_matches_finite_differences():
    for q in random_q(20):
        J = ARM.jacobian(q)
        eps = 1e-7
        for i in range(ARM.n):
            dq = np.zeros(ARM.n)
            dq[i] = eps
            Tp, Tm = ARM.fk(q + dq), ARM.fk(q - dq)
            v = (Tp[:3, 3] - Tm[:3, 3]) / (2 * eps)
            dR = (Tp[:3, :3] - Tm[:3, :3]) / (2 * eps) @ ARM.fk(q)[:3, :3].T
            w = np.array([dR[2, 1], dR[0, 2], dR[1, 0]])
            assert np.allclose(J[:3, i], v, atol=1e-6)
            assert np.allclose(J[3:, i], w, atol=1e-6)


def test_srs_points_are_shoulder_elbow_wrist():
    q = np.zeros(ARM.n)
    S, E, W = ARM.srs_points(q)
    d = [j.d for j in ARM.joints]
    assert np.allclose(S, [0, 0, d[0]])
    assert np.allclose(E, [0, 0, d[0] + d[2]])
    assert np.allclose(W, [0, 0, d[0] + d[2] + d[4]])


def test_load_arm_from_explicit_path(tmp_path):
    from importlib import resources
    text = resources.files("deskarm").joinpath("data/arm.toml").read_text()
    p = tmp_path / "arm.toml"
    p.write_text(text.replace("tcp_offset_z = 0.11", "tcp_offset_z = 0.0"))
    arm = load_arm(p)
    assert isinstance(arm, ArmModel) and arm.n == 7 and arm.tool[2, 3] == 0.0

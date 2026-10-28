"""The actuator bench tools (deskarm.bench) against a simulated driver with a simulated output encoder."""
import numpy as np
import pytest

pytest.importorskip("can")

from deskarm import bench, mks  # noqa: E402
from deskarm.backends import CanBackend  # noqa: E402
from deskarm.bridge import JointAngle  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += max(s, 1e-3)


def rig(joint="J2"):
    arm, clock = bench.bench_arm(joint), Clock()
    cid = int(joint[1:])
    bus = mks.SimBus([cid], clock=clock, encoders={cid: arm.joints[0].ratio})
    backend = CanBackend(arm, bus, np.zeros(1), ids=[cid])
    backend.enable()
    return arm, bus, backend, clock


def test_push_fit_recovers_a_line():
    k, b = 600.0, np.radians(0.5)
    tau = np.array([2.0, 4.0, 6.0, 8.0])
    assert bench.push_fit(tau, 2 * tau / k + b) == pytest.approx((k, b))


def test_bench_arm_is_one_joint_with_its_drive_data():
    arm = bench.bench_arm("J2")
    assert arm.n == 1 and arm.joints[0].ratio == 48 and arm.joints[0].driver == "S57D_CAN"
    with pytest.raises(ValueError):
        bench.bench_arm("J9")


def test_output_unwraps_across_the_zero():
    out = bench.Output()
    for turn, seq in ((0.99, 1), (0.995, 2), (0.005, 3)):
        out.update(JointAngle(turn, 0, True, True, seq))
    assert out.angle == pytest.approx(0.015 * 2 * np.pi)
    out.update(JointAngle(0.5, 2, True, True, 4))                 # weak field: ignored
    assert out.angle == pytest.approx(0.015 * 2 * np.pi)


def test_sine_log_tracks_and_records_the_output():
    _, _, backend, clock = rig()
    amp, w = np.radians(10), 2 * np.pi / 4
    rows = bench.run(backend, lambda t: amp * np.sin(w * t), 4.0, 0.02, clock=clock, sleep=clock.sleep)
    t, cmd, motor, out, ok, _ = map(np.array, zip(*rows[5:]))
    assert ok.all() and np.abs(out.astype(float) - motor.astype(float)).max() < 0.05   # 14-bit steps
    assert np.abs(cmd.astype(float) - motor.astype(float)).max() < 1.0                  # one step behind


def test_push_test_recovers_the_gearbox():
    arm, bus, backend, clock = rig()
    k, b = 600.0, np.radians(0.5)

    def hang(text, tau):
        bus.deflection[2] = tau / k + np.sign(tau) * b / 2

    k_fit, b_fit, rows = bench.push_test(backend, [2, 4, 6, 8], hang, out=lambda s: None,
                                         clock=clock, sleep=clock.sleep)
    assert len(rows) == 4
    assert k_fit == pytest.approx(k, rel=0.05) and b_fit == pytest.approx(b, rel=0.1)   # 14-bit limited


def test_push_test_needs_the_output_encoder():
    arm, clock = bench.bench_arm("J2"), Clock()
    bus = mks.SimBus([2], clock=clock)                            # no encoder
    backend = CanBackend(arm, bus, np.zeros(1), ids=[2])
    backend.enable()
    with pytest.raises(RuntimeError, match="output-encoder"):
        bench.push_test(backend, [2], lambda text, tau: None, out=lambda s: None, clock=clock, sleep=clock.sleep)

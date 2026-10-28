"""DualLoop: the outer loop on the link angle (docs/14 §19), against a stub and the MuJoCo arm."""
from dataclasses import replace

import numpy as np
import pytest

from deskarm import TrapezoidalTrajectory, load_arm
from deskarm.control import DualLoop
from deskarm.counterbalance import spring_torque

FULL = load_arm()
STRETCHED = np.deg2rad([0, 90, 0, 0, 0, 0, 0])
LSB = 2 * np.pi / 16384


class Stub:
    """A backend whose link is wherever the test puts it; records the motor-side commands."""

    def __init__(self, arm, link):
        self.arm, self.link, self.sent, self.enabled, self.faults = arm, np.array(link, float), [], False, {}

    def enable(self):
        self.enabled = True

    def estop(self):
        self.enabled = False

    def command(self, q):
        self.sent.append(np.array(q))

    def read(self):
        return self.link.copy()

    def read_motor(self):
        return self.link.copy()

    def step(self, dt):
        pass


def test_feedforward_is_the_static_sag_and_only_on_encoder_joints():
    stub = Stub(FULL, STRETCHED)
    loop = DualLoop(stub, payload_kg=1.0)
    loop.ki = 0.0
    loop.enable()
    loop.command(STRETCHED)
    loop.step(0.02)
    tau = FULL.gravity_torque(STRETCHED, 1.0) - spring_torque(FULL, STRETCHED)
    k, b = np.array([j.gear_stiffness for j in FULL.joints]), np.array([j.backlash for j in FULL.joints])
    sag = tau / k + np.sign(tau) * b / 2                           # docs/14 §12 Step 2, where |tau| is past the ramp
    offset = stub.sent[-1] - STRETCHED
    assert np.allclose(offset[[1, 2, 3]], sag[[1, 2, 3]]) and np.all(offset[[0, 4, 5, 6]] == 0)
    assert abs(tau[2]) > 1.0                                        # J3 too: the CAD's J4/J6 modules sit to one side


def test_integral_waits_for_the_command_to_stop_and_is_clamped():
    stub = Stub(FULL, FULL.home)
    loop = DualLoop(stub, feedforward=False)
    loop.enable()
    for t in range(50):                                            # a moving command: no integration
        loop.command(FULL.home + np.radians(0.1 * t) * np.eye(7)[1])
        loop.step(0.02)
    assert not loop.integral.any()
    stub.link[1] -= 0.5                                            # held still, link far off (a broken pin)
    for _ in range(1000):
        loop.step(0.02)
    assert 0 < loop.integral[1] == pytest.approx(loop._bound[1])  # clamped to what the gearbox can wind up
    assert not loop.integral[[0, 4, 5, 6]].any()


# ------------------------------------------------------------------------- MuJoCo arm -----
pytest.importorskip("mujoco")
from deskarm.physics import MujocoBackend  # noqa: E402


class Encoder:
    """The MT6701 as CanBackend sees it: 14 bits, one poll old."""

    def __init__(self, be):
        self.be, self.last = be, be.read()

    def __getattr__(self, name):
        return getattr(self.be, name)

    def read(self):
        return np.round(self.last / LSB) * LSB

    def step(self, dt):
        self.last = self.be.read()
        self.be.step(dt)


def hold(backend, q, seconds, dt=0.02):
    backend.enable()
    backend.command(q)
    out = []
    for _ in range(int(seconds / dt)):
        backend.step(dt)
        out.append(backend.read() - q)
    return np.degrees(np.array(out))


def test_link_lands_on_target_with_a_payload_the_model_was_not_told():
    motor_only = hold(MujocoBackend(FULL, q0=STRETCHED, payload_kg=1.0), STRETCHED, 1.0)
    assert np.all(np.abs(motor_only[-1, [1, 3]]) > 0.8)           # 0.87 deg at J2, 1.52 at J4
    be = MujocoBackend(FULL, q0=STRETCHED, payload_kg=1.0)
    err = hold(DualLoop(Encoder(be)), STRETCHED, 7.0)             # told 0 kg: the integral finds the rest
    assert np.all(np.abs(err[-1, [1, 3]]) < 0.02) and not be.faults
    assert np.ptp(err[-50:, [1, 3]], axis=0).max() < 0.005         # settled, no limit cycle on the last bit


def test_wrong_stiffness_in_the_model_is_trimmed_out():
    stiff = replace(FULL, joints=[replace(j, gear_stiffness=1.3 * j.gear_stiffness) for j in FULL.joints])
    be = MujocoBackend(FULL, q0=STRETCHED, payload_kg=1.0)
    err = hold(DualLoop(Encoder(be), payload_kg=1.0, arm=stiff), STRETCHED, 7.0)
    assert np.all(np.abs(err[-1, [1, 3]]) < 0.02)


def test_a_move_ends_on_target_at_the_link():
    q1 = np.deg2rad([40, 70, 20, 80, 0, 30, 0])
    tr = TrapezoidalTrajectory(FULL.home, q1, FULL.vmax, FULL.amax, speed_scale=0.7)
    final = []
    for dual in (False, True):
        be = MujocoBackend(FULL, payload_kg=1.0)
        c = DualLoop(Encoder(be)) if dual else be
        c.enable()
        t = 0.0
        while t < tr.duration + 4.0:
            c.command(tr.sample(min(t, tr.duration))[0])
            c.step(0.02)
            t += 0.02
        assert not be.faults
        final.append(np.degrees(np.abs(be.read() - q1))[1:4])
    assert final[0].max() > 0.4 and final[1].max() < 0.05          # 0.4-0.7 deg left by the motors alone


def test_enable_holds_the_pose():
    """Loop joints hold their link, the others behave exactly as without the loop (they hold their motor)."""
    plain, be = (MujocoBackend(FULL, q0=STRETCHED, payload_kg=1.0) for _ in range(2))
    link0 = be.read()
    loop = DualLoop(Encoder(be), payload_kg=1.0)
    for b in (plain, loop):
        b.enable()
        for _ in range(100):
            b.step(0.02)
    assert np.allclose(be.read_motor()[[0, 4, 5, 6]], plain.read_motor()[[0, 4, 5, 6]], atol=1e-6)
    assert np.allclose(be.read()[1:4], link0[1:4], atol=np.radians(0.03))


def test_too_much_integral_gain_rings_the_first_mode():
    """ki above 2 zeta omega (2.6/s for the 3.8 Hz stretched first mode, docs/14 §19) makes the arm oscillate."""
    be = MujocoBackend(FULL, q0=STRETCHED, payload_kg=1.0)
    loop = DualLoop(Encoder(be))
    loop.ki = 5.0
    err = hold(loop, STRETCHED, 6.0)
    assert np.ptp(err[len(err) // 4:, 1]) > 0.5      # rings (with the CAD masses it can then trip the stall guard)

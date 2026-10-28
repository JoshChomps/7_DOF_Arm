"""ContactGuard: external torque from gear wind-up and stopping on a touch (docs/14 §20). Physics sim only."""
import numpy as np
import pytest

from deskarm import load_arm
from deskarm.counterbalance import spring_torque
from deskarm.trajectory import joint_move

pytest.importorskip("mujoco")
from deskarm.control import ContactGuard  # noqa: E402
from deskarm.physics import MujocoBackend, wall  # noqa: E402

ARM = load_arm()
STRETCHED = np.deg2rad([0, 90, 0, 0, 0, 0, 0])
LSB = 2 * np.pi / 16384


class Encoders:
    """Both encoders one poll old, the link one in 14-bit steps: what CanBackend hands the guard."""

    def __init__(self, be):
        self.be, self.last = be, (be.read(), be.read_motor())

    def __getattr__(self, name):
        return getattr(self.be, name)

    def read(self):
        return np.round(self.last[0] / LSB) * LSB

    def read_motor(self):
        return self.last[1]

    def step(self, dt):
        self.last = (self.be.read(), self.be.read_motor())
        self.be.step(dt)


def guarded(q=STRETCHED, payload=1.0, told=None, streamed=True):
    be = MujocoBackend(ARM, q0=q, payload_kg=payload, streamed=streamed)
    g = ContactGuard(Encoders(be), payload_kg=payload if told is None else told)
    g.enable()
    g.command(q)
    return be, g


def run(g, seconds, dt=0.02, stop=True):
    for _ in range(int(round(seconds / dt))):
        g.step(dt)
        if g.contact and stop:
            return True
    return bool(g.contact)


def test_tool_force_is_jacobian_transpose():
    be = MujocoBackend(ARM, q0=STRETCHED)
    F = np.array([3.0, -2.0, -10.0])
    be.tool_force = F
    be._apply_loads()
    tau = be.data.qfrc_applied[be._va] - spring_torque(ARM, be.read())
    assert np.allclose(tau, ARM.jacobian(be.read())[:3].T @ F, atol=1e-9)
    assert np.allclose(be.tcp(), ARM.fk(be.read())[:3, 3])


def test_gear_torque_is_zero_inside_the_backlash_gap():
    _, g = guarded()
    b = np.array([j.backlash for j in ARM.joints])
    assert not g.gear_torque(np.zeros(7), 0.49 * b).any()
    assert np.allclose(g.gear_torque(np.zeros(7), b), g.k * b / 2)
    assert np.allclose(g.gear_torque(np.zeros(7), -b), -g.k * b / 2)


def test_static_estimate_of_a_held_push_is_jacobian_transpose():
    be, g = guarded()
    g.threshold[:] = np.inf                                        # watch only
    F = np.array([0.0, 0.0, -5.0])
    be.tool_force = F
    run(g, 6.0)                         # 4 s until the TCP moved to 0.11 and the wrist got heavier (21 Oct): it rings longer
    want = ARM.jacobian(be.read())[:3].T @ F
    got = g.static_external(be.read(), be.read_motor())
    assert np.allclose(got[1:4], want[1:4], atol=0.02)             # exact encoders: the model is the plant
    assert np.allclose(g.external[1:4], want[1:4], atol=0.13)      # 14 bits: within k * LSB / 2 (0.115 on J2)


def test_still_arm_does_not_trip_and_a_2_newton_push_does():
    be, g = guarded()
    assert not run(g, 3.0)
    assert np.all(np.abs(g.external[g.mask]) < g.threshold[g.mask])
    be.tool_force = np.array([0.0, 0.0, -2.0])
    t = 0.0
    while not g.contact and t < 1.0:
        g.step(0.02)
        t += 0.02
    assert g.contact.keys() & {"J2", "J4"} and t <= 0.2          # ~0.1 s at 50 Hz (sim); levers 0.52 / 0.33 m,
                                                                  # J4 starts nearer its threshold, so it can win


def test_after_a_trip_commands_are_ignored_until_enable():
    be, g = guarded()
    be.tool_force = np.array([0.0, 0.0, -5.0])
    assert run(g, 1.0)
    held = be.read_motor()
    g.command(STRETCHED + np.radians(10))
    run(g, 0.5, stop=False)
    assert np.all(np.abs(be.read_motor() - held) <= g.back_max + 1e-9) and g.faults
    be.tool_force = np.zeros(3)
    g.enable()
    assert not g.contact and not g.faults


@pytest.mark.parametrize("profile", ["scurve", "trapezoid"])
def test_full_speed_move_with_1_kg_is_no_contact(profile):
    be, g = guarded(ARM.home)
    tr = joint_move(ARM, ARM.home, np.deg2rad([40, 70, 20, 80, 0, 30, 0]), profile)
    static = 0.0
    t = 0.0
    while t < tr.duration + 1.0:
        g.command(tr.sample(min(t, tr.duration))[0])
        g.step(0.02)
        static = max(static, np.abs(g.static_external(be.read(), be.read_motor())[1:4]).max())
        t += 0.02
    assert not g.contact and not be.faults
    assert static > 1.0                                           # without the observer it would have tripped


def test_running_into_a_wall_stops_and_backs_off():
    q = np.deg2rad([0, 60, 0, 90, 0, 30, 0])
    be, g = guarded(q)
    be.tool_force = wall(be.tcp() - [0, 0, 0.01], [0, 0, 1], 5000.0)
    qc, push, t = q.copy(), lambda: np.linalg.norm(be.tool_force(be.tcp())), 0.0
    while not g.contact and t < 2.0:                              # straight down at 50 mm/s
        qc = qc + np.linalg.pinv(ARM.jacobian(qc)[:3]) @ np.array([0, 0, -0.05 * 0.02])
        g.command(qc)
        g.step(0.02)
        t += 0.02
    assert g.contact and 0.2 < t < 0.5                            # touches after 0.2 s
    at_trip = push()
    run(g, 1.5, stop=False)
    assert 2.0 < at_trip < 15.0 and push() < 0.5 * at_trip


def test_unknown_payload_trips_until_tared():
    be, g = guarded(told=0.0)
    assert run(g, 0.5)                                            # 1 kg it was not told about looks like a push
    g.enable()
    g.threshold[:] = np.inf
    run(g, 0.1)
    assert g.tare() == pytest.approx(1.0, abs=0.05)
    g.threshold = g.k * g.lsb + g.margin
    assert not run(g, 2.0)

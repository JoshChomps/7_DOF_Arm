"""Start-up without a jig (deskarm.startup): saved angles, the hall check in CanBackend, and homing."""
import numpy as np
import pytest

pytest.importorskip("can")

from deskarm import mks, startup  # noqa: E402
from deskarm.backends import CanBackend  # noqa: E402
from deskarm.kinematics import arm_from_config, load_config  # noqa: E402

DT = 0.02


def hall_arm(hall_deg=20.0, width_deg=10.0):
    cfg = load_config()
    cfg["joints"][0].update(hall_deg=hall_deg, hall_width_deg=width_deg)        # J1
    return arm_from_config(cfg)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += max(s, 1e-3)


def rig(true_j1_deg, believed_j1_deg=0.0):
    """J1 physically at ``true_j1_deg`` while the backend starts believing ``believed_j1_deg``."""
    arm, clock = hall_arm(), Clock()
    j1 = arm.joints[0]
    cpr = j1.ratio * mks.COUNTS / (2 * np.pi)

    def io(counts):                                   # IN_1 pulled low while the magnet is in the band
        phys = np.radians(true_j1_deg) + counts / cpr
        return 0x02 if abs(phys - j1.hall) <= j1.hall_width / 2 else 0x03

    bus = mks.SimBus(clock=clock, inputs={1: io})
    q = arm.home.copy()
    q[0] = np.radians(believed_j1_deg)
    return arm, CanBackend(arm, bus, q), clock


def run(backend, clock, seconds, target=None):
    for _ in range(int(seconds / DT)):
        if target is not None:
            backend.command(target)
        backend.step(DT)
        clock.t += DT


def test_save_and_load_keep_the_reference(tmp_path):
    path = tmp_path / "state.json"
    assert startup.load(path) == {}
    startup.save([0.1, -0.2], path, ref=([0.0, 0.5], [0.25, 0.75]))
    startup.save([0.3, -0.4], path)                                   # later save: the reference stays
    state = startup.load(path)
    assert np.allclose(np.radians(state["q_deg"]), [0.3, -0.4])
    q_ref, turns = startup.reference(state)
    assert np.allclose(q_ref, [0.0, 0.5]) and np.allclose(turns, [0.25, 0.75])
    assert not (tmp_path / "state.tmp").exists()


def test_start_angles_trust_saved_only_on_hall_joints():
    arm = hall_arm()
    saved = {"q_deg": [10, 20, 30, 40, 50, 60, 70]}
    q, trusted = startup.start_angles(arm, saved)
    assert np.isclose(np.degrees(q[0]), 10) and trusted[0]                   # J1: saved, hall-checked
    assert np.allclose(q[1:], arm.home[1:]) and not trusted[1:].any()        # the rest: jig pose
    coarse = np.full(arm.n, np.nan)
    coarse[1] = 0.3                                                          # J2 output encoder
    q, trusted = startup.start_angles(arm, saved, coarse)
    assert q[1] == 0.3 and trusted[1] and trusted[0] and not trusted[2]


def test_hall_agrees_with_a_right_start():
    arm, backend, clock = rig(true_j1_deg=0.0)
    backend.enable()
    target = arm.home.copy()
    target[0] = np.radians(40)                                               # drive J1 through the band at 20
    run(backend, clock, 2.0, target)
    assert backend.faults == {} and np.isclose(np.degrees(backend.read()[0]), 40, atol=0.01)


def test_hall_catches_a_wrong_start_and_homing_fixes_it():
    arm, backend, clock = rig(true_j1_deg=17.0, believed_j1_deg=0.0)         # pushed 17 deg by hand
    backend.enable()
    target = arm.home.copy()
    target[0] = np.radians(25)                                               # believed: through the band
    run(backend, clock, 1.0, target)
    assert "J1" in backend.faults and not backend.enabled
    backend.enable()
    correction = startup.home(backend, 0, clock=clock, sleep=clock.sleep)
    assert np.degrees(correction) == pytest.approx(17.0, abs=0.5)
    run(backend, clock, 0.5)
    assert backend.faults == {}
    target = backend.read().copy()
    target[0] = np.radians(-10)                                              # and back through it: no fault
    run(backend, clock, 3.0, target)
    assert backend.faults == {} and np.isclose(np.degrees(backend.read()[0]), -10, atol=0.01)


def test_home_refuses_a_joint_without_a_hall():
    arm, backend, clock = rig(true_j1_deg=0.0)
    with pytest.raises(ValueError):
        startup.home(backend, 1, clock=clock, sleep=clock.sleep)

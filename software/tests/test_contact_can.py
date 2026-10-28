"""ContactGuard on CanBackend against simulated drivers (docs/14 §20). SimBus has no arm dynamics: a test sets the
gearbox wind-up (``bus.deflection``) a still arm would have, so this checks the plumbing (polls, 14-bit encoders,
calibration), not the observer's handling of motion, which test_contact.py covers in the physics sim."""
import numpy as np
import pytest

from deskarm import load_arm, mks
from deskarm.counterbalance import spring_torque
from deskarm.encoders import Calibration

pytest.importorskip("can")
from deskarm.backends import CanBackend  # noqa: E402
from deskarm.control import ContactGuard  # noqa: E402

ARM = load_arm()
Q = np.deg2rad([0, 60, 0, 90, 0, 30, 0])
DT = 0.02


def rig(payload=1.0):
    t = [0.0]
    ratios = {i + 1: ARM.joints[i].ratio for i in (1, 2, 3)}
    bus = mks.SimBus(clock=lambda: t[0], turns=np.array([j.ratio for j in ARM.joints]) * Q / (2 * np.pi),
                     encoders=ratios)
    cal = {n: Calibration(1, [0.0, 0.0, 0.0]) for n in ("J2", "J3", "J4")}
    bus.deflection.update(wind_up())            # the loaded arm is already wound up when the backend starts
    g = ContactGuard(CanBackend(ARM, bus, Q, encoders=cal), payload_kg=payload)
    g.enable()
    g.settle()
    return g, bus, t


def wind_up(extra=0.0):
    """Motor minus link (rad) per joint id for a still arm with ``extra`` N*m from outside, set as the bus's deflection."""
    tau = ARM.gravity_torque(Q, 1.0) - spring_torque(ARM, Q) + extra
    k = np.array([j.gear_stiffness for j in ARM.joints])
    half = np.array([j.backlash for j in ARM.joints]) / 2
    d = np.divide(tau, k, out=np.zeros(7), where=k > 0) + half * np.sign(tau)
    return {i + 1: -d[i] for i in (1, 2, 3)}


def run(g, t, seconds):
    for _ in range(int(seconds / DT)):
        g.step(DT)
        t[0] += DT
        if g.contact:
            return True
    return False


def test_still_arm_with_the_right_payload_does_not_trip():
    g, bus, t = rig()
    assert not run(g, t, 2.0) and not g.faults


def test_a_push_on_the_tool_trips_and_enable_resets():
    g, bus, t = rig()
    run(g, t, 0.5)
    F = np.array([0.0, 0.0, -3.0])
    bus.deflection.update(wind_up(ARM.jacobian(Q)[:3].T @ F))
    assert run(g, t, 1.0) and g.contact.keys() & {"J2", "J3", "J4"}
    bus.deflection.update(wind_up())
    g.step(DT)                                  # the encoders report the push gone before the arm is re-enabled
    t[0] += DT
    g.enable()
    assert not g.contact and not run(g, t, 1.0)

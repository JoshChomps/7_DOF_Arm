"""Output encoders: the calibration fit (docs/14 §18) and CanBackend's link reading, against simulated drivers."""
import numpy as np
import pytest

from deskarm import load_arm
from deskarm import mks
from deskarm.encoders import Calibration, fit, gearbox_torque, load, save, sweep, wrap

ARM = load_arm()
LSB = 1 / 16384                                    # one MT6701 step, in turns


def j2_sweeps(sign=-1, zero=3.1, inl=((0.006, 0.002), (0.0015, -0.001)), k=600.0, lost_deg=0.5, poses=(0, 90)):
    """Synthetic J2 calibration data: up and down sweeps at each elbow angle in ``poses`` (deg)."""
    up = np.radians(np.linspace(-115, 115, 400))
    motor, turn, direction, torque = [], [], [], []
    for q4 in poses:
        for d, th in ((1, up), (-1, up[::-1])):
            q = np.tile(ARM.home, (len(th), 1))
            q[:, 1], q[:, 3] = th, np.radians(q4)
            tau = np.array([gearbox_torque(ARM, qi, 1) for qi in q])
            link = th - d * np.radians(lost_deg) / 2 - tau / k
            err = zero + sum(a * np.cos((n + 1) * link) + b * np.sin((n + 1) * link) for n, (a, b) in enumerate(inl))
            raw = (sign * (link + err) / (2 * np.pi)) % 1.0
            motor += list(th); turn += list(np.round(raw / LSB) * LSB); direction += [d] * len(th); torque += list(tau)
    return np.array(motor), np.array(turn), np.array(direction), np.array(torque)


def test_fit_recovers_sign_zero_inl_lost_motion_and_stiffness():
    m, t, d, tau = j2_sweeps()
    cal = fit(m, t, d, tau)
    assert cal.sign == -1
    assert abs(wrap(cal.c[0] - 3.1)) < 1e-3                       # a zero near +-pi survives the wrap
    assert np.allclose(cal.c[1:5], [0.006, 0.002, 0.0015, -0.001], atol=2e-4)
    assert abs(np.degrees(cal.lost_motion) - 0.5) < 0.02
    assert abs(cal.stiffness - 600) < 30
    link = cal.angle(t, m)                                         # the corrected reading = the true link angle
    assert np.max(np.abs(np.degrees(link - (m - d * np.radians(0.25) - tau / 600)))) < 0.03


def test_one_pose_cannot_tell_inl_from_wind_up():
    m, t, d, _ = j2_sweeps(poses=(0,))
    with pytest.raises(ValueError, match="two poses"):
        fit(m, t, d, 10 * np.cos(m + 0.3))           # a load that is a pure first harmonic of the angle


def test_angle_inverts_the_model_and_picks_the_nearest_turn():
    cal = Calibration(sign=1, c=[0.5, 0.01, -0.004, 0.002, 0.0])
    theta = np.radians(np.linspace(-170, 170, 50))
    turn = ((theta + cal.error(theta)) / (2 * np.pi)) % 1.0
    assert np.allclose(cal.angle(turn, theta + 0.3), theta, atol=1e-7)
    assert np.isclose(cal.angle(turn[0], theta[0] + 2 * np.pi), theta[0] + 2 * np.pi)


def test_calibrations_save_and_load_by_joint(tmp_path):
    path = tmp_path / "enc.json"
    assert load(path) == {}
    a, b = Calibration(1, [0.1, 0.0, 0.0], stiffness=550.0), Calibration(-1, [2.0, 0.01, 0.0], lost_motion=0.01)
    save("J2", a, path)
    save("J4", b, path)
    save("J2", b, path)
    assert load(path) == {"J2": b, "J4": b}


# --------------------------------------------------------------- CanBackend with output encoders ----
pytest.importorskip("can")
from deskarm.backends import CanBackend  # noqa: E402

DT = 0.02
INL = {2: lambda th: -1.0 + 0.008 * np.sin(th + 0.4), 4: lambda th: 0.7 - 0.005 * np.cos(2 * th)}


def calibrations():
    """Exact calibrations for the simulated J2 and J4 encoders (INL above, sign +1)."""
    return {"J2": Calibration(1, [-1.0, 0.008 * np.sin(0.4), 0.008 * np.cos(0.4)]),
            "J4": Calibration(1, [0.7, 0.0, 0.0, -0.005, 0.0])}


def rig(q_true, **kw):
    t = [0.0]
    clock = lambda: t[0]  # noqa: E731
    bus = mks.SimBus(clock=clock, turns=np.array([j.ratio for j in ARM.joints]) * q_true / (2 * np.pi),
                     encoders={2: ARM.joints[1].ratio, 4: ARM.joints[3].ratio}, encoder_error=INL)
    return bus, t, clock


def run(be, t, seconds):
    for _ in range(int(seconds / DT)):
        be.step(DT)
        t[0] += DT


def test_start_angle_and_read_come_from_the_output_encoders():
    q_true = np.radians([10, 37.3, -20, 81.6, 5, 30, 0])
    bus, t, _ = rig(q_true)
    be = CanBackend(ARM, bus, ARM.home, encoders=calibrations())       # q_start is wrong on J2 and J4
    assert np.allclose(np.degrees(be.read()[[1, 3]]), [37.3, 81.6], atol=0.03)
    bus.deflection[2] = np.radians(-0.8)                                # the gearbox winds up under load
    run(be, t, 0.1)
    assert np.degrees(be.read()[1] - be.read_motor()[1]) == pytest.approx(-0.8, abs=0.03)


def test_slip_and_silent_encoder_are_faults():
    q_true = np.radians([0, 30, 0, 60, 0, 60, 0])
    bus, t, _ = rig(q_true)
    be = CanBackend(ARM, bus, q_true, encoders=calibrations())
    be.enable()
    run(be, t, 0.2)
    assert not be.faults
    bus.deflection[4] = np.radians(-0.5) - be.slip_limit[3]             # more than J4 can wind up
    run(be, t, 0.1)
    assert "J4" in be.faults and "disagree" in be.faults["J4"] and not be.enabled
    bus.deflection[4] = 0.0                                              # fixed: poll once, then enable
    run(be, t, DT)
    be.enable()
    del bus.encoders[2]                                                  # J2's board goes quiet
    run(be, t, 0.5)
    assert "no good reading" in be.faults["J2"]


def test_a_missing_encoder_is_refused_at_start():
    bus, *_ = rig(np.zeros(7))
    del bus.encoders[4]
    with pytest.raises(RuntimeError, match="J4's output encoder"):
        CanBackend(ARM, bus, ARM.home, encoders=calibrations())


def test_bench_sweep_and_fit_recover_the_sensor():
    from deskarm.bench import bench_arm

    arm = bench_arm("J2")
    t = [0.0]
    clock, sleep = (lambda: t[0]), (lambda s: t.__setitem__(0, t[0] + s))
    bus = mks.SimBus([2], clock=clock, encoders={2: arm.joints[0].ratio}, encoder_error={2: INL[2]})
    be = CanBackend(arm, bus, [0.0], ids=[2])
    be.enable()
    log = sweep(be, 0, speed_deg_s=30, clock=clock, sleep=sleep)
    assert set(log["direction"]) == {1.0, -1.0} and np.ptp(log["motor"]) > np.radians(200)
    cal = fit(**log)
    assert cal.sign == 1 and cal.stiffness is None
    th = np.radians(np.linspace(-110, 110, 23))
    assert np.max(np.abs(np.degrees(cal.error(th) - INL[2](th)))) < 0.02

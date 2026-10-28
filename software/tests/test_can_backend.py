"""CanBackend against the simulated driver chain (deskarm.mks.SimBus), plus the frames it relies on."""
import numpy as np
import pytest

pytest.importorskip("can")

from deskarm import load_arm  # noqa: E402
from deskarm import mks  # noqa: E402
from deskarm.backends import CanBackend, absolute_angle  # noqa: E402
from deskarm.bridge import STATUS_FRAME_ID  # noqa: E402

ARM = load_arm()
RATIO = np.array([j.ratio for j in ARM.joints])
DT = 0.02


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make(q_start=None, turns=None, **kw):
    clock = Clock()
    bus = mks.SimBus(turns=turns, clock=clock)
    return CanBackend(ARM, bus, ARM.home if q_start is None else q_start, **kw), bus, clock


def run(backend, clock, seconds):
    for _ in range(int(seconds / DT)):
        backend.step(DT)
        clock.t += DT


def sent(bus, code, can_id=None):
    return [m for m in bus.sent if m.data[0] == code and (can_id is None or m.arbitration_id == can_id)]


# ------------------------------------------------------------------- frames (manual V1.0.9) ----
def test_move_frames_match_the_manual_examples():
    assert mks.move_frame(1, 600, 2, 0x4000) == (1, bytes.fromhex("F5 02 58 02 00 40 00 92"))     # §11.4.1
    assert mks.move_frame(1, 600, 2, -0x4000) == (1, bytes.fromhex("F5 02 58 02 FF C0 00 11"))
    assert mks.move_frame(1, 300, 2, 0x28000) == (1, bytes.fromhex("F5 01 2C 02 02 80 00 A7"))     # §11.4.3
    with pytest.raises(ValueError):
        mks.move_frame(1, 0, 2, 0)                    # speed 0 means "stop", never a move


def test_position_reply_from_the_manual_parses():
    code, p = mks.reply(1, bytes.fromhex("31 FF FF FF 09 3C 96 0A"))                             # §13.3
    assert code == mks.AXIS and int.from_bytes(p, "big", signed=True) == 0xFFFFFF093C96 - (1 << 48)
    assert mks.reply(1, bytes.fromhex("31 FF FF FF 09 3C 96 0B")) is None                          # bad checksum


def test_acceleration_code_follows_the_manual_ramp():
    assert mks.acc_code(1000) == 236         # §9.1 example: acc 236 adds 1 rpm per ms
    assert mks.acc_code(20_000) == 255 and mks.acc_code(10) == 1


# ------------------------------------------------------------------ coarse + fine (§16) ----
def test_absolute_angle_picks_the_right_motor_turn():
    R, ref_angle, ref_turn = 48.0, 0.3, 0.71
    truth = np.deg2rad(np.array([-110.0, -3.2, 0.0, 47.3, 119.9]))
    turn = (ref_turn + R * (truth - ref_angle) / (2 * np.pi)) % 1.0
    window = np.pi / R                                  # half a motor turn at the joint: 3.75 deg
    for err in (0.0, 0.9 * window, -0.9 * window):
        assert np.allclose(absolute_angle(truth + err, turn, R, ref_angle, ref_turn), truth)
    wrong = absolute_angle(truth + 1.1 * window, turn, R, ref_angle, ref_turn)
    assert np.allclose(wrong - truth, 2 * window)       # past the window: one whole motor turn off


# ------------------------------------------------------------------------- the backend ----
def test_start_pose_is_taken_from_q_start():
    q0 = ARM.home + np.deg2rad([5, -10, 15, 20, -25, 30, 35])
    backend, _, _ = make(q0)
    assert np.allclose(backend.read(), q0, atol=1e-9)


def test_tracks_a_streamed_target_within_one_encoder_count():
    backend, bus, clock = make()
    backend.command(ARM.home + 0.5)                     # ignored while disabled
    run(backend, clock, 0.1)
    assert not sent(bus, mks.MOVE) and np.allclose(backend.read(), ARM.home)
    backend.enable()
    assert len(sent(bus, mks.ENABLE)) == ARM.n and len(sent(bus, mks.HEARTBEAT)) == ARM.n
    goal = ARM.home + np.deg2rad([20, -15, 10, 30, -40, 25, 90])
    backend.command(goal)
    run(backend, clock, 3.0)
    assert np.all(np.abs(backend.read() - goal) <= 1.0 / backend._cpr)
    assert not backend.faults


def test_moves_respect_joint_speed_and_the_int24_coordinate():
    backend, bus, clock = make()
    backend.enable()
    backend.command(ARM.upper)                           # far away: every move asks for full speed
    run(backend, clock, 0.2)
    for j, cid in enumerate(backend.ids):
        moves = sent(bus, mks.MOVE, cid)
        rpm = [int.from_bytes(m.data[1:3], "big") for m in moves]
        assert max(rpm) == backend._rpm_max[j]
        assert backend._rpm_max[j] <= ARM.vmax[j] * RATIO[j] * 60 / (2 * np.pi) + 1


def test_coarse_start_is_made_exact_with_a_reference():
    turns = np.array([0.37, 12.81, -3.4, 7.05, 0.66, -2.2, 4.5])          # motor angles at power-up
    truth = ARM.home + np.deg2rad([12, -30, 40, 55, -70, 35, 100])
    ref = (truth - 2 * np.pi * (turns - np.floor(turns) - 0.25) / RATIO, np.full(ARM.n, 0.25))
    coarse = truth + np.pi / RATIO * np.array([0.5, -0.5, 0.3, -0.3, 0.6, -0.6, 0.0])
    backend, _, _ = make(coarse, turns=turns, ref=ref)
    assert np.allclose(backend.read(), truth, atol=1e-5)          # within a count of the motor encoder
    q, single = backend.reference()
    assert np.allclose(q, truth, atol=1e-5) and np.allclose(single, turns % 1.0, atol=1 / mks.COUNTS)


def test_coarse_start_near_a_turn_boundary_is_refused():
    turns = np.zeros(ARM.n)
    ref = (ARM.home, np.zeros(ARM.n))
    with pytest.raises(RuntimeError, match="J2"):
        make(ARM.home + np.pi / RATIO * np.array([0, 0.9, 0, 0, 0, 0, 0]), turns=turns, ref=ref)


def test_missing_driver_is_named_at_start():
    clock = Clock()
    bus = mks.SimBus(can_ids=[1, 2, 3, 4, 5, 7], clock=clock)
    with pytest.raises(RuntimeError, match=r"J6 \(CAN id 6\)"):
        CanBackend(ARM, bus, ARM.home, timeout=0.05)


def test_estop_stops_and_holds_every_joint():
    backend, bus, clock = make()
    backend.enable()
    backend.command(ARM.home + 0.3)
    run(backend, clock, 0.1)
    backend.estop()
    stops = sent(bus, mks.STOP)
    assert [m.arbitration_id for m in stops] == [mks.BROADCAST, *backend.ids]
    assert not backend.enabled
    n_moves = len(sent(bus, mks.MOVE))
    backend.command(ARM.home)
    run(backend, clock, 0.1)
    assert len(sent(bus, mks.MOVE)) == n_moves          # nothing streamed after the stop
    assert all(bus.enabled.values())                     # still holding (D32), not released


def test_stall_and_silence_become_faults_that_stop_the_arm():
    backend, bus, clock = make()
    backend.enable()
    bus.stalled.add(3)
    run(backend, clock, 0.3)
    assert "J3" in backend.faults and not backend.enabled
    backend.enable()
    assert not backend.faults
    bus.stalled.clear()
    bus.ids.remove(5)                                    # J5's driver stops answering
    run(backend, clock, 0.4)
    assert "J5" in backend.faults["bus"] and not backend.enabled


def test_estop_in_the_bridge_status_frame_disables():
    backend, bus, clock = make()
    backend.enable()
    bus._out.append(mks.message((STATUS_FRAME_ID, bytes([0x01, 0, 0, 0, 0, 0, 0, 1]))))
    run(backend, clock, DT)
    assert "estop" in backend.faults and not backend.enabled


# ------------------------------------------------------------------- speed-mode fallback ----
def test_speed_frames_match_the_manual_examples():
    assert mks.speed_frame(1, 320, 2) == (1, bytes.fromhex("F6 01 40 02 3A"))       # §10.1 forward
    assert mks.speed_frame(1, -320, 2) == (1, bytes.fromhex("F6 81 40 02 BA"))      # reverse
    assert mks.speed_frame(1, 0, 2) == (1, bytes.fromhex("F6 00 00 02 F9"))         # §10.2 ramp to a stop
    with pytest.raises(ValueError):
        mks.speed_frame(1, 3001, 2)


def test_speed_mode_tracks_a_moving_target():
    backend, bus, clock = make(mode="speed")
    backend.enable()
    q0 = ARM.home.copy()
    errors = []
    for k in range(250):                                       # 5 s: a 20 deg/s ramp on J1 for 2 s, then hold
        t = k * DT
        target = q0.copy()
        target[0] += np.radians(20) * min(t, 2.0)
        backend.command(target)
        backend.step(DT)
        clock.t += DT
        errors.append(abs(backend.read()[0] - target[0]))
    assert backend.faults == {}
    assert np.degrees(max(errors[20:100])) < 0.5               # while moving: a fraction of a degree
    assert np.degrees(errors[-1]) < 0.02                       # settled
    assert sent(bus, mks.SPEED) and not sent(bus, mks.MOVE)
    last = sent(bus, mks.SPEED, 1)[-1].data
    assert last[1] & 0x0F == 0 and last[2] == 0                # at rest it sends 0 rpm: stopped


def test_unknown_mode_is_refused():
    with pytest.raises(ValueError):
        make(mode="torque")

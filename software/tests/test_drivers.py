"""The driver set-up tool (deskarm.drivers) against the simulated drivers, and motor_sign in CanBackend."""
import numpy as np
import pytest

pytest.importorskip("can")

from deskarm import load_arm, mks  # noqa: E402
from deskarm import drivers  # noqa: E402
from deskarm.backends import CanBackend  # noqa: E402
from deskarm.kinematics import arm_from_config, load_config  # noqa: E402

ARM = load_arm()
BOARDS = {2: 4}                                   # J2 runs on a SERVO57D


class OneReply:
    """A bus that answers every frame with one fixed frame."""

    def __init__(self, can_id, data):
        self.msg = mks.message((can_id, bytes(data)))

    def send(self, msg, timeout=None):
        pass

    def recv(self, timeout=None):
        return self.msg


def test_read_back_matches_the_manual_example():
    s = next(s for s in drivers.SETTINGS if s.code == 0x83)
    assert mks.frame(1, drivers.READ, b"\x83") == (1, bytes.fromhex("00 83 84"))                  # §5.1.10
    assert drivers.read_setting(OneReply(1, bytes.fromhex("83 0C 80 10")), 1, s) == 3200        # 0x0C80 mA
    assert drivers.read_setting(OneReply(1, bytes.fromhex("83 FF FF 13")), 1, s) is None        # "can't read"


def test_check_writes_what_differs_then_passes():
    bus, lines = mks.SimBus(boards=BOARDS), []
    assert not drivers.check(ARM, bus, out=lines.append)                  # factory settings, not calibrated
    assert any("NOT CALIBRATED" in line for line in lines)
    assert not drivers.check(ARM, bus, write=True, out=lines.append)      # written, still not calibrated
    for cid in bus.ids:
        assert drivers.calibrate(bus, cid, out=lines.append)
    lines.clear()
    assert drivers.check(ARM, bus, out=lines.append), lines
    assert all(len(line.split("\n")) == 1 and line.startswith("J") for line in lines)   # one line per joint
    assert bus.params[2][0x82] == b"\x05" and int.from_bytes(bus.params[2][0x83], "big") == 2800


def test_wrong_board_and_too_much_current_are_reported():
    lines = []
    assert not drivers.check(ARM, mks.SimBus(), out=lines.append)          # J2 on a 42D board
    assert any(line.startswith("J2") and "S57D_CAN" in line for line in lines)
    cfg = load_config()
    cfg["joints"][0]["rated_current"] = 3.5                                  # above a 42D's 3000 mA
    bus, lines = mks.SimBus(boards=BOARDS), []
    drivers.check(arm_from_config(cfg), bus, write=True, out=lines.append)
    assert any("above this board's maximum" in line for line in lines)
    assert bus.params[1][0x83] == (1600).to_bytes(2, "big")                  # left alone


def test_assign_moves_the_only_driver_off_id_1():
    bus = mks.SimBus(range(1, 2))
    assert drivers.assign(bus, 3, out=lambda s: None)
    assert bus.ids == [3] and drivers.version(bus, 3) is not None and drivers.version(bus, 1) is None
    assert not drivers.assign(bus, 4, out=lambda s: None)                   # nobody left at id 1


def test_jog_turns_the_motor_one_turn_positive():
    clock = [0.0]
    bus = mks.SimBus(clock=lambda: clock[0])
    assert drivers.jog(ARM, bus, 1, 2, out=lambda s: None)
    clock[0] += 10.0
    bus.recv()
    assert bus.axis[2] == pytest.approx(mks.COUNTS)


def test_motor_sign_flips_the_counts_in_can_backend():
    cfg = load_config()
    cfg["joints"][1]["motor_sign"] = -1
    arm = arm_from_config(cfg)
    clock = [0.0]
    bus = mks.SimBus(clock=lambda: clock[0])
    backend = CanBackend(arm, bus, arm.home)
    backend.enable()
    q = arm.home.copy()
    q[1] += 0.2                                                  # J2 up by 0.2 rad from home
    backend.command(q)
    for _ in range(300):
        backend.step(0.02)
        clock[0] += 0.02
    assert bus.axis[2] < 0                                       # J2's motor turned the negative way
    assert backend.read()[1] == pytest.approx(q[1], abs=1e-4)    # and the joint reads home + 0.2 rad
    assert np.allclose(backend.read(), q, atol=1e-4)

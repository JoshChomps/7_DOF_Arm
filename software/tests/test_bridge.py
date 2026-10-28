"""End-to-end: python-can's slcan driver <-> the firmware's slcan core running in a pty emulator."""
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest

from deskarm.bridge import STATUS_FRAME_ID, decode_status, open_bus


def test_decode_status_rejects_wrong_length():
    with pytest.raises(ValueError):
        decode_status(bytes(7))


def test_decode_angle_matches_the_firmware_layout():
    """Byte layout of mt6701::angle_frame (firmware/base_controller/src/mt6701); same bytes as its host test."""
    from deskarm.bridge import decode_angle

    a = decode_angle(bytes([0x2A, 0x5C, 0x12, 7]))
    assert a.turn == 0x2A5C / 16384 and a.status == 2 and a.crc_ok and a.present and a.seq == 7
    assert not a.good                                     # field too weak
    assert not decode_angle(bytes([0xFF, 0xFF, 0x8F, 0])).present
    with pytest.raises(ValueError):
        decode_angle(bytes(3))


def test_decode_status_matches_the_firmware_layout():
    """Byte layout of slcan::encode_status (firmware/base_controller/src/slcan/slcan.cpp)."""
    st = decode_status(bytes([0x01 | 0x04 | 0x10, 96, 12, 3, 0x34, 0x12, 0, 1]))
    assert st.estop_pressed and st.error_passive and st.tx_failed
    assert not st.bus_off and not st.rx_overflow and not st.healthy
    assert (st.tx_error_counter, st.rx_error_counter, st.dropped) == (96, 12, 3)
    assert st.uptime_s == 0x1234 and st.firmware == "0.1"
    assert decode_status(bytes(8)).healthy


class FakeBus:
    """Stands in for python-can's slcan bus: silent for the first ``boot_polls`` version requests,
    like an ESP32-S3 that was reset by the serial port being opened and is still booting."""

    def __init__(self, boot_polls, **kwargs):
        self.boot_polls = boot_polls
        self.kwargs = kwargs
        self.calls = []

    def get_version(self, timeout):
        self.calls.append("V")
        if self.boot_polls > 0:
            self.boot_polls -= 1
            return None, None
        return 1, 1

    def set_bitrate(self, bitrate):
        self.calls.append(f"S{bitrate}")

    def shutdown(self):
        self.calls.append("shutdown")


@pytest.fixture
def fake_can(monkeypatch):
    """Install a stand-in ``can`` module; returns the list of buses it has created."""
    import types

    def install(boot_polls):
        made = []

        def bus_factory(**kwargs):
            made.append(FakeBus(boot_polls, **kwargs))
            return made[-1]

        monkeypatch.setitem(sys.modules, "can", types.SimpleNamespace(Bus=bus_factory))
        return made

    return install


def test_open_bus_returns_at_once_when_the_bridge_answers(fake_can):
    made = fake_can(boot_polls=0)
    bus = open_bus("COM5", bitrate=1_000_000)
    assert bus is made[0] and bus.calls == ["V"]                  # no re-open needed
    assert bus.kwargs["interface"] == "slcan" and bus.kwargs["bitrate"] == 1_000_000


def test_open_bus_reopens_the_channel_after_the_bridge_rebooted(fake_can):
    fake_can(boot_polls=3)                                       # chip reset by the port opening
    bus = open_bus("COM5", bitrate=500_000)
    assert bus.calls == ["V", "V", "V", "V", "S500000"]          # waited, then bitrate + open again


def test_open_bus_fails_loudly_on_a_silent_port(fake_can):
    made = fake_can(boot_polls=10**9)
    with pytest.raises(RuntimeError, match="no SLCAN bridge answered"):
        open_bus("COM9", verify_timeout=0.05)
    assert made[0].calls[-1] == "shutdown"


# ---- end to end against the firmware's protocol core (Linux + python-can + g++) ---------------
HOST = Path(__file__).resolve().parents[2] / "firmware" / "base_controller" / "test" / "host"


@pytest.fixture(scope="module")
def emulator_bin(tmp_path_factory):
    pytest.importorskip("can")
    pytest.importorskip("serial")
    if sys.platform != "linux" or shutil.which("g++") is None:
        pytest.skip("pty emulator needs Linux + g++")
    out = tmp_path_factory.mktemp("bridge")
    subprocess.run(["sh", str(HOST / "run.sh")], check=True, env={**os.environ, "BUILD_DIR": str(out)},
                   capture_output=True)
    return out / "pty_bridge"


def start(binary, *args):
    proc = subprocess.Popen([str(binary), *args], stdout=subprocess.PIPE, text=True)
    path = proc.stdout.readline().strip()
    return proc, path


def recv_non_status(bus, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        msg = bus.recv(timeout=0.2)
        if msg is not None and msg.arbitration_id != STATUS_FRAME_ID:
            return msg
    return None


def test_python_can_round_trip_through_firmware_core(emulator_bin):
    import can
    proc, path = start(emulator_bin)
    try:
        bus = can.Bus(interface="slcan", channel=path, bitrate=500_000, sleep_after_open=0.1)
        try:
            assert bus.get_version(timeout=1.0) == (1, 1)
            frames = [
                can.Message(arbitration_id=0x001, data=[0x31, 0x00, 0x10], is_extended_id=False),
                can.Message(arbitration_id=0x1ABCDEF0, data=list(range(8)), is_extended_id=True),
                can.Message(arbitration_id=0x7FF, data=[], is_extended_id=False),
                can.Message(arbitration_id=0x123, is_remote_frame=True, dlc=4, is_extended_id=False),
            ]
            for sent in frames:
                bus.send(sent)
                got = recv_non_status(bus)
                assert got is not None, f"no echo for {sent}"
                assert got.arbitration_id == sent.arbitration_id
                assert got.is_extended_id == sent.is_extended_id
                assert got.is_remote_frame == sent.is_remote_frame
                assert bytes(got.data) == bytes(sent.data)
        finally:
            bus.shutdown()
    finally:
        proc.kill()


@pytest.mark.parametrize("estop", [False, True])
def test_status_frame_reports_estop(emulator_bin, estop):
    import can
    proc, path = start(emulator_bin, *(["--estop"] if estop else []))
    try:
        bus = can.Bus(interface="slcan", channel=path, bitrate=500_000, sleep_after_open=0.1)
        try:
            end = time.monotonic() + 2.0
            status = None
            while time.monotonic() < end and status is None:
                msg = bus.recv(timeout=0.3)
                if msg is not None and msg.arbitration_id == STATUS_FRAME_ID:
                    status = decode_status(msg.data)
            assert status is not None, "no status frame within 2 s"
            assert status.estop_pressed is estop
            assert status.firmware == "0.2"
            assert status.healthy is (not estop)
        finally:
            bus.shutdown()
    finally:
        proc.kill()

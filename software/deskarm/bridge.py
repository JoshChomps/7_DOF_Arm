"""PC side of the ESP32-S3 base controller (USB <-> CAN bridge, SLCAN protocol).

    from deskarm.bridge import open_bus, decode_status, STATUS_FRAME_ID
    bus = open_bus("COM5")             # Windows; "/dev/ttyACM0" on Linux / Raspberry Pi
    for msg in bus:
        if msg.arbitration_id == STATUS_FRAME_ID:
            print(decode_status(msg.data))

Requires the ``hw`` extra (python-can + pyserial). The status frame layout must match
``firmware/base_controller/src/slcan/slcan.h`` (``encode_status``).
"""
from __future__ import annotations

from dataclasses import dataclass

STATUS_FRAME_ID = 0x7F0


@dataclass(frozen=True)
class BridgeStatus:
    estop_pressed: bool
    bus_off: bool
    error_passive: bool
    rx_overflow: bool
    tx_failed: bool
    tx_error_counter: int
    rx_error_counter: int
    dropped: int
    uptime_s: int
    firmware: str

    @property
    def healthy(self) -> bool:
        return not (self.estop_pressed or self.bus_off or self.error_passive)


def decode_status(data: bytes | bytearray) -> BridgeStatus:
    if len(data) != 8:
        raise ValueError(f"status frame must have 8 bytes, got {len(data)}")
    flags = data[0]
    return BridgeStatus(
        estop_pressed=bool(flags & 0x01),
        bus_off=bool(flags & 0x02),
        error_passive=bool(flags & 0x04),
        rx_overflow=bool(flags & 0x08),
        tx_failed=bool(flags & 0x10),
        tx_error_counter=data[1],
        rx_error_counter=data[2],
        dropped=data[3],
        uptime_s=data[4] | (data[5] << 8),
        firmware=f"{data[6]}.{data[7]}",
    )


ANGLE_FRAME_BASE = 0x10        # joint-angle frame id = 0x10 + joint number (J2 = 0x12)


@dataclass(frozen=True)
class JointAngle:
    """One output-encoder reading: the bench MT6701 on the base controller now, board C later."""
    turn: float         # 0..1, one output turn (14 bit)
    status: int         # MT6701 Mg[3:0]: bits 0-1 field (1 too strong, 2 too weak), bit 3 loss of track
    crc_ok: bool
    present: bool
    seq: int            # wraps at 256: a gap means lost frames

    @property
    def good(self) -> bool:
        return self.present and self.crc_ok and self.status == 0


def decode_angle(data: bytes | bytearray) -> JointAngle:
    """Layout of ``mt6701::angle_frame`` (firmware/base_controller/src/mt6701/mt6701.h)."""
    if len(data) != 4:
        raise ValueError(f"joint-angle frame must have 4 bytes, got {len(data)}")
    return JointAngle(turn=((data[0] << 8 | data[1]) & 0x3FFF) / 16384, status=data[2] & 0x0F,
                      crc_ok=bool(data[2] & 0x10), present=not data[2] & 0x80, seq=data[3])


def open_bus(channel: str, bitrate: int = 500_000, sleep_after_open: float = 0.2,
             verify_timeout: float = 5.0):
    """Open the bridge as a python-can bus (bitrate must match the MKS drivers).

    python-can never checks the bridge's replies, so a bridge that missed the "set bitrate" and
    "open" commands looks like a silent bus. That happens for real: the ESP32-S3's USB port
    **resets the chip** when some hosts toggle RTS/DTR on opening the serial port, and the commands
    arrive while it is still booting. So this function asks for the firmware version until the
    bridge answers, re-sends the bitrate + open commands if it had to wait, and raises if nothing
    answers within ``verify_timeout`` seconds (wrong COM port, or not the bridge firmware).
    """
    import time

    import can

    bus = can.Bus(interface="slcan", channel=channel, bitrate=bitrate,
                  sleep_after_open=sleep_after_open)
    deadline = time.monotonic() + verify_timeout
    retried = False
    while bus.get_version(timeout=0.3)[0] is None:
        if time.monotonic() >= deadline:
            bus.shutdown()
            raise RuntimeError(f"no SLCAN bridge answered on {channel} within {verify_timeout:.0f} s: "
                               f"check the port name and that the base-controller firmware is flashed")
        retried = True
    if retried:                       # the bridge (re)booted after we first spoke to it
        bus.set_bitrate(bitrate)      # close, set bitrate, open again
    return bus

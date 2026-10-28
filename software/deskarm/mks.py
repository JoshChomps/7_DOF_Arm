"""MKS SERVO42D/57D CAN protocol: the frames ``CanBackend`` uses, and a simulated driver chain.

Frame layout (MKS SERVO42D/57D CAN user manual V1.0.9, Part 4; multi-byte fields are big-endian)::

    CAN ID (11-bit, 1..2047; 0 = broadcast)    data = [code, payload..., checksum]
    checksum = (CAN ID + code + payload bytes) & 0xFF

Positions are encoder counts on the **motor** shaft, 16384 per motor turn (§1.11.2). The joint is
behind the belt and gearbox, so ``CanBackend`` converts with the whole drive's ratio, and backlash
and wind-up stay invisible to these counts (docs/16 §3).

Stop policy for this arm (docs/09, D32):

* **E-stop** is hardware: the button drops the relay and cuts the 24 V rail. Nothing here is in that
  path. With the power gone the arm may sag (docs/04).
* **Ordinary stops** (a planner fault, the UI's stop button, a torque-guard trip) send the driver's
  "emergency stop" command, code ``F7``: the motor stops and stays powered, so it keeps holding.
  The manual warns against using it above 1000 rpm; this arm's motors stay under 700 rpm.

Written from the manual and **not yet run against a driver**.
"""
from __future__ import annotations

import math
import time
from collections import deque

BROADCAST = 0           # every driver acts on it, none replies
COUNTS = 0x4000         # encoder counts per motor turn (14 bit)
ENCODER = 0x30          # read: carry (int32) + single-turn value (uint16, 0..0x3FFF)
IO = 0x34               # read: bit 0 = IN_1 (home input: the J1/J7 hall switch), bit 1 = IN_2
AXIS = 0x31             # read: position since power-up, int48 counts (the F5 coordinate)
RELEASE = 0x3D          # clear a stall
STALLED = 0x3E          # read: 1 = stalled
HEARTBEAT = 0x98        # stop the motor if no command arrives for this many ms (0 = off)
ENABLE = 0xF3           # 1 = hold the shaft, 0 = release it
SPEED = 0xF6            # run at a speed until told otherwise (0 rpm = stop); the speed-mode fallback
MOVE = 0xF5             # absolute move to a coordinate; a new one replaces a running one
STOP = 0xF7             # "emergency stop the motor": stops and keeps holding
MOVE_FAIL, MOVE_STARTED, MOVE_DONE, MOVE_END_LIMIT = 0, 1, 2, 3     # F5 reply status


def frame(can_id: int, code: int, payload: bytes = b"") -> tuple[int, bytes]:
    """(arbitration id, data bytes) for one command, checksum appended."""
    if not 0 <= can_id <= 0x7FF:
        raise ValueError(f"CAN id {can_id} is outside 0..2047")
    body = bytes([code]) + bytes(payload)
    return can_id, body + bytes([(can_id + sum(body)) & 0xFF])


def reply(can_id: int, data: bytes) -> tuple[int, bytes] | None:
    """(code, payload) of a frame from driver ``can_id``, or None if its checksum is wrong."""
    data = bytes(data)
    if len(data) < 2 or data[-1] != (can_id + sum(data[:-1])) & 0xFF:
        return None
    return data[0], data[1:-1]


def move_frame(can_id: int, rpm: int, acc: int, axis: int) -> tuple[int, bytes]:
    """F5: go to coordinate ``axis`` (counts) at up to ``rpm``, ramping with ``acc`` (see ``acc_code``).

    rpm 0 would mean "stop", so it is refused here. The rpm scale assumes 16, 32 or 64 microsteps
    (manual §9.1), so leave the driver's MStep at one of those.
    """
    if not 1 <= rpm <= 3000 or not 0 <= acc <= 255:
        raise ValueError(f"rpm {rpm} or acc {acc} out of range (1..3000, 0..255)")
    return frame(can_id, MOVE, int(rpm).to_bytes(2, "big") + bytes([acc]) + int(axis).to_bytes(3, "big", signed=True))


def speed_frame(can_id: int, rpm: int, acc: int) -> tuple[int, bytes]:
    """F6: run at ``rpm`` (signed: positive = toward positive counts, the manual's "forward", dir bit 0;
    0 = ramp to a stop with ``acc``, or stop at once with acc 0). Manual §10.1."""
    speed = abs(int(rpm))
    if speed > 3000 or not 0 <= acc <= 255:
        raise ValueError(f"rpm {rpm} or acc {acc} out of range (-3000..3000, 0..255)")
    return frame(can_id, SPEED, bytes([(0x80 if rpm < 0 else 0) | speed >> 8, speed & 0xFF, acc]))


def acc_code(rpm_per_s: float) -> int:
    """F5 acceleration byte for a ramp of ``rpm_per_s``: the driver adds 1 rpm every (256 - acc) x 50 us."""
    return int(min(255, max(1, round(256 - 20_000 / rpm_per_s))))


def stop_frame(can_id: int = BROADCAST) -> tuple[int, bytes]:
    """Stop-and-hold for one driver, or for all of them at once with the broadcast id."""
    return frame(can_id, STOP)


def stop_ok(can_id: int, data: bytes) -> bool:
    """True if ``data`` is that driver's reply to a stop command and it reports success."""
    return reply(can_id, data) == (STOP, b"\x01")


def message(arb_data: tuple[int, bytes]):
    """A python-can message for a ``frame(...)`` result."""
    import can

    arb, data = arb_data
    return can.Message(arbitration_id=arb, data=data, is_extended_id=False)


def stop_all(bus, can_ids=range(1, 8)) -> None:
    """Stop every joint: one broadcast frame, then one addressed frame per driver (those get replies).

    ``bus`` is a python-can bus (``deskarm.bridge.open_bus``). Read the replies with ``stop_ok``.
    """
    for can_id in (BROADCAST, *can_ids):
        bus.send(message(stop_frame(can_id)))


class SimBus:
    """MKS drivers on a CAN bus, simulated, behind the python-can bus interface (``send``/``recv``).

    For developing ``CanBackend`` before the hardware exists, and for its tests. Each driver answers
    the commands listed at the top of this module as the manual describes and moves its motor toward
    the last F5 target at the commanded speed (no acceleration ramp, no load, no "move done" replies).
    ``turns`` is each motor's true angle at power-up in motor turns: the single-turn reading (30H)
    comes from it, while the coordinate (31H) starts at 0 as the manual says. Both are the manual's
    description, **not checked against a driver** yet. ``clock`` lets tests use a fake clock.

    For ``deskarm.drivers`` it also keeps each driver's settings (``params``, code -> value bytes, read
    back with ``00 <code>``), answers the version query (40H) with the board type in ``boards``
    (id -> 2 for a 42D, 4 for a 57D) and the ``calibrated`` set, and takes a new CAN id (8BH). The
    factory values are the manual's defaults where it gives them (mode CR_vFOC, Ma 1600/3200 mA,
    interpolation on, over-current protection off, respond on); MStep 16 is a guess.

    ``encoders`` (driver id -> whole drive ratio) adds an output encoder on that joint, as the bench
    MT6701 on the base controller (or board C) reports it: every position poll (31H) is followed by a
    joint-angle frame, id 0x10 + the driver id (``bridge.decode_angle``). Its angle is the motor's
    divided by the ratio plus ``deflection[id]`` (rad), which a test sets to play a loaded gearbox, plus
    ``encoder_error[id](angle)`` (rad) if given: the sensor's zero offset and INL (``deskarm.encoders``).
    ``inputs`` (driver id -> function of the motor position in counts) gives the IO status byte (34H);
    without one, IN_1 and IN_2 read 1 (idle).
    """

    PER_DRIVER = ("turn0", "axis", "target", "rpm", "enabled", "heartbeat", "heard", "params", "boards")

    def __init__(self, can_ids=range(1, 8), turns=None, clock=time.monotonic, boards=None, encoders=None,
                 inputs=None, encoder_error=None):
        self.ids = list(can_ids)
        self.clock, self.t = clock, clock()
        self.turn0 = {i: (0.0 if turns is None else float(turns[k])) * COUNTS for k, i in enumerate(self.ids)}
        self.axis = dict.fromkeys(self.ids, 0.0)
        self.target = dict.fromkeys(self.ids, 0.0)
        self.rpm = dict.fromkeys(self.ids, 0)
        self.enabled = dict.fromkeys(self.ids, False)
        self.heartbeat = dict.fromkeys(self.ids, 0.0)        # seconds, 0 = off
        self.heard = dict.fromkeys(self.ids, self.t)
        self.boards = {i: (boards or {}).get(i, 2) for i in self.ids}
        self.params = {i: {0x82: b"\x02", 0x83: (3200 if self.boards[i] == 4 else 1600).to_bytes(2, "big"),
                           0x84: b"\x10", 0x88: b"\x00", 0x89: b"\x01", 0x8C: b"\x01\x01"} for i in self.ids}
        self.calibrated: set[int] = set()
        self.stalled: set[int] = set()
        self.encoders = dict(encoders or {})
        self.inputs = dict(inputs or {})
        self.deflection: dict[int, float] = {}
        self.encoder_error = dict(encoder_error or {})
        self._seq = 0
        self.sent = []                                        # every frame received from the host
        self._out = deque()

    def _rename(self, old, new):
        for name in self.PER_DRIVER:
            d = getattr(self, name)
            d[new] = d.pop(old)
        for s in (self.calibrated, self.stalled):
            if old in s:
                s.discard(old)
                s.add(new)
        self.ids[self.ids.index(old)] = new

    def _advance(self):
        now = self.clock()
        dt, self.t = now - self.t, now
        for i in self.ids:
            if self.heartbeat[i] and now - self.heard[i] > self.heartbeat[i]:
                self.target[i] = self.axis[i]                 # no commands for too long: stop
            if self.enabled[i] and i not in self.stalled:
                step = self.rpm[i] / 60 * COUNTS * dt
                self.axis[i] += max(-step, min(step, self.target[i] - self.axis[i]))

    def _answer(self, i, code, p):
        """The reply payload (or (code, payload) if it answers with another code), None for no reply."""
        if code == 0x00:                                      # read a setting: replies under its code
            return p[0], self.params[i].get(p[0], b"\xff\xff")
        if code == 0x40:
            return bytes([(i in self.calibrated) << 4 | self.boards[i], 0x01, 0x09])
        if code == 0x80:
            self.calibrated.add(i)
            self._out.append(message(frame(i, code, b"\x00")))    # "calibrating...", then the result
            return b"\x01"
        if code in (0x82, 0x83, 0x84, 0x88, 0x89, 0x8C):
            self.params[i][code] = bytes(p[:2] if code in (0x83, 0x8C) else p[:1])
            return b"\x02" if code == 0x83 and len(p) == 3 else b"\x01"
        if code == ENCODER:
            carry, value = divmod(round(self.turn0[i] + self.axis[i]), COUNTS)
            return carry.to_bytes(4, "big", signed=True) + value.to_bytes(2, "big")
        if code == AXIS:
            if i in self.encoders:
                angle = 2 * math.pi * (self.turn0[i] + self.axis[i]) / COUNTS / self.encoders[i] + self.deflection.get(i, 0.0)
                turn = (angle + (self.encoder_error[i](angle) if i in self.encoder_error else 0.0)) / (2 * math.pi)
                self._seq = (self._seq + 1) & 0xFF
                self._out.append(message((0x10 + i, (round(turn % 1 * 16384) & 0x3FFF).to_bytes(2, "big")
                                          + bytes([0x10, self._seq]))))
            return round(self.axis[i]).to_bytes(6, "big", signed=True)
        if code == STALLED:
            return bytes([i in self.stalled])
        if code == IO:
            return bytes([self.inputs[i](self.turn0[i] + self.axis[i]) if i in self.inputs else 0x03])
        if code == RELEASE:
            self.stalled.discard(i)
        elif code == HEARTBEAT:
            self.heartbeat[i] = int.from_bytes(p, "big") / 1000
        elif code == ENABLE:
            self.enabled[i], self.target[i] = bool(p[0]), self.axis[i]
        elif code == STOP:
            self.target[i] = self.axis[i]
        elif code == SPEED:
            if not self.enabled[i] or i in self.stalled:
                return b"\x00"
            self.rpm[i] = (p[0] & 0x0F) << 8 | p[1]
            self.target[i] = self.axis[i] + (-1e12 if p[0] & 0x80 else 1e12) * bool(self.rpm[i])
        elif code == MOVE:
            if not self.enabled[i] or i in self.stalled:
                return bytes([MOVE_FAIL])
            self.rpm[i], self.target[i] = int.from_bytes(p[:2], "big"), int.from_bytes(p[3:6], "big", signed=True)
            return bytes([MOVE_STARTED])
        else:
            return None                                       # not simulated: no reply
        return b"\x01"

    def send(self, msg, timeout=None):
        self._advance()
        self.sent.append(msg)
        arb, data = msg.arbitration_id, bytes(msg.data)
        if data[-1] != (arb + sum(data[:-1])) & 0xFF:
            raise ValueError(f"bad checksum in {data.hex(' ')} to CAN id {arb}")
        for i in list(self.ids) if arb == BROADCAST else [arb] if arb in self.ids else []:
            self.heard[i] = self.t
            if data[0] == 0x8B:                               # new CAN id: answers from the old one
                new = int.from_bytes(data[1:3], "big")
                if arb != BROADCAST:
                    ok = 1 <= new <= 0x7FF and new not in self.ids
                    self._out.append(message(frame(i, 0x8B, bytes([ok]))))
                    if ok:
                        self._rename(i, new)
                continue
            out = self._answer(i, data[0], data[1:-1])
            if out is not None and arb != BROADCAST:
                code, out = out if isinstance(out, tuple) else (data[0], out)
                self._out.append(message(frame(i, code, out)))

    def recv(self, timeout=None):
        self._advance()
        return self._out.popleft() if self._out else None

    def shutdown(self):
        pass

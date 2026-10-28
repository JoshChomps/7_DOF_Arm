"""Robot backends: one interface, swappable between simulation and real hardware.

The twin and every script talk to a ``Backend``: ``SimBackend`` (kinematic), ``MujocoBackend``
(deskarm.physics) or ``CanBackend`` (the MKS drivers over CAN, or their simulation ``mks.SimBus``).
Math: docs/14_math_guide.md §16 (absolute joint angle from a coarse reading and the motor encoder).
"""
from __future__ import annotations

import threading
from typing import Protocol

import numpy as np

from . import mks
from .kinematics import ArmModel


class Backend(Protocol):
    def enable(self) -> None: ...
    def estop(self) -> None: ...
    @property
    def enabled(self) -> bool: ...
    def command(self, q: np.ndarray) -> None: ...
    def read(self) -> np.ndarray: ...
    def step(self, dt: float) -> None: ...


class SimBackend:
    """Kinematic simulation of position-controlled joints (like the closed-loop steppers).

    Each joint chases its setpoint at up to its velocity limit. ``backlash`` (rad) adds a
    dead band per joint so controllers can be tested against a realistic printed gearbox, and
    ``resolution`` (rad, scalar or per joint) quantizes motion to whole (micro)steps, e.g.
    ``2*pi / (200 * 16 * gear_ratio)`` for a 1.8 deg stepper at 16x microstepping.
    (Quantized stepper-plant idea from ferdy-alpadry/ros2-udq-arm-handwriting, Apache-2.0.)
    """

    def __init__(self, arm: ArmModel, q0: np.ndarray | None = None, speed_scale: float = 1.0,
                 backlash: float = 0.0, resolution: float | np.ndarray = 0.0):
        self.arm = arm
        self._q = np.array(arm.home if q0 is None else q0, dtype=float)
        self._target = self._q.copy()
        self._enabled = False
        self.speed_scale = speed_scale
        self.backlash = backlash
        self.resolution = np.broadcast_to(np.asarray(resolution, dtype=float), (arm.n,)).copy()
        self._lock = threading.Lock()

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        with self._lock:
            self._target = self._q.copy()
            self._enabled = True

    def estop(self) -> None:
        with self._lock:
            self._enabled = False
            self._target = self._q.copy()

    def command(self, q: np.ndarray) -> None:
        if not self._enabled:
            return
        with self._lock:
            self._target = self.arm.clamp(np.asarray(q, dtype=float))

    def read(self) -> np.ndarray:
        with self._lock:
            return self._q.copy()

    def step(self, dt: float) -> None:
        if not self._enabled:
            return
        with self._lock:
            err = self._target - self._q
            if self.backlash > 0:
                err = np.sign(err) * np.maximum(np.abs(err) - self.backlash / 2, 0.0)
            max_move = self.arm.vmax * self.speed_scale * dt
            q = self._q + np.clip(err, -max_move, max_move)
            res = self.resolution
            self._q = np.where(res > 0, np.round(q / np.where(res > 0, res, 1.0)) * res, q)


def absolute_angle(coarse, turn, ratio, ref_angle, ref_turn):
    """Exact joint angle (rad) from a coarse one plus the motor encoder's single-turn reading.

    ``turn`` and ``ref_turn`` are single-turn readings in turns (0..1); ``ref_angle``/``ref_turn`` were
    recorded together at one pose. Picks the whole number of motor turns that fits ``coarse`` best, so
    ``coarse`` must be within half a motor turn (pi / ratio) of the truth. Works on arrays (one per joint).
    Math: docs/14_math_guide.md §16.
    """
    n = np.round(ratio * (coarse - ref_angle) / (2 * np.pi) - (turn - ref_turn))
    return ref_angle + 2 * np.pi * (n + turn - ref_turn) / ratio


class CanBackend:
    """The real arm: one MKS SERVO42D/57D per joint on CAN (ids 1-7 = J1-J7), python-can ``bus``.

    ``bus`` is ``deskarm.bridge.open_bus("COM5")`` for the hardware or ``deskarm.mks.SimBus()`` before
    it exists. Each ``step`` drains the replies, streams one absolute move (F5) per joint and polls
    each position (31H); ``read`` returns what the polls said one step earlier. Speed per move =
    the distance left / ``dt``, capped at the joint's ``vmax``; ramp from ``amax``. Whether the drivers
    follow such a stream smoothly is Q8, for the bench.

    **Two readings per joint.** ``read_motor`` is the angle the motor implies: its encoder counts divided
    by the whole drive's ratio (belt x reducer), so backlash and wind-up between motor and link don't show,
    and the ratio in ``arm.toml`` must be exact. Each joint's ``motor_sign`` (arm.toml) says which way
    positive counts turn it: the drivers' Dir setting does nothing over CAN. ``read`` is the link angle:
    the same, except on joints given an output-encoder ``Calibration`` in ``encoders`` (joint name ->
    ``deskarm.encoders.Calibration``; D33: J2-J4, frame 0x10 + driver id), where it is that encoder's
    calibrated reading (docs/14_math_guide.md §18). Those joints also fault if the encoder goes quiet
    (``patience`` steps without a good reading) or if link and motor disagree by more than the gearbox can
    wind up (``slip_limit``): a skipped belt or a sheared pin (docs/16 §3.1, job C). Moves are still
    sent on the motor side.

    The drivers count from 0 at power-up, so the arm's angles at start come from ``q_start``, or from the
    output encoder on joints that have one: exact if the arm stands at a known pose, or coarse (output
    encoders, saved angles) and made exact with
    ``ref`` = (angles, single-turn readings) once saved by ``reference()`` at a known pose
    (``absolute_angle``). Faults (no reply, stall, refused move, E-stop seen in the bridge status)
    stop every joint and keep them holding (D32); ``enable`` clears them. Joints with a hall switch
    (``Joint.hall``, on the driver's IN_1) are checked every step: the switch must read "magnet" inside
    its band and not well outside it, or the start angle was wrong (docs/16 §3.3). ``set_angle`` re-zeros
    a joint after homing (``deskarm.startup.home``). Not yet run on hardware.

    ``mode="speed"`` is the fallback in case the drivers can't follow streamed moves smoothly (Q8): each
    step sends a speed (F6) instead, the target's own speed plus ``speed_gain`` x the position error per
    step, so the position loop runs here, one poll behind, at 1 rpm resolution.
    """

    patience = 10            # steps without a reply before a joint counts as lost
    hall_active = 0          # IN_1 level with the magnet there: an A3144 pulls low (check on the bench)
    hall_margin = np.radians(2.0)
    speed_gain = 0.25        # speed mode: fraction of the position error removed per step (< 0.5 with the lag)

    def __init__(self, arm: ArmModel, bus, q_start, *, ids=None, ref=None, heartbeat_ms: int = 500,
                 timeout: float = 0.5, mode: str = "position", encoders=None):
        if mode not in ("position", "speed"):
            raise ValueError(f"mode is 'position' or 'speed', not {mode!r}")
        self.mode = mode

        self.arm, self.bus = arm, bus
        self.ids = list(ids or range(1, arm.n + 1))
        self._index = {cid: j for j, cid in enumerate(self.ids)}
        ratio = np.array([j.ratio for j in arm.joints])
        sign = np.array([j.motor_sign for j in arm.joints])
        self._cpr = sign * ratio * mks.COUNTS / (2 * np.pi)              # counts per joint radian
        self._rpm_max = np.maximum(1, arm.vmax * ratio * 60 / (2 * np.pi)).astype(int)
        self._acc = [mks.acc_code(a * r * 60 / (2 * np.pi)) for a, r in zip(arm.amax, ratio)]
        self.heartbeat_ms, self.timeout = heartbeat_ms, timeout
        self.faults: dict[str, str] = {}
        self.output: dict[int, object] = {}     # joint index -> latest bridge.JointAngle (raw)
        names = [jt.name for jt in arm.joints]
        self._cal = {names.index(k): c for k, c in (encoders or {}).items()}     # joint index -> Calibration
        # more than the whole backlash + the wind-up under 1.5x the joint's torque limit + 1 deg of sensor
        self.slip_limit = np.array([jt.backlash + (1.5 * jt.torque_max / jt.gear_stiffness if jt.gear_stiffness
                                    else 0.0) + np.radians(1.0) for jt in arm.joints])
        self.inputs: dict[int, int] = {}        # joint index -> IO status byte (34H), hall joints only
        self.homing: set[int] = set()           # joints whose hall check is off while they home
        self._halls = [j for j, jt in enumerate(arm.joints) if jt.hall is not None]
        self._enabled, self._target, self._tick, self._prev_goal = False, None, 0, None
        self._heard = np.zeros(arm.n, dtype=int)
        self._seen = np.zeros(arm.n, dtype=int)            # tick of each output encoder's last good reading
        self._lock = threading.Lock()
        p = self._ask(mks.AXIS)                            # the output encoders' readings arrive meanwhile
        self._axis = np.array([int.from_bytes(p[c], "big", signed=True) for c in self.ids])
        q = np.array(q_start, dtype=float)
        for j, cal in self._cal.items():
            r, jt = self.output.get(j), arm.joints[j]
            if r is None or not r.good:
                raise RuntimeError(f"no good reading from {jt.name}'s output encoder ({r}): check board C and its magnet")
            q[j] = cal.angle(r.turn, (jt.lower + jt.upper) / 2)
        if ref is not None:
            coarse, q = q, absolute_angle(q, self._turns(), sign * ratio, *map(np.asarray, ref))
            off = np.abs(q - coarse) > np.pi / ratio * 2 / 3
            if off.any():
                raise RuntimeError(f"start angles of {self._names(off)} are near a motor-turn boundary from the "
                                   f"coarse reading: check the coarse sensor or re-reference")
        self._offset = self._axis - q * self._cpr

    # ------------------------------------------------------------------ Backend API ----
    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        with self._lock:
            self.faults.clear()
            for cid in self.ids:
                self._send(mks.frame(cid, mks.ENABLE, b"\x01"))
                self._send(mks.frame(cid, mks.HEARTBEAT, self.heartbeat_ms.to_bytes(4, "big")))
            self._heard[:] = self._seen[:] = self._tick
            self._target = self._q()
            self._enabled = True

    def estop(self) -> None:
        """Software stop: every motor stops and holds (F7). The real E-stop is the hardware button."""
        with self._lock:
            self._halt()

    def command(self, q: np.ndarray) -> None:
        if self._enabled:
            with self._lock:
                self._target = self.arm.clamp(np.asarray(q, dtype=float))

    def read(self) -> np.ndarray:
        with self._lock:
            return self._link()

    def read_motor(self) -> np.ndarray:
        with self._lock:
            return self._q()

    def step(self, dt: float) -> None:
        with self._lock:
            self._drain()
            self._tick += 1
            lost = self._tick - self._heard > self.patience
            if self._enabled and lost.any():
                self._fault("bus", f"no reply from {self._names(lost)}: check power, CAN wiring and ids")
            self._check_encoders()
            if self._enabled and self.mode == "speed":
                goal = self._offset + self._target * self._cpr
                prev = self._prev_goal if self._prev_goal is not None else goal
                counts_per_s = (goal - prev) / dt + self.speed_gain * (goal - self._axis) / dt
                rpm = np.clip(np.round(counts_per_s / mks.COUNTS * 60), -self._rpm_max, self._rpm_max).astype(int)
                self._prev_goal = goal
                for j, cid in enumerate(self.ids):
                    self._send(mks.speed_frame(cid, int(rpm[j]), self._acc[j]))
            elif self._enabled:
                goal = np.round(self._offset + self._target * self._cpr).astype(int)
                rpm = np.clip(np.abs(goal - self._axis) / mks.COUNTS / dt * 60, 1, self._rpm_max).astype(int)
                for j, cid in enumerate(self.ids):
                    self._send(mks.move_frame(cid, int(rpm[j]), self._acc[j], int(goal[j])))
            self._check_halls(dt)
            for cid in self.ids:
                self._send(mks.frame(cid, mks.AXIS))
                if self._tick % 10 == 0:
                    self._send(mks.frame(cid, mks.STALLED))
            for j in self._halls:
                self._send(mks.frame(self.ids[j], mks.IO))

    def set_angle(self, j: int, angle: float) -> None:
        """Declare joint ``j``'s current angle (rad): after homing onto its hall switch."""
        with self._lock:
            self._offset[j] = self._axis[j] - angle * self._cpr[j]
            self._prev_goal = None                 # no speed feed-forward kick from the jump
            if self._target is not None:
                self._target[j] = angle

    def hall(self, j: int) -> bool | None:
        """True while joint ``j``'s hall switch sees the magnet (None before the first reading)."""
        s = self.inputs.get(j)
        return None if s is None else (s & 1) == self.hall_active

    # ----------------------------------------------------------------------- helpers ----
    def _check_encoders(self):
        if not self._enabled:
            return
        q, link = self._q(), self._link()
        for j in self._cal:
            name = self.arm.joints[j].name
            if self._tick - self._seen[j] > self.patience:
                self._fault(name, f"{name}'s output encoder sent no good reading for {self.patience} steps "
                            f"({self.output.get(j)}): check board C, its magnet and the CAN wiring")
            elif abs(link[j] - q[j]) > self.slip_limit[j]:
                self._fault(name, f"{name}: the output encoder and the motor disagree by "
                            f"{np.degrees(link[j] - q[j]):.1f} deg, more than the gearbox can wind up: "
                            f"skipped belt, slipping or broken gearbox, or a loose magnet")

    def _link(self):
        q = self._q()
        for j, cal in self._cal.items():
            r = self.output.get(j)
            if r is not None and r.good:
                q[j] = cal.angle(r.turn, q[j])
        return q

    def _check_halls(self, dt):
        q = self._q()
        for j in self._halls:
            on = self.hall(j)
            if on is None or j in self.homing:
                continue
            jt = self.arm.joints[j]
            d = abs((q[j] - jt.hall + np.pi) % (2 * np.pi) - np.pi)          # J7 sees its magnet every turn
            slack = self.hall_margin + 2 * jt.vmax * dt                      # the input is one poll behind
            if on != (d <= jt.hall_width / 2) and abs(d - jt.hall_width / 2) > slack:
                self._fault(jt.name, f"{jt.name}: the hall switch says {'near' if on else 'away from'} "
                            f"{np.degrees(jt.hall):.0f} deg but the angle is {np.degrees(q[j]):.1f} deg: "
                            f"the start angle is wrong (moved by hand?). Home it (deskarm.startup.home)")

    def reference(self):
        """(angles, single-turn readings) at the current pose: save them, pass them back as ``ref``."""
        with self._lock:
            return self._q(), self._turns()

    def _q(self):
        return (self._axis - self._offset) / self._cpr

    def _names(self, mask):
        return ", ".join(self.arm.joints[j].name for j in np.flatnonzero(mask))

    def _send(self, arb_data):
        self.bus.send(mks.message(arb_data))

    def _turns(self):
        p = self._ask(mks.ENCODER)
        return np.array([int.from_bytes(p[c][4:6], "big") / mks.COUNTS for c in self.ids])

    def _ask(self, code):
        """Send ``code`` to every driver and wait for all the replies (start-up only)."""
        import time

        got = {}
        for cid in self.ids:
            self._send(mks.frame(cid, code))
        deadline = time.monotonic() + self.timeout
        while (len(got) < len(self.ids) or self._cal.keys() - self.output.keys()) and time.monotonic() < deadline:
            msg = self.bus.recv(0.01)
            if msg is not None and self._angle_frame(msg):
                continue
            r = msg and mks.reply(msg.arbitration_id, msg.data)
            if r and r[0] == code and msg.arbitration_id in self._index:
                got[msg.arbitration_id] = r[1]
        missing = [c for c in self.ids if c not in got]
        if missing:
            names = ", ".join(f"{self.arm.joints[self._index[c]].name} (CAN id {c})" for c in missing)
            raise RuntimeError(f"no reply from {names}: check its power, CAN wiring, id and bitrate")
        return got

    def _angle_frame(self, msg) -> bool:
        """Keep an output-encoder frame (id 0x10 + driver id); False if ``msg`` is not one."""
        from .bridge import ANGLE_FRAME_BASE, decode_angle

        j = self._index.get(msg.arbitration_id - ANGLE_FRAME_BASE)
        if j is None or len(msg.data) != 4:
            return False
        self.output[j] = r = decode_angle(msg.data)
        if r.good:
            self._seen[j] = self._tick
        return True

    def _drain(self):
        from .bridge import STATUS_FRAME_ID, decode_status

        while (msg := self.bus.recv(0.0)) is not None:
            if msg.arbitration_id == STATUS_FRAME_ID:
                if decode_status(msg.data).estop_pressed:
                    self._fault("estop", "E-stop pressed (motor power is off)")
                continue
            if self._angle_frame(msg):
                continue
            j, r = self._index.get(msg.arbitration_id), mks.reply(msg.arbitration_id, msg.data)
            if j is None or r is None:
                continue
            code, p = r
            name = self.arm.joints[j].name
            self._heard[j] = self._tick
            if code == mks.AXIS:
                self._axis[j] = int.from_bytes(p, "big", signed=True)
            elif code == mks.IO:
                self.inputs[j] = p[0]
            elif code == mks.STALLED and p[0]:
                self._fault(name, f"{name} stalled (the driver may have released the shaft)")
            elif code == mks.SPEED and p[0] == 0:
                self._fault(name, f"{name} refused the speed command")
            elif code == mks.MOVE and p[0] in (mks.MOVE_FAIL, mks.MOVE_END_LIMIT):
                self._fault(name, f"{name} refused the move" if p[0] == mks.MOVE_FAIL else f"{name} hit an end limit")
            elif code in (mks.ENABLE, mks.HEARTBEAT, mks.STOP) and p[0] == 0:
                self._fault(name, f"{name} did not accept command {code:02X}h")

    def _fault(self, key, text):
        if key not in self.faults or self._enabled:        # stop once per new fault, not on every repeat
            self.faults[key] = text
            self._halt()

    def _halt(self):
        for cid in (mks.BROADCAST, *self.ids):
            self._send(mks.stop_frame(cid))
        self._enabled, self._target, self._prev_goal = False, None, None

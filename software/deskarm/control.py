"""Joint control on the output encoders: ``DualLoop`` (link angle on target) and ``ContactGuard`` (stop on a touch).

The MKS drivers close a stiff loop on the motor encoder. The printed gearbox behind it bends under load, so the
link sags by tau / k + b / 2 while the motor sits exactly on target (docs/14 §12). ``DualLoop`` wraps any backend
whose ``read()`` is the link and ``read_motor()`` the motor (``MujocoBackend``; ``CanBackend`` with output
encoders) and moves the motor target so the *link* lands on the command:

    motor target = q* + d(q*) + i,    d = tau(q*) / k + b/2 * ramp(tau),    i += ki dt (q* - link)

``d`` is the deflection the model predicts (gravity minus the spring, with the payload it is told), ``i`` an
integral that trims what the model gets wrong. The integral runs only while the command is still (during a move
the link lags for dynamic reasons and it would wind up) and outside a small dead zone (so the encoder's last bit
can't make it hunt), and it is clamped to what the gearbox can wind up. Joints without an output encoder get the
command unchanged. Math, including why ``ki`` must stay well under the first mode: docs/14_math_guide.md §19.

``ContactGuard`` reads the same two encoders as a force sensor. The gearbox is a spring, so its torque is
k * (motor - link) outside the backlash gap; a momentum observer subtracts what the arm's own gravity, spring and
motion need, and what is left is the external torque on each joint. Over a threshold for long enough, the guard stops
the arm and backs the motors off by the wind-up the push caused. Math: docs/14_math_guide.md §20.
"""
from __future__ import annotations

import numpy as np

from .counterbalance import spring_torque
from .physics import build_model, compliant_joints


class DualLoop:
    ki = 1.0                        # 1/s; stable while ki < 2 zeta omega of the first mode (2.6/s stretched)
    still = np.radians(0.5)         # rad/s: the integral runs only while every commanded joint moves slower
    dead_zone = np.radians(0.013)   # rad: just over half an MT6701 step (0.022 deg), so some reading is "on target"
    torque_band = 0.5               # N*m: the backlash term ramps across +-this instead of jumping at tau = 0

    def __init__(self, inner, payload_kg: float = 0.0, joints=("J2", "J3", "J4"), feedforward: bool = True,
                 arm=None):
        """``arm``: the model the loop believes (default the inner backend's); ``payload_kg`` likewise."""
        self.inner, self.payload_kg, self.feedforward = inner, payload_kg, feedforward
        self.arm = arm or inner.arm
        self.mask = np.array([j.name in joints for j in self.arm.joints])
        k = np.array([j.gear_stiffness for j in self.arm.joints])
        self._compliance = np.divide(1.0, k, out=np.zeros_like(k), where=k > 0)
        self._half_b = np.array([j.backlash for j in self.arm.joints]) / 2
        self._bound = 2 * self._half_b + np.array([j.torque_max for j in self.arm.joints]) * self._compliance
        self.integral = np.zeros(self.arm.n)
        self._target = self._prev = None

    # ------------------------------------------------------------------ Backend API ----
    @property
    def enabled(self) -> bool:
        return self.inner.enabled

    def __getattr__(self, name):                     # faults, torque_utilization, ... come from the inner backend
        if name == "inner":
            raise AttributeError(name)
        return getattr(self.inner, name)

    def enable(self) -> None:
        self.inner.enable()
        self.integral[:] = 0
        # hold where it is: the link on loop joints, the motor elsewhere (their link has already sagged)
        self._target, self._prev = np.where(self.mask, self.inner.read(), self.inner.read_motor()), None

    def estop(self) -> None:
        self.inner.estop()
        self._target = None

    def command(self, q) -> None:
        if self.inner.enabled:
            self._target = self.arm.clamp(np.asarray(q, dtype=float))

    def read(self) -> np.ndarray:
        return self.inner.read()

    def read_motor(self) -> np.ndarray:
        return self.inner.read_motor()

    def step(self, dt: float) -> None:
        if self.inner.enabled and self._target is not None:
            q = self._target
            moving = self._prev is not None and np.max(np.abs(q - self._prev)) / dt > self.still
            self._prev = q.copy()
            e = q - self.inner.read()
            grow = self.mask & (np.abs(e) > self.dead_zone) & (not moving)
            self.integral = np.clip(self.integral + self.ki * dt * np.where(grow, e, 0.0), -self._bound, self._bound)
            d = self.deflection(q) if self.feedforward else 0.0
            self.inner.command(q + np.where(self.mask, d + self.integral, 0.0))
        self.inner.step(dt)

    def deflection(self, q) -> np.ndarray:
        """Motor minus link (rad) the model predicts holding still at ``q``: tau / k + b/2 * ramp(tau)."""
        tau = self.arm.gravity_torque(q, self.payload_kg) - spring_torque(self.arm, q)
        return tau * self._compliance + self._half_b * np.clip(tau / self.torque_band, -1.0, 1.0)


class ContactGuard:
    gain = 10.0                 # 1/s: observer bandwidth, the estimate follows a step in force as 1 - exp(-gain t)
    margin = 0.15               # N*m over the encoder's floor k * LSB: what the model is allowed to get wrong
    debounce = 0.03             # s over the threshold before it trips (2 polls at 50 Hz, 3 at 100 Hz)
    back_off = 5.0              # 1/s: after a trip the motors give way at this rate times the push's wind-up
    back_max = np.radians(1.0)  # rad: but no further than this from where they stopped
    lsb = 2 * np.pi / 16384     # rad: one MT6701 step

    def __init__(self, inner, payload_kg: float = 0.0, joints=("J2", "J3", "J4"), arm=None):
        """``inner`` takes motor-side targets and has ``read()`` (link) and ``read_motor()``; ``payload_kg`` as told."""
        self.inner, self.arm = inner, arm or inner.arm
        self.mask = np.array([j.name in joints for j in self.arm.joints])
        self.k = np.array([j.gear_stiffness for j in self.arm.joints])
        self._half_b = np.array([j.backlash for j in self.arm.joints]) / 2
        self.threshold = self.k * self.lsb + self.margin
        self.contact: dict[str, str] = {}
        self.external = np.zeros(self.arm.n)        # N*m on each link from outside (the observer's residual)
        self._set_payload(payload_kg)

    def _set_payload(self, payload_kg: float) -> None:
        import mujoco
        self._mj, self.payload_kg = mujoco, payload_kg
        self._m = build_model(self.arm, payload_kg)
        self._m.dof_armature[compliant_joints(self.arm)] = 0        # the rotor's inertia is behind the gear
        self._d = mujoco.MjData(self._m)
        self._hist, self._z, self._over = [], None, np.zeros(self.arm.n)

    # ------------------------------------------------------------------ Backend API ----
    @property
    def enabled(self) -> bool:
        return self.inner.enabled

    @property
    def faults(self) -> dict:
        return {**self.inner.faults, **self.contact}

    def __getattr__(self, name):
        if name == "inner":
            raise AttributeError(name)
        return getattr(self.inner, name)

    def enable(self) -> None:
        self.contact.clear()
        self.inner.enable()
        self._set_payload(self.payload_kg)

    def estop(self) -> None:
        self.inner.estop()

    def command(self, q) -> None:
        if not self.contact:                        # after a trip only enable() moves the arm again
            self.inner.command(q)

    def read(self) -> np.ndarray:
        return self.inner.read()

    def read_motor(self) -> np.ndarray:
        return self.inner.read_motor()

    def step(self, dt: float) -> None:
        self.inner.step(dt)
        if not self.inner.enabled:
            return
        self.observe(self.inner.read(), self.inner.read_motor(), dt)
        self._over = np.where(self.mask & (np.abs(self.external) > self.threshold), self._over + dt, 0.0)
        hit = self._over >= self.debounce - 1e-9
        if hit.any() and not self.contact:
            for i in np.flatnonzero(hit):
                name = self.arm.joints[i].name
                self.contact[name] = (f"{name}: contact, {self.external[i]:+.2f} N·m from outside "
                                      f"(threshold {self.threshold[i]:.2f})")
            self._stop = self._hold = self.inner.read_motor()               # stop where the motors are
        if self.contact:            # then give way to the push until it fades, at most back_max from the stop
            give = self.mask & (np.abs(self.external) > self.threshold / 2)
            step = np.where(give, self.back_off * dt * self.external / np.where(self.k > 0, self.k, np.inf), 0.0)
            self._hold = np.clip(self._hold + step, self._stop - self.back_max, self._stop + self.back_max)
            self.inner.command(self._hold)

    # ------------------------------------------------------------------ the estimate ----
    def gear_torque(self, link, motor) -> np.ndarray:
        """Torque through each gearbox, k * (motor - link) with the backlash gap taken out (0 inside it)."""
        d = np.asarray(motor) - np.asarray(link)
        return self.k * np.sign(d) * np.maximum(np.abs(d) - self._half_b, 0.0)

    def static_external(self, link, motor) -> np.ndarray:
        """External torque if the arm is still: what gravity minus the spring needs, less what the gear carries."""
        return (self.arm.gravity_torque(link, self.payload_kg) - spring_torque(self.arm, link)
                - self.gear_torque(link, motor))

    def observe(self, link, motor, dt: float) -> np.ndarray:
        """One poll of the momentum observer: r = gain * (p - integral of (tau_gear + spring - bias + Mdot qd + r))."""
        self._hist = (self._hist + [(np.asarray(link, float), np.asarray(motor, float))])[-2:]
        if len(self._hist) < 2:                     # the first poll (enable): the arm is still, p = 0
            self.external, self._M = self.static_external(link, motor), None
            self._z = -self.external / self.gain
            return self.external
        (qa, ma), (qb, mb) = self._hist             # everything half a poll back, between the two readings
        q, qd = (qa + qb) / 2, (qb - qa) / dt
        self._d.qpos[:], self._d.qvel[:] = q, qd
        self._mj.mj_forward(self._m, self._d)
        M = np.zeros((self.arm.n, self.arm.n))
        self._mj.mj_fullM(self._m, self._d, M)
        Mdot = np.zeros_like(M) if self._M is None else (M - self._M) / dt
        tau = (self.gear_torque(qa, ma) + self.gear_torque(qb, mb)) / 2 + spring_torque(self.arm, q)
        self._z = self._z + dt * (tau - self._d.qfrc_bias + Mdot @ qd + self.external)
        self._M = M
        self.external = self.gain * (M @ qd - self._z)
        return self.external

    def settle(self) -> None:
        """At rest with nothing touching, after enable: CanBackend starts its motor angle at the link's, which hides the
        wind-up the load already causes. Put it back (motor = link + tau / k + b/2) so the guard doesn't read gravity as a push."""
        tau = (self.arm.gravity_torque(self.read(), self.payload_kg) - spring_torque(self.arm, self.read()))
        d = np.divide(tau, self.k, out=np.zeros_like(tau), where=self.k > 0) + self._half_b * np.sign(tau)
        for j in np.flatnonzero(self.mask):
            self.inner.set_angle(j, self.read()[j] + d[j])
        self.enable()

    def tare(self) -> float:
        """At rest with nothing touching: fit the payload to the residual (least squares on the guarded joints)."""
        q, m = self.read(), self.read_motor()
        g1 = (self.arm.gravity_torque(q, 1.0) - self.arm.gravity_torque(q, 0.0))[self.mask]
        r = self.static_external(q, m)[self.mask]
        self._set_payload(self.payload_kg - float(g1 @ r) / float(g1 @ g1))   # r = -(m - told) g1
        return self.payload_kg

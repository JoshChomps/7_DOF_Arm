"""Stepper torque vs speed: how much torque a 2-phase hybrid stepper can make at a given speed.

A stepper's *holding* torque is only available near standstill. As it spins, the back-EMF and the
winding inductance eat into the supply voltage, so the driver can no longer push the rated current
and the torque falls: flat up to a corner speed, then roughly as 1 / speed (constant power). On a
24 V supply a NEMA17 delivers only ~15-25 W at the shaft however it is geared, which is what really
limits how fast a loaded joint can move.

Model: the motor as a 2-phase synchronous machine in the rotor (d-q) frame, with p = 50 pole pairs
for a 1.8 deg motor::

    v_d = R i_d - X i_q                 X = p * w * L      (reactance at speed w, mechanical rad/s)
    v_q = R i_q + X i_d + E             E = Kt * w         (back-EMF amplitude)
    torque = Kt * i_q                   Kt = T_hold / (sqrt(2) * I_rated)

subject to |i| <= sqrt(2) * I_rated (same copper loss as both phases at rated current, which is how
holding torque is specified) and |v| <= V_supply (one H-bridge per phase). The envelope is the
largest i_q over all i_d <= 0, i.e. with ideal phase advance ("field weakening"). An open-loop
stepper does this by itself as its load angle grows; whether the MKS closed-loop firmware gets as
close is something to measure on the bench.

The envelope is an **upper bound**: iron losses, detent torque, mid-band resonance and pull-out
margin are not modelled. Plan with a derating factor (``[drive] torque_derate`` in ``arm.toml``,
default 0.75) and compare with the pull-out curve in the motor's datasheet.

Derivation, the constant-power result and plots for this arm's motors: docs/14_math_guide.md §11.
"""
from __future__ import annotations

import numpy as np

SQRT2 = float(np.sqrt(2.0))


def stepper_torque(speed, hold_torque: float, rated_current: float, resistance: float,
                   inductance: float, voltage: float = 24.0, pole_pairs: int = 50) -> np.ndarray:
    """Torque envelope (N*m, motor shaft) at motor ``speed`` (rad/s, scalar or array).

    ``resistance`` (ohm) and ``inductance`` (H) are per phase, ``rated_current`` is the datasheet
    current per phase (A). With ``inductance <= 0`` the curve is flat at the holding torque.
    """
    w = np.abs(np.atleast_1d(np.asarray(speed, dtype=float)))
    if hold_torque <= 0 or rated_current <= 0:
        return np.zeros_like(w)
    if inductance <= 0:
        return np.full_like(w, hold_torque)
    i_max = SQRT2 * rated_current
    kt = hold_torque / i_max
    X = (pole_pairs * w * inductance)[:, None]
    E = (kt * w)[:, None]
    R = resistance
    i_d = -i_max * np.linspace(0.0, 1.0, 201)[None, :]
    # voltage limit, a quadratic in i_q:  (X^2 + R^2) i_q^2 + 2 R E i_q + C <= 0
    A = X**2 + R**2
    C = (R * i_d) ** 2 + (X * i_d + E) ** 2 - voltage**2
    disc = (R * E) ** 2 - A * C
    with np.errstate(invalid="ignore", divide="ignore"):
        iq_voltage = np.where((disc >= 0) & (A > 0),
                              (-R * E + np.sqrt(np.maximum(disc, 0.0))) / np.maximum(A, 1e-30), 0.0)
    iq_current = np.sqrt(np.maximum(i_max**2 - i_d**2, 0.0))
    i_q = np.maximum(np.minimum(iq_voltage, iq_current), 0.0).max(axis=1)
    return kt * i_q


def corner_speed(hold_torque: float, rated_current: float, resistance: float, inductance: float,
                 voltage: float = 24.0, pole_pairs: int = 50, fraction: float = 0.95) -> float:
    """Motor speed (rad/s) at which the envelope drops below ``fraction`` of the holding torque."""
    w = np.linspace(0.0, 400.0, 4001)
    t = stepper_torque(w, hold_torque, rated_current, resistance, inductance, voltage, pole_pairs)
    below = np.flatnonzero(t < fraction * hold_torque)
    return float(w[below[0]]) if below.size else float(w[-1])

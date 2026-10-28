"""Starting the arm without a jig: saved angles, hall checks and homing (docs/16 §3.3).

The drivers forget their turn count when the 24 V rail drops (an E-stop, a power cut), but the host keeps
running and knows the last angles, and the motor encoders still know their angle within one turn. So:

* ``save`` writes the last angles to a small JSON file (and, once, the reference pair from
  ``CanBackend.reference()`` taken at a known pose). Call it after an E-stop and at shutdown.
* ``start_angles`` picks each joint's start: an output-encoder reading where there is one (D33: J2-J4),
  else the saved angle. Saved angles are only *trusted* on joints with a hall switch (J1, J7: no gravity,
  so they don't move while unpowered); the others may have sagged and need a coarse sensor or the jig.
  Pass the result as ``CanBackend(..., q_start, ref=...)`` and ``absolute_angle`` makes it exact.
* The hall switch then checks the trusted joints as they move (``CanBackend._check_halls``). If one was
  moved by hand, ``home`` sweeps it onto its switch and re-zeros it.

Not yet run on hardware: the hall polarity, band width and IN_1 input level are bench questions.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

import numpy as np

DEFAULT = Path.home() / ".deskarm" / "state.json"


def load(path: str | Path = DEFAULT) -> dict:
    """The saved state: {"q_deg": [...], "saved": "...", "ref": {"q_deg": [...], "turns": [...]}} or {}."""
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def save(q, path: str | Path = DEFAULT, ref=None) -> None:
    """Save the joint angles (rad), and ``ref`` = ``CanBackend.reference()`` if given (else keep the old one)."""
    state = load(path)
    state.update(q_deg=np.degrees(np.asarray(q, float)).round(4).tolist(), saved=time.strftime("%Y-%m-%d %H:%M:%S"))
    if ref is not None:
        state["ref"] = {"q_deg": np.degrees(np.asarray(ref[0], float)).round(4).tolist(),
                        "turns": np.asarray(ref[1], float).round(6).tolist()}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(state, indent=1), encoding="utf-8")
    os.replace(tmp, path)                       # never leave a half-written file


def reference(state: dict):
    """The saved reference as (angles in rad, turns), ready for ``CanBackend(ref=...)``, or None."""
    r = state.get("ref")
    return None if not r else (np.radians(r["q_deg"]), np.asarray(r["turns"], float))


def start_angles(arm, state: dict, coarse=None):
    """(q_start, trusted): per joint the coarse reading if given (not NaN), else the saved angle, else home.

    ``trusted[j]`` says the start can be made exact: a coarse reading, or a saved angle on a hall joint.
    Untrusted joints must stand at ``arm.home`` (the jig) or be homed.
    """
    n = arm.n
    q = np.array(arm.home, float)
    trusted = np.zeros(n, bool)
    saved = state.get("q_deg")
    if saved is not None and len(saved) == n:
        hall = np.array([jt.hall is not None for jt in arm.joints])
        q[hall] = np.radians(saved)[hall]
        trusted |= hall
    if coarse is not None:
        c = np.asarray(coarse, float)
        have = ~np.isnan(c)
        q[have], trusted[have] = c[have], True
    return q, trusted


def home(backend, j: int, speed_deg_s: float = 10.0, travel_deg: float | None = None, dt: float = 0.02,
         clock=time.monotonic, sleep=time.sleep) -> float:
    """Sweep joint ``j`` onto its hall switch and re-zero it there. Returns the correction (rad).

    Sweeps toward where the switch should be, up to ``travel_deg`` (default: the joint's whole range),
    then back the other way. The switch turns on at the band's edge on the side it is approached
    from, so that edge sets the angle. Safe on J1 (yaw) and J7 (roll); not for a joint under gravity.
    """
    jt = backend.arm.joints[j]
    if jt.hall is None:
        raise ValueError(f"{jt.name} has no hall switch (hall_deg in arm.toml)")
    travel = np.radians(travel_deg) if travel_deg is not None else jt.upper - jt.lower
    backend.homing.add(j)
    try:
        q0 = backend.read().copy()
        first = 1.0 if jt.hall >= q0[j] else -1.0
        for direction in (first, -first):                # toward it, then back past the start
            start, was = backend.read()[j], backend.hall(j)
            span = travel if direction == first else 2 * travel
            t0, next_t = clock(), clock()
            # the soft limits may stop the sweep short of span, so give up on time as well
            while abs(backend.read()[j] - start) < span and clock() - t0 < span / np.radians(speed_deg_s) + 2:
                target = backend.read().copy()
                target[j] = start + direction * np.radians(speed_deg_s) * (clock() - t0 + dt)
                backend.command(target)
                backend.step(dt)
                if backend.faults:
                    raise RuntimeError(f"homing {jt.name} stopped: {backend.faults}")
                on = backend.hall(j)
                if on and was is False:                  # rising edge: the band's near side
                    believed = backend.read()[j]
                    edge = jt.hall - direction * jt.hall_width / 2
                    backend.set_angle(j, edge)
                    return edge - believed
                was = on if on is not None else was
                next_t += dt
                sleep(max(0.0, next_t - clock()))
        raise RuntimeError(f"{jt.name}: no hall edge found within {np.degrees(travel):.0f} deg each way")
    finally:
        backend.homing.discard(j)

"""Smoke test: boot the twin headless and drive its callbacks like a browser would."""
import socket
import types

import numpy as np
import pytest

viser = pytest.importorskip("viser")
pytest.importorskip("yourdfpy")

from deskarm import SimBackend, load_arm  # noqa: E402
from deskarm.ik import J3  # noqa: E402
from deskarm.twin import TwinApp, rot_to_wxyz, wxyz_to_rot  # noqa: E402

ARM = load_arm()
CLIENT = types.SimpleNamespace(client=object())   # fake "came from a browser" event


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def app():
    a = TwinApp(ARM, SimBackend(ARM), port=free_port())
    yield a
    a.server.stop()


def test_quaternion_round_trip():
    rng = np.random.default_rng(5)
    for _ in range(200):
        R = ARM.fk(ARM.lower + (ARM.upper - ARM.lower) * rng.random(ARM.n))[:3, :3]
        assert np.allclose(wxyz_to_rot(rot_to_wxyz(R)), R, atol=1e-9)


def test_gizmo_drag_solves_ik(app):
    T = ARM.fk(ARM.home)
    app.gizmo.position = T[:3, 3] + np.array([0.03, -0.02, -0.04])
    app.gizmo.wxyz = rot_to_wxyz(T[:3, :3])
    app._on_gizmo(None)
    assert np.allclose(ARM.fk(app.q_ghost)[:3, 3], app.gizmo.position, atol=1e-3)


def test_lock_and_swivel_and_mode(app):
    app.lock_boxes[0].value = True
    app._on_lock(CLIENT, 0)
    q1_before = app.q_ghost[0]
    app.gizmo.position = np.asarray(app.gizmo.position) + np.array([0.0, 0.03, 0.0])
    app._on_gizmo(None)
    assert app.q_ghost[0] == q1_before            # J1 held fixed
    app.lock_boxes[0].value = False
    app._on_lock(CLIENT, 0)

    T_before = ARM.fk(app.q_ghost)
    app.swivel.value = float(app.swivel.value) + 20.0
    app._on_swivel(CLIENT)
    assert np.allclose(ARM.fk(app.q_ghost)[:3, 3], T_before[:3, 3], atol=1e-3)

    app.mode.value = "6-DOF (J3 locked)"
    app._on_mode(CLIENT)
    assert app.q_ghost[J3] == 0.0
    app.mode.value = "7-DOF (redundant)"


def test_execute_moves_sim_backend_to_ghost(app):
    app._set_ghost(ARM.home + np.deg2rad([20, -10, 15, 20, 0, -20, 45]), move_gizmo=True)
    app._enable()
    app.speed.value = 1.0
    app._execute()
    app.run(rate_hz=100, duration=app.traj.duration + 0.5)
    assert np.allclose(app.backend.read(), app.q_ghost, atol=1e-3)
    app._estop()
    assert not app.backend.enabled


def test_execute_through_can_backend_on_simulated_drivers():
    """The twin -> CanBackend -> MKS frames -> simulated drivers path, in real time at the twin's 50 Hz."""
    pytest.importorskip("can")
    from deskarm.backends import CanBackend
    from deskarm.mks import SimBus

    a = TwinApp(ARM, CanBackend(ARM, SimBus(), ARM.home), port=free_port())
    try:
        a._set_ghost(ARM.home + np.deg2rad([15, -10, 10, 20, -15, 20, 45]), move_gizmo=True)
        a._enable()
        a.speed.value = 1.0
        a._execute()
        a.run(rate_hz=50, duration=a.traj.duration + 0.5)
        assert np.allclose(a.backend.read(), a.q_ghost, atol=1e-3)
        assert not a.backend.faults
    finally:
        a.server.stop()


def test_constraint_modes(app):
    app.mode.value = "7-DOF (redundant)"
    for name in ("Lock orientation (translate only)", "Horizontal plane (fixed height + orientation)"):
        app._set_ghost(ARM.home.copy(), move_gizmo=True)
        app.constraint.value = name
        app._on_constraint(CLIENT)
        R0, z0 = ARM.fk(app.q_ghost)[:3, :3].copy(), ARM.fk(app.q_ghost)[2, 3]
        app.gizmo.position = np.asarray(app.gizmo.position) + np.array([0.03, 0.02, 0.02])
        app._on_gizmo(None)
        T = ARM.fk(app.q_ghost)
        assert np.allclose(T[:3, :3], R0, atol=1e-3)             # orientation held
        if name.startswith("Horizontal"):
            assert abs(T[2, 3] - z0) < 1e-3                       # height held
        assert app.gizmo.disable_rotations

    app.constraint.value = "Keep tool vertical (pointing down)"
    app._on_constraint(CLIENT)
    assert ARM.fk(app.q_ghost)[2, 2] < -0.999                     # tool z points down
    app.gizmo.position = np.asarray(app.gizmo.position) + np.array([0.02, -0.03, 0.0])
    app._on_gizmo(None)
    assert ARM.fk(app.q_ghost)[2, 2] < -0.999

    app.constraint.value = "Full pose (move + rotate)"
    app._on_constraint(CLIENT)
    assert not app.gizmo.disable_rotations


def test_desk_guard_blocks_execution(app):
    app.constraint.value = "Full pose (move + rotate)"
    app._enable()
    q_bad = ARM.home.copy()
    q_bad[1], q_bad[3] = np.deg2rad(110), np.deg2rad(60)          # tool driven into the table
    app._set_ghost(q_bad, move_gizmo=True)
    assert "below the desk" in app.readout.content
    app._execute()
    assert app.traj is None and "Not executed" in app.readout.content
    app._set_ghost(ARM.home.copy(), move_gizmo=True)


def test_straight_line_execute_and_program_playback(app, tmp_path):
    app.constraint.value = "Full pose (move + rotate)"
    app._on_constraint(CLIENT)
    app._set_ghost(ARM.home.copy(), move_gizmo=True)
    app._enable()
    app.speed.value = 1.0
    app.backend._q = ARM.home.copy() if hasattr(app.backend, "_q") else None
    # straight line to a shifted ghost
    T = ARM.fk(ARM.home)
    T[:3, 3] += np.array([0.0, 0.08, -0.03])
    from deskarm.ik import solve_ik
    app._set_ghost(solve_ik(ARM, T, ARM.home).q, move_gizmo=True)
    app._execute_line()
    assert app.traj is not None and "straight line" in app.readout.content
    app.run(rate_hz=100, duration=app.traj.duration + 0.3)
    assert np.linalg.norm(ARM.fk(app.backend.read())[:3, 3] - T[:3, 3]) < 1e-3

    # record two waypoints, save, load, play
    app._prog_clear()
    app._set_ghost(ARM.home + np.deg2rad([20, 0, 0, 0, 0, 0, 0]), move_gizmo=True)
    app.prog_move.value = "joint"
    app._prog_record()
    app._set_ghost(ARM.home + np.deg2rad([-15, 5, 0, -5, 0, 0, 0]), move_gizmo=True)
    app.prog_move.value = "linear"
    app._prog_record()
    assert len(app.program.waypoints) == 2 and "P2: linear" in app.prog_list.content
    app.prog_file.value = str(tmp_path / "demo.json")
    app._prog_save()
    app._prog_clear()
    app._prog_load()
    assert len(app.program.waypoints) == 2
    app._prog_preview()
    assert "Path preview: 2 moves" in app.readout.content
    app._prog_play()
    assert app.traj is not None and len(app.queue) == 1
    total = app.traj.duration + app.queue[0].duration
    app.run(rate_hz=100, duration=total + 0.5)
    q_end = app.backend.read()
    assert np.linalg.norm(ARM.fk(q_end)[:3, 3] - ARM.fk(np.array(app.program.waypoints[1].q))[:3, 3]) < 1e-3


def test_twin_on_physics_backend_with_torque_guard():
    pytest.importorskip("mujoco")
    from deskarm.physics import MujocoBackend
    be = MujocoBackend(ARM, payload_kg=4.0)
    a = TwinApp(ARM, be, port=free_port(), payload_kg=4.0)
    try:
        a._enable()
        q_far = ARM.home.copy()
        q_far[1] = np.deg2rad(90)                       # stretch out with a 4 kg payload
        q_far[3] = 0.0
        q_far[5] = 0.0
        a._set_ghost(q_far, move_gizmo=True)
        assert "can't hold this pose" in a.readout.content
        a._execute()
        assert a.traj is None and "Not executed" in a.readout.content
        a.torque_guard.value = False                    # run it anyway: the physics stalls a joint
        a._execute()
        a.run(rate_hz=100, duration=3.0)
        assert not be.enabled and be.faults
        assert "stalled" in a.status.content
    finally:
        a.server.stop()


def test_motion_profiles_and_self_collision_guard(app):
    app.constraint.value = "Full pose (move + rotate)"
    app._on_constraint(CLIENT)
    app._enable()
    app.speed.value = 1.0
    for name in ("S-curve (jerk-limited, smooth)", "Time-optimal (TOPP-RA, torque-aware)"):
        app.profile.value = name
        target = ARM.home + np.deg2rad([25, -10, 10, 15, 0, -15, 30])
        app._set_ghost(target, move_gizmo=True)
        app._execute()
        assert app.traj is not None, app.readout.content
        app.run(rate_hz=100, duration=app.traj.duration + 0.3)
        assert np.allclose(app.backend.read(), target, atol=1e-3)
        app._set_ghost(ARM.home.copy(), move_gizmo=True)
        app._execute()
        app.run(rate_hz=100, duration=app.traj.duration + 0.3)
    app.profile.value = "Trapezoid (fast, simple)"

    tucked = np.deg2rad([0, -30, 0, 120, 0, 90, 0])
    app._set_ghost(tucked, move_gizmo=True)
    assert "self-collision" in app.readout.content
    app._execute()
    assert app.traj is None and "Not executed" in app.readout.content and "hits" in app.readout.content
    app._set_ghost(ARM.home.copy(), move_gizmo=True)

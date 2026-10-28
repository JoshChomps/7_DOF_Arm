import io

import numpy as np
import pytest

from deskarm import SimBackend, TrapezoidalTrajectory, load_arm, to_urdf

ARM = load_arm()


def test_trapezoid_hits_endpoints_and_respects_limits():
    q0, q1 = ARM.home, ARM.home + np.deg2rad([40, -20, 10, 30, 90, -45, 180])
    tr = TrapezoidalTrajectory(q0, q1, ARM.vmax, ARM.amax)
    assert np.allclose(tr.sample(0)[0], q0)
    assert np.allclose(tr.sample(tr.duration)[0], q1)
    ts = np.linspace(0, tr.duration, 400)
    qs = np.array([tr.sample(t)[0] for t in ts])
    vs = np.array([tr.sample(t)[1] for t in ts])
    assert np.all(np.abs(vs) <= ARM.vmax + 1e-9)
    acc = np.diff(vs, axis=0) / np.diff(ts)[:, None]
    assert np.all(np.abs(acc) <= ARM.amax * 1.05 + 1e-6)
    assert np.all(np.diff(qs * np.sign(q1 - q0), axis=0) >= -1e-12)  # monotonic


def test_trapezoid_zero_move():
    tr = TrapezoidalTrajectory(ARM.home, ARM.home, ARM.vmax, ARM.amax)
    assert tr.duration == 0.0 and np.allclose(tr.sample(1.0)[0], ARM.home)


def test_sim_backend_tracks_and_estops():
    be = SimBackend(ARM)
    target = ARM.home + 0.2
    be.command(target)                      # ignored while disabled
    be.step(1.0)
    assert np.allclose(be.read(), ARM.home)
    be.enable()
    be.command(target)
    for _ in range(200):
        be.step(0.01)
    assert np.allclose(be.read(), target)
    be.command(ARM.home)
    be.step(0.01)
    be.estop()
    q = be.read()
    be.step(1.0)
    assert np.allclose(be.read(), q)


def test_urdf_matches_dh_fk_yourdfpy():
    yourdfpy = pytest.importorskip("yourdfpy")
    robot = yourdfpy.URDF.load(io.StringIO(to_urdf(ARM)))
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = ARM.lower * 0.8 + (ARM.upper - ARM.lower) * 0.8 * rng.random(ARM.n)
        robot.update_cfg(q)
        T = robot.get_transform("tcp", "base_link")
        assert np.allclose(T, ARM.fk(q), atol=1e-5)


def test_urdf_loads_in_mujoco_and_matches_fk():
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string(to_urdf(ARM))
    data = mujoco.MjData(model)
    assert model.nq == ARM.n
    q = np.deg2rad([20, 30, -40, 70, 10, 45, 90])
    data.qpos[:] = q
    mujoco.mj_kinematics(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link7")
    assert np.allclose(data.xpos[body], ARM.frames(q)[6][:3, 3], atol=1e-5)


def test_sim_backend_quantizes_to_steps():
    step = 2 * np.pi / (200 * 16 * 30)          # 1.8 deg stepper, 16x microstep, 30:1 gearbox
    be = SimBackend(ARM, q0=np.zeros(ARM.n), resolution=step)
    be.enable()
    be.command(np.full(ARM.n, 0.1234567))
    for _ in range(300):
        be.step(0.01)
    q = be.read()
    assert np.allclose(q / step, np.round(q / step))   # always on a whole step
    assert np.all(np.abs(q - 0.1234567) <= step / 2 + 1e-12)


def test_urdf_with_cad_meshes():
    """The CAD meshes (tools/cad_meshes.py) load on every link, in the right place: at q = 0 the arm stands on the
    board and the tool body (gripper) is centred on the J7 axis and ends above the TCP."""
    yourdfpy = pytest.importorskip("yourdfpy")
    from deskarm.urdf import MESH_DIR
    if not MESH_DIR.is_dir():
        pytest.skip("no CAD meshes")
    robot = yourdfpy.URDF.load(io.StringIO(to_urdf(ARM, meshes=MESH_DIR)))
    links = {link.name: link for link in robot.robot.links}
    for name in ["base_link"] + [f"link{i}" for i in range(1, 8)]:
        assert links[name].visuals and all(v.geometry.mesh for v in links[name].visuals), name
    assert not links["tcp"].visuals                     # the gripper is in link7's meshes
    tool = [g for n, g in robot.scene.geometry.items() if n.startswith("7_printed")][0]
    lo, hi = tool.bounds
    assert np.allclose((lo[:2] + hi[:2]) / 2, 0, atol=0.005)
    assert hi[2] > ARM.joints[-1].d + ARM.tool[2, 3]    # link7 frame: z along J7 from the wrist centre to the TCP

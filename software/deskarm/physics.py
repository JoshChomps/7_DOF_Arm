"""Physics simulation (MuJoCo): gravity, inertia, motor torque limits and stalls.

``MujocoBackend`` is a drop-in replacement for ``SimBackend`` (same Backend interface), so the
twin and scripts run unchanged on it:

* Every joint is driven like a closed-loop stepper behind a gearbox: a stiff position loop whose
  torque is **saturated at the joint's torque limit** (motor torque x ratio x efficiency, from
  ``arm.toml``), plus the motor's rotor inertia reflected through the gearbox (``armature``).
* The limit **falls with speed** along the stepper's torque-speed curve (``deskarm.motor``): full
  torque up to the corner speed, then roughly constant power. A joint asked to move a heavy load
  fast runs out of torque and stalls, even though it could hold or move the same load slowly.
* Link masses and inertias come from the ``[[masses]]`` list in ``arm.toml`` (motors, drivers,
  gearboxes, printed links, tool), the same list the torque budget and spring sizing use.
* If gravity or acceleration needs more torque than a joint has, the joint lags behind its
  reference. A sustained lag is reported as a **stall fault** and the arm stops, which is what the
  MKS drivers do.
* How a driver follows a new target: by default it **chases** it at the joint's full speed and
  then waits, so a target streamed every 20 ms gives stop-go motion at the poll rate. With
  ``streamed=True`` it moves at (distance left) / dt over the step instead, which is
  ``CanBackend``'s F5 rule (speed = distance left / step) if the drivers follow it smoothly (Q8).
* **Gearbox compliance and backlash** (``compliant=True``, the default when ``arm.toml`` has
  ``gear_stiffness``): each joint is two masses. The motor (rotor inertia x ratio^2, driven by the
  servo) and the link are joined by a torsional spring (``gear_stiffness``) with a dead band
  (``backlash_deg``). The link sags under load by tau / k + backlash / 2 even though the motor is
  exactly where it was told to be, which is what the output encoders will show on the real arm.
  ``read()`` is the link (output encoder), ``read_motor()`` the motor side (motor encoder).
* **Counterbalance spring** on J2 (``[counterbalance]`` in ``arm.toml``, see
  ``deskarm.counterbalance``): applied as a joint torque at all times, also when the motors are off.
* E-stop / disabled: printed high-ratio gearboxes aren't backdrivable, so the arm holds its pose
  (``backdrivable=False``, the default). With ``backdrivable=True`` (a BLDC quasi-direct-drive arm)
  it falls under gravity, so you can see why those arms need brakes or a counterbalance.
* ``external_torque`` (per joint, N*m) pushes on the links: a hand pushing the arm, or the bench
  test that measures stiffness and backlash.
* ``tool_force`` (N, base frame) pushes on the tool centre point: a fixed vector, or a function of
  the TCP position called every physics step (a wall the tool runs into: ``wall``).

``TorqueModel`` does inverse dynamics (tau = M(q) qdd + C(q, qd) qd + g(q), including armature)
along a planned trajectory, so a move can be rejected **before** it's executed. The counterbalance
spring's share is subtracted, so the numbers are what the motors must supply, and each sample is
compared with the torque the motor has left **at that speed**.

MuJoCo contacts are off: ``tool_force`` stands in for a touch, and the desk guard (``deskarm.ik``) and the self-collision check
(``deskarm.collision``) reject bad moves before they run.

Math, with diagrams (servo gains, two-mass joint, wind-up, ringing frequency): docs/14_math_guide.md §12.
"""
from __future__ import annotations

import threading

import numpy as np

from .counterbalance import spring_torque
from .kinematics import ArmModel
from .urdf import to_urdf


def _servo_gains(arm: ArmModel, model, max_lag_steps: float, motor_step_deg: float):
    """Stepper-like position-loop gains: full torque at ~1 motor step of lag, critically damped.

    Assumption: a hybrid stepper's torque vs rotor error is ~T_hold sin(50 theta), i.e. peak
    torque at one full step (1.8 deg) and small-signal stiffness 50 T_hold per motor radian. This
    linear "usable torque at one step" is ~2-3x softer than that (and the MKS closed loop may use a
    reduced hold current), which is fine here because the printed gearbox is far softer still.
    """
    import mujoco

    lim = arm.torque_limit()
    ratios = np.array([max(j.ratio, 1.0) for j in arm.joints])
    lag = np.deg2rad(motor_step_deg * max_lag_steps) / ratios
    kp = np.where(np.isfinite(lim), lim / lag, 200.0)
    d = mujoco.MjData(model)
    d.qpos[:] = arm.home
    mujoco.mj_forward(model, d)
    M = np.zeros((arm.n, arm.n))
    mujoco.mj_fullM(model, d, M)
    kv = 2.0 * np.sqrt(kp * np.diag(M))
    return kp, kv, lim, M


def _mjcf(model) -> str:
    import os
    import tempfile

    import mujoco

    fd, path = tempfile.mkstemp(suffix=".xml")
    os.close(fd)
    try:
        mujoco.mj_saveLastXML(path, model)
        with open(path) as f:
            return f.read()
    finally:
        os.remove(path)


def wall(point, normal, stiffness: float = 5000.0):
    """``tool_force`` for a flat wall through ``point`` facing ``normal``: a spring (N/m) once the TCP is behind it."""
    point, n = np.asarray(point, float), np.asarray(normal, float) / np.linalg.norm(normal)
    return lambda p: stiffness * max(0.0, float(n @ (point - p))) * n


def compliant_joints(arm: ArmModel) -> list[int]:
    return [i for i, j in enumerate(arm.joints) if j.gear_stiffness > 0]


def build_model(arm: ArmModel, payload_kg: float = 0.0, timestep: float = 0.0005,
                actuators: bool = False, compliant: bool = False, max_lag_steps: float = 1.0,
                motor_step_deg: float = 1.8, gear_damping_ratio: float = 0.1):
    """MuJoCo model of the arm (from the generated URDF), with armature and no contacts.

    With ``actuators=True`` every joint gets a torque-limited position servo (kp, kv, forcerange)
    integrated implicitly (``implicitfast``), which stays stable at stepper-like stiffness.

    With ``compliant=True`` every joint with a ``gear_stiffness`` gets a separate motor rotor: a
    hinge ``<name>_motor`` on the parent link, on the same axis, carrying the reflected rotor
    inertia. A fixed tendon (link angle - motor angle) with that stiffness and a dead band of
    +-backlash/2 is the gearbox, and the servo drives the motor joint instead of the link. The
    gearbox's internal damping is set so the link ringing on the gear spring has
    ``gear_damping_ratio``, computed per joint on the home-pose inertia. This is an assumption, not
    a measurement: polymer loss factors are ~0.02-0.1 (zeta = half that) and gear/bearing friction
    adds more. The coupled first mode of the default arm comes out at zeta ~0.05 (4-7 Hz).
    Limitations: the damping also acts across the backlash gap (real gears transmit nothing
    there), and there is no Coulomb friction, so a link in its dead band drifts instead of sticking.
    """
    import xml.etree.ElementTree as ET

    import mujoco

    model = mujoco.MjModel.from_xml_string(to_urdf(arm, payload_kg=payload_kg))
    for i, j in enumerate(arm.joints):
        model.dof_armature[i] = j.armature
    if actuators or compliant:
        kp, kv, lim, M = _servo_gains(arm, model, max_lag_steps, motor_step_deg)
        root = ET.fromstring(_mjcf(model))
        parent_of = {c: p for p in root.iter() for c in p}
        bodies = {jt.get("name"): b for b in root.iter("body") for jt in b.findall("joint")}
        flex = compliant_joints(arm) if compliant else []
        tendons = ET.Element("tendon")
        for i, j in enumerate(arm.joints):
            jt = bodies[j.name].find("joint")
            if i not in flex:
                jt.set("armature", f"{j.armature:.6g}")
                continue
            jt.set("armature", "0")
            link = bodies[j.name]
            rotor = ET.Element("body", {"name": f"{j.name}_rotor"})
            for key in ("pos", "quat"):
                if link.get(key):
                    rotor.set(key, link.get(key))
            ET.SubElement(rotor, "inertial", {"pos": "0 0 0", "mass": "1e-4",
                                              "diaginertia": "1e-9 1e-9 1e-9"})
            ET.SubElement(rotor, "joint", {"name": f"{j.name}_motor", "limited": "false",
                                           "armature": f"{max(j.armature, 1e-6):.6g}"})
            parent = parent_of[link]
            parent.insert(list(parent).index(link), rotor)
            link_inertia = max(M[i, i] - j.armature, 1e-6)
            c = 2 * gear_damping_ratio * np.sqrt(j.gear_stiffness * link_inertia)
            fixed = ET.SubElement(tendons, "fixed", {
                "name": f"{j.name}_gear", "stiffness": f"{j.gear_stiffness:.6g}", "damping": f"{c:.6g}",
                "springlength": f"{-j.backlash / 2:.6g} {j.backlash / 2:.6g}"})
            ET.SubElement(fixed, "joint", {"joint": j.name, "coef": "1"})
            ET.SubElement(fixed, "joint", {"joint": f"{j.name}_motor", "coef": "-1"})
        if len(tendons):
            root.append(tendons)
        if actuators:
            act = ET.SubElement(root, "actuator")
            for i, j in enumerate(arm.joints):
                a = {"name": f"{j.name}_servo", "joint": j.name + ("_motor" if i in flex else ""),
                     "kp": f"{kp[i]:.6g}", "kv": f"{kv[i]:.6g}"}
                if np.isfinite(lim[i]):
                    a.update(forcelimited="true", forcerange=f"{-lim[i]:.6g} {lim[i]:.6g}")
                ET.SubElement(act, "position", a)
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    model.opt.timestep = timestep
    model.opt.integrator = int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    return model


class TorqueModel:
    """Inverse dynamics for feasibility checks (no simulation state)."""

    def __init__(self, arm: ArmModel, payload_kg: float = 0.0):
        import mujoco

        self._mj = mujoco
        self.arm = arm
        self.payload_kg = payload_kg
        self.model = build_model(arm, payload_kg)
        self.data = mujoco.MjData(self.model)
        self.limits = arm.torque_limit()             # near standstill

    def limits_at(self, qd) -> np.ndarray:
        """Torque limits at joint speeds ``qd`` (the stepper torque-speed curve)."""
        return self.arm.torque_limit(qd)

    def inverse(self, q, qd=None, qdd=None) -> np.ndarray:
        """Motor torques needed to hold / move along (q, qd, qdd), net of the counterbalance."""
        n = self.arm.n
        self.data.qpos[:] = q
        self.data.qvel[:] = np.zeros(n) if qd is None else qd
        self.data.qacc[:] = np.zeros(n) if qdd is None else qdd
        self._mj.mj_inverse(self.model, self.data)
        return self.data.qfrc_inverse - spring_torque(self.arm, q)

    def gravity(self, q) -> np.ndarray:
        return self.inverse(q)

    def trajectory_peak(self, traj, samples: int = 100):
        """Peak |torque| / limit per joint along a trajectory, and the time of the worst sample.

        The limit is the one at each sample's joint speed, so a fast move with a load can fail
        where the same path taken slowly passes.
        """
        n = self.arm.n
        peak = np.zeros(n)
        worst_t = 0.0
        for t, q, qd, qdd in _states(traj, samples):
            use = np.abs(self.inverse(q, qd, qdd)) / self.limits_at(qd)
            if use.max() > peak.max():
                worst_t = t
            peak = np.maximum(peak, use)
        return peak, worst_t


def _states(traj, samples: int):
    """(t, q, qd, qdd) along a trajectory.

    Dense waypoint trajectories (straight lines, time-optimal moves) use their own waypoints, so
    the accelerations are the ones that will actually be commanded; other trajectories are sampled
    ``samples`` times with exact (S-curve) or finite-difference accelerations.
    """
    T = traj.duration
    if hasattr(traj, "times") and hasattr(traj, "qs") and len(traj.times) > 2:
        ts, qs = traj.times, traj.qs
        v = np.diff(qs, axis=0) / np.diff(ts)[:, None]
        mid = (ts[:-1] + ts[1:]) / 2
        acc = np.diff(v, axis=0) / np.diff(mid)[:, None]
        v = np.vstack([v, np.zeros((1, qs.shape[1]))])          # ends at rest
        acc = np.vstack([v[:1] / max(mid[0], 1e-9), acc, -v[-2:-1] / max(ts[-1] - mid[-1], 1e-9)])
        step = max(1, (len(ts) - 2) // max(samples, 1))
        for k in range(0, len(ts) - 1, step):
            yield ts[k], qs[k], v[k], acc[k]
        return
    dt = max(T / samples, 1e-3)
    for k in range(samples + 1):
        t = T * k / samples
        q, qd = traj.sample(t)
        if hasattr(traj, "sample_acc"):
            qdd = traj.sample_acc(t)
        elif T > 0:
            qdd = (traj.sample(min(t + dt, T))[1] - traj.sample(max(t - dt, 0.0))[1]) / (2 * dt)
        else:
            qdd = np.zeros_like(q)
        yield t, q, qd, qdd


class MujocoBackend:
    """Physics-simulated arm with torque-limited closed-loop stepper joints."""

    def __init__(self, arm: ArmModel, q0: np.ndarray | None = None, payload_kg: float = 0.0,
                 speed_scale: float = 1.0, backdrivable: bool = False, compliant: bool = True,
                 stall_deg: float = 5.0, stall_time: float = 0.15, max_lag_steps: float = 1.0,
                 motor_step_deg: float = 1.8, streamed: bool = False):
        import mujoco

        self._mj = mujoco
        self.arm = arm
        self.payload_kg = payload_kg
        self.streamed = streamed
        self.compliant = compliant and bool(compliant_joints(arm))
        self.model = build_model(arm, payload_kg, actuators=True, compliant=self.compliant,
                                 max_lag_steps=max_lag_steps, motor_step_deg=motor_step_deg)
        m = self.model
        self.data = mujoco.MjData(m)
        jid = lambda name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)  # noqa: E731
        links = [jid(j.name) for j in arm.joints]
        motors = [jid(j.name + "_motor") if jid(j.name + "_motor") >= 0 else jid(j.name)
                  for j in arm.joints]
        self._qa = m.jnt_qposadr[links]           # link angles (output encoder)
        self._va = m.jnt_dofadr[links]
        self._qm = m.jnt_qposadr[motors]          # motor angles in output units (motor encoder)
        self._vm = m.jnt_dofadr[motors]
        self.limits = arm.torque_limit()          # near standstill
        self._limit_now = self.limits.copy()      # at the current motor speeds
        self._limited = np.isfinite(self.limits)
        kp = m.actuator_gainprm[:, 0]
        kv = -m.actuator_biasprm[:, 2]
        self._kv_over_kp = np.where(kp > 0, kv / kp, 0.0)
        self._gain0 = m.actuator_gainprm.copy()
        self._bias0 = m.actuator_biasprm.copy()
        self.speed_scale = speed_scale
        self.backdrivable = backdrivable
        self.stall = np.deg2rad(stall_deg)
        self.stall_time = stall_time
        self.external_torque = np.zeros(arm.n)     # push on the links (N*m per joint)
        self.tool_force = np.zeros(3)              # push on the TCP (N, base frame), or f(p_tcp) -> N
        q = np.array(arm.home if q0 is None else q0, dtype=float)
        self.data.qpos[self._qm] = q
        # Start in static equilibrium: the motors sit at q and every link has already sagged onto
        # its gear spring (tau / k + backlash / 2). Starting with the springs slack would drop the
        # arm onto them and overshoot to twice the static torque, which is not what power-up does.
        sag = np.zeros(arm.n)
        if self.compliant:
            tau = arm.gravity_torque(q, payload_kg) - spring_torque(arm, q)
            for i in compliant_joints(arm):
                j = arm.joints[i]
                if abs(tau[i]) > 1e-6:           # unloaded joints can sit anywhere in the dead band
                    sag[i] = tau[i] / j.gear_stiffness + np.sign(tau[i]) * j.backlash / 2
        self.data.qpos[self._qa] = q - sag
        mujoco.mj_forward(m, self.data)
        self._tool_body = m.jnt_bodyid[links[-1]]  # the TCP is fused into the last link's body
        R, o = self.data.xmat[self._tool_body].reshape(3, 3), self.data.xpos[self._tool_body]
        self._tool_local = R.T @ (arm.fk(self.data.qpos[self._qa])[:3, 3] - o)
        self._ref = q.copy()                 # rate-limited reference (what the drivers follow)
        self._target = q.copy()              # last command
        self._enabled = False
        self._lag_time = np.zeros(arm.n)
        self.faults: dict[str, str] = {}
        self.last_torque = np.zeros(arm.n)
        self.data.ctrl[:] = q
        self._lock = threading.Lock()

    # -------------------------------------------------------------------- Backend API ----
    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        with self._lock:
            self.faults.clear()
            self._lag_time[:] = 0
            self.model.actuator_gainprm[:] = self._gain0     # servos back on
            self.model.actuator_biasprm[:] = self._bias0
            self._ref = self.data.qpos[self._qm].copy()     # the driver starts where the motor is
            self._target = self._ref.copy()
            self.data.qvel[:] = 0
            self._enabled = True

    def estop(self) -> None:
        with self._lock:
            self._enabled = False
            self._target = self.data.qpos[self._qm].copy()

    def command(self, q: np.ndarray) -> None:
        if not self._enabled:
            return
        with self._lock:
            self._target = self.arm.clamp(np.asarray(q, dtype=float))

    def read(self) -> np.ndarray:
        """Joint (link / output) angles."""
        with self._lock:
            return self.data.qpos[self._qa].copy()

    def read_motor(self) -> np.ndarray:
        """Motor angles divided by the ratio: what the stepper drivers' own encoders report.

        Differs from ``read()`` by the gearbox wind-up and backlash (identical when rigid).
        """
        with self._lock:
            return self.data.qpos[self._qm].copy()

    @property
    def torque_utilization(self) -> np.ndarray:
        """|applied torque| / joint limit at the current speed for the last step (1 = saturated)."""
        return np.abs(self.last_torque) / self._limit_now

    def _apply_loads(self) -> None:
        d = self.data
        d.qfrc_applied[:] = 0
        d.qfrc_applied[self._va] = self.external_torque
        if self.arm.counterbalance is not None:
            d.qfrc_applied[self._va] += spring_torque(self.arm, d.qpos[self._qa])
        f = self.tool_force
        if callable(f) or np.any(f):
            b = self._tool_body
            p = d.xpos[b] + d.xmat[b].reshape(3, 3) @ self._tool_local
            f = np.asarray(f(p) if callable(f) else f, dtype=float)
            self._mj.mj_applyFT(self.model, d, f, np.zeros(3), p, b, d.qfrc_applied)

    def tcp(self) -> np.ndarray:
        """Tool centre point position (m, base frame) from the link angles."""
        with self._lock:
            b = self._tool_body
            return self.data.xpos[b] + self.data.xmat[b].reshape(3, 3) @ self._tool_local

    def step(self, dt: float) -> None:
        mj, m, d = self._mj, self.model, self.data
        with self._lock:
            n_sub = max(1, int(round(dt / m.opt.timestep)))
            h = dt / n_sub
            speed = self.arm.vmax * self.speed_scale
            if self.streamed:
                speed = np.minimum(np.abs(self._target - self._ref) / dt, speed)
            for _ in range(n_sub):
                self._apply_loads()
                if not self._enabled:
                    self.last_torque[:] = 0
                    if not self.backdrivable:
                        d.qvel[:] = 0                 # non-backdrivable gearbox holds the pose
                        return
                    m.actuator_gainprm[:, 0] = 0      # servos off: the arm moves freely
                    m.actuator_biasprm[:, 1:3] = 0
                    mj.mj_step(m, d)
                    continue
                # reference chases the commanded target (at the speed limits, or streamed: over dt)
                err = self._target - self._ref
                step = speed * h
                move = np.clip(err, -step, step)
                self._ref += move
                # the torque a stepper can make falls with its speed (deskarm.motor)
                self._limit_now = self.arm.torque_limit(d.qvel[self._vm])
                lim = self._limit_now[self._limited]
                m.actuator_forcerange[self._limited, 0] = -lim
                m.actuator_forcerange[self._limited, 1] = lim
                # position servo (torque-limited) with velocity feed-forward folded into the target
                d.ctrl[:] = self._ref + self._kv_over_kp * (move / h)
                mj.mj_step(m, d)
                self.last_torque = d.actuator_force.copy()
                # stall detection on the motor encoder (what a closed-loop stepper driver sees)
                lagging = np.abs(self._ref - d.qpos[self._qm]) > self.stall
                self._lag_time = np.where(lagging, self._lag_time + h, 0.0)
                stalled = self._lag_time > self.stall_time
                if stalled.any():
                    for i in np.flatnonzero(stalled):
                        name = self.arm.joints[i].name
                        self.faults[name] = (f"{name} stalled: needs more than the "
                                             f"{self._limit_now[i]:.1f} N·m it has at this speed "
                                             f"({self.limits[i]:.1f} N·m at standstill)")
                    self._enabled = False
                    self._target = d.qpos[self._qm].copy()
                    self.last_torque[:] = 0
                    return

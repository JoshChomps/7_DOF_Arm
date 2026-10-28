"""Arm model: DH description, forward kinematics, geometric Jacobian.

Conventions
-----------
* Units are metres and radians.
* Standard DH: ``T_i = Rz(theta_i + offset_i) @ Tz(d_i) @ Tx(a_i) @ Rx(alpha_i)``.
* ``frames(q)[i]`` is DH frame *i* in the base frame: frame 0 is the base, frame i sits on the
  axis of joint i+1 (its z axis *is* that joint axis), frame n is the flange, and the last entry
  is the tool centre point (TCP).
* ``link_frames(q)[i]`` is the body frame of the link that joint i+1 moves (DH frame i turned by
  that joint's angle). It is the frame the URDF links use.

Mass model
----------
The arm's mass is a list of lumps (``[[masses]]`` in ``arm.toml``): one per motor + driver +
gearbox, printed link or tool, each with the joint that carries it and its position **at the zero
pose** (arm straight up). That list is the single source for the URDF inertials, the MuJoCo
physics, ``gravity_torque`` here, the counterbalance sizing and ``tools/torque_budget.py``.

Math, with pictures: docs/14_math_guide.md §1 (rotations and transforms), §2 (DH and forward kinematics),
§3 (Jacobian), §10 (masses and gravity torque), §11 (torque limit vs speed).
"""
from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from importlib import resources
from pathlib import Path

import numpy as np

from .motor import stepper_torque

G = 9.81                    # m/s^2
EMPTY_LINK_MASS = 0.01      # kg given to a link with no lumps, so the physics has something to move
CURVE_POINTS = 129          # samples of each joint's torque-speed curve (0 .. 2 x vmax)


# --------------------------------------------------------------------------- transforms ----
def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def homogeneous(R: np.ndarray | None = None, p=(0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = p
    return T


def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


# --------------------------------------------------------------------------- model ---------
@dataclass(frozen=True)
class Joint:
    name: str
    description: str
    d: float
    a: float
    alpha: float
    offset: float
    lower: float
    upper: float
    vmax: float
    amax: float
    # Drive (used by the physics sim and the torque checks). ``motor_torque`` is the usable motor
    # torque near standstill (N*m): derate x holding torque. The joint's torque limit is
    # motor torque x ratio x efficiency, and it falls with speed along ``torque_curve`` (see
    # deskarm.motor). Reflected rotor inertia ("armature") = rotor_inertia x ratio^2.
    motor_torque: float = 0.0
    ratio: float = 1.0
    efficiency: float = 1.0
    rotor_inertia: float = 0.0
    hold_torque: float = 0.0        # datasheet holding torque (N*m)
    rated_current: float = 0.0      # datasheet current per phase (A)
    resistance: float = 0.0         # phase resistance (ohm)
    inductance: float = 0.0         # phase inductance (H)
    torque_curve: tuple[float, ...] = ()   # joint torque limit (N*m) at joint speeds 0 .. curve_vmax
    curve_vmax: float = 0.0         # joint speed (rad/s) of the last curve sample
    # Gearbox elasticity (physics sim): torsional stiffness at the output (N*m/rad, 0 = rigid)
    # and backlash (rad, total lost motion). Placeholders until measured on the test bench.
    gear_stiffness: float = 0.0
    backlash: float = 0.0
    jmax: float = 0.0               # jerk limit (rad/s^3) for S-curve moves; 0 = 8 x amax
    motor_sign: int = 1             # +1 if positive motor counts turn the joint the positive way, else -1
    driver: str = "S42D_CAN"        # MKS board type, as its version reply (40H) names it
    hall: float | None = None       # joint angle (rad) at the centre of its hall switch's band, if it has one
    hall_width: float = 0.0         # width of the band where the switch reads "magnet" (rad)

    @property
    def torque_max(self) -> float:
        """Joint torque limit near standstill (N*m)."""
        if self.torque_curve:
            return self.torque_curve[0]
        return self.motor_torque * self.ratio * self.efficiency

    def torque_at(self, speed: float) -> float:
        """Joint torque limit (N*m) at joint speed ``speed`` (rad/s)."""
        if not self.torque_curve:
            return self.torque_max
        v = np.linspace(0.0, self.curve_vmax, len(self.torque_curve))
        return float(np.interp(abs(speed), v, self.torque_curve))

    @property
    def armature(self) -> float:
        return self.rotor_inertia * self.ratio**2


@dataclass(frozen=True)
class Lump:
    """A lumped mass riding on one link (an actuator, a printed link, a tool)."""
    name: str
    link: int                       # index of the joint that carries it (-1 = fixed base)
    mass: float                     # kg
    pos: tuple[float, float, float]  # centre of mass in the base frame at the zero pose (m)
    radius: float = 0.03            # size, for its own rotational inertia (solid sphere)


@dataclass(frozen=True)
class Drive:
    """Supply and planning margin shared by every joint (``[drive]`` in arm.toml)."""
    voltage: float = 24.0
    torque_derate: float = 0.75     # fraction of the ideal motor envelope to plan with
    pole_pairs: int = 50            # 1.8 deg hybrid stepper


@dataclass(frozen=True)
class Counterbalance:
    """Spring that helps a joint hold the arm up (see deskarm.counterbalance)."""
    joint: int                      # joint index (J2 = 1)
    anchor_height: float            # a: anchor above the joint axis, on the parent body (m)
    attach_distance: float          # b: attachment along the link from the joint axis (m)
    stiffness: float                # k (N/m)
    free_length: float = 0.0        # 0 = zero-length spring (spring + cable over a pulley)


@dataclass
class ArmModel:
    name: str
    joints: tuple[Joint, ...]
    tool: np.ndarray = field(default_factory=lambda: np.eye(4))  # flange -> TCP
    home: np.ndarray | None = None
    shoulder_frame: int = 1
    elbow_frame: int = 3
    wrist_frame: int = 5
    counterbalance: Counterbalance | None = None
    masses: tuple[Lump, ...] = ()
    drive: Drive = field(default_factory=Drive)

    def __post_init__(self) -> None:
        if self.home is None:
            self.home = np.zeros(self.n)
        self._inertials = self._lump_inertials()
        # torque-speed table: one row per joint, sampled at uniform joint speeds
        K = CURVE_POINTS
        if any(j.torque_curve and len(j.torque_curve) != K for j in self.joints):
            raise ValueError(f"a joint's torque_curve must have {K} samples (0 .. curve_vmax)")
        self._has_drive = np.array([j.torque_max > 0 for j in self.joints])
        self._curve = np.array([j.torque_curve or np.full(K, j.torque_max) for j in self.joints], dtype=float)
        self._curve = np.minimum.accumulate(self._curve, axis=1)          # never rises with speed
        self._curve_dv = np.array([(j.curve_vmax if j.torque_curve else 1.0) / (K - 1)
                                   for j in self.joints])

    # ---- joint-space helpers -----------------------------------------------------------
    @property
    def n(self) -> int:
        return len(self.joints)

    @property
    def names(self) -> list[str]:
        return [j.name for j in self.joints]

    @property
    def lower(self) -> np.ndarray:
        return np.array([j.lower for j in self.joints])

    @property
    def upper(self) -> np.ndarray:
        return np.array([j.upper for j in self.joints])

    @property
    def vmax(self) -> np.ndarray:
        return np.array([j.vmax for j in self.joints])

    @property
    def amax(self) -> np.ndarray:
        return np.array([j.amax for j in self.joints])

    @property
    def jmax(self) -> np.ndarray:
        return np.array([j.jmax if j.jmax > 0 else 8.0 * j.amax for j in self.joints])

    def clamp(self, q: np.ndarray) -> np.ndarray:
        return np.clip(q, self.lower, self.upper)

    def within_limits(self, q: np.ndarray, tol: float = 1e-9) -> bool:
        return bool(np.all(q >= self.lower - tol) and np.all(q <= self.upper + tol))

    # ---- kinematics --------------------------------------------------------------------
    def frames(self, q: np.ndarray) -> list[np.ndarray]:
        """Base frame, every DH frame 1..n, then the TCP (length n + 2)."""
        q = np.asarray(q, dtype=float)
        T = np.eye(4)
        out = [T]
        for qi, j in zip(q, self.joints):
            T = T @ dh_transform(qi + j.offset, j.d, j.a, j.alpha)
            out.append(T)
        out.append(T @ self.tool)
        return out

    def fk(self, q: np.ndarray) -> np.ndarray:
        """Pose of the TCP in the base frame (4x4)."""
        return self.frames(q)[-1]

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """Geometric Jacobian (6 x n) of the TCP: rows = [linear velocity; angular velocity].

        Column i is [z_i x (p - o_i); z_i]: joint axis crossed with the lever arm to the tool
        (docs/14_math_guide.md §3).
        """
        F = self.frames(q)
        p_tcp = F[-1][:3, 3]
        J = np.zeros((6, self.n))
        for i in range(self.n):
            z = F[i][:3, 2]
            o = F[i][:3, 3]
            J[:3, i] = np.cross(z, p_tcp - o)
            J[3:, i] = z
        return J

    def srs_points(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Shoulder centre, elbow and wrist centre positions (used for the swivel angle)."""
        F = self.frames(q)
        return (F[self.shoulder_frame][:3, 3], F[self.elbow_frame][:3, 3], F[self.wrist_frame][:3, 3])

    def manipulability(self, q: np.ndarray) -> float:
        J = self.jacobian(q)
        return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

    def link_frames(self, q: np.ndarray) -> list[np.ndarray]:
        """Body frame of each moving link (the frames the URDF links use), in the base frame."""
        F = self.frames(q)
        return [F[i] @ homogeneous(rot_z(qi + j.offset))
                for i, (qi, j) in enumerate(zip(np.asarray(q, dtype=float), self.joints))]

    # ---- mass model --------------------------------------------------------------------
    def _lump_inertials(self) -> list[tuple[float, np.ndarray, np.ndarray]]:
        """Per moving link: (mass, centre of mass, inertia about it), in the link's body frame."""
        L0 = self.link_frames(np.zeros(self.n))
        out = []
        for i in range(self.n):
            lumps = [m for m in self.masses if m.link == i and m.mass > 0]
            if not lumps:
                m = EMPTY_LINK_MASS
                out.append((m, np.zeros(3), 0.4 * m * 0.03**2 * np.eye(3)))
                continue
            inv = np.linalg.inv(L0[i])
            pts = [(inv @ np.append(m.pos, 1.0))[:3] for m in lumps]
            total = sum(m.mass for m in lumps)
            com = sum(m.mass * p for m, p in zip(lumps, pts)) / total
            inertia = np.zeros((3, 3))
            for m, p in zip(lumps, pts):
                d = p - com
                inertia += 0.4 * m.mass * m.radius**2 * np.eye(3)        # the lump itself
                inertia += m.mass * ((d @ d) * np.eye(3) - np.outer(d, d))  # parallel axes
            out.append((total, com, inertia))
        return out

    def link_inertials(self) -> list[tuple[float, np.ndarray, np.ndarray]]:
        return list(self._inertials)

    @property
    def moving_mass(self) -> float:
        """Mass of everything the joints move (kg), without a payload."""
        return float(sum(m.mass for m in self.masses if m.link >= 0))

    @property
    def total_mass(self) -> float:
        return float(sum(m.mass for m in self.masses))

    def gravity_torque(self, q: np.ndarray, payload_kg: float = 0.0) -> np.ndarray:
        """Joint torques (n,) needed to hold the arm still at ``q`` against gravity.

        Motors only: the counterbalance spring's help is not subtracted here (see
        ``deskarm.counterbalance.spring_torque``). ``payload_kg`` is a point mass at the TCP.
        Matches the MuJoCo model exactly (same masses), and needs only numpy.
        Per mass: m g z_k . (r x z_up), with r the lever arm from joint k's axis (docs/14_math_guide.md §10).
        """
        q = np.asarray(q, dtype=float)
        F = self.frames(q)
        tau = np.zeros(self.n)

        def add(m: float, p: np.ndarray, last: int) -> None:
            for k in range(last + 1):
                r = p - F[k][:3, 3]
                z = F[k][:3, 2]
                tau[k] += m * G * (z[0] * r[1] - z[1] * r[0])      # m g z . (r x z_up)

        for i, (m, com, _) in enumerate(self._inertials):
            L = F[i] @ homogeneous(rot_z(q[i] + self.joints[i].offset))
            add(m, (L @ np.append(com, 1.0))[:3], i)
        if payload_kg > 0:
            add(payload_kg, F[-1][:3, 3], self.n - 1)
        return tau

    # ---- drive limits ------------------------------------------------------------------
    def torque_limit(self, qd: np.ndarray | None = None) -> np.ndarray:
        """Joint torque limits (N*m) at joint speeds ``qd`` (rad/s; shape (n,) or (..., n)).

        Near standstill with ``qd=None``. Joints without drive data are unlimited (inf).
        """
        if qd is None:
            return np.where(self._has_drive, self._curve[:, 0], np.inf)
        K = self._curve.shape[1]
        x = np.minimum(np.abs(np.asarray(qd, dtype=float)) / self._curve_dv, K - 1)
        i = np.minimum(x.astype(int), K - 2)
        f = x - i
        j = np.arange(self.n)
        lim = self._curve[j, i] * (1 - f) + self._curve[j, i + 1] * f
        return np.where(self._has_drive, lim, np.inf)

    def speed_at_torque(self, torque: np.ndarray) -> np.ndarray:
        """Largest joint speed (rad/s) at which each joint can still deliver ``torque`` (N*m).

        Shape (n,) or (..., n). Zero if the joint can't deliver it even at standstill; inf if it
        can at every speed in its curve.
        """
        torque = np.abs(np.asarray(torque, dtype=float))
        out = np.empty(torque.shape)
        K = self._curve.shape[1]
        for k in range(self.n):
            c = self._curve[k] - 1e-12 * np.arange(K)               # strictly falling for interp
            v = self._curve_dv[k] * np.arange(K)
            t = torque[..., k]
            s = np.interp(t, c[::-1], v[::-1])
            out[..., k] = np.where(t <= c[-1], np.inf, np.where(t > c[0], 0.0, s))
        return np.where(self._has_drive, out, np.inf)


# --------------------------------------------------------------------------- loading -------
def load_arm(path: str | Path | None = None) -> ArmModel:
    """Load an arm from a TOML file (defaults to the bundled ``data/arm.toml``)."""
    return arm_from_config(load_config(path))


def load_config(path: str | Path | None = None) -> dict:
    """The arm TOML as a plain dict (defaults to the bundled ``data/arm.toml``)."""
    if path is None:
        text = resources.files("deskarm").joinpath("data/arm.toml").read_text(encoding="utf-8")
    else:
        text = Path(path).read_text(encoding="utf-8")
    return tomllib.loads(text)


def arm_from_config(cfg: dict) -> ArmModel:
    """Build the arm from a config dict (``load_config``), e.g. after editing it to try a variant."""
    rad = np.deg2rad
    dcfg = cfg.get("drive", {})
    drive = Drive(voltage=float(dcfg.get("voltage", 24.0)),
                  torque_derate=float(dcfg.get("torque_derate", 0.75)),
                  pole_pairs=int(dcfg.get("pole_pairs", 50)))

    def joint(j: dict) -> Joint:
        vmax = float(rad(j["vmax_deg"]))
        ratio, eff = float(j.get("ratio", 1.0)), float(j.get("efficiency", 1.0))
        hold = float(j.get("hold_torque", 0.0))
        current = float(j.get("rated_current", 0.0))
        resistance = float(j.get("resistance", 0.0))
        inductance = float(j.get("inductance_mh", 0.0)) * 1e-3
        curve: tuple[float, ...] = ()
        if hold > 0:                                   # datasheet values: speed-dependent limit
            motor_torque = drive.torque_derate * hold
            speeds = np.linspace(0.0, 2 * vmax, CURVE_POINTS)
            if current > 0:
                t = stepper_torque(speeds * ratio, hold, current, resistance, inductance,
                                   drive.voltage, drive.pole_pairs)
            else:
                t = np.full(CURVE_POINTS, hold)
            curve = tuple(float(x) for x in drive.torque_derate * t * ratio * eff)
        else:                                          # plain "usable torque": flat limit
            motor_torque = float(j.get("motor_torque", 0.0))
        return Joint(
            name=j["name"],
            description=j.get("description", ""),
            d=float(j["d"]),
            a=float(j["a"]),
            alpha=float(rad(j["alpha_deg"])),
            offset=float(rad(j.get("offset_deg", 0.0))),
            lower=float(rad(j["min_deg"])),
            upper=float(rad(j["max_deg"])),
            vmax=vmax,
            amax=float(rad(j["amax_deg"])),
            motor_torque=motor_torque,
            ratio=ratio,
            efficiency=eff,
            rotor_inertia=float(j.get("rotor_inertia", 0.0)),
            hold_torque=hold,
            rated_current=current,
            resistance=resistance,
            inductance=inductance,
            torque_curve=curve,
            curve_vmax=2 * vmax,
            gear_stiffness=float(j.get("gear_stiffness", 0.0)),
            backlash=float(rad(j.get("backlash_deg", 0.0))),
            jmax=float(rad(j.get("jmax_deg", 0.0))),
            motor_sign=int(j.get("motor_sign", 1)),
            driver=str(j.get("driver", "S42D_CAN")),
            hall=None if "hall_deg" not in j else float(rad(j["hall_deg"])),
            hall_width=float(rad(j.get("hall_width_deg", 10.0))),
        )

    joints = tuple(joint(j) for j in cfg["joints"])
    names = [j.name for j in joints]
    masses = tuple(
        Lump(name=m.get("name", ""), link=-1 if m["on"] == "base" else names.index(m["on"]),
             mass=float(m["mass"]), pos=tuple(float(x) for x in m["pos"]),
             radius=float(m.get("radius", 0.03)))
        for m in cfg.get("masses", []))
    arm_cfg = cfg.get("arm", {})
    tool = homogeneous(p=(0.0, 0.0, float(cfg.get("tool", {}).get("tcp_offset_z", 0.0))))
    home = cfg.get("home", {}).get("q_deg")
    cb = cfg.get("counterbalance")
    counterbalance = None
    if cb and float(cb.get("stiffness", 0.0)) > 0:
        counterbalance = Counterbalance(
            joint=names.index(cb.get("joint", "J2")),
            anchor_height=float(cb["anchor_height"]),
            attach_distance=float(cb["attach_distance"]),
            stiffness=float(cb["stiffness"]),
            free_length=float(cb.get("free_length", 0.0)),
        )
    return ArmModel(
        name=arm_cfg.get("name", "arm"),
        joints=joints,
        tool=tool,
        home=None if home is None else rad(np.array(home, dtype=float)),
        shoulder_frame=arm_cfg.get("shoulder_frame", 1),
        elbow_frame=arm_cfg.get("elbow_frame", 3),
        wrist_frame=arm_cfg.get("wrist_frame", 5),
        counterbalance=counterbalance,
        masses=masses,
        drive=drive,
    )

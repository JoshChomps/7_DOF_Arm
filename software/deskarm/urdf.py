"""Generate a URDF (simple cylinder geometry) from the DH description.

The URDF reproduces the DH forward kinematics exactly, so the Viser twin, MuJoCo and (later)
ROS 2 all see the same robot. ``meshes=MESH_DIR`` swaps the primitive visuals for the CAD's STLs
(``tools/cad_meshes.py``, one per link and material); the joint frames stay the same. Each link's mass, centre of mass and
inertia come from the ``[[masses]]`` list in ``arm.toml`` (``ArmModel.link_inertials``).

DH -> URDF: joint i's origin is the fixed part of the previous DH row
(``Tz(d) Tx(a) Rx(alpha)`` of row i-1, plus ``Rz(offset_i)``), and the joint rotates about its
local z axis. The flange->TCP transform becomes a fixed joint to a ``tcp`` link.

Math: docs/14_math_guide.md §2 (DH frames and link frames), §1 (roll-pitch-yaw), §10 (link inertials).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .kinematics import ArmModel, homogeneous, rot_x, rot_z

LINK_RADIUS = 0.028
HOUSING_RADIUS = 0.042
HOUSING_LENGTH = 0.07
COLORS = {"base": "0.25 0.25 0.28 1", "housing": "0.95 0.55 0.15 1", "link": "0.85 0.85 0.88 1",
          "tool": "0.2 0.6 0.9 1"}
MESH_DIR = Path(__file__).parent / "data" / "meshes"
MESH_COLORS = {"printed": ("0.93 0.93 0.95 1", "0.25 0.50 0.85 1"), "tpu": "0.15 0.15 0.15 1",
               "motor": "0.08 0.08 0.09 1", "belt": "0.12 0.12 0.12 1", "metal": "0.62 0.64 0.68 1",
               "steel": "0.78 0.79 0.80 1", "pcb": "0.10 0.45 0.20 1"}


def rot_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """URDF rpy (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) > 1e-9:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return float(roll), float(pitch), float(yaw)


def _origin(T: np.ndarray) -> str:
    x, y, z = (float(v) for v in T[:3, 3])
    r, p, yw = rot_to_rpy(T[:3, :3])
    return f'<origin xyz="{x:.6f} {y:.6f} {z:.6f}" rpy="{r:.6f} {p:.6f} {yw:.6f}"/>'


def _cylinder(radius: float, length: float, z_center: float, color: str, x_center: float = 0.0,
              along_x: bool = False) -> str:
    rpy = "0 1.570796 0" if along_x else "0 0 0"
    return (f'<visual><origin xyz="{x_center:.6f} 0 {z_center:.6f}" rpy="{rpy}"/>'
            f'<geometry><cylinder radius="{radius}" length="{length:.6f}"/></geometry>'
            f'<material name="{color}"><color rgba="{COLORS[color]}"/></material></visual>')


def _meshes(folder: Path | None, k: int) -> str | None:
    """Mesh visuals of link k (0 = base) from folder/<k>_<material>.stl, or None to keep the primitives."""
    files = sorted(Path(folder).glob(f"{k}_*.stl")) if folder else []
    out = []
    for f in files:
        rgba = MESH_COLORS.get(f.stem.split("_", 1)[1], "0.7 0.7 0.7 1")
        if isinstance(rgba, tuple):                   # printed: base grey, then white and blue links
            rgba = "0.40 0.40 0.43 1" if k == 0 else rgba[k % 2]
        out.append(f'<visual><geometry><mesh filename="{f.as_posix()}"/></geometry>'
                   f'<material name="{f.stem}"><color rgba="{rgba}"/></material></visual>')
    return "".join(out) or None


def _inertial(mass: float, com: np.ndarray, inertia: np.ndarray) -> str:
    """URDF inertial from a link's mass, centre of mass and inertia tensor (link frame)."""
    x, y, z = (float(v) for v in com)
    i = inertia
    return (f'<inertial><origin xyz="{x:.6f} {y:.6f} {z:.6f}"/><mass value="{mass:.5f}"/>'
            f'<inertia ixx="{i[0, 0]:.6e}" iyy="{i[1, 1]:.6e}" izz="{i[2, 2]:.6e}" '
            f'ixy="{i[0, 1]:.6e}" ixz="{i[0, 2]:.6e}" iyz="{i[1, 2]:.6e}"/></inertial>')


def to_urdf(arm: ArmModel, payload_kg: float = 0.0, meshes: Path | None = None) -> str:
    """URDF of the arm. ``payload_kg`` adds a point mass at the tool centre point (for physics); ``meshes`` is a
    folder of CAD meshes to draw instead of cylinders (absolute paths, for viewers)."""
    parts = [f'<?xml version="1.0"?>\n<robot name="{arm.name}">']
    base_mass = sum(m.mass for m in arm.masses if m.link < 0) or 2.0     # fixed: value doesn't matter
    parts.append('<link name="base_link">'
                 + (_meshes(meshes, 0) or _cylinder(0.09, 0.02, -0.01, "base"))
                 + _inertial(base_mass, np.zeros(3), base_mass * 0.09**2 / 2 * np.eye(3)) + '</link>')

    parent = "base_link"
    prev_fixed = np.eye(4)
    inertials = arm.link_inertials()                 # from the [[masses]] list in arm.toml
    for i, j in enumerate(arm.joints):
        link = f"link{i + 1}"
        origin = prev_fixed @ homogeneous(rot_z(j.offset))
        effort = j.torque_max if j.torque_max > 0 else 30.0
        parts.append(
            f'<joint name="{j.name}" type="revolute"><parent link="{parent}"/><child link="{link}"/>'
            f'{_origin(origin)}<axis xyz="0 0 1"/>'
            f'<limit lower="{j.lower:.6f}" upper="{j.upper:.6f}" effort="{effort:.3f}" '
            f'velocity="{j.vmax:.4f}"/></joint>')
        visuals = _cylinder(HOUSING_RADIUS, HOUSING_LENGTH, 0.0, "housing")
        if abs(j.d) > 1e-9:
            visuals += _cylinder(LINK_RADIUS, abs(j.d), j.d / 2, "link")
        if abs(j.a) > 1e-9:
            visuals += _cylinder(LINK_RADIUS, abs(j.a), j.d, "link", x_center=j.a / 2, along_x=True)
        visuals = _meshes(meshes, i + 1) or visuals
        parts.append(f'<link name="{link}">{visuals}{_inertial(*inertials[i])}</link>')
        prev_fixed = homogeneous(p=(0, 0, j.d)) @ homogeneous(p=(j.a, 0, 0)) @ homogeneous(rot_x(j.alpha))
        parent = link

    tool_len = float(arm.tool[2, 3])
    parts.append(f'<joint name="tcp_joint" type="fixed"><parent link="{parent}"/><child link="tcp"/>'
                 f'{_origin(prev_fixed @ arm.tool)}</joint>')
    tool_visual = _cylinder(0.02, max(tool_len, 0.01), -tool_len / 2, "tool") if tool_len > 0 and not meshes else ""
    payload = ""
    if payload_kg > 0:
        i = 1e-6
        payload = (f'<inertial><origin xyz="0 0 0"/><mass value="{payload_kg:.4f}"/>'
                   f'<inertia ixx="{i}" iyy="{i}" izz="{i}" ixy="0" ixz="0" iyz="0"/></inertial>')
    parts.append(f'<link name="tcp">{tool_visual}{payload}</link>')
    parts.append("</robot>")
    return "\n".join(parts)

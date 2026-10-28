"""CAD meshes for the twin: the parts of DA_ARM_Display, merged per body and material, each moved into its URDF link
frame, in metres (docs/20 §10).

    software/.venv/Scripts/python.exe tools/fusion_run.py display mesh DA_ARM_Display <raw dir>
    software/.venv/Scripts/python.exe tools/cad_meshes.py <raw dir>                 (-> software/deskarm/data/meshes)
    software/.venv/Scripts/python.exe tools/fusion_run.py display frames DA_ARM_Display '<pose>' > frames.json
    software/.venv/Scripts/python.exe tools/cad_meshes.py check frames.json '<pose>'  (twin vs Fusion at a pose)

The raw files are <body>_<material>_<n>.stl in each part's own coordinates (mm), placed by places.json (CAD world,
zero pose). CAD -> arm.toml base: x = -y_cad, y = x_cad
(docs/20 §1); CAD zero = arm.toml zero, so body k is moved by the inverse of link k's URDF frame at q = 0. The "space"
boxes (electronics keep-outs) are left out. `check` compares how each body moves in the twin and in Fusion.
"""
import io
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import trimesh
import yourdfpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software"))
from deskarm import load_arm, to_urdf  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "software" / "deskarm" / "data" / "meshes"
CAD_TO_BASE = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def _link(k: int) -> str:
    return "base_link" if k == 0 else f"link{k}"


def main(raw: str) -> None:
    robot = yourdfpy.URDF.load(io.StringIO(to_urdf(load_arm())), load_meshes=False)
    places = json.loads((Path(raw) / "places.json").read_text())
    groups = defaultdict(list)
    for name, T in places.items():
        k, mat, _ = name[:-4].split("_")
        if mat != "space":
            T = np.array(T).reshape(4, 4)
            T[:3, 3] *= 10                             # cm -> mm
            groups[(int(k), mat)].append(trimesh.load(Path(raw) / name).apply_transform(T))
    OUT.mkdir(exist_ok=True)
    for old in OUT.glob("*.stl"):
        old.unlink()
    for (k, mat), parts in sorted(groups.items()):
        m = trimesh.util.concatenate(parts)
        m.apply_scale(0.001)
        m.apply_transform(CAD_TO_BASE)
        m.apply_transform(np.linalg.inv(robot.get_transform(_link(k), "base_link")))
        m.export(OUT / f"{k}_{mat}.stl")
        print(f"{k}_{mat}: {len(parts)} parts, {len(m.faces)} triangles")


def check(frames: str, pose: dict) -> None:
    """Each body's motion in the twin (URDF link at `pose` x its zero-pose inverse) against Fusion's."""
    robot = yourdfpy.URDF.load(io.StringIO(to_urdf(load_arm())), load_meshes=False)
    zero = [robot.get_transform(_link(k), "base_link") for k in range(8)]
    robot.update_cfg(np.deg2rad([pose.get(f"J{i}", 0) for i in range(1, 8)]))
    fusion = json.loads(Path(frames).read_text())
    print(f"Fusion: parts differ from their body's motion by up to {fusion['spread_mm']:.1e} mm")
    for k in range(1, 8):
        M = np.linalg.inv(CAD_TO_BASE) @ robot.get_transform(_link(k), "base_link") @ np.linalg.inv(zero[k]) @ CAD_TO_BASE
        F = np.array(fusion["motion"][k]).reshape(4, 4)
        print(f"L{k}: rotation {np.abs(F[:3, :3] - M[:3, :3]).max():.1e}, "
              f"position {np.abs(F[:3, 3] * 10 - M[:3, 3] * 1000).max():.3f} mm")


if __name__ == "__main__":
    if sys.argv[1] == "check":
        check(sys.argv[2], json.loads(sys.argv[3]))
    else:
        main(sys.argv[1])

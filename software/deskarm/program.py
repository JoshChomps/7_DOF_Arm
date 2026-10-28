"""Teach & replay: record waypoints, save them as JSON, play them back as a sequence of moves.

Each waypoint stores a joint configuration and how to get there:
* ``"joint"``  - synchronized joint-space move (fast, any path)
* ``"linear"`` - straight line of the tool to the waypoint's TCP pose

    prog = Program()
    prog.add(q1, "joint"); prog.add(q2, "linear", speed=0.5)
    prog.save("programs/pick.json")
    trajs = Program.load("programs/pick.json").build(arm, q_now)
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from .cartesian import linear_move
from .kinematics import ArmModel
from .trajectory import joint_move

MOVES = ("joint", "linear")


@dataclass
class Waypoint:
    q: list[float]
    move: str = "joint"
    speed: float = 0.5            # fraction of the joint / tool speed limits
    name: str = ""

    def __post_init__(self):
        if self.move not in MOVES:
            raise ValueError(f"move must be one of {MOVES}")
        self.q = [float(v) for v in self.q]


@dataclass
class Program:
    waypoints: list[Waypoint] = field(default_factory=list)
    arm_name: str = ""

    def add(self, q, move: str = "joint", speed: float = 0.5, name: str = "") -> Waypoint:
        wp = Waypoint(list(np.asarray(q, dtype=float)), move, speed, name or f"P{len(self.waypoints) + 1}")
        self.waypoints.append(wp)
        return wp

    # ---- files -------------------------------------------------------------------------
    def to_json(self) -> str:
        return json.dumps({"arm": self.arm_name, "units": "rad",
                           "waypoints": [asdict(w) for w in self.waypoints]}, indent=2)

    @classmethod
    def from_json(cls, text: str) -> "Program":
        d = json.loads(text)
        return cls([Waypoint(**w) for w in d.get("waypoints", [])], d.get("arm", ""))

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.to_json())
        return path

    @classmethod
    def load(cls, path: str | Path) -> "Program":
        return cls.from_json(Path(path).read_text())

    # ---- playback ----------------------------------------------------------------------
    def build(self, arm: ArmModel, q_start: np.ndarray, mode: str = "7dof", locked=(),
              task: str = "pose", profile: str = "trapezoid", torque_model=None) -> list:
        """Trajectories from ``q_start`` through every waypoint (raises CartesianError on a bad line).

        ``profile``: "trapezoid", "scurve" or "time-optimal" (torque-aware with ``torque_model``).
        """
        trajs = []
        q = np.asarray(q_start, dtype=float)
        for wp in self.waypoints:
            if len(wp.q) != arm.n:
                raise ValueError(f"waypoint {wp.name} has {len(wp.q)} joints, arm has {arm.n}")
            target = np.asarray(wp.q)
            if wp.move == "joint":
                tr = joint_move(arm, q, target, profile, speed_scale=wp.speed, torque_model=torque_model)
                q_end = target
            else:
                tr, _ = linear_move(arm, q, arm.fk(target), speed_scale=wp.speed, mode=mode,
                                    locked=locked, task=task, profile=profile,
                                    torque_model=torque_model)
                q_end = tr.sample(tr.duration)[0]   # redundancy: may differ from wp.q, same pose
            trajs.append(tr)
            q = q_end
        return trajs


def tcp_path(arm: ArmModel, trajs, samples_per_traj: int = 40) -> np.ndarray:
    """TCP positions along a list of trajectories (for drawing the path)."""
    pts = []
    for tr in trajs:
        for k in range(samples_per_traj + 1):
            pts.append(arm.fk(tr.sample(tr.duration * k / samples_per_traj)[0])[:3, 3])
    return np.array(pts)

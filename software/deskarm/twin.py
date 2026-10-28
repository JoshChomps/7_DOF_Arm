"""Digital twin: a browser UI that plans on a ghost arm and mirrors it onto the (sim or real) arm.

Run::

    python -m deskarm.twin            # then open http://localhost:8080 (also works on your phone)

* Grey arm  = the robot's *measured* state (from the backend).
* Blue ghost = the *target*. Drag the gizmo (IK), move joint sliders, or swivel the elbow.
* "Execute" plays a synchronized trajectory from measured -> ghost. "Live" streams every ghost
  change straight to the robot instead.
* Lock checkboxes keep a joint fixed while IK moves the others.
* "Motion profile" picks trapezoid, jerk-limited S-curve or time-optimal (TOPP-RA) timing, and the
  collision guard refuses moves that hit the desk or the arm itself.
"""
from __future__ import annotations

import argparse
import io
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

from .backends import SimBackend
from .cartesian import CartesianError, linear_move
from .collision import self_collisions, trajectory_collision
from .counterbalance import spring_torque
from .ik import (J3, singularity_warnings, solve_ik, swivel, swivel_angle, trajectory_violation,
                 workspace_violations)
from .kinematics import ArmModel, load_arm
from .program import Program, tcp_path
from .topp import ToppError
from .trajectory import joint_move
from .urdf import MESH_DIR, to_urdf

MODES = {"7-DOF (redundant)": "7dof", "6-DOF (J3 locked)": "6dof"}
# Tool constraints for gizmo-driven IK: (task, translate-only gizmo, active gizmo axes)
CONSTRAINTS = {
    "Full pose (move + rotate)": ("pose", False, (True, True, True)),
    "Position only (orientation free)": ("position", True, (True, True, True)),
    "Keep tool vertical (pointing down)": ("axis", True, (True, True, True)),
    "Lock orientation (translate only)": ("pose", True, (True, True, True)),
    "Horizontal plane (fixed height + orientation)": ("pose", True, (True, True, False)),
}
DOWN = np.diag([1.0, -1.0, -1.0])   # tool z axis pointing at the desk
PROFILES = {
    "Trapezoid (fast, simple)": "trapezoid",
    "S-curve (jerk-limited, smooth)": "scurve",
    "Time-optimal (TOPP-RA, torque-aware)": "time-optimal",
}


# --------------------------------------------------------------------------- quaternions ---
def rot_to_wxyz(R: np.ndarray) -> np.ndarray:
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def wxyz_to_rot(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


# --------------------------------------------------------------------------- app -----------
class TwinApp:
    def __init__(self, arm: ArmModel, backend, port: int = 8080, payload_kg: float | None = None, cad: bool = True):
        import viser
        import yourdfpy
        from viser.extras import ViserUrdf

        self.arm = arm
        self.backend = backend
        self.q_ghost = arm.home.copy()
        self.locked: set[int] = set()
        self.traj = None                      # trajectory being executed
        self.traj_t0 = 0.0
        self.queue: deque = deque()           # trajectories waiting to run (programs)
        self.program = Program(arm_name=arm.name)
        self._program_nodes: list = []
        self._lock = threading.RLock()
        payload = getattr(backend, "payload_kg", 0.0) if payload_kg is None else payload_kg
        self.payload_kg = float(payload)
        try:                                   # inverse dynamics for the torque guard (needs mujoco)
            from .physics import TorqueModel
            self.torque_model = TorqueModel(arm, payload_kg=payload)
        except ImportError:
            self.torque_model = None

        self.server = viser.ViserServer(port=port, label=f"{arm.name} twin")
        s = self.server
        s.scene.set_up_direction("+z")
        s.scene.add_grid("/grid", width=1.2, height=1.2, cell_size=0.05, plane="xy")

        @s.on_client_connect
        def _(client) -> None:  # start every browser with a desk-scale view of the arm
            client.camera.position = (0.75, -0.65, 0.55)
            client.camera.look_at = (0.05, 0.0, 0.25)

        urdf_text = to_urdf(arm)
        cad_text = to_urdf(arm, meshes=MESH_DIR) if cad and MESH_DIR.is_dir() else urdf_text   # the CAD (DA_ARM_Display)
        self.measured = ViserUrdf(s, yourdfpy.URDF.load(io.StringIO(cad_text)), root_node_name="/measured")
        self.ghost = ViserUrdf(s, yourdfpy.URDF.load(io.StringIO(urdf_text)), root_node_name="/ghost",
                               mesh_color_override=(0.30, 0.60, 1.00, 0.35))
        self.elbow_marker = s.scene.add_icosphere("/ghost_elbow", radius=0.012, color=(255, 80, 80))
        self.axis_line = s.scene.add_line_segments("/ghost_sw_axis", np.zeros((1, 2, 3)),
                                                   colors=(255, 80, 80), thickness=0.003)

        T = arm.fk(self.q_ghost)
        self._held_R = T[:3, :3].copy()
        self._held_z = float(T[2, 3])
        self.gizmo = s.scene.add_transform_controls("/target", scale=0.12, position=T[:3, 3],
                                                    wxyz=rot_to_wxyz(T[:3, :3]))
        self.gizmo.on_update(self._on_gizmo)

        g = s.gui
        self.status = g.add_markdown("**Backend:** starting")
        with g.add_folder("Control"):
            self.mode = g.add_dropdown("IK mode", tuple(MODES), initial_value=tuple(MODES)[0])
            self.mode.on_update(self._on_mode)
            self.constraint = g.add_dropdown("Tool constraint", tuple(CONSTRAINTS),
                                             initial_value=tuple(CONSTRAINTS)[0])
            self.constraint.on_update(self._on_constraint)
            self.desk_guard = g.add_checkbox("Collision guard (desk + self-collision)", initial_value=True)
            self.profile = g.add_dropdown("Motion profile", tuple(PROFILES), initial_value=tuple(PROFILES)[0])
            self.enable_btn = g.add_button("Enable", color="green")
            self.enable_btn.on_click(lambda _: self._enable())
            self.estop_btn = g.add_button("E-STOP", color="red")
            self.estop_btn.on_click(lambda _: self._estop())
            self.torque_guard = g.add_checkbox("Torque guard (reject moves the motors can't make)",
                                               initial_value=self.torque_model is not None,
                                               disabled=self.torque_model is None)
            self.exec_btn = g.add_button("Execute: joint move (measured → ghost)")
            self.exec_btn.on_click(lambda _: self._execute())
            self.line_btn = g.add_button("Execute: straight line (measured → ghost)")
            self.line_btn.on_click(lambda _: self._execute_line())
            self.live = g.add_checkbox("Live: stream ghost to robot", initial_value=False)
            self.speed = g.add_slider("Speed scale", 0.05, 1.0, 0.05, 0.5)
            home_btn = g.add_button("Ghost → home")
            home_btn.on_click(lambda _: self._set_ghost(self.arm.home.copy(), move_gizmo=True))
            sync_btn = g.add_button("Ghost → measured")
            sync_btn.on_click(lambda _: self._set_ghost(self.backend.read(), move_gizmo=True))

        with g.add_folder("Program (teach & replay)"):
            self.prog_list = g.add_markdown("_no waypoints yet: pose the ghost, then Record_")
            self.prog_move = g.add_dropdown("Move to waypoint as", ("joint", "linear"), initial_value="joint")
            rec = g.add_button("Record ghost as waypoint")
            rec.on_click(lambda _: self._prog_record())
            undo = g.add_button("Delete last waypoint")
            undo.on_click(lambda _: self._prog_delete_last())
            clear = g.add_button("Clear program")
            clear.on_click(lambda _: self._prog_clear())
            prev = g.add_button("Preview path")
            prev.on_click(lambda _: self._prog_preview())
            self.prog_loop = g.add_checkbox("Loop playback", initial_value=False)
            play = g.add_button("Play program", color="green")
            play.on_click(lambda _: self._prog_play())
            self.prog_file = g.add_text("File", initial_value="programs/demo.json")
            save = g.add_button("Save")
            save.on_click(lambda _: self._prog_save())
            load = g.add_button("Load")
            load.on_click(lambda _: self._prog_load())

        with g.add_folder("Null space (7-DOF)"):
            self.swivel = g.add_slider("Elbow swivel ψ (deg)", -180.0, 180.0, 1.0, 0.0)
            self.swivel.on_update(self._on_swivel)

        self.sliders = []
        self.lock_boxes = []
        with g.add_folder("Joints (deg) & locks"):
            for i, j in enumerate(arm.joints):
                sl = g.add_slider(f"{j.name} {j.description}", float(np.rad2deg(j.lower)),
                                  float(np.rad2deg(j.upper)), 0.5, float(np.rad2deg(self.q_ghost[i])))
                sl.on_update(lambda e, i=i: self._on_slider(e, i))
                cb = g.add_checkbox(f"lock {j.name}", initial_value=False)
                cb.on_update(lambda e, i=i: self._on_lock(e, i))
                self.sliders.append(sl)
                self.lock_boxes.append(cb)

        self.readout = g.add_markdown("")
        self._set_ghost(self.q_ghost, move_gizmo=False)

    # ------------------------------------------------------------------ ghost updates ----
    def _set_ghost(self, q: np.ndarray, move_gizmo: bool, note: str = "") -> None:
        with self._lock:
            self.q_ghost = self.arm.clamp(np.asarray(q, dtype=float))
            q = self.q_ghost
            self.ghost.update_cfg(q)
            for sl, qi in zip(self.sliders, q):
                sl.value = float(round(np.rad2deg(qi), 2))
            psi = swivel_angle(self.arm, q)
            if not np.isnan(psi):
                self.swivel.value = float(round(np.rad2deg(psi), 1))
            S, E, W = self.arm.srs_points(q)
            self.elbow_marker.position = E
            self.axis_line.points = np.array([[S, W]], dtype=np.float32)
            T = self.arm.fk(q)
            if move_gizmo:
                self.gizmo.position = T[:3, 3]
                self.gizmo.wxyz = rot_to_wxyz(T[:3, :3])
            p = T[:3, 3] * 1000
            warns = singularity_warnings(self.arm, q)
            desk = workspace_violations(self.arm, q)
            selfhit = self_collisions(self.arm, q)
            hold = ""
            limits = self.arm.torque_limit()
            if np.isfinite(limits).any():           # static load from the mass list (numpy only)
                load = self.arm.gravity_torque(q, self.payload_kg) - spring_torque(self.arm, q)
                use = np.abs(load) / limits
                k = int(np.argmax(use))
                hold = f"  \n**holding this pose** needs {use[k] * 100:.0f}% of {self.arm.joints[k].name}'s torque"
                if use[k] > 1:
                    warns.append(f"{self.arm.joints[k].name} can't hold this pose with the payload")
            if desk:
                warns.append(f"below the desk / too close to it: {', '.join(desk)}")
            if selfhit:
                warns.append(f"self-collision: {'; '.join(selfhit)}")
            warnings = "".join(f"  \n**Warning:** {w}" for w in warns)
            self.readout.content = (
                f"**Ghost TCP** x {p[0]:.1f}  y {p[1]:.1f}  z {p[2]:.1f} mm  \n"
                f"**ψ** {np.rad2deg(psi) if not np.isnan(psi) else float('nan'):.1f}°  ·  "
                f"**manipulability** {self.arm.manipulability(q):.4f}{hold}  \n{note}{warnings}")
            if self.live.value and self.backend.enabled:
                if (desk or selfhit) and self.desk_guard.value:
                    why = "below the desk" if desk else "colliding with itself"
                    self.readout.content += f"  \n⛔ Live stream paused: ghost is {why}."
                    return
                self.traj = None
                self.backend.command(q)

    def _on_gizmo(self, event) -> None:
        task, translate_only, _ = CONSTRAINTS[self.constraint.value]
        T = np.eye(4)
        T[:3, :3] = wxyz_to_rot(self.gizmo.wxyz)
        T[:3, 3] = self.gizmo.position
        name = self.constraint.value
        if name.startswith("Keep tool vertical"):
            T[:3, :3] = DOWN
        elif name.startswith("Lock orientation") or name.startswith("Horizontal plane"):
            T[:3, :3] = self._held_R
        if name.startswith("Horizontal plane"):
            T[2, 3] = self._held_z
        res = solve_ik(self.arm, T, self.q_ghost, mode=MODES[self.mode.value], locked=self.locked,
                       task=task)
        what = {"pose": "", "position": " (position only)", "axis": " (tool axis)"}[task]
        note = (f"IK{what}: {res.message} · err {res.pos_err * 1000:.2f} mm / "
                f"{np.rad2deg(res.rot_err):.2f}°" + ("" if res.success else " (target not exactly reachable)"))
        self._set_ghost(res.q, move_gizmo=False, note=note)
        if translate_only:  # show the orientation the arm actually has
            self.gizmo.wxyz = rot_to_wxyz(self.arm.fk(self.q_ghost)[:3, :3])

    def _on_constraint(self, event) -> None:
        task, translate_only, axes = CONSTRAINTS[self.constraint.value]
        T = self.arm.fk(self.q_ghost)
        self._held_R = T[:3, :3].copy()            # orientation / height to hold in locked modes
        self._held_z = float(T[2, 3])
        self.gizmo.disable_rotations = translate_only
        self.gizmo.active_axes = axes
        if self.constraint.value.startswith("Keep tool vertical"):
            T_new = T.copy()
            T_new[:3, :3] = DOWN
            res = solve_ik(self.arm, T_new, self.q_ghost, mode=MODES[self.mode.value],
                           locked=self.locked, task="axis")
            self._set_ghost(res.q, move_gizmo=True, note=f"Tool vertical: {res.message}")
        else:
            self._set_ghost(self.q_ghost, move_gizmo=True, note=f"Constraint: {self.constraint.value}")

    def _on_slider(self, event, i: int) -> None:
        if event.client is None:
            return  # programmatic update from _set_ghost
        q = self.q_ghost.copy()
        q[i] = np.deg2rad(self.sliders[i].value)
        if MODES[self.mode.value] == "6dof":
            q[J3] = 0.0
        self._set_ghost(q, move_gizmo=True)

    def _on_lock(self, event, i: int) -> None:
        if self.lock_boxes[i].value:
            self.locked.add(i)
        else:
            self.locked.discard(i)

    def _on_swivel(self, event) -> None:
        if event.client is None:
            return
        if MODES[self.mode.value] != "7dof" or self.locked:
            self._set_ghost(self.q_ghost, move_gizmo=False,
                            note="Swivel needs 7-DOF mode with no locked joints.")
            return
        res = swivel(self.arm, self.q_ghost, np.deg2rad(self.swivel.value))
        self._set_ghost(res.q, move_gizmo=False, note=f"Swivel: {res.message}")

    def _on_mode(self, event) -> None:
        six = MODES[self.mode.value] == "6dof"
        self.sliders[J3].disabled = six
        self.swivel.disabled = six
        if six:
            res = solve_ik(self.arm, self.arm.fk(self.q_ghost), self.q_ghost, mode="6dof", locked=self.locked)
            self._set_ghost(res.q, move_gizmo=True, note=f"6-DOF mode: J3 locked at 0 ({res.message})")

    # ------------------------------------------------------------------ robot commands ---
    def _enable(self) -> None:
        self.backend.enable()

    def _estop(self) -> None:
        self.traj = None
        self.queue.clear()
        self.live.value = False
        self.backend.estop()

    def _note(self, text: str) -> None:
        self.readout.content += f"  \n{text}"

    def _check(self, trajs) -> str | None:
        """Reason to refuse a list of moves (desk / torque guards), or None if they're fine."""
        for k, tr in enumerate(trajs):
            label = f"move {k + 1}: " if len(trajs) > 1 else ""
            if self.desk_guard.value:
                hit = trajectory_violation(self.arm, tr)
                if hit is not None:
                    return (f"{label}at t = {hit[0]:.2f} s the move would put {', '.join(hit[1])} "
                            f"below the desk")
                hit = trajectory_collision(self.arm, tr)
                if hit is not None:
                    return f"{label}at t = {hit[0]:.2f} s: {'; '.join(hit[1])}"
            if self.torque_guard.value and self.torque_model is not None:
                peak, t_worst = self.torque_model.trajectory_peak(tr)
                j = int(np.argmax(peak))
                if peak[j] > 1.0:
                    return (f"{label}{self.arm.joints[j].name} would need {peak[j] * 100:.0f}% of the "
                            f"torque its motor has at that speed (t = {t_worst:.2f} s); slow down, "
                            f"use the time-optimal profile or reduce the payload")
        return None

    def _start(self, trajs, what: str) -> None:
        if not self.backend.enabled:
            self._note("Enable the robot first.")
            return
        reason = self._check(trajs)
        if reason:
            self._note(f"⛔ Not executed: {reason}.")
            return
        self.queue = deque(trajs)
        self.traj = self.queue.popleft()
        self.traj_t0 = time.monotonic()
        total = sum(t.duration for t in trajs)
        self._note(f"▶ {what}: {total:.2f} s")

    def _profile(self) -> str:
        return PROFILES[self.profile.value]

    def _execute(self) -> None:
        try:
            traj = joint_move(self.arm, self.backend.read(), self.q_ghost, self._profile(),
                              speed_scale=self.speed.value, torque_model=self.torque_model)
        except ToppError as e:
            self._note(f"⛔ No feasible timing: {e}.")
            return
        self._start([traj], f"joint move ({self._profile()})")

    def _task(self) -> str:
        return CONSTRAINTS[self.constraint.value][0]

    def _execute_line(self) -> None:
        try:
            traj, info = linear_move(self.arm, self.backend.read(), self.arm.fk(self.q_ghost),
                                     speed_scale=self.speed.value, mode=MODES[self.mode.value],
                                     locked=self.locked, task=self._task(), profile=self._profile(),
                                     torque_model=self.torque_model)
        except (CartesianError, ToppError) as e:
            self._note(f"⛔ Straight line not possible: {e}.")
            return
        slow = f", slowed ×{info.slowed_by:.1f} by joint limits" if info.slowed_by > 1.01 else ""
        lim = f", limited by {info.limited_by}" if info.limited_by and info.limited_by != "none" else ""
        self._start([traj], f"straight line {info.length_m * 1000:.0f} mm ({info.profile}){slow}{lim}")

    # ------------------------------------------------------------------ programs ---------
    def _prog_refresh(self) -> None:
        if not self.program.waypoints:
            self.prog_list.content = "_no waypoints yet: pose the ghost, then Record_"
        else:
            rows = []
            for w in self.program.waypoints:
                p = self.arm.fk(np.array(w.q))[:3, 3] * 1000
                rows.append(f"{w.name}: {w.move}, x {p[0]:.0f} y {p[1]:.0f} z {p[2]:.0f} mm")
            self.prog_list.content = "  \n".join(rows)
        for h in self._program_nodes:
            h.remove()
        self._program_nodes = []
        for k, w in enumerate(self.program.waypoints):
            p = self.arm.fk(np.array(w.q))[:3, 3]
            self._program_nodes.append(self.server.scene.add_icosphere(
                f"/program/wp{k}", radius=0.008, color=(255, 170, 0), position=p))

    def _prog_record(self) -> None:
        self.program.add(self.q_ghost, self.prog_move.value, speed=float(self.speed.value))
        self._prog_refresh()

    def _prog_delete_last(self) -> None:
        if self.program.waypoints:
            self.program.waypoints.pop()
        self._prog_refresh()

    def _prog_clear(self) -> None:
        self.program.waypoints.clear()
        self._prog_refresh()

    def _prog_build(self):
        try:
            return self.program.build(self.arm, self.backend.read(), mode=MODES[self.mode.value],
                                      locked=self.locked, task=self._task(), profile=self._profile(),
                                      torque_model=self.torque_model)
        except (CartesianError, ToppError, ValueError) as e:
            self._note(f"⛔ Program can't be built: {e}.")
            return None

    def _prog_preview(self) -> None:
        trajs = self._prog_build()
        if not trajs:
            return
        self._prog_refresh()
        pts = tcp_path(self.arm, trajs)
        segs = np.stack([pts[:-1], pts[1:]], axis=1).astype(np.float32)
        self._program_nodes.append(self.server.scene.add_line_segments(
            "/program/path", segs, colors=(255, 170, 0), thickness=0.002))
        self._note(f"Path preview: {len(trajs)} moves, {sum(t.duration for t in trajs):.1f} s")

    def _prog_play(self) -> None:
        trajs = self._prog_build()
        if trajs:
            self._start(trajs, f"program ({len(trajs)} moves)")

    def _prog_save(self) -> None:
        path = self.program.save(self.prog_file.value)
        self._note(f"Saved {len(self.program.waypoints)} waypoints to {path}")

    def _prog_load(self) -> None:
        path = Path(self.prog_file.value)
        if not path.exists():
            self._note(f"{path} not found.")
            return
        self.program = Program.load(path)
        self._prog_refresh()
        self._note(f"Loaded {len(self.program.waypoints)} waypoints from {path}")

    # ------------------------------------------------------------------ main loop --------
    def run(self, rate_hz: float = 50.0, duration: float | None = None) -> None:
        dt = 1.0 / rate_hz
        t_end = None if duration is None else time.monotonic() + duration
        last_status = 0.0
        while t_end is None or time.monotonic() < t_end:
            t = time.monotonic()
            if self.traj is not None and self.backend.enabled:
                q_cmd, _ = self.traj.sample(t - self.traj_t0)
                self.backend.command(q_cmd)
                if t - self.traj_t0 >= self.traj.duration:
                    self.traj = None
                    if not self.queue and self.prog_loop.value and self.program.waypoints:
                        trajs = self._prog_build()       # loop: rebuild from where we ended
                        if trajs and not self._check(trajs):
                            self.queue = deque(trajs)
                    if self.queue:
                        self.traj = self.queue.popleft()
                        self.traj_t0 = t
            elif self.traj is not None:                 # disabled mid-move (E-stop or fault)
                self.traj = None
                self.queue.clear()
            self.backend.step(dt)
            q_meas = self.backend.read()
            self.measured.update_cfg(q_meas)
            if t - last_status > 0.2:
                err = np.rad2deg(np.max(np.abs(self.q_ghost - q_meas)))
                state = "**enabled**" if self.backend.enabled else "**disabled / E-stopped**"
                moving = " · executing" if self.traj is not None else ""
                extra = ""
                util = getattr(self.backend, "torque_utilization", None)
                if util is not None:
                    k = int(np.argmax(util))
                    extra += (f"  \ntorque: {self.arm.joints[k].name} at {util[k] * 100:.0f}% of what its "
                              f"motor has at this speed (highest)")
                faults = getattr(self.backend, "faults", None)
                if faults:
                    extra += "  \n**Fault:** " + "; ".join(faults.values()) + " (press Enable to reset)"
                self.status.content = (f"**Backend:** {type(self.backend).__name__} {state}{moving}  \n"
                                       f"max |ghost − measured| = {err:.1f}°{extra}")
                last_status = t
            time.sleep(max(0.0, dt - (time.monotonic() - t)))


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="7-DOF desk arm digital twin")
    ap.add_argument("--config", help="arm TOML (default: bundled data/arm.toml)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--backlash-deg", type=float, default=0.0, help="simulated gearbox backlash")
    ap.add_argument("--step-deg", type=float, default=0.0,
                    help="simulated joint step size, e.g. 360/(200*16*30) = 0.00375 for a 30:1 joint")
    ap.add_argument("--backend", choices=("sim", "physics", "can", "can-sim"), default="sim",
                    help="sim = kinematic (fast, ideal motors); physics = MuJoCo with gravity and "
                         "motor torque limits (stalls if a move is too hard); can = the real MKS drivers "
                         "through the bridge on --channel; can-sim = the same code against simulated drivers. "
                         "With can, the arm must stand at its home pose when the drivers power up")
    ap.add_argument("--channel", default="COM5", help="can: the bridge's serial port (/dev/ttyACM0 on Linux)")
    ap.add_argument("--payload-kg", type=float, default=0.5, help="object in the gripper (physics)")
    ap.add_argument("--backdrivable", action="store_true",
                    help="physics: arm falls when disabled (BLDC-style), instead of holding")
    ap.add_argument("--rigid", action="store_true",
                    help="physics: ignore gearbox stiffness/backlash from arm.toml (ideal gearboxes)")
    ap.add_argument("--dual-loop", action="store_true",
                    help="physics: close an outer loop on the link angle of J2-J4 (deskarm.control.DualLoop), "
                         "so the links land on target instead of sagging on their gearboxes")
    ap.add_argument("--contact-guard", action="store_true",
                    help="physics: stop on a touch from the gear wind-up of J2-J4 (deskarm.control.ContactGuard); "
                         "streams targets like the real drivers, or every move would false-trip it")
    ap.add_argument("--primitive", action="store_true", help="draw cylinders instead of the CAD meshes")
    args = ap.parse_args(argv)
    arm = load_arm(args.config)
    if args.backend in ("can", "can-sim"):
        from .backends import CanBackend
        from .bridge import open_bus
        from .mks import SimBus
        backend = CanBackend(arm, SimBus() if args.backend == "can-sim" else open_bus(args.channel), arm.home)
    elif args.backend == "physics":
        from .physics import MujocoBackend
        backend = MujocoBackend(arm, payload_kg=args.payload_kg, backdrivable=args.backdrivable,
                                compliant=not args.rigid, streamed=args.contact_guard)
        if args.contact_guard:
            from .control import ContactGuard
            backend = ContactGuard(backend, payload_kg=args.payload_kg)
        if args.dual_loop:
            from .control import DualLoop
            backend = DualLoop(backend, payload_kg=args.payload_kg)
    else:
        backend = SimBackend(arm, backlash=np.deg2rad(args.backlash_deg),
                             resolution=np.deg2rad(args.step_deg))
    app = TwinApp(arm, backend, port=args.port, payload_kg=args.payload_kg, cad=not args.primitive)
    print(f"Twin running: open http://localhost:{args.port}")
    app.run()


if __name__ == "__main__":
    main()

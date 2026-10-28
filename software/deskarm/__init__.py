"""deskarm: kinematics, IK, trajectories and a digital twin for the 7-DOF desk arm."""
from .kinematics import ArmModel, Joint, load_arm
from .ik import IKResult, analytic_ik_6dof, analytic_ik_7dof, solve_dls, solve_ik, swivel, swivel_angle
from .trajectory import SCurveTrajectory, TrapezoidalTrajectory, joint_move
from .backends import SimBackend
from .urdf import to_urdf

__all__ = [
    "ArmModel", "Joint", "load_arm", "IKResult", "analytic_ik_6dof", "analytic_ik_7dof", "solve_dls", "solve_ik",
    "swivel", "swivel_angle", "TrapezoidalTrajectory", "SCurveTrajectory", "joint_move", "SimBackend",
    "to_urdf",
]

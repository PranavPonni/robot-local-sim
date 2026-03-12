"""Kinematic tools for 6-DOF manipulator."""
from __future__ import annotations

import numpy as np
from typing import Optional

from app.robot.robot_model import Robot6DoF
from app.math3d.transform import matrix_to_euler_rpy, homogeneous_from_rt


def pose_to_twist(T: np.ndarray) -> np.ndarray:
    # returns 6-vector [vx,vy,vz,wx,wy,wz]
    t = T[0:3, 3]
    R = T[0:3, 0:3]
    roll, pitch, yaw = matrix_to_euler_rpy(R)
    return np.concatenate([t, np.array([roll, pitch, yaw])])


def compute_jacobian(robot: Robot6DoF, joints: Optional[np.ndarray] = None) -> np.ndarray:
    q = robot.joints if joints is None else np.asarray(joints, dtype=float)
    if q.shape != (6,):
        raise ValueError("joints must be length 6")

    link_poses = robot.get_link_poses(q)
    J = np.zeros((6, 6), dtype=float)
    ee_pose = link_poses[-1]
    p_ee = ee_pose[:3, 3]

    for i in range(6):
        pi = link_poses[i][:3, 3]
        zi = link_poses[i][:3, 2]
        Jv = np.cross(zi, p_ee - pi)
        Jw = zi
        J[0:3, i] = Jv
        J[3:6, i] = Jw
    return J


def inverse_kinematics_damped_least_squares(
    robot: Robot6DoF,
    target_pose: np.ndarray,
    initial_joints: Optional[np.ndarray] = None,
    max_iter: int = 200,
    tol: float = 1e-4,
    damping: float = 1e-2,
) -> tuple[np.ndarray, bool, float]:
    q = np.array(robot.joints if initial_joints is None else initial_joints, dtype=float)
    q = robot.clamp_joints(q)

    for iteration in range(max_iter):
        fk_pose = robot.forward_kinematics(q)
        p_current = fk_pose[:3, 3]
        R_current = fk_pose[:3, 0:3]
        p_target = target_pose[:3, 3]
        R_target = target_pose[:3, 0:3]

        dp = p_target - p_current

        # orientation error with axis-angle from R_current to R_target
        R_err = R_target @ R_current.T
        angle = np.arccos(max(-1.0, min(1.0, (np.trace(R_err) - 1) / 2)))
        if abs(angle) < 1e-6:
            dr = np.zeros(3)
        else:
            dr = (angle / (2.0 * np.sin(angle))) * np.array(
                [R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]]
            )

        err = np.concatenate([dp, dr])
        err_norm = np.linalg.norm(err)
        if err_norm < tol:
            return robot.clamp_joints(q), True, err_norm

        J = compute_jacobian(robot, q)
        JJ = J @ J.T + (damping**2) * np.eye(6)
        try:
            dq = J.T @ np.linalg.solve(JJ, err)
        except np.linalg.LinAlgError:
            return q, False, err_norm

        q += dq
        q = robot.clamp_joints(q)

    final_fk = robot.forward_kinematics(q)
    final_err = np.linalg.norm(np.concatenate([target_pose[:3, 3] - final_fk[:3, 3], np.zeros(3)]))
    success = final_err < tol
    return robot.clamp_joints(q), success, final_err

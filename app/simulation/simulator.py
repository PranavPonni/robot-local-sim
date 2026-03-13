"""Robot simulation loop and motion execution."""
from __future__ import annotations

import numpy as np
import json
from typing import Optional, List

from app.robot.robot_model import Robot6DoF
from app.math3d.kinematics import inverse_kinematics_damped_least_squares


class Trajectory:
    def __init__(self):
        self.points: List[np.ndarray] = []

    def record(self, joints: np.ndarray) -> None:
        self.points.append(np.array(joints, dtype=float))

    def clear(self) -> None:
        self.points.clear()

    def to_json(self) -> str:
        return json.dumps({"trajectory": [p.tolist() for p in self.points]}, indent=2)

    @classmethod
    def from_json(cls, data: str) -> "Trajectory":
        obj = json.loads(data)
        traj = cls()
        for p in obj.get("trajectory", []):
            traj.points.append(np.array(p, dtype=float))
        return traj


class Simulator:
    def __init__(self, robot: Robot6DoF):
        self.robot = robot
        self.target_joints: Optional[np.ndarray] = None
        self.target_pose: Optional[np.ndarray] = None
        self.interp_speed = 0.5
        self.is_playing = False
        self.trajectory = Trajectory()
        self.user_mode = "joint"  # or 'cartesian'
        self.motion_queue: List[np.ndarray] = []
        self.recording = False
        self.gripper_open = 0.06  # meters opening width for visual gripper
        self.last_ik_error = 0.0

    def set_joint_target(self, target: np.ndarray) -> None:
        self.target_joints = self.robot.clamp_joints(target)
        self.user_mode = "joint"
        self.is_playing = True

    def _min_link_z(self, joints: np.ndarray) -> float:
        poses = self.robot.get_link_poses(joints)
        return float(min(p[2, 3] for p in poses))

    def _position_error(self, joints: np.ndarray, target_pose: np.ndarray) -> float:
        fk = self.robot.forward_kinematics(joints)
        return float(np.linalg.norm(fk[:3, 3] - target_pose[:3, 3]))

    def _solve_best_q(self, target_pose: np.ndarray, seeds: list[np.ndarray]) -> tuple[np.ndarray, bool, float]:
        q, success, _ = inverse_kinematics_damped_least_squares(
            self.robot,
            target_pose,
            seeds[0],
            max_iter=800,
            tol=1e-6,
            position_only=True,
        )

        min_z = self._min_link_z(q)
        best_pos_err = self._position_error(q, target_pose)
        best_score = best_pos_err + 8.0 * max(0.0, -min_z)
        best_q = q

        for seed in seeds[1:]:
            q_try, ok_try, _ = inverse_kinematics_damped_least_squares(
                self.robot,
                target_pose,
                seed,
                max_iter=800,
                tol=1e-6,
                position_only=True,
            )
            min_z_try = self._min_link_z(q_try)
            pos_err_try = self._position_error(q_try, target_pose)
            score_try = pos_err_try + 8.0 * max(0.0, -min_z_try)
            if score_try < best_score:
                best_q = q_try
                best_score = score_try
                best_pos_err = pos_err_try
                success = ok_try

        return best_q, success, best_pos_err

    def set_cartesian_target(self, target_pose: np.ndarray) -> None:
        self.target_pose = target_pose

        # Keep targets above the floor plane.
        self.target_pose[2, 3] = max(float(self.target_pose[2, 3]), 0.0)

        base_angle = float(np.arctan2(self.target_pose[1, 3], self.target_pose[0, 3]))
        seeds = [
            np.copy(self.robot.joints),
            np.array([0.0, 0.0, 1.62, 0.0, 1.5, 0.5], dtype=float),
            np.array([0.0, -0.6, 1.2, 0.0, 1.2, 0.5], dtype=float),
            np.array([0.0, 0.6, 1.2, 0.0, 1.2, 0.5], dtype=float),
            np.array([base_angle, -0.3, 1.0, 0.0, 1.1, 0.5], dtype=float),
            np.array([base_angle, 0.3, 1.3, 0.0, 1.2, 0.5], dtype=float),
        ]
        best_q, success, best_pos_err = self._solve_best_q(self.target_pose, seeds)

        # Use an elevated two-waypoint approach to avoid dipping below floor on joint interpolation.
        current_ee = self.robot.forward_kinematics(self.robot.joints)[:3, 3]
        target_pos = self.target_pose[:3, 3]
        clearance_z = max(float(current_ee[2]), float(target_pos[2])) + 0.10

        pose_up_current = np.eye(4, dtype=float)
        pose_up_current[:3, 3] = np.array([float(current_ee[0]), float(current_ee[1]), clearance_z], dtype=float)
        pose_up_target = np.eye(4, dtype=float)
        pose_up_target[:3, 3] = np.array([float(target_pos[0]), float(target_pos[1]), clearance_z], dtype=float)

        q_up_current, _, err_up_current = self._solve_best_q(pose_up_current, [np.copy(self.robot.joints), best_q])
        q_up_target, _, err_up_target = self._solve_best_q(pose_up_target, [q_up_current, best_q, np.copy(self.robot.joints)])

        self.motion_queue.clear()
        if err_up_current < 2e-2 and err_up_target < 2e-2:
            self.motion_queue = [q_up_current, q_up_target, best_q]
            self.target_joints = None
            self.is_playing = True
        else:
            self.set_joint_target(best_q)

        self.user_mode = "cartesian"
        self.is_playing = True
        self.last_ik_error = best_pos_err

        if (not success) and (best_pos_err > 1e-3):
            print(f"[IK] Note: moving to best effort (position error={best_pos_err:.6f} m)")

    def step(self, dt: float) -> None:
        if self.recording:
            self.trajectory.record(self.robot.joints)

        if self.motion_queue:
            self._process_motion_queue(dt)
            return

        if not self.is_playing or self.target_joints is None:
            return

        self._advance_to_target(dt)

    def _advance_to_target(self, dt: float) -> None:
        current = self.robot.joints
        delta = self.target_joints - current
        max_step = dt * self.interp_speed
        step = np.clip(delta, -max_step, max_step)
        # Apply step but ensure no link goes below ground (z < 0.0)
        candidate = current + step
        min_z = self._min_link_z(candidate)
        if min_z < 0.0:
            # reduce step size until full arm is above ground or step becomes tiny
            factor = 0.5
            safe_candidate = current.copy()
            while factor > 1e-3:
                trial = current + step * factor
                if self._min_link_z(trial) >= 0.0:
                    safe_candidate = trial
                    break
                factor *= 0.5
            self.robot.joints = safe_candidate
        else:
            self.robot.joints = candidate

        # If we could not move due to floor constraints, stop to avoid infinite play state.
        if np.allclose(self.robot.joints, current, atol=1e-9) and np.linalg.norm(delta) > 1e-6:
            self.is_playing = False
            return

        if self.recording:
            self.trajectory.record(self.robot.joints)

        if np.linalg.norm(self.target_joints - self.robot.joints) < 1e-3:
            self.robot.joints = self.target_joints
            if self.motion_queue:
                self.target_joints = None
            else:
                self.is_playing = False

    def _process_motion_queue(self, dt: float) -> None:
        if not self.motion_queue:
            self.is_playing = False
            return

        if self.target_joints is None:
            self.target_joints = self.robot.clamp_joints(self.motion_queue.pop(0))
            self.is_playing = True

        self._advance_to_target(dt)

        if not self.is_playing and not self.motion_queue:
            self.target_joints = None

    def reset(self) -> None:
        # Initialize robot to home position: [0.0, 0.0, 1.62, 0.0, 1.5, 0.5]
        self.robot.joints = np.array([0.0, 0.0, 1.62, 0.0, 1.5, 0.5], dtype=float)
        self.target_joints = None
        self.target_pose = None
        self.is_playing = False
        self.trajectory.clear()

    def home(self) -> None:
        # Move to home position: [0.0, 0.0, 1.62, 0.0, 1.5, 0.5]
        self.set_joint_target(np.array([0.0, 0.0, 1.62, 0.0, 1.5, 0.5], dtype=float))

    def play_trajectory(self, trajectory: Trajectory) -> None:
        if not trajectory.points:
            self.is_playing = False
            return

        self.motion_queue = [self.robot.clamp_joints(np.array(t, dtype=float)) for t in trajectory.points]
        self.target_joints = None
        self.is_playing = True

        if self.motion_queue:
            self.target_joints = self.motion_queue.pop(0)

    def start_recording(self) -> None:
        self.recording = True
        self.trajectory.clear()

    def stop_recording(self) -> None:
        self.recording = False

    def save_trajectory(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.trajectory.to_json())

    def load_trajectory(self, path: str) -> None:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
        self.trajectory = Trajectory.from_json(text)

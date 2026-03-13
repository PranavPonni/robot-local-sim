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

    def set_joint_target(self, target: np.ndarray) -> None:
        self.target_joints = self.robot.clamp_joints(target)
        self.user_mode = "joint"
        self.is_playing = True

    def set_cartesian_target(self, target_pose: np.ndarray) -> None:
        self.target_pose = target_pose

        # Keep targets above the floor plane.
        self.target_pose[2, 3] = max(float(self.target_pose[2, 3]), 0.0)

        # Solve position IK with multiple seeds and pick lowest position error.
        q, success, err = inverse_kinematics_damped_least_squares(
            self.robot,
            self.target_pose,
            self.robot.joints,
            max_iter=800,
            tol=1e-6,
            position_only=True,
        )

        best_q = q
        best_err = err
        seeds = [
            np.copy(self.robot.joints),
            np.array([0.0, 0.0, 1.62, 0.0, 1.5, 0.5], dtype=float),
            np.array([0.0, -0.6, 1.2, 0.0, 1.2, 0.5], dtype=float),
            np.array([0.0, 0.6, 1.2, 0.0, 1.2, 0.5], dtype=float),
        ]
        for seed in seeds:
            q_try, ok_try, err_try = inverse_kinematics_damped_least_squares(
                self.robot,
                self.target_pose,
                seed,
                max_iter=800,
                tol=1e-6,
                position_only=True,
            )
            if err_try < best_err:
                best_q = q_try
                best_err = err_try
                success = ok_try

        self.set_joint_target(best_q)
        self.user_mode = "cartesian"
        self.is_playing = True

        if not success and best_err > 1e-3:
            print(f"[IK] Note: moving to best effort (position error={best_err:.6f} m)")

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
        # Apply step but ensure intermediate EE doesn't go below ground (z < 0.0)
        candidate = current + step
        fk = self.robot.forward_kinematics(candidate)
        ee_z = fk[:3, 3][2]
        if ee_z < 0.0:
            # reduce step size until EE is above ground or step becomes tiny
            factor = 0.5
            safe_candidate = current.copy()
            while factor > 1e-3:
                trial = current + step * factor
                fk = self.robot.forward_kinematics(trial)
                if fk[:3, 3][2] >= 0.0:
                    safe_candidate = trial
                    break
                factor *= 0.5
            self.robot.joints = safe_candidate
        else:
            self.robot.joints = candidate

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

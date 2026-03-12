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

    def set_joint_target(self, target: np.ndarray) -> None:
        self.target_joints = self.robot.clamp_joints(target)
        self.user_mode = "joint"
        self.is_playing = True

    def set_cartesian_target(self, target_pose: np.ndarray) -> None:
        self.target_pose = target_pose
        q, success, err = inverse_kinematics_damped_least_squares(self.robot, target_pose, self.robot.joints)
        if success:
            self.set_joint_target(q)
        else:
            self.target_joints = None
        self.user_mode = "cartesian"
        self.is_playing = success

    def step(self, dt: float) -> None:
        if not self.is_playing or self.target_joints is None:
            return

        current = self.robot.joints
        delta = self.target_joints - current
        max_step = dt * self.interp_speed
        step = np.clip(delta, -max_step, max_step)
        self.robot.joints = current + step

        self.trajectory.record(self.robot.joints)

        if np.linalg.norm(self.target_joints - self.robot.joints) < 1e-3:
            self.robot.joints = self.target_joints
            self.is_playing = False

    def reset(self) -> None:
        self.robot.joints = np.zeros(6, dtype=float)
        self.target_joints = None
        self.target_pose = None
        self.is_playing = False
        self.trajectory.clear()

    def home(self) -> None:
        self.set_joint_target(np.zeros(6, dtype=float))

    def play_trajectory(self, trajectory: Trajectory) -> None:
        self.motion_queue = [t.copy() for t in trajectory.points]
        self.is_playing = True

    def step_queue(self, dt: float) -> None:
        if not self.is_playing or not self.motion_queue:
            self.is_playing = False
            return
        next_target = self.motion_queue[0]
        self.set_joint_target(next_target)
        self.step(dt)
        if not self.is_playing:
            self.motion_queue.pop(0)
            if not self.motion_queue:
                self.is_playing = False

    def save_trajectory(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.trajectory.to_json())

    def load_trajectory(self, path: str) -> None:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
        self.trajectory = Trajectory.from_json(text)

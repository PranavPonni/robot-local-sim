"""Robot model definition for a generic 6-DOF manipulator."""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from typing import Sequence

from app.math3d.transform import dh_transform


@dataclass
class JointLimits:
    min_angle: float
    max_angle: float
    max_velocity: float = 1.0  # rad/s


@dataclass
class DHLink:
    a: float
    alpha: float
    d: float
    theta_offset: float = 0.0


class Robot6DoF:
    def __init__(self, dh_links: Sequence[DHLink], joint_limits: Sequence[JointLimits], base_transform=None):
        if len(dh_links) != 6 or len(joint_limits) != 6:
            raise ValueError("Robot6DoF requires exactly 6 DH links and 6 joint limits")
        self.dh_links = list(dh_links)
        self.joint_limits = list(joint_limits)
        self.base_transform = np.eye(4) if base_transform is None else base_transform
        self.joints = np.zeros(6, dtype=float)

    @classmethod
    def default(cls) -> "Robot6DoF":
        # Typical simple 6-dof industrial manipulator scale
        links = [
            DHLink(a=0.0, alpha=np.pi / 2, d=0.3),
            DHLink(a=0.3, alpha=0.0, d=0.0),
            DHLink(a=0.25, alpha=0.0, d=0.0),
            DHLink(a=0.0, alpha=-np.pi / 2, d=0.2),  # Joint 4: corrected wrist orientation
            DHLink(a=0.0, alpha=np.pi / 2, d=0.0),   # Joint 5: corrected to match wrist
            DHLink(a=0.0, alpha=0.0, d=0.1),
        ]
        limits = [JointLimits(-np.pi, np.pi, 1.57)] * 6
        return cls(links, limits)

    def set_joints(self, angles: Sequence[float]) -> None:
        if len(angles) != 6:
            raise ValueError("angles must be length 6")
        for idx, angle in enumerate(angles):
            limit = self.joint_limits[idx]
            if angle < limit.min_angle or angle > limit.max_angle:
                raise ValueError(f"Joint {idx} angle {angle:.3f} out of limits")
            self.joints[idx] = float(angle)

    def clamp_joints(self, angles: Sequence[float]) -> np.ndarray:
        clamped = np.zeros(6, dtype=float)
        for idx, angle in enumerate(angles):
            limit = self.joint_limits[idx]
            clamped[idx] = np.clip(angle, limit.min_angle, limit.max_angle)
        return clamped

    def forward_kinematics(self, joints=None) -> np.ndarray:
        from app.math3d.transform import dh_transform

        q = self.joints if joints is None else np.asarray(joints, dtype=float)
        if q.shape != (6,):
            raise ValueError("joints must be length 6")
        T = np.array(self.base_transform, dtype=float)
        for joint, link in zip(q, self.dh_links):
            T = T @ dh_transform(link.a, link.alpha, link.d, joint + link.theta_offset)
        return T

    def get_link_poses(self, joints=None) -> list[np.ndarray]:
        q = self.joints if joints is None else np.asarray(joints, dtype=float)
        poses = [np.array(self.base_transform, dtype=float)]
        T = np.array(self.base_transform, dtype=float)
        for joint, link in zip(q, self.dh_links):
            T = T @ dh_transform(link.a, link.alpha, link.d, joint + link.theta_offset)
            poses.append(T.copy())
        return poses

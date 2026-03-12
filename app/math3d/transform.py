"""3D transform utilities for robotics-kinematics"""
from __future__ import annotations

import numpy as np

Array = np.ndarray


def rot_x(theta: float) -> Array:
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def rot_y(theta: float) -> Array:
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rot_z(theta: float) -> Array:
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def translation(x: float, y: float, z: float) -> Array:
    T = np.eye(4, dtype=float)
    T[0:3, 3] = [x, y, z]
    return T


def homogeneous_from_rt(R: Array, t: Array) -> Array:
    H = np.eye(4, dtype=float)
    H[0:3, 0:3] = R
    H[0:3, 3] = t
    return H


def dh_transform(a: float, alpha: float, d: float, theta: float) -> Array:
    c = np.cos(theta)
    s = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    T = np.array([
        [c, -s * ca, s * sa, a * c],
        [s, c * ca, -c * sa, a * s],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)
    return T


def euler_rpy_to_matrix(roll: float, pitch: float, yaw: float) -> Array:
    R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    return R


def matrix_to_euler_rpy(R: Array) -> tuple[float, float, float]:
    # Assuming R is 3x3 rotation matrix in Rz(yaw)*Ry(pitch)*Rx(roll)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def clamp(value: float, minimum: float, maximum: float) -> float:
    return float(max(minimum, min(maximum, value)))

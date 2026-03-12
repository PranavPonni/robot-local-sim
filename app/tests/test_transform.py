"""Unit tests for transform utilities."""
import numpy as np

from app.math3d.transform import rot_x, rot_y, rot_z, euler_rpy_to_matrix, matrix_to_euler_rpy


def test_rotation_matrix_inverse():
    R = rot_x(0.3) @ rot_y(0.2) @ rot_z(-0.5)
    roll, pitch, yaw = matrix_to_euler_rpy(R)
    R2 = euler_rpy_to_matrix(roll, pitch, yaw)
    assert np.allclose(R, R2, atol=1e-6)

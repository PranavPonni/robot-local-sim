"""Unit tests for forward kinematics and Jacobian."""
import numpy as np

from app.robot.robot_model import Robot6DoF
from app.math3d.kinematics import compute_jacobian


def test_forward_kinematics_identity():
    robot = Robot6DoF.default()
    robot.set_joints([0.0] * 6)
    T = robot.forward_kinematics()
    assert T.shape == (4, 4)
    assert np.allclose(T[3, :], [0.0, 0.0, 0.0, 1.0])


def test_jacobian_dimensions():
    robot = Robot6DoF.default()
    J = compute_jacobian(robot, robot.joints)
    assert J.shape == (6, 6)

"""Unit tests for IK convergence."""
import numpy as np

from app.robot.robot_model import Robot6DoF
from app.math3d.kinematics import inverse_kinematics_damped_least_squares
from app.math3d.transform import euler_rpy_to_matrix, homogeneous_from_rt


def test_ik_reaches_target():
    robot = Robot6DoF.default()
    target = homogeneous_from_rt(euler_rpy_to_matrix(0.0, 0.0, 0.0), np.array([0.2, 0.1, 0.25]))
    q, success, err = inverse_kinematics_damped_least_squares(robot, target, np.zeros(6))
    assert success
    fk = robot.forward_kinematics(q)
    assert np.linalg.norm(fk[:3, 3] - target[:3, 3]) < 1e-2

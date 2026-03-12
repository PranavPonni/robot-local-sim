"""Demo script for programmatic robot motion sequence."""
import time

import numpy as np

from app.robot.robot_model import Robot6DoF
from app.simulation.simulator import Simulator
from app.math3d.transform import euler_rpy_to_matrix, homogeneous_from_rt


def demo_pick_and_place():
    robot = Robot6DoF.default()
    sim = Simulator(robot)

    print("Starting home")
    sim.home()
    for _ in range(40):
        sim.step(0.05)
        time.sleep(0.005)
    print("At home", robot.joints)

    targets = [
        homogeneous_from_rt(euler_rpy_to_matrix(0, 0, 0), np.array([0.2, 0.0, 0.2])),
        homogeneous_from_rt(euler_rpy_to_matrix(0.0, 0.3, 0.2), np.array([0.35, -0.1, 0.15])),
        homogeneous_from_rt(euler_rpy_to_matrix(0.0, -0.4, -0.2), np.array([0.25, 0.1, 0.22])),
    ]

    for pose in targets:
        sim.set_cartesian_target(pose)
        while sim.is_playing:
            sim.step(0.05)
            time.sleep(0.005)
        print("Reached target", robot.joints)

    print("Pick-and-place demo complete")


if __name__ == "__main__":
    demo_pick_and_place()

"""Demo script: programmatic cuboid pick-and-place sequence."""
from __future__ import annotations

import time
import numpy as np

from app.robot.robot_model import Robot6DoF
from app.simulation.simulator import Simulator
from app.scene.scene import Scene
from app.math3d.transform import homogeneous_from_rt


def _run_to_target(sim: Simulator, dt: float = 0.02, max_steps: int = 2500) -> None:
    for _ in range(max_steps):
        sim.step(dt)
        if not sim.is_playing:
            break
        time.sleep(0.002)


def demo_pick_and_place() -> None:
    robot = Robot6DoF.default()
    sim = Simulator(robot)
    scene = Scene()

    # Create a cuboid object at pick location (pose translation is base position).
    cuboid_size = (0.08, 0.05, 0.06)
    pick_base = np.array([0.24, -0.06, 0.0], dtype=float)
    drop_base = np.array([0.34, 0.10, 0.0], dtype=float)
    scene.add_cuboid("box1", homogeneous_from_rt(np.eye(3), pick_base.copy()), cuboid_size)
    obj = scene.objects[-1]

    print("Home...")
    sim.home()
    _run_to_target(sim)

    # Approach above pick point.
    approach_pick = pick_base + np.array([0.0, 0.0, cuboid_size[2] + 0.08], dtype=float)
    sim.set_cartesian_target(homogeneous_from_rt(np.eye(3), approach_pick))
    _run_to_target(sim)
    print("Approached pick point", robot.forward_kinematics()[:3, 3])

    # Grasp: close gripper and attach object to tool.
    sim.gripper_open = 0.01
    carry_offset = np.array([0.0, 0.0, -(cuboid_size[2] + 0.015)], dtype=float)

    # Lift with attached object.
    ee = robot.forward_kinematics()[:3, 3]
    sim.set_cartesian_target(homogeneous_from_rt(np.eye(3), ee + np.array([0.0, 0.0, 0.12], dtype=float)))
    while sim.is_playing:
        sim.step(0.02)
        obj.pose[:3, 3] = robot.forward_kinematics()[:3, 3] + carry_offset
        time.sleep(0.002)
    print("Lifted object", obj.pose[:3, 3])

    # Move above drop location.
    approach_drop = drop_base + np.array([0.0, 0.0, cuboid_size[2] + 0.10], dtype=float)
    sim.set_cartesian_target(homogeneous_from_rt(np.eye(3), approach_drop))
    while sim.is_playing:
        sim.step(0.02)
        obj.pose[:3, 3] = robot.forward_kinematics()[:3, 3] + carry_offset
        time.sleep(0.002)

    # Lower for place.
    place_pose = drop_base + np.array([0.0, 0.0, cuboid_size[2] + 0.02], dtype=float)
    sim.set_cartesian_target(homogeneous_from_rt(np.eye(3), place_pose))
    while sim.is_playing:
        sim.step(0.02)
        obj.pose[:3, 3] = robot.forward_kinematics()[:3, 3] + carry_offset
        time.sleep(0.002)

    # Release object at drop base.
    sim.gripper_open = 0.06
    obj.pose[:3, 3] = drop_base

    # Retreat upward.
    ee = robot.forward_kinematics()[:3, 3]
    sim.set_cartesian_target(homogeneous_from_rt(np.eye(3), ee + np.array([0.0, 0.0, 0.10], dtype=float)))
    _run_to_target(sim)

    print("Placed object at", obj.pose[:3, 3])
    print("Final EE", robot.forward_kinematics()[:3, 3])
    print("Demo complete")


if __name__ == "__main__":
    demo_pick_and_place()

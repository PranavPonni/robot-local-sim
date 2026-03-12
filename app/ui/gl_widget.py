"""3D viewport widget for robot visualization."""
from __future__ import annotations

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QVector3D
import pyqtgraph.opengl as gl
import numpy as np

from app.robot.robot_model import Robot6DoF


class RobotGLView(gl.GLViewWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setCameraPosition(distance=1.2, elevation=20, azimuth=30)
        self.opts["center"] = QVector3D(0.0, 0.0, 0.0)
        self.addItem(gl.GLAxisItem())
        self.grid = gl.GLGridItem()
        self.grid.setSize(1.0, 1.0)
        self.grid.setSpacing(0.05, 0.05)
        self.addItem(self.grid)

        self.link_items: list[gl.GLGraphicsItem] = []
        self.joint_items: list[gl.GLGraphicsItem] = []
        self.ee_marker: gl.GLScatterPlotItem | None = None

    def update_robot(self, robot: Robot6DoF):
        poses = robot.get_link_poses()
        # clear old
        for item in self.link_items:
            self.removeItem(item)
        for item in self.joint_items:
            self.removeItem(item)
        if self.ee_marker:
            self.removeItem(self.ee_marker)

        self.link_items.clear()
        self.joint_items.clear()

        joints_positions = [pose[:3, 3] for pose in poses]

        for i in range(6):
            p0 = joints_positions[i]
            p1 = joints_positions[i + 1]
            line = gl.GLLinePlotItem(pos=np.vstack([p0, p1]), color=(0.6, 0.8, 1.0, 1.0), width=4.0, antialias=True)
            self.addItem(line)
            self.link_items.append(line)

        # joint markers as spheres / scatter
        pos = np.array(joints_positions)
        joint_marker = gl.GLScatterPlotItem(pos=pos, size=10, color=(1.0, 0.4, 0.4, 1.0))
        self.addItem(joint_marker)
        self.joint_items.append(joint_marker)

        # end-effector marker
        ee = np.array([poses[-1][:3, 3]])
        self.ee_marker = gl.GLScatterPlotItem(pos=ee, size=14, color=(0.1, 1.0, 0.1, 1.0))
        self.addItem(self.ee_marker)

        # joint sphere markers
        pos = np.array(joints_positions)
        self.joint_items.append(gl.GLScatterPlotItem(pos=pos, size=8, color=(1.0, 0.3, 0.3, 1.0)))
        self.addItem(self.joint_items[-1])

        # end-effector marker
        ee = np.array([poses[-1][:3, 3]])
        self.ee_marker = gl.GLScatterPlotItem(pos=ee, size=12, color=(0.1, 1.0, 0.1, 1.0))
        self.addItem(self.ee_marker)

    @staticmethod
    def _axis_to_rotation(axis: np.ndarray) -> np.ndarray:
        # create rotation matrix for z-axis to point along axis, simple method
        z = np.array([0.0, 0.0, 1.0])
        v = np.cross(z, axis)
        c = np.dot(z, axis)
        if np.linalg.norm(v) < 1e-6:
            if c > 0:
                return np.eye(3)
            else:
                return np.diag([1.0, -1.0, -1.0])
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=float)
        rot = np.eye(3) + kmat + kmat @ kmat * ((1.0 / (1.0 + c)))
        return rot

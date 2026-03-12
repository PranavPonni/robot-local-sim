"""3D viewport widget for robot visualization."""
from __future__ import annotations

from PySide6.QtWidgets import QWidget
import pyqtgraph.opengl as gl
import numpy as np

from app.robot.robot_model import Robot6DoF


def _make_link_mesh(length: float = 0.1, radius: float = 0.02, color=(0.4, 0.7, 0.9, 1.0)) -> gl.GLMeshItem:
    md = gl.MeshData.cylinder(rows=8, cols=16, radius=[radius, radius], length=length)
    item = gl.GLMeshItem(meshdata=md, smooth=True, color=color, shader="shaded", glOptions="opaque")
    return item


class RobotGLView(gl.GLViewWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setCameraPosition(distance=1.2, elevation=20, azimuth=30)
        self.opts["center"] = np.array([0.0, 0.0, 0.0])
        self.addItem(gl.GLAxisItem())
        self.grid = gl.GLGridItem()
        self.grid.setSize(1.0, 1.0)
        self.grid.setSpacing(0.05, 0.05)
        self.addItem(self.grid)

        self.link_items: list[gl.GLMeshItem] = []
        self.joint_items: list[gl.GLScatterPlotItem] = []
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
            dv = p1 - p0
            length = np.linalg.norm(dv)
            if length < 1e-6:
                continue
            center = (p0 + p1) / 2.0
            axis = dv / length

            # approximate with a small sphere for each joint and a cylinder for each link
            link = _make_link_mesh(length=length, radius=0.015, color=(0.2 + 0.1 * i, 0.5, 0.8, 1.0))
            # set transform
            rot = self._axis_to_rotation(axis)
            tr = np.eye(4)
            tr[0:3, 0:3] = rot
            tr[0:3, 3] = center
            link.setTransform(gl.Transform3D.fromMatrix(tr))
            self.addItem(link)
            self.link_items.append(link)

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

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
        self.gripper_items: list[gl.GLGraphicsItem] = []
        self.gripper_open = 0.06

    def update_robot(self, robot: Robot6DoF):
        poses = robot.get_link_poses()
        # clear old
        for item in self.link_items:
            self.removeItem(item)
        for item in self.joint_items:
            self.removeItem(item)
        for item in self.gripper_items:
            self.removeItem(item)
        if self.ee_marker:
            self.removeItem(self.ee_marker)

        self.link_items.clear()
        self.joint_items.clear()
        self.gripper_items.clear()

        joints_positions = [pose[:3, 3] for pose in poses]

        # thicker links as cylinders
        for i in range(6):
            p0 = joints_positions[i]
            p1 = joints_positions[i + 1]
            dir_vec = p1 - p0
            length = np.linalg.norm(dir_vec)
            if length < 1e-5:
                continue

            axis = dir_vec / length
            mesh = gl.MeshData.cylinder(rows=12, cols=24, radius=[0.02, 0.02], length=length)
            item = gl.GLMeshItem(meshdata=mesh, smooth=True, color=(0.2, 0.7, 1.0, 1.0), shader='shaded', drawEdges=False)

            # align cylinder z-axis to link direction
            z = np.array([0.0, 0.0, 1.0])
            dot = np.clip(np.dot(z, axis), -1.0, 1.0)
            angle = float(np.degrees(np.arccos(dot)))
            if angle > 1e-3:
                rot_axis = np.cross(z, axis)
                if np.linalg.norm(rot_axis) > 1e-6:
                    item.rotate(angle, float(rot_axis[0]), float(rot_axis[1]), float(rot_axis[2]))

            item.translate(float(p0[0]), float(p0[1]), float(p0[2]))
            self.addItem(item)
            self.link_items.append(item)

        # joint spheres
        joint_points = np.vstack(joints_positions)
        joint_item = gl.GLScatterPlotItem(pos=joint_points, size=10, color=(0.05, 0.8, 0.0, 0.95))
        self.joint_items.append(joint_item)
        self.addItem(joint_item)

        # end-effector marker
        ee = np.array([poses[-1][:3, 3]])
        self.ee_marker = gl.GLScatterPlotItem(pos=ee, size=14, color=(1.0, 0.2, 0.2, 1.0))
        self.addItem(self.ee_marker)

        # gripper open/close (fixed world orientation, two block fingers)
        ee_pos = poses[-1][:3, 3]
        half_open = self.gripper_open * 0.5
        finger_length = 0.05
        finger_thickness = 0.006
        finger_height = 0.02

        # Left and right fingers in local world X-axis
        left_center = ee_pos + np.array([half_open + finger_thickness / 2.0, 0.0, -0.02])
        right_center = ee_pos + np.array([-half_open - finger_thickness / 2.0, 0.0, -0.02])

        finger_mesh = self.create_box_mesh(width=finger_thickness, height=finger_height, depth=finger_length)

        left_finger = gl.GLMeshItem(meshdata=finger_mesh, smooth=True, color=(0.2, 0.2, 0.2, 1.0), shader='shaded', drawEdges=False)
        left_finger.translate(float(left_center[0]), float(left_center[1]), float(left_center[2]))
        self.addItem(left_finger)
        self.gripper_items.append(left_finger)

        right_finger = gl.GLMeshItem(meshdata=finger_mesh, smooth=True, color=(0.2, 0.2, 0.2, 1.0), shader='shaded', drawEdges=False)
        right_finger.translate(float(right_center[0]), float(right_center[1]), float(right_center[2]))
        self.addItem(right_finger)
        self.gripper_items.append(right_finger)


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

    @staticmethod
    def create_box_mesh(width: float, height: float, depth: float) -> gl.MeshData:
        hx = width / 2
        hy = height / 2
        hz = depth / 2
        verts = np.array([
            [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
            [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
        ], dtype=float)
        faces = np.array([
            [0, 1, 2], [0, 2, 3],
            [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4],
            [2, 3, 7], [2, 7, 6],
            [1, 2, 6], [1, 6, 5],
            [0, 3, 7], [0, 7, 4],
        ], dtype=int)
        return gl.MeshData(vertexes=verts, faces=faces)


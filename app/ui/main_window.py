"""Main window and GUI for the 6-DOF robot simulator."""
from __future__ import annotations

import os
import numpy as np

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSlider,
    QSpinBox,
    QDoubleSpinBox,
    QPushButton,
    QTextEdit,
    QGroupBox,
    QTabWidget,
    QFileDialog,
)

from app.ui.gl_widget import RobotGLView
from app.robot.robot_model import Robot6DoF
from app.simulation.simulator import Simulator
from app.math3d.transform import euler_rpy_to_matrix, homogeneous_from_rt, matrix_to_euler_rpy
from app.scene.scene import Scene


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("6-DOF Robot Arm Simulator")
        self.setStyleSheet("""
            QMainWindow { background-color: #232323; color: #eee; }
            QLabel, QGroupBox, QPushButton, QTabWidget, QTextEdit { color: #eee; font-size: 12px;}
            QPushButton { background: #007acc; border-radius: 6px; padding: 6px; }
            QPushButton:hover { background: #3399ff; }
            QTabWidget::pane { border: 1px solid #444; background: #292929; }
            QLineEdit, QDoubleSpinBox, QSlider { background: #323232; color: #fff; }
        """)
        self.robot = Robot6DoF.default()
        self.simulator = Simulator(self.robot)
        self.scene = Scene()

        self.gl_view = RobotGLView()
        self.gl_view.gripper_open = self.simulator.gripper_open
        self.info_text = QTextEdit(readOnly=True)
        self.status_label = QLabel("Ready")

        self.joint_sliders: list[QSlider] = []
        self.joint_spinboxes: list[QDoubleSpinBox] = []
        self.cartesian_spins = {}

        self._build_ui()
        self._connect_signals()

        self._update_ui_from_robot()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(30)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)

        layout.addWidget(self.gl_view, 2)

        side_panel = QVBoxLayout()

        tabs = QTabWidget()

        tabs.addTab(self._make_joint_tab(), "Joint")
        tabs.addTab(self._make_cartesian_tab(), "Cartesian")
        tabs.addTab(self._make_trajectory_tab(), "Trajectory")
        tabs.addTab(self._make_scene_tab(), "Scene")
        tabs.addTab(self._make_settings_tab(), "Settings")

        side_panel.addWidget(tabs)

        action_layout = QHBoxLayout()
        for text, slot in [
            ("Home", self.on_home),
            ("Reset", self.on_reset),
            ("Solve IK", self.on_solve_ik),
            ("Execute", self.on_execute),
            ("Demo", self.on_demo),
            ("Save Config", self.on_save_config),
            ("Load Config", self.on_load_config),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(slot)
            action_layout.addWidget(btn)
        side_panel.addLayout(action_layout)

        side_panel.addWidget(QLabel("Log console:"))
        side_panel.addWidget(self.info_text, 1)
        side_panel.addWidget(self.status_label)

        layout.addLayout(side_panel, 1)

    def _make_joint_tab(self) -> QWidget:
        page = QWidget()
        vlayout = QVBoxLayout(page)
        for i in range(6):
            group = QGroupBox(f"Joint {i+1}")
            h = QHBoxLayout(group)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            spin = QDoubleSpinBox()
            spin.setRange(-180.0, 180.0)
            spin.setSingleStep(0.1)
            spin.setDecimals(2)

            h.addWidget(slider)
            h.addWidget(spin)
            group.setLayout(h)
            vlayout.addWidget(group)

            self.joint_sliders.append(slider)
            self.joint_spinboxes.append(spin)

        return page

    def _make_cartesian_tab(self) -> QWidget:
        page = QWidget()
        vlayout = QVBoxLayout(page)
        for name in ["x", "y", "z", "roll", "pitch", "yaw"]:
            h = QHBoxLayout()
            label = QLabel(name)
            spin = QDoubleSpinBox()
            spin.setRange(-2.0, 2.0 if name in ["x", "y", "z"] else 180.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            h.addWidget(label)
            h.addWidget(spin)
            vlayout.addLayout(h)
            self.cartesian_spins[name] = spin
        return page

    def _make_trajectory_tab(self) -> QWidget:
        page = QWidget()
        vlayout = QVBoxLayout(page)
        self.record_button = QPushButton("Start Recording")
        self.record_button.clicked.connect(self.on_record_trajectory)
        btn_replay = QPushButton("Replay")
        btn_replay.clicked.connect(self.on_replay_trajectory)
        btn_save = QPushButton("Save Trajectory")
        btn_save.clicked.connect(self.on_save_trajectory)
        btn_load = QPushButton("Load Trajectory")
        btn_load.clicked.connect(self.on_load_trajectory)
        vlayout.addWidget(self.record_button)
        vlayout.addWidget(btn_replay)
        vlayout.addWidget(btn_save)
        vlayout.addWidget(btn_load)
        return page

    def _make_scene_tab(self) -> QWidget:
        page = QWidget()
        vlayout = QVBoxLayout(page)
        btn_cube = QPushButton("Add Cube")
        btn_cube.clicked.connect(self.on_add_cube)
        btn_clear = QPushButton("Clear Scene")
        btn_clear.clicked.connect(self.on_clear_scene)
        vlayout.addWidget(btn_cube)
        vlayout.addWidget(btn_clear)
        return page

    def _make_settings_tab(self) -> QWidget:
        page = QWidget()
        vlayout = QVBoxLayout(page)

        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.05, 5.0)
        self.speed_spin.setSingleStep(0.05)
        self.speed_spin.setValue(0.5)
        vlayout.addWidget(QLabel("Motion speed"))
        vlayout.addWidget(self.speed_spin)

        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(0, 60)
        self.gripper_slider.setValue(int(np.degrees(self.simulator.gripper_open)))
        self.gripper_value_label = QLabel(f"{np.degrees(self.simulator.gripper_open):.0f}°")
        vlayout.addWidget(QLabel("Gripper angle"))
        vlayout.addWidget(self.gripper_slider)
        vlayout.addWidget(self.gripper_value_label)

        return page

    def _connect_signals(self):
        for idx, (slider, spin) in enumerate(zip(self.joint_sliders, self.joint_spinboxes)):
            slider.valueChanged.connect(lambda val, i=idx: self.on_joint_slider_changed(i, val))
            spin.valueChanged.connect(lambda val, i=idx: self.on_joint_spin_changed(i, val))

        self.gripper_slider.valueChanged.connect(self.on_gripper_changed)

    def _update_ui_from_robot(self):
        for i, angle_rad in enumerate(self.robot.joints):
            deg = np.degrees(angle_rad)
            self.joint_sliders[i].blockSignals(True)
            self.joint_spinboxes[i].blockSignals(True)
            self.joint_sliders[i].setValue(int(deg))
            self.joint_spinboxes[i].setValue(float(deg))
            self.joint_sliders[i].blockSignals(False)
            self.joint_spinboxes[i].blockSignals(False)

        fk = self.robot.forward_kinematics()
        pos = fk[:3, 3]
        rpy = np.degrees(matrix_to_euler_rpy(fk[:3, :3]))
        self.status_label.setText(f"EE: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f} | rpy={rpy[0]:.1f},{rpy[1]:.1f},{rpy[2]:.1f}")
        self.gl_view.update_robot(self.robot)

    def _tick(self):
        self.simulator.interp_speed = self.speed_spin.value()
        self.simulator.step(0.03)
        self.gl_view.update_robot(self.robot)
        self._update_ui_from_robot()

    def on_joint_slider_changed(self, joint_index: int, value: int):
        rad = np.radians(value)
        self.joint_spinboxes[joint_index].blockSignals(True)
        self.joint_spinboxes[joint_index].setValue(value)
        self.joint_spinboxes[joint_index].blockSignals(False)
        self.robot.joints[joint_index] = rad

    def on_joint_spin_changed(self, joint_index: int, value: float):
        rad = np.radians(value)
        self.joint_sliders[joint_index].blockSignals(True)
        self.joint_sliders[joint_index].setValue(int(value))
        self.joint_sliders[joint_index].blockSignals(False)
        self.robot.joints[joint_index] = rad

    def on_home(self):
        self.simulator.home()
        self._log("Home command issued")

    def on_reset(self):
        self.simulator.reset()
        self._log("Reset simulator")

    def on_solve_ik(self):
        target = self._read_cartesian_target()
        if target is None:
            self._log("Invalid cartesian target")
            return
        self.simulator.set_cartesian_target(target)
        self._log("Solve IK requested")

    def on_execute(self):
        self.simulator.set_joint_target(np.copy(self.robot.joints))
        self._log("Execute motion from current joint values")

    def on_demo(self):
        # simple pick-place routine
        self.simulator.reset()
        sequence = [
            np.array([0.0, -0.2, 0.2, 0.0, 0.4, 0.0]),
            np.array([0.1, -0.1, 0.3, 0.1, 0.2, 0.0]),
            np.zeros(6),
        ]
        for p in sequence:
            self.simulator.trajectory.points.append(p)
        self.simulator.play_trajectory(self.simulator.trajectory)
        self._log("Demo pick-and-place started")

    def on_save_config(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Robot Config", "robot_config.json", "JSON files (*.json)")
        if path:
            data = {
                "dh": [{"a": l.a, "alpha": l.alpha, "d": l.d, "theta_offset": l.theta_offset} for l in self.robot.dh_links],
                "joints": self.robot.joints.tolist(),
            }
            import json

            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            self._log(f"Saved config to {path}")

    def on_load_config(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Robot Config", "", "JSON files (*.json)")
        if path:
            import json

            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            joints = np.array(data.get("joints", []), dtype=float)
            if joints.size == 6:
                self.robot.set_joints(joints)
                self._log(f"Loaded config from {path}")
            else:
                self._log("Invalid robot config")

    def on_record_trajectory(self):
        if not self.simulator.recording:
            self.simulator.start_recording()
            self.record_button.setText("Stop Recording")
            self._log("Started trajectory recording")
        else:
            self.simulator.stop_recording()
            self.record_button.setText("Start Recording")
            self._log(f"Stopped recording ({len(self.simulator.trajectory.points)} points)")

    def on_replay_trajectory(self):
        if not self.simulator.trajectory.points:
            self._log("No trajectory to replay")
            return
        self.simulator.play_trajectory(self.simulator.trajectory)
        self._log("Replaying trajectory")

    def on_gripper_changed(self, value: int):
        angle_rad = np.radians(float(value))
        self.simulator.gripper_open = angle_rad
        self.gl_view.gripper_open = angle_rad
        self.gripper_value_label.setText(f"{value}°")

    def on_save_trajectory(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Trajectory", "trajectory.json", "JSON files (*.json)")
        if path:
            self.simulator.save_trajectory(path)
            self._log(f"Saved trajectory to {path}")

    def on_load_trajectory(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Trajectory", "", "JSON files (*.json)")
        if path:
            self.simulator.load_trajectory(path)
            self._log(f"Loaded trajectory from {path}")

    def on_add_cube(self):
        pose = homogeneous_from_rt(np.eye(3), np.array([0.2, 0.0, 0.015]))
        self.scene.add_cube(f"cube{len(self.scene.objects)+1}", pose)
        self._log("Added cube to scene")

    def on_clear_scene(self):
        self.scene.clear()
        self._log("Cleared scene objects")

    def _read_cartesian_target(self):
        try:
            x = float(self.cartesian_spins["x"].value())
            y = float(self.cartesian_spins["y"].value())
            z = float(self.cartesian_spins["z"].value())
            roll = np.radians(float(self.cartesian_spins["roll"].value()))
            pitch = np.radians(float(self.cartesian_spins["pitch"].value()))
            yaw = np.radians(float(self.cartesian_spins["yaw"].value()))
            R = euler_rpy_to_matrix(roll, pitch, yaw)
            return homogeneous_from_rt(R, np.array([x, y, z]))
        except Exception:
            return None

    def _log(self, message: str):
        self.info_text.append(message)


from app.math3d.transform import matrix_to_euler_rpy

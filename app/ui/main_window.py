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
        self.simulator.reset()  # Initialize robot to home position
        self.scene = Scene()

        self.gl_view = RobotGLView()
        self.gl_view.gripper_open = self.simulator.gripper_open
        self.info_text = QTextEdit(readOnly=True)
        self.status_label = QLabel("Ready")

        self.joint_sliders: list[QSlider] = []
        self.joint_spinboxes: list[QDoubleSpinBox] = []
        self.cartesian_spins = {}
        self._max_gripper_open = 0.08  # meters
        self.pick_state: dict | None = None
        self.carrying_cube_name: str | None = None
        
        # Initialize cartesian spinboxes for Solve IK function
        for name in ["x", "y", "z", "roll", "pitch", "yaw"]:
            spin = QDoubleSpinBox()
            spin.setRange(-2.0, 2.0 if name in ["x", "y", "z"] else 180.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            # Set reasonable defaults for picking
            if name == "x":
                spin.setValue(0.3)
            elif name == "z":
                spin.setValue(0.1)
            self.cartesian_spins[name] = spin

        self._build_ui()
        self._connect_signals()

        self._update_ui_from_robot()
        self.gl_view.update_scene(self.scene)

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
            spin = QDoubleSpinBox()
            if i == 5:
                slider.setRange(0, 90)
                spin.setRange(0.0, 90.0)
            else:
                slider.setRange(-180, 180)
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
        
        # Add instructions
        vlayout.addWidget(QLabel("Cube Position:"))
        
        # Cube position controls
        self.cube_pos = {"x": None, "y": None, "z": None}
        for i, axis in enumerate(["x", "y", "z"]):
            h = QHBoxLayout()
            label = QLabel(f"{axis.upper()}:")
            spin = QDoubleSpinBox()
            spin.setRange(-1.0, 1.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.05)
            spin.setValue(0.2 if axis == "x" else 0.0)
            h.addWidget(label)
            h.addWidget(spin)
            vlayout.addLayout(h)
            self.cube_pos[axis] = spin
        
        btn_cube = QPushButton("Add Cube")
        btn_cube.clicked.connect(self.on_add_cube)
        btn_pick = QPushButton("Pick Last Cube with IK")
        btn_pick.clicked.connect(self.on_pick_cube_ik)
        btn_clear = QPushButton("Clear Scene")
        btn_clear.clicked.connect(self.on_clear_scene)
        vlayout.addWidget(btn_cube)
        vlayout.addWidget(btn_pick)
        vlayout.addWidget(btn_clear)
        
        vlayout.addWidget(QLabel("To pick a cube:"))
        vlayout.addWidget(QLabel("1. Add a cube at target position"))
        vlayout.addWidget(QLabel("2. Use Joint tab to position robot"))
        vlayout.addWidget(QLabel("3. Or use Solve IK with target pose"))
        
        vlayout.addStretch()
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
        self.gripper_slider.setRange(0, int(self._max_gripper_open * 1000))
        self.gripper_slider.setValue(int(self.simulator.gripper_open * 1000))
        self.gripper_value_label = QLabel(f"{self.simulator.gripper_open:.3f} m")
        vlayout.addWidget(QLabel("Gripper opening"))
        vlayout.addWidget(self.gripper_slider)
        vlayout.addWidget(self.gripper_value_label)

        return page

    def _connect_signals(self):
        for idx, (slider, spin) in enumerate(zip(self.joint_sliders, self.joint_spinboxes)):
            slider.valueChanged.connect(lambda val, i=idx: self.on_joint_slider_changed(i, val))
            spin.valueChanged.connect(lambda val, i=idx: self.on_joint_spin_changed(i, val))

        self.gripper_slider.valueChanged.connect(self.on_gripper_changed)

    def _joint6_angle_to_open(self, angle_rad: float) -> float:
        # Map joint-6 angle (0..pi/2) to gripper opening (sideways).
        a = float(np.clip(angle_rad, 0.0, np.pi / 2.0))
        open_val = (a / (np.pi / 2.0)) * self._max_gripper_open
        return float(open_val)

    def _open_to_joint6_angle(self, opening: float) -> float:
        frac = np.clip(opening / self._max_gripper_open, 0.0, 1.0)
        return float(frac * (np.pi / 2.0))

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
        # Sync gripper opening from joint-6 if simulator isn't driving it directly
        opening = self._joint6_angle_to_open(self.robot.joints[5])
        self.simulator.gripper_open = opening
        self.gl_view.gripper_open = opening
        try:
            self.gripper_slider.blockSignals(True)
            self.gripper_slider.setValue(int(opening * 1000))
            self.gripper_value_label.setText(f"{opening:.3f} m")
        finally:
            self.gripper_slider.blockSignals(False)

        self.gl_view.update_robot(self.robot)

    def _tick(self):
        self.simulator.interp_speed = self.speed_spin.value()
        self.simulator.step(0.03)
        self._update_pick_sequence()
        self.gl_view.update_robot(self.robot)
        self.gl_view.update_scene(self.scene)
        self._update_ui_from_robot()

    def on_joint_slider_changed(self, joint_index: int, value: int):
        rad = np.radians(value)
        self.joint_spinboxes[joint_index].blockSignals(True)
        self.joint_spinboxes[joint_index].setValue(value)
        self.joint_spinboxes[joint_index].blockSignals(False)
        self.robot.joints[joint_index] = rad
        # If joint 6 (index 5) controls gripper visually, sync gripper opening
        if joint_index == 5:
            opening = self._joint6_angle_to_open(rad)
            self.simulator.gripper_open = opening
            self.gl_view.gripper_open = opening
            self.gripper_slider.blockSignals(True)
            self.gripper_slider.setValue(int(opening * 1000))
            self.gripper_value_label.setText(f"{opening:.3f} m")
            self.gripper_slider.blockSignals(False)

    def on_joint_spin_changed(self, joint_index: int, value: float):
        rad = np.radians(value)
        self.joint_sliders[joint_index].blockSignals(True)
        self.joint_sliders[joint_index].setValue(int(value))
        self.joint_sliders[joint_index].blockSignals(False)
        self.robot.joints[joint_index] = rad
        if joint_index == 5:
            opening = self._joint6_angle_to_open(rad)
            self.simulator.gripper_open = opening
            self.gl_view.gripper_open = opening
            self.gripper_slider.blockSignals(True)
            self.gripper_slider.setValue(int(opening * 1000))
            self.gripper_value_label.setText(f"{opening:.3f} m")
            self.gripper_slider.blockSignals(False)

    def on_home(self):
        self.simulator.home()
        self._log("🏠 Moving to home position [0.0, 0.0, 1.62, 0.0, 1.5, 0.5]")

    def on_reset(self):
        self.simulator.reset()
        self._log("⟲ Reset to home position")

    def on_solve_ik(self):
        target = self._read_cartesian_target()
        if target is None:
            self._log("❌ Invalid cartesian target")
            return
        pos = target[:3, 3]
        self._log(f"🎯 Solving IK for target position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        self.simulator.set_cartesian_target(target)
        if self.simulator.is_playing:
            self._log(f"✓ IK solved! Robot moving...")
        else:
            self._log(f"⚠ Target position may not be reachable")

    def on_execute(self):
        self.simulator.set_joint_target(np.copy(self.robot.joints))
        self._log("▶ Executing motion from current joint values")

    def on_demo(self):
        # Load demo trajectory from demo.json
        self.simulator.reset()
        try:
            import json
            with open("demo.json", "r", encoding="utf-8") as f:
                data = json.load(f)
            self.simulator.trajectory.clear()
            for point in data.get("trajectory", []):
                self.simulator.trajectory.points.append(np.array(point, dtype=float))
            self.simulator.play_trajectory(self.simulator.trajectory)
            self._log(f"Demo started with {len(self.simulator.trajectory.points)} waypoints from demo.json")
        except FileNotFoundError:
            self._log("demo.json not found")
        except Exception as e:
            self._log(f"Error loading demo: {e}")

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
        opening = float(value) / 1000.0
        self.simulator.gripper_open = opening
        self.gl_view.gripper_open = opening
        self.gripper_value_label.setText(f"{opening:.3f} m")
        q6 = self._open_to_joint6_angle(opening)
        self.robot.joints[5] = q6

        deg = np.degrees(q6)
        self.joint_sliders[5].blockSignals(True)
        self.joint_spinboxes[5].blockSignals(True)
        self.joint_sliders[5].setValue(int(deg))
        self.joint_spinboxes[5].setValue(float(deg))
        self.joint_sliders[5].blockSignals(False)
        self.joint_spinboxes[5].blockSignals(False)

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
        x = float(self.cube_pos["x"].value())
        y = float(self.cube_pos["y"].value())
        z = float(self.cube_pos["z"].value())
        
        # Validate position is reachable
        if z < 0:
            self._log("❌ Cube Z position must be above ground (≥ 0)")
            return
        
        pose = homogeneous_from_rt(np.eye(3), np.array([x, y, z]))
        cube_name = f"cube{len(self.scene.objects)+1}"
        self.scene.add_cube(cube_name, pose)
        self._log(f"✓ Added {cube_name} at position ({x:.3f}, {y:.3f}, {z:.3f})")
        self._log(f"  Tip: Use 'Pick Last Cube with IK' or set Solve IK target to ({x:.3f}, {y:.3f}, {z:.3f})")

    def on_clear_scene(self):
        self.scene.clear()
        self.pick_state = None
        self.carrying_cube_name = None
        self._log("Cleared scene objects")

    def on_pick_cube_ik(self):
        if not self.scene.objects:
            self._log("❌ No cubes in scene!")
            return
        
        # Get the last cube position
        last_cube = self.scene.objects[-1]
        cube_pos = last_cube.pose[:3, 3]
        
        # Update Solve IK spinboxes to match cube position
        self.cartesian_spins["x"].blockSignals(True)
        self.cartesian_spins["y"].blockSignals(True)
        self.cartesian_spins["z"].blockSignals(True)
        self.cartesian_spins["x"].setValue(float(cube_pos[0]))
        self.cartesian_spins["y"].setValue(float(cube_pos[1]))
        self.cartesian_spins["z"].setValue(float(cube_pos[2]))
        self.cartesian_spins["x"].blockSignals(False)
        self.cartesian_spins["y"].blockSignals(False)
        self.cartesian_spins["z"].blockSignals(False)
        
        # Approach above the top of cube and then pick/lift in tick callback.
        approach_height = max(0.05, float(last_cube.size) + 0.03)
        target_pos = cube_pos + np.array([0.0, 0.0, approach_height])
        
        # Create target pose (identity rotation, target position)
        target_pose = homogeneous_from_rt(np.eye(3), target_pos)
        
        # Log the attempt
        self._log(f"🎯 Solving IK to pick cube at ({cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f})")
        
        # Start staged pick sequence
        self.pick_state = {"cube": last_cube.name, "stage": "approach", "lift": 0.12}
        self.simulator.set_cartesian_target(target_pose)
        
        # Check if motion started
        if self.simulator.is_playing:
            self._log(f"✓ IK solved! Robot moving to approach position above cube")
        else:
            self._log(f"⚠ IK solver had difficulty - position may not be reachable")

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

    def _get_scene_object(self, name: str):
        for obj in self.scene.objects:
            if obj.name == name:
                return obj
        return None

    def _update_pick_sequence(self):
        # Keep carried cube attached below end-effector.
        if self.carrying_cube_name:
            obj = self._get_scene_object(self.carrying_cube_name)
            if obj is not None:
                ee_pos = self.robot.forward_kinematics()[:3, 3]
                obj.pose[:3, 3] = ee_pos + np.array([0.0, 0.0, -0.06])

        if not self.pick_state:
            return

        if self.simulator.is_playing:
            return

        cube_name = self.pick_state["cube"]
        stage = self.pick_state["stage"]

        if stage == "approach":
            # Close gripper and attach cube.
            self.simulator.gripper_open = 0.01
            self.gl_view.gripper_open = 0.01
            self.carrying_cube_name = cube_name

            fk = self.robot.forward_kinematics()
            lift_target = fk[:3, 3] + np.array([0.0, 0.0, float(self.pick_state["lift"])])
            lift_pose = homogeneous_from_rt(np.eye(3), lift_target)
            self.pick_state["stage"] = "lift"
            self.simulator.set_cartesian_target(lift_pose)
            self._log("🧲 Cube grasped, lifting...")
            return

        if stage == "lift":
            self.pick_state = None
            self._log("✅ Pick complete")


from app.math3d.transform import matrix_to_euler_rpy

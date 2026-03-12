# 6-DOF Robot Arm Local Simulator

A standalone Python desktop app for a generic 6-DOF robot manipulator. Visual simulation, FK/IK, trajectory recording/replay, and pick-and-place demo.

## Structure

- `app/main.py`: entry point
- `app/ui/`: PySide6 GUI and OpenGL viewport
- `app/robot/`: robot kinematic robot model
- `app/math3d/`: transforms, FK, Jacobian, IK
- `app/simulation/`: simulation loop and trajectory operations
- `app/scene/`: scene object model
- `app/io/`: JSON IO utilities
- `app/tests/`: unit tests
- `demo.py`: scripted robot motion
- `example_robot_config.json`, `example_trajectory.json`: config samples

## Setup

1. Create and activate Python 3.11+ virtual environment.
2. Install dependencies:

```bash
pip install -r requirements.txt
```

## Run

```bash
python -m app.main
```

### Demo script

```bash
python demo.py
```

### Browser-based local host mode (optional)

```bash
pip install -r requirements.txt
uvicorn app.server:app --host 0.0.0.0 --port 3000
```

Open http://localhost:3000 in your browser to access a lightweight web dashboard for joint/cartesian control, status display, and actions.

## Features

- 6 revolute joints with configurable DH parameters and limits
- FK via DH chain, Jacobian for 6x6 twist
- Damped least squares numerical IK
- Joint sliders and numeric entry
- Cartesian target entry (x,y,z,roll,pitch,yaw)
- Motion interpolation plus simulated `Home`, `Reset`, `Solve IK`, `Execute`
- Trajectory record/replay/save/load
- Simple scene objects and pick/place demo path
- 3D viewport with link/joint/EE rendering

## Math notes

- Uses standard DH chain in `Robot6DoF` in `app/robot/robot_model.py`
- FK computes full transforms to end effector
- Jacobian computed using classical spatial derivative (cross product for linear part)
- IK solver in `app/math3d/kinematics.py` uses damped least squares with orientation error from rotation error axis-angle

## Known limitations

- No collision physics; only reachability check via IK residual.
- No true Cartesian path planning (joint-space interpolation currently)
- Basic graphics uses pyqtgraph, not a full physics engine
- Scene object rendering does not affect planning.

## Tests

```bash
pip install pytest
pytest -q
```

## Future extension

- Add ROS2 adapter in `app/controllers` and external node
- Collision detection pipelines
- Camera viewpoint presets, keyboard shortcuts, scripting console
- Optional physics backend (PyBullet / ODE)


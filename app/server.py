"""HTTP server layer for remote-control and browser GUI on localhost:3000."""
from __future__ import annotations

import threading
import time
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel
import numpy as np

from app.robot.robot_model import Robot6DoF
from app.simulation.simulator import Simulator
from app.math3d.transform import euler_rpy_to_matrix, homogeneous_from_rt, matrix_to_euler_rpy

app = FastAPI(title="6-DOF Robot Simulator API")

robot = Robot6DoF.default()
sim = Simulator(robot)

class RobotPose(BaseModel):
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

class JointTarget(BaseModel):
    joints: list[float]

@app.on_event("startup")
def start_simulation_thread():
    def loop():
        while True:
            sim.step(1.0 / 30.0)
            time.sleep(1.0 / 30.0)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()

@app.get("/", response_class=HTMLResponse)
def index():
    html = """
<html><head><title>6-DOF Robot Web Control</title>
<style>body{background:#121212;color:#eee;font-family:Arial;}.container{display:grid;grid-template-columns:1fr 1fr;gap:16px;max-width:1100px;margin:16px auto;padding:10px;}
panel{background:#1f1f1f;border:1px solid #444;padding:12px;border-radius:8px;}
button{background:#0087ff;color:#fff;padding:8px 12px;border:none;border-radius:6px;cursor:pointer;}</style>
</head>
<body><h1>6-DOF Robot Web Control (localhost:3000)</h1><div class=container>
<panel><h2>Joint Commands</h2>
<textarea id="joints" rows="4" style="width:100%;background:#292929;color:#fff;border:1px solid #333">0 0 0 0 0 0</textarea><br><button onclick="setJoint()">Set Joints</button></panel>
<panel><h2>Cartesian Command</h2>X:<input id="x" value="0.2" style="width:60px"> Y:<input id="y" value="0" style="width:60px"> Z:<input id="z" value="0.2" style="width:60px"><br>
Roll:<input id="roll" value="0" style="width:60px"> Pitch:<input id="pitch" value="0" style="width:60px"> Yaw:<input id="yaw" value="0" style="width:60px"><br>
<button onclick="setCartesian()">Set Cartesian</button></panel>
<panel><h2>Actions</h2><button onclick="home()">Home</button> <button onclick="reset()">Reset</button> <button onclick="demo()">Demo</button></panel>
<panel><h2>Status</h2><pre id="status">loading...</pre></panel>
</div><script>
async function post(path, body){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});return r.json();}
async function get(path){let r=await fetch(path);return r.json();}
async function update(){let s=await get('/status');document.getElementById('status').textContent=JSON.stringify(s,null,2);} setInterval(update,250);
async function setJoint(){let vals=document.getElementById('joints').value.trim().split(/\\s+/).map(Number);await post('/joint', {joints: vals});}
async function setCartesian(){await post('/cartesian', {x:+document.getElementById('x').value,y:+document.getElementById('y').value,z:+document.getElementById('z').value,roll:+document.getElementById('roll').value,pitch:+document.getElementById('pitch').value,yaw:+document.getElementById('yaw').value});}
async function home(){await post('/home', {});} async function reset(){await post('/reset', {});} async function demo(){await post('/demo', {});} 
</script></body></html>
"""
    return HTMLResponse(html)

@app.get("/status")
def status():
    fk = robot.forward_kinematics()
    pos = fk[:3, 3].tolist()
    rpy = [float(v) for v in np.degrees(matrix_to_euler_rpy(fk[:3, :3]))]
    return {
        "joints": robot.joints.tolist(),
        "ee": {"position": pos, "rpy_deg": rpy},
        "playing": sim.is_playing,
    }

@app.post("/joint")
def set_joint(target: JointTarget):
    if len(target.joints) != 6:
        return JSONResponse({"error": "need 6 joints"}, status_code=400)
    robot.set_joints(target.joints)
    sim.set_joint_target(robot.joints)
    return {"ok": True}

@app.post("/cartesian")
def set_cartesian(target: RobotPose):
    pose = homogeneous_from_rt(euler_rpy_to_matrix(np.radians(target.roll), np.radians(target.pitch), np.radians(target.yaw)), np.array([target.x, target.y, target.z]))
    sim.set_cartesian_target(pose)
    return {"ok": True}

@app.post("/home")
def home():
    sim.home()
    return {"ok": True}

@app.post("/reset")
def reset():
    sim.reset()
    return {"ok": True}

@app.post("/demo")
def demo():
    sim.reset()
    seq = [
        [0.0, -0.2, 0.2, 0.0, 0.4, 0.0],
        [0.1, -0.1, 0.3, 0.1, 0.2, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    for q in seq:
        sim.trajectory.points.append(np.array(q, dtype=float))
    sim.play_trajectory(sim.trajectory)
    return {"ok": True}

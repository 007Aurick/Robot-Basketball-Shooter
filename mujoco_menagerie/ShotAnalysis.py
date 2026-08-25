"""Hold ball, shoot pose, throw once from the free-throw line."""
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import random

scene = Path(__file__).resolve().parent.parent / "worlds" / "court.xml"
print("Loading", scene)
model = mujoco.MjModel.from_xml_path(str(scene))
data = mujoco.MjData(model)

# Ball does not collide while held (stops it from shoving the wrists).
gid = model.geom("ball_geom").id
model.geom_contype[gid] = 0
model.geom_conaffinity[gid] = 0

# Free-throw line, facing hoop.
mujoco.mj_resetDataKeyframe(model, data, 0)
data.qpos[0:3] = [0.0, -4.17, 0.793]
data.qpos[3:7] = [0.70710678, 0.0, 0.0, 0.70710678]
data.ctrl[:] = model.key_ctrl[0]

HOLD = {
    "left_shoulder_pitch_joint": -0.60,
    "left_shoulder_roll_joint": 0.25,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 1.00,
    "right_shoulder_pitch_joint": -0.60,
    "right_shoulder_roll_joint": -0.25,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 1.00,
}
SHOOT = {
    "left_shoulder_pitch_joint": -1.10,
    "left_shoulder_roll_joint": 0.38,
    "left_shoulder_yaw_joint": 0.15,
    "left_elbow_joint": 0.70,
    "right_shoulder_pitch_joint": -1.10,
    "right_shoulder_roll_joint": -0.38,
    "right_shoulder_yaw_joint": -0.15,
    "right_elbow_joint": 0.70,
}
WRISTS = [
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

hold_q = {n: int(model.jnt_qposadr[model.joint(n).id]) for n in HOLD}
hold_a = {n: model.actuator(n).id for n in HOLD}
wrist_q = [int(model.jnt_qposadr[model.joint(n).id]) for n in WRISTS]
wrist_d = [int(model.jnt_dofadr[model.joint(n).id]) for n in WRISTS]
wrist_a = [model.actuator(n).id for n in WRISTS]
shoot_q = {n: int(model.jnt_qposadr[model.joint(n).id]) for n in SHOOT}
shoot_a = {n: model.actuator(n).id for n in SHOOT}

bj = int(model.body_jntadr[model.body("basketball").id])
bq = int(model.jnt_qposadr[bj])
bv = int(model.jnt_dofadr[bj])


def apply_hold_pose():
    data.ctrl[:] = model.key_ctrl[0]
    for n, ang in HOLD.items():
        data.qpos[hold_q[n]] = ang
        data.ctrl[hold_a[n]] = ang
    for i, a in enumerate(wrist_a):
        data.qpos[wrist_q[i]] = 0.0
        data.qvel[wrist_d[i]] = 0.0
        data.ctrl[a] = 0.0
    data.qvel[0:6] = 0.0
    mujoco.mj_forward(model, data)

    rw = data.body("right_wrist_yaw_link")
    lw = data.body("left_wrist_yaw_link")
    palm_r = rw.xpos + rw.xmat.reshape(3, 3) @ np.array([0.08, 0.0, 0.0])
    palm_l = lw.xpos + lw.xmat.reshape(3, 3) @ np.array([0.08, 0.0, 0.0])
    data.qpos[bq : bq + 3] = 0.5 * (palm_r + palm_l)
    data.qpos[bq + 3 : bq + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[bv : bv + 6] = 0.0


def apply_shoot_pose():
    data.ctrl[:] = model.key_ctrl[0]
    for n, ang in SHOOT.items():
        data.qpos[shoot_q[n]] = ang
        data.ctrl[shoot_a[n]] = ang
    for i, a in enumerate(wrist_a):
        data.qpos[wrist_q[i]] = 0.0
        data.qvel[wrist_d[i]] = 0.0
        data.ctrl[a] = 0.0
    data.qvel[0:6] = 0.0
    mujoco.mj_forward(model, data)

    rw = data.body("right_wrist_yaw_link")
    lw = data.body("left_wrist_yaw_link")
    palm_r = rw.xpos + rw.xmat.reshape(3, 3) @ np.array([0.08, 0.0, 0.0])
    palm_l = lw.xpos + lw.xmat.reshape(3, 3) @ np.array([0.08, 0.0, 0.0])
    data.qpos[bq : bq + 3] = 0.5 * (palm_r + palm_l)
    data.qpos[bq + 3 : bq + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[bv : bv + 6] = 0.0


def throw_ball_step():
    data.qpos[bq + 1] += 0.25

    ball_position = data.qpos[bq : bq + 3].copy()
    hoop_position = np.array([0.0, 0.0, 3.20])
    horizontal_distance = hoop_position[1] - ball_position[1]
    height_difference = hoop_position[2] - ball_position[2]
    gravity = 9.81
    angle = np.deg2rad(50)

    clear = horizontal_distance * np.tan(angle) - height_difference
    if clear <= 0:
        raise ValueError(
            f"angle too flat for d={horizontal_distance:.2f}, h={height_difference:.2f}"
        )

    speed = np.sqrt(
        gravity * horizontal_distance**2 / (2 * np.cos(angle)**2 * clear)
    )
    speed *= 1.08

    data.qvel[bv : bv + 3] = [0.0, speed * np.cos(angle), speed * np.sin(angle)]
    data.qvel[bv + 3 : bv + 6] = 0.0

    model.geom_contype[gid] = 1
    model.geom_conaffinity[gid] = 3


released = False

with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0

    while viewer.is_running():
        if t < 3.0:
            apply_hold_pose()
        elif t < 4.0:
            apply_shoot_pose()
        elif not released:
            throw_ball_step()
            released = True
        elif t > 6.0:
            # Straight on only — vary depth from short midrange out to 3PT (~-7.24)
            x = 0.0
            y = random.uniform(-9.0, -3.5)
            data.qpos[0:3] = [x, y, 0.793]
            data.qpos[3:7] = [0.70710678, 0.0, 0.0, 0.70710678]
            data.qvel[0:6] = 0.0
            model.geom_contype[gid] = 0
            model.geom_conaffinity[gid] = 0
            released = False
            t = 0.0
            continue

        mujoco.mj_step(model, data)
        viewer.sync()
        t += model.opt.timestep
        time.sleep(model.opt.timestep)

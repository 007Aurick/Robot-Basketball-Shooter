"""Just hold the ball in front of the chest. Wrists straight. No throw."""
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

scene = Path(__file__).resolve().parent.parent / "worlds" / "court.xml"
print("Loading", scene)
model = mujoco.MjModel.from_xml_path(str(scene))
data = mujoco.MjData(model)#live sim state, 

# Ball does not collide while held (stops it from shoving the wrists).
gid = model.geom("ball_geom").id #id of the ball in XML
model.geom_contype[gid] = 0#ball has no collision type
model.geom_conaffinity[gid] = 0#ball won't collide with anyone else's types

# Free-throw line, facing hoop.
mujoco.mj_resetDataKeyframe(model, data, 0)#loads a saved pose from the model into the live sim
data.qpos[0:3] = [0.0, -4.17, 0.793]#puts the robot in the free-throw line facing the hoop
data.qpos[3:7] = [0.70710678, 0.0, 0.0, 0.70710678]#puts the robot in the free-throw line facing the hoop
data.ctrl[:] = model.key_ctrl[0]#list of motor actuator commands = saved stand targets

# Arms forward, hands close enough to sandwich the ball (r=0.2 → ~0.4m apart).
HOLD = {#dict of joint names and target angles
    "left_shoulder_pitch_joint": -0.60,
    "left_shoulder_roll_joint": 0.25,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 1.00,
    "right_shoulder_pitch_joint": -0.60,
    "right_shoulder_roll_joint": -0.25,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 1.00,
}
WRISTS = [
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

#Find the array slots once
hold_q = {n: int(model.jnt_qposadr[model.joint(n).id]) for n in HOLD}
hold_a = {n: model.actuator(n).id for n in HOLD}
wrist_q = [int(model.jnt_qposadr[model.joint(n).id]) for n in WRISTS]
wrist_d = [int(model.jnt_dofadr[model.joint(n).id]) for n in WRISTS]
wrist_a = [model.actuator(n).id for n in WRISTS]

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

    # Ball centered between the palms.
    rw = data.body("right_wrist_yaw_link")
    lw = data.body("left_wrist_yaw_link")
    palm_r = rw.xpos + rw.xmat.reshape(3, 3) @ np.array([0.08, 0.0, 0.0])
    palm_l = lw.xpos + lw.xmat.reshape(3, 3) @ np.array([0.08, 0.0, 0.0])
    data.qpos[bq : bq + 3] = 0.5 * (palm_r + palm_l)
    data.qpos[bq + 3 : bq + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[bv : bv + 6] = 0.0


apply_hold_pose()

def throw_ball():
print("Holding. Close the window to quit.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    while viewer.is_running()
        if t<1.0:
            apply_hold_pose()
            
        mujoco.mj_step(model, data)
        apply_hold_pose()
        viewer.sync()
        time.sleep(model.opt.timestep)

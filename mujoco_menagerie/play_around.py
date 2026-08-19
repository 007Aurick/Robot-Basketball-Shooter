import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

scene = Path(__file__).resolve().parent / "unitree_g1" / "scene.xml"
model = mujoco.MjModel.from_xml_path(str(scene))
data = mujoco.MjData(model)

# Start from stand pose
key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
mujoco.mj_resetDataKeyframe(model, data, key_id)
stand_ctrl = data.ctrl.copy()

wrist = model.actuator("left_wrist_pitch_joint").id
freq = 2.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    while viewer.is_running():
        data.ctrl[:] = stand_ctrl
        data.ctrl[wrist] = 0.5 * np.sin(2 * np.pi * freq * t)
        mujoco.mj_step(model, data)
        viewer.sync()
        t += model.opt.timestep
        time.sleep(model.opt.timestep)

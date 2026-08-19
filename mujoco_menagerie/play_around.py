import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

scene = Path(__file__).resolve().parent / "unitree_g1" / "scene.xml"
model = mujoco.MjModel.from_xml_path(str(scene))
data = mujoco.MjData(model)

wrist = model.actuator("left_wrist_pitch_joint").id
wrist2 = model.actuator("right_wrist_pitch_joint").id


with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    while viewer.is_running():
        data.ctrl[wrist] = 0.5 * np.sin(t)
        data.ctrl[wrist2] = 0.5 * np.sin(t)
        mujoco.mj_step(model, data)
        viewer.sync()
        t += model.opt.timestep
        time.sleep(model.opt.timestep)

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

# ShotAnalysis.py is in mujoco_menagerie/ → go up to package root, then worlds/
scene = Path(__file__).resolve().parent.parent / "worlds" / "court.xml"
model = mujoco.MjModel.from_xml_path(str(scene))
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    while viewer.is_running():
        # TODO: hold ball at hands, then release with launch velocity
        mujoco.mj_step(model, data)
        viewer.sync()
        t += model.opt.timestep
        time.sleep(model.opt.timestep)

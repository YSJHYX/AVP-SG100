import time

import mujoco
import mujoco.viewer
import numpy as np


MODEL_PATH = (
    "sg100_teleop/assets/sg100/mjcf/"
    "sg100_sim.xml"
)


model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)


# For now mathematical q = 0 is only our test command.
q_target = np.zeros(model.nu)


data.qpos[:] = 0.0
data.qvel[:] = 0.0
data.ctrl[:] = q_target

mujoco.mj_forward(model, data)


print("SG100 controlled viewer")
print("nu =", model.nu)
print()
print("q target:")
print(q_target)


with mujoco.viewer.launch_passive(model, data) as viewer:

    last = time.perf_counter()

    while viewer.is_running():

        now = time.perf_counter()

        while now - last >= model.opt.timestep:

            data.ctrl[:] = q_target

            mujoco.mj_step(model, data)

            last += model.opt.timestep

        viewer.sync()

        time.sleep(0.001)

import math
import time

import mujoco
import mujoco.viewer


MODEL_PATH = "ORIGINAL_3_DOF_ARM_mujoco.xml"


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()
        while viewer.is_running():
            t = time.time() - start

            data.ctrl[0] = 0.7 * math.sin(t)
            data.ctrl[1] = 0.5 * math.sin(t * 0.7)
            data.ctrl[2] = 0.5 * math.cos(t * 0.9)

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()

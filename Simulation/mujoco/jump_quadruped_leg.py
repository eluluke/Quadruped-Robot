import time

import mujoco
import mujoco.viewer
import numpy as np


MODEL_PATH = "Quadruped_Rear_Left_mujoco.xml"

# Joint damping is set in Quadruped_Rear_Left_mujoco.xml:
#   Hip_FR damping=3.0
#   Thigh_FR damping=2.5
#   Shank_FR damping=2.0
# Increase these to reduce oscillation; decrease them for a livelier leg.
# Jump target poses below are also useful tuning knobs.
CONTACT_FORCE_THRESHOLD = 1.0


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def mix(a: np.ndarray, b: np.ndarray, alpha: float) -> np.ndarray:
    return a * (1.0 - alpha) + b * alpha


def foot_contact_force(model: mujoco.MjModel, data: mujoco.MjData, foot_geom: int, floor_geom: int) -> float:
    total = 0.0
    force = np.zeros(6)
    for index in range(data.ncon):
        contact = data.contact[index]
        if {contact.geom1, contact.geom2} == {foot_geom, floor_geom}:
            mujoco.mj_contactForce(model, data, index, force)
            total += max(force[0], 0.0)
    return total


def main() -> None:
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    floor_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    foot_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_contact")

    neutral = np.array([0.0, 0.0, 0.0])
    preload = np.array([0.0, -0.25, 0.25])
    push = np.array([0.0, -0.65, 0.80])
    flight = np.array([0.0, 0.25, -0.20])

    print("Joint damping in XML: Hip_FR=3.0, Thigh_FR=2.5, Shank_FR=2.0")
    print("Jump poses: neutral", neutral, "preload", preload, "push", push, "flight", flight)

    data.ctrl[:] = neutral
    state = "settle"
    state_start = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()
        last_report = start

        while viewer.is_running():
            now = time.time()
            elapsed = now - start
            force = foot_contact_force(model, data, foot_geom, floor_geom)
            in_contact = force > CONTACT_FORCE_THRESHOLD
            state_time = elapsed - state_start

            if state == "settle":
                target = neutral
                if in_contact and elapsed > 0.35:
                    state = "preload"
                    state_start = elapsed
            elif state == "preload":
                target = mix(neutral, preload, smoothstep(state_time / 0.18))
                if state_time > 0.22:
                    state = "push"
                    state_start = elapsed
            elif state == "push":
                if in_contact:
                    target = mix(preload, push, smoothstep(state_time / 0.12))
                else:
                    state = "flight"
                    state_start = elapsed
                    target = flight
                if state_time > 0.16:
                    state = "flight"
                    state_start = elapsed
            else:
                target = flight
                if in_contact and state_time > 0.12:
                    state = "preload"
                    state_start = elapsed

            data.ctrl[:] = target

            mujoco.mj_step(model, data)
            viewer.sync()

            if now - last_report > 0.25:
                print(f"state={state} root_z={data.qpos[0]:.3f} foot_force={force:.1f}N contacts={data.ncon}")
                last_report = now

            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()

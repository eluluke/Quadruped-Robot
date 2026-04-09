import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)


# -----------------------------
# Device IDs
# Change these if needed
# -----------------------------
hip_id = 2
thigh_id = 1
shank_id = 0

joint_ids = [hip_id, thigh_id, shank_id]
joint_names = {
    hip_id: "Hip",
    thigh_id: "Thigh",
    shank_id: "Shank",
}


# -----------------------------
# Final gains
# -----------------------------
kp = 0.2
kd = 0.005
torque_limit = 1.5


# -----------------------------
# Very soft startup gains
# -----------------------------
startup_kp = 0.003
startup_kd = 0.001
startup_torque_limit = 0.03

mid_kp = 0.03
mid_kd = 0.002
mid_torque_limit = 0.10


# -----------------------------
# Relative motion commands
# positive = CCW, negative = CW
# -----------------------------
delta_theta = {
    hip_id: -10.0,
    thigh_id: -15.0,
    shank_id: -15.0,
}


# -----------------------------
# Motion timing
# -----------------------------
move_duration = 1.0  # seconds


# -----------------------------
# Startup stability settings
# -----------------------------
stable_velocity_threshold = 0.2
required_stable_samples = 30
max_startup_time = 4.0

rate_hz = 200.0
rate = RateLimiter(frequency=rate_hz)


def set_gains_for_all(kp_value, kd_value, torque_value):
    for device_id in joint_ids:
        bus.write_position_kp(device_id, kp_value)
        bus.write_position_kd(device_id, kd_value)
        bus.write_torque_limit(device_id, torque_value)


def set_mode_for_all(mode):
    for device_id in joint_ids:
        bus.set_mode(device_id, mode)
        bus.feed(device_id)


def get_feedback_all():
    feedback = {}

    for device_id in joint_ids:
        pos, vel = bus.write_read_pdo_2(device_id, 0.0, 0.0)
        feedback[device_id] = (pos, vel)

    return feedback


def get_stable_start_positions():
    print("Entering damping mode for all joints...")
    set_mode_for_all(recoil.Mode.DAMPING)
    time.sleep(0.1)

    stable_count = 0
    last_positions = {device_id: None for device_id in joint_ids}

    t_start = time.perf_counter()

    while True:
        feedback = get_feedback_all()

        all_valid = True
        all_stable = True

        for device_id in joint_ids:
            pos, vel = feedback[device_id]

            if pos is None or vel is None:
                all_valid = False
                break

            last_positions[device_id] = pos

            if abs(vel) >= stable_velocity_threshold:
                all_stable = False

        if all_valid:
            if all_stable:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= required_stable_samples:
                return last_positions.copy()

        if (time.perf_counter() - t_start) > max_startup_time:
            set_mode_for_all(recoil.Mode.IDLE)
            bus.stop()
            raise RuntimeError(
                "Could not get stable startup positions for all joints.")

        rate.sleep()


def sync_position_reference_all(sync_time=0.3):
    print("Synchronizing controller reference with actual encoder positions...")
    steps = int(sync_time * rate_hz)

    current_positions = {device_id: None for device_id in joint_ids}

    for _ in range(steps):
        # First read actual positions
        feedback = get_feedback_all()

        for device_id in joint_ids:
            pos, vel = feedback[device_id]
            if pos is not None:
                current_positions[device_id] = pos

        # Then command each joint to hold its CURRENT actual position
        for device_id in joint_ids:
            if current_positions[device_id] is not None:
                bus.write_read_pdo_2(
                    device_id, current_positions[device_id], 0.0)

        rate.sleep()

    for device_id in joint_ids:
        if current_positions[device_id] is None:
            raise RuntimeError(
                f"Failed to read position during sync for joint {device_id}")

    return current_positions.copy()


try:
    # --------------------------------
    # Step 1: Get stable startup positions in damping mode
    # --------------------------------
    start_positions = get_stable_start_positions()

    target_positions = {}
    for device_id in joint_ids:
        target_positions[device_id] = start_positions[device_id] + \
            delta_theta[device_id]

    print("Stable startup positions:")
    for device_id in joint_ids:
        print(
            f"{joint_names[device_id]} start:  {start_positions[device_id]:.3f} rad | "
            f"delta: {delta_theta[device_id]:.3f} rad | "
            f"target: {target_positions[device_id]:.3f} rad"
        )

    # --------------------------------
    # Step 2: Load very soft startup gains
    # --------------------------------
    set_gains_for_all(startup_kp, startup_kd, startup_torque_limit)

    # --------------------------------
    # Step 3: Enter position mode
    # --------------------------------
    set_mode_for_all(recoil.Mode.POSITION)
    time.sleep(0.05)

    # --------------------------------
    # Step 4: Synchronize controller references
    # This is the key fix for backdrive + restart issues
    # --------------------------------
    start_positions = sync_position_reference_all(sync_time=0.3)

    target_positions = {}
    for device_id in joint_ids:
        target_positions[device_id] = start_positions[device_id] + \
            delta_theta[device_id]

    print("Re-synced startup positions:")
    for device_id in joint_ids:
        print(
            f"{joint_names[device_id]} start:  {start_positions[device_id]:.3f} rad | "
            f"delta: {delta_theta[device_id]:.3f} rad | "
            f"target: {target_positions[device_id]:.3f} rad"
        )

    # --------------------------------
    # Step 5: Soft hold current positions
    # --------------------------------
    print("Soft holding all joints...")
    hold_time_1 = 0.6
    hold_steps_1 = int(hold_time_1 * rate_hz)

    for _ in range(hold_steps_1):
        for device_id in joint_ids:
            bus.write_read_pdo_2(device_id, start_positions[device_id], 0.0)
        rate.sleep()

    # --------------------------------
    # Step 6: Ramp to medium gains
    # --------------------------------
    set_gains_for_all(mid_kp, mid_kd, mid_torque_limit)

    hold_time_2 = 0.5
    hold_steps_2 = int(hold_time_2 * rate_hz)

    for _ in range(hold_steps_2):
        for device_id in joint_ids:
            bus.write_read_pdo_2(device_id, start_positions[device_id], 0.0)
        rate.sleep()

    # --------------------------------
    # Step 7: Switch to final gains
    # --------------------------------
    set_gains_for_all(kp, kd, torque_limit)

    # --------------------------------
    # Step 8: Smooth cosine move for all joints
    # q(t) = q0 + dq/2 * (1 - cos(pi*t/T))
    # zero start/end velocity
    # --------------------------------
    print("Starting smooth 3-joint move...")
    t0 = time.perf_counter()
    counter = 0

    while True:
        t = time.perf_counter() - t0

        command_positions = {}

        if t >= move_duration:
            for device_id in joint_ids:
                command_positions[device_id] = target_positions[device_id]
        else:
            s = 0.5 * (1.0 - math.cos(math.pi * t / move_duration))
            for device_id in joint_ids:
                command_positions[device_id] = start_positions[device_id] + \
                    delta_theta[device_id] * s

        measured = {}
        for device_id in joint_ids:
            pos, vel = bus.write_read_pdo_2(
                device_id, command_positions[device_id], 0.0)
            measured[device_id] = (pos, vel)

        counter += 1
        if counter % 10 == 0:
            line_parts = []
            for device_id in joint_ids:
                pos, vel = measured[device_id]
                if pos is not None and vel is not None:
                    line_parts.append(
                        f"{joint_names[device_id]} cmd: {command_positions[device_id]:.3f} "
                        f"pos: {pos:.3f} vel: {vel:.3f}"
                    )
            print(" | ".join(line_parts))

        rate.sleep()

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    set_mode_for_all(recoil.Mode.IDLE)
    bus.stop()

import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik
from homing_offsets import HOMING_OFFSET


# ============================================================
# Correct motor IDs
# ============================================================
SHANK_ID = 1
THIGH_ID = 0
HIP_ID = 2

DRIVE_IDS = [THIGH_ID, SHANK_ID]
ALL_IDS = [THIGH_ID, SHANK_ID, HIP_ID]

MOTOR_NAMES = {
    SHANK_ID: "shank",
    THIGH_ID: "thigh",
    HIP_ID: "hip",
}


# ============================================================
# Gear / sign mapping
# ============================================================
GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0


def raw_to_real_joint(motor_id, raw_motor_position):
    return raw_motor_position + HOMING_OFFSET[motor_id]


def real_joint_to_raw(motor_id, desired_joint_angle):
    return MOTOR_SIGN * (desired_joint_angle - HOMING_OFFSET[motor_id]) * GEAR_RATIO


# ============================================================
# Neutral standing pose
# ============================================================
NEUTRAL_X = 0.0
NEUTRAL_Y = 84.26
NEUTRAL_Z = 382.0

MOVE_TO_NEUTRAL_TIME = 5.0

NEUTRAL_KP = 0.035
NEUTRAL_KD = 0.002

NEUTRAL_TORQUE_LIMIT_BY_JOINT = {
    THIGH_ID: 0.38,
    SHANK_ID: 0.38,
    HIP_ID: 0.75,
}


# ============================================================
# Trajectory parameters
# ============================================================
CYCLE_TIME = 2.6
RATE_HZ = 80.0

STEP_LENGTH = 60.0
STEP_HEIGHT = 45.0

Y_PLANE = 84.26
Z_GROUND = 382.0
X_CENTER = 0.0

STANCE_RATIO = 0.50

MOVE_TO_START_TIME = 3.0

STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP = 0.025
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.12

RUN_KP = 0.060
RUN_KD = 0.002
RUN_TORQUE_LIMIT = 0.28

PRINT_EVERY = 20


# ============================================================
# Setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)


# ============================================================
# Low-level helpers
# ============================================================
def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.006)
    bus.feed(motor_id)
    time.sleep(0.006)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_drive_gains(kp, kd, torque_limit):
    for motor_id in DRIVE_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def set_neutral_gains():
    for motor_id in ALL_IDS:
        set_gains(
            motor_id,
            NEUTRAL_KP,
            NEUTRAL_KD,
            NEUTRAL_TORQUE_LIMIT_BY_JOINT[motor_id],
        )


def read_raw_position(motor_id):
    pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)
    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")
    return pos


def sync_reference(motor_id, sync_time=0.35):
    current_pos = None
    steps = int(sync_time * RATE_HZ)

    for _ in range(steps):
        pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(motor_id, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync {MOTOR_NAMES[motor_id]}")

    return current_pos


def move_ids_to_raw_targets(ids, raw_targets, move_time):
    start_raw = {
        motor_id: read_raw_position(motor_id)
        for motor_id in ids
    }

    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        for motor_id in ids:
            cmd = start_raw[motor_id] + (
                raw_targets[motor_id] - start_raw[motor_id]
            ) * s

            bus.write_read_pdo_2(motor_id, cmd, 0.0)

        rate.sleep()


def idle_all_motors():
    print("Putting all motors into IDLE and stopping CAN bus...")

    for motor_id in ALL_IDS:
        try:
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
        except Exception:
            pass

    time.sleep(0.15)

    try:
        bus.stop()
    except Exception:
        pass


# ============================================================
# Neutral move
# ============================================================
def move_to_neutral_standing():
    print("\nMoving leg to neutral standing IK pose...")
    print(f"Neutral foot position: x={NEUTRAL_X}, y={NEUTRAL_Y}, z={NEUTRAL_Z}")

    theta_h, theta_t, theta_s = leg_ik(
        NEUTRAL_X,
        NEUTRAL_Y,
        NEUTRAL_Z,
    )

    raw_targets = {
        HIP_ID: real_joint_to_raw(HIP_ID, theta_h),
        THIGH_ID: real_joint_to_raw(THIGH_ID, theta_t),
        SHANK_ID: real_joint_to_raw(SHANK_ID, theta_s),
    }

    print("\nNeutral IK angles:")
    print(f"  hip   = {theta_h:.6f}")
    print(f"  thigh = {theta_t:.6f}")
    print(f"  shank = {theta_s:.6f}")

    print("\nNeutral raw targets:")
    for motor_id in ALL_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]} = "
            f"{raw_targets[motor_id]:.6f}"
        )

    set_neutral_gains()

    for motor_id in ALL_IDS:
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    move_ids_to_raw_targets(
        ALL_IDS,
        raw_targets,
        MOVE_TO_NEUTRAL_TIME,
    )

    print("Neutral standing move complete.")


# ============================================================
# Foot trajectory
# ============================================================
def foot_trajectory(phase):
    phase = phase % 1.0

    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO

        x = X_CENTER + STEP_LENGTH / 2.0 - STEP_LENGTH * u
        y = Y_PLANE
        z = Z_GROUND

        return x, y, z

    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    x = X_CENTER - STEP_LENGTH / 2.0 + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    y = Y_PLANE

    z = Z_GROUND - STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    return x, y, z


def build_command_table():
    num_points = int(CYCLE_TIME * RATE_HZ)
    table = []

    for i in range(num_points):
        phase = i / num_points
        x, y, z = foot_trajectory(phase)

        theta_h, theta_t, theta_s = leg_ik(x, y, z)

        raw_thigh = real_joint_to_raw(THIGH_ID, theta_t)
        raw_shank = real_joint_to_raw(SHANK_ID, theta_s)

        table.append(
            {
                "x": x,
                "y": y,
                "z": z,
                "theta_h": theta_h,
                "theta_t": theta_t,
                "theta_s": theta_s,
                "raw_thigh": raw_thigh,
                "raw_shank": raw_shank,
            }
        )

    return table


# ============================================================
# Main
# ============================================================
try:
    print("Foot trot with neutral standing startup")
    print("Correct IDs: shank=1, thigh=0, hip=2")
    print("Moves to neutral first, then planar thigh/shank trot.")
    print("On exit, all joints go to IDLE, not DAMPING.")
    print()

    print("Loaded HOMING_OFFSET:")
    for motor_id in ALL_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]} "
            f"(ID {motor_id}) = {HOMING_OFFSET[motor_id]:.6f}"
        )

    for motor_id in ALL_IDS:
        set_gains(
            motor_id,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    print("\nSyncing references...")
    for motor_id in ALL_IDS:
        synced = sync_reference(motor_id)
        print(
            f"  {MOTOR_NAMES[motor_id]} synced raw={synced:.5f}, "
            f"real={raw_to_real_joint(motor_id, synced):.5f}"
        )

    current_raw = {
        motor_id: read_raw_position(motor_id)
        for motor_id in ALL_IDS
    }

    print("\nSoft holding current position...")
    for _ in range(int(0.5 * RATE_HZ)):
        for motor_id in ALL_IDS:
            bus.write_read_pdo_2(motor_id, current_raw[motor_id], 0.0)
        rate.sleep()

    move_to_neutral_standing()

    # For this planar trot, hip does not need to keep fighting.
    print("\nPutting hip into IDLE for planar trot...")
    set_mode_with_spacing(HIP_ID, recoil.Mode.IDLE)

    print("Ramping thigh/shank to medium gains...")
    set_drive_gains(MID_KP, MID_KD, MID_TORQUE_LIMIT)

    current_drive_raw = {
        motor_id: read_raw_position(motor_id)
        for motor_id in DRIVE_IDS
    }

    for _ in range(int(0.4 * RATE_HZ)):
        for motor_id in DRIVE_IDS:
            bus.write_read_pdo_2(motor_id, current_drive_raw[motor_id], 0.0)
        rate.sleep()

    command_table = build_command_table()

    first_point = command_table[0]
    first_targets = {
        THIGH_ID: first_point["raw_thigh"],
        SHANK_ID: first_point["raw_shank"],
    }

    print("\nMoving slowly to first trajectory point...")
    move_ids_to_raw_targets(
        DRIVE_IDS,
        first_targets,
        MOVE_TO_START_TIME,
    )

    print("Switching to trajectory gains...")
    set_drive_gains(RUN_KP, RUN_KD, RUN_TORQUE_LIMIT)

    print("\nStarting cycloid trajectory. Press Ctrl+C to stop.")
    index = 0
    counter = 0

    while True:
        point = command_table[index]

        thigh_pos, thigh_vel = bus.write_read_pdo_2(
            THIGH_ID,
            point["raw_thigh"],
            0.0,
        )

        shank_pos, shank_vel = bus.write_read_pdo_2(
            SHANK_ID,
            point["raw_shank"],
            0.0,
        )

        counter += 1

        if counter % PRINT_EVERY == 0:
            print(
                f"x={point['x']:.1f} "
                f"y={point['y']:.1f} "
                f"z={point['z']:.1f} | "
                f"th_des={point['theta_t']:.3f} "
                f"sh_des={point['theta_s']:.3f}"
            )

        index += 1
        if index >= len(command_table):
            index = 0

        rate.sleep()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    idle_all_motors()
STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP = 0.025
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.12

RUN_KP = 0.060
RUN_KD = 0.002
RUN_TORQUE_LIMIT = 0.28

PRINT_EVERY = 20


# ============================================================
# Setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)


# ============================================================
# Low-level helpers
# ============================================================
def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.006)
    bus.feed(motor_id)
    time.sleep(0.006)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_drive_gains(kp, kd, torque_limit):
    for motor_id in DRIVE_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def read_raw_position(motor_id):
    pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)
    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")
    return pos


def sync_reference(motor_id, sync_time=0.35):
    current_pos = None
    steps = int(sync_time * RATE_HZ)

    for _ in range(steps):
        pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(motor_id, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync {MOTOR_NAMES[motor_id]}")

    return current_pos


def move_to_raw_targets(raw_targets, move_time):
    start_raw = {
        motor_id: read_raw_position(motor_id)
        for motor_id in DRIVE_IDS
    }

    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        for motor_id in DRIVE_IDS:
            cmd = start_raw[motor_id] + (
                raw_targets[motor_id] - start_raw[motor_id]
            ) * s

            bus.write_read_pdo_2(motor_id, cmd, 0.0)

        rate.sleep()


def idle_all_motors():
    print("Putting all motors into IDLE and stopping CAN bus...")

    for motor_id in [SHANK_ID, THIGH_ID, HIP_ID]:
        try:
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
        except Exception:
            pass

    time.sleep(0.15)

    try:
        bus.stop()
    except Exception:
        pass


# ============================================================
# Foot trajectory
# ============================================================
def foot_trajectory(phase):
    phase = phase % 1.0

    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO

        x = X_CENTER + STEP_LENGTH / 2.0 - STEP_LENGTH * u
        y = Y_PLANE
        z = Z_GROUND

        return x, y, z

    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    x = X_CENTER - STEP_LENGTH / 2.0 + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    y = Y_PLANE

    z = Z_GROUND - STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    return x, y, z


def build_command_table():
    num_points = int(CYCLE_TIME * RATE_HZ)
    table = []

    for i in range(num_points):
        phase = i / num_points
        x, y, z = foot_trajectory(phase)

        theta_h, theta_t, theta_s = leg_ik(x, y, z)

        raw_thigh = real_joint_to_raw(THIGH_ID, theta_t)
        raw_shank = real_joint_to_raw(SHANK_ID, theta_s)

        table.append(
            {
                "x": x,
                "y": y,
                "z": z,
                "theta_h": theta_h,
                "theta_t": theta_t,
                "theta_s": theta_s,
                "raw_thigh": raw_thigh,
                "raw_shank": raw_shank,
            }
        )

    return table


# ============================================================
# Main
# ============================================================
try:
    print("Fixed cycloid foot trajectory test")
    print("Correct IDs: shank=1, thigh=0, hip=2")
    print("Only thigh and shank move. Hip is set to IDLE.")
    print("On exit, all joints go to IDLE, not DAMPING.")
    print()

    print("Loaded HOMING_OFFSET:")
    for motor_id in [SHANK_ID, THIGH_ID, HIP_ID]:
        print(
            f"  {MOTOR_NAMES[motor_id]} "
            f"(ID {motor_id}) = {HOMING_OFFSET[motor_id]:.6f}"
        )

    set_mode_with_spacing(HIP_ID, recoil.Mode.IDLE)

    for motor_id in DRIVE_IDS:
        set_gains(
            motor_id,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    print("\nSyncing references...")
    for motor_id in DRIVE_IDS:
        synced = sync_reference(motor_id)
        print(
            f"  {MOTOR_NAMES[motor_id]} synced raw={synced:.5f}, "
            f"real={raw_to_real_joint(motor_id, synced):.5f}"
        )

    current_raw = {
        motor_id: read_raw_position(motor_id)
        for motor_id in DRIVE_IDS
    }

    print("\nSoft holding current position...")
    for _ in range(int(0.5 * RATE_HZ)):
        for motor_id in DRIVE_IDS:
            bus.write_read_pdo_2(motor_id, current_raw[motor_id], 0.0)
        rate.sleep()

    print("Ramping to medium gains...")
    set_drive_gains(MID_KP, MID_KD, MID_TORQUE_LIMIT)

    for _ in range(int(0.4 * RATE_HZ)):
        for motor_id in DRIVE_IDS:
            bus.write_read_pdo_2(motor_id, current_raw[motor_id], 0.0)
        rate.sleep()

    command_table = build_command_table()

    first_point = command_table[0]
    first_targets = {
        THIGH_ID: first_point["raw_thigh"],
        SHANK_ID: first_point["raw_shank"],
    }

    print("\nMoving slowly to first trajectory point...")
    move_to_raw_targets(first_targets, MOVE_TO_START_TIME)

    print("Switching to trajectory gains...")
    set_drive_gains(RUN_KP, RUN_KD, RUN_TORQUE_LIMIT)

    print("\nStarting cycloid trajectory. Press Ctrl+C to stop.")
    index = 0
    counter = 0

    while True:
        point = command_table[index]

        thigh_pos, thigh_vel = bus.write_read_pdo_2(
            THIGH_ID,
            point["raw_thigh"],
            0.0,
        )

        shank_pos, shank_vel = bus.write_read_pdo_2(
            SHANK_ID,
            point["raw_shank"],
            0.0,
        )

        counter += 1

        if counter % PRINT_EVERY == 0:
            print(
                f"x={point['x']:.1f} "
                f"y={point['y']:.1f} "
                f"z={point['z']:.1f} | "
                f"th_des={point['theta_t']:.3f} "
                f"sh_des={point['theta_s']:.3f}"
            )

        index += 1
        if index >= len(command_table):
            index = 0

        rate.sleep()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    idle_all_motors()

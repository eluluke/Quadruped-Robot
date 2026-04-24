import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik
from homing_offsets import HOMING_OFFSET


# ============================================================
# Motor IDs
# ============================================================
SHANK_ID = 0
THIGH_ID = 1
HIP_ID = 2

DRIVE_IDS = [THIGH_ID, SHANK_ID]

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
    return (
        MOTOR_SIGN
        * (desired_joint_angle - HOMING_OFFSET[motor_id])
        * GEAR_RATIO
    )


# ============================================================
# Trajectory tuning parameters
# ============================================================

# Full cycle time. Larger = slower.
CYCLE_TIME = 3.0

# Control update rate. Lower if CAN/control feels noisy.
RATE_HZ = 100.0

# Forward/backward stroke length in x direction [mm].
STEP_LENGTH = 50.0

# Foot lift height [mm].
# z axis points downward, so swing lift means z decreases.
STEP_HEIGHT = 50.0

# Planar motion plane.
Y_PLANE = 84.26

# Straight pull-back line height in IK frame.
Z_GROUND = 380.0

# Step center in x.
X_CENTER = 0.0

# Fraction of cycle spent pulling backward on the straight line.
STANCE_RATIO = 0.45


# ============================================================
# Startup / control tuning
# ============================================================

# Time to move from current pose to first trajectory point.
MOVE_TO_START_TIME = 3.0

# Very soft startup gains.
STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

# Middle gains before full running.
MID_KP = 0.03
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.10

# Main trajectory gains.
# If shaking, reduce KP and TORQUE_LIMIT first.
RUN_KP = 0.08
RUN_KD = 0.002
RUN_TORQUE_LIMIT = 0.25

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
    time.sleep(0.003)
    bus.feed(motor_id)
    time.sleep(0.003)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.002)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.002)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.002)


def set_drive_gains(kp, kd, torque_limit):
    for motor_id in DRIVE_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def sync_reference(motor_id, sync_time=0.30):
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


def read_raw_position(motor_id):
    pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)
    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")
    return pos


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


# ============================================================
# Foot trajectory
# ============================================================
def foot_trajectory(phase):
    phase = phase % 1.0

    # Stance phase:
    # foot pulls backward in a straight line.
    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO

        x = X_CENTER + STEP_LENGTH / 2.0 - STEP_LENGTH * u
        y = Y_PLANE
        z = Z_GROUND

        return x, y, z

    # Swing phase:
    # foot swings forward in a cycloid curve.
    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    x = X_CENTER - STEP_LENGTH / 2.0 + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    y = Y_PLANE

    # z axis points downward, so lifting foot means z decreases.
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
    print("Cycloid foot trajectory test")
    print("Planar motion on y = 84.26 mm")
    print("Hip is set to DAMPING.")
    print()

    print("Loaded HOMING_OFFSET:")
    for motor_id in [SHANK_ID, THIGH_ID, HIP_ID]:
        print(
            f"  {MOTOR_NAMES[motor_id]} "
            f"(ID {motor_id}) = {HOMING_OFFSET[motor_id]:.6f}"
        )

    # Hip damping only
    set_mode_with_spacing(HIP_ID, recoil.Mode.DAMPING)

    # Enter soft position mode for thigh + shank
    for motor_id in DRIVE_IDS:
        set_gains(
            motor_id,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    time.sleep(0.05)

    # Critical reference sync for restart/backdrive safety
    print("\nSyncing references...")
    for motor_id in DRIVE_IDS:
        synced = sync_reference(motor_id, sync_time=0.30)
        print(
            f"  {MOTOR_NAMES[motor_id]} synced raw={synced:.5f}, "
            f"real={raw_to_real_joint(motor_id, synced):.5f}"
        )

    # Soft hold current raw positions
    current_raw = {
        motor_id: read_raw_position(motor_id)
        for motor_id in DRIVE_IDS
    }

    print("\nSoft holding current position...")
    for _ in range(int(0.5 * RATE_HZ)):
        for motor_id in DRIVE_IDS:
            bus.write_read_pdo_2(motor_id, current_raw[motor_id], 0.0)
        rate.sleep()

    # Medium gains
    print("Ramping to medium gains...")
    set_drive_gains(MID_KP, MID_KD, MID_TORQUE_LIMIT)

    for _ in range(int(0.4 * RATE_HZ)):
        for motor_id in DRIVE_IDS:
            bus.write_read_pdo_2(motor_id, current_raw[motor_id], 0.0)
        rate.sleep()

    # Build trajectory table
    command_table = build_command_table()

    first_point = command_table[0]
    first_targets = {
        THIGH_ID: first_point["raw_thigh"],
        SHANK_ID: first_point["raw_shank"],
    }

    print("\nMoving slowly to first trajectory point...")
    move_to_raw_targets(first_targets, MOVE_TO_START_TIME)

    # Final trajectory gains
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
            thigh_real = (
                raw_to_real_joint(THIGH_ID, thigh_pos)
                if thigh_pos is not None else None
            )
            shank_real = (
                raw_to_real_joint(SHANK_ID, shank_pos)
                if shank_pos is not None else None
            )

            print(
                f"x={point['x']:.1f} "
                f"y={point['y']:.1f} "
                f"z={point['z']:.1f} | "
                f"th_des={point['theta_t']:.3f} "
                f"th_real={thigh_real:.3f} | "
                f"sh_des={point['theta_s']:.3f} "
                f"sh_real={shank_real:.3f}"
            )

        index += 1
        if index >= len(command_table):
            index = 0

        rate.sleep()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    print("Setting all joints to DAMPING and stopping bus...")

    try:
        for motor_id in [SHANK_ID, THIGH_ID, HIP_ID]:
            try:
                set_mode_with_spacing(motor_id, recoil.Mode.DAMPING)
            except Exception:
                pass

        time.sleep(0.05)

    finally:
        try:
            bus.stop()
        except Exception:
            pass

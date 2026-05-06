import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik

try:
    from homing_offsets import HOMING_OFFSET, ROLE_TO_ID
except ImportError:
    from homing_offsets import HOMING_OFFSET

    # Fallback to current assembled-leg IDs if ROLE_TO_ID is not present.
    ROLE_TO_ID = {
        "shank": 2,
        "thigh": 0,
        "hip": 1,
    }


# ============================================================
# Foot position IK test with manual homing offsets v2
# ============================================================
#
# Purpose:
#   Move the foot end to a manually entered x y z coordinate.
#
# This is NOT a trajectory script.
# It only:
#   1. Loads homing_offsets.py.
#   2. Uses IK to convert desired x y z to hip/thigh/shank angles.
#   3. Converts desired joint angles to raw motor targets.
#   4. Smoothly moves all joints to that single pose.
#
# Expected homing_offsets.py convention from manual_calibrate_home_offset_v2.py:
#
#   real_joint_angle =
#       raw_motor_position / (MOTOR_SIGN * GEAR_RATIO)
#       + HOMING_OFFSET[id]
#
#   raw_command =
#       MOTOR_SIGN
#       * (desired_joint_angle - HOMING_OFFSET[id])
#       * GEAR_RATIO
#
# ============================================================


# ============================================================
# Motor IDs from homing_offsets.py
# ============================================================
SHANK_ID = ROLE_TO_ID.get("shank", 2)
THIGH_ID = ROLE_TO_ID.get("thigh", 0)
HIP_ID = ROLE_TO_ID.get("hip", 1)

MOTOR_IDS = [HIP_ID, THIGH_ID, SHANK_ID]

MOTOR_NAMES = {
    SHANK_ID: "shank",
    THIGH_ID: "thigh",
    HIP_ID: "hip",
}


# ============================================================
# Gear / direction
# ============================================================
GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0


# ============================================================
# Default neutral standing pose in IK frame
# ============================================================
NEUTRAL_X = 0.0
NEUTRAL_Y = 84.26
NEUTRAL_Z = 382.0

MOVE_TO_NEUTRAL_ON_START = True
MOVE_TO_NEUTRAL_TIME = 5.0


# ============================================================
# Timing
# ============================================================
RATE_HZ = 80.0
MOVE_TIME = 3.0
PRINT_EVERY = 20


# ============================================================
# Gains
# ============================================================
STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP = 0.025
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.12

RUN_KP = 0.055
RUN_KD = 0.003
RUN_TORQUE_LIMIT = 0.28

NEUTRAL_KP = 0.035
NEUTRAL_KD = 0.002

NEUTRAL_TORQUE_LIMIT_BY_JOINT = {
    THIGH_ID: 0.38,
    SHANK_ID: 0.38,
    HIP_ID: 0.85,
}


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
    time.sleep(0.008)

    try:
        bus.feed(motor_id)
    except Exception:
        pass

    time.sleep(0.008)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)

    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)

    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_gains_all(kp, kd, torque_limit):
    for motor_id in MOTOR_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def set_neutral_gains():
    for motor_id in MOTOR_IDS:
        set_gains(
            motor_id,
            NEUTRAL_KP,
            NEUTRAL_KD,
            NEUTRAL_TORQUE_LIMIT_BY_JOINT[motor_id],
        )


def read_raw_position(motor_id):
    pos, _ = bus.write_read_pdo_2(
        motor_id,
        0.0,
        0.0,
    )

    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")

    return pos


def read_all_raw_positions():
    return {
        motor_id: read_raw_position(motor_id)
        for motor_id in MOTOR_IDS
    }


def sync_reference(motor_id, sync_time=0.35):
    """
    Hold current measured raw position after entering POSITION mode.
    This prevents startup jump.
    """
    current_pos = None
    steps = int(sync_time * RATE_HZ)

    for _ in range(steps):
        pos, _ = bus.write_read_pdo_2(
            motor_id,
            0.0,
            0.0,
        )

        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(
                motor_id,
                current_pos,
                0.0,
            )

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync {MOTOR_NAMES[motor_id]}")

    return current_pos


def sync_all_references():
    synced = {}

    for motor_id in MOTOR_IDS:
        print(f"Syncing {MOTOR_NAMES[motor_id]}...")
        synced_raw = sync_reference(motor_id)
        synced[motor_id] = synced_raw

        print(
            f"  {MOTOR_NAMES[motor_id]} synced raw={synced_raw:+.6f}, "
            f"real={raw_to_real(motor_id, synced_raw):+.6f}"
        )

    return synced


def idle_all_motors():
    print("\nPutting all motors into IDLE and stopping CAN bus...")

    for motor_id in MOTOR_IDS:
        try:
            set_mode_with_spacing(
                motor_id,
                recoil.Mode.IDLE,
            )
        except Exception:
            pass

    time.sleep(0.15)

    try:
        bus.stop()
    except Exception:
        pass


# ============================================================
# Angle mapping
# ============================================================
def raw_to_real(motor_id, raw_motor_position):
    """
    Convert raw motor position to output-side real joint angle.
    """
    return (
        raw_motor_position / (MOTOR_SIGN * GEAR_RATIO)
        + HOMING_OFFSET[motor_id]
    )


def real_to_raw(motor_id, desired_joint_angle):
    """
    Convert output-side desired joint angle to raw motor command.
    """
    return (
        MOTOR_SIGN
        * (desired_joint_angle - HOMING_OFFSET[motor_id])
        * GEAR_RATIO
    )


# ============================================================
# Smooth move
# ============================================================
def move_all_to_targets(raw_targets, move_time):
    start_raw = read_all_raw_positions()
    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        measured = {}

        # Moving joints first, hip last is usually better for stability.
        command_order = [THIGH_ID, SHANK_ID, HIP_ID]

        for motor_id in command_order:
            cmd = start_raw[motor_id] + (
                raw_targets[motor_id] - start_raw[motor_id]
            ) * s

            pos, vel = bus.write_read_pdo_2(
                motor_id,
                cmd,
                0.0,
            )

            measured[motor_id] = {
                "cmd": cmd,
                "pos": pos,
                "vel": vel,
            }

        if (i + 1) % PRINT_EVERY == 0:
            line = []

            for motor_id in command_order:
                item = measured[motor_id]
                pos = item["pos"]

                if pos is not None:
                    line.append(
                        f"{MOTOR_NAMES[motor_id]} "
                        f"cmd={item['cmd']:+.3f} "
                        f"raw={pos:+.3f} "
                        f"real={raw_to_real(motor_id, pos):+.3f}"
                    )

            print(" | ".join(line))

        rate.sleep()


# ============================================================
# IK target construction
# ============================================================
def xyz_to_raw_targets(x, y, z):
    theta_h, theta_t, theta_s = leg_ik(
        x,
        y,
        z,
    )

    desired_real = {
        HIP_ID: theta_h,
        THIGH_ID: theta_t,
        SHANK_ID: theta_s,
    }

    raw_targets = {
        motor_id: real_to_raw(
            motor_id,
            desired_real[motor_id],
        )
        for motor_id in MOTOR_IDS
    }

    return desired_real, raw_targets


def print_target_summary(x, y, z, desired_real, raw_targets):
    print("\nRequested foot position:")
    print(f"  x = {x:.3f} mm")
    print(f"  y = {y:.3f} mm")
    print(f"  z = {z:.3f} mm")

    print("\nIK angles:")
    print(f"  hip   = {desired_real[HIP_ID]:+.6f} rad")
    print(f"  thigh = {desired_real[THIGH_ID]:+.6f} rad")
    print(f"  shank = {desired_real[SHANK_ID]:+.6f} rad")

    print("\nRaw motor targets:")
    for motor_id in MOTOR_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]} "
            f"(ID {motor_id}) = {raw_targets[motor_id]:+.6f}"
        )

    print("\nRaw deltas from current:")
    current = read_all_raw_positions()

    for motor_id in MOTOR_IDS:
        delta = raw_targets[motor_id] - current[motor_id]

        print(
            f"  {MOTOR_NAMES[motor_id]} "
            f"delta = {delta:+.6f} raw rad"
        )


# ============================================================
# Neutral move
# ============================================================
def move_to_neutral_standing():
    print("\nMoving leg to neutral standing IK pose...")
    desired_real, raw_targets = xyz_to_raw_targets(
        NEUTRAL_X,
        NEUTRAL_Y,
        NEUTRAL_Z,
    )

    print_target_summary(
        NEUTRAL_X,
        NEUTRAL_Y,
        NEUTRAL_Z,
        desired_real,
        raw_targets,
    )

    set_neutral_gains()

    for motor_id in MOTOR_IDS:
        set_mode_with_spacing(
            motor_id,
            recoil.Mode.POSITION,
        )

    move_all_to_targets(
        raw_targets,
        MOVE_TO_NEUTRAL_TIME,
    )

    print("Neutral standing move complete.")


# ============================================================
# Parse xyz input
# ============================================================
def parse_xyz(text):
    parts = text.replace(",", " ").split()

    if len(parts) != 3:
        raise ValueError("Please enter x y z")

    return (
        float(parts[0]),
        float(parts[1]),
        float(parts[2]),
    )


# ============================================================
# Main
# ============================================================
try:
    print("=" * 80)
    print("Foot position IK test with manual homing offsets v2")
    print("=" * 80)
    print("This script moves the foot to one x y z coordinate at a time.")
    print()
    print("Loaded role -> ID mapping:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} -> ID {ROLE_TO_ID.get(role)}")
    print()
    print("Loaded HOMING_OFFSET:")
    for motor_id in MOTOR_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]} "
            f"(ID {motor_id}) = {HOMING_OFFSET[motor_id]:+.6f}"
        )
    print("=" * 80)

    # Startup soft gains and position mode.
    for motor_id in MOTOR_IDS:
        set_gains(
            motor_id,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )

        set_mode_with_spacing(
            motor_id,
            recoil.Mode.POSITION,
        )

    print("\nSyncing references to current physical pose...")
    sync_all_references()

    current_raw = read_all_raw_positions()

    print("\nSoft holding current position...")
    for _ in range(int(0.5 * RATE_HZ)):
        for motor_id in [THIGH_ID, SHANK_ID, HIP_ID]:
            bus.write_read_pdo_2(
                motor_id,
                current_raw[motor_id],
                0.0,
            )
        rate.sleep()

    if MOVE_TO_NEUTRAL_ON_START:
        move_to_neutral_standing()

    print("\nRamping to normal run gains...")
    set_gains_all(
        MID_KP,
        MID_KD,
        MID_TORQUE_LIMIT,
    )

    time.sleep(0.2)

    set_gains_all(
        RUN_KP,
        RUN_KD,
        RUN_TORQUE_LIMIT,
    )

    print("\nInput x y z in mm.")
    print("Example neutral: 0 84.26 382")
    print("Type q to quit.\n")

    while True:
        text = input("\nEnter x y z (mm): ").strip()

        if text.lower() in ["q", "quit", "exit"]:
            break

        try:
            x, y, z = parse_xyz(text)

            desired_real, raw_targets = xyz_to_raw_targets(
                x,
                y,
                z,
            )

            print_target_summary(
                x,
                y,
                z,
                desired_real,
                raw_targets,
            )

            go = input("\nMove? y/n: ").strip().lower()

            if go != "y":
                continue

            move_all_to_targets(
                raw_targets,
                MOVE_TIME,
            )

            print("Move complete.")

        except Exception as exc:
            print(f"Error: {exc}")

except KeyboardInterrupt:
    print("\nInterrupted.")

finally:
    idle_all_motors()

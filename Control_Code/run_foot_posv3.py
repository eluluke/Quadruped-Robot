import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik

try:
    from homing_offsets import HOMING_OFFSET, ROLE_TO_ID
except ImportError:
    from homing_offsets import HOMING_OFFSET
    ROLE_TO_ID = {
        "shank": 0,
        "thigh": 2,
        "hip": 1,
    }

# ============================================================
# Foot position IK test with manual homing offsets v3 SAFE READ
# ============================================================
# Main fix from v2:
#   Do NOT use write_read_pdo_2(motor_id, 0.0, 0.0) as a "read"
#   while motors are in POSITION mode.
#
# In this firmware interface, write_read_pdo_2 writes a command and then
# returns feedback. So sending 0.0 while in POSITION mode can pull the joint
# toward raw zero and corrupt the startup/current-position reference.
#
# This version:
#   1. Reads raw positions only while motors are IDLE.
#   2. Enters POSITION mode with zero torque first.
#   3. Immediately commands the measured startup raw positions.
#   4. Maintains an active command dictionary and never "reads" by sending 0.
#   5. Moves to neutral only after printing deltas and asking for confirmation
#      when the deltas are large.
# ============================================================


# ============================================================
# Motor IDs from homing_offsets.py
# ============================================================
SHANK_ID = ROLE_TO_ID.get("shank", 0)
THIGH_ID = ROLE_TO_ID.get("thigh", 2)
HIP_ID = ROLE_TO_ID.get("hip", 1)

# Command moving joints first, hip last.
COMMAND_ORDER = [THIGH_ID, SHANK_ID, HIP_ID]
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

# Per-joint sign adapters. Change these only after testing.
# If a joint moves exactly opposite of expected, flip that joint to -1.0.
JOINT_ANGLE_SIGN = {
    HIP_ID: 1.0,
    THIGH_ID: 1.0,
    SHANK_ID: 1.0,
}

# Optional joint assignment adapter. Keep normal first.
# If later you prove IK thigh/shank are swapped physically, change here only.
IK_ROLE_TO_PHYSICAL_ID = {
    "hip": HIP_ID,
    "thigh": THIGH_ID,
    "shank": SHANK_ID,
}


# ============================================================
# Default neutral standing pose in IK frame
# ============================================================
NEUTRAL_X = 0.0
NEUTRAL_Y = 84.26
NEUTRAL_Z = 382.0

MOVE_TO_NEUTRAL_ON_START = True
MOVE_TO_NEUTRAL_TIME = 6.0

# If the auto-neutral move requires more raw radians than this, ask first.
# This protects you from bad offsets/signs causing a mechanical-limit crash.
CONFIRM_IF_RAW_DELTA_OVER = 6.0
ABORT_IF_RAW_DELTA_OVER = 25.0


# ============================================================
# Timing
# ============================================================
RATE_HZ = 80.0
MOVE_TIME = 3.0
PRINT_EVERY = 20


# ============================================================
# Gains
# ============================================================
ARM_KP = 0.0
ARM_KD = 0.0
ARM_TORQUE_LIMIT = 0.0

STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP = 0.025
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.12

RUN_KP = 0.055
RUN_KD = 0.003
RUN_TORQUE_LIMIT = 0.25

NEUTRAL_KP = 0.030
NEUTRAL_KD = 0.002

NEUTRAL_TORQUE_LIMIT_BY_JOINT = {
    THIGH_ID: 0.30,
    SHANK_ID: 0.30,
    HIP_ID: 0.65,
}


# ============================================================
# Setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)

# This tracks what the controller is currently commanding.
# Never read by sending raw 0 while in POSITION mode.
active_cmd = {}


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


def command_position(motor_id, raw_cmd):
    pos, vel = bus.write_read_pdo_2(motor_id, raw_cmd, 0.0)
    active_cmd[motor_id] = raw_cmd
    return pos, vel


def read_raw_while_idle(motor_id):
    # Safe only while IDLE. In POSITION mode this would command raw 0.
    pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)
    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")
    return pos


def read_startup_raw_positions_idle():
    print("\nPutting all motors into IDLE before startup read...")
    for motor_id in MOTOR_IDS:
        set_mode_with_spacing(motor_id, recoil.Mode.IDLE)

    time.sleep(0.25)

    # Flush stale frames while IDLE.
    for _ in range(8):
        for motor_id in MOTOR_IDS:
            try:
                bus.write_read_pdo_2(motor_id, 0.0, 0.0)
            except Exception:
                pass
        rate.sleep()

    raw = {}
    for motor_id in MOTOR_IDS:
        samples = []
        for _ in range(15):
            samples.append(read_raw_while_idle(motor_id))
            rate.sleep()
        samples.sort()
        raw[motor_id] = samples[len(samples) // 2]

    print("Startup raw positions measured in IDLE:")
    for motor_id in MOTOR_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]:5s} ID {motor_id}: "
            f"raw={raw[motor_id]:+.6f}, real={raw_to_real(motor_id, raw[motor_id]):+.6f}"
        )

    return raw


def arm_position_mode_holding(start_raw):
    print("\nArming POSITION mode while holding measured startup pose...")

    # Zero authority first so entering POSITION cannot yank to an old target.
    for motor_id in MOTOR_IDS:
        set_gains(motor_id, ARM_KP, ARM_KD, ARM_TORQUE_LIMIT)
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    # Immediately overwrite target with the measured current raw position.
    for _ in range(int(0.50 * RATE_HZ)):
        for motor_id in COMMAND_ORDER:
            command_position(motor_id, start_raw[motor_id])
        rate.sleep()

    print("Ramping to very soft startup hold...")
    for step in range(1, 21):
        a = step / 20.0
        for motor_id in MOTOR_IDS:
            set_gains(
                motor_id,
                STARTUP_KP * a,
                STARTUP_KD * a,
                STARTUP_TORQUE_LIMIT * a,
            )
        for _ in range(2):
            for motor_id in COMMAND_ORDER:
                command_position(motor_id, start_raw[motor_id])
            rate.sleep()

    print("Startup pose is softly held.")


def hold_active_pose(seconds):
    steps = int(seconds * RATE_HZ)
    for _ in range(steps):
        for motor_id in COMMAND_ORDER:
            command_position(motor_id, active_cmd[motor_id])
        rate.sleep()


def idle_all_motors():
    print("\nPutting all motors into IDLE and stopping CAN bus...")
    for motor_id in MOTOR_IDS:
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
# Angle mapping
# ============================================================
def raw_to_real(motor_id, raw_motor_position):
    return (
        raw_motor_position / (MOTOR_SIGN * GEAR_RATIO)
        + HOMING_OFFSET[motor_id]
    )


def real_to_raw(motor_id, desired_joint_angle):
    sign = JOINT_ANGLE_SIGN[motor_id]
    adjusted_angle = sign * desired_joint_angle
    return (
        MOTOR_SIGN
        * (adjusted_angle - HOMING_OFFSET[motor_id])
        * GEAR_RATIO
    )


# ============================================================
# IK target construction
# ============================================================
def xyz_to_raw_targets(x, y, z):
    theta_h, theta_t, theta_s = leg_ik(x, y, z)

    desired_by_role = {
        "hip": theta_h,
        "thigh": theta_t,
        "shank": theta_s,
    }

    desired_real = {}
    raw_targets = {}

    for role, angle in desired_by_role.items():
        motor_id = IK_ROLE_TO_PHYSICAL_ID[role]
        desired_real[motor_id] = angle
        raw_targets[motor_id] = real_to_raw(motor_id, angle)

    return desired_real, raw_targets


def print_target_summary(x, y, z, desired_real, raw_targets):
    print("\nRequested foot position:")
    print(f"  x = {x:.3f} mm")
    print(f"  y = {y:.3f} mm")
    print(f"  z = {z:.3f} mm")

    print("\nIK angles mapped to physical IDs:")
    for role in ["hip", "thigh", "shank"]:
        motor_id = IK_ROLE_TO_PHYSICAL_ID[role]
        print(
            f"  {role:5s} -> ID {motor_id}: "
            f"desired={desired_real[motor_id]:+.6f} rad"
        )

    print("\nRaw motor targets:")
    for motor_id in MOTOR_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]:5s} "
            f"(ID {motor_id}) = {raw_targets[motor_id]:+.6f}"
        )

    print("\nRaw deltas from active command/current hold:")
    max_delta = 0.0
    for motor_id in MOTOR_IDS:
        current = active_cmd.get(motor_id, 0.0)
        delta = raw_targets[motor_id] - current
        max_delta = max(max_delta, abs(delta))
        print(
            f"  {MOTOR_NAMES[motor_id]:5s} "
            f"delta = {delta:+.6f} raw rad"
        )

    return max_delta


# ============================================================
# Smooth move
# ============================================================
def move_all_to_targets(raw_targets, move_time):
    start_raw = dict(active_cmd)
    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))
        measured = {}

        for motor_id in COMMAND_ORDER:
            cmd = start_raw[motor_id] + (raw_targets[motor_id] - start_raw[motor_id]) * s
            pos, vel = command_position(motor_id, cmd)
            measured[motor_id] = {"cmd": cmd, "pos": pos, "vel": vel}

        if (i + 1) % PRINT_EVERY == 0:
            line = []
            for motor_id in COMMAND_ORDER:
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

    # Snap active command exactly to final target.
    for motor_id in MOTOR_IDS:
        active_cmd[motor_id] = raw_targets[motor_id]


def maybe_confirm_large_move(max_delta, label):
    if max_delta > ABORT_IF_RAW_DELTA_OVER:
        print(
            f"\nABORTING {label}: max raw delta {max_delta:.3f} > "
            f"ABORT_IF_RAW_DELTA_OVER {ABORT_IF_RAW_DELTA_OVER:.3f}."
        )
        print("This usually means homing offset, joint sign, or joint mapping is wrong.")
        return False

    if max_delta > CONFIRM_IF_RAW_DELTA_OVER:
        print(
            f"\nWARNING: {label} requires max raw delta {max_delta:.3f}, "
            f"which is large."
        )
        go = input("Move anyway? y/n: ").strip().lower()
        return go in ("y", "yes")

    return True


# ============================================================
# Neutral move
# ============================================================
def move_to_neutral_standing():
    print("\nPreparing neutral standing IK pose...")
    desired_real, raw_targets = xyz_to_raw_targets(NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z)
    max_delta = print_target_summary(NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z, desired_real, raw_targets)

    if not maybe_confirm_large_move(max_delta, "neutral move"):
        print("Neutral move skipped. Holding current pose instead.")
        return

    print("\nSwitching to neutral move gains...")
    set_neutral_gains()
    hold_active_pose(0.2)

    print("Moving to neutral standing pose...")
    move_all_to_targets(raw_targets, MOVE_TO_NEUTRAL_TIME)
    print("Neutral standing move complete.")


# ============================================================
# Parse xyz input
# ============================================================
def parse_xyz(text):
    parts = text.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError("Please enter x y z")
    return float(parts[0]), float(parts[1]), float(parts[2])


# ============================================================
# Main
# ============================================================
try:
    print("=" * 80)
    print("Foot position IK test with manual homing offsets v3 SAFE READ")
    print("=" * 80)
    print("This script moves the foot to one x y z coordinate at a time.")
    print("IMPORTANT: it never uses raw 0 as a read command while in POSITION mode.")
    print()
    print("Loaded role -> ID mapping:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} -> ID {ROLE_TO_ID.get(role)}")
    print()
    print("Loaded HOMING_OFFSET:")
    for motor_id in MOTOR_IDS:
        print(f"  {MOTOR_NAMES[motor_id]:5s} (ID {motor_id}) = {HOMING_OFFSET[motor_id]:+.6f}")
    print()
    print("Joint signs:")
    for motor_id in MOTOR_IDS:
        print(f"  {MOTOR_NAMES[motor_id]:5s} sign = {JOINT_ANGLE_SIGN[motor_id]:+.1f}")
    print("=" * 80)

    startup_raw = read_startup_raw_positions_idle()
    active_cmd.update(startup_raw)

    arm_position_mode_holding(startup_raw)

    if MOVE_TO_NEUTRAL_ON_START:
        move_to_neutral_standing()

    print("\nRamping to normal run gains while holding active pose...")
    set_gains_all(MID_KP, MID_KD, MID_TORQUE_LIMIT)
    hold_active_pose(0.3)
    set_gains_all(RUN_KP, RUN_KD, RUN_TORQUE_LIMIT)
    hold_active_pose(0.2)

    print("\nInput x y z in mm.")
    print("Example neutral: 0 84.26 382")
    print("Type q to quit.\n")

    while True:
        text = input("\nEnter x y z (mm): ").strip()
        if text.lower() in ["q", "quit", "exit"]:
            break

        try:
            x, y, z = parse_xyz(text)
            desired_real, raw_targets = xyz_to_raw_targets(x, y, z)
            max_delta = print_target_summary(x, y, z, desired_real, raw_targets)

            if not maybe_confirm_large_move(max_delta, "requested move"):
                continue

            move_all_to_targets(raw_targets, MOVE_TIME)
            print("Move complete.")

        except Exception as exc:
            print(f"Error: {exc}")

except KeyboardInterrupt:
    print("\nInterrupted.")

finally:
    idle_all_motors()

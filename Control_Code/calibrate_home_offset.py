import time

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ============================================================
# Basic setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

RATE_HZ = 100.0
rate = RateLimiter(frequency=RATE_HZ)

# Home order: CAN ID 0 -> 1 -> 2
SHANK_ID = 1
THIGH_ID = 0
HIP_ID = 2

JOINT_IDS = [SHANK_ID, THIGH_ID, HIP_ID]

JOINT_NAMES = {
    SHANK_ID: "shank",
    THIGH_ID: "thigh",
    HIP_ID: "hip",
}

# Known real joint angles at the homed mechanical-limit pose
# Unit: radians, in IK frame
KNOWN_HOMED_JOINT_ANGLES = {
    HIP_ID: -1.069,
    THIGH_ID: 1.199,
    SHANK_ID: 2.688,
}

# Change these signs if a joint homes in the wrong direction
HOMING_DIRECTION = {
    SHANK_ID: 1.0,
    THIGH_ID: 1.0,
    HIP_ID: 1.0,
}

# ============================================================
# Homing motion settings
# ============================================================
HOMING_STEP_RAD = 0.003
HOMING_TIMEOUT = 8.0

STALL_VELOCITY_THRESHOLD = 0.035
STALL_MIN_TIME = 0.15

USE_TORQUE_TRIGGER = True
TORQUE_ABS_LIMIT = 2.0
REQUIRE_STALL_AND_TORQUE = False

# ============================================================
# Gains
# ============================================================
SYNC_KP = 0.003
SYNC_KD = 0.001
SYNC_TORQUE_LIMIT = 0.03

HOMING_KP = 0.010
HOMING_KD = 0.002
HOMING_TORQUE_LIMIT = 0.18

PRINT_EVERY = 20

OFFSET_OUTPUT_FILE = "homing_offsets.py"


# ============================================================
# Helper functions
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


def read_pos_vel(motor_id):
    return bus.write_read_pdo_2(motor_id, 0.0, 0.0)


def read_pos_torque(motor_id):
    fn = getattr(bus, "write_read_pdo_3", None)

    if not callable(fn):
        return None, None

    try:
        return fn(motor_id, 0.0, 0.0)
    except Exception:
        return None, None


def sync_reference(motor_id, sync_time=0.35):
    print(f"Syncing reference for {JOINT_NAMES[motor_id]}...")

    set_gains(
        motor_id,
        SYNC_KP,
        SYNC_KD,
        SYNC_TORQUE_LIMIT,
    )

    set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    steps = int(sync_time * RATE_HZ)
    current_pos = None

    for _ in range(steps):
        pos, _ = read_pos_vel(motor_id)

        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(motor_id, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(
            f"Failed to sync reference for {JOINT_NAMES[motor_id]}"
        )

    print(f"{JOINT_NAMES[motor_id]} synced at {current_pos:.5f} rad")
    return current_pos


def home_one_joint(motor_id):
    print("=" * 60)
    print(f"Starting homing for {JOINT_NAMES[motor_id]}")

    set_mode_with_spacing(motor_id, recoil.Mode.DAMPING)
    time.sleep(0.2)

    current_pos = sync_reference(motor_id, sync_time=0.35)

    set_gains(
        motor_id,
        HOMING_KP,
        HOMING_KD,
        HOMING_TORQUE_LIMIT,
    )

    set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    direction = HOMING_DIRECTION[motor_id]
    command_pos = current_pos

    stalled_since = None
    t_start = time.perf_counter()
    counter = 0

    while True:
        now = time.perf_counter()

        command_pos += direction * HOMING_STEP_RAD

        measured_pos, measured_vel = bus.write_read_pdo_2(
            motor_id,
            command_pos,
            0.0,
        )

        _, measured_torque = read_pos_torque(motor_id)

        if measured_pos is None or measured_vel is None:
            raise RuntimeError(
                f"Lost telemetry while homing {JOINT_NAMES[motor_id]}"
            )

        is_stalled = abs(measured_vel) < STALL_VELOCITY_THRESHOLD

        if is_stalled:
            if stalled_since is None:
                stalled_since = now
        else:
            stalled_since = None

        long_enough_stall = (
            stalled_since is not None
            and (now - stalled_since) >= STALL_MIN_TIME
        )

        torque_trigger = False
        if USE_TORQUE_TRIGGER and measured_torque is not None:
            if abs(measured_torque) >= TORQUE_ABS_LIMIT:
                torque_trigger = True

        if REQUIRE_STALL_AND_TORQUE:
            hit_limit = long_enough_stall and torque_trigger
        else:
            hit_limit = long_enough_stall or torque_trigger

        counter += 1
        if counter % PRINT_EVERY == 0:
            print(
                f"{JOINT_NAMES[motor_id]} | "
                f"cmd={command_pos:.4f} "
                f"pos={measured_pos:.4f} "
                f"vel={measured_vel:.4f} "
                f"torque={measured_torque}"
            )

        if hit_limit:
            raw_homed_position = measured_pos
            known_joint_angle = KNOWN_HOMED_JOINT_ANGLES[motor_id]
            homing_offset = known_joint_angle - raw_homed_position

            print(
                f"{JOINT_NAMES[motor_id]} homed | "
                f"raw_encoder_pos={raw_homed_position:.5f} rad | "
                f"known_IK_angle={known_joint_angle:.5f} rad | "
                f"homing_offset={homing_offset:.5f} rad"
            )

            set_mode_with_spacing(motor_id, recoil.Mode.DAMPING)
            print(f"{JOINT_NAMES[motor_id]} set to DAMPING.")

            return raw_homed_position, homing_offset

        if now - t_start > HOMING_TIMEOUT:
            set_mode_with_spacing(motor_id, recoil.Mode.DAMPING)
            raise RuntimeError(
                f"Timeout while homing {JOINT_NAMES[motor_id]}. "
                "Try reversing HOMING_DIRECTION or tuning thresholds."
            )

        rate.sleep()


def write_offsets_file(homing_offsets):
    with open(OFFSET_OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write('"""Automatically generated homing offsets.\n')
        file.write("Do not edit unless you know what you are doing.\n")
        file.write('"""\n\n')

        file.write(
            "# real_joint_angle = raw_encoder_position + HOMING_OFFSET[id]\n")
        file.write(
            "# raw_command = desired_real_joint_angle - HOMING_OFFSET[id]\n\n")

        file.write("HOMING_OFFSET = {\n")
        for motor_id in JOINT_IDS:
            file.write(f"    {motor_id}: {homing_offsets[motor_id]:.9f},\n")
        file.write("}\n")

    print(f"\nSaved offsets to: {OFFSET_OUTPUT_FILE}")


def main():
    homed_raw_positions = {}
    homing_offsets = {}

    print("Minimal one-by-one homing offset calibration.")
    print("Homing order: CAN ID 0 -> 1 -> 2")
    print("Press Ctrl+C to stop.\n")

    for motor_id in JOINT_IDS:
        raw_pos, offset = home_one_joint(motor_id)
        homed_raw_positions[motor_id] = raw_pos
        homing_offsets[motor_id] = offset
        time.sleep(0.3)

    print("\nAll joints homed.")
    print("Raw encoder positions at homed mechanical limit:")

    for motor_id in JOINT_IDS:
        print(
            f"  {JOINT_NAMES[motor_id]} "
            f"(CAN ID {motor_id}): "
            f"{homed_raw_positions[motor_id]:.6f} rad"
        )

    print("\nComputed homing offsets:")

    for motor_id in JOINT_IDS:
        print(
            f"  {JOINT_NAMES[motor_id]} "
            f"(CAN ID {motor_id}): "
            f"{homing_offsets[motor_id]:.6f} rad"
        )

    print("\nUse this mapping later:")
    print("  real_joint_angle = raw_encoder_position + HOMING_OFFSET[id]")
    print("  raw_command = desired_real_joint_angle - HOMING_OFFSET[id]")

    write_offsets_file(homing_offsets)

    print("\nAll joints are now in DAMPING mode.")


try:
    main()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    print("Setting all joints to DAMPING and stopping bus...")

    try:
        for joint_id in JOINT_IDS:
            try:
                set_mode_with_spacing(joint_id, recoil.Mode.DAMPING)
            except Exception:
                pass

        time.sleep(0.05)

    finally:
        try:
            bus.stop()
        except Exception:
            pass

import time
import subprocess
from collections import deque
from statistics import median

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

RATE_HZ = 50.0
rate = RateLimiter(frequency=RATE_HZ)

SHANK_ID = 1
THIGH_ID = 0
HIP_ID = 2

JOINT_IDS = [SHANK_ID, THIGH_ID, HIP_ID]

JOINT_NAMES = {
    SHANK_ID: "shank",
    THIGH_ID: "thigh",
    HIP_ID: "hip",
}

GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0

KNOWN_HOMED_JOINT_ANGLES = {
    SHANK_ID: 2.688,
    THIGH_ID: 1.199,
    HIP_ID: 1.069,
}

HOMING_DIRECTION = {
    SHANK_ID: -1.0,
    THIGH_ID: -1.0,
    HIP_ID: 1.0,
}

# Slow output-side homing speed.
HOMING_STEP_OUTPUT_RAD = 0.0008

ESCAPE_OUTPUT_RAD = 0.10
ESCAPE_TIME = 1.0

HOMING_TIMEOUT = 45.0
MAX_COMMANDED_OUTPUT_TRAVEL = 2.4

SYNC_KP = 0.003
SYNC_KD = 0.001
SYNC_TORQUE_LIMIT = 0.03

# Separate homing gains / torque limits
HOMING_KP = {
    SHANK_ID: 0.018,
    THIGH_ID: 0.020,
    HIP_ID: 0.026,
}

HOMING_KD = {
    SHANK_ID: 0.002,
    THIGH_ID: 0.002,
    HIP_ID: 0.003,
}

# Main change:
# hip needs more torque because it lifts/carries the other two motors.
HOMING_TORQUE_LIMIT = {
    SHANK_ID: 0.70,
    THIGH_ID: 0.55,
    HIP_ID: 0.65,
}

ESCAPE_KP = {
    SHANK_ID: 0.012,
    THIGH_ID: 0.012,
    HIP_ID: 0.018,
}

ESCAPE_KD = {
    SHANK_ID: 0.002,
    THIGH_ID: 0.002,
    HIP_ID: 0.003,
}

ESCAPE_TORQUE_LIMIT = {
    SHANK_ID: 0.28,
    THIGH_ID: 0.32,
    HIP_ID: 0.55,
}

FILTER_SIZE = 5

MAX_ACCEPTED_RAW_JUMP = {
    SHANK_ID: 0.90,
    THIGH_ID: 0.90,
    HIP_ID: 0.90,
}

# Make homing detection much less early.
# These are output-side radians.
# 0.70 rad ≈ 40 deg
# 0.85 rad ≈ 49 deg
MIN_COMMANDED_OUTPUT_TRAVEL = {
    SHANK_ID: 0.85,
    THIGH_ID: 0.85,
    HIP_ID: 0.85,
}

# Force it to keep trying for a few seconds before it is even allowed to declare home.
MIN_TIME_BEFORE_STOP_CHECK = {
    SHANK_ID: 4.0,
    THIGH_ID: 4.5,
    HIP_ID: 5.0,
}

# Require command error to be larger before declaring limit.
RAW_ERROR_LIMIT = {
    SHANK_ID: 2.20,
    THIGH_ID: 2.60,
    HIP_ID: 3.00,
}

# Require error to persist longer.
ERROR_ONLY_CONFIRM_TIME = {
    SHANK_ID: 1.30,
    THIGH_ID: 1.50,
    HIP_ID: 1.80,
}

# Require at least some measured movement after sync.
MIN_REAL_RAW_MOVEMENT_BEFORE_HOMING = {
    SHANK_ID: 0.16,
    THIGH_ID: 0.16,
    HIP_ID: 0.20,
}

PRINT_EVERY = 10
OFFSET_OUTPUT_FILE = "homing_offsets.py"

BRING_CAN_DOWN_ON_EXIT = False


def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.010)
    bus.feed(motor_id)
    time.sleep(0.010)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.006)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.006)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.006)


def set_joint_gains(motor_id, kp_dict, kd_dict, torque_dict):
    set_gains(
        motor_id,
        kp_dict[motor_id],
        kd_dict[motor_id],
        torque_dict[motor_id],
    )


def output_to_raw_step(motor_id, output_step):
    return MOTOR_SIGN * HOMING_DIRECTION[motor_id] * output_step * GEAR_RATIO


def read_raw_pos_vel(motor_id):
    return bus.write_read_pdo_2(motor_id, 0.0, 0.0)


def quiet_other_motors(active_motor_id):
    for motor_id in JOINT_IDS:
        if motor_id != active_motor_id:
            try:
                set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
            except Exception:
                pass

    time.sleep(0.20)


def stop_all_motors():
    print("Stopping all motors...")

    for motor_id in JOINT_IDS:
        try:
            set_mode_with_spacing(motor_id, recoil.Mode.DAMPING)
        except Exception:
            pass

    time.sleep(0.15)

    for motor_id in JOINT_IDS:
        try:
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
        except Exception:
            pass

    time.sleep(0.15)


def shutdown_bus():
    try:
        bus.stop()
    except Exception:
        pass

    if BRING_CAN_DOWN_ON_EXIT:
        try:
            subprocess.run(
                ["sudo", "ip", "link", "set", args.channel, "down"],
                check=False,
            )
        except Exception:
            pass


class PositionFilter:
    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.samples = deque(maxlen=FILTER_SIZE)
        self.filtered = None

    def update(self, raw_pos):
        if raw_pos is None:
            return self.filtered, False

        if self.filtered is None:
            self.samples.append(raw_pos)
            self.filtered = median(self.samples)
            return self.filtered, True

        max_jump = MAX_ACCEPTED_RAW_JUMP[self.motor_id]

        if abs(raw_pos - self.filtered) > max_jump:
            return self.filtered, False

        self.samples.append(raw_pos)
        self.filtered = median(self.samples)
        return self.filtered, True


def sync_reference(motor_id, sync_time=0.8):
    name = JOINT_NAMES[motor_id]
    print(f"Syncing reference for {name}...")

    set_gains(motor_id, SYNC_KP, SYNC_KD, SYNC_TORQUE_LIMIT)
    set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    filt = PositionFilter(motor_id)

    # Flush stale CAN reads.
    for _ in range(10):
        read_raw_pos_vel(motor_id)
        rate.sleep()

    for _ in range(int(sync_time * RATE_HZ)):
        raw_pos, raw_vel = read_raw_pos_vel(motor_id)
        filtered_pos, accepted = filt.update(raw_pos)

        if filtered_pos is not None:
            bus.write_read_pdo_2(motor_id, filtered_pos, 0.0)

        rate.sleep()

    if filt.filtered is None:
        raise RuntimeError(f"Failed to sync {name}")

    print(f"{name} synced at filtered_pos={filt.filtered:.6f} rad")
    return filt.filtered


def escape_from_limit(motor_id):
    name = JOINT_NAMES[motor_id]
    print(f"Escaping slightly away from possible limit for {name}...")

    start_pos = sync_reference(motor_id, sync_time=0.5)

    set_joint_gains(
        motor_id,
        ESCAPE_KP,
        ESCAPE_KD,
        ESCAPE_TORQUE_LIMIT,
    )
    set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    escape_raw_total = -output_to_raw_step(motor_id, ESCAPE_OUTPUT_RAD)
    steps = max(1, int(ESCAPE_TIME * RATE_HZ))

    for i in range(steps):
        alpha = (i + 1) / steps
        cmd = start_pos + alpha * escape_raw_total
        bus.write_read_pdo_2(motor_id, cmd, 0.0)
        rate.sleep()

    time.sleep(0.15)


def home_one_joint(motor_id):
    name = JOINT_NAMES[motor_id]

    print("=" * 60)
    print(f"Starting stronger position homing for {name}")
    print(
        f"{name} homing settings: "
        f"kp={HOMING_KP[motor_id]}, "
        f"kd={HOMING_KD[motor_id]}, "
        f"torque_limit={HOMING_TORQUE_LIMIT[motor_id]}, "
        f"min_cmd_out={MIN_COMMANDED_OUTPUT_TRAVEL[motor_id]}"
    )

    quiet_other_motors(motor_id)

    set_mode_with_spacing(motor_id, recoil.Mode.DAMPING)
    time.sleep(0.25)

    escape_from_limit(motor_id)

    start_pos = sync_reference(motor_id, sync_time=0.7)

    set_joint_gains(
        motor_id,
        HOMING_KP,
        HOMING_KD,
        HOMING_TORQUE_LIMIT,
    )
    set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    filt = PositionFilter(motor_id)
    filt.update(start_pos)

    command_pos = start_pos
    raw_step = output_to_raw_step(motor_id, HOMING_STEP_OUTPUT_RAD)

    t_start = time.perf_counter()
    error_started_at = None

    counter = 0
    accepted_count = 0
    rejected_count = 0

    max_real_raw_movement = 0.0

    while True:
        now = time.perf_counter()
        elapsed = now - t_start

        command_pos += raw_step

        raw_pos, raw_vel = bus.write_read_pdo_2(
            motor_id,
            command_pos,
            0.0,
        )

        filtered_pos, accepted = filt.update(raw_pos)

        if filtered_pos is None:
            raise RuntimeError(f"Lost usable position telemetry for {name}")

        if accepted:
            accepted_count += 1
        else:
            rejected_count += 1

        commanded_output_travel = abs(command_pos - start_pos) / GEAR_RATIO
        raw_error = abs(command_pos - filtered_pos)
        real_raw_movement = abs(filtered_pos - start_pos)
        max_real_raw_movement = max(max_real_raw_movement, real_raw_movement)

        allow_stop_check = (
            elapsed > MIN_TIME_BEFORE_STOP_CHECK[motor_id]
            and commanded_output_travel > MIN_COMMANDED_OUTPUT_TRAVEL[motor_id]
            and max_real_raw_movement > MIN_REAL_RAW_MOVEMENT_BEFORE_HOMING[motor_id]
        )

        error_too_large = (
            allow_stop_check
            and raw_error > RAW_ERROR_LIMIT[motor_id]
        )

        if error_too_large:
            if error_started_at is None:
                error_started_at = now
        else:
            error_started_at = None

        hit_limit = (
            error_started_at is not None
            and now - error_started_at > ERROR_ONLY_CONFIRM_TIME[motor_id]
        )

        counter += 1

        if counter % PRINT_EVERY == 0:
            print(
                f"{name} | "
                f"cmd={command_pos:.3f} "
                f"raw_pos={raw_pos:.3f} "
                f"filt_pos={filtered_pos:.3f} "
                f"cmd_out={commanded_output_travel:.3f} "
                f"real_raw_move={max_real_raw_movement:.3f} "
                f"err={raw_error:.3f} "
                f"allow_stop={allow_stop_check} "
                f"accepted={accepted_count} "
                f"rejected={rejected_count}"
            )

        if hit_limit:
            raw_homed_pos = filtered_pos
            known_angle = KNOWN_HOMED_JOINT_ANGLES[motor_id]
            homing_offset = known_angle - raw_homed_pos

            print(
                f"{name} homed by delayed error detection | "
                f"filtered_encoder_pos={raw_homed_pos:.6f} | "
                f"known_IK_angle={known_angle:.6f} | "
                f"homing_offset={homing_offset:.6f}"
            )

            set_mode_with_spacing(motor_id, recoil.Mode.DAMPING)
            time.sleep(0.2)

            return raw_homed_pos, homing_offset

        if commanded_output_travel > MAX_COMMANDED_OUTPUT_TRAVEL:
            set_mode_with_spacing(motor_id, recoil.Mode.DAMPING)
            raise RuntimeError(
                f"Safety stop for {name}: commanded output travel exceeded "
                f"{MAX_COMMANDED_OUTPUT_TRAVEL:.2f} rad"
            )

        if elapsed > HOMING_TIMEOUT:
            set_mode_with_spacing(motor_id, recoil.Mode.DAMPING)
            raise RuntimeError(
                f"Homing timeout for {name}. "
                f"Last cmd_out={commanded_output_travel:.3f}, "
                f"last err={raw_error:.3f}, "
                f"real_raw_move={max_real_raw_movement:.3f}, "
                f"accepted={accepted_count}, rejected={rejected_count}. "
                f"If it physically hit the limit, lower RAW_ERROR_LIMIT or "
                f"MIN_COMMANDED_OUTPUT_TRAVEL for this joint. "
                f"If it never reached the limit, increase HOMING_TORQUE_LIMIT."
            )

        rate.sleep()


def write_offsets_file(offsets):
    with open(OFFSET_OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write('"""Auto-generated homing offsets."""\n\n')
        file.write("# real_joint_angle = raw_encoder_position + HOMING_OFFSET[id]\n")
        file.write("# raw_command = desired_real_joint_angle - HOMING_OFFSET[id]\n\n")
        file.write("HOMING_OFFSET = {\n")

        for motor_id in JOINT_IDS:
            file.write(f"    {motor_id}: {offsets[motor_id]:.9f},\n")

        file.write("}\n")

    print(f"\nSaved offsets to {OFFSET_OUTPUT_FILE}")


def main():
    raw_positions = {}
    offsets = {}

    print("Stronger position-based homing offset calibration.")
    print("Homing order: shank -> thigh -> hip")
    print("CAN IDs: shank=1, thigh=0, hip=2")
    print("Separate homing torque limits enabled.")
    print("Early stopping made harder.")
    print()

    for motor_id in JOINT_IDS:
        raw_pos, offset = home_one_joint(motor_id)
        raw_positions[motor_id] = raw_pos
        offsets[motor_id] = offset
        time.sleep(0.5)

    print("\nAll joints homed.\n")

    print("Raw / filtered encoder positions:")
    for motor_id in JOINT_IDS:
        print(
            f"  {JOINT_NAMES[motor_id]} "
            f"(CAN ID {motor_id}): "
            f"{raw_positions[motor_id]:.6f}"
        )

    print("\nComputed homing offsets:")
    for motor_id in JOINT_IDS:
        print(
            f"  {JOINT_NAMES[motor_id]} "
            f"(CAN ID {motor_id}): "
            f"{offsets[motor_id]:.6f}"
        )

    print("\nUse later:")
    print("real_joint_angle = raw_encoder + HOMING_OFFSET[id]")

    write_offsets_file(offsets)

    print("\nAll joints now stopped.")


try:
    main()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    stop_all_motors()
    shutdown_bus()

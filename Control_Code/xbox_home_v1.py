import time
import math
import signal
import sys
import select
from statistics import median

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from xbox_controller import XboxController


# ============================================================
# xbox_home_v1.py
# Semi-manual Xbox position-mode homing calibration
# ============================================================
#
# Purpose:
#   Use Xbox joystick to move ONE selected joint slowly and smoothly toward
#   its mechanical homing limit. When the joint reaches the visual/mechanical
#   limit, release joystick so it holds current position, then type y + Enter
#   to capture that joint's raw position and compute HOMING_OFFSET.
#
# Why this exists:
#   Fully manual backdriving can be jerky. This script uses position mode with
#   a slow target velocity so the joint can approach the limit smoothly.
#
# Important:
#   - This script does NOT recalibrate electrical offset.
#   - This script avoids CAN ID 0 by default.
#   - This script uses transmit_pdo_2() for command only.
#   - This script uses read_position_measured() for measured position.
#   - It does NOT use write_read_pdo_2() for encoder readings.
#
# Workflow:
#   1. Reflash IDs so no joint uses CAN ID 0.
#      Default mapping here:
#          shank -> ID 3
#          thigh -> ID 2
#          hip   -> ID 1
#   2. Run this script.
#   3. It reads startup raw positions and enters POSITION mode softly.
#   4. Type a joint ID, e.g. 3, then press Enter.
#   5. Use Xbox left stick Y to move that joint slowly.
#   6. Release joystick at the mechanical limit. The joint will hold.
#   7. Type y + Enter to capture this joint's homing offset.
#   8. Repeat for all joints.
#   9. Type write + Enter to save homing_offsets.py.
#
# Offset convention:
#   real_joint_angle = raw_motor_position / (MOTOR_SIGN * GEAR_RATIO) + HOMING_OFFSET[id]
#
# Therefore at the known homing pose:
#   HOMING_OFFSET[id] = known_homed_angle - raw_homed / (MOTOR_SIGN * GEAR_RATIO)
#
# Raw command from desired real joint angle:
#   raw_command = MOTOR_SIGN * (desired_joint_angle - HOMING_OFFSET[id]) * GEAR_RATIO
#
# ============================================================


# ============================================================
# Current assembled-leg IDs, recommended nonzero IDs
# ============================================================
SHANK_ID = 3
THIGH_ID = 2
HIP_ID = 1

JOINT_IDS = [SHANK_ID, THIGH_ID, HIP_ID]

JOINT_NAMES = {
    SHANK_ID: "shank",
    THIGH_ID: "thigh",
    HIP_ID: "hip",
}

ROLE_TO_ID = {
    "shank": SHANK_ID,
    "thigh": THIGH_ID,
    "hip": HIP_ID,
}

ID_TO_ROLE = {v: k for k, v in ROLE_TO_ID.items()}


# ============================================================
# Known ideal joint angles at mechanical homing pose
# ============================================================
# Update these if your mechanical reference pose changes.
KNOWN_HOMED_JOINT_ANGLES_BY_ROLE = {
    "shank": 2.688,
    "thigh": 1.199,
    "hip": 1.069,
}


# ============================================================
# Gear / sign
# ============================================================
GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0


# ============================================================
# Output file
# ============================================================
OFFSET_OUTPUT_FILE = "homing_offsets.py"


# ============================================================
# Timing and input settings
# ============================================================
RATE_HZ = 80.0
PRINT_EVERY = 12
JOYSTICK_DEADBAND = 0.06
JOYSTICK_FILTER_ALPHA = 0.35

# Raw motor rad/s target velocity at full joystick deflection.
# Keep this small for homing so hitting a limit is not violent.
MAX_RAW_SPEED = 0.55

# Acceleration limit in raw motor rad/s^2 for target velocity.
RAW_ACCEL_LIMIT = 1.50

# Safety: do not let the software target drift too far from the selected
# joint's raw position at the beginning of that joint-homing session.
MAX_RAW_DELTA_FROM_SESSION_START = 45.0

# When joystick is released, periodically refresh hold target to measured raw
# position so the motor does not keep pushing into the mechanical stop.
DEADBAND_MEASURE_REFRESH_INTERVAL = 0.10

# Final capture settings.
CAPTURE_SAMPLE_COUNT = 80
CAPTURE_SAMPLE_HZ = 50.0
CAPTURE_SPREAD_WARNING = 0.10
CAPTURE_SPREAD_HARD_REJECT = 0.35


# ============================================================
# Gains
# ============================================================
# Arming with zero torque helps avoid jumping toward an old firmware target.
ARM_KP = 0.0
ARM_KD = 0.0
ARM_TORQUE_LIMIT = 0.0

# Soft hold while preparing.
STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

# Moving gains for selected joint. Keep gentle but enough to move.
MOVE_KP_BY_ROLE = {
    "shank": 0.030,
    "thigh": 0.030,
    "hip": 0.020,
}

MOVE_KD_BY_ROLE = {
    "shank": 0.002,
    "thigh": 0.002,
    "hip": 0.004,
}

MOVE_TORQUE_LIMIT_BY_ROLE = {
    "shank": 0.28,
    "thigh": 0.32,
    "hip": 0.85,
}

# Holding gains for non-selected joints and after joystick release.
HOLD_KP_BY_ROLE = {
    "shank": 0.035,
    "thigh": 0.035,
    "hip": 0.025,
}

HOLD_KD_BY_ROLE = {
    "shank": 0.002,
    "thigh": 0.002,
    "hip": 0.005,
}

HOLD_TORQUE_LIMIT_BY_ROLE = {
    "shank": 0.30,
    "thigh": 0.35,
    "hip": 0.95,
}


# ============================================================
# Setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)

STOP_REQUESTED = False
SHUTDOWN_STARTED = False


def request_stop(signum=None, frame=None):
    global STOP_REQUESTED
    STOP_REQUESTED = True
    print("\nStop requested. Releasing motors to IDLE...")


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


# ============================================================
# Math / conversion helpers
# ============================================================
def apply_deadband(value, deadband):
    if abs(value) < deadband:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadband) / (1.0 - deadband)


def limit_rate(current, target, max_delta):
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


def raw_to_output_angle(raw_motor_position):
    return raw_motor_position / (MOTOR_SIGN * GEAR_RATIO)


def compute_offset_for_joint(motor_id, raw_homed):
    role = ID_TO_ROLE[motor_id]
    known_angle = KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]
    return known_angle - raw_to_output_angle(raw_homed)


# ============================================================
# CAN / recoil helpers
# ============================================================
def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.010)
    try:
        bus.feed(motor_id)
    except Exception:
        pass
    time.sleep(0.010)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_role_gains(motor_id, gain_type):
    role = ID_TO_ROLE[motor_id]
    if gain_type == "move":
        set_gains(
            motor_id,
            MOVE_KP_BY_ROLE[role],
            MOVE_KD_BY_ROLE[role],
            MOVE_TORQUE_LIMIT_BY_ROLE[role],
        )
    elif gain_type == "hold":
        set_gains(
            motor_id,
            HOLD_KP_BY_ROLE[role],
            HOLD_KD_BY_ROLE[role],
            HOLD_TORQUE_LIMIT_BY_ROLE[role],
        )
    elif gain_type == "startup":
        set_gains(motor_id, STARTUP_KP, STARTUP_KD, STARTUP_TORQUE_LIMIT)
    elif gain_type == "arm":
        set_gains(motor_id, ARM_KP, ARM_KD, ARM_TORQUE_LIMIT)
    else:
        raise ValueError(f"Unknown gain_type: {gain_type}")


def read_position_measured(motor_id):
    value = bus.read_position_measured(motor_id)
    if value is None:
        raise RuntimeError(f"read_position_measured returned None for ID {motor_id}")
    return value


def command_position_only(motor_id, raw_target):
    # Command only. Do not use write_read_pdo_2() here.
    bus.transmit_pdo_2(motor_id, raw_target, 0.0)


def command_all_targets(active_targets):
    # Use a consistent order. Hip last often helps the leg feel more stable.
    command_order = [THIGH_ID, SHANK_ID, HIP_ID]
    for motor_id in command_order:
        command_position_only(motor_id, active_targets[motor_id])


def idle_all_motors():
    global SHUTDOWN_STARTED
    if SHUTDOWN_STARTED:
        return
    SHUTDOWN_STARTED = True

    print("\nPutting all motors into IDLE...")
    for motor_id in JOINT_IDS:
        try:
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
            print(f"  {JOINT_NAMES[motor_id]} IDLE")
        except Exception as exc:
            print(f"  Failed to idle {JOINT_NAMES[motor_id]}: {exc}")

    time.sleep(0.10)
    try:
        bus.stop()
    except Exception:
        pass


def read_all_positions_once():
    return {motor_id: read_position_measured(motor_id) for motor_id in JOINT_IDS}


def initialize_position_hold():
    print("\nReading startup measured positions...")
    startup_raw = read_all_positions_once()
    for motor_id in JOINT_IDS:
        print(f"  {JOINT_NAMES[motor_id]:5s} ID {motor_id}: raw={startup_raw[motor_id]:+.6f}")

    print("\nArming POSITION mode at zero torque...")
    for motor_id in JOINT_IDS:
        set_role_gains(motor_id, "arm")
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    # Immediately command measured current positions at zero torque.
    for _ in range(int(0.35 * RATE_HZ)):
        command_all_targets(startup_raw)
        rate.sleep()

    print("Applying startup hold gains while holding current pose...")
    for motor_id in JOINT_IDS:
        set_role_gains(motor_id, "startup")

    for _ in range(int(0.5 * RATE_HZ)):
        command_all_targets(startup_raw)
        rate.sleep()

    print("Applying normal hold gains...")
    for motor_id in JOINT_IDS:
        set_role_gains(motor_id, "hold")

    for _ in range(int(0.3 * RATE_HZ)):
        command_all_targets(startup_raw)
        rate.sleep()

    return startup_raw.copy()


# ============================================================
# Capture / file writer
# ============================================================
def capture_joint_position(motor_id, active_targets):
    role = ID_TO_ROLE[motor_id]
    print(f"\nCapturing {role} ID {motor_id} measured position...")
    print("  Hold the joint still. Sampling measured position...")

    # Freeze all targets to current measured positions before sampling.
    try:
        measured_now = read_all_positions_once()
        for jid in JOINT_IDS:
            active_targets[jid] = measured_now[jid]
    except Exception as exc:
        print(f"  Warning: could not refresh all measured positions before capture: {exc}")

    capture_rate = RateLimiter(frequency=CAPTURE_SAMPLE_HZ)
    samples = []

    for _ in range(CAPTURE_SAMPLE_COUNT):
        if STOP_REQUESTED:
            break
        command_all_targets(active_targets)
        try:
            samples.append(read_position_measured(motor_id))
        except Exception as exc:
            print(f"  Read warning: {exc}")
        capture_rate.sleep()

    if len(samples) < max(10, CAPTURE_SAMPLE_COUNT // 3):
        print("  ERROR: Not enough valid samples. Capture failed.")
        return None

    samples.sort()
    raw_median = median(samples)
    spread = max(samples) - min(samples)
    offset = compute_offset_for_joint(motor_id, raw_median)
    raw_out = raw_to_output_angle(raw_median)
    known = KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]

    print(f"  {role} raw median = {raw_median:+.9f}")
    print(f"  {role} raw spread = {spread:.9f}")
    print(f"  known homed angle = {known:+.9f}")
    print(f"  raw output angle  = {raw_out:+.9f}")
    print(f"  HOMING_OFFSET    = {offset:+.9f}")

    if spread > CAPTURE_SPREAD_HARD_REJECT:
        print("  BAD: capture spread is above hard reject threshold.")
        answer = input("  Type override to accept this bad capture, or Enter to reject: ").strip().lower()
        if answer != "override":
            print("  Capture rejected.")
            return None
    elif spread > CAPTURE_SPREAD_WARNING:
        print("  WARNING: capture spread is higher than preferred. Consider recapturing.")

    return {
        "role": role,
        "motor_id": motor_id,
        "raw_homed": raw_median,
        "spread": spread,
        "offset": offset,
        "known_angle": known,
    }


def write_offsets_file(captured):
    missing = [role for role in ["shank", "thigh", "hip"] if role not in captured]
    if missing:
        print("\nCannot write homing_offsets.py yet. Missing captures:")
        for role in missing:
            print(f"  {role}")
        return False

    print(f"\nThis will write/overwrite: {OFFSET_OUTPUT_FILE}")
    confirm = input("Type y/yes to write the file: ").strip().lower()
    if confirm not in ("y", "yes"):
        print("Cancelled. homing_offsets.py was not written.")
        return False

    with open(OFFSET_OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write('"""Auto-generated homing offsets.\n\n')
        file.write("Generated by xbox_home_v1.py.\n")
        file.write("Each joint was moved to mechanical homing limit using Xbox position-mode control.\n")
        file.write('"""\n\n')

        file.write(f"GEAR_RATIO = {GEAR_RATIO:.9f}\n")
        file.write(f"MOTOR_SIGN = {MOTOR_SIGN:.9f}\n\n")

        file.write("# Offset convention:\n")
        file.write("# real_joint_angle = raw_motor_position / (MOTOR_SIGN * GEAR_RATIO) + HOMING_OFFSET[id]\n")
        file.write("# raw_command = MOTOR_SIGN * (desired_joint_angle - HOMING_OFFSET[id]) * GEAR_RATIO\n\n")

        file.write("ROLE_TO_ID = {\n")
        for role in ["shank", "thigh", "hip"]:
            file.write(f'    "{role}": {ROLE_TO_ID[role]},\n')
        file.write("}\n\n")

        file.write("KNOWN_HOMED_JOINT_ANGLES_BY_ROLE = {\n")
        for role in ["shank", "thigh", "hip"]:
            file.write(f'    "{role}": {KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:.9f},\n')
        file.write("}\n\n")

        file.write("RAW_HOMED_POSITION = {\n")
        for role in ["shank", "thigh", "hip"]:
            motor_id = ROLE_TO_ID[role]
            file.write(f"    {motor_id}: {captured[role]['raw_homed']:.9f},\n")
        file.write("}\n\n")

        file.write("RAW_READ_SPREAD = {\n")
        for role in ["shank", "thigh", "hip"]:
            motor_id = ROLE_TO_ID[role]
            file.write(f"    {motor_id}: {captured[role]['spread']:.9f},\n")
        file.write("}\n\n")

        file.write("HOMING_OFFSET = {\n")
        for role in ["shank", "thigh", "hip"]:
            motor_id = ROLE_TO_ID[role]
            file.write(f"    {motor_id}: {captured[role]['offset']:.9f},\n")
        file.write("}\n")

    print(f"Saved offsets to {OFFSET_OUTPUT_FILE}")
    return True


def print_captured_status(captured):
    print("\nCaptured homing offsets:")
    for role in ["shank", "thigh", "hip"]:
        if role in captured:
            c = captured[role]
            print(
                f"  {role:5s} ID {c['motor_id']}: "
                f"raw={c['raw_homed']:+.6f}, "
                f"spread={c['spread']:.6f}, "
                f"offset={c['offset']:+.6f}"
            )
        else:
            print(f"  {role:5s}: not captured")


# ============================================================
# Interactive joint homing
# ============================================================
def read_terminal_line_nonblocking():
    ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    if ready:
        return sys.stdin.readline().strip()
    return None


def run_joint_control_loop(selected_id, controller, active_targets, captured):
    role = ID_TO_ROLE[selected_id]
    print("\n" + "=" * 80)
    print(f"Selected {role} ID {selected_id} for Xbox homing")
    print("=" * 80)
    print("Left stick Y controls selected joint raw target velocity.")
    print("Release joystick to hold current position.")
    print("Type y + Enter when this joint is at its mechanical homing limit.")
    print("Type c + Enter to cancel this joint without saving.")
    print("Type q + Enter to quit program.")
    print("=" * 80)

    # Selected joint uses moving gains. Other joints hold.
    for jid in JOINT_IDS:
        set_role_gains(jid, "move" if jid == selected_id else "hold")

    # Refresh selected target to measured position at loop start.
    try:
        active_targets[selected_id] = read_position_measured(selected_id)
    except Exception as exc:
        print(f"Warning: could not refresh selected joint target: {exc}")

    session_start_target = active_targets[selected_id]
    filtered_y = 0.0
    target_raw_speed = 0.0
    current_raw_speed = 0.0
    counter = 0
    last_time = time.time()
    last_deadband_measure_refresh = 0.0

    while not STOP_REQUESTED:
        now = time.time()
        dt = now - last_time
        last_time = now
        if dt <= 0.0 or dt > 0.1:
            dt = 1.0 / RATE_HZ

        line = read_terminal_line_nonblocking()
        if line is not None:
            cmd = line.strip().lower()
            if cmd == "y":
                # Release/hold before capturing.
                try:
                    measured = read_position_measured(selected_id)
                    active_targets[selected_id] = measured
                except Exception:
                    pass
                result = capture_joint_position(selected_id, active_targets)
                if result is not None:
                    captured[role] = result
                    print_captured_status(captured)
                return "captured"
            if cmd in ("c", "cancel"):
                print(f"Cancelled {role} capture.")
                return "cancelled"
            if cmd in ("q", "quit", "exit"):
                return "quit"
            if cmd:
                print("Unknown command during joint control. Use y, c, or q.")

        state = controller.read()
        raw_y = max(-1.0, min(1.0, state.left_y))
        y = apply_deadband(raw_y, JOYSTICK_DEADBAND)
        filtered_y = (1.0 - JOYSTICK_FILTER_ALPHA) * filtered_y + JOYSTICK_FILTER_ALPHA * y

        target_raw_speed = MAX_RAW_SPEED * filtered_y
        max_speed_step = RAW_ACCEL_LIMIT * dt
        current_raw_speed = limit_rate(current_raw_speed, target_raw_speed, max_speed_step)

        if abs(filtered_y) > 1e-4:
            active_targets[selected_id] += current_raw_speed * dt

            delta_from_session_start = active_targets[selected_id] - session_start_target
            if abs(delta_from_session_start) > MAX_RAW_DELTA_FROM_SESSION_START:
                print("\nSAFETY STOP: selected joint target moved too far from session start.")
                print(f"  delta = {delta_from_session_start:+.3f} raw rad")
                print("  Holding current measured position. Capture cancelled.")
                try:
                    active_targets[selected_id] = read_position_measured(selected_id)
                except Exception:
                    pass
                return "safety_stop"
        else:
            # Joystick released: refresh target to measured position occasionally to avoid
            # pushing into a mechanical stop with old integrated target.
            current_raw_speed = 0.0
            if now - last_deadband_measure_refresh > DEADBAND_MEASURE_REFRESH_INTERVAL:
                try:
                    active_targets[selected_id] = read_position_measured(selected_id)
                except Exception:
                    pass
                last_deadband_measure_refresh = now

        command_all_targets(active_targets)
        counter += 1

        if counter % PRINT_EVERY == 0:
            try:
                measured = read_position_measured(selected_id)
            except Exception:
                measured = None

            if measured is None:
                print(
                    f"{role} ID {selected_id} | "
                    f"joy={raw_y:+.2f}/{filtered_y:+.2f} "
                    f"target={active_targets[selected_id]:+.3f} "
                    f"speed={current_raw_speed:+.3f} | measured read failed"
                )
            else:
                err = active_targets[selected_id] - measured
                print(
                    f"{role} ID {selected_id} | "
                    f"joy={raw_y:+.2f}/{filtered_y:+.2f} "
                    f"target={active_targets[selected_id]:+.3f} "
                    f"meas={measured:+.3f} "
                    f"err={err:+.3f} "
                    f"speed={current_raw_speed:+.3f}"
                )

        rate.sleep()

    return "quit"


# ============================================================
# Main
# ============================================================
def main():
    print("=" * 80)
    print("Xbox semi-manual homing calibration v1")
    print("=" * 80)
    print("This script moves one selected joint at a slow Xbox-controlled velocity.")
    print("It captures homing offset when you type y at the mechanical limit.")
    print("It does NOT recalibrate electrical offset.")
    print("It avoids CAN ID 0 by default.")
    print("It uses transmit_pdo_2() for command and read_position_measured() for reads.")
    print()
    print("Current role -> CAN ID mapping:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} -> ID {ROLE_TO_ID[role]}")
    print()
    print("Known mechanical homing angles:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} = {KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:+.6f} rad")
    print("=" * 80)

    controller = None
    captured = {}

    try:
        controller = XboxController(deadzone=JOYSTICK_DEADBAND)
        active_targets = initialize_position_hold()

        while not STOP_REQUESTED:
            print_captured_status(captured)
            print("\nCommands:")
            print("  Type joint CAN ID to control: 1, 2, or 3")
            print("  Type role name: hip, thigh, shank")
            print("  Type write to save homing_offsets.py after all captures")
            print("  Type q to quit")
            text = input("\nSelect joint / command: ").strip().lower()

            if text in ("q", "quit", "exit"):
                break

            if text == "write":
                write_offsets_file(captured)
                continue

            selected_id = None
            if text in ROLE_TO_ID:
                selected_id = ROLE_TO_ID[text]
            else:
                try:
                    maybe_id = int(text)
                    if maybe_id in JOINT_IDS:
                        selected_id = maybe_id
                except ValueError:
                    pass

            if selected_id is None:
                print("Invalid selection. Use 1, 2, 3, hip, thigh, shank, write, or q.")
                continue

            result = run_joint_control_loop(selected_id, controller, active_targets, captured)
            if result == "quit":
                break

        print("\nExiting xbox_home_v1.")

    finally:
        idle_all_motors()
        try:
            if controller is not None:
                controller.close()
        except Exception as exc:
            print(f"Controller close warning: {exc}")


try:
    main()
except KeyboardInterrupt:
    request_stop()
    idle_all_motors()

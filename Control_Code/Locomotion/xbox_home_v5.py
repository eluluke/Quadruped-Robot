"""
xbox_home_v5_clean_ui_fast.py

Semi-manual Xbox homing with:
  - selected-joint-only CAN commands
  - one terminal input thread shared by menu and joint loop
  - faster raw speed defaults
  - clearer capture/write/quit workflow

Why v5 exists
-------------
In v4, the background input thread and main menu input() could compete for
the same terminal line. That made commands like "write" or "y" feel confusing.

This v5 uses ONE input system:
  - background thread reads all terminal lines
  - main menu reads from the same queue
  - joint control loop reads from the same queue

No competing input() calls.

Workflow
--------
1. Run:
       python3 xbox_home_v5_clean_ui_fast.py -c can1

2. At MENU, type:
       shank / thigh / hip
   or:
       3 / 2 / 1

3. Move selected joint with Xbox left stick Y.
   Only selected joint receives position commands.

4. At mechanical limit, release joystick and type:
       y
   This captures the selected joint and returns to menu.

5. Repeat for all three joints.

6. At MENU, type:
       write
   Then type:
       y
   to save homing_offsets.py.

7. Type:
       q
   to quit.
"""

import time
import signal
import sys
import threading
import queue
from statistics import median

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from xbox_controller import XboxController


# ============================================================
# Motor IDs - avoid CAN ID 0
# ============================================================

HIP_ID = 1
THIGH_ID = 2
SHANK_ID = 3

JOINT_IDS = [SHANK_ID, THIGH_ID, HIP_ID]

JOINT_NAMES = {
    HIP_ID: "hip",
    THIGH_ID: "thigh",
    SHANK_ID: "shank",
}

ROLE_TO_ID = {
    "hip": HIP_ID,
    "thigh": THIGH_ID,
    "shank": SHANK_ID,
}

ID_TO_ROLE = {motor_id: role for role, motor_id in ROLE_TO_ID.items()}


# ============================================================
# Known ideal joint angles at mechanical homing pose
# ============================================================

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
# Timing and Xbox tuning
# ============================================================

RATE_HZ = 80.0
PRINT_EVERY = 12

JOYSTICK_DEADBAND = 0.06
JOYSTICK_FILTER_ALPHA = 0.35

# IMPORTANT:
# This is raw MOTOR rad/s, not output joint rad/s.
# Output speed ~= MAX_RAW_SPEED / GEAR_RATIO.
# Example: 4.0 raw rad/s / 17 = 0.235 output rad/s ~= 13.5 deg/s.
MAX_RAW_SPEED = 4.0

# Raw motor rad/s^2 acceleration limit.
RAW_ACCEL_LIMIT = 12.0

# Safety limit for one selected-joint session.
MAX_RAW_DELTA_FROM_SESSION_START = 65.0

# When joystick is released, refresh target to measured raw position so the
# motor does not keep pushing into a mechanical stop.
DEADBAND_MEASURE_REFRESH_INTERVAL = 0.10


# ============================================================
# Capture settings
# ============================================================

CAPTURE_SAMPLE_COUNT = 80
CAPTURE_SAMPLE_HZ = 50.0
CAPTURE_SPREAD_WARNING = 0.10
CAPTURE_SPREAD_HARD_REJECT = 0.35


# ============================================================
# Gains
# ============================================================

ARM_KP = 0.0
ARM_KD = 0.0
ARM_TORQUE_LIMIT = 0.0

MOVE_KP_BY_ROLE = {
    "shank": 0.15,
    "thigh": 0.15,
    "hip": 0.15,
}

MOVE_KD_BY_ROLE = {
    "shank": 0.005,
    "thigh": 0.005,
    "hip": 0.005,
}

MOVE_TORQUE_LIMIT_BY_ROLE = {
    "shank": 0.45,
    "thigh": 0.50,
    "hip": 1.00,
}

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

TERMINAL_QUEUE = queue.Queue()


def terminal_input_worker():
    """Background thread that reads all terminal lines."""
    while True:
        try:
            line = sys.stdin.readline()
            if line == "":
                return
            TERMINAL_QUEUE.put(line.strip())
        except Exception:
            return


def start_terminal_input_thread():
    """Start exactly one terminal reader thread."""
    thread = threading.Thread(target=terminal_input_worker, daemon=True)
    thread.start()
    return thread


def get_terminal_command_nonblocking():
    """Return one queued terminal command or None."""
    try:
        return TERMINAL_QUEUE.get_nowait()
    except queue.Empty:
        return None


def wait_for_terminal_command(prompt):
    """Blocking wait for one terminal command from the shared queue."""
    print(prompt, end="", flush=True)

    while not STOP_REQUESTED:
        cmd = get_terminal_command_nonblocking()
        if cmd is not None:
            print(cmd)
            return cmd.strip().lower()
        time.sleep(0.02)

    return "q"


def clear_terminal_queue():
    """Discard stale terminal commands before entering a new mode."""
    while True:
        try:
            TERMINAL_QUEUE.get_nowait()
        except queue.Empty:
            return


def request_stop(_signum=None, _frame=None):
    """Signal handler."""
    global STOP_REQUESTED
    STOP_REQUESTED = True
    print("\nStop requested. Releasing motors to IDLE...")


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


# ============================================================
# Math helpers
# ============================================================

def apply_deadband(value, deadband):
    """Apply joystick deadband."""
    if abs(value) < deadband:
        return 0.0

    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadband) / (1.0 - deadband)


def limit_rate(current, target, max_delta):
    """Limit rate of change."""
    delta = target - current

    if delta > max_delta:
        return current + max_delta

    if delta < -max_delta:
        return current - max_delta

    return target


def raw_to_output_angle(raw_motor_position):
    """Convert raw motor position to output-side joint angle."""
    return raw_motor_position / (MOTOR_SIGN * GEAR_RATIO)


def compute_offset_for_joint(motor_id, raw_homed):
    """Compute HOMING_OFFSET for one joint."""
    role = ID_TO_ROLE[motor_id]
    known_angle = KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]
    return known_angle - raw_to_output_angle(raw_homed)


# ============================================================
# CAN helpers
# ============================================================

def set_mode_with_spacing(motor_id, mode):
    """Set mode with small delay."""
    bus.set_mode(motor_id, mode)
    time.sleep(0.010)
    try:
        bus.feed(motor_id)
    except Exception:
        pass
    time.sleep(0.010)


def set_gains(motor_id, kp, kd, torque_limit):
    """Set position gains and torque limit."""
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_role_gains(motor_id, gain_type):
    """Set gains for one motor by role."""
    role = ID_TO_ROLE[motor_id]

    if gain_type == "arm":
        set_gains(motor_id, ARM_KP, ARM_KD, ARM_TORQUE_LIMIT)
    elif gain_type == "move":
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
    else:
        raise ValueError(f"Unknown gain_type: {gain_type}")


def read_position_measured(motor_id):
    """Read measured raw position."""
    value = bus.read_position_measured(motor_id)
    if value is None:
        raise RuntimeError(f"read_position_measured returned None for ID {motor_id}")
    return float(value)


def command_position_only(motor_id, raw_target):
    """Command selected motor only."""
    bus.transmit_pdo_2(motor_id, raw_target, 0.0)


def idle_one_motor(motor_id):
    """Put one motor into IDLE."""
    try:
        set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
        print(f"  {JOINT_NAMES[motor_id]} IDLE")
    except Exception as exc:
        print(f"  Failed to idle {JOINT_NAMES[motor_id]}: {exc}")


def idle_all_motors():
    """Safety shutdown: idle all motors."""
    global SHUTDOWN_STARTED

    if SHUTDOWN_STARTED:
        return

    SHUTDOWN_STARTED = True

    print("\nPutting all motors into IDLE for shutdown...")
    for motor_id in JOINT_IDS:
        idle_one_motor(motor_id)

    time.sleep(0.10)
    try:
        bus.stop()
    except Exception:
        pass


# ============================================================
# Selected-joint movement and capture
# ============================================================

def arm_selected_joint_only(motor_id):
    """Enter POSITION mode only for selected joint."""
    role = ID_TO_ROLE[motor_id]

    print(f"\nPreparing selected joint only: {role} ID {motor_id}")

    measured = read_position_measured(motor_id)
    print(f"  current measured raw = {measured:+.6f}")

    print("  entering POSITION mode with zero torque...")
    set_role_gains(motor_id, "arm")
    set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    for _ in range(int(0.20 * RATE_HZ)):
        command_position_only(motor_id, measured)
        rate.sleep()

    print("  applying selected-joint moving gains...")
    set_role_gains(motor_id, "move")

    for _ in range(int(0.15 * RATE_HZ)):
        command_position_only(motor_id, measured)
        rate.sleep()

    return measured


def hold_selected_joint(motor_id, raw_target, seconds=0.25):
    """Hold one selected joint for a short time."""
    for _ in range(int(seconds * RATE_HZ)):
        if STOP_REQUESTED:
            break
        command_position_only(motor_id, raw_target)
        rate.sleep()


def capture_joint_position(motor_id, raw_hold_target):
    """Capture selected joint and compute homing offset."""
    role = ID_TO_ROLE[motor_id]

    print("\n" + "-" * 80)
    print(f"Capturing {role} ID {motor_id}")
    print("Release joystick and hold the joint still...")
    print("-" * 80)

    try:
        raw_hold_target = read_position_measured(motor_id)
    except Exception as exc:
        print(f"  Warning: could not refresh measured position before capture: {exc}")

    capture_rate = RateLimiter(frequency=CAPTURE_SAMPLE_HZ)
    samples = []

    for _ in range(CAPTURE_SAMPLE_COUNT):
        if STOP_REQUESTED:
            break

        command_position_only(motor_id, raw_hold_target)

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
        answer = wait_for_terminal_command("  Type override to accept, or Enter to reject: ")
        if answer != "override":
            print("  Capture rejected.")
            return None
    elif spread > CAPTURE_SPREAD_WARNING:
        print("  WARNING: capture spread is higher than preferred. Consider recapturing.")

    print("  switching selected joint to hold gains at captured position...")
    set_role_gains(motor_id, "hold")
    hold_selected_joint(motor_id, raw_median, seconds=0.35)

    return {
        "role": role,
        "motor_id": motor_id,
        "raw_homed": raw_median,
        "spread": spread,
        "offset": offset,
        "known_angle": known,
    }


# ============================================================
# File writer / status
# ============================================================

def write_offsets_file(captured):
    """Write homing_offsets.py after all captures."""
    missing = [role for role in ["shank", "thigh", "hip"] if role not in captured]

    if missing:
        print("\nCannot write homing_offsets.py yet. Missing captures:")
        for role in missing:
            print(f"  {role}")
        return False

    print("\nReady to write homing_offsets.py with:")
    for role in ["shank", "thigh", "hip"]:
        item = captured[role]
        print(
            f"  {role:5s} ID {item['motor_id']}: "
            f"raw={item['raw_homed']:+.6f}, "
            f"offset={item['offset']:+.6f}, "
            f"spread={item['spread']:.6f}"
        )

    confirm = wait_for_terminal_command("\nWrite homing_offsets.py? Type y/yes: ")

    if confirm not in ("y", "yes"):
        print("Cancelled. homing_offsets.py was not written.")
        return False

    with open(OFFSET_OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write('"""Auto-generated homing offsets.\n\n')
        file.write("Generated by xbox_home_v5_clean_ui_fast.py.\n")
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

    print(f"\nSaved offsets to {OFFSET_OUTPUT_FILE}")
    return True


def print_captured_status(captured):
    """Print captured status."""
    print("\nCaptured homing offsets:")
    for role in ["shank", "thigh", "hip"]:
        if role in captured:
            item = captured[role]
            print(
                f"  {role:5s} ID {item['motor_id']}: "
                f"raw={item['raw_homed']:+.6f}, "
                f"offset={item['offset']:+.6f}, "
                f"spread={item['spread']:.6f}"
            )
        else:
            print(f"  {role:5s}: not captured")


# ============================================================
# Joint control loop
# ============================================================

def run_joint_control_loop(selected_id, controller, captured):
    """Move one selected joint with Xbox."""
    role = ID_TO_ROLE[selected_id]

    print("\n" + "=" * 80)
    print(f"Selected {role} ID {selected_id}")
    print("=" * 80)
    print("Only this selected joint receives CAN position commands.")
    print("Left stick Y controls raw target velocity.")
    print("At mechanical limit: release stick, then type y + Enter.")
    print("To cancel this joint: type c + Enter.")
    print("To quit program: type q + Enter.")
    print("=" * 80)

    clear_terminal_queue()

    raw_target = arm_selected_joint_only(selected_id)
    session_start_target = raw_target

    filtered_y = 0.0
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

        cmd = get_terminal_command_nonblocking()
        if cmd is not None:
            cmd = cmd.strip().lower()
            if cmd:
                print(f"\nCommand received: {cmd!r}")

            if cmd == "y":
                try:
                    raw_target = read_position_measured(selected_id)
                except Exception:
                    pass

                result = capture_joint_position(selected_id, raw_target)

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
                print("Unknown command. Use y, c, or q while moving a joint.")

        state = controller.read()
        raw_y = max(-1.0, min(1.0, float(state.left_y)))

        y = apply_deadband(raw_y, JOYSTICK_DEADBAND)
        filtered_y = (
            (1.0 - JOYSTICK_FILTER_ALPHA) * filtered_y
            + JOYSTICK_FILTER_ALPHA * y
        )

        target_raw_speed = MAX_RAW_SPEED * filtered_y
        max_speed_step = RAW_ACCEL_LIMIT * dt
        current_raw_speed = limit_rate(current_raw_speed, target_raw_speed, max_speed_step)

        if abs(filtered_y) > 1e-4:
            raw_target += current_raw_speed * dt

            delta_from_session_start = raw_target - session_start_target
            if abs(delta_from_session_start) > MAX_RAW_DELTA_FROM_SESSION_START:
                print("\nSAFETY STOP: selected joint target moved too far from session start.")
                print(f"  delta = {delta_from_session_start:+.3f} raw rad")
                print("  Capture cancelled.")

                try:
                    raw_target = read_position_measured(selected_id)
                    hold_selected_joint(selected_id, raw_target, seconds=0.25)
                except Exception:
                    pass

                return "safety_stop"
        else:
            current_raw_speed = 0.0

            if now - last_deadband_measure_refresh > DEADBAND_MEASURE_REFRESH_INTERVAL:
                try:
                    raw_target = read_position_measured(selected_id)
                except Exception:
                    pass
                last_deadband_measure_refresh = now

        command_position_only(selected_id, raw_target)

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
                    f"target={raw_target:+.3f} "
                    f"speed={current_raw_speed:+.3f} | read failed"
                )
            else:
                err = raw_target - measured
                print(
                    f"{role} ID {selected_id} | "
                    f"joy={raw_y:+.2f}/{filtered_y:+.2f} "
                    f"target={raw_target:+.3f} "
                    f"meas={measured:+.3f} "
                    f"err={err:+.3f} "
                    f"speed={current_raw_speed:+.3f}"
                )

        rate.sleep()

    return "quit"


# ============================================================
# Main
# ============================================================

def parse_joint_selection(text):
    """Parse role name or CAN ID."""
    text = text.strip().lower()

    if text in ROLE_TO_ID:
        return ROLE_TO_ID[text]

    try:
        maybe_id = int(text)
    except ValueError:
        return None

    if maybe_id in JOINT_IDS:
        return maybe_id

    return None


def main():
    """Main program."""
    print("=" * 80)
    print("Xbox semi-manual homing calibration v5 - clean UI / faster speed")
    print("=" * 80)
    print("Only the selected joint receives position commands.")
    print("Other joints are not held/spammed while one joint is selected.")
    print("Command path: transmit_pdo_2(); read path: read_position_measured().")
    print()
    print("Current role -> CAN ID mapping:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} -> ID {ROLE_TO_ID[role]}")
    print()
    print("Known mechanical homing angles:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} = {KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:+.6f} rad")
    print()
    print(f"Speed: MAX_RAW_SPEED={MAX_RAW_SPEED}, RAW_ACCEL_LIMIT={RAW_ACCEL_LIMIT}")
    print("=" * 80)

    controller = None
    captured = {}

    try:
        controller = XboxController(deadzone=JOYSTICK_DEADBAND)
        start_terminal_input_thread()

        while not STOP_REQUESTED:
            print_captured_status(captured)

            print("\nMENU")
            print("  Select joint: 1/2/3 or hip/thigh/shank")
            print("  Save file:    write")
            print("  Quit:         q")

            text = wait_for_terminal_command("\nCommand: ")

            if text in ("q", "quit", "exit"):
                break

            if text == "write":
                write_offsets_file(captured)
                continue

            selected_id = parse_joint_selection(text)

            if selected_id is None:
                print("Invalid command. Use 1, 2, 3, hip, thigh, shank, write, or q.")
                continue

            result = run_joint_control_loop(selected_id, controller, captured)

            if result == "quit":
                break

        print("\nExiting xbox_home_v5_clean_ui_fast.")

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

import time
import math
import signal
import sys

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik
from xbox_controller import XboxController


# ============================================================
# Remote-controlled v8 trajectory, frame-mapping version
#
# Motion core is based on your working run_tra_v8.py:
#   - SHANK_ID = 2
#   - THIGH_ID = 0
#   - HIP_ID   = 1
#   - Hip holds measured startup position
#   - Only thigh and shank receive IK trajectory commands
#   - Hip hold is refreshed last
#
# Remote-control layer:
#   - Uses Eli's XboxController helper
#   - Left stick Y controls gait phase speed
#   - Push forward = positive phase speed
#   - Pull backward = negative phase speed
# ============================================================


# ============================================================
# Motor IDs
# ============================================================
SHANK_ID = 2
THIGH_ID = 0
HIP_ID = 1

DRIVE_IDS = [THIGH_ID, SHANK_ID]
ALL_IDS = [HIP_ID, THIGH_ID, SHANK_ID]

MOTOR_NAMES = {
    THIGH_ID: "thigh",
    SHANK_ID: "shank",
    HIP_ID: "hip",
}


# ============================================================
# Xbox controller tuning
# ============================================================
JOYSTICK_DEADBAND = 0.05

# Higher = joystick feels more direct.
# 1.0 = no filtering, 0.0 = ignores new input.
JOYSTICK_FILTER_ALPHA = 0.65

# Full stick gives this many gait cycles per second.
# 0.50 = 2.0 s/cycle, 0.85 = 1.18 s/cycle, 1.0 = 1 s/cycle.
MAX_PHASE_SPEED = 0.95

# How quickly phase speed can change, in cycles/s^2.
# Higher = more responsive; lower = smoother.
PHASE_ACCEL_LIMIT = 5.0

ALLOW_REVERSE = True
FREEZE_PHASE_WHEN_STOPPED = True


# ============================================================
# Gear / sign
# ============================================================
GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0


# ============================================================
# Trajectory tuning
# ============================================================
RATE_HZ = 80.0

X_CENTER = 0.0
Y_PLANE = 84.26
Z_GROUND = 382.0

STEP_LENGTH = 120.0
STEP_HEIGHT = 100.0
STANCE_RATIO = 0.50

MAX_RAW_DELTA_FROM_START = 13.0


# ============================================================
# Trajectory frame mapping
# ============================================================
# The remote controller advances an abstract gait trajectory:
#   forward = fore/aft stance + swing motion
#   lift    = foot lift during swing
#
# Then this section maps that abstract forward/lift path into IK x/z.
#
# Normal old-style mapping:
#   FORWARD_AXIS = "x"
#   LIFT_AXIS    = "z"
#
# Swapped mapping for suspected 90-degree rotated physical response:
#   FORWARD_AXIS = "z"
#   LIFT_AXIS    = "x"
#
# Use this to fix the "D shape / rotated trajectory" issue without changing
# CAN IDs or IK code.
FORWARD_AXIS = "z"   # "x" or "z"
LIFT_AXIS = "x"      # "x" or "z"

FORWARD_SIGN = -1.0
LIFT_SIGN = 1.0


# ============================================================
# Hip hold tuning
# ============================================================
HIP_STARTUP_KP = 0.003
HIP_STARTUP_KD = 0.001
HIP_STARTUP_TORQUE_LIMIT = 0.03

HIP_RUN_KP = 0.02
HIP_RUN_KD = 0.005
HIP_TORQUE_LIMIT = 1.50

# If hip still makes tiny corrections, try True.
# If loop becomes late / vibration increases, keep False.
DOUBLE_REFRESH_HIP = False


# ============================================================
# Thigh + shank control tuning
# ============================================================
STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP = 0.025
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.12

RUN_KP = 0.055
RUN_KD = 0.003
RUN_TORQUE_LIMIT = 0.26

STARTUP_HOLD_TIME = 1.2
MOVE_TO_START_TIME = 2.5

PRINT_EVERY = 80


# ============================================================
# Optional command smoothing
# ============================================================
# Keep this conservative. If it feels delayed, set False.
ENABLE_RAW_COMMAND_RATE_LIMIT = False

MAX_RAW_STEP_PER_LOOP = {
    HIP_ID: 0.08,
    THIGH_ID: 0.20,
    SHANK_ID: 0.24,
}




# ============================================================
# Shutdown / Ctrl+C handling
# ============================================================
STOP_REQUESTED = False
SHUTDOWN_STARTED = False


def request_stop(signum=None, frame=None):
    """
    Signal handler for Ctrl+C / SIGTERM.

    Do not do heavy CAN work inside the signal handler.
    Just set a flag. The main loop will safely idle motors.
    """
    global STOP_REQUESTED

    if not STOP_REQUESTED:
        print("\nStop requested. Exiting control loop...")
    else:
        print("\nStop already requested. Finishing shutdown...")

    STOP_REQUESTED = True


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def should_stop():
    return STOP_REQUESTED


# ============================================================
# Setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)


# ============================================================
# Math helpers
# ============================================================
def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def raw_delta_from_angle_delta(angle_delta):
    return MOTOR_SIGN * angle_delta * GEAR_RATIO


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


def limit_raw_step(prev, desired, max_step):
    delta = desired - prev

    if delta > max_step:
        return prev + max_step

    if delta < -max_step:
        return prev - max_step

    return desired


# ============================================================
# Low-level helpers
# ============================================================
def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.008)
    bus.feed(motor_id)
    time.sleep(0.008)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)

    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)

    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def write_position_command(motor_id, command_pos):
    return bus.write_read_pdo_2(
        motor_id,
        command_pos,
        0.0,
    )


def write_drive_position_command(motor_id, command_pos):
    """
    Only thigh and shank are allowed to receive trajectory commands.
    Hip is handled separately by write_hip_hold_command().
    """
    if motor_id == HIP_ID:
        raise RuntimeError("BUG: attempted to command HIP_ID as drive joint")

    return write_position_command(
        motor_id,
        command_pos,
    )


def write_hip_hold_command(hip_hold_raw):
    return write_position_command(
        HIP_ID,
        hip_hold_raw,
    )


def set_drive_gains(kp, kd, torque_limit):
    for motor_id in DRIVE_IDS:
        if should_stop():
            return
        set_gains(
            motor_id,
            kp,
            kd,
            torque_limit,
        )


def idle_all_motors():
    """
    Fast shutdown helper.

    Important:
    - Send IDLE to all motors.
    - Avoid bus.stop() because if the CAN layer is stuck, bus.stop() can delay
      returning to the terminal.
    """
    global SHUTDOWN_STARTED

    if SHUTDOWN_STARTED:
        return

    SHUTDOWN_STARTED = True

    print("\nPutting all motors into IDLE...")

    for motor_id in ALL_IDS:
        try:
            bus.set_mode(motor_id, recoil.Mode.IDLE)
            time.sleep(0.015)
            print(f"  {MOTOR_NAMES[motor_id]} IDLE")
        except Exception as e:
            print(f"  Failed to idle {MOTOR_NAMES[motor_id]}: {e}")

    # Give firmware a short moment to receive mode changes.
    time.sleep(0.08)

    print("Motors sent to IDLE. Returning to terminal.")


# ============================================================
# Safe initial read
# ============================================================
def read_position_while_idle(motor_id):
    """
    Only use this while the motor is IDLE.
    """
    pos, vel = bus.write_read_pdo_2(
        motor_id,
        0.0,
        0.0,
    )

    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")

    return pos


def read_initial_positions_safely():
    """
    Put hip, thigh, and shank into IDLE and read all three.
    Hip is read only so we can hold its current position.
    """
    print("Putting hip, thigh, and shank into IDLE before initial read...")

    for motor_id in ALL_IDS:
        bus.set_mode(motor_id, recoil.Mode.IDLE)
        time.sleep(0.02)

    time.sleep(0.20)

    # Flush stale frames for all three.
    for _ in range(5):
        for motor_id in ALL_IDS:
            try:
                bus.write_read_pdo_2(motor_id, 0.0, 0.0)
            except Exception:
                pass
        rate.sleep()

    raw = {}

    for motor_id in ALL_IDS:
        samples = []

        for _ in range(10):
            pos = read_position_while_idle(motor_id)
            samples.append(pos)
            rate.sleep()

        samples.sort()
        raw[motor_id] = samples[len(samples) // 2]

    print("Initial raw encoder positions:")
    for motor_id in ALL_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]} = "
            f"{raw[motor_id]:.6f}"
        )

    return raw


# ============================================================
# Hip setup
# ============================================================
def setup_hip_position_hold(hip_hold_raw):
    print("\nEntering POSITION mode for hip hold...")

    set_gains(
        HIP_ID,
        HIP_STARTUP_KP,
        HIP_STARTUP_KD,
        HIP_STARTUP_TORQUE_LIMIT,
    )

    set_mode_with_spacing(
        HIP_ID,
        recoil.Mode.POSITION,
    )

    # Immediately command current raw position.
    for _ in range(8):
        if should_stop():
            return
        write_hip_hold_command(
            hip_hold_raw,
        )
        rate.sleep()

    print(f"Hip holding raw position: {hip_hold_raw:.6f}")


def ramp_hip_to_run_gains():
    print("Switching hip to run hold gains...")

    set_gains(
        HIP_ID,
        HIP_RUN_KP,
        HIP_RUN_KD,
        HIP_TORQUE_LIMIT,
    )


# ============================================================
# Drive startup: thigh + shank only
# ============================================================
def setup_drive_position_mode_and_hold(start_raw, hip_hold_raw):
    print("\nEntering POSITION mode for thigh and shank...")

    for motor_id in DRIVE_IDS:
        if should_stop():
            return
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

        for _ in range(5):
            write_drive_position_command(
                motor_id,
                start_raw[motor_id],
            )

            # Refresh hip after moving-joint command.
            write_hip_hold_command(
                hip_hold_raw,
            )

            rate.sleep()

    print("Soft holding hip/thigh/shank initial positions...")

    for _ in range(int(STARTUP_HOLD_TIME * RATE_HZ)):
        if should_stop():
            return
        for motor_id in DRIVE_IDS:
            write_drive_position_command(
                motor_id,
                start_raw[motor_id],
            )

        # Hip last.
        write_hip_hold_command(
            hip_hold_raw,
        )

        rate.sleep()

    print("Startup hold complete.")


# ============================================================
# Foot trajectory with frame mapping
# ============================================================
def abstract_trajectory(phase):
    """
    Returns an abstract gait point:
        forward: fore/aft coordinate
        lift: positive lift amount
    """
    phase = phase % 1.0

    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO

        # Straight stance section.
        # forward goes +L/2 to -L/2.
        forward = STEP_LENGTH / 2.0 - STEP_LENGTH * u
        lift = 0.0

        return forward, lift, "stance"

    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    # Cycloid swing return.
    # forward goes -L/2 to +L/2 smoothly.
    forward = -STEP_LENGTH / 2.0 + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    lift = STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    return forward, lift, "swing"


def map_forward_lift_to_ik(forward, lift):
    """
    Map abstract forward/lift into the IK x/z frame.

    This is the only place to change if the physical trajectory appears
    rotated or swapped.
    """
    x = X_CENTER
    z = Z_GROUND

    if FORWARD_AXIS == "x":
        x += FORWARD_SIGN * forward
    elif FORWARD_AXIS == "z":
        z += FORWARD_SIGN * forward
    else:
        raise RuntimeError("FORWARD_AXIS must be 'x' or 'z'")

    if LIFT_AXIS == "x":
        x += LIFT_SIGN * lift
    elif LIFT_AXIS == "z":
        z += LIFT_SIGN * lift
    else:
        raise RuntimeError("LIFT_AXIS must be 'x' or 'z'")

    return x, Y_PLANE, z


def foot_trajectory(phase):
    forward, lift, phase_name = abstract_trajectory(phase)
    x, y, z = map_forward_lift_to_ik(
        forward,
        lift,
    )

    return x, y, z, forward, lift, phase_name

def build_targets_for_phase(phase, start_raw, theta_t0, theta_s0):
    x, y, z, forward, lift, phase_name = foot_trajectory(phase)

    _, theta_t, theta_s = leg_ik(
        x,
        y,
        z,
    )

    delta_t = theta_t - theta_t0
    delta_s = theta_s - theta_s0

    raw_thigh = start_raw[THIGH_ID] + raw_delta_from_angle_delta(delta_t)
    raw_shank = start_raw[SHANK_ID] + raw_delta_from_angle_delta(delta_s)

    thigh_delta_raw = raw_thigh - start_raw[THIGH_ID]
    shank_delta_raw = raw_shank - start_raw[SHANK_ID]

    if abs(thigh_delta_raw) > MAX_RAW_DELTA_FROM_START:
        raise RuntimeError(
            f"Thigh command too large: {thigh_delta_raw:.3f} raw rad. "
            f"Reduce STEP_LENGTH or STEP_HEIGHT."
        )

    if abs(shank_delta_raw) > MAX_RAW_DELTA_FROM_START:
        raise RuntimeError(
            f"Shank command too large: {shank_delta_raw:.3f} raw rad. "
            f"Reduce STEP_LENGTH or STEP_HEIGHT."
        )

    return {
        "phase": phase,
        "phase_name": phase_name,
        "forward": forward,
        "lift": lift,
        "x": x,
        "y": y,
        "z": z,
        "delta_t": delta_t,
        "delta_s": delta_s,
        "raw_thigh": raw_thigh,
        "raw_shank": raw_shank,
    }


# ============================================================
# Smooth move to first point
# ============================================================
def smooth_move_to_targets(start_raw, target_raw, move_time, hip_hold_raw):
    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        if should_stop():
            return
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        for motor_id in DRIVE_IDS:
            cmd = start_raw[motor_id] + (
                target_raw[motor_id] - start_raw[motor_id]
            ) * s

            write_drive_position_command(
                motor_id,
                cmd,
            )

        # Hip last.
        write_hip_hold_command(
            hip_hold_raw,
        )

        rate.sleep()


# ============================================================
# Main
# ============================================================
controller = None

try:
    print("=" * 80)
    print("Remote-controlled v8 trajectory with hip hold, v6 frame mapping")
    print("=" * 80)
    print("Current IDs:")
    print(f"  shank={SHANK_ID}, thigh={THIGH_ID}, hip={HIP_ID}")
    print()
    print("Remote control:")
    print(f"  MAX_PHASE_SPEED       = {MAX_PHASE_SPEED} cycles/s")
    print(f"  PHASE_ACCEL_LIMIT     = {PHASE_ACCEL_LIMIT} cycles/s^2")
    print(f"  JOYSTICK_DEADBAND     = {JOYSTICK_DEADBAND}")
    print(f"  JOYSTICK_FILTER_ALPHA = {JOYSTICK_FILTER_ALPHA}")
    print()
    print("Trajectory:")
    print(f"  STEP_LENGTH      = {STEP_LENGTH} mm")
    print(f"  STEP_HEIGHT      = {STEP_HEIGHT} mm")
    print(f"  STANCE_RATIO     = {STANCE_RATIO}")
    print()
    print("Frame mapping:")
    print(f"  FORWARD_AXIS = {FORWARD_AXIS}")
    print(f"  LIFT_AXIS    = {LIFT_AXIS}")
    print(f"  FORWARD_SIGN = {FORWARD_SIGN}")
    print(f"  LIFT_SIGN    = {LIFT_SIGN}")
    print()
    print(f"  DOUBLE_REFRESH_HIP = {DOUBLE_REFRESH_HIP}")
    print("=" * 80)

    controller = XboxController(deadzone=JOYSTICK_DEADBAND)

    # Step 1: Read hip/thigh/shank initial positions.
    start_raw = read_initial_positions_safely()
    hip_hold_raw = start_raw[HIP_ID]

    # Step 2: Put hip into position hold.
    setup_hip_position_hold(
        hip_hold_raw,
    )

    # Step 3: Set up thigh/shank position control.
    setup_drive_position_mode_and_hold(
        start_raw,
        hip_hold_raw,
    )

    # Step 4: IK reference for thigh/shank only.
    _, theta_t0, theta_s0 = leg_ik(
        X_CENTER,
        Y_PLANE,
        Z_GROUND,
    )

    print("\nNominal IK reference:")
    print(f"  theta_t0 = {theta_t0:.6f}")
    print(f"  theta_s0 = {theta_s0:.6f}")
    print("  hip output from IK is discarded; hip holds current raw position")

    # Start at phase 0.0 and move to first point.
    phase = 0.0

    first = build_targets_for_phase(
        phase,
        start_raw,
        theta_t0,
        theta_s0,
    )

    target_raw = {
        THIGH_ID: first["raw_thigh"],
        SHANK_ID: first["raw_shank"],
    }

    print("\nRamping thigh/shank to medium gains...")
    set_drive_gains(
        MID_KP,
        MID_KD,
        MID_TORQUE_LIMIT,
    )

    print("Moving thigh/shank to first trajectory point...")
    smooth_move_to_targets(
        start_raw,
        target_raw,
        MOVE_TO_START_TIME,
        hip_hold_raw,
    )

    print("Switching thigh/shank to run gains...")
    set_drive_gains(
        RUN_KP,
        RUN_KD,
        RUN_TORQUE_LIMIT,
    )

    ramp_hip_to_run_gains()

    print("\nStarting remote-controlled v8 trajectory.")
    print("Left stick Y controls gait phase speed.")
    print("Ctrl+C to stop safely.\n")

    ly_filtered = 0.0
    phase_speed = 0.0
    last_time = time.time()

    prev_cmd = {
        THIGH_ID: target_raw[THIGH_ID],
        SHANK_ID: target_raw[SHANK_ID],
        HIP_ID: hip_hold_raw,
    }

    counter = 0

    while not should_stop():
        now = time.time()
        dt = now - last_time
        last_time = now

        if dt <= 0.0 or dt > 0.1:
            dt = 1.0 / RATE_HZ

        state = controller.read()

        # Eli's helper already applies deadzone and makes left_y positive upward.
        raw_ly = max(-1.0, min(1.0, state.left_y))

        ly_filtered = (
            (1.0 - JOYSTICK_FILTER_ALPHA) * ly_filtered
            + JOYSTICK_FILTER_ALPHA * raw_ly
        )

        if not ALLOW_REVERSE and ly_filtered < 0.0:
            ly_filtered = 0.0

        target_phase_speed = MAX_PHASE_SPEED * ly_filtered

        max_phase_step = PHASE_ACCEL_LIMIT * dt
        phase_speed = limit_rate(
            phase_speed,
            target_phase_speed,
            max_phase_step,
        )

        if abs(phase_speed) > 1e-5:
            phase = (phase + phase_speed * dt) % 1.0
        elif not FREEZE_PHASE_WHEN_STOPPED:
            phase = phase % 1.0

        point = build_targets_for_phase(
            phase,
            start_raw,
            theta_t0,
            theta_s0,
        )

        cmd_thigh = point["raw_thigh"]
        cmd_shank = point["raw_shank"]
        cmd_hip = hip_hold_raw

        if ENABLE_RAW_COMMAND_RATE_LIMIT:
            cmd_thigh = limit_raw_step(
                prev_cmd[THIGH_ID],
                cmd_thigh,
                MAX_RAW_STEP_PER_LOOP[THIGH_ID],
            )

            cmd_shank = limit_raw_step(
                prev_cmd[SHANK_ID],
                cmd_shank,
                MAX_RAW_STEP_PER_LOOP[SHANK_ID],
            )

            cmd_hip = limit_raw_step(
                prev_cmd[HIP_ID],
                cmd_hip,
                MAX_RAW_STEP_PER_LOOP[HIP_ID],
            )

        prev_cmd[THIGH_ID] = cmd_thigh
        prev_cmd[SHANK_ID] = cmd_shank
        prev_cmd[HIP_ID] = cmd_hip

        if DOUBLE_REFRESH_HIP:
            hip_pos, hip_vel = write_hip_hold_command(
                cmd_hip,
            )

        thigh_pos, thigh_vel = write_drive_position_command(
            THIGH_ID,
            cmd_thigh,
        )

        shank_pos, shank_vel = write_drive_position_command(
            SHANK_ID,
            cmd_shank,
        )

        # Hip last.
        hip_pos, hip_vel = write_hip_hold_command(
            cmd_hip,
        )

        counter += 1

        if counter % PRINT_EVERY == 0:
            hip_err = wrap_pi(cmd_hip - hip_pos) if hip_pos is not None else None
            thigh_err = wrap_pi(cmd_thigh - thigh_pos) if thigh_pos is not None else None
            shank_err = wrap_pi(cmd_shank - shank_pos) if shank_pos is not None else None

            print(
                f"LY_raw={raw_ly:+.2f} "
                f"LY={ly_filtered:+.2f} "
                f"phase_speed={phase_speed:+.3f} "
                f"phase={phase:.3f} {point['phase_name']} | "
                f"forward={point['forward']:+.1f} "
                f"lift={point['lift']:+.1f} | "
                f"x={point['x']:.1f} "
                f"z={point['z']:.1f} | "
                f"dT={point['delta_t']:+.4f} "
                f"dS={point['delta_s']:+.4f} | "
                f"raw_t={cmd_thigh:.3f} "
                f"raw_s={cmd_shank:.3f} | "
                f"hip_hold={cmd_hip:.3f} "
                f"hip_pos={hip_pos:.3f} "
                f"hip_err={hip_err:.3f} | "
                f"t_err={thigh_err:.3f} "
                f"s_err={shank_err:.3f}"
            )

        rate.sleep()

except KeyboardInterrupt:
    request_stop()

finally:
    # Motors first, controller cleanup second. This prioritizes safety.
    idle_all_motors()

    try:
        if controller is not None:
            controller.close()
    except Exception as e:
        print(f"Controller close warning: {e}")

    print("Shutdown complete.")

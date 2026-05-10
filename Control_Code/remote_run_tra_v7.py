import time
import math
import signal

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik
from xbox_controller import XboxController


# ============================================================
# Remote-controlled trajectory, smooth-current-start version
#
# Based on your remote_run_tra_v6.py features:
#   - Xbox left stick controls gait phase speed
#   - Frame mapping for your working rotated/swapped trajectory
#   - Hip holds measured startup raw position
#   - Only thigh/shank receive IK trajectory deltas
#   - Hip hold command is refreshed last
#   - Ctrl+C / SIGTERM safe shutdown
#
# Main startup change:
#   - The leg does NOT move to phase 0 at startup.
#   - The script reads the current raw encoder position in IDLE.
#   - It enters POSITION mode with zero torque first.
#   - It immediately overwrites position targets with the measured raw positions.
#   - It ramps gains gradually while repeatedly holding the measured startup pose.
#   - The gait phase starts at the neutral stance point where forward = 0, lift = 0.
# ============================================================


# ============================================================
# Motor IDs
# Keep your current working flipped setup.
# ============================================================
SHANK_ID = 3
THIGH_ID = 2
HIP_ID = 1

# For normal naming / target storage.
DRIVE_IDS = [THIGH_ID, SHANK_ID]
ALL_IDS = [HIP_ID, THIGH_ID, SHANK_ID]

# Command order that helped reduce hip motion:
# drive joints first, hip hold last.
# If you want the previous order, change this to [THIGH_ID, SHANK_ID].
DRIVE_COMMAND_ORDER = [SHANK_ID, THIGH_ID]
POSITION_MODE_ENTRY_ORDER = [SHANK_ID, THIGH_ID, HIP_ID]
HOLD_COMMAND_ORDER = [SHANK_ID, THIGH_ID, HIP_ID]

MOTOR_NAMES = {
    THIGH_ID: "thigh",
    SHANK_ID: "shank",
    HIP_ID: "hip",
}


# ============================================================
# Xbox controller tuning
# ============================================================
JOYSTICK_DEADBAND = 0.05
JOYSTICK_FILTER_ALPHA = 0.65
MAX_PHASE_SPEED = 0.75
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

# Start the gait at the neutral stance point:
# during stance, forward = STEP_LENGTH/2 - STEP_LENGTH*u.
# forward = 0 when u = 0.5, so phase = STANCE_RATIO * 0.5.
NEUTRAL_PHASE = 0.5 * STANCE_RATIO


# ============================================================
# Trajectory frame mapping
# This preserves your currently working flipped/rotated behavior.
# ============================================================
FORWARD_AXIS = "x"   # "x" or "z"
LIFT_AXIS = "z"      # "x" or "z"

FORWARD_SIGN = -1.0
LIFT_SIGN = 1.0


# ============================================================
# Smooth startup / arming tuning
# ============================================================
# First enter POSITION with zero authority so the firmware cannot yank the leg
# toward an old remembered target.
ARM_KP = 0.0
ARM_KD = 0.0
ARM_TORQUE_LIMIT = 0.0

# Very weak hold after targets have been overwritten with current raw positions.
STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

# Intermediate hold before full run gains.
MID_KP = 0.025
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.12

# Running gains for thigh + shank.
RUN_KP = 0.055
RUN_KD = 0.003
RUN_TORQUE_LIMIT = 0.26

# Hip running hold gains.
HIP_RUN_KP = 0.02
HIP_RUN_KD = 0.005
HIP_TORQUE_LIMIT = 1.50

# Number of command cycles used while targets are overwritten at zero torque.
ARM_HOLD_CYCLES = 30

# Ramp timing. During every ramp step, all motors are commanded to their
# measured startup raw positions.
STARTUP_SOFT_HOLD_TIME = 1.0
RAMP_TO_MID_TIME = 1.2
RAMP_TO_RUN_TIME = 1.8

# Keep False unless hip still makes small corrections.
DOUBLE_REFRESH_HIP = False

PRINT_EVERY = 80


# ============================================================
# Optional command smoothing during trajectory motion
# ============================================================
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


def lerp(a, b, u):
    return a + (b - a) * u


# ============================================================
# Low-level helpers
# ============================================================
def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.010)
    bus.feed(motor_id)
    time.sleep(0.010)


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


def hold_startup_pose_once(start_raw):
    """Command all three joints to their measured startup raw positions."""
    last_feedback = {}

    for motor_id in HOLD_COMMAND_ORDER:
        if motor_id == HIP_ID:
            pos, vel = write_hip_hold_command(start_raw[HIP_ID])
        else:
            pos, vel = write_drive_position_command(motor_id, start_raw[motor_id])

        last_feedback[motor_id] = (pos, vel)

    return last_feedback


def set_all_gains(kp_drive, kd_drive, torque_drive, kp_hip, kd_hip, torque_hip):
    for motor_id in DRIVE_COMMAND_ORDER:
        if should_stop():
            return
        set_gains(motor_id, kp_drive, kd_drive, torque_drive)

    if not should_stop():
        set_gains(HIP_ID, kp_hip, kd_hip, torque_hip)


def set_drive_gains(kp, kd, torque_limit):
    for motor_id in DRIVE_COMMAND_ORDER:
        if should_stop():
            return
        set_gains(
            motor_id,
            kp,
            kd,
            torque_limit,
        )


def idle_all_motors():
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

    time.sleep(0.08)
    print("Motors sent to IDLE. Returning to terminal.")


# ============================================================
# Safe initial read
# ============================================================
def read_position_while_idle(motor_id):
    pos, vel = bus.write_read_pdo_2(
        motor_id,
        0.0,
        0.0,
    )

    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")

    return pos


def read_initial_positions_safely():
    print("Putting hip, thigh, and shank into IDLE before initial read...")

    for motor_id in ALL_IDS:
        bus.set_mode(motor_id, recoil.Mode.IDLE)
        time.sleep(0.025)

    time.sleep(0.25)

    # Flush stale frames from the bus.
    for _ in range(8):
        for motor_id in ALL_IDS:
            try:
                bus.write_read_pdo_2(motor_id, 0.0, 0.0)
            except Exception:
                pass
        rate.sleep()

    raw = {}

    for motor_id in ALL_IDS:
        samples = []

        for _ in range(15):
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
# Smooth startup: enter POSITION without moving
# ============================================================
def enter_position_mode_without_jump(start_raw):
    print("\nArming POSITION mode without moving the leg...")

    # 1) Zero all gains/torque first.
    set_all_gains(
        ARM_KP,
        ARM_KD,
        ARM_TORQUE_LIMIT,
        ARM_KP,
        ARM_KD,
        ARM_TORQUE_LIMIT,
    )

    # 2) Enter POSITION mode one motor at a time with zero torque.
    for motor_id in POSITION_MODE_ENTRY_ORDER:
        if should_stop():
            return

        print(f"  Entering POSITION mode: {MOTOR_NAMES[motor_id]}")
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

        # Immediately send the measured current raw position several times.
        for _ in range(8):
            if should_stop():
                return
            hold_startup_pose_once(start_raw)
            rate.sleep()

    # 3) Keep spamming current raw positions with zero authority.
    print("  Overwriting internal targets with measured startup pose...")
    for _ in range(ARM_HOLD_CYCLES):
        if should_stop():
            return
        hold_startup_pose_once(start_raw)
        rate.sleep()

    print("  POSITION mode armed at current raw pose.")


def hold_pose_for_seconds(start_raw, seconds):
    cycles = int(seconds * RATE_HZ)

    for _ in range(cycles):
        if should_stop():
            return
        hold_startup_pose_once(start_raw)
        rate.sleep()


def ramp_gains_while_holding(
    start_raw,
    from_drive,
    to_drive,
    from_hip,
    to_hip,
    ramp_time,
    label,
):
    print(label)

    steps = max(1, int(ramp_time * RATE_HZ))

    for i in range(steps):
        if should_stop():
            return

        u = (i + 1) / steps

        kp_d = lerp(from_drive[0], to_drive[0], u)
        kd_d = lerp(from_drive[1], to_drive[1], u)
        tq_d = lerp(from_drive[2], to_drive[2], u)

        kp_h = lerp(from_hip[0], to_hip[0], u)
        kd_h = lerp(from_hip[1], to_hip[1], u)
        tq_h = lerp(from_hip[2], to_hip[2], u)

        set_all_gains(kp_d, kd_d, tq_d, kp_h, kd_h, tq_h)
        hold_startup_pose_once(start_raw)
        rate.sleep()


def smooth_current_pose_startup(start_raw):
    enter_position_mode_without_jump(start_raw)

    if should_stop():
        return

    ramp_gains_while_holding(
        start_raw,
        from_drive=(ARM_KP, ARM_KD, ARM_TORQUE_LIMIT),
        to_drive=(STARTUP_KP, STARTUP_KD, STARTUP_TORQUE_LIMIT),
        from_hip=(ARM_KP, ARM_KD, ARM_TORQUE_LIMIT),
        to_hip=(STARTUP_KP, STARTUP_KD, STARTUP_TORQUE_LIMIT),
        ramp_time=STARTUP_SOFT_HOLD_TIME,
        label="Ramping to very soft current-pose hold...",
    )

    hold_pose_for_seconds(start_raw, 0.30)

    ramp_gains_while_holding(
        start_raw,
        from_drive=(STARTUP_KP, STARTUP_KD, STARTUP_TORQUE_LIMIT),
        to_drive=(MID_KP, MID_KD, MID_TORQUE_LIMIT),
        from_hip=(STARTUP_KP, STARTUP_KD, STARTUP_TORQUE_LIMIT),
        to_hip=(0.010, 0.003, 0.50),
        ramp_time=RAMP_TO_MID_TIME,
        label="Ramping to medium current-pose hold...",
    )

    ramp_gains_while_holding(
        start_raw,
        from_drive=(MID_KP, MID_KD, MID_TORQUE_LIMIT),
        to_drive=(RUN_KP, RUN_KD, RUN_TORQUE_LIMIT),
        from_hip=(0.010, 0.003, 0.50),
        to_hip=(HIP_RUN_KP, HIP_RUN_KD, HIP_TORQUE_LIMIT),
        ramp_time=RAMP_TO_RUN_TIME,
        label="Ramping to run gains while still holding startup pose...",
    )

    print("Startup complete: leg is holding the measured startup pose.")


# ============================================================
# Foot trajectory with frame mapping
# ============================================================
def abstract_trajectory(phase):
    phase = phase % 1.0

    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO

        forward = STEP_LENGTH / 2.0 - STEP_LENGTH * u
        lift = 0.0

        return forward, lift, "stance"

    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    forward = -STEP_LENGTH / 2.0 + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    lift = STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    return forward, lift, "swing"


def map_forward_lift_to_ik(forward, lift):
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
# Main
# ============================================================
controller = None

try:
    print("=" * 80)
    print("Remote-controlled trajectory with smooth current-pose startup")
    print("=" * 80)
    print("Current IDs:")
    print(f"  shank={SHANK_ID}, thigh={THIGH_ID}, hip={HIP_ID}")
    print()
    print("Command order:")
    print("  drive order = " + " -> ".join(MOTOR_NAMES[m] for m in DRIVE_COMMAND_ORDER))
    print("  hip hold is always refreshed last")
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
    print(f"  NEUTRAL_PHASE    = {NEUTRAL_PHASE}")
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

    # Step 1: Read the real current pose in IDLE.
    start_raw = read_initial_positions_safely()
    hip_hold_raw = start_raw[HIP_ID]

    # Step 2: Enter position mode and hold the measured current pose smoothly.
    smooth_current_pose_startup(start_raw)

    if should_stop():
        raise KeyboardInterrupt

    # Step 3: IK reference for the measured startup pose as neutral trajectory pose.
    _, theta_t0, theta_s0 = leg_ik(
        X_CENTER,
        Y_PLANE,
        Z_GROUND,
    )

    print("\nNominal IK reference:")
    print(f"  theta_t0 = {theta_t0:.6f}")
    print(f"  theta_s0 = {theta_s0:.6f}")
    print("  hip output from IK is discarded; hip holds measured startup raw position")

    # Start at neutral stance so the first trajectory target equals start_raw.
    phase = NEUTRAL_PHASE

    neutral = build_targets_for_phase(
        phase,
        start_raw,
        theta_t0,
        theta_s0,
    )

    print("\nStarting remote-controlled trajectory from neutral stance.")
    print(
        f"  neutral phase={phase:.3f}, "
        f"forward={neutral['forward']:+.1f}, "
        f"lift={neutral['lift']:+.1f}"
    )
    print("Left stick Y controls gait phase speed.")
    print("Ctrl+C to stop safely.\n")

    ly_filtered = 0.0
    phase_speed = 0.0
    last_time = time.time()

    prev_cmd = {
        THIGH_ID: start_raw[THIGH_ID],
        SHANK_ID: start_raw[SHANK_ID],
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
            hip_pos, hip_vel = write_hip_hold_command(cmd_hip)

        feedback = {}

        for motor_id in DRIVE_COMMAND_ORDER:
            if motor_id == THIGH_ID:
                feedback[motor_id] = write_drive_position_command(THIGH_ID, cmd_thigh)
            elif motor_id == SHANK_ID:
                feedback[motor_id] = write_drive_position_command(SHANK_ID, cmd_shank)

        # Hip last.
        hip_pos, hip_vel = write_hip_hold_command(cmd_hip)
        feedback[HIP_ID] = (hip_pos, hip_vel)

        thigh_pos, thigh_vel = feedback.get(THIGH_ID, (None, None))
        shank_pos, shank_vel = feedback.get(SHANK_ID, (None, None))

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
    idle_all_motors()

    try:
        if controller is not None:
            controller.close()
    except Exception as e:
        print(f"Controller close warning: {e}")

    print("Shutdown complete.")

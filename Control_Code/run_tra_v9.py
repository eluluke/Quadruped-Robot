import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik


# ============================================================
# Planar cycloidal trot, HIP POSITION HOLD, FRAME-MAPPING VERSION
#
# Based on your working run_tra_v8.py.
#
# The gait generator creates an abstract path:
#   forward = fore/aft foot motion
#   lift    = positive lift amount
#
# Then this code maps forward/lift into IK x/z.
# This lets you test whether your assembled leg response is effectively
# rotated/swapped without rewriting the IK or motor ID logic.
# ============================================================


# ============================================================
# Motor IDs
# ============================================================
SHANK_ID = 3
THIGH_ID = 2
HIP_ID = 1

DRIVE_IDS = [THIGH_ID, SHANK_ID]
ALL_IDS = [HIP_ID, THIGH_ID, SHANK_ID]

MOTOR_NAMES = {
    THIGH_ID: "thigh",
    SHANK_ID: "shank",
    HIP_ID: "hip",
}


# ============================================================
# Gear / sign
# ============================================================
GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0


# ============================================================
# Trajectory tuning
# ============================================================
RATE_HZ = 80.0
CYCLE_TIME = 1.80

X_CENTER = 0.0
Y_PLANE = 84.26
Z_GROUND = 382.0

STEP_LENGTH = 120.0
STEP_HEIGHT = 100.0
STANCE_RATIO = 0.45

MAX_RAW_DELTA_FROM_START = 13.0


# ============================================================
# Trajectory frame mapping
# ============================================================
# Normal old-style mapping:
#   FORWARD_AXIS = "x"
#   LIFT_AXIS    = "z"
#
# Swapped mapping:
#   FORWARD_AXIS = "z"
#   LIFT_AXIS    = "x"
#
# If your physical foot path looks rotated 90 degrees, use the swapped mode.
FORWARD_AXIS = "x"   # "x" or "z"
LIFT_AXIS = "z"      # "x" or "z"

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

PRINT_EVERY = 20


# ============================================================
# Setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)


# ============================================================
# Helpers
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
    return bus.write_read_pdo_2(motor_id, command_pos, 0.0)


def write_drive_position_command(motor_id, command_pos):
    if motor_id == HIP_ID:
        raise RuntimeError("BUG: attempted to command HIP_ID as drive joint")
    return write_position_command(motor_id, command_pos)


def write_hip_hold_command(hip_hold_raw):
    return write_position_command(HIP_ID, hip_hold_raw)


def set_drive_gains(kp, kd, torque_limit):
    for motor_id in DRIVE_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def idle_all_motors():
    print("Putting all motors into IDLE and stopping CAN bus...")

    for motor_id in ALL_IDS:
        try:
            bus.set_mode(motor_id, recoil.Mode.IDLE)
            time.sleep(0.02)
        except Exception:
            pass

    time.sleep(0.15)

    try:
        bus.stop()
    except Exception:
        pass


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ============================================================
# Safe initial read
# ============================================================
def read_position_while_idle(motor_id):
    pos, vel = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")

    return pos


def read_initial_positions_safely():
    print("Putting hip, thigh, and shank into IDLE before initial read...")

    for motor_id in ALL_IDS:
        bus.set_mode(motor_id, recoil.Mode.IDLE)
        time.sleep(0.02)

    time.sleep(0.20)

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
        print(f"  {MOTOR_NAMES[motor_id]} = {raw[motor_id]:.6f}")

    return raw


# ============================================================
# Hip setup
# ============================================================
def setup_hip_position_hold(hip_hold_raw):
    print("\nEntering POSITION mode for hip hold...")

    set_gains(HIP_ID, HIP_STARTUP_KP, HIP_STARTUP_KD, HIP_STARTUP_TORQUE_LIMIT)
    set_mode_with_spacing(HIP_ID, recoil.Mode.POSITION)

    for _ in range(8):
        write_hip_hold_command(hip_hold_raw)
        rate.sleep()

    print(f"Hip holding raw position: {hip_hold_raw:.6f}")


def ramp_hip_to_run_gains():
    print("Switching hip to run hold gains...")
    set_gains(HIP_ID, HIP_RUN_KP, HIP_RUN_KD, HIP_TORQUE_LIMIT)


# ============================================================
# Drive startup: thigh + shank only
# ============================================================
def setup_drive_position_mode_and_hold(start_raw, hip_hold_raw):
    print("\nEntering POSITION mode for thigh and shank...")

    for motor_id in DRIVE_IDS:
        set_gains(motor_id, STARTUP_KP, STARTUP_KD, STARTUP_TORQUE_LIMIT)
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

        for _ in range(5):
            write_drive_position_command(motor_id, start_raw[motor_id])
            write_hip_hold_command(hip_hold_raw)
            rate.sleep()

    print("Soft holding hip/thigh/shank initial positions...")

    for _ in range(int(STARTUP_HOLD_TIME * RATE_HZ)):
        for motor_id in DRIVE_IDS:
            write_drive_position_command(motor_id, start_raw[motor_id])
        write_hip_hold_command(hip_hold_raw)
        rate.sleep()

    print("Startup hold complete.")


# ============================================================
# Abstract trajectory -> IK mapping
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

    lift = STEP_HEIGHT * (1.0 - math.cos(2.0 * math.pi * u)) / 2.0

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
    x, y, z = map_forward_lift_to_ik(forward, lift)
    return x, y, z, forward, lift, phase_name


def raw_delta_from_angle_delta(angle_delta):
    return MOTOR_SIGN * angle_delta * GEAR_RATIO


def build_relative_command_table(start_raw):
    num_points = int(CYCLE_TIME * RATE_HZ)
    table = []

    _ , theta_t0, theta_s0, = leg_ik(X_CENTER, Y_PLANE, Z_GROUND)

    print("\nNominal IK reference:")
    print(f"  theta_t0 = {theta_t0:.6f}")
    print(f"  theta_s0 = {theta_s0:.6f}")
    print("  hip output from IK is discarded; hip holds current raw position")

    max_thigh_delta = 0.0
    max_shank_delta = 0.0

    for i in range(num_points):
        phase = i / num_points
        x, y, z, forward, lift, phase_name = foot_trajectory(phase)

        _ , theta_t, theta_s = leg_ik(x, y, z)

        delta_t = theta_t - theta_t0
        delta_s = theta_s - theta_s0

        raw_thigh = start_raw[THIGH_ID] + raw_delta_from_angle_delta(delta_t)
        raw_shank = start_raw[SHANK_ID] + raw_delta_from_angle_delta(delta_s)

        thigh_delta_raw = raw_thigh - start_raw[THIGH_ID]
        shank_delta_raw = raw_shank - start_raw[SHANK_ID]

        max_thigh_delta = max(max_thigh_delta, abs(thigh_delta_raw))
        max_shank_delta = max(max_shank_delta, abs(shank_delta_raw))

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

        table.append(
            {
                "phase": phase,
                "phase_name": phase_name,
                "forward": forward,
                "lift": lift,
                "x": x,
                "y": y,
                "z": z,
                "theta_t": theta_t,
                "theta_s": theta_s,
                "delta_t": delta_t,
                "delta_s": delta_s,
                "raw_thigh": raw_thigh,
                "raw_shank": raw_shank,
            }
        )

    print("\nTrajectory command range:")
    print(f"  max thigh raw delta = {max_thigh_delta:.3f}")
    print(f"  max shank raw delta = {max_shank_delta:.3f}")
    print("No hip trajectory values are stored in the command table.")

    print("\nFirst few mapped trajectory points:")
    for p in table[:8]:
        print(
            f"  phase={p['phase']:.3f} {p['phase_name']} | "
            f"forward={p['forward']:+.1f} lift={p['lift']:+.1f} -> "
            f"x={p['x']:+.1f} z={p['z']:+.1f}"
        )

    return table


# ============================================================
# Smooth move to first point
# ============================================================
def smooth_move_to_first_targets(first_targets, move_time, hip_hold_raw):
    start_raw = {
        motor_id: first_targets[motor_id]["start"]
        for motor_id in DRIVE_IDS
    }

    target_raw = {
        motor_id: first_targets[motor_id]["target"]
        for motor_id in DRIVE_IDS
    }

    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        for motor_id in DRIVE_IDS:
            cmd = start_raw[motor_id] + (
                target_raw[motor_id] - start_raw[motor_id]
            ) * s

            write_drive_position_command(motor_id, cmd)

        write_hip_hold_command(hip_hold_raw)
        rate.sleep()


# ============================================================
# Main
# ============================================================
try:
    print("=" * 80)
    print("Planar cycloidal trot, HIP HOLD, FRAME-MAPPING VERSION")
    print("=" * 80)
    print("Current IDs:")
    print(f"  shank={SHANK_ID}, thigh={THIGH_ID}, hip={HIP_ID}")
    print("Hip holds measured startup position.")
    print("Only thigh and shank receive trajectory commands.")
    print()
    print("Abstract trajectory:")
    print(f"  STEP_LENGTH = {STEP_LENGTH} mm")
    print(f"  STEP_HEIGHT = {STEP_HEIGHT} mm")
    print(f"  STANCE_RATIO = {STANCE_RATIO}")
    print()
    print("Frame mapping:")
    print(f"  FORWARD_AXIS = {FORWARD_AXIS}")
    print(f"  LIFT_AXIS    = {LIFT_AXIS}")
    print(f"  FORWARD_SIGN = {FORWARD_SIGN}")
    print(f"  LIFT_SIGN    = {LIFT_SIGN}")
    print()
    print("If the path is rotated, change only FORWARD_AXIS/LIFT_AXIS/signs.")
    print("=" * 80)

    start_raw = read_initial_positions_safely()
    hip_hold_raw = start_raw[HIP_ID]

    setup_hip_position_hold(hip_hold_raw)
    setup_drive_position_mode_and_hold(start_raw, hip_hold_raw)

    print("\nBuilding relative IK trajectory...")
    command_table = build_relative_command_table(start_raw)

    first = command_table[0]

    first_targets = {
        THIGH_ID: {
            "start": start_raw[THIGH_ID],
            "target": first["raw_thigh"],
        },
        SHANK_ID: {
            "start": start_raw[SHANK_ID],
            "target": first["raw_shank"],
        },
    }

    print("\nRamping thigh/shank to medium gains...")
    set_drive_gains(MID_KP, MID_KD, MID_TORQUE_LIMIT)

    print("Moving thigh/shank to first trajectory point...")
    smooth_move_to_first_targets(first_targets, MOVE_TO_START_TIME, hip_hold_raw)

    print("Switching thigh/shank to run gains...")
    set_drive_gains(RUN_KP, RUN_KD, RUN_TORQUE_LIMIT)

    ramp_hip_to_run_gains()

    print("\nStarting mapped planar cycloid trot with hip hold.")
    print("Press Ctrl+C to stop.\n")

    index = 0
    counter = 0

    while True:
        point = command_table[index]

        if DOUBLE_REFRESH_HIP:
            hip_pos, hip_vel = write_hip_hold_command(hip_hold_raw)

        thigh_pos, thigh_vel = write_drive_position_command(
            THIGH_ID,
            point["raw_thigh"],
        )

        shank_pos, shank_vel = write_drive_position_command(
            SHANK_ID,
            point["raw_shank"],
        )

        hip_pos, hip_vel = write_hip_hold_command(hip_hold_raw)

        counter += 1

        if counter % PRINT_EVERY == 0:
            hip_err = wrap_pi(hip_hold_raw - hip_pos) if hip_pos is not None else None

            print(
                f"phase={point['phase']:.3f} {point['phase_name']} | "
                f"forward={point['forward']:+.1f} "
                f"lift={point['lift']:+.1f} | "
                f"x={point['x']:+.1f} "
                f"z={point['z']:+.1f} | "
                f"dtheta_t={point['delta_t']:.4f} "
                f"dtheta_s={point['delta_s']:.4f} | "
                f"raw_t={point['raw_thigh']:.3f} "
                f"raw_s={point['raw_shank']:.3f} | "
                f"hip_hold={hip_hold_raw:.3f} "
                f"hip_pos={hip_pos:.3f} "
                f"hip_err={hip_err:.3f}"
            )

        index += 1
        if index >= len(command_table):
            index = 0

        rate.sleep()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    idle_all_motors()

import time
import math
import sys
from dataclasses import dataclass

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik



# ============================================================
# IK terminal test mode
# Run like: python leg_homing_sequence_torque.py --ik-test
# Then type x y z in mm and get angles back immediately.
# ============================================================
def print_ik_result(x_mm, y_mm, z_mm):
    theta_h, theta_t, theta_s = leg_ik(x_mm, y_mm, z_mm)
    print(
        f"Input Cartesian position [mm]: x={x_mm:.3f}, y={y_mm:.3f}, z={z_mm:.3f}\n"
        f"Output joint angles [rad]: hip={theta_h:.6f}, thigh={theta_t:.6f}, shank={theta_s:.6f}\n"
        f"Output joint angles [deg]: hip={math.degrees(theta_h):.3f}, "
        f"thigh={math.degrees(theta_t):.3f}, shank={math.degrees(theta_s):.3f}"
    )


def run_ik_terminal_mode():
    print("IK terminal test mode")
    print("Type three numbers: x y z   (unit: mm)")
    print("Example: 0 84.26 381.84")
    print("Type 'q' to quit.\n")

    while True:
        try:
            raw = input("Enter x y z in mm: ").strip()
        except EOFError:
            print()
            return

        if not raw:
            continue

        if raw.lower() in {"q", "quit", "exit"}:
            return

        parts = raw.replace(",", " ").split()
        if len(parts) != 3:
            print("Please enter exactly three numbers, for example: 17.55 145.79 -16.13\n")
            continue

        try:
            x_mm, y_mm, z_mm = map(float, parts)
            print_ik_result(x_mm, y_mm, z_mm)
            print()
        except ValueError:
            print("Could not parse the input as three numbers.\n")
        except Exception as exc:
            print(f"IK calculation failed: {exc}\n")


if "--ik-test" in sys.argv:
    run_ik_terminal_mode()
    raise SystemExit(0)

# ============================================================
# User-adjustable configuration
# ============================================================
HIP_ID = 2
THIGH_ID = 1
SHANK_ID = 0
JOINT_IDS = [HIP_ID, THIGH_ID, SHANK_ID]
JOINT_NAMES = {
    HIP_ID: "hip",
    THIGH_ID: "thigh",
    SHANK_ID: "shank",
}

# joint / output angle -> motor angle
GEAR_RATIO = 17.0

# After homing:
#   True  -> automatically start the foot trajectory
#   False -> move to neutral and just hold there
START_TRAJECTORY_AFTER_HOMING = True

# If START_TRAJECTORY_AFTER_HOMING is False, the script stays alive and keeps
# holding neutral until you press Ctrl+C.
HOLD_NEUTRAL_IF_NOT_RUNNING = True

# If True, Ctrl+C / normal script end will set motors to IDLE.
# If False, the script will stop the bus without intentionally idling the motors.
# In practice, for safe testing, keep this True.
IDLE_ON_EXIT = True

# ------------------------------------------------------------
# Homing direction: +1 or -1 for each joint.
# You said you will tune this after testing.
# ------------------------------------------------------------
HOMING_DIRECTION = {
    HIP_ID: 1.0,
    THIGH_ID: 1.0,
    SHANK_ID: 1.0,
}

# ------------------------------------------------------------
# Homing motion / threshold tuning
# ------------------------------------------------------------
HOMING_VELOCITY_CMD = 0.18          # output-side rad/s equivalent, used as incremental walk speed
HOMING_TIMEOUT = 8.0                # seconds per joint
POST_LIMIT_BACKOFF_OUTPUT_RAD = {
    HIP_ID: 0.05,
    THIGH_ID: 0.05,
    SHANK_ID: 0.05,
}

STALL_VELOCITY_THRESHOLD = 0.035    # motor rad/s
STALL_MIN_TIME = 0.10               # seconds
TORQUE_ABS_LIMIT = 2.0              # Nm, adjust from real tests
TORQUE_SLOPE_LIMIT = 20.0           # Nm/s, adjust from real tests
USE_TORQUE_TRIGGER = True
REQUIRE_STALL_AND_TORQUE = False    # False = either one can trigger; True = both required

# ------------------------------------------------------------
# Gains
# ------------------------------------------------------------
HOMING_KP = 0.010
HOMING_KD = 0.002
HOMING_TORQUE_LIMIT = 0.18

HOLD_KP = 0.030
HOLD_KD = 0.003
HOLD_TORQUE_LIMIT = 0.22

RUN_KP = 0.16
RUN_KD = 0.004
RUN_TORQUE_LIMIT = 0.50

# ------------------------------------------------------------
# Known IK-frame reference points
# ------------------------------------------------------------
MAX_CONTRACTION_XYZ_MM = (17.55, 145.79, -16.13)
NEUTRAL_XYZ_MM = (0.0, 84.26, 381.84)
MOVE_TO_NEUTRAL_SECONDS = 3.5

# ------------------------------------------------------------
# Foot trajectory
# ------------------------------------------------------------
CYCLE_TIME = 2.5
STANCE_RATIO = 0.35
STEP_LENGTH = 60.0
STEP_HEIGHT = 70.0

X_CENTER = 0.0
Y_PLANE = 83.07
Z_GROUND = 300.0

# ------------------------------------------------------------
# Timing / prints
# ------------------------------------------------------------
RATE_HZ = 100.0
PRINT_EVERY = 20


# ============================================================
# Small helpers
# ============================================================
def lerp(a, b, u):
    return a + (b - a) * u


@dataclass
class JointState:
    pos: float | None
    aux: float | None


class HomingError(RuntimeError):
    pass


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



def set_gains(motor_id, kp_val, kd_val, torque_val):
    bus.write_position_kp(motor_id, kp_val)
    time.sleep(0.002)
    bus.write_position_kd(motor_id, kd_val)
    time.sleep(0.002)
    bus.write_torque_limit(motor_id, torque_val)
    time.sleep(0.002)



def hold_joint_at_position(motor_id, motor_angle_cmd):
    return bus.write_read_pdo_2(motor_id, motor_angle_cmd, 0.0)



def read_pos_vel(motor_id):
    pos, vel = bus.write_read_pdo_2(motor_id, 0.0, 0.0)
    return JointState(pos=pos, aux=vel)



def read_pos_torque(motor_id):
    """
    Preferred path for Berkeley firmware:
    PDO3 should return [position_measured, torque_measured].
    If the local Python wrapper does not expose write_read_pdo_3, we return None.
    """
    fn = getattr(bus, "write_read_pdo_3", None)
    if not callable(fn):
        return JointState(pos=None, aux=None)

    try:
        pos, torque = fn(motor_id, 0.0, 0.0)
        return JointState(pos=pos, aux=torque)
    except Exception:
        return JointState(pos=None, aux=None)



def read_joint_feedback(motor_id):
    """
    Returns a dict with:
      pos    motor position [rad]
      vel    motor velocity [rad/s]
      torque measured torque [Nm] if PDO3 is available, else None
    """
    pv = read_pos_vel(motor_id)
    pt = read_pos_torque(motor_id)
    torque = pt.aux if pt.aux is not None else None
    return {
        "pos": pv.pos,
        "vel": pv.aux,
        "torque": torque,
    }



def output_to_motor_delta(output_angle_rad):
    return output_angle_rad * GEAR_RATIO



def ik_output_from_xyz(x_mm, y_mm, z_mm):
    theta_h, theta_t, theta_s = leg_ik(x_mm, y_mm, z_mm)
    return {
        HIP_ID: theta_h,
        THIGH_ID: theta_t,
        SHANK_ID: theta_s,
    }


# ============================================================
# Trajectory helper
# ============================================================
def foot_trajectory(phase):
    phase = phase % 1.0

    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO
        x = X_CENTER + (STEP_LENGTH / 2.0) - STEP_LENGTH * u
        y = Y_PLANE
        z = Z_GROUND
        return x, y, z

    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)
    x = X_CENTER - (STEP_LENGTH / 2.0) + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )
    y = Y_PLANE
    z = Z_GROUND + STEP_HEIGHT * (1.0 - math.cos(2.0 * math.pi * u)) / 2.0
    return x, y, z


# ============================================================
# Motion primitives
# ============================================================
def velocity_walk_joint(motor_id, target_motor_angle, step_size_rad):
    state = read_joint_feedback(motor_id)
    if state["pos"] is None:
        return None

    delta = target_motor_angle - state["pos"]
    if abs(delta) <= step_size_rad:
        cmd = target_motor_angle
    else:
        cmd = state["pos"] + math.copysign(step_size_rad, delta)

    pos, vel = hold_joint_at_position(motor_id, cmd)
    torque_state = read_pos_torque(motor_id)
    torque = torque_state.aux if torque_state.aux is not None else None
    return {
        "pos": pos,
        "vel": vel,
        "torque": torque,
    }



def move_all_joints_to_targets(target_motor_angles, seconds):
    start_positions = {}
    for motor_id in JOINT_IDS:
        fb = read_joint_feedback(motor_id)
        if fb["pos"] is None:
            raise HomingError(f"Cannot read {JOINT_NAMES[motor_id]} before move")
        start_positions[motor_id] = fb["pos"]

    steps = max(1, int(seconds * RATE_HZ))
    for i in range(steps):
        u = (i + 1) / steps
        u_smooth = 0.5 - 0.5 * math.cos(math.pi * u)
        for motor_id in JOINT_IDS:
            cmd = lerp(start_positions[motor_id], target_motor_angles[motor_id], u_smooth)
            hold_joint_at_position(motor_id, cmd)
        rate.sleep()



def hold_all_joints_forever(target_motor_angles):
    print("Holding neutral position. Press Ctrl+C to exit.")
    while True:
        for motor_id in JOINT_IDS:
            hold_joint_at_position(motor_id, target_motor_angles[motor_id])
        rate.sleep()


# ============================================================
# Homing
# ============================================================
def home_single_joint_to_limit(motor_id):
    print(f"Homing {JOINT_NAMES[motor_id]} toward hard stop...")

    initial = read_joint_feedback(motor_id)
    if initial["pos"] is None:
        raise HomingError(f"Cannot read start position of {JOINT_NAMES[motor_id]}")

    direction = float(HOMING_DIRECTION[motor_id])
    step_per_cycle = abs(HOMING_VELOCITY_CMD) / RATE_HZ
    commanded_target = initial["pos"]

    stalled_since = None
    last_torque = None
    last_time = None
    t_start = time.perf_counter()

    while True:
        commanded_target += direction * step_per_cycle
        state = velocity_walk_joint(motor_id, commanded_target, step_per_cycle)

        if state is None or state["pos"] is None or state["vel"] is None:
            raise HomingError(f"Lost telemetry while homing {JOINT_NAMES[motor_id]}")

        now = time.perf_counter()
        torque_slope = None
        if state["torque"] is not None and last_torque is not None and last_time is not None:
            dt = max(now - last_time, 1e-6)
            torque_slope = (state["torque"] - last_torque) / dt

        last_torque = state["torque"]
        last_time = now

        # Stall = motor no longer really moving although we keep pushing command.
        is_stalled = abs(state["vel"]) < STALL_VELOCITY_THRESHOLD
        if is_stalled:
            if stalled_since is None:
                stalled_since = now
        else:
            stalled_since = None

        long_enough_stall = (
            stalled_since is not None and (now - stalled_since) >= STALL_MIN_TIME
        )

        torque_trigger = False
        if USE_TORQUE_TRIGGER and state["torque"] is not None:
            if abs(state["torque"]) >= TORQUE_ABS_LIMIT:
                torque_trigger = True
            if torque_slope is not None and abs(torque_slope) >= TORQUE_SLOPE_LIMIT:
                torque_trigger = True

        if REQUIRE_STALL_AND_TORQUE:
            hit_limit = long_enough_stall and torque_trigger
        else:
            hit_limit = long_enough_stall or torque_trigger

        if hit_limit:
            print(
                f"{JOINT_NAMES[motor_id]} stop detected | "
                f"pos={state['pos']:.4f} vel={state['vel']:.4f} "
                f"torque={state['torque']} slope={torque_slope}"
            )
            return state["pos"]

        if now - t_start > HOMING_TIMEOUT:
            raise HomingError(
                f"Timeout while homing {JOINT_NAMES[motor_id]}. "
                f"Try reversing HOMING_DIRECTION or relaxing thresholds."
            )

        rate.sleep()



def back_off_from_limit(limit_motor_angles):
    print("Backing all joints slightly away from the hard stop...")

    backed_off_targets = {}
    for motor_id in JOINT_IDS:
        backoff_motor = output_to_motor_delta(POST_LIMIT_BACKOFF_OUTPUT_RAD[motor_id])
        backed_off_targets[motor_id] = (
            limit_motor_angles[motor_id]
            - HOMING_DIRECTION[motor_id] * backoff_motor
        )

    move_all_joints_to_targets(backed_off_targets, seconds=1.0)
    return backed_off_targets



def build_homed_trajectory_table(homed_motor_angles_at_max_contraction):
    ref_output_angles = ik_output_from_xyz(*MAX_CONTRACTION_XYZ_MM)
    num_points = int(CYCLE_TIME * RATE_HZ)
    table = []

    for point_index in range(num_points):
        phase = point_index / num_points
        x_des, y_des, z_des = foot_trajectory(phase)
        des_output_angles = ik_output_from_xyz(x_des, y_des, z_des)

        motor_cmds = {}
        for motor_id in JOINT_IDS:
            output_delta = des_output_angles[motor_id] - ref_output_angles[motor_id]
            motor_cmds[motor_id] = (
                homed_motor_angles_at_max_contraction[motor_id]
                + output_to_motor_delta(output_delta)
            )

        table.append(
            {
                "x": x_des,
                "y": y_des,
                "z": z_des,
                "cmd_hip": motor_cmds[HIP_ID],
                "cmd_thigh": motor_cmds[THIGH_ID],
                "cmd_shank": motor_cmds[SHANK_ID],
            }
        )

    return table



def run_trajectory_forever(command_table):
    print("Starting regular foot trajectory. Press Ctrl+C to stop.")
    point_index = 0
    counter = 0

    while True:
        point = command_table[point_index]
        hip_pos, _ = hold_joint_at_position(HIP_ID, point["cmd_hip"])
        thigh_pos, _ = hold_joint_at_position(THIGH_ID, point["cmd_thigh"])
        shank_pos, _ = hold_joint_at_position(SHANK_ID, point["cmd_shank"])

        counter += 1
        if counter % PRINT_EVERY == 0:
            print(
                f"x={point['x']:.1f} y={point['y']:.1f} z={point['z']:.1f} | "
                f"hip={hip_pos:.3f} th={thigh_pos:.3f} sh={shank_pos:.3f}"
            )

        point_index += 1
        if point_index >= len(command_table):
            point_index = 0

        rate.sleep()


# ============================================================
# Main
# ============================================================
try:
    print("Putting all three joints into gentle position mode...")
    for motor_id in JOINT_IDS:
        set_gains(motor_id, HOMING_KP, HOMING_KD, HOMING_TORQUE_LIMIT)
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    time.sleep(0.05)

    # 1) Home each joint to a mechanical limit.
    limit_motor_angles = {}
    for motor_id in JOINT_IDS:
        limit_motor_angles[motor_id] = home_single_joint_to_limit(motor_id)

    print(
        "Mechanical limit motor angles:\n"
        f"  hip   = {limit_motor_angles[HIP_ID]:.5f} rad\n"
        f"  thigh = {limit_motor_angles[THIGH_ID]:.5f} rad\n"
        f"  shank = {limit_motor_angles[SHANK_ID]:.5f} rad"
    )

    # 2) Back off slightly so the leg is no longer jammed.
    homed_motor_angles = back_off_from_limit(limit_motor_angles)

    print(
        "Backed-off homed motor angles:\n"
        f"  hip   = {homed_motor_angles[HIP_ID]:.5f} rad\n"
        f"  thigh = {homed_motor_angles[THIGH_ID]:.5f} rad\n"
        f"  shank = {homed_motor_angles[SHANK_ID]:.5f} rad"
    )

    # 3) Treat this backed-off posture as the known max-contraction Cartesian pose.
    max_contraction_output = ik_output_from_xyz(*MAX_CONTRACTION_XYZ_MM)
    neutral_output = ik_output_from_xyz(*NEUTRAL_XYZ_MM)

    print(
        "IK angles at max contraction:\n"
        f"  hip   = {max_contraction_output[HIP_ID]:.5f} rad\n"
        f"  thigh = {max_contraction_output[THIGH_ID]:.5f} rad\n"
        f"  shank = {max_contraction_output[SHANK_ID]:.5f} rad"
    )

    # 4) Move slowly to neutral so it does not kick outward from a folded posture.
    neutral_motor_targets = {}
    for motor_id in JOINT_IDS:
        output_delta = neutral_output[motor_id] - max_contraction_output[motor_id]
        neutral_motor_targets[motor_id] = (
            homed_motor_angles[motor_id] + output_to_motor_delta(output_delta)
        )

    print("Moving slowly to neutral standing pose...")
    for motor_id in JOINT_IDS:
        set_gains(motor_id, HOLD_KP, HOLD_KD, HOLD_TORQUE_LIMIT)
    move_all_joints_to_targets(neutral_motor_targets, MOVE_TO_NEUTRAL_SECONDS)

    print(
        "Now at neutral target. "
        "The leg will hold there as long as this script keeps running in position mode."
    )

    # 5) Either hold neutral or directly start gait.
    if START_TRAJECTORY_AFTER_HOMING:
        for motor_id in JOINT_IDS:
            set_gains(motor_id, RUN_KP, RUN_KD, RUN_TORQUE_LIMIT)
        command_table = build_homed_trajectory_table(homed_motor_angles)
        run_trajectory_forever(command_table)
    else:
        if HOLD_NEUTRAL_IF_NOT_RUNNING:
            hold_all_joints_forever(neutral_motor_targets)
        else:
            print("Configured not to start trajectory and not to hold forever. Exiting.")

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    try:
        if IDLE_ON_EXIT:
            for motor_id in JOINT_IDS:
                try:
                    set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
                except Exception:
                    pass
        time.sleep(0.05)
    finally:
        try:
            bus.stop()
        except Exception:
            pass

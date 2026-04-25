import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik


# ============================================================
# Correct motor IDs
# ============================================================
SHANK_ID = 1
THIGH_ID = 0
HIP_ID = 2

DRIVE_IDS = [THIGH_ID, SHANK_ID]
ALL_IDS = [THIGH_ID, SHANK_ID, HIP_ID]

MOTOR_NAMES = {
    SHANK_ID: "shank",
    THIGH_ID: "thigh",
    HIP_ID: "hip",
}


# ============================================================
# Gear / sign
# ============================================================
GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0


# ============================================================
# Direction tuning
# ============================================================
# You said the foot direction looks correct with this.
X_DIRECTION_SIGN = -1.0

# Your IK frame has z positive downward.
# For physical foot lift, z should decrease.
Z_LIFT_SIGN = 1.0


# ============================================================
# Trajectory tuning
# ============================================================
RATE_HZ = 80.0

# Larger = slower, smaller = faster.
CYCLE_TIME = 2.0

X_CENTER = 0.0
Y_PLANE = 84.26
Z_GROUND = 382.0

# Start conservative. Increase gradually after stable testing.
STEP_LENGTH = 120.0
STEP_HEIGHT = 100.0

# 0.50 = equal stance / swing
# 0.45 = slightly longer smoother swing
# 0.60 = longer stance, faster swing
STANCE_RATIO = 0.50

MAX_RAW_DELTA_FROM_START = 13.0


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


def set_drive_gains(kp, kd, torque_limit):
    for motor_id in DRIVE_IDS:
        set_gains(
            motor_id,
            kp,
            kd,
            torque_limit,
        )


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


# ============================================================
# Safe read while IDLE
# ============================================================
def read_position_while_idle(motor_id):
    """
    Only use this while motor is IDLE.

    write_read_pdo_2 is not a pure read, but while IDLE,
    the command should not move the joint.
    """
    pos, vel = bus.write_read_pdo_2(
        motor_id,
        0.0,
        0.0,
    )

    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")

    return pos


def read_initial_drive_positions_safely():
    """
    Put all motors into IDLE.
    Read only thigh and shank starting positions.
    Hip is deliberately not controlled after this.
    """
    print("Putting all motors into IDLE before initial read...")

    for motor_id in ALL_IDS:
        bus.set_mode(motor_id, recoil.Mode.IDLE)
        time.sleep(0.02)

    time.sleep(0.20)

    # Flush stale CAN frames while motors are IDLE.
    for _ in range(5):
        for motor_id in ALL_IDS:
            try:
                bus.write_read_pdo_2(motor_id, 0.0, 0.0)
            except Exception:
                pass
        rate.sleep()

    raw = {}

    for motor_id in DRIVE_IDS:
        samples = []

        for _ in range(15):
            pos = read_position_while_idle(motor_id)
            samples.append(pos)
            rate.sleep()

        samples.sort()
        raw[motor_id] = samples[len(samples) // 2]

    print("Initial drive raw encoder positions:")
    for motor_id in DRIVE_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]} = "
            f"{raw[motor_id]:.6f}"
        )

    print("\nHip is set to IDLE and will receive NO commands.")
    return raw


# ============================================================
# Hip idle setup
# ============================================================
def setup_hip_idle_only():
    """
    Diagnostic / demo mode:
    hip is completely unmanaged.
    No damping.
    No position hold.
    No write_read_pdo_2.
    No feed.
    """
    print("\nSetting hip to IDLE only.")
    print("After this, the script will not command or feed the hip.")

    bus.set_mode(HIP_ID, recoil.Mode.IDLE)
    time.sleep(0.05)


# ============================================================
# Thigh + shank startup
# ============================================================
def setup_drive_position_mode_and_hold(start_raw):
    """
    Thigh and shank enter position mode and hold their initial raw positions.
    Hip is not included.
    """
    print("\nEntering POSITION mode for thigh and shank only...")

    for motor_id in DRIVE_IDS:
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
            write_position_command(
                motor_id,
                start_raw[motor_id],
            )
            rate.sleep()

    print("Soft holding thigh/shank initial positions...")

    for _ in range(int(STARTUP_HOLD_TIME * RATE_HZ)):
        for motor_id in DRIVE_IDS:
            write_position_command(
                motor_id,
                start_raw[motor_id],
            )

        rate.sleep()

    print("Drive startup hold complete.")


# ============================================================
# Foot trajectory
# ============================================================
def foot_trajectory(phase):
    phase = phase % 1.0

    # --------------------------------------------------------
    # Stance phase:
    # foot on ground, straight stroke.
    # --------------------------------------------------------
    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO

        # local x goes +L/2 to -L/2
        x_local = STEP_LENGTH / 2.0 - STEP_LENGTH * u

        x = X_CENTER + X_DIRECTION_SIGN * x_local
        y = Y_PLANE
        z = Z_GROUND

        return x, y, z

    # --------------------------------------------------------
    # Swing phase:
    # foot lifts and returns with cycloid.
    # --------------------------------------------------------
    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    # local x goes -L/2 to +L/2
    x_local = -STEP_LENGTH / 2.0 + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    lift = STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    x = X_CENTER + X_DIRECTION_SIGN * x_local
    y = Y_PLANE

    # z positive downward, so lifting means z decreases.
    z = Z_GROUND + Z_LIFT_SIGN * lift

    return x, y, z


def raw_delta_from_angle_delta(angle_delta):
    return MOTOR_SIGN * angle_delta * GEAR_RATIO


def build_relative_command_table(start_raw):
    """
    Build trajectory commands for thigh and shank only.

    leg_ik returns:
        theta_h, theta_t, theta_s

    theta_h is computed only for debug and ignored.
    Hip motor command is never generated.
    """
    num_points = int(CYCLE_TIME * RATE_HZ)
    table = []

    theta_h0, theta_t0, theta_s0 = leg_ik(
        X_CENTER,
        Y_PLANE,
        Z_GROUND,
    )

    print("\nNominal IK reference:")
    print(f"  theta_h0 = {theta_h0:.6f}  <-- ignored")
    print(f"  theta_t0 = {theta_t0:.6f}")
    print(f"  theta_s0 = {theta_s0:.6f}")

    max_thigh_delta = 0.0
    max_shank_delta = 0.0
    max_ignored_hip_delta = 0.0

    for i in range(num_points):
        phase = i / num_points
        x, y, z = foot_trajectory(phase)

        theta_h, theta_t, theta_s = leg_ik(
            x,
            y,
            z,
        )

        ignored_delta_h = theta_h - theta_h0

        # Only thigh and shank are used for motor command.
        delta_t = theta_t - theta_t0
        delta_s = theta_s - theta_s0

        raw_thigh = start_raw[THIGH_ID] + raw_delta_from_angle_delta(delta_t)
        raw_shank = start_raw[SHANK_ID] + raw_delta_from_angle_delta(delta_s)

        thigh_delta_raw = raw_thigh - start_raw[THIGH_ID]
        shank_delta_raw = raw_shank - start_raw[SHANK_ID]

        max_thigh_delta = max(
            max_thigh_delta,
            abs(thigh_delta_raw),
        )

        max_shank_delta = max(
            max_shank_delta,
            abs(shank_delta_raw),
        )

        max_ignored_hip_delta = max(
            max_ignored_hip_delta,
            abs(ignored_delta_h),
        )

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
                "x": x,
                "y": y,
                "z": z,

                # Debug only.
                "theta_h_ignored": theta_h,
                "ignored_delta_h": ignored_delta_h,

                # Used.
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
    print(f"  max ignored hip IK delta = {max_ignored_hip_delta:.6f}")

    print(
        "\nImportant: hip IK angle is ignored. "
        "Hip is IDLE and receives no command."
    )

    return table


# ============================================================
# Smooth move to first point
# ============================================================
def smooth_move_to_first_targets(first_targets, move_time):
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

        # Hip is not commanded.

        for motor_id in DRIVE_IDS:
            cmd = start_raw[motor_id] + (
                target_raw[motor_id] - start_raw[motor_id]
            ) * s

            write_position_command(
                motor_id,
                cmd,
            )

        rate.sleep()


# ============================================================
# Main
# ============================================================
try:
    print("Planar cycloidal trot, NO HOMING, HIP IDLE ONLY")
    print("Correct IDs: shank=1, thigh=0, hip=2")
    print("Hip is put into IDLE and receives no commands.")
    print("Only thigh and shank follow trajectory.")
    print("On exit, all motors go to IDLE.")
    print()
    print("Tuning:")
    print(f"  X_DIRECTION_SIGN = {X_DIRECTION_SIGN}")
    print(f"  Z_LIFT_SIGN      = {Z_LIFT_SIGN}")
    print(f"  CYCLE_TIME       = {CYCLE_TIME} s")
    print(f"  STEP_LENGTH      = {STEP_LENGTH} mm")
    print(f"  STEP_HEIGHT      = {STEP_HEIGHT} mm")
    print(f"  STANCE_RATIO     = {STANCE_RATIO}")
    print()

    # Step 1: Read thigh and shank initial raw encoder positions.
    start_raw = read_initial_drive_positions_safely()

    # Step 2: Put hip into IDLE and leave it alone.
    setup_hip_idle_only()

    # Step 3: Thigh/shank startup hold.
    setup_drive_position_mode_and_hold(
        start_raw,
    )

    # Step 4: Build relative IK trajectory for thigh/shank only.
    print("\nBuilding relative IK trajectory...")
    command_table = build_relative_command_table(
        start_raw,
    )

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

    # Step 5: Move thigh/shank to first point.
    print("\nRamping thigh/shank to medium gains...")
    set_drive_gains(
        MID_KP,
        MID_KD,
        MID_TORQUE_LIMIT,
    )

    print("Moving thigh/shank to first trajectory point...")
    smooth_move_to_first_targets(
        first_targets,
        MOVE_TO_START_TIME,
    )

    # Step 6: Run trajectory.
    print("Switching thigh/shank to run gains...")
    set_drive_gains(
        RUN_KP,
        RUN_KD,
        RUN_TORQUE_LIMIT,
    )

    print("\nStarting planar cycloid trot.")
    print("Press Ctrl+C to stop.\n")

    index = 0
    counter = 0

    while True:
        point = command_table[index]

        # Hip is not commanded.

        thigh_pos, thigh_vel = write_position_command(
            THIGH_ID,
            point["raw_thigh"],
        )

        shank_pos, shank_vel = write_position_command(
            SHANK_ID,
            point["raw_shank"],
        )

        counter += 1

        if counter % PRINT_EVERY == 0:
            print(
                f"phase={point['phase']:.3f} | "
                f"x={point['x']:.1f} "
                f"z={point['z']:.1f} | "
                f"dtheta_t={point['delta_t']:.4f} "
                f"dtheta_s={point['delta_s']:.4f} | "
                f"ignored_dhip={point['ignored_delta_h']:.4f} | "
                f"raw_t={point['raw_thigh']:.3f} "
                f"raw_s={point['raw_shank']:.3f}"
            )

        index += 1

        if index >= len(command_table):
            index = 0

        rate.sleep()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    idle_all_motors()

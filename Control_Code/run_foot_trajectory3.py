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
X_DIRECTION_SIGN = -1.0

# Your IK frame has z positive downward.
# If you want physical foot lift, this is usually -1.0.
# If your current physical trajectory looked correct with +1.0, change it back.
Z_LIFT_SIGN = 1.0


# ============================================================
# Speed tuning
# ============================================================
RATE_HZ = 80.0
CYCLE_TIME = 2.0


# ============================================================
# Planar cycloid trajectory tuning
# ============================================================
X_CENTER = 0.0
Y_PLANE = 84.26
Z_GROUND = 382.0

STEP_LENGTH = 20.0
STEP_HEIGHT = 70.0

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

DRIVE_STARTUP_HOLD_TIME = 1.2
MOVE_TO_START_TIME = 2.5

PRINT_EVERY = 20


# ============================================================
# Hip startup hold tuning
# ============================================================
# Hip holds its INITIAL RAW ENCODER POSITION only during startup.
# After trajectory begins, this script never commands HIP_ID again.
HIP_HOLD_KP = 0.008
HIP_HOLD_KD = 0.005
HIP_HOLD_TORQUE_LIMIT = 1.5

HIP_STARTUP_HOLD_TIME = 1.2


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


def write_drive_position_command(motor_id, command_pos):
    """
    Only thigh and shank are allowed to use this function.

    This guard prevents accidental hip commands during trajectory.
    """
    if motor_id == HIP_ID:
        raise RuntimeError(
            "BUG: attempted to command HIP_ID during drive trajectory")

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
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
        except Exception:
            pass

    time.sleep(0.15)

    try:
        bus.stop()
    except Exception:
        pass


# ============================================================
# Safe initial read
# ============================================================
def read_position_while_idle(motor_id):
    """
    Only use this while the motor is IDLE.

    write_read_pdo_2 is not a pure read. It writes a command too.
    While IDLE, the command should not move the joint.
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
    print("Putting all motors into IDLE before initial read...")

    for motor_id in ALL_IDS:
        set_mode_with_spacing(motor_id, recoil.Mode.IDLE)

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
# Hip setup: startup hold only
# ============================================================
def setup_hip_position_hold_startup_only(hip_hold_raw):
    """
    Hip setup is intentionally isolated.

    The hip:
    1. gets gains,
    2. enters POSITION mode,
    3. holds its initial raw angle for HIP_STARTUP_HOLD_TIME,
    4. then receives NO MORE write_read_pdo_2 commands.
    """
    print("\nSetting hip to POSITION mode and holding initial raw angle...")
    print(f"Hip startup hold raw target = {hip_hold_raw:.6f}")

    set_gains(
        HIP_ID,
        HIP_HOLD_KP,
        HIP_HOLD_KD,
        HIP_HOLD_TORQUE_LIMIT,
    )

    set_mode_with_spacing(
        HIP_ID,
        recoil.Mode.POSITION,
    )

    for _ in range(int(HIP_STARTUP_HOLD_TIME * RATE_HZ)):
        # This is the ONLY place after startup where HIP_ID receives PDO command.
        bus.write_read_pdo_2(
            HIP_ID,
            hip_hold_raw,
            0.0,
        )
        rate.sleep()

    print("Hip startup hold complete.")
    print("IMPORTANT: From this point onward, HIP_ID will not be commanded again.")


# ============================================================
# Drive startup: thigh + shank only
# ============================================================
def setup_drive_position_mode_and_hold(start_raw):
    """
    Only thigh and shank enter active trajectory control here.
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
            write_drive_position_command(
                motor_id,
                start_raw[motor_id],
            )
            rate.sleep()

    print("Soft holding thigh/shank initial positions...")

    for _ in range(int(DRIVE_STARTUP_HOLD_TIME * RATE_HZ)):
        for motor_id in DRIVE_IDS:
            write_drive_position_command(
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
    # STANCE PHASE
    # Foot on ground.
    # Straight line stroke.
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
    # SWING PHASE
    # Foot lifts and returns in cycloid.
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

    # If z positive is downward, physical lift usually means z decreases.
    z = Z_GROUND + Z_LIFT_SIGN * lift

    return x, y, z


def raw_delta_from_angle_delta(angle_delta):
    return MOTOR_SIGN * angle_delta * GEAR_RATIO


def build_relative_command_table(start_raw):
    """
    Build trajectory commands for thigh and shank only.

    leg_ik returns:
        theta_h, theta_t, theta_s

    theta_h is ignored completely.
    Hip is NOT included in the command table.
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

        # Only thigh and shank are used.
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
        "\nImportant: hip IK angle is computed only for debugging. "
        "No hip motor command exists in the trajectory table."
    )

    return table


# ============================================================
# Smooth move to first point
# ============================================================
def smooth_move_to_first_targets(first_targets, move_time):
    """
    Move thigh and shank to first trajectory point.
    Hip is not commanded here.
    """
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

        # No hip command here.

        for motor_id in DRIVE_IDS:
            cmd = start_raw[motor_id] + (
                target_raw[motor_id] - start_raw[motor_id]
            ) * s

            write_drive_position_command(
                motor_id,
                cmd,
            )

        rate.sleep()


# ============================================================
# Main
# ============================================================
try:
    print("Planar cycloidal trot, NO HOMING, clean hip startup hold only")
    print("Correct IDs: shank=1, thigh=0, hip=2")
    print("Hip holds initial raw encoder angle only during startup.")
    print("After trajectory starts, HIP_ID receives no PDO commands.")
    print("Only thigh and shank receive trajectory commands.")
    print("On exit, all motors go to IDLE.")
    print()
    print("Tuning:")
    print(f"  X_DIRECTION_SIGN = {X_DIRECTION_SIGN}")
    print(f"  Z_LIFT_SIGN      = {Z_LIFT_SIGN}")
    print(f"  CYCLE_TIME       = {CYCLE_TIME} s")
    print(f"  STEP_LENGTH      = {STEP_LENGTH} mm")
    print(f"  STEP_HEIGHT      = {STEP_HEIGHT} mm")
    print(f"  STANCE_RATIO     = {STANCE_RATIO}")
    print(f"  HIP_HOLD_KP      = {HIP_HOLD_KP}")
    print(f"  HIP_HOLD_KD      = {HIP_HOLD_KD}")
    print(f"  HIP_TORQUE_LIMIT = {HIP_HOLD_TORQUE_LIMIT}")
    print()

    # Step 1: Read initial raw encoder positions safely.
    initial_raw = read_initial_positions_safely()

    hip_hold_raw = initial_raw[HIP_ID]

    start_raw = {
        THIGH_ID: initial_raw[THIGH_ID],
        SHANK_ID: initial_raw[SHANK_ID],
    }

    # Step 2: Hip startup hold only.
    setup_hip_position_hold_startup_only(
        hip_hold_raw,
    )

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

        # No hip command here.

        thigh_pos, thigh_vel = write_drive_position_command(
            THIGH_ID,
            point["raw_thigh"],
        )

        shank_pos, shank_vel = write_drive_position_command(
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

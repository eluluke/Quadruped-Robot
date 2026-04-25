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

JOINT_IDS = [HIP_ID, THIGH_ID, SHANK_ID]

MOTOR_NAMES = {
    HIP_ID: "hip",
    THIGH_ID: "thigh",
    SHANK_ID: "shank",
}


# ============================================================
# Gear / sign
# ============================================================
GEAR_RATIO = 17.0

# This matched your working thigh/shank trajectory.
# If all joints move opposite of expected, change to +1.0.
MOTOR_SIGN = -1.0


# ============================================================
# Direction tuning
# ============================================================
# You found the foot direction looks correct with -1.0.
X_DIRECTION_SIGN = -1.0

# Your IK frame has z positive downward.
# For foot lift, z should decrease, so -1.0 is physically "up".
Z_LIFT_SIGN = 1.0


# ============================================================
# Trajectory tuning
# ============================================================
RATE_HZ = 80.0

# Larger = slower. Start slow for 3-DOF test.
CYCLE_TIME = 2.0

X_CENTER = 0.0
Y_PLANE = 84.26
Z_GROUND = 382.0

# Start modest. You found 15-20 mm is stable and 30 mm starts showing hip motion.
STEP_LENGTH = 70.0
STEP_HEIGHT = 50.0

# 0.50 = equal stance / swing
# 0.45 = longer smoother swing
# 0.60 = longer stance, faster swing
STANCE_RATIO = 0.50

# Safety limit: maximum raw motor command change from startup position.
MAX_RAW_DELTA_FROM_START = {
    HIP_ID: 5.0,
    THIGH_ID: 10.0,
    SHANK_ID: 10.0,
}


# ============================================================
# Control tuning
# ============================================================
STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP = {
    HIP_ID: 0.020,
    THIGH_ID: 0.025,
    SHANK_ID: 0.025,
}

MID_KD = {
    HIP_ID: 0.003,
    THIGH_ID: 0.002,
    SHANK_ID: 0.002,
}

MID_TORQUE_LIMIT = {
    HIP_ID: 0.40,
    THIGH_ID: 0.12,
    SHANK_ID: 0.12,
}

RUN_KP = {
    HIP_ID: 0.030,
    THIGH_ID: 0.055,
    SHANK_ID: 0.055,
}

RUN_KD = {
    HIP_ID: 0.006,
    THIGH_ID: 0.003,
    SHANK_ID: 0.003,
}

RUN_TORQUE_LIMIT = {
    HIP_ID: 1.00,
    THIGH_ID: 0.26,
    SHANK_ID: 0.26,
}

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


def set_joint_gains(kp_dict, kd_dict, torque_dict):
    for motor_id in JOINT_IDS:
        set_gains(
            motor_id,
            kp_dict[motor_id],
            kd_dict[motor_id],
            torque_dict[motor_id],
        )


def write_position_command(motor_id, command_pos):
    return bus.write_read_pdo_2(
        motor_id,
        command_pos,
        0.0,
    )


def idle_all_motors():
    print("Putting all motors into IDLE and stopping CAN bus...")

    for motor_id in JOINT_IDS:
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
# Safe initial read
# ============================================================
def read_position_while_idle(motor_id):
    """
    Only use while motor is IDLE.
    write_read_pdo_2 is not a pure read, but while IDLE it should not move.
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

    for motor_id in JOINT_IDS:
        bus.set_mode(motor_id, recoil.Mode.IDLE)
        time.sleep(0.02)

    time.sleep(0.20)

    # Flush stale CAN frames while motors are IDLE.
    for _ in range(5):
        for motor_id in JOINT_IDS:
            try:
                bus.write_read_pdo_2(motor_id, 0.0, 0.0)
            except Exception:
                pass
        rate.sleep()

    raw = {}

    for motor_id in JOINT_IDS:
        samples = []

        for _ in range(15):
            pos = read_position_while_idle(motor_id)
            samples.append(pos)
            rate.sleep()

        samples.sort()
        raw[motor_id] = samples[len(samples) // 2]

    print("Initial raw encoder positions:")
    for motor_id in JOINT_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]} = "
            f"{raw[motor_id]:.6f}"
        )

    return raw


# ============================================================
# Startup hold
# ============================================================
def setup_all_position_mode_and_hold(start_raw):
    print("\nEntering POSITION mode for hip, thigh, and shank...")

    for motor_id in JOINT_IDS:
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

    print("Soft holding all initial positions...")

    for _ in range(int(STARTUP_HOLD_TIME * RATE_HZ)):
        for motor_id in JOINT_IDS:
            write_position_command(
                motor_id,
                start_raw[motor_id],
            )

        rate.sleep()

    print("Startup hold complete.")


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

    # z positive downward, so foot lift means z decreases.
    z = Z_GROUND + Z_LIFT_SIGN * lift

    return x, y, z


def raw_delta_from_angle_delta(angle_delta):
    return MOTOR_SIGN * angle_delta * GEAR_RATIO


def build_relative_command_table(start_raw):
    """
    Build relative 3-DOF commands.

    Unlike the previous planar test, this version uses:
        delta_h, delta_t, delta_s

    If y is constant and leg_ik is planar-consistent, delta_h should be near zero.
    But we still allow IK to define it.
    """
    num_points = int(CYCLE_TIME * RATE_HZ)
    table = []

    theta_h0, theta_t0, theta_s0 = leg_ik(
        X_CENTER,
        Y_PLANE,
        Z_GROUND,
    )

    print("\nNominal IK reference:")
    print(f"  theta_h0 = {theta_h0:.6f}")
    print(f"  theta_t0 = {theta_t0:.6f}")
    print(f"  theta_s0 = {theta_s0:.6f}")

    max_raw_delta = {
        HIP_ID: 0.0,
        THIGH_ID: 0.0,
        SHANK_ID: 0.0,
    }

    max_angle_delta = {
        HIP_ID: 0.0,
        THIGH_ID: 0.0,
        SHANK_ID: 0.0,
    }

    for i in range(num_points):
        phase = i / num_points
        x, y, z = foot_trajectory(phase)

        theta_h, theta_t, theta_s = leg_ik(
            x,
            y,
            z,
        )

        delta_h = theta_h - theta_h0
        delta_t = theta_t - theta_t0
        delta_s = theta_s - theta_s0

        raw_hip = start_raw[HIP_ID] + raw_delta_from_angle_delta(delta_h)
        raw_thigh = start_raw[THIGH_ID] + raw_delta_from_angle_delta(delta_t)
        raw_shank = start_raw[SHANK_ID] + raw_delta_from_angle_delta(delta_s)

        raw_targets = {
            HIP_ID: raw_hip,
            THIGH_ID: raw_thigh,
            SHANK_ID: raw_shank,
        }

        angle_deltas = {
            HIP_ID: delta_h,
            THIGH_ID: delta_t,
            SHANK_ID: delta_s,
        }

        for motor_id in JOINT_IDS:
            raw_delta = raw_targets[motor_id] - start_raw[motor_id]

            max_raw_delta[motor_id] = max(
                max_raw_delta[motor_id],
                abs(raw_delta),
            )

            max_angle_delta[motor_id] = max(
                max_angle_delta[motor_id],
                abs(angle_deltas[motor_id]),
            )

            if abs(raw_delta) > MAX_RAW_DELTA_FROM_START[motor_id]:
                raise RuntimeError(
                    f"{MOTOR_NAMES[motor_id]} command too large: "
                    f"{raw_delta:.3f} raw rad. "
                    f"Reduce STEP_LENGTH or STEP_HEIGHT."
                )

        table.append(
            {
                "phase": phase,
                "x": x,
                "y": y,
                "z": z,

                "theta_h": theta_h,
                "theta_t": theta_t,
                "theta_s": theta_s,

                "delta_h": delta_h,
                "delta_t": delta_t,
                "delta_s": delta_s,

                "raw_hip": raw_hip,
                "raw_thigh": raw_thigh,
                "raw_shank": raw_shank,
            }
        )

    print("\nTrajectory command range:")
    for motor_id in JOINT_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]}: "
            f"max angle delta = {max_angle_delta[motor_id]:.6f}, "
            f"max raw delta = {max_raw_delta[motor_id]:.3f}"
        )

    return table


# ============================================================
# Smooth move to first point
# ============================================================
def smooth_move_to_first_targets(first_targets, move_time):
    start_raw = {
        motor_id: first_targets[motor_id]["start"]
        for motor_id in JOINT_IDS
    }

    target_raw = {
        motor_id: first_targets[motor_id]["target"]
        for motor_id in JOINT_IDS
    }

    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        for motor_id in JOINT_IDS:
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
    print("3-DOF relative IK cycloidal trot, NO HOMING")
    print("Correct IDs: hip=2, thigh=0, shank=1")
    print("Hip, thigh, and shank are treated equally.")
    print("Since y is constant, hip IK delta should theoretically stay near zero.")
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

    # Step 1: Read initial raw encoder positions.
    start_raw = read_initial_positions_safely()

    # Step 2: Enter position mode and hold current raw positions.
    setup_all_position_mode_and_hold(start_raw)

    # Step 3: Build full relative IK trajectory.
    print("\nBuilding 3-DOF relative IK trajectory...")
    command_table = build_relative_command_table(
        start_raw,
    )

    first = command_table[0]

    first_targets = {
        HIP_ID: {
            "start": start_raw[HIP_ID],
            "target": first["raw_hip"],
        },
        THIGH_ID: {
            "start": start_raw[THIGH_ID],
            "target": first["raw_thigh"],
        },
        SHANK_ID: {
            "start": start_raw[SHANK_ID],
            "target": first["raw_shank"],
        },
    }

    # Step 4: Ramp to medium gains.
    print("\nRamping all joints to medium gains...")
    set_joint_gains(
        MID_KP,
        MID_KD,
        MID_TORQUE_LIMIT,
    )

    print("Moving all joints to first trajectory point...")
    smooth_move_to_first_targets(
        first_targets,
        MOVE_TO_START_TIME,
    )

    # Step 5: Run gains.
    print("Switching all joints to run gains...")
    set_joint_gains(
        RUN_KP,
        RUN_KD,
        RUN_TORQUE_LIMIT,
    )

    print("\nStarting 3-DOF relative IK cycloid trot.")
    print("Press Ctrl+C to stop.\n")

    index = 0
    counter = 0

    while True:
        point = command_table[index]

        hip_pos, hip_vel = write_position_command(
            HIP_ID,
            point["raw_hip"],
        )

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
            hip_err = None
            thigh_err = None
            shank_err = None

            if hip_pos is not None:
                hip_err = point["raw_hip"] - hip_pos

            if thigh_pos is not None:
                thigh_err = point["raw_thigh"] - thigh_pos

            if shank_pos is not None:
                shank_err = point["raw_shank"] - shank_pos

            print(
                f"phase={point['phase']:.3f} | "
                f"x={point['x']:.1f} "
                f"y={point['y']:.1f} "
                f"z={point['z']:.1f} | "
                f"dhip={point['delta_h']:.5f} "
                f"dthigh={point['delta_t']:.5f} "
                f"dshank={point['delta_s']:.5f} | "
                f"hip_err={hip_err:.3f} "
                f"thigh_err={thigh_err:.3f} "
                f"shank_err={shank_err:.3f}"
            )

        index += 1

        if index >= len(command_table):
            index = 0

        rate.sleep()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    idle_all_motors()

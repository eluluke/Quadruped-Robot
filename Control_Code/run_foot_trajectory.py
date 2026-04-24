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
# Speed tuning
# ============================================================
RATE_HZ = 80.0

# Larger = slower, smaller = faster.
# Try:
#   4.0 = slow
#   3.0 = medium
#   2.2 = fast
#   1.6 = aggressive
CYCLE_TIME = 1.8


# ============================================================
# Planar cycloid trajectory tuning
# ============================================================
X_CENTER = 0.0
Y_PLANE = 84.26
Z_GROUND = 382.0

# Bigger step.
# For semi-circle-like swing, STEP_HEIGHT should be roughly
# half of STEP_LENGTH, or slightly larger.
STEP_LENGTH = 150
STEP_HEIGHT = 85.0

# Meaning:
#   0.50 = half cycle on ground, half cycle in air
#   0.60 = longer ground stroke, faster air swing
#   0.45 = slightly longer air swing, smoother arc
# For your semi-circle-looking lifted swing, 0.45-0.50 is good.
STANCE_RATIO = 0.45

# Safety limit: maximum raw motor command change from startup position.
# If this triggers, reduce STEP_LENGTH or STEP_HEIGHT.
MAX_RAW_DELTA_FROM_START = 13.0


# ============================================================
# Control tuning for thigh + shank
# ============================================================
STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP = 0.025
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.12

RUN_KP = 0.070
RUN_KD = 0.002
RUN_TORQUE_LIMIT = 0.34

MOVE_TO_START_TIME = 2.0

PRINT_EVERY = 20


# ============================================================
# Hip position-hold tuning
# ============================================================
# This is now POSITION mode, not DAMPING.
# The script reads the hip raw position at startup and holds it.
HIP_HOLD_KP = 0.080
HIP_HOLD_KD = 0.005
HIP_HOLD_TORQUE_LIMIT = 1.5

# If hip still sags:
#   HIP_HOLD_KP = 0.080
#   HIP_HOLD_TORQUE_LIMIT = 1.50
#
# If hip vibrates:
#   HIP_HOLD_KP = 0.040
#   HIP_HOLD_KD = 0.002
#   HIP_HOLD_TORQUE_LIMIT = 0.90


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
    time.sleep(0.006)
    bus.feed(motor_id)
    time.sleep(0.006)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_drive_gains(kp, kd, torque_limit):
    for motor_id in DRIVE_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def read_raw_position(motor_id):
    pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")

    return pos


def read_drive_raw_positions():
    return {
        motor_id: read_raw_position(motor_id)
        for motor_id in DRIVE_IDS
    }


def sync_reference(motor_id, sync_time=0.35):
    current_pos = None
    steps = int(sync_time * RATE_HZ)

    for _ in range(steps):
        pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(motor_id, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync {MOTOR_NAMES[motor_id]}")

    return current_pos


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


def smooth_move_to_targets(raw_targets, move_time, hip_hold_raw=None):
    start_raw = read_drive_raw_positions()
    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        # Keep hip locked during transition.
        if hip_hold_raw is not None:
            bus.write_read_pdo_2(HIP_ID, hip_hold_raw, 0.0)

        for motor_id in DRIVE_IDS:
            cmd = start_raw[motor_id] + (
                raw_targets[motor_id] - start_raw[motor_id]
            ) * s

            bus.write_read_pdo_2(motor_id, cmd, 0.0)

        rate.sleep()


# ============================================================
# Hip position hold
# ============================================================
def setup_hip_position_hold():
    print("Setting hip to POSITION hold mode...")

    set_gains(
        HIP_ID,
        HIP_HOLD_KP,
        HIP_HOLD_KD,
        HIP_HOLD_TORQUE_LIMIT,
    )

    set_mode_with_spacing(HIP_ID, recoil.Mode.POSITION)

    hip_raw = sync_reference(HIP_ID, sync_time=0.35)

    print(
        f"Hip holding initial raw position: {hip_raw:.6f} "
        f"with kp={HIP_HOLD_KP}, kd={HIP_HOLD_KD}, "
        f"torque_limit={HIP_HOLD_TORQUE_LIMIT}"
    )

    return hip_raw


# ============================================================
# Trajectory and relative IK
# ============================================================
def foot_trajectory(phase):
    phase = phase % 1.0

    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO

        # Stance phase:
        # foot pulls backward in a straight line.
        x = X_CENTER + STEP_LENGTH / 2.0 - STEP_LENGTH * u
        y = Y_PLANE
        z = Z_GROUND

        return x, y, z

    # Swing phase:
    # foot returns forward in a cycloid curve.
    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    x = X_CENTER - STEP_LENGTH / 2.0 + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    y = Y_PLANE

    # z axis points downward, so lifting foot means z decreases.
    z = Z_GROUND - STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    return x, y, z


def raw_delta_from_angle_delta(angle_delta):
    return MOTOR_SIGN * angle_delta * GEAR_RATIO


def build_relative_command_table(start_raw):
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

    max_thigh_delta = 0.0
    max_shank_delta = 0.0

    for i in range(num_points):
        phase = i / num_points
        x, y, z = foot_trajectory(phase)

        theta_h, theta_t, theta_s = leg_ik(x, y, z)

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

    return table


# ============================================================
# Main
# ============================================================
try:
    print("Planar cycloidal trotting test, NO HOMING, HIP POSITION HOLD")
    print("Correct IDs: shank=1, thigh=0, hip=2")
    print("Uses current raw motor positions as hardware starting point.")
    print("Uses leg_ik only to generate relative thigh/shank motion.")
    print("Hip holds its initial raw position in POSITION mode.")
    print("On exit, all motors go to IDLE, not DAMPING.")
    print()
    print("Tuning:")
    print(f"  CYCLE_TIME  = {CYCLE_TIME} s")
    print(f"  STEP_LENGTH = {STEP_LENGTH} mm")
    print(f"  STEP_HEIGHT = {STEP_HEIGHT} mm")
    print(f"  STANCE_RATIO = {STANCE_RATIO}")
    print(f"  HIP_HOLD_KP = {HIP_HOLD_KP}")
    print(f"  HIP_HOLD_KD = {HIP_HOLD_KD}")
    print(f"  HIP_HOLD_TORQUE_LIMIT = {HIP_HOLD_TORQUE_LIMIT}")
    print()

    # --------------------------------------------------------
    # Hip enters position hold first.
    # --------------------------------------------------------
    hip_hold_raw = setup_hip_position_hold()

    # --------------------------------------------------------
    # Thigh + shank enter soft position mode.
    # --------------------------------------------------------
    for motor_id in DRIVE_IDS:
        set_gains(
            motor_id,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    print("\nSyncing thigh/shank current references...")
    for motor_id in DRIVE_IDS:
        synced = sync_reference(motor_id)
        print(f"  {MOTOR_NAMES[motor_id]} synced raw={synced:.6f}")

    start_raw = read_drive_raw_positions()

    print("\nHardware starting raw positions:")
    print(f"  hip hold raw = {hip_hold_raw:.6f}")
    for motor_id in DRIVE_IDS:
        print(f"  {MOTOR_NAMES[motor_id]} = {start_raw[motor_id]:.6f}")

    print("\nHolding current positions softly...")
    for _ in range(int(0.5 * RATE_HZ)):
        bus.write_read_pdo_2(HIP_ID, hip_hold_raw, 0.0)

        for motor_id in DRIVE_IDS:
            bus.write_read_pdo_2(motor_id, start_raw[motor_id], 0.0)

        rate.sleep()

    print("Building relative IK trajectory...")
    command_table = build_relative_command_table(start_raw)

    first = command_table[0]
    first_targets = {
        THIGH_ID: first["raw_thigh"],
        SHANK_ID: first["raw_shank"],
    }

    print("Ramping thigh/shank to medium gains...")
    set_drive_gains(MID_KP, MID_KD, MID_TORQUE_LIMIT)

    print("Moving to first relative trajectory point...")
    smooth_move_to_targets(
        first_targets,
        MOVE_TO_START_TIME,
        hip_hold_raw=hip_hold_raw,
    )

    print("Switching thigh/shank to run gains...")
    set_drive_gains(RUN_KP, RUN_KD, RUN_TORQUE_LIMIT)

    print("\nStarting no-homing planar cycloid trot with hip position hold.")
    print("Press Ctrl+C to stop.\n")

    index = 0
    counter = 0

    while True:
        point = command_table[index]

        # Actively hold hip at its initial position.
        hip_pos, hip_vel = bus.write_read_pdo_2(
            HIP_ID,
            hip_hold_raw,
            0.0,
        )

        thigh_pos, thigh_vel = bus.write_read_pdo_2(
            THIGH_ID,
            point["raw_thigh"],
            0.0,
        )

        shank_pos, shank_vel = bus.write_read_pdo_2(
            SHANK_ID,
            point["raw_shank"],
            0.0,
        )

        counter += 1

        if counter % PRINT_EVERY == 0:
            print(
                f"x={point['x']:.1f} "
                f"z={point['z']:.1f} | "
                f"dtheta_t={point['delta_t']:.4f} "
                f"dtheta_s={point['delta_s']:.4f} | "
                f"raw_t={point['raw_thigh']:.3f} "
                f"raw_s={point['raw_shank']:.3f} | "
                f"hip_hold={hip_hold_raw:.3f}"
            )

        index += 1
        if index >= len(command_table):
            index = 0

        rate.sleep()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    idle_all_motors()
# If this triggers, reduce STEP_LENGTH or STEP_HEIGHT.
MAX_RAW_DELTA_FROM_START = 11.0


# ============================================================
# Control tuning for thigh + shank
# ============================================================
STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP = 0.025
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.12

RUN_KP = 0.065
RUN_KD = 0.002
RUN_TORQUE_LIMIT = 0.30

MOVE_TO_START_TIME = 2.0

PRINT_EVERY = 20


# ============================================================
# Hip damping hold tuning
# ============================================================
# In damping mode, KD and torque limit matter most.
# If hip still sags, increase HIP_HOLD_TORQUE_LIMIT first.
HIP_HOLD_KP = 0.000
HIP_HOLD_KD = 0.08
HIP_HOLD_TORQUE_LIMIT = 2

# More aggressive options:
# HIP_HOLD_KD = 0.030
# HIP_HOLD_TORQUE_LIMIT = 1.00


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
    time.sleep(0.006)
    bus.feed(motor_id)
    time.sleep(0.006)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_drive_gains(kp, kd, torque_limit):
    for motor_id in DRIVE_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def set_hip_damping_hold():
    print(
        f"Setting hip to DAMPING hold: "
        f"kd={HIP_HOLD_KD}, "
        f"torque_limit={HIP_HOLD_TORQUE_LIMIT}"
    )

    set_gains(
        HIP_ID,
        HIP_HOLD_KP,
        HIP_HOLD_KD,
        HIP_HOLD_TORQUE_LIMIT,
    )

    set_mode_with_spacing(HIP_ID, recoil.Mode.DAMPING)


def read_raw_position(motor_id):
    pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")

    return pos


def read_drive_raw_positions():
    return {
        motor_id: read_raw_position(motor_id)
        for motor_id in DRIVE_IDS
    }


def sync_reference(motor_id, sync_time=0.35):
    current_pos = None
    steps = int(sync_time * RATE_HZ)

    for _ in range(steps):
        pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(motor_id, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync {MOTOR_NAMES[motor_id]}")

    return current_pos


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


def smooth_move_to_targets(raw_targets, move_time):
    start_raw = read_drive_raw_positions()
    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        for motor_id in DRIVE_IDS:
            cmd = start_raw[motor_id] + (
                raw_targets[motor_id] - start_raw[motor_id]
            ) * s

            bus.write_read_pdo_2(motor_id, cmd, 0.0)

        rate.sleep()


# ============================================================
# Trajectory and relative IK
# ============================================================
def foot_trajectory(phase):
    phase = phase % 1.0

    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO

        # stance: straight backward pull
        x = X_CENTER + STEP_LENGTH / 2.0 - STEP_LENGTH * u
        y = Y_PLANE
        z = Z_GROUND

        return x, y, z

    # swing: cycloid forward return
    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    x = X_CENTER - STEP_LENGTH / 2.0 + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    y = Y_PLANE

    # z axis points downward, so lifting means z decreases
    z = Z_GROUND - STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    return x, y, z


def raw_delta_from_angle_delta(angle_delta):
    return MOTOR_SIGN * angle_delta * GEAR_RATIO


def build_relative_command_table(start_raw):
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

    max_thigh_delta = 0.0
    max_shank_delta = 0.0

    for i in range(num_points):
        phase = i / num_points
        x, y, z = foot_trajectory(phase)

        theta_h, theta_t, theta_s = leg_ik(x, y, z)

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

    return table


# ============================================================
# Main
# ============================================================
try:
    print("Bigger planar cycloidal trotting test, NO HOMING")
    print("Correct IDs: shank=1, thigh=0, hip=2")
    print("Uses current raw motor positions as hardware starting point.")
    print("Uses leg_ik only to generate relative thigh/shank motion.")
    print("Hip is held in DAMPING with higher holding torque.")
    print("On exit, all motors go to IDLE, not DAMPING.")
    print()
    print("Tuning:")
    print(f"  CYCLE_TIME  = {CYCLE_TIME} s")
    print(f"  STEP_LENGTH = {STEP_LENGTH} mm")
    print(f"  STEP_HEIGHT = {STEP_HEIGHT} mm")
    print(f"  HIP_HOLD_KD = {HIP_HOLD_KD}")
    print(f"  HIP_HOLD_TORQUE_LIMIT = {HIP_HOLD_TORQUE_LIMIT}")
    print()

    # Hip hold: damping mode with high damping torque.
    set_hip_damping_hold()

    # Thigh + shank enter soft position mode.
    for motor_id in DRIVE_IDS:
        set_gains(
            motor_id,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    print("Syncing current references...")
    for motor_id in DRIVE_IDS:
        synced = sync_reference(motor_id)
        print(f"  {MOTOR_NAMES[motor_id]} synced raw={synced:.6f}")

    start_raw = read_drive_raw_positions()

    print("\nHardware starting raw positions:")
    for motor_id in DRIVE_IDS:
        print(f"  {MOTOR_NAMES[motor_id]} = {start_raw[motor_id]:.6f}")

    print("\nHolding current thigh/shank position softly...")
    for _ in range(int(0.5 * RATE_HZ)):
        # Keep hip damping alive.
        bus.feed(HIP_ID)

        for motor_id in DRIVE_IDS:
            bus.write_read_pdo_2(motor_id, start_raw[motor_id], 0.0)

        rate.sleep()

    print("Building bigger relative IK trajectory...")
    command_table = build_relative_command_table(start_raw)

    first = command_table[0]
    first_targets = {
        THIGH_ID: first["raw_thigh"],
        SHANK_ID: first["raw_shank"],
    }

    print("Ramping thigh/shank to medium gains...")
    set_drive_gains(MID_KP, MID_KD, MID_TORQUE_LIMIT)

    print("Moving to first relative trajectory point...")
    smooth_move_to_targets(first_targets, MOVE_TO_START_TIME)

    print("Switching thigh/shank to run gains...")
    set_drive_gains(RUN_KP, RUN_KD, RUN_TORQUE_LIMIT)

    print("\nStarting bigger no-homing planar cycloid trot.")
    print("Press Ctrl+C to stop.\n")

    index = 0
    counter = 0

    while True:
        point = command_table[index]

        # Keep hip damping alive.
        bus.feed(HIP_ID)

        thigh_pos, thigh_vel = bus.write_read_pdo_2(
            THIGH_ID,
            point["raw_thigh"],
            0.0,
        )

        shank_pos, shank_vel = bus.write_read_pdo_2(
            SHANK_ID,
            point["raw_shank"],
            0.0,
        )

        counter += 1

        if counter % PRINT_EVERY == 0:
            print(
                f"x={point['x']:.1f} "
                f"z={point['z']:.1f} | "
                f"dtheta_t={point['delta_t']:.4f} "
                f"dtheta_s={point['delta_s']:.4f} | "
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
# This is only used to calculate delta angles.
# Current motor position is still used as hardware starting point.
# ============================================================
X_CENTER = 0.0
Y_PLANE = 84.26
Z_GROUND = 380.0

STEP_LENGTH = 100.0
STEP_HEIGHT = 80.0
STANCE_RATIO = 0.50

# Safety limit: maximum raw motor command change from startup position.
# This protects against IK sign mistakes.
MAX_RAW_DELTA_FROM_START = 8.0


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
    time.sleep(0.006)
    bus.feed(motor_id)
    time.sleep(0.006)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_drive_gains(kp, kd, torque_limit):
    for motor_id in DRIVE_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def read_raw_position(motor_id):
    pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)
    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")
    return pos


def read_drive_raw_positions():
    return {
        motor_id: read_raw_position(motor_id)
        for motor_id in DRIVE_IDS
    }


def sync_reference(motor_id, sync_time=0.35):
    current_pos = None
    steps = int(sync_time * RATE_HZ)

    for _ in range(steps):
        pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(motor_id, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync {MOTOR_NAMES[motor_id]}")

    return current_pos


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


def smooth_move_to_targets(raw_targets, move_time):
    start_raw = read_drive_raw_positions()
    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        for motor_id in DRIVE_IDS:
            cmd = start_raw[motor_id] + (
                raw_targets[motor_id] - start_raw[motor_id]
            ) * s

            bus.write_read_pdo_2(motor_id, cmd, 0.0)

        rate.sleep()


# ============================================================
# Trajectory and relative IK
# ============================================================
def foot_trajectory(phase):
    phase = phase % 1.0

    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO

        # stance: straight backward pull
        x = X_CENTER + STEP_LENGTH / 2.0 - STEP_LENGTH * u
        y = Y_PLANE
        z = Z_GROUND

        return x, y, z

    # swing: cycloid forward return
    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    x = X_CENTER - STEP_LENGTH / 2.0 + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    y = Y_PLANE

    z = Z_GROUND - STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    return x, y, z


def raw_delta_from_angle_delta(angle_delta):
    return MOTOR_SIGN * angle_delta * GEAR_RATIO


def build_relative_command_table(start_raw):
    num_points = int(CYCLE_TIME * RATE_HZ)
    table = []

    # Nominal reference IK angle.
    theta_h0, theta_t0, theta_s0 = leg_ik(
        X_CENTER,
        Y_PLANE,
        Z_GROUND,
    )

    for i in range(num_points):
        phase = i / num_points
        x, y, z = foot_trajectory(phase)

        theta_h, theta_t, theta_s = leg_ik(x, y, z)

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

        table.append(
            {
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

    return table


# ============================================================
# Main
# ============================================================
try:
    print("Planar cycloidal trotting test, NO HOMING")
    print("Correct IDs: shank=1, thigh=0, hip=2")
    print("Uses current raw motor positions as hardware starting point.")
    print("Uses leg_ik only to generate relative thigh/shank motion.")
    print("Hip stays IDLE.")
    print("On exit, all motors go to IDLE, not DAMPING.")
    print()

    set_mode_with_spacing(HIP_ID, recoil.Mode.IDLE)

    for motor_id in DRIVE_IDS:
        set_gains(
            motor_id,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    print("Syncing current references...")
    for motor_id in DRIVE_IDS:
        synced = sync_reference(motor_id)
        print(f"  {MOTOR_NAMES[motor_id]} synced raw={synced:.6f}")

    start_raw = read_drive_raw_positions()

    print("\nHardware starting raw positions:")
    for motor_id in DRIVE_IDS:
        print(f"  {MOTOR_NAMES[motor_id]} = {start_raw[motor_id]:.6f}")

    print("\nHolding current position softly...")
    for _ in range(int(0.5 * RATE_HZ)):
        for motor_id in DRIVE_IDS:
            bus.write_read_pdo_2(motor_id, start_raw[motor_id], 0.0)
        rate.sleep()

    print("Building relative IK trajectory...")
    command_table = build_relative_command_table(start_raw)

    first = command_table[0]
    first_targets = {
        THIGH_ID: first["raw_thigh"],
        SHANK_ID: first["raw_shank"],
    }

    print("Ramping to medium gains...")
    set_drive_gains(MID_KP, MID_KD, MID_TORQUE_LIMIT)

    print("Moving to first relative trajectory point...")
    smooth_move_to_targets(first_targets, MOVE_TO_START_TIME)

    print("Switching to run gains...")
    set_drive_gains(RUN_KP, RUN_KD, RUN_TORQUE_LIMIT)

    print("\nStarting no-homing planar cycloid trot.")
    print("Press Ctrl+C to stop.\n")

    index = 0
    counter = 0

    while True:
        point = command_table[index]

        thigh_pos, thigh_vel = bus.write_read_pdo_2(
            THIGH_ID,
            point["raw_thigh"],
            0.0,
        )

        shank_pos, shank_vel = bus.write_read_pdo_2(
            SHANK_ID,
            point["raw_shank"],
            0.0,
        )

        counter += 1

        if counter % PRINT_EVERY == 0:
            print(
                f"x={point['x']:.1f} "
                f"z={point['z']:.1f} | "
                f"dtheta_t={point['delta_t']:.4f} "
                f"dtheta_s={point['delta_s']:.4f} | "
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
MAX_STARTUP_TIME = 6.0

PRINT_EVERY = 20


# --------------------------------
# Trajectory
# stance = straight pull-back
# swing = cycloid
# --------------------------------
def foot_trajectory(phase):
    phase = phase % 1.0

    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO
        x = X_CENTER + (STEP_LENGTH / 2.0) - STEP_LENGTH * u
        z = Z_GROUND
        return x, z

    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    x = X_CENTER - (STEP_LENGTH / 2.0) + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )
    z = Z_GROUND + STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    return x, z


# --------------------------------
# IK wrapper
# keep planar motion: y = 0
# --------------------------------
def planar_leg_ik(x, z):
    theta_h, theta_t, theta_s = leg_ik(x, 0.0, z)
    return theta_h, theta_t, theta_s


# --------------------------------
# Set modes / gains
# --------------------------------
def set_mode_with_spacing(jid, mode):
    bus.set_mode(jid, mode)
    time.sleep(0.003)
    bus.feed(jid)
    time.sleep(0.003)


def set_gains(jid, kp_val, kd_val, torque_val):
    bus.write_position_kp(jid, kp_val)
    time.sleep(0.002)
    bus.write_position_kd(jid, kd_val)
    time.sleep(0.002)
    bus.write_torque_limit(jid, torque_val)
    time.sleep(0.002)


# --------------------------------
# Sync current reference for a joint
# Only used for thigh + shank in position mode
# --------------------------------
def sync_reference(jid, sync_time=0.25):
    steps = int(sync_time * RATE_HZ)
    current_pos = None

    for _ in range(steps):
        pos, vel = bus.write_read_pdo_2(jid, 0.0, 0.0)
        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(jid, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync joint {jid}")

    return current_pos


# --------------------------------
# Stable startup
# Only require THIGH + SHANK stable
# Hip stays in damping and is not allowed to block startup
# --------------------------------
def get_stable_start_positions():
    print("Entering damping mode for all joints...")

    for jid in [HIP_ID, THIGH_ID, SHANK_ID]:
        set_mode_with_spacing(jid, recoil.Mode.DAMPING)

    time.sleep(0.1)

    stable_count = 0
    last_hip = None
    last_thigh = None
    last_shank = None

    t_start = time.perf_counter()

    while True:
        hip_pos, hip_vel = bus.write_read_pdo_2(HIP_ID, 0.0, 0.0)
        thigh_pos, thigh_vel = bus.write_read_pdo_2(THIGH_ID, 0.0, 0.0)
        shank_pos, shank_vel = bus.write_read_pdo_2(SHANK_ID, 0.0, 0.0)

        if None not in [
            hip_pos, hip_vel,
            thigh_pos, thigh_vel,
            shank_pos, shank_vel,
        ]:
            last_hip = hip_pos
            last_thigh = thigh_pos
            last_shank = shank_pos

            if (
                abs(thigh_vel) < STABLE_VELOCITY_THRESHOLD
                and abs(shank_vel) < STABLE_VELOCITY_THRESHOLD
            ):
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= REQUIRED_STABLE_SAMPLES:
                return last_hip, last_thigh, last_shank

        if time.perf_counter() - t_start > MAX_STARTUP_TIME:
            raise RuntimeError(
                "Startup not stable "
                f"(hip_vel={hip_vel}, thigh_vel={thigh_vel}, shank_vel={shank_vel})"
            )

        rate.sleep()


# --------------------------------
# Build one full discrete cycle:
# foot points -> IK -> motor commands
# --------------------------------
def build_command_table(start_thigh, start_shank):
    num_points = int(CYCLE_TIME * RATE_HZ)

    x0, z0 = foot_trajectory(0.0)
    _, theta_t_ref0, theta_s_ref0 = planar_leg_ik(x0, z0)

    print(
        f"Ref angles: thigh={theta_t_ref0:.3f}, shank={theta_s_ref0:.3f}"
    )

    table = []

    for k in range(num_points):
        phase = k / num_points
        x_des, z_des = foot_trajectory(phase)

        _, theta_t_des, theta_s_des = planar_leg_ik(x_des, z_des)

        cmd_thigh = start_thigh + (theta_t_des - theta_t_ref0) * GEAR_RATIO
        cmd_shank = start_shank + (theta_s_des - theta_s_ref0) * GEAR_RATIO

        table.append(
            {
                "x": x_des,
                "z": z_des,
                "cmd_thigh": cmd_thigh,
                "cmd_shank": cmd_shank,
            }
        )

    return table


# --------------------------------
# Main
# --------------------------------
try:
    start_hip, start_thigh, start_shank = get_stable_start_positions()

    print(
        f"Startup motor angles: hip={start_hip:.3f}, "
        f"thigh={start_thigh:.3f}, shank={start_shank:.3f}"
    )

    # Hip remains in damping only
    set_mode_with_spacing(HIP_ID, recoil.Mode.DAMPING)

    # Thigh + shank enter position mode softly
    for jid in [THIGH_ID, SHANK_ID]:
        set_gains(
            jid,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )
        set_mode_with_spacing(jid, recoil.Mode.POSITION)

    time.sleep(0.05)

    # Sync actual reference after entering position mode
    start_thigh = sync_reference(THIGH_ID, sync_time=0.25)
    start_shank = sync_reference(SHANK_ID, sync_time=0.25)

    # Soft hold
    for _ in range(int(0.5 * RATE_HZ)):
        bus.write_read_pdo_2(THIGH_ID, start_thigh, 0.0)
        bus.write_read_pdo_2(SHANK_ID, start_shank, 0.0)
        rate.sleep()

    # Medium gains
    for jid in [THIGH_ID, SHANK_ID]:
        set_gains(
            jid,
            MID_KP,
            MID_KD,
            MID_TORQUE_LIMIT,
        )

    for _ in range(int(0.4 * RATE_HZ)):
        bus.write_read_pdo_2(THIGH_ID, start_thigh, 0.0)
        bus.write_read_pdo_2(SHANK_ID, start_shank, 0.0)
        rate.sleep()

    # Final gains
    for jid in [THIGH_ID, SHANK_ID]:
        set_gains(jid, KP, KD, TORQUE_LIMIT)

    # Precompute discrete trajectory commands
    command_table = build_command_table(start_thigh, start_shank)

    print("Starting discrete foot trajectory...")
    counter = 0
    idx = 0

    while True:
        point = command_table[idx]

        thigh_pos, thigh_vel = bus.write_read_pdo_2(
            THIGH_ID,
            point["cmd_thigh"],
            0.0,
        )
        shank_pos, shank_vel = bus.write_read_pdo_2(
            SHANK_ID,
            point["cmd_shank"],
            0.0,
        )

        counter += 1
        if counter % PRINT_EVERY == 0:
            print(
                f"x={point['x']:.1f} z={point['z']:.1f} | "
                f"th_cmd={point['cmd_thigh']:.3f} th_pos={thigh_pos:.3f} | "
                f"sh_cmd={point['cmd_shank']:.3f} sh_pos={shank_pos:.3f}"
            )

        idx += 1
        if idx >= len(command_table):
            idx = 0

        rate.sleep()

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    # Clean shutdown so the next startup is not polluted
    try:
        for jid in [THIGH_ID, SHANK_ID]:
            try:
                set_mode_with_spacing(jid, recoil.Mode.IDLE)
            except Exception:
                pass

        try:
            set_mode_with_spacing(HIP_ID, recoil.Mode.IDLE)
        except Exception:
            pass

        time.sleep(0.05)
    finally:
        try:
            bus.stop()
        except Exception:
            pass

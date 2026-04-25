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
# If the foot trajectory is physically flipped:
#   straight line forward + cycloid backward
# then change this from 1.0 to -1.0.
X_DIRECTION_SIGN = 1.0


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
CYCLE_TIME = 2.8


# ============================================================
# Planar cycloid trajectory tuning
# ============================================================
X_CENTER = 0.0
Y_PLANE = 84.26
Z_GROUND = 382.0

# Bigger step.
# For semi-circle-like swing, STEP_HEIGHT should be around
# half of STEP_LENGTH.
STEP_LENGTH = 80.0
STEP_HEIGHT = 45.0

# Meaning:
#   0.50 = half cycle on ground, half cycle in air
#   0.60 = longer ground stroke, faster air swing
#   0.45 = slightly longer air swing, smoother arc
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
# Hip is held in POSITION mode at its startup position.
HIP_HOLD_KP = 0.080
HIP_HOLD_KD = 0.005
HIP_HOLD_TORQUE_LIMIT = 1.50

# If hip still sags:
#   HIP_HOLD_KP = 0.100
#   HIP_HOLD_KD = 0.006
#   HIP_HOLD_TORQUE_LIMIT = 1.80
#
# If hip vibrates:
#   HIP_HOLD_KP = 0.050
#   HIP_HOLD_KD = 0.003
#   HIP_HOLD_TORQUE_LIMIT = 1.00


# ============================================================
# Startup hold tuning
# ============================================================
# To prevent the leg from jumping at startup, we first enter
# very soft position mode and repeatedly command the current raw position.
STARTUP_HOLD_TIME = 1.0


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


def read_all_raw_positions():
    return {
        motor_id: read_raw_position(motor_id)
        for motor_id in ALL_IDS
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


def hold_current_positions_softly(raw_positions, hold_time):
    steps = int(hold_time * RATE_HZ)

    for _ in range(steps):
        for motor_id, raw_pos in raw_positions.items():
            bus.write_read_pdo_2(motor_id, raw_pos, 0.0)

        rate.sleep()


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

    # --------------------------------------------------------
    # Stance phase:
    # foot is on the ground and moves backward relative to body.
    # --------------------------------------------------------
    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO

        x = X_CENTER + STEP_LENGTH / 2.0 - STEP_LENGTH * u
        y = Y_PLANE
        z = Z_GROUND

        return X_DIRECTION_SIGN * x, y, z

    # --------------------------------------------------------
    # Swing phase:
    # foot lifts and returns forward in a cycloid curve.
    # --------------------------------------------------------
    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    x = X_CENTER - STEP_LENGTH / 2.0 + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    y = Y_PLANE

    # z axis points downward, so lifting foot means z decreases.
    z = Z_GROUND - STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    return X_DIRECTION_SIGN * x, y, z


def raw_delta_from_angle_delta(angle_delta):
    return MOTOR_SIGN * angle_delta * GEAR_RATIO


def build_relative_command_table(start_raw):
    num_points = int(CYCLE_TIME * RATE_HZ)
    table = []

    # Nominal IK reference point.
    theta_h0, theta_t0, theta_s0 = leg_ik(
        X_DIRECTION_SIGN * X_CENTER,
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

        # Planar test: only thigh and shank use IK deltas.
        # Hip is held at its initial raw position.
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
    print(f"  X_DIRECTION_SIGN = {X_DIRECTION_SIGN}")
    print(f"  CYCLE_TIME       = {CYCLE_TIME} s")
    print(f"  STEP_LENGTH      = {STEP_LENGTH} mm")
    print(f"  STEP_HEIGHT      = {STEP_HEIGHT} mm")
    print(f"  STANCE_RATIO     = {STANCE_RATIO}")
    print(f"  HIP_HOLD_KP      = {HIP_HOLD_KP}")
    print(f"  HIP_HOLD_KD      = {HIP_HOLD_KD}")
    print(f"  HIP_TORQUE_LIMIT = {HIP_HOLD_TORQUE_LIMIT}")
    print()

    # --------------------------------------------------------
    # Enter soft position mode for all joints first.
    # This reduces startup jump.
    # --------------------------------------------------------
    print("Entering soft startup position mode...")

    for motor_id in ALL_IDS:
        if motor_id == HIP_ID:
            set_gains(
                motor_id,
                STARTUP_KP,
                STARTUP_KD,
                STARTUP_TORQUE_LIMIT,
            )
        else:
            set_gains(
                motor_id,
                STARTUP_KP,
                STARTUP_KD,
                STARTUP_TORQUE_LIMIT,
            )

        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    print("Syncing all current references...")
    initial_raw = {}

    for motor_id in ALL_IDS:
        synced = sync_reference(motor_id, sync_time=0.35)
        initial_raw[motor_id] = synced
        print(f"  {MOTOR_NAMES[motor_id]} synced raw={synced:.6f}")

    print("\nSoft holding initial positions to prevent startup jump...")
    hold_current_positions_softly(initial_raw, STARTUP_HOLD_TIME)

    # --------------------------------------------------------
    # Hip enters stronger position hold at its initial raw pos.
    # --------------------------------------------------------
    hip_hold_raw = initial_raw[HIP_ID]

    print("\nSwitching hip to stronger position hold...")
    set_gains(
        HIP_ID,
        HIP_HOLD_KP,
        HIP_HOLD_KD,
        HIP_HOLD_TORQUE_LIMIT,
    )
    set_mode_with_spacing(HIP_ID, recoil.Mode.POSITION)

    # Keep hip held for a short moment after gain increase.
    for _ in range(int(0.4 * RATE_HZ)):
        bus.write_read_pdo_2(HIP_ID, hip_hold_raw, 0.0)

        for motor_id in DRIVE_IDS:
            bus.write_read_pdo_2(motor_id, initial_raw[motor_id], 0.0)

        rate.sleep()

    start_raw = {
        THIGH_ID: initial_raw[THIGH_ID],
        SHANK_ID: initial_raw[SHANK_ID],
    }

    print("\nHardware starting raw positions:")
    print(f"  hip hold raw = {hip_hold_raw:.6f}")
    for motor_id in DRIVE_IDS:
        print(f"  {MOTOR_NAMES[motor_id]} = {start_raw[motor_id]:.6f}")

    print("\nBuilding relative IK trajectory...")
    command_table = build_relative_command_table(start_raw)

    first = command_table[0]
    first_targets = {
        THIGH_ID: first["raw_thigh"],
        SHANK_ID: first["raw_shank"],
    }

    print("\nRamping thigh/shank to medium gains...")
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

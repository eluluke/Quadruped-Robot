"""
stand_then_locomotion_v3_modular.py

Automatic sequence:
    1. Start from known max-contraction / mechanical-limit pose.
    2. Move to neutral standing using relative IK deltas.
    3. Hold neutral briefly.
    4. Import trajectory_v3.py.
    5. Build selected trajectory table using trajectory_v3.py.
    6. Run that trajectory relative to the neutral raw pose.

Why this version exists
-----------------------
stand_then_planar_v2 worked, but it had the planar trajectory math written
inside the same file.

This version uses the modular trajectory helper style from locomotion_v2.py:

    from trajectory_v3 import (
        ROLE_HIP, ROLE_THIGH, ROLE_SHANK, JOINT_ROLES,
        TrajectoryConfig, MotorConversionConfig,
        build_relative_command_table,
        summarize_command_table,
        raw_targets_by_id_from_start_raw,
    )

So now you can switch trajectory by changing:

    TRAJECTORY_NAME = "regular_planar"
    TRAJECTORY_NAME = "tilted_planar"
    TRAJECTORY_NAME = "vertical_jump"

The standing stage does NOT use homing_offsets.py.
It assumes the leg starts at the known max-contraction pose.

Run:
    python3 stand_then_locomotion_v3_modular.py -c can1

Before running:
    Put the leg physically at the known max-contraction / mechanical-limit pose.
"""

from __future__ import annotations

import time
import math
import signal
from typing import Dict, Tuple

from loop_rate_limiters import RateLimiter  # type: ignore[import-not-found]
import berkeley_humanoid_lite_lowlevel.recoil as recoil  # type: ignore[import-not-found]
from quadruped_leg_ik import leg_ik

from trajectory_v3 import (
    ROLE_HIP,
    ROLE_THIGH,
    ROLE_SHANK,
    JOINT_ROLES,
    Point,
    TrajectoryConfig,
    MotorConversionConfig,
    build_relative_command_table,
    summarize_command_table,
    raw_targets_by_id_from_start_raw,
)


# ============================================================
# User-switchable trajectory selection
# ============================================================
# Choose:
#   "regular_planar"
#   "tilted_planar"
#   "vertical_jump"
TRAJECTORY_NAME = "regular_planar"


# ============================================================
# CAN IDs
# ============================================================

ROLE_TO_ID: Dict[str, int] = {
    ROLE_HIP: 1,
    ROLE_THIGH: 2,
    ROLE_SHANK: 3,
}

HIP_ID = ROLE_TO_ID[ROLE_HIP]
THIGH_ID = ROLE_TO_ID[ROLE_THIGH]
SHANK_ID = ROLE_TO_ID[ROLE_SHANK]

ID_TO_ROLE = {motor_id: role for role, motor_id in ROLE_TO_ID.items()}

MOTOR_IDS = [HIP_ID, THIGH_ID, SHANK_ID]

# Moving joints first, hip last.
COMMAND_ORDER = [THIGH_ID, SHANK_ID, HIP_ID]

MOTOR_NAMES = {
    HIP_ID: "hip",
    THIGH_ID: "thigh",
    SHANK_ID: "shank",
}


# ============================================================
# Known max-contraction / mechanical-limit joint angles
# ============================================================

KNOWN_LIMIT_ANGLE_BY_ROLE = {
    ROLE_HIP: 1.069,
    ROLE_THIGH: 1.199,
    ROLE_SHANK: 2.688,
}


# ============================================================
# Neutral standing pose in IK frame
# ============================================================

NEUTRAL_X = 0.0
NEUTRAL_Y = 84.26
NEUTRAL_Z = 378.0


# ============================================================
# Motor conversion signs
# ============================================================
# This matched your successful max-contraction -> neutral standing move.
GEAR_RATIO = 17.0
MOTOR_SIGN = 1.0

JOINT_SIGN_BY_ROLE = {
    ROLE_HIP: 1.0,
    ROLE_THIGH: 1.0,
    ROLE_SHANK: 1.0,
}

IK_ROLE_TO_PHYSICAL_ROLE = {
    ROLE_HIP: ROLE_HIP,
    ROLE_THIGH: ROLE_THIGH,
    ROLE_SHANK: ROLE_SHANK,
}


# ============================================================
# Stage 1: standing settings
# ============================================================

STAND_MOVE_SCALE = 1.0
STAND_MOVE_TIME = 3.50
NEUTRAL_HOLD_TIME = 1.0
AUTO_START_TRAJECTORY_AFTER_HOLD = True


# ============================================================
# Stage 2: modular trajectory settings
# ============================================================

RUN_TRAJECTORY_AFTER_STANDING = True

TRAJ_CFG = TrajectoryConfig(
    x_center=NEUTRAL_X,
    y_center=NEUTRAL_Y,
    z_ground=NEUTRAL_Z,

    # Tune these here, no need to rewrite trajectory code.
    step_length=100.0,
    step_height=70.0,
    step_sideways=0.0,
    stance_ratio=0.50,

    cycle_time=1.20,
    rate_hz=80.0,

    # You found this sign combination works for the physical planar trajectory.
    x_forward_sign=1.0,
    y_sideways_sign=1.0,
    z_lift_sign=-1.0,

    # Used only by tilted_planar.
    heading_deg=45.0,

    # Used only by vertical_jump.
    z_jump_amplitude=80.0,
)

TRAJ_CONVERSION = MotorConversionConfig(
    gear_ratio=GEAR_RATIO,
    motor_sign=MOTOR_SIGN,

    hip_sign=JOINT_SIGN_BY_ROLE[ROLE_HIP],
    thigh_sign=JOINT_SIGN_BY_ROLE[ROLE_THIGH],
    shank_sign=JOINT_SIGN_BY_ROLE[ROLE_SHANK],

    enable_hip_deadband=True,
    hip_delta_deadband_rad=1e-4,
)

# If False, trajectory helper still computes hip delta, but we hold hip at
# neutral raw to reduce sag during regular planar gait.
COMMAND_HIP_TRAJECTORY = False

COMMAND_ROLES: Tuple[str, ...] = (
    (ROLE_HIP, ROLE_THIGH, ROLE_SHANK)
    if COMMAND_HIP_TRAJECTORY
    else (ROLE_THIGH, ROLE_SHANK)
)

# Trajectory start phase:
#   0.5 * stance_ratio  -> neutral/mid-stance start
#   stance_ratio        -> beginning of cycloidal swing
#
# Use "swing_start" so the first gait motion after neutral is:
#   cycloidal swing forward -> straight-line stance pullback
TRAJECTORY_START_MODE = "swing_start"


# ============================================================
# Safety
# ============================================================

CONFIRM_IF_RAW_DELTA_OVER = 4.0
ABORT_IF_RAW_DELTA_OVER = 35.0

MAX_TRAJ_RAW_DELTA_BY_ROLE = {
    ROLE_HIP: 8.0,
    ROLE_THIGH: 13.0,
    ROLE_SHANK: 13.0,
}


# ============================================================
# Timing and gains
# ============================================================

RATE_HZ = 80.0
PRINT_EVERY = 20

ARM_KP = 0.0
ARM_KD = 0.0
ARM_TORQUE_LIMIT = 0.0

STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MOVE_KP_BY_ROLE = {
    ROLE_HIP: 0.20,
    ROLE_THIGH: 0.20,
    ROLE_SHANK: 0.20,
}

MOVE_KD_BY_ROLE = {
    ROLE_HIP: 0.008,
    ROLE_THIGH: 0.008,
    ROLE_SHANK: 0.008,
}

MOVE_TORQUE_LIMIT_BY_ROLE = {
    ROLE_HIP: 1.30,
    ROLE_THIGH: 0.75,
    ROLE_SHANK: 0.75,
}

RUN_KP_BY_ROLE = {
    ROLE_HIP: 0.10,
    ROLE_THIGH: 0.20,
    ROLE_SHANK: 0.20,
}

RUN_KD_BY_ROLE = {
    ROLE_HIP: 0.010,
    ROLE_THIGH: 0.008,
    ROLE_SHANK: 0.008,
}

RUN_TORQUE_LIMIT_BY_ROLE = {
    ROLE_HIP: 1.50,
    ROLE_THIGH: 0.80,
    ROLE_SHANK: 0.80,
}

HOLD_KP_BY_ROLE = {
    ROLE_HIP: 0.12,
    ROLE_THIGH: 0.10,
    ROLE_SHANK: 0.10,
}

HOLD_KD_BY_ROLE = {
    ROLE_HIP: 0.010,
    ROLE_THIGH: 0.006,
    ROLE_SHANK: 0.006,
}

HOLD_TORQUE_LIMIT_BY_ROLE = {
    ROLE_HIP: 1.50,
    ROLE_THIGH: 0.80,
    ROLE_SHANK: 0.80,
}


# ============================================================
# Setup
# ============================================================

args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)

STOP_REQUESTED = False
active_cmd: Dict[int, float] = {}


def request_stop(_signum=None, _frame=None):
    """Signal handler."""
    global STOP_REQUESTED
    STOP_REQUESTED = True
    print("\nStop requested.")


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


# ============================================================
# Low-level CAN helpers
# ============================================================

def set_mode_with_spacing(motor_id, mode):
    """Set mode with feed and small delay."""
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
    """Set gains by role and gain type."""
    role = ID_TO_ROLE[motor_id]

    if gain_type == "arm":
        set_gains(motor_id, ARM_KP, ARM_KD, ARM_TORQUE_LIMIT)
    elif gain_type == "startup":
        set_gains(motor_id, STARTUP_KP, STARTUP_KD, STARTUP_TORQUE_LIMIT)
    elif gain_type == "move":
        set_gains(
            motor_id,
            MOVE_KP_BY_ROLE[role],
            MOVE_KD_BY_ROLE[role],
            MOVE_TORQUE_LIMIT_BY_ROLE[role],
        )
    elif gain_type == "run":
        set_gains(
            motor_id,
            RUN_KP_BY_ROLE[role],
            RUN_KD_BY_ROLE[role],
            RUN_TORQUE_LIMIT_BY_ROLE[role],
        )
    elif gain_type == "hold":
        set_gains(
            motor_id,
            HOLD_KP_BY_ROLE[role],
            HOLD_KD_BY_ROLE[role],
            HOLD_TORQUE_LIMIT_BY_ROLE[role],
        )
    else:
        raise RuntimeError(f"Unknown gain type: {gain_type}")


def set_all_gains(gain_type):
    """Set all motor gains."""
    for motor_id in MOTOR_IDS:
        set_role_gains(motor_id, gain_type)


def read_position_measured(motor_id):
    """Read measured raw position."""
    value = bus.read_position_measured(motor_id)
    if value is None:
        raise RuntimeError(f"read_position_measured returned None for ID {motor_id}")
    return float(value)


def read_all_positions():
    """Read all measured positions."""
    values = {}
    for motor_id in MOTOR_IDS:
        values[motor_id] = read_position_measured(motor_id)
        time.sleep(0.004)
    return values


def command_position(motor_id, raw_target):
    """Send position command."""
    bus.transmit_pdo_2(motor_id, raw_target, 0.0)
    active_cmd[motor_id] = raw_target


def command_targets(targets_by_id: Dict[int, float]):
    """Command only IDs present in targets_by_id, in stable order."""
    for motor_id in COMMAND_ORDER:
        if motor_id in targets_by_id:
            command_position(motor_id, targets_by_id[motor_id])


def command_all_active():
    """Command all active targets."""
    for motor_id in COMMAND_ORDER:
        command_position(motor_id, active_cmd[motor_id])


def idle_all_motors(stop_bus=True):
    """Set all motors to IDLE."""
    print("\nPutting all motors into IDLE...")
    for motor_id in MOTOR_IDS:
        try:
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
            print(f"  {MOTOR_NAMES[motor_id]} ID {motor_id} IDLE")
        except Exception as exc:
            print(f"  Failed to idle {MOTOR_NAMES[motor_id]} ID {motor_id}: {exc}")

    time.sleep(0.12)

    if stop_bus:
        try:
            bus.stop()
        except Exception:
            pass


# ============================================================
# Standing-stage IK and conversion helpers
# ============================================================

def standing_ik_angles_by_role():
    """Return neutral IK angles for standing stage."""
    theta_h, theta_t, theta_s = leg_ik(NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z)
    return {
        ROLE_HIP: theta_h,
        ROLE_THIGH: theta_t,
        ROLE_SHANK: theta_s,
    }


def raw_delta_from_angle_delta(role, angle_delta):
    """Convert output joint angle delta to raw motor delta."""
    signed_delta = JOINT_SIGN_BY_ROLE[role] * angle_delta
    return MOTOR_SIGN * signed_delta * GEAR_RATIO


def compute_raw_targets_from_angle_deltas(start_raw, delta_angle_by_role, scale=1.0):
    """Compute raw targets from role angle deltas."""
    targets = {}

    for ik_role, delta_angle in delta_angle_by_role.items():
        physical_role = IK_ROLE_TO_PHYSICAL_ROLE[ik_role]
        motor_id = ROLE_TO_ID[physical_role]
        raw_delta = raw_delta_from_angle_delta(physical_role, delta_angle)
        targets[motor_id] = start_raw[motor_id] + scale * raw_delta

    return targets


def compute_standing_targets(start_raw):
    """Compute raw targets for max-contraction -> neutral standing."""
    neutral_angles = standing_ik_angles_by_role()

    delta_angle_by_role = {}
    for role in JOINT_ROLES:
        delta_angle_by_role[role] = (
            neutral_angles[role] - KNOWN_LIMIT_ANGLE_BY_ROLE[role]
        )

    raw_targets = compute_raw_targets_from_angle_deltas(
        start_raw,
        delta_angle_by_role,
        scale=STAND_MOVE_SCALE,
    )

    return neutral_angles, delta_angle_by_role, raw_targets


def print_standing_plan(start_raw, neutral_angles, delta_angles, raw_targets):
    """Print standing move plan."""
    print("\nNeutral IK angles:")
    for role in JOINT_ROLES:
        print(f"  {role:5s}: {neutral_angles[role]:+.6f} rad")

    print("\nKnown max-contraction angles:")
    for role in JOINT_ROLES:
        print(f"  {role:5s}: {KNOWN_LIMIT_ANGLE_BY_ROLE[role]:+.6f} rad")

    print("\nRelative angle deltas to neutral:")
    for role in JOINT_ROLES:
        print(f"  {role:5s}: {delta_angles[role]:+.6f} rad")

    print("\nRaw standing targets:")
    max_delta = 0.0
    for role in JOINT_ROLES:
        motor_id = ROLE_TO_ID[role]
        delta_raw = raw_targets[motor_id] - start_raw[motor_id]
        max_delta = max(max_delta, abs(delta_raw))
        print(
            f"  {role:5s} ID {motor_id}: "
            f"start={start_raw[motor_id]:+.6f}, "
            f"target={raw_targets[motor_id]:+.6f}, "
            f"delta={delta_raw:+.6f}"
        )

    print("\nSign settings:")
    print(f"  MOTOR_SIGN = {MOTOR_SIGN:+.1f}")
    for role in JOINT_ROLES:
        print(f"  JOINT_SIGN_BY_ROLE[{role!r}] = {JOINT_SIGN_BY_ROLE[role]:+.1f}")

    return max_delta


# ============================================================
# Trajectory helper integration
# ============================================================

def print_trajectory_summary(table):
    """Print trajectory_v3 summary with safety checks."""
    summary = summarize_command_table(table)
    print("\nTrajectory table summary:")

    ok = True
    for role in JOINT_ROLES:
        max_angle = summary[role]["max_abs_angle_delta"]
        max_raw = summary[role]["max_abs_raw_delta"]
        limit = MAX_TRAJ_RAW_DELTA_BY_ROLE[role]

        print(
            f"  {role:5s}: max angle delta={max_angle:+.6f}, "
            f"max raw delta={max_raw:+.3f}, limit={limit:.3f}"
        )

        if max_raw > limit:
            print(f"    ERROR: {role} trajectory delta exceeds limit.")
            ok = False

    if not ok:
        raise RuntimeError("Trajectory table failed safety check.")


def trajectory_targets_from_neutral_raw(neutral_raw, point):
    """
    Convert trajectory_v3 relative point into absolute raw targets.

    If COMMAND_HIP_TRAJECTORY is False, hip is actively held at neutral raw.
    """
    targets = raw_targets_by_id_from_start_raw(
        neutral_raw,
        ROLE_TO_ID,
        point,
        COMMAND_ROLES,
    )

    if not COMMAND_HIP_TRAJECTORY:
        targets[HIP_ID] = neutral_raw[HIP_ID]

    return targets


# ============================================================
# Startup and move helpers
# ============================================================

def read_start_raw_at_limit():
    """Read start raw positions at known mechanical-limit pose."""
    print("\nPutting all motors into IDLE before reading mechanical-limit pose...")
    idle_all_motors(stop_bus=False)
    time.sleep(0.35)

    print("\nReading current raw positions.")
    print("Make sure the leg is physically at the known max-contraction pose.")
    samples_by_id = {motor_id: [] for motor_id in MOTOR_IDS}

    for _ in range(20):
        values = read_all_positions()
        for motor_id in MOTOR_IDS:
            samples_by_id[motor_id].append(values[motor_id])
        rate.sleep()

    start_raw = {}
    for motor_id in MOTOR_IDS:
        samples = sorted(samples_by_id[motor_id])
        start_raw[motor_id] = samples[len(samples) // 2]

    print("\nStart raw positions:")
    for role in JOINT_ROLES:
        motor_id = ROLE_TO_ID[role]
        print(f"  {role:5s} ID {motor_id}: {start_raw[motor_id]:+.6f}")

    return start_raw


def arm_position_mode_holding(start_raw):
    """Enter position mode while holding start raw pose."""
    print("\nArming POSITION mode at zero torque while holding current raw pose...")

    for motor_id in MOTOR_IDS:
        set_role_gains(motor_id, "arm")
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    active_cmd.update(start_raw)

    for _ in range(int(0.35 * RATE_HZ)):
        command_all_active()
        rate.sleep()

    print("Ramping to soft startup hold...")
    for step in range(1, 11):
        a = step / 10.0
        for motor_id in MOTOR_IDS:
            set_gains(
                motor_id,
                STARTUP_KP * a,
                STARTUP_KD * a,
                STARTUP_TORQUE_LIMIT * a,
            )
        for _ in range(2):
            command_all_active()
            rate.sleep()

    print("Startup pose is softly held.")


def maybe_confirm_large_move(max_delta, label):
    """Confirm large move."""
    if max_delta > ABORT_IF_RAW_DELTA_OVER:
        raise RuntimeError(
            f"{label} max raw delta {max_delta:.3f} > "
            f"ABORT_IF_RAW_DELTA_OVER {ABORT_IF_RAW_DELTA_OVER:.3f}"
        )

    if max_delta > CONFIRM_IF_RAW_DELTA_OVER:
        answer = input(f"\n{label} max raw delta is {max_delta:.3f}. Move? y/n: ").strip().lower()
        return answer in ("y", "yes")

    return True


def move_all_to_targets(raw_targets, move_time, label):
    """Smoothly move motors to target."""
    print(f"\n{label}")
    start_cmd = dict(active_cmd)
    steps = int(move_time * RATE_HZ)

    for i in range(steps):
        if STOP_REQUESTED:
            return

        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        cmd = {}
        for motor_id, target in raw_targets.items():
            cmd[motor_id] = start_cmd[motor_id] + (
                target - start_cmd[motor_id]
            ) * s

        command_targets(cmd)

        if (i + 1) % PRINT_EVERY == 0:
            print_feedback_for_targets(cmd)

        rate.sleep()

    for motor_id, target in raw_targets.items():
        active_cmd[motor_id] = target


def print_feedback_for_targets(targets):
    """Print command vs measured raw for target IDs."""
    line = []
    try:
        measured = read_all_positions()
        for motor_id in COMMAND_ORDER:
            if motor_id in targets:
                role = ID_TO_ROLE[motor_id]
                cmd = targets[motor_id]
                raw = measured[motor_id]
                line.append(
                    f"{role}: cmd={cmd:+.3f} raw={raw:+.3f} err={cmd - raw:+.3f}"
                )
    except Exception as exc:
        line.append(f"read warning: {exc}")

    print(" | ".join(line))


def hold_neutral(neutral_raw, seconds):
    """Actively hold neutral raw pose for a set duration."""
    print(f"\nHolding neutral pose for {seconds:.1f} seconds...")
    for motor_id in MOTOR_IDS:
        active_cmd[motor_id] = neutral_raw[motor_id]

    steps = int(seconds * RATE_HZ)
    for i in range(steps):
        if STOP_REQUESTED:
            return
        command_all_active()

        if (i + 1) % PRINT_EVERY == 0:
            measured = read_all_positions()
            print(
                "neutral hold | "
                f"hip err={active_cmd[HIP_ID] - measured[HIP_ID]:+.3f} "
                f"thigh err={active_cmd[THIGH_ID] - measured[THIGH_ID]:+.3f} "
                f"shank err={active_cmd[SHANK_ID] - measured[SHANK_ID]:+.3f}"
            )

        rate.sleep()


# ============================================================
# Main
# ============================================================

try:
    print("=" * 80)
    print("Stand then modular trajectory v4 - swing first")
    print("=" * 80)
    print("Stage 1: max-contraction -> neutral using relative IK deltas")
    print("Stage 2: trajectory_v3 helper table from neutral raw reference")
    print()
    print(f"TRAJECTORY_NAME={TRAJECTORY_NAME}")
    print(f"MOTOR_SIGN={MOTOR_SIGN:+.1f}")
    print(f"STAND_MOVE_SCALE={STAND_MOVE_SCALE}")
    print(f"COMMAND_HIP_TRAJECTORY={COMMAND_HIP_TRAJECTORY}")
    print("=" * 80)

    start_raw = read_start_raw_at_limit()

    neutral_angles, standing_delta_angles, standing_targets = compute_standing_targets(start_raw)
    standing_max_delta = print_standing_plan(
        start_raw,
        neutral_angles,
        standing_delta_angles,
        standing_targets,
    )

    if not maybe_confirm_large_move(standing_max_delta, "Standing move"):
        raise KeyboardInterrupt

    arm_position_mode_holding(start_raw)

    print("\nSwitching to standing move gains...")
    set_all_gains("move")
    time.sleep(0.15)

    move_all_to_targets(
        standing_targets,
        STAND_MOVE_TIME,
        "Moving from max-contraction to neutral standing...",
    )

    neutral_raw = dict(active_cmd)

    print("\nNeutral standing reached.")
    print("Neutral raw reference:")
    for role in JOINT_ROLES:
        motor_id = ROLE_TO_ID[role]
        print(f"  {role:5s} ID {motor_id}: {neutral_raw[motor_id]:+.6f}")

    set_all_gains("hold")
    hold_neutral(neutral_raw, NEUTRAL_HOLD_TIME)

    if RUN_TRAJECTORY_AFTER_STANDING and AUTO_START_TRAJECTORY_AFTER_HOLD:
        print("\nAuto-starting modular trajectory...")

        print("\nBuilding trajectory table from trajectory_v3.py...")
        table = build_relative_command_table(
            TRAJECTORY_NAME,
            TRAJ_CFG,
            TRAJ_CONVERSION,
        )
        print_trajectory_summary(table)

        if TRAJECTORY_START_MODE == "neutral_mid_stance":
            start_phase = 0.5 * TRAJ_CFG.stance_ratio
        elif TRAJECTORY_START_MODE == "swing_start":
            start_phase = TRAJ_CFG.stance_ratio
        else:
            raise RuntimeError(f"Unknown TRAJECTORY_START_MODE: {TRAJECTORY_START_MODE}")

        start_index = int(start_phase * len(table)) % len(table)
        first_targets = trajectory_targets_from_neutral_raw(
            neutral_raw,
            table[start_index],
        )

        print(f"\nTrajectory start mode: {TRAJECTORY_START_MODE}")
        print(f"Moving to trajectory start phase {start_phase:.3f}...")
        set_all_gains("run")
        move_all_to_targets(
            first_targets,
            move_time=0.6,
            label="Moving from neutral to first trajectory point...",
        )

        print("\nStarting modular trajectory. Press Ctrl+C to stop.")
        index = start_index
        counter = 0

        while not STOP_REQUESTED:
            point = table[index]
            targets = trajectory_targets_from_neutral_raw(neutral_raw, point)
            command_targets(targets)

            counter += 1
            if counter % PRINT_EVERY == 0:
                foot = point["foot"]
                line = [
                    f"phase={point['phase']:.3f} {point['phase_name']} | "
                    f"x={foot['x']:+.1f} y={foot['y']:+.1f} z={foot['z']:+.1f}"
                ]

                try:
                    measured = read_all_positions()
                    for motor_id in COMMAND_ORDER:
                        if motor_id in targets:
                            role = ID_TO_ROLE[motor_id]
                            cmd = targets[motor_id]
                            raw = measured[motor_id]
                            line.append(
                                f"{role}: cmd={cmd:+.3f} raw={raw:+.3f} err={cmd - raw:+.3f}"
                            )
                except Exception as exc:
                    line.append(f"read warning: {exc}")

                print(" | ".join(line))

            index = (index + 1) % len(table)
            rate.sleep()

    else:
        print("\nTrajectory disabled. Holding neutral.")
        while not STOP_REQUESTED:
            command_all_active()
            rate.sleep()

except KeyboardInterrupt:
    print("\nInterrupted.")

finally:
    idle_all_motors(stop_bus=True)

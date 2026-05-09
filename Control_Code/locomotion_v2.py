"""
locomotion_v1_fixed.py

One-leg modular trajectory runner.

This version is formatted to avoid the major Mypy/Pylance errors:
    - Project-specific imports have type-ignore comments.
    - trajectory table entries use Point = Dict[str, Any].
    - Avoids variable names that shadow outer loop variables too much.
"""

from __future__ import annotations

import math
import signal
import time
from typing import Dict, Tuple

from loop_rate_limiters import RateLimiter  # type: ignore[import-not-found]
# type: ignore[import-not-found]
import berkeley_humanoid_lite_lowlevel.recoil as recoil

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


TRAJECTORY_NAME = "regular_planar"  # regular_planar, tilted_planar, vertical_jump

ROLE_TO_ID: Dict[str, int] = {
    ROLE_HIP: 1,
    ROLE_THIGH: 2,
    ROLE_SHANK: 3,
}

ALL_IDS = [ROLE_TO_ID[ROLE_HIP],
           ROLE_TO_ID[ROLE_THIGH], ROLE_TO_ID[ROLE_SHANK]]

MOTOR_NAMES: Dict[int, str] = {
    ROLE_TO_ID[ROLE_HIP]: "hip",
    ROLE_TO_ID[ROLE_THIGH]: "thigh",
    ROLE_TO_ID[ROLE_SHANK]: "shank",
}

COMMAND_ORDER = [
    ROLE_TO_ID[ROLE_THIGH],
    ROLE_TO_ID[ROLE_SHANK],
    ROLE_TO_ID[ROLE_HIP],
]

TRAJ_CFG = TrajectoryConfig(
    x_center=0.0,
    y_center=84.26,
    z_ground=382.0,
    step_length=80.0,
    step_height=70.0,
    step_sideways=0.0,
    stance_ratio=0.50,
    cycle_time=2.0,
    rate_hz=80.0,
    x_forward_sign=1.0,
    y_sideways_sign=1.0,
    z_lift_sign=-1.0,
    heading_deg=45.0,
    z_jump_amplitude=70.0,
)

CONVERSION = MotorConversionConfig(
    gear_ratio=17.0,
    motor_sign=-1.0,
    hip_sign=1.0,
    thigh_sign=1.0,
    shank_sign=1.0,
    enable_hip_deadband=True,
    hip_delta_deadband_rad=1e-4,
)

COMMAND_HIP = True
COMMAND_ROLES: Tuple[str, ...] = (
    (ROLE_HIP, ROLE_THIGH, ROLE_SHANK)
    if COMMAND_HIP
    else (ROLE_THIGH, ROLE_SHANK)
)

MAX_RAW_DELTA_FROM_START_BY_ROLE: Dict[str, float] = {
    ROLE_HIP: 8.0,
    ROLE_THIGH: 13.0,
    ROLE_SHANK: 13.0,
}

ARM_KP = 0.0
ARM_KD = 0.0
ARM_TORQUE_LIMIT = 0.0

STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP_BY_ROLE = {ROLE_HIP: 0.020, ROLE_THIGH: 0.025, ROLE_SHANK: 0.025}
MID_KD_BY_ROLE = {ROLE_HIP: 0.003, ROLE_THIGH: 0.002, ROLE_SHANK: 0.002}
MID_TORQUE_BY_ROLE = {ROLE_HIP: 0.60, ROLE_THIGH: 0.14, ROLE_SHANK: 0.14}

RUN_KP_BY_ROLE = {ROLE_HIP: 0.030, ROLE_THIGH: 0.055, ROLE_SHANK: 0.055}
RUN_KD_BY_ROLE = {ROLE_HIP: 0.006, ROLE_THIGH: 0.003, ROLE_SHANK: 0.003}
RUN_TORQUE_BY_ROLE = {ROLE_HIP: 1.00, ROLE_THIGH: 0.26, ROLE_SHANK: 0.26}

RATE_HZ = TRAJ_CFG.rate_hz
STARTUP_HOLD_TIME = 1.0
MOVE_TO_FIRST_TIME = 2.5
PRINT_EVERY = 20

args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)

STOP_REQUESTED = False


def request_stop(_signum=None, _frame=None):
    """Signal handler."""
    global STOP_REQUESTED
    STOP_REQUESTED = True
    print("\nStop requested.")


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.008)
    try:
        bus.feed(motor_id)
    except Exception as exc:
        print(f"feed warning ID {motor_id}: {exc}")
    time.sleep(0.008)


def set_gains(motor_id: int, kp: float, kd: float, torque_limit: float) -> None:
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_gains_by_role(joint_role: str, kp: float, kd: float, torque_limit: float) -> None:
    set_gains(ROLE_TO_ID[joint_role], kp, kd, torque_limit)


def set_all_gains_from_dicts(kp_by_role, kd_by_role, torque_by_role) -> None:
    for joint_role in JOINT_ROLES:
        set_gains_by_role(
            joint_role,
            kp_by_role[joint_role],
            kd_by_role[joint_role],
            torque_by_role[joint_role],
        )


def read_position_measured(motor_id: int) -> float:
    value = bus.read_position_measured(motor_id)
    if value is None:
        raise RuntimeError(
            f"read_position_measured returned None for ID {motor_id}")
    return float(value)


def command_position_only(motor_id: int, raw_target: float) -> None:
    bus.transmit_pdo_2(motor_id, raw_target, 0.0)


def command_targets(raw_targets_by_id: Dict[int, float]) -> None:
    for motor_id in COMMAND_ORDER:
        if motor_id in raw_targets_by_id:
            command_position_only(motor_id, raw_targets_by_id[motor_id])


def idle_all_motors() -> None:
    print("\nPutting all motors into IDLE...")
    for motor_id in ALL_IDS:
        try:
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
            print(f"  {MOTOR_NAMES[motor_id]} IDLE")
        except Exception as exc:
            print(f"  Failed to idle {MOTOR_NAMES[motor_id]}: {exc}")

    time.sleep(0.10)
    try:
        bus.stop()
    except Exception as exc:
        print(f"bus stop warning: {exc}")


def read_startup_positions_idle() -> Dict[int, float]:
    print("\nPutting all motors into IDLE before startup read...")
    for motor_id in ALL_IDS:
        set_mode_with_spacing(motor_id, recoil.Mode.IDLE)

    time.sleep(0.25)

    raw: Dict[int, float] = {}
    print("Reading measured positions:")

    for motor_id in ALL_IDS:
        samples = []
        for _ in range(15):
            try:
                samples.append(read_position_measured(motor_id))
            except Exception as exc:
                print(f"  Read warning for {MOTOR_NAMES[motor_id]}: {exc}")
            rate.sleep()

        if not samples:
            raise RuntimeError(
                f"No valid startup samples for {MOTOR_NAMES[motor_id]}")

        samples.sort()
        raw[motor_id] = samples[len(samples) // 2]
        print(
            f"  {MOTOR_NAMES[motor_id]:5s} ID {motor_id}: raw={raw[motor_id]:+.6f}")

    return raw


def arm_position_mode_holding(start_raw_by_id: Dict[int, float]) -> None:
    print("\nArming POSITION mode at zero torque, holding startup pose...")

    for motor_id in ALL_IDS:
        set_gains(motor_id, ARM_KP, ARM_KD, ARM_TORQUE_LIMIT)
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    for _ in range(int(0.40 * RATE_HZ)):
        command_targets(start_raw_by_id)
        rate.sleep()

    print("Applying soft startup gains...")
    for joint_role in JOINT_ROLES:
        set_gains_by_role(joint_role, STARTUP_KP,
                          STARTUP_KD, STARTUP_TORQUE_LIMIT)

    for _ in range(int(STARTUP_HOLD_TIME * RATE_HZ)):
        command_targets(start_raw_by_id)
        rate.sleep()

    print("Startup hold complete.")


def print_table_summary_and_check(table: list[Point]) -> None:
    summary = summarize_command_table(table)

    print("\nTrajectory table summary:")
    ok = True

    for joint_role in JOINT_ROLES:
        max_raw = summary[joint_role]["max_abs_raw_delta"]
        max_angle = summary[joint_role]["max_abs_angle_delta"]
        limit = MAX_RAW_DELTA_FROM_START_BY_ROLE[joint_role]

        print(
            f"  {joint_role:5s}: max angle delta={max_angle:+.6f} rad, "
            f"max raw delta={max_raw:+.3f}, limit={limit:.3f}"
        )

        if max_raw > limit:
            print(f"    ERROR: {joint_role} raw delta exceeds safety limit.")
            ok = False

    if not ok:
        raise RuntimeError("Trajectory table failed safety check.")


def smooth_move_to_first_point(
    start_raw_by_id: Dict[int, float],
    first_targets_by_id: Dict[int, float],
) -> None:
    print("\nMoving to first trajectory point...")
    steps = int(MOVE_TO_FIRST_TIME * RATE_HZ)

    for i in range(steps):
        if STOP_REQUESTED:
            return

        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        cmd: Dict[int, float] = {}
        for motor_id, target in first_targets_by_id.items():
            cmd[motor_id] = start_raw_by_id[motor_id] + (
                target - start_raw_by_id[motor_id]
            ) * s

        command_targets(cmd)
        rate.sleep()


try:
    print("=" * 80)
    print("locomotion_v1_fixed.py - one-leg modular trajectory runner")
    print("=" * 80)
    print(f"Trajectory selected: {TRAJECTORY_NAME}")
    print(f"Command hip: {COMMAND_HIP}")
    print("Role -> ID:")
    for printed_role in JOINT_ROLES:
        print(f"  {printed_role:5s} -> ID {ROLE_TO_ID[printed_role]}")
    print("=" * 80)

    startup_raw_by_id = read_startup_positions_idle()
    arm_position_mode_holding(startup_raw_by_id)

    print("\nBuilding trajectory table...")
    command_table = build_relative_command_table(
        TRAJECTORY_NAME, TRAJ_CFG, CONVERSION)
    print_table_summary_and_check(command_table)

    first_targets = raw_targets_by_id_from_start_raw(
        startup_raw_by_id,
        ROLE_TO_ID,
        command_table[0],
        COMMAND_ROLES,
    )

    print("\nRamping to medium gains...")
    set_all_gains_from_dicts(
        MID_KP_BY_ROLE, MID_KD_BY_ROLE, MID_TORQUE_BY_ROLE)

    smooth_move_to_first_point(startup_raw_by_id, first_targets)

    print("\nSwitching to run gains...")
    set_all_gains_from_dicts(
        RUN_KP_BY_ROLE, RUN_KD_BY_ROLE, RUN_TORQUE_BY_ROLE)

    print("\nStarting trajectory. Press Ctrl+C to stop.\n")

    index = 0
    counter = 0

    while not STOP_REQUESTED:
        point = command_table[index]
        targets = raw_targets_by_id_from_start_raw(
            startup_raw_by_id,
            ROLE_TO_ID,
            point,
            COMMAND_ROLES,
        )

        command_targets(targets)

        counter += 1
        if counter % PRINT_EVERY == 0:
            foot = point["foot"]
            deltas = point["raw_delta_by_role"]
            print(
                f"phase={point['phase']:.3f} {point['phase_name']} | "
                f"x={foot['x']:+.1f} y={foot['y']:+.1f} z={foot['z']:+.1f} | "
                f"dRaw hip={deltas[ROLE_HIP]:+.3f} "
                f"thigh={deltas[ROLE_THIGH]:+.3f} "
                f"shank={deltas[ROLE_SHANK]:+.3f}"
            )

        index = (index + 1) % len(command_table)
        rate.sleep()

except KeyboardInterrupt:
    request_stop()

finally:
    idle_all_motors()

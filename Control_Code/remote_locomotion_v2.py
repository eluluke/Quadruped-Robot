"""
locomotion_v1_fixed.py

Xbox-controlled one-leg modular trajectory runner.

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

from xbox_controller import XboxController  # type: ignore[import-not-found]

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
PRINT_EVERY = 40

JOYSTICK_DEADBAND = 0.05
JOYSTICK_FILTER_ALPHA = 0.65
MAX_PHASE_SPEED = 0.80
PHASE_ACCEL_LIMIT = 4.0
ALLOW_REVERSE = True
FREEZE_PHASE_WHEN_STOPPED = True
START_PHASE = 0.5 * TRAJ_CFG.stance_ratio

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


def apply_deadband(value: float, deadband: float) -> float:
    """Apply joystick deadband."""
    if abs(value) < deadband:
        return 0.0

    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadband) / (1.0 - deadband)


def limit_rate(current: float, target: float, max_delta: float) -> float:
    """Limit how fast a value can change."""
    delta = target - current

    if delta > max_delta:
        return current + max_delta

    if delta < -max_delta:
        return current - max_delta

    return target


def table_index_from_phase(phase: float, table_len: int) -> int:
    """Convert continuous phase to table index."""
    return int((phase % 1.0) * table_len) % table_len


def smooth_move_to_phase(
    start_raw_by_id: Dict[int, float],
    table: list[Point],
    phase: float,
) -> None:
    """Move smoothly to a selected initial phase."""
    index = table_index_from_phase(phase, len(table))
    targets = raw_targets_by_id_from_start_raw(
        start_raw_by_id,
        ROLE_TO_ID,
        table[index],
        COMMAND_ROLES,
    )

    print(f"\nMoving to initial phase {phase:.3f}...")
    smooth_move_to_first_point(start_raw_by_id, targets)


controller = None


try:
    print("=" * 80)
    print("remote_locomotion_v1_fixed.py - Xbox modular one-leg trajectory runner")
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

    print("\nRamping to medium gains...")
    set_all_gains_from_dicts(
        MID_KP_BY_ROLE, MID_KD_BY_ROLE, MID_TORQUE_BY_ROLE)

    controller = XboxController(deadzone=JOYSTICK_DEADBAND)

    phase = START_PHASE
    smooth_move_to_phase(startup_raw_by_id, command_table, phase)

    print("\nSwitching to run gains...")
    set_all_gains_from_dicts(
        RUN_KP_BY_ROLE, RUN_KD_BY_ROLE, RUN_TORQUE_BY_ROLE)

    print("\nStarting Xbox-controlled trajectory.")
    print("Left stick Y controls gait phase speed. Ctrl+C to stop.\n")

    ly_filtered = 0.0
    phase_speed = 0.0
    last_time = time.time()
    counter = 0

    while not STOP_REQUESTED:
        now = time.time()
        dt = now - last_time
        last_time = now

        if dt <= 0.0 or dt > 0.10:
            dt = 1.0 / RATE_HZ

        state = controller.read()
        raw_ly = max(-1.0, min(1.0, float(state.left_y)))

        ly = apply_deadband(raw_ly, JOYSTICK_DEADBAND)
        ly_filtered = (1.0 - JOYSTICK_FILTER_ALPHA) * \
            ly_filtered + JOYSTICK_FILTER_ALPHA * ly

        if not ALLOW_REVERSE and ly_filtered < 0.0:
            ly_filtered = 0.0

        target_phase_speed = MAX_PHASE_SPEED * ly_filtered
        max_phase_step = PHASE_ACCEL_LIMIT * dt
        phase_speed = limit_rate(
            phase_speed, target_phase_speed, max_phase_step)

        if abs(phase_speed) > 1e-5:
            phase = (phase + phase_speed * dt) % 1.0
        elif not FREEZE_PHASE_WHEN_STOPPED:
            phase = phase % 1.0

        table_index = table_index_from_phase(phase, len(command_table))
        point = command_table[table_index]

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
                f"LY_raw={raw_ly:+.2f} LY={ly_filtered:+.2f} "
                f"phase_speed={phase_speed:+.3f} phase={phase:.3f} "
                f"{point['phase_name']} | "
                f"x={foot['x']:+.1f} y={foot['y']:+.1f} z={foot['z']:+.1f} | "
                f"dRaw hip={deltas[ROLE_HIP]:+.3f} "
                f"thigh={deltas[ROLE_THIGH]:+.3f} "
                f"shank={deltas[ROLE_SHANK]:+.3f}"
            )

        rate.sleep()

except KeyboardInterrupt:
    request_stop()

finally:
    idle_all_motors()
    try:
        if controller is not None:
            controller.close()
    except Exception as exc:
        print(f"Controller close warning: {exc}")

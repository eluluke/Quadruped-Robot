"""
remote_single_leg_trot.py

Manual max-contraction confirmation -> neutral standing -> Xbox-controlled
single-leg regular planar trot.

This script imports:
    - CAN IDs, motor signs, gains, and max-contraction angles from leg_config.py
    - foot trajectory and IK angle deltas from trajectory_config.py
    - Xbox controller settings and wrapper from xbox_config.py

Example:
    python remote_single_leg_trot.py --leg front_right
"""

from __future__ import annotations

import argparse
import signal
import time
from typing import Dict, List

from leg_config import (
    FRONT_RIGHT,
    HIP,
    JOINT_ORDER,
    LEG_ORDER,
    get_joint_gains,
    get_max_contraction_angles_for_leg,
)
from quadruped_utils import limit_rate
from trajectory_config import (
    JOINT_ROLES,
    TRAJ_REGULAR_PLANAR,
    TrajectoryPoint,
    build_angle_delta_table,
    ik_angles_for_xyz,
    table_index_from_phase,
)
from xbox_config import (
    ALLOW_REVERSE,
    FREEZE_PHASE_WHEN_STOPPED,
    JOYSTICK_DEADBAND,
    JOYSTICK_FILTER_ALPHA,
    MAX_PHASE_SPEED,
    PHASE_ACCEL_LIMIT,
    XboxController,
    apply_deadband,
)

import single_leg_trot as base


TRAJECTORY_NAME = TRAJ_REGULAR_PLANAR
TRAJ_CFG = base.TRAJ_CFG

COMMAND_HIP_TRAJECTORY = False
TRAJECTORY_START_MODE = "swing_start"

STAND_MOVE_TIME = 3.50
NEUTRAL_HOLD_TIME = 0.8
MOVE_TO_TRAJ_START_TIME = 0.6
PRINT_EVERY = 40

CONFIRM_IF_RAW_DELTA_OVER = 4.0
ABORT_IF_RAW_DELTA_OVER = 35.0
REQUIRE_SECOND_MOVE_CONFIRMATION = False


def mark_stop() -> None:
    base.STOP_REQUESTED = True
    print("\nStop requested.")


def request_stop(_signum=None, _frame=None) -> None:
    mark_stop()
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--leg",
        choices=LEG_ORDER,
        default=FRONT_RIGHT,
        help="Leg hardware config to use from leg_config.py.",
    )
    return parser.parse_args()


def wait_for_homing_confirmation(leg_name: str) -> None:
    print("=" * 80)
    print("remote_single_leg_trot.py")
    print("=" * 80)
    print(f"Selected leg: {leg_name}")
    print()
    print("Put this leg manually at its max-contraction homing pose.")
    print("No motor commands are sent before you confirm.")
    print("=" * 80)

    while True:
        if base.STOP_REQUESTED:
            raise KeyboardInterrupt

        answer = input("\nType y when the leg is at max contraction, or q to quit: ")
        answer = answer.strip().lower()
        if answer in ("y", "yes"):
            return
        if answer in ("q", "quit", "exit"):
            raise KeyboardInterrupt
        print("Use y to confirm or q to quit.")


def compute_neutral_targets(
    runner: base.SingleLegRunner,
    limit_raw: Dict[int, float],
) -> tuple[Dict[str, float], Dict[str, float], Dict[int, float]]:
    """Compute raw targets from configured max-contraction pose to neutral."""
    neutral_angles = ik_angles_for_xyz(
        TRAJ_CFG.x_center,
        TRAJ_CFG.y_center,
        TRAJ_CFG.z_ground,
    )
    limit_angles = get_max_contraction_angles_for_leg(runner.leg_name)

    delta_angles = {
        role: neutral_angles[role] - limit_angles[role]
        for role in JOINT_ROLES
    }

    targets = {}
    for role in JOINT_ROLES:
        motor_id = runner.role_to_id[role]
        raw_delta = runner.raw_delta_for_role(role, delta_angles[role])
        targets[motor_id] = limit_raw[motor_id] + raw_delta

    return neutral_angles, delta_angles, targets


def print_standing_plan(
    runner: base.SingleLegRunner,
    limit_raw: Dict[int, float],
    neutral_angles: Dict[str, float],
    delta_angles: Dict[str, float],
    neutral_targets: Dict[int, float],
) -> float:
    max_contraction_angles = get_max_contraction_angles_for_leg(runner.leg_name)

    print("\nNeutral IK angles:")
    for role in JOINT_ROLES:
        print(f"  {role:5s}: {neutral_angles[role]:+.6f} rad")

    print("\nConfigured max-contraction angles from leg_config.py:")
    for role in JOINT_ROLES:
        print(f"  {role:5s}: {max_contraction_angles[role]:+.6f} rad")

    print("\nRelative output angle deltas to neutral:")
    for role in JOINT_ROLES:
        print(f"  {role:5s}: {delta_angles[role]:+.6f} rad")

    print("\nRaw neutral targets:")
    max_delta = 0.0
    for role in JOINT_ORDER:
        motor_id = runner.role_to_id[role]
        delta_raw = neutral_targets[motor_id] - limit_raw[motor_id]
        max_delta = max(max_delta, abs(delta_raw))
        print(
            f"  {role:5s} ID {motor_id}: "
            f"start={limit_raw[motor_id]:+.6f}, "
            f"target={neutral_targets[motor_id]:+.6f}, "
            f"delta={delta_raw:+.6f}"
        )

    print("\nGains from leg_config.py:")
    for role in JOINT_ORDER:
        kp, kd, torque_limit = get_joint_gains(runner.leg_name, role)
        print(f"  {role:5s}: kp={kp:.4f}, kd={kd:.4f}, torque={torque_limit:.4f}")

    return max_delta


def maybe_confirm_large_move(max_delta: float, label: str) -> bool:
    if max_delta > ABORT_IF_RAW_DELTA_OVER:
        raise RuntimeError(
            f"{label} max raw delta {max_delta:.3f} exceeds "
            f"abort limit {ABORT_IF_RAW_DELTA_OVER:.3f}."
        )

    if max_delta > CONFIRM_IF_RAW_DELTA_OVER:
        print(
            f"\n{label} max raw delta is {max_delta:.3f}, "
            f"above warning threshold {CONFIRM_IF_RAW_DELTA_OVER:.3f}."
        )

        if not REQUIRE_SECOND_MOVE_CONFIRMATION:
            print("Continuing automatically after homing confirmation.")
            return True

        answer = input("Move? y/n: ")
        return answer.strip().lower() in ("y", "yes")

    return True


def hold_targets(
    runner: base.SingleLegRunner,
    targets: Dict[int, float],
    seconds: float,
    label: str,
) -> None:
    print(f"\n{label}")
    runner.active_cmd.update(targets)
    steps = int(seconds * base.RATE_HZ)

    for i in range(steps):
        if base.STOP_REQUESTED:
            return

        runner.command_targets(targets)
        if (i + 1) % PRINT_EVERY == 0:
            runner.print_feedback()
        runner.rate.sleep()


def trajectory_targets_from_neutral(
    runner: base.SingleLegRunner,
    neutral_raw: Dict[int, float],
    point: TrajectoryPoint,
) -> Dict[int, float]:
    targets = runner.targets_from_reference(
        neutral_raw,
        point,
        command_roles=base.COMMAND_ROLES,
    )

    if not COMMAND_HIP_TRAJECTORY:
        hip_id = runner.role_to_id[HIP]
        targets[hip_id] = neutral_raw[hip_id]

    return targets


def move_to_trajectory_start(
    runner: base.SingleLegRunner,
    neutral_raw: Dict[int, float],
    table: List[TrajectoryPoint],
) -> float:
    if TRAJECTORY_START_MODE == "neutral_mid_stance":
        phase = 0.5 * TRAJ_CFG.stance_ratio
    elif TRAJECTORY_START_MODE == "swing_start":
        phase = TRAJ_CFG.stance_ratio
    else:
        raise RuntimeError(f"Unknown TRAJECTORY_START_MODE: {TRAJECTORY_START_MODE}")

    index = table_index_from_phase(phase, len(table))
    first_targets = trajectory_targets_from_neutral(runner, neutral_raw, table[index])

    print(f"\nTrajectory start mode: {TRAJECTORY_START_MODE}")
    runner.move_to_targets(
        first_targets,
        MOVE_TO_TRAJ_START_TIME,
        f"Moving to trajectory start phase {phase:.3f}...",
        print_every=PRINT_EVERY,
    )

    return phase


def run_xbox_trot(
    runner: base.SingleLegRunner,
    neutral_raw: Dict[int, float],
    table: List[TrajectoryPoint],
    phase: float,
) -> None:
    controller = None

    try:
        controller = XboxController(deadzone=JOYSTICK_DEADBAND)

        print("\nStarting Xbox-controlled single-leg trot.")
        print("Left stick Y controls gait phase speed. Ctrl+C to stop.\n")

        ly_filtered = 0.0
        phase_speed = 0.0
        last_time = time.time()
        counter = 0

        while not base.STOP_REQUESTED:
            now = time.time()
            dt = now - last_time
            last_time = now
            if dt <= 0.0 or dt > 0.10:
                dt = 1.0 / base.RATE_HZ

            state = controller.read()
            raw_ly = max(-1.0, min(1.0, float(state.left_y)))

            ly = apply_deadband(raw_ly, JOYSTICK_DEADBAND)
            ly_filtered = (
                (1.0 - JOYSTICK_FILTER_ALPHA) * ly_filtered
                + JOYSTICK_FILTER_ALPHA * ly
            )

            if not ALLOW_REVERSE and ly_filtered < 0.0:
                ly_filtered = 0.0

            target_phase_speed = MAX_PHASE_SPEED * ly_filtered
            max_phase_step = PHASE_ACCEL_LIMIT * dt
            phase_speed = limit_rate(phase_speed, target_phase_speed, max_phase_step)

            if abs(phase_speed) > 1e-5:
                phase = (phase + phase_speed * dt) % 1.0
            elif not FREEZE_PHASE_WHEN_STOPPED:
                phase %= 1.0

            index = table_index_from_phase(phase, len(table))
            point = table[index]
            targets = trajectory_targets_from_neutral(runner, neutral_raw, point)
            runner.command_targets(targets)

            counter += 1
            if counter % PRINT_EVERY == 0:
                foot = point.foot
                print(
                    f"LY_raw={raw_ly:+.2f} LY={ly_filtered:+.2f} "
                    f"phase_speed={phase_speed:+.3f} phase={phase:.3f} "
                    f"{point.phase_name} | "
                    f"x={foot.x:+.1f} y={foot.y:+.1f} z={foot.z:+.1f}"
                )
                runner.print_feedback()

            runner.rate.sleep()

    finally:
        if controller is not None:
            try:
                controller.close()
            except Exception:
                pass


def main() -> None:
    args = parse_args()
    runner = base.SingleLegRunner(args.leg)

    try:
        wait_for_homing_confirmation(runner.leg_name)

        print("\nConfirmed. Reading max-contraction raw pose and arming position mode.")
        limit_raw = runner.arm_at_current_pose()

        neutral_angles, delta_angles, neutral_targets = compute_neutral_targets(
            runner,
            limit_raw,
        )
        max_delta = print_standing_plan(
            runner,
            limit_raw,
            neutral_angles,
            delta_angles,
            neutral_targets,
        )

        if not maybe_confirm_large_move(max_delta, "Standing move"):
            raise KeyboardInterrupt

        print("\nSwitching to leg_config.py standing/run gains...")
        runner.set_all_config_gains()

        runner.move_to_targets(
            neutral_targets,
            STAND_MOVE_TIME,
            "Moving from max contraction to neutral standing...",
            print_every=PRINT_EVERY,
        )

        neutral_raw = dict(runner.active_cmd)
        hold_targets(
            runner,
            neutral_raw,
            NEUTRAL_HOLD_TIME,
            f"Holding neutral pose for {NEUTRAL_HOLD_TIME:.1f}s...",
        )

        table = build_angle_delta_table(TRAJECTORY_NAME, TRAJ_CFG)
        base.print_trajectory_summary(runner, table)
        phase = move_to_trajectory_start(runner, neutral_raw, table)
        run_xbox_trot(runner, neutral_raw, table, phase)

    except KeyboardInterrupt:
        mark_stop()

    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        runner.idle_all()


if __name__ == "__main__":
    main()

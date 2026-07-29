"""
quadruped_trajectory_control.py

Whole-robot controller with automatic vertical stepping and Xbox-selected
cycloidal walking.

Startup sequence:
    1. Confirm the robot is flat at its selected homing pose.
    2. Record all four current poses as homed.
    3. Move all four legs to their neutral standing poses.
    4. Keep holding neutral while the operator checks the standing pose.
    5. Start vertical stepping after a second y confirmation.
    6. Blend into forward/reverse cycloidal walking with Xbox left-stick Y.
"""

from __future__ import annotations

import signal

from leg_config import get_can_channel, get_neutral_standing_foot_position
from quadruped_main_state import STOP_REQUESTED_REF
from robot_controller import (
    LEG_CONFIRM_COMMANDS,
    RobotController,
    WHOLE_BODY_COMMAND_ORDER,
)
from trajectory_config import TRAJECTORY_CONFIG


GROUND_START_NEUTRAL_TOLERANCE = 3.0

# Automatic marching-in-place speed in gait cycles per second.
AUTO_VERTICAL_PHASE_SPEED = 0.5

# Time to blend completely between vertical and planar trajectories is
# approximately 1 / TRAJECTORY_BLEND_RATE seconds.
TRAJECTORY_BLEND_RATE = 2.0


def request_stop(_signum=None, _frame=None) -> None:
    STOP_REQUESTED_REF["stop"] = True
    print("\nStop requested.")
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def print_header() -> None:
    print("=" * 80)
    print("quadruped_trajectory_control.py")
    print("=" * 80)
    print("Put the robot flat on the ground at the selected homing pose.")
    print("Type y once to record all four legs as homed and stand up.")
    print()
    print("After standing:")
    print("  y = begin automatic vertical stepping")
    print("  q = abort and shut down")
    print()
    print("During locomotion:")
    print("  centered left stick = automatic vertical stepping")
    print("  left stick Y = cycloidal forward/reverse walking")
    print("  Ctrl+C = emergency shutdown")
    print()
    print("Leg mapping and neutral foot references:")
    for command, leg_name in LEG_CONFIRM_COMMANDS.items():
        channel = get_can_channel(leg_name)
        neutral = get_neutral_standing_foot_position(leg_name)
        print(
            f"  {command}: {leg_name:11s} channel={channel} "
            f"neutral={neutral}"
        )
    print()
    print("Control loop command order:")
    for index, leg_name in enumerate(WHOLE_BODY_COMMAND_ORDER, start=1):
        channel = get_can_channel(leg_name)
        print(f"  {index}: {leg_name:11s} channel={channel}")
    print()
    print(
        "Vertical step height: "
        f"{TRAJECTORY_CONFIG.vertical_step_height:.2f} mm"
    )
    print("=" * 80)


def main() -> None:
    STOP_REQUESTED_REF["stop"] = False
    print_header()
    robot = RobotController()

    try:
        while True:
            answer = input("\nType y when the robot is flat and ready: ")
            answer = answer.strip().lower()
            if answer in ("y", "yes"):
                break
            if answer in ("q", "quit"):
                print("Startup aborted.")
                return
            print("Waiting for y. Type q or press Ctrl+C to abort.")

        robot.record_all_homed_at_current_pose()

        standing = robot.move_all_to_neutral(
            confirm_large_move=False,
            reached_raw_tolerance=GROUND_START_NEUTRAL_TOLERANCE,
        )
        if not standing:
            return

        robot.hold_neutral()
        if not robot.confirm_standing_ready():
            return

        robot.run_vertical_idle_xbox_trot(
            auto_vertical_phase_speed=AUTO_VERTICAL_PHASE_SPEED,
            trajectory_blend_rate=TRAJECTORY_BLEND_RATE,
        )

    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        robot.idle_all()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

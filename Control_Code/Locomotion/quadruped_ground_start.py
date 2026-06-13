"""
quadruped_ground_start.py

One-key whole-robot startup for the flat-on-ground homing workflow.

Use this when the robot is already lying flat at the selected homing pose
from leg_config.py. Type y once, then the program records all current raw
leg poses as homed and moves the robot to neutral standing together.
"""

from __future__ import annotations

import signal

from leg_config import get_can_channel
from quadruped_main_state import STOP_REQUESTED_REF
from robot_controller import (
    LEG_CONFIRM_COMMANDS,
    RobotController,
)


def request_stop(_signum=None, _frame=None) -> None:
    STOP_REQUESTED_REF["stop"] = True
    print("\nStop requested.")
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def print_header() -> None:
    print("=" * 80)
    print("quadruped_ground_start.py")
    print("=" * 80)
    print("Put the robot flat on the ground at the selected homing pose.")
    print("Type y once to record all four legs as homed and stand up.")
    print()
    print("Leg mapping:")
    for command, leg_name in LEG_CONFIRM_COMMANDS.items():
        channel = get_can_channel(leg_name)
        print(f"  {command}: {leg_name:11s} channel={channel}")
    print("=" * 80)


def main() -> None:
    print_header()
    robot = RobotController()

    try:
        answer = input("\nType y to record homing pose and stand, or q to quit: ")
        answer = answer.strip().lower()

        if answer not in ("y", "yes"):
            print("Cancelled.")
            return

        robot.record_all_homed_at_current_pose()

        if robot.move_all_to_neutral():
            robot.hold_neutral()
            robot.build_trajectory_table()
            robot.run_xbox_trot()

    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        robot.idle_all()


try:
    main()
except KeyboardInterrupt:
    pass

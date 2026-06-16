"""
quadruped_neutral_hold.py

One-key whole-robot neutral standing hold test.

Use this before trotting when you want to debug:
    - whether all four legs can stand together
    - whether rear legs sag under load
    - whether neutral tracking is stable on the current floor
    - whether Ctrl+C shutdown idles all motors cleanly
"""

from __future__ import annotations

import signal

from leg_config import get_can_channel
from quadruped_main_state import STOP_REQUESTED_REF
from robot_controller import LEG_CONFIRM_COMMANDS, RobotController


GROUND_START_NEUTRAL_TOLERANCE = 3.0


def request_stop(_signum=None, _frame=None) -> None:
    STOP_REQUESTED_REF["stop"] = True
    print("\nStop requested.")
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def print_header() -> None:
    print("=" * 80)
    print("quadruped_neutral_hold.py")
    print("=" * 80)
    print("Put the robot flat on the ground at the selected homing pose.")
    print("Type y once to record all four legs as homed and stand up.")
    print("The robot will hold neutral standing only. No trot will run.")
    print()
    print("Leg mapping:")
    for command, leg_name in LEG_CONFIRM_COMMANDS.items():
        channel = get_can_channel(leg_name)
        print(f"  {command}: {leg_name:11s} channel={channel}")
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
            print("Waiting for y. Press Ctrl+C to abort.")

        robot.record_all_homed_at_current_pose()

        if robot.move_all_to_neutral(
            confirm_large_move=False,
            reached_raw_tolerance=GROUND_START_NEUTRAL_TOLERANCE,
        ):
            robot.hold_neutral_until_stop()

    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        robot.idle_all()


try:
    main()
except KeyboardInterrupt:
    pass

"""
quadruped_march_then_trot.py

Whole-robot startup for testing vertical marching before translation.

Use this when the robot is already lying flat at the selected homing pose
from leg_config.py. Type y once, then the program records all current raw
leg poses as homed, stands up, briefly holds neutral, starts vertical
marching in place, and blends into planar trot only when Xbox input is given.
"""

from __future__ import annotations

import signal

from leg_config import get_can_channel
from quadruped_main_state import STOP_REQUESTED_REF
from robot_controller import (
    LEG_CONFIRM_COMMANDS,
    RobotController,
    VERTICAL_IDLE_PHASE_SPEED,
    WHOLE_BODY_COMMAND_ORDER,
    XBOX_MOTION_THRESHOLD,
)

GROUND_START_NEUTRAL_TOLERANCE = 3.0


def request_stop(_signum=None, _frame=None) -> None:
    STOP_REQUESTED_REF["stop"] = True
    print("\nStop requested.")
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def print_header() -> None:
    print("=" * 80)
    print("quadruped_march_then_trot.py")
    print("=" * 80)
    print("Put the robot flat on the ground at the selected homing pose.")
    print("Type y once to record all four legs as homed and stand up.")
    print("After standing, it marches in place until Xbox stick input.")
    print()
    print("Leg mapping:")
    for command, leg_name in LEG_CONFIRM_COMMANDS.items():
        channel = get_can_channel(leg_name)
        print(f"  {command}: {leg_name:11s} channel={channel}")
    print()
    print("Control loop command order:")
    for index, leg_name in enumerate(WHOLE_BODY_COMMAND_ORDER, start=1):
        channel = get_can_channel(leg_name)
        print(f"  {index}: {leg_name:11s} channel={channel}")
    print()
    print(
        "Vertical idle: "
        f"phase_speed={VERTICAL_IDLE_PHASE_SPEED:.2f} cycles/s, "
        f"xbox_threshold={XBOX_MOTION_THRESHOLD:.2f}"
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
            print("Waiting for y. Press Ctrl+C to abort.")

        robot.record_all_homed_at_current_pose()

        if robot.move_all_to_neutral(
            confirm_large_move=False,
            reached_raw_tolerance=GROUND_START_NEUTRAL_TOLERANCE,
        ):
            robot.hold_neutral()
            robot.run_vertical_idle_xbox_planar_trot()

    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        robot.idle_all()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

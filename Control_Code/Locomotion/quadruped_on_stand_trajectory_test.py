"""
quadruped_on_stand_trajectory_test.py

On-stand whole-robot foot trajectory test.

This program skips homing. It immediately reads the current raw motor
positions, enters position mode at those positions, treats them as the neutral
trajectory reference, and holds all joints. The selected trajectory from
trajectory_config.py starts only after Xbox left stick Y input is detected.
"""

from __future__ import annotations

import signal

from leg_config import get_can_channel
from quadruped_main_state import STOP_REQUESTED_REF
from robot_controller import (
    COMMAND_HIP_TRAJECTORY,
    LEG_CONFIRM_COMMANDS,
    RobotController,
    TRAJECTORY_NAME,
    TRAJECTORY_START_MODE,
    WHOLE_BODY_COMMAND_ORDER,
)


def request_stop(_signum=None, _frame=None) -> None:
    STOP_REQUESTED_REF["stop"] = True
    print("\nStop requested.")
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def print_header() -> None:
    print("=" * 80)
    print("quadruped_on_stand_trajectory_test.py")
    print("=" * 80)
    print("Put the robot safely on a stand before running this program.")
    print("This skips homing and uses the current joint positions as neutral.")
    print("It holds immediately, then starts the trajectory after Xbox input.")
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
    print("Trajectory test selector:")
    print(f"  TRAJECTORY_NAME={TRAJECTORY_NAME}")
    print(f"  COMMAND_HIP_TRAJECTORY={COMMAND_HIP_TRAJECTORY}")
    print(f"  TRAJECTORY_START_MODE={TRAJECTORY_START_MODE}")
    print("  Edit TRAJECTORY_NAME in trajectory_config.py to switch paths.")
    print("=" * 80)


def main() -> None:
    STOP_REQUESTED_REF["stop"] = False
    print_header()
    robot = RobotController()

    try:
        robot.hold_current_pose_as_neutral()
        robot.build_trajectory_tables()
        robot.run_xbox_trot(wait_for_command_before_start=True)

    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        robot.idle_all()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

"""
quadruped_main_slim.py

Highest-level operator program for whole-robot locomotion.

This is the file to run when you want to:
    1. Manually place each leg at its max-contraction homing pose.
    2. Confirm each leg with y1/y2/y3/y4.
    3. Move all legs to neutral standing.
    4. Start Xbox-controlled trot.

All lower-level behavior lives in:
    leg_config.py
    gains_config.py
    xbox_config.py
    trajectory_config.py
    leg_controller.py
    robot_controller.py
    gait_scheduler.py
"""

from __future__ import annotations

import signal

from leg_config import get_can_channel
from quadruped_main_state import STOP_REQUESTED_REF
from quadruped_utils import (
    start_terminal_input_thread,
    wait_for_terminal_command,
)
from robot_controller import (
    COMMAND_HIP_TRAJECTORY,
    LEG_CONFIRM_COMMANDS,
    TRAJECTORY_NAME,
    TRAJECTORY_START_MODE,
    RobotController,
)


def request_stop(_signum=None, _frame=None) -> None:
    STOP_REQUESTED_REF["stop"] = True
    print("\nStop requested.")


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def print_header() -> None:
    print("=" * 80)
    print("quadruped_main_slim.py")
    print("=" * 80)
    print("Whole-robot state machine.")
    print("Startup sends no position commands until a leg is confirmed homed.")
    print()
    print("Leg confirmation mapping:")
    for command, leg_name in LEG_CONFIRM_COMMANDS.items():
        channel = get_can_channel(leg_name)
        print(f"  {command}: {leg_name:11s} channel={channel}")
    print()
    print(f"TRAJECTORY_NAME={TRAJECTORY_NAME}")
    print(f"COMMAND_HIP_TRAJECTORY={COMMAND_HIP_TRAJECTORY}")
    print(f"TRAJECTORY_START_MODE={TRAJECTORY_START_MODE}")
    print("=" * 80)


def print_menu() -> None:
    print("\nCommands:")
    print("  y1 / y2 / y3 / y4  = confirm leg at max contraction")
    print("  status             = show homing status")
    print("  y                  = stand, then start Xbox trot")
    print("  q                  = quit / idle all motors")


def main() -> None:
    print_header()
    start_terminal_input_thread()

    robot = RobotController()

    try:
        while not STOP_REQUESTED_REF["stop"]:
            robot.print_homing_status()
            print_menu()

            cmd = wait_for_terminal_command("\nCommand: ")

            if cmd in ("q", "quit", "exit"):
                break

            if cmd == "status":
                continue

            if cmd in LEG_CONFIRM_COMMANDS:
                robot.mark_leg_homed(cmd)
                continue

            if cmd in ("y", "yes"):
                if not robot.all_homed():
                    print("Not all legs are homed yet. Use y1/y2/y3/y4 first.")
                    continue

                if robot.move_all_to_neutral():
                    robot.hold_neutral()
                    robot.build_trajectory_table()
                    robot.run_xbox_trot()
                break

            print("Unknown command.")

    finally:
        robot.idle_all()


try:
    main()
except KeyboardInterrupt:
    request_stop()

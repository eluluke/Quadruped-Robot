"""
quadruped_main_slim.py

Very small high-level program.

This is the file you run.

It only cares about state:
    - Are legs homed?
    - Did user confirm y1/y2/y3/y4?
    - Did user type y to stand?
    - Did Xbox command arrive to trot?

All low-level details live in:
    leg_config.py
    gains_config.py
    xbox_config.py
    leg_controller.py
    robot_controller.py
    gait_scheduler.py
    trajectory_v3.py
"""

import signal

from quadruped_main_state import STOP_REQUESTED_REF
from quadruped_utils import start_terminal_input_thread, wait_for_terminal_command
from robot_controller import RobotController
from leg_config import LEG_CONFIGS, TRAJECTORY_NAME, COMMAND_HIP_TRAJECTORY, TRAJECTORY_START_MODE


def request_stop(_signum=None, _frame=None):
    STOP_REQUESTED_REF["stop"] = True
    print("\nStop requested.")


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def print_header():
    print("=" * 80)
    print("quadruped_main_slim.py")
    print("=" * 80)
    print("High-level state machine only.")
    print("Startup sends NO motor commands until y1/y2/y3/y4 are typed.")
    print()
    print("Leg mapping:")
    for name, cfg in LEG_CONFIGS.items():
        print(f"  {name}: channel={cfg['channel']} phase_offset={cfg['phase_offset']}")
    print()
    print(f"TRAJECTORY_NAME={TRAJECTORY_NAME}")
    print(f"COMMAND_HIP_TRAJECTORY={COMMAND_HIP_TRAJECTORY}")
    print(f"TRAJECTORY_START_MODE={TRAJECTORY_START_MODE}")
    print("=" * 80)


def print_menu():
    print("\nCommands:")
    print("  y1 / y2 / y3 / y4  = confirm that leg is manually homed")
    print("  status             = show homing status")
    print("  y                  = stand, then start Xbox trot")
    print("  q                  = quit / idle all motors")


def main():
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

            if cmd in ("y1", "y2", "y3", "y4"):
                robot.mark_leg_homed(int(cmd[1]))
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

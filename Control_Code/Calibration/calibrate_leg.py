#!/usr/bin/env python3
"""
Calibrate electrical offset for one complete quadruped leg.

Safety rule: this script only supports one leg at a time. It does not provide
a whole-robot mode.

Examples:
    python3 calibrate_leg.py --leg rear_right
    python3 calibrate_leg.py --leg front_left --yes
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import berkeley_humanoid_lite_lowlevel.recoil as recoil


CONTROL_CODE_DIR = Path(__file__).resolve().parents[1]
LOCOMOTION_DIR = CONTROL_CODE_DIR / "Locomotion"
sys.path.insert(0, str(LOCOMOTION_DIR))

from leg_config import JOINT_ORDER, LEG_ORDER, get_can_channel, get_can_id  # noqa: E402


BITRATE = 1_000_000
DEFAULT_CALIBRATION_CURRENT = 5.0
DEFAULT_WAIT_S = 20.0
PING_TIMEOUT_S = 0.1
COMMAND_SPACING_S = 0.05


@dataclass
class LegMotor:
    leg: str
    joint: str
    channel: str
    motor_id: int

    @property
    def name(self) -> str:
        return f"{self.leg}_{self.joint}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate electrical offset for exactly one configured leg."
    )
    parser.add_argument(
        "--leg",
        choices=LEG_ORDER,
        required=True,
        help="Leg to calibrate from Locomotion/leg_config.py.",
    )
    parser.add_argument(
        "--current",
        type=float,
        default=DEFAULT_CALIBRATION_CURRENT,
        help="Motor calibration current to write before calibration.",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=DEFAULT_WAIT_S,
        help="Seconds to wait after entering calibration mode.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the interactive safety confirmation.",
    )
    return parser.parse_args()


def build_leg_motors(leg: str) -> list[LegMotor]:
    channel = get_can_channel(leg)
    return [
        LegMotor(
            leg=leg,
            joint=joint,
            channel=channel,
            motor_id=get_can_id(leg, joint),
        )
        for joint in JOINT_ORDER
    ]


def confirm_or_exit(motors: list[LegMotor], current: float) -> None:
    print("=" * 72)
    print("One-leg electrical offset calibration")
    print("=" * 72)
    print("This will command CALIBRATION mode for exactly these 3 motors:")
    for motor in motors:
        print(f"  {motor.channel:8s} id={motor.motor_id:2d}  {motor.name}")
    print(f"calibration_current = {current}")
    print()

    answer = input("Type CALIBRATE to continue: ").strip()
    if answer != "CALIBRATE":
        print("Calibration cancelled.")
        raise SystemExit(1)


def require_all_online(bus: recoil.Bus, motors: list[LegMotor]) -> None:
    offline = []
    print("Pinging motors before calibration...")

    for motor in motors:
        online = bus.ping(motor.motor_id, timeout=PING_TIMEOUT_S)
        status = "online" if online else "offline"
        print(f"  {motor.channel:8s} id={motor.motor_id:2d}  {motor.name:24s} {status}")
        if not online:
            offline.append(motor)
        time.sleep(COMMAND_SPACING_S)

    if offline:
        print()
        print("Calibration aborted because not all motors responded.")
        raise SystemExit(1)


def write_calibration_current(
    bus: recoil.Bus,
    motors: list[LegMotor],
    current: float,
) -> None:
    print()
    print("Writing calibration current...")
    for motor in motors:
        bus.write_motor_calibration_current(motor.motor_id, current)
        print(f"  id={motor.motor_id:2d}  {motor.name}")
        time.sleep(COMMAND_SPACING_S)


def enter_calibration_mode(bus: recoil.Bus, motors: list[LegMotor]) -> None:
    print()
    print("Entering CALIBRATION mode...")
    for motor in motors:
        bus.set_mode(motor.motor_id, recoil.Mode.CALIBRATION)
        print(f"  id={motor.motor_id:2d}  {motor.name}")
        time.sleep(COMMAND_SPACING_S)


def wait_for_calibration(wait_s: float) -> None:
    print()
    print(f"Waiting {wait_s:.1f} seconds for calibration to finish...")
    deadline = time.monotonic() + wait_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        print(f"  {remaining:5.1f}s remaining", end="\r", flush=True)
        time.sleep(min(1.0, remaining))
    print("  done          ")


def main() -> int:
    args = parse_args()
    motors = build_leg_motors(args.leg)

    if not args.yes:
        confirm_or_exit(motors, args.current)

    bus = recoil.Bus(channel=motors[0].channel, bitrate=BITRATE)
    try:
        require_all_online(bus, motors)
        write_calibration_current(bus, motors, args.current)
        enter_calibration_mode(bus, motors)
        wait_for_calibration(args.wait)
    finally:
        bus.stop()

    print()
    print(f"Calibration command sequence finished for {args.leg}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

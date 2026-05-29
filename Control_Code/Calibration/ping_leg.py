#!/usr/bin/env python3
"""
Quick ping check for one leg or the whole quadruped.

This script only checks CAN response with the same low-level ping used by
Calibration/ping.py. It does not command motion, change mode, or write config.

Examples:
    python3 ping_leg.py --leg rear_right
    python3 ping_leg.py --leg front_left
    python3 ping_leg.py --whole-robot
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
DEFAULT_LEG = "rear_right"
PING_ATTEMPTS = 3
PING_TIMEOUT_S = 0.1
PING_SPACING_S = 0.02


@dataclass
class PingTarget:
    leg: str
    joint: str
    channel: str
    motor_id: int

    @property
    def name(self) -> str:
        return f"{self.leg}_{self.joint}"


@dataclass
class PingResult:
    target: PingTarget
    responses: int

    @property
    def online(self) -> bool:
        return self.responses > 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Ping all configured motors for one leg or the whole robot."
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--leg",
        choices=LEG_ORDER,
        default=DEFAULT_LEG,
        help="Leg to ping from Locomotion/leg_config.py.",
    )
    group.add_argument(
        "--whole-robot",
        action="store_true",
        help="Ping every leg and every joint in Locomotion/leg_config.py.",
    )
    parser.add_argument(
        "--attempts",
        type=int,
        default=PING_ATTEMPTS,
        help="Ping attempts per motor.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=PING_TIMEOUT_S,
        help="Timeout in seconds for each ping attempt.",
    )
    return parser.parse_args()


def build_targets(args: argparse.Namespace) -> list[PingTarget]:
    legs = LEG_ORDER if args.whole_robot else [args.leg]

    targets = []
    for leg in legs:
        channel = get_can_channel(leg)
        for joint in JOINT_ORDER:
            targets.append(
                PingTarget(
                    leg=leg,
                    joint=joint,
                    channel=channel,
                    motor_id=get_can_id(leg, joint),
                )
            )
    return targets


def ping_target(target: PingTarget, attempts: int, timeout: float) -> PingResult:
    bus = recoil.Bus(channel=target.channel, bitrate=BITRATE)
    responses = 0

    try:
        for _ in range(attempts):
            if bus.ping(target.motor_id, timeout=timeout):
                responses += 1
            time.sleep(PING_SPACING_S)
    finally:
        bus.stop()

    return PingResult(target=target, responses=responses)


def ping_targets(targets: list[PingTarget], attempts: int, timeout: float) -> list[PingResult]:
    results = []

    for target in targets:
        results.append(ping_target(target, attempts=attempts, timeout=timeout))

    return results


def print_results(results: list[PingResult], attempts: int) -> None:
    print("=" * 72)
    print("Quadruped motor ping check")
    print("=" * 72)
    print(f"{'channel':8s} {'id':>3s}  {'motor_name':24s} {'responses':>9s}  status")
    print("-" * 72)

    for result in results:
        status = "online" if result.online else "offline"
        print(
            f"{result.target.channel:8s} "
            f"{result.target.motor_id:3d}  "
            f"{result.target.name:24s} "
            f"{result.responses:>2d}/{attempts:<6d}  "
            f"{status}"
        )

    online_count = sum(1 for result in results if result.online)
    print("-" * 72)
    print(f"Summary: {online_count}/{len(results)} motors online")


def main() -> int:
    args = parse_args()
    targets = build_targets(args)
    results = ping_targets(targets, attempts=args.attempts, timeout=args.timeout)
    print_results(results, attempts=args.attempts)

    return 0 if all(result.online for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())

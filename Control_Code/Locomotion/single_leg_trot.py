"""
single_leg_trot.py

Run a regular planar cycloidal foot trajectory relative to the leg's current
raw motor position.

This script does not define CAN IDs, motor signs, or homing angles. It imports
all hardware-specific data from leg_config.py and imports foot trajectory math
from trajectory_config.py.

Example:
    python single_leg_trot.py --leg front_right
"""

from __future__ import annotations

import argparse
import math
import signal
import time
from typing import Dict, Iterable, List

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from loop_rate_limiters import RateLimiter  # type: ignore[import-not-found]

from leg_config import (
    FRONT_RIGHT,
    GEAR_RATIO,
    HIP,
    JOINT_ORDER,
    LEG_ORDER,
    NEUTRAL_X,
    NEUTRAL_Y,
    NEUTRAL_Z,
    get_can_channel,
    get_joint_gains,
    get_motor_sign,
    get_role_to_id_for_leg,
)
from trajectory_config import (
    JOINT_ROLES,
    ROLE_HIP,
    ROLE_SHANK,
    ROLE_THIGH,
    TRAJ_REGULAR_PLANAR,
    TrajectoryConfig,
    TrajectoryPoint,
    build_angle_delta_table,
    summarize_angle_delta_table,
)


TRAJECTORY_NAME = TRAJ_REGULAR_PLANAR
TRAJ_CFG = TrajectoryConfig(
    x_center=NEUTRAL_X,
    y_center=NEUTRAL_Y,
    z_ground=NEUTRAL_Z,
    step_length=100.0,
    step_height=70.0,
    step_sideways=0.0,
    stance_ratio=0.50,
    cycle_time=1.20,
    rate_hz=80.0,
    x_forward_sign=1.0,
    y_sideways_sign=1.0,
    z_lift_sign=-1.0,
    heading_deg=45.0,
    z_jump_amplitude=80.0,
)

COMMAND_HIP_TRAJECTORY = False
COMMAND_ROLES = JOINT_ROLES if COMMAND_HIP_TRAJECTORY else (ROLE_THIGH, ROLE_SHANK)
COMMAND_ORDER_ROLES = (ROLE_THIGH, ROLE_SHANK, ROLE_HIP)

RATE_HZ = TRAJ_CFG.rate_hz
PRINT_EVERY = 40
STARTUP_HOLD_TIME = 0.8
MOVE_TO_FIRST_TIME = 0.8

ARM_KP = 0.0
ARM_KD = 0.0
ARM_TORQUE_LIMIT = 0.0

STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MAX_TRAJ_RAW_DELTA_BY_ROLE = {
    ROLE_HIP: 8.0,
    ROLE_THIGH: 13.0,
    ROLE_SHANK: 13.0,
}

STOP_REQUESTED = False


def request_stop(_signum=None, _frame=None) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True
    print("\nStop requested.")


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


class SingleLegRunner:
    """Low-level helper for one configured physical leg."""

    def __init__(self, leg_name: str):
        self.leg_name = leg_name
        self.channel = get_can_channel(leg_name)
        self.role_to_id = get_role_to_id_for_leg(leg_name)
        self.id_to_role = {motor_id: role for role, motor_id in self.role_to_id.items()}
        self.motor_ids = [self.role_to_id[role] for role in JOINT_ORDER]
        self.command_order = [self.role_to_id[role] for role in COMMAND_ORDER_ROLES]
        self.bus = recoil.Bus(channel=self.channel, bitrate=1000000)
        self.rate = RateLimiter(frequency=RATE_HZ)
        self.active_cmd: Dict[int, float] = {}

    def set_mode_with_spacing(self, motor_id: int, mode) -> None:
        self.bus.set_mode(motor_id, mode)
        time.sleep(0.008)
        try:
            self.bus.feed(motor_id)
        except Exception:
            pass
        time.sleep(0.008)

    def set_gains(
        self,
        motor_id: int,
        kp: float,
        kd: float,
        torque_limit: float,
    ) -> None:
        self.bus.write_position_kp(motor_id, kp)
        time.sleep(0.004)
        self.bus.write_position_kd(motor_id, kd)
        time.sleep(0.004)
        self.bus.write_torque_limit(motor_id, torque_limit)
        time.sleep(0.004)

    def set_role_gains_from_config(self, role: str) -> None:
        motor_id = self.role_to_id[role]
        kp, kd, torque_limit = get_joint_gains(self.leg_name, role)
        self.set_gains(motor_id, kp, kd, torque_limit)

    def set_all_config_gains(self) -> None:
        for role in JOINT_ORDER:
            self.set_role_gains_from_config(role)

    def set_all_soft_startup_gains(self) -> None:
        for motor_id in self.motor_ids:
            self.set_gains(motor_id, STARTUP_KP, STARTUP_KD, STARTUP_TORQUE_LIMIT)

    def read_position(self, motor_id: int) -> float:
        value = self.bus.read_position_measured(motor_id)
        if value is None:
            raise RuntimeError(
                f"read_position_measured returned None for ID {motor_id}"
            )
        return float(value)

    def read_all_positions(self, samples: int = 15) -> Dict[int, float]:
        values_by_id: Dict[int, List[float]] = {
            motor_id: []
            for motor_id in self.motor_ids
        }

        for _ in range(samples):
            for motor_id in self.motor_ids:
                try:
                    values_by_id[motor_id].append(self.read_position(motor_id))
                except Exception as exc:
                    role = self.id_to_role[motor_id]
                    print(f"  Read warning for {role} ID {motor_id}: {exc}")
                time.sleep(0.003)
            self.rate.sleep()

        raw = {}
        for motor_id, values in values_by_id.items():
            if not values:
                raise RuntimeError(f"No valid samples for ID {motor_id}")
            values.sort()
            raw[motor_id] = values[len(values) // 2]
        return raw

    def command_position(self, motor_id: int, raw_target: float) -> None:
        self.bus.transmit_pdo_2(motor_id, raw_target, 0.0)
        self.active_cmd[motor_id] = raw_target

    def command_targets(self, targets_by_id: Dict[int, float]) -> None:
        for motor_id in self.command_order:
            if motor_id in targets_by_id:
                self.command_position(motor_id, targets_by_id[motor_id])

    def command_all_active(self) -> None:
        self.command_targets(self.active_cmd)

    def arm_at_current_pose(self) -> Dict[int, float]:
        print("\nPutting motors into IDLE before reading current pose...")
        for motor_id in self.motor_ids:
            self.set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
        time.sleep(0.25)

        print("\nReading current raw positions...")
        start_raw = self.read_all_positions()
        for role in JOINT_ORDER:
            motor_id = self.role_to_id[role]
            print(f"  {role:5s} ID {motor_id}: raw={start_raw[motor_id]:+.6f}")

        print("\nArming POSITION mode at zero torque...")
        for motor_id in self.motor_ids:
            self.set_gains(motor_id, ARM_KP, ARM_KD, ARM_TORQUE_LIMIT)
            self.set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

        self.active_cmd = dict(start_raw)
        for _ in range(int(0.35 * RATE_HZ)):
            self.command_all_active()
            self.rate.sleep()

        print("Applying soft startup gains...")
        self.set_all_soft_startup_gains()
        for _ in range(int(STARTUP_HOLD_TIME * RATE_HZ)):
            self.command_all_active()
            self.rate.sleep()

        return start_raw

    def raw_delta_for_role(self, role: str, angle_delta: float) -> float:
        return get_motor_sign(self.leg_name, role) * angle_delta * GEAR_RATIO

    def targets_from_reference(
        self,
        reference_raw: Dict[int, float],
        point: TrajectoryPoint,
        command_roles: Iterable[str] = COMMAND_ROLES,
    ) -> Dict[int, float]:
        targets = {}
        for role in command_roles:
            motor_id = self.role_to_id[role]
            raw_delta = self.raw_delta_for_role(role, point.angle_delta_by_role[role])
            targets[motor_id] = reference_raw[motor_id] + raw_delta

        if not COMMAND_HIP_TRAJECTORY:
            hip_id = self.role_to_id[HIP]
            targets[hip_id] = reference_raw[hip_id]

        return targets

    def move_to_targets(
        self,
        targets: Dict[int, float],
        move_time: float,
        label: str,
        print_every: int = 0,
    ) -> None:
        print(f"\n{label}")
        start_cmd = dict(self.active_cmd)
        steps = max(1, int(move_time * RATE_HZ))
        print(f"  sending interpolated targets for {steps} control steps")

        for i in range(steps):
            if STOP_REQUESTED:
                return

            u = (i + 1) / steps
            s = 0.5 * (1.0 - math.cos(math.pi * u))
            cmd = {
                motor_id: start_cmd[motor_id] + (target - start_cmd[motor_id]) * s
                for motor_id, target in targets.items()
            }
            self.command_targets(cmd)

            if print_every > 0 and (i + 1) % print_every == 0:
                self.print_feedback()

            self.rate.sleep()

    def print_feedback(self) -> None:
        try:
            measured = self.read_all_positions(samples=1)
        except Exception as exc:
            print(f"read warning: {exc}")
            return

        parts = []
        for motor_id in self.command_order:
            role = self.id_to_role[motor_id]
            cmd = self.active_cmd.get(motor_id, measured[motor_id])
            parts.append(f"{role}:err={cmd - measured[motor_id]:+.3f}")
        print(" | ".join(parts))

    def idle_all(self) -> None:
        print("\nPutting motors into IDLE...")
        for motor_id in self.motor_ids:
            try:
                self.set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
                print(f"  {self.id_to_role[motor_id]} ID {motor_id} IDLE")
            except Exception as exc:
                print(f"  Failed to idle ID {motor_id}: {exc}")
        try:
            self.bus.stop()
        except Exception:
            pass

def print_trajectory_summary(
    runner: SingleLegRunner,
    table: List[TrajectoryPoint],
) -> None:
    summary = summarize_angle_delta_table(table)
    print("\nTrajectory table summary:")

    for role in JOINT_ROLES:
        max_angle = summary[role]["max_abs_angle_delta"]
        max_raw = max(
            abs(runner.raw_delta_for_role(role, point.angle_delta_by_role[role]))
            for point in table
        )
        limit = MAX_TRAJ_RAW_DELTA_BY_ROLE[role]
        print(
            f"  {role:5s}: max angle delta={max_angle:+.6f} rad, "
            f"max raw delta={max_raw:+.3f}, limit={limit:.3f}"
        )
        if max_raw > limit:
            raise RuntimeError(f"{role} trajectory raw delta exceeds safety limit.")


def main() -> None:
    args = parse_args()
    runner = SingleLegRunner(args.leg)

    try:
        print("=" * 80)
        print("single_leg_trot.py")
        print("=" * 80)
        print(f"leg={runner.leg_name}, channel={runner.channel}")
        print(f"trajectory={TRAJECTORY_NAME}, command_hip={COMMAND_HIP_TRAJECTORY}")
        print("role -> CAN ID:")
        for role in JOINT_ORDER:
            print(f"  {role:5s} -> ID {runner.role_to_id[role]}")
        print("=" * 80)

        start_raw = runner.arm_at_current_pose()

        table = build_angle_delta_table(TRAJECTORY_NAME, TRAJ_CFG)
        print_trajectory_summary(runner, table)

        first_targets = runner.targets_from_reference(start_raw, table[0])

        print("\nSwitching to leg_config.py run gains...")
        runner.set_all_config_gains()
        runner.move_to_targets(
            first_targets,
            MOVE_TO_FIRST_TIME,
            "Moving to first trajectory point...",
        )

        print("\nStarting regular planar trajectory. Press Ctrl+C to stop.\n")
        index = 0
        counter = 0

        while not STOP_REQUESTED:
            point = table[index]
            targets = runner.targets_from_reference(start_raw, point)
            runner.command_targets(targets)

            counter += 1
            if counter % PRINT_EVERY == 0:
                foot = point.foot
                deltas = point.angle_delta_by_role
                print(
                    f"phase={point.phase:.3f} {point.phase_name} | "
                    f"x={foot.x:+.1f} y={foot.y:+.1f} z={foot.z:+.1f} | "
                    f"dAng hip={deltas[ROLE_HIP]:+.4f} "
                    f"thigh={deltas[ROLE_THIGH]:+.4f} "
                    f"shank={deltas[ROLE_SHANK]:+.4f}"
                )
                runner.print_feedback()

            index = (index + 1) % len(table)
            runner.rate.sleep()

    except KeyboardInterrupt:
        request_stop()

    finally:
        runner.idle_all()


if __name__ == "__main__":
    main()

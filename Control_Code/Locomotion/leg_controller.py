"""
leg_controller.py

One reusable controller for one physical leg.

Hardware details come from leg_config.py. Trajectory points are pure
output-side angle deltas from trajectory_config.py. This class is the place
where those angle deltas become raw motor targets.
"""

from __future__ import annotations

import time
from typing import Dict, Iterable, Optional

import berkeley_humanoid_lite_lowlevel.recoil as recoil

from gains_config import ARM_GAINS, HOLD_GAINS, GainSet
from leg_config import (
    GEAR_RATIO,
    HIP,
    JOINT_ORDER,
    SHANK,
    THIGH,
    get_can_channel,
    get_max_contraction_angles_for_leg,
    get_motor_sign,
    get_role_to_id_for_leg,
)
from trajectory_config import JOINT_ROLES, TrajectoryPoint, ik_angles_for_xyz


COMMAND_ORDER_ROLES = (THIGH, SHANK, HIP)


class LegController:
    """Controller for one configured physical leg."""

    def __init__(self, name: str, rate_hz: float):
        self.name = name
        self.rate_hz = rate_hz
        self.channel = get_can_channel(name)

        self.role_to_id = get_role_to_id_for_leg(name)
        self.id_to_role = {
            motor_id: role
            for role, motor_id in self.role_to_id.items()
        }

        self.hip_id = self.role_to_id[HIP]
        self.thigh_id = self.role_to_id[THIGH]
        self.shank_id = self.role_to_id[SHANK]

        self.motor_ids = [self.role_to_id[role] for role in JOINT_ORDER]
        self.command_order = [
            self.role_to_id[role]
            for role in COMMAND_ORDER_ROLES
        ]

        self.bus = recoil.Bus(channel=self.channel, bitrate=1000000)

        self.limit_raw: Optional[Dict[int, float]] = None
        self.neutral_raw: Optional[Dict[int, float]] = None
        self.active_cmd: Dict[int, float] = {}
        self.state = "unhomed"

    # ========================================================
    # Low-level API
    # ========================================================

    def set_mode_with_spacing(self, motor_id: int, mode) -> None:
        self.bus.set_mode(motor_id, mode)
        time.sleep(0.006)
        try:
            self.bus.feed(motor_id)
        except Exception:
            pass
        time.sleep(0.006)

    def set_gains(
        self,
        motor_id: int,
        kp: float,
        kd: float,
        torque_limit: float,
    ) -> None:
        self.bus.write_position_kp(motor_id, kp)
        time.sleep(0.003)
        self.bus.write_position_kd(motor_id, kd)
        time.sleep(0.003)
        self.bus.write_torque_limit(motor_id, torque_limit)
        time.sleep(0.003)

    def set_role_gains(self, motor_id: int, gains: GainSet) -> None:
        role = self.id_to_role[motor_id]
        self.set_gains(
            motor_id,
            gains.kp[role],
            gains.kd[role],
            gains.torque[role],
        )

    def set_all_gains(self, gains: GainSet) -> None:
        for motor_id in self.motor_ids:
            self.set_role_gains(motor_id, gains)

    def read_position(self, motor_id: int) -> float:
        value = self.bus.read_position_measured(motor_id)
        if value is None:
            raise RuntimeError(
                f"{self.name}: read_position_measured None "
                f"for ID {motor_id}"
            )
        return float(value)

    def read_all_positions(self) -> Dict[int, float]:
        values = {}
        for motor_id in self.motor_ids:
            values[motor_id] = self.read_position(motor_id)
            time.sleep(0.003)
        return values

    def command_position(self, motor_id: int, raw_target: float) -> None:
        self.bus.transmit_pdo_2(motor_id, raw_target, 0.0)
        self.active_cmd[motor_id] = raw_target

    def command_targets(self, targets_by_id: Dict[int, float]) -> None:
        for motor_id in self.command_order:
            if motor_id in targets_by_id:
                self.command_position(motor_id, targets_by_id[motor_id])

    def command_all_active(self) -> None:
        self.command_targets(self.active_cmd)

    def idle(self) -> None:
        print(f"\n{self.name}: IDLE")
        for motor_id in self.motor_ids:
            try:
                self.set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
                role = self.id_to_role[motor_id]
                print(f"  {self.name} {role} ID {motor_id} IDLE")
            except Exception as exc:
                print(f"  {self.name}: failed to idle ID {motor_id}: {exc}")

    def stop_bus(self) -> None:
        try:
            self.bus.stop()
        except Exception:
            pass

    # ========================================================
    # Conversion helpers
    # ========================================================

    def raw_delta_for_role(self, role: str, angle_delta: float) -> float:
        """Convert output-side joint angle delta into raw motor delta."""
        return get_motor_sign(self.name, role) * angle_delta * GEAR_RATIO

    def raw_targets_from_angle_deltas(
        self,
        reference_raw: Dict[int, float],
        angle_delta_by_role: Dict[str, float],
        command_roles: Iterable[str] = JOINT_ROLES,
    ) -> Dict[int, float]:
        targets = {}
        for role in command_roles:
            motor_id = self.role_to_id[role]
            raw_delta = self.raw_delta_for_role(
                role,
                angle_delta_by_role[role],
            )
            targets[motor_id] = reference_raw[motor_id] + raw_delta
        return targets

    # ========================================================
    # Homing / neutral
    # ========================================================

    def mark_homed_at_current_pose(self) -> None:
        """Record current raw position as this leg's max-contraction pose."""
        print()
        print(f"{self.name}: marking homed at current max-contraction pose...")

        raw = self.read_all_positions()
        self.limit_raw = raw
        self.active_cmd = dict(raw)

        for role in JOINT_ORDER:
            motor_id = self.role_to_id[role]
            print(
                f"  {self.name} {role:5s} ID {motor_id}: "
                f"raw={raw[motor_id]:+.6f}"
            )

        self.set_all_gains(ARM_GAINS)
        for motor_id in self.motor_ids:
            self.set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

        for _ in range(int(0.25 * self.rate_hz)):
            self.command_all_active()
            time.sleep(1.0 / self.rate_hz)

        self.set_all_gains(HOLD_GAINS)
        for _ in range(int(0.25 * self.rate_hz)):
            self.command_all_active()
            time.sleep(1.0 / self.rate_hz)

        self.state = "homed"
        print(f"{self.name}: homed and holding max-contraction pose.")

    def compute_neutral_targets(
        self,
        neutral_xyz: tuple[float, float, float],
    ) -> Dict[int, float]:
        """Compute raw targets from max-contraction pose to neutral pose."""
        if self.limit_raw is None:
            raise RuntimeError(
                f"{self.name}: cannot compute neutral before homing."
            )

        neutral_angles = ik_angles_for_xyz(*neutral_xyz)
        limit_angles = get_max_contraction_angles_for_leg(self.name)

        angle_delta_by_role = {
            role: neutral_angles[role] - limit_angles[role]
            for role in JOINT_ROLES
        }

        return self.raw_targets_from_angle_deltas(
            self.limit_raw,
            angle_delta_by_role,
        )

    def command_interpolated_targets(
        self,
        start_raw: Dict[int, float],
        target_raw: Dict[int, float],
        s: float,
    ) -> None:
        """Command smooth interpolation from start_raw to target_raw."""
        cmd = {
            motor_id: start_raw[motor_id] + (target - start_raw[motor_id]) * s
            for motor_id, target in target_raw.items()
        }
        self.command_targets(cmd)

    def finish_neutral(self, neutral_targets: Dict[int, float]) -> None:
        self.neutral_raw = dict(neutral_targets)
        self.active_cmd.update(neutral_targets)
        self.state = "standing"

    # ========================================================
    # Trajectory
    # ========================================================

    def build_trajectory_targets(
        self,
        point: TrajectoryPoint,
        command_roles: Iterable[str],
        command_hip: bool,
    ) -> Dict[int, float]:
        """Convert a pure trajectory point into absolute raw targets."""
        if self.neutral_raw is None:
            raise RuntimeError(f"{self.name}: neutral_raw not set.")

        targets = self.raw_targets_from_angle_deltas(
            self.neutral_raw,
            point.angle_delta_by_role,
            command_roles,
        )

        if not command_hip:
            targets[self.hip_id] = self.neutral_raw[self.hip_id]

        return targets

    def command_trajectory_point(
        self,
        point: TrajectoryPoint,
        command_roles: Iterable[str],
        command_hip: bool,
    ) -> None:
        targets = self.build_trajectory_targets(
            point,
            command_roles,
            command_hip,
        )
        self.command_targets(targets)

    def feedback_line(self, targets: Optional[Dict[int, float]] = None) -> str:
        try:
            measured = self.read_all_positions()
        except Exception as exc:
            return f"{self.name}: read warning {exc}"

        parts = [self.name]
        for motor_id in self.command_order:
            role = self.id_to_role[motor_id]
            raw = measured[motor_id]

            if targets and motor_id in targets:
                cmd = targets[motor_id]
            elif motor_id in self.active_cmd:
                cmd = self.active_cmd[motor_id]
            else:
                cmd = raw

            parts.append(f"{role}:err={cmd - raw:+.2f}")

        return " ".join(parts)

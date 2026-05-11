"""
leg_controller.py

One reusable controller for one physical leg.

A LegController owns:
    - one CAN channel
    - three motor IDs
    - current state
    - max-contraction raw reference
    - neutral raw reference
    - trajectory command function

The main program should not directly call recoil functions.
"""

from __future__ import annotations

import time
from typing import Dict, Optional

import berkeley_humanoid_lite_lowlevel.recoil as recoil  # type: ignore[import-not-found]
from quadruped_leg_ik import leg_ik
from trajectory_v3 import (
    ROLE_HIP,
    ROLE_THIGH,
    ROLE_SHANK,
    JOINT_ROLES,
    raw_targets_by_id_from_start_raw,
)

from leg_config import (
    ROLE_TO_ID,
    COMMAND_ORDER_ROLES,
    KNOWN_LIMIT_ANGLE_BY_ROLE,
    NEUTRAL_X,
    NEUTRAL_Y,
    NEUTRAL_Z,
    GEAR_RATIO,
    MOTOR_SIGN,
    JOINT_SIGN_BY_ROLE,
    IK_ROLE_TO_PHYSICAL_ROLE,
    STAND_MOVE_SCALE,
    RATE_HZ,
    COMMAND_ROLES,
    COMMAND_HIP_TRAJECTORY,
)
from gains_config import ARM_GAINS, STARTUP_GAINS, MOVE_GAINS, RUN_GAINS, HOLD_GAINS, GainSet


class LegController:
    """One CAN bus, one physical leg."""

    def __init__(self, name: str, channel: str, phase_offset: float):
        self.name = name
        self.channel = channel
        self.phase_offset = phase_offset

        self.role_to_id = dict(ROLE_TO_ID)
        self.id_to_role = {motor_id: role for role, motor_id in self.role_to_id.items()}

        self.hip_id = self.role_to_id[ROLE_HIP]
        self.thigh_id = self.role_to_id[ROLE_THIGH]
        self.shank_id = self.role_to_id[ROLE_SHANK]

        self.motor_ids = [self.hip_id, self.thigh_id, self.shank_id]
        self.command_order = [self.role_to_id[role] for role in COMMAND_ORDER_ROLES]

        self.bus = recoil.Bus(channel=channel, bitrate=1000000)

        self.limit_raw: Optional[Dict[int, float]] = None
        self.neutral_raw: Optional[Dict[int, float]] = None
        self.active_cmd: Dict[int, float] = {}

        self.state = "unhomed"

    # ========================================================
    # Low-level API
    # ========================================================

    def set_mode_with_spacing(self, motor_id, mode):
        self.bus.set_mode(motor_id, mode)
        time.sleep(0.006)
        try:
            self.bus.feed(motor_id)
        except Exception:
            pass
        time.sleep(0.006)

    def set_gains(self, motor_id: int, kp: float, kd: float, torque_limit: float):
        self.bus.write_position_kp(motor_id, kp)
        time.sleep(0.003)
        self.bus.write_position_kd(motor_id, kd)
        time.sleep(0.003)
        self.bus.write_torque_limit(motor_id, torque_limit)
        time.sleep(0.003)

    def set_role_gains(self, motor_id: int, gains: GainSet):
        role = self.id_to_role[motor_id]
        self.set_gains(
            motor_id,
            gains.kp[role],
            gains.kd[role],
            gains.torque[role],
        )

    def set_all_gains(self, gains: GainSet):
        for motor_id in self.motor_ids:
            self.set_role_gains(motor_id, gains)

    def read_position(self, motor_id: int) -> float:
        value = self.bus.read_position_measured(motor_id)
        if value is None:
            raise RuntimeError(f"{self.name}: read_position_measured None for ID {motor_id}")
        return float(value)

    def read_all_positions(self) -> Dict[int, float]:
        values = {}
        for motor_id in self.motor_ids:
            values[motor_id] = self.read_position(motor_id)
            time.sleep(0.003)
        return values

    def command_position(self, motor_id: int, raw_target: float):
        self.bus.transmit_pdo_2(motor_id, raw_target, 0.0)
        self.active_cmd[motor_id] = raw_target

    def command_targets(self, targets_by_id: Dict[int, float]):
        for motor_id in self.command_order:
            if motor_id in targets_by_id:
                self.command_position(motor_id, targets_by_id[motor_id])

    def command_all_active(self):
        self.command_targets(self.active_cmd)

    def idle(self):
        print(f"\n{self.name}: IDLE")
        for motor_id in self.motor_ids:
            try:
                self.set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
                print(f"  {self.name} {self.id_to_role[motor_id]} ID {motor_id} IDLE")
            except Exception as exc:
                print(f"  {self.name}: failed to idle ID {motor_id}: {exc}")

    def stop_bus(self):
        try:
            self.bus.stop()
        except Exception:
            pass

    # ========================================================
    # Homing / neutral
    # ========================================================

    def mark_homed_at_current_pose(self):
        """Record current raw position as known max-contraction pose and hold it."""
        print(f"\n{self.name}: marking homed at current pose...")

        raw = self.read_all_positions()
        self.limit_raw = raw
        self.active_cmd = dict(raw)

        for role in JOINT_ROLES:
            motor_id = self.role_to_id[role]
            print(f"  {self.name} {role:5s} ID {motor_id}: raw={raw[motor_id]:+.6f}")

        self.set_all_gains(ARM_GAINS)
        for motor_id in self.motor_ids:
            self.set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

        for _ in range(int(0.25 * RATE_HZ)):
            self.command_all_active()
            time.sleep(1.0 / RATE_HZ)

        self.set_all_gains(HOLD_GAINS)

        for _ in range(int(0.25 * RATE_HZ)):
            self.command_all_active()
            time.sleep(1.0 / RATE_HZ)

        self.state = "homed"
        print(f"{self.name}: homed and holding max-contraction pose.")

    def compute_neutral_targets(self) -> Dict[int, float]:
        """Compute neutral raw targets from known limit pose."""
        if self.limit_raw is None:
            raise RuntimeError(f"{self.name}: cannot compute neutral before homing.")

        theta_h, theta_t, theta_s = leg_ik(NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z)

        neutral_angles = {
            ROLE_HIP: theta_h,
            ROLE_THIGH: theta_t,
            ROLE_SHANK: theta_s,
        }

        targets = {}

        for ik_role, neutral_angle in neutral_angles.items():
            physical_role = IK_ROLE_TO_PHYSICAL_ROLE[ik_role]
            motor_id = self.role_to_id[physical_role]

            start_angle = KNOWN_LIMIT_ANGLE_BY_ROLE[ik_role]
            delta_angle = neutral_angle - start_angle

            signed_delta = JOINT_SIGN_BY_ROLE[physical_role] * delta_angle
            raw_delta = MOTOR_SIGN * signed_delta * GEAR_RATIO

            targets[motor_id] = self.limit_raw[motor_id] + STAND_MOVE_SCALE * raw_delta

        return targets

    def command_interpolated_targets(self, start_raw, target_raw, s: float):
        """Command smooth interpolation from start_raw to target_raw."""
        cmd = {}
        for motor_id, target in target_raw.items():
            cmd[motor_id] = start_raw[motor_id] + (target - start_raw[motor_id]) * s
        self.command_targets(cmd)

    def finish_neutral(self, neutral_targets: Dict[int, float]):
        self.neutral_raw = dict(neutral_targets)
        self.active_cmd.update(neutral_targets)
        self.state = "standing"

    # ========================================================
    # Trajectory
    # ========================================================

    def build_trajectory_targets(self, point) -> Dict[int, float]:
        """Convert trajectory_v3 point to absolute raw targets."""
        if self.neutral_raw is None:
            raise RuntimeError(f"{self.name}: neutral_raw not set.")

        targets = raw_targets_by_id_from_start_raw(
            self.neutral_raw,
            self.role_to_id,
            point,
            COMMAND_ROLES,
        )

        if not COMMAND_HIP_TRAJECTORY:
            targets[self.hip_id] = self.neutral_raw[self.hip_id]

        return targets

    def command_trajectory_point(self, point):
        targets = self.build_trajectory_targets(point)
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

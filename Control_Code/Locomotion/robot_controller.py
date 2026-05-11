"""
robot_controller.py

High-level robot controller.

This class owns four LegController objects and coordinates:
    - manual homing confirmation
    - simultaneous stand-up
    - neutral hold
    - Xbox-controlled trot
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional

from xbox_controller import XboxController  # type: ignore[import-not-found]
from loop_rate_limiters import RateLimiter  # type: ignore[import-not-found]
from trajectory_v3 import (
    JOINT_ROLES,
    build_relative_command_table,
    summarize_command_table,
)

from leg_config import (
    LEG_CONFIGS,
    RATE_HZ,
    PRINT_EVERY,
    NEUTRAL_HOLD_TIME,
    STAND_MOVE_TIME,
    CONFIRM_IF_RAW_DELTA_OVER,
    ABORT_IF_RAW_DELTA_OVER,
    TRAJECTORY_NAME,
    TRAJ_CFG,
    TRAJ_CONVERSION,
    TRAJECTORY_START_MODE,
    MAX_TRAJ_RAW_DELTA_BY_ROLE,
)
from xbox_config import (
    JOYSTICK_DEADBAND,
    JOYSTICK_FILTER_ALPHA,
    MAX_PHASE_SPEED,
    PHASE_ACCEL_LIMIT,
    ALLOW_REVERSE,
    FREEZE_PHASE_WHEN_STOPPED,
)
from gains_config import MOVE_GAINS, RUN_GAINS, HOLD_GAINS
from gait_scheduler import TrotGaitScheduler
from leg_controller import LegController
from quadruped_main_state import STOP_REQUESTED_REF
from quadruped_utils import apply_deadband, limit_rate, smoothstep, table_index_from_phase, wait_for_terminal_command


class RobotController:
    """Four-leg robot controller."""

    def __init__(self):
        self.legs: Dict[str, LegController] = {}
        for leg_name, cfg in LEG_CONFIGS.items():
            self.legs[leg_name] = LegController(
                name=leg_name,
                channel=cfg["channel"],
                phase_offset=cfg["phase_offset"],
            )

        self.gait = TrotGaitScheduler(
            {name: cfg["phase_offset"] for name, cfg in LEG_CONFIGS.items()}
        )

        self.trajectory_table = None

    def idle_all(self):
        for leg in self.legs.values():
            leg.idle()
        for leg in self.legs.values():
            leg.stop_bus()

    def print_homing_status(self):
        print("\nHoming status:")
        for name in ["leg1", "leg2", "leg3", "leg4"]:
            leg = self.legs[name]
            print(f"  {name} ({leg.channel}): {leg.state}")

    def all_homed(self) -> bool:
        return all(
            leg.state in ("homed", "standing", "running")
            for leg in self.legs.values()
        )

    def mark_leg_homed(self, leg_index: int):
        leg_name = f"leg{leg_index}"
        if leg_name not in self.legs:
            print(f"Unknown leg {leg_name}")
            return
        self.legs[leg_name].mark_homed_at_current_pose()

    def move_all_to_neutral(self):
        if not self.all_homed():
            print("Not all legs are homed.")
            return False

        neutral_targets = {
            name: leg.compute_neutral_targets()
            for name, leg in self.legs.items()
        }

        max_delta = 0.0
        print("\nNeutral move plan:")
        for name, leg in self.legs.items():
            print(f"  {name}:")
            for motor_id, target in neutral_targets[name].items():
                start = leg.limit_raw[motor_id]
                delta = target - start
                max_delta = max(max_delta, abs(delta))
                role = leg.id_to_role[motor_id]
                print(f"    {role:5s}: {start:+.3f} -> {target:+.3f} delta={delta:+.3f}")

        if max_delta > ABORT_IF_RAW_DELTA_OVER:
            raise RuntimeError(f"Neutral raw delta {max_delta:.3f} exceeds abort limit.")

        if max_delta > CONFIRM_IF_RAW_DELTA_OVER:
            answer = wait_for_terminal_command(
                f"\nNeutral max raw delta is {max_delta:.3f}. Move all legs? y/n: "
            )
            if answer not in ("y", "yes"):
                return False

        for leg in self.legs.values():
            leg.set_all_gains(MOVE_GAINS)

        start_raw = {
            name: dict(leg.limit_raw)
            for name, leg in self.legs.items()
        }

        print("\nMoving all legs to neutral together...")
        rate = RateLimiter(frequency=RATE_HZ)
        steps = int(STAND_MOVE_TIME * RATE_HZ)

        for i in range(steps):
            if STOP_REQUESTED_REF["stop"]:
                return False

            s = smoothstep((i + 1) / steps)

            for name, leg in self.legs.items():
                leg.command_interpolated_targets(
                    start_raw[name],
                    neutral_targets[name],
                    s,
                )

            if (i + 1) % PRINT_EVERY == 0:
                print(" | ".join(
                    leg.feedback_line(neutral_targets[name])
                    for name, leg in self.legs.items()
                ))

            rate.sleep()

        for name, leg in self.legs.items():
            leg.finish_neutral(neutral_targets[name])

        print("\nAll legs reached neutral reference.")
        return True

    def hold_neutral(self):
        print(f"\nHolding neutral for {NEUTRAL_HOLD_TIME:.1f}s...")
        for leg in self.legs.values():
            leg.set_all_gains(HOLD_GAINS)

        rate = RateLimiter(frequency=RATE_HZ)
        steps = int(NEUTRAL_HOLD_TIME * RATE_HZ)

        for i in range(steps):
            if STOP_REQUESTED_REF["stop"]:
                return

            for leg in self.legs.values():
                leg.command_all_active()

            if (i + 1) % PRINT_EVERY == 0:
                print(" | ".join(leg.feedback_line() for leg in self.legs.values()))

            rate.sleep()

    def build_trajectory_table(self):
        print("\nBuilding trajectory table from trajectory_v3.py...")
        table = build_relative_command_table(
            TRAJECTORY_NAME,
            TRAJ_CFG,
            TRAJ_CONVERSION,
        )

        summary = summarize_command_table(table)
        for role in JOINT_ROLES:
            max_raw = summary[role]["max_abs_raw_delta"]
            limit = MAX_TRAJ_RAW_DELTA_BY_ROLE[role]
            print(f"  {role:5s}: max raw={max_raw:+.3f}, limit={limit:.3f}")
            if max_raw > limit:
                raise RuntimeError(f"{role} trajectory delta too large.")

        self.trajectory_table = table
        return table

    def move_all_to_start_phase(self) -> float:
        if self.trajectory_table is None:
            raise RuntimeError("Trajectory table not built.")

        if TRAJECTORY_START_MODE == "neutral_mid_stance":
            global_phase = 0.5 * TRAJ_CFG.stance_ratio
        elif TRAJECTORY_START_MODE == "swing_start":
            global_phase = TRAJ_CFG.stance_ratio
        else:
            raise RuntimeError(f"Unknown TRAJECTORY_START_MODE: {TRAJECTORY_START_MODE}")

        print(f"\nMoving all legs to trajectory start phase {global_phase:.3f}...")

        targets = {}
        for name, leg in self.legs.items():
            leg_phase = self.gait.leg_phase(name, global_phase)
            idx = table_index_from_phase(leg_phase, len(self.trajectory_table))
            targets[name] = leg.build_trajectory_targets(self.trajectory_table[idx])

        for leg in self.legs.values():
            leg.set_all_gains(RUN_GAINS)

        start_cmd = {name: dict(leg.active_cmd) for name, leg in self.legs.items()}

        rate = RateLimiter(frequency=RATE_HZ)
        steps = int(0.8 * RATE_HZ)

        for i in range(steps):
            if STOP_REQUESTED_REF["stop"]:
                return global_phase

            s = smoothstep((i + 1) / steps)

            for name, leg in self.legs.items():
                leg.command_interpolated_targets(start_cmd[name], targets[name], s)

            rate.sleep()

        for name, leg in self.legs.items():
            leg.active_cmd.update(targets[name])

        return global_phase

    def run_xbox_trot(self):
        if self.trajectory_table is None:
            self.build_trajectory_table()

        controller = XboxController(deadzone=JOYSTICK_DEADBAND)
        global_phase = self.move_all_to_start_phase()

        print("\nStarting Xbox-controlled trot.")
        print("Left stick Y controls gait phase speed. Ctrl+C to stop.")

        ly_filtered = 0.0
        phase_speed = 0.0
        last_time = time.time()
        counter = 0
        rate = RateLimiter(frequency=RATE_HZ)

        try:
            while not STOP_REQUESTED_REF["stop"]:
                now = time.time()
                dt = now - last_time
                last_time = now

                if dt <= 0.0 or dt > 0.10:
                    dt = 1.0 / RATE_HZ

                state = controller.read()
                raw_ly = max(-1.0, min(1.0, float(state.left_y)))

                ly = apply_deadband(raw_ly, JOYSTICK_DEADBAND)
                ly_filtered = (
                    (1.0 - JOYSTICK_FILTER_ALPHA) * ly_filtered
                    + JOYSTICK_FILTER_ALPHA * ly
                )

                if not ALLOW_REVERSE and ly_filtered < 0.0:
                    ly_filtered = 0.0

                target_phase_speed = MAX_PHASE_SPEED * ly_filtered
                max_phase_step = PHASE_ACCEL_LIMIT * dt
                phase_speed = limit_rate(phase_speed, target_phase_speed, max_phase_step)

                if abs(phase_speed) > 1e-5:
                    global_phase = (global_phase + phase_speed * dt) % 1.0
                elif not FREEZE_PHASE_WHEN_STOPPED:
                    global_phase %= 1.0

                for name, leg in self.legs.items():
                    leg_phase = self.gait.leg_phase(name, global_phase)
                    idx = table_index_from_phase(leg_phase, len(self.trajectory_table))
                    leg.command_trajectory_point(self.trajectory_table[idx])

                counter += 1
                if counter % PRINT_EVERY == 0:
                    print(
                        f"LY_raw={raw_ly:+.2f} LY={ly_filtered:+.2f} "
                        f"phase_speed={phase_speed:+.3f} global_phase={global_phase:.3f}"
                    )
                    print(" | ".join(leg.feedback_line() for leg in self.legs.values()))

                rate.sleep()

        finally:
            try:
                controller.close()
            except Exception:
                pass

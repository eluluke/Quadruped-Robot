"""
robot_controller.py

High-level four-leg robot controller.

This class coordinates:
    - manual homing-pose confirmation for each leg
    - simultaneous move from homing pose to neutral standing
    - neutral hold
    - Xbox-controlled diagonal trot
"""

from __future__ import annotations

import math
import time
from typing import Dict, List

from loop_rate_limiters import RateLimiter  # type: ignore[import-not-found]

from gains_config import HOLD_GAINS, MOVE_GAINS, RUN_GAINS
from gait_scheduler import TROT_PHASE_OFFSETS, TrotGaitScheduler
from leg_config import (
    FRONT_LEFT,
    FRONT_RIGHT,
    LEG_ORDER,
    REAR_LEFT,
    REAR_RIGHT,
    get_neutral_standing_foot_position,
)
from leg_controller import LegController
from quadruped_main_state import STOP_REQUESTED_REF
from quadruped_utils import limit_rate, smoothstep, wait_for_terminal_command
from trajectory_config import (
    JOINT_ROLES,
    ROLE_HIP,
    ROLE_SHANK,
    ROLE_THIGH,
    TRAJECTORY_CONFIG,
    TRAJECTORY_NAME,
    TRAJECTORY_RATE_HZ,
    TrajectoryPoint,
    build_angle_delta_table,
    get_trajectory_config_for_leg,
    summarize_angle_delta_table,
    table_index_from_phase,
)
from xbox_config import (
    ALLOW_REVERSE,
    FREEZE_PHASE_WHEN_STOPPED,
    JOYSTICK_DEADBAND,
    JOYSTICK_FILTER_ALPHA,
    MAX_PHASE_SPEED,
    PHASE_ACCEL_LIMIT,
    XboxController,
    apply_deadband,
)


# ============================================================
# Whole-robot locomotion settings
# ============================================================

RATE_HZ = TRAJECTORY_RATE_HZ
PRINT_EVERY = 40
SHUTDOWN_IDLE_ROUNDS = 4

COMMAND_HIP_TRAJECTORY = False
COMMAND_ROLES = (
    JOINT_ROLES
    if COMMAND_HIP_TRAJECTORY
    else (ROLE_THIGH, ROLE_SHANK)
)

TRAJECTORY_START_MODE = "swing_start"

STAND_MOVE_TIME = 6.00
NEUTRAL_SETTLE_TIME = 2.00
NEUTRAL_HOLD_TIME = 0.8
MOVE_TO_TRAJECTORY_START_TIME = 0.8
PRE_STAND_ARM_HOLD_TIME = 0.35
PRE_STAND_STARTUP_HOLD_TIME = 0.8

CONFIRM_IF_RAW_DELTA_OVER = 4.0
ABORT_IF_RAW_DELTA_OVER = 35.0
NEUTRAL_REACHED_RAW_TOLERANCE = 1.0

MAX_TRAJ_RAW_DELTA_BY_ROLE = {
    ROLE_HIP: 8.0,
    ROLE_THIGH: 13.0,
    ROLE_SHANK: 13.0,
}

LEG_CONFIRM_COMMANDS = {
    "y1": FRONT_LEFT,
    "y2": FRONT_RIGHT,
    "y3": REAR_LEFT,
    "y4": REAR_RIGHT,
}


class RobotController:
    """Four-leg robot controller."""

    def __init__(self):
        self.legs: Dict[str, LegController] = {
            leg_name: LegController(leg_name, rate_hz=RATE_HZ)
            for leg_name in LEG_ORDER
        }
        self.gait = TrotGaitScheduler(TROT_PHASE_OFFSETS)
        self.trajectory_tables: Dict[str, List[TrajectoryPoint]] | None = None
        self._shutdown_complete = False

    # ========================================================
    # Shutdown / status
    # ========================================================

    def idle_all(self) -> None:
        if self._shutdown_complete:
            return

        self._shutdown_complete = True
        STOP_REQUESTED_REF["stop"] = True
        print("\nRobot shutdown: IDLE all motors, zero gains, close buses.")

        for round_index in range(SHUTDOWN_IDLE_ROUNDS):
            print(f"  IDLE round {round_index + 1}/{SHUTDOWN_IDLE_ROUNDS}")
            for leg in self.legs.values():
                leg.idle_once()
            time.sleep(0.05)

        for leg in self.legs.values():
            leg.zero_gains_for_shutdown()

        for leg in self.legs.values():
            leg.active_cmd.clear()

        for leg in self.legs.values():
            leg.stop_bus()

    def print_homing_status(self) -> None:
        print("\nHoming status:")
        for command, leg_name in LEG_CONFIRM_COMMANDS.items():
            leg = self.legs[leg_name]
            print(
                f"  {command:2s} {leg_name:11s} "
                f"({leg.channel}): {leg.state}"
            )

    def all_homed(self) -> bool:
        return all(
            leg.state in ("homed", "standing", "running")
            for leg in self.legs.values()
        )

    def mark_leg_homed(self, command: str) -> None:
        leg_name = LEG_CONFIRM_COMMANDS[command]
        self.legs[leg_name].mark_homed_at_current_pose()

    def record_all_homed_at_current_pose(self) -> None:
        """Record all current leg poses as homed without per-leg hold loops."""
        print("\nRecording all current leg poses as homed...")
        for leg_name, leg in self.legs.items():
            print()
            print(f"{leg_name}:")
            leg.record_homing_pose_only()

    def neutral_tracking_error(
        self,
        neutral_targets: Dict[str, Dict[int, float]],
    ) -> float | None:
        """Print and return max raw tracking error for neutral targets."""
        max_error = 0.0
        for name, leg in self.legs.items():
            try:
                measured = leg.read_all_positions()
            except Exception as exc:
                print(f"  {name}: failed final read: {exc}")
                return None

            parts = []
            for motor_id, target in neutral_targets[name].items():
                error = target - measured[motor_id]
                max_error = max(max_error, abs(error))
                role = leg.id_to_role[motor_id]
                parts.append(f"{role}:err={error:+.2f}")
            print(f"  {name}: " + " ".join(parts))

        return max_error

    def settle_at_neutral(
        self,
        neutral_targets: Dict[str, Dict[int, float]],
        seconds: float,
    ) -> None:
        """Keep commanding neutral targets so loaded legs can catch up."""
        print(f"\nSettling at neutral target for {seconds:.1f}s...")
        rate = RateLimiter(frequency=RATE_HZ)
        steps = max(1, int(seconds * RATE_HZ))

        for _ in range(steps):
            if STOP_REQUESTED_REF["stop"]:
                return

            for name, leg in self.legs.items():
                leg.command_targets(neutral_targets[name])

            rate.sleep()

    # ========================================================
    # Standing
    # ========================================================

    def move_all_to_neutral(
        self,
        confirm_large_move: bool = True,
        reached_raw_tolerance: float = NEUTRAL_REACHED_RAW_TOLERANCE,
        continue_if_tracking_error: bool = False,
    ) -> bool:
        if not self.all_homed():
            print("Not all legs are homed.")
            return False

        neutral_targets = {
            name: leg.compute_neutral_targets(
                get_neutral_standing_foot_position(name)
            )
            for name, leg in self.legs.items()
        }

        max_delta = 0.0
        print("\nNeutral move plan:")
        for name, leg in self.legs.items():
            print(f"  {name}:")
            if leg.home_raw is None:
                raise RuntimeError(f"{name}: missing homed raw reference.")

            for motor_id, target in neutral_targets[name].items():
                start = leg.home_raw[motor_id]
                delta = target - start
                max_delta = max(max_delta, abs(delta))
                role = leg.id_to_role[motor_id]
                print(
                    f"    {role:5s}: {start:+.3f} -> "
                    f"{target:+.3f} delta={delta:+.3f}"
                )

        if max_delta > ABORT_IF_RAW_DELTA_OVER:
            raise RuntimeError(
                f"Neutral raw delta {max_delta:.3f} exceeds abort limit."
            )

        if confirm_large_move and max_delta > CONFIRM_IF_RAW_DELTA_OVER:
            answer = wait_for_terminal_command(
                "\nNeutral max raw delta is "
                f"{max_delta:.3f}. Move all legs? y/n: "
            )
            if answer not in ("y", "yes"):
                return False
        elif max_delta > CONFIRM_IF_RAW_DELTA_OVER:
            print(
                "\nNeutral max raw delta is "
                f"{max_delta:.3f}; continuing from ground-start confirmation."
            )

        print("\nRe-arming all legs before standing move...")
        for leg in self.legs.values():
            leg.arm_position_mode_at_home()

        rate = RateLimiter(frequency=RATE_HZ)
        steps = max(1, int(PRE_STAND_ARM_HOLD_TIME * RATE_HZ))
        for _ in range(steps):
            if STOP_REQUESTED_REF["stop"]:
                return False

            for leg in self.legs.values():
                leg.command_all_active()

            rate.sleep()

        for leg in self.legs.values():
            leg.apply_startup_gains()

        steps = max(1, int(PRE_STAND_STARTUP_HOLD_TIME * RATE_HZ))
        for _ in range(steps):
            if STOP_REQUESTED_REF["stop"]:
                return False

            for leg in self.legs.values():
                leg.command_all_active()

            rate.sleep()

        print("\nSwitching to MOVE_GAINS from gains_config.py...")
        for leg in self.legs.values():
            leg.set_all_gains(MOVE_GAINS)

        start_cmd = {
            name: dict(leg.active_cmd)
            for name, leg in self.legs.items()
        }

        print("\nMoving all legs to neutral together...")
        rate = RateLimiter(frequency=RATE_HZ)
        steps = max(1, int(STAND_MOVE_TIME * RATE_HZ))
        print(f"  sending interpolated targets for {steps} control steps")

        for i in range(steps):
            if STOP_REQUESTED_REF["stop"]:
                return False

            u = (i + 1) / steps
            s = 0.5 * (1.0 - math.cos(math.pi * u))

            for name, leg in self.legs.items():
                leg.command_interpolated_targets(
                    start_cmd[name],
                    neutral_targets[name],
                    s,
                )

            rate.sleep()

        print("\nNeutral final tracking check:")
        max_error = self.neutral_tracking_error(neutral_targets)
        if max_error is None:
            return False

        if max_error > reached_raw_tolerance:
            self.settle_at_neutral(neutral_targets, NEUTRAL_SETTLE_TIME)
            print("\nNeutral tracking check after settle:")
            max_error = self.neutral_tracking_error(neutral_targets)
            if max_error is None:
                return False

        if max_error > reached_raw_tolerance:
            print(
                "\nNeutral move did not track. "
                f"Max raw error={max_error:.2f}, "
                f"limit={reached_raw_tolerance:.2f}."
            )
            if continue_if_tracking_error:
                print(
                    "Continuing because this is a neutral-hold diagnostic "
                    "run. Watch the robot and stop with Ctrl+C if it sags."
                )
            else:
                print(
                    "Stopping before Xbox trot so the robot does not continue "
                    "from a bad standing reference."
                )
                return False

        if max_error <= reached_raw_tolerance:
            print("\nAll legs reached neutral reference.")
        else:
            print(
                "\nNeutral reference has visible tracking error, but the "
                "neutral hold test will keep commanding the same targets."
            )

        for name, leg in self.legs.items():
            leg.finish_neutral(neutral_targets[name])

        return True

    def hold_neutral(self) -> None:
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
                print(
                    " | ".join(
                        leg.feedback_line()
                        for leg in self.legs.values()
                    )
                )

            rate.sleep()

    def hold_neutral_until_stop(self) -> None:
        """Hold neutral standing pose until Ctrl+C or external stop request."""
        print("\nHolding neutral standing pose. Press Ctrl+C to stop.")
        for leg in self.legs.values():
            leg.set_all_gains(HOLD_GAINS)

        rate = RateLimiter(frequency=RATE_HZ)
        counter = 0

        while not STOP_REQUESTED_REF["stop"]:
            for leg in self.legs.values():
                leg.command_all_active()

            counter += 1
            if counter % PRINT_EVERY == 0:
                print(
                    " | ".join(
                        leg.feedback_line()
                        for leg in self.legs.values()
                    )
                )

            rate.sleep()

    # ========================================================
    # Trajectory / trot
    # ========================================================

    def build_trajectory_tables(self) -> Dict[str, List[TrajectoryPoint]]:
        print("\nBuilding per-leg trajectory tables from trajectory_config.py...")
        tables = {}

        for name, leg in self.legs.items():
            cfg = get_trajectory_config_for_leg(name)
            table = build_angle_delta_table(TRAJECTORY_NAME, cfg)
            tables[name] = table
            summary = summarize_angle_delta_table(table)

            print(
                f"  {name}: x_sign={cfg.x_forward_sign:+.1f}, "
                f"z_sign={cfg.z_lift_sign:+.1f}, "
                f"neutral=({cfg.x_center:.2f}, "
                f"{cfg.y_center:.2f}, {cfg.z_ground:.2f})"
            )

            for role in JOINT_ROLES:
                max_angle = summary[role]["max_abs_angle_delta"]
                max_raw = max(
                    abs(
                        leg.raw_delta_for_role(
                            role,
                            point.angle_delta_by_role[role],
                        )
                    )
                    for point in table
                )
                limit = MAX_TRAJ_RAW_DELTA_BY_ROLE[role]
                print(
                    f"    {role:5s}: max angle={max_angle:+.6f} rad, "
                    f"max raw={max_raw:+.3f}, limit={limit:.3f}"
                )
                if max_raw > limit:
                    raise RuntimeError(
                        f"{name} {role} trajectory delta too large."
                    )

        self.trajectory_tables = tables
        return tables

    def trajectory_start_phase(self) -> float:
        if TRAJECTORY_START_MODE == "neutral_mid_stance":
            return 0.5 * TRAJECTORY_CONFIG.stance_ratio
        if TRAJECTORY_START_MODE == "swing_start":
            return TRAJECTORY_CONFIG.stance_ratio
        raise RuntimeError(
            f"Unknown TRAJECTORY_START_MODE: {TRAJECTORY_START_MODE}"
        )

    def move_all_to_start_phase(self) -> float:
        if self.trajectory_tables is None:
            raise RuntimeError("Trajectory tables not built.")

        global_phase = self.trajectory_start_phase()
        print()
        print(
            "Moving all legs to trajectory start phase "
            f"{global_phase:.3f}..."
        )

        targets = {}
        for name, leg in self.legs.items():
            table = self.trajectory_tables[name]
            leg_phase = self.gait.leg_phase(name, global_phase)
            idx = table_index_from_phase(leg_phase, len(table))
            targets[name] = leg.build_trajectory_targets(
                table[idx],
                COMMAND_ROLES,
                COMMAND_HIP_TRAJECTORY,
            )

        for leg in self.legs.values():
            leg.set_all_gains(RUN_GAINS)

        start_cmd = {
            name: dict(leg.active_cmd)
            for name, leg in self.legs.items()
        }
        rate = RateLimiter(frequency=RATE_HZ)
        steps = max(1, int(MOVE_TO_TRAJECTORY_START_TIME * RATE_HZ))

        for i in range(steps):
            if STOP_REQUESTED_REF["stop"]:
                return global_phase

            s = smoothstep((i + 1) / steps)
            for name, leg in self.legs.items():
                leg.command_interpolated_targets(
                    start_cmd[name],
                    targets[name],
                    s,
                )
            rate.sleep()

        for name, leg in self.legs.items():
            leg.active_cmd.update(targets[name])

        return global_phase

    def run_xbox_trot(self) -> None:
        if self.trajectory_tables is None:
            self.build_trajectory_tables()

        if self.trajectory_tables is None:
            raise RuntimeError("Trajectory tables failed to build.")

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
                phase_speed = limit_rate(
                    phase_speed,
                    target_phase_speed,
                    max_phase_step,
                )

                if abs(phase_speed) > 1e-5:
                    global_phase = (global_phase + phase_speed * dt) % 1.0
                elif not FREEZE_PHASE_WHEN_STOPPED:
                    global_phase %= 1.0

                for name, leg in self.legs.items():
                    table = self.trajectory_tables[name]
                    leg_phase = self.gait.leg_phase(name, global_phase)
                    idx = table_index_from_phase(
                        leg_phase,
                        len(table),
                    )
                    leg.command_trajectory_point(
                        table[idx],
                        COMMAND_ROLES,
                        COMMAND_HIP_TRAJECTORY,
                    )

                counter += 1
                if counter % PRINT_EVERY == 0:
                    print(
                        f"LY_raw={raw_ly:+.2f} LY={ly_filtered:+.2f} "
                        f"phase_speed={phase_speed:+.3f} "
                        f"global_phase={global_phase:.3f}"
                    )
                    print(
                        " | ".join(
                            leg.telemetry_line(RUN_GAINS)
                            for leg in self.legs.values()
                        )
                    )

                rate.sleep()

        finally:
            try:
                controller.close()
            except Exception:
                pass

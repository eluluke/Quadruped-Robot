"""
robot_controller.py

High-level four-leg robot controller.

This class coordinates:
    - manual max-contraction confirmation for each leg
    - simultaneous move from max contraction to neutral standing
    - neutral hold
    - Xbox-controlled diagonal trot
"""

from __future__ import annotations

import time
from typing import Dict, List

from loop_rate_limiters import RateLimiter  # type: ignore[import-not-found]

from gait_scheduler import TROT_PHASE_OFFSETS, TrotGaitScheduler
from gains_config import HOLD_GAINS, MOVE_GAINS, RUN_GAINS
from leg_config import (
    FRONT_LEFT,
    FRONT_RIGHT,
    LEG_ORDER,
    REAR_LEFT,
    REAR_RIGHT,
)
from leg_controller import LegController
from quadruped_main_state import STOP_REQUESTED_REF
from quadruped_utils import limit_rate, smoothstep, wait_for_terminal_command
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

RATE_HZ = 80.0
PRINT_EVERY = 40

NEUTRAL_X = 0.0
NEUTRAL_Y = 84.26
NEUTRAL_Z = 378.0
NEUTRAL_XYZ = (NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z)

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
    rate_hz=RATE_HZ,
    x_forward_sign=1.0,
    y_sideways_sign=1.0,
    z_lift_sign=-1.0,
    heading_deg=45.0,
    z_jump_amplitude=80.0,
)

COMMAND_HIP_TRAJECTORY = False
COMMAND_ROLES = (
    JOINT_ROLES
    if COMMAND_HIP_TRAJECTORY
    else (ROLE_THIGH, ROLE_SHANK)
)

TRAJECTORY_START_MODE = "swing_start"

STAND_MOVE_TIME = 3.50
NEUTRAL_HOLD_TIME = 0.8
MOVE_TO_TRAJECTORY_START_TIME = 0.8

CONFIRM_IF_RAW_DELTA_OVER = 4.0
ABORT_IF_RAW_DELTA_OVER = 35.0

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
        self.trajectory_table: List[TrajectoryPoint] | None = None

    # ========================================================
    # Shutdown / status
    # ========================================================

    def idle_all(self) -> None:
        for leg in self.legs.values():
            leg.idle()
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

    # ========================================================
    # Standing
    # ========================================================

    def move_all_to_neutral(self) -> bool:
        if not self.all_homed():
            print("Not all legs are homed.")
            return False

        neutral_targets = {
            name: leg.compute_neutral_targets(NEUTRAL_XYZ)
            for name, leg in self.legs.items()
        }

        max_delta = 0.0
        print("\nNeutral move plan:")
        for name, leg in self.legs.items():
            print(f"  {name}:")
            if leg.limit_raw is None:
                raise RuntimeError(f"{name}: missing homed raw reference.")

            for motor_id, target in neutral_targets[name].items():
                start = leg.limit_raw[motor_id]
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

        if max_delta > CONFIRM_IF_RAW_DELTA_OVER:
            answer = wait_for_terminal_command(
                "\nNeutral max raw delta is "
                f"{max_delta:.3f}. Move all legs? y/n: "
            )
            if answer not in ("y", "yes"):
                return False

        for leg in self.legs.values():
            leg.set_all_gains(MOVE_GAINS)

        start_raw = {
            name: dict(leg.limit_raw or {})
            for name, leg in self.legs.items()
        }

        print("\nMoving all legs to neutral together...")
        rate = RateLimiter(frequency=RATE_HZ)
        steps = max(1, int(STAND_MOVE_TIME * RATE_HZ))

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

    # ========================================================
    # Trajectory / trot
    # ========================================================

    def build_trajectory_table(self) -> List[TrajectoryPoint]:
        print("\nBuilding trajectory table from trajectory_config.py...")
        table = build_angle_delta_table(TRAJECTORY_NAME, TRAJ_CFG)
        summary = summarize_angle_delta_table(table)

        for role in JOINT_ROLES:
            max_angle = summary[role]["max_abs_angle_delta"]
            max_raw = max(
                abs(
                    leg.raw_delta_for_role(
                        role,
                        point.angle_delta_by_role[role],
                    )
                )
                for leg in self.legs.values()
                for point in table
            )
            limit = MAX_TRAJ_RAW_DELTA_BY_ROLE[role]
            print(
                f"  {role:5s}: max angle={max_angle:+.6f} rad, "
                f"max raw={max_raw:+.3f}, limit={limit:.3f}"
            )
            if max_raw > limit:
                raise RuntimeError(f"{role} trajectory delta too large.")

        self.trajectory_table = table
        return table

    def trajectory_start_phase(self) -> float:
        if TRAJECTORY_START_MODE == "neutral_mid_stance":
            return 0.5 * TRAJ_CFG.stance_ratio
        if TRAJECTORY_START_MODE == "swing_start":
            return TRAJ_CFG.stance_ratio
        raise RuntimeError(
            f"Unknown TRAJECTORY_START_MODE: {TRAJECTORY_START_MODE}"
        )

    def move_all_to_start_phase(self) -> float:
        if self.trajectory_table is None:
            raise RuntimeError("Trajectory table not built.")

        global_phase = self.trajectory_start_phase()
        print()
        print(
            "Moving all legs to trajectory start phase "
            f"{global_phase:.3f}..."
        )

        targets = {}
        for name, leg in self.legs.items():
            leg_phase = self.gait.leg_phase(name, global_phase)
            idx = table_index_from_phase(leg_phase, len(self.trajectory_table))
            targets[name] = leg.build_trajectory_targets(
                self.trajectory_table[idx],
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
        if self.trajectory_table is None:
            self.build_trajectory_table()

        if self.trajectory_table is None:
            raise RuntimeError("Trajectory table failed to build.")

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
                    leg_phase = self.gait.leg_phase(name, global_phase)
                    idx = table_index_from_phase(
                        leg_phase,
                        len(self.trajectory_table),
                    )
                    leg.command_trajectory_point(
                        self.trajectory_table[idx],
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
                            leg.feedback_line()
                            for leg in self.legs.values()
                        )
                    )

                rate.sleep()

        finally:
            try:
                controller.close()
            except Exception:
                pass

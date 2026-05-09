"""
trajectory_v1.py

Reusable trajectory + IK helper for the quadruped leg.

No CAN bus code here. This module only:
1. Defines foot-space trajectories.
2. Converts foot xyz into IK joint angles.
3. Converts IK angle deltas into relative raw motor command deltas.
4. Builds reusable command tables for locomotion scripts.

Coordinate convention assumes quadruped_leg_ik.py:
    x = forward/backward
    y = lateral hip offset
    z = positive downward

If z is positive downward, foot lift upward means z decreases.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Tuple, Optional

from quadruped_leg_ik import leg_ik

ROLE_HIP = "hip"
ROLE_THIGH = "thigh"
ROLE_SHANK = "shank"
JOINT_ROLES = (ROLE_HIP, ROLE_THIGH, ROLE_SHANK)


@dataclass
class TrajectoryConfig:
    x_center: float = 0.0
    y_center: float = 84.26
    z_ground: float = 382.0

    step_length: float = 120.0
    step_height: float = 100.0
    stance_ratio: float = 0.50

    cycle_time: float = 1.80
    rate_hz: float = 80.0

    # Clean IK convention: forward -> x, lift -> z.
    x_forward_sign: float = 1.0
    z_lift_sign: float = -1.0

    # Used by tilted_planar.
    heading_deg: float = 45.0

    # Used by vertical_jump.
    z_jump_amplitude: float = 80.0


@dataclass
class MotorConversionConfig:
    gear_ratio: float = 17.0
    motor_sign: float = -1.0

    hip_sign: float = 1.0
    thigh_sign: float = 1.0
    shank_sign: float = 1.0

    enable_hip_deadband: bool = True
    hip_delta_deadband_rad: float = 1e-4

    def sign_for_role(self, role: str) -> float:
        if role == ROLE_HIP:
            return self.hip_sign
        if role == ROLE_THIGH:
            return self.thigh_sign
        if role == ROLE_SHANK:
            return self.shank_sign
        raise ValueError(f"Unknown role: {role}")


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_phase(phase: float) -> float:
    return phase % 1.0


def raw_delta_from_angle_delta(angle_delta: float, conversion: MotorConversionConfig) -> float:
    return conversion.motor_sign * angle_delta * conversion.gear_ratio


def ik_angles_for_xyz(x: float, y: float, z: float) -> Dict[str, float]:
    theta_h, theta_t, theta_s = leg_ik(x, y, z)
    return {
        ROLE_HIP: theta_h,
        ROLE_THIGH: theta_t,
        ROLE_SHANK: theta_s,
    }


def cycloid_forward_lift(
    phase: float,
    step_length: float,
    step_height: float,
    stance_ratio: float,
) -> Tuple[float, float, str]:
    """
    Abstract gait curve.

    Returns:
        forward: fore/aft coordinate along selected travel direction.
        lift: positive scalar lift amount.
        phase_name: 'stance' or 'swing'.
    """
    phase = wrap_phase(phase)
    stance_ratio = clamp(stance_ratio, 0.05, 0.95)

    if phase < stance_ratio:
        u = phase / stance_ratio
        forward = step_length / 2.0 - step_length * u
        lift = 0.0
        return forward, lift, "stance"

    u = (phase - stance_ratio) / (1.0 - stance_ratio)
    forward = -step_length / 2.0 + step_length * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )
    lift = step_height * (1.0 - math.cos(2.0 * math.pi * u)) / 2.0
    return forward, lift, "swing"


def regular_planar_xyz(phase: float, cfg: TrajectoryConfig) -> Dict[str, float]:
    """
    Regular x-z planar cycloidal trajectory.
    y is constant, so hip IK delta should be near zero.
    """
    forward, lift, phase_name = cycloid_forward_lift(
        phase, cfg.step_length, cfg.step_height, cfg.stance_ratio
    )
    x = cfg.x_center + cfg.x_forward_sign * forward
    y = cfg.y_center
    z = cfg.z_ground + cfg.z_lift_sign * lift
    return {
        "x": x,
        "y": y,
        "z": z,
        "forward": forward,
        "lift": lift,
        "phase_name": phase_name,
    }


def tilted_planar_xyz(phase: float, cfg: TrajectoryConfig) -> Dict[str, float]:
    """
    Diagonal planar cycloidal trajectory.

    heading_deg rotates the horizontal travel direction in the x-y plane:
        0 deg  -> pure x direction
        45 deg -> diagonal x/y direction
        90 deg -> pure y direction

    Because y changes, IK naturally produces hip motion.
    """
    forward, lift, phase_name = cycloid_forward_lift(
        phase, cfg.step_length, cfg.step_height, cfg.stance_ratio
    )
    heading = math.radians(cfg.heading_deg)
    dx = math.cos(heading) * cfg.x_forward_sign * forward
    dy = math.sin(heading) * cfg.x_forward_sign * forward
    x = cfg.x_center + dx
    y = cfg.y_center + dy
    z = cfg.z_ground + cfg.z_lift_sign * lift
    return {
        "x": x,
        "y": y,
        "z": z,
        "forward": forward,
        "lift": lift,
        "heading_deg": cfg.heading_deg,
        "phase_name": phase_name,
    }


def vertical_jump_xyz(phase: float, cfg: TrajectoryConfig) -> Dict[str, float]:
    """
    Straight-line z-axis motion in IK frame.
    x and y stay fixed; z moves smoothly up/down.
    """
    phase = wrap_phase(phase)
    lift = cfg.z_jump_amplitude * (1.0 - math.cos(2.0 * math.pi * phase)) / 2.0
    x = cfg.x_center
    y = cfg.y_center
    z = cfg.z_ground + cfg.z_lift_sign * lift
    return {
        "x": x,
        "y": y,
        "z": z,
        "forward": 0.0,
        "lift": lift,
        "phase_name": "vertical_jump",
    }


def foot_xyz_for_phase(phase: float, trajectory_name: str, cfg: TrajectoryConfig) -> Dict[str, float]:
    if trajectory_name == "regular_planar":
        return regular_planar_xyz(phase, cfg)
    if trajectory_name == "tilted_planar":
        return tilted_planar_xyz(phase, cfg)
    if trajectory_name == "vertical_jump":
        return vertical_jump_xyz(phase, cfg)
    raise ValueError(
        f"Unknown trajectory_name={trajectory_name!r}. "
        "Use 'regular_planar', 'tilted_planar', or 'vertical_jump'."
    )


def nominal_reference_angles(cfg: TrajectoryConfig) -> Dict[str, float]:
    return ik_angles_for_xyz(cfg.x_center, cfg.y_center, cfg.z_ground)


def joint_angle_deltas_for_phase(
    phase: float,
    trajectory_name: str,
    cfg: TrajectoryConfig,
    conversion: MotorConversionConfig,
    reference_angles: Optional[Dict[str, float]] = None,
) -> Dict[str, object]:
    if reference_angles is None:
        reference_angles = nominal_reference_angles(cfg)

    foot = foot_xyz_for_phase(phase, trajectory_name, cfg)
    angles = ik_angles_for_xyz(foot["x"], foot["y"], foot["z"])

    angle_delta_by_role: Dict[str, float] = {}
    raw_delta_by_role: Dict[str, float] = {}

    for role in JOINT_ROLES:
        delta = angles[role] - reference_angles[role]
        delta *= conversion.sign_for_role(role)

        if (
            role == ROLE_HIP
            and conversion.enable_hip_deadband
            and abs(delta) < conversion.hip_delta_deadband_rad
        ):
            delta = 0.0

        angle_delta_by_role[role] = delta
        raw_delta_by_role[role] = raw_delta_from_angle_delta(delta, conversion)

    return {
        "phase": wrap_phase(phase),
        "trajectory_name": trajectory_name,
        "phase_name": foot["phase_name"],
        "foot": foot,
        "angles": angles,
        "reference_angles": reference_angles,
        "angle_delta_by_role": angle_delta_by_role,
        "raw_delta_by_role": raw_delta_by_role,
    }


def build_relative_command_table(
    trajectory_name: str,
    cfg: TrajectoryConfig,
    conversion: MotorConversionConfig,
    num_points: Optional[int] = None,
) -> List[Dict[str, object]]:
    """
    Build reusable relative trajectory table.

    The table does NOT include absolute raw targets, because those depend on
    startup encoder positions. Locomotion code should do:
        target = startup_raw + raw_delta
    """
    if num_points is None:
        num_points = int(cfg.cycle_time * cfg.rate_hz)
    if num_points < 2:
        raise ValueError("num_points must be at least 2.")

    reference_angles = nominal_reference_angles(cfg)
    table: List[Dict[str, object]] = []

    for i in range(num_points):
        phase = i / num_points
        table.append(
            joint_angle_deltas_for_phase(
                phase=phase,
                trajectory_name=trajectory_name,
                cfg=cfg,
                conversion=conversion,
                reference_angles=reference_angles,
            )
        )

    return table


def summarize_command_table(table: List[Dict[str, object]]) -> Dict[str, Dict[str, float]]:
    summary = {
        role: {"max_abs_angle_delta": 0.0, "max_abs_raw_delta": 0.0}
        for role in JOINT_ROLES
    }

    for point in table:
        angle_deltas = point["angle_delta_by_role"]
        raw_deltas = point["raw_delta_by_role"]
        for role in JOINT_ROLES:
            summary[role]["max_abs_angle_delta"] = max(
                summary[role]["max_abs_angle_delta"], abs(angle_deltas[role])
            )
            summary[role]["max_abs_raw_delta"] = max(
                summary[role]["max_abs_raw_delta"], abs(raw_deltas[role])
            )

    return summary


def raw_targets_from_start_raw(
    start_raw_by_role: Dict[str, float],
    point: Dict[str, object],
    command_roles: Tuple[str, ...] = JOINT_ROLES,
) -> Dict[str, float]:
    raw_delta_by_role = point["raw_delta_by_role"]
    return {
        role: start_raw_by_role[role] + raw_delta_by_role[role]
        for role in command_roles
    }


def raw_targets_by_id_from_start_raw(
    start_raw_by_id: Dict[int, float],
    role_to_id: Dict[str, int],
    point: Dict[str, object],
    command_roles: Tuple[str, ...] = JOINT_ROLES,
) -> Dict[int, float]:
    raw_delta_by_role = point["raw_delta_by_role"]
    targets: Dict[int, float] = {}
    for role in command_roles:
        motor_id = role_to_id[role]
        targets[motor_id] = start_raw_by_id[motor_id] + raw_delta_by_role[role]
    return targets

"""
trajectory_v2_fixed.py

Reusable trajectory + IK helper for one quadruped leg.

This module contains NO CAN bus code and NO Xbox code.

It only:
    1. Defines foot trajectories.
    2. Maps local trajectory coordinates into the IK frame.
    3. Runs leg_ik(x, y, z).
    4. Computes relative joint angle deltas.
    5. Converts joint deltas into relative raw motor command deltas.
    6. Builds reusable command tables.

Important fix compared with trajectory_v2.py:
    - Mypy-friendly return types.
    - Explicit rotation matrix for tilted trajectory.
    - No Dict[str, float] annotations for dictionaries that contain strings.
    - Uses Point = Dict[str, Any] so table entries are indexable in locomotion code.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Optional, Tuple

from quadruped_leg_ik import leg_ik  # type: ignore[import-not-found]


# ============================================================
# Joint role constants
# ============================================================

ROLE_HIP = "hip"
ROLE_THIGH = "thigh"
ROLE_SHANK = "shank"

JOINT_ROLES: Tuple[str, str, str] = (
    ROLE_HIP,
    ROLE_THIGH,
    ROLE_SHANK,
)

Point = Dict[str, Any]


# ============================================================
# Config objects
# ============================================================

@dataclass
class TrajectoryConfig:
    """Trajectory settings in the IK frame."""

    x_center: float = 0.0
    y_center: float = 84.26
    z_ground: float = 382.0

    step_length: float = 80.0
    step_height: float = 60.0
    step_sideways: float = 0.0

    stance_ratio: float = 0.50

    cycle_time: float = 2.20
    rate_hz: float = 80.0

    x_forward_sign: float = 1.0
    y_sideways_sign: float = 1.0
    z_lift_sign: float = -1.0

    heading_deg: float = 45.0
    z_jump_amplitude: float = 80.0


@dataclass
class MotorConversionConfig:
    """Conversion from IK angle delta to raw motor delta."""

    gear_ratio: float = 17.0
    motor_sign: float = -1.0

    hip_sign: float = 1.0
    thigh_sign: float = 1.0
    shank_sign: float = 1.0

    enable_hip_deadband: bool = True
    hip_delta_deadband_rad: float = 1e-4

    def sign_for_role(self, role: str) -> float:
        """Return per-joint sign correction."""
        if role == ROLE_HIP:
            return self.hip_sign
        if role == ROLE_THIGH:
            return self.thigh_sign
        if role == ROLE_SHANK:
            return self.shank_sign
        raise ValueError(f"Unknown joint role: {role}")


# ============================================================
# Math helpers
# ============================================================

def clamp(value: float, low: float, high: float) -> float:
    """Clamp a value to [low, high]."""
    return max(low, min(high, value))


def wrap_phase(phase: float) -> float:
    """Wrap phase to [0, 1)."""
    return phase % 1.0


def rotate_xy_matrix(local_x: float, local_y: float, angle_deg: float) -> Tuple[float, float]:
    """
    Rotate a 2D vector in the x-y plane.

        [x_rot]   [cosθ  -sinθ] [local_x]
        [y_rot] = [sinθ   cosθ] [local_y]
    """
    theta = math.radians(angle_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    x_rot = cos_t * local_x - sin_t * local_y
    y_rot = sin_t * local_x + cos_t * local_y

    return x_rot, y_rot


def raw_delta_from_angle_delta(
    angle_delta: float,
    conversion: MotorConversionConfig,
) -> float:
    """Convert an output-side joint angle delta to raw motor delta."""
    return conversion.motor_sign * angle_delta * conversion.gear_ratio


def ik_angles_for_xyz(x: float, y: float, z: float) -> Dict[str, float]:
    """Return IK angles by joint role."""
    theta_h, theta_t, theta_s = leg_ik(x, y, z)
    return {
        ROLE_HIP: theta_h,
        ROLE_THIGH: theta_t,
        ROLE_SHANK: theta_s,
    }


# ============================================================
# Local trajectory helpers
# ============================================================

def cycloid_forward_lift(
    phase: float,
    step_length: float,
    step_height: float,
    stance_ratio: float,
) -> Tuple[float, float, str]:
    """
    Return local forward/lift values for one gait cycle.

    forward:
        local fore-aft trajectory coordinate.

    lift:
        positive lift amount. Mapping to z is controlled by z_lift_sign.
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


def cycloid_sideways(phase: float, step_sideways: float, stance_ratio: float) -> float:
    """
    Optional local sideways stroke.

    Normally keep step_sideways = 0.0.
    """
    if abs(step_sideways) < 1e-12:
        return 0.0

    phase = wrap_phase(phase)
    stance_ratio = clamp(stance_ratio, 0.05, 0.95)

    if phase < stance_ratio:
        u = phase / stance_ratio
        return step_sideways / 2.0 - step_sideways * u

    u = (phase - stance_ratio) / (1.0 - stance_ratio)
    return -step_sideways / 2.0 + step_sideways * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )


# ============================================================
# Trajectory definitions
# ============================================================

def regular_planar_xyz(phase: float, cfg: TrajectoryConfig) -> Point:
    """
    Regular planar x-z trajectory.

    Local x/y is rotated by 0 degrees.
    """
    forward, lift, phase_name = cycloid_forward_lift(
        phase,
        cfg.step_length,
        cfg.step_height,
        cfg.stance_ratio,
    )

    sideways = cycloid_sideways(phase, cfg.step_sideways, cfg.stance_ratio)

    local_x = cfg.x_forward_sign * forward
    local_y = cfg.y_sideways_sign * sideways

    dx, dy = rotate_xy_matrix(local_x, local_y, 0.0)

    x = cfg.x_center + dx
    y = cfg.y_center + dy
    z = cfg.z_ground + cfg.z_lift_sign * lift

    return {
        "x": x,
        "y": y,
        "z": z,
        "local_x": local_x,
        "local_y": local_y,
        "rotated_dx": dx,
        "rotated_dy": dy,
        "forward": forward,
        "sideways": sideways,
        "lift": lift,
        "heading_deg": 0.0,
        "phase_name": phase_name,
    }


def tilted_planar_xyz(phase: float, cfg: TrajectoryConfig) -> Point:
    """
    Tilted/diagonal planar trajectory using explicit rotation matrix.

    Local trajectory:
        local_x = forward
        local_y = sideways

    IK frame:
        [dx, dy] = R(heading_deg) @ [local_x, local_y]
    """
    forward, lift, phase_name = cycloid_forward_lift(
        phase,
        cfg.step_length,
        cfg.step_height,
        cfg.stance_ratio,
    )

    sideways = cycloid_sideways(phase, cfg.step_sideways, cfg.stance_ratio)

    local_x = cfg.x_forward_sign * forward
    local_y = cfg.y_sideways_sign * sideways

    dx, dy = rotate_xy_matrix(local_x, local_y, cfg.heading_deg)

    x = cfg.x_center + dx
    y = cfg.y_center + dy
    z = cfg.z_ground + cfg.z_lift_sign * lift

    return {
        "x": x,
        "y": y,
        "z": z,
        "local_x": local_x,
        "local_y": local_y,
        "rotated_dx": dx,
        "rotated_dy": dy,
        "forward": forward,
        "sideways": sideways,
        "lift": lift,
        "heading_deg": cfg.heading_deg,
        "phase_name": phase_name,
    }


def vertical_jump_xyz(phase: float, cfg: TrajectoryConfig) -> Point:
    """
    Straight-line vertical z-axis trajectory.

    x and y stay fixed.
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
        "local_x": 0.0,
        "local_y": 0.0,
        "rotated_dx": 0.0,
        "rotated_dy": 0.0,
        "forward": 0.0,
        "sideways": 0.0,
        "lift": lift,
        "heading_deg": None,
        "phase_name": "vertical_jump",
    }


def foot_xyz_for_phase(phase: float, trajectory_name: str, cfg: TrajectoryConfig) -> Point:
    """Return foot point for selected trajectory."""
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


# ============================================================
# IK delta + command table helpers
# ============================================================

def nominal_reference_angles(cfg: TrajectoryConfig) -> Dict[str, float]:
    """Return IK angles at nominal reference pose."""
    return ik_angles_for_xyz(cfg.x_center, cfg.y_center, cfg.z_ground)


def joint_angle_deltas_for_phase(
    phase: float,
    trajectory_name: str,
    cfg: TrajectoryConfig,
    conversion: MotorConversionConfig,
    reference_angles: Optional[Dict[str, float]] = None,
) -> Point:
    """
    Compute one trajectory point.

    The returned dictionary contains:
        foot
        IK angles
        angle_delta_by_role
        raw_delta_by_role
    """
    if reference_angles is None:
        reference_angles = nominal_reference_angles(cfg)

    foot = foot_xyz_for_phase(phase, trajectory_name, cfg)
    angles = ik_angles_for_xyz(float(foot["x"]), float(foot["y"]), float(foot["z"]))

    angle_delta_by_role: Dict[str, float] = {}
    raw_delta_by_role: Dict[str, float] = {}

    for joint_role in JOINT_ROLES:
        delta = angles[joint_role] - reference_angles[joint_role]
        delta *= conversion.sign_for_role(joint_role)

        if (
            joint_role == ROLE_HIP
            and conversion.enable_hip_deadband
            and abs(delta) < conversion.hip_delta_deadband_rad
        ):
            delta = 0.0

        angle_delta_by_role[joint_role] = delta
        raw_delta_by_role[joint_role] = raw_delta_from_angle_delta(delta, conversion)

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
) -> List[Point]:
    """
    Build a reusable relative command table.

    The table stores relative raw deltas, not absolute motor targets.
    """
    if num_points is None:
        num_points = int(cfg.cycle_time * cfg.rate_hz)

    if num_points < 2:
        raise ValueError("num_points must be at least 2.")

    reference_angles = nominal_reference_angles(cfg)
    table: List[Point] = []

    for i in range(num_points):
        phase = i / num_points
        point = joint_angle_deltas_for_phase(
            phase,
            trajectory_name,
            cfg,
            conversion,
            reference_angles,
        )
        table.append(point)

    return table


def summarize_command_table(table: List[Point]) -> Dict[str, Dict[str, float]]:
    """Return max absolute angle/raw delta by joint role."""
    summary: Dict[str, Dict[str, float]] = {
        joint_role: {
            "max_abs_angle_delta": 0.0,
            "max_abs_raw_delta": 0.0,
        }
        for joint_role in JOINT_ROLES
    }

    for point in table:
        angle_deltas = point["angle_delta_by_role"]
        raw_deltas = point["raw_delta_by_role"]

        for joint_role in JOINT_ROLES:
            summary[joint_role]["max_abs_angle_delta"] = max(
                summary[joint_role]["max_abs_angle_delta"],
                abs(angle_deltas[joint_role]),
            )

            summary[joint_role]["max_abs_raw_delta"] = max(
                summary[joint_role]["max_abs_raw_delta"],
                abs(raw_deltas[joint_role]),
            )

    return summary


def raw_targets_from_start_raw(
    start_raw_by_role: Dict[str, float],
    point: Point,
    command_roles: Tuple[str, ...] = JOINT_ROLES,
) -> Dict[str, float]:
    """Convert relative raw deltas to absolute raw targets by role."""
    raw_delta_by_role = point["raw_delta_by_role"]

    targets: Dict[str, float] = {}
    for joint_role in command_roles:
        targets[joint_role] = start_raw_by_role[joint_role] + raw_delta_by_role[joint_role]

    return targets


def raw_targets_by_id_from_start_raw(
    start_raw_by_id: Dict[int, float],
    role_to_id: Dict[str, int],
    point: Point,
    command_roles: Tuple[str, ...] = JOINT_ROLES,
) -> Dict[int, float]:
    """Convert relative raw deltas to absolute raw targets by CAN ID."""
    raw_delta_by_role = point["raw_delta_by_role"]

    targets: Dict[int, float] = {}
    for joint_role in command_roles:
        motor_id = role_to_id[joint_role]
        targets[motor_id] = start_raw_by_id[motor_id] + raw_delta_by_role[joint_role]

    return targets

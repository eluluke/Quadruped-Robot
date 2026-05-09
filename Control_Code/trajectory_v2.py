"""
trajectory_v2.py

Reusable trajectory + IK helper for one quadruped leg.

This file contains NO CAN bus code and NO Xbox code.

Main improvement from trajectory_v1.py:
- The tilted planar trajectory now uses an explicit 2D rotation matrix.
- We define a local trajectory point (local_x, local_y), then rotate it into
  the IK x-y plane:

      [dx]   [cos(theta)  -sin(theta)] [local_x]
      [dy] = [sin(theta)   cos(theta)] [local_y]

Coordinate convention:
- x = forward/backward direction in IK frame
- y = lateral hip-offset direction in IK frame
- z = positive downward in IK frame

Trajectories included:
1. regular_planar: normal x-z cycloidal gait plane.
2. tilted_planar: same local curve, rotated in x-y by heading_deg.
3. vertical_jump: straight z-axis jumping/compression trajectory.

This module computes all 3 IK deltas: hip, thigh, shank.
The locomotion code decides whether to command hip, hold hip, or filter hip.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Tuple

from quadruped_leg_ik import leg_ik


# ============================================================
# Joint role constants
# ============================================================

ROLE_HIP = "hip"
ROLE_THIGH = "thigh"
ROLE_SHANK = "shank"

JOINT_ROLES = (
    ROLE_HIP,
    ROLE_THIGH,
    ROLE_SHANK,
)


# ============================================================
# Configuration objects
# ============================================================

@dataclass
class TrajectoryConfig:
    """All foot-trajectory settings bundled into one object."""

    # Nominal foot center / reference stance point in IK frame.
    x_center: float = 0.0
    y_center: float = 84.26
    z_ground: float = 382.0

    # Cycloidal gait size.
    step_length: float = 120.0
    step_height: float = 100.0

    # Optional local sideways stroke before rotation.
    # For most tests, leave this as 0.0.
    # For future side-stepping, you can make this nonzero.
    step_sideways: float = 0.0

    # phase < stance_ratio -> stance stroke
    # phase >= stance_ratio -> swing/lift return
    stance_ratio: float = 0.50

    # Timing/table settings.
    cycle_time: float = 1.80
    rate_hz: float = 80.0

    # Local-to-IK signs.
    x_forward_sign: float = 1.0
    y_sideways_sign: float = 1.0

    # If z is positive downward, upward foot lift should usually be -1.0.
    z_lift_sign: float = -1.0

    # Used by tilted_planar.
    # 0 deg -> local x aligns with IK x.
    # 45 deg -> local x becomes diagonal x-y motion.
    # 90 deg -> local x aligns with IK y.
    heading_deg: float = 45.0

    # Used by vertical_jump.
    z_jump_amplitude: float = 80.0


@dataclass
class MotorConversionConfig:
    """Converts IK angle deltas into raw motor deltas."""

    gear_ratio: float = 17.0
    motor_sign: float = -1.0

    hip_sign: float = 1.0
    thigh_sign: float = 1.0
    shank_sign: float = 1.0

    # Useful for regular planar trajectories where hip should be nearly zero.
    enable_hip_deadband: bool = True
    hip_delta_deadband_rad: float = 1e-4

    def sign_for_role(self, role: str) -> float:
        if role == ROLE_HIP:
            return self.hip_sign
        if role == ROLE_THIGH:
            return self.thigh_sign
        if role == ROLE_SHANK:
            return self.shank_sign
        raise ValueError(f"Unknown joint role: {role}")


# ============================================================
# Basic math helpers
# ============================================================

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_phase(phase: float) -> float:
    return phase % 1.0


def rotate_xy_matrix(local_x: float, local_y: float, angle_deg: float) -> Tuple[float, float]:
    """
    Rotate a 2D vector in the x-y plane using the standard rotation matrix.

        [x_rot]   [cos(theta)  -sin(theta)] [local_x]
        [y_rot] = [sin(theta)   cos(theta)] [local_y]

    This is the pure matrix version for tilted/diagonal trajectories.
    """
    theta = math.radians(angle_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    x_rot = cos_t * local_x - sin_t * local_y
    y_rot = sin_t * local_x + cos_t * local_y

    return x_rot, y_rot


def raw_delta_from_angle_delta(angle_delta: float, conversion: MotorConversionConfig) -> float:
    return conversion.motor_sign * angle_delta * conversion.gear_ratio


def ik_angles_for_xyz(x: float, y: float, z: float) -> Dict[str, float]:
    theta_h, theta_t, theta_s = leg_ik(x, y, z)
    return {
        ROLE_HIP: theta_h,
        ROLE_THIGH: theta_t,
        ROLE_SHANK: theta_s,
    }


# ============================================================
# Local cycloidal path helpers
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
        local fore-aft trajectory coordinate

    lift:
        positive scalar lift amount

    phase_name:
        "stance" or "swing"
    """
    phase = wrap_phase(phase)
    stance_ratio = clamp(stance_ratio, 0.05, 0.95)

    if phase < stance_ratio:
        u = phase / stance_ratio

        # Straight stance stroke: +L/2 -> -L/2.
        forward = step_length / 2.0 - step_length * u
        lift = 0.0
        return forward, lift, "stance"

    u = (phase - stance_ratio) / (1.0 - stance_ratio)

    # Smooth cycloidal swing: -L/2 -> +L/2.
    forward = -step_length / 2.0 + step_length * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )

    # Smooth lift: 0 -> step_height -> 0.
    lift = step_height * (1.0 - math.cos(2.0 * math.pi * u)) / 2.0

    return forward, lift, "swing"


def cycloid_sideways(phase: float, step_sideways: float, stance_ratio: float) -> float:
    """
    Optional local sideways trajectory component.

    For normal walking tests, step_sideways = 0.0.
    For future side-stepping, this creates a smooth local sideways stroke.
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

def regular_planar_xyz(phase: float, cfg: TrajectoryConfig) -> Dict[str, float]:
    """
    Regular planar x-z trajectory.

    local_x = forward
    local_y = optional sideways, normally 0
    rotation angle = 0 deg
    """
    forward, lift, phase_name = cycloid_forward_lift(
        phase=phase,
        step_length=cfg.step_length,
        step_height=cfg.step_height,
        stance_ratio=cfg.stance_ratio,
    )

    sideways = cycloid_sideways(
        phase=phase,
        step_sideways=cfg.step_sideways,
        stance_ratio=cfg.stance_ratio,
    )

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


def tilted_planar_xyz(phase: float, cfg: TrajectoryConfig) -> Dict[str, float]:
    """
    Tilted / diagonal planar trajectory using an explicit rotation matrix.

    Local trajectory before rotation:
        local_x = forward
        local_y = sideways

    Then rotate local_x/local_y into the IK x-y plane:

        [dx]   [cos(theta)  -sin(theta)] [local_x]
        [dy] = [sin(theta)   cos(theta)] [local_y]

    Important:
    If local_y = 0 and heading_deg = 45, the foot still moves diagonally
    because local forward becomes both dx and dy after rotation.

    If local_y is nonzero, then the whole local 2D trajectory is rotated.
    """
    forward, lift, phase_name = cycloid_forward_lift(
        phase=phase,
        step_length=cfg.step_length,
        step_height=cfg.step_height,
        stance_ratio=cfg.stance_ratio,
    )

    sideways = cycloid_sideways(
        phase=phase,
        step_sideways=cfg.step_sideways,
        stance_ratio=cfg.stance_ratio,
    )

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


def vertical_jump_xyz(phase: float, cfg: TrajectoryConfig) -> Dict[str, float]:
    """
    Straight-line vertical z-axis trajectory.

    x and y stay fixed. z moves smoothly up/down.
    """
    phase = wrap_phase(phase)

    lift = cfg.z_jump_amplitude * (
        1.0 - math.cos(2.0 * math.pi * phase)
    ) / 2.0

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


# ============================================================
# IK delta + raw command table helpers
# ============================================================

def nominal_reference_angles(cfg: TrajectoryConfig) -> Dict[str, float]:
    """IK angles at the nominal reference pose."""
    return ik_angles_for_xyz(cfg.x_center, cfg.y_center, cfg.z_ground)


def joint_angle_deltas_for_phase(
    phase: float,
    trajectory_name: str,
    cfg: TrajectoryConfig,
    conversion: MotorConversionConfig,
    reference_angles: Dict[str, float] | None = None,
) -> Dict[str, object]:
    """
    Compute one trajectory point:
        foot xyz
        IK angles
        IK angle deltas
        raw motor deltas
    """
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
    num_points: int | None = None,
) -> List[Dict[str, object]]:
    """
    Build a reusable relative trajectory table.

    The table stores IK angle deltas and raw motor deltas.
    It does NOT store absolute raw motor targets because those depend on the
    measured startup raw positions in the locomotion code.
    """
    if num_points is None:
        num_points = int(cfg.cycle_time * cfg.rate_hz)

    if num_points < 2:
        raise ValueError("num_points must be at least 2.")

    reference_angles = nominal_reference_angles(cfg)
    table: List[Dict[str, object]] = []

    for i in range(num_points):
        phase = i / num_points
        point = joint_angle_deltas_for_phase(
            phase=phase,
            trajectory_name=trajectory_name,
            cfg=cfg,
            conversion=conversion,
            reference_angles=reference_angles,
        )
        table.append(point)

    return table


def summarize_command_table(table: List[Dict[str, object]]) -> Dict[str, Dict[str, float]]:
    """Compute max absolute IK angle delta and raw motor delta for each role."""
    summary = {
        role: {
            "max_abs_angle_delta": 0.0,
            "max_abs_raw_delta": 0.0,
        }
        for role in JOINT_ROLES
    }

    for point in table:
        angle_deltas = point["angle_delta_by_role"]
        raw_deltas = point["raw_delta_by_role"]

        for role in JOINT_ROLES:
            summary[role]["max_abs_angle_delta"] = max(
                summary[role]["max_abs_angle_delta"],
                abs(angle_deltas[role]),
            )
            summary[role]["max_abs_raw_delta"] = max(
                summary[role]["max_abs_raw_delta"],
                abs(raw_deltas[role]),
            )

    return summary


def raw_targets_from_start_raw(
    start_raw_by_role: Dict[str, float],
    point: Dict[str, object],
    command_roles: Tuple[str, ...] = JOINT_ROLES,
) -> Dict[str, float]:
    """Convert one relative trajectory point into absolute raw targets by role."""
    raw_delta_by_role = point["raw_delta_by_role"]
    targets: Dict[str, float] = {}

    for role in command_roles:
        targets[role] = start_raw_by_role[role] + raw_delta_by_role[role]

    return targets


def raw_targets_by_id_from_start_raw(
    start_raw_by_id: Dict[int, float],
    role_to_id: Dict[str, int],
    point: Dict[str, object],
    command_roles: Tuple[str, ...] = JOINT_ROLES,
) -> Dict[int, float]:
    """Convert one relative trajectory point into absolute raw targets by CAN ID."""
    raw_delta_by_role = point["raw_delta_by_role"]
    targets: Dict[int, float] = {}

    for role in command_roles:
        motor_id = role_to_id[role]
        targets[motor_id] = start_raw_by_id[motor_id] + raw_delta_by_role[role]

    return targets

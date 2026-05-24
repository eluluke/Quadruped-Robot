"""
trajectory_config.py

Pure trajectory and inverse-kinematics helpers for the quadruped.

This module intentionally contains no CAN IDs, no raw motor units, no gear
ratio, and no motor direction signs. It works only in the robot/IK frame:

    foot phase -> foot xyz -> IK joint angles -> output-side angle deltas

The hardware layer should convert these output-side angle deltas into raw
motor commands using leg_config.py, because each physical leg can have its
own CAN IDs, signs, and max-contraction homing angles.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Dict, List, Optional, Tuple

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


# ============================================================
# Trajectory names
# ============================================================

TRAJ_REGULAR_PLANAR = "regular_planar"
TRAJ_TILTED_PLANAR = "tilted_planar"
TRAJ_VERTICAL_JUMP = "vertical_jump"


# ============================================================
# Data objects
# ============================================================

@dataclass(frozen=True)
class TrajectoryConfig:
    """Foot trajectory settings in the IK frame, with distances in mm."""

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


@dataclass(frozen=True)
class FootPoint:
    """One foot target in the IK frame."""

    x: float
    y: float
    z: float
    local_x: float
    local_y: float
    rotated_dx: float
    rotated_dy: float
    forward: float
    sideways: float
    lift: float
    heading_deg: Optional[float]
    phase_name: str


@dataclass(frozen=True)
class TrajectoryPoint:
    """One trajectory sample with pure output-side joint angles."""

    phase: float
    trajectory_name: str
    phase_name: str
    foot: FootPoint
    angles_by_role: Dict[str, float]
    reference_angles_by_role: Dict[str, float]
    angle_delta_by_role: Dict[str, float]


# ============================================================
# Math helpers
# ============================================================

def clamp(value: float, low: float, high: float) -> float:
    """Clamp a value to [low, high]."""
    return max(low, min(high, value))


def wrap_phase(phase: float) -> float:
    """Wrap phase to [0, 1)."""
    return phase % 1.0


def table_index_from_phase(phase: float, table_len: int) -> int:
    """Convert a continuous phase to a command table index."""
    if table_len <= 0:
        raise ValueError("table_len must be positive.")
    return int(wrap_phase(phase) * table_len) % table_len


def rotate_xy(
    local_x: float,
    local_y: float,
    angle_deg: float,
) -> Tuple[float, float]:
    """Rotate a vector in the x-y plane by angle_deg."""
    if abs(angle_deg) < 1e-12:
        return local_x, local_y

    theta = math.radians(angle_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return (
        cos_t * local_x - sin_t * local_y,
        sin_t * local_x + cos_t * local_y,
    )


def ik_angles_for_xyz(x: float, y: float, z: float) -> Dict[str, float]:
    """Return output-side IK angles by joint role."""
    theta_h, theta_t, theta_s = leg_ik(x, y, z)
    return {
        ROLE_HIP: theta_h,
        ROLE_THIGH: theta_t,
        ROLE_SHANK: theta_s,
    }


# ============================================================
# Local trajectory primitives
# ============================================================

def cycloid_forward_lift(
    phase: float,
    step_length: float,
    step_height: float,
    stance_ratio: float,
) -> Tuple[float, float, str]:
    """
    Return local forward coordinate, positive lift amount, and phase name.

    During stance the foot moves in a straight pullback. During swing it uses
    a cycloid, giving smooth lift-off and touchdown.
    """
    phase = wrap_phase(phase)
    stance_ratio = clamp(stance_ratio, 0.05, 0.95)

    if phase < stance_ratio:
        u = phase / stance_ratio
        return step_length * (0.5 - u), 0.0, "stance"

    u = (phase - stance_ratio) / (1.0 - stance_ratio)
    two_pi_u = 2.0 * math.pi * u

    forward = -0.5 * step_length + step_length * (
        u - math.sin(two_pi_u) / (2.0 * math.pi)
    )
    lift = 0.5 * step_height * (1.0 - math.cos(two_pi_u))

    return forward, lift, "swing"


def cycloid_sideways(
    phase: float,
    step_sideways: float,
    stance_ratio: float,
) -> float:
    """Return optional local sideways stroke for the current phase."""
    if abs(step_sideways) < 1e-12:
        return 0.0

    forward_like, _lift, _phase_name = cycloid_forward_lift(
        phase,
        step_sideways,
        0.0,
        stance_ratio,
    )
    return forward_like


# ============================================================
# Foot trajectories
# ============================================================

def planar_foot_point(
    phase: float,
    cfg: TrajectoryConfig,
    heading_deg: float,
) -> FootPoint:
    """Return a regular or tilted planar stepping point."""
    forward, lift, phase_name = cycloid_forward_lift(
        phase,
        cfg.step_length,
        cfg.step_height,
        cfg.stance_ratio,
    )
    sideways = cycloid_sideways(phase, cfg.step_sideways, cfg.stance_ratio)

    local_x = cfg.x_forward_sign * forward
    local_y = cfg.y_sideways_sign * sideways
    dx, dy = rotate_xy(local_x, local_y, heading_deg)

    return FootPoint(
        x=cfg.x_center + dx,
        y=cfg.y_center + dy,
        z=cfg.z_ground + cfg.z_lift_sign * lift,
        local_x=local_x,
        local_y=local_y,
        rotated_dx=dx,
        rotated_dy=dy,
        forward=forward,
        sideways=sideways,
        lift=lift,
        heading_deg=heading_deg,
        phase_name=phase_name,
    )


def regular_planar_xyz(phase: float, cfg: TrajectoryConfig) -> FootPoint:
    """Planar x-z stepping trajectory with no heading rotation."""
    return planar_foot_point(phase, cfg, heading_deg=0.0)


def tilted_planar_xyz(phase: float, cfg: TrajectoryConfig) -> FootPoint:
    """Planar stepping trajectory rotated in the x-y plane."""
    return planar_foot_point(phase, cfg, heading_deg=cfg.heading_deg)


def vertical_jump_xyz(phase: float, cfg: TrajectoryConfig) -> FootPoint:
    """Vertical z-axis trajectory with fixed x and y."""
    phase = wrap_phase(phase)
    lift = 0.5 * cfg.z_jump_amplitude * (1.0 - math.cos(2.0 * math.pi * phase))

    return FootPoint(
        x=cfg.x_center,
        y=cfg.y_center,
        z=cfg.z_ground + cfg.z_lift_sign * lift,
        local_x=0.0,
        local_y=0.0,
        rotated_dx=0.0,
        rotated_dy=0.0,
        forward=0.0,
        sideways=0.0,
        lift=lift,
        heading_deg=None,
        phase_name=TRAJ_VERTICAL_JUMP,
    )


TrajectoryFunction = Callable[[float, TrajectoryConfig], FootPoint]

TRAJECTORY_FUNCTIONS: Dict[str, TrajectoryFunction] = {
    TRAJ_REGULAR_PLANAR: regular_planar_xyz,
    TRAJ_TILTED_PLANAR: tilted_planar_xyz,
    TRAJ_VERTICAL_JUMP: vertical_jump_xyz,
}


def foot_xyz_for_phase(
    phase: float,
    trajectory_name: str,
    cfg: TrajectoryConfig,
) -> FootPoint:
    """Return the foot target for a named trajectory."""
    try:
        trajectory_fn = TRAJECTORY_FUNCTIONS[trajectory_name]
    except KeyError as exc:
        names = ", ".join(sorted(TRAJECTORY_FUNCTIONS))
        raise ValueError(
            f"Unknown trajectory_name={trajectory_name!r}. Use: {names}."
        ) from exc

    return trajectory_fn(phase, cfg)


# ============================================================
# IK angle delta table
# ============================================================

def nominal_reference_angles(cfg: TrajectoryConfig) -> Dict[str, float]:
    """Return IK angles at the nominal neutral/reference pose."""
    return ik_angles_for_xyz(cfg.x_center, cfg.y_center, cfg.z_ground)


def joint_angle_deltas_for_phase(
    phase: float,
    trajectory_name: str,
    cfg: TrajectoryConfig,
    reference_angles: Optional[Dict[str, float]] = None,
) -> TrajectoryPoint:
    """
    Compute one pure trajectory sample.

    angle_delta_by_role is output-side joint angle delta in radians:

        delta = IK(current foot target) - IK(reference foot target)
    """
    if reference_angles is None:
        reference_angles = nominal_reference_angles(cfg)

    foot = foot_xyz_for_phase(phase, trajectory_name, cfg)
    angles = ik_angles_for_xyz(foot.x, foot.y, foot.z)

    angle_delta_by_role = {
        role: angles[role] - reference_angles[role]
        for role in JOINT_ROLES
    }

    return TrajectoryPoint(
        phase=wrap_phase(phase),
        trajectory_name=trajectory_name,
        phase_name=foot.phase_name,
        foot=foot,
        angles_by_role=angles,
        reference_angles_by_role=dict(reference_angles),
        angle_delta_by_role=angle_delta_by_role,
    )


def build_angle_delta_table(
    trajectory_name: str,
    cfg: TrajectoryConfig,
    num_points: Optional[int] = None,
) -> List[TrajectoryPoint]:
    """
    Build one cycle of output-side joint angle deltas.

    Controllers should convert these deltas to raw motor commands using the
    per-leg hardware data in leg_config.py.
    """
    if num_points is None:
        num_points = round(cfg.cycle_time * cfg.rate_hz)

    if num_points < 2:
        raise ValueError("num_points must be at least 2.")

    reference_angles = nominal_reference_angles(cfg)

    return [
        joint_angle_deltas_for_phase(
            phase=i / num_points,
            trajectory_name=trajectory_name,
            cfg=cfg,
            reference_angles=reference_angles,
        )
        for i in range(num_points)
    ]


def summarize_angle_delta_table(
    table: List[TrajectoryPoint],
) -> Dict[str, Dict[str, float]]:
    """Return max absolute output-side angle delta by joint role."""
    summary = {
        role: {
            "max_abs_angle_delta": 0.0,
            "min_angle_delta": 0.0,
            "max_angle_delta": 0.0,
        }
        for role in JOINT_ROLES
    }

    for point in table:
        for role in JOINT_ROLES:
            delta = point.angle_delta_by_role[role]
            summary[role]["max_abs_angle_delta"] = max(
                summary[role]["max_abs_angle_delta"],
                abs(delta),
            )
            summary[role]["min_angle_delta"] = min(
                summary[role]["min_angle_delta"],
                delta,
            )
            summary[role]["max_angle_delta"] = max(
                summary[role]["max_angle_delta"],
                delta,
            )

    return summary

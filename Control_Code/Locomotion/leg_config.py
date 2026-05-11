"""
leg_config.py

All robot-specific configuration for the quadruped project.

This file should contain parameters, not control logic.

The main program should not care about low-level motor IDs, gains, known
mechanical-limit angles, or trajectory numbers. It imports them from here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

from trajectory_v3 import (
    ROLE_HIP,
    ROLE_THIGH,
    ROLE_SHANK,
    JOINT_ROLES,
    TrajectoryConfig,
    MotorConversionConfig,
)


# ============================================================
# CAN / leg layout
# ============================================================

# Each CAN bus controls one leg.
# Rename leg1/leg2/leg3/leg4 to FL/FR/RL/RR later if you want.
LEG_CONFIGS = {
    "leg1": {"channel": "can1", "phase_offset": 0.0},
    "leg2": {"channel": "can2", "phase_offset": 0.5},
    "leg3": {"channel": "can3", "phase_offset": 0.0},
    "leg4": {"channel": "can4", "phase_offset": 0.5},
}

# Same CAN IDs on each leg bus.
ROLE_TO_ID: Dict[str, int] = {
    ROLE_HIP: 1,
    ROLE_THIGH: 2,
    ROLE_SHANK: 3,
}

COMMAND_ORDER_ROLES = (ROLE_THIGH, ROLE_SHANK, ROLE_HIP)


# ============================================================
# Known mechanical reference pose
# ============================================================

# Joint angles at the known max-contraction/mechanical-limit pose.
KNOWN_LIMIT_ANGLE_BY_ROLE: Dict[str, float] = {
    ROLE_HIP: 1.069,
    ROLE_THIGH: 1.199,
    ROLE_SHANK: 2.688,
}


# ============================================================
# Neutral standing pose in IK frame
# ============================================================

NEUTRAL_X = 0.0
NEUTRAL_Y = 84.26
NEUTRAL_Z = 378.0


# ============================================================
# Motor conversion signs
# ============================================================

GEAR_RATIO = 17.0
MOTOR_SIGN = 1.0

JOINT_SIGN_BY_ROLE: Dict[str, float] = {
    ROLE_HIP: 1.0,
    ROLE_THIGH: 1.0,
    ROLE_SHANK: 1.0,
}

IK_ROLE_TO_PHYSICAL_ROLE: Dict[str, str] = {
    ROLE_HIP: ROLE_HIP,
    ROLE_THIGH: ROLE_THIGH,
    ROLE_SHANK: ROLE_SHANK,
}


# ============================================================
# Standing / homing settings
# ============================================================

STAND_MOVE_SCALE = 1.0
STAND_MOVE_TIME = 3.50
NEUTRAL_HOLD_TIME = 0.8

CONFIRM_IF_RAW_DELTA_OVER = 4.0
ABORT_IF_RAW_DELTA_OVER = 35.0


# ============================================================
# Trajectory settings
# ============================================================

TRAJECTORY_NAME = "regular_planar"  # regular_planar, tilted_planar, vertical_jump

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

    # Working physical signs from one-leg tests.
    x_forward_sign=1.0,
    y_sideways_sign=1.0,
    z_lift_sign=-1.0,

    heading_deg=45.0,
    z_jump_amplitude=80.0,
)

TRAJ_CONVERSION = MotorConversionConfig(
    gear_ratio=GEAR_RATIO,
    motor_sign=MOTOR_SIGN,

    hip_sign=JOINT_SIGN_BY_ROLE[ROLE_HIP],
    thigh_sign=JOINT_SIGN_BY_ROLE[ROLE_THIGH],
    shank_sign=JOINT_SIGN_BY_ROLE[ROLE_SHANK],

    enable_hip_deadband=True,
    hip_delta_deadband_rad=1e-4,
)

# For regular_planar, hold hip at neutral raw to reduce sag.
# For tilted_planar, you may need True.
COMMAND_HIP_TRAJECTORY = False

COMMAND_ROLES: Tuple[str, ...] = (
    (ROLE_HIP, ROLE_THIGH, ROLE_SHANK)
    if COMMAND_HIP_TRAJECTORY
    else (ROLE_THIGH, ROLE_SHANK)
)

# First visible motion after neutral.
TRAJECTORY_START_MODE = "swing_start"  # swing_start or neutral_mid_stance

MAX_TRAJ_RAW_DELTA_BY_ROLE: Dict[str, float] = {
    ROLE_HIP: 8.0,
    ROLE_THIGH: 13.0,
    ROLE_SHANK: 13.0,
}


# ============================================================
# Timing
# ============================================================

RATE_HZ = 80.0
PRINT_EVERY = 40

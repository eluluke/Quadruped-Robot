"""
gains_config.py

Motor gain sets for each control state.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from trajectory_v3 import ROLE_HIP, ROLE_THIGH, ROLE_SHANK


@dataclass
class GainSet:
    kp: Dict[str, float]
    kd: Dict[str, float]
    torque: Dict[str, float]


ARM_GAINS = GainSet(
    kp={ROLE_HIP: 0.0, ROLE_THIGH: 0.0, ROLE_SHANK: 0.0},
    kd={ROLE_HIP: 0.0, ROLE_THIGH: 0.0, ROLE_SHANK: 0.0},
    torque={ROLE_HIP: 0.0, ROLE_THIGH: 0.0, ROLE_SHANK: 0.0},
)

STARTUP_GAINS = GainSet(
    kp={ROLE_HIP: 0.003, ROLE_THIGH: 0.003, ROLE_SHANK: 0.003},
    kd={ROLE_HIP: 0.001, ROLE_THIGH: 0.001, ROLE_SHANK: 0.001},
    torque={ROLE_HIP: 0.03, ROLE_THIGH: 0.03, ROLE_SHANK: 0.03},
)

MOVE_GAINS = GainSet(
    kp={ROLE_HIP: 0.20, ROLE_THIGH: 0.20, ROLE_SHANK: 0.20},
    kd={ROLE_HIP: 0.008, ROLE_THIGH: 0.008, ROLE_SHANK: 0.008},
    torque={ROLE_HIP: 1.30, ROLE_THIGH: 0.75, ROLE_SHANK: 0.75},
)

RUN_GAINS = GainSet(
    kp={ROLE_HIP: 0.10, ROLE_THIGH: 0.20, ROLE_SHANK: 0.20},
    kd={ROLE_HIP: 0.010, ROLE_THIGH: 0.008, ROLE_SHANK: 0.008},
    torque={ROLE_HIP: 1.50, ROLE_THIGH: 0.80, ROLE_SHANK: 0.80},
)

HOLD_GAINS = GainSet(
    kp={ROLE_HIP: 0.12, ROLE_THIGH: 0.10, ROLE_SHANK: 0.10},
    kd={ROLE_HIP: 0.010, ROLE_THIGH: 0.006, ROLE_SHANK: 0.006},
    torque={ROLE_HIP: 1.50, ROLE_THIGH: 0.80, ROLE_SHANK: 0.80},
)

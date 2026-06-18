"""
gains_config.py

Shared motor gain sets for high-level quadruped states.

All motor controller gains live here. The hardware file leg_config.py should
only describe CAN IDs, motor signs, homing angles, and geometry constants.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from leg_config import HIP, SHANK, THIGH


@dataclass(frozen=True)
class GainSet:
    kp: Dict[str, float]
    kd: Dict[str, float]
    torque: Dict[str, float]


ARM_GAINS = GainSet(
    kp={HIP: 0.0, THIGH: 0.0, SHANK: 0.0},
    kd={HIP: 0.0, THIGH: 0.0, SHANK: 0.0},
    torque={HIP: 0.0, THIGH: 0.0, SHANK: 0.0},
)

STARTUP_GAINS = GainSet(
    kp={HIP: 0.003, THIGH: 0.003, SHANK: 0.003},
    kd={HIP: 0.001, THIGH: 0.001, SHANK: 0.001},
    torque={HIP: 0.03, THIGH: 0.03, SHANK: 0.03},
)

MOVE_GAINS = GainSet(
    kp={HIP: 0.50, THIGH: 0.50, SHANK: 0.50},
    kd={HIP: 0.030, THIGH: 0.030, SHANK: 0.030},
    torque={HIP: 2.50, THIGH: 2.50, SHANK: 2.50},
)

RUN_GAINS = GainSet(
    kp={HIP: 0.50, THIGH: 0.50, SHANK: 0.50},
    kd={HIP: 0.030, THIGH: 0.030, SHANK: 0.030},
    torque={HIP: 2.50, THIGH: 2.50, SHANK: 2.50},
)

HOLD_GAINS = GainSet(
    kp={HIP: 0.50, THIGH: 0.50, SHANK: 0.50},
    kd={HIP: 0.030, THIGH: 0.030, SHANK: 0.030},
    torque={HIP: 2.50, THIGH: 2.50, SHANK: 2.50},
)

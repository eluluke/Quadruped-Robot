"""
gains_config.py

Shared motor gain sets for high-level quadruped states.

Per-joint hardware gains still live in leg_config.py. These named gain sets
are used during startup, standing transitions, hold, and trot.
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
    kp={HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
    kd={HIP: 0.008, THIGH: 0.008, SHANK: 0.008},
    torque={HIP: 1.30, THIGH: 0.75, SHANK: 0.75},
)

RUN_GAINS = GainSet(
    kp={HIP: 0.10, THIGH: 0.20, SHANK: 0.20},
    kd={HIP: 0.010, THIGH: 0.008, SHANK: 0.008},
    torque={HIP: 1.50, THIGH: 0.80, SHANK: 0.80},
)

HOLD_GAINS = GainSet(
    kp={HIP: 0.12, THIGH: 0.10, SHANK: 0.10},
    kd={HIP: 0.010, THIGH: 0.006, SHANK: 0.006},
    torque={HIP: 1.50, THIGH: 0.80, SHANK: 0.80},
)

"""
gains_config.py

Shared motor gain sets for high-level quadruped states.

All motor controller gains live here. The hardware file leg_config.py should
only describe CAN IDs, motor signs, homing angles, and geometry constants.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict

from leg_config import (
    FRONT_LEFT,
    FRONT_RIGHT,
    HIP,
    REAR_LEFT,
    REAR_RIGHT,
    SHANK,
    THIGH,
)


@dataclass(frozen=True)
class GainSet:
    kp: Dict[str, float] = field(default_factory=dict)
    kd: Dict[str, float] = field(default_factory=dict)
    torque: Dict[str, float] = field(default_factory=dict)
    leg_kp: Dict[str, Dict[str, float]] = field(default_factory=dict)
    leg_kd: Dict[str, Dict[str, float]] = field(default_factory=dict)
    leg_torque: Dict[str, Dict[str, float]] = field(default_factory=dict)

    def values_for(self, leg_name: str, role: str) -> tuple[float, float, float]:
        """Return kp, kd, torque for one physical leg joint."""
        kp = self.leg_kp.get(leg_name, {}).get(role)
        kd = self.leg_kd.get(leg_name, {}).get(role)
        torque = self.leg_torque.get(leg_name, {}).get(role)

        if kp is None:
            kp = self.kp[role]
        if kd is None:
            kd = self.kd[role]
        if torque is None:
            torque = self.torque[role]

        return kp, kd, torque


def _all_legs(hip: float, thigh: float, shank: float) -> Dict[str, Dict[str, float]]:
    """Create an explicit per-leg table that is easy to tune by hand."""
    joint_values = {
        HIP: hip,
        THIGH: thigh,
        SHANK: shank,
    }
    return {
        FRONT_LEFT: dict(joint_values),
        FRONT_RIGHT: dict(joint_values),
        REAR_LEFT: dict(joint_values),
        REAR_RIGHT: dict(joint_values),
    }


ARM_GAINS = GainSet(
    kp={HIP: 0.0, THIGH: 0.0, SHANK: 0.0},
    kd={HIP: 0.0, THIGH: 0.0, SHANK: 0.0},
    torque={HIP: 0.0, THIGH: 0.0, SHANK: 0.0},
    leg_kp=_all_legs(0.0, 0.0, 0.0),
    leg_kd=_all_legs(0.0, 0.0, 0.0),
    leg_torque=_all_legs(0.0, 0.0, 0.0),
)

STARTUP_GAINS = GainSet(
    kp={HIP: 0.003, THIGH: 0.003, SHANK: 0.003},
    kd={HIP: 0.001, THIGH: 0.001, SHANK: 0.001},
    torque={HIP: 0.03, THIGH: 0.03, SHANK: 0.03},
    leg_kp=_all_legs(0.003, 0.003, 0.003),
    leg_kd=_all_legs(0.001, 0.001, 0.001),
    leg_torque=_all_legs(0.03, 0.03, 0.03),
)

MOVE_GAINS = GainSet(
    leg_kp={
        FRONT_LEFT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
        FRONT_RIGHT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
        REAR_LEFT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
        REAR_RIGHT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
    },
    leg_kd={
        FRONT_LEFT: {HIP: 0.01, THIGH: 0.01, SHANK: 0.01},
        FRONT_RIGHT: {HIP: 0.01, THIGH: 0.01, SHANK: 0.01},
        REAR_LEFT: {HIP: 0.01, THIGH: 0.01, SHANK: 0.01},
        REAR_RIGHT: {HIP: 0.01, THIGH: 0.01, SHANK: 0.01},
    },
    leg_torque={
        FRONT_LEFT: {HIP: 1.00, THIGH: 1.00, SHANK: 1.00},
        FRONT_RIGHT: {HIP: 1.00, THIGH: 1.00, SHANK: 1.00},
        REAR_LEFT: {HIP: 1.00, THIGH: 1.00, SHANK: 1.00},
        REAR_RIGHT: {HIP: 1.00, THIGH: 1.00, SHANK: 1.00},
    },
)

RUN_GAINS = GainSet(
    leg_kp={
        FRONT_LEFT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
        FRONT_RIGHT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
        REAR_LEFT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
        REAR_RIGHT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
    },
    leg_kd={
        FRONT_LEFT: {HIP: 0.015, THIGH: 0.015, SHANK: 0.015},
        FRONT_RIGHT: {HIP: 0.015, THIGH: 0.015, SHANK: 0.015},
        REAR_LEFT: {HIP: 0.015, THIGH: 0.015, SHANK: 0.015},
        REAR_RIGHT: {HIP: 0.015, THIGH: 0.015, SHANK: 0.015},
    },
    leg_torque={
        FRONT_LEFT: {HIP: 1.00, THIGH: 1.00, SHANK: 1.00},
        FRONT_RIGHT: {HIP: 1.00, THIGH: 1.00, SHANK: 1.00},
        REAR_LEFT: {HIP: 1.00, THIGH: 1.00, SHANK: 1.00},
        REAR_RIGHT: {HIP: 1.00, THIGH: 1.00, SHANK: 1.00},
    },
)

HOLD_GAINS = GainSet(
    leg_kp={
        FRONT_LEFT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
        FRONT_RIGHT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
        REAR_LEFT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
        REAR_RIGHT: {HIP: 0.20, THIGH: 0.20, SHANK: 0.20},
    },
    leg_kd={
        FRONT_LEFT: {HIP: 0.005, THIGH: 0.005, SHANK: 0.005},
        FRONT_RIGHT: {HIP: 0.005, THIGH: 0.005, SHANK: 0.005},
        REAR_LEFT: {HIP: 0.005, THIGH: 0.005, SHANK: 0.005},
        REAR_RIGHT: {HIP: 0.005, THIGH: 0.005, SHANK: 0.005},
    },
    leg_torque={
        FRONT_LEFT: {HIP: 0.80, THIGH: 0.80, SHANK: 0.80},
        FRONT_RIGHT: {HIP: 0.80, THIGH: 0.80, SHANK: 0.80},
        REAR_LEFT: {HIP: 0.80, THIGH: 0.80, SHANK: 0.80},
        REAR_RIGHT: {HIP: 0.80, THIGH: 0.80, SHANK: 0.80},
    },
)

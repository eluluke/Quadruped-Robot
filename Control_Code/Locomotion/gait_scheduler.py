"""
gait_scheduler.py

Gait phase scheduling.

This module knows only leg names and phase offsets. It does not know about
CAN buses, motor IDs, IK, or Xbox input.
"""

from __future__ import annotations

from typing import Dict

from leg_config import FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT


TROT_PHASE_OFFSETS: Dict[str, float] = {
    FRONT_LEFT: 0.0,
    REAR_RIGHT: 0.0,
    FRONT_RIGHT: 0.5,
    REAR_LEFT: 0.5,
}


class TrotGaitScheduler:
    """Diagonal-pair trot scheduler."""

    def __init__(self, phase_offsets: Dict[str, float] | None = None):
        self.phase_offsets = dict(phase_offsets or TROT_PHASE_OFFSETS)

    def leg_phase(self, leg_name: str, global_phase: float) -> float:
        """Return the wrapped phase for one leg."""
        return (global_phase + self.phase_offsets[leg_name]) % 1.0

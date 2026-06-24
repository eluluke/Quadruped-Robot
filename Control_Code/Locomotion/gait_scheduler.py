"""
gait_scheduler.py

Gait phase scheduling.

This module knows only leg names and phase offsets. It does not know about
CAN buses, motor IDs, IK, or Xbox input.
"""

from __future__ import annotations

from typing import Dict

from leg_config import FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT


CRAWL_PHASE_OFFSETS: Dict[str, float] = {
    FRONT_LEFT: 0.00,
    REAR_RIGHT: 0.25,
    FRONT_RIGHT: 0.50,
    REAR_LEFT: 0.75,
}


# Keep this name for existing imports in robot_controller.py.
TROT_PHASE_OFFSETS = CRAWL_PHASE_OFFSETS


class TrotGaitScheduler:
    """Phase-offset gait scheduler."""

    def __init__(self, phase_offsets: Dict[str, float] | None = None):
        self.phase_offsets = dict(phase_offsets or TROT_PHASE_OFFSETS)

    def leg_phase(self, leg_name: str, global_phase: float) -> float:
        """Return the wrapped phase for one leg."""
        return (global_phase + self.phase_offsets[leg_name]) % 1.0

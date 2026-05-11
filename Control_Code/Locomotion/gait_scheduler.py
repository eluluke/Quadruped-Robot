"""
gait_scheduler.py

Gait phase scheduling.

This file knows about phase offsets, not CAN buses or motor IDs.
"""


class TrotGaitScheduler:
    """Diagonal-pair trot scheduler."""

    def __init__(self, phase_offsets):
        self.phase_offsets = dict(phase_offsets)

    def leg_phase(self, leg_name, global_phase):
        """Return phase for one leg."""
        return (global_phase + self.phase_offsets[leg_name]) % 1.0

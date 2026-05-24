"""
xbox_config.py

Xbox controller configuration and input wrapper.

This is the single Xbox-related module for locomotion code:
    - controller axis mapping
    - joystick deadband/filter tuning
    - gait phase-speed tuning
    - pygame-based XboxController helper
"""

from __future__ import annotations

from dataclasses import dataclass


# ============================================================
# Axis mapping
# ============================================================

AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 2
AXIS_LEFT_TRIGGER = 5   # raw axis: -1.0 released, +1.0 pressed
AXIS_RIGHT_TRIGGER = 4  # raw axis: -1.0 released, +1.0 pressed


# ============================================================
# Locomotion joystick tuning
# ============================================================

JOYSTICK_DEADBAND = 0.05
JOYSTICK_FILTER_ALPHA = 0.65

# Full stick gives this many gait cycles per second.
MAX_PHASE_SPEED = 0.80

# Phase speed acceleration limit in cycles/s^2.
PHASE_ACCEL_LIMIT = 4.0

ALLOW_REVERSE = True
FREEZE_PHASE_WHEN_STOPPED = True


# ============================================================
# Controller state
# ============================================================

@dataclass(frozen=True)
class ControllerState:
    """Snapshot of one controller read."""

    left_x: float
    left_y: float
    right_x: float
    right_y: float
    left_trigger: float
    right_trigger: float


# ============================================================
# Input helpers
# ============================================================

def apply_deadband(value: float, deadband: float = JOYSTICK_DEADBAND) -> float:
    """Zero small stick values and rescale the remaining range to [-1, 1]."""
    if abs(value) < deadband:
        return 0.0

    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadband) / (1.0 - deadband)


def trigger_to_unit(raw: float) -> float:
    """Convert trigger axis from [-1, 1] to [0, 1]."""
    return 0.5 * (raw + 1.0)


# Backward-compatible spelling used by older scripts.
_apply_deadzone = apply_deadband
_trigger_to_unit = trigger_to_unit


class XboxController:
    """
    Thin wrapper around a pygame joystick.

    Stick Y axes are inverted so pushing up gives a positive value.
    Stick values are deadbanded into [-1, 1].
    Trigger values are converted into [0, 1].
    """

    def __init__(
        self,
        joystick_index: int = 0,
        deadzone: float = JOYSTICK_DEADBAND,
    ):
        import pygame  # type: ignore[import-not-found]

        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count == 0:
            raise RuntimeError(
                "No joystick/controller detected. "
                "Pair your Xbox controller and try again."
            )

        self._pygame = pygame
        self._joy = pygame.joystick.Joystick(joystick_index)
        self._joy.init()
        self._deadband = deadzone

        print(f"Controller connected: {self._joy.get_name()}")

    def read(self) -> ControllerState:
        """Pump pygame events and return the current controller state."""
        self._pygame.event.pump()
        joy = self._joy
        deadband = self._deadband

        return ControllerState(
            left_x=apply_deadband(joy.get_axis(AXIS_LEFT_X), deadband),
            left_y=apply_deadband(-joy.get_axis(AXIS_LEFT_Y), deadband),
            right_x=apply_deadband(joy.get_axis(AXIS_RIGHT_X), deadband),
            right_y=apply_deadband(-joy.get_axis(AXIS_RIGHT_Y), deadband),
            left_trigger=trigger_to_unit(joy.get_axis(AXIS_LEFT_TRIGGER)),
            right_trigger=trigger_to_unit(joy.get_axis(AXIS_RIGHT_TRIGGER)),
        )

    def close(self) -> None:
        """Release pygame resources."""
        self._pygame.quit()


# Backward-compatible constant names used by older scripts.
DEFAULT_DEADZONE = JOYSTICK_DEADBAND
AXIS_LEFT_TRIG = AXIS_LEFT_TRIGGER
AXIS_RIGHT_TRIG = AXIS_RIGHT_TRIGGER

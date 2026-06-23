"""
xbox_config.py       Initial developement by Elijah Robinson;

Xbox controller configuration and input wrapper.

This is the single Xbox-related module for locomotion code:
    - controller axis mapping
    - joystick deadband/filter tuning
    - gait phase-speed tuning
    - pygame-based XboxController helper
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import struct


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
MAX_PHASE_SPEED = 1.67

# Phase speed acceleration limit in cycles/s^2.
PHASE_ACCEL_LIMIT = 4.0

ALLOW_REVERSE = True
FREEZE_PHASE_WHEN_STOPPED = True


# ============================================================
# Linux joystick fallback
# ============================================================

LINUX_JOYSTICK_DEVICE = "/dev/input/js0"

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)


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


def clamp_unit(value: float) -> float:
    """Clamp a joystick axis value to [-1, 1]."""
    return max(-1.0, min(1.0, value))


def axis_value(values: list[float], axis: int) -> float:
    """Return one axis value, or 0.0 when the device has fewer axes."""
    if axis < 0 or axis >= len(values):
        return 0.0
    return values[axis]


# Backward-compatible spelling used by older scripts.
_apply_deadzone = apply_deadband
_trigger_to_unit = trigger_to_unit


class LinuxJoystickController:
    """Direct reader for Linux /dev/input/js* joystick devices."""

    def __init__(
        self,
        device_path: str = LINUX_JOYSTICK_DEVICE,
        deadzone: float = JOYSTICK_DEADBAND,
    ):
        self.device_path = os.environ.get("XBOX_JOYSTICK_DEVICE", device_path)
        self._deadband = deadzone
        self._fd = os.open(self.device_path, os.O_RDONLY | os.O_NONBLOCK)
        self._axes = [0.0] * 8

        print(
            f"Controller connected through Linux joystick: {self.device_path}")

    def _read_events(self) -> None:
        while True:
            try:
                data = os.read(self._fd, JS_EVENT_SIZE)
            except BlockingIOError:
                return

            if len(data) != JS_EVENT_SIZE:
                return

            _timestamp, raw_value, event_type, number = struct.unpack(
                JS_EVENT_FORMAT,
                data,
            )
            event_type &= ~JS_EVENT_INIT

            if event_type == JS_EVENT_AXIS:
                while number >= len(self._axes):
                    self._axes.append(0.0)
                self._axes[number] = clamp_unit(raw_value / 32767.0)

    def read(self) -> ControllerState:
        """Read pending Linux joystick events and return controller state."""
        self._read_events()
        deadband = self._deadband

        left_x = axis_value(self._axes, AXIS_LEFT_X)
        left_y = axis_value(self._axes, AXIS_LEFT_Y)
        right_x = axis_value(self._axes, AXIS_RIGHT_X)
        right_y = axis_value(self._axes, AXIS_RIGHT_Y)
        left_trigger = axis_value(self._axes, AXIS_LEFT_TRIGGER)
        right_trigger = axis_value(self._axes, AXIS_RIGHT_TRIGGER)

        return ControllerState(
            left_x=apply_deadband(left_x, deadband),
            left_y=apply_deadband(-left_y, deadband),
            right_x=apply_deadband(right_x, deadband),
            right_y=apply_deadband(-right_y, deadband),
            left_trigger=trigger_to_unit(left_trigger),
            right_trigger=trigger_to_unit(right_trigger),
        )

    def close(self) -> None:
        """Close the Linux joystick file descriptor."""
        try:
            os.close(self._fd)
        except Exception:
            pass


class XboxController:
    """
    Xbox controller wrapper.

    It tries pygame first. If pygame/SDL cannot see the controller on Linux,
    it falls back to reading /dev/input/js0 directly.

    Stick Y axes are inverted so pushing up gives a positive value.
    Stick values are deadbanded into [-1, 1].
    Trigger values are converted into [0, 1].
    """

    def __init__(
        self,
        joystick_index: int = 0,
        deadzone: float = JOYSTICK_DEADBAND,
    ):
        self._backend = "pygame"
        self._linux_fallback: LinuxJoystickController | None = None

        try:
            import pygame  # type: ignore[import-not-found]
        except Exception as exc:
            print(f"pygame unavailable ({exc}); trying Linux joystick.")
            self._use_linux_fallback(deadzone)
            return

        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count == 0:
            print("pygame detected 0 controllers; trying Linux joystick.")
            self._pygame = pygame
            self._use_linux_fallback(deadzone)
            return

        self._pygame = pygame
        self._joy = pygame.joystick.Joystick(joystick_index)
        self._joy.init()
        self._deadband = deadzone

        print(f"Controller connected: {self._joy.get_name()}")

    def _use_linux_fallback(self, deadzone: float) -> None:
        try:
            self._linux_fallback = LinuxJoystickController(
                deadzone=deadzone,
            )
            self._backend = "linux"
        except Exception as exc:
            raise RuntimeError(
                "No joystick/controller detected by pygame or "
                f"{LINUX_JOYSTICK_DEVICE}. Pair your Xbox controller and "
                "verify /dev/input/js0 with jstest."
            ) from exc

    def read(self) -> ControllerState:
        """Pump pygame events and return the current controller state."""
        if self._backend == "linux":
            if self._linux_fallback is None:
                raise RuntimeError("Linux joystick backend not initialized.")
            return self._linux_fallback.read()

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
        if self._backend == "linux":
            if self._linux_fallback is not None:
                self._linux_fallback.close()
            return

        self._pygame.quit()


# Backward-compatible constant names used by older scripts.
DEFAULT_DEADZONE = JOYSTICK_DEADBAND
AXIS_LEFT_TRIG = AXIS_LEFT_TRIGGER
AXIS_RIGHT_TRIG = AXIS_RIGHT_TRIGGER

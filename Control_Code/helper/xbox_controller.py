# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Xbox controller helper — import this in any script that needs gamepad input.

import pygame

# ── Axis indices ──────────────────────────────────────────────────────────────
AXIS_LEFT_X     = 0
AXIS_LEFT_Y     = 1
AXIS_RIGHT_X    = 2
AXIS_RIGHT_Y    = 3
AXIS_LEFT_TRIG  = 5   # -1.0 released → +1.0 fully pressed
AXIS_RIGHT_TRIG = 4   # -1.0 released → +1.0 fully pressed

DEFAULT_DEADZONE = 0.08


def _apply_deadzone(value: float, dz: float) -> float:
    """Zero out small values and rescale the remainder to keep motion smooth."""
    if abs(value) < dz:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dz) / (1.0 - dz)


def _trigger_to_unit(raw: float) -> float:
    """Convert raw trigger axis (-1 released, +1 pressed) to 0..1."""
    return (raw + 1.0) / 2.0


class XboxController:
    """
    Thin wrapper around a pygame joystick.

    Usage
    -----
        ctrl = XboxController()          # init pygame + grab first controller
        ...
        state = ctrl.read()              # call every loop iteration
        print(state.right_y)             # use whatever axes you need
        ctrl.close()                     # clean up on exit

    All axis values are in the range [-1.0, 1.0] after deadzone processing.
    Trigger values are in [0.0, 1.0].
    """

    def __init__(self, joystick_index: int = 0, deadzone: float = DEFAULT_DEADZONE):
        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count == 0:
            raise RuntimeError(
                "No joystick/controller detected. "
                "Pair your Xbox controller and try again."
            )

        self._joy = pygame.joystick.Joystick(joystick_index)
        self._joy.init()
        self._dz = deadzone
        print(f"Controller connected: {self._joy.get_name()}")

    # ── Public API ────────────────────────────────────────────────────────────

    def read(self) -> "ControllerState":
        """
        Pump the pygame event queue and return a ControllerState snapshot.
        Call this once per control loop iteration.
        """
        pygame.event.pump()
        dz = self._dz
        j  = self._joy

        return ControllerState(
            left_x       =  _apply_deadzone( j.get_axis(AXIS_LEFT_X),     dz),
            left_y       =  _apply_deadzone(-j.get_axis(AXIS_LEFT_Y),     dz),  # invert: up = +
            right_x      =  _apply_deadzone( j.get_axis(AXIS_RIGHT_X),    dz),
            right_y      =  _apply_deadzone(-j.get_axis(AXIS_RIGHT_Y),    dz),  # invert: up = +
            left_trigger =  _trigger_to_unit(j.get_axis(AXIS_LEFT_TRIG)),
            right_trigger=  _trigger_to_unit(j.get_axis(AXIS_RIGHT_TRIG)),
        )

    def close(self):
        """Release pygame resources."""
        pygame.quit()


class ControllerState:
    """
    Snapshot of the controller axes at a single point in time.

    Attributes
    ----------
    left_x, left_y       : float  left  stick  [-1, 1]  (Y is +up)
    right_x, right_y     : float  right stick  [-1, 1]  (Y is +up)
    left_trigger         : float  left  trigger [0, 1]
    right_trigger        : float  right trigger [0, 1]
    """

    __slots__ = (
        "left_x", "left_y",
        "right_x", "right_y",
        "left_trigger", "right_trigger",
    )

    def __init__(
        self,
        left_x: float, left_y: float,
        right_x: float, right_y: float,
        left_trigger: float, right_trigger: float,
    ):
        self.left_x        = left_x
        self.left_y        = left_y
        self.right_x       = right_x
        self.right_y       = right_y
        self.left_trigger  = left_trigger
        self.right_trigger = right_trigger

    def __repr__(self):
        return (
            f"ControllerState("
            f"LX={self.left_x:+.2f}  LY={self.left_y:+.2f}  "
            f"RX={self.right_x:+.2f}  RY={self.right_y:+.2f}  "
            f"LT={self.left_trigger:.2f}  RT={self.right_trigger:.2f})"
        )

"""
xbox_config.py

Xbox control parameters.
"""

JOYSTICK_DEADBAND = 0.05
JOYSTICK_FILTER_ALPHA = 0.65

# Full stick gives this many gait cycles per second.
MAX_PHASE_SPEED = 0.80

# Phase speed acceleration limit in cycles/s^2.
PHASE_ACCEL_LIMIT = 4.0

ALLOW_REVERSE = True
FREEZE_PHASE_WHEN_STOPPED = True

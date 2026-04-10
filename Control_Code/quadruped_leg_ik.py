"""Quadruped leg inverse kinematics."""

import math

L_H = 83.07
L_T = 270.0
L_S = 270.0


def clamp(value, low, high):
    """Clamp value to the interval [low, high]."""
    return max(low, min(high, value))


def leg_ik(x, y, z):
    """Return hip, thigh, and shank joint angles in radians."""

    z_1 = math.sqrt(y**2 + z**2 - L_H**2)
    l_1 = math.sqrt(x**2 + z_1**2)

    a_1 = math.acos(
        clamp((L_S**2 + L_T**2 - l_1**2) / (2 * L_S * L_T), -1.0, 1.0)
    )

    a_2 = math.atan2(x, z_1)

    b_1 = math.acos(
        clamp((L_T**2 + l_1**2 - L_S**2) / (2 * L_T * l_1), -1.0, 1.0)
    )

    b_2 = -math.atan2(y, z)
    b_3 = -math.atan2(z_1, L_H)

    theta_h = math.pi / 2 - b_2 - b_3
    theta_t = math.pi - a_1
    theta_s = b_1 - a_2

    return theta_h, theta_t, theta_s

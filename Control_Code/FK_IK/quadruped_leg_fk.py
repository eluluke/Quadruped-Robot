"""Quadruped leg forward kinematics.

This FK is written to be consistent with quadruped_leg_ik.py.

It uses the same IK-frame angle definitions:
    theta_h = hip angle
    theta_t = thigh angle
    theta_s = shank angle

No gear ratio.
No motor sign flip.
No homing offset.

Those should stay in the motor-control layer, not in this FK file.
"""

import math

L_H = 84.26
L_T = 270.0
L_S = 270.0


def clamp(value, low, high):
    return max(low, min(high, value))


def leg_fk(theta_h, theta_t, theta_s):
    """Return foot position x, y, z in mm."""

    # From IK:
    # theta_s = pi - a_1
    # so the equivalent triangle length is:
    l_1 = math.sqrt(
        L_T**2
        + L_S**2
        + 2.0 * L_T * L_S * math.cos(theta_s)
    )

    # From IK:
    # theta_t = b_1 - a_2
    b_1 = math.acos(
        clamp(
            (L_T**2 + l_1**2 - L_S**2) / (2.0 * L_T * l_1),
            -1.0,
            1.0,
        )
    )

    a_2 = b_1 - theta_t

    x = l_1 * math.sin(a_2)
    z_1 = l_1 * math.cos(a_2)

    # From IK:
    # theta_h = pi/2 - b_2 - b_3
    b_3 = math.atan2(z_1, L_H)
    b_2 = math.pi / 2.0 - theta_h - b_3

    r = math.sqrt(z_1**2 + L_H**2)

    y = r * math.sin(b_2)
    z = r * math.cos(b_2)

    return x, y, z
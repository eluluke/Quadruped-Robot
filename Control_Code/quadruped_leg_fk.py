"""Quadruped leg forward kinematics using ONLY motor angles.

This file intentionally does NOT include:
- gear ratio
- joint-angle conversion
- motor/output sign conversion

It is a minimal geometric FK written in the same spirit as the current IK file.
The three input angles are treated directly as the kinematic angles.

Coordinate convention follows the current IK / hand sketch:
- origin at the center of the hip-thigh actuator
- z axis points downward
- y axis is along the shoulder
- x axis is along the hip axis / fore-aft axis used in the IK notes
"""

import math

L_H = 84.26
L_T = 270.0
L_S = 270.0


def leg_fk(theta_h, theta_t, theta_s):
    """Return foot position (x, y, z) from 3 motor angles in radians.

    Inputs:
        theta_h : motor angle 1 [rad]
        theta_t : motor angle 2 [rad]
        theta_s : motor angle 3 [rad]

    Outputs:
        x, y, z : foot position [mm]

    Notes:
    - This is intentionally a minimal FK layer.
    - No gear ratio is applied.
    - No sign correction is applied.
    - The angles are used directly as kinematic inputs.
    """

    # Effective planar distance from hip-thigh actuator center
    # to the foot projection in the hip-offset plane
    d = (
        L_T * math.cos(theta_s)
        + L_S * math.cos(theta_t - theta_s)
    )

    # Fore-aft direction
    x = (
        -L_T * math.sin(theta_s)
        + L_S * math.sin(theta_t - theta_s)
    )

    # Shoulder-offset geometry mapped back to 3D
    y = -(
        L_H * math.cos(theta_h)
        + d * math.sin(theta_h)
    )

    z = (
        L_H * math.sin(theta_h)
        - d * math.cos(theta_h)
    )

    return x, y, z

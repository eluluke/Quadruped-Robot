import math


def clamp(value, low, high):
    """
    Restrict value into the interval [low, high].
    This is useful because floating-point roundoff may produce
    something like 1.00000001, which would make acos crash.
    """
    return max(low, min(high, value))


def leg_ik(x, y, z, l_h, l_t, l_s):
    """
    Inverse kinematics template for a quadruped leg.

    Inputs:
        x, y, z : foot position in global coordinates centered at hip
        l_h     : hip offset length
        l_t     : thigh length
        l_s     : shank length

    Returns:
        theta_h, theta_t, theta_s : joint angles
    """

    z_1 = math.sqrt(y**2 + z**2 - l_h**2)

    a_1_arg = (l_s**2 + l_t**2 - z_1**2 - x**2) / (2 * l_s * l_t)
    a_1_arg = clamp(a_1_arg, -1.0, 1.0)
    a_1 = math.acos(a_1_arg)

    a_2 = math.atan2(x, z_1)

    b_1_arg = (l_t**2 + z_1**2 + x**2 - l_s**2) / \
        (2 * l_t * (z_1**2 + x**2))
    b_1_arg = clamp(b_1_arg, -1.0, 1.0)
    b_1 = math.acos(b_1_arg)

    b_2 = math.atan2(y, z)
    b_3 = math.atan2(z_1, l_h)

    theta_h = math.pi / 2 - b_2 - b_3
    theta_t = math.pi - a_1
    theta_s = b_1 - a_2

    return theta_h, theta_t, theta_s

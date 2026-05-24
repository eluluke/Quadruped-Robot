"""
leg_config_simple.py

Simple, explicit hardware configuration for the quadruped robot.

This file is intentionally written to be EASY TO READ, not overly abstract.

There are 4 legs:
    front_left
    front_right
    rear_left
    rear_right

Each leg has 3 joints:
    hip
    thigh
    shank

Each joint explicitly defines:
    can_id
    kp
    kd
    torque_limit
    max_contraction_angle_rad

Important:
    This file does NOT define trajectory.
    This file does NOT define Xbox control.
    This file does NOT define gait phase.
    This file does NOT define IK math.

It only describes the physical hardware configuration.
"""

# ============================================================
# Joint role names
# ============================================================

HIP = "hip"
THIGH = "thigh"
SHANK = "shank"

JOINT_ORDER = [HIP, THIGH, SHANK]


# ============================================================
# Leg names
# ============================================================

FRONT_LEFT = "front_left"
FRONT_RIGHT = "front_right"
REAR_LEFT = "rear_left"
REAR_RIGHT = "rear_right"

LEG_ORDER = [
    FRONT_LEFT,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT,
]


# ============================================================
# Full robot hardware configuration
# ============================================================
#
# Edit these values directly.
#
# can_channel:
#     Which SocketCAN channel this leg is connected to.
#
# can_id:
#     CAN ID of this exact joint motor controller.
#     Since you have 12 boards, these should be 12 separate IDs if all boards
#     are on the same CAN network, or they can repeat only if each leg is on
#     a fully separate CAN bus.
#
# kp, kd, torque_limit:
#     Position control settings for this exact joint.
#
# max_contraction_angle_rad:
#     Output-side joint angle at that leg's mechanical max-contraction
#     homing pose.
#
#     This is NOT the raw encoder reading.
#     The raw encoder reading is recorded at runtime when you manually put
#     the leg at max contraction and confirm homing.
#
# ============================================================

LEG_CONFIG = {
    FRONT_LEFT: {
        "can_channel": "can2",

        HIP: {
            "can_id": 6,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 1.30,
            "max_contraction_angle_rad": -1.069,
        },

        THIGH: {
            "can_id": 5,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 0.75,
            "max_contraction_angle_rad": 1.264,
        },

        SHANK: {
            "can_id": 4,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 0.75,
            "max_contraction_angle_rad": 2.631,
        },
    },

    FRONT_RIGHT: {
        "can_channel": "can1",

        HIP: {
            "can_id": 3,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 1.30,
            "max_contraction_angle_rad": -1.069,
        },

        THIGH: {
            "can_id": 2,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 0.75,
            "max_contraction_angle_rad": 1.264,
        },

        SHANK: {
            "can_id": 1,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 0.75,
            "max_contraction_angle_rad": 2.631,
        },
    },

    REAR_LEFT: {
        "can_channel": "can4",

        HIP: {
            "can_id": 12,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 1.30,
            "max_contraction_angle_rad": -1.069,
        },

        THIGH: {
            "can_id": 11,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 0.75,
            "max_contraction_angle_rad": 1.020,
        },

        SHANK: {
            "can_id": 10,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 0.75,
            "max_contraction_angle_rad": 2.631,
        },
    },

    REAR_RIGHT: {
        "can_channel": "can3",

        HIP: {
            "can_id": 9,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 1.30,
            "max_contraction_angle_rad": -1.069,
        },

        THIGH: {
            "can_id": 8,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 0.75,
            "max_contraction_angle_rad": 1.020,
        },

        SHANK: {
            "can_id": 7,
            "kp": 0.20,
            "kd": 0.008,
            "torque_limit": 0.75,
            "max_contraction_angle_rad": 2.631,
        },
    },
}


# ============================================================
# Optional conversion constants
# ============================================================
#
# These are not trajectory settings. They are hardware conversion settings.
# Other control files can import these when converting output joint angle delta
# into raw motor command delta.
# ============================================================

GEAR_RATIO = 17.0

# Default direction signs.
# If one joint moves opposite of expected, change that exact joint's sign here.
#
# Example:
#     JOINT_SIGN[REAR_LEFT][HIP] = -1.0

MOTOR_SIGN = {
    FRONT_LEFT: {
        HIP: 1.0,
        THIGH: 1.0,
        SHANK: 1.0,
    },

    FRONT_RIGHT: {
        HIP: 1.0,
        THIGH: 1.0,
        SHANK: 1.0,
    },

    REAR_LEFT: {
        HIP: 1.0,
        THIGH: 1.0,
        SHANK: 1.0,
    },

    REAR_RIGHT: {
        HIP: 1.0,
        THIGH: 1.0,
        SHANK: 1.0,
    },
}


# ============================================================
# Simple accessor functions
# ============================================================

def get_leg_config(leg_name):
    """Return the full config dictionary for one leg."""
    return LEG_CONFIG[leg_name]


def get_joint_config(leg_name, joint_name):
    """Return the config dictionary for one joint in one leg."""
    return LEG_CONFIG[leg_name][joint_name]


def get_can_channel(leg_name):
    """Return the SocketCAN channel for one leg."""
    return LEG_CONFIG[leg_name]["can_channel"]


def get_can_id(leg_name, joint_name):
    """Return CAN ID for one joint."""
    return LEG_CONFIG[leg_name][joint_name]["can_id"]


def get_kp(leg_name, joint_name):
    """Return kp for one joint."""
    return LEG_CONFIG[leg_name][joint_name]["kp"]


def get_kd(leg_name, joint_name):
    """Return kd for one joint."""
    return LEG_CONFIG[leg_name][joint_name]["kd"]


def get_torque_limit(leg_name, joint_name):
    """Return torque limit for one joint."""
    return LEG_CONFIG[leg_name][joint_name]["torque_limit"]


def get_max_contraction_angle(leg_name, joint_name):
    """Return max-contraction output-side angle in radians for one joint."""
    return LEG_CONFIG[leg_name][joint_name]["max_contraction_angle_rad"]


def get_motor_sign(leg_name, joint_name):
    """Return motor sign for one joint."""
    return MOTOR_SIGN[leg_name][joint_name]


def get_role_to_id_for_leg(leg_name):
    """Return role-to-CAN-ID mapping for one leg."""
    return {
        HIP: get_can_id(leg_name, HIP),
        THIGH: get_can_id(leg_name, THIGH),
        SHANK: get_can_id(leg_name, SHANK),
    }


def get_max_contraction_angles_for_leg(leg_name):
    """Return all max-contraction angles for one leg."""
    return {
        HIP: get_max_contraction_angle(leg_name, HIP),
        THIGH: get_max_contraction_angle(leg_name, THIGH),
        SHANK: get_max_contraction_angle(leg_name, SHANK),
    }


def get_joint_gains(leg_name, joint_name):
    """
    Return kp, kd, torque_limit for one joint.

    Example:
        kp, kd, torque = get_joint_gains(REAR_LEFT, SHANK)
    """
    joint = get_joint_config(leg_name, joint_name)
    return joint["kp"], joint["kd"], joint["torque_limit"]


def print_config_summary():
    """Print all 12 joint configurations."""
    print("=" * 80)
    print("Quadruped hardware configuration")
    print("=" * 80)

    for leg_name in LEG_ORDER:
        print()
        print(f"{leg_name}:")
        print(f"  can_channel = {get_can_channel(leg_name)}")

        for joint_name in JOINT_ORDER:
            joint = get_joint_config(leg_name, joint_name)
            print(
                f"  {joint_name:5s}: "
                f"can_id={joint['can_id']:2d}, "
                f"kp={joint['kp']:.4f}, "
                f"kd={joint['kd']:.4f}, "
                f"torque_limit={joint['torque_limit']:.4f}, "
                "max_contraction="
                f"{joint['max_contraction_angle_rad']:.6f} rad, "
                f"motor_sign={get_motor_sign(leg_name, joint_name):+.1f}"
            )

    print("=" * 80)


if __name__ == "__main__":
    print_config_summary()

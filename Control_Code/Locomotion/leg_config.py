"""
leg_config.py

Simple hardware configuration for the quadruped robot.

This file defines:
    - leg names
    - joint names
    - CAN channels
    - CAN IDs
    - position gains
    - motor direction signs
    - the selected manual homing angle

Trajectory, gait phase, Xbox input, and IK math belong in other files.
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
# Shared neutral standing pose
# ============================================================

NEUTRAL_X = 0.0
NEUTRAL_Y = 84.26
NEUTRAL_Z = 385.0

NEUTRAL_STANDING_FOOT_POSITION = (NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z)


# ============================================================
# Homing angle choices
# ============================================================
#
# Put the robot at the selected physical homing pose, then confirm homing
# in the terminal. The controller records the raw encoder readings at that
# moment, and uses the selected homing angles below to move to neutral.
#
# To switch homing style, comment one HOMING_ANGLE line and uncomment the
# other one. The rest of the code only imports HOMING_ANGLE.
# ============================================================

class homing_angle:
    """Candidate output-side joint angles for manual homing poses."""

    max_contraction_angle = {
        HIP: -1.069,
        THIGH: 1.264,
        SHANK: 2.631,
    }

    sleepy_angle = {
        HIP: -0.465499025237436,
        THIGH: 1.11367455104935,
        SHANK: 2.68814970801254,
    }


# HOMING_ANGLE = homing_angle.max_contraction_angle
HOMING_ANGLE = homing_angle.sleepy_angle


# ============================================================
# Full robot hardware configuration
# ============================================================

LEG_CONFIG = {
    FRONT_LEFT: {
        "can_channel": "can2",

        HIP: {
            "can_id": 6,
            "kp": 0.24,
            "kd": 0.010,
            "torque_limit": 1.30,
            "homing_angle_rad": HOMING_ANGLE[HIP],
        },

        THIGH: {
            "can_id": 5,
            "kp": 0.38,
            "kd": 0.014,
            "torque_limit": 1.50,
            "homing_angle_rad": HOMING_ANGLE[THIGH],
        },

        SHANK: {
            "can_id": 4,
            "kp": 0.38,
            "kd": 0.014,
            "torque_limit": 1.50,
            "homing_angle_rad": HOMING_ANGLE[SHANK],
        },
    },

    FRONT_RIGHT: {
        "can_channel": "can1",

        HIP: {
            "can_id": 3,
            "kp": 0.24,
            "kd": 0.010,
            "torque_limit": 1.30,
            "homing_angle_rad": HOMING_ANGLE[HIP],
        },

        THIGH: {
            "can_id": 2,
            "kp": 0.38,
            "kd": 0.014,
            "torque_limit": 1.50,
            "homing_angle_rad": HOMING_ANGLE[THIGH],
        },

        SHANK: {
            "can_id": 1,
            "kp": 0.38,
            "kd": 0.014,
            "torque_limit": 1.50,
            "homing_angle_rad": HOMING_ANGLE[SHANK],
        },
    },

    REAR_LEFT: {
        "can_channel": "can4",

        HIP: {
            "can_id": 12,
            "kp": 0.24,
            "kd": 0.010,
            "torque_limit": 1.30,
            "homing_angle_rad": HOMING_ANGLE[HIP],
        },

        THIGH: {
            "can_id": 11,
            "kp": 0.38,
            "kd": 0.014,
            "torque_limit": 1.50,
            "homing_angle_rad": HOMING_ANGLE[THIGH],
        },

        SHANK: {
            "can_id": 10,
            "kp": 0.38,
            "kd": 0.014,
            "torque_limit": 1.50,
            "homing_angle_rad": HOMING_ANGLE[SHANK],
        },
    },

    REAR_RIGHT: {
        "can_channel": "can3",

        HIP: {
            "can_id": 9,
            "kp": 0.24,
            "kd": 0.010,
            "torque_limit": 1.30,
            "homing_angle_rad": HOMING_ANGLE[HIP],
        },

        THIGH: {
            "can_id": 8,
            "kp": 0.38,
            "kd": 0.014,
            "torque_limit": 1.50,
            "homing_angle_rad": HOMING_ANGLE[THIGH],
        },

        SHANK: {
            "can_id": 7,
            "kp": 0.38,
            "kd": 0.014,
            "torque_limit": 1.50,
            "homing_angle_rad": HOMING_ANGLE[SHANK],
        },
    },
}


# ============================================================
# Hardware conversion constants
# ============================================================

GEAR_RATIO = 17.0

MOTOR_SIGN = {
    FRONT_LEFT: {
        HIP: 1.0,
        THIGH: -1.0,
        SHANK: -1.0,
    },

    FRONT_RIGHT: {
        HIP: -1.0,
        THIGH: 1.0,
        SHANK: 1.0,
    },

    REAR_LEFT: {
        HIP: -1.0,
        THIGH: -1.0,
        SHANK: -1.0,
    },

    REAR_RIGHT: {
        HIP: 1.0,
        THIGH: 1.0,
        SHANK: 1.0,
    },
}
# littlemoomoo tested on 06/04/26, all motor signs are correct;

# ============================================================
# Accessor functions
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


def get_homing_angle(leg_name, joint_name):
    """Return selected homing output-side angle in radians for one joint."""
    return LEG_CONFIG[leg_name][joint_name]["homing_angle_rad"]


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


def get_homing_angles_for_leg(leg_name):
    """Return all selected homing angles for one leg."""
    return {
        HIP: get_homing_angle(leg_name, HIP),
        THIGH: get_homing_angle(leg_name, THIGH),
        SHANK: get_homing_angle(leg_name, SHANK),
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
                f"homing={joint['homing_angle_rad']:.6f} rad, "
                f"motor_sign={get_motor_sign(leg_name, joint_name):+.1f}"
            )

    print("=" * 80)


if __name__ == "__main__":
    print_config_summary()

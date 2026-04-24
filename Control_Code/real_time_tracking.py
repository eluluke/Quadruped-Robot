import time

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

from homing_offsets import HOMING_OFFSET
from quadruped_leg_fk import leg_fk


# ============================================================
# Motor IDs
# ============================================================
SHANK_ID = 0
THIGH_ID = 1
HIP_ID = 2

MOTOR_IDS = [SHANK_ID, THIGH_ID, HIP_ID]

MOTOR_NAMES = {
    SHANK_ID: "shank",
    THIGH_ID: "thigh",
    HIP_ID: "hip",
}


# ============================================================
# Timing
# ============================================================
RATE_HZ = 20.0
PRINT_EVERY = 1

# If True, motors are put in IDLE so you can backdrive freely.
# If IDLE cannot return encoder readings on your firmware, set False.
SET_IDLE_ON_START = True


# ============================================================
# Setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)


# ============================================================
# Helpers
# ============================================================
def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.003)
    bus.feed(motor_id)
    time.sleep(0.003)


def read_raw_position(motor_id):
    pos, vel = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

    if pos is None:
        raise RuntimeError(
            f"Cannot read {MOTOR_NAMES[motor_id]} position"
        )

    return pos, vel


def raw_to_ik_angle(motor_id, raw_angle):
    # Your homing convention:
    # IK-frame angle = raw encoder position + homing offset
    return raw_angle + HOMING_OFFSET[motor_id]


# ============================================================
# Main
# ============================================================
try:
    print("Real-time leg position tracker")
    print("No motor trajectory command is sent.")
    print("Backdrive the leg by hand after homing.")
    print()

    print("Loaded HOMING_OFFSET:")
    for motor_id in MOTOR_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]} "
            f"(ID {motor_id}) = {HOMING_OFFSET[motor_id]:.6f}"
        )

    print()

    if SET_IDLE_ON_START:
        print("Setting all joints to IDLE for free backdriving...")
        for motor_id in MOTOR_IDS:
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
        print()

    print("Reading encoder positions and FK output.")
    print("Press Ctrl+C to stop.\n")

    counter = 0

    while True:
        raw_shank, vel_shank = read_raw_position(SHANK_ID)
        raw_thigh, vel_thigh = read_raw_position(THIGH_ID)
        raw_hip, vel_hip = read_raw_position(HIP_ID)

        theta_s = raw_to_ik_angle(SHANK_ID, raw_shank)
        theta_t = raw_to_ik_angle(THIGH_ID, raw_thigh)
        theta_h = raw_to_ik_angle(HIP_ID, raw_hip)

        x, y, z = leg_fk(theta_h, theta_t, theta_s)

        counter += 1

        if counter % PRINT_EVERY == 0:
            print(
                f"raw: "
                f"hip={raw_hip:.4f}, "
                f"thigh={raw_thigh:.4f}, "
                f"shank={raw_shank:.4f} | "
                f"IK angles: "
                f"hip={theta_h:.4f}, "
                f"thigh={theta_t:.4f}, "
                f"shank={theta_s:.4f} | "
                f"vel: "
                f"hip={vel_hip:.4f}, "
                f"thigh={vel_thigh:.4f}, "
                f"shank={vel_shank:.4f} | "
                f"foot: "
                f"x={x:.2f} mm, "
                f"y={y:.2f} mm, "
                f"z={z:.2f} mm"
            )

        rate.sleep()

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    print("Stopping bus...")
    try:
        bus.stop()
    except Exception:
        pass

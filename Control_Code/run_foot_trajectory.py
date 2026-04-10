import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

# --------------------------------
# Motor IDs
# --------------------------------
HIP_ID = 2
THIGH_ID = 1
SHANK_ID = 0

# --------------------------------
# Gear ratio
# joint angle -> motor angle
# --------------------------------
GEAR_RATIO = 17.0

# --------------------------------
# Gains
# --------------------------------
KP = 0.16
KD = 0.004
TORQUE_LIMIT = 0.5

STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP = 0.03
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.10

# --------------------------------
# Foot trajectory parameters
# --------------------------------
CYCLE_TIME = 2.5
STANCE_RATIO = 0.35
STEP_LENGTH = 60.0
STEP_HEIGHT = 70.0
X_CENTER = 0.0
Z_GROUND = 300.0

# --------------------------------
# Timing / startup
# --------------------------------
RATE_HZ = 100.0
rate = RateLimiter(frequency=RATE_HZ)

STABLE_VELOCITY_THRESHOLD = 0.4
REQUIRED_STABLE_SAMPLES = 20
MAX_STARTUP_TIME = 6.0

PRINT_EVERY = 20


# --------------------------------
# Trajectory
# stance = straight pull-back
# swing = cycloid
# --------------------------------
def foot_trajectory(phase):
    phase = phase % 1.0

    if phase < STANCE_RATIO:
        u = phase / STANCE_RATIO
        x = X_CENTER + (STEP_LENGTH / 2.0) - STEP_LENGTH * u
        z = Z_GROUND
        return x, z

    u = (phase - STANCE_RATIO) / (1.0 - STANCE_RATIO)

    x = X_CENTER - (STEP_LENGTH / 2.0) + STEP_LENGTH * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )
    z = Z_GROUND + STEP_HEIGHT * (
        1.0 - math.cos(2.0 * math.pi * u)
    ) / 2.0

    return x, z


# --------------------------------
# IK wrapper
# keep planar motion: y = 0
# --------------------------------
def planar_leg_ik(x, z):
    theta_h, theta_t, theta_s = leg_ik(x, 0.0, z)
    return theta_h, theta_t, theta_s


# --------------------------------
# Set modes / gains
# --------------------------------
def set_mode_with_spacing(jid, mode):
    bus.set_mode(jid, mode)
    time.sleep(0.003)
    bus.feed(jid)
    time.sleep(0.003)


def set_gains(jid, kp_val, kd_val, torque_val):
    bus.write_position_kp(jid, kp_val)
    time.sleep(0.002)
    bus.write_position_kd(jid, kd_val)
    time.sleep(0.002)
    bus.write_torque_limit(jid, torque_val)
    time.sleep(0.002)


# --------------------------------
# Sync current reference for a joint
# Only used for thigh + shank in position mode
# --------------------------------
def sync_reference(jid, sync_time=0.25):
    steps = int(sync_time * RATE_HZ)
    current_pos = None

    for _ in range(steps):
        pos, vel = bus.write_read_pdo_2(jid, 0.0, 0.0)
        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(jid, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync joint {jid}")

    return current_pos


# --------------------------------
# Stable startup
# Only require THIGH + SHANK stable
# Hip stays in damping and is not allowed to block startup
# --------------------------------
def get_stable_start_positions():
    print("Entering damping mode for all joints...")

    for jid in [HIP_ID, THIGH_ID, SHANK_ID]:
        set_mode_with_spacing(jid, recoil.Mode.DAMPING)

    time.sleep(0.1)

    stable_count = 0
    last_hip = None
    last_thigh = None
    last_shank = None

    t_start = time.perf_counter()

    while True:
        hip_pos, hip_vel = bus.write_read_pdo_2(HIP_ID, 0.0, 0.0)
        thigh_pos, thigh_vel = bus.write_read_pdo_2(THIGH_ID, 0.0, 0.0)
        shank_pos, shank_vel = bus.write_read_pdo_2(SHANK_ID, 0.0, 0.0)

        if None not in [
            hip_pos, hip_vel,
            thigh_pos, thigh_vel,
            shank_pos, shank_vel,
        ]:
            last_hip = hip_pos
            last_thigh = thigh_pos
            last_shank = shank_pos

            if (
                abs(thigh_vel) < STABLE_VELOCITY_THRESHOLD
                and abs(shank_vel) < STABLE_VELOCITY_THRESHOLD
            ):
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= REQUIRED_STABLE_SAMPLES:
                return last_hip, last_thigh, last_shank

        if time.perf_counter() - t_start > MAX_STARTUP_TIME:
            raise RuntimeError(
                "Startup not stable "
                f"(hip_vel={hip_vel}, thigh_vel={thigh_vel}, shank_vel={shank_vel})"
            )

        rate.sleep()


# --------------------------------
# Build one full discrete cycle:
# foot points -> IK -> motor commands
# --------------------------------
def build_command_table(start_thigh, start_shank):
    num_points = int(CYCLE_TIME * RATE_HZ)

    x0, z0 = foot_trajectory(0.0)
    _, theta_t_ref0, theta_s_ref0 = planar_leg_ik(x0, z0)

    print(
        f"Ref angles: thigh={theta_t_ref0:.3f}, shank={theta_s_ref0:.3f}"
    )

    table = []

    for k in range(num_points):
        phase = k / num_points
        x_des, z_des = foot_trajectory(phase)

        _, theta_t_des, theta_s_des = planar_leg_ik(x_des, z_des)

        cmd_thigh = start_thigh + (theta_t_des - theta_t_ref0) * GEAR_RATIO
        cmd_shank = start_shank + (theta_s_des - theta_s_ref0) * GEAR_RATIO

        table.append(
            {
                "x": x_des,
                "z": z_des,
                "cmd_thigh": cmd_thigh,
                "cmd_shank": cmd_shank,
            }
        )

    return table


# --------------------------------
# Main
# --------------------------------
try:
    start_hip, start_thigh, start_shank = get_stable_start_positions()

    print(
        f"Startup motor angles: hip={start_hip:.3f}, "
        f"thigh={start_thigh:.3f}, shank={start_shank:.3f}"
    )

    # Hip remains in damping only
    set_mode_with_spacing(HIP_ID, recoil.Mode.DAMPING)

    # Thigh + shank enter position mode softly
    for jid in [THIGH_ID, SHANK_ID]:
        set_gains(
            jid,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )
        set_mode_with_spacing(jid, recoil.Mode.POSITION)

    time.sleep(0.05)

    # Sync actual reference after entering position mode
    start_thigh = sync_reference(THIGH_ID, sync_time=0.25)
    start_shank = sync_reference(SHANK_ID, sync_time=0.25)

    # Soft hold
    for _ in range(int(0.5 * RATE_HZ)):
        bus.write_read_pdo_2(THIGH_ID, start_thigh, 0.0)
        bus.write_read_pdo_2(SHANK_ID, start_shank, 0.0)
        rate.sleep()

    # Medium gains
    for jid in [THIGH_ID, SHANK_ID]:
        set_gains(
            jid,
            MID_KP,
            MID_KD,
            MID_TORQUE_LIMIT,
        )

    for _ in range(int(0.4 * RATE_HZ)):
        bus.write_read_pdo_2(THIGH_ID, start_thigh, 0.0)
        bus.write_read_pdo_2(SHANK_ID, start_shank, 0.0)
        rate.sleep()

    # Final gains
    for jid in [THIGH_ID, SHANK_ID]:
        set_gains(jid, KP, KD, TORQUE_LIMIT)

    # Precompute discrete trajectory commands
    command_table = build_command_table(start_thigh, start_shank)

    print("Starting discrete foot trajectory...")
    counter = 0
    idx = 0

    while True:
        point = command_table[idx]

        thigh_pos, thigh_vel = bus.write_read_pdo_2(
            THIGH_ID,
            point["cmd_thigh"],
            0.0,
        )
        shank_pos, shank_vel = bus.write_read_pdo_2(
            SHANK_ID,
            point["cmd_shank"],
            0.0,
        )

        counter += 1
        if counter % PRINT_EVERY == 0:
            print(
                f"x={point['x']:.1f} z={point['z']:.1f} | "
                f"th_cmd={point['cmd_thigh']:.3f} th_pos={thigh_pos:.3f} | "
                f"sh_cmd={point['cmd_shank']:.3f} sh_pos={shank_pos:.3f}"
            )

        idx += 1
        if idx >= len(command_table):
            idx = 0

        rate.sleep()

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    # Clean shutdown so the next startup is not polluted
    try:
        for jid in [THIGH_ID, SHANK_ID]:
            try:
                set_mode_with_spacing(jid, recoil.Mode.IDLE)
            except Exception:
                pass

        try:
            set_mode_with_spacing(HIP_ID, recoil.Mode.IDLE)
        except Exception:
            pass

        time.sleep(0.05)
    finally:
        try:
            bus.stop()
        except Exception:
            pass

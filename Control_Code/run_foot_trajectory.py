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
THIGH_ID = 1
SHANK_ID = 0

# --------------------------------
# Final gains
# --------------------------------
kp = 0.2
kd = 0.005
torque_limit = 0.6

# Very soft startup gains
startup_kp = 0.003
startup_kd = 0.001
startup_torque_limit = 0.03

mid_kp = 0.03
mid_kd = 0.002
mid_torque_limit = 0.10

# --------------------------------
# Foot trajectory parameters
# Adjust these later as needed
# --------------------------------
cycle_time = 2.0       # seconds for one full gait cycle
stance_ratio = 0.5     # fraction of cycle spent in straight-line stance
step_length = 80.0     # mm, fore-aft foot travel
step_height = 40.0     # mm, cycloid lift height in swing
x_center = 0.0         # mm, center of foot trajectory in x
z_ground = 300.0       # mm, nominal leg extension / ground height

# NOTE:
# With your IK, y=0 and z must satisfy |z| >= L_H.
# Since L_H = 85.07 mm in your file, z_ground should stay comfortably above that.:contentReference[oaicite:1]{index=1}

# --------------------------------
# Timing / startup stability
# --------------------------------
rate_hz = 200.0
rate = RateLimiter(frequency=rate_hz)

stable_velocity_threshold = 0.2
required_stable_samples = 30
max_startup_time = 4.0

# --------------------------------
# Helper: foot trajectory
# stance = straight line backward
# swing = forward cycloid
# --------------------------------


def foot_trajectory(phase: float):
    """
    phase in [0, 1)

    Returns:
        x, z in mm
    """
    phase = phase % 1.0

    # stance: straight pull-back at constant height
    if phase < stance_ratio:
        u = phase / stance_ratio
        x = x_center + (step_length / 2.0) - step_length * u
        z = z_ground
        return x, z

    # swing: forward cycloid
    u = (phase - stance_ratio) / (1.0 - stance_ratio)

    x = x_center - (step_length / 2.0) + step_length * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )
    z = z_ground + step_height * (1.0 - math.cos(2.0 * math.pi * u)) / 2.0

    return x, z


# --------------------------------
# Helper: get thigh/shank IK only
# hip is ignored for now
# --------------------------------
def planar_leg_ik(x: float, z: float):
    theta_h, theta_t, theta_s = leg_ik(x, 0.0, z)
    return theta_t, theta_s


# --------------------------------
# Phase-0 reference in joint space
# Used to convert desired IK angles into relative motor commands
# --------------------------------
x0, z0 = foot_trajectory(0.0)
theta_t_ref0, theta_s_ref0 = planar_leg_ik(x0, z0)

print(f"Trajectory phase-0 foot point: x={x0:.3f} mm, z={z0:.3f} mm")
print(
    f"Reference joint angles: theta_t={theta_t_ref0:.3f}, theta_s={theta_s_ref0:.3f}")

# --------------------------------
# Damping startup: let BOTH joints settle together
# --------------------------------
print("Entering damping mode for both joints...")
bus.set_mode(THIGH_ID, recoil.Mode.DAMPING)
bus.set_mode(SHANK_ID, recoil.Mode.DAMPING)
bus.feed(THIGH_ID)
bus.feed(SHANK_ID)
time.sleep(0.1)

stable_thigh_positions = []
stable_shank_positions = []
stable_count = 0

t_start = time.perf_counter()

while True:
    thigh_pos, thigh_vel = bus.write_read_pdo_2(THIGH_ID, 0.0, 0.0)
    shank_pos, shank_vel = bus.write_read_pdo_2(SHANK_ID, 0.0, 0.0)

    if (
        thigh_pos is not None and thigh_vel is not None and
        shank_pos is not None and shank_vel is not None
    ):
        if (
            abs(thigh_vel) < stable_velocity_threshold and
            abs(shank_vel) < stable_velocity_threshold
        ):
            stable_thigh_positions.append(thigh_pos)
            stable_shank_positions.append(shank_pos)
            stable_count += 1
        else:
            stable_thigh_positions.clear()
            stable_shank_positions.clear()
            stable_count = 0

        if stable_count >= required_stable_samples:
            break

    if (time.perf_counter() - t_start) > max_startup_time:
        bus.set_mode(THIGH_ID, recoil.Mode.IDLE)
        bus.set_mode(SHANK_ID, recoil.Mode.IDLE)
        bus.stop()
        raise RuntimeError(
            "Could not get stable startup positions for both joints.")

    rate.sleep()

start_thigh_motor = sum(stable_thigh_positions) / len(stable_thigh_positions)
start_shank_motor = sum(stable_shank_positions) / len(stable_shank_positions)

print(f"Stable startup motor angles:")
print(f"  thigh motor = {start_thigh_motor:.3f} rad")
print(f"  shank motor = {start_shank_motor:.3f} rad")

# --------------------------------
# Enter POSITION mode softly for both joints
# --------------------------------
for device_id in [THIGH_ID, SHANK_ID]:
    bus.write_position_kp(device_id, startup_kp)
    bus.write_position_kd(device_id, startup_kd)
    bus.write_torque_limit(device_id, startup_torque_limit)
    bus.set_mode(device_id, recoil.Mode.POSITION)
    bus.feed(device_id)

time.sleep(0.05)

print("Soft holding both joints...")
hold_time_1 = 0.6
hold_steps_1 = int(hold_time_1 * rate_hz)

for _ in range(hold_steps_1):
    bus.write_read_pdo_2(THIGH_ID, start_thigh_motor, 0.0)
    bus.write_read_pdo_2(SHANK_ID, start_shank_motor, 0.0)
    rate.sleep()

# medium gains
for device_id in [THIGH_ID, SHANK_ID]:
    bus.write_position_kp(device_id, mid_kp)
    bus.write_position_kd(device_id, mid_kd)
    bus.write_torque_limit(device_id, mid_torque_limit)

hold_time_2 = 0.5
hold_steps_2 = int(hold_time_2 * rate_hz)

for _ in range(hold_steps_2):
    bus.write_read_pdo_2(THIGH_ID, start_thigh_motor, 0.0)
    bus.write_read_pdo_2(SHANK_ID, start_shank_motor, 0.0)
    rate.sleep()

# final gains
for device_id in [THIGH_ID, SHANK_ID]:
    bus.write_position_kp(device_id, kp)
    bus.write_position_kd(device_id, kd)
    bus.write_torque_limit(device_id, torque_limit)

# --------------------------------
# Main gait loop
# Map desired IK angles to motor commands RELATIVE to startup:
#
# cmd_thigh = start_thigh_motor + (theta_t_des - theta_t_ref0)
# cmd_shank = start_shank_motor + (theta_s_des - theta_s_ref0)
#
# This assumes the leg is assembled near the phase-0 posture when starting.
# If startup posture differs, tune x_center / z_ground or physically place
# the leg closer to the phase-0 pose before running.
# --------------------------------
print("Starting cycloid/straight-line foot trajectory...")
t0 = time.perf_counter()
counter = 0

try:
    while True:
        t = time.perf_counter() - t0
        phase = (t % cycle_time) / cycle_time

        x_des, z_des = foot_trajectory(phase)
        theta_t_des, theta_s_des = planar_leg_ik(x_des, z_des)

        cmd_thigh = start_thigh_motor + (theta_t_des - theta_t_ref0)
        cmd_shank = start_shank_motor + (theta_s_des - theta_s_ref0)

        thigh_pos, thigh_vel = bus.write_read_pdo_2(THIGH_ID, cmd_thigh, 0.0)
        shank_pos, shank_vel = bus.write_read_pdo_2(SHANK_ID, cmd_shank, 0.0)

        counter += 1
        if (
            thigh_pos is not None and thigh_vel is not None and
            shank_pos is not None and shank_vel is not None and
            counter % 10 == 0
        ):
            print(
                f"x={x_des:.1f} z={z_des:.1f} | "
                f"thigh cmd={cmd_thigh:.3f} pos={thigh_pos:.3f} vel={thigh_vel:.3f} | "
                f"shank cmd={cmd_shank:.3f} pos={shank_pos:.3f} vel={shank_vel:.3f}"
            )

        rate.sleep()

except KeyboardInterrupt:
    pass

bus.set_mode(THIGH_ID, recoil.Mode.IDLE)
bus.set_mode(SHANK_ID, recoil.Mode.IDLE)
bus.stop()

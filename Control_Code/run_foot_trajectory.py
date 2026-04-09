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
# GEAR RATIO (CRITICAL FIX)
# --------------------------------
GEAR_RATIO = 17.0

# --------------------------------
# Gains
# --------------------------------
kp = 0.2
kd = 0.005
torque_limit = 0.6

startup_kp = 0.003
startup_kd = 0.001
startup_torque_limit = 0.03

mid_kp = 0.03
mid_kd = 0.002
mid_torque_limit = 0.10

# --------------------------------
# Foot trajectory parameters
# --------------------------------
cycle_time = 2.0
stance_ratio = 0.5
step_length = 80.0
step_height = 40.0
x_center = 0.0
z_ground = 300.0

# --------------------------------
# Timing
# --------------------------------
rate_hz = 200.0
rate = RateLimiter(frequency=rate_hz)

stable_velocity_threshold = 0.2
required_stable_samples = 30
max_startup_time = 4.0


# --------------------------------
# Foot trajectory
# --------------------------------
def foot_trajectory(phase):
    phase = phase % 1.0

    if phase < stance_ratio:
        u = phase / stance_ratio
        x = x_center + (step_length / 2.0) - step_length * u
        z = z_ground
        return x, z

    u = (phase - stance_ratio) / (1.0 - stance_ratio)

    x = x_center - (step_length / 2.0) + step_length * (
        u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    )
    z = z_ground + step_height * (1.0 - math.cos(2.0 * math.pi * u)) / 2.0

    return x, z


# --------------------------------
# IK (still planar)
# --------------------------------
def planar_leg_ik(x, z):
    theta_h, theta_t, theta_s = leg_ik(x, 0.0, z)
    return theta_h, theta_t, theta_s


# --------------------------------
# Phase 0 reference
# --------------------------------
x0, z0 = foot_trajectory(0.0)
theta_h_ref0, theta_t_ref0, theta_s_ref0 = planar_leg_ik(x0, z0)

print(
    f"Ref angles: hip={theta_h_ref0:.3f}, thigh={theta_t_ref0:.3f}, shank={theta_s_ref0:.3f}")

# --------------------------------
# Startup: DAMPING for ALL joints
# --------------------------------
print("Damping mode for all joints...")
for jid in [HIP_ID, THIGH_ID, SHANK_ID]:
    bus.set_mode(jid, recoil.Mode.DAMPING)
    bus.feed(jid)

time.sleep(0.1)

stable_positions = {jid: [] for jid in [HIP_ID, THIGH_ID, SHANK_ID]}
stable_count = 0
t_start = time.perf_counter()

while True:
    hip_pos, hip_vel = bus.write_read_pdo_2(HIP_ID, 0.0, 0.0)
    thigh_pos, thigh_vel = bus.write_read_pdo_2(THIGH_ID, 0.0, 0.0)
    shank_pos, shank_vel = bus.write_read_pdo_2(SHANK_ID, 0.0, 0.0)

    if None not in [hip_pos, hip_vel, thigh_pos, thigh_vel, shank_pos, shank_vel]:

        if (
            abs(hip_vel) < stable_velocity_threshold and
            abs(thigh_vel) < stable_velocity_threshold and
            abs(shank_vel) < stable_velocity_threshold
        ):
            stable_positions[HIP_ID].append(hip_pos)
            stable_positions[THIGH_ID].append(thigh_pos)
            stable_positions[SHANK_ID].append(shank_pos)
            stable_count += 1
        else:
            for jid in stable_positions:
                stable_positions[jid].clear()
            stable_count = 0

        if stable_count >= required_stable_samples:
            break

    if time.perf_counter() - t_start > max_startup_time:
        raise RuntimeError("Startup not stable")

    rate.sleep()

start_hip = sum(stable_positions[HIP_ID]) / len(stable_positions[HIP_ID])
start_thigh = sum(stable_positions[THIGH_ID]) / len(stable_positions[THIGH_ID])
start_shank = sum(stable_positions[SHANK_ID]) / len(stable_positions[SHANK_ID])

print(
    f"Startup motor angles: hip={start_hip:.3f}, thigh={start_thigh:.3f}, shank={start_shank:.3f}")

# --------------------------------
# HIP stays in DAMPING (important)
# --------------------------------
# DO NOT switch hip to position mode
# It will resist motion but not actively move

# --------------------------------
# THIGH + SHANK → POSITION MODE
# --------------------------------
for jid in [THIGH_ID, SHANK_ID]:
    bus.write_position_kp(jid, startup_kp)
    bus.write_position_kd(jid, startup_kd)
    bus.write_torque_limit(jid, startup_torque_limit)
    bus.set_mode(jid, recoil.Mode.POSITION)
    bus.feed(jid)

time.sleep(0.05)

# --------------------------------
# Soft hold
# --------------------------------
for _ in range(int(0.6 * rate_hz)):
    bus.write_read_pdo_2(THIGH_ID, start_thigh, 0.0)
    bus.write_read_pdo_2(SHANK_ID, start_shank, 0.0)
    rate.sleep()

# mid gains
for jid in [THIGH_ID, SHANK_ID]:
    bus.write_position_kp(jid, mid_kp)
    bus.write_position_kd(jid, mid_kd)
    bus.write_torque_limit(jid, mid_torque_limit)

for _ in range(int(0.5 * rate_hz)):
    bus.write_read_pdo_2(THIGH_ID, start_thigh, 0.0)
    bus.write_read_pdo_2(SHANK_ID, start_shank, 0.0)
    rate.sleep()

# final gains
for jid in [THIGH_ID, SHANK_ID]:
    bus.write_position_kp(jid, kp)
    bus.write_position_kd(jid, kd)
    bus.write_torque_limit(jid, torque_limit)

# --------------------------------
# MAIN LOOP
# --------------------------------
print("Starting trajectory...")
t0 = time.perf_counter()

while True:
    t = time.perf_counter() - t0
    phase = (t % cycle_time) / cycle_time

    x_des, z_des = foot_trajectory(phase)
    theta_h_des, theta_t_des, theta_s_des = planar_leg_ik(x_des, z_des)

    # 🔥 APPLY GEAR RATIO HERE
    cmd_thigh = start_thigh + (theta_t_des - theta_t_ref0) * GEAR_RATIO
    cmd_shank = start_shank + (theta_s_des - theta_s_ref0) * GEAR_RATIO

    thigh_pos, thigh_vel = bus.write_read_pdo_2(THIGH_ID, cmd_thigh, 0.0)
    shank_pos, shank_vel = bus.write_read_pdo_2(SHANK_ID, cmd_shank, 0.0)

    if thigh_pos is not None and shank_pos is not None:
        print(
            f"x={x_des:.1f} z={z_des:.1f} | thigh={thigh_pos:.3f} shank={shank_pos:.3f}")

    rate.sleep()

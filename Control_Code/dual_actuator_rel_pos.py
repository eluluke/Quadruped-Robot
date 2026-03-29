import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

# -----------------------------
# Device IDs
# -----------------------------
thigh_id = 1
shank_id = 0

# -----------------------------
# Final gains
# -----------------------------
kp = 0.2
kd = 0.005
torque_limit = 1.5

# -----------------------------
# Very soft startup gains
# -----------------------------
startup_kp = 0.003
startup_kd = 0.001
startup_torque_limit = 0.03

mid_kp = 0.03
mid_kd = 0.002
mid_torque_limit = 0.10

# -----------------------------
# Relative motion commands
# positive = CCW, negative = CW
# -----------------------------
delta_theta_thigh = -15
delta_theta_shank = -15
# -----------------------------
# Motion timing
# -----------------------------
move_duration = 1  # seconds

# -----------------------------
# Startup stability settings
# -----------------------------
stable_velocity_threshold = 0.2   # rad/s
required_stable_samples = 30
max_startup_time = 4.0            # seconds

rate_hz = 200.0
rate = RateLimiter(frequency=rate_hz)

print("Entering damping mode for both joints...")
bus.set_mode(thigh_id, recoil.Mode.DAMPING)
bus.set_mode(shank_id, recoil.Mode.DAMPING)
bus.feed(thigh_id)
bus.feed(shank_id)
time.sleep(0.1)

# --------------------------------
# Wait for BOTH joints to become stable together
# --------------------------------
stable_thigh_positions = []
stable_shank_positions = []
stable_count = 0

t_start = time.perf_counter()

while True:
    thigh_pos, thigh_vel = bus.write_read_pdo_2(thigh_id, 0.0, 0.0)
    shank_pos, shank_vel = bus.write_read_pdo_2(shank_id, 0.0, 0.0)

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
        bus.set_mode(thigh_id, recoil.Mode.IDLE)
        bus.set_mode(shank_id, recoil.Mode.IDLE)
        bus.stop()
        raise RuntimeError(
            "Could not get stable startup positions for both joints.")

    rate.sleep()

start_thigh = sum(stable_thigh_positions) / len(stable_thigh_positions)
start_shank = sum(stable_shank_positions) / len(stable_shank_positions)

target_thigh = start_thigh + delta_theta_thigh
target_shank = start_shank + delta_theta_shank

print(f"Thigh start:   {start_thigh:.3f} rad")
print(f"Shank start:   {start_shank:.3f} rad")
print(f"Thigh delta:   {delta_theta_thigh:.3f} rad")
print(f"Shank delta:   {delta_theta_shank:.3f} rad")
print(f"Thigh target:  {target_thigh:.3f} rad")
print(f"Shank target:  {target_shank:.3f} rad")

# --------------------------------
# Enter POSITION softly for both joints
# --------------------------------
for device_id in [thigh_id, shank_id]:
    bus.write_position_kp(device_id, startup_kp)
    bus.write_position_kd(device_id, startup_kd)
    bus.write_torque_limit(device_id, startup_torque_limit)
    bus.set_mode(device_id, recoil.Mode.POSITION)
    bus.feed(device_id)

time.sleep(0.05)

# Soft hold current positions
print("Soft holding both joints...")
hold_time_1 = 0.6
hold_steps_1 = int(hold_time_1 * rate_hz)

for _ in range(hold_steps_1):
    bus.write_read_pdo_2(thigh_id, start_thigh, 0.0)
    bus.write_read_pdo_2(shank_id, start_shank, 0.0)
    rate.sleep()

# --------------------------------
# Ramp to medium gains
# --------------------------------
for device_id in [thigh_id, shank_id]:
    bus.write_position_kp(device_id, mid_kp)
    bus.write_position_kd(device_id, mid_kd)
    bus.write_torque_limit(device_id, mid_torque_limit)

hold_time_2 = 0.5
hold_steps_2 = int(hold_time_2 * rate_hz)

for _ in range(hold_steps_2):
    bus.write_read_pdo_2(thigh_id, start_thigh, 0.0)
    bus.write_read_pdo_2(shank_id, start_shank, 0.0)
    rate.sleep()

# --------------------------------
# Switch to final gains
# --------------------------------
for device_id in [thigh_id, shank_id]:
    bus.write_position_kp(device_id, kp)
    bus.write_position_kd(device_id, kd)
    bus.write_torque_limit(device_id, torque_limit)

# --------------------------------
# Smooth cosine move for both joints
# q(t) = q0 + dq/2 * (1 - cos(pi*t/T))
# --------------------------------
print("Starting smooth dual-joint move...")
t0 = time.perf_counter()
counter = 0

try:
    while True:
        t = time.perf_counter() - t0

        if t >= move_duration:
            cmd_thigh = target_thigh
            cmd_shank = target_shank
        else:
            s = 0.5 * (1.0 - math.cos(math.pi * t / move_duration))
            cmd_thigh = start_thigh + delta_theta_thigh * s
            cmd_shank = start_shank + delta_theta_shank * s

        thigh_pos, thigh_vel = bus.write_read_pdo_2(thigh_id, cmd_thigh, 0.0)
        shank_pos, shank_vel = bus.write_read_pdo_2(shank_id, cmd_shank, 0.0)

        counter += 1
        if (
            thigh_pos is not None and thigh_vel is not None and
            shank_pos is not None and shank_vel is not None and
            counter % 10 == 0
        ):
            print(
                f"Thigh cmd: {cmd_thigh:.3f} \tpos: {thigh_pos:.3f} \tvel: {thigh_vel:.3f} | "
                f"Shank cmd: {cmd_shank:.3f} \tpos: {shank_pos:.3f} \tvel: {shank_vel:.3f}"
            )

        rate.sleep()

except KeyboardInterrupt:
    pass

bus.set_mode(thigh_id, recoil.Mode.IDLE)
bus.set_mode(shank_id, recoil.Mode.IDLE)
bus.stop()

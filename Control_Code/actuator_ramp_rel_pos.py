import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

# -----------------------------
# Final gains
# -----------------------------
kp = 0.2
kd = 0.005
torque_limit = 0.8

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
# Relative motion command
# positive = CCW, negative = CW
# -----------------------------
delta_theta = -5

# -----------------------------
# Motion timing
# Use time-based smooth interpolation
# -----------------------------
move_duration = 2.0   # seconds

rate_hz = 200.0
rate = RateLimiter(frequency=rate_hz)

# -----------------------------
# Startup stability settings
# -----------------------------
stable_velocity_threshold = 0.2   # rad/s
required_stable_samples = 30
max_startup_time = 4.0            # seconds

print("Entering damping mode for startup settle...")
bus.set_mode(device_id, recoil.Mode.DAMPING)
bus.feed(device_id)
time.sleep(0.1)

# --------------------------------
# Wait for stable encoder readings in damping mode
# --------------------------------
stable_positions = []
stable_count = 0

t_start = time.perf_counter()

while True:
    # In this API, write_read_pdo_2 is the only easy feedback path,
    # so we call it with zero targets while in DAMPING mode.
    measured_position, measured_velocity = bus.write_read_pdo_2(
        device_id, 0.0, 0.0
    )

    if measured_position is not None and measured_velocity is not None:
        if abs(measured_velocity) < stable_velocity_threshold:
            stable_positions.append(measured_position)
            stable_count += 1
        else:
            stable_positions.clear()
            stable_count = 0

        if stable_count >= required_stable_samples:
            break

    if (time.perf_counter() - t_start) > max_startup_time:
        bus.set_mode(device_id, recoil.Mode.IDLE)
        bus.stop()
        raise RuntimeError("Could not get a stable encoder position at startup.")

    rate.sleep()

start_position = sum(stable_positions) / len(stable_positions)
target_angle = start_position + delta_theta

print(f"Stable start position: {start_position:.3f} rad")
print(f"Delta command:         {delta_theta:.3f} rad")
print(f"Target angle:          {target_angle:.3f} rad")

# --------------------------------
# Enter position mode very softly
# --------------------------------
bus.write_position_kp(device_id, startup_kp)
bus.write_position_kd(device_id, startup_kd)
bus.write_torque_limit(device_id, startup_torque_limit)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)
time.sleep(0.05)

# Hold current position softly first
print("Soft hold...")
hold_time_1 = 0.6
hold_steps_1 = int(hold_time_1 * rate_hz)

for _ in range(hold_steps_1):
    measured_position, measured_velocity = bus.write_read_pdo_2(
        device_id, start_position, 0.0
    )
    rate.sleep()

# --------------------------------
# Ramp to medium gains
# --------------------------------
bus.write_position_kp(device_id, mid_kp)
bus.write_position_kd(device_id, mid_kd)
bus.write_torque_limit(device_id, mid_torque_limit)

hold_time_2 = 0.5
hold_steps_2 = int(hold_time_2 * rate_hz)

for _ in range(hold_steps_2):
    measured_position, measured_velocity = bus.write_read_pdo_2(
        device_id, start_position, 0.0
    )
    rate.sleep()

# --------------------------------
# Switch to final gains
# --------------------------------
bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, torque_limit)

# --------------------------------
# Smooth cosine move
# q(t) = q0 + dq/2 * (1 - cos(pi*t/T))
# zero start/end velocity
# --------------------------------
print("Starting smooth move...")
t0 = time.perf_counter()
counter = 0

try:
    while True:
        t = time.perf_counter() - t0

        if t >= move_duration:
            command_angle = target_angle
        else:
            s = 0.5 * (1.0 - math.cos(math.pi * t / move_duration))
            command_angle = start_position + delta_theta * s

        measured_position, measured_velocity = bus.write_read_pdo_2(
            device_id, command_angle, 0.0
        )

        counter += 1
        if measured_position is not None and measured_velocity is not None:
            if counter % 20 == 0:
                print(
                    f"Cmd: {command_angle:.3f} \t"
                    f"Target: {target_angle:.3f} \t"
                    f"Measured pos: {measured_position:.3f} \t"
                    f"vel: {measured_velocity:.3f}"
                )

        rate.sleep()

except KeyboardInterrupt:
    pass

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()

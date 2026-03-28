import time

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

# final gains
kp = 0.2
kd = 0.005
torque_limit = 1.3

# very soft startup gains
startup_kp = 0.008
startup_kd = 0.001
startup_torque_limit = 0.05

# medium transition gains
mid_kp = 0.05
mid_kd = 0.002
mid_torque_limit = 0.20

# --------------------------------
# Relative motion command in radians
# positive = CCW, negative = CW
# --------------------------------
delta_theta = -100

# Motion profile settings
max_velocity = 60.0
acceleration = 50.0
deceleration = 50.0

rate_hz = 200
dt = 1.0 / rate_hz
rate = RateLimiter(frequency=rate_hz)

stable_velocity_threshold = 0.5
required_stable_reads = 20

# -----------------------------
# Start in very soft position mode
# -----------------------------
bus.write_position_kp(device_id, startup_kp)
bus.write_position_kd(device_id, startup_kd)
bus.write_torque_limit(device_id, startup_torque_limit)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)
time.sleep(0.1)

# -----------------------------
# Get one initial measurement
# -----------------------------
measured_position = None
measured_velocity = None

for _ in range(20):
    measured_position, measured_velocity = bus.write_read_pdo_2(
        device_id, 0.0, 0.0
    )
    if measured_position is not None and measured_velocity is not None:
        break
    time.sleep(0.05)

if measured_position is None:
    bus.set_mode(device_id, recoil.Mode.IDLE)
    bus.stop()
    raise RuntimeError("Could not read actuator position at startup.")

# Immediately latch current position as command
command_angle = measured_position
command_velocity = 0.0

# -----------------------------
# Soft capture phase:
# keep following the measured position very gently
# until the joint settles after any backdrive
# -----------------------------
stable_positions = []
stable_count = 0

for _ in range(600):  # 3 seconds max at 200 Hz
    measured_position, measured_velocity = bus.write_read_pdo_2(
        device_id, command_angle, 0.0
    )

    if measured_position is not None and measured_velocity is not None:
        # follow the measured position softly
        command_angle = measured_position

        if abs(measured_velocity) < stable_velocity_threshold:
            stable_positions.append(measured_position)
            stable_count += 1
        else:
            stable_positions.clear()
            stable_count = 0

        if stable_count >= required_stable_reads:
            break

    rate.sleep()

if len(stable_positions) == 0:
    bus.set_mode(device_id, recoil.Mode.IDLE)
    bus.stop()
    raise RuntimeError("Could not get a stable actuator position at startup.")

start_position = sum(stable_positions) / len(stable_positions)
target_angle = start_position + delta_theta

print(f"Start position: {start_position:.3f} rad")
print(f"Delta command:  {delta_theta:.3f} rad")
print(f"Target angle:   {target_angle:.3f} rad")

command_angle = start_position
command_velocity = 0.0

# -----------------------------
# Hold current position gently
# -----------------------------
for _ in range(int(0.8 * rate_hz)):
    measured_position, measured_velocity = bus.write_read_pdo_2(
        device_id, command_angle, 0.0
    )
    rate.sleep()

# -----------------------------
# Ramp gains up to medium first
# -----------------------------
bus.write_position_kp(device_id, mid_kp)
bus.write_position_kd(device_id, mid_kd)
bus.write_torque_limit(device_id, mid_torque_limit)

for _ in range(int(0.5 * rate_hz)):
    measured_position, measured_velocity = bus.write_read_pdo_2(
        device_id, command_angle, 0.0
    )
    rate.sleep()

# -----------------------------
# Then switch to final gains
# -----------------------------
bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, torque_limit)

counter = 0

try:
    while True:
        error = target_angle - command_angle

        if abs(error) < 1e-4 and abs(command_velocity) < 1e-3:
            command_angle = target_angle
            command_velocity = 0.0
        else:
            direction = 1.0 if error > 0 else -1.0

            stopping_distance = (command_velocity ** 2) / (2.0 * deceleration)

            effective_acceleration = acceleration
            if command_velocity < 3.0:
                effective_acceleration = acceleration * 0.15

            if abs(error) > stopping_distance:
                command_velocity += effective_acceleration * dt
                if command_velocity > max_velocity:
                    command_velocity = max_velocity
            else:
                command_velocity -= deceleration * dt
                if command_velocity < 0.0:
                    command_velocity = 0.0

            step = direction * command_velocity * dt

            if abs(step) > abs(error):
                command_angle = target_angle
                command_velocity = 0.0
            else:
                command_angle += step

        measured_position, measured_velocity = bus.write_read_pdo_2(
            device_id, command_angle, 0.0
        )

        counter += 1
        if measured_position is not None and measured_velocity is not None:
            if counter % 10 == 0:
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

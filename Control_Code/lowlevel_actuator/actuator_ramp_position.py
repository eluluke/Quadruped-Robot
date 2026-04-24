import time

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

kp = 0.30
kd = 0.008

# Final target position in radians(angle range from calibrated position:20 rad)
target_angle = -5

# Motion profile settings
max_velocity = 120.0  # rad/s
acceleration = 100.0  # rad/s^2
deceleration = 100.0  # rad/s^2

rate_hz = 200.0
dt = 1.0 / rate_hz
rate = RateLimiter(frequency=rate_hz)

bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, 1.3)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

# Give the actuator a brief moment to enter mode
time.sleep(0.1)

# Start from current measured position, with retry
measured_position = None
measured_velocity = None

for _ in range(20):
    measured_position, measured_velocity = bus.write_read_pdo_2(
        device_id, 0.0, 0.0)
    if measured_position is not None:
        break
    time.sleep(0.05)

if measured_position is None:
    bus.set_mode(device_id, recoil.Mode.IDLE)
    bus.stop()
    raise RuntimeError("Could not read actuator position at startup.")

command_angle = measured_position
command_velocity = 0.0

try:
    while True:
        error = target_angle - command_angle

        if abs(error) < 1e-4 and abs(command_velocity) < 1e-3:
            command_angle = target_angle
            command_velocity = 0.0
        else:
            direction = 1.0 if error > 0 else -1.0

            # Distance needed to stop from current speed
            stopping_distance = (command_velocity ** 2) / (2.0 * deceleration)

            if abs(error) > stopping_distance:
                # Accelerate
                command_velocity += acceleration * dt
                if command_velocity > max_velocity:
                    command_velocity = max_velocity
            else:
                # Decelerate
                command_velocity -= deceleration * dt
                if command_velocity < 0.0:
                    command_velocity = 0.0

            step = direction * command_velocity * dt

            # Prevent overshoot
            if abs(step) > abs(error):
                command_angle = target_angle
                command_velocity = 0.0
            else:
                command_angle += step

        measured_position, measured_velocity = bus.write_read_pdo_2(
            device_id, command_angle, 0.0
        )

        if measured_position is not None and measured_velocity is not None:
            print(
                f"Cmd: {command_angle:.3f} \t"
                f"Target: {target_angle:.3f} \t"
                f"Measured pos: {measured_position:.3f} \t"
                f"vel: {measured_velocity:.3f}"
            )
        else:
            print("Warning: no response from actuator this cycle.")

        rate.sleep()

except KeyboardInterrupt:
    pass

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()

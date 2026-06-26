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
kp = 0.45
kd = 0.015
torque_limit = 0.3

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
move_duration = 5.0  # seconds

rate_hz = 80.0
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
        raise RuntimeError(
            "Could not get a stable encoder position at startup.")

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



config = {
    "position_controller": {},
    "current_controller": {},
    "powerstage": {},
    "motor": {},
    "encoder": {},
}

config["device_id"] = bus._read_parameter_u32(device_id, recoil.Parameter.DEVICE_ID)
config["firmware_version"] = hex(bus._read_parameter_u32(device_id, recoil.Parameter.FIRMWARE_VERSION))
config["watchdog_timeout"] = bus._read_parameter_u32(device_id, recoil.Parameter.WATCHDOG_TIMEOUT)
config["fast_frame_frequency"] = bus.read_fast_frame_frequency(device_id)

config["position_controller"]["gear_ratio"] = bus._read_parameter_f32(device_id, recoil.Parameter.POSITION_CONTROLLER_GEAR_RATIO)
config["position_controller"]["position_kp"] = bus.read_position_kp(device_id)
config["position_controller"]["position_ki"] = bus.read_position_ki(device_id)
config["position_controller"]["velocity_kp"] = bus.read_velocity_kp(device_id)
config["position_controller"]["velocity_ki"] = bus.read_velocity_ki(device_id)
config["position_controller"]["torque_limit"] = bus.read_torque_limit(device_id)
config["position_controller"]["velocity_limit"] = bus.read_velocity_limit(device_id)
config["position_controller"]["position_limit_upper"] = bus.read_position_limit_upper(device_id)
config["position_controller"]["position_limit_lower"] = bus.read_position_limit_lower(device_id)
config["position_controller"]["position_offset"] = bus.read_position_offset(device_id)
config["position_controller"]["torque_filter_alpha"] = bus.read_torque_filter_alpha(device_id)

config["current_controller"]["i_limit"] = bus.read_current_limit(device_id)
config["current_controller"]["i_kp"] = bus.read_current_kp(device_id)
config["current_controller"]["i_ki"] = bus.read_current_ki(device_id)

config["powerstage"]["undervoltage_threshold"] = bus._read_parameter_f32(device_id, recoil.Parameter.POWERSTAGE_UNDERVOLTAGE_THRESHOLD)
config["powerstage"]["overvoltage_threshold"] = bus._read_parameter_f32(device_id, recoil.Parameter.POWERSTAGE_OVERVOLTAGE_THRESHOLD)
config["powerstage"]["bus_voltage_filter_alpha"] = bus.read_bus_voltage_filter_alpha(device_id)

config["motor"]["pole_pairs"] = bus.read_motor_pole_pairs(device_id)
config["motor"]["torque_constant"] = bus.read_motor_torque_constant(device_id)
config["motor"]["phase_order"] = bus.read_motor_phase_order(device_id)
config["motor"]["max_calibration_current"] = bus.read_motor_calibration_current(device_id)

config["encoder"]["cpr"] = bus.read_encoder_cpr(device_id)
config["encoder"]["position_offset"] = bus.read_encoder_position_offset(device_id)
config["encoder"]["velocity_filter_alpha"] = bus.read_encoder_velocity_filter_alpha(device_id)
config["encoder"]["flux_offset"] = bus.read_encoder_flux_offset(device_id)

time.sleep(0.1)
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

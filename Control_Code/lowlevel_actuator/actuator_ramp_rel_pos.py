import time
import math
import struct

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
torque_limit = 0.8
gear_ratio = 25

# Integral gains are written explicitly so a value left by an earlier test
# cannot silently affect this position-control run.
position_ki = 0.0
velocity_ki = 0.0

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
# Relative output-joint motion command.
# Positive = CCW, negative = CW. The firmware position command uses
# motor-side radians, so convert exactly once before building the target.
# -----------------------------
joint_delta_rad = -1.0
motor_delta_rad = joint_delta_rad * gear_ratio

# -----------------------------
# Motion timing
# Use time-based smooth interpolation
# -----------------------------
move_duration = 5.0  # seconds

rate_hz = 80.0
rate = RateLimiter(frequency=rate_hz)

pdo_timeout_s = 0.02
diagnostic_timeout_s = 0.10
telemetry_every = 20
full_diagnostics_every = 80
maximum_consecutive_pdo_timeouts = 5


ERROR_FLAGS = (
    (recoil.ErrorCode.GENERAL, "GENERAL"),
    (recoil.ErrorCode.ESTOP, "ESTOP"),
    (recoil.ErrorCode.INITIALIZATION_ERROR, "INITIALIZATION_ERROR"),
    (recoil.ErrorCode.CALIBRATION_ERROR, "CALIBRATION_ERROR"),
    (recoil.ErrorCode.POWERSTAGE_ERROR, "POWERSTAGE_ERROR"),
    (recoil.ErrorCode.INVALID_MODE, "INVALID_MODE"),
    (recoil.ErrorCode.WATCHDOG_TIMEOUT, "WATCHDOG_TIMEOUT"),
    (recoil.ErrorCode.OVER_VOLTAGE, "OVER_VOLTAGE"),
    (recoil.ErrorCode.OVER_CURRENT, "OVER_CURRENT"),
    (recoil.ErrorCode.OVER_TEMPERATURE, "OVER_TEMPERATURE"),
    (recoil.ErrorCode.CAN_TX_FAULT, "CAN_TX_FAULT"),
    (recoil.ErrorCode.I2C_FAULT, "I2C_FAULT"),
)


def read_u32(parameter):
    try:
        return bus._read_parameter_u32(
            device_id,
            parameter,
            timeout=diagnostic_timeout_s,
        )
    except Exception as exc:
        print(f"  diagnostic u32 read failed: {exc}")
        return None


def read_f32(parameter):
    try:
        return bus._read_parameter_f32(
            device_id,
            parameter,
            timeout=diagnostic_timeout_s,
        )
    except Exception as exc:
        print(f"  diagnostic f32 read failed: {exc}")
        return None


def format_value(value, precision=4):
    if value is None:
        return "no response"
    return f"{value:.{precision}f}"


def format_error(error_value):
    if error_value is None:
        return "no response"
    if error_value == recoil.ErrorCode.NO_ERROR:
        return "0x0000 (NO_ERROR)"

    names = [
        name
        for mask, name in ERROR_FLAGS
        if error_value & mask
    ]
    description = "|".join(names) if names else "UNKNOWN"
    return f"0x{error_value:04X} ({description})"


def write_gains(kp_value, kd_value, torque_value):
    bus.write_position_kp(device_id, kp_value)
    time.sleep(0.004)
    bus.write_position_ki(device_id, position_ki)
    time.sleep(0.004)
    bus.write_position_kd(device_id, kd_value)
    time.sleep(0.004)
    bus.write_velocity_ki(device_id, velocity_ki)
    time.sleep(0.004)
    bus.write_torque_limit(device_id, torque_value)
    time.sleep(0.004)


def read_position_feedback(position_target):
    """Send one position target and wait for the matching TPDO2 response."""
    try:
        bus.transmit_pdo_2(device_id, position_target, 0.0)
        frame = bus.receive(
            filter_device_id=device_id,
            filter_function=recoil.Function.TRANSMIT_PDO_2,
            timeout=pdo_timeout_s,
        )
    except Exception as exc:
        print(f"PDO2 communication error: {exc}")
        return None, None

    if frame is None:
        return None, None
    if len(frame.data) < 8:
        print(f"PDO2 response has invalid length: {len(frame.data)}")
        return None, None

    return struct.unpack("<ff", frame.data[:8])


def read_controller_diagnostics(label):
    mode = read_u32(recoil.Parameter.MODE)
    error = read_u32(recoil.Parameter.ERROR)
    bus_voltage = read_f32(recoil.Parameter.POWERSTAGE_BUS_VOLTAGE_MEASURED)
    position_kp_read = read_f32(
        recoil.Parameter.POSITION_CONTROLLER_POSITION_KP
    )
    position_ki_read = read_f32(
        recoil.Parameter.POSITION_CONTROLLER_POSITION_KI
    )
    velocity_kp_read = read_f32(
        recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KP
    )
    velocity_ki_read = read_f32(
        recoil.Parameter.POSITION_CONTROLLER_VELOCITY_KI
    )
    torque_limit_read = read_f32(
        recoil.Parameter.POSITION_CONTROLLER_TORQUE_LIMIT
    )
    current_limit = read_f32(recoil.Parameter.CURRENT_CONTROLLER_I_LIMIT)

    print(f"\nController diagnostics: {label}")
    mode_text = "no response" if mode is None else f"0x{mode:02X}"
    print(f"  mode:          {mode_text} (POSITION expected: 0x13)")
    print(f"  error:         {format_error(error)}")
    print(f"  bus voltage:   {format_value(bus_voltage)} V")
    print(f"  position kp:   {format_value(position_kp_read)}")
    print(f"  position ki:   {format_value(position_ki_read)}")
    print(f"  velocity kp:   {format_value(velocity_kp_read)}")
    print(f"  velocity ki:   {format_value(velocity_ki_read)}")
    print(f"  torque limit:  {format_value(torque_limit_read)} Nm")
    print(f"  current limit: {format_value(current_limit)} A")

    return mode, error


def read_runtime_diagnostics():
    mode = read_u32(recoil.Parameter.MODE)
    error = read_u32(recoil.Parameter.ERROR)
    bus_voltage = read_f32(recoil.Parameter.POWERSTAGE_BUS_VOLTAGE_MEASURED)
    torque_target = read_f32(
        recoil.Parameter.POSITION_CONTROLLER_TORQUE_TARGET
    )
    torque_measured = read_f32(
        recoil.Parameter.POSITION_CONTROLLER_TORQUE_MEASURED
    )
    iq_target = read_f32(recoil.Parameter.CURRENT_CONTROLLER_I_Q_TARGET)
    iq_measured = read_f32(recoil.Parameter.CURRENT_CONTROLLER_I_Q_MEASURED)

    mode_text = "none" if mode is None else f"0x{mode:02X}"
    print(
        "  controller: "
        f"mode={mode_text}, "
        f"error={format_error(error)}, "
        f"vbus={format_value(bus_voltage, 2)}V, "
        f"tq_target={format_value(torque_target, 3)}Nm, "
        f"tq_measured={format_value(torque_measured, 3)}Nm, "
        f"iq_target={format_value(iq_target, 2)}A, "
        f"iq_measured={format_value(iq_measured, 2)}A"
    )

    return mode, error


def abort_before_motion(message):
    print(f"\nSafety stop: {message}")
    try:
        bus.set_mode(device_id, recoil.Mode.IDLE)
    except Exception as exc:
        print(f"Failed to send IDLE: {exc}")
    finally:
        bus.stop()
    raise SystemExit(1)


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
    measured_position, measured_velocity = read_position_feedback(0.0)

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
target_angle = start_position + motor_delta_rad

print(f"Stable start position: {start_position:.3f} rad")
print(f"Joint delta command:   {joint_delta_rad:.3f} rad")
print(f"Motor delta command:   {motor_delta_rad:.3f} rad")
print(f"Motor target angle:    {target_angle:.3f} rad")

# --------------------------------
# Enter position mode very softly
# --------------------------------
write_gains(startup_kp, startup_kd, startup_torque_limit)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)
time.sleep(0.05)
mode, error = read_controller_diagnostics("startup gains applied")
if mode is not None and mode != recoil.Mode.POSITION:
    abort_before_motion(
        f"Controller rejected POSITION mode; reported mode=0x{mode:02X}."
    )
if error not in (None, recoil.ErrorCode.NO_ERROR):
    abort_before_motion(
        f"Controller reported an error before motion: {format_error(error)}"
    )

# Hold current position softly first
print("Soft hold...")
hold_time_1 = 0.6
hold_steps_1 = int(hold_time_1 * rate_hz)


for _ in range(hold_steps_1):
    measured_position, measured_velocity = read_position_feedback(
        start_position
    )
    rate.sleep()

# --------------------------------
# Ramp to medium gains
# --------------------------------
write_gains(mid_kp, mid_kd, mid_torque_limit)

hold_time_2 = 0.5
hold_steps_2 = int(hold_time_2 * rate_hz)

for _ in range(hold_steps_2):
    measured_position, measured_velocity = read_position_feedback(
        start_position
    )
    rate.sleep()

# --------------------------------
# Switch to final gains
# --------------------------------
write_gains(kp, kd, torque_limit)
mode, error = read_controller_diagnostics("final gains applied")
if mode is not None and mode != recoil.Mode.POSITION:
    abort_before_motion(
        f"Controller left POSITION mode; reported mode=0x{mode:02X}."
    )
if error not in (None, recoil.ErrorCode.NO_ERROR):
    abort_before_motion(
        f"Controller reported an error before ramp: {format_error(error)}"
    )

# --------------------------------
# Smooth cosine move
# q(t) = q0 + dq/2 * (1 - cos(pi*t/T))
# zero start/end velocity
# --------------------------------
print("Starting smooth move...")
t0 = time.perf_counter()
counter = 0
consecutive_pdo_timeouts = 0

try:
    while True:
        t = time.perf_counter() - t0

        if t >= move_duration:
            command_angle = target_angle
        else:
            s = 0.5 * (1.0 - math.cos(math.pi * t / move_duration))
            command_angle = start_position + motor_delta_rad * s

        measured_position, measured_velocity = read_position_feedback(
            command_angle
        )

        counter += 1
        if measured_position is not None and measured_velocity is not None:
            consecutive_pdo_timeouts = 0
            if counter % telemetry_every == 0:
                position_error = command_angle - measured_position
                print(
                    f"Cmd: {command_angle:.3f} \t"
                    f"Measured: {measured_position:.3f} \t"
                    f"Error: {position_error:+.3f} \t"
                    f"Velocity: {measured_velocity:+.3f}"
                )
        else:
            consecutive_pdo_timeouts += 1
            print(
                "No matching PDO2 response "
                f"({consecutive_pdo_timeouts}/"
                f"{maximum_consecutive_pdo_timeouts})"
            )
            if (
                consecutive_pdo_timeouts
                >= maximum_consecutive_pdo_timeouts
            ):
                raise RuntimeError(
                    "Too many consecutive PDO2 timeouts; stopping motor."
                )

        if counter % full_diagnostics_every == 0:
            mode, error = read_runtime_diagnostics()
            if mode is not None and mode != recoil.Mode.POSITION:
                raise RuntimeError(
                    "Controller left POSITION mode during motion: "
                    f"0x{mode:02X}."
                )
            if error not in (None, recoil.ErrorCode.NO_ERROR):
                raise RuntimeError(
                    "Controller fault during motion: "
                    f"{format_error(error)}"
                )

        rate.sleep()

except KeyboardInterrupt:
    pass
except RuntimeError as exc:
    print(f"\nSafety stop: {exc}")

finally:
    try:
        bus.set_mode(device_id, recoil.Mode.IDLE)
    except Exception as exc:
        print(f"Failed to send IDLE: {exc}")
    bus.stop()

import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

# -----------------------------
# Motor IDs
# -----------------------------
HIP_ID = 2
THIGH_ID = 1
SHANK_ID = 0

MOTOR_IDS = [HIP_ID, THIGH_ID, SHANK_ID]
MOTOR_NAMES = {
    HIP_ID: "Hip",
    THIGH_ID: "Thigh",
    SHANK_ID: "Shank",
}

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
# Same delta for all 3 joints
# -----------------------------
delta_theta = 12.0

# -----------------------------
# Motion timing
# -----------------------------
move_duration = 2.0

rate_hz = 200.0
rate = RateLimiter(frequency=rate_hz)

# -----------------------------
# Startup stability settings
# -----------------------------
stable_velocity_threshold = 0.2
required_stable_samples = 30
max_startup_time = 4.0

print_every = 20

print("Entering damping mode for startup settle...")
for motor_id in MOTOR_IDS:
    bus.set_mode(motor_id, recoil.Mode.DAMPING)
    bus.feed(motor_id)
    time.sleep(0.003)

time.sleep(0.1)

# --------------------------------
# Wait for stable encoder readings in damping mode
# --------------------------------
stable_positions = {
    HIP_ID: [],
    THIGH_ID: [],
    SHANK_ID: [],
}
stable_count = 0

t_start = time.perf_counter()

while True:
    measured = {}
    valid = True
    all_stable = True

    for motor_id in MOTOR_IDS:
        measured_position, measured_velocity = bus.write_read_pdo_2(
            motor_id, 0.0, 0.0
        )
        measured[motor_id] = (measured_position, measured_velocity)

        if measured_position is None or measured_velocity is None:
            valid = False
        elif abs(measured_velocity) >= stable_velocity_threshold:
            all_stable = False

        time.sleep(0.001)

    if valid:
        if all_stable:
            for motor_id in MOTOR_IDS:
                stable_positions[motor_id].append(measured[motor_id][0])
            stable_count += 1
        else:
            for motor_id in MOTOR_IDS:
                stable_positions[motor_id].clear()
            stable_count = 0

        if stable_count >= required_stable_samples:
            break

    if (time.perf_counter() - t_start) > max_startup_time:
        for motor_id in MOTOR_IDS:
            bus.set_mode(motor_id, recoil.Mode.IDLE)
            time.sleep(0.003)
        bus.stop()
        raise RuntimeError(
            "Could not get stable encoder positions at startup."
        )

    rate.sleep()

start_positions = {}
target_positions = {}

for motor_id in MOTOR_IDS:
    start_positions[motor_id] = (
        sum(stable_positions[motor_id]) / len(stable_positions[motor_id])
    )
    target_positions[motor_id] = start_positions[motor_id] + delta_theta

print("Stable start positions:")
for motor_id in MOTOR_IDS:
    print(
        f"{MOTOR_NAMES[motor_id]}: "
        f"start={start_positions[motor_id]:.3f} rad | "
        f"target={target_positions[motor_id]:.3f} rad"
    )

# --------------------------------
# Enter position mode very softly
# --------------------------------
for motor_id in MOTOR_IDS:
    bus.write_position_kp(motor_id, startup_kp)
    bus.write_position_kd(motor_id, startup_kd)
    bus.write_torque_limit(motor_id, startup_torque_limit)
    time.sleep(0.003)

for motor_id in MOTOR_IDS:
    bus.set_mode(motor_id, recoil.Mode.POSITION)
    bus.feed(motor_id)
    time.sleep(0.003)

time.sleep(0.05)

# Hold current positions softly first
print("Soft hold...")
hold_time_1 = 0.6
hold_steps_1 = int(hold_time_1 * rate_hz)

for _ in range(hold_steps_1):
    for motor_id in MOTOR_IDS:
        bus.write_read_pdo_2(
            motor_id,
            start_positions[motor_id],
            0.0,
        )
        time.sleep(0.001)
    rate.sleep()

# --------------------------------
# Ramp to medium gains
# --------------------------------
for motor_id in MOTOR_IDS:
    bus.write_position_kp(motor_id, mid_kp)
    bus.write_position_kd(motor_id, mid_kd)
    bus.write_torque_limit(motor_id, mid_torque_limit)
    time.sleep(0.003)

hold_time_2 = 0.5
hold_steps_2 = int(hold_time_2 * rate_hz)

for _ in range(hold_steps_2):
    for motor_id in MOTOR_IDS:
        bus.write_read_pdo_2(
            motor_id,
            start_positions[motor_id],
            0.0,
        )
        time.sleep(0.001)
    rate.sleep()

# --------------------------------
# Switch to final gains
# --------------------------------
for motor_id in MOTOR_IDS:
    bus.write_position_kp(motor_id, kp)
    bus.write_position_kd(motor_id, kd)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.003)

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
            s = 1.0
        else:
            s = 0.5 * (1.0 - math.cos(math.pi * t / move_duration))

        measured = {}

        for motor_id in MOTOR_IDS:
            command_angle = start_positions[motor_id] + delta_theta * s

            measured_position, measured_velocity = bus.write_read_pdo_2(
                motor_id,
                command_angle,
                0.0,
            )

            measured[motor_id] = (command_angle, measured_position, measured_velocity)
            time.sleep(0.001)

        counter += 1
        if counter % print_every == 0:
            for motor_id in MOTOR_IDS:
                command_angle, measured_position, measured_velocity = measured[motor_id]
                if measured_position is not None and measured_velocity is not None:
                    print(
                        f"{MOTOR_NAMES[motor_id]} | "
                        f"Cmd: {command_angle:.3f} | "
                        f"Target: {target_positions[motor_id]:.3f} | "
                        f"Measured pos: {measured_position:.3f} | "
                        f"vel: {measured_velocity:.3f}"
                    )
            print("-----")

        rate.sleep()

except KeyboardInterrupt:
    pass

for motor_id in MOTOR_IDS:
    bus.set_mode(motor_id, recoil.Mode.IDLE)
    time.sleep(0.003)

bus.stop()

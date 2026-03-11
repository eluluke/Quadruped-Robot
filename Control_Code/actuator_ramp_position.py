# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import math
import time

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


def sign(x):
    """Return the sign of x."""
    if x > 0:
        return 1.0
    if x < 0:
        return -1.0
    return 0.0


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

kp = 0.2
kd = 0.005
torque_limit = 0.2

# -----------------------------
# Motion settings
# -----------------------------
target_angle = 1.0       # final target position in rad
max_velocity = 1.0       # rad/s
acceleration = 2.0       # rad/s^2
deceleration = 2.0       # rad/s^2

rate_hz = 200.0
rate = RateLimiter(frequency=rate_hz)

bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, torque_limit)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

# -------------------------------------------------
# Read current position so trajectory starts there
# -------------------------------------------------
measured_position, measured_velocity = bus.write_read_pdo_2(
    device_id, 0.0, 0.0)
if measured_position is None:
    raise RuntimeError("Could not read actuator position at startup.")

start_angle = measured_position
distance = target_angle - start_angle
direction = sign(distance)
distance_abs = abs(distance)

# -----------------------------------------
# Compute trapezoidal / triangular profile
# -----------------------------------------
t_acc = max_velocity / acceleration
t_dec = max_velocity / deceleration

d_acc = 0.5 * acceleration * t_acc**2
d_dec = 0.5 * deceleration * t_dec**2

# Check if full trapezoid is possible
if d_acc + d_dec <= distance_abs:
    # True trapezoidal profile
    d_cruise = distance_abs - d_acc - d_dec
    t_cruise = d_cruise / max_velocity
    v_peak = max_velocity
else:
    # Not enough distance: triangular profile
    v_peak = math.sqrt(
        2.0 * distance_abs / (1.0 / acceleration + 1.0 / deceleration)
    )
    t_acc = v_peak / acceleration
    t_dec = v_peak / deceleration
    t_cruise = 0.0
    d_acc = 0.5 * acceleration * t_acc**2
    d_dec = 0.5 * deceleration * t_dec**2

total_time = t_acc + t_cruise + t_dec

print(f"Start angle:   {start_angle:.3f} rad")
print(f"Target angle:  {target_angle:.3f} rad")
print(f"Distance:      {distance:.3f} rad")
print(f"Peak velocity: {v_peak:.3f} rad/s")
print(f"Accel time:    {t_acc:.3f} s")
print(f"Cruise time:   {t_cruise:.3f} s")
print(f"Decel time:    {t_dec:.3f} s")
print(f"Total time:    {total_time:.3f} s")

t0 = time.time()

try:
    while True:
        t = time.time() - t0

        if t <= t_acc:
            # Acceleration phase
            s = 0.5 * acceleration * t**2
            v_cmd = acceleration * t

        elif t <= t_acc + t_cruise:
            # Constant velocity phase
            dt = t - t_acc
            s = d_acc + v_peak * dt
            v_cmd = v_peak

        elif t <= total_time:
            # Deceleration phase
            dt = t - t_acc - t_cruise
            s = d_acc + (v_peak * t_cruise) + (v_peak *
                                               dt - 0.5 * deceleration * dt**2)
            v_cmd = v_peak - deceleration * dt

        else:
            # Done
            s = distance_abs
            v_cmd = 0.0

        commanded_angle = start_angle + direction * s
        commanded_velocity = direction * v_cmd

        measured_position, measured_velocity = bus.write_read_pdo_2(
            device_id,
            commanded_angle,
            commanded_velocity
        )

        if measured_position is not None and measured_velocity is not None:
            print(
                f"Cmd: {commanded_angle:.3f} rad | "
                f"Cmd vel: {commanded_velocity:.3f} rad/s | "
                f"Meas pos: {measured_position:.3f} rad | "
                f"Meas vel: {measured_velocity:.3f} rad/s"
            )

        rate.sleep()

except KeyboardInterrupt:
    pass

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()

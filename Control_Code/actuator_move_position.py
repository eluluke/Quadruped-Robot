# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

kp = 0.2
kd = 0.005

# Fixed target position in radians,keep displacement small
target_angle = 1.0

rate = RateLimiter(frequency=200.0)

bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, 0.2)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

try:
    while True:
        measured_position, measured_velocity = bus.write_read_pdo_2(
            device_id, target_angle, 0.0
        )

        if measured_position is not None and measured_velocity is not None:
            print(
                f"Target: {target_angle:.3f} \t"
                f"Measured pos: {measured_position:.3f} \t"
                f"vel: {measured_velocity:.3f}"
            )

        rate.sleep()

except KeyboardInterrupt:
    pass

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()

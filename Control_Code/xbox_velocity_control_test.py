# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Example: control actuator speed with an Xbox controller.
#
# Run:  python velocity_actuator.py -c can0 -i 1

from loop_rate_limiters import RateLimiter
import recoil as recoil

from helper.xbox_controller import XboxController

# ── Motion scaling ────────────────────────────────────────────────────────────
MAX_VELOCITY = 3.0   # rad/s — full trigger or full stick deflection

# ── Motor gains ───────────────────────────────────────────────────────────────
# Tune these for your motor. Start low and increase until response feels crisp.
VELOCITY_KP  = 0.1   # proportional gain
VELOCITY_KI  = 0.01  # integral gain
TORQUE_LIMIT = 1.0   # Nm

LOOP_HZ      = 200.0


def main():
    ctrl = XboxController()

    args = recoil.util.get_args()
    bus  = recoil.Bus(channel=args.channel, bitrate=1_000_000)
    device_id = args.id

    rate = RateLimiter(frequency=LOOP_HZ)

    bus.write_velocity_kp(device_id, VELOCITY_KP)
    bus.write_velocity_ki(device_id, VELOCITY_KI)
    bus.write_torque_limit(device_id, TORQUE_LIMIT)
    bus.set_mode(device_id, recoil.Mode.VELOCITY)
    bus.feed(device_id)

    print("+" * 60)
    print("Single Motor Velocity Control Mode Test")
    print(f"CAN Channel: {args.channel}")
    print(f"Device ID: {device_id}")
    print(f"Torque Limit: {TORQUE_LIMIT} Nm")
    print("=" * 60)
    
    print("Controls:")
    print("  Right stick Y          ->  proportional speed  (up = +, down = -)")
    print("  Right trigger          ->  spin forward")
    print("  Left  trigger          ->  spin backward")
    print("  Both triggers together ->  they oppose each other (net = RT - LT)")
    print("  All at zero            ->  motor holds at 0 rad/s")
    print("  Ctrl-C                 ->  safe shutdown")
    print()

    

    try:
        while True:
            state = ctrl.read()

            # Combine stick and triggers — clamp to [-1, 1] before scaling
            combined = state.right_y + (state.right_trigger - state.left_trigger)
            combined = max(-1.0, min(1.0, combined))

            velocity_target = combined * MAX_VELOCITY

            bus.write_velocity_target(device_id, velocity_target)
            bus.feed(device_id)

            measured_vel = bus.read_velocity_measured(device_id)
            if measured_vel is not None:
                print(
                    f"Target vel: {velocity_target:+.3f} rad/s"
                    f"   |   Measured vel: {measured_vel:+.3f} rad/s",
                    end="\r"
                )

            rate.sleep()

    except KeyboardInterrupt:
        print("\nShutting down...")

    finally:
        bus.set_mode(device_id, recoil.Mode.IDLE)
        bus.stop()
        ctrl.close()
        print("Done.")


if __name__ == "__main__":
    main()
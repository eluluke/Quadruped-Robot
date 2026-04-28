from loop_rate_limiters import RateLimiter
import recoil as recoil

from helper.xbox_controller import XboxController

MAX_POSITION = 2.0 # rad - right stick Y full deflection
MAX_VELOCITY = 3.0 # rad/s - riggers full press


KP = 0.2
KD = 0.005
TORQUE_LIMIT = 0.2  # Nm

LOOP_HZ = 200.0

def main():
    ctrl = XboxController()

    args = recoil.util.get_args()
    bus = recoil.Bus(channel=args.channel, bitrate=1_000_000)
    device_id = args.id

    bus.write_position_kp(device_id, KP)
    bus.write_position_kd(device_id, KD)
    bus.write_torque_limit(device_id, TORQUE_LIMIT)
    bus.set_mode(device_id, recoil.Mode.POSITION)
    bus.feed(device_id)

    print("Right stick Y -> position | RT -> +velocity | LT -> -velocity | Ctrl-C to quit")

    rate = RateLimiter(frequency=LOOP_HZ)

    try:
        while True:
            state = ctrl.read()

            position_target = state.right_y * MAX_POSITION
            velocity_target = (state.right_trigger - state.left_trigger) * MAX_VELOCITY

            measured_pos, measured_vel = bus.write_read_pdo_2(device_id, position_target, velocity_target)

            if measured_pos is not None:
                print(f"Target Pos: {position_target:+.3f}  vel: {velocity_target:+.3f} | Meas Pos: {measured_pos:+.3f}  vel: {measured_vel:+.3f}", end="\r")

            rate.sleep()
        
    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        bus.set_mode(device_id, recoil.Mode.IDLE)
        bus.stop()
        ctrl.close()
        print("Done.")

if __name__ == "__main__":
    main()

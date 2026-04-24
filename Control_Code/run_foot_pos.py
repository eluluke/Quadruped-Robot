import time
import math

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil
from quadruped_leg_ik import leg_ik
from homing_offsets import HOMING_OFFSET


args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

# ============================================================
# Correct motor IDs
# ============================================================
SHANK_ID = 1
THIGH_ID = 0
HIP_ID = 2

MOTOR_IDS = [THIGH_ID, SHANK_ID, HIP_ID]

MOTOR_NAMES = {
    SHANK_ID: "shank",
    THIGH_ID: "thigh",
    HIP_ID: "hip",
}

# ============================================================
# Gear / direction
# ============================================================
GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0

# ============================================================
# Neutral standing pose in IK frame
# ============================================================
NEUTRAL_X = 0.0
NEUTRAL_Y = 84.26
NEUTRAL_Z = 382.0

MOVE_TO_NEUTRAL_TIME = 5.0

# ============================================================
# Gains
# ============================================================
STARTUP_KP = 0.003
STARTUP_KD = 0.001
STARTUP_TORQUE_LIMIT = 0.03

MID_KP = 0.025
MID_KD = 0.002
MID_TORQUE_LIMIT = 0.12

RUN_KP = 0.060
RUN_KD = 0.002
RUN_TORQUE_LIMIT = 0.28

NEUTRAL_KP = 0.035
NEUTRAL_KD = 0.002

NEUTRAL_TORQUE_LIMIT_BY_JOINT = {
    THIGH_ID: 0.38,
    SHANK_ID: 0.38,
    HIP_ID: 0.75,
}

# ============================================================
# Timing
# ============================================================
RATE_HZ = 80.0
rate = RateLimiter(frequency=RATE_HZ)

MOVE_TIME = 3.0
PRINT_EVERY = 20


# ============================================================
# Low-level helpers
# ============================================================
def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.006)
    bus.feed(motor_id)
    time.sleep(0.006)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)

    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)

    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_gains_all(kp, kd, torque_limit):
    for motor_id in MOTOR_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def set_neutral_gains():
    for motor_id in MOTOR_IDS:
        set_gains(
            motor_id,
            NEUTRAL_KP,
            NEUTRAL_KD,
            NEUTRAL_TORQUE_LIMIT_BY_JOINT[motor_id],
        )


def sync_reference(motor_id, sync_time=0.35):
    current_pos = None
    steps = int(sync_time * RATE_HZ)

    for _ in range(steps):
        pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(motor_id, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync {MOTOR_NAMES[motor_id]}")

    return current_pos


def sync_all_references():
    for motor_id in MOTOR_IDS:
        print(f"Syncing {MOTOR_NAMES[motor_id]}...")
        synced = sync_reference(motor_id)
        print(f"  {MOTOR_NAMES[motor_id]} synced raw = {synced:.6f}")


def read_raw_position(motor_id):
    pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

    if pos is None:
        raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")

    return pos


def read_all_raw_positions():
    return {
        motor_id: read_raw_position(motor_id)
        for motor_id in MOTOR_IDS
    }


def idle_all_motors():
    print("Putting all motors into IDLE and stopping CAN bus...")

    for motor_id in MOTOR_IDS:
        try:
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
        except Exception:
            pass

    time.sleep(0.15)

    try:
        bus.stop()
    except Exception:
        pass


# ============================================================
# Angle mapping
# ============================================================
def raw_to_real(motor_id, raw_angle):
    return raw_angle + HOMING_OFFSET[motor_id]


def real_to_raw(motor_id, real_angle):
    return MOTOR_SIGN * (real_angle - HOMING_OFFSET[motor_id]) * GEAR_RATIO


# ============================================================
# Smooth move
# ============================================================
def move_all_to_targets(raw_targets, move_time):
    start_raw = read_all_raw_positions()
    steps = int(move_time * RATE_HZ)
    counter = 0

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        measured = {}

        for motor_id in MOTOR_IDS:
            cmd = start_raw[motor_id] + (
                raw_targets[motor_id] - start_raw[motor_id]
            ) * s

            pos, vel = bus.write_read_pdo_2(
                motor_id,
                cmd,
                0.0,
            )

            measured[motor_id] = (cmd, pos, vel)

        counter += 1

        if counter % PRINT_EVERY == 0:
            line = []

            for motor_id in MOTOR_IDS:
                cmd, pos, vel = measured[motor_id]

                if pos is not None:
                    line.append(
                        f"{MOTOR_NAMES[motor_id]} "
                        f"cmd={cmd:.3f} "
                        f"raw={pos:.3f}"
                    )

            print(" | ".join(line))

        rate.sleep()


def move_to_neutral_standing():
    print("\nMoving leg to neutral standing IK pose...")
    print(f"Neutral foot position: x={NEUTRAL_X}, y={NEUTRAL_Y}, z={NEUTRAL_Z}")

    theta_h, theta_t, theta_s = leg_ik(
        NEUTRAL_X,
        NEUTRAL_Y,
        NEUTRAL_Z,
    )

    desired_real = {
        HIP_ID: theta_h,
        THIGH_ID: theta_t,
        SHANK_ID: theta_s,
    }

    raw_targets = {}

    for motor_id in MOTOR_IDS:
        raw_targets[motor_id] = real_to_raw(
            motor_id,
            desired_real[motor_id],
        )

    print("\nNeutral IK angles:")
    print(f"  hip   = {theta_h:.6f}")
    print(f"  thigh = {theta_t:.6f}")
    print(f"  shank = {theta_s:.6f}")

    print("\nNeutral raw targets:")
    for motor_id in MOTOR_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]} = "
            f"{raw_targets[motor_id]:.6f}"
        )

    set_neutral_gains()

    for motor_id in MOTOR_IDS:
        set_mode_with_spacing(motor_id, recoil.Mode.POSITION)

    move_all_to_targets(
        raw_targets,
        MOVE_TO_NEUTRAL_TIME,
    )

    print("Neutral standing move complete.")


# ============================================================
# Parse xyz input
# ============================================================
def parse_xyz(text):
    parts = text.replace(",", " ").split()

    if len(parts) != 3:
        raise ValueError("Please enter x y z")

    return (
        float(parts[0]),
        float(parts[1]),
        float(parts[2]),
    )


# ============================================================
# Main
# ============================================================
try:
    print("Foot position IK test with neutral standing startup")
    print("Correct IDs: shank=1, thigh=0, hip=2")
    print("On exit, all motors go to IDLE, not DAMPING.")
    print()
    print("Loaded offsets:")

    for motor_id in MOTOR_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]}: "
            f"{HOMING_OFFSET[motor_id]:.6f}"
        )

    print("\nInput x y z in mm.")
    print("Type q to quit.\n")

    for motor_id in MOTOR_IDS:
        set_gains(
            motor_id,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )

        set_mode_with_spacing(
            motor_id,
            recoil.Mode.POSITION,
        )

    time.sleep(0.05)

    sync_all_references()

    current_raw = read_all_raw_positions()

    print("\nSoft holding current position...")
    for _ in range(int(0.5 * RATE_HZ)):
        for motor_id in MOTOR_IDS:
            bus.write_read_pdo_2(
                motor_id,
                current_raw[motor_id],
                0.0,
            )
        rate.sleep()

    move_to_neutral_standing()

    print("\nRamping to normal run gains...")
    set_gains_all(
        MID_KP,
        MID_KD,
        MID_TORQUE_LIMIT,
    )

    time.sleep(0.2)

    set_gains_all(
        RUN_KP,
        RUN_KD,
        RUN_TORQUE_LIMIT,
    )

    while True:
        text = input("\nEnter x y z (mm): ").strip()

        if text.lower() in ["q", "quit", "exit"]:
            break

        try:
            x, y, z = parse_xyz(text)

            theta_h, theta_t, theta_s = leg_ik(x, y, z)

            desired_real = {
                HIP_ID: theta_h,
                THIGH_ID: theta_t,
                SHANK_ID: theta_s,
            }

            raw_targets = {}

            for motor_id in MOTOR_IDS:
                raw_targets[motor_id] = real_to_raw(
                    motor_id,
                    desired_real[motor_id],
                )

            print("\nIK angles:")
            print(f"hip   = {theta_h:.6f}")
            print(f"thigh = {theta_t:.6f}")
            print(f"shank = {theta_s:.6f}")

            print("\nRaw motor targets:")
            for motor_id in MOTOR_IDS:
                print(
                    f"{MOTOR_NAMES[motor_id]} = "
                    f"{raw_targets[motor_id]:.6f}"
                )

            print("\nRaw deltas from current:")
            now_raw = read_all_raw_positions()
            for motor_id in MOTOR_IDS:
                delta = raw_targets[motor_id] - now_raw[motor_id]
                print(
                    f"{MOTOR_NAMES[motor_id]} delta = "
                    f"{delta:.6f}"
                )

            go = input("\nMove? y/n: ").strip().lower()

            if go != "y":
                continue

            move_all_to_targets(
                raw_targets,
                MOVE_TIME,
            )

            print("Move complete.")

        except Exception as exc:
            print(f"Error: {exc}")

except KeyboardInterrupt:
    print("\nInterrupted.")

finally:
    idle_all_motors()
# ==========================================
# Helpers
# ==========================================
def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.006)
    bus.feed(motor_id)
    time.sleep(0.006)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.004)
    bus.write_position_kd(motor_id, kd)
    time.sleep(0.004)
    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.004)


def set_gains_all(kp, kd, torque_limit):
    for motor_id in MOTOR_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def sync_reference(motor_id, sync_time=0.35):
    current_pos = None
    steps = int(sync_time * RATE_HZ)

    for _ in range(steps):
        pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(motor_id, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync {MOTOR_NAMES[motor_id]}")

    return current_pos


def sync_all_references():
    for motor_id in MOTOR_IDS:
        print(f"Syncing {MOTOR_NAMES[motor_id]}...")
        synced = sync_reference(motor_id)
        print(f"  {MOTOR_NAMES[motor_id]} synced raw = {synced:.6f}")


def read_all_raw_positions():
    values = {}

    for motor_id in MOTOR_IDS:
        pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

        if pos is None:
            raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")

        values[motor_id] = pos

    return values


def idle_all_motors():
    print("Putting all motors into IDLE and stopping CAN bus...")

    for motor_id in MOTOR_IDS:
        try:
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
        except Exception:
            pass

    time.sleep(0.15)

    try:
        bus.stop()
    except Exception:
        pass


# -----------------------------------------
# Angle mapping
# -----------------------------------------
def raw_to_real(motor_id, raw_angle):
    return raw_angle + HOMING_OFFSET[motor_id]


def real_to_raw(motor_id, real_angle):
    return MOTOR_SIGN * (real_angle - HOMING_OFFSET[motor_id]) * GEAR_RATIO


# -----------------------------------------
# Smooth move
# -----------------------------------------
def move_all_to_targets(raw_targets, move_time):
    start_raw = read_all_raw_positions()

    steps = int(move_time * RATE_HZ)
    counter = 0

    for i in range(steps):
        u = (i + 1) / steps
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        measured = {}

        for motor_id in MOTOR_IDS:
            cmd = start_raw[motor_id] + (
                raw_targets[motor_id] - start_raw[motor_id]
            ) * s

            pos, vel = bus.write_read_pdo_2(
                motor_id,
                cmd,
                0.0,
            )

            measured[motor_id] = (cmd, pos, vel)

        counter += 1

        if counter % PRINT_EVERY == 0:
            line = []

            for motor_id in MOTOR_IDS:
                cmd, pos, vel = measured[motor_id]

                if pos is not None:
                    line.append(
                        f"{MOTOR_NAMES[motor_id]} "
                        f"cmd={cmd:.3f} "
                        f"raw={pos:.3f}"
                    )

            print(" | ".join(line))

        rate.sleep()


# -----------------------------------------
# Parse xyz input
# -----------------------------------------
def parse_xyz(text):
    parts = text.replace(",", " ").split()

    if len(parts) != 3:
        raise ValueError("Please enter x y z")

    return (
        float(parts[0]),
        float(parts[1]),
        float(parts[2]),
    )


# ==========================================
# Main
# ==========================================
try:
    print("Fixed leg IK foot-position test")
    print("Correct IDs: shank=1, thigh=0, hip=2")
    print("On exit, all motors go to IDLE, not DAMPING.")
    print()
    print("Loaded offsets:")

    for motor_id in MOTOR_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]}: "
            f"{HOMING_OFFSET[motor_id]:.6f}"
        )

    print("\nInput x y z in mm.")
    print("Type q to quit.\n")

    for motor_id in MOTOR_IDS:
        set_gains(
            motor_id,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )

        set_mode_with_spacing(
            motor_id,
            recoil.Mode.POSITION,
        )

    time.sleep(0.05)

    sync_all_references()

    current_raw = read_all_raw_positions()

    print("\nSoft holding current position...")
    for _ in range(int(0.5 * RATE_HZ)):
        for motor_id in MOTOR_IDS:
            bus.write_read_pdo_2(
                motor_id,
                current_raw[motor_id],
                0.0,
            )
        rate.sleep()

    print("Ramping gains...")
    set_gains_all(
        MID_KP,
        MID_KD,
        MID_TORQUE_LIMIT,
    )

    for _ in range(int(0.3 * RATE_HZ)):
        for motor_id in MOTOR_IDS:
            bus.write_read_pdo_2(
                motor_id,
                current_raw[motor_id],
                0.0,
            )
        rate.sleep()

    set_gains_all(
        RUN_KP,
        RUN_KD,
        RUN_TORQUE_LIMIT,
    )

    while True:
        text = input("\nEnter x y z (mm): ").strip()

        if text.lower() in ["q", "quit", "exit"]:
            break

        try:
            x, y, z = parse_xyz(text)

            theta_h, theta_t, theta_s = leg_ik(x, y, z)

            desired_real = {
                HIP_ID: theta_h,
                THIGH_ID: theta_t,
                SHANK_ID: theta_s,
            }

            raw_targets = {}

            for motor_id in MOTOR_IDS:
                raw_targets[motor_id] = real_to_raw(
                    motor_id,
                    desired_real[motor_id],
                )

            print("\nIK angles:")
            print(f"hip   = {theta_h:.6f}")
            print(f"thigh = {theta_t:.6f}")
            print(f"shank = {theta_s:.6f}")

            print("\nRaw motor targets:")
            for motor_id in MOTOR_IDS:
                print(
                    f"{MOTOR_NAMES[motor_id]} = "
                    f"{raw_targets[motor_id]:.6f}"
                )

            print("\nRaw deltas from current:")
            now_raw = read_all_raw_positions()
            for motor_id in MOTOR_IDS:
                delta = raw_targets[motor_id] - now_raw[motor_id]
                print(
                    f"{MOTOR_NAMES[motor_id]} delta = "
                    f"{delta:.6f}"
                )

            go = input("\nMove? y/n: ").strip().lower()

            if go != "y":
                continue

            move_all_to_targets(
                raw_targets,
                MOVE_TIME,
            )

            print("Move complete.")

        except Exception as exc:
            print(f"Error: {exc}")

except KeyboardInterrupt:
    print("\nInterrupted.")

finally:
    idle_all_motors()


# ==========================================
# Helpers
# ==========================================
def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.003)
    bus.feed(motor_id)
    time.sleep(0.003)


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.002)

    bus.write_position_kd(motor_id, kd)
    time.sleep(0.002)

    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.002)


def set_gains_all(kp, kd, torque_limit):
    for motor_id in MOTOR_IDS:
        set_gains(motor_id, kp, kd, torque_limit)


def sync_reference(motor_id, sync_time=0.3):
    current_pos = None
    steps = int(sync_time * RATE_HZ)

    for _ in range(steps):
        pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

        if pos is not None:
            current_pos = pos

        if current_pos is not None:
            bus.write_read_pdo_2(motor_id, current_pos, 0.0)

        rate.sleep()

    if current_pos is None:
        raise RuntimeError(f"Failed to sync {MOTOR_NAMES[motor_id]}")

    return current_pos


def sync_all_references():
    for motor_id in MOTOR_IDS:
        print(f"Syncing {MOTOR_NAMES[motor_id]}...")
        synced = sync_reference(motor_id)
        print(f"  {MOTOR_NAMES[motor_id]} synced raw = {synced:.6f}")


def read_all_raw_positions():
    values = {}

    for motor_id in MOTOR_IDS:
        pos, _ = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

        if pos is None:
            raise RuntimeError(f"Cannot read {MOTOR_NAMES[motor_id]}")

        values[motor_id] = pos

    return values


# -----------------------------------------
# Angle mapping
# -----------------------------------------
def raw_to_real(motor_id, raw_angle):
    # Keep this simple:
    # real IK-frame angle = raw encoder angle + homing offset
    # No gear ratio here.
    return raw_angle + HOMING_OFFSET[motor_id]


def real_to_raw(motor_id, real_angle):
    # Gear ratio is applied ONLY when outputting motor command.
    # Direction is flipped after reduction, so MOTOR_SIGN = -1.
    return (
        MOTOR_SIGN
        * (real_angle - HOMING_OFFSET[motor_id])
        * GEAR_RATIO
    )


# -----------------------------------------
# Smooth move
# -----------------------------------------
def move_all_to_targets(raw_targets, move_time):
    start_raw = read_all_raw_positions()

    steps = int(move_time * RATE_HZ)
    counter = 0

    for i in range(steps):
        u = (i + 1) / steps

        # cosine smoothstep
        s = 0.5 * (1.0 - math.cos(math.pi * u))

        measured = {}

        for motor_id in MOTOR_IDS:
            cmd = start_raw[motor_id] + (
                raw_targets[motor_id] - start_raw[motor_id]
            ) * s

            pos, vel = bus.write_read_pdo_2(
                motor_id,
                cmd,
                0.0,
            )

            measured[motor_id] = (cmd, pos, vel)

        counter += 1

        if counter % PRINT_EVERY == 0:
            line = []

            for motor_id in MOTOR_IDS:
                cmd, pos, vel = measured[motor_id]

                if pos is not None:
                    real_pos = raw_to_real(motor_id, pos)

                    line.append(
                        f"{MOTOR_NAMES[motor_id]} "
                        f"cmd={cmd:.3f} "
                        f"raw={pos:.3f} "
                        f"real={real_pos:.3f}"
                    )

            print(" | ".join(line))

        rate.sleep()


# -----------------------------------------
# Parse xyz input
# -----------------------------------------
def parse_xyz(text):
    parts = text.replace(",", " ").split()

    if len(parts) != 3:
        raise ValueError("Please enter x y z")

    return (
        float(parts[0]),
        float(parts[1]),
        float(parts[2]),
    )


# ==========================================
# Main
# ==========================================
try:
    print("Leg IK test using imported HOMING_OFFSET")
    print("Gear ratio is applied only when sending motor command.")
    print("Loaded offsets:")

    for motor_id in MOTOR_IDS:
        print(
            f"  {MOTOR_NAMES[motor_id]}: "
            f"{HOMING_OFFSET[motor_id]:.6f}"
        )

    print("\nInput x y z in mm.")
    print("Type q to quit.\n")

    # --------------------------------
    # Startup soft mode
    # --------------------------------
    for motor_id in MOTOR_IDS:
        set_gains(
            motor_id,
            STARTUP_KP,
            STARTUP_KD,
            STARTUP_TORQUE_LIMIT,
        )

        set_mode_with_spacing(
            motor_id,
            recoil.Mode.POSITION,
        )

    time.sleep(0.05)

    sync_all_references()

    # hold current place
    current_raw = read_all_raw_positions()

    for _ in range(int(0.5 * RATE_HZ)):
        for motor_id in MOTOR_IDS:
            bus.write_read_pdo_2(
                motor_id,
                current_raw[motor_id],
                0.0,
            )
        rate.sleep()

    # medium gains
    set_gains_all(
        MID_KP,
        MID_KD,
        MID_TORQUE_LIMIT,
    )

    for _ in range(int(0.3 * RATE_HZ)):
        for motor_id in MOTOR_IDS:
            bus.write_read_pdo_2(
                motor_id,
                current_raw[motor_id],
                0.0,
            )
        rate.sleep()

    # final gains
    set_gains_all(
        RUN_KP,
        RUN_KD,
        RUN_TORQUE_LIMIT,
    )

    # --------------------------------
    # User loop
    # --------------------------------
    while True:
        text = input("\nEnter x y z (mm): ").strip()

        if text.lower() in ["q", "quit", "exit"]:
            break

        try:
            x, y, z = parse_xyz(text)

            theta_h, theta_t, theta_s = leg_ik(
                x,
                y,
                z,
            )

            desired_real = {
                HIP_ID: theta_h,
                THIGH_ID: theta_t,
                SHANK_ID: theta_s,
            }

            raw_targets = {}

            for motor_id in MOTOR_IDS:
                raw_targets[motor_id] = real_to_raw(
                    motor_id,
                    desired_real[motor_id],
                )

            print("\nIK angles:")
            print(f"hip   = {theta_h:.6f}")
            print(f"thigh = {theta_t:.6f}")
            print(f"shank = {theta_s:.6f}")

            print("\nRaw motor targets after gear ratio:")
            for motor_id in MOTOR_IDS:
                print(
                    f"{MOTOR_NAMES[motor_id]} = "
                    f"{raw_targets[motor_id]:.6f}"
                )

            go = input("\nMove? y/n: ").strip().lower()

            if go != "y":
                continue

            move_all_to_targets(
                raw_targets,
                MOVE_TIME,
            )

            print("Move complete.")

        except Exception as exc:
            print(f"Error: {exc}")

except KeyboardInterrupt:
    print("\nInterrupted.")

finally:
    print("Setting damping...")

    try:
        for motor_id in MOTOR_IDS:
            try:
                set_mode_with_spacing(
                    motor_id,
                    recoil.Mode.DAMPING,
                )
            except Exception:
                pass

        time.sleep(0.05)

    finally:
        try:
            bus.stop()
        except Exception:
            pass

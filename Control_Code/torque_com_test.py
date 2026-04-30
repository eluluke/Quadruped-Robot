import time
import math
import csv
from datetime import datetime

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ============================================================
# Motor IDs
# ============================================================
THIGH_ID = 0
HIP_ID = 2

ALL_IDS = [HIP_ID, THIGH_ID]

MOTOR_NAMES = {
    HIP_ID: "hip",
    THIGH_ID: "thigh",
}


# ============================================================
# Control rate / logging
# ============================================================
RATE_HZ = 80.0
PRINT_EVERY = 20
RUN_TIME_SECONDS = 60.0


# ============================================================
# Gear / sign
# ============================================================
GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0


# ============================================================
# Hip biased position hold tuning
# ============================================================
HIP_KP = 0.008
HIP_KD = 0.010
HIP_TORQUE_LIMIT = 1.50

# Output-side bias in radians.
HIP_OUTPUT_BIAS_RAD = 0.080

# Flip this if bias pulls the wrong way.
HIP_BIAS_SIGN = 1.0


# ============================================================
# Thigh sine tuning
# ============================================================
THIGH_KP = 0.035
THIGH_KD = 0.004
THIGH_TORQUE_LIMIT = 0.22

# Output-side sine amplitude in radians.
THIGH_SINE_AMPLITUDE_RAD = 0.080

# Larger = slower.
THIGH_SINE_PERIOD = 5.0


# ============================================================
# Startup
# ============================================================
STARTUP_HOLD_TIME = 1.2


# ============================================================
# Setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)


# ============================================================
# Helpers
# ============================================================
def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def set_gains(motor_id, kp, kd, torque_limit):
    bus.write_position_kp(motor_id, kp)
    time.sleep(0.006)

    bus.write_position_kd(motor_id, kd)
    time.sleep(0.006)

    bus.write_torque_limit(motor_id, torque_limit)
    time.sleep(0.006)


def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.02)

    try:
        bus.feed(motor_id)
    except Exception:
        pass

    time.sleep(0.02)


def write_position_command(motor_id, command_pos):
    return bus.write_read_pdo_2(
        motor_id,
        command_pos,
        0.0,
    )


def read_position_while_idle(motor_id):
    """
    Only use while motor is IDLE.

    write_read_pdo_2 is not a pure read, but while IDLE,
    the 0.0 command should not move the joint.
    """
    pos, vel = bus.write_read_pdo_2(
        motor_id,
        0.0,
        0.0,
    )

    if pos is None:
        raise RuntimeError(f"Could not read {MOTOR_NAMES[motor_id]} position")

    return pos


def read_initial_positions():
    print("Putting hip and thigh into IDLE before reading initial positions...")

    for motor_id in ALL_IDS:
        bus.set_mode(motor_id, recoil.Mode.IDLE)
        time.sleep(0.03)

    time.sleep(0.25)

    # Flush stale frames.
    for _ in range(5):
        for motor_id in ALL_IDS:
            try:
                bus.write_read_pdo_2(motor_id, 0.0, 0.0)
            except Exception:
                pass
        rate.sleep()

    raw = {}

    for motor_id in ALL_IDS:
        samples = []

        for _ in range(20):
            pos = read_position_while_idle(motor_id)
            samples.append(pos)
            rate.sleep()

        samples.sort()
        raw[motor_id] = samples[len(samples) // 2]

    print("\nInitial raw motor positions:")
    for motor_id in ALL_IDS:
        print(f"  {MOTOR_NAMES[motor_id]} = {raw[motor_id]:.6f}")

    return raw


def output_delta_to_raw(delta_output_rad):
    """
    Convert output-side joint delta in radians to motor raw delta.
    """
    return MOTOR_SIGN * delta_output_rad * GEAR_RATIO


def raw_delta_to_output(delta_raw):
    """
    Convert motor raw delta to approximate output-side joint delta.
    """
    return delta_raw / (MOTOR_SIGN * GEAR_RATIO)


def compute_hip_biased_target(hip_start_raw):
    hip_raw_bias = (
        HIP_BIAS_SIGN
        * MOTOR_SIGN
        * HIP_OUTPUT_BIAS_RAD
        * GEAR_RATIO
    )

    hip_target = hip_start_raw + hip_raw_bias

    print("\nHip bias setup:")
    print(f"  hip_start_raw        = {hip_start_raw:.6f}")
    print(f"  HIP_OUTPUT_BIAS_RAD  = {HIP_OUTPUT_BIAS_RAD:.6f}")
    print(f"  HIP_BIAS_SIGN        = {HIP_BIAS_SIGN}")
    print(f"  MOTOR_SIGN           = {MOTOR_SIGN}")
    print(f"  GEAR_RATIO           = {GEAR_RATIO}")
    print(f"  hip_raw_bias         = {hip_raw_bias:.6f}")
    print(f"  hip_target           = {hip_target:.6f}")

    return hip_target, hip_raw_bias


def setup_position_modes():
    print("\nSetting gains...")

    set_gains(
        HIP_ID,
        HIP_KP,
        HIP_KD,
        HIP_TORQUE_LIMIT,
    )

    set_gains(
        THIGH_ID,
        THIGH_KP,
        THIGH_KD,
        THIGH_TORQUE_LIMIT,
    )

    print("Entering POSITION mode for hip and thigh...")

    for motor_id in ALL_IDS:
        set_mode_with_spacing(
            motor_id,
            recoil.Mode.POSITION,
        )


def startup_hold(hip_target, thigh_start_raw):
    print("\nStartup hold:")
    print("  hip holds biased target")
    print("  thigh holds start")
    print("  shank is not commanded")

    for _ in range(int(STARTUP_HOLD_TIME * RATE_HZ)):
        write_position_command(
            HIP_ID,
            hip_target,
        )

        write_position_command(
            THIGH_ID,
            thigh_start_raw,
        )

        rate.sleep()

    print("Startup hold complete.")


def idle_all():
    print("\nPutting hip and thigh into IDLE and stopping bus...")

    for motor_id in ALL_IDS:
        try:
            bus.set_mode(motor_id, recoil.Mode.IDLE)
            time.sleep(0.03)
        except Exception:
            pass

    time.sleep(0.15)

    try:
        bus.stop()
    except Exception:
        pass

    print("Done.")


def make_log_file():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"hip_thigh_coupling_log_{stamp}.csv"

    fieldnames = [
        "time_s",

        "hip_target_raw",
        "hip_pos_raw",
        "hip_error_raw",
        "hip_error_wrap_raw",
        "hip_error_output_rad",

        "thigh_target_raw",
        "thigh_pos_raw",
        "thigh_error_raw",
        "thigh_error_wrap_raw",

        "thigh_target_delta_raw",
        "thigh_pos_delta_raw",
        "thigh_target_delta_output_rad",
        "thigh_pos_delta_output_rad",
        "thigh_error_output_rad",

        "thigh_sine_command_output_rad",

        "hip_output_bias_rad",
        "hip_bias_sign",
        "thigh_sine_amplitude_rad",
        "thigh_sine_period_s",
        "gear_ratio",
        "motor_sign",
    ]

    f = open(filename, "w", newline="")
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()

    print(f"\nLogging data to: {filename}")

    return filename, f, writer


# ============================================================
# Main
# ============================================================
log_file = None
log_filename = None

try:
    print("=" * 80)
    print("Hip-Thigh Coupling Data Collection, No Shank")
    print("=" * 80)
    print("Purpose:")
    print("  Collect data for mapping thigh target angle to hip position error.")
    print()
    print("Behavior:")
    print("  Hip   = biased position hold")
    print("  Thigh = sinusoidal position motion")
    print("  Shank = not commanded")
    print()
    print("Tuning:")
    print(f"  HIP_OUTPUT_BIAS_RAD       = {HIP_OUTPUT_BIAS_RAD}")
    print(f"  HIP_BIAS_SIGN             = {HIP_BIAS_SIGN}")
    print(f"  THIGH_SINE_AMPLITUDE_RAD  = {THIGH_SINE_AMPLITUDE_RAD}")
    print(f"  THIGH_SINE_PERIOD         = {THIGH_SINE_PERIOD} s")
    print(f"  RUN_TIME_SECONDS          = {RUN_TIME_SECONDS}")
    print("=" * 80)

    # Step 1: Read initial raw positions.
    initial_raw = read_initial_positions()

    hip_start_raw = initial_raw[HIP_ID]
    thigh_start_raw = initial_raw[THIGH_ID]

    # Step 2: Compute hip biased target.
    hip_target, hip_raw_bias = compute_hip_biased_target(
        hip_start_raw,
    )

    # Step 3: Enter position modes.
    setup_position_modes()

    # Step 4: Startup hold.
    startup_hold(
        hip_target,
        thigh_start_raw,
    )

    # Step 5: Open CSV log.
    log_filename, log_file, log_writer = make_log_file()

    print("\nStarting thigh sine data collection.")
    print("Press Ctrl+C to stop.\n")

    start_time = time.time()
    counter = 0

    while True:
        t = time.time() - start_time

        if RUN_TIME_SECONDS is not None and t >= RUN_TIME_SECONDS:
            print("\nRun time complete.")
            break

        # ----------------------------------------------------
        # Thigh sine command in output-side radians.
        # ----------------------------------------------------
        sine_phase = 2.0 * math.pi * t / THIGH_SINE_PERIOD

        thigh_sine_output = THIGH_SINE_AMPLITUDE_RAD * math.sin(
            sine_phase
        )

        thigh_target_raw = thigh_start_raw + output_delta_to_raw(
            thigh_sine_output
        )

        # ----------------------------------------------------
        # Command hip and thigh.
        # ----------------------------------------------------
        hip_pos, hip_vel = write_position_command(
            HIP_ID,
            hip_target,
        )

        thigh_pos, thigh_vel = write_position_command(
            THIGH_ID,
            thigh_target_raw,
        )

        # ----------------------------------------------------
        # Compute errors.
        # ----------------------------------------------------
        hip_error_raw = hip_target - hip_pos
        hip_error_wrap_raw = wrap_pi(hip_error_raw)
        hip_error_output_rad = raw_delta_to_output(hip_error_raw)

        thigh_error_raw = thigh_target_raw - thigh_pos
        thigh_error_wrap_raw = wrap_pi(thigh_error_raw)
        thigh_error_output_rad = raw_delta_to_output(thigh_error_raw)

        thigh_target_delta_raw = thigh_target_raw - thigh_start_raw
        thigh_pos_delta_raw = thigh_pos - thigh_start_raw

        thigh_target_delta_output_rad = raw_delta_to_output(
            thigh_target_delta_raw
        )

        thigh_pos_delta_output_rad = raw_delta_to_output(
            thigh_pos_delta_raw
        )

        # ----------------------------------------------------
        # Write CSV row.
        # ----------------------------------------------------
        log_writer.writerow(
            {
                "time_s": t,

                "hip_target_raw": hip_target,
                "hip_pos_raw": hip_pos,
                "hip_error_raw": hip_error_raw,
                "hip_error_wrap_raw": hip_error_wrap_raw,
                "hip_error_output_rad": hip_error_output_rad,

                "thigh_target_raw": thigh_target_raw,
                "thigh_pos_raw": thigh_pos,
                "thigh_error_raw": thigh_error_raw,
                "thigh_error_wrap_raw": thigh_error_wrap_raw,

                "thigh_target_delta_raw": thigh_target_delta_raw,
                "thigh_pos_delta_raw": thigh_pos_delta_raw,
                "thigh_target_delta_output_rad": thigh_target_delta_output_rad,
                "thigh_pos_delta_output_rad": thigh_pos_delta_output_rad,
                "thigh_error_output_rad": thigh_error_output_rad,

                "thigh_sine_command_output_rad": thigh_sine_output,

                "hip_output_bias_rad": HIP_OUTPUT_BIAS_RAD,
                "hip_bias_sign": HIP_BIAS_SIGN,
                "thigh_sine_amplitude_rad": THIGH_SINE_AMPLITUDE_RAD,
                "thigh_sine_period_s": THIGH_SINE_PERIOD,
                "gear_ratio": GEAR_RATIO,
                "motor_sign": MOTOR_SIGN,
            }
        )

        counter += 1

        if counter % PRINT_EVERY == 0:
            print(
                f"t={t:6.2f}s | "
                f"thigh_cmd_out={thigh_sine_output:+.4f} rad | "
                f"thigh_actual_out={thigh_pos_delta_output_rad:+.4f} rad | "
                f"thigh_err_out={thigh_error_output_rad:+.4f} rad | "
                f"hip_err_out={hip_error_output_rad:+.4f} rad | "
                f"hip_err_raw={hip_error_raw:+.3f}"
            )

        rate.sleep()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    if log_file is not None:
        try:
            log_file.flush()
            log_file.close()
            print(f"\nSaved log file: {log_filename}")
        except Exception:
            pass

    idle_all()

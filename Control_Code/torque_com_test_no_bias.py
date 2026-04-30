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
# Hip pure position hold tuning
# ============================================================
# IMPORTANT:
# This data collection script intentionally uses NO hip compensation
# and NO hip bias. The hip target is exactly the measured startup
# hip position.
#
# Later, after collecting CSV data, we can fit:
#   hip_error = f(thigh_target_delta)
# and create a compensation helper from that relationship.
# ============================================================
HIP_KP = 0.008
HIP_KD = 0.010
HIP_TORQUE_LIMIT = 1.50


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
    pos, vel = bus.write_read_pdo_2(
        motor_id,
        command_pos,
        0.0,
    )

    if pos is None:
        raise RuntimeError(f"Could not read {MOTOR_NAMES[motor_id]} position")

    return pos, vel


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


def startup_hold(hip_hold_raw, thigh_start_raw):
    print("\nStartup hold:")
    print("  hip holds its measured startup position, with NO bias and NO compensation")
    print("  thigh holds its measured startup position")
    print("  shank is not commanded")

    for _ in range(int(STARTUP_HOLD_TIME * RATE_HZ)):
        write_position_command(
            HIP_ID,
            hip_hold_raw,
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
    filename = f"hip_thigh_uncompensated_coupling_log_{stamp}.csv"

    fieldnames = [
        "time_s",

        # Hip: pure hold, no bias.
        "hip_hold_raw",
        "hip_pos_raw",
        "hip_error_raw",
        "hip_error_wrap_raw",
        "hip_error_output_rad",
        "hip_vel_raw",

        # Thigh raw position data.
        "thigh_target_raw",
        "thigh_pos_raw",
        "thigh_error_raw",
        "thigh_error_wrap_raw",
        "thigh_vel_raw",

        # Thigh relative-to-start data.
        "thigh_target_delta_raw",
        "thigh_pos_delta_raw",
        "thigh_target_delta_output_rad",
        "thigh_pos_delta_output_rad",
        "thigh_error_output_rad",

        # Sine command reference.
        "thigh_sine_command_output_rad",
        "thigh_sine_phase_rad",

        # Experiment metadata.
        "hip_compensation_enabled",
        "hip_bias_enabled",
        "hip_bias_output_rad",
        "thigh_sine_amplitude_rad",
        "thigh_sine_period_s",
        "gear_ratio",
        "motor_sign",
        "rate_hz",
        "hip_kp",
        "hip_kd",
        "hip_torque_limit",
        "thigh_kp",
        "thigh_kd",
        "thigh_torque_limit",
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
    print("Hip-Thigh Coupling Data Collection, NO Hip Bias, NO Compensation")
    print("=" * 80)
    print("Purpose:")
    print("  Collect uncompensated data for mapping thigh target angle to hip position error.")
    print()
    print("Behavior:")
    print("  Hip   = holds measured startup position only")
    print("  Thigh = sinusoidal position motion around measured startup position")
    print("  Shank = not commanded")
    print()
    print("Important:")
    print("  This script intentionally does NOT add hip bias or compensation.")
    print("  Use this before fitting the compensation map.")
    print()
    print("Tuning:")
    print(f"  HIP_KP                    = {HIP_KP}")
    print(f"  HIP_KD                    = {HIP_KD}")
    print(f"  HIP_TORQUE_LIMIT          = {HIP_TORQUE_LIMIT}")
    print(f"  THIGH_SINE_AMPLITUDE_RAD  = {THIGH_SINE_AMPLITUDE_RAD}")
    print(f"  THIGH_SINE_PERIOD         = {THIGH_SINE_PERIOD} s")
    print(f"  RUN_TIME_SECONDS          = {RUN_TIME_SECONDS}")
    print("=" * 80)

    # Step 1: Read initial raw positions.
    initial_raw = read_initial_positions()

    hip_start_raw = initial_raw[HIP_ID]
    thigh_start_raw = initial_raw[THIGH_ID]

    # Step 2: Hip target is exactly startup position.
    # No bias. No compensation. This is the baseline data collection stage.
    hip_hold_raw = hip_start_raw

    print("\nHip hold setup:")
    print(f"  hip_start_raw = {hip_start_raw:.6f}")
    print(f"  hip_hold_raw  = {hip_hold_raw:.6f}")
    print("  hip bias      = DISABLED")
    print("  compensation  = DISABLED")

    # Step 3: Enter position modes.
    setup_position_modes()

    # Step 4: Startup hold.
    startup_hold(
        hip_hold_raw,
        thigh_start_raw,
    )

    # Step 5: Open CSV log.
    log_filename, log_file, log_writer = make_log_file()

    print("\nStarting uncompensated thigh sine data collection.")
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
        thigh_sine_phase = 2.0 * math.pi * t / THIGH_SINE_PERIOD

        thigh_sine_output = THIGH_SINE_AMPLITUDE_RAD * math.sin(
            thigh_sine_phase
        )

        thigh_target_raw = thigh_start_raw + output_delta_to_raw(
            thigh_sine_output
        )

        # ----------------------------------------------------
        # Command hip and thigh.
        # Hip command is constant baseline hold.
        # Thigh command is sinusoidal.
        # ----------------------------------------------------
        hip_pos, hip_vel = write_position_command(
            HIP_ID,
            hip_hold_raw,
        )

        thigh_pos, thigh_vel = write_position_command(
            THIGH_ID,
            thigh_target_raw,
        )

        # ----------------------------------------------------
        # Compute errors.
        # ----------------------------------------------------
        hip_error_raw = hip_hold_raw - hip_pos
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

                "hip_hold_raw": hip_hold_raw,
                "hip_pos_raw": hip_pos,
                "hip_error_raw": hip_error_raw,
                "hip_error_wrap_raw": hip_error_wrap_raw,
                "hip_error_output_rad": hip_error_output_rad,
                "hip_vel_raw": hip_vel,

                "thigh_target_raw": thigh_target_raw,
                "thigh_pos_raw": thigh_pos,
                "thigh_error_raw": thigh_error_raw,
                "thigh_error_wrap_raw": thigh_error_wrap_raw,
                "thigh_vel_raw": thigh_vel,

                "thigh_target_delta_raw": thigh_target_delta_raw,
                "thigh_pos_delta_raw": thigh_pos_delta_raw,
                "thigh_target_delta_output_rad": thigh_target_delta_output_rad,
                "thigh_pos_delta_output_rad": thigh_pos_delta_output_rad,
                "thigh_error_output_rad": thigh_error_output_rad,

                "thigh_sine_command_output_rad": thigh_sine_output,
                "thigh_sine_phase_rad": thigh_sine_phase,

                "hip_compensation_enabled": 0,
                "hip_bias_enabled": 0,
                "hip_bias_output_rad": 0.0,
                "thigh_sine_amplitude_rad": THIGH_SINE_AMPLITUDE_RAD,
                "thigh_sine_period_s": THIGH_SINE_PERIOD,
                "gear_ratio": GEAR_RATIO,
                "motor_sign": MOTOR_SIGN,
                "rate_hz": RATE_HZ,
                "hip_kp": HIP_KP,
                "hip_kd": HIP_KD,
                "hip_torque_limit": HIP_TORQUE_LIMIT,
                "thigh_kp": THIGH_KP,
                "thigh_kd": THIGH_KD,
                "thigh_torque_limit": THIGH_TORQUE_LIMIT,
            }
        )

        # Flush every few rows so a Ctrl+C or crash still leaves usable data.
        if counter % PRINT_EVERY == 0 and log_file is not None:
            log_file.flush()

        counter += 1

        if counter % PRINT_EVERY == 0:
            print(
                f"t={t:6.2f}s | "
                f"thigh_cmd_out={thigh_sine_output:+.4f} rad | "
                f"thigh_actual_out={thigh_pos_delta_output_rad:+.4f} rad | "
                f"thigh_err_out={thigh_error_output_rad:+.4f} rad | "
                f"hip_err_out={hip_error_output_rad:+.4f} rad | "
                f"hip_vel_raw={hip_vel:+.4f}"
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

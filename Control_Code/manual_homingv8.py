import time
import threading
from statistics import median

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ============================================================
# Manual homing offset calibration v8 - SDO/separate read version
# ============================================================
#
# Important difference from older versions:
#   This script does NOT use write_read_pdo_2() to read encoder position.
#   It uses bus.read_position_measured(device_id), which is the separate
#   parameter read function exposed by the Berkeley Humanoid Lite recoil API.
#
# Why:
#   write_read_pdo_2() transmits a PDO target and immediately receives PDO2.
#   That is convenient for trajectory control, but it can be fragile for
#   calibration/live-monitoring where we want a pure read without sending a
#   fake position command.
#
# This script does NOT recalibrate electrical offset.
# It only defines the joint/home offset from a mechanical reference pose.
#
# Procedure:
#   1. Run this script.
#   2. It puts all joints into IDLE.
#   3. It runs single-ID read diagnostics using read_position_measured().
#   4. It starts live encoder monitor.
#   5. Manually move/backdrive the leg to max-contraction pose.
#   6. Press Enter while holding the leg steady.
#   7. It reads final median positions.
#   8. It computes HOMING_OFFSET and writes homing_offsets.py after y/yes.
#
# Offset convention:
#   real_joint_angle = raw_motor_position / (MOTOR_SIGN * GEAR_RATIO) + HOMING_OFFSET[id]
#   HOMING_OFFSET[id] = known_homed_joint_angle - raw_homed_position / (MOTOR_SIGN * GEAR_RATIO)
#   raw_command = MOTOR_SIGN * (desired_joint_angle - HOMING_OFFSET[id]) * GEAR_RATIO
# ============================================================


# ============================================================
# Current assembled-leg IDs
# ============================================================
SHANK_ID = 0
THIGH_ID = 2
HIP_ID = 1

JOINT_IDS = [SHANK_ID, THIGH_ID, HIP_ID]

JOINT_NAMES = {
    SHANK_ID: "shank",
    THIGH_ID: "thigh",
    HIP_ID: "hip",
}

ROLE_TO_ID = {
    "shank": SHANK_ID,
    "thigh": THIGH_ID,
    "hip": HIP_ID,
}

READ_ORDER = [SHANK_ID, THIGH_ID, HIP_ID]


# ============================================================
# Gear / sign
# ============================================================
GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0


# ============================================================
# Known ideal joint angles at manual homing pose
# ============================================================
KNOWN_HOMED_JOINT_ANGLES_BY_ROLE = {
    "shank": 2.688,
    "thigh": 1.199,
    "hip": 1.069,
}


# ============================================================
# Output file
# ============================================================
OFFSET_OUTPUT_FILE = "homing_offsets.py"


# ============================================================
# Read / diagnostic settings
# ============================================================
RATE_HZ = 50.0
LIVE_PRINT_HZ = 8.0
SINGLE_ID_DIAG_SECONDS = 2.5
SINGLE_ID_DIAG_HZ = 8.0
SAMPLE_COUNT = 80
FINAL_SAMPLE_HZ = 50.0
MAX_SAMPLE_SPREAD_WARNING = 0.25
MAX_SAMPLE_SPREAD_HARD_REJECT = 1.00
SINGLE_ID_SPREAD_WARNING = 0.25
SINGLE_ID_SPREAD_HARD_REJECT = 1.00
REJECT_BAD_READS_BY_DEFAULT = True
LIVE_JUMP_WARNING_RAW = 0.75


# ============================================================
# Setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)


# ============================================================
# Helpers
# ============================================================
def set_mode_with_spacing(motor_id, mode):
    bus.set_mode(motor_id, mode)
    time.sleep(0.015)
    try:
        bus.feed(motor_id)
    except Exception:
        pass
    time.sleep(0.015)


def idle_all_motors():
    print("\nPutting all motors into IDLE...")
    for motor_id in JOINT_IDS:
        try:
            set_mode_with_spacing(motor_id, recoil.Mode.IDLE)
            print(f"  {JOINT_NAMES[motor_id]} IDLE")
        except Exception as exc:
            print(f"  Failed to idle {JOINT_NAMES[motor_id]}: {exc}")
    time.sleep(0.15)


def shutdown_bus():
    try:
        bus.stop()
    except Exception:
        pass


def raw_to_output_angle(raw_motor_position):
    return raw_motor_position / (MOTOR_SIGN * GEAR_RATIO)


def read_raw_once(motor_id):
    """
    Pure/separate measured-position read.

    This intentionally does NOT call write_read_pdo_2().
    It does not transmit a position target.
    """
    try:
        pos = bus.read_position_measured(motor_id)
    except AttributeError as exc:
        raise RuntimeError(
            "Your recoil.Bus does not expose read_position_measured(). "
            "Check your installed Berkeley Humanoid Lite lowlevel version."
        ) from exc

    if pos is None:
        raise RuntimeError(f"Could not read {JOINT_NAMES[motor_id]} ID {motor_id}")

    return pos


def read_raw_median(motor_id, sample_count=SAMPLE_COUNT, sample_hz=FINAL_SAMPLE_HZ):
    name = JOINT_NAMES[motor_id]
    local_rate = RateLimiter(frequency=sample_hz)
    samples = []

    for _ in range(sample_count):
        samples.append(read_raw_once(motor_id))
        local_rate.sleep()

    samples.sort()
    raw_median = median(samples)
    spread = max(samples) - min(samples)

    print(
        f"  {name:5s} raw median = {raw_median:+.9f} "
        f"out={raw_to_output_angle(raw_median):+.6f} "
        f"(spread={spread:.6f})"
    )

    if spread > MAX_SAMPLE_SPREAD_WARNING:
        print(
            f"    WARNING: {name} moved/noisy during reading. "
            "Hold the joint still and rerun if this is unexpected."
        )

    if spread > MAX_SAMPLE_SPREAD_HARD_REJECT:
        print(f"    BAD: {name} final spread is above hard reject threshold.")

    return raw_median, spread


def compute_offsets(raw_homed_by_id):
    offsets = {}
    for role, motor_id in ROLE_TO_ID.items():
        known_angle = KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]
        raw_homed = raw_homed_by_id[motor_id]
        offsets[motor_id] = known_angle - raw_to_output_angle(raw_homed)
    return offsets


def write_offsets_file(offsets, raw_homed_by_id, sample_spread_by_id, single_id_spread_by_id):
    with open(OFFSET_OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write('"""Auto-generated MANUAL homing offsets.\n\n')
        file.write("Generated by manual_homingv8_sdo_read_position.py.\n")
        file.write("The leg was manually placed at the max-contraction homing pose.\n")
        file.write("Encoder positions were read with bus.read_position_measured(), not write_read_pdo_2().\n")
        file.write('"""\n\n')

        file.write("GEAR_RATIO = 17.0\n")
        file.write("MOTOR_SIGN = -1.0\n\n")

        file.write("# Offset convention:\n")
        file.write("# real_joint_angle = raw_motor_position / (MOTOR_SIGN * GEAR_RATIO) + HOMING_OFFSET[id]\n")
        file.write("# raw_command = MOTOR_SIGN * (desired_joint_angle - HOMING_OFFSET[id]) * GEAR_RATIO\n\n")

        file.write("ROLE_TO_ID = {\n")
        for role in ["shank", "thigh", "hip"]:
            file.write(f'    "{role}": {ROLE_TO_ID[role]},\n')
        file.write("}\n\n")

        file.write("KNOWN_HOMED_JOINT_ANGLES_BY_ROLE = {\n")
        for role in ["shank", "thigh", "hip"]:
            file.write(f'    "{role}": {KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:.9f},\n')
        file.write("}\n\n")

        file.write("RAW_HOMED_POSITION = {\n")
        for motor_id in JOINT_IDS:
            file.write(f"    {motor_id}: {raw_homed_by_id[motor_id]:.9f},\n")
        file.write("}\n\n")

        file.write("RAW_READ_SPREAD = {\n")
        for motor_id in JOINT_IDS:
            file.write(f"    {motor_id}: {sample_spread_by_id[motor_id]:.9f},\n")
        file.write("}\n\n")

        file.write("SINGLE_ID_READ_SPREAD = {\n")
        for motor_id in JOINT_IDS:
            file.write(f"    {motor_id}: {single_id_spread_by_id.get(motor_id, float('nan')):.9f},\n")
        file.write("}\n\n")

        file.write("HOMING_OFFSET = {\n")
        for motor_id in JOINT_IDS:
            file.write(f"    {motor_id}: {offsets[motor_id]:.9f},\n")
        file.write("}\n")

    print(f"\nSaved offsets to {OFFSET_OUTPUT_FILE}")


def single_id_diagnostic():
    print("\n" + "=" * 80)
    print("Single-ID measured-position diagnostic")
    print("=" * 80)
    print("This reads one CAN ID at a time using bus.read_position_measured().")
    print("It does not send position targets and does not use write_read_pdo_2().")

    spread_by_id = {}
    bad_ids = []

    for motor_id in READ_ORDER:
        name = JOINT_NAMES[motor_id]
        print(f"\nReading only {name} ID {motor_id} for {SINGLE_ID_DIAG_SECONDS:.1f} s...")

        samples = []
        local_rate = RateLimiter(frequency=SINGLE_ID_DIAG_HZ)
        count = int(SINGLE_ID_DIAG_SECONDS * SINGLE_ID_DIAG_HZ)

        for i in range(count):
            try:
                raw = read_raw_once(motor_id)
                samples.append(raw)
                if i % max(1, int(SINGLE_ID_DIAG_HZ / 4)) == 0:
                    print(f"  {name:5s} raw={raw:+.6f} out={raw_to_output_angle(raw):+.4f}")
            except Exception as exc:
                print(f"  read error: {exc}")
            local_rate.sleep()

        if not samples:
            spread = float("inf")
            med = float("nan")
        else:
            samples.sort()
            med = median(samples)
            spread = max(samples) - min(samples)

        spread_by_id[motor_id] = spread
        print(f"  Summary {name:5s}: median={med:+.9f}, spread={spread:.6f}")

        if spread > SINGLE_ID_SPREAD_WARNING:
            print(f"  WARNING: {name} single-ID spread is high.")

        if spread > SINGLE_ID_SPREAD_HARD_REJECT:
            print(f"  BAD: {name} single-ID spread is above hard reject threshold.")
            bad_ids.append(motor_id)

    return spread_by_id, bad_ids


def live_encoder_monitor(stop_event):
    local_rate = RateLimiter(frequency=LIVE_PRINT_HZ)
    last_by_id = {}
    start_time = time.time()

    print("\nLIVE ENCODER MONITOR")
    print("  Manually move/backdrive the leg to max-contraction homing pose.")
    print("  Read method: bus.read_position_measured().")
    print("  Press Enter when the leg is stopped and held at max contraction.")
    print("  Ctrl+C cancels.\n")

    while not stop_event.is_set():
        now = time.time() - start_time
        line_parts = [f"t={now:6.2f}s"]

        for motor_id in READ_ORDER:
            name = JOINT_NAMES[motor_id]
            try:
                raw = read_raw_once(motor_id)
                out = raw_to_output_angle(raw)

                warn = ""
                if motor_id in last_by_id:
                    jump = raw - last_by_id[motor_id]
                    if abs(jump) > LIVE_JUMP_WARNING_RAW:
                        warn = f" <-- WARNING: {name} jump {jump:+.3f}"

                last_by_id[motor_id] = raw
                line_parts.append(f"{name} raw={raw:+.6f} out={out:+.4f}{warn}")
            except Exception as exc:
                line_parts.append(f"{name} read_error={exc}")

        print(" | ".join(line_parts))
        local_rate.sleep()


def print_summary(raw_homed_by_id, sample_spread_by_id, offsets):
    print("\nComputed HOMING_OFFSET:")
    for role in ["shank", "thigh", "hip"]:
        motor_id = ROLE_TO_ID[role]
        raw_output_angle = raw_to_output_angle(raw_homed_by_id[motor_id])
        print(
            f"  {role:5s} ID {motor_id}: "
            f"known={KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:+.6f}, "
            f"raw={raw_homed_by_id[motor_id]:+.6f}, "
            f"raw_output={raw_output_angle:+.6f}, "
            f"offset={offsets[motor_id]:+.6f}, "
            f"spread={sample_spread_by_id[motor_id]:.6f}"
        )


def main():
    print("=" * 80)
    print("Manual homing offset calibration v8 - separate measured-position read")
    print("=" * 80)
    print("This script does NOT recalibrate electrical offset.")
    print("This script does NOT use write_read_pdo_2() for encoder readings.")
    print("It reads positions with bus.read_position_measured(device_id).")
    print("It overwrites homing_offsets.py only after stable reads and y/yes confirmation.")
    print()
    print("Current role -> CAN ID mapping:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} -> ID {ROLE_TO_ID[role]}")
    print()
    print("Read order:")
    print("  " + " -> ".join(f"{JOINT_NAMES[mid]}(ID {mid})" for mid in READ_ORDER))
    print()
    print("Known max-contraction joint angles:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} = {KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:+.6f} rad")
    print("=" * 80)

    # Confirm API exists early.
    if not hasattr(bus, "read_position_measured"):
        raise RuntimeError(
            "This recoil.Bus does not have read_position_measured(). "
            "Your installed lowlevel package may differ from the Berkeley Humanoid Lite API."
        )

    idle_all_motors()

    input("\nPress Enter to run the single-ID diagnostic...")
    single_id_spread_by_id, bad_single_ids = single_id_diagnostic()

    if bad_single_ids:
        print("\nBAD single-ID read stability detected on:")
        for motor_id in bad_single_ids:
            print(f"  {JOINT_NAMES[motor_id]} ID {motor_id}")

        if REJECT_BAD_READS_BY_DEFAULT:
            answer = input("Type override to continue anyway, or Enter to abort: ").strip().lower()
            if answer != "override":
                print("Aborted before homing calibration. No file written.")
                return

    input("\nPress Enter to start live encoder monitor...")

    stop_event = threading.Event()
    monitor = threading.Thread(target=live_encoder_monitor, args=(stop_event,), daemon=True)
    monitor.start()

    input()
    stop_event.set()
    monitor.join(timeout=1.0)

    print("\nFinal pose selected. Reading final raw motor positions with median filter...")
    print("Hold the leg steady until all readings finish.\n")

    raw_homed_by_id = {}
    sample_spread_by_id = {}

    for motor_id in READ_ORDER:
        raw, spread = read_raw_median(motor_id)
        raw_homed_by_id[motor_id] = raw
        sample_spread_by_id[motor_id] = spread

    bad_final_ids = [
        motor_id for motor_id, spread in sample_spread_by_id.items()
        if spread > MAX_SAMPLE_SPREAD_HARD_REJECT
    ]

    if bad_final_ids:
        print("\nBAD final read stability detected on:")
        for motor_id in bad_final_ids:
            print(
                f"  {JOINT_NAMES[motor_id]} ID {motor_id}: "
                f"spread={sample_spread_by_id[motor_id]:.6f}"
            )

        if REJECT_BAD_READS_BY_DEFAULT:
            answer = input("Type override to compute/write anyway, or Enter to abort: ").strip().lower()
            if answer != "override":
                print("Aborted. homing_offsets.py was not written.")
                return

    offsets = compute_offsets(raw_homed_by_id)
    print_summary(raw_homed_by_id, sample_spread_by_id, offsets)

    print()
    print(f"This will write/overwrite: {OFFSET_OUTPUT_FILE}")
    confirm = input("Type y/yes to write the file: ").strip().lower()

    if confirm not in ("y", "yes"):
        print("Cancelled. homing_offsets.py was not written.")
        return

    print("Confirmed. Writing homing offset file...")
    write_offsets_file(offsets, raw_homed_by_id, sample_spread_by_id, single_id_spread_by_id)

    print("\nNext test:")
    print("  Run your safe foot-position script.")
    print("  If raw deltas are still huge, do not override; inspect signs/joint mapping first.")


try:
    main()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    idle_all_motors()
    shutdown_bus()

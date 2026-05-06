import time
from statistics import median

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ============================================================
# Manual homing offset calibration v2
# ============================================================
#
# This script does NOT move the joints.
#
# Procedure:
#   1. Run this script.
#   2. It puts all joints into IDLE.
#   3. Manually move/backdrive the leg to the known max-contraction pose.
#   4. Press Enter.
#   5. It reads raw motor positions.
#   6. It computes HOMING_OFFSET and writes homing_offsets.py.
#
# Offset convention used by foot_trot_manual_offset_v2.py:
#
#   real_joint_angle =
#       raw_motor_position / (MOTOR_SIGN * GEAR_RATIO)
#       + HOMING_OFFSET[id]
#
# Therefore:
#
#   HOMING_OFFSET[id] =
#       known_homed_joint_angle
#       - raw_homed_position / (MOTOR_SIGN * GEAR_RATIO)
#
# And:
#
#   raw_command =
#       MOTOR_SIGN * (desired_joint_angle - HOMING_OFFSET[id]) * GEAR_RATIO
#
# ============================================================


# ============================================================
# Current assembled-leg IDs
# ============================================================
SHANK_ID = 2
THIGH_ID = 0
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


# ============================================================
# Gear / sign
# ============================================================
GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0


# ============================================================
# Known ideal joint angles at manual homing pose
# ============================================================
# These are copied from the old automatic homing calibration.
# Update them if your max-contraction definition changes.
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
# Read settings
# ============================================================
RATE_HZ = 50.0
SAMPLE_COUNT = 60
FLUSH_COUNT = 10
MAX_SAMPLE_SPREAD_WARNING = 0.25


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


def read_raw_once(motor_id):
    pos, vel = bus.write_read_pdo_2(motor_id, 0.0, 0.0)

    if pos is None:
        raise RuntimeError(f"Could not read {JOINT_NAMES[motor_id]}")

    return pos


def read_raw_median(motor_id):
    name = JOINT_NAMES[motor_id]

    for _ in range(FLUSH_COUNT):
        try:
            bus.write_read_pdo_2(motor_id, 0.0, 0.0)
        except Exception:
            pass
        rate.sleep()

    samples = []

    for _ in range(SAMPLE_COUNT):
        samples.append(read_raw_once(motor_id))
        rate.sleep()

    samples.sort()

    raw_median = median(samples)
    spread = max(samples) - min(samples)

    print(
        f"  {name:5s} raw median = {raw_median:+.9f} "
        f"(spread={spread:.6f})"
    )

    if spread > MAX_SAMPLE_SPREAD_WARNING:
        print(
            f"    WARNING: {name} moved/noisy during reading. "
            "Try holding the joint more still and rerun calibration."
        )

    return raw_median, spread


def raw_to_output_angle(raw_motor_position):
    return raw_motor_position / (MOTOR_SIGN * GEAR_RATIO)


def compute_offsets(raw_homed_by_id):
    offsets = {}

    for role, motor_id in ROLE_TO_ID.items():
        known_angle = KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]
        raw_homed = raw_homed_by_id[motor_id]

        offsets[motor_id] = known_angle - raw_to_output_angle(raw_homed)

    return offsets


def write_offsets_file(offsets, raw_homed_by_id, sample_spread_by_id):
    with open(OFFSET_OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write('"""Auto-generated MANUAL homing offsets.\\n\\n')
        file.write("Generated by manual_calibrate_home_offset_v2.py.\\n")
        file.write("The leg was manually placed at the max-contraction homing pose.\\n")
        file.write('"""\\n\\n')

        file.write("GEAR_RATIO = 17.0\\n")
        file.write("MOTOR_SIGN = -1.0\\n\\n")

        file.write("# Offset convention:\\n")
        file.write("# real_joint_angle = raw_motor_position / (MOTOR_SIGN * GEAR_RATIO) + HOMING_OFFSET[id]\\n")
        file.write("# raw_command = MOTOR_SIGN * (desired_joint_angle - HOMING_OFFSET[id]) * GEAR_RATIO\\n\\n")

        file.write("ROLE_TO_ID = {\\n")
        for role in ["shank", "thigh", "hip"]:
            file.write(f'    "{role}": {ROLE_TO_ID[role]},\\n')
        file.write("}\\n\\n")

        file.write("KNOWN_HOMED_JOINT_ANGLES_BY_ROLE = {\\n")
        for role in ["shank", "thigh", "hip"]:
            file.write(
                f'    "{role}": {KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:.9f},\\n'
            )
        file.write("}\\n\\n")

        file.write("RAW_HOMED_POSITION = {\\n")
        for motor_id in JOINT_IDS:
            file.write(f"    {motor_id}: {raw_homed_by_id[motor_id]:.9f},\\n")
        file.write("}\\n\\n")

        file.write("RAW_READ_SPREAD = {\\n")
        for motor_id in JOINT_IDS:
            file.write(f"    {motor_id}: {sample_spread_by_id[motor_id]:.9f},\\n")
        file.write("}\\n\\n")

        file.write("HOMING_OFFSET = {\\n")
        for motor_id in JOINT_IDS:
            file.write(f"    {motor_id}: {offsets[motor_id]:.9f},\\n")
        file.write("}\\n")

    print(f"\nSaved offsets to {OFFSET_OUTPUT_FILE}")


def shutdown_bus():
    try:
        bus.stop()
    except Exception:
        pass


# ============================================================
# Main
# ============================================================
def main():
    print("=" * 80)
    print("Manual homing offset calibration v2")
    print("=" * 80)
    print("This script overwrites homing_offsets.py only after you type YES.")
    print()
    print("Current role -> CAN ID mapping:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} -> ID {ROLE_TO_ID[role]}")
    print()
    print("Known max-contraction joint angles:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} = {KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:+.6f} rad")
    print("=" * 80)

    idle_all_motors()

    print("\nACTION REQUIRED:")
    print("  Manually backdrive the leg to the max-contraction homing pose.")
    print("  Hold it there steadily.")
    print()

    input("Press Enter when the leg is manually at max contraction... ")

    print("\nReading raw motor positions...")

    raw_homed_by_id = {}
    sample_spread_by_id = {}

    for motor_id in JOINT_IDS:
        raw, spread = read_raw_median(motor_id)
        raw_homed_by_id[motor_id] = raw
        sample_spread_by_id[motor_id] = spread

    offsets = compute_offsets(raw_homed_by_id)

    print("\nComputed HOMING_OFFSET:")
    for role in ["shank", "thigh", "hip"]:
        motor_id = ROLE_TO_ID[role]
        raw_output_angle = raw_to_output_angle(raw_homed_by_id[motor_id])

        print(
            f"  {role:5s} ID {motor_id}: "
            f"known={KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:+.6f}, "
            f"raw={raw_homed_by_id[motor_id]:+.6f}, "
            f"raw_output={raw_output_angle:+.6f}, "
            f"offset={offsets[motor_id]:+.6f}"
        )

    print()
    print(f"This will write/overwrite: {OFFSET_OUTPUT_FILE}")
    confirm = input("Type YES to write the file: ").strip()

    if confirm != "YES":
        print("Cancelled. homing_offsets.py was not written.")
        return

    write_offsets_file(offsets, raw_homed_by_id, sample_spread_by_id)

    print("\nNext test:")
    print("  Run foot_trot_manual_offset_v2.py.")
    print("  It should move to neutral using these offsets, then start trajectory.")


try:
    main()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    idle_all_motors()
    shutdown_bus()

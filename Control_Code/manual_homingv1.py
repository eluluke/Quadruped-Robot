import time
from statistics import median

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# ============================================================
# Manual homing offset calibration
# ============================================================
#
# Purpose:
#   Fast manual replacement for the slow mechanical-limit homing script.
#
# Manual procedure:
#   1. Power the robot safely.
#   2. Run this script.
#   3. The script puts all three joints into IDLE.
#   4. Manually backdrive the leg to the known max-contraction homing pose.
#   5. Press Enter.
#   6. The script reads each joint's raw encoder position.
#   7. It computes homing offsets and writes homing_offsets.py.
#
# This does NOT detect current spikes, position freeze, or mechanical limits.
# It trusts that you manually placed the leg at the correct homing pose.
#
# ============================================================


# ============================================================
# Motor IDs
# ============================================================
# IMPORTANT:
# Set these to match the current physical assembly.
#
# Your current working assembled trajectory has:
#   shank = 2
#   thigh = 0
#   hip   = 1
#
# If you are using the older foot_trot.py stack, it used:
#   shank = 1
#   thigh = 0
#   hip   = 2
#
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
# Known joint angles at manual homing pose
# ============================================================
# These are the ideal IK/output-side joint angles for the max-contraction pose.
# They are copied from the previous automatic calibrate_home_offset.py.
#
# If the physical "max contraction" pose definition changes, update these.
KNOWN_HOMED_JOINT_ANGLES_BY_ROLE = {
    "shank": 2.688,
    "thigh": 1.199,
    "hip": 1.069,
}


# ============================================================
# Output
# ============================================================
OFFSET_OUTPUT_FILE = "homing_offsets.py"

# Existing convention from your previous homing_offsets.py:
#   real_joint_angle = raw_encoder_position + HOMING_OFFSET[id]
# Therefore:
#   HOMING_OFFSET[id] = known_homed_joint_angle - raw_encoder_at_homed_pose
#
# This preserves compatibility with the original offset file format.
WRITE_COMPATIBLE_HOMING_OFFSET_DICT = True


# ============================================================
# Read settings
# ============================================================
RATE_HZ = 50.0
SAMPLE_COUNT = 60
FLUSH_COUNT = 10
MAX_ACCEPTED_SAMPLE_SPREAD = 0.25  # raw rad; warning only

BRING_CAN_DOWN_ON_EXIT = False


# ============================================================
# Setup
# ============================================================
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)


# ============================================================
# Low-level helpers
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
    pos, vel = bus.write_read_pdo_2(
        motor_id,
        0.0,
        0.0,
    )

    if pos is None:
        raise RuntimeError(f"Could not read {JOINT_NAMES[motor_id]}")

    return pos


def read_raw_median(motor_id):
    name = JOINT_NAMES[motor_id]

    # Flush stale frames.
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

    samples_sorted = sorted(samples)
    med = median(samples_sorted)
    spread = max(samples_sorted) - min(samples_sorted)

    print(
        f"  {name:5s} raw median = {med:+.9f} "
        f"(spread={spread:.6f}, min={min(samples_sorted):+.6f}, max={max(samples_sorted):+.6f})"
    )

    if spread > MAX_ACCEPTED_SAMPLE_SPREAD:
        print(
            f"    WARNING: {name} raw samples have large spread. "
            "The joint may have moved during reading, or feedback is noisy."
        )

    return med, spread


def shutdown_bus():
    try:
        bus.stop()
    except Exception:
        pass


# ============================================================
# Offset computation
# ============================================================
def compute_offsets(raw_homed_by_id):
    offsets = {}

    for role, motor_id in ROLE_TO_ID.items():
        known_angle = KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]
        raw_homed = raw_homed_by_id[motor_id]

        offsets[motor_id] = known_angle - raw_homed

    return offsets


def write_offsets_file(offsets, raw_homed_by_id, spreads_by_id):
    with open(OFFSET_OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write('"""Auto-generated MANUAL homing offsets.\\n\\n')
        file.write("Generated by manual_calibrate_home_offset.py.\\n")
        file.write("The leg was manually placed at the max-contraction homing pose.\\n")
        file.write('"""\\n\\n')

        file.write("# Convention preserved from previous homing_offsets.py:\\n")
        file.write("#   real_joint_angle = raw_encoder_position + HOMING_OFFSET[id]\\n")
        file.write("# Therefore:\\n")
        file.write("#   HOMING_OFFSET[id] = known_homed_joint_angle - raw_encoder_at_homed_pose\\n\\n")

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
            file.write(f"    {motor_id}: {spreads_by_id[motor_id]:.9f},\\n")
        file.write("}\\n\\n")

        file.write("HOMING_OFFSET = {\\n")
        for motor_id in JOINT_IDS:
            file.write(f"    {motor_id}: {offsets[motor_id]:.9f},\\n")
        file.write("}\\n")

    print(f"\nSaved manual homing offsets to {OFFSET_OUTPUT_FILE}")


# ============================================================
# Main
# ============================================================
def main():
    print("=" * 80)
    print("MANUAL homing offset calibration")
    print("=" * 80)
    print("This is a fast manual replacement for the slow limit-detection homing.")
    print()
    print("Current role -> CAN ID mapping:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} -> ID {ROLE_TO_ID[role]}")
    print()
    print("Known max-contraction joint angles:")
    for role in ["shank", "thigh", "hip"]:
        print(f"  {role:5s} = {KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:+.6f} rad")
    print("=" * 80)
    print()

    idle_all_motors()

    print("\nACTION REQUIRED:")
    print("  Manually backdrive the physical leg into the max-contraction homing pose.")
    print("  Make sure the pose is repeatable and pushed gently against the intended mechanical references.")
    print("  The motors are IDLE, so the script will not move the leg.")
    print()

    input("When the leg is at max contraction, press Enter to record raw encoder positions... ")

    print("\nReading raw encoder positions at manual homing pose...")

    raw_homed_by_id = {}
    spreads_by_id = {}

    for motor_id in JOINT_IDS:
        raw, spread = read_raw_median(motor_id)
        raw_homed_by_id[motor_id] = raw
        spreads_by_id[motor_id] = spread

    offsets = compute_offsets(raw_homed_by_id)

    print("\nComputed manual homing offsets:")
    for role in ["shank", "thigh", "hip"]:
        motor_id = ROLE_TO_ID[role]
        print(
            f"  {role:5s} ID {motor_id}: "
            f"known={KNOWN_HOMED_JOINT_ANGLES_BY_ROLE[role]:+.6f}, "
            f"raw={raw_homed_by_id[motor_id]:+.6f}, "
            f"offset={offsets[motor_id]:+.6f}"
        )

    print()
    print("This will overwrite/create:", OFFSET_OUTPUT_FILE)
    answer = input("Write this offset file? Type YES to confirm: ").strip()

    if answer != "YES":
        print("Cancelled. Offset file was not written.")
        return

    write_offsets_file(offsets, raw_homed_by_id, spreads_by_id)

    print("\nNext suggested test:")
    print("  1. Use the generated homing_offsets.py in a neutral/trajectory script.")
    print("  2. Command the leg to neutral standing pose slowly.")
    print("  3. Check whether it returns to the expected neutral pose.")
    print("  4. Then start the trajectory.")


try:
    main()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    idle_all_motors()
    shutdown_bus()

import time
from statistics import median

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ============================================================
# Read-position diagnostic WITHOUT changing motor modes
# ============================================================
# Purpose:
#   Test whether position-read contamination still happens if we do NOT send
#   set_mode(IDLE), feed(), or any position command before reading.
#
# Safety:
#   This script does not command motion.
#   It does not change motor modes.
#   It only calls bus.read_position_measured(device_id).
#
# Use case:
#   Run this while the motors are already in whatever state they are in.
#   For safest testing, power-cycle the motor boards first, do not run another
#   script, then run this diagnostic.
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

READ_ORDER = [SHANK_ID, THIGH_ID, HIP_ID]

GEAR_RATIO = 17.0
MOTOR_SIGN = -1.0

RATE_HZ = 20.0
SINGLE_ID_SECONDS = 3.0
CYCLIC_SECONDS = 8.0
SAMPLES_PER_PRINT = 1

SPREAD_WARN = 0.25
SPREAD_BAD = 1.0
JUMP_WARN = 0.75

args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
rate = RateLimiter(frequency=RATE_HZ)


def raw_to_output_angle(raw):
    return raw / (MOTOR_SIGN * GEAR_RATIO)


def read_measured(motor_id):
    value = bus.read_position_measured(motor_id)

    # Some APIs return a bare float; some may return tuple/list.
    if isinstance(value, (tuple, list)):
        if len(value) == 0:
            raise RuntimeError(f"Empty read result for ID {motor_id}")
        value = value[0]

    if value is None:
        raise RuntimeError(f"None read result for ID {motor_id}")

    return float(value)


def summarize_samples(name, samples):
    if not samples:
        print(f"  Summary {name:5s}: no valid samples")
        return None, None

    s = sorted(samples)
    med = median(s)
    spread = max(s) - min(s)

    print(f"  Summary {name:5s}: median={med:+.9f}, spread={spread:.6f}")

    if spread > SPREAD_BAD:
        print(f"  BAD: {name} spread above {SPREAD_BAD:.3f}")
    elif spread > SPREAD_WARN:
        print(f"  WARNING: {name} spread above {SPREAD_WARN:.3f}")
    else:
        print(f"  GOOD: {name} read is stable")

    return med, spread


def single_id_test():
    print("\n" + "=" * 80)
    print("Single-ID read_position_measured diagnostic WITHOUT mode changes")
    print("=" * 80)
    print("This test does not call set_mode(), feed(), write_read_pdo_2(), or write targets.")

    results = {}

    for motor_id in READ_ORDER:
        name = JOINT_NAMES[motor_id]
        print(f"\nReading only {name} ID {motor_id} for {SINGLE_ID_SECONDS:.1f} s...")

        samples = []
        last = None
        steps = int(SINGLE_ID_SECONDS * RATE_HZ)

        for i in range(steps):
            try:
                raw = read_measured(motor_id)
                samples.append(raw)

                jump_msg = ""
                if last is not None:
                    jump = raw - last
                    if abs(jump) > JUMP_WARN:
                        jump_msg = f"  <-- WARNING jump {jump:+.3f}"
                last = raw

                if i % SAMPLES_PER_PRINT == 0:
                    print(
                        f"  {name:5s} raw={raw:+.6f} "
                        f"out={raw_to_output_angle(raw):+.4f}"
                        f"{jump_msg}"
                    )

            except Exception as exc:
                print(f"  read error for {name} ID {motor_id}: {exc}")

            rate.sleep()

        med, spread = summarize_samples(name, samples)
        results[motor_id] = {
            "name": name,
            "samples": samples,
            "median": med,
            "spread": spread,
        }

    return results


def cyclic_test():
    print("\n" + "=" * 80)
    print("Cyclic all-ID read_position_measured diagnostic WITHOUT mode changes")
    print("=" * 80)
    print("This checks whether cycling IDs causes label contamination.")

    samples_by_id = {motor_id: [] for motor_id in READ_ORDER}
    last_by_id = {motor_id: None for motor_id in READ_ORDER}

    start = time.time()
    next_print = start

    while time.time() - start < CYCLIC_SECONDS:
        values = {}
        warnings = []

        for motor_id in READ_ORDER:
            name = JOINT_NAMES[motor_id]
            try:
                raw = read_measured(motor_id)
                values[motor_id] = raw
                samples_by_id[motor_id].append(raw)

                last = last_by_id[motor_id]
                if last is not None:
                    jump = raw - last
                    if abs(jump) > JUMP_WARN:
                        warnings.append(f"{name} jump {jump:+.3f}")
                last_by_id[motor_id] = raw

            except Exception as exc:
                warnings.append(f"{name} read error: {exc}")

        now = time.time()
        if now >= next_print:
            parts = []
            for motor_id in READ_ORDER:
                name = JOINT_NAMES[motor_id]
                if motor_id in values:
                    raw = values[motor_id]
                    parts.append(f"{name}={raw:+.6f}")
                else:
                    parts.append(f"{name}=ERR")

            line = f"t={now - start:6.2f}s | " + " | ".join(parts)
            if warnings:
                line += "  <-- " + "; ".join(warnings)
            print(line)
            next_print = now + 0.25

        rate.sleep()

    print("\nCyclic summary:")
    for motor_id in READ_ORDER:
        summarize_samples(JOINT_NAMES[motor_id], samples_by_id[motor_id])

    return samples_by_id


def print_interpretation(single_results):
    print("\n" + "=" * 80)
    print("Interpretation guide")
    print("=" * 80)
    print("If shank is stable here, but unstable in manual_homingv8, then the contamination")
    print("was probably caused by earlier set_mode/feed/write-read traffic before reading.")
    print()
    print("If shank is still jumping here, even without mode changes, then the issue is")
    print("inside read_position_measured()/SDO readback path, ID0 behavior, or device response")
    print("association — not the trajectory command path.")
    print()
    print("If shank jumps exactly to thigh/hip values, compare the printed medians above.")
    print("That means the host-side readback is mixing labels, even though motor commands may")
    print("still go to the correct physical actuator.")


try:
    print("=" * 80)
    print("Position read diagnostic: NO IDLE / NO FEED / NO COMMAND")
    print("=" * 80)
    print("Current role -> CAN ID mapping:")
    for motor_id in READ_ORDER:
        print(f"  {JOINT_NAMES[motor_id]:5s} -> ID {motor_id}")
    print()
    print("Recommended clean test:")
    print("  1. Stop all previous scripts.")
    print("  2. Power-cycle or reset the motor boards if possible.")
    print("  3. Run this script before any homing/trajectory script.")
    print("  4. Do not touch the leg during the single-ID test.")
    print("=" * 80)

    input("\nPress Enter to start WITHOUT changing motor modes...")

    single_results = single_id_test()

    input("\nPress Enter to start cyclic all-ID read test...")
    cyclic_test()

    print_interpretation(single_results)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    # Intentionally do NOT set IDLE and do NOT bus.stop aggressively.
    # This diagnostic was designed to avoid adding extra CAN/mode traffic.
    try:
        bus.stop()
    except Exception:
        pass
    print("\nDiagnostic complete.")

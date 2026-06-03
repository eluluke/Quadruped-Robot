"""Simple terminal test for quadruped_leg_ik.py."""

import math

from leg_config import NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z
from quadruped_leg_ik import leg_ik


def parse_xyz(text):
    """Parse one terminal input line as x, y, z."""
    parts = text.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError("Enter exactly three numbers: x y z")
    return float(parts[0]), float(parts[1]), float(parts[2])


def print_angle_row(name, angle):
    """Print one angle in radians and degrees."""
    print(f"{name:>6s}: {angle:+.6f} rad  {math.degrees(angle):+.3f} deg")


def print_ik_result(x, y, z):
    """Compute and print IK plus displacement to neutral standing pose."""
    hip, thigh, shank = leg_ik(x, y, z)
    neutral_hip, neutral_thigh, neutral_shank = leg_ik(
        NEUTRAL_X,
        NEUTRAL_Y,
        NEUTRAL_Z,
    )

    print()
    print(f"Input foot position:   x={x:+.3f}, y={y:+.3f}, z={z:+.3f} mm")
    print(
        "Neutral foot position: "
        f"x={NEUTRAL_X:+.3f}, y={NEUTRAL_Y:+.3f}, z={NEUTRAL_Z:+.3f} mm"
    )

    print("\nInput IK angles:")
    print_angle_row("hip", hip)
    print_angle_row("thigh", thigh)
    print_angle_row("shank", shank)

    print("\nNeutral IK angles:")
    print_angle_row("hip", neutral_hip)
    print_angle_row("thigh", neutral_thigh)
    print_angle_row("shank", neutral_shank)

    print("\nAngular displacement from input pose to neutral pose:")
    print_angle_row("hip", neutral_hip - hip)
    print_angle_row("thigh", neutral_thigh - thigh)
    print_angle_row("shank", neutral_shank - shank)


def main():
    print("Quadruped leg IK terminal test")
    print("Enter foot position as: x y z")
    print("Type q to quit.")

    while True:
        text = input("\nx y z > ").strip()
        if text.lower() in ("q", "quit", "exit"):
            break
        if not text:
            continue

        try:
            x, y, z = parse_xyz(text)
            print_ik_result(x, y, z)
        except Exception as exc:
            print(f"Error: {exc}")


if __name__ == "__main__":
    main()

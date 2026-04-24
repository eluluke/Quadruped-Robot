"""Minimal terminal FK tester.

Type in 3 motor angles manually and get the foot position.
Use this to compare with your SolidWorks model.
"""

import math

from quadruped_leg_fk import leg_fk


def parse_three_numbers(text):
    parts = text.strip().replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError("Please enter exactly 3 numbers.")
    return float(parts[0]), float(parts[1]), float(parts[2])


def main():
    print("=" * 70)
    print("Quadruped Leg FK Terminal Tester")
    print("=" * 70)
    print("Enter 3 motor angles and get foot position (x, y, z).")
    print("Type 'q' to quit.")
    print()

    while True:
        unit = input("Angle unit? Type 'deg' or 'rad': ").strip().lower()

        if unit == "q":
            break

        if unit not in {"deg", "rad"}:
            print("Invalid unit. Please type 'deg' or 'rad'.")
            print()
            continue

        raw = input(
            "Enter theta_h theta_t theta_s "
            f"in {unit} (example: 10 20 30): "
        ).strip()

        if raw.lower() == "q":
            break

        try:
            theta_h, theta_t, theta_s = parse_three_numbers(raw)

            if unit == "deg":
                theta_h = math.radians(theta_h)
                theta_t = math.radians(theta_t)
                theta_s = math.radians(theta_s)

            x, y, z = leg_fk(theta_h, theta_t, theta_s)

            print()
            print("Input angles [rad]:")
            print(f"  theta_h = {theta_h:.6f}")
            print(f"  theta_t = {theta_t:.6f}")
            print(f"  theta_s = {theta_s:.6f}")
            print()
            print("Foot position [mm]:")
            print(f"  x = {x:.6f}")
            print(f"  y = {y:.6f}")
            print(f"  z = {z:.6f}")
            print("-" * 70)
            print()

        except Exception as exc:
            print(f"Error: {exc}")
            print()


if __name__ == "__main__":
    main()

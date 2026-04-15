import math
from quadruped_leg_ik import leg_ik


def main():
    print("Quadruped IK terminal tester")
    print("Enter x y z in mm, for example: 0 84.26 381.84")
    print("Type q to quit.\n")

    while True:
        raw = input("x y z > ").strip()

        if raw.lower() in {"q", "quit", "exit"}:
            print("Exiting.")
            break

        parts = raw.replace(",", " ").split()
        if len(parts) != 3:
            print("Please enter exactly 3 numbers: x y z")
            continue

        try:
            x, y, z = map(float, parts)
            theta_h, theta_t, theta_s = leg_ik(x, y, z)
        except ValueError:
            print("Invalid input. Please enter numeric values.")
            continue
        except Exception as e:
            print(f"IK error: {e}")
            continue

        print(f"Input Cartesian (mm): x={x:.3f}, y={y:.3f}, z={z:.3f}")
        print(
            "Angles (rad): "
            f"hip={theta_h:.6f}, thigh={theta_t:.6f}, shank={theta_s:.6f}"
        )
        print(
            "Angles (deg): "
            f"hip={math.degrees(theta_h):.3f}, "
            f"thigh={math.degrees(theta_t):.3f}, "
            f"shank={math.degrees(theta_s):.3f}"
        )
        print()


if __name__ == "__main__":
    main()

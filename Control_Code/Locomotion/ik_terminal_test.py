"""
ik_terminal_test.py

Terminal inverse-kinematics tester for the quadruped leg.

Purpose:
    Type Cartesian foot coordinates (x, y, z) in the IK frame.
    The script returns the corresponding joint/polar angles:

        hip angle
        thigh angle
        shank angle

    It prints both radians and degrees so you can check whether a newly
    observed max-contraction pose is reasonable.

Coordinate convention:
    x = forward/back direction in IK frame
    y = sideways / hip-offset direction
    z = vertical direction, positive downward if your IK uses z-down

Expected helper:
    Put this file in the same folder as quadruped_leg_ik.py, or make sure
    quadruped_leg_ik.py is importable from your Python path.

Run:
    python ik_terminal_test.py

Example input:
    0 84.26 378
    17.55 145.79 -16.13
    q
"""

from __future__ import annotations

import math
from typing import Tuple


try:
    from quadruped_leg_ik import leg_ik, L_H, L_T, L_S
    USING_IMPORTED_IK = True

except ImportError:
    USING_IMPORTED_IK = False

    L_H = 84.26
    L_T = 270.0
    L_S = 270.0

    def leg_ik(x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Fallback IK matching your quadruped_leg_ik.py."""
        z_1 = math.sqrt(y**2 + z**2 - L_H**2)
        l_1 = math.sqrt(x**2 + z_1**2)

        a_1 = math.acos((L_S**2 + L_T**2 - l_1**2) / (2 * L_S * L_T))
        a_2 = math.atan2(x, z_1)
        b_1 = math.acos((L_T**2 + l_1**2 - L_S**2) / (2 * L_T * l_1))
        b_2 = math.atan2(y, z)
        b_3 = math.atan2(z_1, L_H)

        theta_h = math.pi / 2 - b_2 - b_3
        theta_t = b_1 - a_2
        theta_s = math.pi - a_1

        return theta_h, theta_t, theta_s


KNOWN_MAX_CONTRACTION = {
    "hip": 1.069,
    "thigh": 1.199,
    "shank": 2.688,
}


def deg(rad: float) -> float:
    """Convert radians to degrees."""
    return rad * 180.0 / math.pi


def clamp(value: float, low: float, high: float) -> float:
    """Clamp value to [low, high]."""
    return max(low, min(high, value))


def parse_xyz(text: str) -> Tuple[float, float, float]:
    """Parse x y z from either space-separated or comma-separated input."""
    cleaned = text.replace(",", " ").strip()
    parts = cleaned.split()

    if len(parts) != 3:
        raise ValueError("Please enter exactly three numbers: x y z")

    return float(parts[0]), float(parts[1]), float(parts[2])


def reachability_report(x: float, y: float, z: float) -> str:
    """
    Return a simple reachability report.

    This does not replace IK. It just gives useful diagnostic numbers before
    the acos/sqrt terms fail.
    """
    hip_plane_radius_sq = y**2 + z**2
    hip_clearance_sq = hip_plane_radius_sq - L_H**2

    if hip_clearance_sq < 0.0:
        return (
            "UNREACHABLE around hip offset: "
            f"y^2 + z^2 - L_H^2 = {hip_clearance_sq:.3f} < 0"
        )

    z_1 = math.sqrt(hip_clearance_sq)
    l_1 = math.sqrt(x**2 + z_1**2)

    min_reach = abs(L_T - L_S)
    max_reach = L_T + L_S

    if l_1 < min_reach:
        return (
            "LIKELY UNREACHABLE: target is too close for thigh+shank. "
            f"l_1={l_1:.3f}, min={min_reach:.3f}"
        )

    if l_1 > max_reach:
        return (
            "LIKELY UNREACHABLE: target is too far for thigh+shank. "
            f"l_1={l_1:.3f}, max={max_reach:.3f}"
        )

    return (
        "reachable check OK | "
        f"z_1={z_1:.3f} mm, l_1={l_1:.3f} mm, "
        f"valid range=[{min_reach:.3f}, {max_reach:.3f}] mm"
    )


def safe_leg_ik(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Safer IK wrapper.

    Your original IK is mathematically correct, but acos arguments can go
    slightly outside [-1, 1] due to numerical precision near boundaries.
    This wrapper reproduces the same equations with clamped acos arguments.
    """
    hip_clearance_sq = y**2 + z**2 - L_H**2
    if hip_clearance_sq < 0.0:
        raise ValueError(
            f"Invalid pose: y^2 + z^2 - L_H^2 = {hip_clearance_sq:.6f} < 0"
        )

    z_1 = math.sqrt(hip_clearance_sq)
    l_1 = math.sqrt(x**2 + z_1**2)

    if l_1 <= 1e-9:
        raise ValueError("Invalid pose: l_1 is too close to zero.")

    a_1_arg = (L_S**2 + L_T**2 - l_1**2) / (2 * L_S * L_T)
    b_1_arg = (L_T**2 + l_1**2 - L_S**2) / (2 * L_T * l_1)

    a_1 = math.acos(clamp(a_1_arg, -1.0, 1.0))
    a_2 = math.atan2(x, z_1)
    b_1 = math.acos(clamp(b_1_arg, -1.0, 1.0))
    b_2 = math.atan2(y, z)
    b_3 = math.atan2(z_1, L_H)

    theta_h = math.pi / 2 - b_2 - b_3
    theta_t = b_1 - a_2
    theta_s = math.pi - a_1

    return theta_h, theta_t, theta_s


def print_result(x: float, y: float, z: float) -> None:
    """Print IK result for one Cartesian coordinate."""
    print("\n" + "-" * 72)
    print(f"Input Cartesian coordinate: x={x:+.3f}, y={y:+.3f}, z={z:+.3f} mm")
    print(reachability_report(x, y, z))

    try:
        hip, thigh, shank = safe_leg_ik(x, y, z)

    except Exception as exc:
        print(f"IK failed: {exc}")
        print("-" * 72)
        return

    print("\nJoint / polar angles:")
    print(f"  hip   = {hip:+.9f} rad  = {deg(hip):+.3f} deg")
    print(f"  thigh = {thigh:+.9f} rad  = {deg(thigh):+.3f} deg")
    print(f"  shank = {shank:+.9f} rad  = {deg(shank):+.3f} deg")

    print("\nDifference from known max-contraction angles:")
    print(
        f"  hip   diff = {hip - KNOWN_MAX_CONTRACTION['hip']:+.9f} rad "
        f"= {deg(hip - KNOWN_MAX_CONTRACTION['hip']):+.3f} deg"
    )
    print(
        f"  thigh diff = {thigh - KNOWN_MAX_CONTRACTION['thigh']:+.9f} rad "
        f"= {deg(thigh - KNOWN_MAX_CONTRACTION['thigh']):+.3f} deg"
    )
    print(
        f"  shank diff = {shank - KNOWN_MAX_CONTRACTION['shank']:+.9f} rad "
        f"= {deg(shank - KNOWN_MAX_CONTRACTION['shank']):+.3f} deg"
    )

    print("-" * 72)


def main() -> None:
    print("=" * 72)
    print("Quadruped leg IK terminal tester")
    print("=" * 72)
    print(f"Using imported quadruped_leg_ik.py: {USING_IMPORTED_IK}")
    print(f"Geometry: L_H={L_H:.3f} mm, L_T={L_T:.3f} mm, L_S={L_S:.3f} mm")
    print("\nType Cartesian coordinate as:")
    print("  x y z")
    print("or:")
    print("  x, y, z")
    print("\nType q to quit.")
    print("=" * 72)

    while True:
        text = input("\nEnter x y z > ").strip()

        if text.lower() in ("q", "quit", "exit"):
            print("Done.")
            break

        if not text:
            continue

        try:
            x, y, z = parse_xyz(text)
            print_result(x, y, z)

        except Exception as exc:
            print(f"Input error: {exc}")


if __name__ == "__main__":
    main()

"""Fast cycloidal drive eccentricity screening tool.

This is an analytical pre-ANSYS optimizer. It estimates ring-pin contact load,
local profile curvature, and Hertz line-contact pressure for a cycloidal disk.

Edit the CONFIG values near the top, then run:

    python cycloidal_profile_optimizer.py

The script writes:

    cycloidal_optimization_results.csv

Notes:
    - This is not a replacement for nonlinear contact FEA or real testing.
    - It is meant to quickly reject bad eccentricity/profile choices.
    - Units are mm, N, Nm, MPa unless otherwise stated.
"""

from __future__ import annotations

from dataclasses import dataclass
import csv
import math
from pathlib import Path


@dataclass(frozen=True)
class GearInputs:
    """Edit these values to match your CAD."""

    # Basic cycloidal geometry.
    gear_ratio: int = 25
    ring_pitch_radius_mm: float = 72
    ring_pin_radius_mm: float = 2.6
    face_width_mm: float = 5.0

    # Output pin/hole geometry. The hole must clear the eccentric orbit.
    # Constraint used: e <= output_pin_hole_radius - output_pin_radius - clearance.
    output_pin_hole_radius_mm: float = 5.4

    output_pin_radius_mm: float = 4.0
    radial_clearance_mm: float = 0.02

    # Robot actuator load.
    output_torque_nm: float = 25.0
    shock_factor: float = 2.0
    gearbox_efficiency: float = 0.85
    effective_loaded_ring_pins: int = 9

    # Contact material pair.
    # Conservative printed plastic vs steel pin defaults.
    cycloid_youngs_modulus_mpa: float = 1800.0
    cycloid_poissons_ratio: float = 0.34
    ring_pin_youngs_modulus_mpa: float = 200_000.0
    ring_pin_poissons_ratio: float = 0.30

    # Search range. The script also applies geometry constraints automatically.
    eccentricity_min_mm: float = 0.40
    eccentricity_max_mm: float = 3.50
    eccentricity_samples: int = 180

    # Keep the curtate ratio below 1 to avoid sharp/invalid geometry.
    # lambda = pin_count * eccentricity / ring_pitch_radius.
    curtate_ratio_min: float = 0.35
    curtate_ratio_max: float = 0.85

    # Numerical profile resolution.
    profile_points: int = 1800


CONFIG = GearInputs()


@dataclass(frozen=True)
class CandidateResult:
    eccentricity_mm: float
    curtate_ratio: float
    force_per_pin_n: float
    profile_min_curvature_radius_mm: float
    effective_contact_radius_mm: float
    peak_hertz_pressure_mpa: float
    feasible: bool
    reason: str


def pin_count(cfg: GearInputs) -> int:
    """For a normal N:1 cycloidal drive, use N lobes and N+1 ring pins."""

    return cfg.gear_ratio + 1


def equivalent_modulus_mpa(cfg: GearInputs) -> float:
    """Return Hertz equivalent modulus E' in MPa."""

    inv_e = (
        (1.0 - cfg.cycloid_poissons_ratio**2) / cfg.cycloid_youngs_modulus_mpa
        + (1.0 - cfg.ring_pin_poissons_ratio**2)
        / cfg.ring_pin_youngs_modulus_mpa
    )
    return 1.0 / inv_e


def output_hole_eccentricity_limit(cfg: GearInputs) -> float:
    """Largest eccentricity allowed by the output pin/hole clearance."""

    return (
        cfg.output_pin_hole_radius_mm
        - cfg.output_pin_radius_mm
        - cfg.radial_clearance_mm
    )


def ring_pin_spacing_limit_ok(cfg: GearInputs) -> bool:
    """Check adjacent ring pins do not overlap."""

    p = pin_count(cfg)
    half_spacing = cfg.ring_pitch_radius_mm * math.sin(math.pi / p)
    return cfg.ring_pin_radius_mm + cfg.radial_clearance_mm <= half_spacing


def cycloidal_profile_points(
    eccentricity_mm: float,
    cfg: GearInputs,
) -> list[tuple[float, float]]:
    """Return approximate cycloidal disk profile points.

    This uses the common ring-pin envelope form for a pin-wheel cycloidal disk.
    It is accurate enough for curvature screening.
    Your CAD/ANSYS model remains the final authority.
    """

    p = pin_count(cfg)
    r = cfg.ring_pitch_radius_mm
    rr = cfg.ring_pin_radius_mm
    e = eccentricity_mm
    points: list[tuple[float, float]] = []

    for i in range(cfg.profile_points):
        theta = 2.0 * math.pi * i / cfg.profile_points
        denominator = r / (e * p) - math.cos((1.0 - p) * theta)
        alpha = math.atan2(math.sin((1.0 - p) * theta), denominator)

        x = r * math.cos(theta) - rr * math.cos(theta + alpha) - e * math.cos(
            p * theta
        )
        y = r * math.sin(theta) - rr * math.sin(theta + alpha) - e * math.sin(
            p * theta
        )
        points.append((x, y))

    return points


def curvature_radii_mm(points: list[tuple[float, float]]) -> list[float]:
    """Numerically estimate local radius of curvature for a closed profile."""

    n = len(points)
    radii: list[float] = []
    for i in range(n):
        x_prev, y_prev = points[(i - 1) % n]
        x_now, y_now = points[i]
        x_next, y_next = points[(i + 1) % n]

        dx = 0.5 * (x_next - x_prev)
        dy = 0.5 * (y_next - y_prev)
        ddx = x_next - 2.0 * x_now + x_prev
        ddy = y_next - 2.0 * y_now + y_prev

        numerator = abs(dx * ddy - dy * ddx)
        denominator = (dx * dx + dy * dy) ** 1.5
        if numerator <= 1e-12 or denominator <= 1e-12:
            continue

        radius = denominator / numerator
        if math.isfinite(radius) and radius > 0.0:
            radii.append(radius)

    return radii


def robust_min_curvature_radius_mm(points: list[tuple[float, float]]) -> float:
    """Return a robust small curvature radius without one-point noise spikes."""

    radii = curvature_radii_mm(points)
    if not radii:
        return 0.0

    radii.sort()
    percentile_index = max(0, int(0.02 * len(radii)) - 1)
    return radii[percentile_index]


def contact_force_per_pin_n(eccentricity_mm: float, cfg: GearInputs) -> float:
    """Estimate ring contact force per loaded pin.

    Two simple load paths are considered:
      1. Output torque reacted at ring pitch radius: F = T / R.
      2. Eccentric bearing reaction from motor-side torque: F = T_in / e.

    The larger of the two is used, multiplied by shock factor, then divided by
    the number of loaded ring pins.
    """

    ring_radius_m = cfg.ring_pitch_radius_mm / 1000.0
    eccentricity_m = eccentricity_mm / 1000.0

    force_from_output_radius = cfg.output_torque_nm / ring_radius_m
    input_torque_nm = cfg.output_torque_nm / (
        cfg.gear_ratio * cfg.gearbox_efficiency
    )
    force_from_eccentric = input_torque_nm / eccentricity_m

    total_contact_force = cfg.shock_factor * max(
        force_from_output_radius,
        force_from_eccentric,
    )
    return total_contact_force / cfg.effective_loaded_ring_pins


def hertz_peak_pressure_mpa(
    force_per_pin_n: float,
    cycloid_curvature_radius_mm: float,
    cfg: GearInputs,
) -> tuple[float, float]:
    """Estimate Hertz line-contact pressure and effective contact radius."""

    if cycloid_curvature_radius_mm <= 0.0:
        return float("inf"), 0.0

    effective_radius_mm = 1.0 / (
        1.0 / cfg.ring_pin_radius_mm + 1.0 / cycloid_curvature_radius_mm
    )
    e_prime_mpa = equivalent_modulus_mpa(cfg)

    pressure_mpa = math.sqrt(
        e_prime_mpa
        * force_per_pin_n
        / (math.pi * cfg.face_width_mm * effective_radius_mm)
    )
    return pressure_mpa, effective_radius_mm


def evaluate_candidate(eccentricity_mm: float, cfg: GearInputs) -> CandidateResult:
    """Evaluate one eccentricity candidate."""

    p = pin_count(cfg)
    curtate_ratio = p * eccentricity_mm / cfg.ring_pitch_radius_mm

    if not ring_pin_spacing_limit_ok(cfg):
        return CandidateResult(
            eccentricity_mm,
            curtate_ratio,
            float("inf"),
            0.0,
            0.0,
            float("inf"),
            False,
            "ring pins overlap or clearance is too small",
        )

    if eccentricity_mm > output_hole_eccentricity_limit(cfg):
        return CandidateResult(
            eccentricity_mm,
            curtate_ratio,
            float("inf"),
            0.0,
            0.0,
            float("inf"),
            False,
            "output pin hole does not clear eccentric orbit",
        )

    if not (cfg.curtate_ratio_min <= curtate_ratio <= cfg.curtate_ratio_max):
        return CandidateResult(
            eccentricity_mm,
            curtate_ratio,
            float("inf"),
            0.0,
            0.0,
            float("inf"),
            False,
            "curtate ratio outside recommended range",
        )

    points = cycloidal_profile_points(eccentricity_mm, cfg)
    min_radius = robust_min_curvature_radius_mm(points)
    force = contact_force_per_pin_n(eccentricity_mm, cfg)
    pressure, effective_radius = hertz_peak_pressure_mpa(
        force, min_radius, cfg)

    return CandidateResult(
        eccentricity_mm=eccentricity_mm,
        curtate_ratio=curtate_ratio,
        force_per_pin_n=force,
        profile_min_curvature_radius_mm=min_radius,
        effective_contact_radius_mm=effective_radius,
        peak_hertz_pressure_mpa=pressure,
        feasible=True,
        reason="ok",
    )


def optimize(cfg: GearInputs) -> list[CandidateResult]:
    """Grid-search eccentricity."""

    results: list[CandidateResult] = []
    count = max(cfg.eccentricity_samples, 2)
    for i in range(count):
        ratio = i / (count - 1)
        eccentricity = cfg.eccentricity_min_mm + ratio * (
            cfg.eccentricity_max_mm - cfg.eccentricity_min_mm
        )
        results.append(evaluate_candidate(eccentricity, cfg))
    return results


def best_result(results: list[CandidateResult]) -> CandidateResult:
    feasible = [item for item in results if item.feasible]
    if not feasible:
        raise RuntimeError("No feasible eccentricity candidates found.")
    return min(feasible, key=lambda item: item.peak_hertz_pressure_mpa)


def write_csv(results: list[CandidateResult], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "eccentricity_mm",
                "curtate_ratio",
                "force_per_pin_n",
                "profile_min_curvature_radius_mm",
                "effective_contact_radius_mm",
                "peak_hertz_pressure_mpa",
                "feasible",
                "reason",
            ]
        )
        for item in results:
            writer.writerow(
                [
                    f"{item.eccentricity_mm:.6f}",
                    f"{item.curtate_ratio:.6f}",
                    f"{item.force_per_pin_n:.6f}",
                    f"{item.profile_min_curvature_radius_mm:.6f}",
                    f"{item.effective_contact_radius_mm:.6f}",
                    f"{item.peak_hertz_pressure_mpa:.6f}",
                    item.feasible,
                    item.reason,
                ]
            )


def print_summary(cfg: GearInputs, results: list[CandidateResult]) -> None:
    best = best_result(results)
    feasible_count = sum(1 for item in results if item.feasible)
    print("=" * 72)
    print("Cycloidal eccentricity screening result")
    print("=" * 72)
    print(f"Gear ratio:                 {cfg.gear_ratio}:1")
    print(f"Ring pin count:             {pin_count(cfg)}")
    print(f"Ring pitch radius:          {cfg.ring_pitch_radius_mm:.3f} mm")
    print(f"Ring pin radius:            {cfg.ring_pin_radius_mm:.3f} mm")
    print(f"Face width:                 {cfg.face_width_mm:.3f} mm")
    print(f"Output torque:              {cfg.output_torque_nm:.3f} Nm")
    print(f"Shock factor:               {cfg.shock_factor:.3f}")
    print(f"Effective loaded pins:      {cfg.effective_loaded_ring_pins:.3f}")
    print(
        f"Output hole e limit:        {output_hole_eccentricity_limit(cfg):.3f} mm")
    print(f"Feasible candidates:        {feasible_count}/{len(results)}")
    print("-" * 72)
    print(f"Best eccentricity:          {best.eccentricity_mm:.3f} mm")
    print(f"Best curtate ratio lambda:  {best.curtate_ratio:.3f}")
    print(f"Ring force per loaded pin:  {best.force_per_pin_n:.1f} N")
    print(
        "Min profile curvature:      "
        f"{best.profile_min_curvature_radius_mm:.3f} mm"
    )
    print(
        f"Effective contact radius:   {best.effective_contact_radius_mm:.3f} mm")
    print(
        f"Estimated Hertz pressure:   {best.peak_hertz_pressure_mpa:.1f} MPa")
    print("=" * 72)


def main() -> None:
    cfg = CONFIG
    results = optimize(cfg)
    best = best_result(results)

    csv_path = Path("cycloidal_optimization_results.csv")

    print_summary(cfg, results)
    write_csv(results, csv_path)

    print(f"CSV written to:  {csv_path.resolve()}")


if __name__ == "__main__":
    main()




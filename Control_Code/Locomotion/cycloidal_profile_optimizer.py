"""Fast cycloidal drive profile and eccentricity screening tool.

This is an analytical pre-ANSYS optimizer. It estimates ring-pin contact load,
local profile curvature, and Hertz line-contact pressure for a cycloidal disk.

Edit the CONFIG values near the top, then run:

    python cycloidal_profile_optimizer.py

The script writes:

    cycloidal_optimization_results.csv
    cycloidal_optimization_result.png

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
from statistics import median


@dataclass(frozen=True)
class GearInputs:
    """Edit these values to match your CAD."""

    # Basic cycloidal geometry.
    gear_ratio: int = 17
    ring_pitch_radius_mm: float = 50.0
    ring_pin_radius_mm: float = 2.5
    face_width_mm: float = 10.0

    # Output pin/hole geometry. The hole must clear the eccentric orbit.
    # Constraint used: e <= output_pin_hole_radius - output_pin_radius - clearance.
    output_pin_hole_radius_mm: float = 5.0
    output_pin_radius_mm: float = 3.0
    radial_clearance_mm: float = 0.25

    # Robot actuator load.
    output_torque_nm: float = 25.0
    shock_factor: float = 2.0
    gearbox_efficiency: float = 0.85
    effective_loaded_ring_pins: float = 3.0

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
    It is accurate enough for profile visualization and curvature screening.
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
    pressure, effective_radius = hertz_peak_pressure_mpa(force, min_radius, cfg)

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


def _finite_values(values: list[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def _svg_polyline(
    points: list[tuple[float, float]],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    panel: tuple[float, float, float, float],
) -> str:
    x0, y0, width, height = panel
    dx = max(x_max - x_min, 1e-9)
    dy = max(y_max - y_min, 1e-9)
    coords = []
    for x_value, y_value in points:
        sx = x0 + (x_value - x_min) / dx * width
        sy = y0 + height - (y_value - y_min) / dy * height
        coords.append(f"{sx:.1f},{sy:.1f}")
    return " ".join(coords)


def write_svg_results(
    results: list[CandidateResult],
    best: CandidateResult,
    cfg: GearInputs,
    path: Path,
) -> None:
    """Write a dependency-free SVG visualization."""

    width = 1200
    height = 900
    margin = 70
    panel_w = 470
    panel_h = 310
    profile_panel = (margin, 105, panel_w, panel_h)
    stress_panel = (650, 105, panel_w, panel_h)
    force_panel = (margin, 520, panel_w, panel_h)
    radius_panel = (650, 520, panel_w, panel_h)

    profile = cycloidal_profile_points(best.eccentricity_mm, cfg)
    xs = [point[0] for point in profile]
    ys = [point[1] for point in profile]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    cx = 0.5 * (max(xs) + min(xs))
    cy = 0.5 * (max(ys) + min(ys))
    pad = 0.12 * span
    x_min = cx - 0.5 * span - pad
    x_max = cx + 0.5 * span + pad
    y_min = cy - 0.5 * span - pad
    y_max = cy + 0.5 * span + pad

    e_values = [item.eccentricity_mm for item in results]
    stress_values = [
        item.peak_hertz_pressure_mpa if item.feasible else math.nan
        for item in results
    ]
    force_values = [
        item.force_per_pin_n if item.feasible else math.nan for item in results
    ]
    radius_values = [
        item.profile_min_curvature_radius_mm if item.feasible else math.nan
        for item in results
    ]

    def chart_points(values: list[float]) -> tuple[list[tuple[float, float]], float, float]:
        finite = _finite_values(values)
        if not finite:
            return [], 0.0, 1.0
        y_low = min(finite)
        y_high = max(finite)
        if abs(y_high - y_low) < 1e-9:
            y_high = y_low + 1.0
        points = [
            (e_value, value)
            for e_value, value in zip(e_values, values)
            if math.isfinite(value)
        ]
        return points, y_low, y_high

    def panel_box(panel: tuple[float, float, float, float]) -> str:
        x0, y0, panel_width, panel_height = panel
        return (
            f'<rect x="{x0}" y="{y0}" width="{panel_width}" '
            f'height="{panel_height}" fill="white" stroke="#333" />'
        )

    def chart_svg(
        title: str,
        points: list[tuple[float, float]],
        y_low: float,
        y_high: float,
        panel: tuple[float, float, float, float],
        color: str,
        y_unit: str,
    ) -> list[str]:
        x_low = min(e_values)
        x_high = max(e_values)
        line = _svg_polyline(points, x_low, x_high, y_low, y_high, panel)
        x0, y0, panel_width, panel_height = panel
        best_x = x0 + (best.eccentricity_mm - x_low) / (x_high - x_low) * panel_width
        return [
            panel_box(panel),
            f'<text x="{x0}" y="{y0 - 16}" font-size="18" '
            f'font-weight="700">{title}</text>',
            f'<polyline points="{line}" fill="none" stroke="{color}" '
            'stroke-width="3" />',
            f'<line x1="{best_x:.1f}" y1="{y0}" x2="{best_x:.1f}" '
            f'y2="{y0 + panel_height}" stroke="#111" stroke-dasharray="5,5" />',
            f'<text x="{x0}" y="{y0 + panel_height + 28}" '
            'font-size="13">eccentricity [mm]</text>',
            f'<text x="{x0 + panel_width - 165}" y="{y0 + panel_height + 28}" '
            f'font-size="13">{y_low:.1f} to {y_high:.1f} {y_unit}</text>',
        ]

    stress_points, stress_low, stress_high = chart_points(stress_values)
    force_points, force_low, force_high = chart_points(force_values)
    radius_points, radius_low, radius_high = chart_points(radius_values)

    profile_line = _svg_polyline(profile, x_min, x_max, y_min, y_max, profile_panel)
    ring_circle_radius_px = (
        cfg.ring_pitch_radius_mm / (x_max - x_min) * profile_panel[2]
    )
    ring_pin_radius_px = cfg.ring_pin_radius_mm / (x_max - x_min) * profile_panel[2]
    ring_center_x = profile_panel[0] + (0.0 - x_min) / (x_max - x_min) * profile_panel[2]
    ring_center_y = (
        profile_panel[1]
        + profile_panel[3]
        - (0.0 - y_min) / (y_max - y_min) * profile_panel[3]
    )

    svg: list[str] = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f4f6f8" />',
        '<text x="60" y="48" font-size="26" font-weight="800">'
        "Cycloidal Gear Fast Eccentricity Screening</text>",
        f'<text x="60" y="76" font-size="15">Best e = '
        f'{best.eccentricity_mm:.3f} mm, pressure = '
        f'{best.peak_hertz_pressure_mpa:.1f} MPa, torque = '
        f'{cfg.output_torque_nm:.1f} Nm</text>',
        panel_box(profile_panel),
        f'<text x="{profile_panel[0]}" y="{profile_panel[1] - 16}" '
        f'font-size="18" font-weight="700">Best cycloidal profile</text>',
        f'<circle cx="{ring_center_x:.1f}" cy="{ring_center_y:.1f}" '
        f'r="{ring_circle_radius_px:.1f}" fill="none" stroke="#888" '
        'stroke-dasharray="5,5" />',
    ]

    for i in range(pin_count(cfg)):
        theta = 2.0 * math.pi * i / pin_count(cfg)
        x_pin = cfg.ring_pitch_radius_mm * math.cos(theta)
        y_pin = cfg.ring_pitch_radius_mm * math.sin(theta)
        sx = profile_panel[0] + (x_pin - x_min) / (x_max - x_min) * profile_panel[2]
        sy = (
            profile_panel[1]
            + profile_panel[3]
            - (y_pin - y_min) / (y_max - y_min) * profile_panel[3]
        )
        svg.append(
            f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{ring_pin_radius_px:.1f}" '
            'fill="none" stroke="#3568a8" stroke-width="1" />'
        )

    svg.extend(
        [
            f'<polyline points="{profile_line}" fill="none" stroke="#d33" '
            'stroke-width="2.5" />',
            f'<text x="{profile_panel[0]}" y="{profile_panel[1] + profile_panel[3] + 28}" '
            'font-size="13">x-y profile and ring pins [mm]</text>',
        ]
    )
    svg.extend(
        chart_svg(
            "Estimated Hertz pressure",
            stress_points,
            stress_low,
            stress_high,
            stress_panel,
            "#c0392b",
            "MPa",
        )
    )
    svg.extend(
        chart_svg(
            "Ring load per loaded pin",
            force_points,
            force_low,
            force_high,
            force_panel,
            "#1f77b4",
            "N",
        )
    )
    svg.extend(
        chart_svg(
            "Minimum profile curvature radius",
            radius_points,
            radius_low,
            radius_high,
            radius_panel,
            "#2e8b57",
            "mm",
        )
    )
    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")


def plot_results(
    results: list[CandidateResult],
    best: CandidateResult,
    cfg: GearInputs,
    path: Path,
) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
    except ImportError as exc:
        print("matplotlib is not installed, so no plot was generated.")
        print("Install it with: python -m pip install matplotlib")
        print(exc)
        return

    feasible = [item for item in results if item.feasible]
    x_all = [item.eccentricity_mm for item in results]
    y_all = [
        item.peak_hertz_pressure_mpa if item.feasible else math.nan
        for item in results
    ]
    force_all = [
        item.force_per_pin_n if item.feasible else math.nan for item in results
    ]
    radius_all = [
        item.profile_min_curvature_radius_mm if item.feasible else math.nan
        for item in results
    ]

    profile = cycloidal_profile_points(best.eccentricity_mm, cfg)
    profile_x = [point[0] for point in profile]
    profile_y = [point[1] for point in profile]

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    ax_profile, ax_stress, ax_force, ax_radius = axes.ravel()

    ax_profile.plot(profile_x, profile_y, linewidth=1.4, label="Cycloidal disk")
    ax_profile.add_patch(
        Circle(
            (0, 0),
            cfg.ring_pitch_radius_mm,
            fill=False,
            linestyle="--",
            linewidth=1.0,
            label="Ring pin pitch radius",
        )
    )
    for i in range(pin_count(cfg)):
        theta = 2.0 * math.pi * i / pin_count(cfg)
        x_pin = cfg.ring_pitch_radius_mm * math.cos(theta)
        y_pin = cfg.ring_pitch_radius_mm * math.sin(theta)
        ax_profile.add_patch(
            Circle(
                (x_pin, y_pin),
                cfg.ring_pin_radius_mm,
                fill=False,
                linewidth=0.7,
                alpha=0.7,
            )
        )
    ax_profile.set_aspect("equal", adjustable="box")
    ax_profile.set_title(f"Best profile, e = {best.eccentricity_mm:.3f} mm")
    ax_profile.set_xlabel("x [mm]")
    ax_profile.set_ylabel("y [mm]")
    ax_profile.grid(True, alpha=0.25)
    ax_profile.legend(loc="upper right")

    ax_stress.plot(x_all, y_all, color="tab:red", linewidth=1.8)
    ax_stress.scatter(
        [best.eccentricity_mm],
        [best.peak_hertz_pressure_mpa],
        color="black",
        zorder=5,
    )
    ax_stress.set_title("Estimated Hertz pressure vs eccentricity")
    ax_stress.set_xlabel("eccentricity [mm]")
    ax_stress.set_ylabel("peak pressure [MPa]")
    ax_stress.grid(True, alpha=0.25)

    ax_force.plot(x_all, force_all, color="tab:blue", linewidth=1.8)
    ax_force.set_title("Estimated ring load per loaded pin")
    ax_force.set_xlabel("eccentricity [mm]")
    ax_force.set_ylabel("force per pin [N]")
    ax_force.grid(True, alpha=0.25)

    ax_radius.plot(x_all, radius_all, color="tab:green", linewidth=1.8)
    ax_radius.set_title("Robust minimum profile curvature radius")
    ax_radius.set_xlabel("eccentricity [mm]")
    ax_radius.set_ylabel("radius [mm]")
    ax_radius.grid(True, alpha=0.25)

    if feasible:
        pressures = [item.peak_hertz_pressure_mpa for item in feasible]
        fig.suptitle(
            "Cycloidal eccentricity screening\n"
            f"Torque={cfg.output_torque_nm:.1f} Nm, "
            f"ratio={cfg.gear_ratio}:1, "
            f"median feasible pressure={median(pressures):.1f} MPa",
            fontsize=13,
        )

    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


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
    print(f"Output hole e limit:        {output_hole_eccentricity_limit(cfg):.3f} mm")
    print(f"Feasible candidates:        {feasible_count}/{len(results)}")
    print("-" * 72)
    print(f"Best eccentricity:          {best.eccentricity_mm:.3f} mm")
    print(f"Best curtate ratio lambda:  {best.curtate_ratio:.3f}")
    print(f"Ring force per loaded pin:  {best.force_per_pin_n:.1f} N")
    print(
        "Min profile curvature:      "
        f"{best.profile_min_curvature_radius_mm:.3f} mm"
    )
    print(f"Effective contact radius:   {best.effective_contact_radius_mm:.3f} mm")
    print(f"Estimated Hertz pressure:   {best.peak_hertz_pressure_mpa:.1f} MPa")
    print("=" * 72)


def main() -> None:
    cfg = CONFIG
    results = optimize(cfg)
    best = best_result(results)

    csv_path = Path("cycloidal_optimization_results.csv")
    png_path = Path("cycloidal_optimization_result.png")
    svg_path = Path("cycloidal_optimization_result.svg")

    print_summary(cfg, results)
    write_csv(results, csv_path)
    write_svg_results(results, best, cfg, svg_path)
    plot_results(results, best, cfg, png_path)

    print(f"CSV written to:  {csv_path.resolve()}")
    print(f"SVG written to:  {svg_path.resolve()}")
    if png_path.exists():
        print(f"PNG written to:  {png_path.resolve()}")


if __name__ == "__main__":
    main()

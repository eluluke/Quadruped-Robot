"""
ik_solver.py
============
Inverse kinematics for a 3-motor quadruped leg.

Physical layout
---------------
  Motor 1 — SHOULDER  (abduction / adduction, rotates the whole leg in/out)
  Motor 2 — THIGH     (hip pitch, swings the thigh forward/back)
  Motor 3 — SHIN      (knee pitch, driven by a push-rod that runs through
                        the thigh — so the motor angle IS the absolute shin
                        angle in the world frame, not relative to the thigh)

Coordinate frames
-----------------
  We solve in the SAGITTAL plane (forward X, up Y, lateral Z).

  2-D IK (thigh + shin) is solved first in the plane of the leg.
  The shoulder tilt rotates the plane about the hip Z axis and is
  solved separately from a lateral (Z) target offset.

  All angles in RADIANS.

  Hip origin is at (0, 0).
  +X  = forward
  +Y  = up
  +Z  = outward (right side of robot)

Push-rod / shin note
--------------------
  Because the rod runs THROUGH the thigh, the shin motor controls the
  absolute angle of the shin segment in the world frame (or relative to
  the body), NOT the angle relative to the thigh.

  Let:
    θ_thigh  = thigh angle from +X axis  (motor 2 angle)
    θ_shin_w = shin absolute world angle  (what the push-rod motor holds)
    θ_knee   = included angle at knee     = θ_shin_w − θ_thigh  (for display)

  The IK solver computes θ_thigh and θ_shin_w.
  θ_knee is returned for display/diagnostics only.

  If your linkage behaves differently (shin angle IS relative to thigh),
  set  SHIN_IS_ABSOLUTE = False  in config.py — the solver will add
  the thigh angle back before transmitting.
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class IKResult:
    # ── angles sent to motors (radians) ──────────────────────────────────────
    shoulder_rad: float   # motor 1 — lateral tilt (+ = outward)
    thigh_rad:    float   # motor 2 — thigh angle from +X in sagittal plane
    shin_rad:     float   # motor 3 — shin absolute world angle (or relative, per config)

    # ── diagnostic ───────────────────────────────────────────────────────────
    knee_angle_rad: float   # included knee angle (thigh to shin), for display
    reach:          float   # 2-D distance from hip to foot in sagittal plane

    # ── joint positions (sagittal plane, mm) ─────────────────────────────────
    j_hip:   tuple[float, float] = (0.0, 0.0)   # hip    (always origin in sagittal)
    j_knee:  tuple[float, float] = (0.0, 0.0)   # knee
    j_foot:  tuple[float, float] = (0.0, 0.0)   # foot (end-effector)

    @property
    def shoulder_deg(self)  -> float: return math.degrees(self.shoulder_rad)
    @property
    def thigh_deg(self)     -> float: return math.degrees(self.thigh_rad)
    @property
    def shin_deg(self)      -> float: return math.degrees(self.shin_rad)
    @property
    def knee_angle_deg(self)-> float: return math.degrees(self.knee_angle_rad)

    # aliases so the GUI angle-panel code still works with t1/t2/t3
    @property
    def t1(self) -> float: return self.shoulder_rad
    @property
    def t2(self) -> float: return self.thigh_rad
    @property
    def t3(self) -> float: return self.shin_rad
    @property
    def t1_deg(self) -> float: return self.shoulder_deg
    @property
    def t2_deg(self) -> float: return self.thigh_deg
    @property
    def t3_deg(self) -> float: return self.shin_deg

    # for the math panel
    @property
    def wrist_x(self) -> float: return self.j_knee[0]
    @property
    def wrist_y(self) -> float: return self.j_knee[1]
    @property
    def wrist_dist(self) -> float: return math.hypot(*self.j_knee)
    @property
    def cos_knee(self) -> float:
        try:
            return math.cos(self.knee_angle_rad)
        except Exception:
            return 0.0
    @property
    def knee_ang(self) -> float: return abs(self.knee_angle_rad)
    @property
    def alpha(self) -> float: return math.atan2(self.j_foot[1], self.j_foot[0])
    @property
    def alpha2(self) -> float: return self.knee_angle_rad

    # for the canvas (3-joint forward kinematics in 2-D sagittal plane)
    @property
    def j0(self) -> tuple[float,float]: return self.j_hip
    @property
    def j1(self) -> tuple[float,float]: return self.j_knee
    @property
    def j2(self) -> tuple[float,float]: return self.j_foot
    @property
    def j3(self) -> tuple[float,float]: return self.j_foot  # foot = end-effector


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def solve(
    tx: float,           # target X in sagittal plane (mm)
    ty: float,           # target Y in sagittal plane (mm)
    L_thigh: float,      # thigh link length (mm)   = config.L1
    L_shin:  float,      # shin  link length (mm)   = config.L2
    L_foot:  float = 0.0,# foot  stub length (mm)   = config.L3  (0 if no foot link)
    solution: int = 0,   # 0 = knee forward/down, 1 = knee back/up
    shin_is_absolute: bool = True,  # True = push-rod holds absolute shin angle
    tz: float = 0.0,     # lateral target offset (mm) for shoulder angle
    shoulder_to_hip: float = 0.0,   # lateral distance from shoulder pivot to hip (mm)
) -> Optional[IKResult]:
    """
    Solve IK for the 3-motor leg.

    Parameters
    ----------
    tx, ty            : foot target in sagittal plane (mm)
    L_thigh, L_shin   : link lengths (mm)
    L_foot            : optional foot stub extending from knee in shin direction
    solution          : 0 = elbow-down (knee below line hip→foot),
                        1 = elbow-up   (knee above)
    shin_is_absolute  : if True, motor 3 angle = absolute shin world angle
                        if False, motor 3 angle = shin angle relative to thigh
    tz                : lateral offset of foot from hip column (mm)
    shoulder_to_hip   : lateral distance from shoulder axis to hip (mm)

    Returns None if target is unreachable.
    """
    # ── Shoulder / lateral angle ─────────────────────────────────────────────
    # The shoulder tilts the leg plane. For a simple single-axis shoulder:
    #   shoulder_angle = atan2(tz, 0)  — you can extend this for full 3D
    shoulder_rad = math.atan2(tz, shoulder_to_hip) if (tz != 0 or shoulder_to_hip != 0) else 0.0

    # ── 2-D sagittal IK  (thigh + shin) ──────────────────────────────────────
    # If there's a foot stub, we first back-project through it to find
    # the effective wrist (= ankle) point. For now we assume the foot
    # is always vertical (pointing straight down) — adjust if yours differs.
    foot_angle = -math.pi / 2   # foot points straight down in world frame
    wx = tx - L_foot * math.cos(foot_angle)
    wy = ty - L_foot * math.sin(foot_angle)

    wdist = math.hypot(wx, wy)

    # Reachability
    max_r = L_thigh + L_shin
    min_r = abs(L_thigh - L_shin)
    if wdist > max_r or (wdist < min_r and wdist > 1e-6):
        return None

    # Law of cosines for knee angle
    cos_knee = _clamp(
        (L_thigh**2 + L_shin**2 - wdist**2) / (2 * L_thigh * L_shin),
        -1.0, 1.0
    )
    knee_ang = math.acos(cos_knee)   # always positive

    # Hip angle
    alpha  = math.atan2(wy, wx)
    cos_a2 = _clamp(
        (L_thigh**2 + wdist**2 - L_shin**2) / (2 * L_thigh * wdist),
        -1.0, 1.0
    )
    alpha2 = math.acos(cos_a2)

    if solution == 0:       # knee below / elbow-down
        thigh_rad     = alpha - alpha2
        knee_angle    = knee_ang          # supplementary in world sense
        shin_world    = thigh_rad + (math.pi - knee_ang)
    else:                   # knee above / elbow-up
        thigh_rad     = alpha + alpha2
        knee_angle    = -knee_ang
        shin_world    = thigh_rad - (math.pi - knee_ang)

    # Motor 3 angle: absolute world angle OR relative to thigh
    if shin_is_absolute:
        shin_rad = shin_world
    else:
        shin_rad = shin_world - thigh_rad   # = ±(π − knee_ang)

    # ── Forward kinematics to get joint positions (sagittal plane) ────────────
    kx = L_thigh * math.cos(thigh_rad)
    ky = L_thigh * math.sin(thigh_rad)
    fx = kx + L_shin * math.cos(shin_world)
    fy = ky + L_shin * math.sin(shin_world)

    return IKResult(
        shoulder_rad    = shoulder_rad,
        thigh_rad       = thigh_rad,
        shin_rad        = shin_rad,
        knee_angle_rad  = knee_angle,
        reach           = wdist,
        j_hip           = (0.0, 0.0),
        j_knee          = (kx, ky),
        j_foot          = (fx, fy),
    )

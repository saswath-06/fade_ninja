"""EEZYbotARM MK1 FK / IK (PHONE_POSE_TDD Phase 2).

Simplified 3-DOF RRR model (published defaults — remeasure after print):

    L0  shoulder height above base origin (mm)
    L2  upper-arm length, shoulder -> elbow (mm)
    L3  forearm length, elbow -> tip (mm)

Joint conventions (degrees):
    q1  base yaw: 0 = tip in +X, positive CCW toward +Y (top view)
    q2  shoulder: 0 = upper arm horizontal forward, + = tip up
    q3  elbow relative to upper arm: 0 = fully extended, negative = fold in

FK:
    r = L2*cos(q2) + L3*cos(q2+q3)
    x = r * cos(q1)
    y = r * sin(q1)
    z = L0 + L2*sin(q2) + L3*sin(q2+q3)

IK returns None on geometric miss or joint-limit miss — never clamps XYZ.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# --- T2.0 named constants (published defaults; unverified on hardware) -----

L0_MM = 55.0
L2_MM = 80.0
L3_MM = 80.0

# Mid-workspace home: forward, slightly up, elbow folded
Q_HOME = (0.0, 30.0, -60.0)  # q1, q2, q3 degrees

# Hand-computed P_HOME = FK(Q_HOME) with the formulas above:
# q2=30, q2+q3=-30
# r = 80*cos30 + 80*cos(-30) = 80*(√3/2)*2 = 80*√3 ≈ 138.5640646055
# z = 55 + 80*sin30 + 80*sin(-30) = 55 + 40 - 40 = 55
P_HOME = (L2_MM * math.sqrt(3.0), 0.0, L0_MM)  # (~138.564, 0, 55)

Q1_LIMITS = (-90.0, 90.0)
Q2_LIMITS = (0.0, 90.0)
Q3_LIMITS = (-135.0, 0.0)

Q4_LIMITS = (-45.0, 45.0)  # wrist tilt from phone pitch
Q4_PER_PITCH = 1.0         # deg q4 per deg phone pitch


@dataclass(frozen=True)
class Joints:
    q1: float
    q2: float
    q3: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.q1, self.q2, self.q3)


def _in_limits(q1: float, q2: float, q3: float) -> bool:
    return (Q1_LIMITS[0] <= q1 <= Q1_LIMITS[1]
            and Q2_LIMITS[0] <= q2 <= Q2_LIMITS[1]
            and Q3_LIMITS[0] <= q3 <= Q3_LIMITS[1])


def forward_kinematics(q1_deg: float, q2_deg: float, q3_deg: float,
                       *, l0: float = L0_MM, l2: float = L2_MM,
                       l3: float = L3_MM) -> tuple[float, float, float]:
    q1 = math.radians(q1_deg)
    q2 = math.radians(q2_deg)
    q23 = math.radians(q2_deg + q3_deg)
    r = l2 * math.cos(q2) + l3 * math.cos(q23)
    x = r * math.cos(q1)
    y = r * math.sin(q1)
    z = l0 + l2 * math.sin(q2) + l3 * math.sin(q23)
    return (x, y, z)


def inverse_kinematics(x: float, y: float, z: float,
                       *, l0: float = L0_MM, l2: float = L2_MM,
                       l3: float = L3_MM) -> Joints | None:
    """Return joints or None if geometrically unreachable or outside limits."""
    r = math.hypot(x, y)
    zz = z - l0
    d2 = r * r + zz * zz
    # law of cosines for elbow
    cos_q3 = (d2 - l2 * l2 - l3 * l3) / (2.0 * l2 * l3)
    if cos_q3 < -1.0 - 1e-9 or cos_q3 > 1.0 + 1e-9:
        return None
    cos_q3 = max(-1.0, min(1.0, cos_q3))
    # Prefer the negative (folded) elbow solution to match Q3_LIMITS
    q3 = -math.degrees(math.acos(cos_q3))

    # shoulder from triangle
    k1 = l2 + l3 * math.cos(math.radians(q3))
    k2 = l3 * math.sin(math.radians(q3))
    q2 = math.degrees(math.atan2(zz, r) - math.atan2(k2, k1))

    q1 = math.degrees(math.atan2(y, x)) if r > 1e-9 else 0.0

    if not _in_limits(q1, q2, q3):
        return None
    return Joints(q1, q2, q3)


def pitch_to_q4(pitch_deg: float) -> float:
    """Map phone pitch to wrist tilt; clamp to Q4_LIMITS only."""
    q4 = pitch_deg * Q4_PER_PITCH
    return max(Q4_LIMITS[0], min(Q4_LIMITS[1], q4))


def sample_reachable_workspace(n: int = 40, seed: int = 0
                               ) -> list[tuple[float, float, float]]:
    """Joint-space samples inside limits, converted via FK (T2.2)."""
    import random
    rng = random.Random(seed)
    pts = []
    for _ in range(n):
        q1 = rng.uniform(*Q1_LIMITS)
        q2 = rng.uniform(*Q2_LIMITS)
        q3 = rng.uniform(*Q3_LIMITS)
        pts.append(forward_kinematics(q1, q2, q3))
    return pts

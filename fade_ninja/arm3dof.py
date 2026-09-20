"""The real arm: base yaw, arm pitch, razor tilt.

    q1  base rotation      swings the whole arm left and right
    q2  arm pitch          swings the tip forward/down and back/up on an arc
    q3  razor tilt         flicks the blade up and down; sets hair length

The arm is a rigid link on a servo horn, so the tip sits at a FIXED distance
from the pitch pivot: reach and height are one coupled motion along an arc,
not two independent axes. Two angles therefore place the tip on a sphere, and
the third only aims the tool — the same shape as the rail machine this
project started from, which is why the motion mapping below is the one
already proven there rather than something new.

Every dimension lives in `ArmSpec`. The defaults are placeholders; measure
the real arm and correct them before driving it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .pose_teleop import quat_forward, quat_pitch_deg

TRACK_NORMAL = 0
JOINT_NAMES = ("q1", "q2", "q3")


@dataclass
class ArmSpec:
    """MEASURE THESE. Every number is a guess until the arm exists."""
    arm_len_mm: float = 150.0        # pitch pivot -> razor tip
    pivot_h_mm: float = 60.0         # pitch pivot above the base plate
    q1_limits: tuple[float, float] = (-90.0, 90.0)     # base rotation
    q2_limits: tuple[float, float] = (-20.0, 80.0)     # arm pitch
    q3_limits: tuple[float, float] = (-60.0, 60.0)     # razor tilt
    home: tuple[float, float, float] = (0.0, 30.0, 0.0)

    def limits(self) -> tuple[tuple[float, float], ...]:
        return (self.q1_limits, self.q2_limits, self.q3_limits)


DEFAULT = ArmSpec()


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def forward_kinematics(q1: float, q2: float,
                       spec: ArmSpec = DEFAULT) -> tuple[float, float, float]:
    """Joints -> razor tip (mm). +Y is up; q1 sweeps from +X toward +Z."""
    a, p = math.radians(q1), math.radians(q2)
    r = spec.arm_len_mm * math.cos(p)
    return (r * math.cos(a),
            spec.pivot_h_mm + spec.arm_len_mm * math.sin(p),
            r * math.sin(a))


def inverse_kinematics(x: float, y: float, z: float,
                       spec: ArmSpec = DEFAULT) -> tuple[float, float, bool]:
    """Tip position -> (q1, q2, out_of_reach).

    The arm length is fixed, so only the DIRECTION from the pivot is
    achievable: a target is projected onto the arc rather than refused, and
    the arm reaches as far toward it as the joints allow.
    """
    dy = y - spec.pivot_h_mm
    r = math.hypot(x, z)
    out = False
    if r < 1e-9 and abs(dy) < 1e-9:
        return (0.0, clamp(0.0, *spec.q2_limits), True)

    q1 = math.degrees(math.atan2(z, x))
    q2 = math.degrees(math.atan2(dy, r))

    c1 = clamp(q1, *spec.q1_limits)
    c2 = clamp(q2, *spec.q2_limits)
    if c1 != q1 or c2 != q2:
        out = True
    return (c1, c2, out)


def reach_envelope(spec: ArmSpec = DEFAULT) -> dict:
    """What the tip can actually touch — useful for the viewer and for
    sanity-checking that the head even fits in the workspace."""
    lo, hi = spec.q2_limits
    heights = [spec.pivot_h_mm + spec.arm_len_mm * math.sin(math.radians(a))
               for a in (lo, hi)]
    reaches = [spec.arm_len_mm * math.cos(math.radians(a)) for a in (lo, hi)]
    return {"radius_mm": spec.arm_len_mm,
            "min_height_mm": round(min(heights), 1),
            "max_height_mm": round(max(heights), 1),
            "min_reach_mm": round(min(reaches), 1),
            "max_reach_mm": round(max(reaches), 1),
            "sweep_deg": round(spec.q1_limits[1] - spec.q1_limits[0], 1)}


# ------------------------------------------------------------- teleop

@dataclass
class TeleopConfig:
    """`hand_span_mm` of hand movement sweeps a whole joint.

    The arm's travel is far smaller than a natural arm gesture, so
    millimetre-exact motion pins it at an end stop almost immediately.
    Mapping a gesture onto the full range makes the arm mirror the SHAPE of
    the movement across everything it can reach.
    """
    hand_span_mm: float = 400.0
    auto_align: bool = True
    yaw_offset_deg: float = 0.0
    razor_gain: float = 1.0
    invert_yaw: bool = False
    invert_pitch: bool = False


class Teleop3DOF:
    """Phone pose -> (q1, q2, q3), with a clutch.

    Engaging anchors the phone to wherever the arm currently is, so nothing
    jumps; releasing lets the operator reposition their hand without the arm
    following. Losing tracking holds position rather than guessing.
    """

    def __init__(self, spec: ArmSpec | None = None,
                 config: TeleopConfig | None = None):
        self.spec = spec or ArmSpec()
        self.cfg = config or TeleopConfig()
        self.engaged = False
        self.tracking_ok = True
        self.joints = tuple(self.spec.home)
        self.clamped = (False, False, False)
        self._anchor_hand: tuple[float, float, float] | None = None
        self._anchor_joints: tuple[float, float, float] | None = None
        self._anchor_pitch = 0.0
        self._yaw = 0.0

    def engage(self, x: float, y: float, z: float,
               q1: float, q2: float, q3: float,
               qw: float = 1.0, qx: float = 0.0,
               qy: float = 0.0, qz: float = 0.0) -> None:
        self._anchor_hand = (x, y, z)
        self._anchor_joints = (q1, q2, q3)
        self._anchor_pitch = quat_pitch_deg(qw, qx, qy, qz)
        self._yaw = math.radians(self.cfg.yaw_offset_deg)
        if self.cfg.auto_align:
            # ARKit's axes point wherever the phone did when its session
            # started. Rotate the hand frame so pushing the phone forward
            # pushes the arm away from the operator, whichever way they stand.
            # same convention as the iOS app and pose_teleop: heading from
            # the phone's forward axis, so hand-right lands on dx and
            # hand-forward on dz whichever way the operator is standing
            fx, fy, fz = quat_forward(qw, qx, qy, qz)
            if math.hypot(fx, fz) < 0.15:
                # camera pointing up or down: no usable heading, so take the
                # phone's top edge instead
                fx, fy, fz = 0.0, 0.0, -1.0
            self._yaw += math.atan2(fx, -fz)
        self.engaged = True
        self.joints = (q1, q2, q3)

    def disengage(self) -> None:
        self.engaged = False
        self._anchor_hand = None
        self._anchor_joints = None

    def update(self, x: float, y: float, z: float,
               qw: float, qx: float, qy: float, qz: float,
               track: int = TRACK_NORMAL
               ) -> tuple[float, float, float] | None:
        """-> (q1, q2, q3) or None to hold position."""
        self.tracking_ok = (track == TRACK_NORMAL)
        if not self.engaged or not self.tracking_ok:
            return None
        if self._anchor_hand is None or self._anchor_joints is None:
            return None

        ax, ay, az = self._anchor_hand
        dx = (x - ax) * 1000.0
        dy = (y - ay) * 1000.0
        dz = (z - az) * 1000.0
        if self._yaw:
            c, s = math.cos(self._yaw), math.sin(self._yaw)
            dx, dz = dx * c + dz * s, -dx * s + dz * c

        a1, a2, a3 = self._anchor_joints
        span = max(self.cfg.hand_span_mm, 1.0)
        lim = self.spec.limits()

        # hand sideways sweeps the base; hand up/down swings the arm's arc
        d1 = (dx / span) * (lim[0][1] - lim[0][0])
        d2 = (dy / span) * (lim[1][1] - lim[1][0])
        if self.cfg.invert_yaw:
            d1 = -d1
        if self.cfg.invert_pitch:
            d2 = -d2

        # the razor follows the wrist, gravity-referenced so turning the phone
        # horizontally cannot tilt the blade
        d_pitch = quat_pitch_deg(qw, qx, qy, qz) - self._anchor_pitch
        raw = (a1 + d1, a2 + d2, a3 + d_pitch * self.cfg.razor_gain)

        out = tuple(float(clamp(raw[i], *lim[i])) for i in range(3))
        self.clamped = tuple(bool(abs(out[i] - raw[i]) > 1e-9) for i in range(3))
        self.joints = out
        return out

    def status(self) -> dict:
        return {"engaged": self.engaged,
                "tracking_ok": self.tracking_ok,
                "q1": round(self.joints[0], 2),
                "q2": round(self.joints[1], 2),
                "q3": round(self.joints[2], 2),
                "clamped": dict(zip(JOINT_NAMES, self.clamped)),
                "tip_mm": [round(v, 1) for v in
                           forward_kinematics(self.joints[0], self.joints[1],
                                              self.spec)]}

"""ARKit phone pose -> fade-robot joints (phi, psi, theta).

**The clipper goes where your hand goes.** Move the phone 10 cm and the
clipper tip moves 10 cm along the head: up/down, left/right, forward/back all
behave the way your hand does, because the target is solved in 3D rather than
by wiring one phone axis to one joint.

The machine constrains the tip to a sphere: the carriage rides a fixed-radius
arc (phi) and the whole rail yaws around the head (psi). So the mapping is a
two-step inverse kinematics —

    target point = tip at engage + (phone movement since engage)
    joints       = direction of that point, as elevation and azimuth

Hand movement is resolved into the two directions the machine can actually
move and converted by arc length, so 10 cm of hand travel is 10 cm of
clipper travel across the scalp:

    vertical hand motion   -> phi, at radius R
    horizontal hand motion -> psi, along the latitude circle (radius R*cos el)

The component toward or away from the head is discarded — the rail fixes
stand-off distance and the spring slide absorbs the rest, so pushing at the
head does not drive the arm. Working in joint space rather than moving a
point over the sphere also means a large gesture runs into the rail's end
stop instead of wrapping around the head: the sphere is only ~108 mm, so a
34 cm gesture would otherwise lap it.

Two things keep absolute position usable despite ARKit drift:

**The clutch.** Motion transfers only while engaged. Engaging anchors the
phone to the tip's current position, so nothing jumps, and releasing lets you
reposition your hand — like lifting a mouse. Drift is bounded by one stroke.

**Tilt tracks the wrist 1:1.** Gravity keeps pitch drift-free, so rotating
the phone 10 degrees rotates the clipper 10 degrees.

Unlike replay, live teleop CLAMPS at joint limits rather than refusing: an
operator reaching past the end of the rail should stop at the rail, not have
the session abort.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .arm import HI, LO
from .head import EL_OFFSET_DEG
from .interfaces import AXES

TRACK_NORMAL = 0
RAIL_RADIUS_MM = 108.0          # head max radius + clearance; see head.py


def quat_forward(qw: float, qx: float, qy: float,
                 qz: float) -> tuple[float, float, float]:
    """The phone's forward axis (ARKit camera -Z) in world coordinates."""
    return (-2.0 * (qx * qz + qw * qy),
            2.0 * (qw * qx - qy * qz),
            -(1.0 - 2.0 * (qx * qx + qy * qy)))


def quat_pitch_deg(qw: float, qx: float, qy: float, qz: float) -> float:
    """Elevation of the phone's forward axis, in degrees.

    ARKit's camera looks down -Z with +Y up, so rotating (0,0,-1) by the pose
    quaternion and taking its Y component gives how far the nose points up or
    down. Reading it this way avoids Euler-order and gimbal problems.
    """
    # y component of R(q) @ (0,0,-1)
    fy = 2.0 * (qw * qx - qy * qz)
    return math.degrees(math.asin(max(-1.0, min(1.0, fy))))


def forward_kinematics(phi: float, psi: float,
                       radius: float = RAIL_RADIUS_MM) -> tuple[float, float, float]:
    """Joints -> clipper tip position (mm), head centre at the origin.

    +Y is up; psi sweeps the tip around the vertical axis from +X toward +Z.
    """
    el = math.radians(phi + EL_OFFSET_DEG)
    ps = math.radians(psi)
    return (radius * math.cos(el) * math.cos(ps),
            radius * math.sin(el),
            radius * math.cos(el) * math.sin(ps))


def inverse_kinematics(x: float, y: float, z: float) -> tuple[float, float]:
    """Tip position -> (phi, psi). Only the direction matters: the rail fixes
    the radius, so a point is projected onto the sphere rather than rejected
    for being off it."""
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-6:                       # degenerate: hold at the neutral pose
        return (-EL_OFFSET_DEG, 0.0)
    el = math.degrees(math.asin(max(-1.0, min(1.0, y / n))))
    psi = math.degrees(math.atan2(z, x))
    return (el - EL_OFFSET_DEG, psi)


@dataclass
class TeleopConfig:
    """scale 1.0 means the tip moves exactly as far as your hand does."""
    scale: float = 1.0
    auto_align: bool = True         # on engage, make "push forward" mean "toward the head"
    yaw_offset_deg: float = 0.0     # extra trim on top of auto-align
    theta_gain: float = 1.0         # wrist pitch -> clipper tilt, 1:1
    theta_offset_deg: float = 0.0
    radius_mm: float = RAIL_RADIUS_MM


@dataclass
class TeleopState:
    engaged: bool = False
    tracking_ok: bool = True
    phi: float = 0.0
    psi: float = 0.0
    theta: float = 0.0
    clamped: tuple[bool, bool, bool] = (False, False, False)
    target_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)


class PoseTeleop:
    """Feed it relative ARKit poses; it produces joint targets."""

    def __init__(self, config: TeleopConfig | None = None):
        self.cfg = config or TeleopConfig()
        self.state = TeleopState()
        self._anchor_phone: tuple[float, float, float] | None = None
        self._anchor_tip: tuple[float, float, float] | None = None
        self._anchor_joints: tuple[float, float] | None = None
        self._anchor_pitch: float = 0.0
        self._anchor_theta: float = 0.0
        self._yaw: float = 0.0          # hand frame -> robot frame, radians

    # ------------------------------------------------------------- clutch

    def engage(self, x: float, y: float, z: float,
               phi: float, psi: float, theta: float,
               qw: float = 1.0, qx: float = 0.0,
               qy: float = 0.0, qz: float = 0.0) -> None:
        """Clutch in. Anchors the phone to the tip's current position, so the
        first frame after engaging commands no motion on any axis."""
        self._anchor_phone = (x, y, z)
        self._anchor_joints = (phi, psi)
        self._anchor_tip = forward_kinematics(phi, psi, self.cfg.radius_mm)
        self._anchor_pitch = quat_pitch_deg(qw, qx, qy, qz)
        self._anchor_theta = theta
        self._yaw = math.radians(self.cfg.yaw_offset_deg)
        if self.cfg.auto_align:
            # ARKit's axes point wherever the phone did when its session
            # started, which is unrelated to where the head is. Rotate the
            # hand frame so that pushing the phone forward pushes the clipper
            # at the head; left/right and up/down then follow the operator.
            fx, _fy, fz = quat_forward(qw, qx, qy, qz)
            if abs(fx) > 1e-6 or abs(fz) > 1e-6:
                tx, _ty, tz = self._anchor_tip
                into_head = math.atan2(-tz, -tx)      # from tip toward centre
                self._yaw += into_head - math.atan2(fz, fx)
        self.state.engaged = True

    def disengage(self) -> None:
        self.state.engaged = False
        self._anchor_phone = None
        self._anchor_tip = None
        self._anchor_joints = None

    # -------------------------------------------------------------- update

    def update(self, x: float, y: float, z: float,
               qw: float, qx: float, qy: float, qz: float,
               track: int = TRACK_NORMAL) -> tuple[float, float, float] | None:
        """-> (phi, psi, theta) target, or None to hold position.

        None means the clutch is out or ARKit lost tracking; both are 'stop
        moving', never 'guess'.
        """
        self.state.tracking_ok = (track == TRACK_NORMAL)
        if not self.state.engaged or not self.state.tracking_ok:
            return None
        if self._anchor_phone is None or self._anchor_joints is None:
            return None

        cfg = self.cfg
        ax, ay, az = self._anchor_phone
        # hand movement since engage, metres -> mm, in the robot's frame
        dx = (x - ax) * 1000.0 * cfg.scale
        dy = (y - ay) * 1000.0 * cfg.scale
        dz = (z - az) * 1000.0 * cfg.scale
        if self._yaw:
            c, sn = math.cos(self._yaw), math.sin(self._yaw)
            dx, dz = c * dx - sn * dz, sn * dx + c * dz

        phi, psi = self._joint_step(dx, dy, dz)
        self.state.target_mm = forward_kinematics(phi, psi, cfg.radius_mm)
        d_pitch = quat_pitch_deg(qw, qx, qy, qz) - self._anchor_pitch
        theta = self._anchor_theta + d_pitch * cfg.theta_gain + cfg.theta_offset_deg

        raw = (phi, psi, theta)
        # float()/bool() keep numpy scalars out of state: status() is
        # JSON-encoded straight onto the sim's WebSocket
        out = tuple(float(min(HI[i], max(LO[i], raw[i]))) for i in range(3))
        self.state.clamped = tuple(bool(abs(out[i] - raw[i]) > 1e-9)
                                   for i in range(3))
        self.state.phi, self.state.psi, self.state.theta = out
        return out

    def _joint_step(self, dx: float, dy: float,
                    dz: float) -> tuple[float, float]:
        """Hand movement (mm, robot frame) -> joint targets, by arc length."""
        r = self.cfg.radius_mm
        a_phi, a_psi = self._anchor_joints       # type: ignore[misc]
        el = math.radians(a_phi + EL_OFFSET_DEG)
        ps = math.radians(a_psi)

        # vertical travel rides the arc of radius r
        d_phi = math.degrees(dy / r)

        # horizontal travel rides the latitude circle, radius r*cos(el).
        # the tangent that increases psi:
        along = dx * -math.sin(ps) + dz * math.cos(ps)
        lat_r = max(r * math.cos(el), 1.0)       # guarded; el stays well off the pole
        d_psi = math.degrees(along / lat_r)

        return (a_phi + d_phi, a_psi + d_psi)

    def status(self) -> dict:
        s = self.state
        return {
            "engaged": s.engaged,
            "tracking_ok": s.tracking_ok,
            "phi": round(s.phi, 2),
            "psi": round(s.psi, 2),
            "theta": round(s.theta, 2),
            "clamped": dict(zip(AXES, s.clamped)),
            "target_mm": [round(v, 1) for v in s.target_mm],
        }

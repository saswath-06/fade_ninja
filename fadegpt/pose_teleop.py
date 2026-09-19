"""ARKit phone pose -> fade-robot joints (phi, psi, theta).

The phone's 6DoF pose drives the arm directly: move the phone up and the
clipper climbs the head, swing it around and the clipper travels around the
head, tilt the nose up and the clipper opens up to leave longer hair.

Two things make absolute position usable despite ARKit drift:

**The clutch.** Motion only transfers while the clutch is held. Engaging
anchors the phone pose to the arm's *current* joints, so the arm never jumps
on engage, and releasing lets you reposition your hand — exactly like lifting
a mouse off the pad. Drift is bounded by one stroke instead of a session.

**Tilt tracks the wrist 1:1.** Gravity keeps pitch drift-free, so rotating
the phone 10 degrees rotates the clipper 10 degrees. It is still anchored on
engage, so no axis jumps when the clutch closes.

Unlike replay, live teleop CLAMPS to joint limits rather than refusing: an
operator pushing past the end of the rail should stop at the rail, not have
the session abort.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .arm import HI, LO
from .interfaces import AXES

TRACK_NORMAL = 0


def quat_pitch_deg(qw: float, qx: float, qy: float, qz: float) -> float:
    """Elevation of the phone's forward axis, in degrees.

    ARKit's camera looks down -Z with +Y up, so rotating (0,0,-1) by the
    pose quaternion and taking its Y component gives how far the nose points
    up or down. Reading it this way avoids Euler-order and gimbal problems.
    """
    # y component of R(q) @ (0,0,-1), i.e. row 1 of the rotation matrix
    # dotted with the forward axis: -(2*(qy*qz - qw*qx))
    fy = 2.0 * (qw * qx - qy * qz)
    return math.degrees(math.asin(max(-1.0, min(1.0, fy))))


@dataclass
class TeleopConfig:
    """Hand travel to joint travel. Defaults put the whole fade zone inside
    a comfortable arm movement."""
    phi_deg_per_m: float = 200.0     # phone up/down -> height on the head
    psi_deg_per_m: float = 300.0     # phone left/right -> around the head
    theta_gain: float = 1.0          # wrist pitch -> clipper tilt, 1:1
    theta_offset_deg: float = 0.0
    invert_psi: bool = False


@dataclass
class TeleopState:
    engaged: bool = False
    tracking_ok: bool = True
    phi: float = 0.0
    psi: float = 0.0
    theta: float = 0.0
    clamped: tuple[bool, bool, bool] = (False, False, False)


class PoseTeleop:
    """Feed it relative ARKit poses; it produces joint targets.

    The arm's own limits are the only clamp, and the caller keeps ownership
    of the arm — this class holds no motion state beyond the clutch anchor.
    """

    def __init__(self, config: TeleopConfig | None = None):
        self.cfg = config or TeleopConfig()
        self.state = TeleopState()
        self._anchor_phone: tuple[float, float, float] | None = None
        self._anchor_joints: tuple[float, float, float] | None = None
        self._anchor_pitch: float = 0.0

    # ------------------------------------------------------------- clutch

    def engage(self, x: float, y: float, z: float,
               phi: float, psi: float, theta: float,
               qw: float = 1.0, qx: float = 0.0,
               qy: float = 0.0, qz: float = 0.0) -> None:
        """Clutch pressed. Anchors this phone pose to these joints, so the
        first frame after engaging commands no motion on ANY axis."""
        self._anchor_phone = (x, y, z)
        self._anchor_joints = (phi, psi, theta)
        self._anchor_pitch = quat_pitch_deg(qw, qx, qy, qz)
        self.state.engaged = True

    def disengage(self) -> None:
        self.state.engaged = False
        self._anchor_phone = None
        self._anchor_joints = None

    # -------------------------------------------------------------- update

    def update(self, x: float, y: float, z: float,
               qw: float, qx: float, qy: float, qz: float,
               track: int = TRACK_NORMAL) -> tuple[float, float, float] | None:
        """-> (phi, psi, theta) target, or None to hold position.

        None means either the clutch is out or ARKit lost tracking; both are
        'stop moving', never 'guess'.
        """
        self.state.tracking_ok = (track == TRACK_NORMAL)
        if not self.state.engaged or not self.state.tracking_ok:
            return None
        if self._anchor_phone is None or self._anchor_joints is None:
            return None

        ax, ay, _az = self._anchor_phone
        a_phi, a_psi, a_theta = self._anchor_joints
        cfg = self.cfg

        phi = a_phi + (y - ay) * cfg.phi_deg_per_m
        lateral = (x - ax) * cfg.psi_deg_per_m
        psi = a_psi + (-lateral if cfg.invert_psi else lateral)
        d_pitch = quat_pitch_deg(qw, qx, qy, qz) - self._anchor_pitch
        theta = a_theta + d_pitch * cfg.theta_gain + cfg.theta_offset_deg

        raw = (phi, psi, theta)
        # float()/bool() keep numpy scalars out of state: status() is
        # JSON-encoded straight onto the sim's WebSocket
        out = tuple(float(min(HI[i], max(LO[i], raw[i]))) for i in range(3))
        self.state.clamped = tuple(bool(abs(out[i] - raw[i]) > 1e-9)
                                   for i in range(3))
        self.state.phi, self.state.psi, self.state.theta = out
        return out

    def status(self) -> dict:
        s = self.state
        return {
            "engaged": s.engaged,
            "tracking_ok": s.tracking_ok,
            "phi": round(s.phi, 2),
            "psi": round(s.psi, 2),
            "theta": round(s.theta, 2),
            "clamped": dict(zip(AXES, s.clamped)),
        }

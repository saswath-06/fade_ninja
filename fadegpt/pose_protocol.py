"""UDP pose wire format for ARKit teleop (PHONE_POSE_TDD Phase 1 + Fix pass).

Messages (ASCII, one datagram):

    START <track>
    POSE <t_ms> <x_m> <y_m> <z_m> <qw> <qx> <qy> <qz> <track>
    STOP
    STATUS

track: 0=normal, 1=limited, 2=relocalizing.
START and motion require track == normal.
Positions in POSE are absolute ARKit world meters; PoseSession converts to
metres relative to the first accepted sample after START.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


TRACK_NORMAL = 0
TRACK_LIMITED = 1
TRACK_RELOCALIZING = 2

_QUAT_TOL = 1e-3


class PoseProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class PoseMessage:
    kind: str
    t_ms: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    track: int = TRACK_NORMAL


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


def _finite(name: str, v: float) -> float:
    if not math.isfinite(v):
        raise PoseProtocolError(f"{name} must be finite")
    return v


def _parse_track(token: str) -> int:
    try:
        track = int(token)
    except ValueError as e:
        raise PoseProtocolError(f"bad track value {token!r}") from e
    if track not in (TRACK_NORMAL, TRACK_LIMITED, TRACK_RELOCALIZING):
        raise PoseProtocolError(f"bad track value {track}")
    return track


def parse_message(line: str) -> PoseMessage:
    parts = line.strip().split()
    if not parts:
        raise PoseProtocolError("empty message")
    kind = parts[0].upper()

    if kind in ("STOP", "STATUS"):
        if len(parts) != 1:
            raise PoseProtocolError(f"{kind} takes no arguments")
        return PoseMessage(kind=kind)

    if kind == "START":
        if len(parts) != 2:
            raise PoseProtocolError("START takes track: START <track>")
        return PoseMessage(kind="START", track=_parse_track(parts[1]))

    if kind != "POSE":
        raise PoseProtocolError(f"unknown message {kind}")

    if len(parts) != 10:
        raise PoseProtocolError(
            "POSE takes t_ms x y z qw qx qy qz track")
    try:
        t_ms = int(parts[1])
        x, y, z = (float(parts[2]), float(parts[3]), float(parts[4]))
        qw, qx, qy, qz = (float(parts[5]), float(parts[6]),
                          float(parts[7]), float(parts[8]))
        track = _parse_track(parts[9])
    except PoseProtocolError:
        raise
    except ValueError as e:
        raise PoseProtocolError(f"POSE: bad number ({e})") from None

    for name, v in (("x", x), ("y", y), ("z", z),
                    ("qw", qw), ("qx", qx), ("qy", qy), ("qz", qz)):
        _finite(name, v)

    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if abs(norm - 1.0) > _QUAT_TOL:
        raise PoseProtocolError(f"quaternion not unit (norm={norm})")

    return PoseMessage("POSE", t_ms, x, y, z, qw, qx, qy, qz, track)


@dataclass
class RelativePose:
    t_ms: int
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float
    track: int


class PoseSession:
    """Tracks START origin and relative pose. Tracking must be normal."""

    def __init__(self) -> None:
        self.started = False
        self.origin: Vec3 | None = None
        self.relative = RelativePose(0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                     TRACK_NORMAL)
        self.origin_quat: tuple[float, float, float, float] | None = None

    def handle(self, msg: PoseMessage, track: int | None = None) -> str:
        """Apply a parsed message. Returns an ack string.

        For START, track comes from the message itself (START <track>).
        The optional track= override is kept for tests only.
        """
        if msg.kind == "START":
            tr = msg.track if track is None else track
            if tr != TRACK_NORMAL:
                raise PoseProtocolError("START requires normal tracking")
            self.started = True
            self.origin = None
            self.origin_quat = None
            self.relative = RelativePose(0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                         TRACK_NORMAL)
            return "OK"

        if msg.kind == "STOP":
            self.started = False
            self.origin = None
            self.origin_quat = None
            return "OK"

        if msg.kind == "STATUS":
            return "OK"

        # POSE
        if not self.started:
            raise PoseProtocolError("POSE before START")
        if msg.track != TRACK_NORMAL:
            raise PoseProtocolError("POSE requires normal tracking")

        if self.origin is None:
            self.origin = Vec3(msg.x, msg.y, msg.z)
            self.origin_quat = (msg.qw, msg.qx, msg.qy, msg.qz)
            self.relative = RelativePose(
                msg.t_ms, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, msg.track)
            return "OK"

        rw, rx, ry, rz = _quat_mul(
            _quat_conj(*self.origin_quat),  # type: ignore[misc]
            msg.qw, msg.qx, msg.qy, msg.qz)
        self.relative = RelativePose(
            msg.t_ms,
            msg.x - self.origin.x,
            msg.y - self.origin.y,
            msg.z - self.origin.z,
            rw, rx, ry, rz,
            msg.track,
        )
        return "OK"


def _quat_conj(w, x, y, z):
    return w, -x, -y, -z


def _quat_mul(a, bw, bx, by, bz):
    aw, ax, ay, az = a
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def pitch_deg_from_quat(qw: float, qx: float, qy: float, qz: float) -> float:
    """Extract pitch (nose up/down) in degrees from a relative quaternion."""
    sinp = 2.0 * (qw * qx - qz * qy)
    sinp = max(-1.0, min(1.0, sinp))
    return math.degrees(math.asin(sinp))

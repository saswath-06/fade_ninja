"""Phase 0 interfaces: the five shared blocks both tracks build against.

0.1 Teach log line   t_ms, phi, psi, theta, contact  (50 Hz, from arm encoders)
0.2 Commands         see protocol.py
0.3 Profile          {"name", "points": [{"u", "mm"}]}
0.4 Calibration      {"L_mm", "table": [[theta_deg, mm], ...]}
0.5 Head geometry    {"phi_neckline_deg", "phi_top_deg"}
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np

TICK_MS = 20  # 50 Hz
TICK_S = TICK_MS / 1000.0

AXES = ("phi", "psi", "theta")


@dataclass(frozen=True)
class AxisLimits:
    lo: float        # deg
    hi: float        # deg
    max_rate: float  # deg/s
    max_accel: float  # deg/s^2


# phi: carriage along the arc rail (height on the head)
# psi: rotation of the rail around the head, ear to back to ear
# theta: clipper tilt servo (hair length left)
JOINT_LIMITS = {
    "phi": AxisLimits(0.0, 70.0, 40.0, 400.0),
    "psi": AxisLimits(0.0, 180.0, 60.0, 400.0),
    "theta": AxisLimits(0.0, 60.0, 120.0, 800.0),
}


# ---------------------------------------------------------------- 0.1 teach log

@dataclass
class LogSample:
    t_ms: int
    phi: float
    psi: float
    theta: float
    contact: bool

    def to_line(self) -> str:
        return (f"{self.t_ms},{self.phi:.3f},{self.psi:.3f},"
                f"{self.theta:.3f},{int(self.contact)}")

    @classmethod
    def from_line(cls, line: str) -> "LogSample":
        t, phi, psi, theta, contact = line.strip().split(",")
        return cls(int(t), float(phi), float(psi), float(theta),
                   bool(int(contact)))


LOG_HEADER = "t_ms,phi,psi,theta,contact"


@dataclass
class TeachLog:
    samples: list[LogSample] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.samples)

    def to_csv(self) -> str:
        return "\n".join([LOG_HEADER] + [s.to_line() for s in self.samples])

    @classmethod
    def from_csv(cls, text: str) -> "TeachLog":
        lines = [ln for ln in text.strip().splitlines() if ln.strip()]
        if lines and lines[0].replace(" ", "") == LOG_HEADER:
            lines = lines[1:]
        return cls([LogSample.from_line(ln) for ln in lines])

    def arrays(self) -> dict[str, np.ndarray]:
        return {
            "t_ms": np.array([s.t_ms for s in self.samples]),
            "phi": np.array([s.phi for s in self.samples]),
            "psi": np.array([s.psi for s in self.samples]),
            "theta": np.array([s.theta for s in self.samples]),
            "contact": np.array([s.contact for s in self.samples]),
        }

    @classmethod
    def from_arrays(cls, t_ms, phi, psi, theta, contact) -> "TeachLog":
        return cls([LogSample(int(t), float(p), float(s), float(th), bool(c))
                    for t, p, s, th, c in zip(t_ms, phi, psi, theta, contact)])


# ------------------------------------------------------------------ 0.3 profile

@dataclass
class ProfilePoint:
    u: float   # 0 at neckline, 1 at top of fade zone
    mm: float  # hair length left

@dataclass
class Profile:
    name: str
    points: list[ProfilePoint] = field(default_factory=list)

    def to_json(self) -> str:
        return json.dumps({"name": self.name,
                           "points": [{"u": round(p.u, 4), "mm": round(p.mm, 3)}
                                      for p in self.points]})

    @classmethod
    def from_json(cls, text: str) -> "Profile":
        d = json.loads(text)
        return cls(d["name"], [ProfilePoint(p["u"], p["mm"]) for p in d["points"]])


# -------------------------------------------------------------- 0.4 calibration

@dataclass
class CalibrationTable:
    L_mm: float
    table: list[list[float]]  # [[theta_deg, mm], ...]

    def to_json(self) -> str:
        return json.dumps({"L_mm": self.L_mm, "table": self.table})

    @classmethod
    def from_json(cls, text: str) -> "CalibrationTable":
        d = json.loads(text)
        return cls(float(d["L_mm"]), [[float(a), float(b)] for a, b in d["table"]])


# ------------------------------------------------------------ 0.5 head geometry

@dataclass
class HeadGeometry:
    phi_neckline_deg: float = 8.0
    phi_top_deg: float = 62.0

    def to_json(self) -> str:
        return json.dumps({"phi_neckline_deg": self.phi_neckline_deg,
                           "phi_top_deg": self.phi_top_deg})

    @classmethod
    def from_json(cls, text: str) -> "HeadGeometry":
        d = json.loads(text)
        return cls(float(d["phi_neckline_deg"]), float(d["phi_top_deg"]))

    def u(self, phi):
        """Normalized height in the fade zone: 0 at neckline, 1 at top."""
        span = self.phi_top_deg - self.phi_neckline_deg
        return (np.asarray(phi, dtype=float) - self.phi_neckline_deg) / span

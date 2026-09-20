"""Recording and replaying a 4-axis cut.

The rule from the rail machine carries over unchanged: **record the arm, not
the controller.** What lands in a take is the joint angles the arm was
actually commanded, never the phone's estimate of where a hand was, so a
drifting sensor cannot corrupt the recording.

Replay smooths hand tremor out first, then validates every sample against
the joint limits and REFUSES rather than clipping — a log that asks for an
impossible pose is a bug, and running it anyway near someone's head is the
wrong answer.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import savgol_filter

from .eezy_ik import Q1_LIMITS, Q2_LIMITS, Q3_LIMITS, Q4_LIMITS

TICK_MS = 20                      # 50 Hz, matching the pose stream
SAVGOL_WINDOW = 21                # 420 ms: removes tremor, keeps the path
SAVGOL_POLY = 3
SNAP_EPS_DEG = 1.0                # smoothing may undershoot a pose held ON a limit
LIMITS = (Q1_LIMITS, Q2_LIMITS, Q3_LIMITS, Q4_LIMITS)
JOINTS = ("q1", "q2", "q3", "q4")


class ReplayRefused(RuntimeError):
    """The take failed validation. Nothing has moved."""


@dataclass
class Sample:
    t_ms: int
    q1: float
    q2: float
    q3: float
    q4: float
    reachable: bool

    def as_row(self) -> tuple:
        return (self.t_ms, self.q1, self.q2, self.q3, self.q4, self.reachable)


class JointRecorder:
    """Samples the arm's commanded joints on a fixed tick."""

    def __init__(self):
        self.recording = False
        self._samples: list[Sample] = []
        self._t_ms = 0

    def start(self) -> None:
        self._samples = []
        self._t_ms = 0
        self.recording = True

    def tick(self, q1: float, q2: float, q3: float, q4: float,
             reachable: bool = True) -> None:
        if not self.recording:
            return
        self._samples.append(Sample(self._t_ms, float(q1), float(q2),
                                    float(q3), float(q4), bool(reachable)))
        self._t_ms += TICK_MS

    def stop(self) -> list[tuple]:
        self.recording = False
        return [s.as_row() for s in self._samples]

    @property
    def n(self) -> int:
        return len(self._samples)

    @property
    def elapsed_ms(self) -> int:
        return self._t_ms


# ------------------------------------------------------------- replay prep

def _cols(rows: list[tuple]) -> np.ndarray:
    return np.array([[r[1], r[2], r[3], r[4]] for r in rows], dtype=float)


def smooth(rows: list[tuple], window: int = SAVGOL_WINDOW,
           poly: int = SAVGOL_POLY) -> list[tuple]:
    """Take hand tremor out before the arm is asked to follow it."""
    if len(rows) < window:
        return list(rows)
    q = _cols(rows)
    for j in range(4):
        q[:, j] = savgol_filter(q[:, j], window, poly)
    return [(rows[i][0], *q[i], rows[i][5]) for i in range(len(rows))]


def _snap(rows: list[tuple], eps: float = SNAP_EPS_DEG) -> list[tuple]:
    """Smoothing can undershoot a pose held exactly ON a limit by a fraction
    of a degree. Snap those back; anything past eps is a real violation and
    still gets refused."""
    out = []
    for r in rows:
        vals = list(r[1:5])
        for j, (lo, hi) in enumerate(LIMITS):
            if lo - eps <= vals[j] < lo:
                vals[j] = lo
            elif hi < vals[j] <= hi + eps:
                vals[j] = hi
        out.append((r[0], *vals, r[5]))
    return out


def validate(rows: list[tuple]) -> None:
    """Raise ReplayRefused listing every problem. Never clips."""
    problems = []
    if not rows:
        problems.append("take is empty")
    else:
        t = np.array([r[0] for r in rows])
        if np.any(np.diff(t) <= 0):
            problems.append("timestamps are not strictly increasing")
        q = _cols(rows)
        for j, name in enumerate(JOINTS):
            lo, hi = LIMITS[j]
            bad = np.flatnonzero((q[:, j] < lo) | (q[:, j] > hi))
            if bad.size:
                i = int(bad[0])
                problems.append(
                    f"{name}={q[i, j]:.2f} at t={int(t[i])} ms outside "
                    f"[{lo}, {hi}] ({bad.size} samples)")
    if problems:
        raise ReplayRefused("; ".join(problems))


def prepare(rows: list[tuple], smooth_first: bool = True) -> list[tuple]:
    """Validate, smooth, snap, validate again. Raises before anything moves.

    The RAW take is checked first on purpose. Teleop already clamps to the
    joint limits, so an out-of-range raw sample means the data is corrupt —
    and smoothing would quietly pull a wild value back into range and run it.
    A take containing garbage should be refused, not repaired.
    """
    validate(rows)
    if not smooth_first:
        return list(rows)
    out = _snap(smooth(rows))
    validate(out)
    return out


class TakePlayer:
    """Walks a prepared take one tick at a time.

    Holds no arm reference: the caller decides what to do with each pose, so
    the same player drives the simulator, the viewer, or real servos.
    """

    def __init__(self, rows: list[tuple], smooth_first: bool = True):
        self.rows = prepare(rows, smooth_first)
        self.i = 0

    @property
    def done(self) -> bool:
        return self.i >= len(self.rows)

    @property
    def progress(self) -> float:
        return 0.0 if not self.rows else min(1.0, self.i / len(self.rows))

    def next_pose(self) -> tuple[float, float, float, float] | None:
        if self.done:
            return None
        r = self.rows[self.i]
        self.i += 1
        return (r[1], r[2], r[3], r[4])


def rms_per_joint(a: list[tuple], b: list[tuple]) -> dict[str, float]:
    n = min(len(a), len(b))
    qa, qb = _cols(a[:n]), _cols(b[:n])
    return {JOINTS[j]: float(np.sqrt(np.mean((qa[:, j] - qb[:, j]) ** 2)))
            for j in range(4)}

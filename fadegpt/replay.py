"""Replay: smooth with savgol, validate every sample against joint limits
(refuse to run rather than clipping), then command each sample at its
timestamp. That is the whole mechanism.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import savgol_filter

from .interfaces import AXES, JOINT_LIMITS, TICK_MS, TeachLog
from .rig import Rig

SAVGOL_WINDOW = 21  # 420 ms at 50 Hz: removes hand tremor, keeps the path
SAVGOL_POLY = 3


class ReplayRefused(RuntimeError):
    """The log failed validation. The arm has not moved."""


SNAP_EPS_DEG = 1.0


def _snap_to_limits(log: TeachLog, eps: float = SNAP_EPS_DEG) -> TeachLog:
    """Savgol can undershoot a corner that sits ON a joint limit (e.g. theta
    held at 0) by a fraction of a degree. Snap those artifacts back to the
    limit; anything beyond eps is a real violation and still gets refused."""
    a = log.arrays()
    out = {}
    for ax in AXES:
        lim = JOINT_LIMITS[ax]
        v = a[ax].copy()
        v[(v < lim.lo) & (v >= lim.lo - eps)] = lim.lo
        v[(v > lim.hi) & (v <= lim.hi + eps)] = lim.hi
        out[ax] = v
    return TeachLog.from_arrays(a["t_ms"], out["phi"], out["psi"],
                                out["theta"], a["contact"])


def smooth(log: TeachLog, window: int = SAVGOL_WINDOW,
           poly: int = SAVGOL_POLY) -> TeachLog:
    """Take hand jitter out of a teach log before replay."""
    if len(log) < window:
        return TeachLog(list(log.samples))
    a = log.arrays()
    out = {}
    for ax in AXES:
        out[ax] = savgol_filter(a[ax], window, poly)
    return TeachLog.from_arrays(a["t_ms"], out["phi"], out["psi"],
                                out["theta"], a["contact"])


def validate(log: TeachLog) -> None:
    """Raise ReplayRefused listing every problem. Never clips."""
    problems = []
    if len(log) == 0:
        problems.append("log is empty")
    a = log.arrays() if len(log) else None
    if a is not None:
        if np.any(np.diff(a["t_ms"]) <= 0):
            problems.append("timestamps are not strictly increasing")
        for ax in AXES:
            lim = JOINT_LIMITS[ax]
            bad = np.flatnonzero((a[ax] < lim.lo) | (a[ax] > lim.hi))
            if bad.size:
                i = int(bad[0])
                problems.append(
                    f"{ax}={a[ax][i]:.2f} at t={int(a['t_ms'][i])} ms outside "
                    f"[{lim.lo}, {lim.hi}] ({bad.size} samples)")
            if len(a[ax]) > 1:
                dt_s = np.diff(a["t_ms"]) / 1000.0
                with np.errstate(divide="ignore", invalid="ignore"):
                    rates = np.abs(np.diff(a[ax]) / dt_s)
                fast = np.flatnonzero(rates > lim.max_rate * 1.5)
                if fast.size:
                    problems.append(
                        f"{ax} rate {rates[fast[0]]:.1f} deg/s exceeds "
                        f"{lim.max_rate} deg/s at t={int(a['t_ms'][fast[0]])} ms")
    if problems:
        raise ReplayRefused("; ".join(problems))


def replay(log: TeachLog, rig: Rig, smooth_first: bool = True,
           clipper_on: bool = True, record: bool = False) -> TeachLog | None:
    """Validate, position at the first sample, then track sample-by-sample.

    Returns the re-recorded log when record=True (gate B1.7 test 2).
    """
    lg = _snap_to_limits(smooth(log)) if smooth_first else log
    validate(lg)  # refuse before any motion

    first = lg.samples[0]
    if not rig.arm.homed:
        rig.arm.home()
    rig.goto(first.phi, first.psi, first.theta)

    if record:
        rig.recorder.start()
    rig.clipper_on = clipper_on
    try:
        for s in lg.samples:
            rig.arm.track(s.phi, s.psi, s.theta)
            rig.tick()
    finally:
        rig.clipper_on = False
        rig.arm.stop()
    return rig.recorder.stop() if record else None


def rms_per_axis(a: TeachLog, b: TeachLog) -> dict[str, float]:
    """RMS difference per axis over the overlapping samples."""
    n = min(len(a), len(b))
    aa, bb = a.arrays(), b.arrays()
    return {ax: float(np.sqrt(np.mean((aa[ax][:n] - bb[ax][:n]) ** 2)))
            for ax in AXES}

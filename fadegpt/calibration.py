"""Tilt-angle to hair-length calibration (needed for profiles, not for replay)."""
from __future__ import annotations

import numpy as np

from .interfaces import CalibrationTable


class CalibrationError(ValueError):
    pass


class Calibration:
    """Wraps a theta->mm table with interpolation both ways.

    The table must be strictly monotonic in mm: if it flattens at high tilt
    the heel is lifting off and the mount geometry is wrong (README section 5).
    """

    def __init__(self, table: CalibrationTable):
        arr = np.asarray(table.table, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 2 or arr.shape[0] < 2:
            raise CalibrationError("table must be [[theta_deg, mm], ...] with >= 2 rows")
        thetas, mms = arr[:, 0], arr[:, 1]
        if np.any(np.diff(thetas) <= 0):
            raise CalibrationError("theta column must be strictly increasing")
        if np.any(np.diff(mms) <= 0.05):
            raise CalibrationError(
                "mm column must be strictly increasing; a flat table means the "
                "heel lifts off at high tilt (fix the mount, gate A5)")
        self.raw = table
        self._thetas = thetas
        self._mms = mms

    @property
    def L_mm(self) -> float:
        return self.raw.L_mm

    def theta_to_mm(self, theta):
        return np.interp(theta, self._thetas, self._mms)

    def mm_to_theta(self, mm):
        return np.interp(mm, self._mms, self._thetas)

    @property
    def mm_range(self) -> tuple[float, float]:
        return float(self._mms[0]), float(self._mms[-1])


def default_calibration() -> Calibration:
    """The example table from README section 5 (L = 25 mm)."""
    return Calibration(CalibrationTable(
        L_mm=25.0,
        table=[[0, 0.6], [15, 1.5], [30, 3.1], [45, 5.4], [60, 8.2]],
    ))

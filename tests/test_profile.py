"""B3 log->profile gate, calibration monotonicity, lookup round trips."""
import numpy as np
import pytest

from fade_ninja.calibration import (Calibration, CalibrationError,
                                    default_calibration)
from fade_ninja.interfaces import CalibrationTable, HeadGeometry
from fade_ninja.profile import log_to_profile, profile_curve
from fade_ninja.teach import jittered, ramp_log

CAL = default_calibration()
GEOM = HeadGeometry()


def test_b3_linear_ramp_fits_within_0p2mm():
    log = ramp_log(CAL, GEOM, mm0=0.6, mm1=6.0)
    prof = log_to_profile(log, CAL, GEOM, name="ramp")
    assert len(prof.points) == 5
    for p in prof.points:
        expected = 0.6 + (6.0 - 0.6) * p.u
        assert p.mm == pytest.approx(expected, abs=0.2), \
            f"u={p.u}: {p.mm} vs {expected}"


def test_b3_survives_hand_jitter():
    log = jittered(ramp_log(CAL, GEOM, mm0=0.6, mm1=6.0), sigma_deg=0.25)
    prof = log_to_profile(log, CAL, GEOM)
    for p in prof.points:
        expected = 0.6 + 5.4 * p.u
        assert p.mm == pytest.approx(expected, abs=0.3)


def test_profile_is_monotonic_curve():
    log = ramp_log(CAL, GEOM)
    prof = log_to_profile(log, CAL, GEOM)
    mms = [p.mm for p in prof.points]
    assert all(b >= a for a, b in zip(mms, mms[1:]))
    curve = profile_curve(prof)
    u = np.linspace(0, 1, 200)
    assert np.all(np.diff(curve(u)) >= -1e-6)


def test_calibration_lookup_round_trip():
    thetas = np.linspace(0, 60, 20)
    back = CAL.mm_to_theta(CAL.theta_to_mm(thetas))
    assert np.allclose(back, thetas, atol=1e-9)


def test_flat_calibration_table_rejected():
    # flattens at high tilt = the heel is lifting off (fix mount, gate A5)
    with pytest.raises(CalibrationError, match="heel"):
        Calibration(CalibrationTable(
            25.0, [[0, 0.6], [15, 1.5], [30, 3.1], [45, 3.12], [60, 3.13]]))


def test_non_monotonic_theta_rejected():
    with pytest.raises(CalibrationError):
        Calibration(CalibrationTable(25.0, [[0, 0.6], [15, 1.5], [10, 3.0]]))

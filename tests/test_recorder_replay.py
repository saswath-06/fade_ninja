"""Gates B1.6 (50 Hz logging) and B1.7 (smooth, validate-refuse, replay RMS)."""
import numpy as np
import pytest

from fade_ninja.calibration import default_calibration
from fade_ninja.interfaces import JOINT_LIMITS, TICK_MS, TeachLog
from fade_ninja.replay import ReplayRefused, replay, rms_per_axis, smooth, validate
from fade_ninja.rig import Rig
from fade_ninja.teach import jittered, ramp_log

CAL = default_calibration()


def _record_jog_session(seconds=10.0) -> tuple[Rig, TeachLog]:
    rig = Rig()
    rig.arm.home()
    rig.goto(10, 60, 5)
    rig.recorder.start()
    rig.arm.jog(4, 3, 2)
    rig.run(seconds / 2)
    rig.arm.jog(-2, 4, -1)
    rig.run(seconds / 2)
    rig.arm.stop()
    return rig, rig.recorder.stop()


def test_b16_500_samples_sane():
    _, log = _record_jog_session(10.0)
    assert len(log) == 500
    a = log.arrays()
    assert list(a["t_ms"][:3]) == [0, 20, 40]
    assert np.all(np.diff(a["t_ms"]) == TICK_MS)
    for ax in ("phi", "psi", "theta"):
        lim = JOINT_LIMITS[ax]
        assert np.all(a[ax] >= lim.lo - 1e-9)
        assert np.all(a[ax] <= lim.hi + 1e-9)


def test_savgol_removes_tremor_keeps_path():
    clean = ramp_log(CAL)
    noisy = jittered(clean, sigma_deg=0.25)
    sm = smooth(noisy)
    err_noisy = rms_per_axis(noisy, clean)
    err_sm = rms_per_axis(sm, clean)
    for ax in ("phi", "theta"):
        assert err_sm[ax] < 0.5 * err_noisy[ax], \
            f"{ax}: smoothing barely helped ({err_sm[ax]:.3f} vs {err_noisy[ax]:.3f})"
        assert err_sm[ax] < 0.15, f"{ax}: smoothed path strayed from the ramp"


def test_b17_replay_retraces_within_1_degree_rms():
    _, log = _record_jog_session(6.0)
    rig2 = Rig(seed=42)
    commanded = smooth(log)
    rerecorded = replay(log, rig2, record=True)
    rms = rms_per_axis(commanded, rerecorded)
    for ax, v in rms.items():
        assert v < 1.0, f"{ax} RMS {v:.3f} deg exceeds the 1 degree gate"


def test_b17_refuses_out_of_range_log():
    log = ramp_log(CAL)
    log.samples[100].theta = 85.0  # beyond the 60 deg tilt limit
    rig = Rig()
    rig.arm.home()
    pos_before = rig.arm.pos.copy()
    with pytest.raises(ReplayRefused, match="theta"):
        replay(log, rig, smooth_first=False)
    assert np.all(rig.arm.pos == pos_before), "refusal must come before motion"


def test_refuses_empty_and_bad_timestamps():
    with pytest.raises(ReplayRefused, match="empty"):
        validate(TeachLog([]))
    log = ramp_log(CAL)
    log.samples[10].t_ms = log.samples[9].t_ms  # duplicate timestamp
    with pytest.raises(ReplayRefused, match="timestamps"):
        validate(log)


def test_refuses_impossible_rates():
    log = ramp_log(CAL)
    log.samples[100].phi += 20.0  # a 1000 deg/s teleport between samples
    with pytest.raises(ReplayRefused, match="rate"):
        validate(log)

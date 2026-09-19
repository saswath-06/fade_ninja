"""Vision-in-the-loop gates. The vision stack sees only rendered photographs
(noise, vignette, blur) — never head.hair. Ground truth is used solely to
JUDGE the outcome, the way calipers judge a real cut.

The two fault-injection tests are the point: open-loop replay cannot survive
a missed stripe or a miscalibrated blade; the vision critic must.
"""
import numpy as np
import pytest

from fadegpt import camera, vision
from fadegpt.autopilot import VisionAutopilot
from fadegpt.calibration import Calibration, default_calibration
from fadegpt.head import HAIR_START_MM, Head
from fadegpt.interfaces import CalibrationTable, HeadGeometry
from fadegpt.profile import profile_curve, profile_to_log
from fadegpt.replay import replay
from fadegpt.rig import Rig
from fadegpt.teach import default_fade_mm

GEOM = HeadGeometry()


@pytest.fixture(scope="module")
def ap() -> VisionAutopilot:
    return VisionAutopilot.calibrate()


def true_error_vs_target(head: Head, mm_of_u) -> float:
    """Judge with ground truth: worst per-bin gap between the actual cut and
    the target curve (too long only — vision can't regrow hair)."""
    u, mm = head.length_vs_u()
    valid = ~np.isnan(mm) & (u > 0.05) & (u < 0.95)
    return float(np.max(mm[valid] - np.asarray(mm_of_u(u[valid]))))


# ------------------------------------------------------------ vision alone

def test_camera_brightness_tracks_length():
    short, long_ = Head(), Head()
    short.hair[:] = 2.0
    long_.hair[:] = 6.0
    assert camera.render(long_).mean() > camera.render(short).mean() + 0.05


def test_length_estimation_within_half_mm(ap):
    head, labels = camera.ruler_calibration_head([1.0, 2.2, 4.5, 7.0])
    est = vision.length_map(camera.render(head, seed=9), ap.flat, ap.model)
    for mm, rows in labels:
        got = float(np.median(est[rows, 30:-30]))
        assert got == pytest.approx(mm, abs=0.5), f"{mm} mm read as {got:.2f}"


def test_reference_photo_to_profile(ap):
    ref = camera.render(camera.reference_head(default_fade_mm), seed=11)
    prof = ap.target_from_photo(ref, GEOM)
    curve = profile_curve(prof)
    u = np.linspace(0.05, 0.95, 19)
    err = np.abs(np.asarray(curve(u)) - default_fade_mm(u))
    assert err.max() < 0.4, f"photo-derived target off by {err.max():.2f} mm"


# ------------------------------------------------------- closed loop, clean

def test_autopilot_cuts_a_fade_from_a_photo(ap):
    rig = Rig(head=Head(), seed=0)
    ref = camera.render(camera.reference_head(default_fade_mm), seed=12)
    target = ap.target_from_photo(ref, GEOM)
    rep = ap.cut(rig, target)
    assert rep.converged, rep.notes
    err = true_error_vs_target(rig.head, profile_curve(target))
    assert err < 0.4, f"cut misses photo target by {err:.2f} mm"


# --------------------------------------------------- fault 1: missed stripe

class SabotagedRig(Rig):
    """The clipper silently fails over a psi window during the FIRST cutting
    episode only (a clogged blade that cleared), then behaves."""

    def __init__(self, *a, window=(95.0, 125.0), **kw):
        super().__init__(*a, **kw)
        self.window = window
        self._episode = 0
        self._prev_on = False

    def tick(self):
        on = self.clipper_on
        if on and self._episode == 0 and \
                self.window[0] <= float(self.arm.pos[1]) <= self.window[1]:
            self.clipper_on = False
        super().tick()
        self.clipper_on = on
        if self._prev_on and not on:
            self._episode += 1
        self._prev_on = on


def test_open_loop_cannot_survive_a_missed_stripe():
    rig = SabotagedRig(head=Head(), seed=0)
    prof = VisionAutopilot.calibrate().target_from_photo(
        camera.render(camera.reference_head(default_fade_mm), seed=12), GEOM)
    replay(profile_to_log(prof, rig.cal, rig.head), rig)
    band = rig.head.fade_zone_mask()
    stripe = band & (np.abs(rig.head.psi_grid[None, :] - 105) < 5)
    assert np.any(rig.head.hair[stripe] == HAIR_START_MM), \
        "sabotage failed to leave uncut hair — test is vacuous"


def test_critic_finds_and_fixes_the_missed_stripe(ap):
    rig = SabotagedRig(head=Head(), seed=0)
    ref = camera.render(camera.reference_head(default_fade_mm), seed=12)
    target = ap.target_from_photo(ref, GEOM)
    rep = ap.cut(rig, target, max_iters=4)
    assert rep.corrective_passes > 0, "critic never scheduled a fix"
    assert rep.max_over_history[0] > 2.0, "first critique should see the stripe"
    err = true_error_vs_target(rig.head, profile_curve(target))
    assert err < 0.5, f"stripe not fixed: {err:.2f} mm too long ({rep.notes})"


# ------------------------------------------- fault 2: miscalibrated blade

def biased_cal(scale: float) -> Calibration:
    t = default_calibration().raw
    return Calibration(CalibrationTable(
        t.L_mm, [[a, b * scale] for a, b in t.table]))


def test_open_loop_misses_with_biased_blade():
    rig = Rig(head=Head(), seed=0, physical_cal=biased_cal(1.18))
    prof = VisionAutopilot.calibrate().target_from_photo(
        camera.render(camera.reference_head(default_fade_mm), seed=12), GEOM)
    replay(profile_to_log(prof, rig.cal, rig.head), rig)
    err = true_error_vs_target(rig.head, profile_curve(prof))
    assert err > 0.6, "bias too small to prove anything"


def test_critic_corrects_biased_blade(ap):
    rig = Rig(head=Head(), seed=0, physical_cal=biased_cal(1.18))
    ref = camera.render(camera.reference_head(default_fade_mm), seed=12)
    target = ap.target_from_photo(ref, GEOM)
    rep = ap.cut(rig, target, max_iters=4)
    assert rep.gain_history[0] > ap.baseline_gain * 1.06, \
        "critic should measure the long bias against its rehearsal baseline"
    err = true_error_vs_target(rig.head, profile_curve(target))
    assert err < 0.45, f"bias not corrected: {err:.2f} mm ({rep.notes})"

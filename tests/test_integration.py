"""D3/D4/D5/D6 analogs: teach a fade, replay it on a fresh head, prove the
cuts match; export a profile and run it on a bigger head.

D4 is the product thesis: 'your haircut is a file.'
"""
import numpy as np
import pytest

from fadegpt.head import Head
from fadegpt.interfaces import HeadGeometry
from fadegpt.profile import log_to_profile, profile_curve, profile_to_log
from fadegpt.replay import replay
from fadegpt.rig import Rig
from fadegpt.teach import default_fade_mm, high_fade_mm, teach_fade


@pytest.fixture(scope="module")
def taught():
    """Teach a low fade once (D3); reused by the replay tests."""
    rig = Rig(head=Head(), seed=0)
    log = teach_fade(rig, seed=1)
    return rig, log


def test_d3_teach_produces_a_cut_and_a_log(taught):
    rig, log = taught
    assert len(log) > 500, "the whole session should be recorded"
    assert rig.head.cut_mask().any(), "no hair was cut"
    a = log.arrays()
    assert a["contact"].mean() > 0.5, "heel should be on the scalp most of the time"
    # the cut is a fade: short at the bottom, long at the top
    u, mm = rig.head.length_vs_u()
    valid = ~np.isnan(mm)
    assert mm[valid][-1] - mm[valid][0] > 3.0


def test_d4_replay_reproduces_the_cut_on_a_fresh_head(taught):
    rig_a, log = taught
    rig_b = Rig(head=Head(), seed=99)
    replay(log, rig_b, record=False)

    # the sim analog of the side-by-side photograph:
    u, mm_a = rig_a.head.length_vs_u()
    _, mm_b = rig_b.head.length_vs_u()
    valid = ~np.isnan(mm_a) & ~np.isnan(mm_b)
    assert valid.sum() >= 15
    diff = np.abs(mm_a[valid] - mm_b[valid])
    assert diff.max() < 0.3, \
        f"taught and replayed fades differ by up to {diff.max():.2f} mm"

    # and the cut regions themselves coincide
    a, b = rig_a.head.cut_mask(), rig_b.head.cut_mask()
    iou = (a & b).sum() / (a | b).sum()
    assert iou > 0.97, f"cut footprints only overlap {iou:.1%}"


def test_d5_second_cut_is_visibly_different(taught):
    rig_low, _ = taught
    rig_high = Rig(head=Head(), seed=3)
    teach_fade(rig_high, fade_mm=high_fade_mm, seed=4)

    u, mm_low = rig_low.head.length_vs_u()
    _, mm_high = rig_high.head.length_vs_u()
    mid = (u > 0.3) & (u < 0.7) & ~np.isnan(mm_low) & ~np.isnan(mm_high)
    # a high fade stays short far longer through the middle of the zone
    diff = mm_low[mid] - mm_high[mid]
    assert np.all(diff > 0.4) and diff.mean() > 0.9, \
        "the two cuts should be obviously different in a photo"


def test_d6_profile_transfers_to_a_bigger_head(taught):
    rig_a, log = taught
    geom = HeadGeometry()
    prof = log_to_profile(log, rig_a.cal, geom, name="low_fade")
    curve = profile_curve(prof)

    big = Head(scale=1.1)
    rig_c = Rig(head=big, seed=5)
    plan = profile_to_log(prof, rig_c.cal, big)
    replay(plan, rig_c, record=False)

    u, mm = rig_c.head.length_vs_u()
    valid = ~np.isnan(mm)
    err = np.abs(mm[valid] - curve(u[valid]))
    assert err.max() < 0.4, \
        f"profile on a 10% larger head is off by {err.max():.2f} mm"


def test_d6_profile_matches_what_was_taught(taught):
    rig_a, log = taught
    prof = log_to_profile(log, rig_a.cal, HeadGeometry(), name="low_fade")
    curve = profile_curve(prof)
    u = np.linspace(0.05, 0.95, 19)
    err = np.abs(curve(u) - default_fade_mm(u))
    assert err.max() < 0.3, \
        f"exported profile strays {err.max():.2f} mm from the taught curve"

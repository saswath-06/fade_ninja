"""ARKit pose -> fade-robot joints.

The properties that matter for pointing a clipper at a head: engaging the
clutch never jogs the arm, releasing it lets you reposition your hand for
free, and losing tracking stops motion rather than guessing.
"""
import math

import pytest

from fadegpt.arm import HI, LO
from fadegpt.pose_teleop import (PoseTeleop, TeleopConfig, quat_pitch_deg)

LEVEL = (1.0, 0.0, 0.0, 0.0)          # identity quaternion


def nose_up(deg):
    """Quaternion pitching the phone's forward axis up by deg.

    Rotation about ARKit's +X (right) axis; by the right-hand rule that
    tips the -Z forward axis upward. Matches what tools/fake_phone.py emits.
    """
    h = math.radians(deg / 2)
    return (math.cos(h), math.sin(h), 0.0, 0.0)


def engaged(phi=30.0, psi=90.0, theta=10.0, cfg=None):
    t = PoseTeleop(cfg)
    t.engage(0.0, 0.0, 0.0, phi, psi, theta, *LEVEL)
    return t


# ---------------------------------------------------------------- pitch

def test_level_phone_reads_zero_pitch():
    assert quat_pitch_deg(*LEVEL) == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("deg", [10, 25, 45, -15])
def test_pitch_extraction_matches_the_rotation(deg):
    assert quat_pitch_deg(*nose_up(deg)) == pytest.approx(deg, abs=1e-6)


def test_pitch_is_clamped_not_nan_at_extremes():
    # straight up: asin argument can drift past 1.0 in floating point
    assert quat_pitch_deg(*nose_up(90)) == pytest.approx(90.0, abs=1e-6)
    assert not math.isnan(quat_pitch_deg(0.7071, 0.7071, 0.0, 0.0))


def test_pitch_sign_matches_the_reference_phone():
    """tools/fake_phone.py --pitch D sends qw=cos(D/2), qx=sin(D/2).
    That must read as +D here, or tilt runs backwards on real hardware."""
    for d in (15, 30, -20):
        h = math.radians(d / 2)
        assert quat_pitch_deg(math.cos(h), math.sin(h), 0.0, 0.0) == \
            pytest.approx(d, abs=1e-6)


# --------------------------------------------------------------- clutch

def test_nothing_moves_until_the_clutch_engages():
    t = PoseTeleop()
    assert t.update(0.5, 0.5, 0.5, *LEVEL) is None
    assert not t.status()["engaged"]


def test_engaging_commands_no_motion_on_any_axis():
    """The safety property: closing the clutch must not jog the arm."""
    t = engaged(phi=30.0, psi=90.0, theta=10.0)
    assert t.update(0.0, 0.0, 0.0, *LEVEL) == pytest.approx((30.0, 90.0, 10.0))


def test_engaging_at_a_pitched_wrist_still_does_not_jump():
    t = PoseTeleop()
    q = nose_up(35)
    t.engage(0.0, 0.0, 0.0, 30.0, 90.0, 10.0, *q)
    assert t.update(0.0, 0.0, 0.0, *q) == pytest.approx((30.0, 90.0, 10.0))


def test_releasing_the_clutch_holds_position():
    t = engaged()
    t.update(0.0, 0.1, 0.0, *LEVEL)
    t.disengage()
    assert t.update(0.0, 0.3, 0.0, *LEVEL) is None


def test_reposition_your_hand_for_free():
    """Release, move the hand 40 cm, re-engage: the arm stays put. This is
    what keeps ARKit drift bounded to a single stroke."""
    t = engaged(phi=30.0, psi=90.0, theta=10.0)
    phi, psi, theta = t.update(0.0, 0.05, 0.0, *LEVEL)
    t.disengage()
    t.engage(0.0, 0.45, 0.0, phi, psi, theta, *LEVEL)   # hand moved, arm did not
    assert t.update(0.0, 0.45, 0.0, *LEVEL) == pytest.approx((phi, psi, theta))


# -------------------------------------------------------------- mapping

def test_phone_up_climbs_the_head():
    t = engaged(phi=20.0)
    phi, _, _ = t.update(0.0, 0.1, 0.0, *LEVEL)
    assert phi == pytest.approx(20.0 + 0.1 * TeleopConfig().phi_deg_per_m)


def test_phone_sideways_travels_around_the_head():
    t = engaged(psi=90.0)
    _, psi, _ = t.update(0.1, 0.0, 0.0, *LEVEL)
    assert psi == pytest.approx(90.0 + 0.1 * TeleopConfig().psi_deg_per_m)


def test_psi_can_be_inverted_for_the_other_hand():
    t = engaged(psi=90.0, cfg=TeleopConfig(invert_psi=True))
    _, psi, _ = t.update(0.1, 0.0, 0.0, *LEVEL)
    assert psi < 90.0


def test_wrist_pitch_moves_tilt_one_to_one():
    t = engaged(theta=10.0)
    _, _, theta = t.update(0.0, 0.0, 0.0, *nose_up(20))
    assert theta == pytest.approx(30.0, abs=1e-6)


def test_depth_does_not_drive_any_axis():
    """The rail sets stand-off distance, so phone depth is deliberately
    ignored rather than fighting the spring slide."""
    t = engaged()
    a = t.update(0.0, 0.0, 0.0, *LEVEL)
    b = t.update(0.0, 0.0, 0.35, *LEVEL)
    assert a == pytest.approx(b)


# --------------------------------------------------------------- limits

def test_targets_clamp_to_joint_limits_instead_of_aborting():
    t = engaged(phi=60.0)
    phi, psi, theta = t.update(0.0, 2.0, 0.0, *LEVEL)      # way past the rail
    assert phi == pytest.approx(HI[0])
    assert t.status()["clamped"]["phi"] is True


def test_clamping_the_low_end_too():
    t = engaged(phi=10.0, psi=20.0)
    phi, psi, _ = t.update(-2.0, -2.0, 0.0, *LEVEL)
    assert phi == pytest.approx(LO[0]) and psi == pytest.approx(LO[1])


def test_unclamped_moves_report_clean():
    t = engaged(phi=30.0)
    t.update(0.0, 0.02, 0.0, *LEVEL)
    assert not any(t.status()["clamped"].values())


# -------------------------------------------------------------- tracking

@pytest.mark.parametrize("track", [1, 2])
def test_lost_tracking_stops_motion(track):
    """ARKit unsure -> hold position. Never extrapolate near a head."""
    t = engaged()
    assert t.update(0.0, 0.2, 0.0, *LEVEL, track=track) is None
    assert t.status()["tracking_ok"] is False


def test_tracking_recovers_without_a_jump():
    t = engaged(phi=30.0)
    t.update(0.0, 0.05, 0.0, *LEVEL)
    assert t.update(0.0, 0.30, 0.0, *LEVEL, track=1) is None   # lost
    resumed = t.update(0.0, 0.05, 0.0, *LEVEL)                 # back where it was
    assert resumed[0] == pytest.approx(30.0 + 0.05 * TeleopConfig().phi_deg_per_m)

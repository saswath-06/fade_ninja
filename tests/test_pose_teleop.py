"""ARKit pose -> fade-robot joints.

The headline property: the clipper tip travels the same distance your hand
does, in the same direction. Plus the properties that matter for pointing a
clipper at a head — engaging never jogs the arm, releasing lets you
reposition for free, and losing tracking stops motion rather than guessing.
"""
import math

import pytest

from fadegpt.arm import HI, LO
from fadegpt.pose_teleop import (PoseTeleop, TeleopConfig, forward_kinematics,
                                 inverse_kinematics, quat_forward,
                                 quat_pitch_deg, RAIL_RADIUS_MM)

LEVEL = (1.0, 0.0, 0.0, 0.0)


def yaw_quat(deg):
    h = math.radians(deg / 2)
    return (math.cos(h), 0.0, math.sin(h), 0.0)


def nose_up(deg):
    """Rotation about ARKit's +X (right) tips the -Z forward axis upward.
    Matches what tools/fake_phone.py emits."""
    h = math.radians(deg / 2)
    return (math.cos(h), math.sin(h), 0.0, 0.0)


def rotate(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    return (vx * (1 - 2 * (y * y + z * z)) + vy * 2 * (x * y - w * z) + vz * 2 * (x * z + w * y),
            vx * 2 * (x * y + w * z) + vy * (1 - 2 * (x * x + z * z)) + vz * 2 * (y * z - w * x),
            vx * 2 * (x * z - w * y) + vy * 2 * (y * z + w * x) + vz * (1 - 2 * (x * x + y * y)))


def arc_mm(j0, j1):
    """Path the tip actually traces between two joint poses, in mm.

    The machine moves along its own axes, so the path is the rail arc for phi
    and the latitude circle for psi — not the great-circle shortcut between
    the endpoints. ds^2 = (R dphi)^2 + (R cos(el) dpsi)^2.
    """
    from fadegpt.head import EL_OFFSET_DEG
    el = math.radians(j0[0] + EL_OFFSET_DEG)
    d_phi = math.radians(j1[0] - j0[0])
    d_psi = math.radians(j1[1] - j0[1])
    return RAIL_RADIUS_MM * math.hypot(d_phi, math.cos(el) * d_psi)


METRIC = TeleopConfig(mode="metric")


def engaged(phi=30.0, psi=90.0, theta=15.0, q=LEVEL, cfg=None):
    t = PoseTeleop(cfg)
    t.engage(0.0, 0.0, 0.0, phi, psi, theta, *q)
    return t


# ------------------------------------------------------------- kinematics

@pytest.mark.parametrize("phi,psi", [(0, 0), (30, 90), (70, 180), (42, 117)])
def test_fk_ik_round_trip(phi, psi):
    got = inverse_kinematics(*forward_kinematics(phi, psi))
    assert got == pytest.approx((phi, psi), abs=1e-6)


def test_tip_always_sits_on_the_rail_radius():
    for phi, psi in [(0, 0), (35, 45), (70, 180)]:
        x, y, z = forward_kinematics(phi, psi)
        assert math.sqrt(x*x + y*y + z*z) == pytest.approx(RAIL_RADIUS_MM)


def test_ik_projects_points_off_the_sphere():
    """Only direction matters; the rail fixes the radius."""
    far = tuple(v * 3.0 for v in forward_kinematics(40, 60))
    assert inverse_kinematics(*far) == pytest.approx((40, 60), abs=1e-6)


def test_ik_survives_the_origin():
    assert inverse_kinematics(0.0, 0.0, 0.0) == pytest.approx((20.0, 0.0))


# ------------------------------------------------------------ orientation

def test_level_phone_reads_zero_pitch():
    assert quat_pitch_deg(*LEVEL) == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("deg", [10, 25, 45, -15])
def test_pitch_extraction_matches_the_rotation(deg):
    assert quat_pitch_deg(*nose_up(deg)) == pytest.approx(deg, abs=1e-6)


def test_pitch_sign_matches_the_reference_phone():
    """tools/fake_phone.py --pitch D sends qw=cos(D/2), qx=sin(D/2). That must
    read as +D here, or tilt runs backwards on real hardware."""
    for d in (15, 30, -20):
        h = math.radians(d / 2)
        assert quat_pitch_deg(math.cos(h), math.sin(h), 0.0, 0.0) == \
            pytest.approx(d, abs=1e-6)


def test_forward_axis_is_minus_z_when_level():
    assert quat_forward(*LEVEL) == pytest.approx((0.0, 0.0, -1.0), abs=1e-6)


# ------------------------------------------- the point: hand-to-tip is 1:1

@pytest.mark.parametrize("hand_mm", [20.0, 50.0, 100.0])
def test_metric_mode_moves_the_tip_exactly_as_far_as_the_hand(hand_mm):
    t = engaged(phi=35.0, psi=90.0, cfg=METRIC)
    start = (35.0, 90.0)
    right = rotate(LEVEL, (1.0, 0.0, 0.0))
    j = t.update(*[c * hand_mm / 1000.0 for c in right], *LEVEL)
    assert arc_mm(start, j) == pytest.approx(hand_mm, rel=1e-9)


def test_scale_shrinks_the_motion_proportionally():
    t = engaged(phi=35.0, cfg=TeleopConfig(mode="metric", scale=0.5))
    j = t.update(0.10, 0.0, 0.0, *LEVEL)
    assert arc_mm((35.0, 90.0), j) == pytest.approx(50.0, rel=1e-9)


def test_hand_up_climbs_the_head():
    t = engaged(phi=20.0)
    phi, _, _ = t.update(0.0, 0.04, 0.0, *LEVEL)
    assert phi > 20.0


def test_hand_down_descends():
    t = engaged(phi=40.0)
    phi, _, _ = t.update(0.0, -0.04, 0.0, *LEVEL)
    assert phi < 40.0


def test_pushing_at_the_head_barely_moves_the_arm():
    """Stand-off is the rail's job and the spring slide's; the arm should not
    chase a hand pressed toward the scalp."""
    t = engaged(phi=20.0, psi=90.0, cfg=METRIC)          # tip points along +Z
    toward = (0.0, 0.0, -0.08)               # straight at the head centre
    j = t.update(*toward, *LEVEL)
    assert arc_mm((20.0, 90.0), j) < 0.2 * 80.0


def test_any_operator_orientation_behaves_the_same():
    """Auto-align: ARKit's axes point wherever the phone did at session
    start, so 'forward' must be derived from the phone, not the world."""
    results = []
    for heading in (0, 90, 215, -130):
        q = yaw_quat(heading)
        t = engaged(phi=30.0, psi=90.0, q=q)
        fwd = rotate(q, (0.0, 0.0, -1.0))
        right = rotate(q, (1.0, 0.0, 0.0))
        got = []
        for vec in (fwd, right):
            j = t.update(*[c * 0.10 for c in vec], *q)
            got.append((round(j[0], 4), round(j[1], 4)))
        results.append(got)
    assert all(r == results[0] for r in results), results


def test_auto_align_can_be_turned_off():
    a = engaged(q=yaw_quat(90), cfg=TeleopConfig(auto_align=True))
    b = engaged(q=yaw_quat(90), cfg=TeleopConfig(auto_align=False))
    ja = a.update(0.0, 0.0, -0.10, *yaw_quat(90))
    jb = b.update(0.0, 0.0, -0.10, *yaw_quat(90))
    assert ja != jb


# --------------------------------------------------------------- clutch

def test_nothing_moves_until_the_clutch_engages():
    t = PoseTeleop()
    assert t.update(0.5, 0.5, 0.5, *LEVEL) is None
    assert not t.status()["engaged"]


def test_engaging_commands_no_motion_on_any_axis():
    t = engaged(phi=30.0, psi=90.0, theta=15.0)
    assert t.update(0.0, 0.0, 0.0, *LEVEL) == pytest.approx((30.0, 90.0, 15.0))


def test_engaging_at_a_pitched_wrist_still_does_not_jump():
    q = nose_up(35)
    t = engaged(phi=30.0, psi=90.0, theta=15.0, q=q)
    assert t.update(0.0, 0.0, 0.0, *q) == pytest.approx((30.0, 90.0, 15.0))


def test_releasing_the_clutch_holds_position():
    t = engaged()
    t.update(0.0, 0.03, 0.0, *LEVEL)
    t.disengage()
    assert t.update(0.0, 0.3, 0.0, *LEVEL) is None


def test_reposition_your_hand_for_free():
    """Release, move the hand 40 cm, re-engage: the arm stays put. This is
    what bounds ARKit drift to a single stroke."""
    t = engaged(phi=30.0, psi=90.0, theta=15.0)
    phi, psi, theta = t.update(0.02, 0.0, 0.0, *LEVEL)
    t.disengage()
    t.engage(0.0, 0.45, 0.0, phi, psi, theta, *LEVEL)
    assert t.update(0.0, 0.45, 0.0, *LEVEL) == pytest.approx((phi, psi, theta))


# -------------------------------------------------------------- tilt

def test_wrist_pitch_moves_tilt_one_to_one():
    t = engaged(theta=15.0)
    _, _, theta = t.update(0.0, 0.0, 0.0, *nose_up(20))
    assert theta == pytest.approx(35.0, abs=1e-6)


# ------------------------------------------------------------- limits

def test_targets_clamp_to_joint_limits_instead_of_aborting():
    t = engaged(phi=60.0)
    phi, _, _ = t.update(0.0, 2.0, 0.0, *LEVEL)
    assert phi == pytest.approx(HI[0])
    assert t.status()["clamped"]["phi"] is True


def test_unclamped_moves_report_clean():
    t = engaged(phi=30.0)
    t.update(0.0, 0.01, 0.0, *LEVEL)
    assert not any(t.status()["clamped"].values())


def test_status_is_json_safe():
    import json
    t = engaged(phi=60.0)
    t.update(0.0, 2.0, 0.0, *LEVEL)
    json.dumps(t.status())


# ------------------------------------------------------------- tracking

@pytest.mark.parametrize("track", [1, 2])
def test_lost_tracking_stops_motion(track):
    t = engaged()
    assert t.update(0.0, 0.2, 0.0, *LEVEL, track=track) is None
    assert t.status()["tracking_ok"] is False


def test_tracking_recovers_without_a_jump():
    t = engaged(phi=30.0)
    a = t.update(0.0, 0.02, 0.0, *LEVEL)
    assert t.update(0.0, 0.30, 0.0, *LEVEL, track=1) is None
    assert t.update(0.0, 0.02, 0.0, *LEVEL) == pytest.approx(a)


# ------------------------------------------------------- proportional mode

def test_a_full_gesture_sweeps_a_whole_axis():
    """The rail has only 132 mm of vertical travel, so the default maps a
    natural gesture onto the whole range instead of pinning at the end."""
    span = TeleopConfig().hand_span_mm / 1000.0
    t = engaged(phi=0.0, psi=90.0)
    phi, _, _ = t.update(0.0, span, 0.0, *LEVEL)
    assert phi == pytest.approx(HI[0], abs=1e-6)


def test_half_a_gesture_reaches_half_the_axis():
    span = TeleopConfig().hand_span_mm / 1000.0
    t = engaged(phi=0.0, psi=90.0)
    phi, _, _ = t.update(0.0, span / 2, 0.0, *LEVEL)
    assert phi == pytest.approx((HI[0] - LO[0]) / 2, abs=1e-6)


def test_a_small_lift_does_not_pin_the_arm():
    """The complaint that prompted this mode: 10 cm used to exhaust the rail."""
    t = engaged(phi=35.0, psi=90.0)
    phi, _, _ = t.update(0.0, 0.10, 0.0, *LEVEL)
    assert LO[0] < phi < HI[0]
    assert not t.status()["clamped"]["phi"]


def test_hand_span_sets_sensitivity():
    twitchy = engaged(phi=0.0, cfg=TeleopConfig(hand_span_mm=200.0))
    calm = engaged(phi=0.0, cfg=TeleopConfig(hand_span_mm=800.0))
    assert twitchy.update(0.0, 0.1, 0.0, *LEVEL)[0] > \
        calm.update(0.0, 0.1, 0.0, *LEVEL)[0]


def test_proportional_mode_still_auto_aligns():
    results = []
    for heading in (0, 120, -75):
        q = yaw_quat(heading)
        t = engaged(phi=30.0, psi=90.0, q=q)
        right = rotate(q, (1.0, 0.0, 0.0))
        j = t.update(*[c * 0.12 for c in right], *q)
        results.append((round(j[0], 4), round(j[1], 4)))
    assert all(r == results[0] for r in results), results

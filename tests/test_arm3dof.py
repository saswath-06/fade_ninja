"""The real 3-DOF arm: base yaw, arm pitch, razor tilt."""
import math

import pytest

from fade_ninja.arm3dof import (ArmSpec, Teleop3DOF, TeleopConfig,
                                forward_kinematics, inverse_kinematics,
                                reach_envelope)

SPEC = ArmSpec()
LEVEL = (1.0, 0.0, 0.0, 0.0)


def nose_up(deg):
    h = math.radians(deg / 2)
    return (math.cos(h), math.sin(h), 0.0, 0.0)


def engaged(q=(0.0, 30.0, 0.0), cfg=None, spec=None):
    t = Teleop3DOF(spec or SPEC, cfg)
    t.engage(0.0, 0.0, 0.0, *q, *LEVEL)
    return t


# ------------------------------------------------------------ kinematics

@pytest.mark.parametrize("q1,q2", [(0, 0), (45, 30), (-60, 70), (90, -20)])
def test_fk_ik_round_trip(q1, q2):
    tip = forward_kinematics(q1, q2)
    got1, got2, out = inverse_kinematics(*tip)
    assert not out
    assert (got1, got2) == pytest.approx((q1, q2), abs=1e-6)


def test_the_tip_stays_at_a_fixed_distance_from_the_pivot():
    """A rigid link on a servo horn: reach and height are one arc, so the tip
    can never be nearer or further than the arm is long."""
    for q1, q2 in [(0, 0), (30, 60), (-80, -15)]:
        x, y, z = forward_kinematics(q1, q2)
        d = math.sqrt(x*x + (y - SPEC.pivot_h_mm)**2 + z*z)
        assert d == pytest.approx(SPEC.arm_len_mm, abs=1e-6)


def test_pitching_up_raises_the_tip_and_shortens_the_reach():
    low = forward_kinematics(0, 0)
    high = forward_kinematics(0, 60)
    assert high[1] > low[1], "pitching up should raise the tip"
    assert math.hypot(high[0], high[2]) < math.hypot(low[0], low[2])


def test_targets_off_the_arc_are_projected_not_refused():
    """Only the direction from the PIVOT is achievable, so a target four
    times too far away still resolves to the same joint angles."""
    x, y, z = forward_kinematics(20, 40)
    dy = y - SPEC.pivot_h_mm
    far = (x * 4, SPEC.pivot_h_mm + dy * 4, z * 4)
    q1, q2, _ = inverse_kinematics(*far)
    assert (q1, q2) == pytest.approx((20, 40), abs=1e-6)


def test_out_of_range_targets_clamp_and_say_so():
    _, q2, out = inverse_kinematics(0.0, 5000.0, 0.0)
    assert out and q2 == pytest.approx(SPEC.q2_limits[1])


def test_envelope_reports_what_the_arm_can_touch():
    e = reach_envelope()
    assert e["radius_mm"] == SPEC.arm_len_mm
    assert e["max_height_mm"] > e["min_height_mm"]
    assert e["max_reach_mm"] > e["min_reach_mm"]
    assert e["sweep_deg"] == 180.0


# ---------------------------------------------------------------- clutch

def test_nothing_moves_before_the_clutch_engages():
    t = Teleop3DOF(SPEC)
    assert t.update(0.4, 0.4, 0.4, *LEVEL) is None


def test_engaging_commands_no_motion():
    t = engaged((10.0, 40.0, 5.0))
    assert t.update(0.0, 0.0, 0.0, *LEVEL) == pytest.approx((10.0, 40.0, 5.0))


def test_releasing_then_re_engaging_does_not_move_the_arm():
    t = engaged((10.0, 40.0, 5.0))
    q = t.update(0.05, 0.0, 0.0, *LEVEL)
    t.disengage()
    t.engage(0.0, 0.42, 0.0, *q, *LEVEL)     # hand moved a long way meanwhile
    assert t.update(0.0, 0.42, 0.0, *LEVEL) == pytest.approx(q)


# --------------------------------------------------------------- mapping

def test_hand_sideways_turns_the_base():
    t = engaged((0.0, 30.0, 0.0))
    q1, q2, _ = t.update(0.1, 0.0, 0.0, *LEVEL)
    assert q1 != pytest.approx(0.0)
    assert q2 == pytest.approx(30.0), "sideways should not pitch the arm"


def test_hand_up_pitches_the_arm():
    t = engaged((0.0, 30.0, 0.0))
    q1, q2, _ = t.update(0.0, 0.1, 0.0, *LEVEL)
    assert q2 > 30.0
    assert q1 == pytest.approx(0.0), "up should not turn the base"


def test_a_full_gesture_sweeps_a_whole_joint():
    span = TeleopConfig().hand_span_mm / 1000.0
    t = engaged((SPEC.q1_limits[0], 30.0, 0.0))
    q1, _, _ = t.update(span, 0.0, 0.0, *LEVEL)
    assert q1 == pytest.approx(SPEC.q1_limits[1], abs=1e-6)


def test_a_small_movement_does_not_pin_the_arm():
    t = engaged((0.0, 30.0, 0.0))
    q1, q2, _ = t.update(0.08, 0.06, 0.0, *LEVEL)
    assert SPEC.q1_limits[0] < q1 < SPEC.q1_limits[1]
    assert SPEC.q2_limits[0] < q2 < SPEC.q2_limits[1]


def test_wrist_flicks_the_razor_one_to_one():
    t = engaged((0.0, 30.0, 0.0))
    _, _, q3 = t.update(0.0, 0.0, 0.0, *nose_up(25))
    assert q3 == pytest.approx(25.0, abs=1e-6)


def test_turning_the_phone_horizontally_does_not_flick_the_razor():
    """Gravity-referenced tilt: the bug that made the blade swing 30 degrees
    when the phone was merely turned must not come back on this arm."""
    import math as m
    for yaw in (30, 90, 150):
        a = m.radians(yaw / 2)
        q = (m.cos(a), 0.0, m.sin(a), 0.0)        # pure yaw about gravity
        t = Teleop3DOF(SPEC)
        t.engage(0.0, 0.0, 0.0, 0.0, 30.0, 0.0, *LEVEL)
        _, _, q3 = t.update(0.0, 0.0, 0.0, *q)
        assert q3 == pytest.approx(0.0, abs=1e-6)


def test_inversion_flags_flip_each_axis():
    a = engaged(cfg=TeleopConfig())
    b = engaged(cfg=TeleopConfig(invert_yaw=True, invert_pitch=True))
    qa = a.update(0.1, 0.1, 0.0, *LEVEL)
    qb = b.update(0.1, 0.1, 0.0, *LEVEL)
    assert (qa[0] - 0.0) * (qb[0] - 0.0) < 0
    assert (qa[1] - 30.0) * (qb[1] - 30.0) < 0


# ---------------------------------------------------------------- limits

def test_commands_clamp_at_the_joint_limits():
    t = engaged((0.0, 30.0, 0.0))
    q1, q2, _ = t.update(3.0, 3.0, 0.0, *LEVEL)
    assert q1 == pytest.approx(SPEC.q1_limits[1])
    assert q2 == pytest.approx(SPEC.q2_limits[1])
    assert t.status()["clamped"]["q1"] is True


@pytest.mark.parametrize("track", [1, 2])
def test_lost_tracking_holds_position(track):
    t = engaged()
    assert t.update(0.0, 0.3, 0.0, *LEVEL, track=track) is None
    assert t.status()["tracking_ok"] is False


def test_status_is_json_safe():
    import json
    t = engaged()
    t.update(0.05, 0.05, 0.0, *LEVEL)
    json.dumps(t.status())


def test_a_different_arm_spec_is_honoured():
    """The numbers are placeholders until the arm is measured."""
    spec = ArmSpec(arm_len_mm=95.0, pivot_h_mm=40.0, q2_limits=(0.0, 45.0))
    x, y, z = forward_kinematics(0, 45, spec)
    assert math.hypot(x, y - spec.pivot_h_mm) == pytest.approx(95.0)
    t = Teleop3DOF(spec)
    t.engage(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, *LEVEL)
    _, q2, _ = t.update(0.0, 5.0, 0.0, *LEVEL)
    assert q2 == pytest.approx(45.0), "should clamp to THIS arm's limit"

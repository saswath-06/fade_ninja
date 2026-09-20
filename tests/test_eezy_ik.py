"""EEZYbotARM FK/IK gates (PHONE_POSE_TDD Phase 2 + Fix Edit 4)."""
import math

import pytest

from fade_ninja.eezy_ik import (
                                L0_MM,
    L2_MM,
    L3_MM,
    P_HOME,
    Q_HOME,
    Q4_LIMITS,
    forward_kinematics,
    inverse_kinematics,
    pitch_to_q4,
    sample_reachable_workspace,
)


def test_t20_named_constants_and_hand_computed_home():
    assert L0_MM == 55.0 and L2_MM == 80.0 and L3_MM == 80.0
    assert Q_HOME == (0.0, 45.0, -90.0)
    # Hand: r = 80*cos45 + 80*cos(-45) = 80*√2; z = 55
    expected = (80.0 * math.sqrt(2.0), 0.0, 55.0)
    assert P_HOME[0] == pytest.approx(expected[0])
    assert P_HOME[1] == pytest.approx(expected[1])
    assert P_HOME[2] == pytest.approx(expected[2])


def test_t21_fk_home_matches_p_home():
    xyz = forward_kinematics(*Q_HOME)
    assert xyz[0] == pytest.approx(P_HOME[0], abs=1.0)
    assert xyz[1] == pytest.approx(P_HOME[1], abs=1.0)
    assert xyz[2] == pytest.approx(P_HOME[2], abs=1.0)


def test_t22_ik_round_trip_limit_constrained():
    for p in sample_reachable_workspace(50, seed=7):
        j = inverse_kinematics(*p)
        assert j is not None, f"FK sample should be reachable: {p}"
        back = forward_kinematics(j.q1, j.q2, j.q3)
        assert math.dist(p, back) < 2.0, f"round-trip {p} -> {back}"


def test_t23_geometric_unreachable_returns_none():
    assert inverse_kinematics(500.0, 0.0, 55.0) is None
    assert inverse_kinematics(0.0, 0.0, 55.0) is None


def test_t24_joint_limit_miss_returns_none_never_clamps():
    j = inverse_kinematics(-120.0, 0.0, 55.0)
    assert j is None


def test_t25_pitch_to_q4_linear_and_clamped():
    assert pitch_to_q4(10.0) == pytest.approx(10.0)
    assert pitch_to_q4(-10.0) == pytest.approx(-10.0)
    assert pitch_to_q4(90.0) == Q4_LIMITS[1]
    assert pitch_to_q4(-90.0) == Q4_LIMITS[0]


def test_home_envelope_has_usable_forward_reach():
    """Around P_HOME, +X and −X reachable spans should both be meaningful.

    Old home at ~138 mm left only ~21 mm forward — this guards that regression.
    """
    hx, hy, hz = P_HOME
    fwd = 0.0
    back = 0.0
    for dx in range(1, 80):
        if inverse_kinematics(hx + dx, hy, hz) is not None:
            fwd = float(dx)
        if inverse_kinematics(hx - dx, hy, hz) is not None:
            back = float(dx)
    assert fwd >= 40.0, f"forward reach from home only {fwd} mm"
    assert back >= 40.0, f"backward reach from home only {back} mm"
    # Not required to be perfectly equal; within 2:1 is fine
    assert max(fwd, back) / max(min(fwd, back), 1.0) < 2.5

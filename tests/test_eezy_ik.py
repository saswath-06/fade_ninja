"""EEZYbotARM FK/IK gates (PHONE_POSE_TDD Phase 2)."""
import math

import pytest

from fadegpt.eezy_ik import (
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
    assert Q_HOME == (0.0, 30.0, -60.0)
    # Hand: r = 80*cos30 + 80*cos(-30) = 80*√3; z = 55
    expected = (80.0 * math.sqrt(3.0), 0.0, 55.0)
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
    # Far beyond L2+L3
    assert inverse_kinematics(500.0, 0.0, 55.0) is None
    assert inverse_kinematics(0.0, 0.0, 55.0) is None  # r=0 collapsed


def test_t24_joint_limit_miss_returns_none_never_clamps():
    # Algebraically reachable behind the base (q1≈180) but outside Q1 ±90.
    # Must return None — never clamp yaw into range (that would move the tip).
    j = inverse_kinematics(-120.0, 0.0, 55.0)
    assert j is None


def test_t25_pitch_to_q4_linear_and_clamped():
    assert pitch_to_q4(10.0) == pytest.approx(10.0)
    assert pitch_to_q4(-10.0) == pytest.approx(-10.0)
    assert pitch_to_q4(90.0) == Q4_LIMITS[1]
    assert pitch_to_q4(-90.0) == Q4_LIMITS[0]

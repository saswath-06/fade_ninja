"""4DOF arm: workspace clamping and axis independence.

Two operator-visible properties:

1. Moving the phone along one axis moves the tip along one axis. Clipping a
   solved joint angle swings the tip sideways, which reads as the arm drifting
   in directions you never commanded.
2. Reaching past the envelope extends the arm toward the target rather than
   freezing it mid-pose with the elbow folded.
"""
import math

import pytest

from fadegpt.eezy_ik import (L0_MM, L2_MM, L3_MM, P_HOME, Q2_LIMITS,
                             Q3_LIMITS, forward_kinematics,
                             inverse_kinematics, inverse_kinematics_clamped,
                             workspace_reach)
from fadegpt.pose_mapper import PoseMapper
from fadegpt.pose_protocol import RelativePose


def rel(x=0.0, y=0.0, z=0.0):
    return RelativePose(0, x, y, z, 1.0, 0.0, 0.0, 0.0, 0)


# ---------------------------------------------------------------- workspace

def test_inner_reach_respects_the_elbow_limit():
    """Not |l2-l3|: the elbow cannot fold past Q3_LIMITS, so the arm bottoms
    out well before folding flat."""
    d_min, d_max = workspace_reach()
    assert d_max == pytest.approx(L2_MM + L3_MM)
    fold = math.radians(Q3_LIMITS[0])
    expected = math.sqrt(L2_MM**2 + L3_MM**2 + 2*L2_MM*L3_MM*math.cos(fold))
    assert d_min == pytest.approx(expected)
    assert d_min > abs(L2_MM - L3_MM)


def test_clamped_ik_agrees_with_strict_ik_inside_the_workspace():
    for p in [P_HOME, (100.0, 30.0, 90.0), (60.0, -40.0, 120.0)]:
        strict = inverse_kinematics(*p)
        if strict is None:
            continue
        j, out = inverse_kinematics_clamped(*p)
        assert not out
        assert (j.q1, j.q2, j.q3) == pytest.approx(
            (strict.q1, strict.q2, strict.q3), abs=1e-6)


def test_clamped_ik_always_returns_a_pose_within_limits():
    for p in [(0, 0, 1000), (1000, 0, 55), (0, 0, -500), (0.0, 0.0, L0_MM),
              (-400.0, 300.0, 40.0)]:
        j, _ = inverse_kinematics_clamped(*p)
        assert Q2_LIMITS[0] - 1e-6 <= j.q2 <= Q2_LIMITS[1] + 1e-6
        assert Q3_LIMITS[0] - 1e-6 <= j.q3 <= Q3_LIMITS[1] + 1e-6


# ------------------------------------------------------- reaching for height

def test_elbow_straightens_progressively_with_height():
    """The reported bug: the shoulder maxed out and the elbow stayed bent."""
    last = -180.0
    for z in (150.0, 180.0, 210.0, 260.0):
        j, _ = inverse_kinematics_clamped(0.0, 0.0, z)
        assert j.q3 >= last, f"elbow folded back up at z={z}"
        last = j.q3
    j, out = inverse_kinematics_clamped(0.0, 0.0, 400.0)
    assert out
    assert j.q3 == pytest.approx(0.0, abs=1e-6), "fully straight at full stretch"
    assert j.q2 == pytest.approx(Q2_LIMITS[1], abs=1e-6), "shoulder straight up"


def test_full_stretch_reaches_the_arms_true_maximum_height():
    j, _ = inverse_kinematics_clamped(0.0, 0.0, 10_000.0)
    tip = forward_kinematics(j.q1, j.q2, j.q3)
    assert tip[2] == pytest.approx(L0_MM + L2_MM + L3_MM, abs=1e-6)


def test_strict_ik_still_refuses_out_of_range():
    """The clamped solver is additive; the strict one keeps its contract."""
    assert inverse_kinematics(500.0, 0.0, 55.0) is None


# ------------------------------------------------------- axis independence

def _wander(axis, distance=0.30, steps=120):
    """Max tip movement in the two axes that should not move."""
    m = PoseMapper()
    m.update(rel(), now=0.0)
    worst, t = 0.0, 0.0
    for k in range(steps):
        t += 0.02
        st = m.update(rel(**{axis: distance * min(k / (steps / 2), 1.0)}), now=t)
        tip = forward_kinematics(st.q1, st.q2, st.q3)
        if axis == "y":      # phone +Y drives arm +Z only
            off = math.hypot(tip[0] - P_HOME[0], tip[1] - P_HOME[1])
        elif axis == "x":    # phone +X drives arm -Y only
            off = math.hypot(tip[0] - P_HOME[0], tip[2] - P_HOME[2])
        else:                # phone +Z drives arm -X only
            off = math.hypot(tip[1] - P_HOME[1], tip[2] - P_HOME[2])
        worst = max(worst, off)
    return worst


@pytest.mark.parametrize("axis", ["x", "y", "z"])
def test_single_axis_phone_motion_stays_on_one_arm_axis(axis):
    """Clipping solved joint angles used to push the tip ~15 mm sideways on
    the z sweep, because the elbow hit its limit part-way through."""
    assert _wander(axis) < 0.5


@pytest.mark.parametrize("axis", ["x", "y", "z"])
def test_driving_past_the_envelope_parks_on_the_workspace_boundary(axis):
    """Axis purity cannot survive past the envelope, and shouldn't: the
    closest reachable point to a target 36 cm away from a 16 cm arm lies
    along the ray to it, so the tip necessarily shifts off the commanded
    axis. What must hold is that the arm sits exactly on its boundary,
    fully extended, rather than stopping somewhere arbitrary inside it."""
    _, d_max = workspace_reach()
    m = PoseMapper()
    m.update(rel(), now=0.0)
    t = 0.0
    for k in range(200):                      # long enough to finish slewing
        t += 0.02
        st = m.update(rel(**{axis: 1.2}), now=t)
    assert not st.reachable
    tip = forward_kinematics(st.q1, st.q2, st.q3)
    d = math.hypot(math.hypot(tip[0], tip[1]), tip[2] - L0_MM)
    assert d == pytest.approx(d_max, abs=1e-6), "should be fully extended"
    assert st.q3 == pytest.approx(0.0, abs=1e-6), "elbow straight at full reach"


def test_joints_arrive_together_rather_than_one_finishing_early():
    """Independent per-joint rate limits let fast joints finish first, which
    bends the tip path during the transient."""
    m = PoseMapper(max_slew=(10.0, 10.0, 10.0, 10.0))
    m.update(rel(), now=0.0)
    start = m.state()
    st = m.update(rel(x=0.2, y=0.2), now=0.1)
    moved = [abs(st.q1 - start.q1), abs(st.q2 - start.q2), abs(st.q3 - start.q3)]
    assert max(moved) <= 10.0 * 0.1 + 1e-6, "a joint exceeded its rate limit"
    assert max(moved) > 0.0

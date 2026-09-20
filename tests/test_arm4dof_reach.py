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


# ------------------------------------------------------------- convergence

def test_the_tip_actually_reaches_a_held_target():
    """Convergence was never pinned: a mapper that merely moves toward the
    target looks right in a screenshot but never arrives."""
    from fadegpt.eezy_ik import forward_kinematics as fk
    m = PoseMapper()
    m.update(rel(), now=0.0)
    hold = (0.30, 0.19, 0.15)
    target = m.phone_to_arm_mm(*hold)
    t = 0.0
    for _ in range(300):                       # 6 s at 50 Hz
        t += 0.02
        st = m.update(rel(x=hold[0], y=hold[1], z=hold[2]), now=t)
    tip = fk(st.q1, st.q2, st.q3)
    assert math.dist(tip, target) < 0.5, "arm never converged on the target"


def test_tracking_lag_stays_responsive():
    """60 deg/s left the arm ~0.9 s behind the hand, which reads as bad
    tracing. Guard the responsiveness we settled on."""
    from fadegpt.eezy_ik import forward_kinematics as fk
    m = PoseMapper()
    m.update(rel(), now=0.0)
    hold = (0.30, 0.19, 0.15)
    target = m.phone_to_arm_mm(*hold)
    t = 0.0
    for _ in range(300):
        t += 0.02
        st = m.update(rel(x=hold[0], y=hold[1], z=hold[2]), now=t)
        if math.dist(fk(st.q1, st.q2, st.q3), target) < 1.0:
            break
    assert t < 0.45, f"took {t:.2f}s to catch up with the hand"


# ------------------------------------------------- operator frame alignment

def _heading(Ro):
    """Mirrors the Swift in ios/FadePose: heading from the camera's forward
    axis, falling back to the phone's top edge when it points up or down."""
    f = [-Ro[0][2], -Ro[1][2], -Ro[2][2]]
    if math.hypot(f[0], f[2]) < 0.15:
        f = [Ro[0][1], Ro[1][1], Ro[2][1]]
    return math.atan2(f[0], -f[2])


def _correct(Ro, world_delta):
    th = _heading(Ro)
    ca, sa = math.cos(th), math.sin(th)
    dx, dy, dz = world_delta
    return (dx * ca + dz * sa, dy, -dx * sa + dz * ca)


def _rot_y(deg):
    a = math.radians(deg)
    return [[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]]


def _rot_x(deg):
    a = math.radians(deg)
    return [[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _col(M, j):
    v = [M[0][j], M[1][j], M[2][j]]
    return v


def _horiz(v):
    h = [v[0], 0.0, v[2]]
    n = math.hypot(h[0], h[2])
    return [h[0] / n, 0.0, h[2] / n] if n > 1e-9 else h


ORIENTATIONS = [
    ("upright facing forward", [[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
    ("upright turned 40", _rot_y(40)),
    ("upright turned -115", _rot_y(-115)),
    ("flat screen up", _rot_x(-90)),
    ("flat turned 40", _mul(_rot_y(40), _rot_x(-90))),
    ("tilted 45 down", _rot_x(-45)),
    ("tilted down turned 70", _mul(_rot_y(70), _rot_x(-60))),
]


@pytest.mark.parametrize("name,Ro", ORIENTATIONS)
def test_operator_axes_map_to_one_arm_axis_each(name, Ro):
    """However the phone is held, moving along the operator's own up / right
    / forward must land on exactly one axis.

    The app used to send inverse(origin) * current, i.e. movement in the
    origin CAMERA's axes — so holding the phone flat made 'up' arrive as
    forward, which yawed the arm instead of raising it.
    """
    fwd = _horiz([-c for c in _col(Ro, 2)])
    if math.hypot(fwd[0], fwd[2]) < 0.15:
        fwd = _horiz(_col(Ro, 1))
    body = {"up": [0.0, 1.0, 0.0], "right": _horiz(_col(Ro, 0)), "forward": fwd}
    expect = {"up": 1, "right": 0, "forward": 2}      # -> y, x, z
    for move, world in body.items():
        got = _correct(Ro, world)
        dominant = max(range(3), key=lambda i: abs(got[i]))
        assert dominant == expect[move], f"{name}: {move} landed on {'xyz'[dominant]}"
        assert abs(got[dominant]) == pytest.approx(1.0, abs=1e-5)
        for i in range(3):
            if i != dominant:
                assert abs(got[i]) < 1e-5, f"{name}: {move} bled into {'xyz'[i]}"

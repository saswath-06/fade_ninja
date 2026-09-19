"""Pose → joints mapper gates (PHONE_POSE_TDD Phase 3)."""
import math

import pytest

from fadegpt.eezy_ik import P_HOME, Q_HOME
from fadegpt.pose_mapper import DEFAULT_SCALE, PoseMapper
from fadegpt.pose_protocol import RelativePose


def rel(x=0.0, y=0.0, z=0.0, qw=1.0, qx=0.0, qy=0.0, qz=0.0, t_ms=0):
    return RelativePose(t_ms, x, y, z, qw, qx, qy, qz, 0)


def test_t31_origin_maps_to_home():
    m = PoseMapper()
    st = m.update(rel(), now=0.0)
    assert (st.q1, st.q2, st.q3) == pytest.approx(Q_HOME, abs=1e-6)
    assert st.reachable


def test_t32_phone_plus_y_raises_tip_z():
    m = PoseMapper()
    m.update(rel(), now=0.0)
    z0 = m.state().tip_z
    st = m.update(rel(y=0.05), now=10.0)
    assert st.reachable
    assert st.tip_z > z0 + 5.0


def test_t33_lateral_rotates_q1():
    m = PoseMapper()
    m.update(rel(), now=0.0)
    st = m.update(rel(x=0.05), now=10.0)
    assert st.reachable
    assert st.q1 < -1.0


def test_t34_pitch_only_moves_q4():
    m = PoseMapper()
    m.update(rel(), now=0.0)
    q123 = (m.state().q1, m.state().q2, m.state().q3)
    half = math.radians(10.0)
    st = m.update(rel(qw=math.cos(half), qx=math.sin(half)), now=10.0)
    assert (st.q1, st.q2, st.q3) == pytest.approx(q123, abs=0.5)
    assert st.q4 == pytest.approx(20.0, abs=0.5)


def test_t35_default_scale_ten_cm_phone_to_three_cm_tip():
    assert DEFAULT_SCALE == 0.3
    m = PoseMapper()
    ax, ay, az = m.phone_to_arm_mm(0.0, 0.0, -0.10)
    assert ax - P_HOME[0] == pytest.approx(30.0)
    assert ay == pytest.approx(P_HOME[1])
    assert az == pytest.approx(P_HOME[2])


def test_t36_unreachable_holds_last_good():
    m = PoseMapper()
    m.update(rel(), now=0.0)
    good = m.state()
    st = m.update(rel(y=5.0), now=10.0)
    assert not st.reachable
    assert (st.q1, st.q2, st.q3) == pytest.approx(
        (good.q1, good.q2, good.q3), abs=1e-6)


def test_t37_slew_limits_joint_speed():
    m = PoseMapper(max_slew=(10.0, 10.0, 10.0, 10.0))
    m.update(rel(), now=0.0)
    st = m.update(rel(x=0.2), now=0.1)
    assert abs(st.q1 - Q_HOME[0]) <= 10.0 * 0.1 + 1e-6

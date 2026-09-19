"""Pose wire format (T1): START / POSE / STOP / tracking gate."""
import math

import pytest

from fadegpt.pose_protocol import (
    TRACK_LIMITED,
    TRACK_NORMAL,
    TRACK_RELOCALIZING,
    PoseProtocolError,
    PoseSession,
    parse_message,
)


def test_parse_pose_valid():
    msg = parse_message(
        "POSE 120 0.01 -0.02 0.03 1 0 0 0 0"
    )
    assert msg.kind == "POSE"
    assert msg.t_ms == 120
    assert msg.x == pytest.approx(0.01)
    assert msg.y == pytest.approx(-0.02)
    assert msg.z == pytest.approx(0.03)
    assert msg.qw == pytest.approx(1.0)
    assert msg.track == TRACK_NORMAL


def test_parse_start_stop_status():
    assert parse_message("START").kind == "START"
    assert parse_message("STOP").kind == "STOP"
    assert parse_message("STATUS").kind == "STATUS"


@pytest.mark.parametrize(
    "bad",
    [
        "",
        "POSE 1 2 3",
        "POSE 1 0 0 0 1 0 0 0",
        "POSE x 0 0 0 1 0 0 0 0",
        "POSE 1 0 0 0 nan 0 0 0 0",
        "NOPE",
    ],
)
def test_reject_bad_lines(bad):
    with pytest.raises(PoseProtocolError):
        parse_message(bad)


def test_quaternion_must_be_unit():
    with pytest.raises(PoseProtocolError):
        parse_message("POSE 1 0 0 0 2 0 0 0 0")
    # near-unit is ok
    n = math.sqrt(0.5)
    msg = parse_message(f"POSE 1 0 0 0 {n} {n} 0 0 0")
    assert abs(msg.qw) == pytest.approx(n)


def test_start_requires_normal_tracking():
    s = PoseSession()
    with pytest.raises(PoseProtocolError):
        s.handle(parse_message("START"), track=TRACK_LIMITED)
    out = s.handle(parse_message("START"), track=TRACK_NORMAL)
    assert out == "OK"
    assert s.started


def test_start_resets_origin_and_accepts_relative_poses():
    s = PoseSession()
    s.handle(parse_message("START"), track=TRACK_NORMAL)
    # Absolute ARKit sample at origin-ish
    s.handle(parse_message("POSE 0 1.0 2.0 3.0 1 0 0 0 0"))
    assert s.origin is not None
    assert s.relative.x == pytest.approx(0.0)
    assert s.relative.y == pytest.approx(0.0)
    assert s.relative.z == pytest.approx(0.0)

    s.handle(parse_message("POSE 20 1.05 2.0 3.0 1 0 0 0 0"))
    assert s.relative.x == pytest.approx(0.05)
    assert s.relative.y == pytest.approx(0.0)
    assert s.relative.z == pytest.approx(0.0)


def test_motion_rejected_when_tracking_not_normal():
    s = PoseSession()
    s.handle(parse_message("START"), track=TRACK_NORMAL)
    s.handle(parse_message("POSE 0 0 0 0 1 0 0 0 0"))
    with pytest.raises(PoseProtocolError):
        s.handle(parse_message("POSE 10 0.1 0 0 1 0 0 0 1"))  # limited
    # last good relative pose retained
    assert s.relative.x == pytest.approx(0.0)


def test_stop_clears_session():
    s = PoseSession()
    s.handle(parse_message("START"), track=TRACK_NORMAL)
    s.handle(parse_message("STOP"))
    assert not s.started
    assert s.origin is None

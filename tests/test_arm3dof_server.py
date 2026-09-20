"""Phone -> 3-DOF arm, assembled: pose packets in, servo commands out."""
import math
import time

import pytest

from arm3dof_server import Arm3Server, serve_viewer
from fade_ninja.arm3dof import ArmSpec
from fade_ninja.servo_link import FakeServo3Board, Servo3Driver

SPEC = ArmSpec()


def pose(i, x, y, z):
    return f"POSE {i * 20} {x:.5f} {y:.5f} {z:.5f} 1 0 0 0 0"


@pytest.fixture
def rig():
    board = FakeServo3Board(SPEC)
    hw = Servo3Driver(SPEC, board)
    hw.home()
    srv = Arm3Server(SPEC, port=0, hardware=hw)
    yield srv, board, hw
    srv.close()


def drive(srv, n=60, amp=0.12):
    srv._handle("START 0", None)
    for i in range(n):
        t = i / 50.0
        srv._handle(pose(i, amp * math.sin(t * 2), amp * math.sin(t * 1.3),
                         -0.04), None)


def test_pose_packets_reach_the_servos(rig):
    srv, board, _ = rig
    start = tuple(board.position)
    drive(srv)
    assert tuple(board.position) != start


def test_start_engages_without_moving_the_arm(rig):
    srv, board, _ = rig
    before = tuple(board.position)
    srv._handle("START 0", None)
    srv._handle(pose(0, 0, 0, 0), None)
    assert tuple(board.position) == pytest.approx(before, abs=0.01)


def test_stop_releases_the_clutch(rig):
    srv, _, _ = rig
    drive(srv, n=10)
    srv._handle("STOP", None)
    assert srv.teleop.engaged is False
    held = srv.joints
    srv._handle(pose(99, 0.4, 0.4, 0), None)      # refused: POSE after STOP
    assert srv.joints == held


def test_servo_commands_stay_inside_the_arm_limits(rig):
    srv, board, _ = rig
    srv._handle("START 0", None)
    lim = SPEC.limits()
    for i in range(90):
        srv._handle(pose(i, 3.0 * math.sin(i / 4), 3.0, -3.0), None)
        for j in range(3):
            assert lim[j][0] - 1e-6 <= board.position[j] <= lim[j][1] + 1e-6


def test_recording_captures_three_joints(rig):
    srv, _, _ = rig
    srv.recorder.start()
    drive(srv, n=40)
    rows = srv.recorder.stop()
    assert len(rows) > 20
    assert len(rows[0]) == 6                      # t_ms, q1..q4, reachable
    assert rows[-1][4] == 0.0, "the unused fourth slot should stay zero"


def test_garbage_is_counted_not_crashed(rig):
    srv, _, _ = rig
    for junk in ("", "POSE nope", "HELLO", "POSE 1 2 3"):
        srv._handle(junk, None)
    assert srv.rejected >= 1
    assert srv.last_error


def test_status_is_json_safe_and_carries_the_spec(rig):
    import json
    srv, _, _ = rig
    drive(srv, n=5)
    st = srv.status_dict()
    json.dumps(st)
    assert st["spec"]["arm_len_mm"] == SPEC.arm_len_mm
    assert "tip_mm" in st


def test_a_quiet_phone_reads_as_stale(rig):
    srv, _, _ = rig
    drive(srv, n=5)
    assert srv.stale is False
    srv._last_rx -= 1.0
    assert srv.stale is True


def test_ping_is_answered_and_not_counted():
    """The phone cannot tell a working link from a black hole without a
    reply, and the probe must not show up as a rejected packet."""
    import socket
    srv = Arm3Server(ArmSpec(), port=0)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2.0)
        s.sendto(b"PING", ("127.0.0.1", srv.port))
        assert s.recvfrom(64)[0] == b"PONG"
        assert srv.packets == 0 and srv.rejected == 0
    finally:
        srv.close()


def test_garbage_gets_an_error_reply_not_silence():
    """Silence and a dropped packet look the same from the phone."""
    import socket
    srv = Arm3Server(ArmSpec(), port=0)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2.0)
        s.sendto(b"NONSENSE 1 2 3", ("127.0.0.1", srv.port))
        assert s.recvfrom(512)[0].startswith(b"ERR")
        assert srv.rejected == 1
    finally:
        srv.close()


def test_status_endpoint_serves_json():
    import json as _json
    import urllib.request
    srv = Arm3Server(ArmSpec(), port=0)
    httpd = serve_viewer(srv, 0)
    try:
        url = f"http://127.0.0.1:{httpd.server_address[1]}/status"
        with urllib.request.urlopen(url, timeout=5) as r:
            d = _json.loads(r.read())
        assert "packets" in d and "q1" in d
    finally:
        httpd.shutdown()
        srv.close()

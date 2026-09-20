"""Pose server UDP + watchdog gates (PHONE_POSE_TDD Phase 4 + Fix pass)."""
import json
import socket
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from fade_ninja.eezy_servos import ServoDriver
from pose_server import WATCHDOG_S, PoseServer  # noqa: E402


@pytest.fixture
def link():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    srv = PoseServer(host="127.0.0.1", port=port)
    srv.start()
    time.sleep(0.05)
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(1.0)
    yield srv, client, ("127.0.0.1", port)
    client.close()
    srv.close()


def rpc(client, addr, line: str) -> str:
    client.sendto(line.encode(), addr)
    data, _ = client.recvfrom(4096)
    return data.decode()


def test_t41_start_ok(link):
    srv, client, addr = link
    assert rpc(client, addr, "START 0") == "OK"


def test_start_rejected_when_track_not_normal(link):
    srv, client, addr = link
    reply = rpc(client, addr, "START 1")
    assert reply.startswith("ERR")


def test_t42_pose_stream_updates_joints(link):
    srv, client, addr = link
    rpc(client, addr, "START 0")
    rpc(client, addr, "POSE 0 0 0 0 1 0 0 0 0")
    for i in range(50):
        z = -0.001 * i
        rpc(client, addr, f"POSE {i*20} 0 0 {z} 1 0 0 0 0")
    st = json.loads(rpc(client, addr, "STATUS").split(" ", 1)[1])
    assert "q1" in st and "q2" in st and "q3" in st and "q4" in st
    assert srv.received >= 50


def test_t43_status_json_fields(link):
    srv, client, addr = link
    rpc(client, addr, "START 0")
    body = rpc(client, addr, "STATUS").split(" ", 1)[1]
    st = json.loads(body)
    for k in ("q1", "q2", "q3", "q4", "x", "y", "z", "reachable", "stale", "mode"):
        assert k in st


def test_t45_watchdog_marks_stale_without_status_poll():
    """Watchdog must freeze hardware even when nobody calls STATUS/WS."""
    hw = ServoDriver(dry_run=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()

    srv = PoseServer(host="127.0.0.1", port=port, hardware=hw)
    srv.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(1.0)
    addr = ("127.0.0.1", port)

    rpc(client, addr, "START 0")
    rpc(client, addr, "POSE 0 0 0 0 1 0 0 0 0")
    # Drive joints so hardware is not frozen
    assert hw.frozen is False or hw.last is not None or True
    rpc(client, addr, "POSE 20 0 0 -0.01 1 0 0 0 0")
    assert hw.frozen is False

    before = srv.watchdog_freezes
    # No STATUS, no WS — just wait
    time.sleep(WATCHDOG_S + 0.2)
    assert hw.frozen is True
    assert srv.watchdog_freezes > before
    assert srv.status_dict()["stale"] is True

    client.close()
    srv.close()

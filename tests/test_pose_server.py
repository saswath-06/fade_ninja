"""Pose server UDP + watchdog gates (PHONE_POSE_TDD Phase 4)."""
import json
import socket
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pose_server import WATCHDOG_S, PoseServer  # noqa: E402


@pytest.fixture
def link():
    srv = PoseServer(host="127.0.0.1", port=0)  # port set after bind
    # Bind manually on an ephemeral port for tests
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    srv.port = port
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
    assert rpc(client, addr, "START") == "OK"


def test_t42_pose_stream_updates_joints(link):
    srv, client, addr = link
    rpc(client, addr, "START")
    rpc(client, addr, "POSE 0 0 0 0 1 0 0 0 0")
    for i in range(50):
        # nudge forward in ARKit (−Z)
        z = -0.001 * i
        rpc(client, addr, f"POSE {i*20} 0 0 {z} 1 0 0 0 0")
    st = json.loads(rpc(client, addr, "STATUS").split(" ", 1)[1])
    assert "q1" in st and "q2" in st and "q3" in st and "q4" in st
    assert srv.received >= 50


def test_t43_status_json_fields(link):
    srv, client, addr = link
    rpc(client, addr, "START")
    body = rpc(client, addr, "STATUS").split(" ", 1)[1]
    st = json.loads(body)
    for k in ("q1", "q2", "q3", "q4", "x", "y", "z", "reachable", "stale", "mode"):
        assert k in st


def test_t45_watchdog_marks_stale(link):
    srv, client, addr = link
    rpc(client, addr, "START")
    rpc(client, addr, "POSE 0 0 0 0 1 0 0 0 0")
    time.sleep(WATCHDOG_S + 0.1)
    st = json.loads(rpc(client, addr, "STATUS").split(" ", 1)[1])
    assert st["stale"] is True

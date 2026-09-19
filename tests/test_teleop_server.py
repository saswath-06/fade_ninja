"""TeleopServer: the clutch wiring between phone messages and the arm.

Drives the server over real UDP with the same message shapes FadePose sends.
"""
import json
import socket
import time

import pytest

from fadegpt.head import Head
from fadegpt.rig import Rig
from teleop_sim import TeleopServer


@pytest.fixture
def srv():
    rig = Rig(head=Head(), seed=0)
    rig.arm.home()
    rig.goto(20.0, 90.0, 15.0)
    s = TeleopServer(rig, port=0)
    s.port = s.sock.getsockname()[1]
    yield s
    s.close()


def send(srv, line, wait=True):
    c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    c.settimeout(2.0)
    c.sendto(line.encode(), ("127.0.0.1", srv.port))
    try:
        if wait:
            c.recvfrom(256)
    except socket.timeout:
        pass
    finally:
        c.close()


def settle(srv, n=30):
    for _ in range(n):
        srv.tick()


def test_status_is_json_serialisable(srv):
    json.dumps(srv.status_dict())      # the sim WebSocket depends on this


def test_pose_before_start_is_rejected_not_obeyed(srv):
    before = srv.rig.arm.pos.copy()
    send(srv, "POSE 0 0.5 0.5 0 1 0 0 0 0")
    settle(srv)
    assert srv.rejected >= 1
    assert srv.rig.arm.pos == pytest.approx(before, abs=0.2)


def test_start_engages_the_clutch_without_moving_the_arm(srv):
    before = srv.rig.arm.pos.copy()
    send(srv, "START")
    send(srv, "POSE 0 0 0 0 1 0 0 0 0")     # origin sample
    settle(srv)
    assert srv.teleop.status()["engaged"] is True
    assert srv.rig.arm.pos == pytest.approx(before, abs=0.3)


def test_moving_the_phone_up_drives_phi(srv):
    send(srv, "START")
    send(srv, "POSE 0 0 0 0 1 0 0 0 0")
    settle(srv, 5)
    phi_before = float(srv.rig.encoders[0])
    send(srv, "POSE 100 0 0.1 0 1 0 0 0 0")
    settle(srv, 60)
    assert float(srv.rig.encoders[0]) > phi_before + 5


def test_stop_releases_the_clutch_and_saves_a_take(srv, tmp_path, monkeypatch):
    import teleop_sim
    monkeypatch.setattr(teleop_sim, "OUT", tmp_path)
    send(srv, "START")
    send(srv, "POSE 0 0 0 0 1 0 0 0 0")
    for i in range(40):
        send(srv, f"POSE {i*20} 0 {i*0.002:.4f} 0 1 0 0 0 0", wait=False)
        srv.tick()
    send(srv, "STOP")
    settle(srv, 5)
    assert srv.teleop.status()["engaged"] is False
    assert len(srv.saved) == 1
    assert srv.saved[0].exists()
    assert srv.saved[0].read_text().startswith("t_ms,phi,psi,theta,contact")


def test_quiet_phone_stops_the_arm(srv):
    """Dead-man: the link going silent must not leave the arm tracking."""
    send(srv, "START")
    send(srv, "POSE 0 0 0 0 1 0 0 0 0")
    send(srv, "POSE 20 0 0.2 0 1 0 0 0 0")
    settle(srv, 10)
    srv.last_rx -= 1.0                      # pretend the phone went away
    srv.tick()
    assert srv.rig.arm.mode == "idle"


def test_garbage_does_not_crash_the_server(srv):
    for junk in ("", "POSE nope", "HELLO", "POSE 1 2 3"):
        send(srv, junk, wait=False)
    time.sleep(0.2)
    settle(srv)
    json.dumps(srv.status_dict())
    assert srv.rejected >= 1

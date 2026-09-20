"""Browser pendant: the page is served over TLS and a real WebSocket client
drives the same packet pipeline as the native app.

A hand-rolled client exercises the actual frame codec (masked client frames,
the 126-length path, close frames) rather than stubbing it.
"""
import base64
import os
import socket
import ssl
import struct
import time

import pytest

from fade_ninja.pendant import RateMap
from fade_ninja.webpendant import WebPendantReceiver


@pytest.fixture(scope="module")
def server():
    rx = WebPendantReceiver(port=0)
    rx.port = rx._srv.getsockname()[1]
    yield rx
    rx.close()


def tls_connect(port):
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE      # self-signed, as Safari sees it
    raw = socket.create_connection(("127.0.0.1", port), timeout=5)
    return ctx.wrap_socket(raw, server_hostname="localhost")


class WSClient:
    def __init__(self, port):
        self.sock = tls_connect(port)
        key = base64.b64encode(os.urandom(16))
        self.sock.sendall(
            b"GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
            b"Connection: Upgrade\r\nSec-WebSocket-Key: " + key +
            b"\r\nSec-WebSocket-Version: 13\r\n\r\n")
        reply = self.sock.recv(1024)
        assert b"101" in reply, f"no upgrade: {reply[:80]!r}"

    def send(self, text: str):
        payload = text.encode()
        mask = os.urandom(4)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        n = len(payload)
        if n < 126:
            head = struct.pack("!BB", 0x81, 0x80 | n)
        else:
            head = struct.pack("!BBH", 0x81, 0x80 | 126, n)
        self.sock.sendall(head + mask + masked)

    def close(self):
        self.sock.sendall(struct.pack("!BB", 0x88, 0x80) + os.urandom(4))
        self.sock.close()


def wait_for(fn, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        got = fn()
        if got is not None:
            return got
        time.sleep(0.01)
    return None


def pend(seq, pitch=0.0, roll=0.0, slider=0.0, cut=0, rec=0):
    return f"PEND {seq} {seq * 20} {pitch:.2f} {roll:.2f} {slider:.3f} {cut}{rec}"


def test_page_is_served_over_tls(server):
    sock = tls_connect(server.port)
    sock.sendall(b"GET / HTTP/1.1\r\nHost: localhost\r\n\r\n")
    data = b""
    while b"</html>" not in data and len(data) < 200_000:
        chunk = sock.recv(8192)
        if not chunk:
            break
        data += chunk
    sock.close()
    assert b"200 OK" in data
    assert b"FADE PENDANT" in data
    # the iOS motion-permission call is the whole reason for HTTPS
    assert b"requestPermission" in data


def test_websocket_carries_pendant_packets(server):
    ws = WSClient(server.port)
    ws.send(pend(1, pitch=25.0, roll=-10.0, slider=0.5, cut=1, rec=1))
    got = wait_for(server.latest)
    assert got is not None, "packet never arrived"
    assert got.pitch == pytest.approx(25.0)
    assert got.roll == pytest.approx(-10.0)
    assert got.slider == pytest.approx(0.5)
    assert got.cut and got.rec
    ws.close()


def test_browser_packets_drive_the_same_rate_map(server):
    ws = WSClient(server.port)
    ws.send(pend(50, pitch=40.0, roll=-40.0, slider=1.0))
    got = wait_for(lambda: server.latest() if
                   (server.latest() and server.latest().seq >= 50) else None)
    assert got is not None
    m = RateMap()
    dphi, dpsi, dtheta = m.rates(got)
    assert dphi == pytest.approx(m.phi_rate)
    assert dpsi == pytest.approx(-m.psi_rate)
    assert dtheta == pytest.approx(m.theta_rate)
    ws.close()


def test_watchdog_applies_to_the_browser_link_too(server):
    ws = WSClient(server.port)
    ws.send(pend(100, pitch=30.0))
    assert wait_for(lambda: server.latest() if
                    (server.latest() and server.latest().seq >= 100) else None)
    time.sleep(0.35)
    assert server.latest() is None, "a quiet browser must read as dead"
    ws.close()


def test_long_frames_and_garbage_do_not_kill_the_link(server):
    ws = WSClient(server.port)
    ws.send("x" * 200)                    # >126: exercises the 16-bit length
    ws.send("not a pendant packet")
    ws.send(pend(200, pitch=12.0))
    got = wait_for(lambda: server.latest() if
                   (server.latest() and server.latest().seq >= 200) else None)
    assert got is not None and got.pitch == pytest.approx(12.0)
    ws.close()

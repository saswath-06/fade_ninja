"""Browser teach pendant: the phone's gyroscope over Safari, no app install.

iOS exposes the same sensors to the browser as to a native app, through
DeviceOrientationEvent. This serves a page to the phone and takes its
orientation stream over a WebSocket, producing the same PendantPacket the
native app produces — so pendant.RateMap and everything downstream is shared.

Two iOS constraints shape this module:

1. DeviceOrientationEvent.requestPermission() needs a *secure context*, so
   the page must be served over HTTPS. We generate a self-signed cert; Safari
   warns once and you tap through.
2. Browsers cannot send UDP, hence WebSocket. The frame codec here is the
   minimal client->server subset, which keeps this dependency-free.
"""
from __future__ import annotations

import base64
import hashlib
import socket
import ssl
import struct
import subprocess
import threading
import time
from pathlib import Path

from .pendant import WATCHDOG_S, PendantPacket

WS_MAGIC = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
CERT_DIR = Path.home() / ".cache" / "fadegpt"


# --------------------------------------------------------------- TLS cert

def ensure_cert() -> tuple[Path, Path]:
    """Self-signed cert for the LAN address. Safari warns once, then the page
    counts as a secure context and the motion API unlocks."""
    CERT_DIR.mkdir(parents=True, exist_ok=True)
    cert, key = CERT_DIR / "pendant.crt", CERT_DIR / "pendant.key"
    if cert.exists() and key.exists():
        return cert, key
    subprocess.run(
        ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
         "-keyout", str(key), "-out", str(cert), "-days", "365",
         "-subj", "/CN=fadegpt-pendant",
         "-addext", "subjectAltName=IP:0.0.0.0,DNS:localhost"],
        check=True, capture_output=True)
    return cert, key


# ---------------------------------------------------------- WebSocket bits

def _handshake(conn: ssl.SSLSocket, request: bytes) -> bool:
    key = None
    for line in request.split(b"\r\n"):
        if line.lower().startswith(b"sec-websocket-key:"):
            key = line.split(b":", 1)[1].strip()
    if key is None:
        return False
    accept = base64.b64encode(hashlib.sha1(key + WS_MAGIC).digest())
    conn.sendall(b"HTTP/1.1 101 Switching Protocols\r\n"
                 b"Upgrade: websocket\r\nConnection: Upgrade\r\n"
                 b"Sec-WebSocket-Accept: " + accept + b"\r\n\r\n")
    return True


def _recv_exact(conn: ssl.SSLSocket, n: int) -> bytes | None:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _read_frame(conn: ssl.SSLSocket) -> str | None:
    """One client->server text frame, or None when the socket closes."""
    head = _recv_exact(conn, 2)
    if head is None:
        return None
    opcode = head[0] & 0x0F
    masked = bool(head[1] & 0x80)
    length = head[1] & 0x7F
    if length == 126:
        ext = _recv_exact(conn, 2)
        if ext is None:
            return None
        length = struct.unpack(">H", ext)[0]
    elif length == 127:
        ext = _recv_exact(conn, 8)
        if ext is None:
            return None
        length = struct.unpack(">Q", ext)[0]
    if length > 4096:            # a pendant sample is ~60 bytes
        return None
    mask = _recv_exact(conn, 4) if masked else b"\x00\x00\x00\x00"
    if mask is None:
        return None
    payload = _recv_exact(conn, length) if length else b""
    if payload is None:
        return None
    if masked:
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    if opcode == 0x8:            # close
        return None
    if opcode != 0x1:            # ignore ping/pong/binary
        return ""
    return payload.decode("utf-8", "replace")


# ------------------------------------------------------------- the server

class WebPendantReceiver:
    """Same surface as PendantReceiver: .latest(), .live, .close()."""

    def __init__(self, port: int = 8443, page: Path | None = None):
        self.port = port
        self.page = page or (Path(__file__).parent.parent / "phone" /
                             "pendant.html")
        self._latest: PendantPacket | None = None
        self._stamp = 0.0
        self._lock = threading.Lock()
        self._running = True
        self.received = 0
        self.dropped = 0
        self._last_seq = -1
        self.clients = 0

        cert, key = ensure_cert()
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert, key)
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", port))
        srv.listen(8)
        srv.settimeout(0.5)
        self._srv = srv
        self._ctx = ctx
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self) -> None:
        while self._running:
            try:
                raw, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._serve, args=(raw,),
                             daemon=True).start()

    def _serve(self, raw: socket.socket) -> None:
        try:
            conn = self._ctx.wrap_socket(raw, server_side=True)
        except (ssl.SSLError, OSError):
            raw.close()
            return
        try:
            conn.settimeout(10.0)
            request = conn.recv(2048)
            if not request:
                return
            if b"upgrade: websocket" in request.lower():
                if not _handshake(conn, request):
                    return
                self.clients += 1
                conn.settimeout(None)
                self._pump(conn)
                self.clients -= 1
            else:
                body = self.page.read_bytes()
                conn.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: text/html; "
                             b"charset=utf-8\r\nContent-Length: " +
                             str(len(body)).encode() +
                             b"\r\nCache-Control: no-store\r\n\r\n" + body)
        except (OSError, ssl.SSLError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _pump(self, conn: ssl.SSLSocket) -> None:
        while self._running:
            try:
                line = _read_frame(conn)
            except (OSError, ssl.SSLError):
                return
            if line is None:
                return
            if not line:
                continue
            try:
                pkt = PendantPacket.parse(line)
            except ValueError:
                continue
            with self._lock:
                if self._last_seq >= 0 and pkt.seq > self._last_seq + 1:
                    self.dropped += pkt.seq - self._last_seq - 1
                self._last_seq = pkt.seq
                self._latest = pkt
                self._stamp = time.monotonic()
                self.received += 1

    def latest(self) -> PendantPacket | None:
        with self._lock:
            if self._latest is None:
                return None
            if time.monotonic() - self._stamp > WATCHDOG_S:
                return None
            return self._latest

    @property
    def live(self) -> bool:
        return self.latest() is not None

    def close(self) -> None:
        self._running = False
        try:
            self._srv.close()
        except OSError:
            pass

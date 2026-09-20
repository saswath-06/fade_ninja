"""UDP pose teleop server (PHONE_POSE_TDD Phase 4 + Fix pass).

Listens for ARKit POSE datagrams, runs PoseMapper, replies to STATUS,
runs a dedicated 20 Hz watchdog (independent of browser polling), and
serves tools/eezy_sim.html over a tiny HTTP+WS port.
"""
from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from fadegpt.eezy_servos import ServoDriver
from fadegpt.pendant import local_ip
from fadegpt.pose_mapper import PoseMapper
from fadegpt.pose_protocol import (
    PoseProtocolError,
    PoseSession,
    parse_message,
)

DEFAULT_PORT = 8463
WATCHDOG_S = 0.25
WATCHDOG_HZ = 20.0
SIM_HTTP_PORT = 8464
SIM_PAGE = Path(__file__).parent / "tools" / "arm4dof_sim.html"


class PoseServer:
    def __init__(self, host: str = "0.0.0.0", port: int = DEFAULT_PORT,
                 scale: float = 0.3, hardware: ServoDriver | None = None):
        self.host = host
        self.port = port
        self.session = PoseSession()
        self.mapper = PoseMapper(scale=scale)
        self.hardware = hardware
        self._lock = threading.Lock()
        self._last_pose_mono = 0.0
        self._mode = "idle"
        self._stale = False
        self._running = False
        self._sock: socket.socket | None = None
        self.received = 0
        self.watchdog_freezes = 0

    # ------------------------------------------------------------- state

    def _eval_watchdog(self) -> None:
        """Called by the timer thread — never depends on STATUS/WS clients."""
        with self._lock:
            stale = (self._mode == "tracking"
                     and (time.monotonic() - self._last_pose_mono) > WATCHDOG_S)
            self._stale = stale
            if stale and self.hardware is not None and not self.hardware.frozen:
                self.hardware.freeze()
                self.watchdog_freezes += 1

    def status_dict(self) -> dict:
        """Pure read of current state (no side effects)."""
        with self._lock:
            st = self.mapper.state()
            return {
                "mode": "stale" if self._stale else self._mode,
                "stale": self._stale,
                "started": self.session.started,
                "q1": st.q1, "q2": st.q2, "q3": st.q3, "q4": st.q4,
                "x": st.tip_x, "y": st.tip_y, "z": st.tip_z,
                "reachable": st.reachable,
                "phone_x": self.session.relative.x,
                "phone_y": self.session.relative.y,
                "phone_z": self.session.relative.z,
            }

    def handle_line(self, line: str) -> str:
        try:
            msg = parse_message(line)
        except PoseProtocolError as e:
            return f"ERR {e}"

        if msg.kind == "STATUS":
            return "OK " + json.dumps(self.status_dict(), separators=(",", ":"))

        try:
            if msg.kind == "START":
                # Track comes from the wire: START <track>
                ack = self.session.handle(msg)
                self.mapper.reset()
                with self._lock:
                    self._mode = "tracking"
                    self._stale = False
                    self._last_pose_mono = time.monotonic()
                return ack

            if msg.kind == "STOP":
                ack = self.session.handle(msg)
                with self._lock:
                    self._mode = "idle"
                    self._stale = False
                if self.hardware is not None:
                    self.hardware.freeze()
                return ack

            # POSE
            ack = self.session.handle(msg)
            with self._lock:
                self._last_pose_mono = time.monotonic()
                self._stale = False
                st = self.mapper.update(self.session.relative)
                self.received += 1
                if self.hardware is not None:
                    self.hardware.set_joints(st.q1, st.q2, st.q3, st.q4)
            return ack
        except PoseProtocolError as e:
            return f"ERR {e}"

    # ------------------------------------------------------------- loops

    def start(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.settimeout(0.1)
        self._sock = sock
        self._running = True
        threading.Thread(target=self._udp_loop, daemon=True).start()
        threading.Thread(target=self._watchdog_loop, daemon=True).start()

    def _watchdog_loop(self) -> None:
        period = 1.0 / WATCHDOG_HZ
        while self._running:
            self._eval_watchdog()
            time.sleep(period)

    def _udp_loop(self) -> None:
        assert self._sock is not None
        while self._running:
            try:
                data, addr = self._sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                line = data.decode("utf-8", "replace").strip()
            except Exception:
                continue
            reply = self.handle_line(line)
            try:
                self._sock.sendto(reply.encode(), addr)
            except OSError:
                pass

    def close(self) -> None:
        self._running = False
        if self._sock is not None:
            self._sock.close()
            self._sock = None


# --------------------------------------------------------------- HTTP + WS

def _ws_accept(key: bytes) -> bytes:
    import base64
    import hashlib
    magic = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
    return base64.b64encode(hashlib.sha1(key + magic).digest())


def _ws_send_text(conn: socket.socket, text: str) -> None:
    payload = text.encode()
    header = bytes([0x81, len(payload)]) if len(payload) < 126 else None
    if header is None:
        header = bytes([0x81, 126]) + len(payload).to_bytes(2, "big")
    conn.sendall(header + payload)


def serve_sim(server: PoseServer, http_port: int = SIM_HTTP_PORT,
              host: str = "0.0.0.0") -> ThreadingHTTPServer:
    def page_text() -> str:
        # read per request so editing the viewer only needs a browser refresh
        return (SIM_PAGE.read_text() if SIM_PAGE.exists()
                else "<h1>missing tools/arm4dof_sim.html</h1>")
    pose_ref = server

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            is_ws = (self.path.startswith("/ws")
                     or self.headers.get("Upgrade", "").lower() == "websocket")
            if is_ws:
                key = None
                for k, v in self.headers.items():
                    if k.lower() == "sec-websocket-key":
                        key = v.encode()
                if not key:
                    self.send_error(400)
                    return
                accept = _ws_accept(key)
                self.send_response(101, "Switching Protocols")
                self.send_header("Upgrade", "websocket")
                self.send_header("Connection", "Upgrade")
                self.send_header("Sec-WebSocket-Accept", accept.decode())
                self.end_headers()
                conn = self.connection
                try:
                    while True:
                        _ws_send_text(conn, json.dumps(pose_ref.status_dict()))
                        time.sleep(0.05)
                except Exception:
                    pass
                return  # never fall through to HTTP page write

            body = page_text().encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    httpd = ThreadingHTTPServer((host, http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(description="ARKit pose → EEZY IK server")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--http-port", type=int, default=SIM_HTTP_PORT)
    p.add_argument("--scale", type=float, default=0.3)
    p.add_argument("--hardware", action="store_true")
    p.add_argument("--dry-run", action="store_true",
                   help="with --hardware, print pulses, no GPIO")
    args = p.parse_args(argv)

    hw = None
    if args.hardware or args.dry_run:
        hw = ServoDriver(dry_run=True if args.dry_run or not args.hardware
                         else False)

    srv = PoseServer(host=args.host, port=args.port, scale=args.scale,
                     hardware=hw)
    srv.start()
    httpd = serve_sim(srv, args.http_port, host=args.host)
    print(f"Pose UDP  udp://{local_ip()}:{args.port}")
    print(f"Sim UI    http://{local_ip()}:{args.http_port}/")
    print("Send START <track> then POSE… from ios/FadePose or tools/fake_phone.py")
    print("(Server ACKs every datagram; the phone may ignore replies.)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        httpd.shutdown()


if __name__ == "__main__":
    main()

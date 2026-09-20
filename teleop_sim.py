"""ARKit phone pose -> fade robot, with a live 3D view of the arm.

    uv run python teleop_sim.py            # sim arm + 3D viewer
    uv run python teleop_sim.py --hardware /dev/ttyACM0

Point FadePose at this machine (same UDP port its README uses), press
Connect, then Start. Start/Stop is the clutch: while started, moving the
phone moves the arm; stopping lets you reposition your hand without the arm
following. Open the printed http:// URL to watch the arm.

Each engaged stroke is recorded from the ARM's encoders and saved, so a
session here produces ordinary teach logs that replay with teach_phone.py.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from fade_ninja.head import Head
from fade_ninja.interfaces import TICK_S
from fade_ninja.pendant import local_ip
from fade_ninja.pose_protocol import PoseProtocolError, PoseSession, parse_message
from fade_ninja.pose_teleop import PoseTeleop, TeleopConfig
from fade_ninja.rig import Rig
from pose_server import _ws_accept, _ws_send_text   # reuse the tested helpers

DEFAULT_UDP_PORT = 8463
DEFAULT_HTTP_PORT = 8465
VIEWER = Path(__file__).parent / "tools" / "arm_sim.html"
OUT = Path(__file__).parent / "out"


class TeleopServer:
    def __init__(self, rig: Rig, port: int = DEFAULT_UDP_PORT,
                 config: TeleopConfig | None = None):
        self.rig = rig
        self.teleop = PoseTeleop(config)
        self.session = PoseSession()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)
        self.port = port

        self._lock = threading.Lock()
        self._running = True
        self.packets = 0
        self.rejected = 0
        self.last_error = ""       # why the newest bad packet was refused
        self.last_bad_line = ""
        self.last_rx = 0.0
        self.phone = (0.0, 0.0, 0.0)
        self.saved: list[Path] = []
        self._target: tuple[float, float, float] | None = None
        threading.Thread(target=self._udp_loop, daemon=True).start()

    # ------------------------------------------------------------- inbound

    def _udp_loop(self) -> None:
        while self._running:
            try:
                data, addr = self.sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break
            for line in data.decode("utf-8", "replace").splitlines():
                if line.strip():
                    self._handle(line.strip(), addr)

    def _handle(self, line: str, addr) -> None:
        try:
            msg = parse_message(line)
            ack = self.session.handle(msg)
        except PoseProtocolError as e:
            with self._lock:
                self.rejected += 1
                if str(e) != self.last_error:
                    print(f"  ! rejected: {e}   <- {line[:70]}")
                self.last_error = str(e)
                self.last_bad_line = line[:70]
            return
        try:
            self.sock.sendto(ack.encode(), addr)
        except OSError:
            pass

        with self._lock:
            self.packets += 1
            self.last_rx = time.monotonic()

            if msg.kind == "START":
                # clutch in: anchor the phone's fresh origin to where the arm
                # is right now, so nothing jumps
                enc = self.rig.encoders
                self.teleop.engage(0.0, 0.0, 0.0,
                                   float(enc[0]), float(enc[1]), float(enc[2]))
                self.rig.recorder.start()
                self.rig.clipper_on = True
                print("  clutch IN — recording")
            elif msg.kind == "STOP":
                self.teleop.disengage()
                self._target = None
                self.rig.clipper_on = False
                self._save_take()
                print("  clutch OUT")
            elif msg.kind == "POSE":
                rel = self.session.relative
                self.phone = (rel.x, rel.y, rel.z)
                self._target = self.teleop.update(
                    rel.x, rel.y, rel.z, rel.qw, rel.qx, rel.qy, rel.qz,
                    track=msg.track)

    def _save_take(self) -> None:
        if not self.rig.recorder.recording:
            return
        log = self.rig.recorder.stop()
        if len(log) < 10:
            return
        OUT.mkdir(exist_ok=True)
        path = OUT / f"pose_take_{len(self.saved) + 1}.csv"
        path.write_text(log.to_csv())
        self.saved.append(path)
        print(f"  saved {len(log)} samples -> {path}")

    # -------------------------------------------------------------- motion

    def tick(self) -> None:
        with self._lock:
            target = self._target
            stale = time.monotonic() - self.last_rx > 0.25 if self.packets else True
        if target is not None and not stale:
            self.rig.arm.track(*target)
        else:
            self.rig.arm.stop()          # dead-man: quiet phone holds position
        self.rig.tick()

    # -------------------------------------------------------------- status

    def status_dict(self) -> dict:
        with self._lock:
            enc = self.rig.encoders
            stale = time.monotonic() - self.last_rx > 0.25 if self.packets else True
            return {
                "joints": {"phi": round(float(enc[0]), 2),
                           "psi": round(float(enc[1]), 2),
                           "theta": round(float(enc[2]), 2)},
                "phone": {"x": round(self.phone[0], 4),
                          "y": round(self.phone[1], 4),
                          "z": round(self.phone[2], 4)},
                "teleop": self.teleop.status(),
                "packets": self.packets,
                "rejected": self.rejected,
                "last_error": self.last_error,
                "last_bad_line": self.last_bad_line,
                "stale": stale,
                "clipper": bool(self.rig.clipper_on),
                "takes": len(self.saved),
            }

    def close(self) -> None:
        self._running = False
        self.sock.close()


def serve_viewer(server: TeleopServer, http_port: int) -> ThreadingHTTPServer:
    def page_bytes() -> bytes:
        # read per request: editing the viewer then refreshing the browser is
        # the whole iteration loop, and the file is a few KB
        if VIEWER.exists():
            return VIEWER.read_bytes()
        return b"<h1>tools/arm_sim.html is missing</h1>"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.headers.get("Upgrade", "").lower() == "websocket":
                key = next((v.encode() for k, v in self.headers.items()
                            if k.lower() == "sec-websocket-key"), None)
                if not key:
                    self.send_error(400)
                    return
                self.send_response(101, "Switching Protocols")
                self.send_header("Upgrade", "websocket")
                self.send_header("Connection", "Upgrade")
                self.send_header("Sec-WebSocket-Accept", _ws_accept(key).decode())
                self.end_headers()
                try:
                    while True:
                        _ws_send_text(self.connection,
                                      json.dumps(server.status_dict()))
                        time.sleep(0.04)
                except OSError:
                    return
            body = page_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

    httpd = ThreadingHTTPServer(("0.0.0.0", http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def make_config(args) -> TeleopConfig:
    """Split out so the CLI's mapping onto TeleopConfig is testable: the
    server tests build TeleopServer directly and would not catch a rename."""
    return TeleopConfig(mode="metric" if args.metric else "proportional",
                        hand_span_mm=args.hand_span,
                        scale=args.scale,
                        yaw_offset_deg=args.yaw_offset,
                        auto_align=not args.no_auto_align)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=DEFAULT_UDP_PORT)
    ap.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT)
    ap.add_argument("--hardware", metavar="SERIAL_PORT")
    ap.add_argument("--hand-span", type=float, default=400.0,
                    help="mm of hand movement that sweeps a whole axis "
                         "(smaller = more sensitive)")
    ap.add_argument("--metric", action="store_true",
                    help="millimetre-exact motion instead of proportional; "
                         "true to scale but the rail is only 13cm of travel")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="extra multiplier on top of the mode")
    ap.add_argument("--yaw-offset", type=float, default=0.0,
                    help="trim, in degrees, if left/right feels rotated")
    ap.add_argument("--no-auto-align", action="store_true",
                    help="do not point 'forward' at the head on engage")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.hardware:
        from fade_ninja.hardware_arm import ArmConfig, HardwareArm
        rig = Rig(head=None)
        rig.arm = HardwareArm.open(ArmConfig(port=args.hardware))
    else:
        rig = Rig(head=Head(), seed=0)
    rig.arm.home()
    rig.goto(20.0, 90.0, 15.0)

    cfg = make_config(args)
    server = TeleopServer(rig, args.port, cfg)
    serve_viewer(server, args.http_port)

    ip = local_ip()
    # 127.0.0.1 rather than localhost: the HTTP server is IPv4-only, and
    # browsers that resolve localhost to ::1 first will hang on it
    print(f"\n  FadePose  ->  {ip}:{args.port}   (UDP)")
    print(f"  3D viewer ->  http://127.0.0.1:{args.http_port}/")
    print(f"                http://{ip}:{args.http_port}/   (from another device)\n")
    print("  Press Start in the app to engage the clutch; Stop to release")
    print("  and reposition your hand. Ctrl-C to quit.\n")

    next_tick = time.monotonic()
    try:
        while True:
            server.tick()
            next_tick += TICK_S
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("\n  stopping")
    finally:
        server._save_take()
        server.close()
        if server.saved:
            print(f"  {len(server.saved)} take(s) saved; replay with:")
            print(f"    uv run python teach_phone.py --replay {server.saved[-1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

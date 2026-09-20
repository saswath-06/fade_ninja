"""Phone -> the real 3-DOF arm.

    uv run python arm3dof_server.py                       # simulate
    uv run python arm3dof_server.py --servo-port /dev/ttyACM0
    uv run python arm3dof_server.py --dry-run             # print pulses only

Same shape as the 4-DOF server, cut down to the arm that actually got built:
base rotation, arm pitch, razor tilt. The phone speaks the same POSE protocol
FadePose already sends, so nothing changes on the phone.

Recording, the cut library and replay are the existing ones — a take here is
three joint angles instead of four, and everything downstream is unchanged.
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

from fade_ninja.arm3dof import ArmSpec, Teleop3DOF, TeleopConfig, reach_envelope
from fade_ninja.pendant import local_ip
from fade_ninja.pose_protocol import PoseProtocolError, PoseSession, parse_message
from fade_ninja.servo_link import FakeServo3Board, ServoCal, Servo3Driver
from fade_ninja.take import JointRecorder

DEFAULT_UDP = 8463
DEFAULT_HTTP = 8464
VIEWER = Path(__file__).parent / "tools" / "arm3dof_sim.html"
STALE_S = 0.25


class Arm3Server:
    def __init__(self, spec: ArmSpec, port: int = DEFAULT_UDP,
                 hardware=None, cfg: TeleopConfig | None = None):
        self.spec = spec
        self.teleop = Teleop3DOF(spec, cfg)
        self.session = PoseSession()
        self.hardware = hardware
        self.recorder = JointRecorder()
        self.joints = tuple(spec.home)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]

        self._lock = threading.Lock()
        self._running = True
        self.packets = 0
        self.rejected = 0
        self.last_error = ""
        self._last_rx = 0.0
        threading.Thread(target=self._udp_loop, daemon=True).start()

    # ------------------------------------------------------------ inbound

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
                self.last_error = str(e)
            return
        if addr is not None:
            try:
                self.sock.sendto(ack.encode(), addr)
            except OSError:
                pass      # the ack is courtesy; control must not depend on it

        with self._lock:
            self.packets += 1
            self._last_rx = time.monotonic()

            if msg.kind == "START":
                # clutch in: anchor the hand to where the arm is right now
                self.teleop.engage(0.0, 0.0, 0.0, *self.joints)
                print("  clutch IN")
            elif msg.kind == "STOP":
                self.teleop.disengage()
                print("  clutch OUT")
            elif msg.kind == "POSE":
                rel = self.session.relative
                got = self.teleop.update(rel.x, rel.y, rel.z, rel.qw, rel.qx,
                                         rel.qy, rel.qz, track=msg.track)
                if got is not None:
                    self.joints = got
                    self.recorder.tick(got[0], got[1], got[2], 0.0,
                                       not any(self.teleop.clamped))
                    if self.hardware is not None:
                        self.hardware.set_joints(*got)

    # ------------------------------------------------------------- status

    @property
    def stale(self) -> bool:
        return (time.monotonic() - self._last_rx > STALE_S) if self.packets \
            else True

    def status_dict(self) -> dict:
        with self._lock:
            st = self.teleop.status()
            st.update(packets=self.packets, rejected=self.rejected,
                      last_error=self.last_error, stale=self.stale,
                      recording=self.recorder.recording,
                      rec_samples=self.recorder.n,
                      spec={"arm_len_mm": self.spec.arm_len_mm,
                            "pivot_h_mm": self.spec.pivot_h_mm,
                            "q1": list(self.spec.q1_limits),
                            "q2": list(self.spec.q2_limits),
                            "q3": list(self.spec.q3_limits)})
            return st

    def close(self) -> None:
        self._running = False
        self.sock.close()


def serve_viewer(server: Arm3Server, http_port: int) -> ThreadingHTTPServer:
    from pose_server import _ws_accept, _ws_send_text

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
            body = (VIEWER.read_bytes() if VIEWER.exists()
                    else b"<h1>tools/arm3dof_sim.html is missing</h1>")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

    httpd = ThreadingHTTPServer(("0.0.0.0", http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", type=int, default=DEFAULT_UDP)
    p.add_argument("--http-port", type=int, default=DEFAULT_HTTP)
    p.add_argument("--servo-port", default=None, metavar="SERIAL",
                   help="drive the real servos, e.g. /dev/ttyACM0")
    p.add_argument("--dry-run", action="store_true",
                   help="print pulse widths; nothing moves")
    p.add_argument("--arm-len", type=float, default=150.0, help="mm")
    p.add_argument("--pivot-h", type=float, default=60.0, help="mm")
    p.add_argument("--hand-span", type=float, default=400.0,
                   help="mm of hand movement that sweeps a whole joint")
    p.add_argument("--invert-yaw", action="store_true")
    p.add_argument("--invert-pitch", action="store_true")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    spec = ArmSpec(arm_len_mm=args.arm_len, pivot_h_mm=args.pivot_h)
    cfg = TeleopConfig(hand_span_mm=args.hand_span,
                       invert_yaw=args.invert_yaw,
                       invert_pitch=args.invert_pitch)

    hw = None
    if args.servo_port:
        hw = Servo3Driver.open(spec, args.servo_port)
        hw.home()
        print(f"Servos    {args.servo_port} (homed)")
    elif args.dry_run:
        hw = Servo3Driver(spec, FakeServo3Board(spec), dry_run=True)
        hw.home()
        print("Servos    dry run (pulses printed, nothing moves)")

    srv = Arm3Server(spec, args.port, hw, cfg)
    serve_viewer(srv, args.http_port)

    env = reach_envelope(spec)
    print(f"\n  Arm       {spec.arm_len_mm:.0f}mm reach, pivot {spec.pivot_h_mm:.0f}mm up")
    print(f"  Envelope  height {env['min_height_mm']:.0f}-{env['max_height_mm']:.0f}mm, "
          f"reach {env['min_reach_mm']:.0f}-{env['max_reach_mm']:.0f}mm, "
          f"sweep {env['sweep_deg']:.0f} deg")
    print(f"  FadePose  {local_ip()}:{srv.port}   (UDP)")
    print(f"  Viewer    http://127.0.0.1:{args.http_port}/\n")
    print("  Press Start in the app to engage; Stop to release.  Ctrl-C quits.\n")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n  stopping")
    finally:
        if hw is not None:
            hw.freeze()
        srv.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Synthetic ARKit phone for pose_server (fire-and-forget, Fix pass Edit 5).

Sends UDP datagrams without waiting for ACKs (like the real phone).
STATUS is polled on a separate socket ~0.5 s so the stream is not stalled.
"""
from __future__ import annotations

import argparse
import math
import socket
import time


def fire(sock: socket.socket, addr, line: str) -> None:
    sock.sendto(line.encode(), addr)


def poll_status(sock: socket.socket, addr) -> str:
    sock.sendto(b"STATUS", addr)
    sock.settimeout(0.5)
    try:
        data, _ = sock.recvfrom(4096)
        return data.decode()
    except socket.timeout:
        return ""


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8463)
    p.add_argument("--circle", type=float, default=0.05,
                   help="circle radius in metres on the ARKit XZ (table) plane")
    p.add_argument("--vertical", type=float, default=0.0,
                   help="if >0, oscillate phone Y (up) by this amplitude (m)")
    p.add_argument("--sweep", type=float, default=0.0,
                   help="if >0, sweep forward (−Z) up to this distance (m) "
                        "to exercise reachable=false")
    p.add_argument("--drop", type=int, default=0,
                   help="drop every Nth POSE (0 = none) to exercise watchdog")
    p.add_argument("--track", type=int, default=0,
                   help="track code on POSE/START (0 normal, 1 limited, 2 relocalizing)")
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--seconds", type=float, default=8.0)
    p.add_argument("--pitch", type=float, default=0.0,
                   help="constant pitch deg for tilt-only demos")
    args = p.parse_args()

    addr = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    fire(sock, addr, f"START {args.track}")
    # Seed origin (absolute ARKit metres)
    fire(sock, addr, f"POSE 0 0 0 0 1 0 0 0 {args.track}")

    dt = 1.0 / args.hz
    t0 = time.monotonic()
    n = 0
    last_status = 0.0
    while time.monotonic() - t0 < args.seconds:
        t = time.monotonic() - t0
        if args.sweep > 0:
            # Ramp forward (−Z) then hold at max
            frac = min(t / max(args.seconds * 0.6, 1e-3), 1.0)
            x, y, z = 0.0, 0.0, -args.sweep * frac
        elif args.vertical > 0:
            x = 0.0
            y = args.vertical * math.sin(t)
            z = 0.0
        else:
            # Circle on ARKit XZ table plane
            x = args.circle * math.cos(t)
            z = -args.circle * math.sin(t)
            y = 0.0

        half = math.radians(args.pitch / 2.0)
        qw, qx = math.cos(half), math.sin(half)
        t_ms = int(t * 1000)
        line = (f"POSE {t_ms} {x:.5f} {y:.5f} {z:.5f} "
                f"{qw:.5f} {qx:.5f} 0 0 {args.track}")

        drop = args.drop > 0 and (n % args.drop == 0) and n > 0
        if not drop:
            fire(sock, addr, line)
        n += 1

        if time.monotonic() - last_status > 0.5:
            st = poll_status(status_sock, addr)
            if st:
                print(st[:220])
            last_status = time.monotonic()

        if args.drop > 0 and drop:
            # Gap longer than watchdog when dropping a burst
            time.sleep(max(dt, 0.3))
        else:
            time.sleep(dt)

    fire(sock, addr, "STOP")
    sock.close()
    status_sock.close()


if __name__ == "__main__":
    main()

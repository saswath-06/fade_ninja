#!/usr/bin/env python3
"""Synthetic ARKit phone for pose_server (PHONE_POSE_TDD Phase 4)."""
from __future__ import annotations

import argparse
import math
import socket
import time


def send(sock: socket.socket, addr, line: str) -> str:
    sock.sendto(line.encode(), addr)
    sock.settimeout(1.0)
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
                   help="circle radius in metres (ARKit XY plane ≈ table)")
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--seconds", type=float, default=8.0)
    p.add_argument("--pitch", type=float, default=0.0,
                   help="constant pitch deg for tilt-only demos")
    args = p.parse_args()

    addr = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(send(sock, addr, "START"))

    # Seed origin
    print(send(sock, addr, "POSE 0 0 0 0 1 0 0 0 0"))

    dt = 1.0 / args.hz
    t0 = time.monotonic()
    n = 0
    while time.monotonic() - t0 < args.seconds:
        t = time.monotonic() - t0
        # Circle in ARKit XZ (horizontal): x=r cos, z=-r sin (forward/back)
        x = args.circle * math.cos(t)
        z = -args.circle * math.sin(t)
        y = 0.0
        half = math.radians(args.pitch / 2.0)
        qw, qx = math.cos(half), math.sin(half)
        t_ms = int(t * 1000)
        line = f"POSE {t_ms} {x:.5f} {y:.5f} {z:.5f} {qw:.5f} {qx:.5f} 0 0 0"
        reply = send(sock, addr, line)
        n += 1
        if n % 25 == 0:
            st = send(sock, addr, "STATUS")
            print(st[:200])
        time.sleep(dt)

    print(send(sock, addr, "STOP"))
    sock.close()


if __name__ == "__main__":
    main()

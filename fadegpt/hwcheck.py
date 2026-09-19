"""Bench link check: prove the Pi <-> Arduino chain with NO motors attached.

The firmware counts the steps it generates whether or not drivers/motors are
wired, so this verifies the entire control path — serial, velocity commands,
position readback, rates — on a bare board.

    uv run python -m fadegpt.hwcheck /dev/ttyACM0          # full check
    uv run python -m fadegpt.hwcheck /dev/ttyACM0 --watch  # stream POS lines
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from .hardware_arm import ArmConfig, HardwareArm, SerialLink
from .interfaces import TICK_S


def run_ticks(arm: HardwareArm, seconds: float) -> None:
    n = int(seconds / TICK_S)
    for _ in range(n):
        t0 = time.monotonic()
        arm.tick()
        time.sleep(max(0.0, TICK_S - (time.monotonic() - t0)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port", help="e.g. /dev/ttyACM0 or /dev/ttyUSB0")
    ap.add_argument("--watch", action="store_true",
                    help="just stream POS lines until Ctrl-C")
    args = ap.parse_args()

    link = SerialLink(args.port, 115200)
    print(f"1. HELLO            -> {link.command('HELLO')}")

    if args.watch:
        try:
            while True:
                print(link.command("P"))
                time.sleep(0.2)
        except KeyboardInterrupt:
            return 0

    cfg = ArmConfig(port=args.port)
    arm = HardwareArm(link, cfg)
    link.command("Z")           # motor-free bench: zero instead of HOME
    arm.homed = True            # DO NOT do this once endstops exist
    arm.pos[:] = 0

    print("2. JOG 5 deg/s on phi for 2 s (dry: watch the step counter move)")
    arm.jog(5, 0, 0)
    run_ticks(arm, 0.5)         # accel ramp
    p0 = arm.pos.copy()
    run_ticks(arm, 2.0)
    rate = (arm.pos - p0) / 2.0
    print(f"   measured rate: {rate[0]:.2f} deg/s "
          f"({'PASS' if abs(rate[0] - 5) < 0.3 else 'FAIL'}; gate B1.5 wants 5)")

    print("3. MOVE 10 20 15 and settle")
    arm.move_to(10, 20, 15)
    t0 = time.monotonic()
    while arm.mode == "move" and time.monotonic() - t0 < 15:
        arm.tick()
        time.sleep(TICK_S)
    err = np.abs(arm.pos - [10, 20, 15])
    ok = arm.mode == "idle" and np.all(err < 2.0)
    print(f"   settled at {np.round(arm.pos, 2).tolist()} "
          f"({'PASS' if ok else 'FAIL'}; gate B1.3 wants within 2 deg)")

    print("4. STOP + watchdog: waiting 0.5 s with no commands...")
    arm.stop()
    time.sleep(0.5)
    print(f"   {link.command('P')}   (speeds are zeroed by the 250 ms watchdog)")

    print("\nLink is good. Next: wire ONE stepper driver and re-run — the "
          "same numbers should hold with real motion.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

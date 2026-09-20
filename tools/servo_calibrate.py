"""Find each servo's real pulse range, direction and offset.

The one part of the hardware bring-up that cannot be done in software: horn
mounting angle, direction and mechanical end-stops differ for every build, so
the numbers have to be measured with the arm in front of you.

    uv run python tools/servo_calibrate.py auto          # the 3-DOF arm
    uv run python tools/servo_calibrate.py auto --dof 4  # the 4-joint rig

For each joint it drives the servo to the joint's low limit, then its high
limit, and asks whether the arm actually went the way you expected. Nudge
until the linkage lines up with the angle you are commanding, then it prints
the numbers to paste into ServoConfig and into the firmware.

Safety: support the arm before starting. A servo finding its end-stop under
load can strip a horn, so move in small steps and stop if anything buzzes.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fade_ninja.arm3dof import ArmSpec  # noqa: E402
from fade_ninja.servo_link import (JOINT_LIMITS, JOINT_NAMES, ServoCal,  # noqa: E402
                                   ServoConfig, ServoDriver)

STEP_US = 25

ARM3_NAMES = ("q1 base rotation", "q2 arm pitch", "q3 razor tilt")


def ask(prompt: str, allowed: str) -> str:
    while True:
        got = input(prompt).strip().lower()
        if got in allowed.split("/"):
            return got
        print(f"  please answer one of: {allowed}")


def calibrate_joint(link, index: int, names, limits) -> ServoCal:
    name = names[index]
    lo, hi = limits[index]
    cal = ServoCal()
    print(f"\n=== {name}  (joint range {lo:.0f} to {hi:.0f} degrees)")
    print("    Support the arm. Commands move in 25us steps.")

    for edge, deg in (("LOW", lo), ("HIGH", hi)):
        print(f"\n  {edge} end: commanding {deg:.0f} degrees")
        us = cal.min_us if edge == "LOW" else cal.max_us
        while True:
            link.command(f"US {index} {us}")
            print(f"    pulse now {us}us")
            a = ask("    [k]eep  [+] wider  [-] narrower  : ", "k/+/-")
            if a == "k":
                break
            us += STEP_US if a == "+" else -STEP_US
            us = max(400, min(2600, us))
        if edge == "LOW":
            cal.min_us = us
        else:
            cal.max_us = us

    a = ask("\n  Did the arm move the way the angles said? [y/n] : ", "y/n")
    if a == "n":
        cal.invert = True
        print("    marked inverted")
    return cal


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port", nargs="?", default="auto",
                    help="serial port, or 'auto' to find the board")
    ap.add_argument("--dof", type=int, default=3, choices=(3, 4),
                    help="3 = the arm that got built (default), 4 = the rig")
    ap.add_argument("--joint", type=int, default=None,
                    help="calibrate only this joint")
    args = ap.parse_args()

    n = args.dof
    if n == 3:
        spec = ArmSpec()
        names, limits = ARM3_NAMES, spec.limits()
    else:
        names, limits = JOINT_NAMES, JOINT_LIMITS
    if args.joint is not None and not 0 <= args.joint < n:
        print(f"  --joint must be 0-{n - 1} for a {n}-DOF arm")
        return 1

    from fade_ninja.hardware_arm import LinkError, SerialLink
    sketch = "fade_ninja_arm3" if n == 3 else "fade_ninja_servos"
    try:
        link = SerialLink(args.port, 115200)
        hello = link.command("HELLO")
    except LinkError as e:
        print(f"  {e}")
        print(f"  if the port is right, flash firmware/{sketch} first")
        return 1
    print(f"board: {hello}  on {link.port}")
    if not hello.startswith("OK fade-ninja-servo"):
        print(f"  that is not the servo firmware — flash firmware/{sketch} first")
        return 1

    print("\nThis needs a firmware that accepts 'US <joint> <microseconds>'.")
    print("firmware/fade_ninja_arm3 does; the 4-servo reference sketch does not.")
    idxs = [args.joint] if args.joint is not None else list(range(n))
    cals = [ServoCal() for _ in range(n)]
    try:
        for i in idxs:
            cals[i] = calibrate_joint(link, i, names, limits)
    except KeyboardInterrupt:
        print("\nstopped")
        return 1

    print("\n\n--- paste into ServoConfig ---")
    print("cals=(")
    for i, c in enumerate(cals):
        print(f"    ServoCal(min_us={c.min_us}, max_us={c.max_us}, "
              f"invert={c.invert}, offset_deg={c.offset_deg}),"
              f"   # {names[i]}")
    print(")")
    print("\n--- paste into the sketch ---")
    d = f"[{n}]"
    print(f"const int   US_MIN{d} = {{ " + ", ".join(str(c.min_us) for c in cals) + " };")
    print(f"const int   US_MAX{d} = {{ " + ", ".join(str(c.max_us) for c in cals) + " };")
    print(f"const bool  INVERT{d} = {{ "
          + ", ".join("true" if c.invert else "false" for c in cals) + " };")
    return 0


if __name__ == "__main__":
    sys.exit(main())

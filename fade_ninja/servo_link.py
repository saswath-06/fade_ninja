"""Joint angles -> servos, over serial to an Arduino.

The earlier driver assumed the Pi would drive servos from its own GPIO. The
arm is driven by an Arduino instead, so this sends four joint angles down a
serial line and the board does the PWM. Same shape as `hardware_arm.py`: the
transport is injectable, and `FakeServoBoard` speaks the firmware's side of
the protocol in Python so everything here is testable with no board attached.

Wire protocol, 115200 baud, newline-terminated, one reply per command:

    HELLO                  -> OK fade-ninja-servo v1
    J <q1> <q2> <q3> <q4>  -> OK            degrees, the joint frame
    HOME                   -> OK            ease to the safe pose
    RELAX                  -> OK            detach servos (arm goes limp)
    P                      -> POS <q1> <q2> <q3> <q4>

Two guards live on this side and are repeated in firmware on purpose:

* **limits** — a command outside a joint's range is clamped here and clamped
  again on the board, because a laptop that crashes mid-command should not be
  able to ask a servo for a pose the linkage cannot reach.
* **slew** — a servo snaps to whatever pulse it is first given. Straight after
  connect, or after any gap in commands, the first target could be far from
  where gravity left the arm, so movement is capped per tick and the arm eases
  there instead of slamming.

Per-servo calibration is deliberately NOT guessed. The defaults below are
placeholders; `ServoCal` has to be measured on the bench, because horn
mounting angle, direction and end-stops differ for every build.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

from .eezy_ik import Q1_LIMITS, Q2_LIMITS, Q3_LIMITS, Q4_LIMITS

JOINT_LIMITS = (Q1_LIMITS, Q2_LIMITS, Q3_LIMITS, Q4_LIMITS)
JOINT_NAMES = ("q1", "q2", "q3", "q4")
SAFE_POSE = (0.0, 45.0, -90.0, 0.0)     # mid-workspace, matches Q_HOME
MAX_STEP_DEG = 4.0                       # per tick, at 50 Hz -> 200 deg/s
HELLO_REPLY = "OK fade-ninja-servo v1"


class ServoLinkError(RuntimeError):
    pass


@dataclass
class ServoCal:
    """One servo's mapping from joint degrees to pulse width.

    MEASURE THESE. The defaults assume a generic 500-2500us servo mounted
    with no offset and no reversal, which is true of approximately no real
    build. `tools/servo_calibrate.py` walks through finding them.
    """
    min_us: int = 500
    max_us: int = 2500
    invert: bool = False
    offset_deg: float = 0.0

    def pulse_for(self, deg: float, lo: float, hi: float) -> int:
        d = deg + self.offset_deg
        t = (d - lo) / (hi - lo) if hi > lo else 0.0
        t = max(0.0, min(1.0, t))
        if self.invert:
            t = 1.0 - t
        return int(round(self.min_us + t * (self.max_us - self.min_us)))


@dataclass
class ServoConfig:
    port: str = "/dev/ttyACM0"
    baud: int = 115200
    cals: tuple[ServoCal, ServoCal, ServoCal, ServoCal] = field(
        default_factory=lambda: (ServoCal(), ServoCal(), ServoCal(), ServoCal()))
    max_step_deg: float = MAX_STEP_DEG
    safe_pose: tuple[float, float, float, float] = SAFE_POSE


def clamp_joints(q: tuple[float, float, float, float]
                 ) -> tuple[tuple[float, float, float, float], bool]:
    """-> (clamped, was_clamped). Never raises: a live arm should stop at its
    limit, not abort mid-cut."""
    out, hit = [], False
    for i, v in enumerate(q):
        lo, hi = JOINT_LIMITS[i]
        c = min(hi, max(lo, float(v)))
        hit = hit or c != float(v)
        out.append(c)
    return (out[0], out[1], out[2], out[3]), hit


def step_toward(current: tuple[float, float, float, float],
                target: tuple[float, float, float, float],
                max_step: float) -> tuple[float, float, float, float]:
    """Move at most max_step degrees on each joint, so the arm eases rather
    than snaps after a reconnect."""
    out = []
    for c, t in zip(current, target):
        d = t - c
        out.append(c + (d if abs(d) <= max_step
                        else (max_step if d > 0 else -max_step)))
    return (out[0], out[1], out[2], out[3])


class ServoDriver:
    """What pose_server drives. Same surface as the old dry-run stub:
    `.frozen`, `.freeze()`, `.set_joints()`."""

    def __init__(self, link=None, config: ServoConfig | None = None,
                 dry_run: bool = False):
        self.cfg = config or ServoConfig()
        self.dry_run = dry_run or link is None
        self.link = link
        self.frozen = True
        self.last: tuple[float, float, float, float] | None = None
        self.pulses: tuple[int, int, int, int] | None = None
        self.clamped = False
        self.position = tuple(self.cfg.safe_pose)
        if self.link is not None:
            reply = self.link.command("HELLO")
            if not reply.startswith("OK fade-ninja-servo"):
                raise ServoLinkError(f"unexpected HELLO reply: {reply!r}")

    @classmethod
    def open(cls, config: ServoConfig | None = None) -> "ServoDriver":
        cfg = config or ServoConfig()
        from .hardware_arm import SerialLink
        return cls(SerialLink(cfg.port, cfg.baud), cfg)

    # ------------------------------------------------------------- motion

    def home(self) -> None:
        """Ease to the safe pose before anything else commands the arm."""
        if self.link is not None:
            reply = self.link.command("HOME", timeout=10.0)
            if not reply.startswith("OK"):
                raise ServoLinkError(f"homing refused: {reply}")
        self.position = tuple(self.cfg.safe_pose)
        self.frozen = False

    def set_joints(self, q1: float, q2: float, q3: float, q4: float) -> None:
        target, self.clamped = clamp_joints((q1, q2, q3, q4))
        # never jump: the first command after a connect can be far from where
        # the arm is actually sitting
        self.position = step_toward(self.position, target,
                                    self.cfg.max_step_deg)
        self.frozen = False
        self.last = self.position
        self.pulses = tuple(
            self.cfg.cals[i].pulse_for(self.position[i], *JOINT_LIMITS[i])
            for i in range(4))
        if self.link is not None:
            reply = self.link.command(
                "J %.2f %.2f %.2f %.2f" % self.position)
            if not reply.startswith("OK"):
                raise ServoLinkError(f"J refused: {reply}")
        elif self.dry_run:
            print(f"SERVO us={self.pulses} deg=("
                  + ", ".join(f"{v:.1f}" for v in self.position) + ")")

    def freeze(self) -> None:
        """Stop commanding. Servos keep holding — a limp arm falls."""
        self.frozen = True
        if self.link is None and self.dry_run:
            print("SERVO freeze")

    def relax(self) -> None:
        """Detach the servos. The arm WILL drop under its own weight; support
        it first."""
        if self.link is not None:
            self.link.command("RELAX")
        self.frozen = True

    def read_position(self) -> tuple[float, float, float, float] | None:
        if self.link is None:
            return self.position
        parts = self.link.command("P").split()
        if len(parts) != 5 or parts[0] != "POS":
            raise ServoLinkError(f"bad POS reply: {parts}")
        return tuple(float(x) for x in parts[1:])  # type: ignore[return-value]

    def close(self) -> None:
        if self.link is not None and hasattr(self.link, "close"):
            self.link.close()


class Servo3Driver:
    """Three servos: base rotation, arm pitch, razor tilt.

    Shares ServoCal, the clamping and the easing with the four-servo driver;
    the arm that actually got built has three joints, so this is the one that
    ships. Limits come from the ArmSpec rather than the 4-DOF constants.
    """

    def __init__(self, spec, link=None, cals=None,
                 max_step_deg: float = MAX_STEP_DEG, dry_run: bool = False):
        self.spec = spec
        self.link = link
        self.dry_run = dry_run or link is None
        self.cals = tuple(cals) if cals else tuple(ServoCal() for _ in range(3))
        self.max_step = max_step_deg
        self.frozen = True
        self.clamped = False
        self.position = tuple(spec.home)
        self.pulses: tuple[int, int, int] | None = None
        if self.link is not None:
            reply = self.link.command("HELLO")
            if not reply.startswith("OK fade-ninja-servo"):
                raise ServoLinkError(f"unexpected HELLO reply: {reply!r}")

    @classmethod
    def open(cls, spec, port: str, baud: int = 115200, **kw) -> "Servo3Driver":
        from .hardware_arm import SerialLink
        return cls(spec, SerialLink(port, baud), **kw)

    def home(self) -> None:
        if self.link is not None:
            reply = self.link.command("HOME", timeout=10.0)
            if not reply.startswith("OK"):
                raise ServoLinkError(f"homing refused: {reply}")
        self.position = tuple(self.spec.home)
        self.frozen = False

    def set_joints(self, q1: float, q2: float, q3: float) -> None:
        lim = self.spec.limits()
        target, self.clamped = [], False
        for i, v in enumerate((q1, q2, q3)):
            lo, hi = lim[i]
            c = min(hi, max(lo, float(v)))
            self.clamped = self.clamped or c != float(v)
            target.append(c)
        # never jump: the arm may be resting anywhere when the link opens
        self.position = tuple(
            step_toward(tuple(self.position) + (0.0,),
                        tuple(target) + (0.0,), self.max_step)[:3])
        self.frozen = False
        self.pulses = tuple(
            self.cals[i].pulse_for(self.position[i], *lim[i]) for i in range(3))
        if self.link is not None:
            reply = self.link.command("J %.2f %.2f %.2f" % self.position)
            if not reply.startswith("OK"):
                raise ServoLinkError(f"J refused: {reply}")
        elif self.dry_run:
            print(f"SERVO us={self.pulses} deg=("
                  + ", ".join(f"{v:.1f}" for v in self.position) + ")")

    def freeze(self) -> None:
        """Stop commanding. Servos keep holding — a limp arm falls."""
        self.frozen = True

    def relax(self) -> None:
        if self.link is not None:
            self.link.command("RELAX")
        self.frozen = True

    def close(self) -> None:
        if self.link is not None and hasattr(self.link, "close"):
            self.link.close()


class FakeServo3Board:
    """The firmware's side for a three-servo arm, in Python."""

    def __init__(self, spec, max_step_deg: float = MAX_STEP_DEG):
        self.spec = spec
        self.position = list(spec.home)
        self.max_step = max_step_deg
        self.attached = True
        self.clamped_count = 0

    def command(self, line: str, timeout: float = 2.0) -> str:
        if line == "HELLO":
            return HELLO_REPLY
        if line == "HOME":
            self.position = list(self.spec.home)
            self.attached = True
            return "OK"
        if line == "RELAX":
            self.attached = False
            return "OK"
        if line == "P":
            return "POS " + " ".join(f"{v:.2f}" for v in self.position)
        if line.startswith("J "):
            try:
                vals = [float(x) for x in line[2:].split()]
            except ValueError:
                return "ERR bad J"
            if len(vals) != 3:
                return "ERR J takes three angles"
            lim = self.spec.limits()
            target, hit = [], False
            for i, v in enumerate(vals):
                lo, hi = lim[i]
                c = min(hi, max(lo, v))
                hit = hit or c != v
                target.append(c)
            if hit:
                self.clamped_count += 1
            self.attached = True
            self.position = list(step_toward(tuple(self.position) + (0.0,),
                                             tuple(target) + (0.0,),
                                             self.max_step)[:3])
            return "OK"
        return "ERR unknown"


class FakeServoBoard:
    """The firmware's side of the protocol, in Python.

    Mirrors what the sketch must do — clamp to limits, ease toward the target,
    hold on silence — so the driver is testable and the sketch has something
    unambiguous to match.
    """

    def __init__(self, max_step_deg: float = MAX_STEP_DEG):
        self.position = list(SAFE_POSE)
        self.max_step = max_step_deg
        self.attached = True
        self.commands = 0
        self.clamped_count = 0

    def command(self, line: str, timeout: float = 2.0) -> str:
        self.commands += 1
        if line == "HELLO":
            return HELLO_REPLY
        if line == "HOME":
            self.position = list(SAFE_POSE)
            self.attached = True
            return "OK"
        if line == "RELAX":
            self.attached = False
            return "OK"
        if line == "P":
            return "POS " + " ".join(f"{v:.2f}" for v in self.position)
        if line.startswith("J "):
            try:
                vals = [float(x) for x in line[2:].split()]
            except ValueError:
                return "ERR bad J"
            if len(vals) != 4:
                return "ERR J takes four angles"
            target, hit = clamp_joints((vals[0], vals[1], vals[2], vals[3]))
            if hit:
                self.clamped_count += 1
            self.attached = True
            self.position = list(step_toward(tuple(self.position), target,
                                             self.max_step))
            return "OK"
        return "ERR unknown"

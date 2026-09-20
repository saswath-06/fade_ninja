"""Pi-side driver for the Arduino motor board — drop-in for VirtualArm.

The Pi keeps the brains: the same tested velocity planner (plan_velocity),
joint limits in degrees, and mode logic. Each 20 ms tick it sends the Arduino
one velocity command and reads back step counters + switches.

Everything is testable without hardware via FakeArduino, which implements the
firmware's serial contract in Python.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .arm import AMAX, HI, LO, VMAX, ArmError, move_settled, plan_velocity
from .interfaces import TICK_S


@dataclass
class ArmConfig:
    port: str = "/dev/ttyACM0"       # Uno: ttyACM*, Nano clones: ttyUSB*
    baud: int = 115200
    # steps per degree of JOINT motion. Placeholder = NEMA17, 1.8 deg/step,
    # 16 microsteps, direct drive (3200/360). MEASURE at gate A1 — belt or
    # gear ratios multiply this.
    steps_per_deg: tuple[float, float] = (8.889, 8.889)   # phi, psi
    invert: tuple[bool, bool] = (False, False)


class LinkError(RuntimeError):
    pass


class HardwareArm:
    """Same public surface as VirtualArm: home/move_to/jog/track/stop/tick/
    encoders/status.  Rig(head=None) + HardwareArm = the real bench."""

    def __init__(self, link, config: ArmConfig | None = None):
        self.cfg = config or ArmConfig()
        self.link = link                      # anything with .command(str)->str
        # cmd is the integrated commanded position — the control loop plans
        # against THIS, never against the readback: closing the loop on
        # step counters with a cycle of serial latency limit-cycles around
        # the target (found in emulation before it found the bench).
        self.cmd = LO.copy()
        self.pos = LO.copy()                  # measured: step counters
        self.vel = np.zeros(3)
        self.mode = "idle"
        self.target = self.cmd.copy()
        self.jog_rates = np.zeros(3)
        self.homed = False
        self.contact = False
        self.endstops = (False, False)
        hello = self.link.command("HELLO")
        if not hello.startswith("OK robot-barber-arm"):
            raise LinkError(f"unexpected HELLO reply: {hello!r}")
        self.link.command("STOP")
        self._read_position()

    @classmethod
    def open(cls, config: ArmConfig | None = None) -> "HardwareArm":
        cfg = config or ArmConfig()
        return cls(SerialLink(cfg.port, cfg.baud), cfg)

    # ------------------------------------------------------------- commands

    def home(self) -> None:
        reply = self.link.command("HOME", timeout=45.0)
        if not reply.startswith("OK"):
            raise ArmError(f"homing failed: {reply}")
        self.cmd = LO.copy()
        self.pos = LO.copy()
        self.vel = np.zeros(3)
        self.mode = "idle"
        self.homed = True

    def move_to(self, phi: float, psi: float, theta: float) -> None:
        if not self.homed:
            raise ArmError("not homed")
        t = np.array([phi, psi, theta], dtype=float)
        if np.any(t < LO) or np.any(t > HI):
            raise ArmError(f"MOVE target {t.tolist()} outside joint limits")
        self.target = t
        self.mode = "move"

    def jog(self, dphi: float, dpsi: float, dtheta: float) -> None:
        if not self.homed:
            raise ArmError("not homed")
        self.jog_rates = np.clip([dphi, dpsi, dtheta], -VMAX, VMAX)
        self.mode = "jog"

    def track(self, phi: float, psi: float, theta: float) -> None:
        if not self.homed:
            raise ArmError("not homed")
        self.target = np.clip([phi, psi, theta], LO, HI)
        self.mode = "track"

    def stop(self) -> None:
        self.mode = "idle"
        self.jog_rates = np.zeros(3)
        self.link.command("STOP")

    # ------------------------------------------------------------- control

    def tick(self, dt: float = TICK_S) -> None:
        vel = plan_velocity(self.mode, self.cmd, self.vel, self.target,
                            self.jog_rates, dt)
        # software joint limits: never command velocity into a wall
        at_lo, at_hi = self.cmd <= LO + 1e-9, self.cmd >= HI - 1e-9
        vel[at_lo & (vel < 0)] = 0.0
        vel[at_hi & (vel > 0)] = 0.0
        self.vel = vel
        self.cmd = np.clip(self.cmd + vel * dt, LO, HI)

        spd = self.cfg.steps_per_deg
        sgn = [-1.0 if inv else 1.0 for inv in self.cfg.invert]
        reply = self.link.command(
            f"V {vel[0] * spd[0] * sgn[0]:.1f} "
            f"{vel[1] * spd[1] * sgn[1]:.1f} {vel[2]:.1f}")
        if not reply.startswith("OK"):
            raise LinkError(f"V rejected: {reply}")
        self._read_position()

        if self.mode == "move" and move_settled(self.cmd, self.vel, self.target):
            self.cmd = self.target.copy()
            self.vel[:] = 0.0
            self.mode = "idle"
            self.link.command("STOP")

    @property
    def step_error(self) -> np.ndarray:
        """Commanded minus counted, in degrees. Growing beyond ~2 deg on the
        bench means lost steps: lower the acceleration (gate B1.4)."""
        return self.cmd - self.pos

    # ------------------------------------------------------------- sensing

    def _read_position(self) -> None:
        reply = self.link.command("P")
        parts = reply.split()
        if len(parts) != 7 or parts[0] != "POS":
            raise LinkError(f"bad POS reply: {reply!r}")
        phi_steps, psi_steps, theta_cdeg, es_phi, es_psi, contact = \
            (int(p) for p in parts[1:])
        spd, inv = self.cfg.steps_per_deg, self.cfg.invert
        self.pos = np.array([
            LO[0] + (-phi_steps if inv[0] else phi_steps) / spd[0],
            LO[1] + (-psi_steps if inv[1] else psi_steps) / spd[1],
            theta_cdeg / 100.0,
        ])
        self.endstops = (bool(es_phi), bool(es_psi))
        self.contact = bool(contact)

    def encoders(self) -> np.ndarray:
        return self.pos.copy()

    def status(self) -> dict:
        return {"mode": self.mode, "homed": self.homed,
                "phi": round(float(self.pos[0]), 2),
                "psi": round(float(self.pos[1]), 2),
                "theta": round(float(self.pos[2]), 2),
                "endstops": list(self.endstops)}


# ------------------------------------------------------------------ links

def list_serial_ports() -> list[tuple[str, str]]:
    """Every serial device the OS can see, as (device, description)."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [(p.device, p.description or "") for p in list_ports.comports()]


# macOS calls the Arduino /dev/cu.usbmodem*, Linux /dev/ttyACM*. Nothing in
# the code should depend on which laptop is plugged in.
_BOARD_HINTS = ("usbmodem", "usbserial", "wchusbserial", "ttyACM", "ttyUSB")


def find_serial_port() -> str:
    """Auto-detect the board. Raises with the actual port list on failure,
    because 'port not found' with no list sends people hunting the wrong
    thing — usually a charge-only cable or a board that never enumerated."""
    ports = list_serial_ports()
    hits = [d for d, _ in ports if any(h in d for h in _BOARD_HINTS)]
    if len(hits) == 1:
        return hits[0]
    if len(hits) > 1:
        raise LinkError("several boards are plugged in; pass one with "
                        f"--servo-port: {', '.join(hits)}")
    seen = ", ".join(d for d, _ in ports) or "none at all"
    raise LinkError(
        "no Arduino-like serial port found. Ports present: " + seen + ".\n"
        "  - a charge-only USB cable enumerates nothing; try a data cable\n"
        "  - check the board appears in Arduino IDE's Tools > Port\n"
        "  - /dev/ttyACM0 is a Linux name; macOS uses /dev/cu.usbmodem*")


class SerialLink:
    """pyserial transport. Imported lazily so the rest of the package never
    needs pyserial installed."""

    def __init__(self, port: str, baud: int):
        try:
            import serial  # pip install pyserial
        except ImportError as e:      # noqa: F841
            raise LinkError("pyserial is not installed — run: "
                            "uv pip install pyserial") from None
        if port in ("", "auto"):
            port = find_serial_port()
        self.port = port
        try:
            self.ser = serial.Serial(port, baud, timeout=2.0)
        except serial.SerialException as e:
            seen = ", ".join(d for d, _ in list_serial_ports()) or "none at all"
            raise LinkError(f"could not open {port}: {e}\n"
                            f"  ports present: {seen}\n"
                            "  pass --servo-port with no value to auto-detect"
                            ) from None
        import time
        time.sleep(2.0)               # Uno resets on port open
        self.ser.reset_input_buffer()

    def command(self, line: str, timeout: float = 2.0) -> str:
        self.ser.timeout = timeout
        self.ser.write((line + "\n").encode())
        reply = self.ser.readline().decode().strip()
        if not reply:
            raise LinkError(f"no reply to {line!r} — check port and firmware")
        return reply


class FakeArduino:
    """The firmware's serial contract in Python: integrates commanded
    velocities into step counters. Lets every gate run with no board."""

    def __init__(self):
        self.steps = [0.0, 0.0]       # phi, psi
        self.theta_cdeg = 0.0
        self.sps = [0.0, 0.0]
        self.theta_dps = 0.0
        self.contact = False

    def advance(self, dt: float) -> None:
        self.steps[0] += self.sps[0] * dt
        self.steps[1] += self.sps[1] * dt
        self.theta_cdeg = min(6000.0, max(0.0,
                              self.theta_cdeg + self.theta_dps * 100.0 * dt))

    def command(self, line: str, timeout: float = 2.0) -> str:
        if line == "HELLO":
            return "OK robot-barber-arm v1"
        if line.startswith("V "):
            a, b, c = (float(x) for x in line[2:].split())
            self.advance(TICK_S)      # one tick elapses per V, like the bench
            self.sps = [a, b]
            self.theta_dps = c
            return "OK"
        if line == "P":
            return (f"POS {round(self.steps[0])} {round(self.steps[1])} "
                    f"{round(self.theta_cdeg)} 0 0 {int(self.contact)}")
        if line == "Z" or line == "HOME":
            self.steps = [0.0, 0.0]
            self.theta_cdeg = 0.0
            self.sps = [0.0, 0.0]
            self.theta_dps = 0.0
            return "OK"
        if line == "STOP":
            self.sps = [0.0, 0.0]
            self.theta_dps = 0.0
            return "OK"
        return "ERR unknown"

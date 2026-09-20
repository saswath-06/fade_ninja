"""Phone teach pendant: iPhone tilt -> JOG rates.

The phone is a CONTROLLER, not a sensor (README section 3). It streams its
orientation over UDP; this module turns that into rate commands for the arm.
The phone's numbers are then discarded — what gets recorded is where the ARM
actually went, read from its encoders at 50 Hz.

    pitch  (nose up/down)  -> theta rate   clipper tilt = hair length
    roll   (wrist twist)   -> psi rate     around the head
    thumb slider           -> phi rate     height up the head

Rate control with a deadzone, so IMU drift is forgiving: the barber watches
the real clipper and corrects by eye.

Wire format, one UDP datagram per sample, plain ASCII:

    PEND <seq> <t_ms> <pitch_deg> <roll_deg> <slider> <cut> <rec>
"""
from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass

import numpy as np

DEFAULT_PORT = 8470
WATCHDOG_S = 0.25          # no packet for this long -> all rates zero


@dataclass
class PendantPacket:
    seq: int
    t_ms: int
    pitch: float           # deg, + = nose up
    roll: float            # deg, + = right
    slider: float          # -1..1, thumb control
    cut: bool              # clipper motor / dead-man
    rec: bool              # recording requested

    @classmethod
    def parse(cls, line: str) -> "PendantPacket":
        parts = line.strip().split()
        if len(parts) != 7 or parts[0] != "PEND":
            raise ValueError(f"bad pendant packet: {line!r}")
        return cls(int(parts[1]), int(parts[2]), float(parts[3]),
                   float(parts[4]), float(parts[5]),
                   parts[6][0] == "1", parts[6][1] == "1")

    def to_line(self) -> str:
        return (f"PEND {self.seq} {self.t_ms} {self.pitch:.2f} {self.roll:.2f} "
                f"{self.slider:.3f} {int(self.cut)}{int(self.rec)}")


@dataclass
class RateMap:
    """Tilt-to-rate mapping. Tune here, not in the app — no rebuild needed."""
    deadzone_deg: float = 5.0
    full_deg: float = 40.0        # tilt this far for full speed
    phi_rate: float = 22.0        # deg/s at full deflection
    psi_rate: float = 30.0
    theta_rate: float = 45.0
    expo: float = 1.6             # >1 softens around centre for fine control

    def _axis(self, value_deg: float) -> float:
        mag = abs(value_deg)
        if mag <= self.deadzone_deg:
            return 0.0
        span = max(self.full_deg - self.deadzone_deg, 1e-6)
        frac = min((mag - self.deadzone_deg) / span, 1.0)
        return np.sign(value_deg) * frac ** self.expo

    def rates(self, pkt: PendantPacket) -> tuple[float, float, float]:
        """-> (dphi, dpsi, dtheta) in deg/s."""
        slider = float(np.clip(pkt.slider, -1.0, 1.0))
        if abs(slider) < 0.08:
            slider = 0.0
        return (slider * self.phi_rate,
                self._axis(pkt.roll) * self.psi_rate,
                self._axis(pkt.pitch) * self.theta_rate)


class PendantReceiver:
    """Listens for pendant datagrams in a background thread.

    Only the newest packet matters — this is a live control stream, so a
    dropped datagram is better ignored than replayed late.
    """

    def __init__(self, port: int = DEFAULT_PORT, host: str = "0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(0.2)
        self.port = port
        self._latest: PendantPacket | None = None
        self._stamp = 0.0
        self._lock = threading.Lock()
        self._running = True
        self.received = 0
        self.dropped = 0
        self._last_seq = -1
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            try:
                data, _ = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                pkt = PendantPacket.parse(data.decode())
            except (ValueError, UnicodeDecodeError):
                continue
            with self._lock:
                if self._last_seq >= 0 and pkt.seq > self._last_seq + 1:
                    self.dropped += pkt.seq - self._last_seq - 1
                self._last_seq = pkt.seq
                self._latest = pkt
                self._stamp = time.monotonic()
                self.received += 1

    def latest(self) -> PendantPacket | None:
        """Newest packet, or None if the link has gone quiet (dead-man)."""
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
        self.sock.close()


def local_ip() -> str:
    """Best-guess LAN address to type into the phone."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))   # no packets sent; picks the route
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()

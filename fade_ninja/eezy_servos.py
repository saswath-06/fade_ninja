"""Pi PWM servo stub for EEZYbotARM (PHONE_POSE_TDD Phase 7).

Dry-run prints pulse widths. Live GPIO is intentionally not enabled until
link lengths and servo class are measured.
"""
from __future__ import annotations


def angle_to_pulse_us(deg: float, lo: float = -90.0, hi: float = 90.0,
                      us_lo: int = 500, us_hi: int = 2500) -> int:
    t = (deg - lo) / (hi - lo)
    t = max(0.0, min(1.0, t))
    return int(us_lo + t * (us_hi - us_lo))


class ServoDriver:
    def __init__(self, dry_run: bool = True):
        self.dry_run = dry_run
        self.frozen = True
        self.last: tuple[float, float, float, float] | None = None
        self.pulses: tuple[int, int, int, int] | None = None

    def freeze(self) -> None:
        self.frozen = True
        if self.dry_run:
            print("SERVO freeze")

    def set_joints(self, q1: float, q2: float, q3: float, q4: float) -> None:
        self.frozen = False
        self.last = (q1, q2, q3, q4)
        self.pulses = (
            angle_to_pulse_us(q1),
            angle_to_pulse_us(q2, 0, 90),
            angle_to_pulse_us(q3, -135, 0),
            angle_to_pulse_us(q4, -45, 45),
        )
        if self.dry_run:
            print(f"SERVO us={self.pulses} deg=({q1:.1f},{q2:.1f},{q3:.1f},{q4:.1f})")

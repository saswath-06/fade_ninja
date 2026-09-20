"""The whole bench in one object: arm + head + recorder + clipper switch.

One tick = one 20 ms firmware loop: step the arm, read encoders, check
contact, cut if the clipper is on, feed the recorder.
"""
from __future__ import annotations

import numpy as np

from .arm import VirtualArm
from .calibration import Calibration, default_calibration
from .cutting import apply_cut
from .head import Head
from .interfaces import TICK_S
from .recorder import Recorder


class Rig:
    def __init__(self, head: Head | None = None,
                 cal: Calibration | None = None, seed: int = 0,
                 physical_cal: Calibration | None = None):
        self.arm = VirtualArm(seed=seed)
        self.head = head
        self.cal = cal or default_calibration()
        # what the blade PHYSICALLY leaves. Differs from self.cal (what the
        # software believes) to model a miscalibrated mount — fault injection
        # for the vision critic.
        self.physical_cal = physical_cal or self.cal
        self.recorder = Recorder()
        self.clipper_on = False
        self.contact = False
        self.encoders = self.arm.encoders()

    def tick(self) -> None:
        self.arm.tick(TICK_S)
        self.encoders = self.arm.encoders()
        phi, psi, theta = (float(v) for v in self.encoders)
        if self.head is not None:
            self.contact = apply_cut(self.head, phi, psi, theta,
                                     self.physical_cal, self.clipper_on)
        else:
            # hardware bench: the spring-slide switch, reported by the arm
            self.contact = bool(getattr(self.arm, "contact", False))
        self.recorder.tick(self.encoders, self.contact)

    def run(self, seconds: float) -> None:
        for _ in range(int(round(seconds / TICK_S))):
            self.tick()

    def settle(self, timeout_s: float = 20.0) -> bool:
        """Tick until the current MOVE finishes. True if it settled."""
        for _ in range(int(timeout_s / TICK_S)):
            self.tick()
            if self.arm.mode == "idle":
                return True
        return False

    def goto(self, phi: float, psi: float, theta: float) -> None:
        self.arm.move_to(phi, psi, theta)
        if not self.settle():
            raise RuntimeError("MOVE did not settle")

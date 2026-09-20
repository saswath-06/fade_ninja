"""REC START/STOP: 50 Hz logging from the arm's encoders.

Glove/teleop values are never saved — only where the arm actually went.
"""
from __future__ import annotations

from .interfaces import TICK_MS, LogSample, TeachLog


class Recorder:
    def __init__(self):
        self.recording = False
        self._samples: list[LogSample] = []
        self._t_ms = 0

    def start(self) -> None:
        self._samples = []
        self._t_ms = 0
        self.recording = True

    def tick(self, encoders, contact: bool) -> None:
        if not self.recording:
            return
        phi, psi, theta = (float(v) for v in encoders)
        self._samples.append(LogSample(self._t_ms, phi, psi, theta, bool(contact)))
        self._t_ms += TICK_MS

    def stop(self) -> TeachLog:
        self.recording = False
        return TeachLog(self._samples)

"""Virtual 3-axis arm: rate/accel-limited dynamics, joint limits, quantized encoders.

Stands in for the ESP32 + steppers + tilt servo. All precision in the real
system comes from the arm's encoders, so the sim models exactly that: what the
encoders would read at each 20 ms tick.
"""
from __future__ import annotations

import numpy as np

from .interfaces import AXES, JOINT_LIMITS, TICK_S


class ArmError(RuntimeError):
    pass


LO = np.array([JOINT_LIMITS[a].lo for a in AXES])
HI = np.array([JOINT_LIMITS[a].hi for a in AXES])
VMAX = np.array([JOINT_LIMITS[a].max_rate for a in AXES])
AMAX = np.array([JOINT_LIMITS[a].max_accel for a in AXES])

ENC_RES_DEG = 0.1     # encoder quantization
ENC_NOISE_DEG = 0.02  # electrical noise, 1 sigma


def plan_velocity(mode: str, pos: np.ndarray, vel: np.ndarray,
                  target: np.ndarray, jog_rates: np.ndarray,
                  dt: float) -> np.ndarray:
    """The velocity command for one 20 ms tick. Shared by the simulated arm
    and the hardware driver, so both obey the same tested motion profiles."""
    if mode == "move":
        err = target - pos
        # sqrt profile: decelerate so we arrive with zero velocity;
        # the |err|/dt cap prevents overshoot on the final tick
        v_des = np.sign(err) * np.minimum.reduce(
            [VMAX, np.sqrt(2 * AMAX * np.abs(err)), np.abs(err) / dt])
    elif mode == "jog":
        v_des = jog_rates
    elif mode == "track":
        v_des = np.clip((target - pos) / dt, -VMAX, VMAX)
    else:
        v_des = np.zeros(3)
    # accel limit relative to the current velocity
    return np.clip(v_des, vel - AMAX * dt, vel + AMAX * dt)


def move_settled(pos: np.ndarray, vel: np.ndarray, target: np.ndarray) -> bool:
    return bool(np.all(np.abs(target - pos) < 0.02)
                and np.all(np.abs(vel) < 0.5))


class VirtualArm:
    """Modes: idle | move (go to target, then idle) | jog (hold rates) |
    track (follow a stream of targets, one per tick — used by replay)."""

    def __init__(self, seed: int = 0):
        self.pos = LO.copy()
        self.vel = np.zeros(3)
        self.mode = "idle"
        self.target = self.pos.copy()
        self.jog_rates = np.zeros(3)
        self.homed = False
        self._rng = np.random.default_rng(seed)

    # ------------------------------------------------------------- commands

    def home(self) -> None:
        """Drive to the endstops, back off, zero. Instant in sim."""
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
        """Follow one replay sample. Caller validates against limits first."""
        if not self.homed:
            raise ArmError("not homed")
        self.target = np.clip([phi, psi, theta], LO, HI)
        self.mode = "track"

    def stop(self) -> None:
        self.mode = "idle"
        self.jog_rates = np.zeros(3)

    # ------------------------------------------------------------- dynamics

    def tick(self, dt: float = TICK_S) -> None:
        self.vel = plan_velocity(self.mode, self.pos, self.vel, self.target,
                                 self.jog_rates, dt)
        self.pos = self.pos + self.vel * dt

        # hard joint limits: stop dead at the wall
        below, above = self.pos < LO, self.pos > HI
        self.pos = np.clip(self.pos, LO, HI)
        self.vel[below | above] = 0.0

        if self.mode == "move" and move_settled(self.pos, self.vel, self.target):
            self.pos = self.target.copy()
            self.vel[:] = 0.0
            self.mode = "idle"

    # ------------------------------------------------------------- sensing

    def encoders(self) -> np.ndarray:
        """What the firmware would log: quantized, slightly noisy positions."""
        noisy = self.pos + self._rng.normal(0.0, ENC_NOISE_DEG, 3)
        return np.round(noisy / ENC_RES_DEG) * ENC_RES_DEG

    def status(self) -> dict:
        enc = self.encoders()
        return {"mode": self.mode, "homed": self.homed,
                "phi": round(float(enc[0]), 2), "psi": round(float(enc[1]), 2),
                "theta": round(float(enc[2]), 2)}

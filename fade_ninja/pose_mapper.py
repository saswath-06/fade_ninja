"""Map ARKit-relative pose → EEZY joints (PHONE_POSE_TDD Phase 3).

Frames (locked):
    arm_x_mm = scale * phone_z_m * (-1000)   # ARKit −Z forward → arm +X
    arm_y_mm = scale * phone_x_m * (-1000)   # ARKit +X right  → arm −Y
    arm_z_mm = scale * phone_y_m *  1000     # ARKit +Y up     → arm +Z

Default scale 0.3. Only pitch → q4; roll/yaw discarded.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .eezy_ik import (
    P_HOME,
    Q_HOME,
    forward_kinematics,
    inverse_kinematics,
    inverse_kinematics_clamped,
    pitch_to_q4,
)
from .pose_protocol import RelativePose, pitch_deg_from_quat

DEFAULT_SCALE = 0.3
# q1..q4. 60 deg/s left the arm ~0.9 s behind the hand, which reads as bad
# tracking; 180 brings it to ~0.3 s and still bounds servo speed. Lower it
# for real hardware once the servos' real limits are measured.
MAX_SLEW_DEG_S = (180.0, 180.0, 240.0, 300.0)


@dataclass
class MapperState:
    q1: float
    q2: float
    q3: float
    q4: float
    reachable: bool
    tip_x: float
    tip_y: float
    tip_z: float


class PoseMapper:
    def __init__(self, scale: float = DEFAULT_SCALE,
                 max_slew: tuple[float, float, float, float] = MAX_SLEW_DEG_S):
        self.scale = scale
        self.max_slew = max_slew
        self._q = list(Q_HOME) + [0.0]  # q1..q4
        self._reachable = True
        self._last_t: float | None = None
        # Tip at home
        hx, hy, hz = P_HOME
        self._home_tip = (hx, hy, hz)

    def reset(self) -> MapperState:
        self._q = list(Q_HOME) + [0.0]
        self._reachable = True
        self._last_t = None
        return self.state()

    def state(self) -> MapperState:
        x, y, z = forward_kinematics(self._q[0], self._q[1], self._q[2])
        return MapperState(
            self._q[0], self._q[1], self._q[2], self._q[3],
            self._reachable, x, y, z,
        )

    def phone_to_arm_mm(self, phone_x: float, phone_y: float,
                        phone_z: float) -> tuple[float, float, float]:
        s = self.scale * 1000.0
        arm_x = s * phone_z * (-1.0)
        arm_y = s * phone_x * (-1.0)
        arm_z = s * phone_y
        hx, hy, hz = self._home_tip
        return (hx + arm_x, hy + arm_y, hz + arm_z)

    def update(self, rel: RelativePose, *, now: float | None = None) -> MapperState:
        now = time.monotonic() if now is None else now
        dt = 0.02 if self._last_t is None else max(now - self._last_t, 1e-3)
        self._last_t = now

        target = self.phone_to_arm_mm(rel.x, rel.y, rel.z)
        joints, out_of_reach = inverse_kinematics_clamped(*target)
        self._reachable = not out_of_reach

        pitch = pitch_deg_from_quat(rel.qw, rel.qx, rel.qy, rel.qz)
        q4_tgt = pitch_to_q4(pitch)

        self._slew_together([joints.q1, joints.q2, joints.q3, q4_tgt], dt)
        return self.state()

    def _slew_together(self, desired: list[float], dt: float) -> None:
        """Rate-limit every joint by the SAME fraction so they arrive
        together.

        Limiting each joint independently lets the fast ones finish early,
        which bends the tip's path: commanding a straight move in one axis
        visibly wanders in the others until the slowest joint catches up.
        Scaling the whole step keeps each joint under its own limit while
        holding the shape of the motion.
        """
        need = 0.0
        for i in range(4):
            if self.max_slew[i] > 0:
                need = max(need, abs(desired[i] - self._q[i]) / self.max_slew[i])
        f = 1.0 if need <= dt or need <= 0.0 else dt / need
        for i in range(4):
            self._q[i] += (desired[i] - self._q[i]) * f

    @staticmethod
    def _slew(current: float, target: float, max_deg_s: float, dt: float) -> float:
        step = max_deg_s * dt
        delta = target - current
        if abs(delta) <= step:
            return target
        return current + math.copysign(step, delta)

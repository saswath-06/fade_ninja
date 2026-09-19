"""The Phase B gates re-run through the hardware driver against an emulated
Arduino (FakeArduino implements the firmware's serial contract). When the real
board is plugged in, hwcheck runs the same checks over actual serial."""
import numpy as np
import pytest

from fadegpt.arm import ArmError
from fadegpt.hardware_arm import ArmConfig, FakeArduino, HardwareArm
from fadegpt.interfaces import TICK_S
from fadegpt.replay import replay
from fadegpt.rig import Rig
from fadegpt.teach import ramp_log
from fadegpt.calibration import default_calibration


def bench_arm() -> HardwareArm:
    arm = HardwareArm(FakeArduino(), ArmConfig())
    arm.home()
    return arm


def run(arm, seconds):
    for _ in range(int(seconds / TICK_S)):
        arm.tick()


def test_hello_handshake_rejected_for_wrong_device():
    class NotOurBoard(FakeArduino):
        def command(self, line, timeout=2.0):
            return "OK some-other-firmware" if line == "HELLO" \
                else super().command(line, timeout)
    with pytest.raises(Exception, match="HELLO"):
        HardwareArm(NotOurBoard(), ArmConfig())


def test_b13_move_settles_within_2_degrees():
    arm = bench_arm()
    arm.move_to(30, 90, 15)
    for _ in range(2000):
        arm.tick()
        if arm.mode == "idle":
            break
    assert arm.mode == "idle", "MOVE never settled"
    assert np.all(np.abs(arm.pos - [30, 90, 15]) < 2.0)
    assert np.all(np.abs(arm.pos - [30, 90, 15]) < 0.3)  # ~2 steps


def test_b15_jog_rate_on_step_counters():
    arm = bench_arm()
    arm.jog(5, 0, 2)
    run(arm, 0.5)
    p0 = arm.pos.copy()
    run(arm, 2.0)
    rate = (arm.pos - p0) / 2.0
    assert rate[0] == pytest.approx(5.0, abs=0.2)
    assert rate[2] == pytest.approx(2.0, abs=0.2)
    arm.jog(0, 0, 0)
    run(arm, 0.5)
    p1 = arm.pos.copy()
    run(arm, 0.5)
    assert np.all(np.abs(arm.pos - p1) < 0.05)


def test_joint_limits_hold_on_hardware_driver():
    arm = bench_arm()
    arm.jog(-40, -60, -120)  # drive at the low endstops
    run(arm, 2.0)
    assert np.all(arm.pos >= -0.1)
    with pytest.raises(ArmError):
        arm.move_to(10, 200, 10)


def test_unhomed_refuses_motion():
    arm = HardwareArm(FakeArduino(), ArmConfig())
    with pytest.raises(ArmError):
        arm.jog(1, 0, 0)


def test_replay_pipeline_runs_on_hardware_driver():
    """B1.7 end to end: the SAME replay() call that drives the sim drives the
    hardware driver, refusal and all."""
    rig = Rig(head=None)
    rig.arm = bench_arm()
    log = ramp_log(default_calibration(), duration_s=2.0)
    rerec = replay(log, rig, record=True)
    assert len(rerec) == len(log)
    a, b = log.arrays(), rerec.arrays()
    for ax in ("phi", "psi", "theta"):
        rms = float(np.sqrt(np.mean((a[ax] - b[ax]) ** 2)))
        assert rms < 1.0, f"{ax} RMS {rms:.3f} deg over the 1 degree gate"

    bad = ramp_log(default_calibration(), duration_s=2.0)
    bad.samples[50].theta = 85.0
    from fadegpt.replay import ReplayRefused
    with pytest.raises(ReplayRefused):
        replay(bad, rig, smooth_first=False)

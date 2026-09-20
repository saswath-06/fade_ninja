"""Phase B1.1-B1.5: motion, homing, MOVE accuracy, repeatability, JOG rates."""
import numpy as np
import pytest

from fade_ninja.arm import ArmError, VirtualArm
from fade_ninja.interfaces import TICK_S
from fade_ninja.rig import Rig


def test_b11_axes_move_independently():
    rig = Rig()
    rig.arm.home()
    for axis, target in [(0, (20, 0, 0)), (1, (20, 45, 0)), (2, (20, 45, 30))]:
        before = rig.arm.pos.copy()
        rig.goto(*target)
        moved = np.abs(rig.arm.pos - before) > 0.01
        assert moved[axis], f"axis {axis} did not move"


def test_b12_homing_repeats():
    marks = []
    for _ in range(5):
        rig = Rig()
        rig.arm.home()
        rig.goto(33, 120, 40)  # power cycle = fresh rig; drive somewhere
        rig.arm.home()
        marks.append(rig.arm.encoders())
    marks = np.array(marks)
    assert np.all(np.abs(marks - marks[0]) <= 0.1 + 1e-9)


def test_b13_move_within_2_degrees():
    rig = Rig()
    rig.arm.home()
    rig.goto(30, 90, 15)
    enc = rig.arm.encoders()
    assert np.all(np.abs(enc - np.array([30, 90, 15])) < 2.0)  # the gate
    assert np.all(np.abs(enc - np.array([30, 90, 15])) < 0.2)  # sim is better


def test_b14_repeatability_10x():
    rig = Rig()
    rig.arm.home()
    readings = []
    for _ in range(10):
        rig.goto(60, 150, 50)   # away
        rig.goto(30, 90, 15)    # and back
        readings.append(rig.arm.encoders())
    readings = np.array(readings)
    spread = readings.max(axis=0) - readings.min(axis=0)
    assert np.all(spread <= 0.2), f"drift {spread} means lost steps"


def test_b15_jog_steady_rate_and_stop():
    rig = Rig()
    rig.arm.home()
    rig.goto(10, 90, 10)
    rig.arm.jog(5, 0, 0)
    rig.run(0.5)  # let the rate settle past the accel ramp
    p0 = rig.arm.pos.copy()
    rig.run(2.0)
    rate = (rig.arm.pos - p0) / 2.0
    assert rate[0] == pytest.approx(5.0, abs=0.2)
    assert abs(rate[1]) < 0.05 and abs(rate[2]) < 0.05

    rig.arm.jog(0, 0, 0)
    rig.run(0.5)
    p1 = rig.arm.pos.copy()
    rig.run(1.0)
    assert np.all(np.abs(rig.arm.pos - p1) < 0.01), "JOG 0 0 0 must stop"


def test_jog_rates_clamped_to_limits():
    arm = VirtualArm()
    arm.home()
    arm.jog(1000, -1000, 1000)
    assert np.all(np.abs(arm.jog_rates) <= [40, 60, 120])


def test_move_refused_outside_limits_or_unhomed():
    arm = VirtualArm()
    with pytest.raises(ArmError):
        arm.move_to(10, 10, 10)  # not homed
    arm.home()
    with pytest.raises(ArmError):
        arm.move_to(10, 200, 10)  # psi beyond 180


def test_axes_never_leave_limits_under_jog():
    rig = Rig()
    rig.arm.home()
    rig.arm.jog(-40, -60, -120)  # drive into the low endstops
    rig.run(3.0)
    assert np.all(rig.arm.pos >= [0, 0, 0])
    assert np.all(rig.arm.vel == 0)

"""Joint angles -> servos.

The properties that matter when a motor is on the other end: a command
outside the linkage's range is clamped rather than sent, the arm eases toward
a far target instead of slamming to it, and freezing keeps the servos holding
rather than dropping the arm.
"""
import pytest

from fade_ninja.eezy_ik import Q1_LIMITS, Q2_LIMITS, Q3_LIMITS, Q4_LIMITS
from fade_ninja.servo_link import (SAFE_POSE, FakeServoBoard, ServoCal,
                                   ServoConfig, ServoDriver, ServoLinkError,
                                   clamp_joints, step_toward)


def driver(max_step=4.0, cals=None):
    board = FakeServoBoard(max_step_deg=max_step)
    cfg = ServoConfig(max_step_deg=max_step)
    if cals:
        cfg.cals = cals
    return ServoDriver(board, cfg), board


# ----------------------------------------------------------- calibration

def test_default_calibration_spans_the_joint_range():
    c = ServoCal()
    assert c.pulse_for(Q2_LIMITS[0], *Q2_LIMITS) == 500
    assert c.pulse_for(Q2_LIMITS[1], *Q2_LIMITS) == 2500


def test_invert_flips_the_direction():
    c = ServoCal(invert=True)
    assert c.pulse_for(Q2_LIMITS[0], *Q2_LIMITS) == 2500
    assert c.pulse_for(Q2_LIMITS[1], *Q2_LIMITS) == 500


def test_offset_shifts_the_zero():
    plain, shifted = ServoCal(), ServoCal(offset_deg=10.0)
    assert shifted.pulse_for(30, *Q2_LIMITS) > plain.pulse_for(30, *Q2_LIMITS)


def test_narrow_travel_servo_is_respected():
    """A servo that only sweeps 1000-2000us must never be driven past it."""
    c = ServoCal(min_us=1000, max_us=2000)
    for deg in (Q3_LIMITS[0], -60, Q3_LIMITS[1]):
        assert 1000 <= c.pulse_for(deg, *Q3_LIMITS) <= 2000


def test_pulses_stay_inside_travel_even_for_out_of_range_angles():
    c = ServoCal()
    assert c.pulse_for(-999, *Q1_LIMITS) == 500
    assert c.pulse_for(999, *Q1_LIMITS) == 2500


# ---------------------------------------------------------------- limits

def test_out_of_range_commands_are_clamped_not_sent():
    q, hit = clamp_joints((999.0, -50.0, 40.0, 900.0))
    assert hit
    assert q[0] == Q1_LIMITS[1]
    assert q[1] == Q2_LIMITS[0]
    assert q[2] == Q3_LIMITS[1]
    assert q[3] == Q4_LIMITS[1]


def test_in_range_commands_pass_through_untouched():
    q, hit = clamp_joints((10.0, 45.0, -90.0, 5.0))
    assert not hit and q == (10.0, 45.0, -90.0, 5.0)


def test_driver_reports_when_it_clamped():
    d, _ = driver()
    d.home()
    d.set_joints(500, 45, -90, 0)
    assert d.clamped is True
    d.set_joints(10, 45, -90, 0)
    assert d.clamped is False


def test_the_board_clamps_too():
    """Defence in depth: a laptop sending nonsense must not reach the horn."""
    board = FakeServoBoard()
    board.command("J 999 999 999 999")
    assert board.clamped_count == 1
    assert board.position[1] <= Q2_LIMITS[1]
    assert board.position[2] <= Q3_LIMITS[1]


# ------------------------------------------------------------ easing

def test_a_far_target_is_approached_not_jumped_to():
    d, _ = driver(max_step=4.0)
    d.home()
    start = d.position
    d.set_joints(90, 90, 0, 45)
    moved = max(abs(a - b) for a, b in zip(d.position, start))
    assert moved <= 4.0 + 1e-9, "the arm slammed instead of easing"


def test_easing_still_arrives():
    d, _ = driver(max_step=4.0)
    d.home()
    for _ in range(200):
        d.set_joints(60, 80, -20, 30)
    assert d.position == pytest.approx((60, 80, -20, 30), abs=0.01)


def test_step_toward_never_overshoots():
    got = step_toward((0, 0, 0, 0), (1.0, -1.0, 0.5, 0.0), 4.0)
    assert got == pytest.approx((1.0, -1.0, 0.5, 0.0))


def test_first_command_after_connect_cannot_be_a_jump():
    """Servos snap to their first pulse; the arm may be resting anywhere."""
    d, board = driver(max_step=4.0)
    d.set_joints(-90, 90, -135, 45)
    assert max(abs(a - b) for a, b in zip(d.position, SAFE_POSE)) <= 4.0


# ------------------------------------------------------------ protocol

def test_handshake_rejects_the_wrong_firmware():
    class Other(FakeServoBoard):
        def command(self, line, timeout=2.0):
            return "OK some-other-board" if line == "HELLO" \
                else super().command(line, timeout)
    with pytest.raises(ServoLinkError, match="HELLO"):
        ServoDriver(Other(), ServoConfig())


def test_home_returns_to_the_safe_pose():
    d, board = driver()
    d.set_joints(40, 70, -30, 20)
    d.home()
    assert d.position == pytest.approx(SAFE_POSE)
    assert tuple(board.position) == pytest.approx(SAFE_POSE)


def test_position_can_be_read_back():
    d, _ = driver()
    d.home()
    for _ in range(50):
        d.set_joints(20, 60, -70, 10)
    assert d.read_position() == pytest.approx(d.position, abs=0.02)


def test_board_refuses_malformed_commands():
    board = FakeServoBoard()
    assert board.command("J 1 2 3").startswith("ERR")
    assert board.command("J a b c d").startswith("ERR")
    assert board.command("NOPE").startswith("ERR")


# ------------------------------------------------------------- safety

def test_freeze_keeps_the_servos_holding():
    """Freezing must not detach: a limp arm falls under its own weight."""
    d, board = driver()
    d.home()
    d.set_joints(20, 60, -70, 10)
    d.freeze()
    assert d.frozen is True
    assert board.attached is True, "freeze detached the servos"


def test_relax_is_the_explicit_way_to_let_go():
    d, board = driver()
    d.home()
    d.relax()
    assert board.attached is False
    assert d.frozen is True


def test_dry_run_needs_no_board():
    d = ServoDriver(dry_run=True)
    d.set_joints(0, 45, -90, 0)
    assert d.pulses is not None and len(d.pulses) == 4
    d.freeze()
    assert d.frozen

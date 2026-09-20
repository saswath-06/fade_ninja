"""The live path, end to end: teach with servos attached, save, replay.

Everything else is tested in pieces. This checks the one thing that only
shows up assembled — that a saved cut, replayed later, actually reaches the
servos rather than only moving the simulated arm.
"""
import math
import time

import pytest

from fade_ninja.servo_link import FakeServoBoard, ServoConfig, ServoDriver
from fade_ninja.store import Store
from fade_ninja.take import TakePlayer
from pose_server import PoseServer


def pose_line(i, x, y, z):
    return f"POSE {i * 20} {x:.5f} {y:.5f} {z:.5f} 1 0 0 0 0"


@pytest.fixture
def rig(tmp_path):
    board = FakeServoBoard()
    hw = ServoDriver(board, ServoConfig())
    hw.home()
    srv = PoseServer(port=0, hardware=hw)
    store = Store(url="", path=tmp_path / "live.db")
    yield srv, board, store, hw
    store.close()


def drive(srv, seconds=1.0, hz=50):
    """Feed pose packets the way the phone would."""
    srv.handle_line("START 0")
    n = int(seconds * hz)
    for i in range(n):
        t = i / hz
        srv.handle_line(pose_line(i, 0.12 * math.sin(t * 2.0),
                                  0.08 * math.sin(t * 1.3), -0.05))
    return n


def test_teleop_reaches_the_servos(rig):
    srv, board, _, _ = rig
    start = tuple(board.position)
    drive(srv, seconds=0.6)
    assert tuple(board.position) != start, "driving never reached the servos"


def test_record_save_replay_moves_the_servos(rig):
    """The whole product, with hardware attached."""
    srv, board, store, _ = rig
    user = store.sign_in("bench")

    srv.begin_recording()
    drive(srv, seconds=1.2)
    rows = srv.end_recording()
    assert len(rows) > 40, f"only {len(rows)} samples recorded"

    take = store.save_take(user.id, "bench cut", rows)
    assert take.n_samples == len(rows)

    # park the arm somewhere else, then replay and watch the servos move
    srv.stop_playback()
    board.position = [0.0, 45.0, -90.0, 0.0]
    seen = set()
    srv.begin_playback(TakePlayer(store.samples(take.id)), take.name)
    deadline = time.monotonic() + 3.0
    while srv.player is not None and time.monotonic() < deadline:
        srv.playback_tick()
        seen.add(tuple(round(v, 1) for v in board.position))
        time.sleep(0.005)

    assert len(seen) > 5, f"servos held still during replay ({len(seen)} poses)"


def test_replay_runs_with_no_phone_attached(rig):
    """A saved cut must play with the phone unplugged — that is the demo."""
    srv, board, store, _ = rig
    user = store.sign_in("bench")
    srv.begin_recording()
    drive(srv, seconds=1.0)
    take = store.save_take(user.id, "solo", srv.end_recording())

    fresh_board = FakeServoBoard()
    solo = PoseServer(port=0, hardware=ServoDriver(fresh_board, ServoConfig()))
    solo.hardware.home()
    start = tuple(fresh_board.position)
    solo.begin_playback(TakePlayer(store.samples(take.id)), "solo")
    deadline = time.monotonic() + 3.0
    while solo.player is not None and time.monotonic() < deadline:
        solo.playback_tick()
        time.sleep(0.005)
    assert tuple(fresh_board.position) != start


def test_servo_commands_stay_inside_the_linkage_limits(rig):
    """However wild the phone gets, the horn must never be asked for a pose
    the arm cannot hold."""
    from fade_ninja.servo_link import JOINT_LIMITS
    srv, board, _, _ = rig
    srv.handle_line("START 0")
    for i in range(120):                       # deliberately extreme motion
        srv.handle_line(pose_line(i, 2.0 * math.sin(i / 3), 2.0, -2.0))
        for j, (lo, hi) in enumerate(JOINT_LIMITS):
            assert lo - 1e-6 <= board.position[j] <= hi + 1e-6, \
                f"q{j+1}={board.position[j]} escaped [{lo}, {hi}]"


def test_the_arm_eases_rather_than_snapping_on_the_first_command(rig):
    srv, board, _, _ = rig
    before = tuple(board.position)
    srv.handle_line("START 0")
    srv.handle_line(pose_line(0, 1.5, 1.5, -1.5))   # a far target, immediately
    moved = max(abs(a - b) for a, b in zip(board.position, before))
    assert moved <= 4.0 + 1e-6, f"servos jumped {moved:.1f} deg on first command"

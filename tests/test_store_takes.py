"""Per-user cut library, 4-axis recording, and replay safety."""
import math

import pytest

from fade_ninja.store import Store, StoreError, normalise_handle
from fade_ninja.take import (JointRecorder, ReplayRefused, TakePlayer, prepare,
                             rms_per_joint, validate)


@pytest.fixture
def store(tmp_path):
    s = Store(url="", path=tmp_path / "t.db")
    yield s
    s.close()


def ramp(n=200, jitter=0.0, seed=0):
    import random
    rng = random.Random(seed)
    return [(i * 20, i * 0.1, 45 + 10 * math.sin(i / 25) + rng.gauss(0, jitter),
             -90 + i * 0.05, 5 * math.sin(i / 15), True) for i in range(n)]


# ------------------------------------------------------------- handles

@pytest.mark.parametrize("raw,want", [("Saswath", "saswath"), ("  AB  ", "ab"),
                                      ("a_b.c-d", "a_b.c-d")])
def test_handles_are_normalised(raw, want):
    assert normalise_handle(raw) == want


@pytest.mark.parametrize("bad", ["", "a", "!!", "has space", "x" * 40, ".lead"])
def test_bad_handles_rejected(bad):
    with pytest.raises(StoreError):
        normalise_handle(bad)


def test_sign_in_is_idempotent(store):
    a = store.sign_in("Saswath")
    b = store.sign_in("saswath")
    assert a.id == b.id, "same handle must not create a second account"


# --------------------------------------------------------------- library

def test_takes_are_scoped_to_their_owner(store):
    me, you = store.sign_in("me"), store.sign_in("you")
    mine = store.save_take(me.id, "low fade", ramp())
    assert [t.id for t in store.takes_for(you.id)] == []
    assert [t.id for t in store.takes_for(me.id)] == [mine.id]


def test_delete_requires_the_owner(store):
    me, you = store.sign_in("me"), store.sign_in("you")
    t = store.save_take(me.id, "fade", ramp())
    assert store.delete_take(t.id, you.id) is False, "wrong user deleted a take"
    assert store.delete_take(t.id, me.id) is True
    assert store.take(t.id) is None


def test_samples_round_trip_in_order(store):
    u = store.sign_in("me")
    rows = ramp(120)
    t = store.save_take(u.id, "fade", rows)
    got = store.samples(t.id)
    assert len(got) == len(rows)
    assert [r[0] for r in got] == sorted(r[0] for r in got)
    assert got[0][1] == pytest.approx(rows[0][1])


def test_empty_takes_are_refused(store):
    u = store.sign_in("me")
    with pytest.raises(StoreError):
        store.save_take(u.id, "nothing", [])


def test_chart_summary_is_downsampled(store):
    u = store.sign_in("me")
    t = store.save_take(u.id, "long", ramp(2000))
    pts = store.take_summary(t.id, buckets=100)
    assert 50 <= len(pts) <= 220, len(pts)
    assert set(pts[0]) == {"t_ms", "q1", "q2", "q3", "q4"}


# -------------------------------------------------------------- recording

def test_recorder_stamps_real_elapsed_time():
    """Ticks arrive with pose packets, which do not land on a 20ms grid.
    Assuming they do compressed a 4s cut into 3.3s and replayed it fast."""
    import time
    r = JointRecorder()
    r.start()
    for _ in range(5):
        r.tick(0, 45, -90, 0)
        time.sleep(0.03)
    rows = r.stop()
    assert len(rows) == 5
    stamps = [x[0] for x in rows]
    assert stamps == sorted(set(stamps)), "timestamps must strictly rise"
    assert 100 <= stamps[-1] <= 260, f"should reflect ~120ms of real time, got {stamps[-1]}"


def test_recorder_ignores_ticks_before_start():
    r = JointRecorder()
    r.tick(0, 45, -90, 0)
    assert r.n == 0


# ---------------------------------------------------------------- replay

def test_smoothing_removes_tremor_but_keeps_the_path():
    clean, shaky = ramp(300), ramp(300, jitter=0.8)
    err = rms_per_joint(prepare(shaky), clean)
    raw = rms_per_joint(shaky, clean)
    assert err["q2"] < raw["q2"] * 0.6
    assert err["q2"] < 0.4


def test_player_walks_every_sample_once():
    rows = ramp(150)
    p = TakePlayer(rows)
    seen = []
    while not p.done:
        seen.append(p.next_pose())
    assert len(seen) == len(rows)
    assert p.next_pose() is None
    assert p.progress == pytest.approx(1.0)


def test_a_corrupt_sample_is_refused_not_smoothed_away():
    """Smoothing would quietly pull a wild value back into range and run it.
    Teleop already clamps, so an out-of-range raw sample means corruption."""
    rows = ramp(200)
    rows[100] = (rows[100][0], 0.0, 200.0, -90.0, 0.0, True)
    with pytest.raises(ReplayRefused, match="q2"):
        prepare(rows)


def test_empty_and_out_of_order_takes_are_refused():
    with pytest.raises(ReplayRefused, match="empty"):
        validate([])
    rows = ramp(50)
    rows[10] = (rows[9][0], *rows[10][1:])
    with pytest.raises(ReplayRefused, match="timestamps"):
        validate(rows)


def test_a_pose_held_on_a_joint_limit_still_replays():
    """Savgol undershoots a corner sitting exactly on a limit; that must not
    read as a violation."""
    rows = [(i * 20, 0.0, 90.0, -135.0, 45.0, True) for i in range(120)]
    out = prepare(rows)
    assert len(out) == 120
    assert out[60][2] == pytest.approx(90.0, abs=1e-6)


# --------------------------------------------------------------- tracing

def test_tracing_is_a_no_op_without_a_dsn():
    """The robot must not gain a hard dependency on an observability vendor."""
    from fade_ninja import telemetry_trace as trace
    assert trace.init(dsn="") is False
    assert trace.is_enabled() is False
    with trace.span("robot.ik", "solve") as s:
        assert s is None
    with trace.transaction("tick") as t:
        assert t is None
    trace.breadcrumb("link stale", level="warning")   # must not raise
    trace.measure("tracking_lag", 12.5)


def test_replay_follows_the_recorded_timeline():
    """A cut recorded over 2s must take about 2s to replay, whatever rate the
    playback loop happens to tick at."""
    rows = [(i * 40, 0.0, 45.0, -90.0, 0.0, True) for i in range(50)]  # 25 Hz, 2s
    p = TakePlayer(rows, smooth_first=False)
    assert p.pose_at(0) is not None
    assert p.pose_at(1000) is not None
    assert p.pose_at(1960) is not None
    assert p.pose_at(2200) is None, "should finish at the recorded duration"


def test_replay_holds_each_sample_until_its_time():
    rows = [(0, 1.0, 45.0, -90.0, 0.0, True), (500, 2.0, 45.0, -90.0, 0.0, True)]
    p = TakePlayer(rows, smooth_first=False)
    assert p.pose_at(0)[0] == pytest.approx(1.0)
    assert p.pose_at(250)[0] == pytest.approx(1.0), "second sample came early"
    assert p.pose_at(500)[0] == pytest.approx(2.0)

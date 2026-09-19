"""Phone teach pendant: wire format, rate mapping, dead-man watchdog, and the
whole point — a phone-driven take replays like any other teach log.

A fake phone sends real UDP datagrams, so the transport is exercised rather
than stubbed.
"""
import socket
import time

import numpy as np
import pytest

from fadegpt.head import Head
from fadegpt.interfaces import TICK_S, TeachLog
from fadegpt.pendant import (DEFAULT_PORT, PendantPacket, PendantReceiver,
                             RateMap)
from fadegpt.replay import replay, rms_per_axis, smooth
from fadegpt.rig import Rig


def pkt(seq=1, t_ms=0, pitch=0.0, roll=0.0, slider=0.0, cut=False, rec=False):
    return PendantPacket(seq, t_ms, pitch, roll, slider, cut, rec)


# ------------------------------------------------------------- wire format

def test_packet_round_trip():
    p = pkt(seq=42, t_ms=1240, pitch=-12.5, roll=30.25, slider=-0.75,
            cut=True, rec=True)
    again = PendantPacket.parse(p.to_line())
    assert (again.seq, again.t_ms) == (42, 1240)
    assert again.pitch == pytest.approx(-12.5)
    assert again.roll == pytest.approx(30.25)
    assert again.slider == pytest.approx(-0.75)
    assert again.cut and again.rec


@pytest.mark.parametrize("bad", ["", "NOPE 1 2 3 4 5 6", "PEND 1 2 3"])
def test_bad_packets_rejected(bad):
    with pytest.raises(ValueError):
        PendantPacket.parse(bad)


# ----------------------------------------------------------- rate mapping

def test_deadzone_holds_still():
    m = RateMap()
    dphi, dpsi, dtheta = m.rates(pkt(pitch=3.0, roll=-4.0, slider=0.05))
    assert (dphi, dpsi, dtheta) == (0.0, 0.0, 0.0), \
        "a hand held roughly still must command zero rate"


def test_full_deflection_hits_configured_rate():
    m = RateMap()
    _, dpsi, dtheta = m.rates(pkt(pitch=60.0, roll=-60.0))  # past full scale
    assert dtheta == pytest.approx(m.theta_rate)
    assert dpsi == pytest.approx(-m.psi_rate)


def test_rates_are_signed_and_monotonic():
    m = RateMap()
    prev = 0.0
    for tilt in (10, 20, 30, 40):
        r = m.rates(pkt(pitch=tilt))[2]
        assert r > prev
        prev = r
    assert m.rates(pkt(pitch=-25))[2] == pytest.approx(-m.rates(pkt(pitch=25))[2])


def test_slider_drives_phi_only():
    dphi, dpsi, dtheta = RateMap().rates(pkt(slider=1.0))
    assert dphi == pytest.approx(RateMap().phi_rate)
    assert (dpsi, dtheta) == (0.0, 0.0)


def test_expo_gives_fine_control_near_centre():
    m = RateMap()
    half = m.deadzone_deg + (m.full_deg - m.deadzone_deg) / 2
    assert m.rates(pkt(pitch=half))[2] < 0.5 * m.theta_rate


# -------------------------------------------------------- UDP + watchdog

class FakePhone:
    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port = port
        self.seq = 0

    def send(self, **kw):
        self.seq += 1
        p = pkt(seq=self.seq, t_ms=self.seq * 20, **kw)
        self.sock.sendto(p.to_line().encode(), ("127.0.0.1", self.port))
        return p


@pytest.fixture
def link():
    rx = PendantReceiver(port=0)
    port = rx.sock.getsockname()[1]
    yield rx, FakePhone(port)
    rx.close()


def _await_packet(rx, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if rx.latest() is not None:
            return True
        time.sleep(0.01)
    return False


def test_packets_arrive_over_real_udp(link):
    rx, phone = link
    phone.send(pitch=20.0, cut=True)
    assert _await_packet(rx), "no packet received"
    got = rx.latest()
    assert got.pitch == pytest.approx(20.0) and got.cut


def test_watchdog_drops_the_link_when_phone_goes_quiet(link):
    rx, phone = link
    phone.send(pitch=30.0)
    assert _await_packet(rx)
    time.sleep(0.35)                      # longer than WATCHDOG_S
    assert rx.latest() is None, "a silent phone must read as dead, not stale"
    assert not rx.live


def test_dead_link_means_zero_rates(link):
    """The safety property: a crashed phone stops the arm."""
    rx, phone = link
    phone.send(pitch=40.0, slider=1.0)
    assert _await_packet(rx)
    rig = Rig(head=None)
    rig.arm.home()
    rig.arm.jog(*RateMap().rates(rx.latest()))
    assert np.any(rig.arm.jog_rates != 0)

    time.sleep(0.35)
    if rx.latest() is None:
        rig.arm.jog(0, 0, 0)
    assert np.all(rig.arm.jog_rates == 0)


def test_dropped_datagrams_are_counted(link):
    rx, phone = link
    phone.send(pitch=1.0)
    assert _await_packet(rx)
    phone.seq += 5                        # simulate loss
    phone.send(pitch=2.0)
    end = time.monotonic() + 1.0
    while time.monotonic() < end and rx.dropped == 0:
        time.sleep(0.01)
    assert rx.dropped >= 5


# ------------------------------------------------ the point: teach -> replay

def test_phone_taught_cut_replays(tmp_path):
    """Drive the arm from a scripted 'phone', record the ARM's encoders, then
    replay that log — the glove workflow, with an iPhone in the glove's place.
    """
    rig = Rig(head=Head(), seed=0)
    rig.arm.home()
    rig.goto(10.0, 80.0, 10.0)
    m = RateMap()

    rig.recorder.start()
    rig.clipper_on = True
    # a deliberate move: sweep up the head while easing the tilt open
    script = ([pkt(slider=0.8, pitch=12.0)] * 100 +
              [pkt(slider=0.8, pitch=25.0)] * 100 +
              [pkt(slider=0.0, roll=20.0)] * 60)
    for p in script:
        rig.arm.jog(*m.rates(p))
        rig.tick()
    rig.arm.jog(0, 0, 0)
    rig.clipper_on = False
    log = rig.recorder.stop()

    assert len(log) == len(script)
    a = log.arrays()
    assert a["phi"][-1] > a["phi"][0] + 5, "slider should have climbed the head"
    assert a["theta"][-1] > a["theta"][0], "tilt should have opened up"

    # the saved log is an ordinary teach log: round-trips and replays
    path = tmp_path / "take.csv"
    path.write_text(log.to_csv())
    reloaded = TeachLog.from_csv(path.read_text())
    assert len(reloaded) == len(log)
    for got, want in zip(reloaded.samples, log.samples):
        # CSV keeps 3 decimals — far finer than the 0.1 deg encoder step
        assert got.t_ms == want.t_ms and got.contact == want.contact
        assert (got.phi, got.psi, got.theta) == \
            pytest.approx((want.phi, want.psi, want.theta), abs=1e-3)

    fresh = Rig(head=Head(), seed=5)
    rerec = replay(reloaded, fresh, record=True)
    for ax, v in rms_per_axis(smooth(reloaded), rerec).items():
        assert v < 1.0, f"{ax} RMS {v:.3f} deg over the 1 degree gate"

    # and the replayed cut matches the taught one
    _, mm_a = rig.head.length_vs_u()
    _, mm_b = fresh.head.length_vs_u()
    valid = ~np.isnan(mm_a) & ~np.isnan(mm_b)
    assert valid.any()
    assert np.max(np.abs(mm_a[valid] - mm_b[valid])) < 0.3


def test_phone_numbers_are_never_saved():
    """The log holds the ARM's encoders, not the phone's estimate — which is
    why a drifting phone still produces a faithful recording."""
    rig = Rig(head=None, seed=0)
    rig.arm.home()
    rig.recorder.start()
    drifting = pkt(pitch=30.0, roll=15.0, slider=0.5)
    for _ in range(50):
        rig.arm.jog(*RateMap().rates(drifting))
        rig.tick()
    log = rig.recorder.stop()
    a = log.arrays()
    # arm positions integrate smoothly from zero; they are not the phone's
    # constant 30/15 tilt readings
    assert a["theta"][0] < 1.0
    assert not np.allclose(a["theta"], 30.0)
    assert np.all(np.diff(a["theta"]) >= -1e-9), "arm should ramp, not jump"

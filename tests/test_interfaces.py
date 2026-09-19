"""Phase 0: all five shared blocks round-trip through their wire formats."""
import pytest

from fadegpt.interfaces import (CalibrationTable, HeadGeometry, LogSample,
                                Profile, ProfilePoint, TeachLog)
from fadegpt.protocol import Command, ProtocolError, parse_command


def test_log_line_round_trip():
    s = LogSample(t_ms=1240, phi=30.5, psi=91.2, theta=15.0, contact=True)
    assert LogSample.from_line(s.to_line()) == s


def test_teach_log_csv_round_trip():
    log = TeachLog([LogSample(i * 20, 10.0 + i, 90.0, 5.0, i % 2 == 0)
                    for i in range(10)])
    again = TeachLog.from_csv(log.to_csv())
    assert again.samples == log.samples
    assert log.to_csv().splitlines()[0] == "t_ms,phi,psi,theta,contact"


def test_profile_json_round_trip():
    p = Profile("low_fade", [ProfilePoint(0.0, 0.5), ProfilePoint(0.45, 1.6),
                             ProfilePoint(1.0, 6.0)])
    again = Profile.from_json(p.to_json())
    assert again.name == "low_fade"
    assert [(pt.u, pt.mm) for pt in again.points] == \
        [(pt.u, pt.mm) for pt in p.points]


def test_calibration_json_round_trip():
    c = CalibrationTable(25.0, [[0, 0.6], [15, 1.5], [30, 3.1],
                                [45, 5.4], [60, 8.2]])
    again = CalibrationTable.from_json(c.to_json())
    assert again == c


def test_head_geometry_json_round_trip():
    g = HeadGeometry(8.0, 62.0)
    assert HeadGeometry.from_json(g.to_json()) == g


def test_commands_parse():
    assert parse_command("HOME") == Command("HOME")
    assert parse_command("MOVE 30 90 15") == Command("MOVE", (30.0, 90.0, 15.0))
    assert parse_command("JOG 5 0 0") == Command("JOG", (5.0, 0.0, 0.0))
    assert parse_command("REC START") == Command("REC", ("START",))
    assert parse_command("rec stop") == Command("REC", ("STOP",))
    cmd = parse_command("REPLAY 500")
    assert cmd.name == "REPLAY" and cmd.expect_lines == 500
    assert parse_command("STATUS") == Command("STATUS")


@pytest.mark.parametrize("bad", ["", "MOVE 1 2", "MOVE a b c", "REC GO",
                                 "REPLAY x", "FLY 1 2 3", "REPLAY -3"])
def test_bad_commands_refused(bad):
    with pytest.raises(ProtocolError):
        parse_command(bad)

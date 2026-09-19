"""T6.1 golden-string: sample lines the FadePose app emits must parse cleanly."""
from pathlib import Path

import pytest

from fadegpt.pose_protocol import TRACK_NORMAL, PoseSession, parse_message

# Checked-in contract samples (must stay in sync with ios/FadePose).
GOLDEN = Path(__file__).resolve().parent / "golden_fadepose_lines.txt"


def test_golden_file_exists():
    assert GOLDEN.is_file()


def test_golden_strings_accepted_by_protocol():
    session = PoseSession()
    lines = [ln.strip() for ln in GOLDEN.read_text().splitlines()
             if ln.strip() and not ln.startswith("#")]
    assert lines[0] == "START"
    assert session.handle(parse_message(lines[0]), track=TRACK_NORMAL) == "OK"
    for line in lines[1:]:
        if line == "STOP":
            assert session.handle(parse_message(line)) == "OK"
            continue
        msg = parse_message(line)
        assert msg.kind == "POSE"
        assert msg.track == TRACK_NORMAL
        assert session.handle(msg) == "OK"

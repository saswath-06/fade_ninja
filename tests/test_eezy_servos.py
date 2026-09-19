"""Phase 7 dry-run servo pulse mapping."""
from fadegpt.eezy_servos import ServoDriver, angle_to_pulse_us


def test_angle_to_pulse_endpoints():
    assert angle_to_pulse_us(-90) == 500
    assert angle_to_pulse_us(90) == 2500


def test_dry_run_set_joints_records_pulses():
    d = ServoDriver(dry_run=True)
    d.set_joints(0, 30, -60, 10)
    assert d.pulses is not None
    assert len(d.pulses) == 4
    d.freeze()
    assert d.frozen

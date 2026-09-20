"""Finding the board, and failing to, without hardware.

The port name differs per OS (macOS /dev/cu.usbmodem*, Linux /dev/ttyACM*),
and the everyday failures -- charge-only cable, unflashed board -- must come
out as a readable message rather than a pyserial traceback.
"""
import subprocess
import sys

import pytest

from fade_ninja.hardware_arm import LinkError, find_serial_port
import fade_ninja.hardware_arm as ha


def fake_ports(monkeypatch, devices):
    monkeypatch.setattr(ha, "list_serial_ports",
                        lambda: [(d, "") for d in devices])


def test_finds_the_mac_name(monkeypatch):
    fake_ports(monkeypatch, ["/dev/cu.Bluetooth-Incoming-Port",
                             "/dev/cu.usbmodemE83DC1E566BC2"])
    assert find_serial_port() == "/dev/cu.usbmodemE83DC1E566BC2"


def test_finds_the_linux_name(monkeypatch):
    fake_ports(monkeypatch, ["/dev/ttyS0", "/dev/ttyACM0"])
    assert find_serial_port() == "/dev/ttyACM0"


def test_ignores_bluetooth_and_console(monkeypatch):
    # these two are always present on a Mac and are never the arm
    fake_ports(monkeypatch, ["/dev/cu.Bluetooth-Incoming-Port",
                             "/dev/cu.debug-console"])
    with pytest.raises(LinkError) as e:
        find_serial_port()
    assert "cu.Bluetooth-Incoming-Port" in str(e.value)   # lists what it saw


def test_no_ports_at_all_names_the_cable(monkeypatch):
    fake_ports(monkeypatch, [])
    with pytest.raises(LinkError) as e:
        find_serial_port()
    assert "charge-only" in str(e.value)


def test_two_boards_refuses_to_guess(monkeypatch):
    fake_ports(monkeypatch, ["/dev/ttyACM0", "/dev/ttyACM1"])
    with pytest.raises(LinkError) as e:
        find_serial_port()
    assert "--servo-port" in str(e.value)


def test_bad_port_is_a_message_not_a_traceback():
    """The exact command that failed for real: a Linux path on a Mac."""
    r = subprocess.run([sys.executable, "arm3dof_server.py",
                        "--servo-port", "/dev/nope-not-a-port"],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 2
    assert "Traceback" not in r.stderr
    assert "Cannot reach the servo board" in r.stdout


def test_list_ports_exits_clean():
    r = subprocess.run([sys.executable, "arm3dof_server.py", "--list-ports"],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0

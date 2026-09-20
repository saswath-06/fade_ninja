"""B1.8 analog: send a log to the virtual ESP32 over TCP, it echoes it back
intact; commands arrive and execute end to end."""
import asyncio
import json
import threading

import pytest

from fade_ninja.calibration import default_calibration
from fade_ninja.teach import ramp_log

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from server import VirtualESP32  # noqa: E402


class ServerFixture:
    """Runs the asyncio server in a thread; talks to it over real TCP."""

    def __init__(self):
        self.port = None
        self._ready = threading.Event()
        self._loop = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        assert self._ready.wait(5), "server did not start"

    def _run(self):
        async def main():
            esp = VirtualESP32(tick_interval=0.0)  # fast mode
            server = await asyncio.start_server(esp.handle_client,
                                                "127.0.0.1", 0)
            self.port = server.sockets[0].getsockname()[1]
            tick = asyncio.create_task(esp.tick_forever())
            self._ready.set()
            try:
                async with server:
                    await server.serve_forever()
            finally:
                tick.cancel()

        self._loop = asyncio.new_event_loop()
        try:
            self._loop.run_until_complete(main())
        except Exception:
            pass


@pytest.fixture(scope="module")
def conn():
    import socket
    srv = ServerFixture()
    sock = socket.create_connection(("127.0.0.1", srv.port), timeout=10)
    f = sock.makefile("rw", newline="\n")

    def talk(*lines):
        for ln in lines:
            f.write(ln + "\n")
        f.flush()
        return f.readline().strip()

    yield talk, f
    sock.close()


def test_b18_log_round_trips_over_tcp(conn):
    talk, f = conn
    log = ramp_log(default_calibration(), duration_s=2.0)
    lines = log.to_csv().splitlines()  # header + 100 samples
    resp = talk(f"LOAD {len(log)}", *lines)
    assert resp == f"OK LOADED {len(log)}"

    resp = talk("DUMP")
    assert resp == f"OK LOG {len(log)}"
    echoed = [f.readline().strip() for _ in range(len(log) + 1)]  # header + n
    assert echoed == lines, "log did not survive the transport"


def test_commands_execute_over_tcp(conn):
    talk, f = conn
    assert talk("HOME") == "OK"
    assert talk("MOVE 10 20 5") == "OK"
    for _ in range(300):  # fast-mode ticks run continuously; poll STATUS
        resp = talk("STATUS")
        st = json.loads(resp.removeprefix("OK "))
        if st["mode"] == "idle":
            break
    assert st["mode"] == "idle"
    assert abs(st["phi"] - 10) < 0.3 and abs(st["psi"] - 20) < 0.3

    assert talk("REC START") == "OK"
    assert talk("JOG 3 0 0") == "OK"
    import time
    time.sleep(0.1)  # let some fast ticks accumulate samples
    assert talk("STOP") == "OK"
    resp = talk("REC STOP")
    assert resp.startswith("OK LOG ")
    n = int(resp.split()[-1])
    assert n > 0
    for _ in range(n + 1):
        f.readline()


def test_bad_input_gets_err_not_crash(conn):
    talk, _ = conn
    assert talk("FLY 1 2 3").startswith("ERR")
    assert talk("MOVE 999 0 0").startswith("ERR")
    assert talk("DUMP") .startswith("OK LOG")  # still alive from earlier LOAD

"""Virtual ESP32: a TCP server speaking the 0.2 command protocol, driving the
simulated rig at 50 Hz.

This is the "laptop pretending to be the ESP32" from gate B2.2 — point the
iOS app (or netcat) at it. Line-based, one command per line; REPLAY/LOAD are
followed by that many raw log lines.

    uv run python server.py [--port 8462] [--fast]
"""
from __future__ import annotations

import argparse
import asyncio
import json
from collections import deque

from fadegpt.arm import ArmError
from fadegpt.head import Head
from fadegpt.interfaces import TICK_S, LOG_HEADER, TeachLog
from fadegpt.protocol import ProtocolError, parse_command
from fadegpt.replay import ReplayRefused, _snap_to_limits, smooth, validate
from fadegpt.rig import Rig


class VirtualESP32:
    def __init__(self, rig: Rig | None = None, tick_interval: float = TICK_S):
        self.rig = rig or Rig(head=Head())
        self.tick_interval = tick_interval
        self._stored: TeachLog | None = None
        self._queue: deque = deque()
        self._positioning = False

    # ------------------------------------------------------------ tick loop

    async def tick_forever(self):
        while True:
            if self._positioning:
                if self.rig.arm.mode == "idle":
                    self._positioning = False
                    self.rig.clipper_on = True
            elif self._queue:
                s = self._queue.popleft()
                self.rig.arm.track(s.phi, s.psi, s.theta)
                if not self._queue:  # replay finished
                    self.rig.clipper_on = False
                    self.rig.arm.stop()
            self.rig.tick()
            if self.tick_interval > 0:
                await asyncio.sleep(self.tick_interval)
            else:
                await asyncio.sleep(0)

    # ------------------------------------------------------------ commands

    def execute(self, line: str, payload: list[str]) -> list[str]:
        try:
            cmd = parse_command(line)
        except ProtocolError as e:
            return [f"ERR {e}"]
        try:
            return self._dispatch(cmd, payload)
        except (ArmError, ReplayRefused, ValueError) as e:
            return [f"ERR {e}"]

    def _dispatch(self, cmd, payload: list[str]) -> list[str]:
        rig = self.rig
        if cmd.name == "HOME":
            rig.arm.home()
            return ["OK"]
        if cmd.name == "MOVE":
            rig.arm.move_to(*cmd.args)
            return ["OK"]
        if cmd.name == "JOG":
            rig.arm.jog(*cmd.args)
            return ["OK"]
        if cmd.name == "REC":
            if cmd.args[0] == "START":
                rig.recorder.start()
                return ["OK"]
            log = rig.recorder.stop()
            return [f"OK LOG {len(log)}"] + log.to_csv().splitlines()
        if cmd.name == "LOAD":
            self._stored = TeachLog.from_csv("\n".join(payload))
            return [f"OK LOADED {len(self._stored)}"]
        if cmd.name == "DUMP":
            if self._stored is None:
                return ["ERR nothing loaded"]
            return [f"OK LOG {len(self._stored)}"] + \
                self._stored.to_csv().splitlines()
        if cmd.name == "REPLAY":
            log = TeachLog.from_csv("\n".join(payload))
            lg = _snap_to_limits(smooth(log))
            validate(lg)  # raises ReplayRefused -> ERR, arm never moves
            if not rig.arm.homed:
                rig.arm.home()
            first = lg.samples[0]
            rig.arm.move_to(first.phi, first.psi, first.theta)
            self._positioning = True
            self._queue = deque(lg.samples)
            return [f"OK REPLAYING {len(lg)}"]
        if cmd.name == "STOP":
            self._queue.clear()
            self._positioning = False
            rig.clipper_on = False
            rig.arm.stop()
            return ["OK"]
        if cmd.name == "STATUS":
            st = rig.arm.status()
            st.update(contact=rig.contact, clipper=rig.clipper_on,
                      replaying=len(self._queue),
                      recording=rig.recorder.recording)
            return ["OK " + json.dumps(st)]
        return ["ERR unhandled"]

    # ------------------------------------------------------------ transport

    async def handle_client(self, reader: asyncio.StreamReader,
                            writer: asyncio.StreamWriter):
        try:
            await self._client_loop(reader, writer)
        except (ConnectionResetError, BrokenPipeError):
            pass  # client went away mid-write; nothing to clean up

    async def _client_loop(self, reader: asyncio.StreamReader,
                           writer: asyncio.StreamWriter):
        while True:
            raw = await reader.readline()
            if not raw:
                break
            line = raw.decode().strip()
            if not line:
                continue
            payload: list[str] = []
            try:
                n = parse_command(line).expect_lines
            except ProtocolError:
                n = 0
            read = 0
            while read < n:
                ln = (await reader.readline()).decode().strip()
                if ln.replace(" ", "") == LOG_HEADER:
                    continue  # header doesn't count against <n> data lines
                payload.append(ln)
                read += 1
            for out in self.execute(line, payload):
                writer.write((out + "\n").encode())
            await writer.drain()
        writer.close()

    async def serve(self, host: str = "127.0.0.1", port: int = 8462):
        server = await asyncio.start_server(self.handle_client, host, port)
        asyncio.create_task(self.tick_forever())
        addr = server.sockets[0].getsockname()
        print(f"virtual ESP32 listening on {addr[0]}:{addr[1]}")
        async with server:
            await server.serve_forever()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=8462)
    ap.add_argument("--fast", action="store_true",
                    help="tick as fast as possible (simulation, not realtime)")
    ap.add_argument("--hardware", metavar="SERIAL_PORT",
                    help="drive the real Arduino motor board instead of the "
                         "sim, e.g. --hardware /dev/ttyACM0")
    args = ap.parse_args()
    if args.hardware:
        from fadegpt.hardware_arm import ArmConfig, HardwareArm
        rig = Rig(head=None)  # no virtual head: contact comes from the switch
        rig.arm = HardwareArm.open(ArmConfig(port=args.hardware))
        esp = VirtualESP32(rig=rig, tick_interval=TICK_S)  # hardware = realtime
    else:
        esp = VirtualESP32(tick_interval=0.0 if args.fast else TICK_S)
    asyncio.run(esp.serve(port=args.port))


if __name__ == "__main__":
    main()

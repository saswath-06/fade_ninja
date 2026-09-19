"""0.2 command protocol, same over serial/BLE/TCP.

HOME
MOVE <phi> <psi> <theta>        degrees
JOG  <dphi> <dpsi> <dtheta>     deg/sec, teach mode
REC START | REC STOP            REC STOP returns the log
REPLAY <n>                      followed by n log lines
LOAD <n> | DUMP                 store / echo a log (transport check, B1.8)
STOP | STATUS
"""
from __future__ import annotations

from dataclasses import dataclass


class ProtocolError(ValueError):
    pass


@dataclass
class Command:
    name: str
    args: tuple = ()
    # Commands that are followed by <n> raw log lines set expect_lines.
    expect_lines: int = 0


def parse_command(line: str) -> Command:
    parts = line.strip().split()
    if not parts:
        raise ProtocolError("empty command")
    name = parts[0].upper()
    args = parts[1:]

    if name in ("HOME", "STOP", "STATUS", "DUMP"):
        if args:
            raise ProtocolError(f"{name} takes no arguments")
        return Command(name)

    if name in ("MOVE", "JOG"):
        if len(args) != 3:
            raise ProtocolError(f"{name} takes 3 numbers: phi psi theta")
        try:
            vals = tuple(float(a) for a in args)
        except ValueError:
            raise ProtocolError(f"{name}: arguments must be numbers") from None
        return Command(name, vals)

    if name == "REC":
        if len(args) != 1 or args[0].upper() not in ("START", "STOP"):
            raise ProtocolError("REC takes START or STOP")
        return Command(name, (args[0].upper(),))

    if name in ("REPLAY", "LOAD"):
        if len(args) != 1:
            raise ProtocolError(f"{name} takes the number of log lines")
        try:
            n = int(args[0])
        except ValueError:
            raise ProtocolError(f"{name}: line count must be an integer") from None
        if n <= 0:
            raise ProtocolError(f"{name}: line count must be positive")
        return Command(name, (n,), expect_lines=n)

    raise ProtocolError(f"unknown command {name}")

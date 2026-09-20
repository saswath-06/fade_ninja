"""Optional Sentry tracing for the control loop.

This is a latency system, not a CRUD app: the interesting failures are "the
arm lagged the hand" and "a pose took too long to solve", which only show up
as timing. So the spans wrap the three stages that can actually be slow —
parsing a pose datagram, solving inverse kinematics, and writing servos —
and a breadcrumb records when the phone link goes stale.

Entirely optional: with no SENTRY_DSN set, every helper is a no-op and the
sentry_sdk import is never attempted, so the robot has no new dependency.
"""
from __future__ import annotations

import contextlib
import os

_sentry = None
_enabled = False


def init(dsn: str | None = None, traces_sample_rate: float = 0.2) -> bool:
    """Turn tracing on if a DSN is available. Returns whether it is active."""
    global _sentry, _enabled
    dsn = dsn or os.environ.get("SENTRY_DSN")
    if not dsn:
        return False
    try:
        import sentry_sdk
    except ImportError:
        return False
    sentry_sdk.init(
        dsn=dsn,
        traces_sample_rate=traces_sample_rate,
        # the loop runs 50x a second; profiling every tick would cost more
        # than the work being measured
        profiles_sample_rate=0.0,
        send_default_pii=False,
    )
    _sentry = sentry_sdk
    _enabled = True
    return True


@property
def enabled() -> bool:                      # pragma: no cover - trivial
    return _enabled


def is_enabled() -> bool:
    return _enabled


@contextlib.contextmanager
def span(op: str, description: str = ""):
    """Time one stage. A no-op when tracing is off, so call sites stay clean."""
    if not _enabled:
        yield None
        return
    with _sentry.start_span(op=op, description=description) as s:
        yield s


@contextlib.contextmanager
def transaction(name: str, op: str = "robot.tick"):
    if not _enabled:
        yield None
        return
    with _sentry.start_transaction(name=name, op=op) as t:
        yield t


def breadcrumb(message: str, category: str = "robot", level: str = "info",
               **data) -> None:
    if not _enabled:
        return
    _sentry.add_breadcrumb(category=category, message=message, level=level,
                           data=data or None)


def measure(name: str, value: float, unit: str = "millisecond") -> None:
    """Attach a number to the current transaction, e.g. tracking lag."""
    if not _enabled:
        return
    scope = _sentry.get_current_scope()
    tx = getattr(scope, "transaction", None)
    if tx is not None and hasattr(tx, "set_measurement"):
        tx.set_measurement(name, value, unit)


def capture(exc: BaseException) -> None:
    if _enabled:
        _sentry.capture_exception(exc)

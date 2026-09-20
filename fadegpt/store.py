"""Per-user cut library and joint telemetry.

Two kinds of data, stored differently on purpose:

* **users / takes** — a handful of rows, read constantly. Plain relational.
* **telemetry** — every joint angle at 50 Hz, append-only, always queried as
  "this take, in time order". That is a time series, so on TimescaleDB the
  table becomes a hypertable and gets time-bucketed reads for charting.

Backend is chosen by DATABASE_URL: Postgres/TimescaleDB when set, otherwise
SQLite so the thing runs with no setup at all. The SQL is kept to the subset
both speak, with the placeholder style swapped per driver.
"""
from __future__ import annotations

import os
import re
import sqlite3
import time
import uuid
from dataclasses import dataclass, asdict
from pathlib import Path

DEFAULT_SQLITE = Path(__file__).parent.parent / "out" / "fadeninja.db"
HANDLE_RE = re.compile(r"^[a-z0-9_][a-z0-9_.-]{1,30}$")


class StoreError(RuntimeError):
    pass


@dataclass
class User:
    id: str
    handle: str
    created_at: float


@dataclass
class Take:
    id: str
    user_id: str
    name: str
    created_at: float
    duration_ms: int
    n_samples: int

    def to_json(self) -> dict:
        d = asdict(self)
        d["duration_s"] = round(self.duration_ms / 1000.0, 1)
        return d


def normalise_handle(handle: str) -> str:
    """Handles are the identity here, so normalise before they become one."""
    h = (handle or "").strip().lower()
    if not HANDLE_RE.match(h):
        raise StoreError(
            "handle must be 2-31 characters: letters, digits, _ . -")
    return h


class Store:
    def __init__(self, url: str | None = None, path: Path | None = None):
        self.url = url if url is not None else os.environ.get("DATABASE_URL")
        self.timescale = False
        if self.url:
            try:
                import psycopg  # noqa: F401
            except ImportError as e:
                raise StoreError(
                    "DATABASE_URL is set but psycopg is not installed: "
                    "uv sync --extra db") from e
            import psycopg
            self._conn = psycopg.connect(self.url, autocommit=True)
            self._ph = "%s"
            self.kind = "postgres"
        else:
            p = path or DEFAULT_SQLITE
            p.parent.mkdir(parents=True, exist_ok=True)
            self._conn = sqlite3.connect(str(p), check_same_thread=False)
            self._conn.execute("PRAGMA journal_mode=WAL")
            self._ph = "?"
            self.kind = "sqlite"
        self._migrate()

    # ------------------------------------------------------------- schema

    def _sql(self, q: str) -> str:
        return q.replace("?", self._ph) if self._ph != "?" else q

    def _exec(self, q: str, args: tuple = ()):
        cur = self._conn.cursor()
        cur.execute(self._sql(q), args)
        return cur

    def _migrate(self) -> None:
        self._exec("""CREATE TABLE IF NOT EXISTS users (
            id TEXT PRIMARY KEY, handle TEXT UNIQUE NOT NULL,
            created_at DOUBLE PRECISION NOT NULL)""")
        self._exec("""CREATE TABLE IF NOT EXISTS takes (
            id TEXT PRIMARY KEY, user_id TEXT NOT NULL, name TEXT NOT NULL,
            created_at DOUBLE PRECISION NOT NULL,
            duration_ms INTEGER NOT NULL, n_samples INTEGER NOT NULL)""")
        self._exec("""CREATE TABLE IF NOT EXISTS telemetry (
            take_id TEXT NOT NULL, user_id TEXT NOT NULL,
            t_ms INTEGER NOT NULL, ts DOUBLE PRECISION NOT NULL,
            q1 REAL NOT NULL, q2 REAL NOT NULL,
            q3 REAL NOT NULL, q4 REAL NOT NULL,
            reachable INTEGER NOT NULL)""")
        self._exec("CREATE INDEX IF NOT EXISTS telemetry_take "
                   "ON telemetry (take_id, t_ms)")
        if self.kind == "postgres":
            try:
                self._exec("CREATE EXTENSION IF NOT EXISTS timescaledb")
                self._exec("SELECT create_hypertable('telemetry', 'ts', "
                           "if_not_exists => TRUE, migrate_data => TRUE)")
                self.timescale = True
            except Exception:
                # plain Postgres is fine; only the time-bucketed read differs
                self.timescale = False
        if self.kind == "sqlite":
            self._conn.commit()

    # -------------------------------------------------------------- users

    def sign_in(self, handle: str) -> User:
        """Sign-in creates the account if it is new.

        No password: this is a demo identity, not authentication, and saying
        so is better than implying a security property that is not there.
        """
        h = normalise_handle(handle)
        row = self._exec("SELECT id, handle, created_at FROM users "
                         "WHERE handle = ?", (h,)).fetchone()
        if row:
            return User(row[0], row[1], row[2])
        u = User(uuid.uuid4().hex[:12], h, time.time())
        self._exec("INSERT INTO users (id, handle, created_at) "
                   "VALUES (?, ?, ?)", (u.id, u.handle, u.created_at))
        self._commit()
        return u

    def user_by_id(self, user_id: str) -> User | None:
        row = self._exec("SELECT id, handle, created_at FROM users "
                         "WHERE id = ?", (user_id,)).fetchone()
        return User(row[0], row[1], row[2]) if row else None

    # -------------------------------------------------------------- takes

    def save_take(self, user_id: str, name: str, samples: list) -> Take:
        """samples: [(t_ms, q1, q2, q3, q4, reachable), ...]"""
        if not samples:
            raise StoreError("refusing to save an empty take")
        take = Take(uuid.uuid4().hex[:12], user_id, name.strip() or "untitled",
                    time.time(), int(samples[-1][0]), len(samples))
        self._exec("INSERT INTO takes (id, user_id, name, created_at, "
                   "duration_ms, n_samples) VALUES (?, ?, ?, ?, ?, ?)",
                   (take.id, take.user_id, take.name, take.created_at,
                    take.duration_ms, take.n_samples))
        base = take.created_at
        rows = [(take.id, user_id, int(s[0]), base + s[0] / 1000.0,
                 float(s[1]), float(s[2]), float(s[3]), float(s[4]),
                 int(bool(s[5]))) for s in samples]
        cur = self._conn.cursor()
        cur.executemany(self._sql(
            "INSERT INTO telemetry (take_id, user_id, t_ms, ts, q1, q2, q3, "
            "q4, reachable) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)"), rows)
        self._commit()
        return take

    def takes_for(self, user_id: str, limit: int = 50) -> list[Take]:
        rows = self._exec(
            "SELECT id, user_id, name, created_at, duration_ms, n_samples "
            "FROM takes WHERE user_id = ? ORDER BY created_at DESC "
            f"LIMIT {int(limit)}", (user_id,)).fetchall()
        return [Take(*r) for r in rows]

    def take(self, take_id: str) -> Take | None:
        row = self._exec(
            "SELECT id, user_id, name, created_at, duration_ms, n_samples "
            "FROM takes WHERE id = ?", (take_id,)).fetchone()
        return Take(*row) if row else None

    def samples(self, take_id: str) -> list[tuple]:
        return [tuple(r) for r in self._exec(
            "SELECT t_ms, q1, q2, q3, q4, reachable FROM telemetry "
            "WHERE take_id = ? ORDER BY t_ms", (take_id,)).fetchall()]

    def delete_take(self, take_id: str, user_id: str) -> bool:
        """Scoped by user so an id alone cannot delete someone else's take."""
        cur = self._exec("DELETE FROM takes WHERE id = ? AND user_id = ?",
                         (take_id, user_id))
        gone = cur.rowcount > 0
        if gone:
            self._exec("DELETE FROM telemetry WHERE take_id = ?", (take_id,))
        self._commit()
        return gone

    # ---------------------------------------------------------- analytics

    def take_summary(self, take_id: str, buckets: int = 120) -> list[dict]:
        """Downsampled joint traces for charting.

        On TimescaleDB this is a time_bucket aggregate; elsewhere the same
        shape is produced by striding, so the chart code stays identical.
        """
        rows = self.samples(take_id)
        if not rows:
            return []
        if self.timescale:
            width = max(1, int(rows[-1][0] / max(buckets, 1)))
            agg = self._exec(
                "SELECT (t_ms / ?) * ? AS b, AVG(q1), AVG(q2), AVG(q3), "
                "AVG(q4) FROM telemetry WHERE take_id = ? "
                "GROUP BY b ORDER BY b", (width, width, take_id)).fetchall()
            return [{"t_ms": int(r[0]), "q1": float(r[1]), "q2": float(r[2]),
                     "q3": float(r[3]), "q4": float(r[4])} for r in agg]
        step = max(1, len(rows) // max(buckets, 1))
        return [{"t_ms": int(r[0]), "q1": float(r[1]), "q2": float(r[2]),
                 "q3": float(r[3]), "q4": float(r[4])}
                for r in rows[::step]]

    def stats(self) -> dict:
        n_u = self._exec("SELECT COUNT(*) FROM users").fetchone()[0]
        n_t = self._exec("SELECT COUNT(*) FROM takes").fetchone()[0]
        n_s = self._exec("SELECT COUNT(*) FROM telemetry").fetchone()[0]
        return {"backend": self.kind, "timescale": self.timescale,
                "users": n_u, "takes": n_t, "samples": n_s}

    def _commit(self) -> None:
        if self.kind == "sqlite":
            self._conn.commit()

    def close(self) -> None:
        self._conn.close()

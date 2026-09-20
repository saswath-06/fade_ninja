"""HTTP API behind the dashboard: sign in, list cuts, record, replay.

Kept apart from pose_server's transport so it can be tested without sockets.
Every call takes the signed-in user id and scopes to it, so one person's take
ids are useless against another's library.
"""
from __future__ import annotations

import json

from .store import Store, StoreError
from .take import ReplayRefused, TakePlayer, prepare


class ApiError(Exception):
    def __init__(self, message: str, status: int = 400):
        super().__init__(message)
        self.status = status
        self.message = message


class DashboardAPI:
    """`session` is the live teleop server; it supplies the recorder and the
    current joints, and accepts a player to run a saved cut."""

    def __init__(self, store: Store, session):
        self.store = store
        self.session = session

    # ------------------------------------------------------------- users

    def sign_in(self, handle: str) -> dict:
        try:
            user = self.store.sign_in(handle)
        except StoreError as e:
            raise ApiError(str(e)) from None
        return {"user": {"id": user.id, "handle": user.handle},
                "takes": [t.to_json() for t in self.store.takes_for(user.id)]}

    def _user(self, user_id: str):
        user = self.store.user_by_id(user_id or "")
        if user is None:
            raise ApiError("signed out — sign in again", 401)
        return user

    # ------------------------------------------------------------- takes

    def takes(self, user_id: str) -> dict:
        user = self._user(user_id)
        return {"takes": [t.to_json() for t in self.store.takes_for(user.id)]}

    def start_recording(self, user_id: str) -> dict:
        self._user(user_id)
        if not self.session.started:
            raise ApiError("press Start in the phone app first", 409)
        self.session.begin_recording()
        return {"recording": True}

    def stop_recording(self, user_id: str, name: str) -> dict:
        user = self._user(user_id)
        rows = self.session.end_recording()
        if not rows:
            raise ApiError("nothing was recorded", 409)
        take = self.store.save_take(user.id, name, rows)
        return {"take": take.to_json(),
                "takes": [t.to_json() for t in self.store.takes_for(user.id)]}

    def replay(self, user_id: str, take_id: str) -> dict:
        user = self._user(user_id)
        take = self.store.take(take_id)
        if take is None or take.user_id != user.id:
            raise ApiError("no such cut in your library", 404)
        rows = self.store.samples(take_id)
        try:
            player = TakePlayer(rows)
        except ReplayRefused as e:
            # the arm has not moved; say why rather than failing silently
            raise ApiError(f"refused to replay: {e}", 422) from None
        self.session.begin_playback(player, take.name)
        return {"replaying": take.to_json()}

    def stop_replay(self, user_id: str) -> dict:
        self._user(user_id)
        self.session.stop_playback()
        return {"replaying": None}

    def delete(self, user_id: str, take_id: str) -> dict:
        user = self._user(user_id)
        if not self.store.delete_take(take_id, user.id):
            raise ApiError("no such cut in your library", 404)
        return {"takes": [t.to_json() for t in self.store.takes_for(user.id)]}

    def chart(self, user_id: str, take_id: str) -> dict:
        user = self._user(user_id)
        take = self.store.take(take_id)
        if take is None or take.user_id != user.id:
            raise ApiError("no such cut in your library", 404)
        return {"take": take.to_json(),
                "points": self.store.take_summary(take_id)}

    def stats(self) -> dict:
        return self.store.stats()

    # ----------------------------------------------------------- routing

    def handle(self, path: str, body: dict) -> dict:
        user_id = body.get("user_id", "")
        if path == "/api/signin":
            return self.sign_in(body.get("handle", ""))
        if path == "/api/takes":
            return self.takes(user_id)
        if path == "/api/record/start":
            return self.start_recording(user_id)
        if path == "/api/record/stop":
            return self.stop_recording(user_id, body.get("name", ""))
        if path == "/api/replay":
            return self.replay(user_id, body.get("take_id", ""))
        if path == "/api/replay/stop":
            return self.stop_replay(user_id)
        if path == "/api/delete":
            return self.delete(user_id, body.get("take_id", ""))
        if path == "/api/chart":
            return self.chart(user_id, body.get("take_id", ""))
        if path == "/api/stats":
            return self.stats()
        raise ApiError(f"unknown endpoint {path}", 404)

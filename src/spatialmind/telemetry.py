"""Append-only task event stream and resumable checkpoints."""

from __future__ import annotations

import json
import sqlite3
from pathlib import Path
from typing import Any

from spatialmind.models import utc_now


class EventStore:
    def __init__(self, path: str | Path = ":memory:") -> None:
        self.db = sqlite3.connect(str(path))
        self.db.row_factory = sqlite3.Row
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                task_id TEXT NOT NULL,
                time TEXT NOT NULL,
                type TEXT NOT NULL,
                data TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS events_by_task ON events(task_id, id);
            CREATE TABLE IF NOT EXISTS checkpoints (
                task_id TEXT PRIMARY KEY,
                updated_at TEXT NOT NULL,
                state TEXT NOT NULL
            );
        """)
        self.db.commit()

    def emit(self, task_id: str, event_type: str, **data: Any) -> int:
        with self.db:
            cursor = self.db.execute(
                "INSERT INTO events(task_id,time,type,data) VALUES (?,?,?,?)",
                (task_id, utc_now(), event_type, json.dumps(data, sort_keys=True)),
            )
        return int(cursor.lastrowid)

    def save(self, task_id: str, state: dict[str, Any]) -> None:
        with self.db:
            self.db.execute(
                "INSERT INTO checkpoints VALUES (?,?,?) ON CONFLICT(task_id) "
                "DO UPDATE SET updated_at=excluded.updated_at,state=excluded.state",
                (task_id, utc_now(), json.dumps(state, sort_keys=True)),
            )

    def load(self, task_id: str) -> dict[str, Any] | None:
        row = self.db.execute(
            "SELECT state FROM checkpoints WHERE task_id=?", (task_id,)
        ).fetchone()
        return json.loads(row["state"]) if row else None

    def events(self, task_id: str) -> list[dict[str, Any]]:
        rows = self.db.execute(
            "SELECT * FROM events WHERE task_id=? ORDER BY id", (task_id,)
        ).fetchall()
        return [
            {"id":row["id"], "task_id":row["task_id"], "time":row["time"],
             "type":row["type"], "data":json.loads(row["data"])}
            for row in rows
        ]

    def count(self, task_id: str) -> int:
        return int(self.db.execute(
            "SELECT COUNT(*) FROM events WHERE task_id=?", (task_id,)
        ).fetchone()[0])

    def close(self) -> None:
        self.db.close()

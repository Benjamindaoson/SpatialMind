"""SQLite temporal spatial memory with explicit negative evidence semantics."""

from __future__ import annotations

import json
import sqlite3
from dataclasses import dataclass
from pathlib import Path

from spatialmind.models import Detection, Observation, Point


@dataclass(frozen=True)
class RememberedObject:
    object_id: str
    label: str
    position: Point
    room: str | None
    confidence: float
    status: str
    last_observation_id: int
    sightings: int


class SpatialMemory:
    def __init__(self, path: str | Path = ":memory:") -> None:
        self.db = sqlite3.connect(str(path), check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS observations (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                source TEXT NOT NULL,
                payload TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS objects (
                object_id TEXT PRIMARY KEY,
                label TEXT NOT NULL,
                x INTEGER NOT NULL,
                y INTEGER NOT NULL,
                room TEXT,
                confidence REAL NOT NULL,
                status TEXT NOT NULL,
                last_observation_id INTEGER NOT NULL,
                sightings INTEGER NOT NULL
            );
            CREATE INDEX IF NOT EXISTS objects_label ON objects(label);
        """)
        self.db.commit()

    def close(self) -> None:
        self.db.close()

    def __enter__(self) -> "SpatialMemory":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _associate(self, detection: Detection) -> str:
        if detection.track_id:
            return detection.track_id
        # Without a tracker, associate only nearby objects with the same label.
        rows = self.db.execute(
            "SELECT object_id, x, y FROM objects WHERE label = ? AND status = 'observed'",
            (detection.label.lower(),),
        ).fetchall()
        near = [
            (abs(row["x"] - detection.position.x) + abs(row["y"] - detection.position.y),
             row["object_id"])
            for row in rows
        ]
        if near and min(near)[0] <= 2:
            return min(near)[1]
        # Deterministic anonymous id based on observation position, not an oracle id.
        return f"anon:{detection.label.lower()}:{detection.position.x}:{detection.position.y}"

    def update(self, observation: Observation) -> int:
        """Update sightings and invalidate only cells that were actually observable."""
        payload = json.dumps(observation.to_dict(), sort_keys=True)
        with self.db:
            cursor = self.db.execute(
                "INSERT INTO observations(timestamp, source, payload) VALUES (?, ?, ?)",
                (observation.timestamp, observation.source, payload),
            )
            obs_id = int(cursor.lastrowid)
            seen_ids = set()
            for detection in observation.detections:
                object_id = self._associate(detection)
                seen_ids.add(object_id)
                prior = self.db.execute(
                    "SELECT sightings FROM objects WHERE object_id = ?", (object_id,)
                ).fetchone()
                sightings = int(prior["sightings"]) + 1 if prior else 1
                self.db.execute("""
                    INSERT INTO objects (
                        object_id, label, x, y, room, confidence,
                        status, last_observation_id, sightings
                    ) VALUES (?, ?, ?, ?, ?, ?, 'observed', ?, ?)
                    ON CONFLICT(object_id) DO UPDATE SET
                        label=excluded.label, x=excluded.x, y=excluded.y,
                        room=excluded.room, confidence=excluded.confidence,
                        status='observed',
                        last_observation_id=excluded.last_observation_id,
                        sightings=excluded.sightings
                """, (
                    object_id, detection.label.lower(), detection.position.x,
                    detection.position.y, detection.room or observation.room,
                    detection.confidence, obs_id, sightings,
                ))
            for row in self.db.execute(
                "SELECT object_id, x, y, confidence, status FROM objects"
            ).fetchall():
                if (row["object_id"] not in seen_ids
                        and row["status"] == "observed"
                        and Point(row["x"], row["y"]) in observation.visible_cells):
                    self.db.execute(
                        "UPDATE objects SET confidence=?, status='missing', "
                        "last_observation_id=? WHERE object_id=?",
                        (round(float(row["confidence"]) * 0.25, 4),
                         obs_id, row["object_id"]),
                    )
        return obs_id

    def get(self, object_id: str) -> RememberedObject | None:
        row = self.db.execute(
            "SELECT * FROM objects WHERE object_id=?", (object_id,)
        ).fetchone()
        return self._from_row(row) if row else None

    def candidates(self, target: str, *, include_missing: bool = False) -> list[RememberedObject]:
        target = target.lower().strip()
        rows = self.db.execute(
            "SELECT * FROM objects ORDER BY confidence DESC, sightings DESC"
        ).fetchall()
        return [
            result
            for row in rows
            if target in (result := self._from_row(row)).label
            and (include_missing or result.status == "observed")
        ]

    @staticmethod
    def _from_row(row: sqlite3.Row) -> RememberedObject:
        return RememberedObject(
            object_id=row["object_id"], label=row["label"],
            position=Point(row["x"], row["y"]), room=row["room"],
            confidence=float(row["confidence"]), status=row["status"],
            last_observation_id=int(row["last_observation_id"]),
            sightings=int(row["sightings"]),
        )

    def observation(self, obs_id: int) -> dict | None:
        row = self.db.execute(
            "SELECT payload FROM observations WHERE id=?", (obs_id,)
        ).fetchone()
        return json.loads(row["payload"]) if row else None

    def count(self) -> int:
        return int(self.db.execute("SELECT COUNT(*) FROM objects").fetchone()[0])

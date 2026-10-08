"""Metric temporal object belief memory with conservative negative observations.

SQLite persists versions & evidence. ID association is gated by label,
distance, uncertainty; ambiguous matches create separate hypotheses.
"""
from __future__ import annotations

import math
import sqlite3
import time
import uuid
from dataclasses import dataclass

from spatialmind.physical import MetricObservation, ObjectEstimate, Pose2D


@dataclass(frozen=True)
class Belief:
    object_id: str
    label: str
    pose: Pose2D
    confidence: float
    last_seen: float
    evidence_ref: str | None
    status: str
    misses: int
    revision: int


class BeliefMemory:
    def __init__(self, db_path: str = ":memory:", *, resolution: float = 0.25,
                 association_radius_m: float = 1.0, max_age_s: float = 3600,
                 min_coverage_quality: float = 0.85, missing_after: int = 2):
        self.db = sqlite3.connect(db_path)
        self.db.row_factory = sqlite3.Row
        if resolution <= 0 or association_radius_m <= 0 or max_age_s <= 0 or missing_after < 1:
            raise ValueError("Invalid memory configuration")
        self.resolution, self.association_radius_m = resolution, association_radius_m
        self.max_age_s = max_age_s
        self.min_coverage_quality, self.missing_after = min_coverage_quality, missing_after
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS beliefs(
              object_id TEXT PRIMARY KEY, label TEXT, x REAL, y REAL,
              frame_id TEXT, variance REAL, confidence REAL, last_seen REAL,
              evidence_ref TEXT, status TEXT, misses INTEGER, revision INTEGER
            );
            CREATE TABLE IF NOT EXISTS belief_events(
              id INTEGER PRIMARY KEY AUTOINCREMENT, object_id TEXT,
              event TEXT, observed_at REAL, evidence_ref TEXT, revision INTEGER
            );
        """)
        self.db.commit()

    @staticmethod
    def _to_belief(row: sqlite3.Row) -> Belief:
        return Belief(
            row["object_id"],row["label"],
            Pose2D(row["x"],row["y"],frame_id=row["frame_id"],
                   stamp=row["last_seen"],position_variance=row["variance"]),
            row["confidence"],row["last_seen"],row["evidence_ref"],
            row["status"],row["misses"],row["revision"],
        )

    def get(self, object_id: str) -> Belief | None:
        row=self.db.execute("SELECT * FROM beliefs WHERE object_id=?",(object_id,)).fetchone()
        return self._to_belief(row) if row else None

    def candidates(self, label: str, *, now: float | None = None,
                   include_missing: bool = False) -> list[Belief]:
        moment=time.time() if now is None else now
        rows=self.db.execute(
            "SELECT * FROM beliefs WHERE label=? ORDER BY confidence DESC",(label,)
        ).fetchall()
        return [b for row in rows if (b:=self._to_belief(row))
                and (include_missing or b.status=="observed")
                and moment-b.last_seen < self.max_age_s]

    def _identity(self, d: ObjectEstimate) -> str:
        if d.track_id:
            return d.track_id
        rows=self.db.execute(
            "SELECT * FROM beliefs WHERE label=? AND status != 'missing'",(d.label,)
        ).fetchall()
        near=[]
        for row in rows:
            b=self._to_belief(row)
            try: gap=d.pose.distance(b.pose)
            except ValueError: continue
            threshold=self.association_radius_m+2*math.sqrt(
                d.pose.position_variance+b.pose.position_variance)
            if gap<=threshold:
                near.append((gap,b.object_id))
        # Two similarly plausible IDs? Refuse to silently merge.
        near.sort()
        if near and (len(near)==1 or near[1][0]-near[0][0] > 0.3):
            return near[0][1]
        return uuid.uuid4().hex

    def update(self, obs: MetricObservation) -> list[str]:
        changed=[]
        matched:set[str]=set()
        with self.db:
            for d in obs.detections:
                identity=self._identity(d)
                matched.add(identity)
                old=self.get(identity)
                revision=(old.revision+1) if old else 1
                self.db.execute("""
                    INSERT INTO beliefs VALUES(?,?,?,?,?,?,?,?,?,?,?,?)
                    ON CONFLICT(object_id) DO UPDATE SET
                      label=excluded.label,x=excluded.x,y=excluded.y,
                      frame_id=excluded.frame_id,variance=excluded.variance,
                      confidence=excluded.confidence,last_seen=excluded.last_seen,
                      evidence_ref=excluded.evidence_ref,status='observed',
                      misses=0,revision=excluded.revision
                """, (identity,d.label,d.pose.x,d.pose.y,d.pose.frame_id,
                      d.pose.position_variance,d.confidence,obs.pose.stamp,
                      d.evidence_ref,"observed",0,revision))
                self.db.execute("INSERT INTO belief_events(object_id,event,observed_at,evidence_ref,revision) VALUES(?,?,?,?,?)",
                                (identity,"seen",obs.pose.stamp,d.evidence_ref,revision))
                changed.append(identity)
            # Negative evidence requires current, strong, observed coverage.
            if obs.coverage_quality>=self.min_coverage_quality:
                for row in self.db.execute(
                    "SELECT * FROM beliefs WHERE status != 'missing'"
                ).fetchall():
                    b=self._to_belief(row)
                    if b.object_id in matched or b.pose.frame_id != obs.pose.frame_id:
                        continue
                    if b.pose.cell(self.resolution) not in obs.visible_cells:
                        continue
                    if abs(obs.pose.stamp-b.last_seen)>self.max_age_s:
                        continue
                    misses=b.misses+1
                    status="missing" if misses >= self.missing_after else "uncertain"
                    self.db.execute(
                        "UPDATE beliefs SET status=?,misses=?,revision=? WHERE object_id=?",
                        (status,misses,b.revision+1,b.object_id))
                    self.db.execute("INSERT INTO belief_events(object_id,event,observed_at,evidence_ref,revision) VALUES(?,?,?,?,?)",
                                    (b.object_id,"negative_view",obs.pose.stamp,None,b.revision+1))
                    changed.append(b.object_id)
        return changed

    def count(self) -> int:
        return self.db.execute("SELECT COUNT(*) FROM beliefs").fetchone()[0]

    def close(self) -> None:
        self.db.close()

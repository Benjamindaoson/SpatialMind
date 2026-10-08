"""Typed contracts shared by the agent, memory and robot adapters."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from enum import StrEnum
from typing import Any


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True, order=True)
class Point:
    x: int
    y: int

    def distance(self, other: "Point") -> int:
        return abs(self.x - other.x) + abs(self.y - other.y)


@dataclass(frozen=True)
class Detection:
    """Object id is an optional track id from perception, not a ground-truth requirement."""

    label: str
    position: Point
    confidence: float = 1.0
    track_id: str | None = None
    room: str | None = None

    def __post_init__(self) -> None:
        if not 0 <= self.confidence <= 1:
            raise ValueError("confidence must be between 0 and 1")


@dataclass(frozen=True)
class Observation:
    pose: Point
    room: str | None
    detections: tuple[Detection, ...]
    visible_cells: frozenset[Point]
    timestamp: str = field(default_factory=utc_now)
    source: str = "sim"

    def to_dict(self) -> dict[str, Any]:
        return {
            "pose": asdict(self.pose),
            "room": self.room,
            "detections": [asdict(item) for item in self.detections],
            "visible_cells": [asdict(p) for p in sorted(self.visible_cells)],
            "timestamp": self.timestamp,
            "source": self.source,
        }


@dataclass(frozen=True)
class NavigationResult:
    success: bool
    steps: int
    reason: str = "ok"
    blocked_at: Point | None = None


class TaskStatus(StrEnum):
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    INTERRUPTED = "interrupted"


@dataclass(frozen=True)
class TaskRequest:
    kind: str
    target: str
    room: str | None = None

    def __post_init__(self) -> None:
        if self.kind not in {"find", "navigate"}:
            raise ValueError("Only find and navigate are currently supported")
        if not self.target:
            raise ValueError("target cannot be empty")


@dataclass
class TaskResult:
    task_id: str
    request: TaskRequest
    status: TaskStatus
    steps: int
    navigation_failures: int
    replans: int
    evidence_id: int | None
    position: Point
    reason: str
    events: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "task_id": self.task_id,
            "request": asdict(self.request),
            "status": self.status.value,
            "steps": self.steps,
            "navigation_failures": self.navigation_failures,
            "replans": self.replans,
            "evidence_id": self.evidence_id,
            "position": asdict(self.position),
            "reason": self.reason,
            "events": self.events,
        }

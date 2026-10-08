"""Explicit token-budget-friendly context projections; never truncate task invariants."""

from __future__ import annotations

import json
from typing import Any

from spatialmind.memory import SpatialMemory
from spatialmind.models import TaskRequest


class ContextEngine:
    def __init__(self, memory: SpatialMemory, max_characters: int = 2400) -> None:
        if max_characters < 300:
            raise ValueError("max_characters must be >= 300")
        self.memory = memory
        self.max_characters = max_characters

    def build(
        self, request: TaskRequest, robot_pose: tuple[int, int],
        recent_events: list[dict[str, Any]]
    ) -> dict[str, Any]:
        """Keep hard constraints; fit optional memory/events into the remaining budget."""
        required: dict[str, Any] = {
            "goal": {"kind": request.kind, "target": request.target, "room": request.room},
            "robot_pose": list(robot_pose),
        }
        result = dict(required)
        result["spatial_candidates"] = []
        result["recent_events"] = []
        for item in self.memory.candidates(request.target, include_missing=True)[:10]:
            candidate = {
                "id": item.object_id,
                "label": item.label,
                "pose": [item.position.x, item.position.y],
                "confidence": item.confidence,
                "status": item.status,
            }
            trial = dict(result)
            trial["spatial_candidates"] = result["spatial_candidates"] + [candidate]
            if len(json.dumps(trial)) <= self.max_characters:
                result = trial
        for event in reversed(recent_events[-20:]):
            compact = {"type": event["type"], "data": event["data"]}
            trial = dict(result)
            trial["recent_events"] = [compact] + result["recent_events"]
            if len(json.dumps(trial)) <= self.max_characters:
                result = trial
        return result

"""Memory-driven viewpoint selection. Uses only the known static map, never hidden objects."""

from __future__ import annotations

from dataclasses import dataclass

from spatialmind.memory import SpatialMemory
from spatialmind.models import Point, TaskRequest
from spatialmind.world import GridWorld


@dataclass(frozen=True)
class Decision:
    target: Point
    reason: str
    room: str
    source_id: str | None = None


class ActiveSearchPlanner:
    def __init__(self, world: GridWorld, sensor_range: int = 3) -> None:
        self.world = world
        self.sensor_range = sensor_range

    @staticmethod
    def _priors(target: str) -> dict[str, float]:
        if "first aid" in target:
            return {"storage": 0.55, "lab": 0.2, "office": 0.15, "meeting": 0.1}
        if "toolbox" in target:
            return {"lab": 0.55, "storage": 0.25, "office": 0.1, "meeting": 0.1}
        return {room: 0.25 for room in ("lab", "storage", "office", "meeting")}

    def choose(
        self,
        request: TaskRequest,
        pose: Point,
        memory: SpatialMemory,
        observed_cells: set[Point],
        tried_positions: set[Point],
        tried_memory_ids: set[str],
    ) -> Decision | None:
        if request.kind == "navigate":
            goal = self.world.room_center(request.room or request.target)
            if goal in tried_positions:
                return None
            return Decision(goal, "room_navigation", request.room or request.target)

        for item in memory.candidates(request.target):
            if (item.object_id not in tried_memory_ids and
                    item.position not in tried_positions and
                    item.confidence >= 0.5 and
                    (request.room is None or self.world.room_at(item.position) == request.room)):
                return Decision(
                    item.position, "verify_spatial_memory",
                    self.world.room_at(item.position), item.object_id,
                )

        priors = self._priors(request.target)
        if request.room is not None:
            priors = {name: (1.0 if name == request.room else 0.0) for name in priors}
        best: tuple[float, Point, str] | None = None
        for room, prior in priors.items():
            if prior == 0:
                continue
            room_points = self.world.room_points(room)
            unseen = set(room_points) - observed_cells
            if not unseen:
                continue
            for waypoint in room_points:
                if waypoint in tried_positions:
                    continue
                # Approximate visibility from static geometry. True observations
                # are ray-cast by the robot, not inferred here.
                coverage = sum(
                    1 for p in unseen if waypoint.distance(p) <= self.sensor_range
                )
                if coverage == 0:
                    continue
                score = prior * (coverage + 0.1) / (1 + 0.035 * pose.distance(waypoint))
                if best is None or score > best[0]:
                    best = (score, waypoint, room)
        if best is None:
            return None
        return Decision(best[1], "active_information_search", best[2])

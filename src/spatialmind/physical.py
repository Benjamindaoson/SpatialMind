"""Frame-aware metric geometry and hardware-independent robot contracts.

Grid Point coordinates are NOT metric poses. Every physical input carries
a reference frame and observation timestamp; consumers reject stale data.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"
    stamp: float = field(default_factory=time.time)
    position_variance: float = 0.0

    def __post_init__(self):
        if not self.frame_id or not all(map(math.isfinite, (
            self.x, self.y, self.yaw, self.stamp, self.position_variance
        ))) or self.position_variance < 0:
            raise ValueError("Invalid pose/frame/covariance")

    def distance(self, other: "Pose2D") -> float:
        if self.frame_id != other.frame_id:
            raise ValueError("Cannot compare poses in different frames")
        return math.hypot(self.x - other.x, self.y - other.y)

    def cell(self, resolution: float = 0.25) -> tuple[int, int]:
        if resolution <= 0:
            raise ValueError("resolution must be positive")
        return round(self.x / resolution), round(self.y / resolution)


@dataclass(frozen=True)
class FrameTransform:
    """Planar rigid transform from source_frame to target_frame."""
    source_frame: str
    target_frame: str
    x: float
    y: float
    yaw: float

    def apply(self, pose: Pose2D) -> Pose2D:
        if pose.frame_id != self.source_frame:
            raise ValueError(f"Expected {self.source_frame}, got {pose.frame_id}")
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(
            self.x + c * pose.x - s * pose.y,
            self.y + s * pose.x + c * pose.y,
            pose.yaw + self.yaw,
            self.target_frame, pose.stamp, pose.position_variance,
        )


@dataclass(frozen=True)
class MetricObservation:
    pose: Pose2D
    detections: tuple["ObjectEstimate", ...] = ()
    visible_cells: frozenset[tuple[int, int]] = frozenset()
    coverage_quality: float = 0.0
    source: str = "unknown"

    def __post_init__(self):
        if not 0 <= self.coverage_quality <= 1:
            raise ValueError("coverage_quality must be between 0 and 1")
        for d in self.detections:
            if d.pose.frame_id != self.pose.frame_id:
                raise ValueError("Detection and robot pose must share a frame")


@dataclass(frozen=True)
class ObjectEstimate:
    label: str
    pose: Pose2D
    confidence: float
    track_id: str | None = None
    evidence_ref: str | None = None

    def __post_init__(self):
        if not self.label or not 0 <= self.confidence <= 1:
            raise ValueError("Invalid object label/confidence")


@dataclass(frozen=True)
class MotionResult:
    success: bool
    reason: str
    duration_s: float = 0.0
    distance_m: float = 0.0
    nav_goal_id: str = ""

    def __post_init__(self):
        if self.duration_s < 0 or self.distance_m < 0:
            raise ValueError("Negative motion duration/distance")


@runtime_checkable
class MetricRobot(Protocol):
    @property
    def pose(self) -> Pose2D: ...
    async def navigate(self, target: Pose2D, *, timeout_s: float = 120) -> MotionResult: ...
    async def observe(self) -> MetricObservation: ...
    async def stop(self) -> None: ...


class SemanticMap(Protocol):
    def search_waypoints(self, room: str | None = None) -> list[tuple[str, Pose2D]]: ...
    def room_for(self, pose: Pose2D) -> str | None: ...
    def reachable(self, pose: Pose2D) -> bool: ...


class RoomMap:
    """Versioned semantic waypoints; configured from a calibrated real map.

    Preconfigured poses must be measured in the same ROS map coordinate frame.
    """
    def __init__(self, waypoints: dict[str, list[Pose2D]], version: str = "v1"):
        if not waypoints or not version:
            raise ValueError("Nonempty map and version required")
        self._waypoints = dict(waypoints)
        self.version = version
        frames = {p.frame_id for group in waypoints.values() for p in group}
        if len(frames) != 1:
            raise ValueError("All semantic locations must share one frame")

    def search_waypoints(self, room: str | None = None) -> list[tuple[str, Pose2D]]:
        if room is not None and room not in self._waypoints:
            return []
        return [(name, pose) for name, group in self._waypoints.items()
                for pose in group if room is None or name == room]

    def room_for(self, pose: Pose2D) -> str | None:
        candidates = self.search_waypoints()
        if not candidates:
            return None
        room, closest = min(candidates, key=lambda pair: pair[1].distance(pose))
        return room if closest.distance(pose) <= 4 else None

    def reachable(self, pose: Pose2D) -> bool:
        return any(pose.frame_id == p.frame_id for _, p in self.search_waypoints())

    @classmethod
    def from_json(cls, path: str) -> "RoomMap":
        import json
        with open(path, encoding="utf-8") as source:
            obj = json.load(source)
        if obj.get("calibrated") is not True:
            raise ValueError("Semantic waypoints are not calibrated for the active ROS map")
        return cls({
            room: [Pose2D(float(p["x"]), float(p["y"]), float(p.get("yaw", 0)),
                          obj.get("frame_id", "map"))
                   for p in entries]
            for room, entries in obj["rooms"].items()
        }, obj.get("version", "v1"))

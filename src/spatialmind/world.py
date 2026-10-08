"""Deterministic grid world with walls, uncertain observations and movable objects.

A small executable testbed, *not* a physics or ROS/Gazebo simulator.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Protocol

from spatialmind.models import Detection, NavigationResult, Observation, Point


@dataclass
class WorldObject:
    object_id: str
    label: str
    position: Point


class RobotAdapter(Protocol):
    @property
    def pose(self) -> Point: ...

    def navigate(self, target: Point) -> NavigationResult: ...

    def observe(self) -> Observation: ...

    def stop(self) -> None: ...


class GridWorld:
    """Four semantic regions connected by two doorways."""

    width = 20
    height = 12

    def __init__(self) -> None:
        self.walls: set[Point] = set()
        for x in range(self.width):
            self.walls.add(Point(x, 0))
            self.walls.add(Point(x, self.height - 1))
        for y in range(self.height):
            self.walls.add(Point(0, y))
            self.walls.add(Point(self.width - 1, y))
        for y in range(1, self.height - 1):
            if y not in {3, 8}:
                self.walls.add(Point(9, y))
        for x in list(range(1, 9)) + list(range(10, 19)):
            if x not in {4, 15}:
                self.walls.add(Point(x, 6))
        self.objects: dict[str, WorldObject] = {
            "toolbox_1": WorldObject("toolbox_1", "blue toolbox", Point(3, 3)),
            "firstaid_1": WorldObject("firstaid_1", "red first aid kit", Point(4, 9)),
        }
        self.obstacles: set[Point] = set()

    def passable(self, point: Point, *, include_obstacles: bool = True) -> bool:
        return (
            0 <= point.x < self.width
            and 0 <= point.y < self.height
            and point not in self.walls
            and (not include_obstacles or point not in self.obstacles)
        )

    def room_at(self, point: Point) -> str:
        if point.x < 9:
            return "lab" if point.y < 6 else "storage"
        return "meeting" if point.y < 6 else "office"

    def room_points(self, room: str) -> list[Point]:
        names = {"lab", "storage", "meeting", "office"}
        if room not in names:
            raise ValueError(f"Unknown room: {room}")
        return [
            Point(x, y)
            for y in range(1, self.height - 1)
            for x in range(1, self.width - 1)
            if self.passable(Point(x, y), include_obstacles=False)
            and self.room_at(Point(x, y)) == room
        ]

    def room_center(self, room: str) -> Point:
        points = self.room_points(room)
        avg_x = sum(p.x for p in points) / len(points)
        avg_y = sum(p.y for p in points) / len(points)
        return min(points, key=lambda p: (p.x - avg_x) ** 2 + (p.y - avg_y) ** 2)

    def move_object(self, object_id: str, destination: Point) -> None:
        if object_id not in self.objects:
            raise KeyError(object_id)
        if not self.passable(destination):
            raise ValueError("Object destination must be traversable")
        self.objects[object_id].position = destination

    def set_obstacle(self, point: Point, blocked: bool = True) -> None:
        if not self.passable(point, include_obstacles=False):
            raise ValueError("Obstacle location is outside free space")
        if blocked:
            self.obstacles.add(point)
        else:
            self.obstacles.discard(point)

    def render(self, robot: Point | None = None) -> str:
        canvas = [["#" if Point(x, y) in self.walls else "." for x in range(self.width)]
                  for y in range(self.height)]
        for point in self.obstacles:
            canvas[point.y][point.x] = "X"
        for item in self.objects.values():
            canvas[item.position.y][item.position.x] = "T" if "toolbox" in item.label else "F"
        if robot is not None:
            canvas[robot.y][robot.x] = "R"
        return "\n".join("".join(row) for row in canvas)


class SimRobot:
    """Grid navigation and line-of-sight sensing with explicit unknown obstacles."""

    def __init__(
        self, world: GridWorld, pose: Point = Point(2, 2), sensor_range: int = 3
    ) -> None:
        if not world.passable(pose):
            raise ValueError("Robot must spawn in free space")
        if sensor_range < 1:
            raise ValueError("sensor_range must be >= 1")
        self.world = world
        self._pose = pose
        self.sensor_range = sensor_range
        self.known_obstacles: set[Point] = set()
        self.motion_steps = 0

    @property
    def pose(self) -> Point:
        return self._pose

    def stop(self) -> None:
        """A no-op in the sequential simulator; real adapters must implement emergency stop."""

    def _neighbors(self, p: Point) -> list[Point]:
        return [Point(p.x + 1, p.y), Point(p.x, p.y + 1),
                Point(p.x - 1, p.y), Point(p.x, p.y - 1)]

    def _path(self, target: Point) -> list[Point] | None:
        if not self.world.passable(target, include_obstacles=False):
            return None
        queue: deque[Point] = deque([self._pose])
        previous: dict[Point, Point | None] = {self._pose: None}
        while queue:
            point = queue.popleft()
            if point == target:
                path = []
                while previous[point] is not None:
                    path.append(point)
                    point = previous[point]  # type: ignore[assignment]
                return list(reversed(path))
            for nxt in self._neighbors(point):
                if (nxt not in previous and nxt not in self.known_obstacles
                        and self.world.passable(nxt, include_obstacles=False)):
                    previous[nxt] = point
                    queue.append(nxt)
        return None

    def navigate(self, target: Point) -> NavigationResult:
        path = self._path(target)
        if path is None:
            return NavigationResult(False, 0, "unreachable")
        traversed = 0
        for point in path:
            if point in self.world.obstacles:
                self.known_obstacles.add(point)
                return NavigationResult(False, traversed, "blocked", blocked_at=point)
            self._pose = point
            traversed += 1
            self.motion_steps += 1
        return NavigationResult(True, traversed)

    def _has_line_of_sight(self, target: Point) -> bool:
        """Bresenham ray casting. Walls and dynamic obstacles occlude observations."""
        x0, y0 = self._pose.x, self._pose.y
        x1, y1 = target.x, target.y
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while (x0, y0) != (x1, y1):
            error = 2 * err
            if error > -dy:
                err -= dy
                x0 += sx
            if error < dx:
                err += dx
                y0 += sy
            point = Point(x0, y0)
            if point in self.world.walls or point in self.world.obstacles:
                return False
        return True

    def observe(self) -> Observation:
        cells = frozenset(
            Point(x, y)
            for x in range(max(1, self._pose.x - self.sensor_range),
                           min(self.world.width - 1, self._pose.x + self.sensor_range + 1))
            for y in range(max(1, self._pose.y - self.sensor_range),
                           min(self.world.height - 1, self._pose.y + self.sensor_range + 1))
            if self._pose.distance(Point(x, y)) <= self.sensor_range
            and self.world.passable(Point(x, y))
            and self._has_line_of_sight(Point(x, y))
        )
        detections = tuple(
            Detection(item.label, item.position, 1.0, item.object_id, self.world.room_at(item.position))
            for item in self.world.objects.values()
            if item.position in cells
        )
        return Observation(
            pose=self._pose, room=self.world.room_at(self._pose),
            detections=detections, visible_cells=cells,
        )

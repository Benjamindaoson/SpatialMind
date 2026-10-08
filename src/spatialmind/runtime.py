"""Event-driven agent loop with memory verification, replanning and checkpoints."""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from uuid import uuid4

from spatialmind.context import ContextEngine
from spatialmind.language import RuleBasedInterpreter
from spatialmind.memory import SpatialMemory
from spatialmind.models import Point, TaskRequest, TaskResult, TaskStatus
from spatialmind.planner import ActiveSearchPlanner
from spatialmind.telemetry import EventStore
from spatialmind.world import GridWorld, RobotAdapter


class AgentRuntime:
    def __init__(
        self, robot: RobotAdapter, world_map: GridWorld,
        memory: SpatialMemory, events: EventStore,
        *, max_navigation_retries: int = 2,
    ) -> None:
        self.robot = robot
        self.world_map = world_map
        self.memory = memory
        self.events = events
        self.interpreter = RuleBasedInterpreter()
        self.planner = ActiveSearchPlanner(world_map)
        self.context = ContextEngine(memory)
        self.max_navigation_retries = max_navigation_retries

    def _observe(self, task_id: str, state: dict) -> tuple[int, bool]:
        observation = self.robot.observe()
        obs_id = self.memory.update(observation)
        state["observed_cells"] = sorted({tuple(p) for p in state["observed_cells"]} | {
            (p.x, p.y) for p in observation.visible_cells
        })
        self.events.emit(
            task_id, "observation", observation_id=obs_id,
            robot_pose=asdict(observation.pose),
            detections=[asdict(d) for d in observation.detections],
            visible_count=len(observation.visible_cells),
        )
        self.events.emit(
            task_id, "memory_update", observation_id=obs_id,
            entity_count=self.memory.count(),
        )
        request = TaskRequest(**state["request"])
        matches = [
            d for d in observation.detections
            if request.kind == "find"
            and request.target in d.label.lower()
            and (not request.room or self.world_map.room_at(d.position) == request.room)
        ]
        if matches:
            state["evidence_id"] = obs_id
            self.events.emit(
                task_id, "goal_verified", observation_id=obs_id,
                target=request.target, position=asdict(matches[0].position),
            )
        return obs_id, bool(matches)

    def _checkpoint(self, task_id: str, state: dict) -> None:
        state["robot_pose"] = [self.robot.pose.x, self.robot.pose.y]
        self.events.save(task_id, state)

    def _result(self, task_id: str, state: dict) -> TaskResult:
        result = TaskResult(
            task_id=task_id, request=TaskRequest(**state["request"]),
            status=TaskStatus(state["status"]), steps=int(state["actions"]),
            navigation_failures=int(state["navigation_failures"]),
            replans=int(state["replans"]), evidence_id=state["evidence_id"],
            position=self.robot.pose, reason=state["reason"],
            events=self.events.count(task_id),
        )
        return result

    def run(
        self, command: str | None = None, *,
        task_id: str | None = None,
        resume: bool = False,
        max_actions: int = 80,
    ) -> TaskResult:
        if max_actions < 1:
            raise ValueError("max_actions must be >= 1")
        task_id = task_id or uuid4().hex[:12]
        if resume:
            state = self.events.load(task_id)
            if state is None:
                raise ValueError(f"Unknown checkpoint: {task_id}")
            if state["status"] == TaskStatus.SUCCEEDED.value:
                return self._result(task_id, state)
            state["status"] = TaskStatus.RUNNING.value
            old_pose = state.get("robot_pose")
            if old_pose != [self.robot.pose.x, self.robot.pose.y]:
                self.events.emit(
                    task_id, "pose_changed_on_resume",
                    previous=old_pose, current=asdict(self.robot.pose),
                )
                # The old coverage is still valid; current localization is refreshed.
            self.events.emit(task_id, "task_resumed", max_actions=max_actions)
        else:
            if command is None:
                raise ValueError("command is required for a new task")
            if self.events.load(task_id) is not None:
                raise ValueError("task_id already exists; use resume=True")
            request = self.interpreter.parse(command)
            state = {
                "request": asdict(request), "status": TaskStatus.RUNNING.value,
                "reason": "in_progress", "actions": 0,
                "navigation_failures": 0, "replans": 0,
                "evidence_id": None, "observed_cells": [],
                "tried_positions": [], "tried_memory_ids": [],
            }
            self.events.emit(
                task_id, "task_created", command=command, request=asdict(request),
            )
        _, found = self._observe(task_id, state)
        request = TaskRequest(**state["request"])
        if found:
            return self._finish(task_id, state, TaskStatus.SUCCEEDED, "observed_target")
        if request.kind == "navigate" and self.robot.pose == self.world_map.room_center(
            request.room or request.target
        ):
            return self._finish(task_id, state, TaskStatus.SUCCEEDED, "reached_room")

        for _ in range(max_actions):
            observed = {Point(x, y) for x, y in state["observed_cells"]}
            tried = {Point(x, y) for x, y in state["tried_positions"]}
            decision = self.planner.choose(
                request, self.robot.pose, self.memory, observed, tried,
                set(state["tried_memory_ids"]),
            )
            if decision is None:
                return self._finish(task_id, state, TaskStatus.FAILED, "search_exhausted")
            context = self.context.build(
                request, (self.robot.pose.x, self.robot.pose.y),
                self.events.events(task_id)[-12:],
            )
            self.events.emit(
                task_id, "plan_decision",
                target=asdict(decision.target), room=decision.room,
                reason=decision.reason, memory_id=decision.source_id,
                context_characters=len(str(context)),
            )
            navigation = None
            for attempt in range(self.max_navigation_retries + 1):
                self.events.emit(
                    task_id, "navigation_goal", goal=asdict(decision.target), attempt=attempt,
                )
                navigation = self.robot.navigate(decision.target)
                state["actions"] += 1
                self.events.emit(
                    task_id, "navigation_result", success=navigation.success,
                    reason=navigation.reason, steps=navigation.steps,
                    blocked_at=asdict(navigation.blocked_at) if navigation.blocked_at else None,
                )
                if navigation.success:
                    break
                state["navigation_failures"] += 1
                state["replans"] += 1
                if navigation.reason != "blocked":
                    break
            state["tried_positions"].append([decision.target.x, decision.target.y])
            if decision.source_id:
                state["tried_memory_ids"].append(decision.source_id)
            self._checkpoint(task_id, state)
            _, found = self._observe(task_id, state)
            if found:
                return self._finish(
                    task_id, state, TaskStatus.SUCCEEDED, "visually_verified"
                )
            if navigation is not None and navigation.success and request.kind == "navigate":
                return self._finish(task_id, state, TaskStatus.SUCCEEDED, "reached_room")
            state["replans"] += 1
            self._checkpoint(task_id, state)
        return self._finish(task_id, state, TaskStatus.INTERRUPTED, "action_budget_reached")

    def _finish(
        self, task_id: str, state: dict, status: TaskStatus, reason: str
    ) -> TaskResult:
        state["status"] = status.value
        state["reason"] = reason
        self._checkpoint(task_id, state)
        self.events.emit(
            task_id, "task_finished", status=status.value, reason=reason,
            actions=state["actions"], evidence_id=state["evidence_id"],
        )
        return self._result(task_id, state)


def open_runtime(
    world: GridWorld | None = None,
    *, data_dir: str | Path = ".spatialmind",
    sensor_range: int = 3,
) -> tuple[AgentRuntime, GridWorld]:
    """Convenience factory for the executable grid-world backend."""
    from spatialmind.world import SimRobot

    location = Path(data_dir)
    location.mkdir(parents=True, exist_ok=True)
    world = world or GridWorld()
    robot = SimRobot(world, sensor_range=sensor_range)
    memory = SpatialMemory(location / "memory.sqlite3")
    events = EventStore(location / "events.sqlite3")
    return AgentRuntime(robot, world, memory, events), world

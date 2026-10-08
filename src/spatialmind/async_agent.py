"""Async metric robot agent: observable goals, active search, revisions and cancellation.

Decision code knows only MetricRobot, SemanticMap and BeliefMemory.
It never directly commands velocity or reads the simulator's hidden world state.
"""
from __future__ import annotations

import asyncio
import time
import uuid
from dataclasses import asdict, dataclass
from typing import Any

from spatialmind.belief import BeliefMemory
from spatialmind.physical import MetricRobot, MotionResult, Pose2D, RoomMap
from spatialmind.telemetry import EventStore


@dataclass(frozen=True)
class Mission:
    kind: str
    target: str
    room: str | None = None
    max_actions: int = 20
    max_failures: int = 6
    timeout_s: float = 240.0

    def __post_init__(self):
        if self.kind not in {"find", "navigate"} or not self.target:
            raise ValueError("Only named find and semantic navigate missions are supported")
        if self.max_actions < 1 or self.max_failures < 1 or self.timeout_s <= 0:
            raise ValueError("Budgets must be positive")


@dataclass(frozen=True)
class MissionOutcome:
    task_id: str
    status: str
    reason: str
    nav_actions: int
    nav_failures: int
    replans: int
    motion_m: float
    elapsed_s: float
    evidence_ref: str | None
    last_pose: Pose2D

    def to_dict(self) -> dict:
        return asdict(self)


class AsyncPhysicalAgent:
    """Single-active-mission task actor; no reentrant concurrent robot missions."""

    def __init__(
        self, robot: MetricRobot, semantic_map: RoomMap,
        memory: BeliefMemory, events: EventStore,
        *, min_detection_confidence: float = 0.65,
    ):
        self.robot, self.map, self.memory, self.events = robot, semantic_map, memory, events
        self.min_detection_confidence = min_detection_confidence
        self._task: asyncio.Task | None = None
        self._commands: asyncio.Queue[tuple[str, Any]] = asyncio.Queue()
        self._active_id: str | None = None

    async def start(self, mission: Mission, *, task_id: str | None = None) -> str:
        if self._task is not None and not self._task.done():
            raise RuntimeError("Only one physical motion task may be active")
        self._commands = asyncio.Queue()
        self._active_id = task_id or uuid.uuid4().hex[:12]
        self._task = asyncio.create_task(self._execute(mission, self._active_id))
        return self._active_id

    async def join(self) -> MissionOutcome:
        if self._task is None:
            raise RuntimeError("No task has been started")
        return await self._task

    async def run(self, mission: Mission) -> MissionOutcome:
        await self.start(mission)
        return await self.join()

    async def revise(self, mission: Mission) -> None:
        if self._task is None or self._task.done():
            raise RuntimeError("No live mission to revise")
        await self._commands.put(("revise", mission))

    async def cancel(self) -> None:
        if self._task is None or self._task.done():
            return
        await self._commands.put(("cancel", None))

    def _checkpoint(self, task_id: str, mission: Mission, *,
                    visited: set[str], actions: int, failures: int, replans: int,
                    motion_m: float, status: str = "running") -> None:
        self.events.save(task_id, {
            "mission": asdict(mission), "visited": sorted(visited),
            "actions": actions, "failures": failures, "replans": replans,
            "motion_m": motion_m, "status": status,
            "map_version": self.map.version,
            "robot_pose": asdict(self.robot.pose),
        })

    async def _execute(self, mission: Mission, task_id: str) -> MissionOutcome:
        started=time.monotonic()
        actions=failures=replans=0
        motion_m=0.0
        visited:set[str]=set()
        evidence_ref=None
        status,reason="failed","unknown"
        self.events.emit(task_id,"mission_created",mission=asdict(mission),
                         map_version=self.map.version)
        self._checkpoint(task_id,mission,visited=visited,actions=actions,
                         failures=failures,replans=replans,motion_m=motion_m)
        try:
            while actions < mission.max_actions and failures < mission.max_failures:
                if time.monotonic()-started > mission.timeout_s:
                    status,reason="interrupted","time_budget_exceeded"
                    break
                # Commands are checked before and during every navigation.
                pending=await self._poll_command(task_id)
                if pending is not None:
                    command,data=pending
                    if command=="cancel":
                        status,reason="cancelled","user_cancelled"
                        await self.robot.stop()
                        break
                    if command=="revise":
                        mission=data
                        visited.clear()
                        replans+=1
                        self.events.emit(task_id,"mission_revised",mission=asdict(mission))
                obs=await self.robot.observe()
                updates=self.memory.update(obs)
                self.events.emit(task_id,"physical_observation",pose=asdict(obs.pose),
                                 detections=[asdict(x) for x in obs.detections],
                                 observed_cells=len(obs.visible_cells),
                                 coverage_quality=obs.coverage_quality,
                                 memory_updates=updates)
                verified=[d for d in obs.detections if mission.kind=="find"
                          and d.label==mission.target
                          and d.confidence>=self.min_detection_confidence
                          and (mission.room is None or self.map.room_for(d.pose)==mission.room)]
                if verified:
                    status,reason="succeeded","sensor_verified"
                    evidence_ref=verified[0].evidence_ref
                    if not evidence_ref:
                        # Explicit sensor event reference; not a claimed source image.
                        evidence_ref=f"obs:{task_id}:{actions}"
                    self.events.emit(task_id,"goal_verified",evidence_ref=evidence_ref)
                    break
                goal,source=self._choose(mission,visited)
                if goal is None:
                    status,reason="failed","exhausted_search_space"
                    break
                goal_key=f"{source}:{goal.x:.3f},{goal.y:.3f}"
                visited.add(goal_key)
                nav_id=uuid.uuid4().hex[:12]
                self.events.emit(task_id,"navigate_requested",goal=asdict(goal),
                                 source=source,nav_goal_id=nav_id)
                nav=asyncio.create_task(self.robot.navigate(goal,timeout_s=min(
                    mission.timeout_s-(time.monotonic()-started),90.0)))
                command_task=asyncio.create_task(self._commands.get())
                done,pending_tasks=await asyncio.wait(
                    {nav,command_task},return_when=asyncio.FIRST_COMPLETED)
                if command_task in done:
                    cmd,new_mission=command_task.result()
                    await self.robot.stop()
                    nav.cancel()
                    try:
                        await nav
                    except asyncio.CancelledError:
                        pass
                    if cmd=="cancel":
                        status,reason="cancelled","user_cancelled"
                        break
                    mission=new_mission
                    visited.clear()
                    replans+=1
                    self.events.emit(task_id,"mission_revised",mission=asdict(mission))
                    self._checkpoint(task_id,mission,visited=visited,actions=actions,
                                     failures=failures,replans=replans,motion_m=motion_m)
                    continue
                command_task.cancel()
                try:
                    await command_task
                except asyncio.CancelledError:
                    pass
                result=nav.result()
                actions+=1
                motion_m+=result.distance_m
                self.events.emit(task_id,"navigate_result",success=result.success,
                                 reason=result.reason,nav_goal_id=nav_id,
                                 distance_m=result.distance_m,duration_s=result.duration_s)
                if not result.success:
                    failures+=1
                    replans+=1
                elif mission.kind=="navigate":
                    status,reason="succeeded","navigation_feedback_confirmed"
                    break
                else:
                    replans+=1
                self._checkpoint(task_id,mission,visited=visited,actions=actions,
                                 failures=failures,replans=replans,motion_m=motion_m)
            else:
                status,reason="interrupted","action_or_failure_budget"
        except asyncio.CancelledError:
            status,reason="cancelled","runtime_cancelled"
            await self.robot.stop()
            raise
        except Exception as exc:
            status,reason="failed",f"runtime_error:{type(exc).__name__}"
            self.events.emit(task_id,"execution_exception",kind=type(exc).__name__)
        self._checkpoint(task_id,mission,visited=visited,actions=actions,
                         failures=failures,replans=replans,motion_m=motion_m,status=status)
        self.events.emit(task_id,"mission_finished",status=status,reason=reason,
                         evidence_ref=evidence_ref,actions=actions,motion_m=motion_m)
        return MissionOutcome(task_id,status,reason,actions,failures,replans,
                              round(motion_m,4),round(time.monotonic()-started,4),
                              evidence_ref,self.robot.pose)

    async def _poll_command(self, task_id: str):
        try:
            return self._commands.get_nowait()
        except asyncio.QueueEmpty:
            return None

    def _choose(self, mission: Mission, visited: set[str]):
        choices=[]
        if mission.kind=="find":
            for memory in self.memory.candidates(mission.target):
                key=f"memory:{memory.pose.x:.3f},{memory.pose.y:.3f}"
                if key not in visited and self.map.reachable(memory.pose):
                    choices.append((100*memory.confidence-memory.pose.distance(self.robot.pose),
                                    memory.pose,"memory"))
        for room,goal in self.map.search_waypoints(mission.room):
            if mission.kind=="navigate" and room!=mission.target:
                continue
            name=f"{room}:{goal.x:.3f},{goal.y:.3f}"
            if name in visited or not self.map.reachable(goal):
                continue
            # Existing semantic priors intentionally kept modest; no ground truth access.
            prior=2 if mission.target in ("blue toolbox","toolbox") and room=="lab" else 0
            if mission.target=="red first aid kit" and room=="storage":
                prior=2
            score=prior*10-goal.distance(self.robot.pose)
            choices.append((score,goal,room))
        if not choices:
            return None,None
        _,pose,reason=max(choices,key=lambda x:(x[0],-x[1].x,-x[1].y))
        return pose,reason

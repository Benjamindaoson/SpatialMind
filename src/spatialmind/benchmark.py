"""Repeatable interventions and ablation experiments (no fabricated accuracy values)."""

from __future__ import annotations

import json
import statistics
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from spatialmind.memory import SpatialMemory
from spatialmind.models import Point, TaskStatus
from spatialmind.planner import ActiveSearchPlanner
from spatialmind.runtime import AgentRuntime
from spatialmind.telemetry import EventStore
from spatialmind.world import GridWorld, SimRobot


@dataclass
class Trial:
    scenario: str
    policy: str
    success: bool
    nav_steps: int
    actions: int
    replans: int
    navigation_failures: int
    evidence: bool

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class NoMemory(SpatialMemory):
    """Ablation: observations are available, but objects cannot be retrieved later."""

    def candidates(self, target: str, *, include_missing: bool = False) -> list:
        return []


class SequentialPlanner(ActiveSearchPlanner):
    """Ablation: fixed room order instead of prior-weighted viewpoint selection."""

    def _priors(self, target: str) -> dict[str, float]:
        return {"lab": 0.25, "storage": 0.25, "office": 0.25, "meeting": 0.25}


def run_trial(scenario: str, policy: str = "full") -> Trial:
    if scenario not in {"find_toolbox", "moved_toolbox", "occluded_toolbox",
                        "blocked_corridor", "find_first_aid"}:
        raise ValueError(f"Unknown scenario: {scenario}")
    if policy not in {"full", "no_memory", "uniform_search"}:
        raise ValueError(f"Unknown policy: {policy}")
    with tempfile.TemporaryDirectory() as temporary:
        world = GridWorld()
        robot = SimRobot(world)
        memory = (NoMemory if policy == "no_memory" else SpatialMemory)(
            Path(temporary) / "memory.db"
        )
        events = EventStore(Path(temporary) / "events.db")
        runtime = AgentRuntime(robot, world, memory, events)
        if policy == "uniform_search":
            runtime.planner = SequentialPlanner(world)
        if scenario == "moved_toolbox":
            # Historical sighting is retained while the object moves.
            memory.update(robot.observe())
            world.move_object("toolbox_1", Point(16, 9))
        elif scenario == "occluded_toolbox":
            memory.update(robot.observe())
            world.move_object("toolbox_1", Point(17, 9))
            world.set_obstacle(Point(16, 9))
        elif scenario == "blocked_corridor":
            world.set_obstacle(Point(9, 3))
        command = (
            "Find the red first aid kit" if scenario == "find_first_aid"
            else "Find the blue toolbox"
        )
        result = runtime.run(command, max_actions=90)
        trial = Trial(
            scenario, policy, result.status is TaskStatus.SUCCEEDED,
            robot.motion_steps, result.steps, result.replans,
            result.navigation_failures, result.evidence_id is not None,
        )
        memory.close()
        events.close()
        return trial


def run_suite(
    scenarios: list[str] | None = None,
    policies: list[str] | None = None,
    output: str | Path | None = None,
) -> dict[str, Any]:
    scenarios = scenarios or [
        "find_toolbox", "moved_toolbox", "occluded_toolbox",
        "blocked_corridor", "find_first_aid",
    ]
    policies = policies or ["full", "no_memory", "uniform_search"]
    trials = [run_trial(s, p) for p in policies for s in scenarios]
    metrics = {}
    for policy in policies:
        values = [t for t in trials if t.policy == policy]
        metrics[policy] = {
            "trials": len(values),
            "success_rate": sum(t.success for t in values) / len(values),
            "mean_navigation_steps": statistics.mean(t.nav_steps for t in values),
            "mean_replans": statistics.mean(t.replans for t in values),
        }
    report: dict[str, Any] = {
        "benchmark": "spatialmind-grid-v0.1",
        "notice": "Deterministic toy-grid integration tests, not Gazebo or real-robot results.",
        "metrics": metrics,
        "trials": [t.as_dict() for t in trials],
    }
    if output is not None:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report

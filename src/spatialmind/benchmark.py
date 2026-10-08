"""Seeded spatial interventions, transparent metrics and replayable event evidence.

All numbers come from executed deterministic grid-world trials.
No statement here implies physical-robot validation or statistical generalization.
"""
from __future__ import annotations

import json
import math
import random
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

SCENARIOS = (
    "find_toolbox", "moved_toolbox", "occluded_toolbox",
    "blocked_corridor", "find_first_aid", "memory_hint_toolbox",
)
POLICIES = ("full", "no_memory", "uniform_search")


@dataclass(frozen=True)
class Trial:
    scenario: str
    policy: str
    success: bool
    nav_steps: int
    actions: int
    replans: int
    navigation_failures: int
    evidence: bool
    seed: int = 0

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class NoMemory(SpatialMemory):
    """Ablation: object sightings are stored but not exposed to the planner."""
    def candidates(self, target: str, *, include_missing: bool = False) -> list:
        return []


class UniformSearchPlanner(ActiveSearchPlanner):
    """Ablation: identical priors for all semantic rooms."""
    def _priors(self, target: str) -> dict[str, float]:
        return {"lab": 0.25, "storage": 0.25, "office": 0.25, "meeting": 0.25}


def _configure_scenario(world: GridWorld, robot: SimRobot,
                        memory: SpatialMemory, scenario: str, seed: int) -> None:
    rng = random.Random(seed)
    if scenario == "find_toolbox":
        if seed:
            room = rng.choice(["lab", "storage", "meeting", "office"])
            world.move_object("toolbox_1", rng.choice(world.room_points(room)))
    elif scenario == "memory_hint_toolbox":
        target = Point(16, 9) if seed == 0 else rng.choice(world.room_points("office"))
        world.move_object("toolbox_1", target)
        # Past remote observation is generated through the SAME sensor contract.
        memory.update(SimRobot(world, pose=target).observe())
    elif scenario == "moved_toolbox":
        memory.update(robot.observe())
        destination = Point(16, 9) if seed == 0 else rng.choice(world.room_points("office"))
        world.move_object("toolbox_1", destination)
    elif scenario == "occluded_toolbox":
        memory.update(robot.observe())
        world.move_object("toolbox_1", Point(17, 9))
        world.set_obstacle(Point(16, 9))
    elif scenario == "blocked_corridor":
        world.set_obstacle(Point(9, 3))
    elif scenario == "find_first_aid" and seed:
        room = rng.choice(["storage", "lab", "office"])
        world.move_object("firstaid_1", rng.choice(world.room_points(room)))


def run_trial(
    scenario: str, policy: str = "full", *, seed: int = 0,
    trace_dir: str | Path | None = None,
) -> Trial:
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {scenario}")
    if policy not in POLICIES:
        raise ValueError(f"Unknown policy: {policy}")
    if seed < 0:
        raise ValueError("seed must be nonnegative")
    with tempfile.TemporaryDirectory() as temporary:
        world = GridWorld()
        robot = SimRobot(world)
        memory = (NoMemory if policy == "no_memory" else SpatialMemory)(
            Path(temporary) / "memory.db")
        events = EventStore(Path(temporary) / "events.db")
        runtime = AgentRuntime(robot, world, memory, events)
        if policy == "uniform_search":
            runtime.planner = UniformSearchPlanner(world)
        _configure_scenario(world, robot, memory, scenario, seed)
        instruction = (
            "Go to the meeting room" if scenario == "blocked_corridor"
            else "Find the red first aid kit" if scenario == "find_first_aid"
            else "Find the blue toolbox"
        )
        result = runtime.run(instruction, max_actions=120)
        trial = Trial(
            scenario=scenario, policy=policy,
            success=result.status is TaskStatus.SUCCEEDED,
            nav_steps=robot.motion_steps, actions=result.steps,
            replans=result.replans, navigation_failures=result.navigation_failures,
            evidence=result.evidence_id is not None, seed=seed,
        )
        if trace_dir is not None:
            root = Path(trace_dir)
            root.mkdir(parents=True, exist_ok=True)
            path = root / f"{scenario}__{policy}__seed{seed}.json"
            path.write_text(json.dumps({
                "trial": trial.as_dict(), "result": result.to_dict(),
                "event_stream": events.events(result.task_id),
            }, indent=2), encoding="utf-8")
        events.close()
        memory.close()
        return trial


def wilson_interval(successes: int, total: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score CI for binomial success; interpret cautiously for tiny n."""
    if total <= 0 or not 0 <= successes <= total:
        raise ValueError("invalid success count")
    p = successes / total
    denom = 1 + z*z/total
    center = (p + z*z/(2*total)) / denom
    half = z * math.sqrt((p*(1-p) + z*z/(4*total))/total) / denom
    return max(0.0, center-half), min(1.0, center+half)


def percentile_nearest_rank(values: list[int], percentile: float) -> int:
    if not values or not 0 < percentile <= 1:
        raise ValueError("invalid percentile or empty measurements")
    sorted_values = sorted(values)
    return sorted_values[math.ceil(percentile * len(sorted_values)) - 1]


def run_suite(
    scenarios: list[str] | None = None,
    policies: list[str] | None = None,
    output: str | Path | None = None,
    *, seeds: int = 1, trace_dir: str | Path | None = None,
) -> dict[str, Any]:
    if seeds < 1:
        raise ValueError("seeds must be >= 1")
    scenarios = list(SCENARIOS) if scenarios is None else scenarios
    policies = list(POLICIES) if policies is None else policies
    if not scenarios or not policies:
        raise ValueError("at least one scenario and policy are required")
    trials = [
        run_trial(s, p, seed=seed, trace_dir=trace_dir)
        for seed in range(seeds) for p in policies for s in scenarios
    ]
    metrics: dict[str, Any] = {}
    for policy in policies:
        values = [t for t in trials if t.policy == policy]
        successes = sum(t.success for t in values)
        interval = wilson_interval(successes, len(values))
        metrics[policy] = {
            "trials": len(values),
            "success_count": successes,
            "success_rate": successes / len(values),
            "success_ci95_wilson": [round(x, 4) for x in interval],
            "mean_navigation_steps": statistics.mean(t.nav_steps for t in values),
            "p95_navigation_steps": percentile_nearest_rank([t.nav_steps for t in values], .95),
            "mean_replans": statistics.mean(t.replans for t in values),
            "mean_navigation_failures": statistics.mean(t.navigation_failures for t in values),
        }
    report: dict[str, Any] = {
        "benchmark": "spatialmind-grid-v0.2",
        "notice": "Seeded toy-grid integration experiments, not Gazebo or real-robot results.",
        "scenario_count": len(scenarios),
        "seed_count": seeds,
        "metrics": metrics,
        "trials": [trial.as_dict() for trial in trials],
    }
    if output is not None:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report

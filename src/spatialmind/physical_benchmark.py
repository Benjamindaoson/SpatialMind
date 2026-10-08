"""Paired metric-robot interventions across multiple layout and observation seeds.

Differentiates real simulator oracle from Agent-observable detections.
Never passes hidden ground truth to the decision policy.
"""
from __future__ import annotations

import asyncio
import csv
import json
import math
import random
import statistics
import tempfile
from dataclasses import asdict,dataclass
from pathlib import Path

from spatialmind.async_agent import AsyncPhysicalAgent,Mission
from spatialmind.belief import BeliefMemory
from spatialmind.metric_sim import MetricSimRobot,demo_room_map
from spatialmind.physical import MetricObservation,ObjectEstimate,Pose2D
from spatialmind.telemetry import EventStore
from spatialmind.benchmark import wilson_interval,percentile_nearest_rank


SCENARIOS=("fresh","remembered","moved","occluded","blocked","duplicates","missed_detections")
POLICIES=("full","no_memory","static_memory","nearest_search")


class NoMemory(BeliefMemory):
    def candidates(self,label,*,now=None,include_missing=False):
        return []


class StaticMemory(BeliefMemory):
    """Frozen historical object knowledge; incoming observations do not change beliefs."""
    frozen=False
    def update(self,obs):
        if self.frozen:
            return []
        return super().update(obs)


class NearestSearchAgent(AsyncPhysicalAgent):
    """Counterfactual: ignore semantic target-room prior for unknown locations."""
    def _choose(self,mission,visited):
        if mission.kind=="navigate":
            return super()._choose(mission,visited)
        remembered=self.memory.candidates(mission.target)
        for b in remembered:
            key=f"memory:{b.pose.x:.3f},{b.pose.y:.3f}"
            if key not in visited:
                return b.pose,"memory"
        choices=[(goal.distance(self.robot.pose),room,goal)
                 for room,goal in self.map.search_waypoints(mission.room)
                 if f"{room}:{goal.x:.3f},{goal.y:.3f}" not in visited]
        if not choices:
            return None,None
        _,room,goal=min(choices,key=lambda item:(item[0],item[1]))
        return goal,room


class NoisyMetricRobot(MetricSimRobot):
    def __init__(self,*,seed:int,layout_id:int,drop_probability:float=0,
                 false_positive_probability:float=0,**kwargs):
        super().__init__(**kwargs)
        self.rng=random.Random(seed)
        self.drop_probability=drop_probability
        self.false_positive_probability=false_positive_probability
        self.layout_id=layout_id
        if layout_id not in (0,1,2,3):
            raise ValueError("layout_id must be 0..3")
        if layout_id>=1:
            # Different static topology; additional horizontal partition has
            # two known door gaps. No target truth exposed to planner.
            gaps=[(4,15),(3,16),(5,14)][layout_id-1]
            for x in range(1,19):
                if x not in gaps and x != 9:
                    self.walls.add((x,6))
        if layout_id==3:
            # One doorway moves, the robot's BFS sees only known static walls.
            self.walls.discard((9,3))
            self.walls.add((9,4))

    async def observe(self):
        original=await super().observe()
        detections=[d for d in original.detections
                    if self.rng.random()>=self.drop_probability]
        if self.rng.random()<self.false_positive_probability:
            detections.append(ObjectEstimate(
                "blue toolbox",Pose2D(self.pose.x+0.3,self.pose.y+0.2,
                                      stamp=original.pose.stamp),
                .8,track_id="false-positive",
                evidence_ref="sim:false-positive"))
        return MetricObservation(
            original.pose,tuple(detections),original.visible_cells,
            original.coverage_quality,original.source)


@dataclass(frozen=True)
class Trial:
    scenario:str
    policy:str
    seed:int
    layout_id:int
    success:bool
    claimed_success:bool
    false_positive_completion:bool
    navigated_m:float
    nav_actions:int
    nav_failures:int
    replans:int
    elapsed_s:float
    evidence_ref:str|None

    def row(self):
        return asdict(self)


async def _trial(scenario,policy,seed,trace_dir=None):
    rng=random.Random(seed+3000)
    layout=seed%4
    robot=NoisyMetricRobot(seed=seed,layout_id=layout,
                           perception_range_m=2.5,
                           drop_probability=.2 if scenario=="missed_detections" else 0,
                           false_positive_probability=.035 if scenario=="missed_detections" else 0)
    obj_pos=Pose2D(14+rng.randrange(4),8+rng.randrange(2))
    if scenario in ("remembered","moved","missed_detections","duplicates","occluded"):
        robot.place("toolbox_a",obj_pos)
    if scenario=="duplicates":
        robot.objects["toolbox_b"]=("blue toolbox",Pose2D(4,9))
    if scenario=="occluded":
        robot.block((13,8))
    if scenario=="blocked":
        robot.block((9,3))
    with tempfile.TemporaryDirectory() as folder:
        mem_cls={"full":BeliefMemory,"no_memory":NoMemory,
                 "static_memory":StaticMemory,"nearest_search":BeliefMemory}[policy]
        memory=mem_cls(str(Path(folder)/"beliefs.sqlite"))
        events=EventStore(str(Path(folder)/"events.sqlite"))
        try:
            # One historical observation available to all policies.
            if scenario in ("remembered","moved","missed_detections"):
                historical=MetricSimRobot(pose=Pose2D(16,9),perception_range_m=4)
                historical.objects=robot.objects.copy()
                historical.walls=robot.walls.copy()
                if scenario=="moved":
                    historical.objects["toolbox_a"]=("blue toolbox",Pose2D(3,3))
                memory.update(await historical.observe())
                if isinstance(memory,StaticMemory):
                    memory.frozen=True
            if scenario=="moved":
                robot.place("toolbox_a",obj_pos)
            mission=(
                Mission("navigate","meeting","meeting",max_actions=15)
                if scenario=="blocked" else Mission("find","blue toolbox",max_actions=24)
            )
            actor_cls=NearestSearchAgent if policy=="nearest_search" else AsyncPhysicalAgent
            actor=actor_cls(robot,demo_room_map(),memory,events)
            outcome=await actor.run(mission)
            claimed=outcome.status=="succeeded"
            # Ground truth is consulted only AFTER the policy has completed.
            if mission.kind=="find":
                correct=claimed and outcome.evidence_ref is not None and any(
                    outcome.evidence_ref.startswith(f"sim:{identifier}:")
                    for identifier,(label,_) in robot.objects.items() if label==mission.target)
            else:
                correct=claimed and robot.pose.distance(
                    min(demo_room_map().search_waypoints("meeting"),
                        key=lambda pair:pair[1].distance(robot.pose))[1])<=.75
            trial=Trial(scenario,policy,seed,layout,correct,claimed,
                        claimed and not correct,outcome.motion_m,outcome.nav_actions,
                        outcome.nav_failures,outcome.replans,outcome.elapsed_s,
                        outcome.evidence_ref)
            if trace_dir:
                path=Path(trace_dir)
                path.mkdir(parents=True,exist_ok=True)
                (path/f"{scenario}_{policy}_{seed}.json").write_text(json.dumps({
                    "trial":trial.row(),"mission":asdict(mission),
                    "result":outcome.to_dict(),
                    "events":events.events(outcome.task_id),
                },indent=2),encoding="utf-8")
            return trial
        finally:
            events.close()
            memory.close()


def run_physical_benchmark(
    *,seeds:int=3,output_dir:str="artifacts/metric-benchmark",
    trace:bool=False,
):
    if seeds<1:
        raise ValueError("seeds must be positive")
    out=Path(output_dir)
    out.mkdir(parents=True,exist_ok=True)
    trace_dir=out/"traces" if trace else None

    async def execute():
        return [
            await _trial(scenario,policy,seed,trace_dir)
            for seed in range(seeds)
            for scenario in SCENARIOS
            for policy in POLICIES
        ]
    trials=asyncio.run(execute())
    data=[t.row() for t in trials]
    with (out/"trials.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=list(data[0]))
        writer.writeheader()
        writer.writerows(data)
    groups={}
    for policy in POLICIES:
        rows=[t for t in trials if t.policy==policy]
        succeeded=sum(t.success for t in rows)
        groups[policy]={
            "n":len(rows),
            "success_count":succeeded,
            "success_rate":succeeded/len(rows),
            "success_ci95_wilson":[round(x,4) for x in wilson_interval(succeeded,len(rows))],
            "mean_distance_m":round(statistics.mean(t.navigated_m for t in rows),3),
            "p95_distance_m":percentile_nearest_rank(
                [math.ceil(t.navigated_m) for t in rows],.95),
            "mean_nav_failures":round(statistics.mean(t.nav_failures for t in rows),3),
            "mean_replans":round(statistics.mean(t.replans for t in rows),3),
            "p50_latency_s":round(statistics.median(t.elapsed_s for t in rows),3),
            "p95_latency_s":round(sorted(t.elapsed_s for t in rows)[
                math.ceil(.95*len(rows))-1],3),
            "false_positive_completions":sum(t.false_positive_completion for t in rows),
        }
    paired={}
    for scenario in SCENARIOS:
        rows=[t for t in trials if t.scenario==scenario]
        paired[scenario]={policy:{
            "n":sum(t.policy==policy for t in rows),
            "success_rate":sum(t.success for t in rows if t.policy==policy)/seeds,
            "mean_distance_m":round(statistics.mean(t.navigated_m for t in rows
                                                    if t.policy==policy),3),
        } for policy in POLICIES}
    report={
        "benchmark":"spatialmind-metric-intervention-v1",
        "notice":"Metric-grid sensor simulation, NOT Gazebo nor physical hardware.",
        "seeds":seeds,"unique_topologies":min(seeds,4),
        "scenarios":list(SCENARIOS),"policies":list(POLICIES),
        "trial_count":len(trials),"aggregate":groups,"by_scenario":paired,
    }
    (out/"metrics.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    _report_svg(groups,out/"comparison.svg")
    return report


def _report_svg(metrics,filename):
    width,height=850,400
    labels=list(metrics)
    maximum=max(m["mean_distance_m"] for m in metrics.values()) or 1
    svg=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
         '<rect width="850" height="400" rx="18" fill="#101d2d"/>',
         '<text x="40" y="54" font-family="Arial" font-size="26" fill="#f2f8ff">SpatialMind / Paired policy distance</text>',
         '<text x="40" y="80" font-family="Arial" font-size="14" fill="#9db4c9">Mean simulated navigation metres; lower is better. Not real robot data.</text>']
    for i,key in enumerate(labels):
        val=metrics[key]["mean_distance_m"]
        y=116+i*65
        bar=max(0,int(430*val/maximum))
        svg.append(f'<text x="40" y="{y+21}" fill="#dce9f8" font-size="17" font-family="Arial">{key}</text>')
        svg.append(f'<rect x="230" y="{y}" width="{bar}" height="32" rx="5" fill="#61dfbc"/>')
        svg.append(f'<text x="{248+bar}" y="{y+23}" fill="#fff" font-size="16" font-family="Arial">{val:.2f} m</text>')
    svg.append('</svg>')
    filename.write_text("\n".join(svg),encoding="utf-8")

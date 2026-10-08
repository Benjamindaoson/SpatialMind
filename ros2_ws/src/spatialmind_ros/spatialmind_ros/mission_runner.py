"""Run a real Nav2 mission with measured RGB-D observation feedback."""
from __future__ import annotations
import argparse
import asyncio
import json
from pathlib import Path
from dataclasses import asdict
from datetime import datetime, timezone
import os

from spatialmind.async_agent import AsyncPhysicalAgent, Mission
from spatialmind.belief import BeliefMemory
from spatialmind.physical import RoomMap
from spatialmind.telemetry import EventStore
from spatialmind_ros.nav2_robot import Nav2MetricRobot


async def execute(args):
    semantic_map=RoomMap.from_json(args.semantic_map)
    robot=Nav2MetricRobot()
    data=Path(args.data_dir)
    if args.experiment_id:
        data=data/args.experiment_id/args.policy
    data.mkdir(parents=True,exist_ok=True)
    from spatialmind.physical_benchmark import NoMemory,StaticMemory,NearestSearchAgent
    memory_type={"full":BeliefMemory,"no_memory":NoMemory,
                 "static_memory":StaticMemory,"nearest_search":BeliefMemory}[args.policy]
    memory=memory_type(str(data/"physical-memory.sqlite"))
    if isinstance(memory,StaticMemory):
        memory.frozen=True
    events=EventStore(str(data/"physical-events.sqlite"))
    actor=NearestSearchAgent if args.policy=="nearest_search" else AsyncPhysicalAgent
    agent=actor(robot,semantic_map,memory,events)
    try:
        outcome=await agent.run(Mission(args.kind,args.target,args.room,
                           max_actions=args.max_actions,timeout_s=args.timeout))
        report={"result":outcome.to_dict(),"policy":args.policy,
                "experiment_id":args.experiment_id,"scenario":args.scenario,
                "seed":args.seed,"world_id":args.world_id,
                "semantic_map_version":semantic_map.version,
                "git_sha":os.getenv("GITHUB_SHA","unknown"),
                "generated_utc":datetime.now(timezone.utc).isoformat(),
                "caveat":"Sensor-verified agent claim; external oracle must separately grade true success."}
        (data/"result.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
        print(json.dumps(report,indent=2))
        if outcome.status!="succeeded":
            raise SystemExit(2)
    finally:
        await robot.stop()
        robot.close()
        events.close()
        memory.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--semantic-map",required=True)
    parser.add_argument("--kind",choices=["find","navigate"],required=True)
    parser.add_argument("--target",required=True)
    parser.add_argument("--room")
    parser.add_argument("--data-dir",default=".spatialmind/physical")
    parser.add_argument("--policy",default="full",choices=(
        "full","no_memory","static_memory","nearest_search"))
    parser.add_argument("--experiment-id")
    parser.add_argument("--scenario",default="unspecified")
    parser.add_argument("--seed",type=int,default=0)
    parser.add_argument("--world-id",default="spatialmind")
    parser.add_argument("--max-actions",type=int,default=15)
    parser.add_argument("--timeout",type=float,default=240)
    asyncio.run(execute(parser.parse_args()))

if __name__=="__main__":
    main()

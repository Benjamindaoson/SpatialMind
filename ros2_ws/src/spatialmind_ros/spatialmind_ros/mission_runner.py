"""Run a real Nav2 mission with measured RGB-D observation feedback."""
from __future__ import annotations
import argparse
import asyncio
import json
from pathlib import Path

from spatialmind.async_agent import AsyncPhysicalAgent, Mission
from spatialmind.belief import BeliefMemory
from spatialmind.physical import RoomMap
from spatialmind.telemetry import EventStore
from spatialmind_ros.nav2_robot import Nav2MetricRobot


async def execute(args):
    semantic_map=RoomMap.from_json(args.semantic_map)
    robot=Nav2MetricRobot()
    data=Path(args.data_dir)
    data.mkdir(parents=True,exist_ok=True)
    memory=BeliefMemory(str(data/"physical-memory.sqlite"))
    events=EventStore(str(data/"physical-events.sqlite"))
    agent=AsyncPhysicalAgent(robot,semantic_map,memory,events)
    try:
        outcome=await agent.run(Mission(args.kind,args.target,args.room,
                           max_actions=args.max_actions,timeout_s=args.timeout))
        print(json.dumps(outcome.to_dict(),indent=2))
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
    parser.add_argument("--max-actions",type=int,default=15)
    parser.add_argument("--timeout",type=float,default=240)
    asyncio.run(execute(parser.parse_args()))

if __name__=="__main__":
    main()

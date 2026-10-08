#!/usr/bin/env python3
"""Apply a controlled target displacement in *Gazebo simulation only*.

Do not confuse service acknowledgement with robot visual confirmation.
An explicit --apply flag is required to alter the running simulator.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess

from spatialmind.physical import RoomMap

TARGETS={
    "blue_toolbox":0.18,
    "red_first_aid":0.20,
}


def build_request(object_id:str,room:str,semantic_map:RoomMap)->str:
    if object_id not in TARGETS:
        raise ValueError("Unsupported experiment object")
    positions=semantic_map.search_waypoints(room)
    if not positions:
        raise ValueError("Unknown room or no calibrated viewpoint")
    x,y=positions[0][1].x,positions[0][1].y
    return (f'name: "{object_id}" position {{ x: {x} y: {y} '
            f'z: {TARGETS[object_id]} }}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--semantic-map",default="simulation/generated/semantic_map.json")
    parser.add_argument("--world",default="spatialmind")
    parser.add_argument("--object",choices=sorted(TARGETS),default="blue_toolbox")
    parser.add_argument("--room",required=True,choices=[
        "lab","storage","meeting","office"])
    parser.add_argument("--apply",action="store_true",help="Actually call Gazebo set_pose")
    args=parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_-]+",args.world):
        raise ValueError("Unsafe Gazebo world name")
    semantic=RoomMap.from_json(args.semantic_map)
    request=build_request(args.object,args.room,semantic)
    command=[
        "gz","service","-s",f"/world/{args.world}/set_pose",
        "--reqtype","gz.msgs.Pose","--reptype","gz.msgs.Boolean",
        "--timeout","5000","--req",request,
    ]
    if not args.apply:
        print(json.dumps({"status":"dry_run","request":request,
                          "service":command[3]},indent=2))
        return
    output=subprocess.run(command,check=False,text=True,capture_output=True,timeout=10)
    print(json.dumps({
        "status":"service_responded" if output.returncode==0 else "service_failed",
        "returncode":output.returncode,
        "response":output.stdout.strip(),
        "error":output.stderr.strip(),
        "verification_required":"Confirm new position via robot RGB-D and observation trace.",
    },indent=2))
    if output.returncode!=0:
        raise SystemExit(output.returncode)


if __name__=="__main__":
    main()

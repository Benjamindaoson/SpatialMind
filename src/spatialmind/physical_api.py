"""Physical Agent mission API using the metric sensor simulator.

Lifespan creates SQLite connections in the same asyncio thread as agent
operations. It intentionally does not expose arbitrary motor commands.
"""
from __future__ import annotations

import os
from contextlib import asynccontextmanager
from dataclasses import asdict
from importlib.resources import files
from pathlib import Path
from uuid import uuid4

from spatialmind.async_agent import AsyncPhysicalAgent,Mission
from spatialmind.belief import BeliefMemory
from spatialmind.metric_sim import MetricSimRobot,demo_room_map
from spatialmind.physical import Pose2D
from spatialmind.telemetry import EventStore

try:
    from fastapi import FastAPI,HTTPException
    from fastapi.responses import HTMLResponse
    from pydantic import BaseModel,Field
except ImportError as exc:
    raise RuntimeError('Install API dependencies: pip install -e ".[api]"') from exc


class MissionInput(BaseModel):
    kind:str="find"
    target:str="blue toolbox"
    room:str|None=None
    max_actions:int=Field(20,ge=1,le=100)


class MoveObjectInput(BaseModel):
    object_id:str="toolbox_a"
    room:str="office"


@asynccontextmanager
async def lifespan(app):
    data=Path(os.getenv("SPATIALMIND_DATA_DIR",".spatialmind"))/"physical"
    data.mkdir(parents=True,exist_ok=True)
    robot=MetricSimRobot(latency_s=.015)
    memory=BeliefMemory(str(data/"beliefs.sqlite"))
    events=EventStore(str(data/"missions.sqlite"))
    actor=AsyncPhysicalAgent(robot,demo_room_map(),memory,events)
    app.state.robot=robot
    app.state.map=actor.map
    app.state.memory=memory
    app.state.events=events
    app.state.actor=actor
    try:
        yield
    finally:
        await actor.cancel()
        if actor._task is not None:
            try:
                await actor.join()
            except Exception:
                pass
        events.close()
        memory.close()


app=FastAPI(title="SpatialMind Physical Agent Control",version="0.2.0",
            lifespan=lifespan)


@app.get("/",response_class=HTMLResponse)
async def index():
    return files("spatialmind").joinpath("static/physical.html").read_text(encoding="utf-8")


@app.get("/healthz")
async def health():
    return {"status":"ok","backend":"metric-sensor-simulation"}


@app.get("/api/physical/state")
async def state():
    from spatialmind.belief import BeliefMemory
    actor=app.state.actor
    robot=app.state.robot
    task=actor._task
    task_id=actor._active_id
    outcome=task.result().to_dict() if task is not None and task.done() else None
    remembered=[asdict(x) for label in ("blue toolbox","red first aid kit")
                for x in app.state.memory.candidates(label,include_missing=True)]
    return {
        "backend":"metric-sensor-simulation",
        "task_id":task_id,
        "task_status":("running" if task is not None and not task.done()
                       else outcome["status"] if outcome else "idle"),
        "result":outcome,
        "robot_pose":asdict(robot.pose),
        "semantic_waypoints":[{"room":room,"pose":asdict(p)}
                              for room,p in app.state.map.search_waypoints()],
        "memory":remembered,
        "recent_events":app.state.events.events(task_id)[-35:] if task_id else [],
        "sim_obstacles":sorted(list(robot.obstacles)),
    }


@app.post("/api/physical/tasks")
async def start_mission(body:MissionInput):
    try:
        mission=Mission(body.kind,body.target,body.room,max_actions=body.max_actions)
        task_id=await app.state.actor.start(mission)
        return {"task_id":task_id,"status":"running"}
    except (RuntimeError,ValueError) as exc:
        raise HTTPException(409,str(exc)) from exc


@app.post("/api/physical/tasks/{task_id}/revise")
async def revise(task_id:str,body:MissionInput):
    if app.state.actor._active_id!=task_id:
        raise HTTPException(404,"Unknown active mission")
    try:
        mission=Mission(body.kind,body.target,body.room,max_actions=body.max_actions)
        await app.state.actor.revise(mission)
    except (RuntimeError,ValueError) as exc:
        raise HTTPException(409,str(exc)) from exc
    return {"task_id":task_id,"status":"revision_requested"}


@app.post("/api/physical/tasks/{task_id}/cancel")
async def cancel(task_id:str):
    if app.state.actor._active_id!=task_id:
        raise HTTPException(404,"Unknown mission")
    await app.state.actor.cancel()
    return {"task_id":task_id,"status":"cancel_requested"}


@app.post("/api/physical/intervene/move")
async def move(body:MoveObjectInput):
    actor=app.state.actor
    if actor._task is not None and not actor._task.done():
        raise HTTPException(409,"World mutation is disabled during live robot motion")
    if body.object_id not in app.state.robot.objects:
        raise HTTPException(404,"Unknown object")
    candidates=app.state.map.search_waypoints(body.room)
    if not candidates:
        raise HTTPException(422,"Unknown semantic room")
    pose=candidates[0][1]
    app.state.robot.place(body.object_id,pose)
    return {"object_id":body.object_id,"room":body.room,
            "position":asdict(pose)}

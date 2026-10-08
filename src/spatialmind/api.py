"""Optional FastAPI dashboard for the deterministic grid simulation."""
from __future__ import annotations
import os
from dataclasses import asdict
from importlib.resources import files
from pathlib import Path
from spatialmind.runtime import open_runtime
try:
    from fastapi import FastAPI, HTTPException
    from fastapi.responses import HTMLResponse
    from pydantic import BaseModel, Field
except ImportError as exc:
    raise RuntimeError('Install the optional API dependencies: pip install -e ".[api]"') from exc

class TaskBody(BaseModel):
    instruction: str = Field(min_length=1, max_length=500)
    max_actions: int = Field(default=80, ge=1, le=500)

class ChatBody(BaseModel):
    message: str = Field(min_length=1, max_length=500)
    session_id: str | None = None
    max_actions: int = Field(default=80, ge=1, le=500)

class ObstacleBody(BaseModel):
    x: int
    y: int
    blocked: bool = True

app = FastAPI(title="SpatialMind Mission Control", version="0.1.0")
interpreter = None
if os.getenv("SPATIALMIND_LLM_MODEL"):
    from spatialmind.llm import OpenAICompatibleInterpreter
    interpreter = OpenAICompatibleInterpreter(
        model=os.environ["SPATIALMIND_LLM_MODEL"],
        base_url=os.getenv("SPATIALMIND_LLM_BASE_URL", "https://api.openai.com/v1"),
    )
runtime, world = open_runtime(
    data_dir=Path(os.getenv("SPATIALMIND_DATA_DIR", ".spatialmind")),
    interpreter=interpreter,
)
last_task_id: str | None = None
sessions: dict[str, object] = {}

@app.get("/", response_class=HTMLResponse)
async def index() -> str:
    return files("spatialmind").joinpath("static/index.html").read_text(encoding="utf-8")

@app.get("/api/state")
async def state() -> dict:
    records = runtime.memory.candidates("", include_missing=True)
    return {
        "grid": world.render(runtime.robot.pose).splitlines(),
        "pose": asdict(runtime.robot.pose),
        "memory": [
            {"id": item.object_id, "label": item.label,
             "position": asdict(item.position), "status": item.status,
             "confidence": item.confidence, "sightings": item.sightings}
            for item in records
        ],
        "last_task_id": last_task_id,
        "events": runtime.events.events(last_task_id)[-30:] if last_task_id else [],
        "backend": "grid-simulation",
    }

@app.post("/api/tasks")
async def create_task(body: TaskBody) -> dict:
    global last_task_id
    try:
        result = runtime.run(body.instruction, max_actions=body.max_actions)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    last_task_id = result.task_id
    return result.to_dict()

@app.post("/api/chat")
async def chat(body: ChatBody) -> dict:
    global last_task_id
    from spatialmind.dialogue import DialogueSession
    if body.session_id and body.session_id in sessions:
        session = sessions[body.session_id]
    else:
        session = DialogueSession(runtime)
        sessions[session.session_id] = session
    answer = session.say(body.message, max_actions=body.max_actions)
    if answer.result:
        last_task_id = answer.result.task_id
    return answer.to_dict()


@app.post("/api/world/move/{object_id}/{room}")
async def move_object(object_id: str, room: str) -> dict:
    if object_id not in world.objects:
        raise HTTPException(404, detail="Unknown object id")
    try:
        world.move_object(object_id, world.room_center(room))
    except ValueError as exc:
        raise HTTPException(422, detail=str(exc)) from exc
    return {"object_id": object_id, "room": room,
            "position": asdict(world.objects[object_id].position)}

@app.post("/api/world/obstacle")
async def set_obstacle(body: ObstacleBody) -> dict:
    from spatialmind.models import Point
    try:
        world.set_obstacle(Point(body.x, body.y), body.blocked)
    except ValueError as exc:
        raise HTTPException(422, detail=str(exc)) from exc
    return {"position": {"x":body.x, "y":body.y}, "blocked":body.blocked}

@app.get("/healthz")
async def healthz() -> dict:
    return {"status": "ok", "backend": "grid-simulation"}

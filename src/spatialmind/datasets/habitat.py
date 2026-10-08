"""Habitat assets and navigation episodes indexing.

Do not pass simulator goal positions or ground truth to an online policy.
Scenes and episodes remain native Habitat assets, never silently converted
to ROS/Nav2 trajectories.
"""
from __future__ import annotations

import gzip
import json
from pathlib import Path
from typing import Iterator, Any

from .core import DatasetRecord


def _read_json(path: Path) -> dict:
    loader = gzip.open if path.name.endswith(".gz") else open
    with loader(path, "rt", encoding="utf-8") as f:
        return json.load(f)


def _safe_goal_hint(obj: Any) -> dict:
    """Allow only goal identity/modalities, never positions or oracle viewpoints."""
    if isinstance(obj, str):
        return {"description": obj[:250]}
    if not isinstance(obj, dict):
        return {"modality": "unspecified"}
    allowed = ("object_category", "goal_type", "goal_category", "category",
               "language_description", "description", "instruction", "modality")
    return {k: str(obj[k])[:250] for k in allowed if isinstance(obj.get(k), (str, int))}


def _episode_records(name: str, path: Path, root: Path, remaining: int) -> Iterator[DatasetRecord]:
    doc = _read_json(path)
    episodes = doc.get("episodes", [])
    if not isinstance(episodes, list):
        raise ValueError(f"Invalid episodes list: {path}")
    split = next((p for p in ("train", "val_seen", "val_unseen", "val", "minival", "test")
                  if p in path.parts or path.name.startswith(p + ".")), "unspecified")
    for ep in episodes[:remaining]:
        scene = str(ep.get("scene_id", "unknown_scene"))
        episode_id = str(ep.get("episode_id", "unknown_episode"))
        raw_goals = ep.get("tasks", ep.get("task_sequence", ep.get("goals", [])))
        if not isinstance(raw_goals, list):
            raw_goals = []
        modalities = [_safe_goal_hint(goal) for goal in raw_goals]
        if name == "hm3d":
            modalities = [_safe_goal_hint(ep)]
        if name == "goat" and not modalities:
            # A GOAT episode missing subtasks is not a valid sequential-goal task.
            raise ValueError(f"GOAT episode {episode_id} has no tasks/goals")
        uid = f"{path.relative_to(root).as_posix()}:{episode_id}"
        yield DatasetRecord(
            name, "navigation_episode", uid, scene, split,
            data={
                "episode_id": episode_id,
                "scene_id": scene,
                "start_position": ep.get("start_position"),
                "start_rotation": ep.get("start_rotation"),
                "goal_hints": modalities,
                "subtask_count": len(raw_goals) if name == "goat" else 1,
                "source_file": path.relative_to(root).as_posix(),
                "execution_engine": "habitat_required",
                "no_goal_truth_in_policy_record": True,
            },
        )
        # Oracle is partitioned to a separate file. Even though raw episodes
        # contain goal metadata, consumers should not read oracle.jsonl.
        yield DatasetRecord(
            name, "oracle_navigation", uid, scene, split,
            data={
                "source_file": path.relative_to(root).as_posix(),
                "episode_id": episode_id,
                "goals": ep.get("goals"),
                "tasks": ep.get("tasks", ep.get("task_sequence")),
                "goals_by_category": doc.get("goals_by_category", {}),
                "info": ep.get("info", {}),
            },
        )


def habitat_records(name: str, root: Path, *, limit: int = 1000) -> Iterator[DatasetRecord]:
    if name not in {"replicacad", "hm3d", "goat"}:
        raise ValueError(name)
    emitted = 0
    if name == "replicacad":
        scenes = sorted(root.rglob("*.scene_instance.json"))
        if not scenes:
            scenes = sorted(root.rglob("*.scene_dataset_config.json"))
        for file in scenes:
            if emitted >= limit:
                break
            doc = _read_json(file)
            yield DatasetRecord(
                name, "scene_asset", file.relative_to(root).as_posix(),
                file.stem, data={
                    "source_file": file.relative_to(root).as_posix(),
                    "stage_instance_count": len(doc.get("object_instances", [])),
                    "articulated_instance_count": len(doc.get("articulated_object_instances", [])),
                    "scene_description": "ReplicaCAD native Habitat scene metadata",
                    "requires_habitat_sim": True,
                },
            )
            emitted += 1
        return

    # Native HM3D scenes: index GLB assets without reading mesh contents.
    if name == "hm3d":
        for file in sorted(root.rglob("*.glb")):
            if emitted >= limit:
                break
            yield DatasetRecord(
                name, "scene_asset", file.relative_to(root).as_posix(), file.stem,
                data={"source_file": file.relative_to(root).as_posix(),
                      "size_bytes": file.stat().st_size,
                      "requires_habitat_sim": True},
            )
            emitted += 1
    remaining = limit - emitted
    if remaining <= 0:
        return
    for file in sorted(root.rglob("*.json.gz")) + sorted(root.rglob("*.json")):
        if remaining <= 0:
            break
        # Scene config and metadata aren't episodes; skip those.
        if "scene_" in file.name or "config" in file.name:
            continue
        try:
            doc = _read_json(file)
        except (OSError, json.JSONDecodeError):
            continue
        if not isinstance(doc, dict) or not isinstance(doc.get("episodes"), list):
            continue
        for record in _episode_records(name, file, root, remaining):
            yield record
            if not record.kind.startswith("oracle_"):
                emitted += 1
                remaining -= 1
                if remaining <= 0:
                    # Oracle from paired yielding order still emitted? Caller
                    # consumes sequence until next item, so avoid break here.
                    pass
        if remaining <= 0:
            break

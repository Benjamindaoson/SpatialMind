"""Offline RGB-D frame replay into SpatialMind's measured-vision primitives.

Actual image/depth bytes are decoded. TUM ground truth represents the CAMERA,
not a robot's autonomous navigation. No successful navigation is inferred.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from spatialmind.physical import MetricObservation, Pose2D
from spatialmind.vision import (
    BoundingBox, CameraModel, detect_color_regions, project_optical_bbox,
    visible_optical_depth_cells,
)
from .core import read_records, safe_input_file


def calibration_from_json(path: str | Path) -> tuple[CameraModel, float]:
    doc = json.loads(Path(path).read_text(encoding="utf-8"))
    required = ("width", "height", "fx", "fy", "cx", "cy", "depth_scale_to_m")
    if any(field not in doc for field in required):
        raise ValueError(f"Missing camera calibration fields: {required}")
    scale = float(doc["depth_scale_to_m"])
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("Invalid depth scale")
    k = CameraModel(int(doc["width"]), int(doc["height"]),
                    float(doc["fx"]), float(doc["fy"]),
                    float(doc["cx"]), float(doc["cy"]))
    return k, scale


def _load_images(rgb_file: Path, depth_file: Path, width: int, height: int, scale: float):
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError('Install Pillow: pip install -e ".[datasets]"') from exc
    with Image.open(rgb_file) as photo:
        rgb = photo.convert("RGB")
        if rgb.size != (width, height):
            raise ValueError("RGB image and calibration dimensions differ")
        pixels = rgb.tobytes()
    with Image.open(depth_file) as photo:
        if photo.size != (width, height):
            raise ValueError("Depth and RGB dimensions differ (registration needed)")
        # Preserve 16bit data (converting to L silently truncates depth).
        if photo.mode not in ("I", "I;16", "I;16L", "I;16B"):
            raise ValueError("Depth must be registered 16-bit integer PNG")
        depths = [float(pixel) * scale for pixel in photo.getdata()]
    return pixels, depths


def replay_rgbd(
    *, manifest: str | Path, camera_file: str | Path,
    output: str | Path, limit: int = 300,
    min_pixels: int = 8,
    preserve_observations: bool = True,
) -> dict:
    """Process real images. Only world-camera poses produce mapped object estimates.

    Negative evidence is DISABLED for uncalibrated detector recall by forcing
    coverage_quality=0. Use with fixed-camera offline recordings, not navigation.
    """
    if limit < 1 or min_pixels < 1:
        raise ValueError("limit/min_pixels must be positive")
    doc = json.loads(Path(manifest).read_text(encoding="utf-8"))
    if doc["dataset"] not in {"tum", "openloris"}:
        raise ValueError("RGB-D replay only accepts TUM and OpenLORIS")
    root = Path(doc["source_root"]).resolve()
    index = Path(doc["records_path"])
    if not index.is_file():
        raise FileNotFoundError(index)
    k, scale = calibration_from_json(camera_file)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    from spatialmind.belief import BeliefMemory
    memory = BeliefMemory(str(output / "replay_beliefs.sqlite"))
    result = {
        "dataset": doc["dataset"], "frames_read": 0,
        "frames_with_real_pose": 0, "frames_without_3d_localization": 0,
        "detected_color_regions": 0, "map_projected_observations": 0,
        "corrupt_frames": 0, "memory_objects": 0,
        "warning": (
            "Offline RGB-D replay, not a robot navigation success result. "
            "Color detector is a controlled marker baseline, NOT general object recognition."
        ),
    }
    try:
        with (output / "observations.jsonl").open("w", encoding="utf-8") as observations:
            for record in read_records(index):
                if result["frames_read"] >= limit:
                    break
                if record.kind != "rgbd_frame":
                    continue
                result["frames_read"] += 1
                rgb_file = safe_input_file(root, record.data["rgb"])
                depth_file = safe_input_file(root, record.data["depth"])
                rgb, depths = _load_images(rgb_file, depth_file, k.width, k.height, scale)
                boxes = detect_color_regions(rgb, k.width, k.height, min_pixels=min_pixels)
                result["detected_color_regions"] += len(boxes)
                pose = record.data.get("camera_world_pose")
                entry = {
                    "frame_id": record.id, "timestamp": record.timestamp,
                    "dataset": record.dataset, "rgb": record.data["rgb"],
                    "depth": record.data["depth"],
                    "bbox": [{"x0": b.x0, "y0": b.y0, "x1": b.x1,
                              "y1": b.y1, "label": b.label,
                              "confidence": b.confidence} for b in boxes],
                    "localized_objects": [],
                    "source": "real_recorded_rgbd",
                }
                if pose is not None:
                    result["frames_with_real_pose"] += 1
                    x, y, z = tuple(map(float, pose["xyz"]))
                    quat = tuple(map(float, pose["xyzw"]))
                    stamp = float(record.timestamp)
                    estimates = []
                    for box in boxes:
                        estimate = project_optical_bbox(
                            box, depths, k, (x, y, z), quat,
                            stamp=stamp, map_frame="dataset_world",
                            evidence_ref=f"{record.dataset}:{record.id}",
                        )
                        if estimate:
                            estimates.append(estimate)
                            entry["localized_objects"].append({
                                "label": estimate.label,
                                "x": estimate.pose.x, "y": estimate.pose.y,
                                "confidence": estimate.confidence,
                                "position_variance": estimate.pose.position_variance,
                            })
                    # Pose is the TUM *camera* optical center, not robot base_link.
                    camera_pose = Pose2D(x, y, frame_id="dataset_world", stamp=stamp)
                    coverage = visible_optical_depth_cells(
                        depths, k, (x, y, z), quat,
                    )
                    memory.update(MetricObservation(
                        camera_pose, tuple(estimates), coverage,
                        coverage_quality=0.0, source=f"offline:{record.dataset}",
                    ))
                    result["map_projected_observations"] += len(estimates)
                else:
                    result["frames_without_3d_localization"] += 1
                if preserve_observations:
                    observations.write(json.dumps(entry, sort_keys=True) + "\n")
        result["memory_objects"] = memory.count()
        (output / "replay_report.json").write_text(
            json.dumps(result, indent=2), encoding="utf-8"
        )
    finally:
        memory.close()
    return result

"""TUM and exported OpenLORIS RGB-D time association.

Only explicitly matched frame pairs are exported. Camera-frame ground truth
is NOT a robot pose or a map-frame localization. No object labels are invented.
"""
from __future__ import annotations

import bisect
import math
from pathlib import Path
from typing import Iterator

from .core import DatasetRecord, safe_input_file


def _index_file(root: Path, basename: str | None, choices: tuple[str, ...]) -> Path:
    if basename:
        return safe_input_file(root, basename)
    for relative in choices:
        try:
            return safe_input_file(root, relative)
        except FileNotFoundError:
            continue
    raise FileNotFoundError(f"Missing index: provide --rgb-index / --depth-index; tried {choices}")


def read_timestamp_index(root: Path, filename: str, *, fields: int = 2) -> list[tuple]:
    path = safe_input_file(root, filename)
    result = []
    with path.open(encoding="utf-8") as f:
        for line_number, text in enumerate(f, 1):
            stripped = text.split("#", 1)[0].strip()
            if not stripped:
                continue
            parts = stripped.split()
            if len(parts) != fields:
                raise ValueError(f"{path.name}:{line_number} expected {fields} fields")
            stamp = float(parts[0])
            if not math.isfinite(stamp):
                raise ValueError("Non-finite timestamp")
            rest = parts[1:]
            if fields == 2:
                safe_input_file(root, rest[0])
                result.append((stamp, rest[0]))
            else:
                floats = tuple(map(float, rest))
                if not all(math.isfinite(v) for v in floats):
                    raise ValueError("Non-finite pose")
                result.append((stamp, floats))
    result.sort(key=lambda item: item[0])
    return result


def nearest(entries: list[tuple], timestamps: list[float], stamp: float, tolerance: float):
    if not entries:
        return None
    index = bisect.bisect_left(timestamps, stamp)
    options = [entries[j] for j in (index-1, index) if 0 <= j < len(entries)]
    match = min(options, key=lambda e: abs(e[0]-stamp))
    return match if abs(match[0]-stamp) <= tolerance else None


def rgbd_records(
    name: str, root: Path, *, limit: int = 1000,
    rgb_index: str | None = None, depth_index: str | None = None,
    pose_index: str | None = None, max_delta_s: float = .04,
    max_pose_delta_s: float = .05, split: str = "unspecified",
) -> Iterator[DatasetRecord]:
    if name not in {"tum", "openloris"}:
        raise ValueError(name)
    if max_delta_s <= 0 or max_pose_delta_s <= 0:
        raise ValueError("Sync tolerances must be positive")
    rgbfile = _index_file(root, rgb_index, (
        "rgb.txt", "color.txt", "d400_color.txt", "d400/color.txt",
    ))
    depthfile = _index_file(root, depth_index, (
        "depth.txt", "aligned_depth.txt", "d400_aligned_depth.txt",
        "d400/aligned_depth.txt",
    ))
    rgb = read_timestamp_index(root, str(rgbfile.relative_to(root)), fields=2)
    depth = read_timestamp_index(root, str(depthfile.relative_to(root)), fields=2)
    poses = []
    if pose_index is not None:
        poses = read_timestamp_index(root, pose_index, fields=8)
    else:
        for name_candidate in ("groundtruth.txt", "ground_truth.txt"):
            if (root / name_candidate).is_file():
                poses = read_timestamp_index(root, name_candidate, fields=8)
                break
    depth_times = [r[0] for r in depth]
    pose_times = [r[0] for r in poses]
    seen_depth = set()
    count = 0
    for stamp, rgb_path in rgb:
        paired = nearest(depth, depth_times, stamp, max_delta_s)
        if not paired or paired[1] in seen_depth:
            continue
        seen_depth.add(paired[1])
        pose = nearest(poses, pose_times, stamp, max_pose_delta_s)
        # TUM is camera optical-center GT. OpenLORIS export must label its pose frame.
        camera_pose = (
            {"xyz": list(pose[1][:3]), "xyzw": list(pose[1][3:]),
             "timestamp": pose[0], "frame": "dataset_world_camera_optical"}
            if pose and name == "tum" else None
        )
        yield DatasetRecord(
            dataset=name,
            kind="rgbd_frame",
            id=f"{root.name}:{count:08d}",
            sequence=root.name, split=split, timestamp=stamp,
            data={
                "rgb": rgb_path, "depth": paired[1],
                "depth_timestamp": paired[0],
                "rgb_depth_delta_s": abs(stamp - paired[0]),
                "camera_world_pose": camera_pose,
                "pose_index_matched": pose is not None,
                "pose_raw": list(pose[1]) if pose and name == "openloris" else None,
                "depth_unit": "raw_uint16",
                "depth_scale_to_m": (1/5000 if name == "tum" else None),
                "calibration": "must_supply_camera_json_for_projection",
                "observation_source": "offline_dataset_not_robot_action",
            },
        )
        count += 1
        if count >= limit:
            break

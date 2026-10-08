"""3RScan multi-scan change/instance annotations (offline evaluation only).

The 3RScan.json rescan transformations contain millimetre translations;
we retain source units and matrices rather than silently aligning them.
"""
from __future__ import annotations

import json
import re
from zipfile import ZipFile
from pathlib import Path
from typing import Iterator

from .core import DatasetRecord, safe_input_file


def _load_json(path: Path):
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def scan_records(root: Path, *, limit: int = 1000) -> Iterator[DatasetRecord]:
    emitted = 0
    meta_path = root / "3RScan.json"
    if meta_path.is_file():
        metadata = _load_json(meta_path)
        if not isinstance(metadata, list):
            raise ValueError("3RScan.json must be a list")
        for scene in metadata:
            reference = str(scene["reference"])
            split = str(scene.get("type", "unspecified"))
            yield DatasetRecord(
                "3rscan", "scan_group", reference, reference, split,
                data={"reference_scan": reference,
                      "rescan_ids": [str(x["reference"]) for x in scene.get("scans", [])],
                      "change_annotation_count": len(scene.get("scans", [])),
                      "is_ground_truth_metadata": True},
            )
            emitted += 1
            if emitted >= limit:
                return
            for rescan in scene.get("scans", []):
                child = str(rescan["reference"])
                payload = {
                    "reference_scan": reference, "rescan": child,
                    "rigid_object_changes": rescan.get("rigid", []),
                    "removed_instances": rescan.get("removed", []),
                    "nonrigid_instances": rescan.get("nonrigid", []),
                    "rescan_to_reference_transform": rescan.get("transform"),
                    "transform_translation_unit": "millimetres_as_provided_by_3rscan",
                    "alignment_required_before_metric_comparison": True,
                }
                yield DatasetRecord(
                    "3rscan", "oracle_scan_change", f"{reference}:{child}",
                    reference, split, data=payload,
                )
                emitted += 1
                if emitted >= limit:
                    return

    # Scan instance labels are ground truth, not robot detections.
    for semseg in sorted(root.rglob("semseg.v2.json")):
        if emitted >= limit:
            break
        if not semseg.is_relative_to(root):
            continue
        doc = _load_json(semseg)
        yield DatasetRecord(
            "3rscan", "scan_asset", semseg.parent.name, semseg.parent.name,
            data={"scan_id": semseg.parent.name,
                  "semantic_annotation": semseg.relative_to(root).as_posix(),
                  "requires_scene_alignment": True},
        )
        emitted += 1
        if emitted >= limit:
            break
        scan_id = str(doc.get("scan_id", semseg.parent.name))
        for instance in doc.get("segGroups", []):
            if emitted >= limit:
                break
            instance_id = str(instance.get("objectId", instance.get("id", "")))
            if not instance_id:
                continue
            box = instance.get("obb", {})
            yield DatasetRecord(
                "3rscan", "oracle_object_instance", f"{scan_id}:{instance_id}",
                scan_id,
                data={
                    "scan_id": scan_id,
                    "instance_id": instance_id,
                    "semantic_label": instance.get("label"),
                    "obb": box,  # scan-local coordinates, not a ROS map pose
                    "source_annotation": semseg.relative_to(root).as_posix(),
                    "coordinate_frame": "3rscan_scan_local",
                },
            )
            emitted += 1

    # Sequence archive is referenced but NEVER extracted automatically.
    for sequence_zip in sorted(root.rglob("sequence.zip")):
        if emitted >= limit:
            break
        yield DatasetRecord(
            "3rscan", "rgbd_archive", sequence_zip.parent.name,
            sequence_zip.parent.name,
            data={
                "archive_path": sequence_zip.relative_to(root).as_posix(),
                "contains": "registered RGB-D, intrinsics and camera poses",
                "not_extracted": True,
            },
        )
        emitted += 1
        if emitted >= limit:
            break
        # Read ZIP central directory only; never auto-extract 3RScan's licensed
        # image payload or trust arbitrary archive member paths.
        with ZipFile(sequence_zip) as archive:
            entries = set(archive.namelist())
            for rgb in sorted(entries):
                match = re.search(r"(frame-[0-9]+)\\.color\\.jpg$", rgb)
                if not match or rgb.startswith("/") or ".." in rgb.split("/"):
                    continue
                stem = rgb[:-len(".color.jpg")]
                depth = stem + ".depth.pgm"
                pose = stem + ".pose.txt"
                if depth not in entries or pose not in entries:
                    continue
                yield DatasetRecord(
                    "3rscan", "scan_rgbd_frame",
                    f"{sequence_zip.parent.name}:{match.group(1)}",
                    sequence_zip.parent.name,
                    data={
                        "archive_path": sequence_zip.relative_to(root).as_posix(),
                        "rgb_member": rgb,
                        "depth_member": depth,
                        "pose_member": pose,
                        "camera_info_member": next(
                            (name for name in entries if name.endswith("/_info.txt")),
                            None,
                        ),
                        "depth_scale_to_m": 0.001,
                        "camera_pose_convention": "camera_to_scan_world_matrix",
                        "frame": "3rscan_scan_local",
                        "source": "offline_3rscan_sequence_not_robot_action",
                    },
                )
                emitted += 1
                if emitted >= limit:
                    return

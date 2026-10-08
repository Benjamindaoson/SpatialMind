"""Verified official upstream entrypoints. Source rights do NOT follow SpatialMind's MIT license."""
from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class DatasetInfo:
    id: str
    name: str
    upstream: str
    rights: str
    acquisition: str
    formats: tuple[str, ...]
    usage: str
    needs_approval: bool = False

    def to_dict(self) -> dict:
        return asdict(self)


DATASETS = {
    "tum": DatasetInfo(
        "tum", "TUM RGB-D", "https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download",
        "Academic attribution and upstream dataset terms apply.",
        "Download individual TGZ sequences from official TUM page and extract locally.",
        ("rgb.txt", "depth.txt", "groundtruth.txt"),
        "timestamp-synchronized camera/depth/pose replay; NOT autonomous robot success",
    ),
    "openloris": DatasetInfo(
        "openloris", "OpenLORIS-Scene",
        "https://lifelong-robotic-vision.github.io/dataset/scene.html",
        "Follow OpenLORIS download terms and attribution.",
        "Official download page provides Google Drive / Baidu links; obtain sequences manually.",
        ("RGB/depth text indexes", "ground truth or odometry indexes", "ROS bags"),
        "mobile-robot sensor/relocalization replay; real camera calibration required",
    ),
    "3rscan": DatasetInfo(
        "3rscan", "3RScan", "https://github.com/WaldJohannaU/3RScan",
        "Research dataset access/download conditions apply; toolkit's MIT is not data license.",
        "Obtain permitted 3RScan.json and per-scan semseg.v2.json files from official releases.",
        ("3RScan.json", "semseg.v2.json", "sequence.zip", "mesh.refined.v2.obj"),
        "cross-scan object identity, change detection and alignment; keep split grouping",
        True,
    ),
    "replicacad": DatasetInfo(
        "replicacad", "ReplicaCAD", "https://aihabitat.org/datasets/replica_cad/",
        "Dataset assets CC BY 4.0 with attribution. Check bundled components.",
        "habitat_sim.utils.datasets_download --uids replica_cad_dataset --data-path DATA_ROOT",
        ("*.scene_instance.json", "*.scene_dataset_config.json", "*.glb"),
        "Habitat scene assets and reconfiguration metadata; not an automatically converted Gazebo world",
    ),
    "hm3d": DatasetInfo(
        "hm3d", "Habitat Matterport3D / HM3D ObjectNav",
        "https://github.com/matterport/habitat-matterport-3dresearch",
        "Academic non-commercial, account/data-access agreement and applicable terms.",
        "Acquire via approved Matterport token and Habitat downloader; ObjectNav episodes separately.",
        ("*.basis.glb", "*.scene_dataset_config.json", "*.json.gz"),
        "Habitat scene index + ObjectNav episode goals, oracle isolated from policy",
        True,
    ),
    "goat": DatasetInfo(
        "goat", "GOAT-Bench", "https://github.com/Ram81/goat-bench",
        "Check GOAT episode license and HM3D scene licenses independently.",
        "Download GOAT episode archives from official repo; HM3D scenes separately with access.",
        ("*.json.gz", "*.json"),
        "sequential category/language/image-goal navigation episodes, not ROS2 execution",
        True,
    ),
}


def dataset_info(name: str) -> DatasetInfo:
    try:
        return DATASETS[name.lower()]
    except KeyError as exc:
        raise ValueError(f"Unknown dataset {name!r}; choose {', '.join(DATASETS)}") from exc

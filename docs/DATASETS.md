# SpatialMind Data Pipeline — Six Upstream Sources

**Status:** Dataset adapters and offline replay are implemented and CI-tested against deliberately tiny **synthetic fixtures**. The repository does **not** contain downloaded 3RScan, HM3D, OpenLORIS, ReplicaCAD, GOAT-Bench, or TUM raw data. **No real-dataset benchmark scores or real Nav2/Gazebo success rates are claimed.**

Raw datasets are not committed to GitHub. They remain in a local `data/` mount and are governed by each provider's **own** usage terms (SpatialMind's MIT license covers this integration code, not third-party data).

## Architecture

```text
Approved local data/ mounts                    Dataset adapters
---------------------------------------------- -------------------------------------
TUM RGB-D         rgb.txt/depth.txt/GT         RGB-D sync/trajectory association ─┐
OpenLORIS         exported RGB-D indexes       Sensor-only or calibrated replay ───┤
3RScan            3RScan.json, semseg.v2.json   Scan/instance/change annotations ───┤
ReplicaCAD        *.scene_instance.json        Habitat scene asset catalog ────────┤
HM3D              GLB + ObjectNav *.json.gz    Scene/episode catalog ──────────────┤
GOAT-Bench        episode *.json.gz             Sequential multimodal goals ───────┘
                                                       │
                                        prepare → manifest.json
                                                       │
                          ┌────────────────────────────┴─────────────────────────┐
                          ▼                                                      ▼
                records.jsonl (policy)                               oracle.jsonl (eval only)
                          │                                                      │
         RGB-D pixel/depth replay / Habitat episode                      sealed annotations for
         task identity + metadata                                        independent evaluator
                          │                                                      │
                 Temporal belief store                             paired scoring / diagnostics
```

The Habitat sources **cannot be meaningfully run through Nav2 just by reading JSON**. They remain native Habitat assets until an interactive Habitat-Sim/agent bridge is actually executed. Scene/object annotations and GOAT answers must not enter the policy's memory as observations.

## Getting started

```bash
python -m pip install -e ".[datasets]"
spatialmind data sources
# Or: python -m spatialmind.datasets sources
python scripts/run_data_pipeline.py --config configs/datasets.example.json
```

This prints a separate `missing_local_data` result for every absent dataset rather than manufacturing records. To require every selected source to exist, use `--fail-on-missing`.

The example configuration mounts assets under `data/`. Use a separate disk and symlinks if desired; the large original files should not be committed.

### TUM RGB-D — early sensor validation

Official [TUM RGB-D Download](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download) and [File Formats](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats). Start with `fr1/xyz`, unpack the downloaded sequence into `data/tum/rgbd_dataset_freiburg1_xyz`. This contains `rgb.txt`, `depth.txt`, `groundtruth.txt`, PNG frames. Its registered 16-bit depth images use **raw value / 5000 = meters**.

```bash
spatialmind data prepare --dataset tum \
  --source data/tum/rgbd_dataset_freiburg1_xyz --output artifacts/datasets/tum
spatialmind data replay --manifest artifacts/datasets/tum/manifest.json \
  --camera configs/cameras/tum_fr1.json --output artifacts/datasets/tum/replay --limit 100
spatialmind data verify --manifest artifacts/datasets/tum/manifest.json
```

TUM GT is the **camera optical center** in its recording coordinate frame, not the autonomous robot base pose. Replay uses a transparently limited color-region detector with real RGB PNG/depth/pose bytes. The resulting belief objects are suitable for wiring tests, **not** claims of general object recognition or navigation success.

### OpenLORIS-Scene — actual mobile sensor sequences

Official [dataset](https://lifelong-robotic-vision.github.io/dataset/scene.html) and [processing tools](https://github.com/lifelong-robotic-vision/openloris-scene-tools). Choose a small office sequence with registered D435i RGB + aligned depth; dataset archives/download links are provided by the official page. The native source is often a ROS bag; **export its images and time indexes using the author's processing tools** first. We do not silently pretend TUM's index naming matches the original bag.

```bash
spatialmind data prepare --dataset openloris --source data/openloris/office1-1 \
  --output artifacts/datasets/openloris \
  --rgb-index color.txt --depth-index aligned_depth.txt
```

If the official sequence is still in a ROS1 bag, SpatialMind can export **specified raw image topics** using the optional `rosbags` dependency. Inspect available topics first; never assume topic names, camera calibration or coordinate frames:

```bash
pip install -e ".[datasets-rosbag]"
# Set topic names based on the particular recorded bag:
spatialmind data export-bag --bag data/openloris/office1-1.bag \
  --rgb-topic /YOUR_RGB_IMAGE_TOPIC \
  --depth-topic /YOUR_ALIGNED_DEPTH_IMAGE_TOPIC \
  --camera-info-topic /YOUR_RGB_CAMERA_INFO_TOPIC \
  --depth-scale-to-m 0.001 --max-frames 1000 \
  --output data/openloris/office1-1
# Then run "spatialmind data prepare --dataset openloris" as above.
```

Only raw `rgb8/bgr8` and registered `16UC1/mono16` topic encodings are currently supported. Confirm the depth scale from the sensor and validate RGB-depth registration before using any metric projection. This does NOT generate a camera-to-world pose or a localization result.

If data use different index names, pass their exact paths. OpenLORIS pose records are not automatically assumed to describe camera-optical world pose; `camera_world_pose` remains null. Provide a calibrated camera and validated TF/trajectory transformation before reporting metric localization. Pixel-only RGB-D replay is supported without pretending the camera has a global map pose.

### 3RScan — temporal object changes, offline oracle only

Official [3RScan toolkit](https://github.com/WaldJohannaU/3RScan), [FAQ](https://github.com/WaldJohannaU/3RScan/blob/master/FAQ.md). Follow dataset access conditions; use scene-level `3RScan.json` plus allowed per-scan `semseg.v2.json` and optional `sequence.zip`.

```bash
spatialmind data prepare --dataset 3rscan --source data/3rscan \
  --output artifacts/datasets/3rscan --limit 2000
```

The importer also inspects `sequence.zip` without extracting it and indexes each complete `frame-*.color.jpg / depth.pgm / pose.txt` triple as a scan-local RGB-D frame reference. No automatic geometric registration or global-coordinate merge occurs.

Outputs reference/rescan scene links in `records.jsonl`, and instance labels plus object-level rigid/nonrigid/removed changes in separate `oracle.jsonl`. **3RScan transformation translations may be in millimetres**, so the importer retains source units and explicitly requires proper alignment before comparing poses across scans. The adapter does not claim to perform 3D mesh registration.

### ReplicaCAD — interactive Habitat objects and scenes

Official [ReplicaCAD](https://aihabitat.org/datasets/replica_cad/) is CC BY 4.0. Install a compatible Habitat-Sim environment separately and download through its official utility:

```bash
python -m habitat_sim.utils.datasets_download \
  --uids replica_cad_dataset --data-path data/habitat
# Adjust --source below to the actual resulting replica_cad directory:
spatialmind data prepare --dataset replicacad --source data/habitat/replica_cad \
  --output artifacts/datasets/replicacad
```

Indexes `*.scene_instance.json` + scene configuration metadata; no direct Gazebo conversion is performed.

### HM3D + HM3D ObjectNav — academic research terms

Official [Matterport HM3D research assets](https://github.com/matterport/habitat-matterport-3dresearch) are restricted to permitted research usage. Follow the agreement and obtain Matterport API credentials locally; **never commit tokens**.

Use Habitat's official downloader for `hm3d_minival_v0.2`, and obtain ObjectNav episodes from the official [Habitat-Lab DATASETS.md](https://github.com/facebookresearch/habitat-lab/blob/main/DATASETS.md), choosing a compatible HM3D semantics version. Supply a directory containing the scene assets and episode archives:

```bash
spatialmind data prepare --dataset hm3d --source data/hm3d \
  --output artifacts/datasets/hm3d
```

Scenes and `navigation_episode` records go into the policy index; task answers and goal locations are quarantined into `oracle.jsonl`. The listed goal category may be provided to a policy, but target *location* must not.

### GOAT-Bench — sequential language/category/image goals

Official [GOAT-Bench](https://github.com/Ram81/goat-bench) includes episode archives and instructions. It requires **separately acquired and licensed HM3D scenes** for simulation. Use native Habitat `*.json.gz` episodes:

```bash
spatialmind data prepare --dataset goat --source data/goat \
  --output artifacts/datasets/goat
```

The index retains episode ID, scene ID, starting pose, count and safe per-goal modalities/descriptions. Goal-coordinate answers are stored separately under `oracle.jsonl`. **This code does not yet execute GOAT episodes in Habitat-Sim or reproduce official GOAT policy scores.**

## Batch pipeline

```bash
python scripts/run_data_pipeline.py --config configs/datasets.example.json
python scripts/run_data_pipeline.py --dataset tum --dataset openloris \
  --with-replay
python scripts/run_data_pipeline.py --config configs/datasets.example.json \
  --fail-on-missing
```

Products (per dataset): `manifest.json` containing hashes, source directory, record counts and rights notice; `records.jsonl` for permitted input, `oracle.jsonl` only when ground-truth annotations exist; TUM/OpenLORIS `replay/observations.jsonl`, `replay/replay_beliefs.sqlite` and `replay/replay_report.json`.

### Data anti-leakage and honesty controls

- No network fetch, checkpoint download, account access or agreement acceptance is performed by the preparation scripts.
- Raw assets are referenced locally, never embedded in the repository or redistributed.
- `oracle.jsonl` must not be passed to a planner, an LLM context, policy memory or learned target-location predictor at evaluation time.
- TUM/OpenLORIS are **offline observations**; they are not autonomous action rollouts and should be evaluated on perception/tracking/localization, not task completion/SPL.
- Habitat scenes/episodes need an actual Habitat environment and runtime policy bridge for closed-loop ObjectNav/GOAT metrics.
- The toy-grid and metric-grid numbers must remain separate from all public-data results.
- Log split, source version, scene/sequence ID, hashes, calibration, synchronization tolerances, detector settings, and any human labels.
- This package's MIT LICENSE is **not a redistribution grant** for any upstream dataset.

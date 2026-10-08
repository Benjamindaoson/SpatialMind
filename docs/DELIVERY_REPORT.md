# SpatialMind v0.2 Implementation & Evidence Report

## Executive status

This delivery implemented software modules across **all six planned workstreams**. It does **NOT** mean all six physical-world acceptance gates are complete.

- Repository: https://github.com/Benjamindaoson/SpatialMind
- Core CI: https://github.com/Benjamindaoson/SpatialMind/actions/workflows/ci.yml
- Separate ROS2 Jazzy + Gazebo Harmonic workflow: https://github.com/Benjamindaoson/SpatialMind/actions/workflows/ros2-verify.yml
- Original software baseline: https://github.com/Benjamindaoson/SpatialMind/actions/runs/37757260097

## Six-stage delivery ledger

| Stage | Committed work | Validation | Remaining physical evidence |
|---|---|---|---|
| G0 — architecture | Metric frame/time/covariance contract, async Robot protocol, RoomMap, task actor with single-active-mission guard | Automated frame tests, navigation mocks, interruption and resume tests | Live TF and AMCL / Nav2 endpoint verification |
| G1 — Gazebo + Nav2 | Generated SDF world, aligned occupancy map and semantic waypoints, RGB-D robot SDF injector, ament package, Nav2 action client and Gazebo launch scripts | Static SDF/map generation tests and dedicated ROS integration CI | Actual robot movement in headless Gazebo, Nav2 goal result, video/rosbag |
| G2 — vision + spatial memory | RGB8/BGR8, 16-bit/float depth decoding, synced CameraInfo + TF2 projection, color-region segmentation, conservative FOV-based negative evidence, metric temporal belief store | Pure-Python pixel/depth/calibration and memory tests | Actual Gazebo RGB-D topics, optical TF calibration, robust real-world detector/tracker |
| G3 — long-horizon agent | Async mission, revise/cancel, bounded retry, failure events, sensor-grounded goal completion, checkpoint persisted and restored with frame/map checks | Software tests for revisions/cancel/recovery/no-camera navigation | Real Nav2 preemption, autonomous search under live visual observations |
| G4 — strong ablations | 7 intervention families, 4 policies, 4 static topology variants, metric distances, independent post-hoc oracle, seeded paired bootstrap, SVG/CSV/JSON/trace exports, Gazebo set_pose intervention helper | CI synthetic metric-sim runs and regression tests | >=200 distinct real Gazebo/real-robot trials with reproducible paired results |
| G5 — MLOps | LLM call/token budgets, event-based triggering, cache invalidation, latency/RSS measurement, disk quota helper, rosbag recording, local Docker apps and CI | Core CI and Docker API smoke tests | Real model inference/quantization benchmark, platform safety sign-off, real VRAM and latency measurements |

## Current reproducible commands

~~~bash
python -m pip install -e ".[api,dev]"
python -m unittest discover -s tests -v

spatialmind physical-benchmark --seeds 3 \
  --output-dir artifacts/physical-eval --trace

uvicorn spatialmind.physical_api:app \
  --host 127.0.0.1 --port 8001

python scripts/generate_gazebo_scene.py \
  --out simulation/generated

docker compose up --build
~~~

Ports: original Grid Mission Control on 8000; async Physical Agent Lab on 8001. Both are intentionally bound to loopback in Compose.

ROS Gazebo runbook: [ROS2_GAZEBO.md](ROS2_GAZEBO.md).

Field protocol: [PHYSICAL_EVALUATION.md](PHYSICAL_EVALUATION.md).

## Assessment criteria that are NOT passed yet

1. No verified, executable Gazebo RGB-D + Nav2 end-to-end mission trace has been uploaded to the repository.
2. No hardware trials were performed.
3. The detector is a color-connected-components reference baseline, not a general-purpose VLM object perception model.
4. No independently labeled >=200-mission ROS/Gazebo benchmark has passed.
5. No honest real-model quantization/VRAM speedup figure exists.

Until those happen, résumé metrics must be described as **grid/metric simulation results only**. Never claim end-to-end deployed Physical AI, 3D visual SLAM, trained VLA, real robot success or guaranteed safety.

## Code provenance and evidence rules

- The decision policy must not read simulator ground-truth object locations.
- Simulated ground truth may only be used by a separate evaluator after mission completion.
- Every mission's action, navigation result, observation and temporal memory revision should be linked to task and evidence IDs.
- Negative results must not be hidden; compare the full policy fairly to No-Memory, Static-Memory and Nearest-Search.
- Experiments must preserve code SHA, environment seed, map version, sensor parameters and raw logs.
- Security: no API keys in telemetry, and never expose unauthenticated experiment control to public networks.

## High-priority remaining acceptance run

On a compatible Ubuntu 24.04 ROS2 Jazzy + Gazebo Harmonic machine:

1. Complete the ROS build / SDF / Nav2 bringup CI gate.
2. Check camera topic, TF2 optical projection and real Rendered RGB-D frames against a calibrated landmark.
3. Run a live target-search mission and save rosbag2 + agent trace; independently judge completion.
4. Intervene by moving target with the Gazebo service, validate stale-memory correction and actual navigation recovery.
5. Run randomized paired baselines with source-level equality of environments and >=200 distinct instances.
6. Report speed, latency, safety outcomes and error bars; only then mark G1/G2/G4/G5 physical acceptance as complete.

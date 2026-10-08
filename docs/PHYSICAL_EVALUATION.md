# Physical AI Benchmark: Interventional Evaluation Protocol

## Versions and honesty

SpatialMind has **two different synthetic testbeds**:

- Legacy grid-v0.1: fixed 2D topology, deterministic objects, three policies; saved for regression and historical trace comparability.
- Metric-intervention-v1: 4 topology variants, seven intervention families, four policy configurations, metric odometry, sensor dropouts, false detections and an **independent post-hoc oracle** to distinguish claimed from actual success.

Neither is a Gazebo/ROS2 benchmark. Never combine their metrics or claim measured physical-world improvements.

## Reproduce the multi-policy experiment

~~~bash
pip install -e ".[api,dev]"
spatialmind physical-benchmark --seeds 30 \
  --output-dir artifacts/metric-30-seeds --trace
~~~

Artifacts: trials.csv, metrics.json, comparison.svg, traces/*.json.

Metrics are calculated from actual executed tasks. The source does not contain hand-inserted success rates or path improvements.

## Interventions

- Fresh visible object (sanity).
- Historical remote object position.
- Stale memory following object displacement.
- Visual occlusion.
- Blocked crossing.
- Two objects with the same label.
- Missed detections and synthetic false positives.

Pair each (scenario, seed, topology) across all policies. No policy reads the hidden object dictionary; the oracle checks evidence **after** completion.

## Baselines

- Full Agent: temporal belief update, object-memory candidate and semantic-prior search.
- No-Memory: identical navigation/sensor but no historical memory retrieval.
- Static-Memory: original historical belief frozen; online corrections omitted.
- Nearest-Search: accessible candidate with shortest nominal distance; no semantic prior.

A baseline outperforming Full must be reported, not excluded.

## Field-ready acceptance gate G4

1. Use the real Gazebo/ROS2 robot, Nav2 Action and image/depth/TF pipeline; do not replace sensor predictions with simulator object IDs.
2. Collect >=200 distinct, independent paired tasks across different maps, viewpoints, object placements, obstacles and random seeds (not repeated identical trajectories).
3. Store success, false-positive completion, actual navigation distance (metres), recovery rate, memory invalidation error, camera-to-map localization error, P50/P95 task + model latencies, peak RSS/VRAM, token count and cost.
4. Compute paired deltas with bootstrap confidence intervals, and error categories with paired task IDs.
5. Archive full rosbag2 logs, config, world and map hashes, git revision, model ID and code/test version.
6. Check safety envelopes, navigation stoppage and uncertain perception under obstruction.
7. Publish representative negative examples where Full loses to simpler strategies.

## MLOps fairness

The InferenceGovernor only permits semantic event-based model invocation and enforces call/token/cost budgets, preventing LLM calls in the control loop. Cost comparisons must provide configured provider token prices; a zero default cost means pricing not configured, **not free production inference**.

For actual inference acceleration experiments, compare fixed hardware/model, quantization, caching and task quality; disclose speed, CPU/GPU resources and errors. Do not claim GPU optimization from CPU unit tests.

## Interpretation

Source-tested 100% synthetic completion is evidence of correct software execution under that suite, not general mobile robot reliability. Baseline results and loss cases must remain visible.

# Metric Physical-Agent Benchmark — Verified CI Results

**Run date:** 2026-10-08  
**Source of truth:** [GitHub Actions Core CI, run 37784917085](https://github.com/Benjamindaoson/SpatialMind/actions/runs/37784917085)  
**Code revision:** `0256b857784b9646c91751ac5310dc78524438f2`  
**Status:** Unit/API/Docker checks **passed**: 49 Python tests, 336 executed paired metric-simulation missions.  
**Environment:** **Pure Python metric grid**, 4 fixed topology variants; 12 seeds × 7 interventions × 4 policies. **Not Gazebo, ROS2, or real hardware.**

## Results — full disclosure

| Policy | Verified success | Mean simulated movement | Mean replans | False-positive completions |
|---|---:|---:|---:|---:|
| Full Temporal Agent | **81/84 (96.43%)** | 20.869 m | 2.690 | 0 |
| No-Memory | 78/84 (92.86%) | **20.619 m** | 3.750 | 1 |
| Static-Memory | **81/84 (96.43%)** | 20.869 m | 2.690 | 0 |
| Nearest-Search | **82/84 (97.62%)** | **19.167 m** | **2.536** | 0 |

### Paired intervention deltas (84 paired tasks per comparison)

Positive distance delta = Full travels fewer simulated metres.

| Comparison | Success difference Full − Baseline | Mean metres saved Full − Baseline | Paired bootstrap 95% CI on metres |
|---|---:|---:|---:|
| vs No-Memory | +3.57 percentage points | −0.250 m | [−2.369, +1.500] |
| vs Static-Memory | 0 pp | 0.000 m | [0.000, 0.000] |
| vs Nearest-Search | −1.19 pp | −1.702 m | [−4.107, 0.000] |

**Interpretation:**

- In this test suite Full used fewer replans and had more successful missions than No-Memory, but these differences are not sufficient to establish a robust physical-world benefit.
- **Nearest-Search outperformed Full** in overall success and distance for this limited synthetic benchmark. Improving active-search viewpoint selection and reducing semantic-prior overconfidence is a priority. No favorable-only scenario selection.
- Full and Static-Memory were identical at this level of intervention; current benchmark does **not** distinguish dynamic memory update value. A stronger stale-memory identity/occlusion benchmark must be designed.
- These are **simulated grid metres**, not actual odometry from Gazebo. P95 model latency/VRAM and real sensor precision are not established.
- Some seeds share deterministic physical layout/target placements. Re-running more seeds alone does not constitute 200 independent physical environments.

## How to reproduce

```bash
pip install -e ".[api,dev]"
spatialmind physical-benchmark --seeds 12 \
  --output-dir artifacts/metric-12seeds --trace
```

The generated `trials.csv`, `metrics.json`, `comparison.svg` and `traces/*.json` are also archived as the CI artifact `grid-benchmark` attached to the run.

## Acceptance blockers

1. Execute actual ROS2/Jazzy + Gazebo/Harmonic navigation and perception under an appropriate ROS environment.
2. Prove real RGB-D semantic localization and object identity under occlusion/motion.
3. Improve active search against nearest-neighbor baseline under matched seeds.
4. Run multiple genuinely distinct maps/initial positions with ≥200 external-evidence-grounded paired missions.
5. Benchmark real LLM/VLM speed and memory on specified hardware.

Until then, do not attribute these metrics to real mobile robots or Gazebo experiments.

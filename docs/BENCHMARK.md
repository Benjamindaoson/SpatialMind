# Evaluation Protocol

![Intervention protocol](images/intervention.svg)

## Research question

When the natural-language goal is fixed but the physical environment changes, does the agent revise its beliefs and actions in ways consistent with the new observations and task constraints?

**Variables** — Intervention type (target move / sensor occlusion / obstacle), memory policy (full / disabled), search policy (prior-driven / uniform), environment and seed.

**Outcomes** — End-to-end goal success with positive evidence; navigational action attempts; path length measured in grid steps; replans; unsuccessful navigation results; wrong-memory invalidation; action latency (future ROS2/Gazebo milestone).

## v0.1 executable suite

Run:

~~~bash
spatialmind benchmark --output artifacts/benchmark.json
spatialmind benchmark --seeds 20 --output artifacts/benchmark-20seeds.json --trace-dir artifacts/traces
~~~

Current deterministic scenarios:

| ID | World modification | Task |
|---|---|---|
| find_toolbox | none | find blue toolbox |
| moved_toolbox | store memory then move toolbox to office | find blue toolbox |
| memory_hint_toolbox | a separate sensor observation confirms toolbox in office | find blue toolbox from distant initial pose |
| occluded_toolbox | move toolbox and insert local obstacle | find blue toolbox |
| blocked_corridor | obstruct the east crossing | navigate to meeting room |
| find_first_aid | different target class and location | find red first aid kit |

Policy names: full, no_memory, uniform_search. Results are captured in JSON; reports are generated through actual code execution and are not hardcoded. A source-level study should additionally archive the full event database, map configuration and code revision.

## Anti-leakage

The active planner is given only map geometry, candidate memory records, current robot pose and already observed cells. The planner must not read the simulator's hidden object positions. The simulated sensor can use object ground truth to generate a detection, but that information cannot bypass RobotAdapter.observe().

**Exception to address before publication:** Synthetic detections use stable object IDs and perfect labels/coordinates. This eliminates an important real-world perception error source and must be replaced by noisy perception and track matching for physically meaningful results.

## Future statistical evaluation

- Minimum 100–200 **distinct** randomized task instances across independent world layouts, object starting positions, perturbations and seeds, with no train/test world leakage.
- Compare success proportions with bootstrap confidence intervals; pair interventions across policies by identical seeds.
- Distinguish **correct abstention** from false failure; distinguish timeout from incorrect task planning.
- Report P50/P95 end-to-end and model latency, context tokens, peak RAM/VRAM, traversed distance, collision/safety violations, checkpoint restore rate, and cost per solved mission.
- Publish failure examples and cases where the full agent loses to the simpler baselines.
- Run equivalent evaluation in Gazebo before any claim of physical validity; clearly label real-robot sample sizes.

## Interpretation limits

v0.1 trials check **software behavior**. Even a 100% result in this suite does not imply real-world success, open-world generalization, or outperforming published methods. The research contribution, if any, must be evaluated on sufficiently hard interventions, meaningful baselines, and independently reproducible physical perception and navigation.

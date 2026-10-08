# SpatialMind Architecture & Design Decisions

![Architecture](images/architecture.svg)

## Architecture invariants

1. **Separate decision and motion:** The agent selects semantic waypoints, target verification and retry policy; the robot adapter owns the details of navigation execution. Nav2 remains the authoritative navigation controller in a ROS2 deployment.
2. **Observation is evidence, not a guarantee:** A detection is usable evidence, not a universal state of the physical world. Simulated stable IDs are an explicit test assumption.
3. **Negative evidence needs visibility:** Only when a previously occupied cell is provably in the new sensor coverage may the old observation be downgraded.
4. **Context is a projection of state:** Keep the task goal, robot position and constraints in structured fields rather than treating a summarized LLM prompt as the source of truth.
5. **Bounded recovery:** Each navigation failure has an explicit reason, a retry budget and observable events.
6. **Reproducibility:** Events are append-only, checkpoints persist, and test policies are named and versioned.

## Core dataflow

~~~mermaid
sequenceDiagram
    actor User
    participant Runtime as Agent Runtime
    participant Context
    participant Memory as Spatial Memory
    participant Robot as Robot Adapter
    participant Trace as Event Store
    User->>Runtime: instruction
    Runtime->>Robot: observe()
    Robot-->>Runtime: detection + visible cells
    Runtime->>Memory: update observation and possible negative evidence
    Runtime->>Trace: observation, memory_update
    loop Until verified, exhausted or interrupted
        Runtime->>Context: goal + state + relevant memory/events
        Runtime->>Memory: candidate locations
        Runtime->>Robot: navigate(selected viewpoint)
        Robot-->>Runtime: result / blocked / unreachable
        Runtime->>Trace: plan, action, retry, checkpoint
        Runtime->>Robot: observe()
        Robot-->>Runtime: new sensor evidence
        Runtime->>Memory: evidence-based revision
    end
    Runtime-->>User: structured result + evidence ID
~~~

## Task and memory contracts

- All robot motion requests use typed points; a ROS2 adapter performs metric-map conversion.
- Each observation contains timestamp, robot pose, detected objects and **visible_cells**.
- Object records retain object/track id, label, position, room, confidence, status, sightings and the last observation ID.
- The source of truth for individual decisions is the structured agent state; the event log records each transition and can be replayed independently.
- SQLite checkpoints persist progress **but the simulator's actual world is not persisted**. On process restart, physical pose and observations must be re-grounded and task assumptions reevaluated.

## Known simplifications

The grid planner uses an approximate Manhattan-radius information-gain score over the known static map. It does not see the actual hidden object list or unseen dynamic obstacle positions. The robot's sensor uses line-of-sight ray casting for observations. This distinction allows meaningful, if simplified, partial observability.

No claim of fully general Chinese/English intent grounding, dynamic topological SLAM, robot pose uncertainty propagation, RGB-D object detection or Gazebo integration is made in the current milestone.

## Next-step integration seams

1. Replace static GridWorld map with an occupancy/semantic map service (TF-aware).
2. Implement a map-frame perception adapter returning object tracks and visible free-space polygons/voxels.
3. Wire ROS2 navigation feedback, cancellation, sensor callbacks and a safety supervisor to an asynchronous agent runtime.
4. Run Gazebo and physical trials under identical task specifications and explicit observation/proprioception logs.

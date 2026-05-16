# Pick chain redesign — M5 (2026-05-16)

## Context

M5 (end-to-end pick) hit 11+ failed iterations (v11 → v21) chasing distinct
bugs:

- v11–v12: PYTHONPATH layout for non-colcon repo packages
- v13: Isaac Sim 5.0 `IsaacCreateViewport` width/height removed
- v14: empty `/joint_states` → MoveIt zero-pose self-collision
- v15: orientation tolerance too tight
- v17: no scene lighting → FP gets no RGB texture
- v18: FP locks on table (no mask isolating cube)
- v19: depth-window mask isolates cube, FP returns correct depth
- v20: ArmPlanner returns "goal_rejected handle=None" — rclpy executor
  deadlock when subscription callback calls action client
- v21: MultiThreadedExecutor alone did not fix it
- v22 (not run): would have used `ReentrantCallbackGroup`; even then
  exposed to rclpy [issue #1123](https://github.com/ros2/rclpy/issues/1123)
  race-condition bug

Web search confirms most of the pain is documented rclpy fragility, not
unique. The current orchestrator hand-rolls a pattern that the
[Karelics writeup](https://karelics.fi/deadlocks-in-rclpy/) explicitly
warns against (synchronous action call inside a subscription callback).
Continuing to patch is a strict downgrade vs. redesigning the pick chain
with the lessons learned.

This spec redesigns ONLY the pick chain (orchestrator + pose source +
MoveIt invocation). Everything outside it stays: AMR fleet, coordinator,
table/cube/camera/lighting/TF setup, FoundationPose itself, ArmPlanner
class, ManipulationPipeline, scenario YAML, run_scenario.py.

## Goals

- **Eliminate the callback-deadlock class of bugs** by moving all action
  client work to a dedicated worker thread with its own rclpy executor.
- **Decouple perception from orchestration** via a `PoseSource` protocol
  with two interchangeable implementations (FoundationPose, simulator
  ground truth). Set via a ROS2 parameter.
- **Cut iteration time** from ~9 min (full sim) to ~30 s for the
  pick-chain debug loop via a no-sim test harness.
- **Ship M5 acceptance** (`orders_completed = 1`) on `pose_source=gt`;
  treat `pose_source=fp` as the stretch goal.

## Non-goals

- Rewriting Nav2 / pure_pursuit / coordinator / scenario / sim setup.
- Replacing FoundationPose with a different perception model.
- Adding real gripper execution (M5 stays `plan_only=True`).
- Changing Phase 2's acceptance metric (still 1 order completed on
  `hungarian_cbs`).

## Architecture

### Component boundaries

**Kept as-is**:

- `manipulation/pose_estimation.py` — FoundationPose wrapper with the v19
  nearest-depth mask.
- `manipulation/grasping.py` — `TopDownGrasp`, `TopDownGraspFromPose`.
- `manipulation/motion_planning.py` — `ArmPlanner` class.
- `manipulation/pipeline.py` — `ManipulationPipeline` (minor refactor to
  accept any `PoseSource`).
- `wdt_vast/run_scenario.py` — fleet + Franka + table + cube + lighting +
  TFs + subprocess orchestration.
- All ROS2 packages outside `wdt_manipulation_bringup`.

**Rewritten**:

- `ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py`
  — replaced with a thin rclpy node + a `PickWorker` thread.

**Added**:

- `manipulation/pose_source.py` — `PoseSource` Protocol with two
  implementations.
- `wdt_vast/sim_world_pose_publisher.py` — Isaac Sim-side publisher of
  `/world/cube_pose` for `gt` mode.
- `tests/unit/test_pose_source.py`, `tests/unit/test_pick_worker.py`.
- `tests/integration/test_pick_chain_fast.py` — no-sim harness.

### Threading model

The orchestrator process has two threads:

1. **Main thread** runs `rclpy.spin(node)` on a `SingleThreadedExecutor`.
   Handles only:
   - `/cell/cam/{rgb,depth,info}` subscriptions (cache the latest frame
     under a `threading.Lock`).
   - `/cell/start_pick` subscription — snapshot the cached cam frame,
     enqueue `(order_id, snapshot, t_received)` on the worker queue,
     return.
   - `/cell/pick_result` publisher (thread-safe).
   - tf2 listener.

   Callbacks are tiny; never call action clients here.

2. **Worker thread** owns:
   - Its own `rclpy.Node` (used only for the MoveGroup action client).
   - Its own `SingleThreadedExecutor` it spins on this thread.
   - A `queue.Queue` of pending pick requests.

   For each request: call `PoseSource.get_pose(...)` → tf2 transform →
   `TopDownGrasp.propose_at(...)` → `ArmPlanner.plan_to_pose(...)` →
   publish `PickResult` via the main node's publisher. Synchronous from
   the worker's perspective.

The two executors are independent. The MoveGroup goal-acceptance
message arrives on the worker's executor, which is actively spinning
inside `plan_to_pose`. No deadlock. No race with subscription callbacks.

Worker starts on `__init__`, stops on shutdown (drains queue, joins).

### Pose source abstraction

```python
# manipulation/pose_source.py

from typing import Protocol
import numpy as np

class PoseSource(Protocol):
    """Returns the 6D pose of the pick target."""
    def get_pose(
        self,
        rgb: np.ndarray | None,
        depth: np.ndarray | None,
        camera_K: np.ndarray | None,
        cad_path: str,
    ) -> tuple[np.ndarray, str] | None:
        """Return (translation_3, source_frame_id) or None on failure."""
```

**`FoundationPosePoseSource`** — wraps `PoseEstimator`. Runs FP with the
v19 nearest-depth mask. Returns
`(pose.translation, "cell_cam_optical")`.

**`GroundTruthPoseSource`** — subscribes to `/world/cube_pose`
(`geometry_msgs/PoseStamped`). Returns
`(latest_pose.position, latest_pose.header.frame_id)`. Ignores rgb /
depth / K / cad_path. Returns `None` if no pose received yet.

Selected via the orchestrator's ROS2 parameter `pose_source: "fp"` (default)
or `"gt"`. Set in run_scenario.py's `_ros2_popen("pick_orch", ...)` call.

### `/world/cube_pose` publisher

`wdt_vast/sim_world_pose_publisher.py` is a tiny rclpy node launched as a
subprocess by run_scenario.py. It reads the cube's USD prim's
worldspace transform each tick (via Isaac Sim's USD API) and publishes
`geometry_msgs/PoseStamped` on `/world/cube_pose` at 10 Hz.

The publisher runs in the Kit python (3.11) like `synthetic_cell_camera`
did — but unlike `synthetic_cell_camera`, this one has access to Isaac
Sim's stage. It runs as a SEPARATE subprocess of run_scenario.py because
it needs Kit; the orchestrator subprocess (which reads `/world/cube_pose`)
runs under `/usr/bin/python3` (3.10) and DOESN'T need Kit.

Frame id is `"world"`.

### Pick chain flow

```
1. Main thread, /cell/start_pick callback
   - Snapshot {rgb, depth, K} from cached state under lock
   - Enqueue (order_id, snapshot, t_received)
   - Return  (<1ms)

2. Worker thread, dequeue
   - pose_source.get_pose(rgb, depth, K, cad_path)
     - FP: ~3s. GT: ~10ms
     - Returns (translation, source_frame_id) or None
   - If None: PickResult(success=false, "no_pose"); publish; loop

3. Worker thread, TF transform
   - tf_buffer.lookup_transform(target="panda_link0",
                                source=source_frame_id, time=Time())
   - First call: ~50ms (TF discovery). Subsequent: cached, <1ms
   - If exception: PickResult(success=false, "tf_lookup_failed"); publish

4. Worker thread, grasp generation
   - TopDownGrasp.propose_at(panda_t)
   - grasp_t = panda_t + (0,0,+0.05); rotation = gripper-down
   - (single candidate, symmetric cube, deterministic)

5. Worker thread, MoveIt plan
   - arm.plan_to_pose(grasp_t, rotation)
   - Uses worker's node + worker's executor
   - Retry up to 3x on "handle=None" race-condition (rclpy #1123 workaround):
     - sleep 0.2s between retries
     - If still None: PickResult(success=false, "plan_action_failed")
   - On status≠SUCCEEDED or error_code≠1: PickResult("plan_no_solution")

6. Worker thread, publish result
   - PickResult{success, attempts, cycle_time_s, failure_reason}
   - Publish via main node's publisher (thread-safe)
   - Diagnostic INFO log: pose chain + grasp pose
   - Loop to step 2
```

**Expected timings**:

- Step 1 (callback): <1ms
- Step 2 (PoseSource): 3s (FP) or <10ms (GT)
- Step 3 (TF, cached): <1ms after first call
- Step 4 (grasp): <1ms
- Step 5 (MoveIt plan_only): ~200ms (measured in v20)
- Step 6 (publish): <1ms

Total: ~3.2s in `fp` mode, ~210ms in `gt` mode. Both well under the 5s
window that prior versions hit with the deadlock.

## Error handling

| `failure_reason` | Source | Meaning |
|---|---|---|
| `no_cam_data` | step 1 | start_pick arrived before any cam frame cached. Coordinator may retry. |
| `no_pose` | step 2 | PoseSource returned None. FP: registration failed. GT: no pose msg received. |
| `tf_lookup_failed` | step 3 | tf2 couldn't resolve source → panda_link0 in 2s. Static TF publisher crashed or not up yet. |
| `plan_no_solution` | step 5 | MoveIt returned status=SUCCEEDED with error_code≠1 OR OMPL timed out. Pose is unreachable or in collision. |
| `plan_action_failed` | step 5 | 3 retries on action client; never got a handle. rclpy race condition or move_group unreachable. |
| `plan_timeout` | step 5 | result_future exceeded GOAL_TIMEOUT_S=15s. move_group alive but stuck. |
| `worker_crashed: <type>` | any | Worker raised an unhandled exception. Sentinel result published; worker exits. run_scenario.py supervises. |

Distinguishing `plan_no_solution` from `plan_action_failed` is critical:
the former is physics ("can't reach"), the latter is infrastructure
("can't talk to MoveIt"). v20 conflated them as `exhausted_candidates`
which wasted iterations.

Diagnostic INFO log fires every pick:

```
pose chain: source_frame=cell_cam_optical
            pose_in_source=(x, y, z)
            -> panda_link0=(x', y', z')
            grasp_pose=(x'+0, y'+0, z'+0.05)
```

Logged to `<out_dir>/pick_orch.log`. Sufficient to replay analysis
without re-running the sim.

## Testing

### Unit (local Mac, ~10s total)

- `tests/unit/test_pose_source.py` — both implementations with mocked
  inputs.
- `tests/unit/test_arm_planner.py` — existing, unchanged.
- `tests/unit/test_pipeline.py` — existing, minor refactor for the new
  interface.
- `tests/unit/test_pick_worker.py` — new. Instantiate the worker with
  mock PoseSource + mock ArmPlanner, push fake requests, assert correct
  `PickResult` publishes. Validates threading + queue without rclpy
  spinning a real graph.

### Fast harness (vast.ai, ~30s per iteration)

`tests/integration/test_pick_chain_fast.py` — launches
`move_group` + `franka_ready_joint_states` + the orchestrator (NO Isaac
Sim, NO Carter, NO nav). Publishes fake `/cell/cam/info`, fake
`/world/cube_pose` at a known reachable point. Publishes
`/cell/start_pick`. Asserts `/cell/pick_result` arrives with `success:
true` in <2s.

Second variant for `fp` mode: publishes fake `/cell/cam/{rgb,depth}`
containing M4's synthetic cube fixture; asserts FP locks and pick
succeeds in <5s.

### Full end-to-end (vast.ai, ~9 min)

The existing run_scenario.py smoke. Runs after fast harness passes.
`gt` mode is the M5 acceptance demo (`orders_completed=1`). `fp` mode is
the M6 stretch goal — same orchestrator code, flag flipped.

## Success criteria

- All existing unit tests still pass; new ones pass.
- Fast harness `gt` mode: `success: true` in <2s, reproducibly.
- Fast harness `fp` mode: `success: true` in <5s, reproducibly.
- Full smoke `gt` mode: `metrics.orders_completed=1`.
- Full smoke `fp` mode: stretch — may or may not succeed; documented
  either way in `docs/results-phase-2.md`.

## Out of scope (deferred to later)

- Real gripper execution (controller manager + trajectory action).
- Multiple SKUs (one CAD path per orchestrator instance for now).
- Concurrent picks (worker is single-threaded, queue serializes).
- Adapting the depth-window mask threshold per-SKU (the 15 cm window
  works for an 8 cm cube on a 0.7 m-tall table).

# Eleven iterations into a single milestone

*How we eliminated a class of rclpy bugs in our warehouse digital twin's pick chain.*

---

## The scoreboard

Eleven failed iterations on M5 — the "do one pick" milestone of a Phase 2 sim build. Then a redesign, a 161-millisecond cycle, and a clean log: `orders_completed: 1`. This post is about the iterations, the actual bug, and why the fix is more interesting than the bug.

The project: an open-source warehouse digital twin on Isaac Sim 5.0 + ROS2 Humble. Phase 1 shipped a six-AMR fleet + scenario harness. Phase 2's spec said "close the loop": real Nav2-class navigation + real MoveIt2 + FoundationPose perception → an AMR drives to a shelf, drives to a pick cell, and the cell-side orchestrator picks one cube.

M5 was that loop. Eleven failed runs (v11 through v21) and a destroyed instance later, the code was a layer cake of half-debugged fixes. The pick orchestrator deserved a clean redesign.

## The actual bug

For ten of those iterations, we were chasing distinct issues: PYTHONPATH layout for non-colcon Python packages, an `IsaacCreateViewport` API change in Isaac Sim 5.0, an empty `/joint_states` message causing MoveIt to fall back to a self-colliding Panda zero pose, an orientation-tolerance constraint too tight for IK sampling near the workspace edge, missing scene lighting starving FoundationPose's RGB texture loss, missing CAD masking causing FP to lock onto the table instead of the cube, and so on.

Then v20. The pick orchestrator's logs showed something different:

```
[orchestrator] cycle_time_s: 4.998
[orchestrator] cycle_time_s: 4.999
[orchestrator] cycle_time_s: 5.001
```

Every single pick attempt finished in almost exactly 5.0 seconds. That was suspicious because `ACTION_TIMEOUT_S` in our MoveIt action client was `5.0`. The orchestrator wasn't picking — it was *waiting for `wait_for_server` to time out*.

The pattern was the rclpy callback-deadlock. Here's how it happens:

```python
# Inside a /cell/start_pick subscription callback
def on_start_pick(self, msg: String) -> None:
    ...
    # Action client send + spin (synchronous wait for goal accept)
    send_future = self._move_group_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=5.0)
    handle = send_future.result()  # always None — handle never accepted
    ...
```

The trouble is what `spin_until_future_complete` actually does: it asks the executor to process callbacks until the future completes. Inside the executor, the same callback group is already executing the `/cell/start_pick` subscription callback. The action server's `goal_response` arrives, the executor wants to run its callback — but it's serialized behind the running one. The `spin` never gets a chance to drive the future to completion. Five seconds later, the timeout fires; the future returns `None`; `handle.accepted` blows up; we record "pick failed."

This is widely documented. Karelics has [a thorough writeup](https://karelics.fi/deadlocks-in-rclpy/) of the pattern. The rclpy maintainers acknowledge it. The usual community advice: switch to `MultiThreadedExecutor` and put the action client in a `ReentrantCallbackGroup`.

v21 did exactly that. v21 also deadlocked, just less reliably. The MTE + reentrant route has a real race condition — see [rclpy issue #1123](https://github.com/ros2/rclpy/issues/1123) — that hasn't been fixed. Once you've seen `cycle_time_s ≈ 5.0` you can't unsee it; this whole region of rclpy is fragile.

## Why "the obvious fix" is the wrong fix

`ReentrantCallbackGroup + MultiThreadedExecutor` works *if every callback you ever write* is reentrant-safe. That's a property of your entire callback graph forever, not just the code you wrote today. Building on top of MTE means inheriting issue #1123's race indefinitely. And the testability story is bad — you can't unit-test a callback that depends on the executor's threading model without running rclpy itself.

We wanted a design that didn't ask the question. The action client shouldn't be on the same executor as the subscription that triggers it, period. The subscription callback shouldn't be in the same Python call stack as the action's `spin_until_future_complete`. The cleanest way to enforce that is a thread boundary.

## The redesign

Two threads, two executors, one orchestrator process:

```
┌─ pick_cell_orchestrator (rclpy Node, main thread) ────────────────┐
│  Subscriptions (callbacks tiny — cache state, return):            │
│    /cell/cam/{rgb,depth,info}  → snapshot under lock              │
│    /world/cube_pose            → GroundTruthPoseSource.set_latest │
│    /cell/start_pick            → enqueue PickRequest, return      │
│  Publishers: /cell/pick_result (thread-safe).                     │
│  TF2 listener (read-only).                                        │
│  Executor: SingleThreadedExecutor on rclpy.spin(node).            │
└──────────────────────────────────────────────────────────────────┘
                              │ queue.Queue
                              ▼
┌─ PickWorker (separate Python thread + separate rclpy.Node) ───────┐
│  Owns: rclpy.Node("pick_worker_arm")                              │
│        SingleThreadedExecutor (passed to ArmPlanner)              │
│        spin_once(timeout=0.1) ticks the executor in background    │
│  Loop:                                                            │
│    1. dequeue PickRequest                                         │
│    2. pose_source.get_pose(rgb, depth, K, cad_path)               │
│       ↦ (translation, source_frame_id) | None                     │
│    3. tf_lookup(source_frame → planning_frame)                    │
│    4. TopDownGrasp.propose_at(panda_t)                            │
│    5. arm.plan_to_pose(grasp_t, R)                                │
│       — uses *its own* executor for spin_until_future_complete    │
│    6. publish PickResult                                          │
└──────────────────────────────────────────────────────────────────┘
```

The main-thread subscription callback never calls an action client. It captures state, drops a `PickRequest` on a `queue.Queue`, and returns in microseconds. The worker thread is the only place that talks to MoveIt, and it uses *its own* executor, so `spin_until_future_complete` is not racing the main-thread executor at all.

`ArmPlanner` was a small refactor: it takes an optional `executor=` constructor kwarg. When the worker passes its own executor, `rclpy.spin_until_future_complete` is served by that specific executor, not the global default. Default `executor=None` preserves the existing call sites.

The third piece: `PoseSource` as a `Protocol`. The worker doesn't care which:

```python
class PoseSource(Protocol):
    def get_pose(
        self,
        rgb: np.ndarray | None,
        depth: np.ndarray | None,
        camera_K: np.ndarray | None,
        cad_path: str,
    ) -> tuple[np.ndarray, str] | None: ...
```

Two implementations:

- `FoundationPosePoseSource` — wraps the existing FP estimator, returns `(translation, "cell_cam_optical")`. Takes ~3 s per call.
- `GroundTruthPoseSource` — fed by a `/world/cube_pose` subscription on the main thread, returns the latest cube pose under a lock. Takes ~10 µs.

A single ROS2 parameter on the orchestrator (`pose_source: "fp" | "gt"`) picks one. The worker code is identical for both. This let us validate the orchestrator architecture against GT pose first, then run the same code with FP without touching the orchestrator.

## How this eliminates the class of bug

The rclpy callback-deadlock has a structural pre-condition: the same executor that's running your callback is also the only one that can service your future. Cut the second half — give the action call its own executor — and the pre-condition can't be satisfied. It's not about being careful with reentrancy; the topology of the executor graph makes the bug impossible.

There's a second-order win: the failure mode of `spin_until_future_complete` becomes *real timeout from MoveIt*, not *deadlock under load*. When `ArmPlanner` returns `"goal_rejected handle=None"`, it's now actually telling you the action server didn't accept the goal — usually the rclpy #1123 race (which can still surface inside `ActionClient.send_goal_async` itself, just not as an executor deadlock). We added a tight retry on exactly that signature:

```python
if "handle=None" in last_message:
    if attempt < self._max_race_retries:
        time.sleep(self._race_retry_sleep_s)
        continue
    publish(PickResult(..., failure_reason="plan_action_failed"))
    return
# Real planner failure — surface the message, no retry.
publish(PickResult(..., failure_reason=f"plan_no_solution({last_message})"))
```

Distinguishing "infrastructure flake" (retry up to 3×) from "real physics failure" (surface immediately) turned out to matter for debug speed — v20's logs had conflated them as "exhausted_candidates", which is what caused the four-iteration spiral of guessing about IK and tolerances.

## Validation

**Unit tests.** `PickWorker` is a plain Python class with constructor-injected dependencies:

```python
worker = PickWorker(
    pose_source=_StubPoseSource((np.array([0.4, 0.0, -0.25]), "panda_link0")),
    arm_planner=_StubArmPlanner([_StubArmResult(True, "ok")]),
    publish_result=results.append,
    tf_lookup=lambda _src: np.eye(4),
    cad_path="x",
)
worker.start()
worker.enqueue(PickRequest(...))
```

Six tests, no rclpy spinning, covering success, `no_pose`, `tf_lookup_failed`, `plan_no_solution`, `plan_action_failed`, and the race-retry behavior. Total runtime: 0.12 s. That's the real testability win; the worker's correctness has nothing to do with whether rclpy is up.

**Fast harness.** With Isaac Sim out of the loop entirely, we launch a real `move_group` + a Franka `/joint_states` publisher + the orchestrator, then publish a fake `/world/cube_pose` at a Franka-reachable target. Iteration time: ~30 s including `move_group`'s load. That fast harness shipped the architecture in one shot — `pick_result {success: true, attempts: 1, cycle_time_s: 0.161, failure_reason: ""}`.

**Full smoke.** End-to-end on Isaac Sim 5.0: AMR navigates spawn → shelf (57 s wall) → cell (122 s wall), coordinator publishes `/cell/start_pick`, orchestrator handles it (161 ms), `metrics.json` shows `orders_completed: 1, pick_success_rate: 1.0`. The whole 8-minute smoke contains zero `cycle_time_s ≈ 5.0` lines.

## The unfortunate epilogue

Then we tried FP mode on the same hardware. The orchestrator did exactly what it should have. The worker called `PoseEstimator.estimate(...)`, FP loaded its two checkpoints, and PyTorch responded:

```
RuntimeError: CUDA error: no kernel image is available for execution on the device
```

PyTorch 2.4.0+cu124 ships precompiled kernels for `sm_50..sm_90` (Maxwell→Hopper). Our smoke ran on a freshly-rented RTX 5090, which is `sm_120` (Blackwell). PyTorch ≥2.6 / cu126 adds Blackwell support; the FP upstream we're tracking hasn't validated against 2.6 yet, so the install pin is still 2.4.0.

The interesting part: the orchestrator handled it perfectly. `PickWorker._loop`'s outer `try/except` caught the unhandled `RuntimeError`, published a sentinel `pick_result{success: false, failure_reason: "worker_crashed: RuntimeError: CUDA error: no kernel image..."}`, the coordinator received it, marked the order FAILED, the recorder captured it. No deadlock, no hang, no zombies. **The redesign's failure path works as designed.** The compatibility fix is Phase 3 — pick a Hopper/Ampere host or wait for FP's 2.6 validation.

## Takeaways

Three things from the eleven iterations:

1. **`cycle_time_s ≈ ACTION_TIMEOUT_S` is a deadlock signature.** If your callback's wall time is mysteriously equal to your timeout, you're not in a slow path — you're in a deadlock that's just polite enough to return. The fix is structural, not parametric.

2. **`MultiThreadedExecutor + ReentrantCallbackGroup` solves the instance, not the class.** Worker-thread + own-executor is harder to write once and easier to live with forever. The Karelics writeup is the right starting point.

3. **`Protocol`-based dependency injection makes the failure modes testable.** Six unit tests covering the worker's full state machine, zero rclpy. When the M6 FP smoke crashed on a CUDA kernel mismatch six months later, we already knew the failure-path code was correct — the test covering `worker_crashed: ...` had passed locally on day one.

The redesign repo is [zeon01/warehouse-digital-twin](https://github.com/zeon01/warehouse-digital-twin). The architectural artifacts are in `manipulation/pose_source.py`, `manipulation/pick_worker.py`, and `ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py`. The expert consultation we ran in parallel (the doc that argued us out of the "force frame namespacing" rabbit hole) is at `docs/m5-expert-consultation.md` + `docs/m5-expert-response.md`.

The release is [v0.2.0](https://github.com/zeon01/warehouse-digital-twin/releases/tag/v0.2.0). Phase 3 starts when the PyTorch pin moves.

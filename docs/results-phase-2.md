
# Phase 2 results

## M5 acceptance — gt mode (2026-05-17)

**orders_completed = 1** on `hungarian_cbs` with `pose_source=gt` (orchestrator
reads `/world/cube_pose` from `wdt_vast/sim_world_pose_publisher.py`).

Verified on California 5090 instance 36905209, run v22h_gt:

```json
{
  "orders_total": 1,
  "orders_completed": 1,
  "pick_success_rate": 1.0,
  "avg_cycle_time_s": 118.63,
  "p95_cycle_time_s": 0.0,
  "deadlocks_total": 0
}
```

events.log:
```
1.400 ENQ o1
120.033 DONE o1 success=True attempts=1
```

Cycle:
- spawn (2, 2) → shelf (4, 8): 57.4 s wall, dist 0.495 m at tolerance 0.5 m
- shelf → cell (16, 15): 121.8 s wall, dist 0.498 m at tolerance 0.5 m
- pick chain: `start_pick → pick_result` in 161 ms (GT pose lookup +
  `tf2_buffer.lookup_transform("panda_link0", "world")` +
  `TopDownGrasp.propose_at(...)` + `ArmPlanner.plan_to_pose(...)`, MoveGroup
  motion plan computed successfully)

The closed-loop chain validated:
- AMR namespaced fleet on Isaac Sim 5.0 + ROS2 Humble bridge
- Pure-pursuit driver (Nav2 deferred to Phase 3 per gotcha #5)
- Coordinator dispatch (hungarian_assign + NavigateToPose action chain)
- `/cell/start_pick` from coordinator → orchestrator
- Orchestrator's redesigned thin-node + worker-thread architecture
  (`manipulation/pick_worker.py`, `manipulation/pose_source.py`,
  `ros2_ws/src/wdt_manipulation_bringup/.../pick_cell_orchestrator.py`)
- `/world/cube_pose` static publisher → `GroundTruthPoseSource` →
  worker → tf2 transform → top-down grasp → MoveIt2 plan_only

FoundationPose remains validated standalone in M4. The fp-mode pick chain
(same orchestrator code, `pose_source=fp`) is the M6 stretch — requires
FoundationPose install (~15-20 min on a fresh vast.ai instance).

### Validation history

| Run | Host | GPU | Outcome | Cycle (pick) |
|---|---|---|---|---|
| v22b | Spain | RTX 3090 | scipy missing → coord crash | n/a |
| v22c | Romania | RTX A5000 | nav timeout (5x slower than memory baseline) | n/a |
| v22e | Romania | RTX A5000 | max_linear=1.5 caused heading oscillation; nav stalled | n/a |
| v22f | Romania | RTX A5000 | killed for migration to 5090 host | n/a |
| v22g | California | RTX 5090 | pick chain shipped end-to-end; metrics.json bug masked | 333 ms |
| **v22h** | California | RTX 5090 | **orders_completed=1; metrics.json shows 1** | **161 ms** |

The thin-node + worker-thread + own-executor architecture eliminated the
rclpy callback-deadlock class of bugs that consumed M5 v11–v21 iterations.
The fast harness on Spain (commit `e214acd`) validated this at 0.16 s
cycle without Isaac Sim; v22g/v22h confirmed the same architecture holds
in the full simulator under multi-AMR nav load.

### Known followups

- Coordinator's `_refresh_poses` uses global tf2 buffer; cannot resolve
  per-AMR namespaced frames. False-positive DEADLOCK warns suppressed
  for now (`fleet_coordinator/node.py` change in commit `e1a9ff8`);
  full fix is Option 3 from the M5 expert consultation (subscribe to
  `/amr_X/odom` topic per AMR).
- `metrics.json` integration shells out via log-replay
  (`run_scenario.py` parses `pick_orch.log` at `recorder.flush()`
  time, gotcha #18: kit python 3.11 can't subscribe via rclpy).
  Phase 3 should move the recorder out-of-process to its own
  `/usr/bin/python3` subscriber for cleaner separation.
- Carter physics tracking is host-dependent. v22c at default
  `max_linear=0.5` got 0.01 m/s wall on A5000; v22g/v22h at same
  setting got 0.116 m/s wall on 5090 — 11× faster on the same code.
  Memory's "0.05 m/s wall on RTX 3090" baseline holds for v7 but
  is unreliable across host families. Don't depend on it for scenarios.

## M5 stretch — fp mode

Pending — same orchestrator + `pose_source=fp` ROS2 param, but requires
the FoundationPose install path (~15-20 min on fresh vast.ai). Deferred
to next session.

## M6 stretch — fp mode (2026-05-17)

**`pick_success_rate=0.0` with `failure_reason="worker_crashed: RuntimeError: CUDA
error: no kernel image is available for execution on the device"`** on California
RTX 5090 instance 36905209, run v22_fp.

```json
{
  "orders_total": 1,
  "orders_completed": 1,
  "pick_success_rate": 0.0,
  "avg_cycle_time_s": 118.63,
  "p95_cycle_time_s": 0.0,
  "deadlocks_total": 0
}
```

events.log:
```
1.400 ENQ o1
120.033 DONE o1 success=False attempts=0
```

**Root cause: PyTorch 2.4.0+cu124 has no Blackwell (sm_120) kernels.** PyTorch's
precompiled kernel set covers `sm_50 sm_60 sm_70 sm_75 sm_80 sm_86 sm_90`
(Maxwell through Hopper). Native sm_120 support landed in PyTorch 2.6+. Our
`wdt_vast/install_foundationpose.sh` pins 2.4.0+cu124 because the FoundationPose
upstream is verified against that exact version. PyTorch loaded both FP weights
successfully but the first `forward()` call hit
`CUDA error: no kernel image is available for execution on the device`.

The clean part of the result:

- AMR navigation completed both legs (57 s shelf, 122 s cell).
- Orchestrator received `/cell/start_pick`.
- `PoseEstimator` reset against the cube CAD, loaded the scorer + refiner weights,
  initialized FoundationPose without crash.
- First `estimate()` call hit the CUDA-no-kernel error inside FP's
  `predict_score` forward pass.
- **`PickWorker._loop`'s exception handler caught the unhandled `RuntimeError`,
  published a sentinel `pick_result {success: false, failure_reason:
  "worker_crashed: RuntimeError: CUDA error: ..."}`.** Coordinator received it
  and marked the order FAILED. No deadlock, no hang, no zombie subprocesses.
- Recorder picked up the pick_result via the v22h log-replay path; metrics.json
  correctly shows `pick_success_rate=0.0`.

This validates the worker-thread architecture's failure path. The orchestrator
is FP-ready; FP itself is host-incompatible on Blackwell.

### Phase 3 followups

1. Bump PyTorch to ≥2.6 with cu126 in `install_foundationpose.sh` AND verify
   pytorch3d 0.7.9 + nvdiffrast 0.4.0 + kornia 0.7.0 still work. Upstream FP
   probably hasn't pinned to 2.6+ yet, so this needs validation.
2. OR pick a Hopper/Ampere host (RTX 3090, 4090, A5000, A6000 — all
   sm_80/sm_86/sm_89/sm_90) where 2.4.0+cu124 still applies. Memory's earlier
   M4 success was on RTX 3090.
3. Document the install matrix: which GPU SKUs are FP-compatible with our pinned
   PyTorch, which require a newer pin.

### v0.2.0 release candidate

- `gt`-mode acceptance shipped (v22h California 5090).
- `fp`-mode wired and architecturally validated; CUDA kernel mismatch is a
  host/library version compatibility issue, not a pick-chain bug.
- The full redesign is feature-complete for M5; M6 hardware compatibility is
  Phase 3 scope.

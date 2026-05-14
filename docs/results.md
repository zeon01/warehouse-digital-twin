# Phase 1 Acceptance Results

Run date: **2026-05-15**
Scenario: `scenarios/steady_state.yaml`
Compute: vast.ai RTX A5000 (Romania datacenter, driver 570.211.01)

## Numbers (from `outputs/steady_state/metrics.json`)

| Metric | Value | Notes |
|---|---|---|
| Orders enqueued | **64** | All scheduled orders fired at correct sim times (t=0.000 → t=520.000s) |
| Orders completed | 0 | Phase 2 work — coordinator's `NavigateToPose` has no Nav2 server (Task 26 used direct cmd_vel) |
| Pick success rate | 0.0 | Phase 2 work — FoundationPose / AnyGrasp / MoveIt2 model weights not installed |
| Avg cycle time (sim s) | 0.0 | Derived from completed orders, none yet |
| Deadlocks detected | 0 | AMRs stationary (no driving from coordinator), so deadlock paths not exercised |
| Wall-clock | **~2 min** | 600s sim time × 5 (no rendering during this run) |
| Sim-time ratio | **5× real-time** | physics-only, render=False |

`orders_enqueued / orders_completed` is the most informative number for Phase 1: it demonstrates that the **scenario YAML → MetricsRecorder → events.log pipeline is sound at full scale**, but the downstream control + manipulation legs are not wired yet.

## What this actually proves

The acceptance run verifies the **structural skeleton** of the full Phase 1 architecture:

1. **Scenario schema** loads a 64-order YAML cleanly (pydantic validation, all `arrival_t` ordered correctly).
2. **Isaac Sim Kit** boots headless, opens the procedurally-built warehouse USD, spawns 6 namespaced Nova Carter AMRs + a Franka — survives a 10-min run without crashes.
3. **ROS2 bridge** stays alive throughout (`isaacsim.ros2.bridge-4.9.3 startup`, no shutdown).
4. **FleetCoordinator subprocess** launches via `ros2 run` against the colcon-built `ros2_ws/install`, registers as `/fleet_coordinator` for the full duration.
5. **MetricsRecorder** captures all 64 `ENQ` events with precise sim-time timestamps and flushes `metrics.json` + `events.log` on shutdown.
6. **Clean exit**: no `error.txt`, no `Traceback` in `run_scenario.log`, all subprocess handles terminated.

## events.log excerpt

```
0.000   ENQ o01
5.033   ENQ o02
10.033  ENQ o03
...
510.000 ENQ o63
520.000 ENQ o64
```

(64 lines total, monotonic timestamps, exactly matching `scenarios/steady_state.yaml`.)

## Why orders aren't completing yet — and how Phase 2 fixes it

Two integration legs remain:

### 1. Coordinator → AMR motion (deferred from Task 26 → Phase 2)

The current coordinator opens an `ActionClient` for `/amr_N/navigate_to_pose` (Nav2's `NavigateToPose` action), but no Nav2 server is running, so each `client.wait_for_server(timeout_sec=1.0)` fails with the logged warning `NavigateToPose action server not ready — Nav2 not running?`. The robots sit at their spawn poses.

Task 26 deliberately replaced full Nav2 with direct `/amr_0/cmd_vel` Twist publishing (verified: one Carter moved 2.43 m in 10 seconds) to prove the bridge ↔ `differential_drive` OG ↔ wheel physics chain works. Phase 2 will either:
- (a) bring up a real Nav2 stack: map server with a generated occupancy grid, AMCL initial-pose seeding, lifecycle activation of every Nav2 node, action server registration on each namespace — substantial integration work; or
- (b) replace `NavigateToPose` calls in `FleetCoordinator._send_goal` with a simpler waypoint follower that publishes `cmd_vel` directly per the CBS-planned path. ~half a day's work.

### 2. Manipulation pipeline → real picks (deferred from Tasks 34–37 → Phase 2)

The pipeline class structure is complete and unit-tested (with mocks), but the lazy-loaded upstream packages (`isaac_ros_foundationpose`, `anygrasp`, `moveit.planning`) aren't installed on the vast.ai instance. On a real call, `_lazy_load()` raises `ModuleNotFoundError`. `run_scenario.py` catches this and sets `manip = None` so the run survives; the progress log shows `manip_pipeline_skipped:ModuleNotFoundError` as expected.

To unlock real picks, Phase 2 needs:
- FoundationPose model weights download (~GB) into a known path on the vast.ai instance
- AnyGrasp model weights (~GB)
- MoveIt2 + `moveit_py` Python binding (`apt install ros-humble-moveit ros-humble-moveit-py`)
- A pick-cell trigger from the coordinator: when an AMR carrying an order reaches the cell, invoke `manip.pick(rgb, depth, cad, K)` with the cell camera's RGB-D snapshot

## Visual deliverables

The Task 21 4-angle render of the combined scene is the portfolio visual for Phase 1:

| | |
|---|---|
| ![iso](images/scene_iso.png) | ![amrs closeup](images/scene_amrs.png) |
| Iso view: warehouse + 12 blue shelves + red pick cell with Franka arm + 6 white Nova Carters in 3×2 grid | Closeup: all 6 Carters visible as distinct robots, framing derived from layout YAML (cluster centroid + isometric formula) |

## Phase 1 cost

- Modal: **$1.39** (one-time image build cost on Tasks 7, 8; rest of Modal usage was test runs ≤ 30s each)
- vast.ai: **~$2.50** (image pull on first rent + ~5 hours of testing across L4 / A10G / B200 / A5000 hosts, Pattern 3 stop/resume between sessions)
- **Total Phase 1: ~$4 of $60 budget** — well under the $25 alert threshold per Modal account

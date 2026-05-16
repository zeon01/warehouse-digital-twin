# M5 expert consultation — pick chain redesign, blocking TF/nav regression

**Date:** 2026-05-17
**Stage:** Phase 2 M5 (end-to-end pick chain) — architecturally validated via fast harness, blocked on a pre-existing AMR-navigation TF wiring issue.
**Repo:** https://github.com/zeon01/warehouse-digital-twin (MIT, public)
**Audience:** A senior engineer fluent in ROS2 (Humble), Isaac Sim 5.0, Nav2 / TF2, and at least one of: pure-pursuit / Carter URDF / MoveIt2 / vast.ai.

---

## 1. What we are building

An open-source warehouse digital twin as a portfolio project, targeting the Q1-2026 commercial pain points around AMR + manipulation orchestration (KION/Accenture/Siemens at GXO; Cyngn validation pipelines). The project is a single repo that, at the end of Phase 2, will:

- Procedurally generate a warehouse USD scene (shelves + aisles + pick cell)
- Spawn N namespaced Nova Carter AMRs and a Franka Panda in Isaac Sim 5.0
- Drive the AMRs through full Nav2 (eventually) or a pure-pursuit fallback (current production path)
- Pick objects at the cell with FoundationPose perception → top-down grasp generator → MoveIt2 plan (plan_only for now; no real gripper execution yet)
- Run a 3-config × 5-seed planner ablation (greedy_greedy / hungarian_greedy / hungarian_cbs) on a 64-order steady-state scenario
- Report `orders_completed`, `pick_success_rate`, cycle-time distributions

**Hybrid compute architecture** (forced by Isaac Sim 5.0 needing Vulkan, which Modal can't satisfy on any GPU tier we tried — L4 / A10G / B200):

- **vast.ai** (datacenter RTX 3090 / A5000 with driver 570+): Isaac Sim 5.0 + ROS2 Humble + render passes
- **Modal**: Budget tracker + a few CPU-only utilities (image build done but unused for sim)
- **Local Mac**: USD authoring (`usd-core`), pure-Python coordinator + grasping + pose-source unit tests, scenario YAML, render orchestration via SSH

Phase 1 (Isaac Sim boot + multi-AMR spawn + scenario harness) shipped as v0.1.0 with 39 commits and 14 unit tests green.

## 2. Phase 2 milestone status

| Milestone | State | Verified on | Notes |
|---|---|---|---|
| M0 — map gen + Carter URDF | GREEN | Local | trimesh-built map + URDF round-trip |
| M1 — single-AMR smoke | GREEN (via pure-pursuit fallback) | Quebec 36866311 (destroyed) | Nav2 DWB is silent because Carter LIDAR doesn't fire under standalone Isaac Sim (gotcha #5); pure-pursuit bypasses costmap entirely |
| M2 — multi-AMR fleet smoke | GREEN | Quebec 23764234 (destroyed) | 6 Carters, 6/6 SUCCEEDED, ~146 s wall each, 3 m offset, 480 s timeout |
| M3 — MoveIt2 plan-to-pose | GREEN | Quebec 34643374 (destroyed) | OMPL plans in 0.02 s; needs `/joint_states` publisher + plan_only=True (no controller manager) |
| M4 — FoundationPose synthetic smoke | GREEN | Quebec 36866311 | 8 cm cube mesh + 480×640 depth → FP estimate in 12 s |
| **M5 — end-to-end pick chain** | **Architecturally validated via fast harness; full smoke blocked** | **Spain 33388214 (live)** | **This is the consultation topic** |
| M6 — pick chain with live FoundationPose | Stretch | — | Same orchestrator, `pose_source=fp` flag |
| M7-M9 — ablation runs | Future | — | After M5 ships |

## 3. M5 redesign — what it is and why

### 3.1 The 11-iteration failure chain (M5 v11–v21)

We burned eleven iterations on layered rclpy / FP / MoveIt issues. Each iteration shipped a fix but the next iteration hit a different bug. The bottom of the rabbit hole turned out to be a documented rclpy fragility: synchronous action-client calls inside subscription callbacks deadlock the executor (see [Karelics' "Deadlocks in rclpy"](https://karelics.fi/deadlocks-in-rclpy/) and [rclpy issue #1123](https://github.com/ros2/rclpy/issues/1123)).

Our v20 orchestrator hit it verbatim — `cycle_time_s ≈ 5.0` matched `ACTION_TIMEOUT_S` exactly, with no actual MoveIt activity in between. v21 tried `MultiThreadedExecutor`; same deadlock. The v22-that-wasn't (using `ReentrantCallbackGroup`) would still be exposed to the rclpy #1123 race condition.

### 3.2 The redesign

Architecturally redesigned the pick chain to eliminate that whole class of bugs:

```
┌─ pick_cell_orchestrator (rclpy Node, main thread) ─────────────┐
│  Subscribers (all callbacks tiny — cache state, return):       │
│    /cell/cam/rgb,depth,info  → cache under lock                │
│    /world/cube_pose          → forward to GroundTruthPoseSource │
│    /cell/start_pick          → snapshot cache, enqueue request  │
│  Publishers:                                                    │
│    /cell/pick_result         (JSON PickResult)                 │
└────────────────────────────────────────────────────────────────┘
                              │ queue.Queue
                              ↓
┌─ PickWorker (separate Python thread + separate rclpy Node) ────┐
│  Loop: dequeue PickRequest                                      │
│    1. pose_source.get_pose(rgb, depth, K, cad_path)             │
│       - FP: ~3 s    GT: ~10 ms                                  │
│    2. tf_lookup(source_frame → panda_link0)                     │
│    3. TopDownGrasp.propose_at(t + (0, 0, +0.05))                │
│    4. ArmPlanner.plan_to_pose(grasp_t, R) — owns its own rclpy   │
│       SingleThreadedExecutor; spin_until_future_complete is     │
│       served by THAT executor, not the main-thread one          │
│    5. Publish PickResult                                        │
└────────────────────────────────────────────────────────────────┘
```

Key idea: the worker's executor is independent of the main-thread executor, so `spin_until_future_complete` inside the MoveGroup action call cannot deadlock with the subscription callback that started the chain.

Two `PoseSource` implementations behind a `Protocol`:

- `FoundationPosePoseSource` — wraps the existing `PoseEstimator`, returns `(translation, "cell_cam_optical")`
- `GroundTruthPoseSource` — reads from a `/world/cube_pose` subscription, returns `(translation, header.frame_id)`

ROS2 parameter `pose_source: "fp" | "gt"` selects which one. M5 acceptance is on `gt`; M6 stretch is `fp`.

Selected design over alternatives because:
- **Avoids `MultiThreadedExecutor`** which still has the #1123 race (and confused us in v21)
- **Allows GT-mode acceptance** without paying the FoundationPose install cost — important because FP install on a fresh vast.ai instance is ~15-20 min and a documented yak-shave
- **Cleanly testable** — the worker is a plain Python class injected with mock `pose_source`, mock `arm_planner`, mock `tf_lookup`. No rclpy spinning in unit tests
- **Cheap iteration** — a "fast harness" (no Isaac Sim, just `move_group` + `franka_ready_joint_states` + the orchestrator + fake `/world/cube_pose`) reproduces the full pick chain in ~30 s vs ~10 min for the full simulator smoke

### 3.3 Implementation: 10 tasks, 8 done, 2 blocked

Implementation plan: `docs/superpowers/plans/2026-05-16-pick-chain-redesign.md`. All seven local-code tasks (Tasks 1–7) shipped + the fast harness (Task 8). Code commits on `main`:

- `10fe960` — `manipulation/pose_source.py` + `FoundationPosePoseSource`
- `b3efb76` — `GroundTruthPoseSource` unit tests
- `06965b3` — `ArmPlanner` accepts explicit executor
- `d041417` — `manipulation/pick_worker.py` + 6 unit tests
- `cf8abc5` — Rewrote `pick_cell_orchestrator.py` as thin node + `PickWorker`
- `3341cc5` — `wdt_vast/sim_world_pose_publisher.py`
- `0d8e520` — `wdt_vast/run_scenario.py` wiring (pose_source param + cube pose publisher launch)
- `e214acd` — `tests/integration/test_pick_chain_fast.py`
- `178883e` — fix: publish `/world/cube_pose` from system-python (kit lacks rclpy — see §5.1)

### 3.4 The fast harness DOES work

Run on Spain instance 33388214 (RTX 3090, driver 570.195.03, ROS2 Humble + MoveIt2 apt installs from our `bootstrap_phase{1,2}.sh`, no Isaac Sim, no Carter, no Nav2):

```
==> result: {'order_id': 'fast_harness_o1', 'success': True, 'attempts': 1, 'cycle_time_s': 0.16142253205180168, 'failure_reason': ''}
EXIT=0
```

161 ms cycle, first-attempt success, no deadlock. The thin-node + worker-thread architecture works end-to-end against a real `move_group` action server. **The redesign's core claim — eliminating the rclpy callback-deadlock class of bugs — is validated.**

## 4. The blocker: AMR navigation TF wiring

Running the full smoke (`run_scenario.py /work/scenarios/smoke.yaml ... --pose-source gt`) on Spain produces:

```json
{
  "orders_total": 1,
  "orders_completed": 0,
  "pick_success_rate": 0.0,
  ...
}
```

The orchestrator and worker are correct (proven by the fast harness). What fails is the AMR ever reaching the pick cell so that the coordinator publishes `/cell/start_pick`. Drilling in:

### 4.1 The symptom

`coordinator.log` (filtered, deadlock noise removed):

```
[INFO] fleet_coordinator: fleet_coordinator up — 2 AMRs, pick_cell at (16.00, 15.00)
[INFO] fleet_coordinator: enqueued order o1 at (4.00, 8.00)
[WARN] DEADLOCK detected: {'amr_0', 'amr_1'}  (every 1 s, forever)
[INFO] fleet_coordinator: enqueued order o1 at (4.00, 8.00)  (47 s later — see §4.5)
```

`pure_pursuit.log`, `amr_0` specifically:

```
[INFO] [amr_0.pure_pursuit_driver]: pure_pursuit_driver ready — action /amr_0/navigate_to_pose cmd_vel→cmd_vel frames=map↔base_link
[INFO] [amr_0.pure_pursuit_driver]: goal received: (4.00, 8.00) in map
[WARN] TF lookup map->base_link failed: "base_link" passed to lookupTransform argument source_frame does not exist.
[WARN] TF lookup map->base_link failed: ...  (every 2 s, forever)
```

The pp_driver action server is up, the coordinator's goal arrives, but the driver cannot resolve `(map → base_link)` to compute the next velocity command — so no `/cmd_vel` is ever published, and Carter never moves.

### 4.2 What we expect

Each AMR group launched via `wdt_pure_pursuit/launch/multi_amr.launch.py` under `PushRosNamespace(amr_X)` and `remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]` should produce, *within the `/amr_X/` namespace*:

```
/amr_X/tf_static  →  map → odom        (from static_transform_publisher in the launch)
/amr_X/tf         →  odom → base_link  (from Carter's USD OmniGraph)
```

So `lookup_transform("map", "base_link")` on a buffer listening to `/amr_X/{tf,tf_static}` (post-remap, post-namespace) should resolve. The `map → odom` static TF is set to each AMR's spawn pose (so `map` is shared "world", `odom` is the AMR's spawn-locked frame).

### 4.3 What we see

The static TF publisher does publish `map → odom` to `/amr_0/tf_static` (confirmed by `ros2 topic echo /amr_0/tf_static` returning the expected message).

But `/amr_0/tf` is **empty** — `ros2 topic echo /amr_0/tf --once` hangs and then `--no-arr` reports no message. That matches the pp_driver warning: `base_link` is missing because Carter's dynamic TF chain (`odom → base_link → chassis_link → …`) never reaches the pp_driver's buffer.

### 4.4 The M2 precedent — same symptom, different root cause

The M2 multi-AMR smoke (commit `9c52ae8`, "fix(m2): pure-pursuit fleet smoke green on Quebec (6/6 SUCCEEDED)") hit *exactly this symptom*. Two-line summary of the fix:

> `wdt_vast/sim_fleet.py`: drop the redundant `_namespace_subtree` loop. `spawn_amr_fleet` already calls `_namespace_subtree` per AMR inside. Calling it AGAIN here double-prepended pattern-2 OG namespace constants (`amr_0` → `amr_0/amr_0`), routing /tf to a 4th-level path no subscriber sees. Symptom: `fleet_namespaced n_per_amr=[42, 42, ...]` matched single-AMR counts (pattern-1 SET is idempotent), but `/amr_0/tf` was REGISTERED yet EMPTY, so all 6 pp_drivers got "TF lookup: base_link does not exist". Inverse of `sim_carter_single`'s bug (gotcha #18) — that helper had to ADD the call; this one had to REMOVE it.

We removed the double call back then. `run_scenario.py` today calls `spawn_amr_fleet(world, poses)` exactly once and never calls `_namespace_subtree` directly. The M2 smoke is reproducibly green with this code. **But the M5 smoke on the same code shows the same symptom.**

What's different between an M2 multi-AMR smoke and an M5 end-to-end smoke?

- M2 hard-codes 6 AMRs at the small-layout grid; M5 reads `fleet_size: 2` from `scenarios/smoke.yaml`
- M2 sends `/amr_X/navigate_to_pose` goals directly from a Python test harness (no coordinator); M5 routes through `fleet_coordinator`
- M5 spawns extra entities — a pick table, a pick cube (DynamicCuboid), cell lighting, a synthetic-camera OG, a `world → panda_link0` static TF, the move_group launch, the pick_cell_orchestrator, the `/world/cube_pose` publisher
- M5 calls Isaac Sim's `sim_cell_camera.py` which adds a `IsaacCreateRenderProduct` + viewport. This used to crash on `width`/`height` args (gotcha #27) and was fixed in v13
- multi_amr.launch.py creates 6 pp_drivers regardless of `fleet_size`. With `fleet_size=2`, pp_drivers 2..5 are running with no AMR to control. (We doubt this causes the regression, but flag it for completeness.)

### 4.5 Mystery: the duplicate "enqueued order o1"

The coordinator log shows `enqueued order o1 at (4.00, 8.00)` twice, 47 seconds apart. `run_scenario.py` calls `ros2 topic pub --once /orders/enqueue …` exactly once (the inner-while increments `next_order_idx` immediately). Possible explanations:

- DDS late-join QoS: the subscriber didn't exist when the publisher fired; the publisher re-delivers (we use default QoS — RELIABLE + KEEP_LAST 10).
- The pub subprocess gets re-spawned. The order loop calls `subprocess.Popen` without `wait`, and the outer while runs at sim time, not wall.
- A separate ros2 topic pub server holds the message.

This duplicate may be related to the navigation failure (e.g., the coordinator's first `_send_nav_goal` to `amr_0` silently times out at the action layer, then a second pass tries `amr_1`). Both AMRs hit the same TF problem.

### 4.6 The coordinator-vs-AMR-pose mismatch

Separately, the coordinator does `self._tf_buffer.lookup_transform("map", f"{a}/base_link", Time())` on a *global* TF listener (no namespace). Since the AMRs' TF is published to `/amr_X/tf` (namespaced), this lookup *will always fail*, and `self._poses[a]` stays at `(0.0, 0.0)`. The deadlock monitor sees both robots stuck at `(0, 0)` for 5+ s and fires DEADLOCK (false positive).

This second issue is a real coordinator bug independent of the pp_driver TF issue, but the v7 smoke ("amr_0 arrived at shelf for order o1") suggests the coordinator was at least *capable* of dispatching goals before — so the bug doesn't completely block, just produces a noisy log and a wrong deadlock flag.

## 5. What we've already tried / learned

### 5.1 Kit Python 3.11 vs Humble rclpy 3.10 mismatch (gotcha #18)

The plan's original `sim_world_pose_publisher.py` was supposed to run inside `/isaac-sim/python.sh` (kit's 3.11) so it could read the cube prim's worldspace transform directly. We tested this on Spain:

```bash
/isaac-sim/python.sh -c "import rclpy"
# ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
# The C extension '/opt/ros/humble/lib/python3.10/site-packages/_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so' isn't present
```

ROS2 Humble's `rclpy` only ships a `cpython-310` binary. Kit's 3.11 cannot import rclpy. We refactored `sim_world_pose_publisher.py` to a system-python (3.10) static publisher with cube spawn coords passed as CLI args from `run_scenario.py`. This works (the topic is publishing the correct pose; orchestrator's `GroundTruthPoseSource` receives it). Documented in `commit 178883e`.

### 5.2 Python dependencies missing on a fresh-bootstrap instance

After our `bootstrap_phase{1,2}.sh` runs (ROS2 Humble + Nav2 + MoveIt2 + Franka), the coordinator subprocess crashed with `ModuleNotFoundError: No module named 'scipy'`. We previously hid this by always running `install_foundationpose.sh` which pins scipy + numpy + lxml etc. Without FP install, the system Python is bare.

Fix: `apt install python3-pip && pip install scipy==1.11.4 pydantic pandas networkx`. After this, the coordinator boots cleanly and we see the TF symptom.

### 5.3 Known Carter-LIDAR issue (gotcha #5)

`Nova_Carter_ROS.usd`'s pre-baked OmniGraph registers `/amr_0/front_3d_lidar/lidar_points` but `ros2 topic hz` returns zero messages — the LIDAR sensor needs an additional `attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")` activation step we haven't wired in. This is why we run pure-pursuit instead of Nav2 (Nav2's DWB scores every trajectory as obstructed without LIDAR). Pure-pursuit doesn't need LIDAR — just `map → base_link`. So this is *upstream* of the current TF problem.

### 5.4 24 cataloged gotchas

`docs/gotchas.md` (and the memory file `feedback-nav2-isaac-sim-gotchas.md`) lists 34 specific Isaac Sim + ROS2 integration gotchas we've hit and fixed. Of note for the current consultation:

- #1 `RewrittenYaml` mandatory with `PushRosNamespace` for nested params
- #13 `tf2_ros::TransformListener` and `StaticTransformBroadcaster` *hardcode* absolute `/tf` and `/tf_static`. With `PushRosNamespace`, you must explicitly remap `(/tf, tf)` and `(/tf_static, tf_static)` on every node that uses tf2. Without this, the namespaced launch publishes globally, and the per-AMR pp_driver listens locally — they never connect.
- #14 `robot_state_publisher` needs `ParameterValue(Command([...]), value_type=str)` wrapping the xacro Command — newer launch_ros tries to YAML-parse all params unless explicitly told the value is a string. Worked on Romania driver 570.211, broke on Quebec driver 590.48.01. Could the same kind of driver-dependent quirk be in play here on Spain (570.195.03)?
- #17 `UnsupportedTypeSupport: nav2_msgs__srv__dynamic_edges`: Isaac Sim 5.0 bundles newer ROS2 message libs at `/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib`. Subprocesses inheriting this `LD_LIBRARY_PATH` crash when dlopening Humble's apt-installed message types. `run_scenario.py` scrubs `/isaac-sim/` from subprocess `LD_LIBRARY_PATH` (keeps it for the parent sim).
- #18 Kit Python 3.11 vs Humble rclpy 3.10 (§5.1 above).
- #20 `ROS_DOMAIN_ID` silos subprocesses from the sim's bridge — don't set it; Isaac Sim's bridge uses whatever the SimulationApp's domain is (default 0).
- #21 Carter's URDF achieved speed is ~42% of commanded `max_linear` in render+play sim. With `max_linear=0.5`, wall speed is ~0.05 m/s.
- #22 Sim/wall ratio on RTX 3090 with multi-AMR + render=True is ~0.23 (60 s sim ≈ 261 s wall).

## 6. Our current debugging hypotheses

In rough order of plausibility:

**Hypothesis A: Carter's USD OG namespacing has a side effect on frame IDs.**
Setting `inputs:nodeNamespace = "amr_0"` on a `ROS2PublishTransformTree` OG node might prefix the published *frame IDs* (so the published TF is `amr_0/odom → amr_0/base_link`), not just the topic. If true, the pp_driver lookup of `(map, base_link)` fails because the actual published source frame is `amr_0/base_link`. Counter-evidence: M2 multi smoke worked with the same setup and `frames=map↔base_link` in the pp_driver. Either M2 was using a different USD revision, or the frame IDs are NOT prefixed and we have some other bug.

**Hypothesis B: A pattern-2 namespacing constant got over-applied or under-applied.**
The `_namespace_subtree` helper handles two patterns: (1) direct `inputs:nodeNamespace` on publisher OGs, and (2) a "namespace" / "node_namespace" constant OG node that other publishers reference. Pattern 2 is the M2 commit's culprit when double-applied. Could *some other code path* in `run_scenario.py` (cell-camera spawn, lighting, pick-table spawn, the synthetic `/cell/cam/*` rig) be touching Carter's OG namespacing again?

**Hypothesis C: An OG load-order issue.**
We spawn the fleet, then spawn the table + cube + lighting + cell camera. Maybe the additional spawns invalidate the per-AMR OG namespacing somehow (USD authoring order matters for some Isaac Sim extensions).

**Hypothesis D: A QoS / DDS configuration drift.**
The pp_driver's TF listener uses RELIABLE QoS by default. Carter's OG publishes /tf with... we'd need to check. Maybe a mismatch.

**Hypothesis E: pp_driver's TransformListener subscribes to `/tf` and `/tf_static`, but the launch's `remappings=[("/tf", "tf"), …]` only applies to the Node directive, not to the C++ tf2_ros library that hardcodes the absolute topic path internally.** Gotcha #13 in our list says exactly this — but multi_amr.launch.py *does* apply the remap. Worth re-verifying it actually takes effect for the listener (vs only the publisher).

## 7. Plan on potential solutions to try

These are not prioritized — we want the expert's view first.

### Option 1: Verify what Carter publishes — empirical first step

Boot the sim, let Carter's USD load, and inspect on a separate SSH session:

```bash
ros2 topic echo /amr_0/tf --once
ros2 topic list | grep -E "amr_0|tf"
ros2 topic info /amr_0/tf
ros2 topic info /amr_0/tf --verbose  # publisher/subscriber QoS
ros2 run tf2_ros tf2_echo map base_link --topic /amr_0/tf
```

If `/amr_0/tf` shows `child_frame_id: amr_0/base_link`, the OG is namespacing frame IDs → fix is to override either the pp_driver's `base_frame` parameter to `amr_0/base_link`, OR to unprefix the OG's frame IDs.

If `/amr_0/tf` is empty, Carter's OG is publishing somewhere else (likely `/tf` globally). Then we need to either remap on the publish side or have pp_driver listen to `/tf` globally.

### Option 2: Force-namespace pp_driver's `base_frame`

Modify `multi_amr.launch.py` to set `base_frame:=amr_{i}/base_link`. If Hypothesis A is correct, this would unblock pp_driver. But it would break the M2 multi smoke, which we *don't* want.

A better variant: make `base_frame` a per-AMR parameter that defaults to `"base_link"` but can be set to `"amr_0/base_link"` if needed. The downside is it's a fix specific to one OG configuration; we'd want to understand what's happening rather than band-aid.

### Option 3: Switch the coordinator's pose source

Stop relying on TF for AMR pose tracking. Subscribe to `/amr_X/odom` (Nova Carter publishes this topic via the same OG that handles TF) and update `self._poses[a]` from there. Pros: independent of the TF wiring mess. Cons: doesn't fix pp_driver's TF lookup; just fixes the false-positive DEADLOCK flag in the coordinator.

### Option 4: Add explicit `tf_remap` (or a small bridging node)

If Carter publishes to `/tf` globally with frame_id `amr_0/base_link`, but pp_driver wants to see it locally as `base_link`, we could:

- Run `tf2_ros tf_remap` (an actual ROS2 tool) per AMR to rewrite frame_ids: `amr_0/base_link → base_link` published into `/amr_0/tf`
- Or write a 30-line Python relay

### Option 5: Drop the per-AMR namespacing of pp_driver

Run a single pp_driver per AMR but at *global* TF level using `/amr_X/base_link` as the base frame. Equivalent to option 2 but at the launch level (no PushRosNamespace, just per-AMR launch args).

### Option 6: Bypass AMR navigation entirely for M5 acceptance

Hardcode "the AMR is at the cell" in run_scenario.py: skip the coordinator's NAV_TO_SHELF/NAV_TO_CELL legs, just publish `/cell/start_pick` directly after the sim warms up. This sidesteps the nav problem for M5 acceptance but doesn't fix it — the M7+ ablation NEEDS real nav for `pick_success_rate` to be meaningful.

For a portfolio demo: option 6 ships M5 quickly but lies about authenticity (which the user has already vetoed for the Phase 1 demo video).

### Option 7: Switch to Nav2 with synthetic LIDAR

Nav2 needs LIDAR for the costmap. Carter's LIDAR doesn't publish (gotcha #5). We could:
- Synthesize LIDAR from depth → laserscan via `pointcloud_to_laserscan` (apt-installed)
- Or fix Carter's LIDAR per the canonical Isaac Sim example (gotcha #5 explicitly cites the fix code)

This is Phase 3 territory; doesn't help M5 in the near-term.

### Option 8: Roll back to the v7 working state

The memory says M5 v7 reached "amr_0 arrived at shelf for order o1" after a fresh-container boot, then v8-v10 broke. We hypothesized stale Isaac Sim bridge/OG state. The next session on a fresh instance might have reverted to v7's working state, but the redesign happened mid-debug so we can't easily isolate.

We could `git checkout` the v7 state, apply just the orchestrator+worker thread part of the redesign, and rerun. Risky — the redesign touches `pick_cell_orchestrator.py` and the spawn coords / lighting code; bisecting would take real time.

## 8. Specific questions for the expert

### 8.1 Isaac Sim ROS2 bridge

1. **What does `inputs:nodeNamespace` on `ROS2PublishTransformTree` actually do in Isaac Sim 5.0?** Does it prefix frame IDs, or only the topic? If frame IDs, is there a way to suppress that?
2. **For a multi-namespaced Nova Carter setup, what's the canonical TF tree?** Is each Carter expected to publish `amr_X/odom → amr_X/base_link → …`, with a global `map → amr_X/odom` shared static TF? Or `odom → base_link` raw, with each in its own `/amr_X/tf` topic?
3. **Why does `/amr_0/tf` show up in `ros2 topic list` but be empty?** Are there race-condition scenarios where the OG is partially initialized?
4. **Is there a built-in Isaac Sim helper for multi-AMR namespacing that we should be using instead of our custom `_namespace_subtree`?**

### 8.2 ROS2 / TF / tf2_ros internals

5. **Does `Node(remappings=[("/tf", "tf"), ("/tf_static", "tf_static")])` correctly route the absolute `/tf` topic that `tf2_ros::TransformListener` hardcodes?** We believe yes (gotcha #13), but symptoms suggest otherwise. Is there a way to verify the listener actually subscribes to `/amr_0/tf` and not `/tf`?
6. **For multi-namespaced Nav2-style setups, what's the proven pattern?** `nav2_bringup`'s `multirobot` example uses RewrittenYaml + PushRosNamespace; we mirror that. Anything we might be missing?
7. **`ros2 topic pub --once` retry semantics:** does it ever re-publish? Why might the same `--once` invocation result in two received messages 47 s apart?

### 8.3 Architecture

8. **Is the worker-thread + own-executor pattern (§3.2) the right answer to the rclpy callback-deadlock problem?** Or would `ReentrantCallbackGroup` + `MultiThreadedExecutor` (which we explicitly avoided) actually have worked if we'd been careful?
9. **For a portfolio project, is "real Nav2 + pure-pursuit fallback for control + plan_only MoveIt + GT-pose acceptance + FP-pose stretch" a defensible scope, or are we drawing the line in the wrong place?**

### 8.4 Pragmatic

10. **Given our state (fast harness green, full smoke blocked on a TF mystery, vast.ai-renting-by-the-hour budget), which of Options 1-7 above would you try first?**
11. **Is there an obvious "is your fleet TF wired correctly?" diagnostic command sequence we should be running before re-launching the full smoke?**
12. **Any pattern we should be using for the kit-Python 3.11 vs Humble rclpy 3.10 problem beyond "shell out to /usr/bin/python3"?** This was the trigger for refactoring `sim_world_pose_publisher.py` (commit `178883e`).

## 9. Concrete artifacts

- Spec: `docs/superpowers/specs/2026-05-16-pick-chain-redesign-design.md`
- Plan: `docs/superpowers/plans/2026-05-16-pick-chain-redesign.md`
- Gotchas log: `docs/gotchas.md`
- Pick chain code:
  - `manipulation/pose_source.py` (5 unit tests pass)
  - `manipulation/pick_worker.py` (6 unit tests pass)
  - `manipulation/motion_planning.py` (ArmPlanner; explicit executor kwarg)
  - `ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py`
  - `wdt_vast/sim_world_pose_publisher.py`
- Run-orchestration code:
  - `wdt_vast/run_scenario.py` (parent kit process)
  - `wdt_vast/bootstrap_phase{1,2}.sh`
  - `wdt_vast/install_foundationpose.sh`
- Multi-AMR launches:
  - `ros2_ws/src/wdt_pure_pursuit/launch/multi_amr.launch.py`
  - `sim/multi_robot.py` (`spawn_amr_fleet` + `_namespace_subtree`)
- Last full-smoke output (Spain 33898889, 2026-05-17 ~03:00 UTC):
  - `/tmp/m5_smoke_v22b_gt/coordinator.log`
  - `/tmp/m5_smoke_v22b_gt/pure_pursuit.log`
  - `/tmp/m5_smoke_v22b_gt/pick_orch.log`
  - `/tmp/m5_smoke_v22b_gt.log` (sim stdout/stderr, 50 KB)
  - `/tmp/m5_smoke_v22b_gt/metrics.json`

The Spain instance is stopped (no charges) but can be resumed in <2 min with `vastai start instance 36898889` if the expert wants live access for diagnostics.

## 10. Where we'd like the conversation to land

After this consultation we want to either:

1. **Have a single concrete TF fix to try**, with a clear acceptance test (e.g. "amr_0 drives to (4, 8) and triggers `/cell/start_pick`"). If we can get that, M5 acceptance ships within a session — the orchestrator and worker are proven.
2. **Pivot scope to "Option 6"** with the expert's blessing: ship M5 by publishing `/cell/start_pick` directly from `run_scenario.py` after a sim-warm-up delay, document AMR navigation as Phase 3 work. Defensible because (a) the orchestrator's pick chain is the architecturally novel piece, (b) AMR + Nav2 is a well-trodden third-party stack, (c) our portfolio target is the manipulation cell + orchestrator design, not the AMR autonomy.
3. **Get a structured TF diagnostic checklist** so we (or any future contributor) can debug this class of bugs in <30 minutes next time, rather than 9 iterations.

Thank you. The repo is public and welcome to clone; the bootstrap chain on a fresh vast.ai box is ~25 minutes if all scripts go to plan.

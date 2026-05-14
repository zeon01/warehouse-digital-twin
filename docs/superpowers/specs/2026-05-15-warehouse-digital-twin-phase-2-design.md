# Warehouse Digital Twin — Phase 2 Design (Close Integration Gaps + Planner Ablation)

**Date:** 2026-05-15
**Status:** Draft (pending user review)
**Owner:** Saad Sharif Ahmed
**Phase 1 spec:** `docs/superpowers/specs/2026-05-14-warehouse-digital-twin-design.md`
**Phase 1 plan:** `docs/superpowers/plans/2026-05-14-warehouse-digital-twin-phase-1.md`
**Release target:** `v0.2.0`

---

## 1. Overview

Phase 1 shipped `v0.1.0` with the structural skeleton of a warehouse digital twin: 6 namespaced Nova Carter AMRs, 1 Franka Panda, ROS2 bridge, fleet coordinator with Hungarian + CBS planning, manipulation pipeline scaffolding, and a 64-order steady_state scenario. But two integration gaps prevent the demo from producing real numbers:

1. **Nav2 was not wired** — the coordinator's `NavigateToPose` action client has no action server backing it, so AMRs report navigation goals as "not ready" and never move autonomously beyond direct `cmd_vel` smoke tests.
2. **Manipulation models were not installed** — `ManipulationPipeline._lazy_load()` raises `ImportError` for FoundationPose / AnyGrasp / MoveIt2, so `run_scenario` catches the error and sets `manip=None`, leaving `pick_success_rate=0` and `orders_completed=0`.

Phase 2 closes both gaps and uses the now-real system to run a **planner ablation study**: the same `steady_state.yaml` scenario, executed across three planner configurations (`greedy_greedy` / `hungarian_greedy` / `hungarian_cbs`, see §6) with five random seeds each, producing the first defensible portfolio numbers ("`hungarian_cbs` reduces deadlocks by X%, p<0.05").

### Commercial framing

Phase 1 set up the architecture. Phase 2 produces the *measurement* — and measurement is what KION/Cyngn-style commercial integrators are actually paid for. A digital twin that closes its own loop and reports planner-comparison metrics is portfolio-defensible in a way a structural skeleton is not.

---

## 2. Goals and Non-Goals

### Goals
- Replay Phase 1's 64-order `steady_state.yaml` scenario end-to-end with **real** Nav2 (planner + AMCL + costmap + controller) and **real** manipulation (MoveIt2 motion + FoundationPose perception).
- Report headline metrics with non-zero, defensible numbers: orders/hr, mean cycle time, deadlocks/min, pick success rate.
- Run an ablation across three planner configs (`greedy_greedy`, `hungarian_greedy`, `hungarian_cbs`, defined in §6) × five seeds × 64-order steady_state = 15 runs. Report mean ± std per metric.
- Ship `v0.2.0` with: real demo video, `docs/results-phase-2.md`, ablation plots, updated README.

### Non-goals (deferred to Phase 3)
- Scale-up to 12–20 AMRs / 50×50 m warehouse / live web dashboard.
- AnyGrasp integration (license + CUDA cost not worth it; deterministic top-down grasp is sufficient for Phase 2 metrics).
- Sim-to-real bridge.
- Custom-trained perception or control models.

---

## 3. Decisions log

| Decision | Choice | Rationale |
|---|---|---|
| Phase 2 scope | A (close integration gaps) + B (planner ablation) | User wanted "all three" originally; bundled A+B into one phase, C (scale-up) deferred to Phase 3 to keep cohesion. A and B share the same scenario file so they amortize infrastructure work. |
| Success bar | Replay 64-order steady_state with real manipulation | Cleanest before/after narrative: Phase 1 produced zeros, Phase 2 produces real numbers on the *same* scenario. Naturally feeds the ablation (B = run that scenario across planners). |
| Nav2 scope | Full stack (map + AMCL + costmap + planner + controller + lifecycle) | User chose authenticity over scope. Real Nav2 is portfolio-credible; minimal Nav2 was the recommendation but user vetoed. |
| Manipulation models | MoveIt2 + FoundationPose real, **deterministic top-down grasp** (no AnyGrasp) | AnyGrasp's research-use license registration is a multi-day blocker; warehouse SKUs are constrained enough that a deterministic top-down grasp at the estimated pose is defensible. |
| Ablation methodology | N=5 seeds × 3 planner configs = 15 runs total | Standard ML-paper methodology, ~7.5 hr vast.ai compute (~$3), enough for mean ± std and p-value claims. The three configs span the task-allocation × path-planning grid — see §6. |
| Map generation | Pre-bake PGM + YAML offline from procedural USD | Fast startup, deterministic, no runtime extraction. Committed to repo. |
| Manipulation failure handling | Pipeline-internal grasp retries (K=3, existing); order-level: one pipeline attempt, on failure mark order FAILED, AMR continues | Existing pipeline already does bounded grasp retries. Coarser order-level retries would confound the ablation by introducing AMR-blocking time that depends on perception quality, not planner quality. |
| FoundationPose distribution | Pre-compile CUDA wheels into a tarball on Modal Volume; vast.ai pulls on first run | CUDA op compilation is a known fragile step; pre-baking sidesteps driver/CUDA mismatch loops during dev. |

---

## 4. System architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│ vast.ai RTX A5000 (Romania) — Isaac Sim 5.0 + ROS2 Humble                │
│                                                                          │
│ ┌─────────────────┐  ┌────────────────────────────────┐  ┌────────────┐ │
│ │ Isaac Sim       │  │ Per-AMR Nav2 (×6)              │  │ MoveIt2    │ │
│ │  • 6× Carter    │  │  • map_server (pre-baked PGM)  │  │ move_group │ │
│ │  • 1× Franka    │◄─┤  • AMCL (LIDAR)                │  │  on Panda  │ │
│ │  • pick cell    │  │  • planner_server (NavfnPlanner)│ │            │ │
│ │  • shelves      │  │  • controller_server (DWB)     │  │            │ │
│ │  • RTX render   │  │  • bt_navigator + lifecycle    │  │            │ │
│ └────────┬────────┘  └────────────┬───────────────────┘  └──────┬─────┘ │
│          │                        │                              │       │
│          │ /tf, /scan, /cam       │ /robot_N/navigate_to_pose    │       │
│          │                        │                              │       │
│          ▼                        ▼                              ▼       │
│ ┌─────────────────────────────────────────────────────────────────────┐ │
│ │ fleet_coordinator (custom)                                          │ │
│ │  • config = (allocator, path_planner), e.g. (hungarian, cbs)       │ │
│ │  • drives state machine: ASSIGNED → NAVIGATING → AT_CELL → PICKING │ │
│ │  • triggers /cell/start_pick on AMR arrival                        │ │
│ │  • subscribes /cell/pick_result to advance/fail orders             │ │
│ └─────────────────────────────────────────────────────────────────────┘ │
│          ▲                                            ▲                  │
│          │ /cell/start_pick, /cell/pick_result        │                  │
│          ▼                                            │                  │
│ ┌─────────────────────────────────────────┐  ┌───────┴──────────────┐   │
│ │ pick_cell_orchestrator (new)            │  │ MetricsRecorder      │   │
│ │  • subscribes /cell/start_pick          │  │  • subscribes events │   │
│ │  • captures /cell/cam/{rgb,depth}       │  │  • writes CSV + JSON │   │
│ │  • runs ManipulationPipeline.pick():    │  │  • assembles MP4     │   │
│ │     FoundationPose → TopDownGrasp →     │  └──────────────────────┘   │
│ │     MoveIt2 plan+execute                │                              │
│ │  • publishes /cell/pick_result          │                              │
│ └─────────────────────────────────────────┘                              │
└──────────────────────────────────────────────────────────────────────────┘
```

### 4.1 Map generation (offline, in `warehouse.generators`)
Extend `build_scene` to emit a 2D occupancy grid as PGM + YAML. Walk the procedural USD (shelves, columns, pick cell, drop-off), rasterize obstacles into a top-down grid at 5 cm/px. Outputs `warehouse/maps/{layout_name}.{pgm,yaml}`, committed to the repo. No runtime extraction.

### 4.2 URDF/SRDF deltas (`ros2_ws/src/`)
- **`wdt_carter_description`** (new): URDF exposing Carter's existing LIDAR scan (`/robot_N/scan`) and TF chain to ROS2. Isaac Sim's bundled Carter already has the sensors; this package just provides ROS2's view of the robot footprint.
- **`wdt_franka_description`** (new): URDF + SRDF based on public `franka_description` (Panda variant). SRDF defines move_groups `panda_arm` and `panda_hand`.

### 4.3 Nav2 stack (`ros2_ws/src/wdt_nav2_bringup`, new)
Per-AMR namespaced launch file. Components: `map_server` (PGM-backed), `amcl` (LIDAR-based), `planner_server` (NavfnPlanner, 5 cm grid), `controller_server` (DWB), `bt_navigator`, `behavior_server`, `lifecycle_manager`. Costmap config: 5 cm resolution, inflation radius = Carter footprint + 10 cm. One Nav2 stack per AMR namespace (`/robot_N/...`). Brings up via a single composite launch that takes `num_robots` as a parameter.

### 4.4 Manipulation (existing `manipulation/` + new `wdt_manipulation_bringup`)
- `manipulation/pose_estimation.py` — `_lazy_load()` becomes real: pulls FoundationPose weights from Modal Volume on first call, compiles CUDA ops (or extracts pre-built wheels), caches in `models/foundationpose/`. Public interface unchanged.
- `manipulation/grasping.py` — add a new `TopDownGrasp` class alongside the existing `GraspGenerator` (the unused AnyGrasp wrapper). `TopDownGrasp.propose(depth, camera_K)` returns a single `GraspCandidate` at the estimated pose translation: gripper Z-axis aligned to world `-z`, +5 cm standoff above the object's top surface. Duck-typed — same `.propose()` signature as `GraspGenerator`, no shared base class needed.
- `manipulation/motion_planning.py` — existing `MoveIt2Planner` wrapper now talks to a real `move_group` via `moveit_py`.
- **`pick_cell_orchestrator`** (new ROS2 node in `wdt_manipulation_bringup`): subscribes `/cell/start_pick`, captures `/cell/cam/{rgb,depth}`, instantiates `ManipulationPipeline(FoundationPose, TopDownGrasp, MoveIt2Planner)`, calls `.pick()`, publishes `/cell/pick_result`.

### 4.5 Coordinator state machine (`coordinator/`)
The existing state enum and order lifecycle are unchanged. Wiring deltas:
- `NavigateToPose` action client now receives real results from Nav2 (success/abort/cancel); on `AT_CELL` (arrival success), publish `/cell/start_pick` and transition to `PICKING`.
- Subscribe `/cell/pick_result`; on `success=True`, send `NavigateToPose` to the order's drop-off; on `success=False`, mark order `FAILED`, log `failure_reason`, return AMR to assignment pool.
- Two new launch args expose the ablation knobs: `coordinator.allocator ∈ {greedy, hungarian}` (task allocation, dispatches to `assignment.hungarian_assign` or a new nearest-AMR helper) and `coordinator.path_planner ∈ {greedy, cbs}` (path planning, resolved via the existing `strategy.get_planner()` registry — CBS gets registered as `"cbs"` in `coordinator/strategy.py` as part of M8 prep).

---

## 5. End-to-end data flow (one order, green path)

```
1. Order generator        → /fleet/orders/new {order_id, sku, drop_off}
2. fleet_coordinator      ← consumes /fleet/orders/new
                          → planner.assign() returns AMR_K
                          → publishes /robot_K/navigate_to_pose goal = pick_cell
3. AMR_K Nav2             ← AMCL localizes, planner_server plans, DWB executes
                          → action result success at AT_CELL
4. fleet_coordinator      ← action result success
                          → publishes /cell/start_pick {order_id}
                          → state PICKING
5. pick_cell_orchestrator ← /cell/start_pick
                          → captures /cell/cam/{rgb,depth}
                          → ManipulationPipeline.pick():
                              FoundationPose → 6-DoF pose
                              TopDownGrasp → 1 candidate (top-down, 5 cm standoff)
                              MoveIt2 → plan + execute → PickResult
                          → publishes /cell/pick_result {success, attempts, cycle_time_s}
6. fleet_coordinator      ← /cell/pick_result
                          → on success: NavigateToPose to drop_off
                          → on failure: mark order FAILED, AMR_K returns to assignment pool
7. AMR_K Nav2             → arrives at drop_off → action success
8. fleet_coordinator      → publishes /fleet/events {order_complete, order_id, cycle_time}
9. MetricsRecorder        ← /fleet/events, /cell/pick_result → CSV + JSON
```

Every topic in this flow exists in the Phase 1 design; Phase 2's job is to make each step return real results instead of stubs.

---

## 6. Ablation methodology (B)

**Independent variable.** A planner *config* is a pair (`allocator`, `path_planner`). Two existing modules back this:
- Task allocation lives in `coordinator/assignment.py` (`hungarian_assign`) and a new `nearest_assign` helper added in the ablation prep step.
- Path planning lives in `coordinator/strategy.py` (`GreedyPlanner` registered today; CBS wrapper from `coordinator/cbs.py` gets registered as `"cbs"` during M8 prep).

The three configs we run, chosen to isolate each axis without exploding the run count:

| Config | `allocator` | `path_planner` | Question it answers |
|---|---|---|---|
| `greedy_greedy` | nearest-AMR | straight-line | Baseline. How much do we even need either layer? |
| `hungarian_greedy` | Hungarian optimal | straight-line | Does optimal *task allocation* reduce cycle time / deadlocks on its own? |
| `hungarian_cbs` | Hungarian optimal | CBS conflict-free | Does *path conflict resolution* further reduce deadlocks given optimal allocation? (This is the Phase 1 production config.) |

The fourth cell (`greedy_greedy` vs `greedy_cbs`) is omitted: with greedy allocation, conflict-resolution gains are confounded by suboptimal AMR-to-order pairing, so the comparison is hard to interpret. Three configs cover the headline questions in 15 runs instead of 20.

**Random seed.** Controls (a) order arrival times (Poisson process with the scenario's λ) and (b) which SKU each order requests. Seeds: `{42, 43, 44, 45, 46}`.

**Scenario.** Unchanged from Phase 1's `scenarios/steady_state.yaml` — 64 orders, 6 Carters, 1 Franka, fixed warehouse layout.

**Per-run output.** `runs/{config}/{seed}/{metrics.json, events.log, replicator/rgb_*.png}` where `config ∈ {greedy_greedy, hungarian_greedy, hungarian_cbs}`.

**Cross-run aggregation.** New `metrics/aggregate.py` reads all 15 `metrics.json` files and produces:
- `docs/results-phase-2.md` — markdown table with mean ± std per config per metric, plus p-values from a two-sided t-test against the `greedy_greedy` baseline.
- `docs/images/ablation/{throughput,cycle_time,deadlocks,pick_rate}.png` — matplotlib bar charts with error bars.

**Headline metrics for README + release notes:**
- Orders completed per hour (sim time)
- Mean cycle time per order (s)
- Deadlocks per minute
- Pick success rate (fraction of `/cell/pick_result` events with `success=True`)

**Run orchestration.** New `wdt_vast/run_ablation.py` script on vast.ai loops over `(config, seed)`, invokes the existing `wdt_vast/run_scenario.py` entrypoint with `--allocator` and `--path-planner` overrides, copies outputs back to the local Mac via rsync. Approximately 30 min wall-clock per run × 15 = 7.5 hr total. Executed as one long session on vast.ai with `tee` + a tailed remote log per established logging discipline.

---

## 7. Testing strategy

| Tier | Where | Runtime | What it covers |
|---|---|---|---|
| Unit | Mac, in CI | <30 s | Map exporter rasterization correctness; `TopDownGrasp` returns sane pose; `pick_cell_orchestrator` callback dispatch (ROS2 + CUDA mocked); ablation aggregator math. |
| Integration | vast.ai, manual | ~5 min | `wdt_nav2_bringup` single-AMR launch + navigate-to-pose smoke; MoveIt2 `plan_to_pose` with mocked perception. |
| Smoke | vast.ai, manual | ~2 min sim, ~5 min wall | `smoke.yaml` (1 order, 1 AMR, real manipulation, ≤2 min sim time) — proves end-to-end glue. |
| Acceptance | vast.ai, batched | ~7.5 hr | The 15-run ablation. CI doesn't gate this; ship-gates it via `make ablation-acceptance` checklist in `docs/results-phase-2.md`. |

CI runs only the unit tier — integration + smoke + acceptance live on vast.ai and are gated by milestones, not every PR.

---

## 8. Risks and mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| FoundationPose CUDA ops fail to build on vast.ai (driver/CUDA mismatch) | High | Pre-bake compiled wheels into a tarball on Modal Volume; first vast.ai run pulls + extracts rather than compiling. |
| AMCL diverges on the procedural warehouse (sparse LIDAR features, repetitive shelf geometry) | Medium | Seed AMCL with ground-truth pose at startup; tune particle count + sensor σ; if divergence detected during smoke, fall back to ground-truth pose publisher and document AMCL as a stretch goal. |
| MoveIt2 collision-check timeout on the large warehouse environment | Medium | Restrict the move_group planning scene to a 2 m bubble around the pick cell. |
| 30 min/run wall-clock balloons to 60+ min, blowing the 7.5 hr budget for 15 runs | Medium | Time-box first real-manipulation run; if it exceeds 45 min, reduce ablation scenario to 32 orders and document as a known limitation in `results-phase-2.md`. |
| Per-AMR Nav2 namespacing issues (Nav2's lifecycle isn't always namespace-clean) | Medium | Validate single-AMR Nav2 (M1) before scaling to 6 (M2). |
| Cross-AMR DDS discovery storms with 6 Nav2 stacks | Low | Configure CycloneDDS with a static discovery file if FastDDS misbehaves. |
| AnyGrasp gets resurrected mid-Phase-2 as "we should do it properly" | Low | Decision is logged in §3; deterministic top-down grasp is sufficient for Phase 2 metrics. Revisit in Phase 3. |

---

## 9. Milestones (build sequence, ~3 weeks)

| # | Milestone | Wall-clock | Closes with |
|---|---|---|---|
| M0 | Map generation + Carter URDF/ROS2 wiring | 1–2 days | PGM committed, single-AMR TF + scan visible in `rviz2` on vast.ai |
| M1 | Nav2 single-AMR bringup + navigate-to-pose smoke | 2–3 days | One Carter navigates to a hardcoded pose autonomously |
| M2 | Nav2 multi-AMR (6 Carters) | 1 day | 6 Carters spawn with namespaced Nav2 stacks; each can independently navigate |
| M3 | MoveIt2 + Franka URDF/SRDF, plan-to-pose with mocked perception | 1–2 days | Franka executes a hand-coded grasp trajectory |
| M4 | FoundationPose install + integration with `pose_estimation.py` | 2–3 days | Real `_lazy_load()` succeeds; pose estimation returns plausible 6-DoF poses on a test scene |
| M5 | `pick_cell_orchestrator` wiring + first end-to-end pick on `smoke.yaml` | 1–2 days | 1 order completes end-to-end with real Nav2 + real manipulation |
| M6 | Coordinator state machine wired to real action results | 1 day | State transitions driven by real Nav2 action results; no `not ready` logs |
| M7 | First full 64-order steady_state with real manipulation (`hungarian_cbs` only) | 1 day | `orders_completed > 0`, defensible headline numbers |
| M8 | Ablation runner + 15 runs + aggregator | 2 days | `docs/results-phase-2.md` populated with mean ± std table; ablation plots committed |
| M9 | Results writeup + `v0.2.0` release | 1 day | GitHub release created; README updated with Phase 2 numbers and video |

Each milestone closes with a commit + a green smoke run on vast.ai. M1, M5, M7 are the "show-and-tell" checkpoints — if any of those blows out, we re-plan rather than push through.

---

## 10. Out-of-scope explicit list

The following are *intentionally* deferred to Phase 3 and should not be added to Phase 2 scope without re-running the brainstorm:

- Scale-up to 12–20 AMRs / 50×50 m warehouse
- Live web dashboard / real-time visualization beyond Replicator capture
- AnyGrasp integration
- Custom-trained perception or grasp models
- Sim-to-real bridge or hardware-in-the-loop
- Multi-floor warehouse, charging-station modeling, dynamic obstacles
- Failure-injection / chaos suite
- Photorealism beyond Isaac Sim's default RTX path tracing

---

## 11. Success criteria for shipping `v0.2.0`

- ✅ All 9 milestones (M0–M9) closed and committed.
- ✅ `orders_completed > 0` on the 64-order steady_state with `hungarian_cbs` planner.
- ✅ 15-run ablation completed; `docs/results-phase-2.md` reports mean ± std + p-values for the four headline metrics.
- ✅ Real-manipulation demo video (≥60s, ≤90s) embedded in README and attached to GitHub release.
- ✅ Total Phase 2 spend ≤ $15 (within $56 remaining budget across both Modal accounts + vast.ai).
- ✅ `git tag v0.2.0` pushed, GitHub release created with assets: video, `results-phase-2.md`, `metrics.json` for each of the 15 runs, ablation plots.

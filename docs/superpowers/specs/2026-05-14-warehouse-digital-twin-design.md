# Warehouse Digital Twin with Fleet + Manipulation Cell — Design

**Date:** 2026-05-14
**Status:** Draft (pending user review)
**Owner:** Saad Sharif Ahmed
**Codename:** `isaac-sim` (working directory)

---

## 1. Overview

An open-source warehouse digital twin built on NVIDIA Isaac Sim that combines two commercial-grade subsystems in one demo:

1. **Fleet coordination** — a fleet of Nova Carter AMRs navigating a warehouse using ROS2 + Nav2, with a custom multi-agent coordinator handling task allocation and path conflict resolution.
2. **Manipulation cell** — a Franka Panda arm at a pick station that unloads totes using pre-trained vision-language-action components (FoundationPose for 6-DoF pose estimation, AnyGrasp for grasp synthesis, MoveIt2 for motion planning).

Both subsystems run concurrently inside a single Modal container (cloud GPU), publish observability to a metrics/video recorder, and produce a portfolio-ready demo + numbers.

### Commercial framing

This project intentionally mirrors the deployment architecture that **KION + Accenture + Siemens are shipping for GXO Logistics** and **Cyngn is using to validate warehouse autonomy before real-facility rollout**. Q1 2026 robotics funding sent **~70% of $2.26B to warehouse/industrial automation**, and the DHL Supply Chain report flagged the largest operational pain point as *site-specific validation before real deployment*. A digital-twin pipeline of this shape is exactly what those companies are paying integrators for.

## 2. Goals and Non-Goals

### Goals
- A working end-to-end warehouse digital twin runnable with one `modal run` command
- Combined fleet + manipulation demo on the same scene
- Industry-standard ROS2 + Nav2 + MoveIt2 stack (recognizable to recruiters)
- Zero model training in Phase 1 and Phase 2 (pre-trained perception models only)
- Reproducible, with metrics, video, and a portfolio-quality README
- Total compute cost under $60 across both Modal accounts

### Non-goals (Phase 1 + 2)
- Sim-to-real on hardware (no physical robot)
- Custom-trained perception or control models *(Phase 3 optional only)*
- Multi-floor, outdoor, or pedestrian-aware scenarios
- Production observability (Prometheus / Grafana)
- Photorealism beyond Isaac Sim's default RTX path tracing
- Real-time human teleop or VR interaction

## 3. Decisions Log

| Decision | Choice | Rationale |
|---|---|---|
| Project scope | Combined fleet + pick cell | User-selected: biggest commercial story, mapped to KION/Cyngn use case |
| Execution model | Sequential phases (1 → 2 → 3) | User-selected: lowest risk of abandonment |
| Container topology | Single Modal container | Avoids ROS2 inter-container networking pain |
| Per-robot navigation | Nav2 (off-the-shelf) | Industry-standard; hireable skill |
| Multi-agent coordination | Custom: Hungarian assignment + CBS | Standard 2-tier architecture; pluggable for ablations |
| Arm motion planning | MoveIt2 (not cuRobo) | Keeps stack uniformly ROS2; cuRobo is a Phase 3 swap |
| Manipulation perception | FoundationPose + AnyGrasp (pre-trained) | Zero-training in P1/P2; both ship as Isaac ROS packages |
| Compute platform | Modal (cloud GPU) | User has $60 credits; eliminates local hardware need |
| Primary GPU | L40S for runs, L4 for dev | Has RT cores, finishes 3–5× faster than L4 at 2.5× cost → time-to-result wins |
| ROS2 distribution | Humble (Ubuntu 22.04) | Matches Isaac Sim 5.x base image |
| Phase 3 (training) | Optional, deferred | Decided only if Phase 2 finishes with budget headroom |

## 4. Architecture

```
┌─────────────────────────── Modal Container (GPU) ──────────────────────────┐
│                                                                            │
│  ┌──── Isaac Sim (headless) ─────┐    ┌──── ROS2 Stack ──────────────┐     │
│  │ Warehouse USD scene           │◄───┤ Nav2 (per-AMR)               │     │
│  │  • 6 Nova Carter AMRs         │    │  • global + local planner    │     │
│  │  • 1 Franka pick cell         │    │  • costmaps, recovery        │     │
│  │  • tote bin + SKUs            │    ├──────────────────────────────┤     │
│  │  • overhead + side cameras    │    │ Fleet Coordinator (custom)   │     │
│  │                               │    │  • Hungarian task allocation │     │
│  │ Isaac ↔ ROS2 bridge           │    │  • CBS multi-agent planner   │     │
│  │  pub: tf, odom, scan, image   │───►│  • deadlock detection        │     │
│  │  sub: cmd_vel, arm cmds       │    ├──────────────────────────────┤     │
│  └───────────────────────────────┘    │ Manipulation Pipeline (Py)   │     │
│                ▲                       │  • FoundationPose            │     │
│                │                       │  • AnyGrasp                  │     │
│                │                       │  • MoveIt2                   │     │
│                │                       └──────────────────────────────┘     │
│                │                                  ▼                          │
│                │            ┌─────── Metrics & Recorder ────────┐            │
│                └────────────┤ throughput, deadlocks, cycle time │            │
│                             │ video recording (MP4)             │            │
│                             └───────────────────────────────────┘            │
│                                              │                               │
│                                              ▼                               │
└──────────────── persistent volume (USD cache + outputs) ────────────────────┘
                                              │
                                              ▼
                                       Local Mac (developer)
                                       code edit • view demo • assemble portfolio
```

### Architectural principles
- **Single container, single volume** — minimum moving parts
- **GPU per call**, not per deployment — pay for the right GPU for each scenario
- **Components decoupled by event/topic boundaries** — each component testable in isolation
- **Sim time, not wall time** — metrics are calibrated to in-sim seconds so they're comparable across machines

## 5. Components

### 5.1 Warehouse Scene Builder
- **Purpose:** Generate a USD warehouse scene from a YAML layout config.
- **Inputs:** `layout.yaml` (warehouse dims, AMR count + spawn poses, pick-station position, shelf layout, SKU palette).
- **Outputs:** USD file in `/vol/scenes/`.
- **Location:** `warehouse/generators/build_scene.py`
- **Depends on:** Isaac Sim USD APIs (no ROS2 dependency).

### 5.2 Isaac Sim Runner *(includes `omni.isaac.ros2_bridge`)*
- **Purpose:** Boot Isaac Sim headless, load a scene, expose sim state on ROS2 topics, accept control commands.
- **Topics published:** `/tf`, `/robot_N/odom`, `/robot_N/scan`, `/robot_N/camera/*`, `/joint_states`, `/cell/cam/{rgb,depth}`.
- **Topics subscribed:** `/robot_N/cmd_vel`, `/arm/joint_command`.
- **Depends on:** Isaac Sim 5.x, ROS2 bridge extension.

### 5.3 Fleet Coordinator *(custom ROS2 node, Python)*
- **Purpose:** Assign orders to robots; resolve multi-agent path conflicts; trigger pick cell when a tote arrives.
- **Inputs:** Order queue (from scenario config), robot poses (TF lookups), Nav2 plans (subscriber).
- **Outputs:** `/robot_N/goal_pose` per robot via `NavigateToPose` action; events on `/fleet/events`.
- **Algorithms:** Hungarian-algorithm task allocation + Conflict-Based Search (CBS) for path conflict resolution.
- **Pluggable:** strategy interface — Phase 2 can swap planners (greedy / priority / CBS / ECBS) via config.
- **Location:** `ros2_ws/src/fleet_coordinator/`
- **Depends on:** Nav2 action client, tf2.

### 5.4 Nav2 Stack *(off-the-shelf, per-AMR config)*
- **Purpose:** Per-robot navigation — global plan, local plan, costmap, recovery.
- **Interface:** Standard Nav2 — `NavigateToPose` action in, `cmd_vel` out.
- **What we author:** per-robot YAML configs (planner, controller, costmap params).
- **Depends on:** `nav2_bringup`, lifecycle manager.

### 5.5 Manipulation Pipeline *(Python service, drives MoveIt2)*
- **Purpose:** When a tote arrives at the pick cell, perceive SKUs, plan grasp, execute pick → place.
- **Inputs:** trigger message (`/cell/start_pick`), RGB-D from cell camera.
- **Outputs:** arm trajectories via MoveIt2 (`/move_group` action); `/cell/pick_result` (success/fail/cycle time).
- **Stages:**
  1. FoundationPose — 6-DoF pose for each SKU in the tote (zero-shot, weights pre-loaded)
  2. AnyGrasp — top-K grasp candidates ranked by score
  3. MoveIt2 — collision-free trajectory to the highest-ranked grasp
  4. Execute: approach → close gripper → lift → move to place pose → release
- **Retries:** up to 3 grasp attempts on failure; then order marked `blocked`.
- **Location:** `manipulation/pipeline.py`
- **Depends on:** isaac_ros_foundationpose, isaac_ros_grasp (or AnyGrasp standalone), moveit2.

### 5.6 Scenario Runner *(Modal entrypoint)*
- **Purpose:** One command to launch a full scenario end-to-end and capture artifacts.
- **Inputs:** `scenario.yaml` (which scene, order list, duration, recording flags).
- **Outputs to `/vol/runs/<timestamp>/`:** `metrics.json`, `video.mp4`, `events.log`, `screenshots/`.
- **Location:** `modal/run_sim.py`
- **Depends on:** all above.

### 5.7 Observability *(thin wrappers, in-process)*
- **Metrics Recorder:** subscribes to `/fleet/events` and `/cell/pick_result`; writes CSV/JSON.
- **Video Recorder:** uses Isaac Sim's `omni.replicator` writer to dump camera frames; ffmpeg assembles MP4.

## 6. Data Flow & Lifecycle

### Lifecycle of a single order

```
Scenario.yaml       Fleet Coordinator         Nav2 (AMR_3)         Manipulation Pipeline
     │                    │                       │                        │
     │  enqueue order ───►│                       │                        │
     │                    │ pick AMR_3            │                        │
     │                    │ (Hungarian)           │                        │
     │                    │ NavigateToPose ──────►│ plan → drive (~12s)    │
     │                    │ ◄── arrived ──────────│                        │
     │             [Isaac Sim attaches tote prim to AMR_3]                 │
     │                    │ NavigateToPose ──────►│ drive (~18s)           │
     │                    │ ◄── arrived ──────────│                        │
     │                    │ /cell/start_pick ─────────────────────────────►│
     │                    │                       │                        │ FoundationPose
     │                    │                       │                        │ AnyGrasp
     │                    │                       │                        │ MoveIt2 plan+execute
     │                    │ ◄── pick_result ──────────────────────────────│
     │                    │ NavigateToPose ──────►│  return zone (~10s)    │
     │                    │ ◄── arrived ──────────│                        │
     │ ◄── order done ────│                       │                        │
```

End-to-end cycle (one order, one robot): ~50s sim time. Target throughput with 6 AMRs: ~400 orders/hr sim.

### AMR state machine

```
IDLE ──(assigned)──► NAVIGATING_TO_SHELF ──(arrived)──► LOADED
                                │                          │
                                │ (Nav2 fail)              │
                                ▼                          ▼
                              FAILED                  NAVIGATING_TO_CELL
                                                          │
                                                          ▼
                                                       AT_CELL ──(pick done)──► RETURNING
                                                          │                        │
                                                          └──(pick fail)──► FAILED │
                                                                                    │
                                                                                    ▼
                                                                                  IDLE
```

### Multi-agent interactions

| Event | Coordinator behavior |
|---|---|
| Two AMRs assigned to overlapping corridor | CBS pre-computes conflict; reroutes lower-priority robot via alternate path before issuing Nav2 goal |
| Live conflict (CBS missed) | Nav2 local planner handles via DWB obstacle avoidance; on failure, robot reports failure |
| Two robots idle ≤1m apart for >5s | Deadlock recovery: lower-priority robot retreats 2m, replans |
| Battery <20% (optional Phase 1) | Coordinator re-routes to charge dock; AMR removed from assignment pool |
| Manipulation cell busy | New orders for that cell queue; fleet routing continues |

### Magic attach vs. real pick

Phase 1 + 2 use **magic attach** at the shelf — when AMR reaches `shelf_X`, Isaac Sim programmatically attaches a tote prim to the AMR. **Real manipulation is only at the pick cell** where the Franka unloads the tote. This focuses real-perception/real-motion-planning effort on the highest-impact subsystem.

## 7. Modal Infrastructure

### Image

```
Base: nvcr.io/nvidia/isaac-sim:5.x-headless  (Ubuntu 22.04)
  + apt: ros-humble-desktop, nav2-bringup, moveit, foxglove-bridge
  + pip: numpy, opencv-python, scipy, networkx, modal
  + Isaac ROS packages: foundationpose, grasp
  + system: ffmpeg, xvfb
  + copied: ros2_ws (pre-built with colcon), modal/, manipulation/, warehouse/, scenarios/
```

One-time build cost: ~15min. Warm container start: ~30s.

### Volume layout

```
isaac-volume/
├── assets/                    (~5GB — Isaac asset packs, pre-pulled)
├── scenes/                    (generated USDs)
├── models/                    (~2GB — FoundationPose + AnyGrasp weights)
└── runs/<timestamp>/
    ├── metrics.json
    ├── events.log
    ├── video.mp4
    └── screenshots/
```

### GPU per phase

| Activity | GPU | Hourly cost |
|---|---|---|
| Scene authoring, smoke tests | L4 | ~$0.80 |
| Demo runs, video recording, ablations | **L40S (default)** | ~$2.00 |
| ML training (Phase 3 only) | H100 | ~$5–8 |

### Cost ceiling

- $25 alert per Modal account
- $28 hard stop per Modal account
- Switch to secondary $30 account on overrun

## 8. Error Handling

| Failure | Detection | Recovery |
|---|---|---|
| Image pull fails | Modal native | Re-run; layers cached |
| Container OOM | Modal kills | Reduce AMR count / camera resolution, retry |
| Isaac Sim crash mid-run | try/except around sim loop | Flush partial metrics + video, exit 2, scenario flagged `crashed=true` |
| ROS2 node death | `ros2 lifecycle` monitor in Scenario Runner | Phase 1: scenario fails. Phase 2 stretch: lifecycle restart |
| Modal timeout | Modal kills | Increase per-scenario timeout (default 30min) |
| GPU unavailable | Modal error | Fallback to L4 (slower) or retry |
| Nav2 plan fails | Action result | Coordinator marks goal failed; retry with relaxed costmap; if still fails, order marked failed |
| Grasp fails | Pipeline result | Retry up to 3 times with different candidates; then order `blocked` |
| Tote pose not detected | Pipeline timeout (10s) | Abort pick, order `blocked` |

## 9. Testing & Verification

### Test pyramid

```
┌─────────────────────────┐
│ Manual demo recording   │  ← 1–2 per phase milestone
│ (full scenario run)     │
├─────────────────────────┤
│ Integration on Modal    │  ← run on push-to-main, ~$0.15/commit
│ (headless smoke tests)  │
├─────────────────────────┤
│ Unit tests (local)      │  ← run on every commit, free
│ (algorithms, parsers)   │
└─────────────────────────┘
```

### Unit tests (pure Python, run on Mac)
- Scene Builder: YAML → USD structure assertions
- Fleet Coordinator: Hungarian + CBS against hand-crafted scenarios
- Manipulation stages: fixtures of saved RGB-D → expected pose/grasp output ±tolerance
- Metrics Recorder: synthetic event stream → expected JSON
- Scenario parser: malformed YAML → helpful error

### Integration tests (Modal headless sim)

| Test | Duration | Cost |
|---|---|---|
| Boot Isaac + ROS2, assert topics publish | 60s | ~$0.02 |
| Single AMR navigates to a goal | 90s | ~$0.03 |
| Single pick at the cell, known tote | 120s | ~$0.04 |
| End-to-end: 2 AMRs, 1 order, completion event | 180s | ~$0.05 |

### Manual verification gate

A run is "verified" when:
- Demo video plays end-to-end without visual glitches
- `metrics.json` shows plausible numbers (non-zero deadlocks across 50 orders; throughput consistent with assigned orders)
- `events.log` is free of `ERROR`-level lines

## 10. Success Criteria

### Phase 1 — "core combined demo works" *(4–5 weeks)*
- ✅ 6 Nova Carter AMRs run concurrently for ≥10min sim with zero crashes
- ✅ Pick cell completes ≥80% of attempted picks (50+ attempts in eval)
- ✅ At least one CBS-resolved conflict captured in demo video
- ✅ Demo video: 60–90s, end-to-end order lifecycle
- ✅ Metrics in `docs/results.md`: throughput (orders/hr sim), avg cycle time, pick success rate, deadlocks/min
- ✅ README portfolio-quality: architecture diagram, embedded video/GIF, run-it-yourself instructions, attribution

### Phase 2 — "scale + rigor" *(2–4 weeks)*
- ✅ Planner ablation table: ≥3 planners (greedy / priority / CBS) across same scenarios
- ✅ Scenario library: ≥3 distinct scenarios (steady-state, peak-hour, hot-SKU-cluster) with results recorded
- ✅ Scale validation: ≥12 AMRs once; throughput-vs-fleet-size curve in `results.md`
- ✅ Web dashboard for live metrics (optional but high-impact)
- ✅ README updated with ablation + scaling results

### Phase 3 — *optional, only if pursued* "I trained a model" *(1–2 weeks)*
- ✅ Synthetic dataset via Isaac Replicator with DR (≥10K labeled images)
- ✅ Custom small model trained on synthetic data
- ✅ Sim2real evaluation: real-image performance with and without DR — ablation table
- ✅ Results section added to `docs/results.md`

### Portfolio-ready acceptance test

A recruiter or eng hiring manager landing on the GitHub README in ≤90 seconds should see:
1. Embedded 60–90s video at top — fleet running + pick cell in action
2. One-sentence commercial pitch ("Same architecture KION/Cyngn/GXO are deploying")
3. Architecture diagram (polished version of Section 4)
4. Numbers — throughput, success rate, ablation table
5. Stack badges — Isaac Sim, ROS2, Nav2, MoveIt2, FoundationPose, Modal
6. "Run it yourself" — one `modal run` command

## 11. Proposed Repository Layout

```
isaac-sim/
├── README.md                     # Portfolio front page
├── docs/
│   ├── superpowers/specs/        # Design docs (this file)
│   ├── architecture.md           # Polished system overview
│   ├── results.md                # Metrics, ablations
│   └── images/                   # Diagrams, screenshots
├── modal/
│   ├── image.py                  # Modal image definition
│   ├── run_sim.py                # Modal entrypoint
│   ├── volumes.py                # Volume config
│   └── budget.py                 # Cost tracker
├── warehouse/
│   ├── scene/                    # USD scene templates
│   ├── assets/                   # SKU meshes, props (or refs to Isaac assets)
│   └── generators/build_scene.py # Programmatic scene builder
├── ros2_ws/
│   └── src/
│       ├── warehouse_bringup/    # Launch files
│       ├── fleet_coordinator/    # Hungarian + CBS
│       ├── manipulation_cell/    # Pick station bringup
│       └── isaac_bridge/         # Custom Isaac ↔ ROS2 nodes if needed
├── manipulation/
│   ├── pipeline.py               # FoundationPose → AnyGrasp → MoveIt2
│   ├── pose_estimation/
│   ├── grasping/
│   └── motion_planning/
├── metrics/
│   ├── recorder.py
│   └── dashboard/                # Phase 2 stretch
├── scenarios/
│   ├── smoke_test.yaml
│   ├── steady_state.yaml
│   ├── peak_hour.yaml            # Phase 2
│   └── hot_sku_cluster.yaml      # Phase 2
├── scripts/
│   ├── record_demo.py
│   └── ablation_runner.py        # Phase 2
└── tests/
    ├── unit/
    └── integration/
```

## 12. Phasing Summary

| Phase | Duration | Compute | Deliverables | Gate |
|---|---|---|---|---|
| Phase 1 | 4–5 weeks | ~$15–25 | Core demo + 6 AMRs + 1 pick cell + video + metrics | All Phase 1 success criteria met |
| Phase 2 | 2–4 weeks | ~$10–20 | Planner ablation + 3 scenarios + scale to 12 AMRs + dashboard | All Phase 2 success criteria met |
| Phase 3 (optional) | 1–2 weeks | ~$10–20 | Synthetic data + custom-trained model + sim2real ablation | Phase 3 success criteria met (only pursue with budget headroom) |

**Total compute estimate:** $35–65 across both phases (~$45–85 with Phase 3). Fits within $60 Modal credits with safety margin from second account.

## 13. Open Questions

(To be resolved at implementation time, not blockers for the spec)

- Whether to package the project as an OSS kit later (Approach C from brainstorm) — defer until after Phase 1 is shipped
- Charging station modeling: include in Phase 1 or defer to Phase 2 — *current default: defer*
- Whether to integrate Foxglove for live ROS2 visualization vs. only Rviz — *current default: Foxglove, since browser-accessible from Mac*
- Whether ablation runner triggers GitHub Actions or stays manual — *current default: manual, to avoid CI cost surprises*

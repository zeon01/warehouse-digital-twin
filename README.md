# Warehouse Digital Twin

[![Unit Tests](https://github.com/zeon01/warehouse-digital-twin/actions/workflows/unit-tests.yml/badge.svg)](https://github.com/zeon01/warehouse-digital-twin/actions/workflows/unit-tests.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

> Reference implementation of the digital-twin validation pipeline used in commercial warehouse automation — same architecture KION (with Accenture / Siemens) is deploying for **GXO Logistics**, and Cyngn is using to validate autonomy before real-facility rollout.

![Combined scene — 6 Nova Carter AMRs + Franka pick cell](docs/images/scene_iso.png)

## What's in here

| Component | Where | Status |
|-----------|-------|--------|
| Procedural warehouse builder (USD + materials + lighting) | `warehouse/` | ✅ Working — `python -m warehouse.generators.build_scene small` |
| Isaac Sim 5.0 + ROS2 bridge orchestration | `sim/`, `wdt_vast/` | ✅ Working — 6 AMRs + Franka + 60 namespaced ROS2 topics |
| Fleet coordinator (Hungarian + CBS + deadlock) | `coordinator/`, `ros2_ws/src/fleet_coordinator/` | ✅ Tested in isolation; ROS2 node colcon-built |
| Manipulation pipeline (FoundationPose + AnyGrasp + MoveIt2) | `manipulation/` | ⚠️ Wrappers + interfaces; model installs are Phase 2 |
| Nav2 per-AMR config + launch | `ros2_ws/src/warehouse_bringup/` | ⚠️ Config done; full Nav2 wiring is Phase 2 (Task 26 uses direct `cmd_vel` instead) |
| Metrics + video recorder | `metrics/` | ✅ Working — pytest-covered, MP4 assembler via ffmpeg |
| Scenario runner (end-to-end) | `wdt_vast/run_scenario.py`, `scenarios/` | ✅ Composes end-to-end; 64-order steady-state verified |

## What works in Phase 1 — verified

- **Procedural warehouse** generation from YAML → coloured USD scene in <2 seconds locally (`outputs/scenes/small.usd`).
- **Isaac Sim Kit boots headless** on a vast.ai RTX A5000, opens the warehouse USD, spawns 6 namespaced Nova Carter AMRs + a Franka, renders 4 camera angles to portfolio-quality PNGs ([overhead](docs/images/scene_overhead.png), [iso](docs/images/scene_iso.png), [amrs closeup](docs/images/scene_amrs.png)).
- **ROS2 bridge** publishes 60 topics across 6 AMR namespaces (`/amr_N/cmd_vel`, `/amr_N/chassis/odom`, `/amr_N/tf`, `/amr_N/front_3d_lidar/lidar_points`, stereo cameras, 4 IMUs each).
- **Direct cmd_vel motion**: one Nova Carter moved **2.43 m** in 10 sim seconds in response to `/amr_0/cmd_vel` Twist publishes — proves bridge ↔ `differential_drive` OmniGraph ↔ wheel-joint physics chain is sound.
- **Coordinator + planner unit tests** all pass: Hungarian assignment (3 tests), CBS multi-agent path planner (2), pairwise deadlock detector (2), strategy registry (2).
- **64-order steady-state scenario** runs end-to-end on the rented A5000 in ~2 min wall-clock (10 min sim time at 5× real-time): all 64 orders enqueued at correct sim times, MetricsRecorder + events.log emitted cleanly, coordinator subprocess survives the full run.

## Visuals from the Phase 1 demo scene

| Iso (3/4 view) | AMR cluster closeup | Overhead |
|---|---|---|
| ![iso](docs/images/scene_iso.png) | ![amrs](docs/images/scene_amrs.png) | ![overhead](docs/images/scene_overhead.png) |

## Architecture

```mermaid
graph TB
    subgraph Local["Local Mac (dev)"]
        SceneGen["warehouse.generators<br/>USD builder · usd-core"]
        Coord["coordinator/<br/>Hungarian · CBS · deadlock"]
        Manip["manipulation/<br/>FoundationPose · AnyGrasp · MoveIt2 wrappers"]
        Metrics["metrics/<br/>recorder + ffmpeg"]
    end

    subgraph VastAI["vast.ai RTX A5000 (Linux + Vulkan)"]
        IsaacSim["Isaac Sim 5.0 Kit<br/>headless · python.sh"]
        ROS2["ROS2 Humble<br/>CycloneDDS rmw"]
        Bridge["isaacsim.ros2.bridge<br/>OmniGraph publishers"]
        Fleet["FleetCoordinator node<br/>colcon-built ros2_ws"]
    end

    SceneGen -->|outputs/scenes/*.usd| IsaacSim
    IsaacSim --> Bridge
    Bridge <--> ROS2
    Fleet --> ROS2
    Coord -.->|wired in run_scenario| Fleet
    Manip -.->|wired in run_scenario| Fleet
    IsaacSim -->|Replicator BasicWriter| Metrics
```

[Detailed design spec](docs/superpowers/specs/2026-05-14-warehouse-digital-twin-design.md) · [Full Phase 1 plan with all 48 tasks](docs/superpowers/plans/2026-05-14-warehouse-digital-twin-phase-1.md)

## Why Modal + vast.ai

We tried Modal for the entire stack first — it's elegant for cloud GPU but **its containers can't run Isaac Sim 5.0's Vulkan stack** (verified on L4, A10G, B200 — all fail with `VkResult: ERROR_DEVICE_LOST` or `ERROR_INITIALIZATION_FAILED` despite the host having driver 580.95 and Vulkan ICD configured). vast.ai datacenter hosts (RTX A5000 with driver 570+) work cleanly with the same Isaac Sim image. The hybrid split:

- **Modal**: budget tracker, CI image (built once), any non-render Linux work
- **vast.ai**: all Isaac Sim + ROS2 + render runs (the rented instance is stopped between sessions, idling at ~$0.025/hr)
- **Local Mac**: USD authoring (usd-core), pure-Python coordinator + manipulation tests, scenario YAML, render orchestration

## Run it yourself

```bash
git clone https://github.com/zeon01/warehouse-digital-twin.git
cd warehouse-digital-twin
python -m pip install -e ".[dev]"

# Build a USD scene locally — no GPU needed
python -m warehouse.generators.build_scene small
# → outputs/scenes/small.usd

# Run the pure-Python coordinator + manipulation tests
pytest tests/unit/
# → 14 passing, 2 skipped (fixtures)
```

The Isaac Sim rendering + ROS2 integration runs need a vast.ai (or equivalent) RTX instance with driver ≥570; setup instructions are in [`wdt_vast/README.md`](wdt_vast/README.md).

## What's next (Phase 2)

The current scenario runner composes the full pipeline structurally — `orders_total=64, orders_completed=0` is the documented Phase 1 state. To turn that 0 into real pick numbers, Phase 2 will:

1. **Wire Nav2 properly** — map server + AMCL + lifecycle activation, replacing the current `cmd_vel`-only motion. The plumbing is there; needs a map and a few launch tweaks.
2. **Install FoundationPose + AnyGrasp + MoveIt2** model weights (~GB each) on the instance and trigger the manipulation pipeline when an AMR with an order reaches the pick cell.
3. **Capture the demo video** — add a `BasicWriter` to the overhead camera inside `run_scenario`, run the steady-state with `record_video: true`, stitch via `metrics.video.assemble_mp4`.

Phase 3 (stretch): custom-trained perception model with a sim-to-real ablation.

## Stack

NVIDIA Isaac Sim 5.0 · ROS2 Humble · CycloneDDS · Nav2 · MoveIt2 · FoundationPose · AnyGrasp · usd-core · pydantic · pytest · ruff · Modal · vast.ai

## License

MIT — see [LICENSE](LICENSE).

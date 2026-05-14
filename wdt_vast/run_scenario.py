"""End-to-end scenario runner for the warehouse digital twin.

Invoked on a vast.ai instance via:
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    /isaac-sim/python.sh wdt_vast/run_scenario.py <scenario.yaml> <out_dir>

The plan's Task 42 placed this on Modal as `wdt_modal/run_sim.py`, but
Modal's container Vulkan stack can't run Isaac Sim's renderer (Tasks
7-9 / 16). So the full integration runs on vast.ai instead. Task 43
wires the FleetCoordinator ROS2 node and the ManipulationPipeline into
the loop; Task 44 runs the smoke scenario; Task 45 runs the full
acceptance scenario with video.

Pipeline:
    1. Load Scenario(YAML) and Layout(YAML).
    2. Boot Kit + enable ROS2 bridge.
    3. Open the procedurally-built warehouse USD.
    4. Spawn N namespaced Nova Carters at the AMR poses; spawn Franka.
    5. Launch the FleetCoordinator ROS2 node as a subprocess.
    6. Step the world for scenario.duration_s. Inject orders at their
       `arrival_t` by publishing PoseStamped on /orders/enqueue.
    7. MetricsRecorder hooks fire on every coordinator-tracked event.
    8. Optionally record overhead camera frames each N frames for the
       MP4 (Task 40's metrics.video.assemble_mp4 finishes that off).

Outputs to <out_dir>:
    progress.txt   — phase markers
    metrics.json   — orders_total, pick_success_rate, etc.
    events.log     — flat ENQ/ASN/DONE/DEADLOCK event log
    frames/*.png   — overhead frames (if record_video)
    error.txt      — traceback if anything raises
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path

scenario_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/scenarios/smoke.yaml"
out_dir = Path(sys.argv[2] if len(sys.argv) > 2 else "/tmp/run_out")
out_dir.mkdir(parents=True, exist_ok=True)

PROGRESS = out_dir / "progress.txt"
ERROR = out_dir / "error.txt"
PROGRESS.write_text("")


def mark(phase: str) -> None:
    with PROGRESS.open("a") as fh:
        fh.write(f"{datetime.now(timezone.utc).isoformat()}  {phase}\n")


try:
    mark("script_start")
    sys.path.insert(0, "/tmp")
    from metrics.recorder import MetricsRecorder
    from scenarios.schema import load_scenario
    from warehouse.layout import load_layout

    mark("imports_ok")

    scenario = load_scenario(scenario_path)
    layout = load_layout(f"/tmp/warehouse/layouts/{scenario.layout}.yaml")
    mark(f"scenario_loaded_{scenario.name}_orders={len(scenario.orders)}")

    recorder = MetricsRecorder(out_dir=out_dir)

    from isaacsim import SimulationApp  # noqa: E402

    sim = SimulationApp({"headless": True})

    from isaacsim.core.api import World  # noqa: E402
    from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

    enable_extension("isaacsim.ros2.bridge")
    mark("simapp_booted_bridge_enabled")

    from sim.multi_robot import spawn_amr_fleet  # noqa: E402
    from sim.spawn import spawn_franka  # noqa: E402

    world = World()
    world.scene.add_default_ground_plane()

    gx, gy = layout.amrs.spawn.grid
    ox, oy = layout.amrs.spawn.origin_xy
    spacing = layout.amrs.spawn.spacing_m
    poses = [(ox + c * spacing, oy + r * spacing) for r in range(gy) for c in range(gx)][
        : scenario.fleet_size
    ]
    spawn_amr_fleet(world, poses)
    mark(f"fleet_spawned_n={len(poses)}")

    px, py = layout.pick_cell.position_xy
    spawn_franka(world, "/World/pick_arm", "pick_arm", position_xyz=(px, py, 1.0))
    mark("franka_spawned")

    # Launch the FleetCoordinator as a subprocess (Task 43 wiring).
    # ros2_ws must already be built on the instance — see wdt_vast/README.md.
    amr_ids = [f"amr_{i}" for i in range(scenario.fleet_size)]
    ros_env = os.environ.copy()
    ros_env["ROS_DOMAIN_ID"] = "42"
    coordinator_proc = subprocess.Popen(
        [
            "bash",
            "-lc",
            "source /opt/ros/humble/setup.bash && "
            "source /work/ros2_ws/install/setup.bash && "
            f"ros2 run fleet_coordinator fleet_coordinator_node "
            f"--ros-args -p amr_ids:='{amr_ids}'",
        ],
        env=ros_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    mark(f"coordinator_launched_pid={coordinator_proc.pid}")

    # Manipulation pipeline — instantiated in-process per Task 43 plan.
    # **Phase 1 gap:** the actual FoundationPose + AnyGrasp + MoveIt2
    # packages aren't installed on the vast.ai instance (model weights
    # ~GB each, MoveIt2 needs its own ROS deps). The pipeline's
    # `_lazy_load()` raises ImportError on first call, so we wrap the
    # instantiation in a try and disable the pipeline if the deps are
    # missing — keeps run_scenario alive end-to-end. Phase 2 will land
    # the model installs + a real per-order trigger from the
    # coordinator's "near pick cell" signal.
    manip = None
    try:
        from manipulation.grasping import GraspGenerator
        from manipulation.motion_planning import ArmPlanner
        from manipulation.pipeline import ManipulationPipeline
        from manipulation.pose_estimation import PoseEstimator

        manip = ManipulationPipeline(
            pose_estimator=PoseEstimator(model_dir="/vol/models/foundationpose"),
            grasp_generator=GraspGenerator(model_dir="/vol/models/anygrasp"),
            arm=ArmPlanner(planning_group="panda_arm"),
        )
        mark("manip_pipeline_constructed")
    except Exception as e:
        mark(f"manip_pipeline_skipped:{type(e).__name__}")

    world.reset()
    mark("world_reset")

    # Settle physics a beat before order injection starts.
    for _ in range(30):
        world.step(render=True)

    # Drive the sim for scenario.duration_s of sim time at 30 Hz; inject
    # orders at their arrival_t into the recorder (the coordinator picks
    # them up via its /orders/enqueue subscription once that publisher is
    # wired — for now we just record).
    t = 0.0
    dt = 1.0 / 30.0
    next_order_idx = 0
    sorted_orders = sorted(scenario.orders, key=lambda o: o.arrival_t)

    while t < scenario.duration_s:
        while next_order_idx < len(sorted_orders) and sorted_orders[next_order_idx].arrival_t <= t:
            o = sorted_orders[next_order_idx]
            recorder.on_order_enqueued(order_id=o.id, at=t)
            next_order_idx += 1
        world.step(render=scenario.record_video)
        t += dt

    mark(f"loop_done_orders_enqueued={next_order_idx}")
    recorder.flush()

    coordinator_proc.terminate()
    try:
        coordinator_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        coordinator_proc.kill()

    sim.close()
    mark("sim_closed")
    print(json.dumps({"run_dir": str(out_dir), "orders_enqueued": next_order_idx}))
except Exception as e:
    ERROR.write_text(f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
    mark(f"EXCEPTION:{type(e).__name__}")
    raise

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

import argparse
import json
import os
import subprocess
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """CLI for run_scenario.

    Phase 2 adds --allocator, --path-planner, --seed to drive the
    planner-ablation grid via ``wdt_vast/run_ablation.py``. The first
    two positionals (scenario, out_dir) stay for Phase 1 compatibility.
    """
    parser = argparse.ArgumentParser(prog="run_scenario")
    parser.add_argument(
        "scenario",
        nargs="?",
        default="/tmp/scenarios/smoke.yaml",
        help="path to scenario YAML",
    )
    parser.add_argument(
        "out_dir",
        nargs="?",
        default="/tmp/run_out",
        help="output directory",
    )
    parser.add_argument(
        "--allocator",
        choices=["greedy", "hungarian"],
        default="hungarian",
        help="task allocator (Phase 2 ablation axis 1)",
    )
    parser.add_argument(
        "--path-planner",
        choices=["greedy", "cbs"],
        default="cbs",
        help="multi-agent path planner (Phase 2 ablation axis 2)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="RNG seed for order-arrival jitter (Phase 2)",
    )
    return parser.parse_args(argv)


_args = _parse_args()
scenario_path = _args.scenario
out_dir = Path(_args.out_dir)
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

    # Phase 2 ablation: jitter order arrivals so each seed produces a
    # different stochastic schedule. The deterministic baseline (Phase 1)
    # is recovered by passing --seed 42 with jitter_s=0; we keep
    # jitter_s=5 by default so the seeds matter.
    from scenarios.schema import apply_seed_jitter

    scenario.orders = apply_seed_jitter(scenario.orders, _args.seed, jitter_s=5.0)
    mark(
        f"scenario_loaded_{scenario.name}_orders={len(scenario.orders)}"
        f"_alloc={_args.allocator}_planner={_args.path_planner}_seed={_args.seed}"
    )

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

    # M5 acceptance: orders_completed=1 means the full chain runs:
    #   order -> coordinator -> AMR nav -> at_cell -> /cell/start_pick
    #     -> orchestrator -> FP+grasp+MoveIt plan -> /cell/pick_result
    # That requires four ROS2 stacks alive in parallel with the sim:
    #   1. wdt_pure_pursuit multi_amr — NavigateToPose servers per AMR.
    #   2. wdt_manipulation_bringup move_group — MoveIt2 OMPL planner.
    #   3. wdt_vast/synthetic_cell_camera — fake /cell/cam/{rgb,depth,info}
    #      until Isaac Sim Camera plumbing is wired (M5b / Phase 3).
    #   4. wdt_manipulation_bringup pick_cell_orchestrator — pose + grasp
    #      + MoveIt plan from camera frames + a CAD path.
    #   5. fleet_coordinator — top-level state machine.
    # Per gotcha #18/#19 (memory), Carter's diff_drive OG only listens
    # on /amr_i/cmd_vel once `_namespace_subtree` has been called — that
    # is handled by spawn_amr_fleet (don't double-call from here).

    amr_ids = [f"amr_{i}" for i in range(scenario.fleet_size)]
    pick_xy = list(layout.pick_cell.position_xy)
    ros_env = os.environ.copy()
    ros_env["ROS_DOMAIN_ID"] = "42"

    def _ros2_popen(name: str, cmd_str: str) -> subprocess.Popen:
        """Background-launch a ros2 command with sourcing baked in.

        setsid + redirect-from-/dev/null per gotcha #23 so the child
        survives even when the parent shell goes away during long sim
        loops.
        """
        log = out_dir / f"{name}.log"
        return subprocess.Popen(
            [
                "bash",
                "-lc",
                "set +u && "
                "source /opt/ros/humble/setup.bash && "
                "source /work/ros2_ws/install/setup.bash && "
                "set -u && " + cmd_str,
            ],
            env=ros_env,
            stdout=open(log, "w"),
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )

    # Generate the synthetic CAD that synthetic_cell_camera's depth bump
    # matches — 8 cm cube. Orchestrator's cad_path param points here.
    import trimesh  # noqa: E402

    m5_cad = "/tmp/m5_smoke_box.obj"
    trimesh.creation.box(extents=(0.08, 0.08, 0.08)).export(m5_cad)
    mark(f"m5_cad_written={m5_cad}")

    # 1. Pure-pursuit fleet — NavigateToPose action servers per AMR.
    pp_proc = _ros2_popen(
        "pure_pursuit",
        "ros2 launch wdt_pure_pursuit multi_amr.launch.py",
    )
    mark(f"pure_pursuit_launched_pid={pp_proc.pid}")

    # 2. MoveIt2 move_group — OMPL planner.
    move_group_proc = _ros2_popen(
        "move_group",
        "ros2 launch wdt_manipulation_bringup move_group.launch.py",
    )
    mark(f"move_group_launched_pid={move_group_proc.pid}")

    # 3. Synthetic cell camera — until Isaac Sim Camera + ROS2CameraHelper
    #    is wired in (M5b / Phase 3).
    cam_proc = _ros2_popen(
        "synth_cell_cam",
        "/usr/bin/python3 /work/wdt_vast/synthetic_cell_camera.py",
    )
    mark(f"synth_cell_cam_launched_pid={cam_proc.pid}")

    # 4. Pick cell orchestrator — subscribes to /cell/cam/* + /cell/start_pick,
    #    runs FP + TopDownGrasp + ArmPlanner (plan_only), publishes
    #    /cell/pick_result. Override cad_path to the synthetic box we
    #    just wrote.
    orch_proc = _ros2_popen(
        "pick_orch",
        "ros2 run wdt_manipulation_bringup pick_cell_orchestrator "
        f"--ros-args -p cad_path:={m5_cad}",
    )
    mark(f"pick_orchestrator_launched_pid={orch_proc.pid}")

    # 5. Fleet coordinator — top-level state machine.
    coordinator_proc = _ros2_popen(
        "coordinator",
        "ros2 run fleet_coordinator fleet_coordinator_node "
        f"--ros-args -p amr_ids:='{amr_ids}' "
        f"-p pick_cell_xy:='{pick_xy}' "
        f"-p allocator:={_args.allocator} "
        f"-p path_planner:={_args.path_planner}",
    )
    mark(f"coordinator_launched_pid={coordinator_proc.pid}")

    # Give the ROS2 stacks time to come up before injecting orders.
    # move_group takes ~20 s on its own; pure-pursuit needs ~10 s; the
    # rest are fast.
    ros2_warmup_s = 30
    mark(f"sleeping_{ros2_warmup_s}s_for_ros2_warmup")
    import time as _time

    _time.sleep(ros2_warmup_s)
    mark("ros2_warmup_done")

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

    # Phase 2: publish each order on /orders/enqueue when its arrival_t hits.
    # Phase 1 only recorded them to disk; the coordinator never saw them
    # because nothing was publishing to the topic it subscribes to.
    import rclpy  # noqa: E402
    from geometry_msgs.msg import PoseStamped  # noqa: E402

    if not rclpy.ok():
        rclpy.init()
    order_node = rclpy.create_node("run_scenario_order_publisher")
    order_pub = order_node.create_publisher(PoseStamped, "/orders/enqueue", 10)
    mark("order_publisher_up")

    while t < scenario.duration_s:
        while next_order_idx < len(sorted_orders) and sorted_orders[next_order_idx].arrival_t <= t:
            o = sorted_orders[next_order_idx]
            recorder.on_order_enqueued(order_id=o.id, at=t)
            msg = PoseStamped()
            msg.header.frame_id = o.id  # coordinator reads this as the order id
            msg.header.stamp = order_node.get_clock().now().to_msg()
            msg.pose.position.x = float(o.shelf_xy[0])
            msg.pose.position.y = float(o.shelf_xy[1])
            msg.pose.orientation.w = 1.0
            order_pub.publish(msg)
            next_order_idx += 1
        rclpy.spin_once(order_node, timeout_sec=0.0)
        world.step(render=scenario.record_video)
        t += dt

    order_node.destroy_node()
    rclpy.shutdown()

    mark(f"loop_done_orders_enqueued={next_order_idx}")
    recorder.flush()

    # Shut down ROS2 subprocesses in reverse-launch order.
    ros2_procs = [coordinator_proc, orch_proc, cam_proc, move_group_proc, pp_proc]
    for proc in ros2_procs:
        proc.terminate()
    for proc in ros2_procs:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    sim.close()
    mark("sim_closed")
    print(json.dumps({"run_dir": str(out_dir), "orders_enqueued": next_order_idx}))
except Exception as e:
    ERROR.write_text(f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
    mark(f"EXCEPTION:{type(e).__name__}")
    raise

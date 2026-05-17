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
    parser.add_argument(
        "--pose-source",
        choices=("fp", "gt"),
        default="gt",
        help="Pose source for the pick orchestrator. gt = ground truth "
        "from /world/cube_pose (M5 acceptance default); fp = live "
        "FoundationPose (M6 stretch).",
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

    # M5 v13: spawn a real table + cube + RGB-D camera so FoundationPose has
    # a real cube to register against (synthetic depth in v7-v12 returned
    # noise-confidence poses and unreachable grasp targets). Geometry chosen
    # from reachability math:
    #   Franka mounted at world (px, py, 1.0) → panda_link0 = world (px, py, 1.0)
    #   reachable grasp pose at panda_link0 (0.40, 0, -0.20)
    #   ↓ TopDownGrasp adds (0, 0, +0.05) standoff
    #   cube center at panda_link0 (0.40, 0, -0.25) = world (px+0.40, py, 0.75)
    #   cube edge 0.08 → bottom at z=0.71 → table top at z=0.71
    #   table height 0.7 → table center at z=0.36
    from sim.spawn import (  # noqa: E402
        spawn_pick_cell_lighting,
        spawn_pick_cube,
        spawn_pick_table,
    )

    table_center = (px + 0.40, py, 0.36)
    cube_center = (px + 0.40, py, 0.75)
    spawn_pick_table(world, center_xyz=table_center, size_xyz=(0.6, 0.6, 0.7))
    spawn_pick_cube(world, center_xyz=cube_center, edge_m=0.08)
    mark(f"pick_table_spawned_center={table_center}")
    mark(f"pick_cube_spawned_center={cube_center}")

    # M5 v17 fix: scene was rendering with no light source — depth was
    # correct but RGB was nearly black. FoundationPose uses both
    # channels for pose refinement; without RGB texture it returns
    # near-uniform scores and a random pose, leading to unreachable
    # grasp targets and "Unable to sample any valid states for goal
    # tree". Distant + dome lighting fixes the RGB channel so FP gets
    # real visual signal on the cube faces.
    spawn_pick_cell_lighting()
    mark("pick_cell_lighting_spawned")

    # Cell camera — looks at the cube from (px+0.40, py-0.80, 1.50) tilted
    # 46.8° down. Publishes /cell/cam/{rgb,depth,info} with frame_id=
    # cell_cam_optical. The static TF (world → cell_cam_optical) is launched
    # below as a separate subprocess so it survives even if the Isaac Sim
    # process restarts within a session.
    from sim.cell_camera import (  # noqa: E402
        DEFAULT_FOCAL_LENGTH_MM,
        DEFAULT_HORIZONTAL_APERTURE_MM,
        DEFAULT_VERTICAL_APERTURE_MM,
        build_ros2_camera_graph,
        spawn_cell_camera,
    )

    cam_pos_world = (px + 0.40, py - 0.80, 1.50)
    cam_euler_deg = (46.8, 0.0, 0.0)
    spawn_cell_camera(
        position_xyz=cam_pos_world,
        euler_xyz_deg=cam_euler_deg,
        focal_length_mm=DEFAULT_FOCAL_LENGTH_MM,
        horizontal_aperture_mm=DEFAULT_HORIZONTAL_APERTURE_MM,
        vertical_aperture_mm=DEFAULT_VERTICAL_APERTURE_MM,
    )
    build_ros2_camera_graph()
    mark(f"cell_camera_spawned_pos={cam_pos_world}_euler={cam_euler_deg}")

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

    # Defensive: kill any orphaned ros2 procs from a prior run. `start_new_session=True`
    # below detaches them from our shell so SIGTERM/SIGKILL on the bash wrapper doesn't
    # propagate. If a prior run was interrupted (kill -9 of the sim, SSH disconnect,
    # whatever), pp_drivers / coordinator / pick_orch survive and compete for the same
    # action names. Hit on M5 v8 (2026-05-16) — "Ignoring unexpected goal response. There
    # may be more than one action server" warning came from a v7 ghost coordinator.
    for pat in (
        "wdt_pure_pursuit/lib/wdt_pure_pursuit/pure_pursuit_driver",
        "fleet_coordinator/lib/fleet_coordinator/fleet_coordinator_node",
        "wdt_manipulation_bringup/lib/wdt_manipulation_bringup/pick_cell_orchestrator",
        "wdt_vast/synthetic_cell_camera.py",  # legacy v7-v12; harmless if absent
        "wdt_vast/franka_ready_joint_states.py",
        "wdt_vast/sim_world_pose_publisher.py",
        "tf2_ros/static_transform_publisher.*cell_cam_optical",
        "moveit_ros_move_group/move_group",
    ):
        subprocess.run(
            ["pkill", "-9", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
    mark("orphan_ros2_pkill_done")

    amr_ids = [f"amr_{i}" for i in range(scenario.fleet_size)]
    pick_xy = list(layout.pick_cell.position_xy)
    ros_env = os.environ.copy()
    # NOTE: do NOT set ROS_DOMAIN_ID here — Isaac Sim's ROS2 bridge
    # publishes on the env's default domain (typically 0 unless the
    # caller exports it). Setting subprocesses to a different domain
    # (Phase 1 used 42) silently silos them from the sim's TF + cmd_vel.
    # Verified 2026-05-16: pp_driver on domain 42 saw /amr_0/tf as
    # "does not exist" while bare SSH on domain 0 saw it at 35 Hz.
    ros_env.pop("ROS_DOMAIN_ID", None)
    # Strip Isaac Sim's Python 3.11 paths out of PYTHONPATH for subprocess
    # environments. The ros2 CLI runs under system /usr/bin/python3 (3.10);
    # if PYTHONPATH includes /isaac-sim/kit/python/lib/python3.11/*, the
    # first `import re` crashes with "SRE module mismatch" (verified
    # 2026-05-16 — coordinator + pick_orch + ros2 topic pub all died
    # this way). Keep /opt/ros/humble/* — those are the actual ROS2
    # python entries.
    _pp_parts = ros_env.get("PYTHONPATH", "").split(":")
    _pp_clean = [p for p in _pp_parts if p and "/isaac-sim/" not in p]
    # Add /tmp so subprocess ros2 nodes can import the non-colcon python
    # packages at the repo root (sim/, manipulation/, metrics/, etc.). The
    # parent script does sys.path.insert(0, "/tmp") on line ~105, but that
    # only affects the kit process — child ros2 procs need /tmp on their
    # PYTHONPATH explicitly. v11 hit this with pick_cell_orchestrator dying
    # at import on `from manipulation.grasping import TopDownGrasp`.
    if "/tmp" not in _pp_clean:
        _pp_clean.append("/tmp")
    ros_env["PYTHONPATH"] = ":".join(_pp_clean)
    # PYTHONHOME from Isaac Sim's kit also has to go — Python 3.11 home
    # poisons the 3.10 ros2 CLI.
    ros_env.pop("PYTHONHOME", None)
    # CRITICAL: also scrub /isaac-sim/ from LD_LIBRARY_PATH. We set
    # /isaac-sim/exts/isaacsim.ros2.bridge/humble/lib for the SIM
    # process (gotcha #17 — without it the ros2 bridge ext hangs), but
    # that path has Isaac-Sim-bundled copies of nav2_msgs, geometry_msgs,
    # tf2_msgs, etc. compiled against a NEWER ROS2 version. Their
    # typesupport_c .so files reference symbols (e.g.
    # `nav2_msgs__srv__dynamic_edges__response__convert_to_py`) that
    # don't exist in the apt-installed C libs. When the ros2 subprocess
    # dlopens nav2_msgs/typesupport_c, it picks up the Isaac Sim copy
    # first and fails with `UnsupportedTypeSupport`. Verified 2026-05-16:
    # ALL pure_pursuit_drivers + fleet_coordinator died this way.
    _ld_parts = ros_env.get("LD_LIBRARY_PATH", "").split(":")
    ros_env["LD_LIBRARY_PATH"] = ":".join(p for p in _ld_parts if p and "/isaac-sim/" not in p)

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
    # goal_timeout_s: pp_driver uses wallclock for elapsed (it's not on
    # /clock). Carter does ~0.05 m/s wall at max_linear=0.5 → 6m leg
    # is 120s wall, 10m leg is 200s wall. Use 600s timeout for margin.
    # Sticking with the conservative max_linear=0.5 for stability —
    # bumping to 1.5 was tried in M5 v8/v9 and may correlate with
    # Carter physics blowups that left base_link out of the TF tree.
    pp_proc = _ros2_popen(
        "pure_pursuit",
        "ros2 launch wdt_pure_pursuit multi_amr.launch.py "
        "goal_timeout_s:=1200.0 goal_tolerance_m:=0.5",
    )
    mark(f"pure_pursuit_launched_pid={pp_proc.pid}")

    # 2. MoveIt2 move_group — OMPL planner.
    move_group_proc = _ros2_popen(
        "move_group",
        "ros2 launch wdt_manipulation_bringup move_group.launch.py",
    )
    mark(f"move_group_launched_pid={move_group_proc.pid}")

    # 3. Static TF world → cell_cam_optical (replaces synthetic_cell_camera
    #    from v7-v12; the real camera is now spawned in Isaac Sim above).
    #    Quaternion (x=0.917, y=0, z=0, w=-0.397) is rotation by -133.2° about X,
    #    which composes the USD camera's 46.8° X tilt with the optical-frame
    #    Y-flip + Z-flip (USD Y-up → optical Y-down, USD anti-look → optical
    #    forward). Validated against the geometry: cube at world (px+0.40,
    #    py, 0.75) projects to optical (0, 0, 1.097); composing
    #    T_panda_from_world ∘ T_world_from_optical maps that back to
    #    panda_link0 (0.40, 0, -0.25), giving a (0.40, 0, -0.20) grasp pose
    #    after +0.05 standoff — reachable.
    cam_x, cam_y, cam_z = cam_pos_world
    cam_proc = _ros2_popen(
        "cell_cam_static_tf",
        (
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {cam_x} --y {cam_y} --z {cam_z} "
            "--qx 0.917 --qy 0.0 --qz 0.0 --qw -0.397 "
            "--frame-id world --child-frame-id cell_cam_optical"
        ),
    )
    mark(f"cell_cam_static_tf_launched_pid={cam_proc.pid}")

    # 3b. Static TF world → panda_link0. robot_state_publisher (launched via
    # move_group.launch.py) publishes the Franka URDF's panda_link0→panda_link8
    # chain but doesn't anchor the URDF root in world. Without this, the
    # orchestrator's TF lookup cell_cam_optical→panda_link0 fails — both
    # frames need to share a common ancestor. Franka mounted at (px, py, 1.0)
    # upright (no rotation).
    panda_tf_proc = _ros2_popen(
        "panda_link0_static_tf",
        (
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {px} --y {py} --z 1.0 "
            "--qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 "
            "--frame-id world --child-frame-id panda_link0"
        ),
    )
    mark(f"panda_link0_static_tf_launched_pid={panda_tf_proc.pid}")

    # 3c. Franka "ready" /joint_states publisher. MoveIt's planning_scene
    # monitor needs a valid (non-self-colliding, non-singular) start state
    # before it can plan; the Panda's URDF zero-position has panda_link5
    # and panda_link7 in self-collision so every goal returns "Skipping
    # invalid start state". Until Isaac Sim's Franka articulation publishes
    # its own /joint_states (M5b / Phase 3), this static publisher fills
    # the gap. Verified failure mode in M5 v13. Same gotcha as M3 smoke
    # (wdt_vast/moveit_plan_smoke.py).
    js_proc = _ros2_popen(
        "franka_ready_joint_states",
        "/usr/bin/python3 /work/wdt_vast/franka_ready_joint_states.py",
    )
    mark(f"franka_ready_joint_states_launched_pid={js_proc.pid}")

    # 3d. /world/cube_pose static publisher. The orchestrator's
    # GroundTruthPoseSource subscribes to this topic. Kit python (3.11)
    # can't import rclpy (gotcha #18) so we publish from /usr/bin/python3
    # (3.10) with the cube's spawn coords passed as args. plan_only=True
    # in MoveIt makes any physics drift on the resting DynamicCuboid
    # harmless. Only consumed when pose_source=gt; topic is cheap when
    # pose_source=fp.
    cube_pose_proc = _ros2_popen(
        "world_cube_pose",
        "/usr/bin/python3 /work/wdt_vast/sim_world_pose_publisher.py "
        f"--x {cube_center[0]} --y {cube_center[1]} --z {cube_center[2]} "
        f"--frame-id world",
    )
    mark(f"world_cube_pose_launched_pid={cube_pose_proc.pid}")

    # 4. Pick cell orchestrator — subscribes to /cell/cam/* + /cell/start_pick,
    #    runs FP + TopDownGrasp + ArmPlanner (plan_only), publishes
    #    /cell/pick_result. Override cad_path to the synthetic box we
    #    just wrote. --pose-source picks fp (live FoundationPose) or gt
    #    (subscribe to /world/cube_pose).
    orch_proc = _ros2_popen(
        "pick_orch",
        "ros2 run wdt_manipulation_bringup pick_cell_orchestrator "
        f"--ros-args -p cad_path:={m5_cad} "
        f"-p pose_source:={_args.pose_source}",
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

    # Start the simulation timeline. Nova Carter's OmniGraph publishers
    # (TF, cmd_vel subscriber, LIDAR, IMU, cameras) all use
    # OnPlaybackTick triggers — they DON'T fire from world.step() alone.
    # Without world.play(), /amr_0/tf stays empty and pure_pursuit's
    # tf2 buffer reports "base_link frame does not exist". Verified
    # 2026-05-16: M5 v5 hit this; gotcha #3 in
    # feedback-nav2-isaac-sim-gotchas.
    world.play()
    mark("world_playing")

    # Settle physics a beat before order injection starts.
    for _ in range(30):
        world.step(render=True)

    # Optional overhead video capture: spawn a top-down camera + Replicator
    # BasicWriter that drops rgb_NNNN.png frames under <out_dir>/frames/.
    # Gated on scenario.record_video so the smoke (`record_video: false`)
    # skips the IO cost; steady_state (`record_video: true`) records.
    # Step every Nth render tick to throttle wall-fps; 6 ≈ 2 wall-fps at
    # the typical 12 wall-fps render rate.
    video_writer = None
    video_step_every = 6
    if scenario.record_video:
        from sim.overhead_capture import spawn_overhead_capture, step_writer  # noqa: E402

        try:
            video_writer = spawn_overhead_capture(out_dir=str(out_dir))
            mark("overhead_capture_spawned")
        except Exception as exc:  # pragma: no cover — non-fatal
            print(f"[run_scenario] overhead capture failed to spawn: {exc}")

    # Drive the sim for scenario.duration_s of sim time at 30 Hz; inject
    # orders at their arrival_t into the recorder (the coordinator picks
    # them up via its /orders/enqueue subscription once that publisher is
    # wired — for now we just record).
    t = 0.0
    dt = 1.0 / 30.0
    next_order_idx = 0
    sorted_orders = sorted(scenario.orders, key=lambda o: o.arrival_t)

    # Order injection: shell out to `ros2 topic pub --once`. We CAN'T
    # `import rclpy` here — Isaac Sim's kit/python is 3.11 and Humble's
    # rclpy only ships the cpython-310 binding (the
    # `_rclpy_pybind11.cpython-310-…so` mismatch is the root cause of
    # this entire architectural split — see memory note about FP py3.10
    # install). Shell-out via the system /usr/bin/python3-backed ros2
    # CLI sidesteps the issue.

    def _publish_order(order_id: str, shelf_x: float, shelf_y: float) -> None:
        yaml_arg = (
            "{header: {frame_id: " + order_id + "}, "
            f"pose: {{position: {{x: {shelf_x}, y: {shelf_y}, z: 0.0}}, "
            "orientation: {w: 1.0}}}"
        )
        subprocess.Popen(
            [
                "bash",
                "-lc",
                "set +u && source /opt/ros/humble/setup.bash && set -u && "
                "ros2 topic pub --once /orders/enqueue "
                f"geometry_msgs/msg/PoseStamped '{yaml_arg}'",
            ],
            env=ros_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    mark("order_publisher_via_shell_ready")

    while t < scenario.duration_s:
        while next_order_idx < len(sorted_orders) and sorted_orders[next_order_idx].arrival_t <= t:
            o = sorted_orders[next_order_idx]
            recorder.on_order_enqueued(order_id=o.id, at=t)
            _publish_order(o.id, float(o.shelf_xy[0]), float(o.shelf_xy[1]))
            mark(f"order_published_{o.id}")
            next_order_idx += 1
        # render=True regardless of scenario.record_video — Carter's
        # diff_drive OG subscriber fires only on render ticks (gotcha
        # #4 in feedback-nav2-isaac-sim-gotchas). With render=False the
        # AMR ignores all cmd_vel and never moves.
        world.step(render=True)
        # Throttled overhead frame capture (only when scenario.record_video).
        # rep.orchestrator.step() triggers the BasicWriter to flush one
        # rgb_NNNN.png to <out_dir>/frames/. Once the run completes,
        # metrics.video.assemble_mp4(out_dir/"frames", out_dir/"run.mp4")
        # stitches the frames into an MP4 via ffmpeg.
        if video_writer is not None and (int(t / dt) % video_step_every == 0):
            try:
                step_writer()
            except Exception as exc:  # pragma: no cover
                print(f"[run_scenario] step_writer failed: {exc}")
                video_writer = None
        t += dt

    mark(f"loop_done_orders_enqueued={next_order_idx}")

    # Replay pick_orch.log to feed pick_result events into the recorder.
    # The coordinator subscribes to /cell/pick_result and tracks completion
    # internally; run_scenario.py runs in kit python 3.11 which can't
    # import rclpy (gotcha #18), so we can't subscribe directly. Cheapest
    # fix: parse the orchestrator's log file before flushing metrics.
    import re as _re

    pick_log = out_dir / "pick_orch.log"
    if pick_log.exists():
        pick_re = _re.compile(r"pick_result: (\{.*\})")
        for line in pick_log.read_text().splitlines():
            m = pick_re.search(line)
            if not m:
                continue
            try:
                payload = json.loads(m.group(1))
            except json.JSONDecodeError:
                continue
            order_id = payload.get("order_id")
            if order_id and order_id in recorder._orders:
                recorder.on_order_completed(
                    order_id=order_id,
                    at=t,
                    pick_success=bool(payload.get("success", False)),
                    pick_attempts=int(payload.get("attempts", 1)),
                )

    recorder.flush()

    # Assemble the overhead video, if frames were captured. Errors are
    # non-fatal — the metrics.json + events.log are the canonical run
    # output; video is portfolio polish.
    if video_writer is not None:
        try:
            from metrics.video import assemble_mp4  # noqa: E402

            frame_dir = out_dir / "frames"
            mp4_path = out_dir / "run.mp4"
            if any(frame_dir.glob("rgb_*.png")):
                assemble_mp4(frame_dir, mp4_path, fps=10)
                mark(f"video_assembled={mp4_path}")
            else:
                print("[run_scenario] no frames captured; skipping MP4 assembly")
        except Exception as exc:  # pragma: no cover
            print(f"[run_scenario] video assembly failed: {exc}")

    # Shut down ROS2 subprocesses in reverse-launch order. `start_new_session=True`
    # put each child in its own process group, so we must killpg the GROUP not just
    # the bash wrapper (which exits early after exec'ing ros2). Without killpg, the
    # actual ros2 node binaries survive and become orphans for the next run.
    import signal as _signal

    ros2_procs = [coordinator_proc, orch_proc, cam_proc, move_group_proc, pp_proc]
    for proc in ros2_procs:
        try:
            os.killpg(os.getpgid(proc.pid), _signal.SIGTERM)
        except ProcessLookupError:
            pass
    for proc in ros2_procs:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), _signal.SIGKILL)
            except ProcessLookupError:
                pass

    sim.close()
    mark("sim_closed")
    print(json.dumps({"run_dir": str(out_dir), "orders_enqueued": next_order_idx}))
except Exception as e:
    ERROR.write_text(f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
    mark(f"EXCEPTION:{type(e).__name__}")
    raise

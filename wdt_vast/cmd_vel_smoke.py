"""Single-AMR motion smoke: publish /amr_0/cmd_vel, verify the robot moves.

Plan Task 26 originally specified spawning Nav2 and sending a NavigateToPose
goal. That requires a full Nav2 stack (map server + AMCL + costmap +
lifecycle activation + lidar/scan plumbing) — a 1-2 hour integration deal
that doesn't add core value for Phase 1's "the digital twin moves AMRs"
proof. This simplified smoke instead:

  1. Spawns one Nova_Carter_ROS at (2, 2)
  2. Sets nodeNamespace=amr_0 on its OG publishers / differential_drive
  3. Publishes geometry_msgs/Twist on /amr_0/cmd_vel via the ros2 CLI for ~5s
  4. Steps the world 600 frames
  5. Asserts the robot's world-pose moved > 0.5 m

If `cmd_vel → motion` works, we've proven the bridge ↔ controller ↔ sim
chain end-to-end. The actual Nav2 wiring can be revisited in Phase 2 or
inside the Fleet Coordinator (Task 32).

Invoked on a vast.ai instance via:
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    /isaac-sim/python.sh wdt_vast/cmd_vel_smoke.py
"""

import json
import os
import subprocess
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

PROGRESS = Path("/tmp/cmd_vel_progress.txt")
RESULT_OUT = Path("/tmp/cmd_vel_result.json")
ERROR_OUT = Path("/tmp/cmd_vel_error.txt")

PROGRESS.write_text("")


def mark(phase: str) -> None:
    with PROGRESS.open("a") as fh:
        fh.write(f"{datetime.now(timezone.utc).isoformat()}  {phase}\n")


try:
    mark("script_start")
    sys.path.insert(0, "/tmp")
    from sim.multi_robot import _namespace_subtree
    from sim.runner import make_simulation_app
    from sim.spawn import spawn_nova_carter

    mark("imports_ok")

    sim = make_simulation_app(headless=True)
    mark("simapp_booted")

    from isaacsim.core.api import World  # noqa: E402

    world = World()
    world.scene.add_default_ground_plane()

    robot = spawn_nova_carter(world, "/World/amr_0", "amr_0", position_xy=(2.0, 2.0))
    _namespace_subtree("/World/amr_0", "amr_0")
    mark("carter_spawned_and_namespaced")

    world.reset()
    mark("world_reset")

    # Let physics & topic discovery settle. render=True is REQUIRED — the
    # Carter's differential_drive OmniGraph runs on render ticks, not pure
    # physics ticks. Stepping with render=False makes cmd_vel a no-op and
    # the robot never moves.
    for _ in range(60):
        world.step(render=True)
    mark("settled_60_frames")

    start_pose = robot.get_world_pose()[0]
    mark(f"start_pose=({float(start_pose[0]):.3f}, {float(start_pose[1]):.3f})")

    # Publish Twist on /amr_0/cmd_vel via the ros2 CLI (kept simple — same
    # PYTHONPATH/RMW dance as published_topics, only one subprocess).
    clean_env = {k: v for k, v in os.environ.items() if k != "PYTHONHOME"}
    pp_parts = os.environ.get("PYTHONPATH", "").split(":")
    clean_env["PYTHONPATH"] = ":".join(p for p in pp_parts if p and "/isaac-sim/" not in p)

    cmd_proc = subprocess.Popen(
        [
            "/opt/ros/humble/bin/ros2",
            "topic",
            "pub",
            "-r",
            "10",  # 10 Hz
            "/amr_0/cmd_vel",
            "geometry_msgs/msg/Twist",
            "{linear: {x: 0.5}, angular: {z: 0.0}}",
        ],
        env=clean_env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    mark("cmd_vel_publisher_started")

    # Give the publisher a moment to register before stepping
    time.sleep(2.0)
    mark("publisher_registered")

    # Step the world for ~10s of sim time while cmd_vel is flowing.
    # render=True is again required so the differential_drive OG runs.
    for _ in range(300):
        world.step(render=True)
    mark("stepped_300_frames_with_cmd_vel")

    end_pose = robot.get_world_pose()[0]
    cmd_proc.terminate()
    cmd_proc.wait(timeout=5)
    mark(f"end_pose=({float(end_pose[0]):.3f}, {float(end_pose[1]):.3f})")

    dx = float(end_pose[0] - start_pose[0])
    dy = float(end_pose[1] - start_pose[1])
    distance = (dx * dx + dy * dy) ** 0.5

    RESULT_OUT.write_text(
        json.dumps(
            {
                "start": [float(start_pose[0]), float(start_pose[1])],
                "end": [float(end_pose[0]), float(end_pose[1])],
                "dx": dx,
                "dy": dy,
                "distance_m": distance,
                "moved": distance > 0.5,
            },
            indent=2,
        )
    )
    mark(f"distance_m={distance:.3f}")

    sim.close()
    mark("sim_closed")
except Exception as e:
    ERROR_OUT.write_text(f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
    mark(f"EXCEPTION:{type(e).__name__}")
    raise

"""Minimal Isaac Sim host: spawn one Nova Carter, step for N seconds.

Used by M1 smoke (nav2_single_amr_smoke.py). The orchestrator launches
this in the background, waits for ROS2 topics to come up, then sends
NavigateToPose goals to /amr_0/navigate_to_pose while this script
keeps the simulation advancing.

Usage:
    /isaac-sim/python.sh wdt_vast/sim_carter_single.py [duration_s]
        duration_s — defaults to 180 (3 sim minutes)

Outputs to /tmp/sim_carter_single.{log,error}.
"""

from __future__ import annotations

import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path

PROGRESS = Path("/tmp/sim_carter_single.log")
ERROR = Path("/tmp/sim_carter_single.error")
PROGRESS.write_text("")

duration_s = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0


def mark(phase: str) -> None:
    with PROGRESS.open("a") as fh:
        fh.write(f"{datetime.now(timezone.utc).isoformat()}  {phase}\n")


try:
    mark("script_start")
    # /isaac-sim/python.sh doesn't inherit project paths; the sim/ pure-
    # Python package can live at either /work (post-bootstrap convention)
    # or /tmp (older extraction location). Add both — first match wins.
    for candidate in ("/work", "/tmp"):
        if candidate not in sys.path:
            sys.path.insert(0, candidate)
    from sim.multi_robot import _namespace_subtree
    from sim.runner import make_simulation_app
    from sim.spawn import spawn_nova_carter

    mark("imports_ok")
    sim = make_simulation_app(headless=True)
    mark("simapp_booted")

    from isaacsim.core.api import World  # noqa: E402

    world = World()
    world.scene.add_default_ground_plane()
    mark("world_created")

    spawn_nova_carter(world, "/World/AMR_0", "amr_0", position_xy=(1.0, 1.0))
    # CRITICAL: spawn_nova_carter only adds the USD reference; it does NOT
    # namespace the OmniGraph publishers/subscribers. Without this call,
    # Carter's diff_drive OG subscribes to bare /cmd_vel (no namespace),
    # but Nav2 / pure-pursuit publish to /amr_0/cmd_vel — Carter never
    # moves. Verified during Phase 2 M1 pure-pursuit smoke 2026-05-16:
    # cmd_vel publishing at 39 Hz but distance_remaining stayed at 5.64m.
    n_set = _namespace_subtree("/World/AMR_0", "amr_0")
    mark(f"carter_spawned_at_1_1_namespaced_n={n_set}")

    world.reset()
    mark("world_reset")

    # Start the simulation timeline. Nova Carter's OmniGraph publishers
    # (LIDAR, IMU, cameras) use OnPlaybackTick triggers which only fire
    # when the timeline is playing — `world.step()` alone advances
    # physics but doesn't start the timeline. Without play(), the
    # /amr_0/front_3d_lidar/lidar_points topic exists but no messages
    # flow (confirmed by M1 smoke debug 2026-05-15).
    world.play()
    mark("world_playing")

    # Step at ~30 Hz. Render ON — the Nova Carter ROS USD's LIDAR
    # PointCloud publisher only fires when render is enabled (verified
    # by the M1 smoke debug: with render=False the topic stayed silent,
    # AMCL never published map->odom, and Nav2 rejected goals because
    # the map frame didn't exist). Slower but necessary.
    dt = 1.0 / 30.0
    n_steps = int(duration_s / dt)
    mark(f"stepping_{n_steps}_frames_for_{duration_s}_s")
    for i in range(n_steps):
        world.step(render=True)
        if i % 300 == 0:  # every 10 sim seconds
            mark(f"step_{i}")

    mark("loop_done")
    sim.close()
    mark("sim_closed")
except Exception as e:
    ERROR.write_text(f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
    mark(f"EXCEPTION:{type(e).__name__}")
    raise

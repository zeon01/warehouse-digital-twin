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
    sys.path.insert(0, "/tmp")
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
    mark("carter_spawned_at_1_1")

    world.reset()
    mark("world_reset")

    # Step at ~30 Hz. Render off — the orchestrator doesn't need visuals,
    # just physics + ROS2 publishers.
    dt = 1.0 / 30.0
    n_steps = int(duration_s / dt)
    mark(f"stepping_{n_steps}_frames_for_{duration_s}_s")
    for i in range(n_steps):
        world.step(render=False)
        if i % 300 == 0:  # every 10 sim seconds
            mark(f"step_{i}")

    mark("loop_done")
    sim.close()
    mark("sim_closed")
except Exception as e:
    ERROR.write_text(f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
    mark(f"EXCEPTION:{type(e).__name__}")
    raise

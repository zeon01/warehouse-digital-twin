"""Multi-AMR Isaac Sim host: spawn N Carters and step for the duration.

Used by `pure_pursuit_multi_smoke.py` and any other multi-AMR smoke
or scenario runner that needs the full 6-Carter fleet alive on its
own. Mirrors `sim_carter_single.py` but iterates over the layout's
3x2 grid.

Critical: each Carter MUST have its OmniGraph subtree namespaced so
pubs/subs route to /amr_i/... instead of all colliding on /tf,
/cmd_vel, etc. `spawn_amr_fleet` does the spawn but does NOT call
``_namespace_subtree`` per-AMR — that's done here, matching the
fix applied to `sim_carter_single.py` (gotcha #18 in
feedback-foundationpose-install-gotchas).

Usage:
    /isaac-sim/python.sh wdt_vast/sim_fleet.py [duration_s] [num_amrs]
        duration_s — defaults to 180 (3 sim minutes)
        num_amrs   — defaults to 6 (matches layouts/small.yaml)

Outputs:
    /tmp/sim_fleet.{log,error}
"""

from __future__ import annotations

import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path

PROGRESS = Path("/tmp/sim_fleet.log")
ERROR = Path("/tmp/sim_fleet.error")
PROGRESS.write_text("")

duration_s = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
num_amrs = int(sys.argv[2]) if len(sys.argv) > 2 else 6

# Same grid as warehouse/layouts/small.yaml + DEFAULT_SPAWN_POSES in
# wdt_pure_pursuit/launch/multi_amr.launch.py. Keep these three sources
# of truth aligned — if any one shifts, the others must too.
_SPAWN_POSES: list[tuple[float, float]] = [
    (2.0, 2.0),
    (3.5, 2.0),
    (5.0, 2.0),
    (2.0, 3.5),
    (3.5, 3.5),
    (5.0, 3.5),
]


def mark(phase: str) -> None:
    with PROGRESS.open("a") as fh:
        fh.write(f"{datetime.now(timezone.utc).isoformat()}  {phase}\n")


try:
    mark("script_start")
    for candidate in ("/work", "/tmp"):
        if candidate not in sys.path:
            sys.path.insert(0, candidate)
    from sim.multi_robot import _namespace_subtree, spawn_amr_fleet
    from sim.runner import make_simulation_app

    mark("imports_ok")
    sim = make_simulation_app(headless=True)
    mark("simapp_booted")

    from isaacsim.core.api import World  # noqa: E402

    world = World()
    world.scene.add_default_ground_plane()
    mark("world_created")

    poses = _SPAWN_POSES[:num_amrs]
    spawn_amr_fleet(world, poses)
    mark(f"fleet_spawned_n={len(poses)}")

    # spawn_amr_fleet DOESN'T call _namespace_subtree (it's a flat helper
    # that just instantiates Carters). Apply per-AMR namespacing here so
    # diff_drive OG subscribes to /amr_i/cmd_vel and TF publishes to
    # /amr_i/tf. Without this, all 6 Carters share /cmd_vel and only
    # one (the first to subscribe) gets the commands. Bit me with
    # sim_carter_single.py 2026-05-16 — same fix.
    ns_counts = []
    for i in range(len(poses)):
        n = _namespace_subtree(f"/World/amr_{i}", f"amr_{i}")
        ns_counts.append(n)
    mark(f"fleet_namespaced n_per_amr={ns_counts}")

    world.reset()
    mark("world_reset")

    world.play()
    mark("world_playing")

    dt = 1.0 / 30.0
    n_steps = int(duration_s / dt)
    mark(f"stepping_{n_steps}_frames_for_{duration_s}_s")
    for i in range(n_steps):
        world.step(render=True)
        if i % 300 == 0:
            mark(f"step_{i}")

    mark("loop_done")
    sim.close()
    mark("sim_closed")
except Exception as e:
    ERROR.write_text(f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
    mark(f"EXCEPTION:{type(e).__name__}")
    raise

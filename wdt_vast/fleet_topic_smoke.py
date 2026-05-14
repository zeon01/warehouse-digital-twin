"""Spawn 6 Nova Carters per the small layout, verify namespaced topics.

Invoked on a vast.ai instance via:
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    /isaac-sim/python.sh wdt_vast/fleet_topic_smoke.py

Reads /tmp/warehouse/layouts/small.yaml (must be uploaded alongside sim/)
and spawns one Carter per AMR spawn pose, in a `amr_{i}` namespace.

Outputs to:
- /tmp/fleet_progress.txt — phase markers
- /tmp/fleet_topics.json  — full topic list + per-namespace breakdown
"""

import json
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path

PROGRESS = Path("/tmp/fleet_progress.txt")
TOPICS_OUT = Path("/tmp/fleet_topics.json")
ERROR_OUT = Path("/tmp/fleet_error.txt")

PROGRESS.write_text("")


def mark(phase: str) -> None:
    with PROGRESS.open("a") as fh:
        fh.write(f"{datetime.now(timezone.utc).isoformat()}  {phase}\n")


try:
    mark("script_start")
    sys.path.insert(0, "/tmp")
    from sim.multi_robot import spawn_amr_fleet
    from sim.runner import make_simulation_app, published_topics
    from warehouse.layout import load_layout

    mark("imports_ok")

    sim = make_simulation_app(headless=True)
    mark("simapp_booted")

    from isaacsim.core.api import World  # noqa: E402

    world = World()
    world.scene.add_default_ground_plane()

    layout = load_layout("/tmp/warehouse/layouts/small.yaml")
    gx, gy = layout.amrs.spawn.grid
    ox, oy = layout.amrs.spawn.origin_xy
    spacing = layout.amrs.spawn.spacing_m
    poses = [(ox + c * spacing, oy + r * spacing) for r in range(gy) for c in range(gx)][
        : layout.amrs.count
    ]
    mark(f"poses_computed_count={len(poses)}")

    robots = spawn_amr_fleet(world, poses)
    namespacing_counts = [getattr(r, "_namespace_nodes_set", -1) for r in robots]
    mark(f"fleet_spawned_namespacing={namespacing_counts}")

    world.reset()
    mark("world_reset")

    for _ in range(180):
        world.step(render=True)
    mark("stepped_180_frames")

    topics = published_topics(timeout_s=15.0)
    by_namespace: dict[str, list[str]] = {}
    for t in topics:
        # Group amr_N topics by namespace
        parts = t.lstrip("/").split("/", 1)
        if parts[0].startswith("amr_"):
            by_namespace.setdefault(parts[0], []).append(t)

    TOPICS_OUT.write_text(
        json.dumps(
            {
                "all": topics,
                "by_namespace": by_namespace,
                "namespaces_seen": sorted(by_namespace.keys()),
                "amr_count_expected": layout.amrs.count,
            },
            indent=2,
        )
    )
    mark(f"topics_count={len(topics)}_namespaces={len(by_namespace)}")

    sim.close()
    mark("sim_closed")
except Exception as e:
    ERROR_OUT.write_text(f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
    mark(f"EXCEPTION:{type(e).__name__}")
    raise

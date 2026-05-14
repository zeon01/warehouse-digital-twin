"""Spawn one Nova Carter, step the world, dump ROS2 topic list.

Invoked on a vast.ai instance via:
    source /opt/ros/humble/setup.bash
    /isaac-sim/python.sh wdt_vast/carter_topic_smoke.py

Writes outputs to:
- /tmp/carter_progress.txt — phase markers (timestamped)
- /tmp/carter_topics.json  — final published-topics list
- /tmp/carter_error.txt    — traceback if any phase raises

Expected: topics include `/tf`, `/tf_static`, `/clock`, and Carter-specific
odom/cmd_vel topics. If only `/parameter_events` and `/rosout` appear, the
Carter USD's OmniGraph publishers aren't being instantiated — the plan
note for Task 18 says to mirror the Isaac ROS Carter Sample OG graph, but
in practice the Carter USD ships with publishers wired in already.
"""

import json
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path

PROGRESS = Path("/tmp/carter_progress.txt")
TOPICS_OUT = Path("/tmp/carter_topics.json")
ERROR_OUT = Path("/tmp/carter_error.txt")

PROGRESS.write_text("")


def mark(phase: str) -> None:
    with PROGRESS.open("a") as fh:
        fh.write(f"{datetime.now(timezone.utc).isoformat()}  {phase}\n")


try:
    mark("script_start")
    sys.path.insert(0, "/tmp")
    from sim.runner import make_simulation_app, published_topics
    from sim.spawn import spawn_nova_carter

    mark("imports_ok")
    sim = make_simulation_app(headless=True)
    mark("simapp_booted_bridge_enabled")

    from isaacsim.core.api import World  # noqa: E402

    world = World()
    world.scene.add_default_ground_plane()
    mark("world_created")

    spawn_nova_carter(world, "/World/AMR_0", "amr_0", position_xy=(2.0, 2.0))
    mark("carter_spawned")

    world.reset()
    mark("world_reset")

    for _ in range(120):
        world.step(render=True)
    mark("stepped_120_frames")

    topics = published_topics(timeout_s=15.0)
    TOPICS_OUT.write_text(json.dumps(topics, indent=2))
    mark(f"topics_count={len(topics)}")

    sim.close()
    mark("sim_closed")
except Exception as e:
    ERROR_OUT.write_text(f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
    mark(f"EXCEPTION:{type(e).__name__}")
    raise

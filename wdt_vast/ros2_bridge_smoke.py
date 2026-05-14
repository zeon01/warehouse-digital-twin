"""Smoke-test the ROS2 bridge extension: boot Kit + bridge, dump topic list.

Invoked on a vast.ai instance via:
    /isaac-sim/python.sh wdt_vast/ros2_bridge_smoke.py

Writes results to FILES rather than stdout (Python under python.sh has its
stdout block-buffered when piped to a log file, so prints often get
dropped on Kit shutdown):

- /tmp/ros2_progress.txt — one line per phase the script reached
- /tmp/ros2_topics.json  — the final published-topics list

Expected topics include at least `/clock`, `/parameter_events`, `/rosout`
even without any spawned robot. Per-robot topics (`/tf`, `/odom`,
`/cmd_vel`) require explicit OmniGraph publishers — covered in Task 18.
"""

import json
import sys
import traceback
from pathlib import Path

PROGRESS = Path("/tmp/ros2_progress.txt")
TOPICS_OUT = Path("/tmp/ros2_topics.json")
ERROR_OUT = Path("/tmp/ros2_error.txt")


def mark(phase: str) -> None:
    """Append a phase marker (and timestamp) to the progress file."""
    from datetime import datetime, timezone

    with PROGRESS.open("a") as fh:
        fh.write(f"{datetime.now(timezone.utc).isoformat()}  {phase}\n")


# Reset progress file for this run
PROGRESS.write_text("")

try:
    mark("script_start")
    sys.path.insert(0, "/tmp")
    from sim.runner import make_simulation_app, published_topics

    mark("imports_ok")
    sim = make_simulation_app(headless=True)
    mark("simapp_booted")

    from isaacsim.core.api import World  # noqa: E402  (must import after SimulationApp)

    mark("world_imported")
    world = World()
    world.scene.add_default_ground_plane()
    world.reset()
    mark("world_reset")

    for _ in range(60):
        world.step(render=True)
    mark("stepped_60_frames")

    topics = published_topics(timeout_s=10.0)
    mark(f"published_topics_returned_count={len(topics)}")
    TOPICS_OUT.write_text(json.dumps(topics, indent=2))
    mark("topics_written")

    sim.close()
    mark("sim_closed")
except Exception as e:
    ERROR_OUT.write_text(f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}")
    mark(f"EXCEPTION:{type(e).__name__}")
    raise

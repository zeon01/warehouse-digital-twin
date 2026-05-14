"""Boot Isaac Sim Kit headless with the ROS2 bridge enabled, plus helpers.

Module is only callable inside Isaac Sim's Python runtime
(/isaac-sim/kit/python/bin/python3 launched via /isaac-sim/python.sh).

**Invocation requirement:** the calling shell MUST have `ROS_DISTRO=humble`
and the ROS2 library path on `LD_LIBRARY_PATH` before launching python.sh.
The simplest way is to source ROS2 system-wide first:

    source /opt/ros/humble/setup.bash
    /isaac-sim/python.sh wdt_vast/some_script.py

The bridge extension probes ROS_DISTRO during its startup and refuses to
initialize without it. Setting these env vars inside Python (os.environ)
does NOT work because the bridge's native libraries are loaded via dlopen
before user code runs.
"""

from __future__ import annotations


def make_simulation_app(headless: bool = True):
    """Factory: boot Kit and enable the ROS2 bridge extension.

    Returns the `SimulationApp` instance; caller is responsible for
    calling `sim.close()` when done.

    Note on extension name: the canonical name in Isaac Sim 5.0 is
    `isaacsim.ros2.bridge`. The legacy `omni.isaac.ros2_bridge` alias is
    NOT a no-op alias in 5.0 — passing it to `enable_extensions` silently
    fails to load anything (no error, no warning, just an empty topic
    list). Verified by inspecting `/isaac-sim/exts/`: only
    `isaacsim.ros2.bridge` is present.
    """
    from isaacsim import SimulationApp

    sim = SimulationApp({"headless": headless})

    # The `enable_extensions` SimulationApp kwarg is silently ignored for the
    # ROS2 bridge in Isaac Sim 5.0 — verified empirically (no log mentions, no
    # topics). Enabling via the runtime extension manager AFTER boot works.
    from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

    enable_extension("isaacsim.ros2.bridge")

    return sim


def published_topics(timeout_s: float = 5.0) -> list[str]:
    """Poll `ros2 topic list` until non-empty or timeout, return the list.

    Sources `/opt/ros/humble/setup.bash` and the local ROS2 environment
    that Isaac Sim's bridge publishes into. Polls because the bridge can
    take a moment after `world.step(render=True)` calls to actually flush
    topic registrations.
    """
    import subprocess
    import time

    end = time.time() + timeout_s
    last: list[str] = []
    while time.time() < end:
        out = subprocess.run(
            ["bash", "-lc", "source /opt/ros/humble/setup.bash && ros2 topic list"],
            capture_output=True,
            text=True,
            check=False,
        )
        last = [t for t in out.stdout.splitlines() if t.strip()]
        if last:
            return last
        time.sleep(0.5)
    return last

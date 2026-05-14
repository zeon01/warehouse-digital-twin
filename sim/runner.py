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

    Inherits the parent process env (which must already have ROS2 sourced —
    `ROS_DISTRO`, `AMENT_PREFIX_PATH`, `LD_LIBRARY_PATH`, and ideally
    `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to avoid FastDDS conflicts with
    Isaac Sim's bundled DDS). DO NOT use `bash -lc` — login shells reset
    env vars and silently break DDS routing, so the topic list looks empty
    even when the bridge is publishing.
    """
    import os
    import subprocess
    import time

    # Resolve the ros2 binary via PATH (the parent shell sourced ROS2)
    ros2 = "/opt/ros/humble/bin/ros2"
    if not os.path.exists(ros2):
        return []

    # Strip Isaac Sim's Python 3.11 paths out of PYTHONPATH but KEEP the
    # ROS2 entry. The ros2 CLI runs under system /usr/bin/python3 (3.10);
    # if PYTHONPATH includes /isaac-sim/kit/python/lib/python3.11/*, the
    # first `import re` crashes with "SRE module mismatch". If we clear
    # PYTHONPATH entirely, ros2cli can't find its own package metadata
    # and fails with "No package metadata was found for ros2cli".
    pp_parts = os.environ.get("PYTHONPATH", "").split(":")
    pp_clean = ":".join(p for p in pp_parts if p and "/isaac-sim/" not in p)
    clean_env = {k: v for k, v in os.environ.items() if k != "PYTHONHOME"}
    clean_env["PYTHONPATH"] = pp_clean

    end = time.time() + timeout_s
    last: list[str] = []
    out = None
    while time.time() < end:
        out = subprocess.run(
            [ros2, "topic", "list"],
            capture_output=True,
            text=True,
            check=False,
            env=clean_env,
        )
        last = [t for t in out.stdout.splitlines() if t.strip()]
        if last:
            return last
        time.sleep(0.5)
    if out is not None:
        from pathlib import Path

        Path("/tmp/topics_debug.txt").write_text(
            f"returncode={out.returncode}\nstdout={out.stdout!r}\nstderr={out.stderr!r}\n"
        )
    return last

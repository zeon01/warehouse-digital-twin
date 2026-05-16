"""M1 smoke: drive one Carter to a hardcoded pose via full Nav2 stack.

Invoked on a vast.ai instance:
    source /opt/ros/humble/setup.bash
    source /work/ros2_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    /usr/bin/python3 wdt_vast/nav2_single_amr_smoke.py [out_dir]

Flow:
1. Pop a sim host (`sim_carter_single.py`) — spawns Carter at (1, 1).
2. Wait ~45 s for Kit boot + ROS2 bridge to advertise topics.
3. Launch RSP (carter_description) and Nav2 (single_amr) in the amr_0
   namespace.
4. Wait ~25 s for lifecycle activation.
5. Send NavigateToPose to (8, 8) on /amr_0/navigate_to_pose.
6. Print SUCCEEDED / FAILED based on action result.

Outputs to <out_dir>:
    sim.log         — Isaac Sim stdout
    rsp.log         — robot_state_publisher stdout
    nav2.log        — Nav2 launch stdout (full lifecycle, AMCL, DWB)
    goal_result.txt — `ros2 action send_goal --feedback` output

Exit 0 = success, 1 = failure.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

# Isaac Sim 5.0's ROS2 bridge enable_extension hangs without its own lib
# path on LD_LIBRARY_PATH at process-launch (NVIDIA forum 349495).
ISAAC_BRIDGE_LIB = "/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib"


def _sim_env() -> dict:
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ISAAC_BRIDGE_LIB + ":" + env.get("LD_LIBRARY_PATH", "")
    return env


OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/m1_smoke")
OUT.mkdir(parents=True, exist_ok=True)

GOAL_XY = (8.0, 8.0)
NS = "amr_0"
SIM_DURATION_S = 600  # keep sim alive for the whole smoke + nav
SIM_BOOT_S = 45
# AMCL needs to receive several scans before it publishes map->odom;
# 25 s wasn't enough on the first attempt (goal rejected because map
# frame didn't exist yet). Give the TF chain a full minute.
NAV2_ACTIVATE_S = 60
GOAL_TIMEOUT_S = 180


def _popen(cmd: list[str], log_name: str, env: dict | None = None) -> subprocess.Popen:
    log = OUT / log_name
    return subprocess.Popen(cmd, stdout=open(log, "w"), stderr=subprocess.STDOUT, env=env)


def main() -> int:
    print(f"==> M1 smoke -> {OUT}")

    sim = _popen(
        ["/isaac-sim/python.sh", "wdt_vast/sim_carter_single.py", str(SIM_DURATION_S)],
        "sim.log",
        env=_sim_env(),
    )
    print(f"sim pid={sim.pid}, sleeping {SIM_BOOT_S}s for Kit boot")
    time.sleep(SIM_BOOT_S)

    rsp = _popen(
        [
            "ros2",
            "launch",
            "wdt_carter_description",
            "carter_description.launch.py",
            f"robot_namespace:={NS}",
        ],
        "rsp.log",
    )
    nav2 = _popen(
        [
            "ros2",
            "launch",
            "wdt_nav2_bringup",
            "single_amr.launch.py",
            f"robot_namespace:={NS}",
        ],
        "nav2.log",
    )
    print(f"rsp pid={rsp.pid}, nav2 pid={nav2.pid}, sleeping {NAV2_ACTIVATE_S}s for lifecycle")
    time.sleep(NAV2_ACTIVATE_S)

    print(f"==> sending NavigateToPose -> ({GOAL_XY[0]}, {GOAL_XY[1]})")
    goal_yaml = (
        "{pose: {header: {frame_id: map}, "
        f"pose: {{position: {{x: {GOAL_XY[0]}, y: {GOAL_XY[1]}, z: 0.0}}, "
        "orientation: {w: 1.0}}}}"
    )
    goal = subprocess.run(
        [
            "ros2",
            "action",
            "send_goal",
            f"/{NS}/navigate_to_pose",
            "nav2_msgs/action/NavigateToPose",
            goal_yaml,
            "--feedback",
        ],
        capture_output=True,
        text=True,
        timeout=GOAL_TIMEOUT_S,
    )
    (OUT / "goal_result.txt").write_text(goal.stdout + "\n---STDERR---\n" + goal.stderr)

    print("==> shutting down")
    for p in (nav2, rsp, sim):
        p.terminate()
    for p in (nav2, rsp, sim):
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()

    out = goal.stdout
    if "SUCCEEDED" in out or "Result: success" in out.lower():
        print("M1 SMOKE PASS")
        return 0
    print("M1 SMOKE FAIL")
    print(out[-2000:])
    return 1


if __name__ == "__main__":
    sys.exit(main())

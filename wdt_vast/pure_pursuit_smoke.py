"""M1 fallback smoke: drive Carter via the pure-pursuit driver, not Nav2.

Replaces Nav2's controller + planner + bt_navigator + behavior_server
with a single ``pure_pursuit_driver`` node that publishes cmd_vel via
a hand-rolled go-to-goal control law. Same /<ns>/navigate_to_pose
action name, so the action client (or fleet_coordinator) is unchanged.

Used when ``nav2_cmd_vel_probe.py`` returns the "DWB never publishes"
verdict — the costmap depends on a LIDAR publisher that doesn't fire
on Isaac Sim's standalone-python Nova Carter, so DWB scores every
trajectory as obstructed. Skipping Nav2's controller entirely takes
the costmap out of the picture.

Outputs to <out_dir>:
    sim.log, rsp.log, pp.log       — process logs
    goal_result.txt                — action result
    pose_trace.txt                 — tf echo samples during execution

Invoke:
    /usr/bin/python3 wdt_vast/pure_pursuit_smoke.py [out_dir]
"""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/pp_smoke")
OUT.mkdir(parents=True, exist_ok=True)

GOAL_XY = (5.0, 5.0)  # far enough to require real travel but well inside the map
NS = "amr_0"
SIM_DURATION_S = 600
SIM_BOOT_S = 45
PP_ACTIVATE_S = 10  # pure-pursuit has no lifecycle — instantly ready
GOAL_TIMEOUT_S = 90


def _popen(cmd: list[str], log_name: str) -> subprocess.Popen:
    log = OUT / log_name
    return subprocess.Popen(cmd, stdout=open(log, "w"), stderr=subprocess.STDOUT)


def main() -> int:
    print(f"==> pure-pursuit smoke -> {OUT}")

    sim = _popen(
        ["/isaac-sim/python.sh", "wdt_vast/sim_carter_single.py", str(SIM_DURATION_S)],
        "sim.log",
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
    pp = _popen(
        [
            "ros2",
            "launch",
            "wdt_pure_pursuit",
            "single_amr.launch.py",
            f"robot_namespace:={NS}",
        ],
        "pp.log",
    )
    print(f"rsp pid={rsp.pid}, pp pid={pp.pid}, sleeping {PP_ACTIVATE_S}s for tf + action server")
    time.sleep(PP_ACTIVATE_S)

    print(f"==> sending NavigateToPose -> ({GOAL_XY[0]}, {GOAL_XY[1]})")
    goal_yaml = (
        "{pose: {header: {frame_id: map}, "
        f"pose: {{position: {{x: {GOAL_XY[0]}, y: {GOAL_XY[1]}, z: 0.0}}, "
        "orientation: {w: 1.0}}}}"
    )
    goal_proc = subprocess.Popen(
        [
            "ros2",
            "action",
            "send_goal",
            f"/{NS}/navigate_to_pose",
            "nav2_msgs/action/NavigateToPose",
            goal_yaml,
            "--feedback",
        ],
        stdout=open(OUT / "goal_result.txt", "w"),
        stderr=subprocess.STDOUT,
    )

    # Trace pose every ~3 s for the duration of the goal — useful for
    # confirming actual motion vs. the action-result success-claim.
    pose_log = open(OUT / "pose_trace.txt", "w")
    deadline = time.monotonic() + GOAL_TIMEOUT_S
    while time.monotonic() < deadline and goal_proc.poll() is None:
        echo = subprocess.run(
            ["ros2", "topic", "echo", "/tf", "--once"],
            capture_output=True,
            text=True,
            timeout=4,
        )
        pose_log.write(f"--- t={time.monotonic():.1f}\n{echo.stdout}\n")
        pose_log.flush()
        time.sleep(3.0)
    pose_log.close()

    try:
        goal_proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        goal_proc.terminate()

    goal_text = (OUT / "goal_result.txt").read_text() if (OUT / "goal_result.txt").exists() else ""

    print("==> shutting down")
    for p in (pp, rsp, sim):
        p.terminate()
    for p in (pp, rsp, sim):
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()

    if "SUCCEEDED" in goal_text:
        print("PURE-PURSUIT SMOKE PASS")
        return 0
    print("PURE-PURSUIT SMOKE FAIL")
    print(goal_text[-2000:])
    return 1


if __name__ == "__main__":
    sys.exit(main())

"""M1 cmd_vel probe: diagnose where the Nav2 → Carter chain breaks.

Last session's M1 smoke ended with the Nav2 stack accepting goals
("Begin navigating from (1.01, 1.00) to (8.00, 8.00)") but Carter
moving only ~4 cm in 3 minutes. Two suspects:

    (a) DWB never publishes cmd_vel — costmap obstacle layer rejects
        every trajectory because the Carter LIDAR publisher doesn't
        fire (Phase 2 known gap, see feedback-nav2-isaac-sim-gotchas
        gotcha #5).
    (b) DWB publishes cmd_vel but Carter's differential_drive
        OmniGraph subscriber isn't applying velocities (would be a
        Phase 1 regression — cmd_vel_smoke.py proved Carter CAN move).

This probe disambiguates by:

    1. Spawning the same sim + Nav2 stack as nav2_single_amr_smoke.py
    2. Sending a *closer* goal at (2, 1) — within 1 m of spawn, rules
       out far-edge reachability / costmap-corner DWB scoring.
    3. While the goal is active, running ``ros2 topic hz`` and
       ``ros2 topic echo`` on /amr_0/cmd_vel for 30 s in parallel.

    Verdict:
        hz reports ≥ 1 Hz   → DWB is publishing, Carter subscriber broken
        hz reports 0 Hz     → DWB never publishes, costmap/DWB issue
        echo shows >0 lin.x → DWB intends motion (regardless of hz noise)

Outputs to <out_dir>:
    sim.log, rsp.log, nav2.log     — as in nav2_single_amr_smoke.py
    goal_result.txt                — action result
    cmd_vel_hz.txt                 — ros2 topic hz output (30 s window)
    cmd_vel_echo.txt               — first ~10 Twist messages
    verdict.txt                    — automated probe verdict

Invoke (after the bootstrap chain + sim is reachable):
    /usr/bin/python3 wdt_vast/nav2_cmd_vel_probe.py [out_dir]
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import time
from pathlib import Path

# Isaac Sim 5.0's `enable_extension("isaacsim.ros2.bridge")` hangs silently
# unless the bridge's own shared-library path is on LD_LIBRARY_PATH at
# process-launch time (setting it inside Python is too late; dlopen caches
# at process start). Per NVIDIA forum thread 349495 (May 2026). Without
# this, sim_carter_single.py hangs after Kit "app ready", never publishes
# odom→base_link, and Nav2 sees two unconnected TF trees.
ISAAC_BRIDGE_LIB = "/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib"


def _sim_env() -> dict:
    """Env dict to pass to subprocess.Popen for any /isaac-sim/python.sh call.

    Prepends Isaac Sim's ROS2 bridge lib path to LD_LIBRARY_PATH so the
    bridge extension can dlopen its native libs.
    """
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ISAAC_BRIDGE_LIB + ":" + env.get("LD_LIBRARY_PATH", "")
    return env


OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/m1_cmd_vel_probe")
OUT.mkdir(parents=True, exist_ok=True)

# Close-in goal: 1 m east, 0 m north from spawn (1, 1) → goal (2, 1).
# Spawn-corner reachability is the simplest hypothesis to rule out.
GOAL_XY = (2.0, 1.0)
NS = "amr_0"
SIM_DURATION_S = 600
SIM_BOOT_S = 45
NAV2_ACTIVATE_S = 60
HZ_WINDOW_S = 30
GOAL_TIMEOUT_S = 90


def _popen(cmd: list[str], log_name: str, env: dict | None = None) -> subprocess.Popen:
    log = OUT / log_name
    return subprocess.Popen(cmd, stdout=open(log, "w"), stderr=subprocess.STDOUT, env=env)


def _capture(cmd: list[str], timeout_s: float) -> str:
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        return proc.stdout + "\n---STDERR---\n" + proc.stderr
    except subprocess.TimeoutExpired as e:

        def _decode(buf) -> str:
            if buf is None:
                return ""
            if isinstance(buf, bytes | bytearray):
                return buf.decode(errors="replace")
            return str(buf)

        return _decode(e.stdout) + "\n---TIMEOUT---\n" + _decode(e.stderr)


def _verdict(hz_text: str, echo_text: str, goal_text: str) -> str:
    hz_match = re.search(r"average rate:\s*([\d.]+)", hz_text)
    avg_hz = float(hz_match.group(1)) if hz_match else 0.0

    linear_match = re.findall(r"linear:\s*\n\s*x:\s*(-?[\d.]+)", echo_text)
    max_lin = max((abs(float(x)) for x in linear_match), default=0.0)

    lines = [f"cmd_vel avg_hz = {avg_hz:.2f}", f"max |linear.x| seen = {max_lin:.3f} m/s"]

    if avg_hz < 0.5 and max_lin < 0.01:
        lines.append("VERDICT: DWB never published cmd_vel — Nav2 controller / costmap issue")
        lines.append("Next step: pure-pursuit fallback (run wdt_vast/pure_pursuit_smoke.py)")
    elif avg_hz >= 1.0 and max_lin > 0.01:
        lines.append("VERDICT: DWB IS publishing cmd_vel — Carter subscriber not applying")
        lines.append(
            "Next step: verify diff_drive OmniGraph nodeNamespace, check /amr_0/cmd_vel subs"
        )
    else:
        lines.append(f"VERDICT: AMBIGUOUS (hz={avg_hz:.2f}, max_lin={max_lin:.3f})")
        lines.append("Manual inspection needed — see cmd_vel_hz.txt + cmd_vel_echo.txt")

    if "SUCCEEDED" in goal_text:
        lines.append("Note: NavigateToPose action SUCCEEDED — Carter did reach the goal")
    return "\n".join(lines) + "\n"


def main() -> int:
    print(f"==> M1 cmd_vel probe -> {OUT}")

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

    print(f"==> sending NavigateToPose -> ({GOAL_XY[0]}, {GOAL_XY[1]}) [close-in probe]")
    goal_yaml = (
        "{pose: {header: {frame_id: map}, "
        f"pose: {{position: {{x: {GOAL_XY[0]}, y: {GOAL_XY[1]}, z: 0.0}}, "
        "orientation: {w: 1.0}}}}"
    )
    # Fire the goal in the background so the hz/echo probes can run while
    # the action is active.
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

    # Let DWB warm up — first cmd_vel typically appears 2-3 s post-goal.
    time.sleep(3.0)

    print(f"==> probing /{NS}/cmd_vel for {HZ_WINDOW_S}s")
    # Run hz + echo in parallel via two subprocesses.
    hz_proc = subprocess.Popen(
        ["ros2", "topic", "hz", f"/{NS}/cmd_vel"],
        stdout=open(OUT / "cmd_vel_hz.txt", "w"),
        stderr=subprocess.STDOUT,
    )
    echo_text = _capture(
        ["ros2", "topic", "echo", f"/{NS}/cmd_vel", "--once"],
        timeout_s=HZ_WINDOW_S,
    )
    # echo --once only grabs one; for a fuller sample, do 5 more echoes.
    for _ in range(5):
        echo_text += "---\n" + _capture(
            ["ros2", "topic", "echo", f"/{NS}/cmd_vel", "--once"],
            timeout_s=5,
        )
    (OUT / "cmd_vel_echo.txt").write_text(echo_text)

    # Let hz collect at least HZ_WINDOW_S of data.
    time.sleep(HZ_WINDOW_S)
    hz_proc.terminate()
    try:
        hz_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        hz_proc.kill()

    # Wait for the goal to resolve (or time out).
    try:
        goal_proc.wait(timeout=GOAL_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        goal_proc.terminate()

    goal_text = (OUT / "goal_result.txt").read_text() if (OUT / "goal_result.txt").exists() else ""
    hz_text = (OUT / "cmd_vel_hz.txt").read_text() if (OUT / "cmd_vel_hz.txt").exists() else ""

    verdict = _verdict(hz_text, echo_text, goal_text)
    (OUT / "verdict.txt").write_text(verdict)
    print("==> verdict:\n" + verdict)

    print("==> shutting down")
    for p in (nav2, rsp, sim):
        p.terminate()
    for p in (nav2, rsp, sim):
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()

    return 0


if __name__ == "__main__":
    sys.exit(main())

"""M2 smoke: drive N Carters concurrently via pure-pursuit drivers.

Spawn the 6-AMR fleet, launch the multi-AMR pure-pursuit stack, then
send NavigateToPose goals to all `/amr_i/navigate_to_pose` action
servers in parallel. Each Carter gets its own goal offset from its
spawn pose (no two AMRs share a destination, avoids contention).

Pass criterion: every per-AMR action ends in SUCCEEDED within
GOAL_TIMEOUT_S. Partial fails are reported with per-AMR distance
diagnostics.

Outputs to <out_dir>:
    sim.log, pp.log               — process logs
    goal_amr_<i>.txt              — per-AMR action result
    summary.txt                   — pass/fail summary

Invoke:
    /usr/bin/python3 wdt_vast/pure_pursuit_multi_smoke.py [out_dir]
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

ISAAC_BRIDGE_LIB = "/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib"


def _sim_env() -> dict:
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ISAAC_BRIDGE_LIB + ":" + env.get("LD_LIBRARY_PATH", "")
    return env


OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/pp_multi_smoke")
OUT.mkdir(parents=True, exist_ok=True)

# Must match wdt_pure_pursuit/launch/multi_amr.launch.py
# DEFAULT_SPAWN_POSES and warehouse/layouts/small.yaml. If you change
# one, change all three.
SPAWN_POSES: list[tuple[float, float]] = [
    (2.0, 2.0),
    (3.5, 2.0),
    (5.0, 2.0),
    (2.0, 3.5),
    (3.5, 3.5),
    (5.0, 3.5),
]
NUM_AMRS = len(SPAWN_POSES)

# Each AMR drives diagonally +3, +3 from its spawn (~4.2 m). All goals
# stay well inside the 20x30 m warehouse and far enough that motion is
# non-trivial. 6 m offset was tried first but at 6 Carters rendering on
# one GPU the sim is too slow to cover that within a reasonable smoke
# budget — 3 m keeps each AMR's drive under ~3 min wall.
GOAL_OFFSET: tuple[float, float] = (3.0, 3.0)

SIM_DURATION_S = 900
SIM_BOOT_S = 45
PP_ACTIVATE_S = 12  # slightly longer than single-AMR — 6 drivers all spinning up
# Wall-clock budget per goal. With 6 Carters rendering at full RTX
# rates, the sim runs ~10-20x slower than real-time on RTX 3090; a
# 6m diagonal at 0.5 m/s sim-time = ~12 s SIM-time = ~120-240 s WALL.
# Budget 480 s per goal to leave margin for the slowest AMR.
GOAL_TIMEOUT_S = 480


def _popen(cmd: list[str], log_path: Path, env: dict | None = None) -> subprocess.Popen:
    return subprocess.Popen(cmd, stdout=open(log_path, "w"), stderr=subprocess.STDOUT, env=env)


def _send_goal(amr_idx: int, goal_xy: tuple[float, float]) -> subprocess.Popen:
    goal_yaml = (
        "{pose: {header: {frame_id: map}, "
        f"pose: {{position: {{x: {goal_xy[0]}, y: {goal_xy[1]}, z: 0.0}}, "
        "orientation: {w: 1.0}}}}"
    )
    return subprocess.Popen(
        [
            "ros2",
            "action",
            "send_goal",
            f"/amr_{amr_idx}/navigate_to_pose",
            "nav2_msgs/action/NavigateToPose",
            goal_yaml,
            "--feedback",
        ],
        stdout=open(OUT / f"goal_amr_{amr_idx}.txt", "w"),
        stderr=subprocess.STDOUT,
    )


def main() -> int:
    print(f"==> M2 pure-pursuit multi smoke -> {OUT}")

    sim = _popen(
        [
            "/isaac-sim/python.sh",
            "wdt_vast/sim_fleet.py",
            str(SIM_DURATION_S),
            str(NUM_AMRS),
        ],
        OUT / "sim.log",
        env=_sim_env(),
    )
    print(f"sim pid={sim.pid}, sleeping {SIM_BOOT_S}s for Kit boot + fleet namespacing")
    time.sleep(SIM_BOOT_S)

    # Pass goal_timeout_s through to per-AMR pp_driver — multi-AMR sim
    # runs ~10x slower than realtime so the 60-s default would abort
    # before any AMR can complete a 6 m drive.
    pp = _popen(
        [
            "ros2",
            "launch",
            "wdt_pure_pursuit",
            "multi_amr.launch.py",
            f"goal_timeout_s:={GOAL_TIMEOUT_S}",
        ],
        OUT / "pp.log",
    )
    print(f"pp pid={pp.pid}, sleeping {PP_ACTIVATE_S}s for {NUM_AMRS} action servers")
    time.sleep(PP_ACTIVATE_S)

    print(f"==> sending {NUM_AMRS} NavigateToPose goals (concurrent)")
    goal_procs: list[subprocess.Popen] = []
    for i, spawn in enumerate(SPAWN_POSES):
        goal_xy = (spawn[0] + GOAL_OFFSET[0], spawn[1] + GOAL_OFFSET[1])
        print(f"  amr_{i}: {spawn} -> {goal_xy}")
        goal_procs.append(_send_goal(i, goal_xy))

    print(f"==> waiting up to {GOAL_TIMEOUT_S}s for all goals to complete")
    deadline = time.monotonic() + GOAL_TIMEOUT_S
    for i, proc in enumerate(goal_procs):
        remaining = max(0.0, deadline - time.monotonic())
        try:
            proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            print(f"  amr_{i}: action timed out, terminating")
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    print("==> tallying results")
    results = {}
    for i in range(NUM_AMRS):
        result_file = OUT / f"goal_amr_{i}.txt"
        text = result_file.read_text() if result_file.exists() else ""
        succeeded = "SUCCEEDED" in text
        results[f"amr_{i}"] = succeeded

    pass_count = sum(1 for ok in results.values() if ok)
    summary_lines = [f"PASS {pass_count}/{NUM_AMRS}"]
    for amr_id, ok in results.items():
        summary_lines.append(f"  {amr_id}: {'SUCCEEDED' if ok else 'FAILED/TIMEOUT'}")
    summary = "\n".join(summary_lines) + "\n"
    (OUT / "summary.txt").write_text(summary)
    print("\n==> summary:\n" + summary)

    print("==> shutting down")
    for p in (pp, sim):
        p.terminate()
    for p in (pp, sim):
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()

    if pass_count == NUM_AMRS:
        print("M2 MULTI-AMR SMOKE PASS")
        return 0
    print("M2 MULTI-AMR SMOKE FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())

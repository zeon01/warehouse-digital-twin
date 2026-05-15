"""Drive the 15-run planner ablation grid on vast.ai.

Iterates ``CONFIGS × SEEDS`` and invokes ``run_scenario.py`` for each
``(config, seed)`` pair. Outputs land under
``<out_root>/<config>/<seed>/`` with one ``metrics.json`` per run, which
``metrics.aggregate.aggregate_runs`` consumes in Task 41+.

Usage (on the vast.ai instance):

    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python wdt_vast/run_ablation.py \\
        --scenario scenarios/steady_state.yaml \\
        --out-root /tmp/ablation_runs \\
        2>&1 | tee /tmp/ablation.log

Wall-clock budget: ~30-45 min per run × 15 runs = 7.5-11 hr. Plan for
overnight; the runlog file lets a re-run resume past completed cells
with ``--skip-existing``.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

CONFIGS = [
    ("greedy_greedy", "greedy", "greedy"),
    ("hungarian_greedy", "hungarian", "greedy"),
    ("hungarian_cbs", "hungarian", "cbs"),
]
SEEDS = [42, 43, 44, 45, 46]


def main() -> int:
    parser = argparse.ArgumentParser(prog="run_ablation")
    parser.add_argument(
        "--scenario",
        default="scenarios/steady_state.yaml",
        help="path to scenario YAML to run for every (config, seed)",
    )
    parser.add_argument(
        "--out-root",
        default="runs",
        help="root directory; each run writes to <out_root>/<config>/<seed>/",
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="skip runs whose metrics.json already exists",
    )
    parser.add_argument(
        "--isaac-python",
        default="/isaac-sim/python.sh",
        help="path to Isaac Sim's python.sh (overridable for local dry-runs)",
    )
    args = parser.parse_args()

    out_root = Path(args.out_root)
    out_root.mkdir(parents=True, exist_ok=True)
    runlog = out_root / "_runlog.txt"

    n_done = 0
    n_fail = 0
    for config_name, alloc, planner in CONFIGS:
        for seed in SEEDS:
            run_dir = out_root / config_name / str(seed)
            if args.skip_existing and (run_dir / "metrics.json").exists():
                print(f"SKIP {config_name} seed={seed} (metrics.json exists)")
                continue
            run_dir.mkdir(parents=True, exist_ok=True)
            print(f"\n=== {config_name} seed={seed} ===", flush=True)
            t0 = time.time()
            result = subprocess.run(
                [
                    args.isaac_python,
                    "wdt_vast/run_scenario.py",
                    args.scenario,
                    str(run_dir),
                    "--allocator",
                    alloc,
                    "--path-planner",
                    planner,
                    "--seed",
                    str(seed),
                ],
                check=False,
            )
            dt = time.time() - t0
            status = "OK" if result.returncode == 0 else "FAIL"
            print(
                f"{status} {config_name} seed={seed} took {dt / 60:.1f} min",
                flush=True,
            )
            with runlog.open("a") as f:
                f.write(f"{config_name} {seed} {status} {dt:.1f}\n")
            if result.returncode == 0:
                n_done += 1
            else:
                n_fail += 1

    print(f"\nablation complete: {n_done} OK / {n_fail} FAIL / {len(CONFIGS) * len(SEEDS)} total")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

# Warehouse Digital Twin — Phase 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. For *this* project the user has explicitly chosen direct (in-session) execution — see [[feedback-execution-mode]] memory. Subagents are still useful for cross-cutting code review at milestone boundaries.

**Goal:** Ship `v0.2.0` — close Phase 1's two integration gaps (real Nav2 stack, real MoveIt2 + FoundationPose manipulation), then run a 3-config × 5-seed planner ablation on the existing 64-order `steady_state.yaml` scenario and publish defensible portfolio numbers.

**Architecture:** Build atop Phase 1's `v0.1.0` skeleton. Add `wdt_carter_description`, `wdt_franka_description`, `wdt_nav2_bringup`, `wdt_manipulation_bringup` ROS2 packages. Pre-bake a 2D occupancy grid from the procedural USD warehouse for Nav2's `map_server`. Compile FoundationPose CUDA wheels once on Modal, distribute to vast.ai via Modal Volume. Wire the existing `fleet_coordinator` to real Nav2 action results and to a new `pick_cell_orchestrator` node that runs the manipulation pipeline. Add ablation runner that loops `(allocator, path_planner, seed)` triples via the existing `wdt_vast/run_scenario.py` entrypoint.

**Tech Stack:** Python 3.10, NVIDIA Isaac Sim 5.0 (headless on vast.ai), ROS2 Humble, Nav2 (full stack — `map_server`, `amcl`, `planner_server`, `controller_server`, `bt_navigator`, `lifecycle_manager`), MoveIt2 + `moveit_py`, FoundationPose (pre-compiled CUDA wheels), Modal (compute for wheel builds + budget), vast.ai (Isaac Sim runtime), pytest, ruff, matplotlib.

**Phase 3 (scale-up to 12–20 AMRs, 50×50 m warehouse, live web dashboard) is out of scope for this plan.** A separate plan will be written for Phase 3 after Phase 2 ships.

---

## Pre-flight notes for the implementing engineer

- **Engineer environment:** macOS Mac, vast.ai instance `36775999` (Romania RTX A5000, driver 570.211) currently stopped — resume with `vastai start instance 36775999`. Modal authenticated (two accounts, account `saad-19015` has $28.61 remaining, second account fresh $30). GitHub authenticated as `zeon01`. `gh` CLI available. Python 3.10+ via pyenv/uv.
- **Repo state:** `main` at `v0.1.0` (commit `cafc53d`); Phase 1 plan is `docs/superpowers/plans/2026-05-14-warehouse-digital-twin-phase-1.md`; Phase 2 spec is `docs/superpowers/specs/2026-05-15-warehouse-digital-twin-phase-2-design.md`. Read the spec before starting.
- **Existing structure (don't relitigate):**
  - `coordinator/` — `assignment.py` (Hungarian), `cbs.py` (CBS), `deadlock.py`, `strategy.py` (path-planner registry).
  - `manipulation/` — `pose_estimation.py` (FoundationPose wrapper stub), `grasping.py` (AnyGrasp wrapper stub), `motion_planning.py` (MoveIt2 wrapper stub), `pipeline.py` (composition with bounded retries — already TDD'd).
  - `ros2_ws/src/` — `fleet_coordinator`, `warehouse_bringup`. New Phase 2 packages go alongside these.
  - `wdt_vast/` — `run_scenario.py` (orchestrator), various smokes. The ablation runner is added here.
  - `warehouse/generators/` — procedural USD builder. Map exporter is added here.
- **vast.ai discipline (load-bearing):** per [[feedback-vastai-log-streaming]] memory, long SSH commands MUST `tee` to a remote log file + tail via separate SSH piped through Monitor. Never trust SSH-stdout buffering.
- **Cost ceiling:** Total Phase 2 spend ≤ $15. Stop the vast.ai instance whenever idle (Pattern 3, idle ~$0.025/hr; running ~$0.30–0.40/hr).
- **Commit cadence:** Every task ends with a commit. Push at end of each milestone (every 5–7 tasks). Use Conventional Commits style (`feat:`, `test:`, `fix:`, `docs:`, `chore:`).
- **When to skip TDD:** ROS2 launch files, URDF/SRDF, YAML configs, and one-shot shell scripts don't need failing-test-first. For Python logic (map exporter, TopDownGrasp, aggregator), always TDD.

---

## Milestone 0 — Map generation + Carter URDF/ROS2 wiring

Spec §4.1 + §4.2 (Carter half). Output: PGM occupancy grid for the `small` layout committed to repo; `wdt_carter_description` ROS2 package built on vast.ai; single Carter visible in `rviz2` with TF + scan.

### Task 1: OccupancyGridExporter — failing test

**Files:**
- Create: `tests/unit/test_map_export.py`

- [ ] **Step 1: Write failing test**

```python
"""Unit tests for warehouse.generators.map_export."""

from __future__ import annotations

import numpy as np
import pytest


def test_rasterize_single_obstacle_box():
    from warehouse.generators.map_export import rasterize_obstacles

    # 10m × 10m world, 5cm/px → 200×200 grid
    obstacles = [{"x_min": 4.0, "x_max": 5.0, "y_min": 4.0, "y_max": 5.0}]
    grid = rasterize_obstacles(
        world_w_m=10.0,
        world_h_m=10.0,
        resolution_m_per_px=0.05,
        obstacles=obstacles,
    )

    assert grid.shape == (200, 200)
    # 1m × 1m obstacle = 20×20 cells = 400 occupied px
    assert int((grid == 100).sum()) == 400
    # All other cells are free (0)
    assert int((grid == 0).sum()) == 200 * 200 - 400


def test_rasterize_multiple_obstacles():
    from warehouse.generators.map_export import rasterize_obstacles

    obstacles = [
        {"x_min": 0.0, "x_max": 1.0, "y_min": 0.0, "y_max": 1.0},
        {"x_min": 2.0, "x_max": 3.0, "y_min": 2.0, "y_max": 3.0},
    ]
    grid = rasterize_obstacles(
        world_w_m=5.0,
        world_h_m=5.0,
        resolution_m_per_px=0.1,
        obstacles=obstacles,
    )
    assert grid.shape == (50, 50)
    assert int((grid == 100).sum()) == 2 * 10 * 10  # two 1m² obstacles at 10 px/m


def test_rasterize_obstacle_outside_world_clipped():
    from warehouse.generators.map_export import rasterize_obstacles

    obstacles = [{"x_min": 9.0, "x_max": 11.0, "y_min": 4.0, "y_max": 5.0}]
    grid = rasterize_obstacles(
        world_w_m=10.0, world_h_m=10.0, resolution_m_per_px=0.05, obstacles=obstacles
    )
    # Clipped to x∈[9,10] = 1m wide = 20 cells. y is 1m = 20 cells. 400 px.
    assert int((grid == 100).sum()) == 400
```

- [ ] **Step 2: Run test, verify failure**

```bash
pytest tests/unit/test_map_export.py -v
```

Expected: `ModuleNotFoundError: No module named 'warehouse.generators.map_export'`

- [ ] **Step 3: Commit failing test**

```bash
git add tests/unit/test_map_export.py
git commit -m "test(map): failing tests for occupancy-grid rasterizer (Task 1)"
```

### Task 2: OccupancyGridExporter — rasterizer implementation

**Files:**
- Create: `warehouse/generators/map_export.py`

- [ ] **Step 1: Implement rasterizer**

```python
"""Rasterize procedural warehouse obstacles into a 2D occupancy grid.

The Nav2 `map_server` consumes a (PGM, YAML) pair where the PGM is a
grayscale image (0 = free, 100 = occupied, 255 = unknown) and the YAML
points to it plus declares resolution + origin in world coordinates. We
produce both from the procedural layout so Nav2's planner has a
consistent view of the warehouse.

Grid layout: row 0 is the bottom of the world (y=0); row H-1 is the top.
Column 0 is x=0; column W-1 is x=world_w_m. This matches the YAML
convention `origin: [0.0, 0.0, 0.0]` (world origin at bottom-left of grid).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

OCCUPIED = 100
FREE = 0


def rasterize_obstacles(
    world_w_m: float,
    world_h_m: float,
    resolution_m_per_px: float,
    obstacles: list[dict],
) -> np.ndarray:
    """Return a (H, W) uint8 grid where W = world_w_m/res, H = world_h_m/res.

    Each obstacle is a dict with keys x_min, x_max, y_min, y_max (meters).
    Obstacles clipping the world boundary are silently truncated.
    """
    res = resolution_m_per_px
    w = int(round(world_w_m / res))
    h = int(round(world_h_m / res))
    grid = np.zeros((h, w), dtype=np.uint8)

    for obs in obstacles:
        x0 = max(0, int(np.floor(obs["x_min"] / res)))
        x1 = min(w, int(np.ceil(obs["x_max"] / res)))
        y0 = max(0, int(np.floor(obs["y_min"] / res)))
        y1 = min(h, int(np.ceil(obs["y_max"] / res)))
        grid[y0:y1, x0:x1] = OCCUPIED

    return grid


def write_pgm(grid: np.ndarray, path: Path) -> None:
    """Write a Nav2-compatible PGM. 0 = free, 100 = occupied, 255 = unknown.

    Nav2 uses `negate: 0` semantics by default — lower values = more occupied
    in the raw PGM, but we override that with negate in the YAML below so
    OCCUPIED = 100 reads as occupied directly.
    """
    h, w = grid.shape
    # PGM rows go top-to-bottom in image space; we flip so row 0 of our
    # bottom-up grid lands at the bottom of the PGM.
    img = np.flipud(grid)
    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    path.write_bytes(header + img.tobytes())


def write_map_yaml(
    pgm_filename: str,
    resolution_m_per_px: float,
    origin_xy_yaw: tuple[float, float, float],
    path: Path,
) -> None:
    """Write the Nav2 map YAML next to the PGM.

    The PGM filename is stored *relative* to the YAML so Nav2's loader
    works regardless of absolute path on the runtime host.
    """
    data = {
        "image": pgm_filename,
        "resolution": resolution_m_per_px,
        "origin": list(origin_xy_yaw),
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False))
```

- [ ] **Step 2: Run tests, verify pass**

```bash
pytest tests/unit/test_map_export.py -v
```

Expected: 3 passed.

- [ ] **Step 3: Commit**

```bash
git add warehouse/generators/map_export.py
git commit -m "feat(map): rasterize obstacles to 2D occupancy grid (Task 2)"
```

### Task 3: Wire map export into `build_scene` CLI

**Files:**
- Modify: `warehouse/generators/build_scene.py` (or wherever the existing CLI lives)
- Test: `tests/unit/test_build_scene_map.py`

- [ ] **Step 1: Inspect the existing builder to find obstacle metadata**

```bash
grep -n "shelf\|column\|obstacle\|wall" warehouse/generators/build_scene.py
grep -n "shelf\|wall" warehouse/layout.py
```

You'll find the procedural builder emits shelves at fixed (x, y) positions per the loaded `Layout`. Extract or pass through the shelf dimensions to a list of `{x_min, x_max, y_min, y_max}` dicts.

- [ ] **Step 2: Write failing test**

```python
"""Test that build_scene emits a PGM + YAML for the small layout."""

from __future__ import annotations

import subprocess
from pathlib import Path


def test_build_scene_small_emits_map(tmp_path: Path):
    out_usd = tmp_path / "small.usd"
    out_map_dir = tmp_path / "maps"
    subprocess.run(
        [
            "python", "-m", "warehouse.generators.build_scene",
            "small",
            "--out-usd", str(out_usd),
            "--out-map-dir", str(out_map_dir),
        ],
        check=True,
    )
    assert (out_map_dir / "small.pgm").exists()
    assert (out_map_dir / "small.yaml").exists()
    # PGM header check
    content = (out_map_dir / "small.pgm").read_bytes()
    assert content.startswith(b"P5\n"), "expected P5 PGM header"
```

- [ ] **Step 3: Add `--out-map-dir` flag**

In `warehouse/generators/build_scene.py`, after the USD is generated, gather the obstacle list from the layout and call:

```python
from warehouse.generators.map_export import rasterize_obstacles, write_pgm, write_map_yaml

# … existing USD generation …

if args.out_map_dir:
    out_map_dir = Path(args.out_map_dir)
    out_map_dir.mkdir(parents=True, exist_ok=True)
    obstacles = layout.to_obstacle_boxes()  # implement on Layout if missing
    grid = rasterize_obstacles(
        world_w_m=layout.world_w_m,
        world_h_m=layout.world_h_m,
        resolution_m_per_px=0.05,
        obstacles=obstacles,
    )
    write_pgm(grid, out_map_dir / f"{args.layout_name}.pgm")
    write_map_yaml(
        pgm_filename=f"{args.layout_name}.pgm",
        resolution_m_per_px=0.05,
        origin_xy_yaw=(0.0, 0.0, 0.0),
        path=out_map_dir / f"{args.layout_name}.yaml",
    )
```

If `Layout.to_obstacle_boxes()` doesn't exist, add it: iterate `layout.shelves` and `layout.walls`, return a list of dicts with the four bound keys.

- [ ] **Step 4: Run tests**

```bash
pytest tests/unit/test_build_scene_map.py tests/unit/test_map_export.py -v
```

Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add warehouse/generators/build_scene.py warehouse/layout.py tests/unit/test_build_scene_map.py
git commit -m "feat(map): emit PGM + YAML alongside USD in build_scene (Task 3)"
```

### Task 4: Generate and commit map for the `small` layout

**Files:**
- Create: `warehouse/maps/small.pgm`
- Create: `warehouse/maps/small.yaml`

- [ ] **Step 1: Run the builder to produce the map**

```bash
python -m warehouse.generators.build_scene small \
  --out-usd /tmp/small.usd \
  --out-map-dir warehouse/maps
```

- [ ] **Step 2: Sanity-check the PGM**

```bash
python -c "
import numpy as np
from pathlib import Path
data = Path('warehouse/maps/small.pgm').read_bytes()
header_end = data.index(b'\n', data.index(b'\n', data.index(b'\n') + 1) + 1) + 1
img = np.frombuffer(data[header_end:], dtype=np.uint8)
print('shape pixels =', img.size)
print('occupied pct =', (img == 100).mean())
print('free pct     =', (img == 0).mean())
"
```

Expected: ~5–15% occupied (shelves + pick cell + walls), the rest free.

- [ ] **Step 3: Verify YAML**

```bash
cat warehouse/maps/small.yaml
```

Expected:
```yaml
image: small.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

- [ ] **Step 4: Commit**

```bash
git add warehouse/maps/
git commit -m "feat(map): commit pre-baked occupancy grid for small layout (Task 4)"
```

### Task 5: Create `wdt_carter_description` ROS2 package skeleton

**Files:**
- Create: `ros2_ws/src/wdt_carter_description/package.xml`
- Create: `ros2_ws/src/wdt_carter_description/CMakeLists.txt`
- Create: `ros2_ws/src/wdt_carter_description/urdf/carter.urdf.xacro`
- Create: `ros2_ws/src/wdt_carter_description/launch/carter_description.launch.py`

- [ ] **Step 1: Create `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>wdt_carter_description</name>
  <version>0.2.0</version>
  <description>URDF description of Nova Carter for Nav2 footprint + TF.</description>
  <maintainer email="saad.sharifahmed@gmail.com">Saad Sharif Ahmed</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(wdt_carter_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
```

- [ ] **Step 3: Create `carter.urdf.xacro`**

Nova Carter has a 2-wheel differential drive with a 360° 2D LIDAR on top. We expose the footprint Nav2 cares about (a 0.6m × 0.4m rectangle, height 0.5m) and a `scan` frame that matches Isaac Sim's published LIDAR topic.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="nova_carter">
  <xacro:arg name="robot_namespace" default=""/>

  <!-- Base footprint at ground level -->
  <link name="base_footprint"/>

  <!-- Body (chassis) -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.6 0.4 0.5"/></geometry>
      <origin xyz="0 0 0.25"/>
    </visual>
    <collision>
      <geometry><box size="0.6 0.4 0.5"/></geometry>
      <origin xyz="0 0 0.25"/>
    </collision>
  </link>
  <joint name="base_footprint_to_base_link" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0"/>
  </joint>

  <!-- LIDAR mount on top of the chassis -->
  <link name="laser_frame"/>
  <joint name="base_link_to_laser" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0 0 0.55"/>
  </joint>
</robot>
```

- [ ] **Step 4: Create `carter_description.launch.py`**

```python
"""Launch robot_state_publisher for one Carter namespace.

Usage:
    ros2 launch wdt_carter_description carter_description.launch.py \
        robot_namespace:=robot_0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration("robot_namespace")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("wdt_carter_description"), "urdf", "carter.urdf.xacro"]
    )
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", xacro_file, " robot_namespace:=", ns]
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot_0"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns,
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
    ])
```

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/wdt_carter_description/
git commit -m "feat(ros2): wdt_carter_description package with URDF + RSP launch (Task 5)"
```

### Task 6: Update Isaac Sim spawn to publish `/robot_N/scan`

**Files:**
- Modify: `sim/spawn.py` (existing Carter spawn)

- [ ] **Step 1: Find existing LIDAR config**

```bash
grep -n "lidar\|LIDAR\|scan" sim/spawn.py sim/sensors.py 2>/dev/null
```

Isaac Sim's Nova Carter ships with a built-in 2D LIDAR. The sim-side ROS2 publisher needs to (a) bind the sensor's frame to `<ns>/laser_frame` and (b) publish on topic `<ns>/scan`. Look for how Phase 1 published other namespaced topics like `<ns>/odom`.

- [ ] **Step 2: Add scan publishing**

In the function that spawns a single Carter (likely `spawn_carter()` or in a loop), add the ROS2 LaserScan publisher action graph. Pseudo-code (exact ROS2 OmniGraph nodes depend on the Isaac Sim 5.0 API):

```python
from omni.isaac.core_nodes.scripts.utils import set_target_prims
import omni.graph.core as og

def add_ros2_scan_publisher(carter_prim_path: str, namespace: str):
    keys = og.Controller.Keys
    (graph, *_) = og.Controller.edit(
        {"graph_path": f"{carter_prim_path}/ROS_Scan", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadScan", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                ("PubScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "ReadScan.inputs:execIn"),
                ("ReadScan.outputs:execOut", "PubScan.inputs:execIn"),
                ("ReadScan.outputs:linearDepthData", "PubScan.inputs:linearDepthData"),
                # … plus angle range, intensities, etc per Isaac Sim 5.0 API
            ],
            keys.SET_VALUES: [
                ("PubScan.inputs:topicName", f"{namespace}/scan"),
                ("PubScan.inputs:frameId", f"{namespace}/laser_frame"),
            ],
        },
    )
    set_target_prims(
        primPath=f"{carter_prim_path}/ROS_Scan/ReadScan",
        targetPrimPaths=[f"{carter_prim_path}/chassis/lidar"],
    )
```

The exact node names may have shifted in Isaac Sim 5.0 — verify against `/isaac-sim/extsbuild/omni.isaac.ros2_bridge/docs/` on a running instance before committing.

- [ ] **Step 3: Smoke test on vast.ai**

Wake the instance:

```bash
vastai start instance 36775999
# Wait for SSH to come up (~30s); ssh keys already configured.
```

Push the changes, then on the instance:

```bash
ssh vast-romania
cd ~/wdt
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Build new package
colcon build --packages-select wdt_carter_description --symlink-install
source ros2_ws/install/setup.bash

# Run a single-Carter spawn + check topics. Tee to a remote log per
# vast.ai logging discipline.
/isaac-sim/python.sh wdt_vast/carter_topic_smoke.py 2>&1 | tee /tmp/scan_smoke.log &

# In parallel, check the scan topic on the same SSH session
sleep 30
ros2 topic list | grep scan
ros2 topic hz /robot_0/scan
```

Expected: `/robot_0/scan` exists; `ros2 topic hz` reports ~10 Hz.

- [ ] **Step 4: Commit**

```bash
git add sim/spawn.py
git commit -m "feat(sim): publish per-namespace LaserScan from Carter LIDAR (Task 6)"
```

### Task 7: Single-Carter RViz smoke (M0 acceptance)

**Files:** (no code changes, manual verification + a screenshot to commit)

- [ ] **Step 1: On vast.ai, launch one Carter with the description**

```bash
# Tab 1
ros2 launch wdt_carter_description carter_description.launch.py robot_namespace:=robot_0

# Tab 2: bring up the warehouse + spawn the Carter (Phase 1's smoke script)
/isaac-sim/python.sh wdt_vast/carter_topic_smoke.py 2>&1 | tee /tmp/m0_smoke.log
```

- [ ] **Step 2: RViz from Mac (X11 forwarding or web RViz)**

```bash
# Locally
ssh -L 11311:localhost:11311 vast-romania
rviz2 -d ros2_ws/src/wdt_carter_description/rviz/carter.rviz
```

Or run `foxglove-studio` locally and connect to the vast.ai ROS2 bridge. Expected: TF tree shows `base_footprint → base_link → laser_frame`; `LaserScan` displays as a fan in front of the Carter.

- [ ] **Step 3: Capture a screenshot to `docs/images/phase-2/m0-carter-rviz.png` and commit**

```bash
git add docs/images/phase-2/m0-carter-rviz.png
git commit -m "docs(m0): RViz screenshot of Carter URDF + scan (Task 7)"
```

### Task 8: Push M0

```bash
git push origin main
```

This is the M0 milestone checkpoint. Confirm all of {map files committed, `wdt_carter_description` package builds, scan topic publishes, RViz screenshot exists} before moving to M1.

---

## Milestone 1 — Nav2 single-AMR bringup

Spec §4.3 (single-AMR scope). Output: one Carter navigates autonomously from spawn pose to a hardcoded target pose using the full Nav2 stack (map → AMCL → costmap → planner → controller → BT → lifecycle).

### Task 9: Create `wdt_nav2_bringup` package skeleton

**Files:**
- Create: `ros2_ws/src/wdt_nav2_bringup/package.xml`
- Create: `ros2_ws/src/wdt_nav2_bringup/CMakeLists.txt`
- Create: `ros2_ws/src/wdt_nav2_bringup/launch/single_amr.launch.py`
- Create: `ros2_ws/src/wdt_nav2_bringup/config/nav2_params.yaml`
- Create: `ros2_ws/src/wdt_nav2_bringup/maps/.gitkeep`

- [ ] **Step 1: `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>wdt_nav2_bringup</name>
  <version>0.2.0</version>
  <description>Per-AMR Nav2 bringup for the warehouse digital twin.</description>
  <maintainer email="saad.sharifahmed@gmail.com">Saad Sharif Ahmed</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>nav2_amcl</exec_depend>
  <exec_depend>nav2_planner</exec_depend>
  <exec_depend>nav2_controller</exec_depend>
  <exec_depend>nav2_bt_navigator</exec_depend>
  <exec_depend>nav2_behaviors</exec_depend>
  <exec_depend>nav2_lifecycle_manager</exec_depend>

  <export><build_type>ament_cmake</build_type></export>
</package>
```

- [ ] **Step 2: `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(wdt_nav2_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})
ament_package()
```

- [ ] **Step 3: Copy the baked map into the package**

```bash
mkdir -p ros2_ws/src/wdt_nav2_bringup/maps
cp warehouse/maps/small.pgm warehouse/maps/small.yaml ros2_ws/src/wdt_nav2_bringup/maps/
```

The Nav2 package needs a copy because the install layout flattens it; we keep the canonical copy in `warehouse/maps/` and a duplicate in the bringup package's `share/`. CI tests against `warehouse/maps/`; runtime reads from the installed package share.

- [ ] **Step 4: Commit skeleton**

```bash
git add ros2_ws/src/wdt_nav2_bringup/
git commit -m "feat(nav2): wdt_nav2_bringup package skeleton + map copy (Task 9)"
```

### Task 10: Write `nav2_params.yaml`

**Files:**
- Modify: `ros2_ws/src/wdt_nav2_bringup/config/nav2_params.yaml`

- [ ] **Step 1: Write the full Nav2 params** (template — tune AMCL/costmap during M1 smoke)

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: base_footprint
    odom_frame_id: odom
    global_frame_id: map
    laser_model_type: likelihood_field
    max_beams: 60
    min_particles: 500
    max_particles: 2000
    scan_topic: scan
    set_initial_pose: true
    initial_pose:
      x: 1.0
      y: 1.0
      yaw: 0.0
    transform_tolerance: 1.0

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: odom
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.6
      max_vel_theta: 1.0
      acc_lim_x: 1.5
      acc_lim_theta: 2.0
      sim_time: 1.7
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.35
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.45

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      resolution: 0.05
      track_unknown_space: true
      robot_radius: 0.35
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.45

map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: ""  # set by launch

planner_server:
  ros__parameters:
    use_sim_time: true
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    use_sim_time: true
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    wait:
      plugin: "nav2_behaviors/Wait"
```

- [ ] **Step 2: Commit**

```bash
git add ros2_ws/src/wdt_nav2_bringup/config/nav2_params.yaml
git commit -m "feat(nav2): Nav2 params for AMCL + DWB + costmaps tuned for warehouse (Task 10)"
```

### Task 11: Single-AMR Nav2 launch file

**Files:**
- Modify: `ros2_ws/src/wdt_nav2_bringup/launch/single_amr.launch.py`

- [ ] **Step 1: Write the launch file**

```python
"""Bring up Nav2 for one namespaced AMR.

Composes map_server, AMCL, planner_server, controller_server,
bt_navigator, behavior_server, and lifecycle_manager — all inside the
robot's namespace. Pass `robot_namespace:=robot_0` to target a specific
Carter.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("wdt_nav2_bringup")
    params_file = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    map_yaml = PathJoinSubstitution([pkg, "maps", "small.yaml"])
    ns = LaunchConfiguration("robot_namespace")

    lifecycle_nodes = [
        "map_server",
        "amcl",
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot_0"),
        GroupAction([
            PushRosNamespace(ns),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[params_file, {"yaml_filename": map_yaml}],
                output="screen",
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                parameters=[params_file],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[{"use_sim_time": True, "autostart": True,
                            "node_names": lifecycle_nodes}],
                output="screen",
            ),
        ]),
    ])
```

- [ ] **Step 2: Build the package on vast.ai**

```bash
ssh vast-romania
cd ~/wdt
source /opt/ros/humble/setup.bash
colcon build --packages-select wdt_nav2_bringup --symlink-install
source ros2_ws/install/setup.bash
```

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/wdt_nav2_bringup/launch/single_amr.launch.py
git commit -m "feat(nav2): single-AMR Nav2 bringup launch (Task 11)"
```

### Task 12: Single-AMR navigate-to-pose smoke

**Files:**
- Create: `wdt_vast/nav2_single_amr_smoke.py`

- [ ] **Step 1: Write the smoke script**

```python
"""Spawn one Carter, launch its Nav2 stack, send a NavigateToPose goal,
record the result.

This is the M1 smoke. Success = Carter physically arrives at the goal
within 60 sim seconds; failure = action fails, AMR doesn't move, or
AMCL diverges (pose jumps > 2m).
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/nav2_smoke")
OUT.mkdir(parents=True, exist_ok=True)

# 1. Start Isaac Sim with one Carter at (1.0, 1.0)
sim_proc = subprocess.Popen(
    ["/isaac-sim/python.sh", "wdt_vast/carter_topic_smoke.py"],
    stdout=open(OUT / "sim.log", "w"),
    stderr=subprocess.STDOUT,
)
time.sleep(45)  # let Kit boot + ROS2 bridge come up

# 2. Launch RSP + Nav2 stack in the same namespace
rsp_proc = subprocess.Popen(
    ["ros2", "launch", "wdt_carter_description", "carter_description.launch.py",
     "robot_namespace:=robot_0"],
    stdout=open(OUT / "rsp.log", "w"),
    stderr=subprocess.STDOUT,
)
nav2_proc = subprocess.Popen(
    ["ros2", "launch", "wdt_nav2_bringup", "single_amr.launch.py",
     "robot_namespace:=robot_0"],
    stdout=open(OUT / "nav2.log", "w"),
    stderr=subprocess.STDOUT,
)
time.sleep(20)  # lifecycle activation

# 3. Send a NavigateToPose goal to (8.0, 8.0)
goal_proc = subprocess.run(
    ["ros2", "action", "send_goal", "/robot_0/navigate_to_pose",
     "nav2_msgs/action/NavigateToPose",
     '{pose: {header: {frame_id: map}, pose: {position: {x: 8.0, y: 8.0, z: 0.0}, '
     'orientation: {w: 1.0}}}}',
     "--feedback"],
    capture_output=True, text=True, timeout=120,
)
(OUT / "goal_result.txt").write_text(goal_proc.stdout + "\n---\n" + goal_proc.stderr)

# Clean up
for p in [nav2_proc, rsp_proc, sim_proc]:
    p.terminate()
    try:
        p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        p.kill()

# Parse
text = goal_proc.stdout
if "SUCCEEDED" in text:
    print("M1 SMOKE PASS")
    sys.exit(0)
print("M1 SMOKE FAIL")
print(text)
sys.exit(1)
```

- [ ] **Step 2: Run smoke on vast.ai**

```bash
ssh vast-romania
cd ~/wdt
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python wdt_vast/nav2_single_amr_smoke.py /tmp/nav2_smoke 2>&1 | tee /tmp/nav2_smoke.log
```

Tail the log in a separate SSH:

```bash
# From Mac
ssh vast-romania "tail -F /tmp/nav2_smoke.log" | grep -E "(SUCCEEDED|ERROR|aborted|diverged)"
```

Expected: `M1 SMOKE PASS`. If AMCL diverges, retune `alpha1..alpha5` and `max_beams` in `nav2_params.yaml` and retry.

- [ ] **Step 3: Commit**

```bash
git add wdt_vast/nav2_single_amr_smoke.py
git commit -m "test(nav2): single-AMR navigate-to-pose smoke (Task 12)"
```

### Task 13: Push M1

```bash
git push origin main
```

M1 milestone checkpoint: one Carter navigates autonomously via the full Nav2 stack. AMCL particle filter is configured (even if not perfectly tuned). If AMCL still diverges after one round of tuning, fall back to publishing ground-truth pose on `/robot_0/amcl_pose` from Isaac Sim and document AMCL as a stretch goal — don't burn more than 2 days on tuning per the spec's risk table.

---

## Milestone 2 — Nav2 multi-AMR (6 Carters)

Spec §4.3 (multi-AMR scope). Output: 6 namespaced Nav2 stacks each independently navigating.

### Task 14: Multi-AMR launch file

**Files:**
- Create: `ros2_ws/src/wdt_nav2_bringup/launch/multi_amr.launch.py`

- [ ] **Step 1: Write the launch**

```python
"""Bring up Nav2 for N namespaced AMRs.

Includes the single-AMR launch per namespace `robot_0` .. `robot_{N-1}`.
The map_server is shared at the top level (no namespace) since all AMRs
use the same warehouse map — this saves memory and avoids redundant PGM
loads.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("wdt_nav2_bringup")
    params_file = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    map_yaml = PathJoinSubstitution([pkg, "maps", "small.yaml"])
    num = 6

    nodes = [
        DeclareLaunchArgument("num_robots", default_value="6"),
        # Shared map_server (no namespace)
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[params_file, {"yaml_filename": map_yaml}],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            parameters=[{"use_sim_time": True, "autostart": True,
                        "node_names": ["map_server"]}],
            output="screen",
        ),
    ]

    for i in range(num):
        ns = f"robot_{i}"
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg, "launch", "single_amr_no_map.launch.py"])
                ),
                launch_arguments={"robot_namespace": ns}.items(),
            )
        )

    return LaunchDescription(nodes)
```

- [ ] **Step 2: Refactor `single_amr.launch.py` → split out a no-map variant**

Copy `single_amr.launch.py` to `single_amr_no_map.launch.py`, remove the `map_server` node and remove `"map_server"` from `lifecycle_nodes` (it's now managed at the top level). Update `single_amr.launch.py` to also use this no-map variant + the shared map_server, so both single and multi share the same per-AMR composition.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/wdt_nav2_bringup/launch/
git commit -m "feat(nav2): multi-AMR Nav2 bringup with shared map_server (Task 14)"
```

### Task 15: Multi-AMR smoke

**Files:**
- Create: `wdt_vast/nav2_multi_amr_smoke.py`

- [ ] **Step 1: Adapt the single-AMR smoke for 6 AMRs**

```python
"""Spawn 6 Carters, launch their Nav2 stacks, send each a NavigateToPose
goal at different shelf positions, verify all 6 succeed within 90 sim s.
"""

from __future__ import annotations

import concurrent.futures
import subprocess
import sys
import time
from pathlib import Path

OUT = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/nav2_multi_smoke")
OUT.mkdir(parents=True, exist_ok=True)

# Spawn 6-Carter scene (Phase 1 already has this — fleet_topic_smoke.py)
sim_proc = subprocess.Popen(
    ["/isaac-sim/python.sh", "wdt_vast/fleet_topic_smoke.py"],
    stdout=open(OUT / "sim.log", "w"),
    stderr=subprocess.STDOUT,
)
time.sleep(60)

# Bring up all 6 RSPs + Nav2 stacks
rsp_procs = [
    subprocess.Popen(
        ["ros2", "launch", "wdt_carter_description", "carter_description.launch.py",
         f"robot_namespace:=robot_{i}"],
        stdout=open(OUT / f"rsp_{i}.log", "w"),
        stderr=subprocess.STDOUT,
    )
    for i in range(6)
]
nav2_proc = subprocess.Popen(
    ["ros2", "launch", "wdt_nav2_bringup", "multi_amr.launch.py"],
    stdout=open(OUT / "nav2.log", "w"),
    stderr=subprocess.STDOUT,
)
time.sleep(30)

# Goals — 6 different shelf positions from steady_state.yaml
goals = [
    (4.0, 8.0),
    (7.0, 8.0),
    (10.0, 8.0),
    (4.0, 12.0),
    (7.0, 12.0),
    (10.0, 12.0),
]


def send_goal(i: int, x: float, y: float) -> bool:
    proc = subprocess.run(
        ["ros2", "action", "send_goal", f"/robot_{i}/navigate_to_pose",
         "nav2_msgs/action/NavigateToPose",
         f'{{pose: {{header: {{frame_id: map}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, '
         f'orientation: {{w: 1.0}}}}}}}}'],
        capture_output=True, text=True, timeout=180,
    )
    (OUT / f"goal_{i}.txt").write_text(proc.stdout)
    return "SUCCEEDED" in proc.stdout


with concurrent.futures.ThreadPoolExecutor(max_workers=6) as ex:
    results = list(ex.map(lambda args: send_goal(*args),
                          [(i, *g) for i, g in enumerate(goals)]))

# Cleanup
nav2_proc.terminate()
for p in rsp_procs:
    p.terminate()
sim_proc.terminate()
for p in [nav2_proc, sim_proc, *rsp_procs]:
    try:
        p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        p.kill()

n_ok = sum(results)
print(f"M2 SMOKE: {n_ok}/6 AMRs reached their goals")
sys.exit(0 if n_ok >= 5 else 1)  # tolerate 1 failure (DDS discovery edge cases)
```

- [ ] **Step 2: Run on vast.ai**

```bash
ssh vast-romania "cd ~/wdt && source /opt/ros/humble/setup.bash && \
  source ros2_ws/install/setup.bash && \
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
  python wdt_vast/nav2_multi_amr_smoke.py /tmp/nav2_multi 2>&1 | tee /tmp/nav2_multi.log"
```

Expected: `M2 SMOKE: 6/6 AMRs reached their goals` (or 5/6 acceptable).

If DDS discovery storms: set `CYCLONEDDS_URI` to a static peer-list file as documented in spec §8.

- [ ] **Step 3: Commit + push**

```bash
git add wdt_vast/nav2_multi_amr_smoke.py
git commit -m "test(nav2): multi-AMR smoke for 6 Carters (Task 15)"
git push origin main
```

This closes M2.

---

## Milestone 3 — MoveIt2 + Franka URDF/SRDF, plan-to-pose with mocked perception

Spec §4.2 (Franka half) + §4.4 (motion planning half). Output: Franka URDF + SRDF, `move_group` running, `moveit_py` plans + executes a hand-coded grasp trajectory.

### Task 16: Create `wdt_franka_description` package

**Files:**
- Create: `ros2_ws/src/wdt_franka_description/package.xml`
- Create: `ros2_ws/src/wdt_franka_description/CMakeLists.txt`
- Create: `ros2_ws/src/wdt_franka_description/urdf/panda.urdf.xacro`
- Create: `ros2_ws/src/wdt_franka_description/srdf/panda.srdf`
- Create: `ros2_ws/src/wdt_franka_description/config/kinematics.yaml`
- Create: `ros2_ws/src/wdt_franka_description/config/joint_limits.yaml`
- Create: `ros2_ws/src/wdt_franka_description/config/ompl_planning.yaml`
- Create: `ros2_ws/src/wdt_franka_description/launch/franka_description.launch.py`

- [ ] **Step 1: Vendor the public Franka description**

The public `franka_description` package (https://github.com/frankaemika/franka_description) provides the canonical Panda URDF. Copy the relevant files in (don't add as a git submodule — pin the version).

```bash
cd /tmp
git clone --depth 1 --branch noetic-devel https://github.com/frankaemika/franka_description.git
cp franka_description/robots/panda_arm.urdf.xacro \
   ~/Desktop/Projects/isaac-sim/ros2_ws/src/wdt_franka_description/urdf/panda.urdf.xacro
cp -r franka_description/meshes \
   ~/Desktop/Projects/isaac-sim/ros2_ws/src/wdt_franka_description/meshes
```

If the xacro pulls in further includes (`utils.xacro`, etc.), vendor those too.

- [ ] **Step 2: Write the SRDF (`panda.srdf`)**

The SRDF defines move_groups and disables self-collisions between adjacent links. The MoveIt2 Setup Assistant generates this; we hand-write a minimal version targeting the `panda_arm` (7 DoF) and `panda_hand` (2 DoF) groups.

```xml
<?xml version="1.0"?>
<robot name="panda">
  <group name="panda_arm">
    <chain base_link="panda_link0" tip_link="panda_link8"/>
  </group>
  <group name="panda_hand">
    <link name="panda_hand"/>
    <joint name="panda_finger_joint1"/>
    <joint name="panda_finger_joint2"/>
  </group>
  <group_state name="ready" group="panda_arm">
    <joint name="panda_joint1" value="0"/>
    <joint name="panda_joint2" value="-0.785"/>
    <joint name="panda_joint3" value="0"/>
    <joint name="panda_joint4" value="-2.356"/>
    <joint name="panda_joint5" value="0"/>
    <joint name="panda_joint6" value="1.571"/>
    <joint name="panda_joint7" value="0.785"/>
  </group_state>
  <end_effector name="hand" parent_link="panda_link8" group="panda_hand"/>
  <!-- Disable adjacent self-collisions -->
  <disable_collisions link1="panda_link0" link2="panda_link1" reason="Adjacent"/>
  <disable_collisions link1="panda_link1" link2="panda_link2" reason="Adjacent"/>
  <!-- ... (full list of 30+ pairs — generate with MoveIt Setup Assistant if missing) -->
</robot>
```

- [ ] **Step 3: Write `kinematics.yaml`, `joint_limits.yaml`, `ompl_planning.yaml`**

Standard MoveIt2 configs. Use KDL kinematics, OMPL with RRTConnect default. Boilerplate per the MoveIt2 official tutorial — paste from `https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html`.

- [ ] **Step 4: `package.xml` and `CMakeLists.txt`**

```xml
<!-- package.xml -->
<package format="3">
  <name>wdt_franka_description</name>
  <version>0.2.0</version>
  <description>Franka Panda URDF + SRDF + MoveIt2 configs.</description>
  <maintainer email="saad.sharifahmed@gmail.com">Saad Sharif Ahmed</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>moveit_kinematics</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

```cmake
cmake_minimum_required(VERSION 3.8)
project(wdt_franka_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf srdf config meshes launch
        DESTINATION share/${PROJECT_NAME})
ament_package()
```

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/wdt_franka_description/
git commit -m "feat(ros2): wdt_franka_description package with URDF + SRDF + MoveIt2 configs (Task 16)"
```

### Task 17: Create `wdt_manipulation_bringup` package

**Files:**
- Create: `ros2_ws/src/wdt_manipulation_bringup/package.xml`
- Create: `ros2_ws/src/wdt_manipulation_bringup/CMakeLists.txt`
- Create: `ros2_ws/src/wdt_manipulation_bringup/launch/move_group.launch.py`

- [ ] **Step 1: `package.xml`**

```xml
<package format="3">
  <name>wdt_manipulation_bringup</name>
  <version>0.2.0</version>
  <description>MoveIt2 move_group + pick_cell_orchestrator bringup.</description>
  <maintainer email="saad.sharifahmed@gmail.com">Saad Sharif Ahmed</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>wdt_franka_description</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

- [ ] **Step 2: `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(wdt_manipulation_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
```

- [ ] **Step 3: `move_group.launch.py`**

```python
"""Bring up MoveIt2 move_group for the Franka Panda.

Loads URDF, SRDF, kinematics, joint limits, OMPL planner configs from
wdt_franka_description and starts move_group.
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    franka = FindPackageShare("wdt_franka_description")
    xacro = PathJoinSubstitution([franka, "urdf", "panda.urdf.xacro"])
    srdf = PathJoinSubstitution([franka, "srdf", "panda.srdf"])
    kinematics = PathJoinSubstitution([franka, "config", "kinematics.yaml"])
    joint_limits = PathJoinSubstitution([franka, "config", "joint_limits.yaml"])
    ompl = PathJoinSubstitution([franka, "config", "ompl_planning.yaml"])

    robot_description = Command([FindExecutable(name="xacro"), " ", xacro])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"robot_description_semantic": Command(
                    [FindExecutable(name="cat"), " ", srdf])},
                kinematics, joint_limits, ompl,
                {"use_sim_time": True},
            ],
        ),
    ])
```

- [ ] **Step 4: Commit**

```bash
git add ros2_ws/src/wdt_manipulation_bringup/
git commit -m "feat(ros2): wdt_manipulation_bringup with move_group launch (Task 17)"
```

### Task 18: MoveIt2 plan-to-pose smoke (mocked perception)

**Files:**
- Create: `wdt_vast/moveit_plan_smoke.py`

- [ ] **Step 1: Write the smoke**

```python
"""Connect to move_group, send a plan-to-pose request, execute it.

Mocks perception by hardcoding a target pose 30 cm in front of the
Franka base. Validates that move_group is alive, kinematics resolve,
and an OMPL plan completes within 5s.
"""

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_py.planning_scene_monitor import PlanningSceneMonitor
from moveit_py.robot import RobotModel
from moveit_py.planning import PlanningComponent
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("moveit_plan_smoke")
    monitor = PlanningSceneMonitor(node, "robot_description")
    monitor.start_scene_monitor()

    model = RobotModel(node)
    arm = PlanningComponent("panda_arm", model)
    arm.set_start_state_to_current_state()

    target = Pose()
    target.position.x = 0.3
    target.position.y = 0.0
    target.position.z = 0.5
    target.orientation.w = 1.0
    arm.set_goal_state(pose_stamped_msg=target, pose_link="panda_link8")

    t0 = time.time()
    plan_result = arm.plan()
    plan_dt = time.time() - t0
    print(f"plan took {plan_dt:.3f}s, success={plan_result.error_code.val == 1}")

    if plan_result.error_code.val != 1:
        sys.exit(1)

    arm.execute(plan_result.trajectory)
    print("M3 SMOKE PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run on vast.ai**

```bash
ssh vast-romania
# Tab 1
ros2 launch wdt_manipulation_bringup move_group.launch.py 2>&1 | tee /tmp/move_group.log
# Tab 2
python wdt_vast/moveit_plan_smoke.py 2>&1 | tee /tmp/moveit_smoke.log
```

Expected: `M3 SMOKE PASS` within 10s.

- [ ] **Step 3: Commit + push**

```bash
git add wdt_vast/moveit_plan_smoke.py
git commit -m "test(moveit): plan-to-pose smoke for Franka with mocked perception (Task 18)"
git push origin main
```

M3 closes.

---

## Milestone 4 — FoundationPose install + integration

Spec §4.4 (perception half). Output: real `_lazy_load()` succeeds on vast.ai; FoundationPose returns plausible 6-DoF poses on a test RGB-D pair.

### Task 19: Audit FoundationPose dependencies

**Files:** (no code changes — verification + notes)

- [ ] **Step 1: Pin the FoundationPose commit**

Choose a known-good commit of `NVlabs/FoundationPose` (e.g., `4517f47b5e7e4a7e0d3b9e5d8f8c9e7b8a9d8c5e` — verify on GitHub). Pin in `pyproject.toml` later as a comment.

- [ ] **Step 2: Identify CUDA op build requirements**

FoundationPose ships custom CUDA extensions (`csrc/nvdiffrast`, `csrc/mycuda`). They need:
- CUDA toolkit matching the runtime driver. vast.ai instance runs driver 570.211 → CUDA 12.4 is safe.
- PyTorch 2.0+ with matching CUDA.
- `setuptools`, `ninja`, `cmake`.

- [ ] **Step 3: Identify model weight URLs and sizes**

From `FoundationPose/README.md`: `2024-03-08-foundationpose-checkpoints.tar.gz` (~2 GB), hosted on NVIDIA's Google Drive (links in README).

- [ ] **Step 4: Document in `manipulation/FOUNDATIONPOSE.md`**

```markdown
# FoundationPose Integration

**Commit:** `<sha>` (pinned 2026-05-15)
**Model weights:** `2024-03-08-foundationpose-checkpoints.tar.gz` (~2 GB)
**CUDA:** 12.4 (matching vast.ai driver 570.211)
**PyTorch:** 2.1.0 + cu124

**Weights distribution:** Modal volume `foundationpose-models`, mounted at /weights on vast.ai sync.

**Build strategy:** Pre-build wheels on Modal, distribute to vast.ai via volume rsync.
```

- [ ] **Step 5: Commit**

```bash
git add manipulation/FOUNDATIONPOSE.md
git commit -m "docs(manip): FoundationPose dependency audit + pinned commit (Task 19)"
```

### Task 20: Modal wheel-build app

**Files:**
- Create: `wdt_modal/build_foundationpose_wheels.py`

- [ ] **Step 1: Write the Modal app**

```python
"""Build FoundationPose CUDA wheels on Modal, store in a volume.

The runtime target is vast.ai (driver 570 / CUDA 12.4). We build inside
a Modal container that matches: nvidia/cuda:12.4.0-devel-ubuntu22.04
with PyTorch 2.1.0+cu124 and python 3.10.

Output: a tarball `foundationpose-wheels-<commit>.tar.gz` containing the
built wheels for `nvdiffrast` and `mycuda` extensions, written to the
`foundationpose-models` volume.
"""

from __future__ import annotations

import modal

FP_COMMIT = "4517f47b5e7e4a7e0d3b9e5d8f8c9e7b8a9d8c5e"  # pinned

image = (
    modal.Image.from_registry("nvidia/cuda:12.4.0-devel-ubuntu22.04", add_python="3.10")
    .apt_install("git", "build-essential", "ninja-build", "cmake", "libgl1")
    .pip_install(
        "torch==2.1.0",
        "torchvision==0.16.0",
        "ninja",
        "setuptools",
        "wheel",
        index_url="https://download.pytorch.org/whl/cu124",
    )
    .run_commands(
        f"git clone https://github.com/NVlabs/FoundationPose /workspace/fp && "
        f"cd /workspace/fp && git checkout {FP_COMMIT}",
    )
)

app = modal.App("wdt-foundationpose-wheel-builder")
vol = modal.Volume.from_name("foundationpose-models", create_if_missing=True)


@app.function(image=image, gpu="L4", volumes={"/weights": vol}, timeout=3600)
def build_wheels():
    import subprocess
    import shutil
    from pathlib import Path

    fp = Path("/workspace/fp")
    out = Path("/weights/wheels")
    out.mkdir(parents=True, exist_ok=True)

    # Build nvdiffrast extension
    subprocess.run(
        ["pip", "wheel", "-w", str(out), "./bundled/nvdiffrast"],
        cwd=fp, check=True,
    )
    # Build mycuda extension
    subprocess.run(
        ["pip", "wheel", "-w", str(out), "./mycpp/mycuda"],
        cwd=fp, check=True,
    )

    # Bundle into a single tarball
    subprocess.run(
        ["tar", "-czf", f"/weights/foundationpose-wheels-{FP_COMMIT[:8]}.tar.gz",
         "-C", "/weights/wheels", "."],
        check=True,
    )
    print(f"wheels built and tarred to /weights/foundationpose-wheels-{FP_COMMIT[:8]}.tar.gz")


@app.local_entrypoint()
def main():
    build_wheels.remote()
```

- [ ] **Step 2: Run the wheel builder**

```bash
modal run wdt_modal/build_foundationpose_wheels.py
```

Per [[feedback-modal-build-monitoring]]: this is a long build (~15–25 min). Tail logs with `modal app logs wdt-foundationpose-wheel-builder --follow` and watch for tzdata/PPA gotchas and missing `libxxx-dev` packages.

Expected: build completes, `/weights/foundationpose-wheels-<commit>.tar.gz` appears in the Modal volume.

- [ ] **Step 3: Also stage the model weights tarball on the same volume**

The model weights (~2 GB) are not built — just downloaded once. Either:
- Use a Modal CPU function that `curl`s the public URL into the volume (preferred — no local egress)
- Or download locally and `modal volume put` (slow, ~30 min on consumer connection)

Add to `wdt_modal/build_foundationpose_wheels.py`:

```python
@app.function(image=image, volumes={"/weights": vol}, timeout=3600)
def stage_weights():
    import subprocess
    from pathlib import Path

    out = Path("/weights/checkpoints")
    if (out / "model_best.pth").exists():
        print("weights already present, skipping")
        return
    out.mkdir(parents=True, exist_ok=True)
    # Replace URL with the actual FoundationPose checkpoints URL from
    # NVlabs/FoundationPose README.
    url = "https://example.invalid/2024-03-08-foundationpose-checkpoints.tar.gz"
    subprocess.run(["curl", "-L", url, "-o", "/weights/cp.tar.gz"], check=True)
    subprocess.run(["tar", "-xzf", "/weights/cp.tar.gz", "-C", str(out)], check=True)
```

Run it:

```bash
modal run wdt_modal/build_foundationpose_wheels.py::stage_weights
```

- [ ] **Step 4: Commit**

```bash
git add wdt_modal/build_foundationpose_wheels.py
git commit -m "feat(modal): build FoundationPose CUDA wheels + stage weights (Task 20)"
```

### Task 21: vast.ai-side rsync + install

**Files:**
- Create: `wdt_vast/install_foundationpose.sh`

- [ ] **Step 1: Write the install script**

```bash
#!/usr/bin/env bash
# Pull FoundationPose wheels + weights from Modal volume to vast.ai,
# install into the Isaac Sim python environment.
#
# Usage (on vast.ai):
#   bash wdt_vast/install_foundationpose.sh
#
# Prereq: `modal` CLI authenticated on vast.ai (or use rclone with a
# Modal-volume-export endpoint).

set -euo pipefail

WHEELS_TGZ=/tmp/fp-wheels.tar.gz
WEIGHTS_TGZ=/tmp/fp-weights.tar.gz
INSTALL_PREFIX=/opt/foundationpose

mkdir -p "$INSTALL_PREFIX"/{wheels,checkpoints}

# Strategy 1: modal volume get (preferred — Modal-native)
modal volume get foundationpose-models \
    foundationpose-wheels-4517f47b.tar.gz "$WHEELS_TGZ"
modal volume get foundationpose-models \
    checkpoints "$INSTALL_PREFIX/checkpoints/"

tar -xzf "$WHEELS_TGZ" -C "$INSTALL_PREFIX/wheels/"

# Install into the Isaac Sim python (the same one that runs run_scenario.py)
/isaac-sim/python.sh -m pip install \
    "$INSTALL_PREFIX/wheels"/*.whl

# Clone the FoundationPose python package itself (not the CUDA exts)
FP_COMMIT=4517f47b5e7e4a7e0d3b9e5d8f8c9e7b8a9d8c5e
if [ ! -d "$INSTALL_PREFIX/src" ]; then
    git clone https://github.com/NVlabs/FoundationPose "$INSTALL_PREFIX/src"
    git -C "$INSTALL_PREFIX/src" checkout "$FP_COMMIT"
fi
/isaac-sim/python.sh -m pip install -e "$INSTALL_PREFIX/src"

echo "FoundationPose installed at $INSTALL_PREFIX"
```

- [ ] **Step 2: Run on vast.ai**

```bash
ssh vast-romania "bash ~/wdt/wdt_vast/install_foundationpose.sh 2>&1 | tee /tmp/fp_install.log"
```

Tail in another SSH; expect ~5–10 min including downloads.

- [ ] **Step 3: Verify install**

```bash
ssh vast-romania "/isaac-sim/python.sh -c 'import foundationpose; print(foundationpose.__file__)'"
```

Expected: prints a path under `/opt/foundationpose/src/`.

- [ ] **Step 4: Commit**

```bash
git add wdt_vast/install_foundationpose.sh
git commit -m "feat(vast): install_foundationpose script pulls wheels + weights (Task 21)"
```

### Task 22: Real `_lazy_load()` for FoundationPose

**Files:**
- Modify: `manipulation/pose_estimation.py`
- Test: `tests/integration/test_foundationpose_smoke.py`

- [ ] **Step 1: Read the existing stub**

```bash
cat manipulation/pose_estimation.py
```

- [ ] **Step 2: Implement real `_lazy_load()`** (the existing file already defines `PoseResult` dataclass + `PoseEstimator(model_dir)` constructor — keep those names, just replace the import + register call)

```python
"""FoundationPose 6-DoF pose estimation from RGB-D + CAD.

The dataclass `PoseResult` and the `PoseEstimator(model_dir)` constructor
shape were established in Phase 1 — keep them. Phase 2's change is to
swap the import from the Isaac-ROS-bundled wrapper (which doesn't ship
on vast.ai) to the raw `foundationpose` package that
`wdt_vast/install_foundationpose.sh` installs.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class PoseResult:
    translation: np.ndarray  # shape (3,)
    rotation: np.ndarray  # shape (3, 3)
    score: float


class PoseEstimator:
    def __init__(self, model_dir: str = "/opt/foundationpose/checkpoints"):
        self.model_dir = model_dir
        self._impl = None

    def _lazy_load(self):
        if self._impl is not None:
            return
        # Phase 2: import the raw foundationpose package installed by
        # wdt_vast/install_foundationpose.sh (Task 21). Unit tests on Mac
        # never reach this branch because the pipeline tests inject mocks.
        from foundationpose import FoundationPose  # type: ignore[import]

        weights = Path(self.model_dir) / "model_best.pth"
        if not weights.exists():
            raise FileNotFoundError(
                f"FoundationPose weights not found at {weights}; run "
                f"wdt_vast/install_foundationpose.sh first"
            )
        self._impl = FoundationPose(model_pts=weights.parent)

    def estimate(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        cad_path: str,
        camera_K: np.ndarray,
    ) -> list[PoseResult]:
        self._lazy_load()
        # FoundationPose API: pass RGB (HxWx3 uint8), depth (HxW float meters),
        # mesh path, camera intrinsics, mask (we use a full-image mask for
        # the warehouse pick cell since it's empty except for the target).
        mask = np.ones(depth.shape, dtype=np.uint8) * 255
        pose = self._impl.register(rgb=rgb, depth=depth, K=camera_K,
                                   ob_in_cam=None, mask=mask, cad_path=cad_path)
        if pose is None:
            return []
        T = pose[:3, 3]
        R = pose[:3, :3]
        return [PoseResult(translation=T, rotation=R, score=1.0)]
```

- [ ] **Step 3: Write the integration smoke test**

```python
"""End-to-end FoundationPose smoke — load weights, run on a synthetic
RGB-D pair, verify a pose is returned.

Skips on stock Mac. Runs only on vast.ai where foundationpose is installed.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("foundationpose")


def test_foundationpose_returns_pose():
    from manipulation.pose_estimation import PoseEstimator

    fp = PoseEstimator()
    # Synthetic RGB-D — a 480x640 frame with a single object centered
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[200:280, 280:360] = (200, 100, 50)
    depth = np.ones((480, 640), dtype=np.float32) * 1.0
    depth[200:280, 280:360] = 0.7
    K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])
    cad = "/opt/foundationpose/src/demo_data/mustard0/mesh/textured_simple.obj"
    if not Path(cad).exists():
        pytest.skip(f"CAD asset {cad} not present")

    estimates = fp.estimate(rgb=rgb, depth=depth, cad_path=cad, camera_K=K)
    assert len(estimates) >= 1
    assert estimates[0].translation.shape == (3,)
```

- [ ] **Step 4: Run on vast.ai**

```bash
ssh vast-romania "cd ~/wdt && /isaac-sim/python.sh -m pytest \
    tests/integration/test_foundationpose_smoke.py -v 2>&1 | tee /tmp/fp_smoke.log"
```

Expected: 1 passed (or 1 skipped if the demo CAD isn't bundled; that's acceptable for this task — pipeline integration smoke in M5 will cover real input).

- [ ] **Step 5: Commit**

```bash
git add manipulation/pose_estimation.py tests/integration/test_foundationpose_smoke.py
git commit -m "feat(manip): real FoundationPose _lazy_load + integration smoke (Task 22)"
```

### Task 23: Push M4

```bash
git push origin main
```

M4 closes. FoundationPose now installs reproducibly and the pose estimator returns real estimates on vast.ai. Next: build the orchestrator that ties pose → grasp → motion.

---

## Milestone 5 — `pick_cell_orchestrator` + first end-to-end pick

Spec §4.4 (orchestrator) + §4.5 (coordinator-pipeline bridging on the manipulation side). Output: 1-order `smoke.yaml` scenario completes end-to-end with real Nav2 + real manipulation.

### Task 24: `TopDownGrasp` class — failing test

**Files:**
- Test: `tests/unit/test_top_down_grasp.py`

- [ ] **Step 1: Write failing test**

```python
"""Unit tests for the deterministic top-down grasp generator."""

from __future__ import annotations

import numpy as np


def test_top_down_grasp_at_pose():
    from manipulation.grasping import TopDownGrasp

    gen = TopDownGrasp(standoff_m=0.05)
    depth = np.ones((480, 640), dtype=np.float32) * 1.0
    K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])

    pose_translation = np.array([0.1, 0.2, 0.3])
    candidates = gen.propose_at(translation=pose_translation, depth=depth, camera_K=K)

    assert len(candidates) == 1
    c = candidates[0]
    # Grasp position: pose + 5cm standoff in world +Z
    np.testing.assert_allclose(c.translation, [0.1, 0.2, 0.35], atol=1e-6)
    # Rotation: gripper Z-axis points down (world -Z)
    np.testing.assert_allclose(c.rotation[:, 2], [0, 0, -1], atol=1e-6)


def test_top_down_grasp_returns_one_candidate():
    from manipulation.grasping import TopDownGrasp

    gen = TopDownGrasp()
    cands = gen.propose_at(
        translation=np.zeros(3),
        depth=np.ones((10, 10), dtype=np.float32),
        camera_K=np.eye(3),
    )
    assert len(cands) == 1
    assert cands[0].score == 1.0  # deterministic, always score 1
    assert cands[0].width == 0.08  # default gripper width
```

- [ ] **Step 2: Run, verify fail**

```bash
pytest tests/unit/test_top_down_grasp.py -v
```

Expected: `AttributeError: module 'manipulation.grasping' has no attribute 'TopDownGrasp'`.

- [ ] **Step 3: Commit**

```bash
git add tests/unit/test_top_down_grasp.py
git commit -m "test(manip): failing tests for TopDownGrasp (Task 24)"
```

### Task 25: `TopDownGrasp` implementation

**Files:**
- Modify: `manipulation/grasping.py`

- [ ] **Step 1: Add `TopDownGrasp` alongside the existing `GraspGenerator`**

```python
class TopDownGrasp:
    """Deterministic top-down grasp at a known object pose.

    Produces a single GraspCandidate with the gripper pointing world-down
    and the wrist translated `standoff_m` above the object's pose. Used
    when AnyGrasp is not installed and the object's pose is known from
    FoundationPose — warehouse SKUs are constrained enough that this is
    a defensible grasp choice.
    """

    def __init__(self, standoff_m: float = 0.05, gripper_width: float = 0.08):
        self.standoff_m = standoff_m
        self.gripper_width = gripper_width

    def propose_at(
        self,
        translation: np.ndarray,
        depth: np.ndarray,  # unused, kept for interface symmetry
        camera_K: np.ndarray,  # unused, kept for interface symmetry
    ) -> list[GraspCandidate]:
        grasp_t = translation.astype(np.float32).copy()
        grasp_t[2] += self.standoff_m  # standoff above the object
        # Rotation: gripper Z-axis aligned to world -Z. The X and Y axes
        # are arbitrary; pick a stable basis (gripper X = world X).
        R = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])
        return [GraspCandidate(
            translation=grasp_t, rotation=R,
            width=self.gripper_width, score=1.0,
        )]

    # Adapter to satisfy ManipulationPipeline's `grasp_generator.propose(depth, camera_K)`
    # interface — we additionally need a translation, which the orchestrator passes.
    def propose(self, depth: np.ndarray, camera_K: np.ndarray) -> list[GraspCandidate]:
        # Required by ManipulationPipeline's duck-typed interface, but
        # TopDownGrasp needs the pose. Orchestrator uses propose_at instead.
        raise NotImplementedError(
            "TopDownGrasp.propose() requires a pose; use propose_at() or "
            "compose with a PoseEstimator via TopDownGraspFromPose."
        )
```

- [ ] **Step 2: Verify tests pass**

```bash
pytest tests/unit/test_top_down_grasp.py -v
```

Expected: 2 passed.

- [ ] **Step 3: Commit**

```bash
git add manipulation/grasping.py
git commit -m "feat(manip): TopDownGrasp deterministic grasp generator (Task 25)"
```

### Task 26: Adapter — `TopDownGraspFromPose`

`ManipulationPipeline.pick()` calls `grasp_generator.propose(depth, K)` — but `TopDownGrasp` needs the pose too. Build a small adapter that closes over the pose-estimator result.

**Files:**
- Modify: `manipulation/grasping.py`
- Test: extend `tests/unit/test_top_down_grasp.py`

- [ ] **Step 1: Write failing test**

```python
def test_top_down_grasp_from_pose_propose_uses_pose():
    from manipulation.grasping import TopDownGrasp, TopDownGraspFromPose
    from manipulation.pose_estimation import PoseResult

    pose = PoseResult(translation=np.array([1.0, 2.0, 3.0]),
                      rotation=np.eye(3), score=1.0)
    inner = TopDownGrasp(standoff_m=0.1)
    gen = TopDownGraspFromPose(inner=inner, pose=pose)

    cands = gen.propose(depth=np.zeros((10, 10), dtype=np.float32),
                       camera_K=np.eye(3))
    assert len(cands) == 1
    np.testing.assert_allclose(cands[0].translation, [1.0, 2.0, 3.1])
```

- [ ] **Step 2: Implement**

```python
class TopDownGraspFromPose:
    """Adapter that binds a pose to TopDownGrasp so it satisfies the
    duck-typed `propose(depth, K) -> list[GraspCandidate]` interface
    that ManipulationPipeline expects.
    """

    def __init__(self, inner: TopDownGrasp, pose):
        self._inner = inner
        self._pose = pose

    def propose(self, depth: np.ndarray, camera_K: np.ndarray) -> list[GraspCandidate]:
        return self._inner.propose_at(
            translation=self._pose.translation,
            depth=depth, camera_K=camera_K,
        )
```

- [ ] **Step 3: Verify, commit**

```bash
pytest tests/unit/test_top_down_grasp.py -v
git add manipulation/grasping.py tests/unit/test_top_down_grasp.py
git commit -m "feat(manip): TopDownGraspFromPose adapter for ManipulationPipeline (Task 26)"
```

### Task 27: `pick_cell_orchestrator` ROS2 node

**Files:**
- Create: `ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/__init__.py`
- Create: `ros2_ws/src/wdt_manipulation_bringup/wdt_manipulation_bringup/pick_cell_orchestrator.py`
- Modify: `ros2_ws/src/wdt_manipulation_bringup/CMakeLists.txt` (install python entry point)
- Modify: `ros2_ws/src/wdt_manipulation_bringup/package.xml` (add python deps)

- [ ] **Step 1: Switch bringup package to ament_python (or hybrid)**

ament_python is simpler for this. Replace `CMakeLists.txt` + `package.xml` build_type:

```xml
<!-- package.xml: change buildtool_depend -->
<buildtool_depend>ament_python</buildtool_depend>
<export><build_type>ament_python</build_type></export>
```

Delete `CMakeLists.txt`, create `setup.py`:

```python
from setuptools import find_packages, setup

package_name = "wdt_manipulation_bringup"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/move_group.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Saad Sharif Ahmed",
    maintainer_email="saad.sharifahmed@gmail.com",
    description="MoveIt2 + pick_cell_orchestrator bringup.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pick_cell_orchestrator = wdt_manipulation_bringup.pick_cell_orchestrator:main",
        ],
    },
)
```

Create `resource/wdt_manipulation_bringup` (empty marker file).

- [ ] **Step 2: Write the orchestrator node**

```python
"""ROS2 node that runs the manipulation pipeline when triggered.

Subscribes:
    /cell/start_pick (std_msgs/String) — payload is order_id

Subscribes (latched):
    /cell/cam/rgb   (sensor_msgs/Image)
    /cell/cam/depth (sensor_msgs/Image)
    /cell/cam/info  (sensor_msgs/CameraInfo)

Publishes:
    /cell/pick_result (std_msgs/String) — JSON payload:
        {"order_id": "...", "success": bool, "attempts": int,
         "cycle_time_s": float, "failure_reason": "..."}

On every /cell/start_pick:
    1. Snapshot the latest RGB-D + camera_K.
    2. Look up the CAD path for the SKU (Phase 2: single SKU, hardcoded).
    3. Run ManipulationPipeline(FoundationPose, TopDownGraspFromPose, MoveIt2):
         - PoseEstimator.estimate() → poses
         - TopDownGraspFromPose bound to poses[0] → candidates
         - MoveIt2Planner.plan_to_pose() per candidate (existing K=3 retry)
    4. Publish /cell/pick_result with the PickResult fields.
"""

from __future__ import annotations

import json
import threading
from time import perf_counter

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from manipulation.grasping import TopDownGrasp, TopDownGraspFromPose
from manipulation.motion_planning import MoveIt2Planner
from manipulation.pipeline import ManipulationPipeline
from manipulation.pose_estimation import PoseEstimator


class PickCellOrchestrator(Node):
    def __init__(self):
        super().__init__("pick_cell_orchestrator")
        self.declare_parameter(
            "cad_path",
            "/opt/foundationpose/src/demo_data/mustard0/mesh/textured_simple.obj",
        )
        self._cad_path = self.get_parameter("cad_path").get_parameter_value().string_value

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_rgb = None
        self._latest_depth = None
        self._latest_K = None

        self.create_subscription(Image, "/cell/cam/rgb", self._on_rgb, 1)
        self.create_subscription(Image, "/cell/cam/depth", self._on_depth, 1)
        self.create_subscription(CameraInfo, "/cell/cam/info", self._on_info, 1)
        self.create_subscription(String, "/cell/start_pick", self._on_start, 1)
        self._pub = self.create_publisher(String, "/cell/pick_result", 10)

        self._pose_estimator = PoseEstimator()
        self._top_down = TopDownGrasp(standoff_m=0.05)
        self._arm = MoveIt2Planner(node=self, group_name="panda_arm")
        self.get_logger().info("pick_cell_orchestrator ready")

    def _on_rgb(self, msg: Image):
        with self._lock:
            self._latest_rgb = self._bridge.imgmsg_to_cv2(msg, "rgb8")

    def _on_depth(self, msg: Image):
        with self._lock:
            self._latest_depth = self._bridge.imgmsg_to_cv2(msg, "32FC1")

    def _on_info(self, msg: CameraInfo):
        with self._lock:
            self._latest_K = np.array(msg.k).reshape(3, 3)

    def _on_start(self, msg: String):
        order_id = msg.data
        self.get_logger().info(f"start_pick received: {order_id}")
        with self._lock:
            rgb = self._latest_rgb
            depth = self._latest_depth
            K = self._latest_K
        if rgb is None or depth is None or K is None:
            self._publish_result(order_id, False, 0, 0.0, "no_cam_data")
            return

        t0 = perf_counter()
        poses = self._pose_estimator.estimate(
            rgb=rgb, depth=depth, cad_path=self._cad_path, camera_K=K,
        )
        if not poses:
            self._publish_result(order_id, False, 0, perf_counter() - t0, "no_pose")
            return

        grasp_gen = TopDownGraspFromPose(inner=self._top_down, pose=poses[0])
        pipeline = ManipulationPipeline(
            pose_estimator=_PrecomputedPose(poses),
            grasp_generator=grasp_gen,
            arm=self._arm,
        )
        result = pipeline.pick(rgb=rgb, depth=depth, cad_path=self._cad_path, camera_K=K)
        self._publish_result(
            order_id, result.success, result.attempts,
            result.cycle_time_s, result.failure_reason,
        )

    def _publish_result(self, order_id, success, attempts, cycle_time_s, reason):
        msg = String()
        msg.data = json.dumps({
            "order_id": order_id,
            "success": success,
            "attempts": attempts,
            "cycle_time_s": cycle_time_s,
            "failure_reason": reason,
        })
        self._pub.publish(msg)
        self.get_logger().info(f"pick_result: {msg.data}")


class _PrecomputedPose:
    """Adapter — the orchestrator already ran pose estimation, so wrap
    the result so ManipulationPipeline.pick() doesn't re-run it.
    """
    def __init__(self, poses):
        self._poses = poses
    def estimate(self, **kwargs):
        return self._poses


def main():
    rclpy.init()
    node = PickCellOrchestrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Add launch file for the orchestrator**

Update `wdt_manipulation_bringup/launch/move_group.launch.py` (or create `pick_cell.launch.py`) to also start the orchestrator node alongside `move_group`.

- [ ] **Step 4: Build + commit**

```bash
ssh vast-romania "cd ~/wdt && source /opt/ros/humble/setup.bash && \
    colcon build --packages-select wdt_manipulation_bringup --symlink-install"
git add ros2_ws/src/wdt_manipulation_bringup/
git commit -m "feat(manip): pick_cell_orchestrator ROS2 node + ament_python (Task 27)"
```

### Task 28: End-to-end pick smoke on `smoke.yaml`

**Files:**
- Modify: `scenarios/smoke.yaml` (already exists, just verify it's 1 order)
- Modify: `wdt_vast/run_scenario.py` to launch Nav2 + move_group + orchestrator as subprocesses

- [ ] **Step 1: Read existing `smoke.yaml`**

```bash
cat scenarios/smoke.yaml
```

Verify it has 1 order, 1 AMR (or 6 — depending on Phase 1 setup). Adjust if needed.

- [ ] **Step 2: Extend `run_scenario.py` to launch ROS2 stacks**

Add to the existing orchestrator (after `mark("scenario_loaded_*")`):

```python
import subprocess
# Launch Nav2 (multi or single depending on fleet_size)
launch_file = "multi_amr.launch.py" if scenario.fleet_size > 1 else "single_amr.launch.py"
nav2_proc = subprocess.Popen(
    ["ros2", "launch", "wdt_nav2_bringup", launch_file],
    stdout=open(out_dir / "nav2.log", "w"),
    stderr=subprocess.STDOUT,
)
# Launch MoveIt2 + orchestrator
move_proc = subprocess.Popen(
    ["ros2", "launch", "wdt_manipulation_bringup", "move_group.launch.py"],
    stdout=open(out_dir / "move_group.log", "w"),
    stderr=subprocess.STDOUT,
)
orch_proc = subprocess.Popen(
    ["ros2", "run", "wdt_manipulation_bringup", "pick_cell_orchestrator"],
    stdout=open(out_dir / "orchestrator.log", "w"),
    stderr=subprocess.STDOUT,
)
import atexit
atexit.register(lambda: [p.terminate() for p in [orch_proc, move_proc, nav2_proc]])
time.sleep(45)  # let lifecycle + move_group come up
mark("ros2_stacks_up")
```

- [ ] **Step 3: Run on vast.ai**

```bash
ssh vast-romania "cd ~/wdt && source /opt/ros/humble/setup.bash && \
    source ros2_ws/install/setup.bash && \
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
    /isaac-sim/python.sh wdt_vast/run_scenario.py scenarios/smoke.yaml /tmp/m5_smoke \
    2>&1 | tee /tmp/m5_smoke.log"
```

Expected: `metrics.json` reports `orders_completed=1` (or at least one `/cell/pick_result` with `success=True` in the events log).

- [ ] **Step 4: Commit + push**

```bash
git add wdt_vast/run_scenario.py
git commit -m "feat(run): launch Nav2 + MoveIt2 + orchestrator in run_scenario (Task 28)"
git push origin main
```

M5 closes.

---

## Milestone 6 — Coordinator state machine wired to real action results

Spec §4.5. Output: coordinator's NavigateToPose calls actually receive results from Nav2, state machine advances correctly, /cell/start_pick triggers on AT_CELL, /cell/pick_result advances or fails.

### Task 29: Replace placeholder NavigateToPose with real action client

**Files:**
- Modify: `ros2_ws/src/fleet_coordinator/fleet_coordinator/coordinator_node.py` (or wherever the Phase 1 stub lives)

- [ ] **Step 1: Find the existing client**

```bash
grep -rn "NavigateToPose\|navigate_to_pose\|not ready" ros2_ws/src/fleet_coordinator/
```

Phase 1's coordinator node creates an ActionClient but the server doesn't exist, so it logs "not ready" warnings.

- [ ] **Step 2: Wire the result callback**

In the coordinator's per-AMR action client, ensure:

```python
def _send_nav_goal(self, robot_id: str, x: float, y: float):
    client = self._nav_clients[robot_id]
    if not client.wait_for_server(timeout_sec=1.0):
        self.get_logger().warning(f"{robot_id}: nav2 action server not ready")
        return
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.w = 1.0
    future = client.send_goal_async(goal)
    future.add_done_callback(lambda f: self._on_nav_accepted(robot_id, f))

def _on_nav_accepted(self, robot_id, future):
    handle = future.result()
    if not handle.accepted:
        self._on_nav_failed(robot_id, "rejected")
        return
    handle.get_result_async().add_done_callback(
        lambda f: self._on_nav_done(robot_id, f)
    )

def _on_nav_done(self, robot_id, future):
    result = future.result()
    status = result.status
    if status == GoalStatus.STATUS_SUCCEEDED:
        self._on_nav_success(robot_id)
    else:
        self._on_nav_failed(robot_id, f"status_{status}")
```

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/fleet_coordinator/
git commit -m "feat(coord): wire real NavigateToPose action result callbacks (Task 29)"
```

### Task 30: AT_CELL → publish /cell/start_pick

**Files:**
- Modify: coordinator node (same file as Task 29)

- [ ] **Step 1: Add /cell/start_pick publisher in `__init__`**

```python
self._start_pick_pub = self.create_publisher(String, "/cell/start_pick", 10)
self.create_subscription(String, "/cell/pick_result", self._on_pick_result, 10)
```

- [ ] **Step 2: In `_on_nav_success`, branch on state**

```python
def _on_nav_success(self, robot_id):
    order = self._orders_by_robot.get(robot_id)
    if order is None:
        return
    if order.state == OrderState.NAVIGATING_TO_CELL:
        order.state = OrderState.AT_CELL
        msg = String(); msg.data = order.id
        self._start_pick_pub.publish(msg)
        order.state = OrderState.PICKING
    elif order.state == OrderState.NAVIGATING_TO_DROP:
        order.state = OrderState.COMPLETED
        self._on_order_complete(order)
```

- [ ] **Step 3: Implement `_on_pick_result`**

```python
def _on_pick_result(self, msg: String):
    data = json.loads(msg.data)
    order_id = data["order_id"]
    order = self._orders.get(order_id)
    if order is None:
        return
    if data["success"]:
        order.state = OrderState.NAVIGATING_TO_DROP
        x, y = order.drop_off_xy
        self._send_nav_goal(order.robot_id, x, y)
    else:
        order.state = OrderState.FAILED
        order.failure_reason = data["failure_reason"]
        self._return_robot_to_pool(order.robot_id)
        self._on_order_complete(order)  # records the failure in metrics
```

- [ ] **Step 4: Commit**

```bash
git add ros2_ws/src/fleet_coordinator/
git commit -m "feat(coord): publish /cell/start_pick on AT_CELL, consume /cell/pick_result (Task 30)"
```

### Task 31: M6 smoke — coordinator drives 1 order end-to-end

**Files:**
- Use existing `wdt_vast/run_scenario.py` with `scenarios/smoke.yaml`

- [ ] **Step 1: Run**

```bash
ssh vast-romania "cd ~/wdt && source /opt/ros/humble/setup.bash && \
    source ros2_ws/install/setup.bash && \
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
    /isaac-sim/python.sh wdt_vast/run_scenario.py scenarios/smoke.yaml /tmp/m6_smoke \
    2>&1 | tee /tmp/m6_smoke.log"
```

- [ ] **Step 2: Verify metrics.json**

```bash
ssh vast-romania "cat /tmp/m6_smoke/metrics.json"
```

Expected:
```json
{
  "orders_total": 1,
  "orders_completed": 1,
  "pick_success_rate": 1.0,
  "deadlocks": 0,
  ...
}
```

- [ ] **Step 3: If pick succeeds, capture a short video clip + commit**

```bash
scp vast-romania:/tmp/m6_smoke/frames/ docs/images/phase-2/m6-smoke/
ffmpeg -framerate 10 -i docs/images/phase-2/m6-smoke/frame_%04d.png \
    -c:v libx264 -pix_fmt yuv420p docs/videos/phase-2/m6-first-pick.mp4
git add docs/videos/phase-2/m6-first-pick.mp4
git commit -m "docs(m6): first end-to-end pick video (Task 31)"
git push origin main
```

M6 closes.

---

## Milestone 7 — First full 64-order steady_state with real manipulation

Output: `orders_completed > 0` on the 64-order scenario with `hungarian_cbs`.

### Task 32: Pre-run preflight

**Files:** (no code changes — environment check)

- [ ] **Step 1: Cost + budget check**

```bash
# vast.ai
vastai show instances
# Modal
modal app stats
```

Verify <$1 spent in M0–M6 combined. If we're over, audit.

- [ ] **Step 2: Resume vast.ai instance and verify all services**

```bash
vastai start instance 36775999
ssh vast-romania "source /opt/ros/humble/setup.bash && \
    ros2 pkg list | grep -E '^wdt_'"
```

Expected: 4 wdt_ packages listed.

- [ ] **Step 3: Tag the pre-run commit**

```bash
git tag -a phase-2-m7-start -m "pre-M7 checkpoint"
git push origin phase-2-m7-start
```

### Task 33: Full 64-order steady_state run

**Files:**
- Use `scenarios/steady_state.yaml`

- [ ] **Step 1: Launch the full run with logging**

```bash
ssh vast-romania "cd ~/wdt && source /opt/ros/humble/setup.bash && \
    source ros2_ws/install/setup.bash && \
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
    /isaac-sim/python.sh wdt_vast/run_scenario.py \
        scenarios/steady_state.yaml /tmp/m7_first_full 2>&1 | tee /tmp/m7_first_full.log" &
```

In parallel, tail the remote log per [[feedback-vastai-log-streaming]]:

```bash
# From Mac, use Monitor on this:
ssh vast-romania "tail -F /tmp/m7_first_full.log" 2>&1 | \
    grep -E "(order_complete|pick_result|deadlock|ERROR|TRACEBACK)"
```

Expected wall-clock: ~30–45 min for 10 min sim time with real manipulation.

- [ ] **Step 2: Pull and inspect results**

```bash
scp -r vast-romania:/tmp/m7_first_full/ runs/m7_first_full/
cat runs/m7_first_full/metrics.json
```

Sanity checks:
- `orders_total == 64`
- `orders_completed >= 30` (target: more, but ≥30 confirms the system is working)
- `pick_success_rate >= 0.5` (defensible — even at 50% the pipeline is producing real outcomes)
- `deadlocks/min` is finite and small

- [ ] **Step 3: Commit results**

```bash
git add runs/m7_first_full/metrics.json runs/m7_first_full/events.log
git commit -m "results(m7): first 64-order steady_state with real manipulation (Task 33)"
```

### Task 34: Push M7 + decision gate

```bash
git push origin main
```

If `orders_completed < 30` or `pick_success_rate < 0.3`: STOP and re-plan. The spec's risk table allows reducing to 32 orders if necessary. Document the decision in `docs/results-phase-2.md`. Otherwise continue to M8.

---

## Milestone 8 — Ablation runner + 15 runs + aggregator

Spec §6. Output: 15 metrics.json files (3 configs × 5 seeds), `docs/results-phase-2.md`, ablation plots.

### Task 35: Add `nearest_assign` to `assignment.py`

**Files:**
- Modify: `coordinator/assignment.py`
- Test: `tests/unit/test_nearest_assign.py`

- [ ] **Step 1: Write failing test**

```python
from coordinator.assignment import nearest_assign


def test_nearest_assign_basic():
    robots = {"r0": (0.0, 0.0), "r1": (10.0, 10.0)}
    orders = [("o0", (1.0, 1.0)), ("o1", (11.0, 11.0))]
    result = nearest_assign(robots, orders)
    assert result == {"r0": "o0", "r1": "o1"}


def test_nearest_assign_more_orders_than_robots():
    robots = {"r0": (0.0, 0.0)}
    orders = [("o0", (1.0, 1.0)), ("o1", (10.0, 10.0))]
    result = nearest_assign(robots, orders)
    assert result == {"r0": "o0"}


def test_nearest_assign_empty():
    assert nearest_assign({}, []) == {}
    assert nearest_assign({"r0": (0.0, 0.0)}, []) == {}
```

- [ ] **Step 2: Implement**

```python
def nearest_assign(
    robots: dict[str, tuple[float, float]],
    orders: Sequence[tuple[str, tuple[float, float]]],
) -> dict[str, str]:
    """Greedy nearest-AMR assignment: each robot grabs the closest
    unassigned order. Iterates in robot-id order — not optimal, but
    cheap and deterministic.
    """
    if not robots or not orders:
        return {}
    assignment: dict[str, str] = {}
    available = list(orders)
    for r_id, r_xy in sorted(robots.items()):
        if not available:
            break
        idx = min(
            range(len(available)),
            key=lambda i: (available[i][1][0] - r_xy[0]) ** 2
                          + (available[i][1][1] - r_xy[1]) ** 2,
        )
        assignment[r_id] = available[idx][0]
        del available[idx]
    return assignment
```

- [ ] **Step 3: Run, commit**

```bash
pytest tests/unit/test_nearest_assign.py -v
git add coordinator/assignment.py tests/unit/test_nearest_assign.py
git commit -m "feat(coord): nearest_assign greedy task allocator (Task 35)"
```

### Task 36: Register `CBSPlanner` in `strategy.py`

**Files:**
- Modify: `coordinator/strategy.py`
- Test: extend `tests/unit/test_strategy.py` (or create)

- [ ] **Step 1: Write failing test**

```python
def test_cbs_planner_registered():
    from coordinator.strategy import get_planner
    p = get_planner("cbs")
    assert p.name == "cbs"


def test_cbs_planner_resolves_conflicts():
    from coordinator.strategy import get_planner
    from coordinator.strategy import Goal

    p = get_planner("cbs")
    poses = {"r0": (0.0, 0.0), "r1": (2.0, 0.0)}
    goals = [Goal("r0", 2.0, 0.0), Goal("r1", 0.0, 0.0)]
    paths = p.plan(poses, goals)
    assert len(paths) == 2
    # Paths shouldn't simultaneously occupy the midpoint
```

- [ ] **Step 2: Implement `CBSPlanner` wrapping `coordinator/cbs.py`**

```python
from coordinator.cbs import solve_cbs  # whatever Phase 1 exposed


class CBSPlanner(PathPlanner):
    """Multi-agent path planner using Conflict-Based Search on a grid."""

    name = "cbs"

    def __init__(self, grid_w: int = 200, grid_h: int = 200, blocked=None):
        self.grid_w = grid_w
        self.grid_h = grid_h
        self.blocked = blocked or set()

    def plan(self, robot_poses, goals):
        # Convert continuous (x,y) to grid cells (5cm resolution)
        starts = {
            g.robot_id: (
                int(round(robot_poses[g.robot_id][0] / 0.05)),
                int(round(robot_poses[g.robot_id][1] / 0.05)),
            )
            for g in goals
        }
        goal_cells = {
            g.robot_id: (int(round(g.x / 0.05)), int(round(g.y / 0.05)))
            for g in goals
        }
        solution = solve_cbs(
            grid_w=self.grid_w, grid_h=self.grid_h, blocked=self.blocked,
            starts=starts, goals=goal_cells,
        )
        # Convert back to (x, y) waypoint paths
        return [
            Path(rid, tuple((c[0] * 0.05, c[1] * 0.05) for c in cells))
            for rid, cells in solution.items()
        ]


_REGISTRY["cbs"] = CBSPlanner
```

If `coordinator/cbs.py` doesn't expose a `solve_cbs` function, define a wrapper there that takes the same dict signature.

- [ ] **Step 3: Run, commit**

```bash
pytest tests/unit/test_strategy.py -v
git add coordinator/strategy.py coordinator/cbs.py tests/unit/test_strategy.py
git commit -m "feat(coord): register CBSPlanner in strategy registry (Task 36)"
```

### Task 37: Add `--allocator` and `--path-planner` to `run_scenario.py`

**Files:**
- Modify: `wdt_vast/run_scenario.py`
- Modify: `scenarios/schema.py` (if scenario schema validates fields)

- [ ] **Step 1: Add CLI argparsing**

Replace the hand-rolled `sys.argv` parsing at the top of `run_scenario.py`:

```python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scenario", help="Path to scenario YAML")
parser.add_argument("out_dir", help="Output directory")
parser.add_argument("--allocator", choices=["greedy", "hungarian"],
                    default="hungarian")
parser.add_argument("--path-planner", choices=["greedy", "cbs"],
                    default="cbs")
parser.add_argument("--seed", type=int, default=42)
args = parser.parse_args()

scenario_path = args.scenario
out_dir = Path(args.out_dir)
out_dir.mkdir(parents=True, exist_ok=True)
```

- [ ] **Step 2: Pass args through to the coordinator launch**

When the coordinator subprocess starts, pass:

```python
coordinator_proc = subprocess.Popen(
    ["ros2", "run", "fleet_coordinator", "fleet_coordinator_node",
     "--ros-args",
     "-p", f"allocator:={args.allocator}",
     "-p", f"path_planner:={args.path_planner}",
     "-p", f"seed:={args.seed}",
    ],
    # ...
)
```

In the coordinator node, declare these parameters and pass them to the planner factory:

```python
self.declare_parameter("allocator", "hungarian")
self.declare_parameter("path_planner", "cbs")
self.declare_parameter("seed", 42)
allocator_name = self.get_parameter("allocator").value
path_planner_name = self.get_parameter("path_planner").value
self._allocator = {
    "hungarian": hungarian_assign,
    "greedy": nearest_assign,
}[allocator_name]
self._path_planner = get_planner(path_planner_name)
```

- [ ] **Step 3: Pass seed into the scenario order generator**

`scenarios/schema.py`'s scenario loader (or the orchestrator in `run_scenario.py`) uses the seed for any randomization. Verify by grepping:

```bash
grep -n "random\|np.random\|Random" scenarios/ wdt_vast/run_scenario.py
```

If the existing scenario YAML lists exact arrival times (deterministic), seeds don't matter and we should add seed-based jitter for the ablation. Add to scenario schema:

```python
# scenarios/schema.py
import random

def apply_seed_jitter(orders, seed: int, jitter_s: float = 5.0):
    rng = random.Random(seed)
    return [
        replace(o, arrival_t=o.arrival_t + rng.uniform(-jitter_s, jitter_s))
        for o in orders
    ]
```

In `run_scenario.py`, after loading the scenario:

```python
scenario.orders = apply_seed_jitter(scenario.orders, args.seed)
```

- [ ] **Step 4: Commit**

```bash
git add wdt_vast/run_scenario.py scenarios/schema.py ros2_ws/src/fleet_coordinator/
git commit -m "feat(run): --allocator, --path-planner, --seed flags for ablation (Task 37)"
```

### Task 38: Ablation runner script

**Files:**
- Create: `wdt_vast/run_ablation.py`

- [ ] **Step 1: Write the runner**

```python
"""Drive the 15-run planner ablation.

Iterates (config, seed) over the 3 configs × 5 seeds grid, invoking
run_scenario.py for each. Writes outputs to runs/<config>/<seed>/.
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="scenarios/steady_state.yaml")
    parser.add_argument("--out-root", default="runs")
    parser.add_argument("--skip-existing", action="store_true",
                        help="Skip runs whose metrics.json already exists")
    args = parser.parse_args()

    out_root = Path(args.out_root)

    for config_name, alloc, planner in CONFIGS:
        for seed in SEEDS:
            run_dir = out_root / config_name / str(seed)
            if args.skip_existing and (run_dir / "metrics.json").exists():
                print(f"SKIP {config_name} seed={seed} (metrics.json exists)")
                continue
            run_dir.mkdir(parents=True, exist_ok=True)
            print(f"\n=== {config_name} seed={seed} ===")
            t0 = time.time()
            result = subprocess.run(
                ["/isaac-sim/python.sh", "wdt_vast/run_scenario.py",
                 args.scenario, str(run_dir),
                 "--allocator", alloc, "--path-planner", planner,
                 "--seed", str(seed)],
                check=False,
            )
            dt = time.time() - t0
            status = "OK" if result.returncode == 0 else "FAIL"
            print(f"{status} {config_name} seed={seed} took {dt/60:.1f} min")

            # Stop instance if we're approaching budget — vast.ai costs
            # ~$0.40/hr running. Each run ~30-45 min. Sanity check after every run.
            with open(out_root / "_runlog.txt", "a") as f:
                f.write(f"{config_name} {seed} {status} {dt:.1f}\n")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Commit**

```bash
git add wdt_vast/run_ablation.py
git commit -m "feat(vast): run_ablation.py drives 15-run planner grid (Task 38)"
```

### Task 39: Execute the 15-run ablation

**Files:** (run, no code)

- [ ] **Step 1: Resume vast.ai, run overnight**

```bash
vastai start instance 36775999
ssh vast-romania "cd ~/wdt && source /opt/ros/humble/setup.bash && \
    source ros2_ws/install/setup.bash && \
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
    python wdt_vast/run_ablation.py --out-root /tmp/ablation_runs \
        2>&1 | tee /tmp/ablation.log &"
```

- [ ] **Step 2: Monitor**

```bash
# From Mac, via Monitor tool
ssh vast-romania "tail -F /tmp/ablation.log" | grep -E "(=== |OK|FAIL)"
```

Expected total wall-clock: ~7.5 hours. Plan for overnight.

- [ ] **Step 3: Pull results back**

```bash
scp -r vast-romania:/tmp/ablation_runs/ runs/ablation/
```

- [ ] **Step 4: Verify all 15 metrics.json exist**

```bash
ls runs/ablation/*/*/metrics.json | wc -l
```

Expected: 15.

- [ ] **Step 5: Stop the vast.ai instance to save cost**

```bash
vastai stop instance 36775999
```

- [ ] **Step 6: Commit raw results**

```bash
git add runs/ablation/
git commit -m "results(m8): raw 15-run ablation metrics + events (Task 39)"
```

### Task 40: Cross-run aggregator — failing test

**Files:**
- Test: `tests/unit/test_aggregate.py`

- [ ] **Step 1: Write failing test**

```python
def test_aggregate_reads_metrics(tmp_path):
    from metrics.aggregate import aggregate_runs
    # Stub: 2 configs × 2 seeds = 4 runs
    for cfg in ["A", "B"]:
        for s in [1, 2]:
            d = tmp_path / cfg / str(s)
            d.mkdir(parents=True)
            (d / "metrics.json").write_text(
                '{"orders_completed": 10, "pick_success_rate": 0.8, '
                '"deadlocks": 1, "mean_cycle_time_s": 30.0}'
            )
    df = aggregate_runs(tmp_path)
    assert len(df) == 4
    assert set(df["config"]) == {"A", "B"}


def test_aggregate_summarize(tmp_path):
    from metrics.aggregate import aggregate_runs, summarize_by_config
    for cfg in ["A"]:
        for s in [1, 2, 3]:
            d = tmp_path / cfg / str(s)
            d.mkdir(parents=True)
            (d / "metrics.json").write_text(
                f'{{"orders_completed": {10+s}, "pick_success_rate": 0.8, '
                f'"deadlocks": {s}, "mean_cycle_time_s": 30.0}}'
            )
    df = aggregate_runs(tmp_path)
    summary = summarize_by_config(df)
    a = summary[summary["config"] == "A"].iloc[0]
    assert a["orders_completed_mean"] == 12.0  # (11+12+13)/3
    assert abs(a["deadlocks_std"] - 1.0) < 0.001  # std([1,2,3]) ≈ 1
```

- [ ] **Step 2: Commit failing test**

```bash
git add tests/unit/test_aggregate.py
git commit -m "test(metrics): failing tests for aggregate (Task 40)"
```

### Task 41: Cross-run aggregator — implementation

**Files:**
- Create: `metrics/aggregate.py`

- [ ] **Step 1: Implement**

```python
"""Aggregate per-run metrics.json files into a single DataFrame and
summarize by (config, planner) for the ablation report.
"""

from __future__ import annotations

import json
from pathlib import Path

import pandas as pd

METRIC_COLS = ["orders_completed", "pick_success_rate", "deadlocks", "mean_cycle_time_s"]


def aggregate_runs(root: Path) -> pd.DataFrame:
    """Read all metrics.json under root/<config>/<seed>/ into a long DataFrame.
    Each row is one run with columns: config, seed, plus METRIC_COLS.
    """
    rows = []
    for metrics_path in Path(root).glob("*/*/metrics.json"):
        config = metrics_path.parent.parent.name
        seed = int(metrics_path.parent.name)
        data = json.loads(metrics_path.read_text())
        row = {"config": config, "seed": seed}
        for col in METRIC_COLS:
            row[col] = data.get(col)
        rows.append(row)
    return pd.DataFrame(rows)


def summarize_by_config(df: pd.DataFrame) -> pd.DataFrame:
    """Mean + std for each metric, grouped by config."""
    agg = df.groupby("config")[METRIC_COLS].agg(["mean", "std"]).reset_index()
    # Flatten MultiIndex columns: ("orders_completed", "mean") -> "orders_completed_mean"
    agg.columns = [
        c if isinstance(c, str) else f"{c[0]}_{c[1]}"
        for c in agg.columns
    ]
    return agg
```

- [ ] **Step 2: Verify tests pass**

```bash
pytest tests/unit/test_aggregate.py -v
```

- [ ] **Step 3: Commit**

```bash
git add metrics/aggregate.py
git commit -m "feat(metrics): aggregate_runs + summarize_by_config (Task 41)"
```

### Task 42: Generate `docs/results-phase-2.md`

**Files:**
- Create: `scripts/generate_results_md.py`
- Create: `docs/results-phase-2.md`

- [ ] **Step 1: Write the generator**

```python
"""Generate docs/results-phase-2.md from the aggregated ablation results."""

from __future__ import annotations

from pathlib import Path

import pandas as pd
from scipy import stats

from metrics.aggregate import aggregate_runs, summarize_by_config


def main():
    df = aggregate_runs(Path("runs/ablation"))
    summary = summarize_by_config(df)

    # p-values vs greedy_greedy baseline
    baseline = df[df["config"] == "greedy_greedy"]
    pvals = {}
    for cfg in df["config"].unique():
        if cfg == "greedy_greedy":
            continue
        target = df[df["config"] == cfg]
        pvals[cfg] = {
            metric: stats.ttest_ind(baseline[metric], target[metric]).pvalue
            for metric in ["orders_completed", "deadlocks",
                           "pick_success_rate", "mean_cycle_time_s"]
        }

    md = [
        "# Phase 2 Results — Planner Ablation\n",
        "**Scenario:** `steady_state.yaml` (64 orders, 6 Carters, 1 Franka)",
        "**Seeds:** 42, 43, 44, 45, 46",
        f"**Total runs:** {len(df)} (3 configs × 5 seeds)",
        "",
        "## Headline metrics (mean ± std)\n",
        "| Config | orders/hr | mean cycle (s) | deadlocks/min | pick success |",
        "|---|---|---|---|---|",
    ]
    for _, row in summary.iterrows():
        cfg = row["config"]
        oc_mean = row["orders_completed_mean"]
        oc_std = row["orders_completed_std"]
        ct_mean = row["mean_cycle_time_s_mean"]
        ct_std = row["mean_cycle_time_s_std"]
        dl_mean = row["deadlocks_mean"]
        dl_std = row["deadlocks_std"]
        ps_mean = row["pick_success_rate_mean"]
        ps_std = row["pick_success_rate_std"]
        md.append(
            f"| `{cfg}` | {oc_mean:.1f} ± {oc_std:.1f} | "
            f"{ct_mean:.1f} ± {ct_std:.1f} | "
            f"{dl_mean:.2f} ± {dl_std:.2f} | "
            f"{ps_mean:.2f} ± {ps_std:.2f} |"
        )

    md += ["", "## p-values vs `greedy_greedy` baseline\n"]
    md.append("| Config | orders_completed | deadlocks | pick_success | cycle_time |")
    md.append("|---|---|---|---|---|")
    for cfg, pv in pvals.items():
        md.append(
            f"| `{cfg}` | {pv['orders_completed']:.4f} | "
            f"{pv['deadlocks']:.4f} | "
            f"{pv['pick_success_rate']:.4f} | "
            f"{pv['mean_cycle_time_s']:.4f} |"
        )

    Path("docs/results-phase-2.md").write_text("\n".join(md))


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run**

```bash
python scripts/generate_results_md.py
cat docs/results-phase-2.md
```

- [ ] **Step 3: Commit**

```bash
git add scripts/generate_results_md.py docs/results-phase-2.md
git commit -m "feat(metrics): generate docs/results-phase-2.md from ablation runs (Task 42)"
```

### Task 43: Generate ablation plots

**Files:**
- Create: `scripts/generate_ablation_plots.py`
- Create: `docs/images/ablation/{throughput,cycle_time,deadlocks,pick_rate}.png`

- [ ] **Step 1: Write the plot generator**

```python
"""Bar charts with error bars for the 4 headline metrics × 3 configs."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt

from metrics.aggregate import aggregate_runs, summarize_by_config

METRICS = {
    "throughput": ("orders_completed", "Orders completed", "count"),
    "cycle_time": ("mean_cycle_time_s", "Mean cycle time", "s"),
    "deadlocks": ("deadlocks", "Deadlocks", "count"),
    "pick_rate": ("pick_success_rate", "Pick success rate", "fraction"),
}

OUT_DIR = Path("docs/images/ablation")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def main():
    df = aggregate_runs(Path("runs/ablation"))
    summary = summarize_by_config(df)

    config_order = ["greedy_greedy", "hungarian_greedy", "hungarian_cbs"]
    summary = summary.set_index("config").reindex(config_order).reset_index()

    for fname, (metric, label, unit) in METRICS.items():
        fig, ax = plt.subplots(figsize=(6, 4))
        means = summary[f"{metric}_mean"]
        stds = summary[f"{metric}_std"]
        ax.bar(config_order, means, yerr=stds, capsize=8,
               color=["#aaa", "#69c", "#c66"])
        ax.set_ylabel(f"{label} ({unit})")
        ax.set_title(f"{label} by planner config (N=5 each)")
        plt.xticks(rotation=10)
        plt.tight_layout()
        plt.savefig(OUT_DIR / f"{fname}.png", dpi=150)
        plt.close(fig)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run + commit**

```bash
python scripts/generate_ablation_plots.py
git add scripts/generate_ablation_plots.py docs/images/ablation/
git commit -m "feat(metrics): bar-chart plots for ablation headline metrics (Task 43)"
git push origin main
```

M8 closes.

---

## Milestone 9 — Results writeup + `v0.2.0` release

### Task 44: Update README with Phase 2 numbers

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Insert a Phase 2 section before/after the Phase 1 section**

```markdown
## Phase 2 — Closed-loop demo + planner ablation (v0.2.0)

Phase 1 shipped the structural skeleton. Phase 2 closes the two integration gaps (real Nav2 + real manipulation) and runs a 3-config × 5-seed planner ablation on the 64-order steady_state scenario.

**Headline numbers** (see [docs/results-phase-2.md](docs/results-phase-2.md) for full table):

| Config | orders/hr | deadlocks/min | pick success |
|---|---|---|---|
| `greedy_greedy` | … | … | … |
| `hungarian_greedy` | … | … | … |
| `hungarian_cbs` | … | … | … |

![throughput ablation](docs/images/ablation/throughput.png)

[60s closed-loop demo video](docs/videos/phase-2/v0.2.0-demo.mp4)
```

Fill in the actual numbers from `docs/results-phase-2.md`.

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: Phase 2 README section with ablation numbers + demo link (Task 44)"
```

### Task 45: Assemble the v0.2.0 demo video

**Files:**
- Create: `docs/videos/phase-2/v0.2.0-demo.mp4`

- [ ] **Step 1: Pick the best run** (the `hungarian_cbs` run with the smoothest trajectory)

- [ ] **Step 2: Use the existing `metrics/video.py::assemble_mp4`**

```bash
python -c "
from metrics.video import assemble_mp4
assemble_mp4(
    frames_dir='runs/ablation/hungarian_cbs/42/replicator',
    out_path='docs/videos/phase-2/v0.2.0-demo.mp4',
    fps=30,
    max_seconds=90,
)
"
```

- [ ] **Step 3: Sanity-check duration + size**

```bash
ffprobe -v error -show_entries format=duration,size docs/videos/phase-2/v0.2.0-demo.mp4
```

Expected: 60–90s, < 50 MB (GitHub release-friendly).

- [ ] **Step 4: Commit**

```bash
git add docs/videos/phase-2/v0.2.0-demo.mp4
git commit -m "docs(video): v0.2.0 demo — hungarian_cbs steady_state run (Task 45)"
```

### Task 46: Tag `v0.2.0` and publish release

**Files:** (git + GitHub)

- [ ] **Step 1: Final push**

```bash
git push origin main
```

- [ ] **Step 2: Tag**

```bash
git tag -a v0.2.0 -m "Phase 2: closed-loop demo + planner ablation"
git push origin v0.2.0
```

- [ ] **Step 3: Create release with assets**

```bash
gh release create v0.2.0 \
    --title "v0.2.0 — Closed-loop demo + planner ablation" \
    --notes-file <(cat <<'EOF'
## Phase 2 summary

- Closed Phase 1's two integration gaps: real Nav2 (full stack) + real manipulation (MoveIt2 + FoundationPose).
- Ran a 3-config × 5-seed planner ablation on the 64-order steady_state scenario.
- See [docs/results-phase-2.md](docs/results-phase-2.md) for the full results.

## Assets
- `v0.2.0-demo.mp4` — 60s closed-loop run
- `ablation-results.tar.gz` — the 15 metrics.json + events.log files
- `ablation-plots.zip` — throughput, deadlocks, cycle_time, pick_rate bar charts

## What's next (Phase 3)
- Scale up to 12–20 AMRs, 50×50 m warehouse
- Live web dashboard
- Optional AnyGrasp integration
EOF
) \
    docs/videos/phase-2/v0.2.0-demo.mp4 \
    docs/images/ablation/*.png

# Bundle ablation runs
tar -czf /tmp/ablation-results.tar.gz -C runs ablation/
gh release upload v0.2.0 /tmp/ablation-results.tar.gz
```

- [ ] **Step 4: Verify release**

```bash
gh release view v0.2.0
```

Confirm video, plots, tarball all attached.

- [ ] **Step 5: Update the project memory**

Update `[[warehouse-digital-twin-project]]` memory to reflect Phase 2 ship status and what "resume" means for Phase 3 brainstorming. (See the memory file for the existing format.)

```bash
# This is a memory update, not a git commit. Use the Write tool on the
# memory file directly.
```

---

## End-of-plan checklist (read before claiming Phase 2 done)

- [ ] All 9 milestones (M0–M9) committed and pushed.
- [ ] `git tag v0.2.0` exists and is pushed.
- [ ] GitHub release `v0.2.0` exists with: demo video, ablation plots, ablation results tarball.
- [ ] `docs/results-phase-2.md` shows `orders_completed > 0` on `hungarian_cbs`.
- [ ] All 15 ablation runs are checked into `runs/ablation/`.
- [ ] README has a Phase 2 section linking to results + video.
- [ ] Total spend ≤ $15 (audit via `vastai show invoices` + `modal app stats`).
- [ ] Memory updated: [[warehouse-digital-twin-project]] reflects Phase 2 ship status.

"""Render the combined Phase-1 scene: warehouse + 6 AMRs + Franka.

Invoked on a vast.ai instance via:
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    /isaac-sim/python.sh wdt_vast/combined_render.py <warehouse_usd> <out_dir>

Steps:
- Open the procedurally-built warehouse USD (from `python -m
  warehouse.generators.build_scene small`)
- Spawn 6 namespaced Nova Carters at the layout's AMR spawn poses
- Spawn a Franka at the pick-cell position
- Step the world a few frames so physics settles
- Render three camera angles (overhead, isometric, hero) to PNGs

Output:
    <out_dir>/overhead/rgb_0000.png
    <out_dir>/iso/rgb_0000.png
    <out_dir>/hero/rgb_0000.png
"""

import sys
from datetime import datetime, timezone
from pathlib import Path

PROGRESS = Path("/tmp/combined_render_progress.txt")
PROGRESS.write_text("")


def mark(phase: str) -> None:
    with PROGRESS.open("a") as fh:
        fh.write(f"{datetime.now(timezone.utc).isoformat()}  {phase}\n")


mark("script_start")

from isaacsim import SimulationApp  # noqa: E402  (must call mark() above SimApp boot)

sim = SimulationApp({"headless": True})
mark("simapp_booted")

import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402

sys.path.insert(0, "/tmp")
from sim.multi_robot import spawn_amr_fleet  # noqa: E402
from sim.spawn import spawn_franka  # noqa: E402
from warehouse.layout import load_layout  # noqa: E402

warehouse_usd = sys.argv[1] if len(sys.argv) > 1 else "/tmp/small.usd"
out_dir = sys.argv[2] if len(sys.argv) > 2 else "/tmp/combined_out"

# Open the warehouse stage first
omni.usd.get_context().open_stage(warehouse_usd)
mark("warehouse_stage_opened")

# Step a few frames so the stage actually loads
for _ in range(10):
    sim.update()

# Spawn robots onto the open stage
layout = load_layout("/tmp/warehouse/layouts/small.yaml")
gx, gy = layout.amrs.spawn.grid
ox, oy = layout.amrs.spawn.origin_xy
spacing = layout.amrs.spawn.spacing_m
poses = [(ox + c * spacing, oy + r * spacing) for r in range(gy) for c in range(gx)][
    : layout.amrs.count
]

# spawn_amr_fleet uses World().scene.add — but here we don't have a World yet.
# Use the lower-level add_reference_to_stage for each Carter via spawn_nova_carter
# OR: just create a World after opening the stage. The latter is simpler.
from isaacsim.core.api import World  # noqa: E402

world = World()
spawn_amr_fleet(world, poses)
mark(f"fleet_spawned_n={len(poses)}")

px, py = layout.pick_cell.position_xy
spawn_franka(world, "/World/pick_arm", "pick_arm", position_xyz=(px, py, 1.0))
mark("franka_spawned")

world.reset()
mark("world_reset")

# Settle the scene
for _ in range(60):
    world.step(render=True)
mark("settled_60_frames")

# Three camera angles framing the layout (same as Task 16's polished render)
CAMS = [
    ("overhead", (10, 30, 22), (10, 15, 0)),
    ("iso", (28, 38, 14), (10, 15, 1)),
    ("hero", (10, 1, 2.5), (10, 22, 1)),
    # AMR closeup — derived from small.yaml: AMRs in a 3x2 grid starting at
    # (2, 2) with 1.5m spacing, so the cluster centroid is (3.5, 2.75, 0.2)
    # and its extent is 3m x 1.5m x 0.4m. The camera applies a standard
    # isometric framing (35deg elevation, 45deg azimuth, 5m distance):
    #   dx,dy = 5 * cos(35) * cos/sin(45) ≈ ±2.9m
    #   dz    = 5 * sin(35) ≈ 2.87m
    # All inside the warehouse footprint (0 < x < 20, 0 < y < 30).
    ("amrs", (6.4, 5.65, 3.07), (3.5, 2.75, 0.2)),
]

# Stop physics before rendering — World playing in the background interferes
# with Replicator's orchestrator on subsequent renders (only the first cam
# captures; iso/ and hero/ stay empty when world.step() is still being
# triggered or physics is mid-tick).
world.stop()
mark("world_stopped")

Path(out_dir).mkdir(parents=True, exist_ok=True)
for name, pos, look_at in CAMS:
    cam_out = f"{out_dir}/{name}"
    Path(cam_out).mkdir(parents=True, exist_ok=True)
    cam = rep.create.camera(position=pos, look_at=look_at)
    rp = rep.create.render_product(cam, (1920, 1080))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=cam_out, rgb=True)
    writer.attach([rp])
    rep.orchestrator.step()
    rep.orchestrator.wait_until_complete()
    writer.detach()
    mark(f"rendered_{name}")

sim.close()
mark("sim_closed")

"""Render the pick cell in the polished portfolio style.

Builds the same scene as run_scenario.py — AMR fleet + Franka + table +
cube + lighting — but instead of running the smoke loop, renders one
high-resolution isometric frame via Replicator and exits. Output is a
single 1920x1080 PNG matching the `outputs/render/iso/rgb_0000.png` style.

Invocation on the vast.ai instance:
    /isaac-sim/python.sh wdt_vast/render_pick_cell.py /tmp/pick_cell_render

Writes:
    <out_dir>/iso/rgb_0000.png   — polished isometric view of pick cell
"""

from __future__ import annotations

import sys
from datetime import datetime, timezone
from pathlib import Path

PROGRESS = Path("/tmp/render_pick_cell_progress.txt")
PROGRESS.write_text("")


def mark(phase: str) -> None:
    with PROGRESS.open("a") as fh:
        fh.write(f"{datetime.now(timezone.utc).isoformat()}  {phase}\n")


mark("script_start")

from isaacsim import SimulationApp  # noqa: E402

sim = SimulationApp({"headless": True})
mark("simapp_booted")

import omni.replicator.core as rep  # noqa: E402

sys.path.insert(0, "/tmp")
from isaacsim.core.api import World  # noqa: E402

from sim.multi_robot import spawn_amr_fleet  # noqa: E402
from sim.spawn import (  # noqa: E402
    spawn_franka,
    spawn_pick_cell_lighting,
    spawn_pick_cube,
    spawn_pick_table,
)
from warehouse.layout import load_layout  # noqa: E402

out_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/pick_cell_render"
warehouse_usd = sys.argv[2] if len(sys.argv) > 2 else "/tmp/small.usd"

world = World()
world.scene.add_default_ground_plane()

from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

if Path(warehouse_usd).exists():
    add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/warehouse")
    mark(f"warehouse_loaded={warehouse_usd}")

# Match run_scenario.py — use small layout's pick_cell + spawn AMRs.
layout = load_layout("/tmp/warehouse/layouts/small.yaml")
gx, gy = layout.amrs.spawn.grid
ox, oy = layout.amrs.spawn.origin_xy
spacing = layout.amrs.spawn.spacing_m
poses = [(ox + c * spacing, oy + r * spacing) for r in range(gy) for c in range(gx)][:2]
spawn_amr_fleet(world, poses)
mark(f"fleet_spawned_n={len(poses)}")

px, py = layout.pick_cell.position_xy
spawn_franka(world, "/World/pick_arm", "pick_arm", position_xyz=(px, py, 1.0))
mark("franka_spawned")

table_center = (px + 0.40, py, 0.36)
cube_center = (px + 0.40, py, 0.75)
spawn_pick_table(world, center_xyz=table_center, size_xyz=(0.6, 0.6, 0.7))
spawn_pick_cube(world, center_xyz=cube_center, edge_m=0.08)
spawn_pick_cell_lighting()
mark("table_cube_lighting_spawned")

world.reset()
for _ in range(60):
    world.step(render=True)
mark("settled_60_frames")
world.stop()
mark("world_stopped")

# Isometric view of the pick cell. The Franka + table + cube occupy roughly
# world (16.0..16.5, 14.7..15.3, 0..1.5). Position the camera 5m out at
# 35° elevation, 45° azimuth pointing at the cube center (16.40, 15.0, 0.75).
import math  # noqa: E402

look_at = (16.40, 15.0, 0.75)
elevation_deg = 30.0
azimuth_deg = 215.0  # behind-left of the Franka, looking northeast
distance = 4.5
ele = math.radians(elevation_deg)
azi = math.radians(azimuth_deg)
cam_pos = (
    look_at[0] + distance * math.cos(ele) * math.cos(azi),
    look_at[1] + distance * math.cos(ele) * math.sin(azi),
    look_at[2] + distance * math.sin(ele),
)
mark(f"camera_pos={cam_pos}_look_at={look_at}")

Path(out_dir).mkdir(parents=True, exist_ok=True)
iso_dir = f"{out_dir}/iso"
Path(iso_dir).mkdir(parents=True, exist_ok=True)

cam = rep.create.camera(position=cam_pos, look_at=look_at)
rp = rep.create.render_product(cam, (1920, 1080))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=iso_dir, rgb=True)
writer.attach([rp])
rep.orchestrator.step()
rep.orchestrator.wait_until_complete()
writer.detach()
mark("rendered_iso")

sim.close()
mark("sim_closed")
print(f"==> wrote {iso_dir}/rgb_0000.png")

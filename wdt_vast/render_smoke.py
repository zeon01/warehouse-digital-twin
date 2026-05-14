"""Standalone Isaac Sim smoke render — boots Kit headless, renders a PNG.

Invoked on a vast.ai instance via:
    /isaac-sim/python.sh wdt_vast/render_smoke.py <out_dir>

Self-contained (no project imports). Same Kit boot pattern that works under
NVIDIA's python.sh launcher: env vars + LD_PRELOAD=libcarb.so are set by
python.sh before this script runs.
"""

import os
import sys

from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.api import World  # noqa: E402  (must import after SimulationApp)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

for _ in range(10):
    world.step(render=True)

out_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/smoke_out"
os.makedirs(out_dir, exist_ok=True)

cam = rep.create.camera(position=(5, 5, 5), look_at=(0, 0, 0))
rp = rep.create.render_product(cam, (1280, 720))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=out_dir, rgb=True)
writer.attach([rp])
rep.orchestrator.step()
rep.orchestrator.wait_until_complete()

sim.close()
print(f"RENDER_DONE:{out_dir}")

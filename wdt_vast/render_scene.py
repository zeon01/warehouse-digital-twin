"""Render an existing USD scene from an overhead camera to a PNG.

Invoked on a vast.ai instance via:
    /isaac-sim/python.sh wdt_vast/render_scene.py <usd_path> <out_dir>

Loads the USD into Kit, places an overhead camera that frames the warehouse
footprint, and saves a single 1920x1080 PNG via Replicator's BasicWriter.
"""

import os
import sys

from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402

usd_path = sys.argv[1]
out_dir = sys.argv[2] if len(sys.argv) > 2 else "/tmp/render_out"

omni.usd.get_context().open_stage(usd_path)

os.makedirs(out_dir, exist_ok=True)

# Overhead camera, positioned to look down at the small layout's center
# (~10, 15) from 25m up. Larger layouts will frame differently.
cam = rep.create.camera(position=(10, 15, 25), look_at=(10, 15, 0))
rp = rep.create.render_product(cam, (1920, 1080))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=out_dir, rgb=True)
writer.attach([rp])
rep.orchestrator.step()
rep.orchestrator.wait_until_complete()

sim.close()
print(f"RENDER_DONE:{out_dir}")

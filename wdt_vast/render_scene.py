"""Render an existing USD scene from three camera angles to PNGs.

Invoked on a vast.ai instance via:
    /isaac-sim/python.sh wdt_vast/render_scene.py <usd_path> <out_dir>

Loads the USD into Kit and produces three 1920x1080 PNGs:
- overhead.png   — tilted overhead, full layout visible
- iso.png        — isometric corner shot, shows 3D extent
- hero.png       — ground-level perspective for a more cinematic frame

The camera positions assume a layout roughly 20x30m centered around (10, 15).
For different-sized layouts the framing will be off; future work can derive
camera positions from the LayoutConfig instead.
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

# Three camera angles: name → (position, look_at)
CAMS = [
    ("overhead", (10, 30, 22), (10, 15, 0)),  # tilted overhead, looking south
    ("iso", (28, 38, 14), (10, 15, 1)),  # isometric from NE corner
    ("hero", (10, 1, 2.5), (10, 22, 1)),  # interior, south entrance looking N
]

for name, pos, look_at in CAMS:
    cam_out = os.path.join(out_dir, name)
    os.makedirs(cam_out, exist_ok=True)
    cam = rep.create.camera(position=pos, look_at=look_at)
    rp = rep.create.render_product(cam, (1920, 1080))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=cam_out, rgb=True)
    writer.attach([rp])
    rep.orchestrator.step()
    rep.orchestrator.wait_until_complete()
    writer.detach()
    print(f"WROTE:{name}:{cam_out}")

sim.close()
print(f"RENDER_DONE:{out_dir}")

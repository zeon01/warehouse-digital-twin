"""Boot Isaac Sim headless and render one frame to a PNG on the volume.

KNOWN BLOCKER (2026-05-15): This module CANNOT successfully render on Modal
today. Kit boots fine via the subprocess pattern below, but every render
attempt fails with Vulkan ERROR_DEVICE_LOST (L4, B200) or
ERROR_INITIALIZATION_FAILED (A10G), despite Modal's containers having a
valid Vulkan setup (driver 580.95, NVIDIA_DRIVER_CAPABILITIES=all,
VK_DRIVER_FILES configured, libGLX_nvidia.so present). The root cause
appears to be that Modal containers don't expose the specific Vulkan
extensions (likely RT and/or swapchain) that Isaac Sim 5.0 negotiates at
device creation time. CUDA compute works fine — only the graphics
pipeline is broken.

Architecture decision (still valid once Vulkan is unblocked):
Isaac Sim 5.0's Kit C extensions (carb._carb, omni.*, isaacsim.simulation_app)
require LD_PRELOAD=/isaac-sim/kit/libcarb.so set before the Python process
starts. Setting LD_PRELOAD in-function doesn't work — the env change happens
after Python is already loaded. So the simulation runs as a subprocess via
/isaac-sim/python.sh, which is NVIDIA's launcher: it sources setup_python_env.sh
(PYTHONPATH, LD_LIBRARY_PATH, CARB_APP_PATH, EXP_PATH), sets LD_PRELOAD, and
execs Kit's bundled Python 3.11.

The Modal function is a thin orchestration wrapper around that subprocess.

Path forward when revisiting:
- Try `--/renderer/active=raster` kit arg to bypass Vulkan RT
- File support ticket with Modal — give them the exact failure modes
- Or migrate rendering tasks to Brev / Runpod / Lambda which have
  a Vulkan setup tested with Isaac Sim 5.0.
"""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

from wdt_modal.app import app
from wdt_modal.volumes import RUNS_PATH, VOLUME_MOUNT, isaac_volume

_ISAAC_SCRIPT = '''"""Inner script — runs under /isaac-sim/python.sh."""
import os
import sys

from isaacsim import SimulationApp

sim = SimulationApp({"headless": True})

from isaacsim.core.api import World  # noqa: E402  (must import after SimulationApp)
import omni.replicator.core as rep  # noqa: E402

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

# Step a few times so the renderer warms up.
for _ in range(10):
    world.step(render=True)

out_dir = sys.argv[1]
os.makedirs(out_dir, exist_ok=True)

cam = rep.create.camera(position=(5, 5, 5), look_at=(0, 0, 0))
rp = rep.create.render_product(cam, (1280, 720))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=out_dir, rgb=True)
writer.attach([rp])
rep.orchestrator.step()
rep.orchestrator.wait_until_complete()

sim.close()
print(f"INNER_DONE:{out_dir}")
'''


@app.function(
    # Plan default. Note: rendering is currently blocked on ALL Modal GPU
    # types — see docstring at top of this module. L4 is the cheapest;
    # leaving it here as the default so when the Vulkan issue is resolved,
    # cost is reasonable. Tested L4, A10G, B200 — all failed identically.
    gpu="L4",
    timeout=900,
    startup_timeout=600,
    volumes={VOLUME_MOUNT: isaac_volume},
)
def boot_and_screenshot() -> str:
    """Boot Isaac Sim Kit headless, render one frame, return the output dir."""
    ts = time.strftime("%Y-%m-%dT%H-%M-%S")
    out_dir = f"{RUNS_PATH}/smoke-{ts}"

    script_path = "/tmp/isaac_smoke_inner.py"
    Path(script_path).write_text(_ISAAC_SCRIPT)

    subprocess.run(
        ["/isaac-sim/python.sh", script_path, out_dir],
        check=True,
    )

    isaac_volume.commit()
    return out_dir

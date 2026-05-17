"""Layer 1: probe whether Modal's RTX PRO 6000 container exposes Vulkan + DRM.

No NGC creds needed. Verifies the necessary container-runtime plumbing
(libvulkan, /dev/dri/renderD128, EGL) before we bother pulling Isaac
Sim's 30 GB image. If this layer fails, Isaac Sim cannot run on this
Modal SKU regardless of NGC.

Layer 2 (pull nvcr.io/nvidia/isaac-sim:5.0.0 + boot SimulationApp) is in
``probe_isaac_sim_layer2.py`` — needs an ``ngc-pull`` Modal Secret.

Run:
    modal run wdt_modal/probe_isaac_sim.py::vulkan_probe
"""

from __future__ import annotations

import modal

app = modal.App("warehouse-isaac-sim-probe")

vulkan_image = modal.Image.debian_slim().apt_install(
    "vulkan-tools",
    "libvulkan1",
    "libegl1",
    "libgl1",
    "mesa-vulkan-drivers",
    "pciutils",
)


@app.function(image=vulkan_image, gpu="RTX-PRO-6000", timeout=300)
def vulkan_probe() -> dict:
    """Verify Vulkan + DRM render node + EGL on Modal's RTX PRO 6000."""
    import os
    import subprocess

    def run(cmd: list[str]) -> tuple[int, str, str]:
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            return p.returncode, p.stdout, p.stderr
        except Exception as exc:  # noqa: BLE001
            return -1, "", str(exc)

    result: dict = {}

    rc, out, err = run(
        ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"]
    )
    result["nvidia_smi"] = {"rc": rc, "stdout": out.strip(), "stderr": err.strip()}

    try:
        result["dev_dri"] = (
            sorted(os.listdir("/dev/dri")) if os.path.exists("/dev/dri") else "missing"
        )
    except Exception as exc:  # noqa: BLE001
        result["dev_dri"] = f"err: {exc}"

    rc, out, err = run(["vulkaninfo", "--summary"])
    result["vulkaninfo_summary"] = {
        "rc": rc,
        "stdout_first_500": out[:500],
        "stderr_first_500": err[:500],
    }

    rc, out, err = run(["vulkaninfo"])
    devices = []
    for line in out.splitlines():
        if "deviceName" in line:
            devices.append(line.strip())
    result["vulkan_devices"] = devices[:5]

    rc, out, _ = run(["ldconfig", "-p"])
    result["egl_libs"] = [
        line.strip() for line in out.splitlines() if "libEGL" in line or "libGLESv2" in line
    ][:10]

    return result


@app.local_entrypoint()
def main():
    from pprint import pprint

    pprint(vulkan_probe.remote())

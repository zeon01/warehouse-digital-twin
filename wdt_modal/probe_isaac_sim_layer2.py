"""Layer 2: pull nvcr.io/nvidia/isaac-sim:5.0.0 on Modal + run a probe.

Stripped to 3 stages so we can localize where Modal+IsaacSim fails:

  - ``smoke_nvidia_smi``: just nvidia-smi inside the Isaac Sim container
    on an RTX-PRO-6000 instance. Verifies image pull + GPU mount.
  - ``smoke_vulkaninfo``: vulkaninfo inside the Isaac Sim container.
    Verifies the NVIDIA Vulkan ICD bundled in the image is loadable.
  - ``smoke_sim_app``: launch SimulationApp(headless=True). The real test.

Run sequentially via ``modal run``:
    modal run wdt_modal/probe_isaac_sim_layer2.py::smoke_nvidia_smi
    modal run wdt_modal/probe_isaac_sim_layer2.py::smoke_vulkaninfo
    modal run wdt_modal/probe_isaac_sim_layer2.py::smoke_sim_app
"""

from __future__ import annotations

import modal

app = modal.App("warehouse-isaac-sim-probe-l2")

# Isaac Sim image. add_python adds Modal's bundled Python alongside the
# image's existing /isaac-sim/python.sh — required so Modal can run our
# function. The Isaac Sim image is ~30 GB so first build can take 5-10 min.
isaac_image = modal.Image.from_registry(
    "nvcr.io/nvidia/isaac-sim:5.0.0",
    secret=modal.Secret.from_name("ngc-pull"),
    add_python="3.10",
).env(
    {
        "ACCEPT_EULA": "Y",
        "OMNI_KIT_ACCEPT_EULA": "Y",
        "PRIVACY_CONSENT": "Y",
    }
)


@app.function(image=isaac_image, gpu="RTX-PRO-6000", timeout=300)
def smoke_nvidia_smi() -> dict:
    import subprocess

    p = subprocess.run(
        ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"],
        capture_output=True,
        text=True,
        timeout=30,
    )
    return {"rc": p.returncode, "stdout": p.stdout.strip(), "stderr": p.stderr.strip()}


@app.function(image=isaac_image, gpu="RTX-PRO-6000", timeout=300)
def smoke_vulkaninfo() -> dict:
    import subprocess

    # Use the Isaac Sim image's bundled vulkaninfo (usually under
    # /isaac-sim/kit/...) or fall back to system if missing.
    candidates = [
        "/isaac-sim/kit/_build/target-deps/vulkansdk/x86_64-linux/bin/vulkaninfo",
        "/usr/bin/vulkaninfo",
        "vulkaninfo",
    ]
    for cmd in candidates:
        try:
            p = subprocess.run([cmd], capture_output=True, text=True, timeout=30)
        except FileNotFoundError:
            continue
        devices = [line.strip() for line in p.stdout.splitlines() if "deviceName" in line]
        return {
            "cmd": cmd,
            "rc": p.returncode,
            "vulkan_devices": devices[:5],
            "stdout_first_400": p.stdout[:400],
            "stderr_first_400": p.stderr[:400],
        }
    return {"error": "no vulkaninfo found"}


@app.function(image=isaac_image, gpu="RTX-PRO-6000", timeout=600)
def smoke_sim_app() -> dict:
    import subprocess

    script = (
        "import sys\n"
        "try:\n"
        "    from isaacsim import SimulationApp\n"
        "    app = SimulationApp({'headless': True})\n"
        "    print('SIM_APP_OK')\n"
        "    app.close()\n"
        "    print('SHUTDOWN_OK')\n"
        "except Exception as e:\n"
        "    print(f'SIM_APP_FAIL: {type(e).__name__}: {e}', file=sys.stderr)\n"
        "    sys.exit(2)\n"
    )

    p = subprocess.run(
        ["/isaac-sim/python.sh", "-c", script],
        capture_output=True,
        text=True,
        timeout=400,
    )
    return {
        "returncode": p.returncode,
        "stdout_last_2000": p.stdout[-2000:],
        "stderr_last_2000": p.stderr[-2000:],
    }


@app.local_entrypoint()
def main(stage: str = "nvidia_smi"):
    from pprint import pprint

    if stage == "nvidia_smi":
        pprint(smoke_nvidia_smi.remote())
    elif stage == "vulkaninfo":
        pprint(smoke_vulkaninfo.remote())
    elif stage == "sim_app":
        pprint(smoke_sim_app.remote())
    else:
        raise SystemExit(f"unknown stage: {stage}")

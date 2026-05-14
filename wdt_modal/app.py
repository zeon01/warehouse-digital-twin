"""Modal app definition for the warehouse digital twin."""

from __future__ import annotations

import modal

from wdt_modal.image import image

app = modal.App("warehouse-digital-twin", image=image)


@app.function(gpu="L4", timeout=300, startup_timeout=900)
def healthcheck() -> dict[str, str]:
    """Smoke-test that the Modal image boots, GPU works, and ROS2 is available."""
    import subprocess

    gpu = subprocess.run(
        ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()

    ros = subprocess.run(
        [
            "bash",
            "-lc",
            "source /opt/ros/humble/setup.bash >/dev/null "
            "&& dpkg-query -W -f='ros-humble-ros2cli=${Version}' ros-humble-ros2cli "
            "|| echo missing",
        ],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()

    isaac = subprocess.run(
        ["ls", "/isaac-sim"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()

    return {"gpu": gpu, "ros2": ros, "isaac_dir_present": "yes" if isaac else "no"}

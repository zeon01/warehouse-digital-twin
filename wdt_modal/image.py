"""Modal image definition for Isaac Sim 5.x + ROS2 Humble."""

from __future__ import annotations

import modal

ISAAC_SIM_IMAGE = "nvcr.io/nvidia/isaac-sim:5.0.0"

image = (
    # add_python="3.11" matches the Python that Isaac Sim 5.0 ships its Kit C
    # extensions for (carb._carb.cpython-311-*.so etc.). A 3.10 interpreter cannot
    # load those, so SimulationApp() would always fail.
    modal.Image.from_registry(ISAAC_SIM_IMAGE, add_python="3.11")
    # Clear the base image's ENTRYPOINT — Isaac Sim's default entrypoint auto-launches
    # Omniverse Kit (loading 100+ extensions), which takes >15min and blows out Modal's
    # startup_timeout. Our function code will explicitly invoke SimulationApp() when it
    # needs Kit (Task 9+).
    .entrypoint([])
    # Pre-seed timezone so the tzdata postinst (pulled transitively by ros-humble-desktop)
    # never opens its interactive geographic-area prompt and hangs the build.
    .env({"DEBIAN_FRONTEND": "noninteractive", "TZ": "Etc/UTC"})
    .run_commands(
        "ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime",
        "echo 'Etc/UTC' > /etc/timezone",
    )
    .apt_install(
        "curl",
        "gnupg2",
        "lsb-release",
        "software-properties-common",
        "ffmpeg",
        "xvfb",
        "tzdata",
    )
    # ROS2 Humble apt repo
    .run_commands(
        "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key "
        "-o /usr/share/keyrings/ros-archive-keyring.gpg",
        'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/'
        "ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu "
        '$(. /etc/os-release && echo $UBUNTU_CODENAME) main" '
        "| tee /etc/apt/sources.list.d/ros2.list",
    )
    # Downgrade libbrotli1 to the stock Ubuntu 22.04 version before installing ROS2.
    # The Isaac Sim base image ships a newer libbrotli1 from a sury.org PPA, which
    # conflicts with libbrotli-dev pinned by ros-humble-desktop.
    .run_commands(
        "apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6",
    )
    .apt_install(
        "ros-humble-desktop",
        "ros-humble-nav2-bringup",
        "ros-humble-moveit",
        "ros-humble-foxglove-bridge",
        "python3-colcon-common-extensions",
    )
    .pip_install(
        "modal>=0.64",
        "pydantic>=2.5",
        "numpy>=1.26",
        "scipy>=1.11",
        "networkx>=3.2",
        "opencv-python-headless>=4.9",
        "pyyaml>=6.0",
    )
    .env(
        {
            "ROS_DISTRO": "humble",
            "ROS_DOMAIN_ID": "42",
            "ISAAC_PATH": "/isaac-sim",
            "PYTHONUNBUFFERED": "1",
            # Required by the Isaac Sim 5.0 base image's entrypoint — without these,
            # Omniverse Kit refuses to start and any function on this image exits 1.
            "ACCEPT_EULA": "Y",
            "PRIVACY_CONSENT": "Y",
        }
    )
    # Mount the wdt_modal package into containers so functions can do
    # `from wdt_modal.image import image` and similar. copy=False (default) means
    # this is added at container startup, not baked into the image — fast iteration.
    .add_local_python_source("wdt_modal")
)

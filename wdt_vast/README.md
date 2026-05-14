# wdt_vast — Isaac Sim rendering on vast.ai

Modal's containers don't expose the Vulkan extensions Isaac Sim 5.0 needs
(verified across L4 / A10G / B200 — see `feedback-modal-build-monitoring`).
All rendering tasks live here instead, on rented vast.ai instances.

## Workflow (Pattern 3 — stop between sessions)

1. **First time:** rent an instance and let it pull the Isaac Sim image (~10 min).

   ```bash
   vastai create instance <offer-id> --image nvcr.io/nvidia/isaac-sim:5.0.0 --disk 80 --ssh
   ```

   Pick a CZ or EU host with a recent driver (580+); CN-cheap hosts are slow
   to pull from NGC.

2. **Resume from stopped state:**

   ```bash
   vastai start instance <instance-id>
   # Wait until `vastai show instance <id> --raw | jq .actual_status` returns "running"
   ```

3. **Run a render** — copy the script over and invoke via `python.sh`:

   ```bash
   scp -P <ssh-port> wdt_vast/render_smoke.py root@<ssh-host>:/tmp/
   ssh -p <ssh-port> root@<ssh-host> '/isaac-sim/python.sh /tmp/render_smoke.py /tmp/out_dir'
   scp -P <ssh-port> 'root@<ssh-host>:/tmp/out_dir/*' outputs/<task-name>/
   ```

3a. **For ROS2-using scripts** (bridge smoke, fleet, Nav2): instance needs
   a one-time ROS2 + Nav2 bootstrap, then every invocation sources ROS2:

   ```bash
   # One-time setup per fresh instance:
   ssh ... '
     apt-get install -y curl gnupg2 lsb-release
     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
       > /etc/apt/sources.list.d/ros2.list
     apt-get update
     # Downgrade libbrotli1 — the Isaac Sim image ships 1.1.0 from sury.org,
     # ros-humble-* deps pin libbrotli-dev which requires 1.0.9. Without this
     # the apt install of nav2 fails with "broken packages".
     apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6
     apt-get install -y \
       ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp ros-humble-nav2-bringup \
       python3-colcon-common-extensions build-essential
   '

   # Every invocation:
   ssh ... '
     source /opt/ros/humble/setup.bash
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     /isaac-sim/python.sh ...
   '
   ```

   Three known gotchas (memorized for future sessions):
   - The bridge extension `isaacsim.ros2.bridge` probes `ROS_DISTRO` at
     startup; the legacy `omni.isaac.ros2_bridge` alias is silently ignored
     in 5.0 (no error, no topics).
   - Isaac Sim's bundled FastDDS conflicts with system ROS2's FastDDS in the
     same process — segfault on first `world.step(render=True)`. Switching
     both to CycloneDDS via `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` fixes it.
   - `python3-colcon-common-extensions` alone isn't enough to build packages
     with `ament_cmake` — also need `build-essential` (g++) even for
     data-only packages because ament_cmake's CMake config runs a compiler
     check up front.

3b. **Build the warehouse_bringup ROS2 workspace** (one-time, ~1 second):

   ```bash
   tar czf /tmp/ros2_ws.tar.gz ros2_ws/
   scp -P <ssh-port> /tmp/ros2_ws.tar.gz root@<ssh-host>:/tmp/
   ssh ... '
     rm -rf /work && mkdir /work && cd /work && tar xzf /tmp/ros2_ws.tar.gz
     source /opt/ros/humble/setup.bash
     cd /work/ros2_ws && colcon build --symlink-install
   '
   # Verify: ros2 launch warehouse_bringup amr.launch.py --show-args
   ```

   The built install/ stays on the host disk across stop/resume, so the build
   only needs to re-run if the package sources change.

4. **Stop after each session** to drop billing to storage-only (~$0.025/hr):

   ```bash
   vastai stop instance <instance-id>
   ```

## Current instance

- Host: Romania (datacenter)
- GPU: RTX A5000 (24GB), driver 570.211.01 (above Isaac Sim 5.0's 535.129 min)
- Cost: $0.253/hr running, ~$0.025/hr stopped
- Image: `nvcr.io/nvidia/isaac-sim:5.0.0` (cached on host disk)
- Disk: 80GB
- Bootstrapped: ROS2 Humble + Nav2 + cyclonedds + libbrotli1 downgrade + colcon-built ros2_ws at /work/ros2_ws

## Why a subprocess wrapper isn't needed here

Unlike Modal, we don't need to layer `python.sh` over our own Python — when
we SSH into the vast.ai instance we're already root inside the Isaac Sim
container, so `python.sh` is the natural entry point and `LD_PRELOAD=libcarb.so`
is set correctly by it.

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

3a. **For ROS2-using scripts:** ROS2 Humble must be installed on the instance
   (`apt install ros-humble-ros-base`, one-time) AND sourced before python.sh:

   ```bash
   ssh ... 'source /opt/ros/humble/setup.bash && /isaac-sim/python.sh ...'
   ```

   The bridge extension `isaacsim.ros2.bridge` probes `ROS_DISTRO` at startup
   and refuses to initialize without it. The legacy `omni.isaac.ros2_bridge`
   alias is silently ignored in 5.0 — use the new name.

4. **Stop after each session** to drop billing to storage-only (~$0.025/hr):

   ```bash
   vastai stop instance <instance-id>
   ```

## Current instance

- Host: CZ
- GPU: RTX 3090 (24GB), driver 580.95.05
- Cost: $0.275/hr running, ~$0.025/hr stopped
- Image: `nvcr.io/nvidia/isaac-sim:5.0.0` (cached on host disk)
- Disk: 80GB

## Why a subprocess wrapper isn't needed here

Unlike Modal, we don't need to layer `python.sh` over our own Python — when
we SSH into the vast.ai instance we're already root inside the Isaac Sim
container, so `python.sh` is the natural entry point and `LD_PRELOAD=libcarb.so`
is set correctly by it.

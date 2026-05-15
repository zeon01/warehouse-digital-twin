#!/usr/bin/env bash
# Phase 2 vast.ai bootstrap — apt deps for MoveIt2, Franka description,
# and the cv_bridge needed by pick_cell_orchestrator.
#
# Prereqs (already done for Phase 1, see wdt_vast/README.md):
#   - Isaac Sim 5.0 image
#   - ROS2 Humble + Nav2 bringup
#   - rmw-cyclonedds-cpp
#   - libbrotli1=1.0.9-2build6 downgrade
#   - colcon-built /work/ros2_ws
#
# Run on a fresh vast.ai instance (or one that hasn't been bootstrapped
# for Phase 2 yet):
#
#     ssh root@<instance> bash < wdt_vast/bootstrap_phase2.sh
#
# Or interactively after SCP:
#
#     scp wdt_vast/bootstrap_phase2.sh root@<instance>:/tmp/
#     ssh root@<instance> 'bash /tmp/bootstrap_phase2.sh'

set -euo pipefail

echo "==> Phase 2 vast.ai bootstrap"
echo "    Adds MoveIt2 + Franka description + cv_bridge to a Phase 1 instance."

if ! grep -qE "ros-humble-ros-base" <(dpkg -l 2>/dev/null); then
  echo "ERROR: ROS2 Humble base not detected — run the Phase 1 bootstrap from"
  echo "       wdt_vast/README.md §3a before this script."
  exit 1
fi

# MoveIt2 (full stack) brings in move_group, moveit_py, kinematics plugins,
# the OMPL planner, and the Panda config used by wdt_manipulation_bringup.
# Franka description ships the canonical Panda URDF + meshes.
# cv_bridge converts sensor_msgs/Image <-> numpy/cv2 inside the orchestrator.
APT_PKGS=(
  ros-humble-moveit
  ros-humble-moveit-py
  ros-humble-moveit-resources-panda-moveit-config
  ros-humble-franka-description
  ros-humble-cv-bridge
  ros-humble-vision-opencv
  ros-humble-pointcloud-to-laserscan
)

echo "==> apt-get update"
apt-get update -y

echo "==> apt-get install: ${APT_PKGS[*]}"
apt-get install -y "${APT_PKGS[@]}"

echo "==> verifying ROS2 package visibility"
source /opt/ros/humble/setup.bash
for pkg in moveit_ros_move_group moveit_resources_panda_moveit_config \
           franka_description cv_bridge; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "ERROR: ros2 cannot find $pkg after apt install — check apt log"
    exit 1
  fi
done

echo "==> Phase 2 bootstrap complete"
echo "    Next: scp updated ros2_ws and rebuild with colcon (Phase 2 packages"
echo "    add wdt_carter_description, wdt_nav2_bringup, wdt_franka_description,"
echo "    wdt_manipulation_bringup)."

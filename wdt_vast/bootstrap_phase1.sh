#!/usr/bin/env bash
# Phase 1 vast.ai bootstrap — ROS2 Humble + Nav2 + cyclonedds + libbrotli
# downgrade. Run once per fresh instance, before bootstrap_phase2.sh.
#
# Was previously inlined in wdt_vast/README.md §3a; scripted here so the
# "resume on a new instance" flow is reproducible.
#
# Run on the vast.ai instance:
#   scp wdt_vast/bootstrap_phase1.sh vast-romania:/tmp/
#   ssh vast-romania 'bash /tmp/bootstrap_phase1.sh'

set -euo pipefail

echo "==> Phase 1 vast.ai bootstrap (ROS2 Humble + Nav2 + cyclonedds)"

if dpkg -l ros-humble-ros-base 2>/dev/null | grep -q "^ii"; then
  echo "    ROS2 Humble already installed; skipping"
  exit 0
fi

echo "==> apt deps for ROS2 repo bootstrap"
apt-get install -y --no-install-recommends curl gnupg2 lsb-release

echo "==> add ROS2 apt key + repo"
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
. /etc/os-release
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
  > /etc/apt/sources.list.d/ros2.list
apt-get update -y

# Isaac Sim 5.0 image ships libbrotli 1.1.0 from sury.org's PPA, but
# ros-humble-* depends on libbrotli-dev which needs 1.0.9. Force the
# downgrade to satisfy nav2's chain.
echo "==> downgrade libbrotli1 (Isaac Sim ships 1.1.0; ROS2 Humble needs 1.0.9)"
apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6

echo "==> install ROS2 Humble base + Nav2 + cyclonedds + colcon"
apt-get install -y --no-install-recommends \
  ros-humble-ros-base \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-nav2-bringup \
  python3-colcon-common-extensions \
  build-essential

echo "==> verify"
set +u
source /opt/ros/humble/setup.bash
set -u
for pkg in nav2_bringup rmw_cyclonedds_cpp; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "ERROR: ros2 cannot find $pkg after install"
    exit 1
  fi
done

echo "==> Phase 1 bootstrap complete"
echo "    Next: scp + bash wdt_vast/bootstrap_phase2.sh"

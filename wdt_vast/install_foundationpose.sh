#!/usr/bin/env bash
# Install FoundationPose from source on a vast.ai instance.
#
# Pulls weights from the Modal volume populated by
# wdt_modal/stage_foundationpose_weights.py, builds the mycpp pybind11
# extension, and pip-installs the package into Isaac Sim's python.sh
# environment.
#
# Prereqs (one-time per instance):
#   pip install --user modal       # gives modal CLI on PATH
#   modal token new                # interactive token paste
#
# Usage:
#   bash wdt_vast/install_foundationpose.sh
#
# Idempotent — re-runs skip clone and weight pulls if already present.

set -euo pipefail

FP_COMMIT=a1b694b83e633c2cb6115b9063d940a687759392
PREFIX=/opt/foundationpose
SRC="$PREFIX/src"
WEIGHTS="$SRC/weights"
ISAAC_PY=/isaac-sim/python.sh

REFINER_RUN=2023-10-28-18-33-37
SCORER_RUN=2024-01-11-20-02-45

echo "==> apt deps for mycpp build"
apt-get update -y
apt-get install -y --no-install-recommends \
  cmake ninja-build build-essential \
  libeigen3-dev libboost-all-dev \
  python3-pybind11

echo "==> cloning + pinning FoundationPose ($FP_COMMIT)"
mkdir -p "$PREFIX"
if [ ! -d "$SRC/.git" ]; then
  git clone https://github.com/NVlabs/FoundationPose "$SRC"
fi
git -C "$SRC" fetch --depth 1 origin "$FP_COMMIT"
git -C "$SRC" checkout "$FP_COMMIT"

echo "==> building mycpp (pybind11)"
cd "$SRC/mycpp"
rm -rf build && mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_ROOT_DIR="$($ISAAC_PY -c 'import sys; print(sys.prefix)')"
cmake --build . -j"$(nproc)"

echo "==> pip-installing PyTorch (cu124) + nvdiffrast + FoundationPose deps"
$ISAAC_PY -m pip install --upgrade pip
$ISAAC_PY -m pip install \
  torch==2.1.0 torchvision==0.16.0 \
  --index-url https://download.pytorch.org/whl/cu124
$ISAAC_PY -m pip install nvdiffrast
$ISAAC_PY -m pip install -r "$SRC/requirements.txt"
$ISAAC_PY -m pip install -e "$SRC"
$ISAAC_PY -m pip install trimesh  # used by manipulation.pose_estimation

echo "==> staging weights from Modal volume foundationpose-models"
mkdir -p "$WEIGHTS"
for run in "$REFINER_RUN" "$SCORER_RUN"; do
  if [ ! -f "$WEIGHTS/$run/model_best.pth" ]; then
    echo "    pulling $run"
    modal volume get foundationpose-models "$run" "$WEIGHTS/$run"
  else
    echo "    [skip] $run already present"
  fi
done

echo "==> verifying import"
$ISAAC_PY -c "
import sys
sys.path.insert(0, '$SRC')
from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
print('FoundationPose import OK')
"

echo "==> done. FoundationPose installed at $PREFIX"

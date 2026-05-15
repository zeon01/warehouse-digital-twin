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

echo "==> apt deps for mycpp build + CUDA toolkit for nvdiffrast"
apt-get update -y
apt-get install -y --no-install-recommends \
  cmake ninja-build build-essential \
  libeigen3-dev libboost-all-dev \
  python3-pybind11 \
  wget gnupg

# NVIDIA's apt repo for CUDA toolkit 12.4 (matches the cu124 PyTorch
# wheel ABI). Isaac Sim's image ships runtime libs but no nvcc, which
# nvdiffrast's setup.py needs to compile its CUDA kernels.
if [ ! -f /etc/apt/sources.list.d/cuda-ubuntu2204-x86_64.list ]; then
  wget -qO /tmp/cuda-keyring.deb \
    https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
  dpkg -i /tmp/cuda-keyring.deb
  apt-get update -y
fi
# cuda-toolkit-12-4 pulls ~3 GB — full nvcc + headers + libs. The
# minimal `cuda-nvcc-12-4 cuda-cudart-dev-12-4` set is smaller but
# breaks on missing thrust headers, so we take the full toolkit.
apt-get install -y --no-install-recommends cuda-toolkit-12-4
export PATH="/usr/local/cuda-12.4/bin:$PATH"
export CUDA_HOME=/usr/local/cuda-12.4

echo "==> cloning + pinning FoundationPose ($FP_COMMIT)"
mkdir -p "$PREFIX"
if [ ! -d "$SRC/.git" ]; then
  # If weights or other content were pre-staged (e.g. via scp from a
  # Modal-volume pull on a machine that has the modal CLI), move them
  # aside so `git clone` sees an empty dir, then restore after clone.
  if [ -d "$SRC" ] && [ -n "$(ls -A "$SRC" 2>/dev/null)" ]; then
    mv "$SRC" "${SRC}.staged"
  fi
  git clone https://github.com/NVlabs/FoundationPose "$SRC"
  if [ -d "${SRC}.staged" ]; then
    cp -r "${SRC}.staged"/. "$SRC/"
    rm -rf "${SRC}.staged"
  fi
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
# PyTorch's cu124 index starts at 2.4.0 — earlier versions only shipped
# cu118/cu121 wheels. FoundationPose's upstream doesn't pin a major
# version, so we take the earliest cu124-compatible pair. Isaac Sim's
# bundled torch (2.7.0+cu128) gets shadowed by this install; the boot
# smoke verified Isaac Sim still starts on 2.4.0+cu124.
$ISAAC_PY -m pip install --upgrade pip
if ! $ISAAC_PY -c "import torch; assert torch.__version__.startswith('2.4')" 2>/dev/null; then
  $ISAAC_PY -m pip install \
    torch==2.4.0 torchvision==0.19.0 \
    --index-url https://download.pytorch.org/whl/cu124
fi

# nvdiffrast isn't on PyPI — install from upstream git. The package's
# setup.py imports torch to compile CUDA extensions, so we must use
# --no-build-isolation so the build sees our installed torch. (Pre-
# install build deps that --no-build-isolation skips.)
$ISAAC_PY -m pip install setuptools wheel ninja
$ISAAC_PY -m pip install --no-build-isolation \
  git+https://github.com/NVlabs/nvdiffrast.git

$ISAAC_PY -m pip install -r "$SRC/requirements.txt"

# FoundationPose's repo isn't a pip-installable package (no setup.py,
# no pyproject.toml) — upstream expects you to add the source dir to
# PYTHONPATH and import modules directly (e.g. `from estimater import
# FoundationPose`). Drop a .pth file into Isaac Sim's site-packages so
# the dir is on sys.path automatically.
SITE_PACKAGES=$(
  $ISAAC_PY -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"
)
echo "$SRC" > "$SITE_PACKAGES/foundationpose.pth"
echo "    wrote $SITE_PACKAGES/foundationpose.pth -> $SRC"

$ISAAC_PY -m pip install trimesh  # used by manipulation.pose_estimation

# FP's requirements.txt pulled numpy 2.x which breaks Isaac Sim deps
# (numba, nvidia-srl-usd, osqp all pin numpy<2.0 or scipy<1.12).
# Pin both back to versions Isaac Sim ships with.
$ISAAC_PY -m pip install --force-reinstall \
  "numpy>=1.21.5,<2.0" "scipy<1.12" "lxml<5.0"

echo "==> staging weights from Modal volume foundationpose-models"
mkdir -p "$WEIGHTS"
for run in "$REFINER_RUN" "$SCORER_RUN"; do
  if [ -f "$WEIGHTS/$run/model_best.pth" ]; then
    echo "    [skip] $run already present"
  elif command -v modal >/dev/null 2>&1; then
    echo "    pulling $run via modal CLI"
    modal volume get foundationpose-models "$run" "$WEIGHTS/$run"
  else
    echo "    ERROR: $WEIGHTS/$run missing and modal CLI not installed."
    echo "    Either install modal+auth on this host, or pre-stage weights via:"
    echo "        # on a host with modal auth:"
    echo "        modal volume get foundationpose-models $run /tmp/$run"
    echo "        scp -r /tmp/$run THIS_HOST:$WEIGHTS/$run"
    exit 1
  fi
done

echo "==> verifying import"
# The .pth file from earlier should already have $SRC on sys.path, but
# fallback to an explicit insert in case Isaac Sim's python setup ignores
# the .pth (some embedded interpreters do).
$ISAAC_PY -c "
import sys
sys.path.insert(0, '$SRC')
from estimater import FoundationPose
from learning.training.predict_score import ScorePredictor
from learning.training.predict_pose_refine import PoseRefinePredictor
print('FoundationPose import OK')
"

echo "==> done. FoundationPose installed at $PREFIX"

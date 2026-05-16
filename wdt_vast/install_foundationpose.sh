#!/usr/bin/env bash
# Install FoundationPose from source on a vast.ai instance.
#
# Target Python is configurable via the FP_TARGET_PY env var. The default
# is /usr/bin/python3 (system Python 3.10) because pick_cell_orchestrator
# needs both rclpy (ROS2 Humble — only py3.10 binding) and FoundationPose
# in one process. Override to /isaac-sim/python.sh for a Phase 3 setup
# that runs FP from Isaac Sim's bundled Python.
#
# Prereqs (one-time per instance):
#   pip install --user modal       # gives modal CLI on PATH (optional —
#                                    you can scp wheels/weights instead)
#   modal token new                # interactive token paste
#
# Usage:
#   bash wdt_vast/install_foundationpose.sh
#   FP_TARGET_PY=/isaac-sim/python.sh bash wdt_vast/install_foundationpose.sh
#
# Idempotent — re-runs skip clone, weight pulls, and torch install if
# already present.

set -euo pipefail

FP_COMMIT=a1b694b83e633c2cb6115b9063d940a687759392
PREFIX=/opt/foundationpose
SRC="$PREFIX/src"
WEIGHTS="$SRC/weights"
TARGET_PY="${FP_TARGET_PY:-/usr/bin/python3}"

REFINER_RUN=2023-10-28-18-33-37
SCORER_RUN=2024-01-11-20-02-45

# Where the cp310 pytorch3d wheel lives (cp311 lives in wheels/).
TARGET_MAJOR_MINOR=$("$TARGET_PY" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
if [ "$TARGET_MAJOR_MINOR" = "3.10" ]; then
  PT3D_VOL_SUBDIR=wheels-py310
  PT3D_LOCAL_SUBDIR=wheels-py310
elif [ "$TARGET_MAJOR_MINOR" = "3.11" ]; then
  PT3D_VOL_SUBDIR=wheels
  PT3D_LOCAL_SUBDIR=wheels
else
  echo "ERROR: target python is $TARGET_MAJOR_MINOR but only 3.10/3.11 are supported"
  exit 1
fi

echo "==> target python: $TARGET_PY ($TARGET_MAJOR_MINOR)"
echo "==> using pytorch3d wheels from /weights/$PT3D_VOL_SUBDIR"

echo "==> apt deps for mycpp build + CUDA toolkit for nvdiffrast"
apt-get update -y
apt-get install -y --no-install-recommends \
  cmake ninja-build build-essential \
  libeigen3-dev libboost-all-dev \
  python3-pybind11 \
  python3-pip python3-dev \
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
apt-get install -y --no-install-recommends cuda-toolkit-12-4
export PATH="/usr/local/cuda-12.4/bin:$PATH"
export CUDA_HOME=/usr/local/cuda-12.4

echo "==> cloning + pinning FoundationPose ($FP_COMMIT)"
mkdir -p "$PREFIX"
if [ ! -d "$SRC/.git" ]; then
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

echo "==> building mycpp (pybind11) for $TARGET_PY"
cd "$SRC/mycpp"
rm -rf build && mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_ROOT_DIR="$($TARGET_PY -c 'import sys; print(sys.prefix)')"
cmake --build . -j"$(nproc)"

echo "==> pip-installing PyTorch (cu124) + nvdiffrast + FoundationPose deps"
$TARGET_PY -m pip install --upgrade pip
if ! $TARGET_PY -c "import torch; assert torch.__version__.startswith('2.4')" 2>/dev/null; then
  $TARGET_PY -m pip install \
    torch==2.4.0 torchvision==0.19.0 \
    --index-url https://download.pytorch.org/whl/cu124
fi

# nvdiffrast — clone locally then pip install from path. With
# --no-build-isolation pip uses the OUTSIDE setuptools; Ubuntu 22.04
# ships setuptools 59.6 which ignores the [project] table in
# pyproject.toml and silently produces an "UNKNOWN-0.0.0" package
# (no metadata, no installable files). nvdiffrast's pyproject.toml
# requires setuptools>=64 — upgrade BEFORE the install.
$TARGET_PY -m pip install --upgrade "setuptools>=68" wheel ninja
$TARGET_PY -m pip uninstall -y UNKNOWN 2>/dev/null || true
if [ ! -d "$PREFIX/nvdiffrast" ]; then
  git clone --depth 1 https://github.com/NVlabs/nvdiffrast.git "$PREFIX/nvdiffrast"
fi
$TARGET_PY -m pip install --no-build-isolation "$PREFIX/nvdiffrast"
# Verify nvdiffrast actually installed (UNKNOWN trap caught us twice)
$TARGET_PY -m pip show nvdiffrast >/dev/null || {
  echo "ERROR: nvdiffrast did not install correctly (still showing UNKNOWN?)"
  $TARGET_PY -m pip list 2>&1 | grep -iE "nvdiff|UNKNOWN"
  exit 1
}

# pytorch3d wheel pulled from Modal volume (or pre-staged via scp).
PT3D_WHEEL_DIR="$PREFIX/$PT3D_LOCAL_SUBDIR"
mkdir -p "$PT3D_WHEEL_DIR"
if ! ls "$PT3D_WHEEL_DIR"/pytorch3d-*.whl >/dev/null 2>&1; then
  if command -v modal >/dev/null 2>&1; then
    modal volume get foundationpose-models "$PT3D_VOL_SUBDIR" "$PT3D_WHEEL_DIR/.."
  else
    echo "    ERROR: pytorch3d wheel missing at $PT3D_WHEEL_DIR/ and modal CLI not installed."
    echo "    On a host with modal auth:"
    echo "        modal volume get foundationpose-models $PT3D_VOL_SUBDIR /tmp/fp_$PT3D_VOL_SUBDIR"
    echo "        scp /tmp/fp_$PT3D_VOL_SUBDIR/pytorch3d-*.whl THIS_HOST:$PT3D_WHEEL_DIR/"
    exit 1
  fi
fi
$TARGET_PY -m pip install "$PT3D_WHEEL_DIR"/pytorch3d-*.whl

# Ubuntu 22.04's system Python has blinker 1.4 installed via apt+distutils
# (no metadata manifest). FP's requirements.txt transitively wants a
# newer blinker (Flask 3.x dep) and pip refuses to uninstall the
# distutils-managed one. --ignore-installed forces pip to install
# alongside, which is fine.
$TARGET_PY -m pip install --ignore-installed --no-deps "blinker>=1.6"

$TARGET_PY -m pip install -r "$SRC/requirements.txt"

# FP isn't pip-installable; drop a .pth file in $TARGET_PY's site-packages.
SITE_PACKAGES=$(
  $TARGET_PY -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"
)
echo "$SRC" > "$SITE_PACKAGES/foundationpose.pth"
echo "    wrote $SITE_PACKAGES/foundationpose.pth -> $SRC"

$TARGET_PY -m pip install trimesh

# FP's requirements pulled numpy 2.x. Pin back to Isaac-Sim-compatible
# versions. (Only matters for the py3.11 target — py3.10 system python
# is independent of Isaac Sim's bundled deps, but pinning here keeps
# both targets consistent.)
$TARGET_PY -m pip install --force-reinstall \
  "numpy>=1.21.5,<2.0" "scipy<1.12" "lxml<5.0"

# Older datacenter CPUs (e.g. Ivy Bridge — Xeon E5-2697 v2 on vast.ai
# Quebec mach 52360) lack AVX2/FMA, which the prebuilt kornia_rs Rust
# wheel requires. `import kornia` then SIGILLs silently and FP's
# estimater chain fails to import. Downgrade kornia to 0.7.0 (pre-Rust
# era, pure-Python) and uninstall kornia_rs entirely. Skip on hosts
# that DO support AVX2 — kornia 0.8.x is fine there — but the pin
# harms nothing on a modern CPU.
echo "==> pinning kornia 0.7.0 (Ivy-Bridge AVX2-free hosts SIGILL on kornia_rs)"
$TARGET_PY -m pip install --upgrade "kornia==0.7.0"
$TARGET_PY -m pip uninstall -y kornia_rs 2>/dev/null || true

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
    echo "        modal volume get foundationpose-models $run /tmp/$run"
    echo "        scp -r /tmp/$run THIS_HOST:$WEIGHTS/$run"
    exit 1
  fi
done

echo "==> verifying import via $TARGET_PY"
$TARGET_PY -c "
import sys
sys.path.insert(0, '$SRC')
from estimater import FoundationPose
from learning.training.predict_score import ScorePredictor
from learning.training.predict_pose_refine import PoseRefinePredictor
print('FoundationPose import OK')
"

echo "==> done. FoundationPose installed at $PREFIX for $TARGET_PY"

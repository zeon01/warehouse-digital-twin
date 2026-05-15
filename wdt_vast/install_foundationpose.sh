#!/usr/bin/env bash
# Pull FoundationPose wheels + weights from Modal volume to vast.ai, then
# install into the Isaac Sim Python environment.
#
# Usage (on vast.ai):
#     bash wdt_vast/install_foundationpose.sh
#
# Prereqs (one-time per instance):
#     pip install --user modal
#     modal token new       # interactive — paste from modal.com/settings/tokens
#
# Output:
#     /opt/foundationpose/wheels/        — extracted pip wheels
#     /opt/foundationpose/checkpoints/   — model weights (~2 GB)
#     /opt/foundationpose/src/           — pinned FoundationPose source
#     foundationpose installed in Isaac Sim's python.sh environment

set -euo pipefail

FP_COMMIT=4517f47b5e7e4a7e0d3b9e5d8f8c9e7b8a9d8c5e
FP_COMMIT_SHORT=${FP_COMMIT:0:8}
WHEELS_TGZ_NAME="foundationpose-wheels-${FP_COMMIT_SHORT}.tar.gz"

INSTALL_PREFIX=/opt/foundationpose
WHEELS_DIR="$INSTALL_PREFIX/wheels"
CHECKPOINTS_DIR="$INSTALL_PREFIX/checkpoints"
SRC_DIR="$INSTALL_PREFIX/src"

mkdir -p "$WHEELS_DIR" "$CHECKPOINTS_DIR"

WHEELS_TGZ=/tmp/$WHEELS_TGZ_NAME
if [ ! -f "$WHEELS_TGZ" ]; then
  echo "==> pulling wheels tarball from Modal"
  modal volume get foundationpose-models "$WHEELS_TGZ_NAME" "$WHEELS_TGZ"
fi
echo "==> extracting wheels"
tar -xzf "$WHEELS_TGZ" -C "$WHEELS_DIR"

if [ ! -f "$CHECKPOINTS_DIR/model_best.pth" ]; then
  echo "==> pulling checkpoints from Modal (~2 GB, may take ~5-10 min)"
  modal volume get foundationpose-models checkpoints "$CHECKPOINTS_DIR/"
fi

echo "==> cloning + pinning FoundationPose source"
if [ ! -d "$SRC_DIR" ]; then
  git clone https://github.com/NVlabs/FoundationPose "$SRC_DIR"
fi
git -C "$SRC_DIR" fetch --depth 1 origin "$FP_COMMIT"
git -C "$SRC_DIR" checkout "$FP_COMMIT"

echo "==> pip-installing wheels + source into Isaac Sim's python.sh"
/isaac-sim/python.sh -m pip install "$WHEELS_DIR"/*.whl
/isaac-sim/python.sh -m pip install -e "$SRC_DIR"

echo "==> verifying import"
/isaac-sim/python.sh -c "import foundationpose; print('foundationpose at', foundationpose.__file__)"

echo "==> done. FoundationPose installed at $INSTALL_PREFIX"

# ----------------------------------------------------------------------
# FALLBACK (uncomment if Modal-built wheels fail to load due to PyTorch
# ABI mismatch). Builds the CUDA extensions directly on vast.ai using
# the runtime PyTorch + CUDA. Slow (~25 min) but uses the exact stack.
# ----------------------------------------------------------------------
# echo "==> fallback: building CUDA extensions on vast.ai"
# cd "$SRC_DIR/bundled/nvdiffrast" && /isaac-sim/python.sh -m pip install -e .
# cd "$SRC_DIR/mycpp/mycuda" && /isaac-sim/python.sh -m pip install -e .

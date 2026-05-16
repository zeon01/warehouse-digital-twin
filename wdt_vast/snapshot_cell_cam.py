"""One-shot subscribe to /cell/cam/{rgb,depth} and save PNGs to /tmp.

Used to visually verify the cell camera framing — what FoundationPose sees.
The depth image is also colormapped to PNG (since native depth is 32FC1 in
meters, not directly renderable as 8-bit).

Usage on the vast.ai instance (after Isaac Sim is running):
    /usr/bin/python3 wdt_vast/snapshot_cell_cam.py /tmp/cell_snap

Writes:
    <out_dir>/cell_rgb.png       — raw RGB
    <out_dir>/cell_depth.png     — 8-bit colormapped depth
    <out_dir>/cell_depth.npy     — raw float32 depth in meters
    <out_dir>/meta.json          — camera info + min/max depth + frame_id
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class Snapshot(Node):
    def __init__(self) -> None:
        super().__init__("snapshot_cell_cam")
        self._bridge = CvBridge()
        self.rgb: np.ndarray | None = None
        self.depth: np.ndarray | None = None
        self.info: dict | None = None
        self.frame_id: str | None = None
        self.create_subscription(Image, "/cell/cam/rgb", self._on_rgb, 1)
        self.create_subscription(Image, "/cell/cam/depth", self._on_depth, 1)
        self.create_subscription(CameraInfo, "/cell/cam/info", self._on_info, 1)

    def _on_rgb(self, msg: Image) -> None:
        if self.rgb is None:
            self.rgb = self._bridge.imgmsg_to_cv2(msg, "rgb8")
            self.frame_id = msg.header.frame_id

    def _on_depth(self, msg: Image) -> None:
        if self.depth is None:
            self.depth = self._bridge.imgmsg_to_cv2(msg, "32FC1")

    def _on_info(self, msg: CameraInfo) -> None:
        if self.info is None:
            self.info = {
                "frame_id": msg.header.frame_id,
                "height": int(msg.height),
                "width": int(msg.width),
                "K": list(msg.k),
                "D": list(msg.d),
                "distortion_model": msg.distortion_model,
            }

    def ready(self) -> bool:
        return self.rgb is not None and self.depth is not None and self.info is not None


def _colormap_depth(depth: np.ndarray) -> np.ndarray:
    """8-bit grayscale visualization of a 32FC1 depth (meters).
    Ignores 0/NaN; min-max normalizes the valid region.
    """
    d = depth.astype(np.float32)
    mask = np.isfinite(d) & (d > 0)
    if not mask.any():
        return np.zeros(d.shape, dtype=np.uint8)
    dmin = float(d[mask].min())
    dmax = float(d[mask].max())
    if dmax - dmin < 1e-6:
        return np.zeros(d.shape, dtype=np.uint8)
    n = np.where(mask, (d - dmin) / (dmax - dmin), 0.0)
    return (n * 255.0).astype(np.uint8)


def main(out_dir_arg: str) -> int:
    out_dir = Path(out_dir_arg)
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Snapshot()
    t_deadline = time.time() + 10.0
    while time.time() < t_deadline and not node.ready():
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.ready():
        print(
            f"snapshot timed out: rgb={node.rgb is not None} "
            f"depth={node.depth is not None} info={node.info is not None}",
            file=sys.stderr,
        )
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        return 2

    rgb = node.rgb
    depth = node.depth
    # Save raw RGB as PNG via PIL (no cv2 dep here).
    from PIL import Image as PILImage

    PILImage.fromarray(rgb, mode="RGB").save(out_dir / "cell_rgb.png")
    PILImage.fromarray(_colormap_depth(depth), mode="L").save(out_dir / "cell_depth.png")
    np.save(out_dir / "cell_depth.npy", depth)

    valid = depth[np.isfinite(depth) & (depth > 0)]
    meta = {
        "rgb_shape": list(rgb.shape),
        "depth_shape": list(depth.shape),
        "depth_min_m": float(valid.min()) if valid.size else None,
        "depth_max_m": float(valid.max()) if valid.size else None,
        "depth_median_m": float(np.median(valid)) if valid.size else None,
        "depth_frame_id": node.frame_id,
        "camera_info": node.info,
    }
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=2))
    print(f"saved cell_rgb.png cell_depth.png cell_depth.npy meta.json under {out_dir}")
    print(
        f"depth: min={meta['depth_min_m']:.3f} median={meta['depth_median_m']:.3f} "
        f"max={meta['depth_max_m']:.3f} (m)"
    )

    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: snapshot_cell_cam.py <out_dir>", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))

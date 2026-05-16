"""Fast harness for the M5 pick chain — runs on the vast.ai instance.

Skips Isaac Sim entirely. Launches move_group +
franka_ready_joint_states + pick_cell_orchestrator (gt mode), publishes
a synthetic /world/cube_pose at a known Franka-reachable point + a
trivial /cell/cam/info (so the cam-state cache is populated even
though gt-mode ignores it), publishes /cell/start_pick, asserts
/cell/pick_result arrives with success=true within 2 s.

Iteration time: ~30 s (most of it is move_group boot).

Invocation on the instance:
    source /opt/ros/humble/setup.bash
    source /work/ros2_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    /usr/bin/python3 /work/tests/integration/test_pick_chain_fast.py
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

PANDA_BASE_WORLD = (16.0, 15.0, 1.0)
# Reachable cube center in world coords for panda_link0 (0.40, 0, -0.25).
CUBE_WORLD = (16.40, 15.0, 0.75)
TIMEOUT_S = 30.0


class Harness(Node):
    def __init__(self) -> None:
        super().__init__("pick_chain_fast_harness")
        self._info_pub = self.create_publisher(CameraInfo, "/cell/cam/info", 1)
        self._cube_pub = self.create_publisher(PoseStamped, "/world/cube_pose", 1)
        self._start_pub = self.create_publisher(String, "/cell/start_pick", 1)
        self._result_sub = self.create_subscription(
            String, "/cell/pick_result", self._on_result, 10
        )
        self.result: dict | None = None

    def _on_result(self, msg: String) -> None:
        try:
            self.result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.result = {"error": "bad_json", "raw": msg.data}

    def publish_info_and_cube(self) -> None:
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = "cell_cam_optical"
        info.height = 480
        info.width = 640
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [600.0, 0.0, 320.0, 0.0, 0.0, 600.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)

        cube = PoseStamped()
        cube.header.stamp = self.get_clock().now().to_msg()
        cube.header.frame_id = "world"
        cube.pose.position.x = CUBE_WORLD[0]
        cube.pose.position.y = CUBE_WORLD[1]
        cube.pose.position.z = CUBE_WORLD[2]
        cube.pose.orientation.w = 1.0
        self._cube_pub.publish(cube)

    def fire_start_pick(self, order_id: str) -> None:
        msg = String()
        msg.data = order_id
        self._start_pub.publish(msg)


def _start_dep(name: str, cmd: str, env: dict) -> subprocess.Popen:
    """Launch a child process with full ROS2 sourcing baked in."""
    return subprocess.Popen(
        [
            "bash",
            "-lc",
            f"source /opt/ros/humble/setup.bash && "
            f"source /work/ros2_ws/install/setup.bash && "
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"{cmd}",
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def main() -> int:
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    # /tmp on the PYTHONPATH for non-colcon repo packages (manipulation).
    pp_parts = [p for p in env.get("PYTHONPATH", "").split(":") if p]
    if "/tmp" not in pp_parts:
        pp_parts.append("/tmp")
    env["PYTHONPATH"] = ":".join(pp_parts)

    deps = []
    try:
        deps.append(
            _start_dep(
                "move_group",
                "ros2 launch wdt_manipulation_bringup move_group.launch.py",
                env,
            )
        )
        deps.append(
            _start_dep("jsp", "/usr/bin/python3 /work/wdt_vast/franka_ready_joint_states.py", env)
        )
        deps.append(
            _start_dep(
                "panda_link0_tf",
                f"ros2 run tf2_ros static_transform_publisher --x {PANDA_BASE_WORLD[0]} "
                f"--y {PANDA_BASE_WORLD[1]} --z {PANDA_BASE_WORLD[2]} "
                f"--qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 "
                f"--frame-id world --child-frame-id panda_link0",
                env,
            )
        )
        deps.append(
            _start_dep(
                "pick_orch",
                "ros2 run wdt_manipulation_bringup pick_cell_orchestrator "
                "--ros-args -p pose_source:=gt -p cad_path:=/tmp/m5_smoke_box.obj",
                env,
            )
        )

        # Give move_group + RSP ~20 s to come up.
        time.sleep(20.0)

        rclpy.init()
        node = Harness()

        deadline = time.time() + TIMEOUT_S
        order_id = "fast_harness_o1"
        published_start = False
        while time.time() < deadline and node.result is None:
            node.publish_info_and_cube()
            if not published_start and time.time() > deadline - TIMEOUT_S + 21.0:
                node.fire_start_pick(order_id)
                published_start = True
            rclpy.spin_once(node, timeout_sec=0.1)

        ok = node.result is not None and node.result.get("success") is True
        print(f"==> result: {node.result}")
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        return 0 if ok else 2
    finally:
        for p in deps:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())

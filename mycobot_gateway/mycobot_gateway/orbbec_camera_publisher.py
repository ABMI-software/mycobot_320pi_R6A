#!/usr/bin/env python3
"""ROS2 publisher for Orbbec Astra RGB via OpenNI shared-memory grabber.

Uses teleop.orbbec_capture.open_orbbec() and publishes sensor_msgs/Image.
Default output topic matches the real pick-and-place launch: /camera/color/image_raw.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# Import helper from repository teleop module.
_REPO_ROOT = Path(__file__).resolve().parents[2]
_TELEOP_DIR = _REPO_ROOT / "teleop"
if _TELEOP_DIR.is_dir() and str(_TELEOP_DIR) not in sys.path:
    sys.path.insert(0, str(_TELEOP_DIR))

from orbbec_capture import open_orbbec  # type: ignore  # noqa: E402


class OrbbecCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("orbbec_camera_publisher")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        topic = str(self.get_parameter("image_topic").value)
        fps = float(self.get_parameter("fps").value)
        self._frame_id = str(self.get_parameter("frame_id").value)

        self._pub = self.create_publisher(Image, topic, 10)
        self._bridge = CvBridge()

        # Spawn oni_grabber automatically if needed.
        self._cap = open_orbbec(auto_spawn=True, open_timeout=10.0)
        if not self._cap.isOpened():
            raise RuntimeError("Failed to open Orbbec stream")

        self.create_timer(max(0.01, 1.0 / max(1.0, fps)), self._tick)

        self.get_logger().info(
            f"Orbbec RGB publisher ready | topic={topic} fps={fps:.1f}"
        )

    def _tick(self) -> None:
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._cap.release()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    # Prevent Conda user-site leakage in mixed Python environments.
    os.environ.pop("PYTHONHOME", None)
    rclpy.init(args=args)
    node = OrbbecCameraPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Republish Zenoh JPEG frames into ROS 2 sensor_msgs/CompressedImage.

Example:
  $ ros_exec python3 ros2_republish_camera.py \
      --in-key rt/vision/cam0 --out-topic /camera/cam0/image/compressed

Given/When/Then:
- Given a Zenoh key with JPEG bytes, when running, then publishes CompressedImage.
"""
from __future__ import annotations

import argparse
import dataclasses
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


@dataclasses.dataclass
class CameraRepubConfig:
    in_key: str = "rt/vision/cam0"
    out_topic: str = "/camera/cam0/image/compressed"


class CameraRepubNode(Node):
    def __init__(self, cfg: CameraRepubConfig) -> None:
        super().__init__("camera_repub")
        self._cfg = cfg
        self._pub = self.create_publisher(CompressedImage, cfg.out_topic, 10)
        self._start_zenoh()

    def _start_zenoh(self) -> None:
        import zenoh  # type: ignore

        self._z_session = zenoh.open(zenoh.Config())
        self._z_sub = self._z_session.declare_subscriber(self._cfg.in_key, self._on_z_msg)
        self.get_logger().info(f"Subscribing Zenoh: {self._cfg.in_key}")

    def _on_z_msg(self, sample) -> None:  # noqa: ANN001
        try:
            payload: bytes = sample.payload.to_bytes()
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = payload
            self._pub.publish(msg)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"camera repub error: {exc}")


def parse_args(argv: Optional[list[str]] = None) -> CameraRepubConfig:
    p = argparse.ArgumentParser()
    p.add_argument("--in-key", default="rt/vision/cam0")
    p.add_argument("--out-topic", default="/camera/cam0/image/compressed")
    ns = p.parse_args(argv)
    return CameraRepubConfig(in_key=ns.in_key, out_topic=ns.out_topic)


def main() -> None:  # pragma: no cover
    cfg = parse_args()
    rclpy.init()
    node = CameraRepubNode(cfg)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


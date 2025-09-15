#!/usr/bin/env python3
"""Republish Zenoh PCM chunks into ROS 2 as UInt8MultiArray.

Subscribes to a Zenoh key carrying S16_LE PCM chunks and republishes on a
ROS 2 topic. Keeps dependencies minimal (requires rclpy and zenoh only).

Example:
  $ ros_exec python3 ros2_republish_audio.py \
      --in-key rt/audio/ear0 --out-topic /audio/ear0/raw --rate 16000 --channels 1

Given/When/Then:
- Given a Zenoh key with PCM bytes, when running, then publishes ROS 2 arrays.
"""
from __future__ import annotations

import argparse
import dataclasses
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


@dataclasses.dataclass
class AudioRepubConfig:
    in_key: str = "rt/audio/ear0"
    out_topic: str = "/audio/ear0/raw"
    rate: int = 16000
    channels: int = 1


class AudioRepubNode(Node):
    def __init__(self, cfg: AudioRepubConfig) -> None:
        super().__init__("audio_repub")
        self._cfg = cfg
        self._pub = self.create_publisher(UInt8MultiArray, cfg.out_topic, 10)
        self._start_zenoh()

    def _start_zenoh(self) -> None:
        import zenoh  # type: ignore

        self._z_session = zenoh.open(zenoh.Config())
        self._z_sub = self._z_session.declare_subscriber(self._cfg.in_key, self._on_z_msg)
        self.get_logger().info(f"Subscribing Zenoh: {self._cfg.in_key}")

    def _on_z_msg(self, sample) -> None:  # noqa: ANN001 - zenoh callback signature
        try:
            payload: bytes = sample.payload.to_bytes()
            msg = UInt8MultiArray(data=list(payload))
            self._pub.publish(msg)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f"audio repub error: {exc}")


def parse_args(argv: Optional[list[str]] = None) -> AudioRepubConfig:
    p = argparse.ArgumentParser()
    p.add_argument("--in-key", default="rt/audio/ear0")
    p.add_argument("--out-topic", default="/audio/ear0/raw")
    p.add_argument("--rate", type=int, default=16000)
    p.add_argument("--channels", type=int, default=1)
    ns = p.parse_args(argv)
    return AudioRepubConfig(in_key=ns.in_key, out_topic=ns.out_topic, rate=ns.rate, channels=ns.channels)


def main() -> None:  # pragma: no cover
    cfg = parse_args()
    rclpy.init()
    node = AudioRepubNode(cfg)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


#!/usr/bin/env python3
"""Subscribe to zenoh GNSS key and publish ROS2 NavSatFix messages.

Expects `zenoh` and `rclpy` to be available in the environment. The zenoh
key and output topic can be set via environment variables:

- `GNSS_IN_KEY` (default `rt/gnss/ear0`)
- `GNSS_OUT_TOPIC` (default `/gps/fix`)

"""
from __future__ import annotations

import os
import json
import sys

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix
except Exception as exc:  # pragma: no cover - runtime dependency
    print(f"[ros2_republish_gnss] missing rclpy or sensor_msgs: {exc}", file=sys.stderr)
    raise

try:
    import zenoh
except Exception as exc:  # pragma: no cover - runtime dependency
    print(f"[ros2_republish_gnss] missing zenoh: {exc}", file=sys.stderr)
    raise


class ZenohGnssBridge(Node):
    def __init__(self, in_key: str, out_topic: str):
        super().__init__('zenoh_gnss_bridge')
        self.pub = self.create_publisher(NavSatFix, out_topic, 10)
        self.session = zenoh.open()
        self.key = in_key
        self.session.subscribe(self.key, self.zenoh_cb)
        self.get_logger().info(f'Subscribed to zenoh key {self.key}')

    def zenoh_cb(self, sample):
        payload = sample.payload
        try:
            text = payload.decode() if isinstance(payload, (bytes, bytearray)) else str(payload)
            # Try to parse NMEA GGA or RMC if JSON encoded; otherwise send raw data in status
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'
            # Best-effort: parse simple comma-separated NMEA for lat/lon in GGA
            if text.startswith('$'):
                parts = text.strip().split(',')
                if parts[0].endswith('GGA') and len(parts) > 5 and parts[2] and parts[4]:
                    # lat format: ddmm.mmmm, N/S
                    lat_raw = parts[2]
                    lat_dir = parts[3]
                    lon_raw = parts[4]
                    lon_dir = parts[5]
                    try:
                        lat = float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
                        if lat_dir == 'S':
                            lat = -lat
                        lon = float(lon_raw[:3]) + float(lon_raw[3:]) / 60.0
                        if lon_dir == 'W':
                            lon = -lon
                        msg.latitude = lat
                        msg.longitude = lon
                        msg.status.status = NavSatFix.STATUS_FIX
                    except Exception:
                        pass
            else:
                # Try JSON containing lat/lon
                try:
                    j = json.loads(text)
                    if 'lat' in j and 'lon' in j:
                        msg.latitude = float(j['lat'])
                        msg.longitude = float(j['lon'])
                except Exception:
                    pass
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error handling GNSS sample: {e}')


def main(args=None):
    in_key = os.environ.get('GNSS_IN_KEY', 'rt/gnss/ear0')
    out_topic = os.environ.get('GNSS_OUT_TOPIC', '/gps/fix')
    rclpy.init(args=args)
    node = ZenohGnssBridge(in_key, out_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

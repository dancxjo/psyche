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
    from sensor_msgs.msg import NavSatFix, NavSatStatus
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

    def _nmea_to_decimal(self, raw: str, direction: str) -> float | None:
        """Convert NMEA lat/lon ddmm.mmmm and direction to decimal degrees.

        Returns None on parse error.
        """
        try:
            if not raw:
                return None
            # Latitude ddmm.mmmm (2 digits deg), Longitude dddmm.mmmm (3 digits deg)
            if '.' not in raw:
                return None
            parts = raw.split('.')
            head = parts[0]
            if len(head) <= 4:  # minimal sanity
                deg_len = 2
            else:
                # heuristically: lat has 4 digits before dot for ddmm, lon has 5
                deg_len = 2 if len(head) in (4, 3) else 3
            deg = float(raw[:deg_len])
            minutes = float(raw[deg_len:])
            dec = deg + minutes / 60.0
            if direction in ('S', 'W'):
                dec = -dec
            return dec
        except Exception:
            return None

    def zenoh_cb(self, sample):
        payload = sample.payload
        try:
            text = payload.decode() if isinstance(payload, (bytes, bytearray)) else str(payload)
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'

            status = NavSatStatus()
            status.status = NavSatStatus.STATUS_NO_FIX
            status.service = NavSatStatus.SERVICE_GPS if hasattr(NavSatStatus, 'SERVICE_GPS') else 1

            # Helper to set covariance if we have hdop
            def set_cov(hdop: float | None, alt_acc: float | None = None):
                if hdop is None:
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    return
                # approximate horizontal accuracy: hdop * 5 meters (rough heuristic)
                hacc = hdop * 5.0
                vacc = (alt_acc if alt_acc is not None else 10.0)
                cov = [hacc * hacc, 0.0, 0.0, 0.0, hacc * hacc, 0.0, 0.0, 0.0, vacc * vacc]
                msg.position_covariance = cov
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            if text.startswith('$'):
                parts = text.strip().split(',')
                # GGA: Global Positioning System Fix Data
                if parts[0].endswith('GGA') and len(parts) > 9:
                    lat = self._nmea_to_decimal(parts[2], parts[3]) if parts[2] and parts[3] else None
                    lon = self._nmea_to_decimal(parts[4], parts[5]) if parts[4] and parts[5] else None
                    try:
                        fix_quality = int(parts[6]) if parts[6] else 0
                    except Exception:
                        fix_quality = 0
                    hdop = None
                    try:
                        hdop = float(parts[8]) if parts[8] else None
                    except Exception:
                        hdop = None
                    alt = None
                    try:
                        alt = float(parts[9]) if parts[9] else None
                    except Exception:
                        alt = None

                    if lat is not None and lon is not None:
                        msg.latitude = lat
                        msg.longitude = lon
                        msg.altitude = alt if alt is not None else 0.0
                        status.status = NavSatStatus.STATUS_FIX if fix_quality > 0 else NavSatStatus.STATUS_NO_FIX
                        set_cov(hdop, None)

                # RMC: Recommended Minimum Specific GNSS Data (contains status)
                elif parts[0].endswith('RMC') and len(parts) > 3:
                    rmc_status = parts[2]
                    lat = self._nmea_to_decimal(parts[3], parts[4]) if parts[3] and parts[4] else None
                    lon = self._nmea_to_decimal(parts[5], parts[6]) if parts[5] and parts[6] else None
                    if lat is not None and lon is not None:
                        msg.latitude = lat
                        msg.longitude = lon
                    if rmc_status == 'A':
                        status.status = NavSatStatus.STATUS_FIX
                    else:
                        status.status = NavSatStatus.STATUS_NO_FIX

            else:
                # Try JSON containing lat/lon/alt/hdop
                try:
                    j = json.loads(text)
                    lat = float(j.get('lat')) if j.get('lat') is not None else None
                    lon = float(j.get('lon')) if j.get('lon') is not None else None
                    alt = float(j.get('alt')) if j.get('alt') is not None else None
                    hdop = float(j.get('hdop')) if j.get('hdop') is not None else None
                    if lat is not None and lon is not None:
                        msg.latitude = lat
                        msg.longitude = lon
                        if alt is not None:
                            msg.altitude = alt
                        status.status = NavSatStatus.STATUS_FIX
                        set_cov(hdop, None)
                except Exception:
                    pass

            # Attach status if available
            try:
                msg.status = status
            except Exception:
                # older ROS versions may not include status on NavSatFix; ignore
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

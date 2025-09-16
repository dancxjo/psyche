#!/usr/bin/env python3
"""ROS2 node to read NMEA from serial and publish NavSatFix + TimeReference."""
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import TimeReference
import serial
import pynmea2


class UbloxPublisher(Node):
    def __init__(self):
        super().__init__('ublox_publisher')
        port = self.declare_parameter('serial_port', '/dev/ttyUSB0').get_parameter_value().string_value
        baud = self.declare_parameter('baud', 9600).get_parameter_value().integer_value

        self.pub_nav = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.pub_time = self.create_publisher(TimeReference, 'gps/time_reference', 10)

        self.get_logger().info(f'Opening serial port {port} @ {baud}')
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1)
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                if not line.startswith('$'):
                    continue
                msg = pynmea2.parse(line)
                if isinstance(msg, pynmea2.types.talker.GGA):
                    nav = NavSatFix()
                    nav.header.stamp = self.get_clock().now().to_msg()
                    nav.header.frame_id = 'gps'
                    nav.latitude = msg.latitude
                    nav.longitude = msg.longitude
                    nav.altitude = float(msg.altitude) if msg.altitude else 0.0
                    nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    self.pub_nav.publish(nav)
                elif isinstance(msg, pynmea2.types.talker.RMC):
                    tr = TimeReference()
                    tr.header.stamp = self.get_clock().now().to_msg()
                    tr.time_ref = self.get_clock().now().to_msg()
                    tr.source = 'ublox'
                    self.pub_time.publish(tr)
            except Exception as e:
                self.get_logger().warning(f'Error parsing/reading NMEA: {e}')
                time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UbloxPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

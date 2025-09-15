"""
Zenoh to ROS2 IMU Bridge

- Subscribes to zenoh topic 'imu/data'
- Publishes to ROS2 topic '/imu/data' (sensor_msgs/msg/Imu)

Requires: zenoh, rclpy

Example:
    python zenoh_ros2_imu_bridge.py
"""

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import zenoh

class ZenohImuBridge(Node):
    def __init__(self):
        super().__init__('zenoh_imu_bridge')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.session = zenoh.open()
        self.key = 'imu/data'
        self.session.subscribe(self.key, self.zenoh_callback)
        self.get_logger().info('Subscribed to zenoh topic imu/data')

    def zenoh_callback(self, sample):
        try:
            data = json.loads(sample.payload.decode())
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            msg.linear_acceleration.x = data['accel_x']
            msg.linear_acceleration.y = data['accel_y']
            msg.linear_acceleration.z = data['accel_z']
            msg.angular_velocity.x = data['gyro_x']
            msg.angular_velocity.y = data['gyro_y']
            msg.angular_velocity.z = data['gyro_z']
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error parsing IMU data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ZenohImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

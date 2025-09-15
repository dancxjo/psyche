#!/usr/bin/env python3
"""ROS2 node wrapper for IMU MPU-6050 service.

This wrapper allows the IMU service to run as a proper ROS2 node
while also publishing to zenoh topics through the bridge.
"""
from __future__ import annotations

import logging
import time
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Header
except ImportError:
    # Fallback when ROS2 not available
    rclpy = None
    Node = object
    Imu = dict
    Header = dict

import sys
import pathlib
# Add services directory to path
sys.path.append(str(pathlib.Path(__file__).parent))
from imu_mpu6050 import MPU6050, IMUData


class IMUNode(Node):
    """ROS2 node for publishing IMU data from MPU-6050."""
    
    def __init__(self):
        """Initialize the IMU ROS2 node."""
        super().__init__('imu_mpu6050')
        
        # Create publisher
        self.publisher = self.create_publisher(Imu, '/ear/imu/data', 10)
        
        # Initialize MPU-6050 sensor
        self.sensor = MPU6050()
        self.sensor_connected = self.sensor.connect()
        
        if not self.sensor_connected:
            self.get_logger().warning("Running with mock IMU data")
        
        # Create timer for publishing at 50Hz
        self.timer = self.create_timer(0.02, self.publish_imu_data)
        
        self.get_logger().info('IMU MPU-6050 node started')
    
    def publish_imu_data(self) -> None:
        """Timer callback to publish IMU data."""
        # Read sensor data
        imu_data = self.sensor.read_imu_data()
        
        # Create ROS2 Imu message
        msg = Imu()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ear_imu_link'
        
        # Linear acceleration (convert g to m/s²)
        msg.linear_acceleration.x = imu_data.accel_x * 9.80665
        msg.linear_acceleration.y = imu_data.accel_y * 9.80665
        msg.linear_acceleration.z = imu_data.accel_z * 9.80665
        
        # Angular velocity (convert degrees/s to rad/s)
        msg.angular_velocity.x = imu_data.gyro_x * 0.017453293
        msg.angular_velocity.y = imu_data.gyro_y * 0.017453293
        msg.angular_velocity.z = imu_data.gyro_z * 0.017453293
        
        # Orientation (not calculated from raw MPU-6050)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        # Covariance matrices
        msg.orientation_covariance = [-1.0] + [0.0] * 8  # -1 indicates unknown
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9
        
        # Publish the message
        self.publisher.publish(msg)
        
        self.get_logger().debug(
            f'Published IMU: accel=({imu_data.accel_x:.2f}, {imu_data.accel_y:.2f}, {imu_data.accel_z:.2f}), '
            f'gyro=({imu_data.gyro_x:.2f}, {imu_data.gyro_y:.2f}, {imu_data.gyro_z:.2f}), '
            f'temp={imu_data.temperature:.1f}°C'
        )


def main(args=None):
    """Main function for ROS2 node."""
    if rclpy is None:
        # Fallback to standalone service when ROS2 not available
        print("ROS2 not available, running standalone IMU service")
        from imu_mpu6050 import main as standalone_main
        standalone_main()
        return
        
    rclpy.init(args=args)
    
    node = IMUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
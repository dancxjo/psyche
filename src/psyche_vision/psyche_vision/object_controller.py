#!/usr/bin/env python3
"""
Object controller node for psyche robot.
Subscribes to target poses and publishes cmd_vel to center the object.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
import numpy as np
import time


class ObjectController(Node):
    """Simple proportional controller to center detected objects."""
    
    def __init__(self):
        super().__init__('object_controller')
        
        # Parameters
        self.declare_parameter('max_angular_velocity', 0.5)  # rad/s
        self.declare_parameter('proportional_gain', 2.0)  # Higher = more aggressive
        self.declare_parameter('angle_tolerance_degrees', 2.0)  # Deadzone
        self.declare_parameter('min_confidence', 0.3)  # Minimum confidence to act
        self.declare_parameter('timeout_seconds', 2.0)  # Stop if no target for this long
        self.declare_parameter('image_width', 1280)  # pixels; used with /target_point
        self.declare_parameter('camera_fov_degrees', 60.0)  # FOV for pixel->angle
        self.declare_parameter('use_target_point', True)  # prefer /target_point over /target_pose
        
        # State
        self.last_target_time = None
        self.current_target_angle = None
        self.current_confidence = 0.0
        
        # Publishers and subscribers
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_callback,
            10
        )

        # Optional subscription to normalized /target_point
        self.point_sub = self.create_subscription(
            PointStamped,
            '/target_point',
            self.point_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Timer for periodic control updates
        self.timer = self.create_timer(0.1, self.control_callback)  # 10Hz
        
        self.get_logger().info('Object controller started')
    
    def target_callback(self, msg):
        """Process incoming target pose messages."""
        # Extract bearing angle from orientation.z and confidence from position.z
        bearing_radians = msg.pose.orientation.z
        self.current_target_angle = np.degrees(bearing_radians)
        self.current_confidence = msg.pose.position.z
        self.last_target_time = time.time()
    def point_callback(self, msg: PointStamped):
        """Process incoming normalized target point messages."""
        try:
            if not self.get_parameter('use_target_point').value:
                return
            # msg.point.x, msg.point.y are normalized [0..1]
            img_w = float(self.get_parameter('image_width').value)
            fov_deg = float(self.get_parameter('camera_fov_degrees').value)
            x_norm = float(msg.point.x)
            # Convert normalized x to pixel coordinate
            x_px = x_norm * img_w
            pixel_offset = x_px - (img_w / 2.0)
            pixels_per_degree = img_w / fov_deg if fov_deg > 1e-6 else img_w / 60.0
            bearing_deg = pixel_offset / pixels_per_degree
            self.current_target_angle = bearing_deg
            # Confidence unknown here; default to 0.5 unless caller encodes >0 in z
            zval = float(msg.point.z)
            self.current_confidence = zval if zval > 0.0 else 0.5
            self.last_target_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'/target_point handling failed: {e}')

        
        self.get_logger().debug(
            f'Target update: angle={self.current_target_angle:.2f}°, '
            f'confidence={self.current_confidence:.2f}'
        )
    
    def control_callback(self):
        """Periodic control loop."""
        current_time = time.time()
        
        # Check if we have a recent target
        timeout = self.get_parameter('timeout_seconds').value
        if (self.last_target_time is None or 
            current_time - self.last_target_time > timeout):
            # No recent target - stop the robot
            self.publish_stop_command()
            return
        
        # Check if target is confident enough
        min_confidence = self.get_parameter('min_confidence').value
        if self.current_confidence < min_confidence:
            self.publish_stop_command()
            return
        
        # Compute control command
        angular_velocity = self.compute_angular_velocity()
        
        # Publish command
        self.publish_velocity_command(angular_velocity)
    
    def compute_angular_velocity(self):
        """Compute angular velocity to center the target."""
        if self.current_target_angle is None:
            return 0.0
        
        # Get parameters
        max_angular_vel = self.get_parameter('max_angular_velocity').value
        proportional_gain = self.get_parameter('proportional_gain').value
        angle_tolerance = self.get_parameter('angle_tolerance_degrees').value
        
        # Check if target is already centered (within tolerance)
        if abs(self.current_target_angle) < angle_tolerance:
            return 0.0
        
        # Proportional controller: angular_vel = gain * error
        # Positive angle = target to right = turn right (positive angular velocity)
        # Negative angle = target to left = turn left (negative angular velocity)
        angular_velocity = proportional_gain * np.radians(self.current_target_angle)
        
        # Clamp to maximum velocity
        angular_velocity = np.clip(angular_velocity, -max_angular_vel, max_angular_vel)
        
        return angular_velocity
    
    def publish_velocity_command(self, angular_velocity):
        """Publish cmd_vel command."""
        twist = Twist()
        twist.linear.x = 0.0  # Don't move forward/backward
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_velocity
        
        self.cmd_vel_pub.publish(twist)
        
        if angular_velocity != 0.0:
            direction = "right" if angular_velocity > 0 else "left"
            self.get_logger().info(
                f'Turning {direction}: angular_vel={angular_velocity:.3f} rad/s, '
                f'target_angle={self.current_target_angle:.2f}°'
            )
    
    def publish_stop_command(self):
        """Publish zero velocity to stop the robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
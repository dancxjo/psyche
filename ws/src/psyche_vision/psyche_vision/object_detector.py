#!/usr/bin/env python3
"""
Object detector node for psyche robot.
Detects objects in camera images and publishes target poses.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetector(Node):
    """Simple color-based object detector."""
    
    def __init__(self):
        super().__init__('object_detector')
        
        # Parameters
        self.declare_parameter('target_color_lower', [0, 100, 100])  # HSV lower bound (red)
        self.declare_parameter('target_color_upper', [10, 255, 255])  # HSV upper bound
        self.declare_parameter('min_area', 500)  # Minimum contour area
        self.declare_parameter('image_center_x', 640)  # Image width / 2
        self.declare_parameter('camera_fov_degrees', 60.0)  # Camera field of view
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )
        
        self.target_pub = self.create_publisher(
            PoseStamped, 
            '/target_pose', 
            10
        )
        
        # Debug image publisher (optional)
        self.debug_pub = self.create_publisher(
            Image,
            '/vision_debug',
            10
        )
        
        self.get_logger().info('Object detector started')
    
    def image_callback(self, msg):
        """Process incoming camera images."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect object
            target_x, confidence = self.detect_object(cv_image)
            
            if target_x is not None:
                # Compute bearing angle
                bearing = self.compute_bearing(target_x, cv_image.shape[1])
                
                # Publish target pose
                self.publish_target_pose(bearing, confidence, msg.header.stamp)
                
                self.get_logger().info(f'Target detected: x={target_x}, bearing={bearing:.2f}°, conf={confidence:.2f}')
            else:
                # No target detected - could publish a "no target" message if needed
                pass
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def detect_object(self, image):
        """
        Detect target object using color filtering.
        Returns target center x-coordinate and confidence score.
        """
        # Convert to HSV for better color filtering
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Get color thresholds from parameters
        lower = np.array(self.get_parameter('target_color_lower').value)
        upper = np.array(self.get_parameter('target_color_upper').value)
        min_area = self.get_parameter('min_area').value
        
        # Create mask for target color
        mask = cv2.inRange(hsv, lower, upper)
        
        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, 0.0
        
        # Find largest contour that meets minimum area requirement
        largest_contour = None
        largest_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area and area > largest_area:
                largest_area = area
                largest_contour = contour
        
        if largest_contour is None:
            return None, 0.0
        
        # Get centroid of largest contour
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return None, 0.0
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        # Publish debug image
        self.publish_debug_image(image, mask, largest_contour, cx, cy)
        
        # Confidence based on relative size
        image_area = image.shape[0] * image.shape[1]
        confidence = min(1.0, largest_area / (image_area * 0.1))
        
        return cx, confidence
    
    def compute_bearing(self, target_x, image_width):
        """
        Compute bearing angle in degrees from image center.
        Positive angle = target is to the right, negative = target is to the left.
        """
        image_center_x = self.get_parameter('image_center_x').value
        camera_fov_degrees = self.get_parameter('camera_fov_degrees').value
        
        # Update image center if needed
        if image_center_x != image_width // 2:
            self.set_parameters([rclpy.Parameter('image_center_x', value=image_width // 2)])
            image_center_x = image_width // 2
        
        # Convert pixel offset to angle
        pixel_offset = target_x - image_center_x
        pixels_per_degree = image_width / camera_fov_degrees
        bearing_degrees = pixel_offset / pixels_per_degree
        
        return bearing_degrees
    
    def publish_target_pose(self, bearing_degrees, confidence, timestamp):
        """Publish target pose message."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'camera_link'
        
        # Store bearing in orientation.z (yaw) and confidence in position.z
        pose_msg.pose.orientation.z = np.radians(bearing_degrees)
        pose_msg.pose.position.z = confidence
        
        self.target_pub.publish(pose_msg)
    
    def publish_debug_image(self, original, mask, contour, cx, cy):
        """Publish debug visualization."""
        try:
            # Create debug image
            debug_img = original.copy()
            
            # Draw contour
            cv2.drawContours(debug_img, [contour], -1, (0, 255, 0), 2)
            
            # Draw centroid
            cv2.circle(debug_img, (cx, cy), 10, (255, 0, 0), -1)
            
            # Draw center line
            height, width = debug_img.shape[:2]
            cv2.line(debug_img, (width//2, 0), (width//2, height), (0, 0, 255), 2)
            
            # Convert and publish
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Debug image publishing failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
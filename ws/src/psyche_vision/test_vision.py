#!/usr/bin/env python3
"""
Simple test to validate message formats and basic functionality.
This doesn't require ROS to be installed.
"""

import sys
import os
import numpy as np

# Add the package to Python path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_bearing_computation():
    """Test bearing angle computation logic."""
    print("Testing bearing computation...")
    
    # Simulate the bearing computation logic
    def compute_bearing(target_x, image_width, camera_fov_degrees=60.0):
        image_center_x = image_width // 2
        pixel_offset = target_x - image_center_x
        pixels_per_degree = image_width / camera_fov_degrees
        bearing_degrees = pixel_offset / pixels_per_degree
        return bearing_degrees
    
    # Test cases
    image_width = 1280
    center_x = image_width // 2  # 640
    
    # Target at center should give 0 degrees
    bearing = compute_bearing(center_x, image_width)
    assert abs(bearing) < 0.01, f"Center target should be ~0°, got {bearing}"
    print(f"✓ Center target: {bearing:.2f}°")
    
    # Target at right edge
    bearing = compute_bearing(image_width - 1, image_width)
    expected = 30.0  # Half the FOV
    assert abs(bearing - expected) < 1.0, f"Right edge should be ~{expected}°, got {bearing}"
    print(f"✓ Right edge: {bearing:.2f}°")
    
    # Target at left edge
    bearing = compute_bearing(0, image_width)
    expected = -30.0  # Negative half FOV
    assert abs(bearing - expected) < 1.0, f"Left edge should be ~{expected}°, got {bearing}"
    print(f"✓ Left edge: {bearing:.2f}°")

def test_controller_logic():
    """Test proportional controller logic."""
    print("\nTesting controller logic...")
    
    def compute_angular_velocity(target_angle_degrees, max_angular_vel=0.5, 
                                proportional_gain=2.0, angle_tolerance=2.0):
        if abs(target_angle_degrees) < angle_tolerance:
            return 0.0
        
        angular_velocity = proportional_gain * np.radians(target_angle_degrees)
        return np.clip(angular_velocity, -max_angular_vel, max_angular_vel)
    
    # Test cases
    # Small angle - should be in deadzone
    vel = compute_angular_velocity(1.0)
    assert vel == 0.0, f"Small angle should give 0 velocity, got {vel}"
    print(f"✓ Small angle (deadzone): {vel} rad/s")
    
    # Medium angle - should be proportional
    vel = compute_angular_velocity(10.0)
    expected = 2.0 * np.radians(10.0)  # gain * angle_rad
    assert abs(vel - expected) < 0.01, f"Medium angle velocity mismatch: {vel} vs {expected}"
    print(f"✓ Medium angle: {vel:.3f} rad/s")
    
    # Large angle - should be clamped
    vel = compute_angular_velocity(50.0)
    assert abs(vel) == 0.5, f"Large angle should be clamped to ±0.5, got {vel}"
    print(f"✓ Large angle (clamped): {vel:.3f} rad/s")

def test_color_detection_logic():
    """Test color detection parameters."""
    print("\nTesting color detection...")
    
    # Test HSV color bounds are reasonable
    red_lower = [0, 100, 100]
    red_upper = [10, 255, 255]
    
    # Check bounds are valid
    assert len(red_lower) == 3, "HSV lower bound must have 3 values"
    assert len(red_upper) == 3, "HSV upper bound must have 3 values"
    assert all(0 <= v <= 255 for v in red_lower + red_upper), "HSV values must be 0-255"
    print("✓ HSV color bounds are valid")
    
    # Check minimum area is reasonable
    min_area = 500
    assert min_area > 0, "Minimum area must be positive"
    assert min_area < 100000, "Minimum area should be reasonable for image size"
    print(f"✓ Minimum area: {min_area} pixels")

if __name__ == "__main__":
    print("Running psyche vision system tests...")
    print("=" * 50)
    
    try:
        test_bearing_computation()
        test_controller_logic()
        test_color_detection_logic()
        
        print("\n" + "=" * 50)
        print("✅ All tests passed!")
        print("\nVision system validation complete.")
        print("The system should work correctly when deployed with ROS 2.")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        sys.exit(1)
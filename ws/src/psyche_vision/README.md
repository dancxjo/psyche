# Psyche Vision System

Object detection and tracking system for the psyche robot that implements the "ants across the bridge" architecture.

## Architecture

The system follows a modular design with two main nodes:

```
/camera/image_raw → object_detector → /target_pose → object_controller → /cmd_vel → create_driver
                         ↓
                   /vision_debug (optional debug visualization)
```

### Nodes

#### object_detector
- **Input**: `/camera/image_raw` (sensor_msgs/Image)
- **Output**: `/target_pose` (geometry_msgs/PoseStamped), `/vision_debug` (sensor_msgs/Image)
- **Function**: Detects objects using color filtering and publishes target bearing

**Parameters:**
- `target_color_lower`: HSV lower bound for target color [0, 100, 100]
- `target_color_upper`: HSV upper bound for target color [10, 255, 255] 
- `min_area`: Minimum contour area in pixels (500)
- `camera_fov_degrees`: Camera field of view (60.0)

#### object_controller
- **Input**: `/target_pose` (geometry_msgs/PoseStamped)
- **Output**: `/cmd_vel` (geometry_msgs/Twist)
- **Function**: Implements proportional controller to center detected objects

**Parameters:**
- `max_angular_velocity`: Maximum turning speed in rad/s (0.3)
- `proportional_gain`: Controller gain (1.5)
- `angle_tolerance_degrees`: Deadzone in degrees (3.0)
- `min_confidence`: Minimum confidence to act (0.2)
- `timeout_seconds`: Stop if no target for this duration (3.0)

## Usage

### Via Service System
The vision system is integrated as a service:

```bash
# Provision vision dependencies
sudo /opt/psyched/provision/services/vision.sh provision

# Launch via systemd
sudo /etc/psyched/vision.launch.sh
```

### Direct Launch
```bash
ros2 launch psyche_vision vision_launch.py
```

### Manual Node Launch
```bash
# Terminal 1: Object detector
ros2 run psyche_vision object_detector

# Terminal 2: Object controller  
ros2 run psyche_vision object_controller
```

## Configuration

### Color Detection
By default, the system detects red objects. To change target color:

```bash
ros2 param set /object_detector target_color_lower "[100, 150, 100]"  # Green HSV lower
ros2 param set /object_detector target_color_upper "[130, 255, 255]"  # Green HSV upper
```

### Controller Tuning
```bash
ros2 param set /object_controller proportional_gain 2.0              # More aggressive
ros2 param set /object_controller max_angular_velocity 0.5           # Faster turning
```

## Debugging

The system publishes debug visualization on `/vision_debug` showing:
- Detected object contours (green)
- Object centroid (blue circle)
- Image center line (red)

View with:
```bash
ros2 run rqt_image_view rqt_image_view /vision_debug
```

## Safety Features

- **Timeout protection**: Robot stops if no target detected for configured duration
- **Confidence thresholding**: Only acts on targets above minimum confidence
- **Velocity limits**: Angular velocity is clamped to safe maximum
- **Deadzone**: Small angle tolerance prevents oscillation around target

## Integration

The vision service is added to the cerebellum host configuration and will be automatically provisioned and launched with other robot services.
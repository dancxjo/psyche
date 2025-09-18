"""Behavioral tests for the eye_kinect provisioning service.

We follow a Given/When/Then narrative to assert that the Kinect v1 driver
publishes everything downstream nodes need to build a point cloud.
"""

from pathlib import Path


def _load_eye_kinect_service_script() -> str:
    """Return the eye_kinect provision script as text."""

    # Given the repository root (two directories up from this test module)
    repo_root = Path(__file__).resolve().parents[2]

    # When we read the eye_kinect provisioning script
    script_path = repo_root / "provision" / "services" / "eye_kinect.sh"
    return script_path.read_text(encoding="utf-8")


def test_eye_kinect_provisions_depth_image_proc() -> None:
    """The service should queue depth_image_proc so the point cloud node is available."""

    script_text = _load_eye_kinect_service_script()

    # Given the provisioning script text
    # When we scan the queued apt packages
    # Then the depth_image_proc package should be present
    assert "ros-${ROS_DISTRO:-jazzy}-depth-image-proc" in script_text


def test_eye_kinect_launcher_uses_psyche_launch_file() -> None:
    """The launcher should delegate to a psyche_vision launch file."""

    script_text = _load_eye_kinect_service_script()

    # Given the provisioning script text
    # When we inspect the launcher command
    # Then we should see ros2 launch pointing at psyche_vision's eye_kinect launch file
    assert "ros2 launch psyche_vision eye_kinect.launch.py" in script_text


def test_eye_kinect_launch_spawns_point_cloud_process() -> None:
    """The launch file should run the depth_image_proc point cloud executable."""

    repo_root = Path(__file__).resolve().parents[2]
    launch_file = repo_root / "ws" / "src" / "psyche_vision" / "launch" / "eye_kinect.launch.py"

    # Given the launch file text
    text = launch_file.read_text(encoding="utf-8")

    # When we look for the point cloud node configuration
    # Then we should find the depth_image_proc executable and Kinect remappings
    assert 'package="depth_image_proc"' in text
    assert 'executable="point_cloud_xyzrgb"' in text
    assert "rgb/image_rect_color" in text
    assert "depth/image_rect" in text
    assert '("points", "/camera/depth/points")' in text

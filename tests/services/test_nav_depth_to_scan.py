"""Behavioral tests for the nav service depth image handling.

We follow a Given/When/Then narrative to describe how the nav service should
bootstrap depth sensors so that Nav2 receives a 2D laser scan.
"""

from pathlib import Path


def _load_nav_service_script() -> str:
    """Return the nav service provision script as text."""

    # Given the repository root (two directories up from this test module)
    repo_root = Path(__file__).resolve().parents[2]

    # When we read the nav provisioning script
    script_path = repo_root / "provision" / "services" / "nav.sh"
    return script_path.read_text(encoding="utf-8")


def _extract_launcher_snippet(script_text: str) -> str:
    """Return the launcher definition portion of the nav service script."""

    start_token = "common_install_launcher nav"
    start_index = script_text.index(start_token)
    return script_text[start_index:]


def test_nav_service_queues_depthimage_to_laserscan_package() -> None:
    """The nav service should ensure the depth-to-scan package gets installed."""

    script_text = _load_nav_service_script()

    # Given the provisioning script text
    # When we scan the queued apt packages
    # Then the depthimage_to_laserscan package should be present
    assert "ros-${ROS_DISTRO:-jazzy}-depthimage-to-laserscan" in script_text


def test_nav_launcher_spawns_depth_to_scan_node() -> None:
    """The nav launcher should spawn depthimage_to_laserscan with Kinect remaps."""

    script_text = _load_nav_service_script()
    launcher = _extract_launcher_snippet(script_text)

    # Given the launcher snippet of the nav service
    # When we inspect the runtime commands
    # Then we should find the depthimage_to_laserscan node with Kinect topic remaps
    assert "depthimage_to_laserscan_node" in launcher
    assert "-r image:=/camera/depth/image_raw" in launcher
    assert "-r camera_info:=/camera/depth/camera_info" in launcher
    assert "-r scan:=/scan" in launcher

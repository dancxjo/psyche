"""Test IMU configuration for ear device."""
import pathlib
import textwrap
import tempfile

import pytest
import sys
sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

from tools.validate_device_toml import validate
from layer3.launcher.launch_from_toml import parse_device


def test_ear_device_config() -> None:
    """Test that ear device configuration is valid."""
    # Given the ear device configuration
    ear_config = pathlib.Path(__file__).parent.parent / "devices" / "ear.toml"
    
    # When validating the configuration
    # Then no exception should be raised
    validate(ear_config)


def test_ear_imu_node_configuration() -> None:
    """Test that ear device has IMU node properly configured."""
    # Given the ear device configuration
    ear_config = pathlib.Path(__file__).parent.parent / "devices" / "ear.toml"
    
    # When parsing the device configuration
    config = parse_device(ear_config)
    
    # Then it should have ROS2 enabled
    assert config.layer2.get("ros_distro") == "jazzy"
    assert config.layer2.get("rmw") == "rmw_cyclonedds_cpp"
    assert config.layer2.get("domain_id") == 42
    
    # And it should have an IMU node configured
    assert len(config.nodes) == 1
    
    imu_node = config.nodes[0]
    assert imu_node.name == "imu_mpu6050"
    assert imu_node.type == "direct"
    assert imu_node.package == "python3"
    assert "imu_mpu6050_node.py" in imu_node.executable
    assert imu_node.namespace == "/ear"


def test_launcher_handles_direct_type() -> None:
    """Test that launcher can handle direct execution type."""
    # Given a TOML with direct execution type
    with tempfile.NamedTemporaryFile(mode='w', suffix='.toml', delete=False) as f:
        f.write(textwrap.dedent("""
            [device]
            id = "test"
            
            [layer1]
            mode = "client"
            
            [layer2]
            ros_distro = "jazzy"
            
            [[layer3.nodes]]
            name = "test_node"
            type = "direct"
            package = "python3"
            executable = "/path/to/script.py"
        """))
        temp_path = pathlib.Path(f.name)
    
    try:
        # When parsing the configuration
        config = parse_device(temp_path)
        
        # Then it should parse correctly
        assert len(config.nodes) == 1
        node = config.nodes[0]
        assert node.type == "direct"
        assert node.package == "python3"
        assert node.executable == "/path/to/script.py"
        
    finally:
        temp_path.unlink()
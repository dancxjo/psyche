"""Tests for :mod:`tools.psy_config` shared host configuration helper.

These tests exercise both the Python API and the command line interface so
provisioning scripts can rely on deterministic behaviour when querying host
information.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

# The helper lives in the repository's ``tools`` package so ensure the project
# root is discoverable when running ``pytest`` from arbitrary directories.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools import psy_config


@pytest.fixture()
def sample_host(tmp_path: Path) -> tuple[Path, Path]:
    """Create a temporary host configuration for testing.

    The fixture mirrors the structure expected by provisioning scripts::

        <root>/provision/hosts/<hostname>.toml

    Returns a tuple of ``(root, host_file)`` so tests can point the helper at a
    specific directory without mutating the actual repository layout.
    """

    root = tmp_path / "psyche"
    host_dir = root / "provision" / "hosts"
    host_dir.mkdir(parents=True)
    host_file = host_dir / "demo.toml"
    host_file.write_text(
        """
        # Example configuration with comments and whitespace.
        ros_distro = "iron"
        domain_id = 17
        rmw = "rmw_fastrtps_cpp"

        services = [
          "ros",
          "workspace",  # build overlay
          "voice"       # piper text to speech
        ]
        """
    )
    return root, host_file


def test_load_host_config_round_trip(sample_host: tuple[Path, Path]) -> None:
    root, host_file = sample_host

    config = psy_config.load_host_config(path=host_file)

    assert config.path == host_file
    assert config.host == "demo"
    assert config.root == root
    assert config.services == ("ros", "workspace", "voice")
    assert config.ros_distro() == "iron"
    assert config.domain_id() == 17
    assert config.rmw() == "rmw_fastrtps_cpp"
    assert config.get("missing", default="fallback") == "fallback"


def test_resolve_host_file_with_explicit_host(sample_host: tuple[Path, Path]) -> None:
    root, host_file = sample_host

    resolved = psy_config.resolve_host_file(host="demo", root=root)

    assert resolved == host_file


def test_missing_host_config_returns_defaults(tmp_path: Path) -> None:
    root = tmp_path / "psy"
    (root / "provision" / "hosts").mkdir(parents=True)

    config = psy_config.load_host_config(root=root, host="unknown")

    assert config.path == root / "provision" / "hosts" / "unknown.toml"
    assert config.services == tuple()
    assert config.ros_distro() == "jazzy"
    assert config.domain_id() == 42
    assert config.rmw() == "rmw_cyclonedds_cpp"


def test_ros_environment_mapping(sample_host: tuple[Path, Path]) -> None:
    root, _ = sample_host
    config = psy_config.load_host_config(root=root, host="demo")

    env = config.ros_environment()

    assert env == {
        "ROS_DISTRO": "iron",
        "ROS_DOMAIN_ID": "17",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }


def test_cli_services_and_get(sample_host: tuple[Path, Path]) -> None:
    root, _ = sample_host
    cmd_base = [
        sys.executable,
        "-m",
        "tools.psy_config",
        "--root",
        str(root),
        "--host",
        "demo",
    ]

    services = subprocess.run(
        [*cmd_base, "services"],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip().splitlines()
    assert services == ["ros", "workspace", "voice"]

    domain_id = subprocess.run(
        [*cmd_base, "get", "domain_id"],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    assert domain_id == "17"

    missing = subprocess.run(
        [*cmd_base, "get", "missing", "--default", "fallback"],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    assert missing == "fallback"


def test_cli_handles_missing_file_gracefully(tmp_path: Path) -> None:
    root = tmp_path / "psy"
    (root / "provision" / "hosts").mkdir(parents=True)

    cmd = [
        sys.executable,
        "-m",
        "tools.psy_config",
        "--root",
        str(root),
        "--host",
        "absent",
        "services",
    ]

    services = subprocess.run(
        cmd,
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    assert services == ""

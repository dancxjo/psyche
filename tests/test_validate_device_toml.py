import pathlib
import textwrap

import pytest
import sys
sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

from tools.validate_device_toml import ValidationError, validate


def test_validate_good_device(tmp_path: pathlib.Path) -> None:
    """Ensure a complete TOML passes validation."""
    # Given a TOML with all required sections
    good = tmp_path / "device.toml"
    good.write_text(
        textwrap.dedent(
            """
            [device]
            id = "demo"
            
            [layer1]
            mode = "peer"
            
            [layer2]
            ros_distro = "jazzy"
            
            [layer3]
            nodes = []
            """
        )
    )
    # When validating
    validate(good)
    # Then no exception is raised


def test_validate_missing_section(tmp_path: pathlib.Path) -> None:
    """Missing sections raise :class:`ValidationError`."""
    # Given a TOML missing layer2
    bad = tmp_path / "device.toml"
    bad.write_text(
        textwrap.dedent(
            """
            [device]
            id = "demo"
            
            [layer1]
            mode = "peer"
            """
        )
    )
    # When / Then
    with pytest.raises(ValidationError):
        validate(bad)


def test_validate_layer1_web_enabled_boolean(tmp_path: pathlib.Path) -> None:
    """layer1.services.web.enabled must be a boolean if present."""
    bad = tmp_path / "device.toml"
    bad.write_text(
        textwrap.dedent(
            """
            [device]
            id = "demo"

            [layer1]
            mode = "peer"
            [layer1.services.web]
            enabled = "yes"

            [layer2]
            ros_distro = "jazzy"

            [layer3]
            nodes = []
            """
        )
    )
    with pytest.raises(ValidationError):
        validate(bad)

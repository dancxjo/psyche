#!/usr/bin/env python3
"""Validate device TOML files.

Example:
    $ python validate_device_toml.py devices/forebrain.toml
"""
from __future__ import annotations

import argparse
import pathlib
import tomllib

REQUIRED_TOP_LEVEL = ["device", "layer1", "layer2", "layer3"]


class ValidationError(ValueError):
    """Raised when a TOML file is missing required sections."""


def validate(path: pathlib.Path) -> None:
    """Validate a device TOML file.

    Parameters
    ----------
    path:
        Path to the TOML file.
    """
    data = tomllib.loads(path.read_text())
    for key in REQUIRED_TOP_LEVEL:
        if key not in data:
            raise ValidationError(f"Missing section [{key}] in {path}")

    # Optional: validate layer3.services.web.enabled or layer1.services.web.enabled
    # is boolean when present. Layer1 declaration overrides layer3.
    web_cfg_layer1 = data.get("layer1", {}).get("services", {}).get("web", {})
    web_cfg_layer3 = data.get("layer3", {}).get("services", {}).get("web", {})
    web_cfg = web_cfg_layer1 or web_cfg_layer3 or {}
    web_enabled = web_cfg.get("enabled")
    if web_enabled is not None and not isinstance(web_enabled, bool):
        raise ValidationError(
            f"layer1.services.web.enabled or layer3.services.web.enabled must be a boolean in {path}"
        )


def main() -> None:  # pragma: no cover - CLI wrapper
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=pathlib.Path)
    args = parser.parse_args()
    validate(args.file)


if __name__ == "__main__":  # pragma: no cover
    main()

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


def main() -> None:  # pragma: no cover - CLI wrapper
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=pathlib.Path)
    args = parser.parse_args()
    validate(args.file)


if __name__ == "__main__":  # pragma: no cover
    main()

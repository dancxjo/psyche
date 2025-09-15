#!/usr/bin/env python3
"""Launch ROS 2 entities described in a device TOML.

This minimal launcher parses the device configuration and prints the
planned actions. Actual ROS launching can be extended later.

Example:
    $ python launch_from_toml.py --device devices/forebrain.toml
"""
from __future__ import annotations

import argparse
import dataclasses
import os
import pathlib
import tomllib
from typing import List


@dataclasses.dataclass
class Node:
    """Represent a single node or launch description."""

    name: str
    type: str
    package: str
    executable: str
    namespace: str = "/"
    params: List[str] | None = None


@dataclasses.dataclass
class DeviceConfig:
    """Device-level configuration extracted from TOML."""

    layer2: dict
    nodes: List[Node]


def parse_device(path: pathlib.Path) -> DeviceConfig:
    """Parse a device TOML into a :class:`DeviceConfig`.

    Parameters
    ----------
    path:
        Path to the TOML file.

    Returns
    -------
    DeviceConfig
        Parsed configuration.
    """
    data = tomllib.loads(path.read_text())
    nodes = [Node(**n) for n in data.get("layer3", {}).get("nodes", [])]
    layer2 = data.get("layer2", {})
    return DeviceConfig(layer2=layer2, nodes=nodes)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=pathlib.Path, required=True)
    args = parser.parse_args()
    cfg = parse_device(args.device)

    # Export Layer 2 environment
    for key, value in cfg.layer2.items():
        os.environ[str(key).upper()] = str(value)

    for node in cfg.nodes:
        if node.type == "direct":
            # Direct script execution
            cmd = [node.package, node.executable]
        elif node.type == "launch":
            cmd = ["ros2", "launch", node.package, node.executable]
        else:
            cmd = ["ros2", "run", node.package, node.executable]
        print("Would run:", " ".join(cmd))


if __name__ == "__main__":  # pragma: no cover
    main()

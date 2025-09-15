"""Tests for zenoh_camera_pub configuration parsing.

Given/When/Then:
- Given default args, when parsing, then camera config has sane defaults.
- Given env/args, then overrides apply correctly.
"""
from __future__ import annotations

from layer3.services.zenoh_camera_pub import CameraConfig, parse_args


def test_default_camera_config():
    # Given no CLI args
    cfg = parse_args([])
    # Then defaults are applied
    assert cfg.key == "rt/vision/cam0"
    assert cfg.device_index == 0
    assert cfg.width == 640
    assert cfg.height == 480
    assert cfg.fps == 15
    assert 1 <= cfg.quality <= 100


def test_overrides_from_args():
    # Given custom args
    cfg = parse_args([
        "--key",
        "rt/vision/cam1",
        "--device",
        "2",
        "--width",
        "800",
        "--height",
        "600",
        "--fps",
        "10",
        "--quality",
        "70",
    ])
    # Then values reflect overrides
    assert cfg.key == "rt/vision/cam1"
    assert cfg.device_index == 2
    assert cfg.width == 800
    assert cfg.height == 600
    assert cfg.fps == 10
    assert cfg.quality == 70


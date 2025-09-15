"""Tests for zenoh_audio_pub configuration and chunk sizing.

Given/When/Then:
- Given default args, when parsing, then config uses sane defaults.
- Given a sample rate and chunk ms, then chunk byte size matches expectation.
"""
from __future__ import annotations

from layer3.services.zenoh_audio_pub import AudioConfig, build_arecord_cmd, parse_args


def test_default_config_parsing():
    # Given no CLI args
    # When parsing
    cfg = parse_args([])
    # Then defaults are applied
    assert cfg.key == "rt/audio/ear0"
    assert cfg.rate == 16000
    assert cfg.chunk_ms == 20
    assert cfg.channels == 1


def test_chunk_size_calculation():
    # Given a 16kHz mono, 20ms chunk, S16_LE
    cfg = AudioConfig(rate=16000, chunk_ms=20, channels=1)
    # When computing bytes per chunk
    # Then 16000 * 0.02 * 2 = 640 * 2 = 1280 bytes
    assert cfg.samples_per_chunk == 320
    assert cfg.bytes_per_chunk == 640  # 320 samples * 1 ch * 2 bytes


def test_arecord_cmd_contains_core_flags():
    # Given a config with custom device
    cfg = AudioConfig(device="plughw:1,0")
    # When building arecord command
    cmd = build_arecord_cmd(cfg)
    # Then expected flags are present
    assert "arecord" in cmd[0]
    assert "-D" in cmd and "plughw:1,0" in cmd


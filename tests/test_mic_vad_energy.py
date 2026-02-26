"""Tests for MicVADNode energy fallback logic."""
from __future__ import annotations

import os
import sys
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
MIC_NODE_PATH = REPO_ROOT / "provision" / "services" / "mic_node.py"

@pytest.fixture(autouse=True)
def _stub_rclpy(monkeypatch):
    if "rclpy" not in sys.modules:
        rclpy_stub = types.ModuleType("rclpy")
        rclpy_stub.ok = lambda: True
        rclpy_stub.spin_once = lambda *_, **__: None
        rclpy_stub.spin = lambda *_, **__: None
        rclpy_stub.init = lambda *_, **__: None
        rclpy_stub.shutdown = lambda *_, **__: None
        monkeypatch.setitem(sys.modules, "rclpy", rclpy_stub)
    if "rclpy.node" not in sys.modules:
        node_mod = types.ModuleType("rclpy.node")

        class _DummyNode:
            def __init__(self, *_a, **_k):
                pass

            def get_logger(self):
                class _SpyLogger:
                    def info(self, m): pass
                    def warn(self, m): pass
                    def error(self, m): pass
                return _SpyLogger()

            def create_publisher(self, *_a, **_k):
                class _SpyPublisher:
                    def publish(self, msg): pass
                return _SpyPublisher()

            def destroy_node(self):
                return None

        node_mod.Node = _DummyNode
        monkeypatch.setitem(sys.modules, "rclpy.node", node_mod)

    if "std_msgs.msg" not in sys.modules:
        std_msgs_msg = types.ModuleType("std_msgs.msg")

        class _Bool:
            __slots__ = ("data",)
            def __init__(self, data=False):
                self.data = data

        std_msgs_msg.Bool = _Bool
        monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
    yield
    # cleanup dynamic module import
    sys.modules.pop("tests.mic_node_under_test_energy", None)


@pytest.fixture
def mic_node_module(monkeypatch):
    import importlib.util

    spec = importlib.util.spec_from_file_location("tests.mic_node_under_test_energy", MIC_NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_detect_voice_empty_frame(mic_node_module, monkeypatch):
    monkeypatch.setenv("PSY_MIC_DISABLE_AUDIO", "1")
    node = mic_node_module.MicVADNode()
    node._run = False  # Stop background thread for deterministic unit test
    # Empty frame should return False
    assert node._detect_voice(b"") is False


def test_detect_voice_energy_fallback_basic(mic_node_module, monkeypatch):
    monkeypatch.setenv("PSY_MIC_DISABLE_AUDIO", "1")
    node = mic_node_module.MicVADNode()
    node._run = False  # Stop background thread for deterministic unit test
    node._vad_impl = None  # Force energy fallback
    node._energy_threshold = 100
    node._energy_sustain_frames = 1  # Simplified for basic test
    node._energy_tail_frames = 1

    # Silent frame: average energy 0 < 100
    silent_frame = b"\x00\x00" * 10
    assert node._detect_voice(silent_frame) is False

    # Loud frame: average energy 200 > 100
    loud_frame = b"\xc8\x00" * 10  # 200 in little-endian PCM16
    assert node._detect_voice(loud_frame) is True


def test_detect_voice_vad_exception_fallback(mic_node_module, monkeypatch):
    monkeypatch.setenv("PSY_MIC_DISABLE_AUDIO", "1")
    node = mic_node_module.MicVADNode()
    node._run = False  # Stop background thread for deterministic unit test

    class BrokenVAD:
        def is_speech(self, frame, sample_rate):
            raise RuntimeError("VAD error")

    node._vad_impl = BrokenVAD()
    node._energy_threshold = 100
    node._energy_sustain_frames = 1

    loud_frame = b"\xc8\x00" * 10
    # Should catch exception and fall back to energy detection
    assert node._detect_voice(loud_frame) is True


def test_detect_voice_energy_sustain_tail_hysteresis(mic_node_module, monkeypatch):
    monkeypatch.setenv("PSY_MIC_DISABLE_AUDIO", "1")
    node = mic_node_module.MicVADNode()
    node._run = False  # Stop background thread for deterministic unit test
    node._vad_impl = None
    node._energy_threshold = 100
    node._energy_sustain_frames = 2
    node._energy_tail_frames = 3

    silent_frame = b"\x00\x00" * 10
    loud_frame = b"\xc8\x00" * 10

    # Reset state to ensure deterministic test regardless of background thread
    node._energy_active = 0
    node._energy_inactive = 0
    node._last_state = None

    # Initial state: None
    assert node._last_state is None

    # 1. Loud frame 1 -> active=1, return False (sustain=2)
    assert node._detect_voice(loud_frame) is False
    assert node._energy_active == 1
    # Node's loop would call _conditional_publish, we simulate it
    node._last_state = False

    # 2. Loud frame 2 -> active=2, return True
    assert node._detect_voice(loud_frame) is True
    assert node._energy_active == 2
    node._last_state = True

    # 3. Silent frame 1 -> inactive=1, return True (tail=3)
    assert node._detect_voice(silent_frame) is True
    assert node._energy_inactive == 1
    node._last_state = True

    # 4. Silent frame 2 -> inactive=2, return True
    assert node._detect_voice(silent_frame) is True
    assert node._energy_inactive == 2
    node._last_state = True

    # 5. Silent frame 3 -> inactive=3, return False
    assert node._detect_voice(silent_frame) is False
    assert node._energy_inactive == 3
    node._last_state = False

    # 6. Hysteresis check: Loud frame 1 again -> active=1, return False (last_state)
    assert node._detect_voice(loud_frame) is False
    assert node._energy_active == 1
    node._last_state = False

    # 7. Another loud frame -> active=2, return True
    assert node._detect_voice(loud_frame) is True
    assert node._energy_active == 2
    node._last_state = True

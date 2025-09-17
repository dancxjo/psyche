"""Tests for MicVADNode VAD topic publication.

We stub a lightweight rclpy environment so the node can run without ROS
installed. Audio capture is disabled via `PSY_MIC_DISABLE_AUDIO` causing the
node to emit silent frames; the fallback energy detector should keep the
state False (no voice) but still publish at least one Bool message on the
expected topic name /voice/<host>/vad.
"""
from __future__ import annotations

import os
import sys
import time
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
MIC_NODE_PATH = REPO_ROOT / "provision" / "services" / "mic_node.py"


class _SpyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):  # pragma: no cover - very small
        self.messages.append(msg)


class _SpyLogger:
    def __init__(self):
        self.records = []

    def info(self, m):
        self.records.append(("info", m))

    def warn(self, m):  # pragma: no cover
        self.records.append(("warn", m))

    def error(self, m):  # pragma: no cover
        self.records.append(("error", m))


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
                self._logger = _SpyLogger()
                self._pubs = []

            def get_logger(self):
                return self._logger

            def create_publisher(self, *_a, **_k):
                pub = _SpyPublisher()
                self._pubs.append(pub)
                return pub

            def destroy_node(self):  # pragma: no cover
                return None

        node_mod.Node = _DummyNode
        monkeypatch.setitem(sys.modules, "rclpy.node", node_mod)

    if "std_msgs.msg" not in sys.modules:
        std_msgs_msg = types.ModuleType("std_msgs.msg")

        class _Bool:  # pragma: no cover
            __slots__ = ("data",)

            def __init__(self, data=False):
                self.data = data

        std_msgs_msg.Bool = _Bool
        monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)
    yield
    # cleanup dynamic module import
    sys.modules.pop("tests.mic_node_under_test", None)


@pytest.fixture
def mic_node_module(monkeypatch):
    import importlib.util

    spec = importlib.util.spec_from_file_location("tests.mic_node_under_test", MIC_NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_vad_topic_publishes(monkeypatch, mic_node_module):
    monkeypatch.setenv("PSY_MIC_DISABLE_AUDIO", "1")
    monkeypatch.setenv("PSY_MIC_SAMPLE_RATE", "16000")
    monkeypatch.setenv("PSY_MIC_FRAME_MS", "20")

    MicVADNode = mic_node_module.MicVADNode
    node = MicVADNode()

    # Allow some loop iterations
    time.sleep(0.15)
    pubs = getattr(node, "_pubs", [])  # from stub node
    assert pubs, "publisher list should not be empty"
    vad_pub = pubs[0]
    assert vad_pub.messages, "Expected at least one Bool publish"
    # All messages should be Bool False given silence
    assert all(getattr(m, "data", None) is False for m in vad_pub.messages)

    node.destroy_node()

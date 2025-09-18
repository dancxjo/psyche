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
    def __init__(self, msg_type, topic):
        self.messages = []
        self.msg_type = msg_type
        self.topic = topic

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

            def create_publisher(self, msg_type, topic, *_a, **_k):
                pub = _SpyPublisher(msg_type, topic)
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

    if "builtin_interfaces.msg" not in sys.modules:
        builtin_mod = types.ModuleType("builtin_interfaces.msg")

        class _Time:  # pragma: no cover
            __slots__ = ("sec", "nanosec")

            def __init__(self, sec=0, nanosec=0):
                self.sec = sec
                self.nanosec = nanosec

        builtin_mod.Time = _Time
        monkeypatch.setitem(sys.modules, "builtin_interfaces.msg", builtin_mod)

    if "psyche_interfaces.msg" not in sys.modules:
        psyche_mod = types.ModuleType("psyche_interfaces.msg")

        class _AudioPCM:  # pragma: no cover
            __slots__ = ("stamp", "sample_rate", "channels", "sample_format", "frame_index", "data")

            def __init__(self):
                self.stamp = builtin_mod.Time()
                self.sample_rate = 0
                self.channels = 0
                self.sample_format = 0
                self.frame_index = 0
                self.data = b""

        class _AudioChunk:  # pragma: no cover
            __slots__ = (
                "stamp_start",
                "stamp_end",
                "segment_id",
                "sequence",
                "sample_rate",
                "channels",
                "wav_data",
            )

            def __init__(self):
                self.stamp_start = builtin_mod.Time()
                self.stamp_end = builtin_mod.Time()
                self.segment_id = ""
                self.sequence = 0
                self.sample_rate = 0
                self.channels = 0
                self.wav_data = b""

        class _AudioSegment:  # pragma: no cover
            __slots__ = (
                "stamp_start",
                "stamp_end",
                "segment_id",
                "sample_rate",
                "channels",
                "wav_data",
                "cut_by_timeout",
            )

            def __init__(self):
                self.stamp_start = builtin_mod.Time()
                self.stamp_end = builtin_mod.Time()
                self.segment_id = ""
                self.sample_rate = 0
                self.channels = 0
                self.wav_data = b""
                self.cut_by_timeout = False

        psyche_mod.AudioPCM = _AudioPCM
        psyche_mod.AudioChunk = _AudioChunk
        psyche_mod.AudioSegment = _AudioSegment
        monkeypatch.setitem(sys.modules, "psyche_interfaces.msg", psyche_mod)
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
    monkeypatch.setenv("PSY_HOST", "testhost")
    monkeypatch.setenv("PSY_MIC_DISABLE_AUDIO", "1")
    monkeypatch.setenv("PSY_MIC_SAMPLE_RATE", "16000")
    monkeypatch.setenv("PSY_MIC_FRAME_MS", "20")

    MicVADNode = mic_node_module.MicVADNode
    node = MicVADNode()

    # Allow some loop iterations
    time.sleep(0.15)
    pubs = getattr(node, "_pubs", [])  # from stub node
    assert pubs, "publisher list should not be empty"
    # Legacy VAD topic should still exist
    vad_pub = next(p for p in pubs if p.topic == "/voice/testhost/vad")
    assert vad_pub.messages, "Expected at least one Bool publish"
    assert all(getattr(m, "data", None) is False for m in vad_pub.messages)

    # New audio VAD topic mirrors the same state
    audio_vad = next(p for p in pubs if p.topic == "/audio/vad")
    assert audio_vad.messages, "Expected /audio/vad to publish"
    assert all(getattr(m, "data", None) is False for m in audio_vad.messages)

    # Audio PCM stream should carry monotonically increasing frames
    pcm_pub = next(p for p in pubs if p.topic == "/audio/pcm")
    assert pcm_pub.messages, "Expected AudioPCM stream"
    indices = [msg.frame_index for msg in pcm_pub.messages]
    assert indices == sorted(indices)

    node.destroy_node()


def test_segmenter_emits_final_segment(monkeypatch, mic_node_module):
    monkeypatch.setenv("PSY_HOST", "testhost")
    monkeypatch.setenv("PSY_MIC_DISABLE_AUDIO", "0")
    monkeypatch.setenv("PSY_MIC_SAMPLE_RATE", "16000")
    monkeypatch.setenv("PSY_MIC_FRAME_MS", "20")
    # Relax thresholds for deterministic tests
    monkeypatch.setenv("PSY_MIC_SEGMENT_MIN_SPEECH_MS", "40")
    monkeypatch.setenv("PSY_MIC_SEGMENT_CLOSE_SILENCE_MS", "80")
    monkeypatch.setenv("PSY_MIC_SEGMENT_CURRENT_UPDATE_MS", "40")

    MicVADNode = mic_node_module.MicVADNode
    node = MicVADNode()

    frame_bytes = node.frame_bytes
    loud_frame = (b"\xff\x7f" + b"\x00\x80") * (frame_bytes // 4)
    silence_frame = b"\x00" * frame_bytes

    class _PatternSource:
        def __init__(self):
            self._frames = [loud_frame] * 4 + [silence_frame] * 10
            self._index = 0

        def read(self):
            frame = self._frames[self._index]
            self._index = (self._index + 1) % len(self._frames)
            return frame

        def close(self):  # pragma: no cover - compatibility
            return None

    node._audio = _PatternSource()

    speech_pattern = [True] * 4 + [False] * 10
    speech_iter = iter(speech_pattern * 3)

    def fake_detect_voice(_frame):
        return next(speech_iter, False)

    monkeypatch.setattr(node, "_detect_voice", fake_detect_voice)

    time.sleep(0.5)
    pubs = getattr(node, "_pubs", [])
    chunk_pub = next(p for p in pubs if p.topic == "/audio/segment/current")
    final_pub = next(p for p in pubs if p.topic == "/audio/segment/final")

    assert chunk_pub.messages, "Expected rolling chunks to publish"
    assert final_pub.messages, "Expected final segments to publish"

    final_segment = final_pub.messages[-1]
    assert getattr(final_segment, "segment_id", ""), "segment_id should be populated"
    assert getattr(final_segment, "cut_by_timeout", True) is False
    assert getattr(final_segment, "wav_data", b"")[:4] == b"RIFF"

    node.destroy_node()

    node.destroy_node()

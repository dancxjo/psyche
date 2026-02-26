"""Behavioural tests for VoiceNode: queuing, interrupt, resume, and abandon.

All subprocess and OS-signal calls are stubbed so no audio hardware or TTS
binary is required.  The tests validate the four scenarios described in the
"Voice as service" issue:

1. Text published to /voice/<host> is synthesised and played.
2. A second message published while the first is playing is queued and played
   afterwards.
3. An interrupt command pauses playback (SIGSTOP); a resume command picks back
   up where it left off (SIGCONT).
4. An abandon command stops the current utterance immediately and flushes the
   queue; subsequent text plays without delay.
"""

from __future__ import annotations

import os
import queue as _queue
import signal
import subprocess
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
VOICE_NODE_PATH = REPO_ROOT / "provision" / "services" / "voice_node.py"


# ---------------------------------------------------------------------------
# Minimal ROS stubs
# ---------------------------------------------------------------------------


class _SpyLogger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def info(self, m: str) -> None:
        self.messages.append(("info", m))

    def warn(self, m: str) -> None:
        self.messages.append(("warn", m))

    def error(self, m: str) -> None:
        self.messages.append(("error", m))


@pytest.fixture(autouse=True)
def _stub_rclpy(monkeypatch):
    """Inject minimal rclpy / std_msgs stubs."""

    if "rclpy" not in sys.modules:
        rclpy_stub = types.ModuleType("rclpy")
        rclpy_stub.ok = lambda: True
        # Add a small sleep so the player-loop thread doesn't busy-spin.
        rclpy_stub.spin_once = lambda *_, **__: time.sleep(0.01)
        rclpy_stub.spin = lambda *_, **__: None
        rclpy_stub.init = lambda *_, **__: None
        rclpy_stub.shutdown = lambda *_, **__: None
        monkeypatch.setitem(sys.modules, "rclpy", rclpy_stub)

    if "rclpy.node" not in sys.modules:
        node_stub = types.ModuleType("rclpy.node")

        class _DummyNode:
            def __init__(self, *_a, **_k):
                self._logger = _SpyLogger()

            def get_logger(self):
                return self._logger

            def create_subscription(self, *_a, **_k):
                return None

            def create_publisher(self, *_a, **_k):
                class _Pub:
                    def publish(self, *_a2, **_k2):
                        return None

                return _Pub()

            def get_clock(self):
                class _Clock:
                    def now(self_inner):
                        class _T:
                            def to_msg(self_t):
                                return None

                        return _T()

                return _Clock()

            def destroy_node(self) -> None:
                return None

        node_stub.Node = _DummyNode
        monkeypatch.setitem(sys.modules, "rclpy.node", node_stub)

    if "std_msgs" not in sys.modules:
        monkeypatch.setitem(sys.modules, "std_msgs", types.ModuleType("std_msgs"))

    if "std_msgs.msg" not in sys.modules:
        msg_stub = types.ModuleType("std_msgs.msg")

        class _Empty:
            __slots__ = ()

        class _String:
            __slots__ = ("data",)

            def __init__(self, data: str = "") -> None:
                self.data = data

        msg_stub.Empty = _Empty
        msg_stub.String = _String
        monkeypatch.setitem(sys.modules, "std_msgs.msg", msg_stub)

    yield

    for name in list(sys.modules):
        if "voice_node_behavior_under_test" in name:
            sys.modules.pop(name, None)


@pytest.fixture
def voice_node_module():
    """Freshly imported voice_node module for each test."""
    import importlib.util

    name = "tests.voice_node_behavior_under_test"
    sys.modules.pop(name, None)
    spec = importlib.util.spec_from_file_location(name, VOICE_NODE_PATH)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    sys.modules[name] = mod
    return mod


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------


class _InstantEngine:
    """TTS stub: writes a minimal WAV immediately; records utterances."""

    name = "instant-stub"

    def __init__(self) -> None:
        self.synthesized: list[str] = []
        self._lock = threading.Lock()

    def synthesize(self, text: str) -> str:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fh:
            fh.write(b"\x00" * 44)  # placeholder bytes so file exists
            path = fh.name
        with self._lock:
            self.synthesized.append(text)
        return path

    def describe(self) -> str:
        return self.name

    def shutdown(self) -> None:
        pass


class _FakeProc:
    """Simulates an aplay process with a configurable playback duration."""

    def __init__(self, duration: float = 0.05) -> None:
        self._duration = duration
        self._start = time.monotonic()
        self._killed = False
        self.pid = 99999  # arbitrary; signals are intercepted

    def poll(self) -> int | None:
        if self._killed:
            return -15
        if time.monotonic() - self._start >= self._duration:
            return 0
        return None

    def terminate(self) -> None:
        self._killed = True

    def kill(self) -> None:
        self._killed = True

    def wait(self, timeout: float | None = None) -> int:
        return 0


def _build_node(voice_node_module, engine, popen_factory, monkeypatch):
    """Construct a VoiceNode with injected engine and mocked subprocess.Popen."""
    vn = voice_node_module

    monkeypatch.setattr(vn, "PiperEngine", lambda *_: engine)
    monkeypatch.setattr(vn, "EspeakEngine", lambda *_: engine)
    monkeypatch.setenv("PSY_TTS_ENGINE", "piper")
    monkeypatch.setenv("PSY_HOST", "testhost")
    monkeypatch.setattr(subprocess, "Popen", popen_factory)

    node = vn.VoiceNode()
    return node


def _wait(condition, timeout: float = 3.0, interval: float = 0.02) -> bool:
    """Poll *condition* until it returns True or *timeout* expires."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(interval)
    return False


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_text_is_synthesised_and_played(monkeypatch, voice_node_module):
    """Publishing a string to the voice topic results in TTS synthesis and aplay."""
    engine = _InstantEngine()
    procs: list[_FakeProc] = []

    def _popen(cmd, *_a, **_k):
        p = _FakeProc(duration=0.05)
        procs.append(p)
        return p

    monkeypatch.setattr(os, "kill", lambda *_: None)
    node = _build_node(voice_node_module, engine, _popen, monkeypatch)

    String = sys.modules["std_msgs.msg"].String
    node.text_cb(String("hello robot"))

    assert _wait(lambda: len(engine.synthesized) >= 1), "utterance should be synthesised"
    assert _wait(lambda: len(procs) >= 1), "aplay should be launched"
    assert engine.synthesized[0] == "hello robot"

    node.destroy_node()


def test_second_message_is_queued_during_playback(monkeypatch, voice_node_module):
    """A message published while one is playing is queued and played afterwards."""
    engine = _InstantEngine()
    procs: list[_FakeProc] = []
    # First utterance is long; second is short.
    durations = [0.4, 0.05]
    call_count = [0]

    def _popen(cmd, *_a, **_k):
        dur = durations[min(call_count[0], len(durations) - 1)]
        call_count[0] += 1
        p = _FakeProc(duration=dur)
        procs.append(p)
        return p

    monkeypatch.setattr(os, "kill", lambda *_: None)
    node = _build_node(voice_node_module, engine, _popen, monkeypatch)

    String = sys.modules["std_msgs.msg"].String
    node.text_cb(String("first"))
    # Let the first start
    assert _wait(lambda: len(procs) >= 1), "first aplay should start"

    node.text_cb(String("second"))

    assert _wait(lambda: len(engine.synthesized) >= 2, timeout=4.0), (
        "both utterances should be synthesised"
    )
    assert engine.synthesized[0] == "first"
    assert engine.synthesized[1] == "second"
    assert len(procs) == 2, "aplay called once per utterance"

    node.destroy_node()


def test_interrupt_sends_sigstop_and_sets_paused(monkeypatch, voice_node_module):
    """_interrupt() pauses playback by sending SIGSTOP to the aplay process."""
    engine = _InstantEngine()
    long_proc = _FakeProc(duration=5.0)
    signals: list[tuple[int, int]] = []

    monkeypatch.setattr(os, "kill", lambda pid, sig: signals.append((pid, sig)))
    node = _build_node(
        voice_node_module, engine, lambda *_, **__: long_proc, monkeypatch
    )

    String = sys.modules["std_msgs.msg"].String
    node.text_cb(String("speak slowly"))

    assert _wait(lambda: node.current_proc is not None), "playback should have started"

    node._interrupt()

    assert node.paused is True, "node should mark itself as paused"
    assert any(sig == signal.SIGSTOP for _, sig in signals), (
        "SIGSTOP must be sent to the aplay process"
    )

    node.destroy_node()


def test_resume_sends_sigcont_and_clears_paused(monkeypatch, voice_node_module):
    """_resume() after _interrupt() sends SIGCONT and clears the paused flag."""
    engine = _InstantEngine()
    long_proc = _FakeProc(duration=5.0)
    signals: list[tuple[int, int]] = []

    monkeypatch.setattr(os, "kill", lambda pid, sig: signals.append((pid, sig)))
    node = _build_node(
        voice_node_module, engine, lambda *_, **__: long_proc, monkeypatch
    )

    String = sys.modules["std_msgs.msg"].String
    node.text_cb(String("speak slowly again"))

    assert _wait(lambda: node.current_proc is not None), "playback should have started"

    node._interrupt()
    assert node.paused is True

    node._resume()

    assert node.paused is False, "paused flag should be cleared after resume"
    assert any(sig == signal.SIGCONT for _, sig in signals), (
        "SIGCONT must be sent to resume playback"
    )

    node.destroy_node()


def test_abandon_terminates_process_and_flushes_queue(monkeypatch, voice_node_module):
    """_abandon() terminates the playing utterance and empties the queue."""
    engine = _InstantEngine()
    long_proc = _FakeProc(duration=5.0)

    monkeypatch.setattr(os, "kill", lambda *_: None)
    node = _build_node(
        voice_node_module, engine, lambda *_, **__: long_proc, monkeypatch
    )

    String = sys.modules["std_msgs.msg"].String
    node.text_cb(String("currently playing"))

    assert _wait(lambda: node.current_proc is not None), "playback should have started"

    # Enqueue additional messages that should be discarded.
    node.text_cb(String("queued 1"))
    node.text_cb(String("queued 2"))

    node._abandon()

    assert long_proc._killed, "current aplay process must be terminated"
    assert node.queue.empty(), "queue must be empty after abandon"

    node.destroy_node()


def test_abandon_allows_new_text_to_play_promptly(monkeypatch, voice_node_module):
    """After an abandon, new text arrives and is played without being blocked."""
    engine = _InstantEngine()
    long_proc = _FakeProc(duration=5.0)
    short_proc = _FakeProc(duration=0.05)
    call_count = [0]

    def _popen(cmd, *_a, **_k):
        p = long_proc if call_count[0] == 0 else short_proc
        call_count[0] += 1
        return p

    monkeypatch.setattr(os, "kill", lambda *_: None)
    node = _build_node(voice_node_module, engine, _popen, monkeypatch)

    String = sys.modules["std_msgs.msg"].String
    node.text_cb(String("original long text"))

    assert _wait(lambda: node.current_proc is not None), "first playback should start"

    node._abandon()

    # Wait briefly for abandon to propagate
    assert _wait(lambda: node.current_proc is None, timeout=2.0), (
        "player should finish processing abandoned utterance"
    )

    t0 = time.monotonic()
    node.text_cb(String("fresh text after abandon"))

    assert _wait(lambda: "fresh text after abandon" in engine.synthesized, timeout=3.0), (
        "new text should be synthesised after abandon"
    )
    assert time.monotonic() - t0 < 2.5, "new text should play promptly after abandon"

    node.destroy_node()


def test_cmd_topic_dispatches_interrupt_resume_abandon(monkeypatch, voice_node_module):
    """The /cmd topic string commands dispatch to the correct internal methods."""
    vn = voice_node_module
    engine = _InstantEngine()

    monkeypatch.setattr(vn, "PiperEngine", lambda *_: engine)
    monkeypatch.setattr(vn, "EspeakEngine", lambda *_: engine)
    monkeypatch.setenv("PSY_TTS_ENGINE", "piper")
    monkeypatch.setenv("PSY_HOST", "testhost")
    monkeypatch.setattr(subprocess, "Popen", lambda *_, **__: _FakeProc(0.05))
    monkeypatch.setattr(os, "kill", lambda *_: None)

    node = vn.VoiceNode()

    calls: list[str] = []
    monkeypatch.setattr(node, "_interrupt", lambda: calls.append("interrupt"))
    monkeypatch.setattr(node, "_resume", lambda: calls.append("resume"))
    monkeypatch.setattr(node, "_abandon", lambda: calls.append("abandon"))

    String = sys.modules["std_msgs.msg"].String
    node.cmd_cb(String("interrupt"))
    node.cmd_cb(String("pause"))
    node.cmd_cb(String("stop"))
    node.cmd_cb(String("resume"))
    node.cmd_cb(String("continue"))
    node.cmd_cb(String("abandon"))
    node.cmd_cb(String("cancel"))
    node.cmd_cb(String("flush"))

    assert calls.count("interrupt") == 3
    assert calls.count("resume") == 2
    assert calls.count("abandon") == 3

    node.destroy_node()

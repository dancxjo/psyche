"""Tests for Piper CLI detection to avoid the GTK front-end wrapper.

These tests focus on the behaviour of :class:`PiperEngine` when only the GTK
launcher is present versus when a headless CLI binary exists. The
implementation should avoid executing the GTK GUI and instead prefer an actual
CLI binary, whether provided via PATH or explicit configuration.
"""

from __future__ import annotations

import os
import re
import stat
import sys
import types
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
VOICE_NODE_PATH = REPO_ROOT / "provision" / "services" / "voice_node.py"


class _SpyLogger:
    """Minimal logger capturing emitted messages for assertions."""

    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def info(self, message: str) -> None:
        self.messages.append(("info", message))

    def warn(self, message: str) -> None:  # pragma: no cover - not exercised yet
        self.messages.append(("warn", message))

    def error(self, message: str) -> None:  # pragma: no cover - not exercised yet
        self.messages.append(("error", message))


@pytest.fixture(autouse=True)
def _stub_rclpy_modules(monkeypatch):
    """Provide lightweight ``rclpy`` shims so ``voice_node`` can be imported."""

    if "rclpy" not in sys.modules:
        rclpy_stub = types.ModuleType("rclpy")
        rclpy_stub.ok = lambda: True
        rclpy_stub.spin_once = lambda *_, **__: None
        rclpy_stub.spin = lambda *_, **__: None
        rclpy_stub.init = lambda *_, **__: None
        rclpy_stub.shutdown = lambda *_, **__: None
        monkeypatch.setitem(sys.modules, "rclpy", rclpy_stub)
    if "rclpy.node" not in sys.modules:
        node_stub = types.ModuleType("rclpy.node")

        class _DummyNode:
            def __init__(self, *_args, **_kwargs):
                self._logger = _SpyLogger()

            def get_logger(self) -> _SpyLogger:
                return self._logger

            def create_subscription(self, *_args, **_kwargs):  # pragma: no cover
                return None

            def create_publisher(self, *_args, **_kwargs):  # pragma: no cover - new for conversation pub
                class _Pub:
                    def publish(self, *_a, **_k):
                        return None
                return _Pub()

        node_stub.Node = _DummyNode
        monkeypatch.setitem(sys.modules, "rclpy.node", node_stub)

    if "std_msgs" not in sys.modules:
        std_msgs_stub = types.ModuleType("std_msgs")
        monkeypatch.setitem(sys.modules, "std_msgs", std_msgs_stub)
    if "std_msgs.msg" not in sys.modules:
        msg_stub = types.ModuleType("std_msgs.msg")

        class _Empty:  # pragma: no cover - behaviour not exercised directly
            __slots__ = ()

        class _String:  # pragma: no cover - behaviour not exercised directly
            __slots__ = ("data",)

            def __init__(self, data: str = "") -> None:
                self.data = data

        msg_stub.Empty = _Empty
        msg_stub.String = _String
        monkeypatch.setitem(sys.modules, "std_msgs.msg", msg_stub)

    yield

    # Clean up any modules imported during the test so each test starts fresh.
    for name in list(sys.modules):
        if name.startswith("voice_node") or name.startswith("tests.voice_node"):
            sys.modules.pop(name, None)
@pytest.fixture
def voice_node_module():
    """Fixture returning a freshly imported ``voice_node`` module."""

    # Delay importlib utilisation until needed to keep imports lightweight.
    import importlib.util

    module_name = "tests.voice_node_under_test"
    if module_name in sys.modules:
        sys.modules.pop(module_name)

    spec = importlib.util.spec_from_file_location(module_name, VOICE_NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    sys.modules[module_name] = module
    return module


def _write_executable(path: Path, contents: str) -> Path:
    path.write_text(contents)
    path.chmod(path.stat().st_mode | stat.S_IEXEC)
    return path


@pytest.fixture
def voice_model(tmp_path: Path) -> Path:
    """Create a dummy Piper model file used during tests."""

    model = tmp_path / "en_US-test.onnx"
    model.write_bytes(b"\x00")
    return model


def test_piper_engine_rejects_gtk_wrapped_binary(monkeypatch, tmp_path, voice_model, voice_node_module):
    """Ensure GTK launcher scripts are rejected even if they are the only binary."""

    gui_wrapper = _write_executable(
        tmp_path / "piper",
        """#!/usr/bin/env python3
import gi

gi.require_version(\"Gtk\", \"3.0\")
""",
    )

    monkeypatch.setenv("PATH", f"{tmp_path}:{os.environ.get('PATH', '')}")
    monkeypatch.setenv("PSY_VOICE_MODEL", str(voice_model))
    monkeypatch.delenv("PSY_PIPER_BIN", raising=False)

    PiperEngine = voice_node_module.PiperEngine

    with pytest.raises(RuntimeError):
        PiperEngine(_SpyLogger())

    assert gui_wrapper.exists(), "sanity check: GUI wrapper should remain present"


def test_piper_engine_prefers_cli_over_gui(monkeypatch, tmp_path, voice_model, voice_node_module):
    """When both binaries exist, prefer the CLI implementation."""

    gui_wrapper = _write_executable(
        tmp_path / "piper",
        """#!/usr/bin/env python3
import gi

gi.require_version('Gtk', '3.0')
""",
    )
    cli_binary = _write_executable(
        tmp_path / "piper-tts",
        """#!/usr/bin/env bash
echo \"piper cli stub\"
""",
    )

    monkeypatch.setenv("PATH", f"{tmp_path}:{os.environ.get('PATH', '')}")
    monkeypatch.setenv("PSY_VOICE_MODEL", str(voice_model))
    monkeypatch.delenv("PSY_PIPER_BIN", raising=False)

    PiperEngine = voice_node_module.PiperEngine
    engine = PiperEngine(_SpyLogger())

    assert engine.piper_bin == str(cli_binary)
    assert gui_wrapper.exists()


def test_voice_provision_config_defaults_to_piper():
    """Voice provisioning script should default to the Piper engine."""

    script_text = (REPO_ROOT / "provision" / "services" / "voice.sh").read_text()

    assert 'engine="piper"' in script_text, "piper should be the preferred engine"
    assert re.search(r'PSY_TTS_ENGINE=\${engine}', script_text), (
        "config writer must honour the selected engine"
    )


def test_voice_node_falls_back_to_espeak_when_piper_unavailable(
    monkeypatch, voice_node_module
):
    """If Piper initialisation fails we should instantiate the espeak engine."""

    class _FailingPiper(voice_node_module.TTSEngine):
        def __init__(self, logger):
            super().__init__(logger)
            raise RuntimeError("piper missing")

    class _FakeEspeak(voice_node_module.TTSEngine):
        def __init__(self, logger):
            super().__init__(logger)
            self._logger.info("espeak initialised")

    monkeypatch.setenv("PSY_TTS_ENGINE", "piper")
    monkeypatch.setattr(voice_node_module, "PiperEngine", _FailingPiper)
    monkeypatch.setattr(voice_node_module, "EspeakEngine", _FakeEspeak)

    dummy = voice_node_module.VoiceNode.__new__(voice_node_module.VoiceNode)
    dummy._logger = _SpyLogger()

    engine = voice_node_module.VoiceNode._select_engine(dummy)
    assert isinstance(engine, _FakeEspeak)


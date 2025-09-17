#!/usr/bin/env python3
import glob
import os
import queue
import shlex
import shutil
import signal
import socket
import subprocess
import tempfile
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


class TTSEngine:
    """Abstract interface for synthesizing speech to a WAV file."""

    name = "tts"

    def __init__(self, logger):
        self._logger = logger

    def synthesize(self, text: str) -> Optional[str]:  # pragma: no cover - interface
        raise NotImplementedError

    def describe(self) -> str:
        return self.name

    def shutdown(self) -> None:
        """Clean up resources prior to process exit."""
        return None


class PiperEngine(TTSEngine):
    name = "piper"

    def __init__(self, logger):
        super().__init__(logger)

        primary_model = os.environ.get(
            "PSY_VOICE_MODEL", "/opt/psyched/voices/en_US-kyle-high.onnx"
        )
        fallback_list = os.environ.get("PSY_VOICE_MODEL_FALLBACKS", "").strip()
        self.model_candidates: List[str] = [primary_model]
        if fallback_list:
            self.model_candidates.extend([p for p in fallback_list.split(":") if p])

        self.model_path: Optional[str] = self._resolve_model_path(initial=True)
        self.piper_bin = self._resolve_piper_bin()

    # Public API -----------------------------------------------------------------
    def synthesize(self, text: str) -> Optional[str]:
        model_path = self._resolve_model_path()
        if not model_path:
            self._logger.error("No usable Piper model found; skipping utterance.")
            return None

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as handle:
            wav_path = handle.name

        # Append newline so CLI engines flush the final token read from stdin.
        payload = text if text.endswith("\n") else f"{text}\n"
        proc = subprocess.run(
            [self.piper_bin, "-m", model_path, "-o", wav_path],
            input=payload.encode("utf-8"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if proc.returncode != 0 or not os.path.exists(wav_path):
            self._logger.error(
                f"piper synth failed rc={proc.returncode}: {proc.stderr.decode(errors='ignore')[:200]}"
            )
            try:
                os.unlink(wav_path)
            except FileNotFoundError:
                pass
            return None

        return wav_path

    def describe(self) -> str:
        model = self.model_path or "<unresolved>"
        return f"piper ({model})"

    # Internal helpers -----------------------------------------------------------
    def _resolve_piper_bin(self) -> str:
        configured = os.environ.get("PSY_PIPER_BIN", "").strip()
        candidates: List[str] = []
        if configured:
            candidates.append(configured)
        candidates.extend(["piper-tts", "piper"])

        examined = set()
        gtk_markers = (
            b"gi.require_version('Gtk'",
            b'gi.require_version("Gtk"',
        )
        for candidate in candidates:
            if not candidate:
                continue
            resolved = shutil.which(candidate)
            if resolved is None and os.path.isabs(candidate) and os.access(candidate, os.X_OK):
                resolved = candidate
            if not resolved or resolved in examined:
                continue
            examined.add(resolved)
            try:
                with open(resolved, "rb") as handle:
                    head = handle.read(4096)
                if any(marker in head for marker in gtk_markers):
                    continue
            except OSError:
                pass
            return resolved

        raise RuntimeError(
            "No Piper CLI binary found. Install 'piper-tts' or set PSY_PIPER_BIN to a valid executable."
        )

    def _resolve_model_path(self, initial: bool = False) -> Optional[str]:
        current_path = getattr(self, "model_path", None)
        for candidate in self.model_candidates:
            if not candidate:
                continue
            if os.path.isfile(candidate):
                if candidate != current_path:
                    self._logger.info(f"Using Piper model: {candidate}")
                self.model_path = candidate
                current_path = candidate
                return candidate

        search_dirs: List[str] = []
        for candidate in self.model_candidates:
            candidate_dir = os.path.dirname(candidate)
            if candidate_dir and os.path.isdir(candidate_dir):
                search_dirs.append(candidate_dir)
        default_dir = os.environ.get("PSY_VOICE_MODEL_DIR", "/opt/psyched/voices")
        if default_dir and os.path.isdir(default_dir):
            search_dirs.append(default_dir)

        discovered: List[str] = []
        seen = set()
        for directory in search_dirs:
            for path in sorted(glob.glob(os.path.join(directory, "*.onnx"))):
                full = os.path.abspath(path)
                if full not in seen:
                    seen.add(full)
                    discovered.append(full)
        if discovered:
            for path in discovered:
                if path not in self.model_candidates:
                    self.model_candidates.append(path)
            fallback = discovered[0]
            self._logger.warn(f"Falling back to discovered Piper model: {fallback}")
            self.model_path = fallback
            return fallback

        if initial:
            self._logger.warn(
                f"No available voice model found among candidates: {self.model_candidates}."
            )
        else:
            self._logger.error(
                f"No Piper model files exist for candidates: {self.model_candidates}."
            )
        self.model_path = None
        return None


class EspeakEngine(TTSEngine):
    name = "espeak"

    def __init__(self, logger):
        super().__init__(logger)
        self.espeak_bin = self._resolve_binary()
        self.voice = os.environ.get("PSY_ESPEAK_VOICE", "").strip()
        self.rate = os.environ.get("PSY_ESPEAK_RATE", "").strip()
        self.pitch = os.environ.get("PSY_ESPEAK_PITCH", "").strip()
        extra = os.environ.get("PSY_ESPEAK_ARGS", "").strip()
        self.extra_args = shlex.split(extra) if extra else []

    def synthesize(self, text: str) -> Optional[str]:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as handle:
            wav_path = handle.name

        cmd = [self.espeak_bin, "--stdin", "-w", wav_path]
        if self.voice:
            cmd.extend(["-v", self.voice])
        if self.rate:
            cmd.extend(["-s", self.rate])
        if self.pitch:
            cmd.extend(["-p", self.pitch])
        cmd.extend(self.extra_args)

        # Append newline so CLI engines flush the final token read from stdin.
        payload = text if text.endswith("\n") else f"{text}\n"
        proc = subprocess.run(
            cmd,
            input=payload.encode("utf-8"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if proc.returncode != 0 or not os.path.exists(wav_path):
            self._logger.error(
                "espeak-ng synth failed rc=%s: %s",
                proc.returncode,
                proc.stderr.decode(errors="ignore")[:200],
            )
            try:
                os.unlink(wav_path)
            except FileNotFoundError:
                pass
            return None
        return wav_path

    def describe(self) -> str:
        details = []
        if self.voice:
            details.append(self.voice)
        if self.rate:
            details.append(f"rate={self.rate}")
        if self.pitch:
            details.append(f"pitch={self.pitch}")
        if details:
            return f"espeak ({', '.join(details)})"
        return "espeak"

    def _resolve_binary(self) -> str:
        configured = os.environ.get("PSY_ESPEAK_BIN", "espeak-ng").strip()
        resolved = shutil.which(configured)
        if not resolved and os.path.isabs(configured) and os.access(configured, os.X_OK):
            resolved = configured
        if not resolved:
            raise RuntimeError(
                "espeak-ng binary not found. Install 'espeak-ng' or set PSY_ESPEAK_BIN"
            )
        return resolved


class VoiceNode(Node):
    def __init__(self):
        super().__init__("psy_voice")
        host = os.environ.get("PSY_HOST", "") or socket.gethostname().split(".")[0]
        self.base_topic = f"/voice/{host}"

        self.engine = self._select_engine()

        self.queue: "queue.Queue[str]" = queue.Queue()
        self.current_proc: Optional[subprocess.Popen] = None
        self.current_wav: Optional[str] = None
        self.paused = False
        self.abandon_flag = False
        self.lock = threading.RLock()

        self.create_subscription(String, self.base_topic, self.text_cb, 10)
        self.create_subscription(String, f"{self.base_topic}/cmd", self.cmd_cb, 10)
        self.create_subscription(Empty, f"{self.base_topic}/interrupt", lambda _: self._interrupt(), 10)
        self.create_subscription(Empty, f"{self.base_topic}/resume", lambda _: self._resume(), 10)
        self.create_subscription(Empty, f"{self.base_topic}/abandon", lambda _: self._abandon(), 10)

        self.player_thread = threading.Thread(target=self._player_loop, daemon=True)
        self.player_thread.start()

        self.get_logger().info(
            f"VoiceNode ready. Text: {self.base_topic} | Cmd: {self.base_topic}/cmd | Engine: {self.engine.describe()}"
        )

    # Subscription callbacks -----------------------------------------------------
    def text_cb(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"Queue: {text[:64]}...")
        self.queue.put(text)

    def cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ("interrupt", "pause", "stop"):
            self._interrupt()
        elif cmd in ("resume", "continue"):
            self._resume()
        elif cmd in ("abandon", "cancel", "flush"):
            self._abandon()
        else:
            self.get_logger().warn(f"Unknown cmd: {cmd}")

    # Player control -------------------------------------------------------------
    def _interrupt(self):
        with self.lock:
            if self.current_proc and not self.paused:
                try:
                    os.kill(self.current_proc.pid, signal.SIGSTOP)
                    self.paused = True
                    self.get_logger().info("Paused (interrupt).")
                except Exception as exc:  # pragma: no cover - best effort
                    self.get_logger().error(f"Interrupt failed: {exc}")

    def _resume(self):
        with self.lock:
            if self.current_proc and self.paused:
                try:
                    os.kill(self.current_proc.pid, signal.SIGCONT)
                    self.paused = False
                    self.get_logger().info("Resumed.")
                except Exception as exc:  # pragma: no cover - best effort
                    self.get_logger().error(f"Resume failed: {exc}")

    def _abandon(self):
        with self.lock:
            self.abandon_flag = True
            try:
                while True:
                    self.queue.get_nowait()
            except queue.Empty:
                pass
            if self.current_proc:
                try:
                    self.current_proc.terminate()
                    self.get_logger().info(
                        "Abandon: terminated current playback and flushed queue."
                    )
                except Exception as exc:  # pragma: no cover - best effort
                    self.get_logger().error(f"Abandon terminate failed: {exc}")

    def _player_loop(self):
        while rclpy.ok():
            try:
                text = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if text is None:
                continue
            self._speak_text(text)

    def _speak_text(self, text: str):
        wav_path: Optional[str] = None
        try:
            wav_path = self.engine.synthesize(text)
            if not wav_path or not os.path.exists(wav_path):
                return

            with self.lock:
                self.current_wav = wav_path
                self.current_proc = subprocess.Popen(["aplay", "-q", wav_path])
                self.paused = False
                self.abandon_flag = False

            while True:
                with self.lock:
                    proc = self.current_proc
                    abandon = self.abandon_flag
                if proc is None:
                    break
                ret = proc.poll()
                if ret is not None:
                    break
                if abandon:
                    try:
                        proc.terminate()
                    except Exception:  # pragma: no cover - best effort
                        pass
                    break
                rclpy.spin_once(self, timeout_sec=0.05)

        except Exception as exc:  # pragma: no cover - broad catch for logging
            self.get_logger().error(f"Playback error: {exc}")
        finally:
            with self.lock:
                if self.current_proc:
                    try:
                        self.current_proc.wait(timeout=0.2)
                    except Exception:
                        try:
                            self.current_proc.kill()
                        except Exception:
                            pass
                self.current_proc = None
                self.paused = False
                self.abandon_flag = False
                self.current_wav = None
            if wav_path and os.path.exists(wav_path):
                try:
                    os.unlink(wav_path)
                except Exception:
                    pass

    # Engine selection -----------------------------------------------------------
    def _select_engine(self) -> TTSEngine:
        requested = os.environ.get("PSY_TTS_ENGINE", "piper").strip().lower()
        if not requested:
            requested = "piper"

        if requested in ("espeak-ng", "espeakng"):
            requested = "espeak"

        if requested == "auto":
            try:
                engine = PiperEngine(self.get_logger())
                self.get_logger().info("Auto-selected Piper TTS engine")
                return engine
            except Exception as exc:
                self.get_logger().warn(
                    f"Auto TTS selection failed to init Piper: {exc}. Falling back to espeak."
                )
                requested = "espeak"

        if requested == "piper":
            try:
                return PiperEngine(self.get_logger())
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to initialise Piper engine ({exc}); falling back to espeak."
                )
                requested = "espeak"

        if requested != "espeak":
            self.get_logger().warn(
                f"Unknown PSY_TTS_ENGINE value '{requested}'; defaulting to espeak."
            )
        return EspeakEngine(self.get_logger())

    def destroy_node(self):
        try:
            if hasattr(self, "engine") and self.engine:
                self.engine.shutdown()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

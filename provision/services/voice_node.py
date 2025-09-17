#!/usr/bin/env python3
import os
import signal
import socket
import subprocess
import tempfile
import threading
import queue
import shutil
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty


class PiperVoiceNode(Node):
    def __init__(self):
        super().__init__('piper_voice')
        host = os.environ.get('PSY_HOST', '') or socket.gethostname().split('.')[0]
        self.base_topic = f'/voice/{host}'

        # Model selection with fallback list
        # Primary model from PSY_VOICE_MODEL; additional fallbacks from PSY_VOICE_MODEL_FALLBACKS (colon-separated)
        primary_model = os.environ.get('PSY_VOICE_MODEL', '/opt/psyched/voices/en_US-kyle-high.onnx')
        fallback_list = os.environ.get('PSY_VOICE_MODEL_FALLBACKS', '').strip()
        candidates = [primary_model]
        if fallback_list:
            candidates.extend([p for p in fallback_list.split(':') if p])
        resolved_model = None
        for cand in candidates:
            if os.path.isfile(cand):
                resolved_model = cand
                break
        if not resolved_model:
            self.get_logger().warn(
                f"No available voice model found among candidates: {candidates}. Using primary path anyway; synthesis may fail."
            )
            resolved_model = primary_model
        self.model_path = resolved_model
        self.piper_bin = self._resolve_piper_bin()

        self.queue: 'queue.Queue[str]' = queue.Queue()
        self.current_proc: Optional[subprocess.Popen] = None
        self.current_wav: Optional[str] = None
        self.paused = False
        self.abandon_flag = False
        self.lock = threading.RLock()

        self.create_subscription(String, self.base_topic, self.text_cb, 10)
        self.create_subscription(String, f'{self.base_topic}/cmd', self.cmd_cb, 10)
        self.create_subscription(Empty, f'{self.base_topic}/interrupt', lambda _: self._interrupt(), 10)
        self.create_subscription(Empty, f'{self.base_topic}/resume', lambda _: self._resume(), 10)
        self.create_subscription(Empty, f'{self.base_topic}/abandon', lambda _: self._abandon(), 10)

        self.player_thread = threading.Thread(target=self._player_loop, daemon=True)
        self.player_thread.start()

        self.get_logger().info(
            f'PiperVoiceNode ready. Text: {self.base_topic} | Cmd: {self.base_topic}/cmd | '
            f'Piper: {self.piper_bin}'
        )

    def text_cb(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f'Queue: {text[:64]}...')
        self.queue.put(text)

    def cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ('interrupt', 'pause', 'stop'):
            self._interrupt()
        elif cmd in ('resume', 'continue'):
            self._resume()
        elif cmd in ('abandon', 'cancel', 'flush'):
            self._abandon()
        else:
            self.get_logger().warn(f'Unknown cmd: {cmd}')

    def _interrupt(self):
        with self.lock:
            if self.current_proc and not self.paused:
                try:
                    os.kill(self.current_proc.pid, signal.SIGSTOP)
                    self.paused = True
                    self.get_logger().info('Paused (interrupt).')
                except Exception as e:
                    self.get_logger().error(f'Interrupt failed: {e}')

    def _resume(self):
        with self.lock:
            if self.current_proc and self.paused:
                try:
                    os.kill(self.current_proc.pid, signal.SIGCONT)
                    self.paused = False
                    self.get_logger().info('Resumed.')
                except Exception as e:
                    self.get_logger().error(f'Resume failed: {e}')

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
                    self.get_logger().info('Abandon: terminated current playback and flushed queue.')
                except Exception as e:
                    self.get_logger().error(f'Abandon terminate failed: {e}')

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
        wav_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tf:
                wav_path = tf.name
            synth = subprocess.run(
                [self.piper_bin, '-m', self.model_path, '-o', wav_path],
                input=text.encode('utf-8'),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            if synth.returncode != 0 or not os.path.exists(wav_path):
                self.get_logger().error(
                    f'piper synth failed rc={synth.returncode}: '
                    f'{synth.stderr.decode(errors="ignore")[:200]}'
                )
                return

            with self.lock:
                self.current_wav = wav_path
                self.current_proc = subprocess.Popen(['aplay', '-q', wav_path])
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
                    except Exception:
                        pass
                    break
                rclpy.spin_once(self, timeout_sec=0.05)

        except Exception as e:
            self.get_logger().error(f'Playback error: {e}')
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
                cw = self.current_wav
                self.current_wav = None
            if wav_path and os.path.exists(wav_path):
                try:
                    os.unlink(wav_path)
                except Exception:
                    pass

    def _resolve_piper_bin(self) -> str:
        configured = os.environ.get('PSY_PIPER_BIN', '').strip()
        candidates = []
        if configured:
            candidates.append(configured)
        candidates.extend(['piper-tts', 'piper'])

        examined = set()
        for candidate in candidates:
            if not candidate:
                continue
            resolved = shutil.which(candidate)
            if resolved is None and os.path.isabs(candidate):
                if os.access(candidate, os.X_OK):
                    resolved = candidate
            if not resolved or resolved in examined:
                continue
            examined.add(resolved)
            try:
                with open(resolved, 'rb') as handle:
                    head = handle.read(4096)
                if b"gi.require_version('Gtk'" in head:
                    continue
            except OSError:
                pass
            return resolved

        raise RuntimeError(
            "No Piper CLI binary found. Install 'piper-tts' or set PSY_PIPER_BIN to a valid executable."
        )


def main():
    rclpy.init()
    node = PiperVoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

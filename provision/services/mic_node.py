#!/usr/bin/env python3
"""Microphone Voice Activity Detection (VAD) node.

Publishes realtime boolean voice-activity frames so downstream nodes (e.g. a
dialogue manager) can gate ASR or UI indicators. Topic naming mirrors the
`voice_node` conventions: `/voice/<host>/vad` using the same host resolution
logic ( `PSY_HOST` env or short hostname ).

Design goals:
 - Zero external runtime deps besides optional `webrtcvad` (graceful fallback)
 - Importable in lightweight test environments without audio hardware
 - Reactive publish rate ~20 Hz (configurable) with frame-based VAD
 - Minimal internal buffering; latency <= 100ms nominal

Environment variables:
 PSY_HOST                Override detected host (default: short hostname)
 PSY_MIC_SAMPLE_RATE     Sample rate (8000|16000|32000|48000) default 16000
 PSY_MIC_FRAME_MS        Frame size in ms (10|20|30) default 20
 PSY_MIC_VAD_AGGRESSIVE  webrtcvad aggressiveness (0-3) default 2
 PSY_MIC_VAD_PUBLISH_HZ  Max publish frequency (default derived from frame)
 PSY_MIC_DEVICE          ALSA device (default: default)
 PSY_MIC_DISABLE_AUDIO   If set (any value) disables real audio capture; node
                         synthesizes a silent stream for tests.

Topic:
 /voice/<host>/vad  (std_msgs/Bool) True when voice detected in the most
                    recent frame, False otherwise.
"""
from __future__ import annotations

import collections
import os
import socket
import struct
import sys
import threading
import time
from typing import Optional

try:  # pragma: no cover - optional dependency
    import webrtcvad  # type: ignore
except Exception:  # pragma: no cover - degrade gracefully
    webrtcvad = None  # type: ignore

try:  # pragma: no cover - rclpy not present in test stubs
    import rclpy
    from rclpy.node import Node
except Exception:  # pragma: no cover - lightweight test shim will inject
    rclpy = None  # type: ignore
    Node = object  # type: ignore

try:  # pragma: no cover - message import optional in tests
    from std_msgs.msg import Bool
except Exception:  # pragma: no cover
    class Bool:  # type: ignore
        __slots__ = ("data",)

        def __init__(self, data: bool = False):
            self.data = data


class _NullAudioSource:
    """Silence generator used when audio capture is disabled.

    Produces zeroed PCM16 frames at the configured frame size/rate so logic can
    proceed deterministically in tests.
    """

    def __init__(self, bytes_per_frame: int):
        self._frame = b"\x00" * bytes_per_frame

    def read(self) -> bytes:
        time.sleep(0)  # yield
        return self._frame

    def close(self) -> None:
        return None


class _AlsaPcmSource:
    """Lightweight ALSA capture using pyalsaaudio if available.

    We avoid adding a hard dependency: if import fails, we fail fast so tests
    can stub behaviour. This keeps the provisioning layer responsible for
    installing system packages.
    """

    def __init__(self, device: str, rate: int, channels: int, period_bytes: int):
        try:  # pragma: no cover - import depends on system
            import alsaaudio  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(f"pyalsaaudio not available: {exc}")

        self._pcm = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NORMAL, device=device)
        self._pcm.setchannels(channels)
        self._pcm.setrate(rate)
        self._pcm.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        self._pcm.setperiodsize(period_bytes // (channels * 2))  # frames per period

    def read(self) -> bytes:
        length, data = self._pcm.read()  # pragma: no cover - depends on hardware
        if length <= 0:
            return b""
        return data

    def close(self) -> None:  # pragma: no cover - hardware path
        try:
            self._pcm.close()
        except Exception:
            pass


class MicVADNode(Node):
    def __init__(self) -> None:  # pragma: no cover - runtime path
        super().__init__("psy_mic_vad")
        host = os.environ.get("PSY_HOST", "") or socket.gethostname().split(".")[0]
        self.base_topic = f"/voice/{host}"
        self.pub = self.create_publisher(Bool, f"{self.base_topic}/vad", 10)

        self.sample_rate = int(os.environ.get("PSY_MIC_SAMPLE_RATE", "16000"))
        self.frame_ms = int(os.environ.get("PSY_MIC_FRAME_MS", "20"))
        if self.frame_ms not in (10, 20, 30):
            self.get_logger().warn(f"Invalid PSY_MIC_FRAME_MS={self.frame_ms}; using 20")
            self.frame_ms = 20
        self.frame_bytes = int(self.sample_rate * (self.frame_ms / 1000.0) * 2)  # mono 16bit

        self.vad_publish_hz = float(
            os.environ.get("PSY_MIC_VAD_PUBLISH_HZ", str(1000.0 / self.frame_ms))
        )
        self.min_pub_interval = 1.0 / self.vad_publish_hz if self.vad_publish_hz > 0 else 0

        self._last_pub = 0.0
        self._last_state: Optional[bool] = None

        self._init_vad()
        self._init_audio()

        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"MicVADNode ready. Topic: {self.base_topic}/vad | Backend: {'webrtcvad' if self._vad_impl else 'energy'}"
        )

    # Setup helpers -----------------------------------------------------------
    def _init_vad(self) -> None:
        self._vad_impl = None
        if webrtcvad is not None:
            try:
                aggressiveness = int(os.environ.get("PSY_MIC_VAD_AGGRESSIVE", "2"))
                if aggressiveness < 0 or aggressiveness > 3:
                    aggressiveness = 2
                self._vad_impl = webrtcvad.Vad(aggressiveness)  # type: ignore
            except Exception as exc:  # pragma: no cover - optional path
                self.get_logger().warn(f"Failed to init webrtcvad: {exc}; using energy fallback")

        # Simple energy threshold fallback parameters
        self._energy_threshold = 500  # tuned conservative low amplitude
        self._energy_sustain_frames = 2
        self._energy_tail_frames = 4
        self._energy_active = 0
        self._energy_inactive = 0

    def _init_audio(self) -> None:
        if os.environ.get("PSY_MIC_DISABLE_AUDIO"):
            self._audio = _NullAudioSource(self.frame_bytes)
            return
        device = os.environ.get("PSY_MIC_DEVICE", "default")
        try:
            self._audio = _AlsaPcmSource(device, self.sample_rate, 1, self.frame_bytes)
        except Exception as exc:
            self.get_logger().warn(f"Audio capture unavailable ({exc}); using silence source")
            self._audio = _NullAudioSource(self.frame_bytes)

    # Processing loop ---------------------------------------------------------
    def _loop(self) -> None:  # pragma: no cover - runtime path
        while self._run:
            frame = self._audio.read()
            if len(frame) != self.frame_bytes:
                # pad or skip to maintain cadence
                if len(frame) < self.frame_bytes:
                    frame = frame + b"\x00" * (self.frame_bytes - len(frame))
                else:
                    frame = frame[: self.frame_bytes]
            state = self._detect_voice(frame)
            self._conditional_publish(state)
        try:
            self._audio.close()
        except Exception:  # pragma: no cover - defensive
            pass

    def _detect_voice(self, frame: bytes) -> bool:
        if self._vad_impl is not None:
            try:
                return bool(self._vad_impl.is_speech(frame, self.sample_rate))  # type: ignore
            except Exception:  # pragma: no cover - fallback safety
                pass
        # Energy fallback -----------------------------------------------------
        # Interpret PCM16 little-endian mono
        if not frame:
            return False
        count = len(frame) // 2
        # Unpack shorts; to keep cheap avoid numpy
        energy = 0
        for i in range(count):
            (sample,) = struct.unpack_from('<h', frame, i * 2)
            energy += abs(sample)
        avg = energy / max(1, count)
        active = avg > self._energy_threshold
        if active:
            self._energy_active += 1
            self._energy_inactive = 0
        else:
            self._energy_inactive += 1
            self._energy_active = 0
        if active and self._energy_active >= self._energy_sustain_frames:
            return True
        if not active and self._energy_inactive >= self._energy_tail_frames:
            return False
        # Hysteresis: retain previous
        return self._last_state if self._last_state is not None else False

    def _conditional_publish(self, state: bool) -> None:
        now = time.time()
        if state == self._last_state and (now - self._last_pub) < self.min_pub_interval:
            return
        msg = Bool(data=state)
        try:
            self.pub.publish(msg)  # type: ignore[attr-defined]
        except Exception:  # pragma: no cover - test shims may not implement
            pass
        self._last_state = state
        self._last_pub = now

    def destroy_node(self):  # pragma: no cover - runtime path
        self._run = False
        try:
            self._thread.join(timeout=0.5)
        except Exception:
            pass
        super().destroy_node()


def main():  # pragma: no cover - integration entrypoint
    if rclpy is None:
        print("rclpy not available", file=sys.stderr)
        return 1
    rclpy.init()
    node = MicVADNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())

#!/usr/bin/env python3
"""Microphone audio pipeline node.

Extends the original VAD-only service to publish the rich audio stream used by
the ROS 2 ASR stack. The node continues to provide the legacy
`/voice/<host>/vad` topic for compatibility while mirroring the state on the new
`/audio/vad` topic and exposing PCM frames plus rolling/final utterance
segments.

Design goals:
 - Zero external runtime deps besides optional `webrtcvad` (graceful fallback)
 - Importable in lightweight test environments without audio hardware
 - Reactive publish rate ~20 Hz (configurable) with frame-based VAD
 - Minimal internal buffering; latency <= 100ms nominal

Environment variables:
 PSY_HOST                          Override detected host (default: short hostname)
 PSY_MIC_SAMPLE_RATE               Sample rate (8000|16000|32000|48000) default 16000
 PSY_MIC_FRAME_MS                  Frame size in ms (10|20|30) default 20
 PSY_MIC_VAD_AGGRESSIVE            webrtcvad aggressiveness (0-3) default 2
 PSY_MIC_VAD_PUBLISH_HZ            Max publish frequency (default derived from frame)
 PSY_MIC_DEVICE                    ALSA device (default: default)
 PSY_MIC_DISABLE_AUDIO             If set (any value) disables real audio capture; node
                                   synthesizes a silent stream for tests.
 PSY_MIC_SEGMENT_MIN_SPEECH_MS     Speech duration required to open a segment (default 120)
 PSY_MIC_SEGMENT_CLOSE_SILENCE_MS  Silence duration required to close a segment (default 700)
 PSY_MIC_SEGMENT_MAX_MS            Hard cap before a timeout cut is emitted (default 15000)
 PSY_MIC_SEGMENT_CHUNK_MS          Chunk size for timeout splits (default 3000)
 PSY_MIC_SEGMENT_CURRENT_UPDATE_MS Interval between rolling chunk updates (default 200)
 PSY_MIC_SILENCE_HOLD_MS           Hold time before `/audio/is_quiet` flips true (default 300)

Topics:
 /voice/<host>/vad                 Legacy boolean VAD feed (std_msgs/Bool)
 /audio/vad                       Boolean VAD mirroring the legacy state (std_msgs/Bool)
 /audio/is_quiet                  Boolean silence detection with configurable hold (std_msgs/Bool)
 /audio/pcm                       Raw PCM frames (psyche_interfaces/AudioPCM)
 /audio/segment/current           Rolling utterance buffer (psyche_interfaces/AudioChunk)
 /audio/segment/final             Finalised utterances (psyche_interfaces/AudioSegment)
 /audio/segment/final/chunks3s    Timeout splits for long segments (psyche_interfaces/AudioChunk)
"""
from __future__ import annotations

import collections
import io
import os
import socket
import struct
import sys
import threading
import time
import wave
from typing import Deque, Iterable, List, Optional

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

try:  # pragma: no cover - optional message dependency
    from builtin_interfaces.msg import Time
except Exception:  # pragma: no cover
    class Time:  # type: ignore
        __slots__ = ("sec", "nanosec")

        def __init__(self, sec: int = 0, nanosec: int = 0):
            self.sec = sec
            self.nanosec = nanosec

try:  # pragma: no cover - optional message dependency
    from psyche_interfaces.msg import AudioChunk, AudioPCM, AudioSegment
except Exception:  # pragma: no cover
    class AudioPCM:  # type: ignore
        __slots__ = ("stamp", "sample_rate", "channels", "sample_format", "frame_index", "data")

        def __init__(self):
            self.stamp = Time()
            self.sample_rate = 0
            self.channels = 0
            self.sample_format = 0
            self.frame_index = 0
            self.data = b""

    class AudioChunk:  # type: ignore
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
            self.stamp_start = Time()
            self.stamp_end = Time()
            self.segment_id = ""
            self.sequence = 0
            self.sample_rate = 0
            self.channels = 0
            self.wav_data = b""

class AudioSegment:  # type: ignore
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
        self.stamp_start = Time()
        self.stamp_end = Time()
        self.segment_id = ""
        self.sample_rate = 0
        self.channels = 0
        self.wav_data = b""
        self.cut_by_timeout = False


def _time_from_seconds(value: float) -> Time:
    """Convert floating-point seconds to a ROS ``Time`` message."""

    secs = int(value)
    nanosec = int(round((value - secs) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        secs += 1
        nanosec -= 1_000_000_000
    return Time(sec=secs, nanosec=nanosec)


def _pcm_to_wav(data: bytes, sample_rate: int, channels: int) -> bytes:
    """Encode raw PCM16 data into a little-endian WAV container."""

    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as wav:
        wav.setnchannels(channels)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(data)
    return buffer.getvalue()


class _SegmentState:
    __slots__ = (
        "segment_id",
        "start_time",
        "end_time",
        "buffer",
        "frames",
        "sequence",
        "last_chunk_frames",
        "cut_by_timeout",
    )

    def __init__(
        self,
        *,
        segment_id: str,
        start_time: float,
        end_time: float,
        buffer: Optional[bytearray] = None,
        frames: int = 0,
        sequence: int = 0,
        last_chunk_frames: int = 0,
        cut_by_timeout: bool = False,
    ) -> None:
        self.segment_id = segment_id
        self.start_time = start_time
        self.end_time = end_time
        self.buffer = buffer if buffer is not None else bytearray()
        self.frames = frames
        self.sequence = sequence
        self.last_chunk_frames = last_chunk_frames
        self.cut_by_timeout = cut_by_timeout


class _SegmenterResult:
    __slots__ = ("chunks", "final_segment", "timeout_chunks")

    def __init__(
        self,
        *,
        chunks: Optional[List[AudioChunk]] = None,
        final_segment: Optional[AudioSegment] = None,
        timeout_chunks: Optional[List[AudioChunk]] = None,
    ) -> None:
        self.chunks = chunks or []
        self.final_segment = final_segment
        self.timeout_chunks = timeout_chunks or []


class _SilenceHold:
    """Hysteresis helper for the ``/audio/is_quiet`` boolean."""

    def __init__(self, frame_ms: int, hold_ms: int) -> None:
        self._hold_frames = max(1, int(round(hold_ms / max(1, frame_ms))))
        self._silence_frames = self._hold_frames
        self._quiet = True

    def update(self, speech: bool) -> bool:
        if speech:
            self._silence_frames = 0
            self._quiet = False
            return self._quiet
        if self._silence_frames < self._hold_frames:
            self._silence_frames += 1
        if self._silence_frames >= self._hold_frames:
            self._quiet = True
        return self._quiet


class _SegmentAssembler:
    """Incremental audio segment builder driven by VAD booleans."""

    def __init__(
        self,
        sample_rate: int,
        frame_ms: int,
        *,
        channels: int = 1,
        min_speech_ms: int = 120,
        close_silence_ms: int = 700,
        max_segment_ms: int = 15_000,
        current_update_ms: int = 200,
        timeout_chunk_ms: int = 3_000,
    ) -> None:
        self._sample_rate = sample_rate
        self._frame_ms = max(1, frame_ms)
        self._channels = channels
        self._frame_bytes = int(sample_rate * (self._frame_ms / 1000.0) * channels * 2)
        self._min_speech_frames = max(1, int(round(min_speech_ms / self._frame_ms)))
        self._close_silence_frames = max(1, int(round(close_silence_ms / self._frame_ms)))
        self._max_segment_frames = max(1, int(round(max_segment_ms / self._frame_ms)))
        self._current_update_frames = max(1, int(round(current_update_ms / self._frame_ms)))
        self._timeout_chunk_frames = max(1, int(round(timeout_chunk_ms / self._frame_ms)))

        self._speech_run = 0
        self._silence_run = 0
        self._segment_counter = 0
        self._segment: Optional[_SegmentState] = None
        self._pending: Deque[tuple[bytes, float]] = collections.deque()
        self._pending_limit = max(self._min_speech_frames, self._close_silence_frames)

    def handle_frame(self, frame: bytes, stamp: float, speech: bool) -> _SegmenterResult:
        """Consume a frame and emit any resulting messages."""

        chunks: List[AudioChunk] = []
        final_segment: Optional[AudioSegment] = None
        timeout_chunks: List[AudioChunk] = []

        if self._segment is None:
            self._pending.append((frame, stamp))
            if len(self._pending) > self._pending_limit:
                self._pending.popleft()
            if speech:
                self._speech_run += 1
            else:
                self._speech_run = 0
            if speech and self._speech_run >= self._min_speech_frames:
                start_stamp = self._pending[0][1]
                self._segment = _SegmentState(
                    segment_id=f"seg_{self._segment_counter:08d}",
                    start_time=start_stamp,
                    end_time=start_stamp,
                    buffer=bytearray(),
                    frames=0,
                    sequence=0,
                    last_chunk_frames=0,
                    cut_by_timeout=False,
                )
                self._segment_counter += 1
                while self._pending:
                    pending_frame, pending_stamp = self._pending.popleft()
                    self._append_frame(pending_frame, pending_stamp)
        else:
            self._append_frame(frame, stamp)
            if speech:
                self._speech_run += 1
                self._silence_run = 0
            else:
                self._silence_run += 1
                self._speech_run = 0

            if (
                self._segment.frames - self._segment.last_chunk_frames
                >= self._current_update_frames
            ):
                chunks.append(self._make_chunk(self._segment))
                self._segment.last_chunk_frames = self._segment.frames
                self._segment.sequence += 1

            should_timeout = self._segment.frames >= self._max_segment_frames
            should_close = (not speech and self._silence_run >= self._close_silence_frames) or should_timeout
            if should_close:
                if self._segment.frames > self._segment.last_chunk_frames:
                    chunks.append(self._make_chunk(self._segment))
                    self._segment.last_chunk_frames = self._segment.frames
                    self._segment.sequence += 1
                if should_timeout:
                    self._segment.cut_by_timeout = True
                final_segment = self._make_final(self._segment)
                if self._segment.cut_by_timeout or self._segment.frames >= self._timeout_chunk_frames:
                    timeout_chunks = list(self._make_timeout_chunks(self._segment))
                self._segment = None
                self._pending.clear()
                self._speech_run = 0
                self._silence_run = 0

        return _SegmenterResult(chunks=chunks, final_segment=final_segment, timeout_chunks=timeout_chunks)

    def _append_frame(self, frame: bytes, stamp: float) -> None:
        if not self._segment:
            return
        self._segment.buffer.extend(frame)
        self._segment.frames += 1
        if self._segment.frames == 1:
            self._segment.start_time = stamp
        self._segment.end_time = stamp

    def _make_chunk(self, state: _SegmentState) -> AudioChunk:
        chunk = AudioChunk()
        chunk.stamp_start = _time_from_seconds(state.start_time)
        chunk.stamp_end = _time_from_seconds(state.end_time)
        chunk.segment_id = state.segment_id
        chunk.sequence = state.sequence
        chunk.sample_rate = self._sample_rate
        chunk.channels = self._channels
        chunk.wav_data = _pcm_to_wav(bytes(state.buffer), self._sample_rate, self._channels)
        return chunk

    def _make_final(self, state: _SegmentState) -> AudioSegment:
        segment = AudioSegment()
        segment.stamp_start = _time_from_seconds(state.start_time)
        segment.stamp_end = _time_from_seconds(state.end_time)
        segment.segment_id = state.segment_id
        segment.sample_rate = self._sample_rate
        segment.channels = self._channels
        segment.wav_data = _pcm_to_wav(bytes(state.buffer), self._sample_rate, self._channels)
        segment.cut_by_timeout = state.cut_by_timeout
        return segment

    def _make_timeout_chunks(self, state: _SegmentState) -> Iterable[AudioChunk]:
        pcm = bytes(state.buffer)
        if not pcm:
            return []
        frames_total = state.frames
        bytes_per_frame = self._frame_bytes
        for idx, frame_start in enumerate(range(0, frames_total, self._timeout_chunk_frames)):
            frame_end = min(frames_total, frame_start + self._timeout_chunk_frames)
            start_time = state.start_time + (frame_start * self._frame_ms) / 1000.0
            end_time = state.start_time + (frame_end * self._frame_ms) / 1000.0
            chunk = AudioChunk()
            chunk.stamp_start = _time_from_seconds(start_time)
            chunk.stamp_end = _time_from_seconds(end_time)
            chunk.segment_id = state.segment_id
            chunk.sequence = idx
            chunk.sample_rate = self._sample_rate
            chunk.channels = self._channels
            start_byte = frame_start * bytes_per_frame
            end_byte = frame_end * bytes_per_frame
            chunk.wav_data = _pcm_to_wav(pcm[start_byte:end_byte], self._sample_rate, self._channels)
            yield chunk
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
        self.audio_vad_pub = self.create_publisher(Bool, "/audio/vad", 10)
        self.quiet_pub = self.create_publisher(Bool, "/audio/is_quiet", 10)
        self.pcm_pub = self.create_publisher(AudioPCM, "/audio/pcm", 10)
        self.segment_current_pub = self.create_publisher(AudioChunk, "/audio/segment/current", 10)
        self.segment_final_pub = self.create_publisher(AudioSegment, "/audio/segment/final", 10)
        self.segment_timeout_pub = self.create_publisher(
            AudioChunk, "/audio/segment/final/chunks3s", 10
        )

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
        self._frame_index = 0

        self._last_state: Optional[bool] = None
        self._last_vad_state: Optional[bool] = None
        self._last_quiet_state: Optional[bool] = None
        self._last_pub_times = {"vad": 0.0, "quiet": 0.0}

        def _safe_int(name: str, default: int) -> int:
            value = os.environ.get(name)
            if value is None:
                return default
            try:
                return int(value)
            except ValueError:
                self.get_logger().warn(f"Invalid {name}={value}; using {default}")
                return default

        min_speech_ms = _safe_int("PSY_MIC_SEGMENT_MIN_SPEECH_MS", 120)
        close_silence_ms = _safe_int("PSY_MIC_SEGMENT_CLOSE_SILENCE_MS", 700)
        max_segment_ms = _safe_int("PSY_MIC_SEGMENT_MAX_MS", 15_000)
        chunk_ms = _safe_int("PSY_MIC_SEGMENT_CHUNK_MS", 3_000)
        current_update_ms = _safe_int("PSY_MIC_SEGMENT_CURRENT_UPDATE_MS", 200)

        self._segmenter = _SegmentAssembler(
            self.sample_rate,
            self.frame_ms,
            channels=1,
            min_speech_ms=min_speech_ms,
            close_silence_ms=close_silence_ms,
            max_segment_ms=max_segment_ms,
            current_update_ms=current_update_ms,
            timeout_chunk_ms=chunk_ms,
        )
        silence_hold_ms = _safe_int("PSY_MIC_SILENCE_HOLD_MS", 300)
        self._silence_hold = _SilenceHold(self.frame_ms, silence_hold_ms)

        self._init_vad()
        self._init_audio()

        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            "MicVADNode ready. Legacy topic: %s/vad | PCM: /audio/pcm | Backend: %s"
            % (self.base_topic, "webrtcvad" if self._vad_impl else "energy")
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
            frame = bytes(frame)
            stamp = time.time()
            state = self._detect_voice(frame)
            self._last_state = state
            self._publish_vad(state)
            quiet = self._silence_hold.update(state)
            self._publish_quiet(quiet)
            self._publish_pcm(frame, stamp)
            self._process_segments(frame, stamp, state)
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

    def _publish_vad(self, state: bool) -> None:
        now = time.time()
        if state == self._last_vad_state and (now - self._last_pub_times["vad"]) < self.min_pub_interval:
            return
        try:
            self.pub.publish(Bool(data=state))  # type: ignore[attr-defined]
        except Exception:  # pragma: no cover - test shims may not implement
            pass
        try:
            self.audio_vad_pub.publish(Bool(data=state))  # type: ignore[attr-defined]
        except Exception:  # pragma: no cover - test shims may not implement
            pass
        self._last_vad_state = state
        self._last_pub_times["vad"] = now

    def _publish_quiet(self, quiet: bool) -> None:
        now = time.time()
        if quiet == self._last_quiet_state and (now - self._last_pub_times["quiet"]) < self.min_pub_interval:
            return
        try:
            self.quiet_pub.publish(Bool(data=quiet))  # type: ignore[attr-defined]
        except Exception:  # pragma: no cover - test shims may not implement
            pass
        self._last_quiet_state = quiet
        self._last_pub_times["quiet"] = now

    def _publish_pcm(self, frame: bytes, stamp: float) -> None:
        msg = AudioPCM()
        msg.stamp = _time_from_seconds(stamp)
        msg.sample_rate = self.sample_rate
        msg.channels = 1
        msg.sample_format = 1
        msg.frame_index = self._frame_index
        msg.data = bytes(frame)
        try:
            self.pcm_pub.publish(msg)  # type: ignore[attr-defined]
        except Exception:  # pragma: no cover - test shims may not implement
            pass
        self._frame_index += 1

    def _process_segments(self, frame: bytes, stamp: float, state: bool) -> None:
        result = self._segmenter.handle_frame(frame, stamp, state)
        for chunk in result.chunks:
            try:
                self.segment_current_pub.publish(chunk)  # type: ignore[attr-defined]
            except Exception:  # pragma: no cover - test shims may not implement
                pass
        if result.final_segment is not None:
            try:
                self.segment_final_pub.publish(result.final_segment)  # type: ignore[attr-defined]
            except Exception:  # pragma: no cover
                pass
        for chunk in result.timeout_chunks:
            try:
                self.segment_timeout_pub.publish(chunk)  # type: ignore[attr-defined]
            except Exception:  # pragma: no cover
                pass

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

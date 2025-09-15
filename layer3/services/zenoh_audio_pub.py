#!/usr/bin/env python3
"""Publish microphone PCM as Zenoh stream using arecord.

This tool captures mono PCM16 audio from ALSA via ``arecord`` and publishes
fixed-size chunks to a Zenoh key. It avoids non-stdlib Python audio deps and
keeps the footprint small for constrained devices like the ear (Pi Zero 2 W).

Requirements:
- ``arecord`` (alsa-utils)
- ``zenoh`` Python package (install via pip)

Example:
    $ python3 zenoh_audio_pub.py \
        --key rt/audio/ear0 \
        --rate 16000 --chunk-ms 20 \
        --device plughw:1,0

Given/When/Then (behavior):
- Given ALSA and zenoh are available, when started, then it publishes
  PCM chunks at the requested rate and size to the configured key.
"""
from __future__ import annotations

import argparse
import dataclasses
import os
import signal
import subprocess
import sys
import time
from typing import Optional


ZENOH_IMPORTED = False


@dataclasses.dataclass
class AudioConfig:
    """Configuration for audio capture and publishing.

    Attributes
    ----------
    key: Zenoh key expression to publish chunks under.
    rate: Sample rate in Hz (e.g., 16000).
    chunk_ms: Chunk duration in milliseconds (e.g., 20).
    channels: Number of channels (1=mono).
    device: ALSA device string (e.g., 'plughw:1,0').
    format: ALSA sample format (fixed 'S16_LE').
    """

    key: str = "rt/audio/ear0"
    rate: int = 16000
    chunk_ms: int = 20
    channels: int = 1
    device: str = "default"
    format: str = "S16_LE"

    @property
    def bytes_per_sample(self) -> int:
        return 2  # S16_LE

    @property
    def samples_per_chunk(self) -> int:
        return int(self.rate * (self.chunk_ms / 1000))

    @property
    def bytes_per_chunk(self) -> int:
        return self.samples_per_chunk * self.channels * self.bytes_per_sample


def build_arecord_cmd(cfg: AudioConfig) -> list[str]:
    return [
        "arecord",
        "-q",
        "-f",
        cfg.format,
        "-r",
        str(cfg.rate),
        "-c",
        str(cfg.channels),
        "-t",
        "raw",
        "-D",
        cfg.device,
    ]


def run(cfg: AudioConfig) -> int:
    global ZENOH_IMPORTED  # noqa: PLW0603
    if not ZENOH_IMPORTED:
        try:
            import zenoh  # type: ignore  # noqa: F401
        except Exception as exc:  # pragma: no cover - runtime availability
            print(f"[zenoh_audio_pub] Missing zenoh package: {exc}", file=sys.stderr)
            return 2
        else:
            ZENOH_IMPORTED = True
    import zenoh  # type: ignore  # late import for types
    session = zenoh.open(zenoh.Config())
    pub = session.declare_publisher(cfg.key)
    cmd = build_arecord_cmd(cfg)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)

    stop = False

    def _handle(sig, frame):  # noqa: ANN001 - stdlib signal handler signature
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    try:
        assert proc.stdout is not None
        chunk_size = cfg.bytes_per_chunk
        while not stop:
            buf = proc.stdout.read(chunk_size)
            if not buf:
                break
            pub.put(buf)
    finally:
        try:
            proc.terminate()
        except Exception:
            pass
        session.close()
    return 0


def parse_args(argv: Optional[list[str]] = None) -> AudioConfig:
    p = argparse.ArgumentParser()
    p.add_argument("--key", default=os.environ.get("AUDIO_KEY", "rt/audio/ear0"))
    p.add_argument("--rate", type=int, default=int(os.environ.get("AUDIO_RATE", 16000)))
    p.add_argument("--chunk-ms", type=int, default=int(os.environ.get("AUDIO_CHUNK_MS", 20)))
    p.add_argument("--channels", type=int, default=int(os.environ.get("AUDIO_CHANNELS", 1)))
    p.add_argument("--device", default=os.environ.get("AUDIO_DEVICE", "default"))
    ns = p.parse_args(argv)
    return AudioConfig(
        key=ns.key,
        rate=ns.rate,
        chunk_ms=ns.chunk_ms,
        channels=ns.channels,
        device=ns.device,
    )


def main() -> None:  # pragma: no cover - thin wrapper
    cfg = parse_args()
    sys.exit(run(cfg))


if __name__ == "__main__":
    main()

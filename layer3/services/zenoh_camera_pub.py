#!/usr/bin/env python3
"""Publish webcam JPEG frames as Zenoh stream using OpenCV.

Captures frames from a V4L2 webcam via OpenCV, encodes as JPEG, and publishes
each frame to a Zenoh key. Intended to run on the cerebellum host.

Requirements:
- ``python3-opencv`` (or ``opencv-python``)
- ``zenoh`` Python package (install via pip)

Example:
    $ python3 zenoh_camera_pub.py \
        --key rt/vision/cam0 --width 640 --height 480 --fps 15 --quality 80

Given/When/Then (behavior):
- Given OpenCV and zenoh are available, when started, then it publishes
  JPEG frames at the requested resolution/fps to the configured key.
"""
from __future__ import annotations

import argparse
import dataclasses
import os
import signal
import sys
import time
from typing import Optional

CV_IMPORTED = False
ZENOH_IMPORTED = False


@dataclasses.dataclass
class CameraConfig:
    key: str = "rt/vision/cam0"
    device_index: int = 0
    width: int = 640
    height: int = 480
    fps: int = 15
    quality: int = 80  # JPEG quality 0..100


def run(cfg: CameraConfig) -> int:
    global CV_IMPORTED, ZENOH_IMPORTED  # noqa: PLW0603
    if not CV_IMPORTED:
        try:
            import cv2  # type: ignore  # noqa: F401
        except Exception as exc:  # pragma: no cover
            print(f"[zenoh_camera_pub] Missing OpenCV: {exc}", file=sys.stderr)
            return 2
        else:
            CV_IMPORTED = True
    if not ZENOH_IMPORTED:
        try:
            import zenoh  # type: ignore  # noqa: F401
        except Exception as exc:  # pragma: no cover
            print(f"[zenoh_camera_pub] Missing zenoh package: {exc}", file=sys.stderr)
            return 2
        else:
            ZENOH_IMPORTED = True
    import cv2  # type: ignore
    import zenoh  # type: ignore
    session = zenoh.open(zenoh.Config())
    pub = session.declare_publisher(cfg.key)

    cap = cv2.VideoCapture(cfg.device_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
    cap.set(cv2.CAP_PROP_FPS, cfg.fps)

    interval = 1.0 / max(cfg.fps, 1)
    stop = False

    def _handle(sig, frame):  # noqa: ANN001
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    try:
        while not stop:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(cfg.quality)]
            )
            if not ok:
                continue
            pub.put(buf.tobytes())
            time.sleep(interval)
    finally:
        cap.release()
        session.close()
    return 0


def parse_args(argv: Optional[list[str]] = None) -> CameraConfig:
    p = argparse.ArgumentParser()
    p.add_argument("--key", default=os.environ.get("CAM_KEY", "rt/vision/cam0"))
    p.add_argument("--device", type=int, default=int(os.environ.get("CAM_DEVICE", 0)))
    p.add_argument("--width", type=int, default=int(os.environ.get("CAM_WIDTH", 640)))
    p.add_argument("--height", type=int, default=int(os.environ.get("CAM_HEIGHT", 480)))
    p.add_argument("--fps", type=int, default=int(os.environ.get("CAM_FPS", 15)))
    p.add_argument("--quality", type=int, default=int(os.environ.get("CAM_QUALITY", 80)))
    ns = p.parse_args(argv)
    return CameraConfig(
        key=ns.key,
        device_index=ns.device,
        width=ns.width,
        height=ns.height,
        fps=ns.fps,
        quality=ns.quality,
    )


def main() -> None:  # pragma: no cover
    cfg = parse_args()
    sys.exit(run(cfg))


if __name__ == "__main__":
    main()

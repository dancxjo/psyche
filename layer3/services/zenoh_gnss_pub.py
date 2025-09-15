#!/usr/bin/env python3
"""Publish NMEA sentences from a serial GNSS receiver to Zenoh.

Configuration via environment variables:
- `GNSS_DEVICE` - serial device path (default `/dev/serial0`)
- `GNSS_BAUD` - baud rate (default `9600`)
- `GNSS_KEY` - zenoh key to publish NMEA under (default `rt/gnss/ear0`)

This script keeps dependencies minimal (uses `serial` from pyserial and `zenoh`).
"""
from __future__ import annotations

import os
import sys
import time
import signal

try:
    import serial
except Exception as exc:  # pragma: no cover - runtime dependency
    print(f"[zenoh_gnss_pub] missing pyserial: {exc}", file=sys.stderr)
    raise

try:
    import zenoh
except Exception as exc:  # pragma: no cover - runtime dependency
    print(f"[zenoh_gnss_pub] missing zenoh: {exc}", file=sys.stderr)
    raise


def main() -> int:
    dev = os.environ.get("GNSS_DEVICE", "/dev/ttyACM0")
    baud = int(os.environ.get("GNSS_BAUD", "9600"))
    key = os.environ.get("GNSS_KEY", "rt/gnss/ear0")

    print(f"[zenoh_gnss_pub] opening {dev}@{baud} -> zenoh key {key}")
    try:
        ser = serial.Serial(dev, baud, timeout=1)
    except Exception as exc:
        print(f"[zenoh_gnss_pub] failed to open serial device {dev}: {exc}", file=sys.stderr)
        return 2

    session = zenoh.open(zenoh.Config())
    pub = session.declare_publisher(key)

    stop = False

    def _handle(sig, frame):  # pragma: no cover - signal handler
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    try:
        while not stop:
            try:
                line = ser.readline()
            except Exception as exc:
                print(f"[zenoh_gnss_pub] serial read error: {exc}", file=sys.stderr)
                time.sleep(1)
                continue
            if not line:
                continue
            # Normalize to str
            if isinstance(line, bytes):
                try:
                    line = line.decode(errors="ignore")
                except Exception:
                    line = str(line)
            pub.put(line)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

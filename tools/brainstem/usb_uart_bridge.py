"""Pico W USB↔UART transparent bridge for iRobot Create 1.

This MicroPython script turns a Raspberry Pi Pico W into a transparent
USB CDC to UART bridge suitable for the iRobot Create 1 Open Interface.

It forwards all bytes received from the board's USB serial (CDC) to the
hardware UART and vice versa without alteration.

Notes
-----
- Default baud for iRobot Create 1 OI is 57600 8-N-1.
- Uses UART0 on the Pico W with TX=GP0 and RX=GP1 by default.
- Designed for MicroPython on Pico W (not CPython).

Example
-------
Given a Pico W flashed with MicroPython:

1. Copy this file to the Pico as ``main.py``.
2. Wire Pico UART0 pins to the Create 1 serial port (through proper
   level shifting if required by your interface board):
   - Pico GP0 (TX) -> Robot RX
   - Pico GP1 (RX) -> Robot TX
   - Common GND between Pico and robot
3. Open the Pico's USB serial from a host at 57600 baud and send OI
   commands; the robot's responses will be relayed back.

"""
from __future__ import annotations

# MicroPython-specific modules; type: ignore used for static linters.
import sys
import time
try:
    import machine  # type: ignore
    import uselect  # type: ignore
except Exception as exc:  # pragma: no cover - MicroPython only
    raise RuntimeError("This script must run on MicroPython (Pico W)") from exc


def _setup_uart(baud: int = 57600) -> "machine.UART":
    """Configure and return UART0 for the iRobot Create 1.

    Parameters
    ----------
    baud: int
        Baud rate for the UART. Default is 57600.

    Returns
    -------
    machine.UART
        Configured UART instance.
    """
    uart = machine.UART(0, baudrate=baud, bits=8, parity=None, stop=1)
    return uart


def _setup_usb_poll() -> "uselect.poll":
    """Create a non-blocking poller for USB CDC stdin.

    Returns
    -------
    uselect.poll
        Poll object monitoring sys.stdin for readability.
    """
    poller = uselect.poll()
    # Register raw stdin stream; MicroPython maps the USB CDC to stdin/stdout
    poller.register(sys.stdin, uselect.POLLIN)
    return poller


def bridge(baud: int = 57600, usb_chunk: int = 64, uart_chunk: int = 64) -> None:
    """Run the transparent bridge loop.

    Parameters
    ----------
    baud: int
        UART baud rate. Defaults to 57600 for Create 1.
    usb_chunk: int
        Max bytes to read from USB per cycle.
    uart_chunk: int
        Max bytes to read from UART per cycle.
    """
    uart = _setup_uart(baud)
    poller = _setup_usb_poll()

    usb_in = sys.stdin.buffer
    usb_out = sys.stdout.buffer

    # Small sleep to allow host to open the CDC endpoint after reset
    time.sleep(0.2)

    while True:
        # UART -> USB
        try:
            n = uart.any()
            if n:
                data = uart.read(min(n, uart_chunk))
                if data:
                    usb_out.write(data)
                    usb_out.flush()
        except Exception:
            # Avoid crashing on transient UART issues
            pass

        # USB -> UART (non-blocking via poll)
        try:
            if poller.poll(0):  # readable
                data = usb_in.read(usb_chunk)
                if data:
                    uart.write(data)
        except Exception:
            # Ignore USB read/write hiccups, continue bridging
            pass

        # Yield a tiny slice to avoid hogging CPU
        time.sleep(0.001)


def main() -> None:  # pragma: no cover - runtime entry for MicroPython
    bridge()


if __name__ == "__main__":  # pragma: no cover
    main()


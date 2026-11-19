#!/usr/bin/env python3
"""
Ubuntu Bluetooth Serial Logger & Terminal (RFCOMM)

Connects to an ESP32 Bluetooth Classic SPP device, prints incoming data to the
terminal, exports all received data to a file in real time, and automatically
reconnects if the connection drops or the ESP32 reboots.

Setup (once):
  bluetoothctl pair <ESP32_MAC>
  bluetoothctl trust <ESP32_MAC>

Usage:
  python3 bt_serial_terminal.py --mac AA:BB:CC:DD:EE:FF
  python3 bt_serial_terminal.py --mac AA:BB:CC:DD:EE:FF -o run1.txt --timestamps
"""

import argparse
import signal
import socket
import sys
import time
from datetime import datetime


def timestamp_prefix() -> str:
    return datetime.now().strftime("[%H:%M:%S.%f")[:-3] + "] "


def run_logger(
        mac: str,
        channel: int,
        out_path: str,
        timestamps: bool,
        reconnect_delay: float,
) -> None:
    running = True

    def handle_sigint(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)

    at_line_start = True

    with open(out_path, "a", encoding="utf-8", buffering=1) as out_file:
        print(f"[BT] Exporting data to: {out_path}")

        while running:
            sock = None
            try:
                print(f"[BT] Connecting to {mac} (channel {channel})...")
                sock = socket.socket(
                    socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM
                )
                sock.settimeout(5.0)
                sock.connect((mac, channel))
                sock.settimeout(1.0)
                print("[BT] Connected. Press Ctrl+C to stop and save.")

                while running:
                    try:
                        data = sock.recv(1024)
                    except socket.timeout:
                        continue

                    if not data:
                        raise ConnectionResetError("Remote device closed connection")

                    text = data.decode("utf-8", errors="replace")
                    if timestamps:
                        parts = []
                        for ch in text:
                            if at_line_start and ch not in ("\r", "\n"):
                                parts.append(timestamp_prefix())
                                at_line_start = False
                            parts.append(ch)
                            if ch == "\n":
                                at_line_start = True
                        text = "".join(parts)

                    sys.stdout.write(text)
                    sys.stdout.flush()
                    out_file.write(text)
                    out_file.flush()

            except (OSError, ConnectionError, TimeoutError) as exc:
                if not running:
                    break
                print(
                    f"\n[BT] Connection interrupted ({exc}). "
                    f"Reconnecting in {reconnect_delay:.1f}s..."
                )
                time.sleep(reconnect_delay)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass

    print(f"\n[BT] Stopped. Saved output to {out_path}")


def main() -> None:
    default_out = f"bt_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

    parser = argparse.ArgumentParser(
        description="Auto-reconnecting Bluetooth SPP terminal & file exporter for Ubuntu"
    )
    parser.add_argument(
        "--mac",
        required=True,
        help="Bluetooth MAC address of the ESP32 (e.g. AA:BB:CC:DD:EE:FF)",
    )
    parser.add_argument(
        "-o",
        "--out",
        default=default_out,
        help=f"Output file path (default: {default_out})",
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=1,
        help="RFCOMM channel (default: 1 for ESP32 BluetoothSerial)",
    )
    parser.add_argument(
        "--timestamps",
        action="store_true",
        help="Prefix each line with a local timestamp",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=2.0,
        help="Seconds to wait before reconnecting after an interruption (default: 2.0)",
    )
    args = parser.parse_args()

    if not hasattr(socket, "AF_BLUETOOTH"):
        sys.exit(
            "Error: This Python build does not support AF_BLUETOOTH. "
            "On Ubuntu, use the system python3 (/usr/bin/python3)."
        )

    run_logger(
        mac=args.mac,
        channel=args.channel,
        out_path=args.out,
        timestamps=args.timestamps,
        reconnect_delay=args.reconnect_delay,
    )


if __name__ == "__main__":
    main()
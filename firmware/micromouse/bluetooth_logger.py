#!/usr/bin/env python3
"""
Ubuntu Bluetooth Serial Logger & Dual-Encoder Plotter (RFCOMM)

Usage:
  python3 bt_serial_terminal.py --mac AA:BB:CC:DD:EE:FF --plot
  python3 bt_serial_terminal.py --mac AA:BB:CC:DD:EE:FF --live -o run1.txt
"""

import argparse
import signal
import socket
import sys
import time
from collections import deque
from datetime import datetime


def timestamp_prefix() -> str:
    return datetime.now().strftime("[%H:%M:%S.%f")[:-3] + "] "


def parse_csv_line(line: str):
    parts = line.strip().split(",")
    if len(parts) != 5:
        return None
    try:
        t_ms = float(parts[0])
        raw_l = float(parts[1])
        filt_l = float(parts[2])
        raw_r = float(parts[3])
        filt_r = float(parts[4])
        return t_ms, raw_l, filt_l, raw_r, filt_r
    except ValueError:
        return None


def make_plots(live: bool):
    import matplotlib.pyplot as plt

    if live:
        plt.ion()
    fig, (ax_l, ax_r) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    fig.suptitle("Encoder Velocity: Raw vs. Filtered")

    (line_raw_l,) = ax_l.plot([], [], alpha=0.45, linewidth=1.0, label="Left Raw (m/s)")
    (line_flt_l,) = ax_l.plot([], [], linewidth=2.0, label="Left Filtered (m/s)")
    ax_l.set_ylabel("Left Vel (m/s)")
    ax_l.grid(True, alpha=0.3)
    ax_l.legend(loc="upper right")

    (line_raw_r,) = ax_r.plot([], [], alpha=0.45, linewidth=1.0, label="Right Raw (m/s)")
    (line_flt_r,) = ax_r.plot([], [], linewidth=2.0, label="Right Filtered (m/s)")
    ax_r.set_xlabel("Time (s)")
    ax_r.set_ylabel("Right Vel (m/s)")
    ax_r.grid(True, alpha=0.3)
    ax_r.legend(loc="upper right")

    plt.tight_layout()
    return plt, fig, ax_l, ax_r, line_raw_l, line_flt_l, line_raw_r, line_flt_r


def run_logger(
        mac: str,
        channel: int,
        out_path: str,
        timestamps: bool,
        reconnect_delay: float,
        plot_on_exit: bool,
        live_plot: bool,
        live_window: int,
) -> None:
    running = True

    def handle_sigint(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)

    at_line_start = True
    line_buf = ""
    t0_ms = None

    # Full-run buffers for final plot
    all_t, all_rl, all_fl, all_rr, all_fr = [], [], [], [], []
    # Rolling buffers for live plot
    win_t = deque(maxlen=live_window)
    win_rl = deque(maxlen=live_window)
    win_fl = deque(maxlen=live_window)
    win_rr = deque(maxlen=live_window)
    win_fr = deque(maxlen=live_window)

    plt_ctx = make_plots(live=True) if live_plot else None
    last_draw = time.monotonic()

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
                sock.settimeout(0.1 if live_plot else 1.0)
                print("[BT] Connected. Press Ctrl+C to stop and save.")

                while running:
                    try:
                        data = sock.recv(1024)
                    except socket.timeout:
                        if live_plot and plt_ctx:
                            plt_ctx[0].pause(0.001)
                        continue

                    if not data:
                        raise ConnectionResetError("Remote device closed connection")

                    raw_text = data.decode("utf-8", errors="replace")

                    # Parse complete lines for plotting
                    if plot_on_exit or live_plot:
                        line_buf += raw_text
                        while "\n" in line_buf:
                            one_line, line_buf = line_buf.split("\n", 1)
                            parsed = parse_csv_line(one_line)
                            if parsed is not None:
                                t_ms, rl, fl, rr, fr = parsed
                                if t0_ms is None:
                                    t0_ms = t_ms
                                t_s = (t_ms - t0_ms) / 1000.0
                                all_t.append(t_s)
                                all_rl.append(rl)
                                all_fl.append(fl)
                                all_rr.append(rr)
                                all_fr.append(fr)
                                if live_plot:
                                    win_t.append(t_s)
                                    win_rl.append(rl)
                                    win_fl.append(fl)
                                    win_rr.append(rr)
                                    win_fr.append(fr)

                    if live_plot and plt_ctx and (time.monotonic() - last_draw) >= 0.05 and win_t:
                        plt, fig, ax_l, ax_r, l_rl, l_fl, l_rr, l_fr = plt_ctx
                        t_list = list(win_t)
                        l_rl.set_data(t_list, list(win_rl))
                        l_fl.set_data(t_list, list(win_fl))
                        l_rr.set_data(t_list, list(win_rr))
                        l_fr.set_data(t_list, list(win_fr))
                        ax_l.relim()
                        ax_l.autoscale_view()
                        ax_r.relim()
                        ax_r.autoscale_view()
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()
                        last_draw = time.monotonic()

                    text = raw_text
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

    if (plot_on_exit or live_plot) and all_t:
        if plt_ctx is None:
            plt_ctx = make_plots(live=False)
        plt, fig, ax_l, ax_r, l_rl, l_fl, l_rr, l_fr = plt_ctx
        if live_plot:
            plt.ioff()
        l_rl.set_data(all_t, all_rl)
        l_fl.set_data(all_t, all_fl)
        l_rr.set_data(all_t, all_rr)
        l_fr.set_data(all_t, all_fr)
        ax_l.relim()
        ax_l.autoscale_view()
        ax_r.relim()
        ax_r.autoscale_view()
        print(f"[PLOT] Showing full capture ({len(all_t)} samples)...")
        plt.show()


def main() -> None:
    default_out = f"bt_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

    parser = argparse.ArgumentParser(
        description="Auto-reconnecting Bluetooth SPP terminal, exporter, and encoder plotter"
    )
    parser.add_argument("--mac", required=True, help="Bluetooth MAC address")
    parser.add_argument("-o", "--out", default=default_out, help=f"Output file path (default: {default_out})")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM channel (default: 1)")
    parser.add_argument("--timestamps", action="store_true", help="Prefix each line with a local timestamp")
    parser.add_argument("--reconnect-delay", type=float, default=2.0, help="Reconnect delay in seconds")
    parser.add_argument("--plot", action="store_true", help="Plot full run after pressing Ctrl+C")
    parser.add_argument("--live", action="store_true", help="Live-plot rolling window while receiving")
    parser.add_argument("--window", type=int, default=400, help="Samples shown in live window (default: 400 = 2s @ 200Hz)")
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
        plot_on_exit=args.plot,
        live_plot=args.live,
        live_window=args.window,
    )


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""Giam sat thoi gian thuc + gui lenh cho firmware lab3_pid_motor.

- Doc dong CSV "t_ms,mode,sp,rpm,u,d" tu Serial, ve live-plot bang matplotlib.
- O nhap lenh o cuoi cua so: go lenh (vd: "kp 0.4", "sp 80") roi Enter.
- Toan bo du lieu va lenh duoc ghi vao file CSV trong thu muc logs/.

Chay:  python3 pid_live.py            (tu tim cong; hoac --port /dev/ttyUSB0)
"""
import argparse
import collections
import datetime
import os
import threading
import time

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=10.0, help="so giay hien thi tren do thi")
    args = ap.parse_args()

    os.makedirs("logs", exist_ok=True)
    fname = datetime.datetime.now().strftime("logs/live_%Y%m%d_%H%M%S.csv")
    fout = open(fname, "w", buffering=1)
    fout.write("t_ms,mode,sp,rpm,u,d\n")
    print(f"Ghi du lieu vao {fname}")

    from lab3_link import _open, find_port
    ser = _open(find_port(args.port), args.baud)
    time.sleep(0.3)
    ser.reset_input_buffer()

    n = int(args.window / 0.01) + 10
    buf = {k: collections.deque(maxlen=n) for k in ("t", "sp", "rpm", "u")}
    lock = threading.Lock()
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            if line.startswith("#"):
                print(line)
                fout.write(line + "\n")
                continue
            parts = line.split(",")
            if len(parts) not in (5, 6):
                continue
            try:
                t, mode, sp, rpm, u = (float(p) for p in parts[:5])
            except ValueError:
                continue
            fout.write(line + "\n")
            with lock:
                buf["t"].append(t / 1000.0)
                buf["sp"].append(sp if mode == 2 else float("nan"))
                buf["rpm"].append(rpm)
                buf["u"].append(u)

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    fig.subplots_adjust(bottom=0.15)
    l_sp, = ax1.plot([], [], "--", label="setpoint (rpm)")
    l_rpm, = ax1.plot([], [], label="toc do do (rpm)")
    l_u, = ax2.plot([], [], color="tab:green", label="u (% duty)")
    ax1.set_ylabel("rpm"); ax1.grid(True); ax1.legend(loc="upper left")
    ax2.set_ylabel("u (%)"); ax2.set_xlabel("t (s)"); ax2.grid(True); ax2.set_ylim(-5, 105)
    ax2.legend(loc="upper left")

    box_ax = fig.add_axes([0.12, 0.03, 0.6, 0.05])
    box = TextBox(box_ax, "Lenh: ")

    def on_submit(text):
        text = text.strip()
        if not text:
            return
        ser.write((text + "\n").encode("ascii"))
        fout.write(f"# cmd {time.time():.3f} {text}\n")
        print(f"> {text}")
        box.set_val("")

    box.on_submit(on_submit)

    def update(_):
        with lock:
            t = list(buf["t"]); sp = list(buf["sp"]); rpm = list(buf["rpm"]); u = list(buf["u"])
        if not t:
            return l_sp, l_rpm, l_u
        l_sp.set_data(t, sp); l_rpm.set_data(t, rpm); l_u.set_data(t, u)
        ax1.set_xlim(max(t[0], t[-1] - args.window), t[-1] + 0.1)
        finite = [v for v in rpm + sp if v == v]
        lo, hi = min(finite), max(finite)
        pad = max(5.0, 0.1 * (hi - lo))
        ax1.set_ylim(lo - pad, hi + pad)
        return l_sp, l_rpm, l_u

    _anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop.set()
        try:
            ser.write(b"stop\n")
        except serial.SerialException:
            pass
        th.join(timeout=1)
        ser.close()
        fout.close()
        print(f"Da luu {fname}")


if __name__ == "__main__":
    main()

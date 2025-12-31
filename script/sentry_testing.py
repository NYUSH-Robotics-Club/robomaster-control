#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import re
from collections import deque

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ================= CONFIG =================
PORT = "/dev/tty.modem3088"
BAUD = 115200
BUFFER = 600
MAX_MOTORS = 4          # set to your s_steer_motor_count
WINDOW_SEC = 30.0
# ==========================================

# Matches:
# [STR] i=0 id=1 tgt=123.4 ang=4567 spd=12.3 dt=10 out=8000
STR_RE = re.compile(
    r"^\[STR\]\s+i=(\d+)\s+id=(\d+)\s+tgt=([-+]?\d+(?:\.\d+)?)\s+"
    r"ang=(\d+)\s+spd=([-+]?\d+(?:\.\d+)?)\s+dt=(\d+)\s+out=([-+]?\d+)"
)

def parse_str_line(line: str):
    m = STR_RE.match(line.strip())
    if not m:
        return None
    idx = int(m.group(1))
    mid = int(m.group(2))
    tgt = float(m.group(3))
    ang = int(m.group(4))   # encoder ticks (uint)
    return idx, mid, tgt, ang

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"[OK] Opened {PORT} @ {BAUD}")

    t = [deque(maxlen=BUFFER) for _ in range(MAX_MOTORS)]
    tgt = [deque(maxlen=BUFFER) for _ in range(MAX_MOTORS)]
    ang = [deque(maxlen=BUFFER) for _ in range(MAX_MOTORS)]
    motor_id = ["?"] * MAX_MOTORS

    start = None
    raw_print_count = 0

    fig, axes = plt.subplots(MAX_MOTORS, 1, figsize=(12, 3 * MAX_MOTORS), sharex=True)
    if MAX_MOTORS == 1:
        axes = [axes]

    lines_tgt = []
    lines_ang = []

    for i in range(MAX_MOTORS):
        axes[i].grid(True)
        axes[i].set_ylabel(f"i={i}")
        lt, = axes[i].plot([], [], "--", label="target")
        la, = axes[i].plot([], [], label="angle")
        axes[i].legend(loc="upper right")
        lines_tgt.append(lt)
        lines_ang.append(la)

    axes[-1].set_xlabel("time (s)")

    def update(_frame):
        nonlocal start, raw_print_count

        while ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()

            # show first few raw lines for sanity
            if raw_print_count < 20 and line:
                print("[RAW]", line)
                raw_print_count += 1

            parsed = parse_str_line(line)
            if not parsed:
                continue

            idx, mid, _tgt, _ang = parsed
            if idx < 0 or idx >= MAX_MOTORS:
                continue

            if start is None:
                start = time.time()

            tt = time.time() - start

            motor_id[idx] = str(mid)
            t[idx].append(tt)
            tgt[idx].append(_tgt)
            ang[idx].append(_ang)

        any_last = False
        for i in range(MAX_MOTORS):
            if not t[i]:
                continue

            lines_tgt[i].set_data(t[i], tgt[i])
            lines_ang[i].set_data(t[i], ang[i])

            axes[i].relim()
            axes[i].autoscale_view()
            axes[i].set_ylabel(f"i={i} id={motor_id[i]}")
            any_last = True

        if any_last:
            xmax = max(t[i][-1] for i in range(MAX_MOTORS) if t[i])
            xmin = max(0.0, xmax - WINDOW_SEC)
            axes[-1].set_xlim(xmin, xmax)

        fig.suptitle("Steer Motors – Target vs Current Angle")
        return lines_tgt + lines_ang

    ani = FuncAnimation(fig, update, interval=100, blit=False)

    try:
        plt.tight_layout()
        plt.show()
    finally:
        ser.close()
        print("Serial closed")

if __name__ == "__main__":
    main()

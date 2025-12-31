#!/usr/bin/env python3
import time
import re
from collections import deque

import serial
from serial.tools import list_ports

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

BAUD = 115200
BUFFER = 600
MAX_MOTORS = 8          # change if you might have more than 8 steer motors
WINDOW_SEC = 30.0       # show last 30s on x-axis
PORT_SUBSTR = "tty.usbmodem"

# Matches:
#   [STR] i=0 id=1 tgt=123.4 ang=4567 spd=12.3 dt=10 out=8000
STR_RE = re.compile(
    r"^\[STR\]\s+i=(\d+)\s+id=(\d+)\s+tgt=([-+]?\d+(?:\.\d+)?)\s+"
    r"ang=(\d+)\s+spd=([-+]?\d+(?:\.\d+)?)\s+dt=(\d+)\s+out=([-+]?\d+)\s*$"
)

def autodetect_port():
    ports = list(list_ports.comports())
    # prefer usbmodem ports
    for p in ports:
        dev = p.device or ""
        if PORT_SUBSTR in dev:
            return dev
    # fallback: try any port with usbmodem in description/hwid
    for p in ports:
        text = f"{p.device} {p.description} {p.hwid}"
        if PORT_SUBSTR in text:
            return p.device
    return None

def parse_str_line(line: str):
    m = STR_RE.match(line.strip())
    if not m:
        return None
    idx = int(m.group(1))
    mid = int(m.group(2))
    tgt = float(m.group(3))
    ang = int(m.group(4))      # encoder ticks (uint)
    # spd = float(m.group(5))  # not used for this plot
    # dt  = int(m.group(6))    # not used for this plot
    # out = int(m.group(7))    # not used for this plot
    return idx, mid, tgt, ang

def ensure_buffers(n, t, tgt, ang, motor_id):
    while len(t) < n:
        t.append(deque(maxlen=BUFFER))
        tgt.append(deque(maxlen=BUFFER))
        ang.append(deque(maxlen=BUFFER))
        motor_id.append("?")

def main():
    port = autodetect_port()
    if not port:
        raise RuntimeError(
            f"Could not find a serial port containing '{PORT_SUBSTR}'.\n"
            f"Plug in the device and check `ls /dev/tty.*`."
        )

    ser = serial.Serial(port, BAUD, timeout=0.1)
    print(f"[OK] Opened {port} @ {BAUD}")

    # dynamic buffers per motor index
    t = []
    tgt = []
    ang = []
    motor_id = []

    start = None
    raw_print_count = 0

    # Set up figure with MAX_MOTORS subplots; we will only show those that receive data.
    fig, axes = plt.subplots(MAX_MOTORS, 1, figsize=(12, 2.2 * MAX_MOTORS), sharex=True)
    if MAX_MOTORS == 1:
        axes = [axes]

    line_tgt = []
    line_ang = []

    for i in range(MAX_MOTORS):
        axes[i].grid(True)
        axes[i].set_ylabel(f"i={i}")
        lt, = axes[i].plot([], [], linestyle="--", label="tgt")
        la, = axes[i].plot([], [], label="ang")
        axes[i].legend(loc="upper right")
        line_tgt.append(lt)
        line_ang.append(la)

    axes[-1].set_xlabel("time (s)")

    def update(_frame):
        nonlocal start, raw_print_count

        # read all pending lines
        while ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()

            # print a few raw lines so you can confirm it’s receiving
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

            ensure_buffers(idx + 1, t, tgt, ang, motor_id)

            motor_id[idx] = str(mid)
            t[idx].append(tt)
            tgt[idx].append(_tgt)
            ang[idx].append(_ang)

        # update plots
        any_last = []
        for i in range(MAX_MOTORS):
            if i >= len(t) or not t[i]:
                continue

            line_tgt[i].set_data(t[i], tgt[i])
            line_ang[i].set_data(t[i], ang[i])

            axes[i].relim()
            axes[i].autoscale_view()
            axes[i].set_ylabel(f"i={i} id={motor_id[i]}")

            any_last.append(t[i][-1])

        # x window last WINDOW_SEC seconds
        if any_last:
            xmax = max(any_last)
            xmin = max(0.0, xmax - WINDOW_SEC)
            axes[-1].set_xlim(xmin, xmax)

        fig.suptitle("Steer Motors: Target vs Current Angle ([STR] lines)")
        return line_tgt + line_ang

    ani = FuncAnimation(fig, update, interval=100, blit=False)

    try:
        plt.tight_layout()
        plt.show()
    finally:
        ser.close()
        print("Serial closed")

if __name__ == "__main__":
    main()

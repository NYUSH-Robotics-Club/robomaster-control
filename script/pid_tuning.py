#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ultra-Fast Minimal Yaw Plotter
Plots only:
 - target_angle
 - current_angle
 - cmd (motor command)
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

DEFAULT_PORT = '/dev/tty.usbmodem3064356030341'
DEFAULT_BAUD = 115200
BUFFER_SIZE = 800       # keep last N points
CSV_PREFIX = "YAW_CSV"

class MinimalYawPlotter:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate

        self.ser = None
        self.start_ts = None

        # Buffers
        self.t = deque(maxlen=BUFFER_SIZE)
        self.target = deque(maxlen=BUFFER_SIZE)
        self.current = deque(maxlen=BUFFER_SIZE)
        self.cmd = deque(maxlen=BUFFER_SIZE)

        # Figure
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle("Minimal Yaw Plotter (Fast)", fontsize=14)

        # Lines
        self.line_targ, = self.ax1.plot([], [], 'b-', lw=1.5, label="Target Angle")
        self.line_curr, = self.ax1.plot([], [], 'r-', lw=1.5, label="Current Angle")
        self.line_cmd,  = self.ax2.plot([], [], 'm-', lw=1.5, label="Command")

        # Axis titles
        self.ax1.set_title("Target vs Current")
        self.ax1.set_ylabel("Angle")
        self.ax1.grid(True)
        self.ax1.legend()

        self.ax2.set_title("Command Output")
        self.ax2.set_ylabel("Cmd")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.grid(True)
        self.ax2.legend()

        plt.tight_layout()

    # -------------------------
    # Serial Setup
    # -------------------------
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
            print(f"[OK] Connected to {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print("[ERROR] Could not open serial:", e)
            return False

    # -------------------------
    # CSV Parsing
    # -------------------------
    def parse_line(self, line):
        if not line.startswith(CSV_PREFIX):
            return None

        parts = line.strip().split(',')
        if len(parts) < 6:
            return None

        try:
            ts = int(parts[1])
            target = float(parts[2])
            current = float(parts[3])
            speed = int(parts[4])        # ignored
            cmd = float(parts[5])
        except:
            return None

        return ts, target, current, cmd

    # -------------------------
    # Read Serial (Non-blocking)
    # -------------------------
    def read_serial(self):
        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline().decode("utf-8", errors="ignore")
                parsed = self.parse_line(raw)

                if parsed:
                    ts, target, current, cmd = parsed

                    if self.start_ts is None:
                        self.start_ts = ts

                    t = (ts - self.start_ts) / 1000.0

                    self.t.append(t)
                    self.target.append(target)
                    self.current.append(current)
                    self.cmd.append(cmd)

        except Exception as e:
            print("Serial read error:", e)

    # -------------------------
    # Plot Update
    # -------------------------
    def update(self, frame):
        self.read_serial()
        if not self.t:
            return

        t = list(self.t)
        self.line_targ.set_data(t, list(self.target))
        self.line_curr.set_data(t, list(self.current))
        self.line_cmd.set_data(t, list(self.cmd))

        # time window: last 10s
        xmin = max(0, t[-1] - 10)
        xmax = t[-1] + 0.1

        self.ax1.set_xlim(xmin, xmax)
        self.ax2.set_xlim(xmin, xmax)

        # auto y-scale
        if len(self.target) > 1:
            lo = min(min(self.target), min(self.current))
            hi = max(max(self.target), max(self.current))
            d = (hi - lo) * 0.2 if hi != lo else 1
            self.ax1.set_ylim(lo - d, hi + d)

        if len(self.cmd) > 1:
            lo = min(self.cmd)
            hi = max(self.cmd)
            d = (hi - lo) * 0.2 if hi != lo else 100
            self.ax2.set_ylim(lo - d, hi + d)

    # -------------------------
    # Run
    # -------------------------
    def run(self):
        if not self.connect_serial():
            return

        print("[READY] Waiting for YAW_CSV data...")
        ani = animation.FuncAnimation(self.fig, self.update, interval=40, blit=False)

        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("[CLOSED] Serial port closed.")

def main():
    port = DEFAULT_PORT
    baud = DEFAULT_BAUD

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baud = int(sys.argv[2])

    plot = MinimalYawPlotter(port, baud)
    plot.run()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time Yaw Monitor (Simple 4-parameter version)
Reads CSV from serial: YAW_CSV,timestamp,yaw_raw,vision_yaw,yaw_rate
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

DEFAULT_PORT = 'COM6'
DEFAULT_BAUD = 115200
BUFFER_SIZE = 1000


class YawSimplePlotter:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        # Buffers
        self.time_data = deque(maxlen=BUFFER_SIZE)
        self.yaw_raw = deque(maxlen=BUFFER_SIZE)
        self.vision_yaw = deque(maxlen=BUFFER_SIZE)
        self.yaw_rate = deque(maxlen=BUFFER_SIZE)

        self.start_time = None

        # 2x2 layout
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Yaw Monitor (Raw / Vision / Rate)', fontsize=14, fontweight='bold')

        self.ax_raw = self.axes[0, 0]
        self.ax_vision = self.axes[0, 1]
        self.ax_rate = self.axes[1, 0]
        self.ax_empty = self.axes[1, 1]
        self.ax_empty.axis('off')

        # Plot lines
        self.line_raw, = self.ax_raw.plot([], [], label='yaw_raw', linewidth=1.5)
        self.line_vision, = self.ax_vision.plot([], [], label='vision_yaw', linewidth=1.5)
        self.line_rate, = self.ax_rate.plot([], [], label='yaw_rate', linewidth=1.5)

        # Axis settings
        settings = [
            (self.ax_raw, "IMU Raw Yaw", "Yaw (deg)"),
            (self.ax_vision, "Vision Yaw", "Yaw (deg)"),
            (self.ax_rate, "Yaw Rate", "deg/s"),
        ]

        for ax, title, ylabel in settings:
            ax.set_title(title)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(ylabel)
            ax.grid(True)
            ax.legend()

        plt.tight_layout()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error: {e}")
            return False

    def parse_csv_line(self, line):
        """
        Format:
        YAW_CSV,timestamp,yaw_raw,vision_yaw,yaw_rate
        """
        if not line.startswith("YAW_CSV"):
            return None

        parts = line.strip().split(',')
        if len(parts) < 5:
            return None

        try:
            return {
                'timestamp': int(parts[1]),
                'yaw_raw': float(parts[2]),
                'vision_yaw': float(parts[3]),
                'yaw_rate': float(parts[4]),
            }
        except ValueError:
            return None

    def read_serial_data(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore')

                data = self.parse_csv_line(line)
                if data:
                    if self.start_time is None:
                        self.start_time = data['timestamp']

                    time_s = (data['timestamp'] - self.start_time) / 1000.0

                    self.time_data.append(time_s)
                    self.yaw_raw.append(data['yaw_raw'])
                    self.vision_yaw.append(data['vision_yaw'])
                    self.yaw_rate.append(data['yaw_rate'])

        except Exception as e:
            print(f"Serial read error: {e}")

    def update_plot(self, frame):
        self.read_serial_data()
        if not self.time_data:
            return

        t = list(self.time_data)

        self.line_raw.set_data(t, list(self.yaw_raw))
        self.line_vision.set_data(t, list(self.vision_yaw))
        self.line_rate.set_data(t, list(self.yaw_rate))

        # Auto-scale
        for ax in [self.ax_raw, self.ax_vision, self.ax_rate]:
            ax.relim()
            ax.autoscale_view()

        return [self.line_raw, self.line_vision, self.line_rate]

    def run(self):
        if not self.connect_serial():
            return

        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        plt.show()

        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial closed.")


def main():
    port = DEFAULT_PORT
    baud = DEFAULT_BAUD

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baud = int(sys.argv[2])

    plotter = YawSimplePlotter(port, baud)
    plotter.run()


if __name__ == "__main__":
    main()

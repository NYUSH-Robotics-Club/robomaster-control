#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time Yaw Motor Data Plotter
Reads CSV data from serial port and plots in real-time
Usage: python plot_yaw_data.py [COM_PORT] [BAUD_RATE]
Example: python plot_yaw_data.py COM3 115200
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

# Default serial port settings
DEFAULT_PORT = 'COM10'
DEFAULT_BAUD = 115200

# Data buffer size (number of points to keep)
BUFFER_SIZE = 1000

# CSV data format: YAW_CSV,timestamp,target_angle,current_angle,speed_rpm,cmd,rate_input,joy_smoothed,error,g_gz,c_gz

class YawDataPlotter:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
        # Data buffers
        self.time_data = deque(maxlen=BUFFER_SIZE)
        self.target_angle = deque(maxlen=BUFFER_SIZE)
        self.current_angle = deque(maxlen=BUFFER_SIZE)
        self.speed_rpm = deque(maxlen=BUFFER_SIZE)
        self.cmd = deque(maxlen=BUFFER_SIZE)
        self.rate_input = deque(maxlen=BUFFER_SIZE)
        self.joy_smoothed = deque(maxlen=BUFFER_SIZE)
        self.error = deque(maxlen=BUFFER_SIZE)
        self.g_gz = deque(maxlen=BUFFER_SIZE)
        self.c_gz = deque(maxlen=BUFFER_SIZE)
        
        # Time reference
        self.start_time = None
        
        # Setup figure and subplots
        self.fig, self.axes = plt.subplots(3, 2, figsize=(14, 10))
        self.fig.suptitle('Yaw Motor Real-time Data', fontsize=14, fontweight='bold')
        
        # Configure subplots
        self.ax1 = self.axes[0, 0]  # Angle comparison
        self.ax2 = self.axes[0, 1]  # Speed
        self.ax3 = self.axes[1, 0]  # Command output
        self.ax4 = self.axes[1, 1]  # Input rate
        self.ax5 = self.axes[2, 0]  # Error
        self.ax6 = self.axes[2, 1]  # Gyro data
        
        # Initialize plot lines
        self.line1_target, = self.ax1.plot([], [], 'b-', label='Target Angle', linewidth=1.5)
        self.line1_current, = self.ax1.plot([], [], 'r-', label='Current Angle', linewidth=1.5)
        self.line2, = self.ax2.plot([], [], 'g-', label='Speed (RPM)', linewidth=1.5)
        self.line3, = self.ax3.plot([], [], 'm-', label='Command', linewidth=1.5)
        self.line4_input, = self.ax4.plot([], [], 'c-', label='Rate Input', linewidth=1.5)
        self.line4_smoothed, = self.ax4.plot([], [], 'y-', label='Joy Smoothed', linewidth=1.5)
        self.line5, = self.ax5.plot([], [], 'orange', label='Error', linewidth=1.5)
        self.line6_g, = self.ax6.plot([], [], 'purple', label='g_gz', linewidth=1.5)
        self.line6_c, = self.ax6.plot([], [], 'brown', label='c_gz', linewidth=1.5)
        
        # Configure axes
        self.ax1.set_title('Angle (Target vs Current)')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Angle')
        self.ax1.legend()
        self.ax1.grid(True)
        
        self.ax2.set_title('Motor Speed')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('RPM')
        self.ax2.legend()
        self.ax2.grid(True)
        
        self.ax3.set_title('Control Command')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Current')
        self.ax3.legend()
        self.ax3.grid(True)
        
        self.ax4.set_title('Input Rate')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Normalized Rate')
        self.ax4.legend()
        self.ax4.grid(True)
        
        self.ax5.set_title('Angle Error')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Error')
        self.ax5.legend()
        self.ax5.grid(True)
        
        self.ax6.set_title('Gyro Data')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Angular Velocity')
        self.ax6.legend()
        self.ax6.grid(True)
        
        plt.tight_layout()
        
    def connect_serial(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return False
    
    def parse_csv_line(self, line):
        """Parse CSV line and extract data"""
        # Look for YAW_CSV prefix
        if not line.startswith('YAW_CSV'):
            return None
        
        try:
            # Remove YAW_CSV prefix and split by comma
            parts = line.strip().split(',')
            if len(parts) < 11:
                return None
            
            # Extract data (skip first element which is 'YAW_CSV')
            timestamp = int(parts[1])
            target_angle = float(parts[2])
            current_angle = float(parts[3])
            speed_rpm = int(parts[4])
            cmd = float(parts[5])
            rate_input = float(parts[6])
            joy_smoothed = float(parts[7])
            error = float(parts[8])
            g_gz = float(parts[9])
            c_gz = float(parts[10])
            
            return {
                'timestamp': timestamp,
                'target_angle': target_angle,
                'current_angle': current_angle,
                'speed_rpm': speed_rpm,
                'cmd': cmd,
                'rate_input': rate_input,
                'joy_smoothed': joy_smoothed,
                'error': error,
                'g_gz': g_gz,
                'c_gz': c_gz
            }
        except (ValueError, IndexError) as e:
            return None
    
    def read_serial_data(self):
        """Read and parse data from serial port"""
        if self.ser is None or not self.ser.is_open:
            return
        
        try:
            # Read available data
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore')
                data = self.parse_csv_line(line)
                
                if data is not None:
                    # Initialize start time
                    if self.start_time is None:
                        self.start_time = data['timestamp']
                    
                    # Calculate relative time in seconds
                    rel_time = (data['timestamp'] - self.start_time) / 1000.0
                    
                    # Add to buffers
                    self.time_data.append(rel_time)
                    self.target_angle.append(data['target_angle'])
                    self.current_angle.append(data['current_angle'])
                    self.speed_rpm.append(data['speed_rpm'])
                    self.cmd.append(data['cmd'])
                    self.rate_input.append(data['rate_input'])
                    self.joy_smoothed.append(data['joy_smoothed'])
                    self.error.append(data['error'])
                    self.g_gz.append(data['g_gz'])
                    self.c_gz.append(data['c_gz'])
        except Exception as e:
            print(f"Error reading serial data: {e}")
    
    def update_plot(self, frame):
        """Update plot with new data"""
        # Read new data from serial
        self.read_serial_data()
        
        if len(self.time_data) == 0:
            return
        
        # Convert deques to lists for plotting
        time_list = list(self.time_data)
        
        # Update plots
        self.line1_target.set_data(time_list, list(self.target_angle))
        self.line1_current.set_data(time_list, list(self.current_angle))
        self.line2.set_data(time_list, list(self.speed_rpm))
        self.line3.set_data(time_list, list(self.cmd))
        self.line4_input.set_data(time_list, list(self.rate_input))
        self.line4_smoothed.set_data(time_list, list(self.joy_smoothed))
        self.line5.set_data(time_list, list(self.error))
        self.line6_g.set_data(time_list, list(self.g_gz))
        self.line6_c.set_data(time_list, list(self.c_gz))
        
        # Auto-scale axes
        if len(time_list) > 0:
            time_min = max(0, time_list[-1] - 10)  # Show last 10 seconds
            time_max = time_list[-1] + 1
            
            self.ax1.set_xlim(time_min, time_max)
            self.ax2.set_xlim(time_min, time_max)
            self.ax3.set_xlim(time_min, time_max)
            self.ax4.set_xlim(time_min, time_max)
            self.ax5.set_xlim(time_min, time_max)
            self.ax6.set_xlim(time_min, time_max)
            
            # Auto-scale y-axes
            if len(self.target_angle) > 0:
                angle_min = min(min(self.target_angle), min(self.current_angle))
                angle_max = max(max(self.target_angle), max(self.current_angle))
                margin = (angle_max - angle_min) * 0.1
                self.ax1.set_ylim(angle_min - margin, angle_max + margin)
            
            if len(self.speed_rpm) > 0:
                speed_min = min(self.speed_rpm)
                speed_max = max(self.speed_rpm)
                margin = (speed_max - speed_min) * 0.1 if speed_max != speed_min else 10
                self.ax2.set_ylim(speed_min - margin, speed_max + margin)
            
            if len(self.cmd) > 0:
                cmd_min = min(self.cmd)
                cmd_max = max(self.cmd)
                margin = (cmd_max - cmd_min) * 0.1 if cmd_max != cmd_min else 1000
                self.ax3.set_ylim(cmd_min - margin, cmd_max + margin)
            
            if len(self.rate_input) > 0:
                rate_min = min(min(self.rate_input), min(self.joy_smoothed))
                rate_max = max(max(self.rate_input), max(self.joy_smoothed))
                margin = (rate_max - rate_min) * 0.1 if rate_max != rate_min else 0.1
                self.ax4.set_ylim(rate_min - margin, rate_max + margin)
            
            if len(self.error) > 0:
                err_min = min(self.error)
                err_max = max(self.error)
                margin = (err_max - err_min) * 0.1 if err_max != err_min else 10
                self.ax5.set_ylim(err_min - margin, err_max + margin)
            
            if len(self.g_gz) > 0:
                gyro_min = min(min(self.g_gz), min(self.c_gz))
                gyro_max = max(max(self.g_gz), max(self.c_gz))
                margin = (gyro_max - gyro_min) * 0.1 if gyro_max != gyro_min else 0.1
                self.ax6.set_ylim(gyro_min - margin, gyro_max + margin)
        
        return [self.line1_target, self.line1_current, self.line2, self.line3,
                self.line4_input, self.line4_smoothed, self.line5, self.line6_g, self.line6_c]
    
    def run(self):
        """Start the plotter"""
        if not self.connect_serial():
            return
        
        print("Starting real-time plotter. Close the window to stop.")
        print("Waiting for data...")
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nStopping plotter...")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("Serial port closed.")

def main():
    # Parse command line arguments
    port = DEFAULT_PORT
    baudrate = DEFAULT_BAUD
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])
    
    print(f"Yaw Motor Data Plotter")
    print(f"Port: {port}, Baudrate: {baudrate}")
    print(f"Looking for CSV data with prefix 'YAW_CSV'")
    print()
    
    plotter = YawDataPlotter(port, baudrate)
    plotter.run()

if __name__ == '__main__':
    main()


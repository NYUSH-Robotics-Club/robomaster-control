#!/usr/bin/env python3
"""
CAN Log Parser for RoboMaster Control Firmware

Reads serial output or log files and extracts/analyzes CAN TX/RX messages.
Usage:
    python parse_can_logs.py /dev/ttyACM0  # Read from serial port
    python parse_can_logs.py logfile.txt   # Read from log file
"""

import sys
import re
import serial
import argparse
from collections import defaultdict
from datetime import datetime


class CANLogParser:
    def __init__(self):
        self.rx_count = defaultdict(int)
        self.tx_count = defaultdict(int)
        self.error_count = 0

    def parse_line(self, line):
        """Parse a single line and extract CAN message if present"""
        line = line.strip()

        # Match CAN RX pattern: [CAN_RX] CH=%d ID=0x%03X DLC=%d DATA=%02X %02X ...
        rx_match = re.match(
            r'\[CAN_RX\]\s+CH=(\d+)\s+ID=0x([0-9A-Fa-f]{3})\s+DLC=(\d+)\s+DATA=([0-9A-Fa-f\s]+)',
            line
        )
        if rx_match:
            channel = int(rx_match.group(1))
            can_id = int(rx_match.group(2), 16)
            dlc = int(rx_match.group(3))
            data_str = rx_match.group(4).split()
            data = [int(b, 16) for b in data_str[:dlc]]

            self.rx_count[(channel, can_id)] += 1
            self.print_rx_message(channel, can_id, dlc, data)
            return 'rx'

        # Match CAN TX pattern: [CAN_TX] CH=%d ID=0x%03X DLC=%d DATA=%02X %02X ... STATUS=%s
        tx_match = re.match(
            r'\[CAN_TX\]\s+CH=(\d+)\s+ID=0x([0-9A-Fa-f]{3})\s+DLC=(\d+)\s+DATA=([0-9A-Fa-f\s]+)\s+STATUS=(\w+)',
            line
        )
        if tx_match:
            channel = int(tx_match.group(1))
            can_id = int(tx_match.group(2), 16)
            dlc = int(tx_match.group(3))
            data_str = tx_match.group(4).split()
            data = [int(b, 16) for b in data_str[:dlc]]
            status = tx_match.group(5)

            self.tx_count[(channel, can_id)] += 1
            if status != 'OK':
                self.error_count += 1

            self.print_tx_message(channel, can_id, dlc, data, status)
            return 'tx'

        return None

    def print_rx_message(self, channel, can_id, dlc, data):
        """Print formatted RX message with motor type interpretation"""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        motor_type = self.identify_motor_type(can_id)

        print(f"[{timestamp}] RX | CAN{channel} | 0x{can_id:03X} | {motor_type:8s} | ", end='')

        if len(data) >= 8:
            # Parse motor feedback
            angle = (data[0] << 8) | data[1]
            speed = (data[2] << 8) | data[3]
            if speed > 32767:
                speed -= 65536  # Convert to signed
            current = (data[4] << 8) | data[5]
            if current > 32767:
                current -= 65536
            temp = data[6] if len(data) > 6 else 0

            print(f"Angle={angle:5d} Speed={speed:6d} Current={current:6d} Temp={temp:3d}°C")
        else:
            data_hex = ' '.join([f'{b:02X}' for b in data])
            print(f"Data: {data_hex}")

    def print_tx_message(self, channel, can_id, dlc, data, status):
        """Print formatted TX message with current values"""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        frame_type = self.identify_frame_type(can_id)
        status_icon = '✓' if status == 'OK' else '✗'

        print(f"[{timestamp}] TX | CAN{channel} | 0x{can_id:03X} | {frame_type:8s} | {status_icon} | ", end='')

        if len(data) >= 8:
            # Parse motor currents (4 motors per frame)
            currents = []
            for i in range(4):
                current = (data[i*2] << 8) | data[i*2+1]
                if current > 32767:
                    current -= 65536  # Convert to signed
                currents.append(current)

            current_str = ' '.join([f'{c:6d}' for c in currents])
            print(f"Currents: [{current_str}]")
        else:
            data_hex = ' '.join([f'{b:02X}' for b in data])
            print(f"Data: {data_hex}")

    def identify_motor_type(self, can_id):
        """Identify motor type from CAN RX ID"""
        if 0x201 <= can_id <= 0x208:
            return "M3508"
        elif 0x205 <= can_id <= 0x20B:
            return "GM6020"
        else:
            return "Unknown"

    def identify_frame_type(self, can_id):
        """Identify TX frame type"""
        if can_id == 0x200:
            return "M3508-1"  # Motors 1-4
        elif can_id == 0x1FF:
            return "Mixed"     # M3508 5-8 or GM6020 1-4
        elif can_id == 0x2FF:
            return "GM6020-2"  # GM6020 5-7
        else:
            return "Unknown"

    def print_statistics(self):
        """Print message statistics"""
        print("\n" + "="*80)
        print("CAN Message Statistics")
        print("="*80)

        print("\nRX Messages:")
        for (channel, can_id), count in sorted(self.rx_count.items()):
            motor_type = self.identify_motor_type(can_id)
            print(f"  CAN{channel} | 0x{can_id:03X} ({motor_type:8s}): {count:6d} messages")

        print("\nTX Messages:")
        for (channel, can_id), count in sorted(self.tx_count.items()):
            frame_type = self.identify_frame_type(can_id)
            print(f"  CAN{channel} | 0x{can_id:03X} ({frame_type:8s}): {count:6d} messages")

        if self.error_count > 0:
            print(f"\nTransmit Errors: {self.error_count}")

        total_rx = sum(self.rx_count.values())
        total_tx = sum(self.tx_count.values())
        print(f"\nTotal RX: {total_rx} | Total TX: {total_tx}")
        print("="*80 + "\n")


def read_serial(port, baudrate=115200):
    """Read and parse CAN logs from serial port"""
    print(f"Opening serial port {port} at {baudrate} baud...")
    parser = CANLogParser()

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print("Listening for CAN messages (Ctrl+C to stop)...\n")

            while True:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore')
                    if line:
                        parser.parse_line(line)
                except UnicodeDecodeError:
                    continue

    except KeyboardInterrupt:
        print("\n\nStopped by user")
        parser.print_statistics()
    except Exception as e:
        print(f"Error: {e}")


def read_file(filename):
    """Read and parse CAN logs from log file"""
    print(f"Reading log file {filename}...")
    parser = CANLogParser()

    try:
        with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                parser.parse_line(line)

        parser.print_statistics()

    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
    except Exception as e:
        print(f"Error: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='Parse CAN messages from RoboMaster control firmware',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s /dev/ttyACM0              # Read from serial port (Linux)
  %(prog)s COM3                       # Read from serial port (Windows)
  %(prog)s -b 921600 /dev/ttyUSB0     # Custom baud rate
  %(prog)s logfile.txt                # Read from log file
        """
    )

    parser.add_argument('source', help='Serial port or log file path')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Serial port baud rate (default: 115200)')

    args = parser.parse_args()

    # Detect if source is a file or serial port
    if args.source.startswith('/dev/') or args.source.startswith('COM'):
        read_serial(args.source, args.baudrate)
    else:
        read_file(args.source)


if __name__ == '__main__':
    main()

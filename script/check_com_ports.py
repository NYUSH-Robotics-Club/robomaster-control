#!/usr/bin/env python3
"""
check_com_ports.py

List all available COM ports and check if they can be opened.
"""

import serial.tools.list_ports
import serial
import sys

print("=" * 60)
print("Available COM Ports:")
print("=" * 60)

ports = list(serial.tools.list_ports.comports())
if not ports:
    print("No COM ports found!")
    print("\nTroubleshooting:")
    print("1. Check if STM32 is connected via USB")
    print("2. Check Device Manager for 'Ports (COM & LPT)'")
    print("3. Try unplugging and replugging USB cable")
    sys.exit(1)

for port in ports:
    print(f"\n{port.device}:")
    print(f"  Description: {port.description}")
    print(f"  Hardware ID: {port.hwid}")
    
    # Try to open the port to check if it's available
    try:
        ser = serial.Serial(port.device, timeout=0.1)
        ser.close()
        print(f"  Status: [OK] Available (can be opened)")
    except serial.SerialException as e:
        print(f"  Status: [ERROR] {e}")

print("\n" + "=" * 60)
print("To use a port, run:")
print("  python script/cmd_vel_forwarder.py --port <COM_PORT> --vx 0.5 --rate 20")
print("=" * 60)

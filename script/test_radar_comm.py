#!/usr/bin/env python3
"""
Radar command test script for STM32 firmware.

Sends velocity commands (vx, vy, wz) in the protocol format:
  [SYNC1=0xA5] [SYNC2=0x5A] [vx:float] [vy:float] [wz:float] [CRC8:1B]
  Total: 14 bytes

Usage:
  python3 test_radar_comm.py /dev/ttyACM0 115200
"""

import serial
import struct
import time
import sys
import argparse


def crc8(data):
    """Calculate CRC8 checksum using polynomial 0x07."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def encode_radar_cmd(vx, vy, wz):
    """
    Encode radar command into frame format.
    
    Args:
        vx (float): X velocity in m/s
        vy (float): Y velocity in m/s
        wz (float): Angular velocity in rad/s
    
    Returns:
        bytes: 14-byte frame ready to send
    """
    # Build frame: [SYNC1][SYNC2][vx:4B][vy:4B][wz:4B]
    frame = bytearray([0xA5, 0x5A])
    frame += struct.pack('<f', vx)  # Little-endian float
    frame += struct.pack('<f', vy)
    frame += struct.pack('<f', wz)
    
    # Calculate and append CRC8 (over first 13 bytes)
    crc = crc8(frame[:13])
    frame.append(crc)
    
    return bytes(frame)


def send_radar_cmd(port, vx, vy, wz, count=1, interval=0.1):
    """
    Send radar commands to serial port.
    
    Args:
        port (serial.Serial): Open serial port
        vx (float): X velocity in m/s
        vy (float): Y velocity in m/s
        wz (float): Angular velocity in rad/s
        count (int): Number of frames to send
        interval (float): Interval between sends (seconds)
    """
    frame = encode_radar_cmd(vx, vy, wz)
    print(f"Frame bytes: {frame.hex()}")
    print(f"Sending {count} frame(s)...")
    
    for i in range(count):
        port.write(frame)
        print(f"  Sent {i+1}/{count}: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
        if i < count - 1:
            time.sleep(interval)


def test_sequence(port, duration=10):
    """
    Send a test sequence of velocity commands.
    
    Args:
        port (serial.Serial): Open serial port
        duration (float): Total duration in seconds
    """
    print(f"\n=== Test Sequence ({duration}s) ===\n")
    
    # Test 1: Forward motion
    print("Test 1: Forward (vx=0.5 m/s)")
    start = time.time()
    while time.time() - start < 2:
        send_radar_cmd(port, 0.5, 0.0, 0.0, count=1, interval=0.0)
        time.sleep(0.05)  # 20 Hz
    
    # Test 2: Strafe left
    print("\nTest 2: Strafe Left (vy=0.5 m/s)")
    start = time.time()
    while time.time() - start < 2:
        send_radar_cmd(port, 0.0, 0.5, 0.0, count=1, interval=0.0)
        time.sleep(0.05)
    
    # Test 3: Rotation
    print("\nTest 3: Rotation (wz=0.5 rad/s)")
    start = time.time()
    while time.time() - start < 2:
        send_radar_cmd(port, 0.0, 0.0, 0.5, count=1, interval=0.0)
        time.sleep(0.05)
    
    # Test 4: Combined motion
    print("\nTest 4: Combined (vx=0.3, vy=0.2, wz=0.1)")
    start = time.time()
    while time.time() - start < 2:
        send_radar_cmd(port, 0.3, 0.2, 0.1, count=1, interval=0.0)
        time.sleep(0.05)
    
    # Test 5: Stop
    print("\nTest 5: Stop")
    send_radar_cmd(port, 0.0, 0.0, 0.0, count=5, interval=0.05)
    
    print("\n=== Test Complete ===\n")


def main():
    parser = argparse.ArgumentParser(
        description="Test radar communication with STM32 firmware"
    )
    parser.add_argument("port", help="Serial port (e.g., /dev/ttyACM0 or COM3)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate")
    parser.add_argument("--vx", type=float, default=0.5, help="X velocity (m/s)")
    parser.add_argument("--vy", type=float, default=0.0, help="Y velocity (m/s)")
    parser.add_argument("--wz", type=float, default=0.0, help="Angular velocity (rad/s)")
    parser.add_argument("--count", type=int, default=1, help="Number of frames to send")
    parser.add_argument("--interval", type=float, default=0.1, help="Interval between frames (s)")
    parser.add_argument("--test-seq", action="store_true", help="Run test sequence")
    parser.add_argument("--duration", type=float, default=10, help="Test sequence duration (s)")
    
    args = parser.parse_args()
    
    try:
        print(f"Opening {args.port} at {args.baudrate} baud...")
        port = serial.Serial(args.port, args.baudrate, timeout=1)
        time.sleep(0.5)  # Wait for port to stabilize
        print("Port opened successfully.\n")
        
        if args.test_seq:
            test_sequence(port, args.duration)
        else:
            send_radar_cmd(port, args.vx, args.vy, args.wz, args.count, args.interval)
        
        port.close()
        print("Port closed.")
    
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        if 'port' in locals():
            port.close()


if __name__ == "__main__":
    main()

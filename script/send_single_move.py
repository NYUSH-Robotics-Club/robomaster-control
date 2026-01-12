#!/usr/bin/env python3
"""
send_single_move.py

发送一次速度命令，然后自动停止。
用于测试底盘移动一小段距离。

Usage:
  python3 send_single_move.py --port COM9 --vx 0.2 --duration 0.5
  # 以0.2 m/s的速度移动0.5秒，然后停止
"""

import argparse
import serial
import struct
import time
import sys

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def encode_radar_cmd(vx: float, vy: float, wz: float) -> bytes:
    frame = bytearray([0xA5, 0x5A])
    frame += struct.pack('<f', vx)
    frame += struct.pack('<f', vy)
    frame += struct.pack('<f', wz)
    crc = crc8(frame[:14])
    frame.append(crc)
    return bytes(frame)

def main():
    parser = argparse.ArgumentParser(description='Send a single velocity command for a short duration')
    parser.add_argument('--port', required=True, help='Serial port (e.g., COM9 or /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--vx', type=float, default=0.2, help='X velocity (m/s)')
    parser.add_argument('--vy', type=float, default=0.0, help='Y velocity (m/s)')
    parser.add_argument('--wz', type=float, default=0.0, help='Angular velocity (rad/s)')
    parser.add_argument('--duration', type=float, default=0.5, help='Duration in seconds')
    parser.add_argument('--rate', type=float, default=20.0, help='Send rate (Hz)')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
        print(f"Connected to {args.port}")
        time.sleep(0.2)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        sys.exit(1)

    interval = 1.0 / max(1.0, args.rate)
    num_frames = int(args.duration * args.rate)
    
    print(f"Sending {num_frames} frames at {args.rate} Hz")
    print(f"Command: vx={args.vx:.3f} vy={args.vy:.3f} wz={args.wz:.3f}")
    print(f"Duration: {args.duration:.2f} seconds")
    print("Starting...")

    try:
        # 发送速度命令
        for i in range(num_frames):
            frame = encode_radar_cmd(args.vx, args.vy, args.wz)
            ser.write(frame)
            if i % 10 == 0:  # 每10帧打印一次
                print(f"  Frame {i+1}/{num_frames}")
            time.sleep(interval)
        
        # 发送停止命令（0速度）
        print("\nStopping...")
        for i in range(int(0.2 * args.rate)):  # 发送0.2秒的停止命令确保停止
            frame = encode_radar_cmd(0.0, 0.0, 0.0)
            ser.write(frame)
            time.sleep(interval)
        
        print("Done!")
        
    except KeyboardInterrupt:
        print("\nInterrupted, sending stop command...")
        for _ in range(10):
            frame = encode_radar_cmd(0.0, 0.0, 0.0)
            ser.write(frame)
            time.sleep(0.05)
    finally:
        ser.close()

if __name__ == '__main__':
    main()

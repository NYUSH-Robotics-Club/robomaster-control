#!/usr/bin/env python3
"""
radar_diagnostics.py - Diagnose radar data reception and mode switching issues
"""

import serial
import struct
import time
import sys
from datetime import datetime

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

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 radar_diagnostics.py /dev/ttyACM0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
        print(f"[INFO] Connected to {port}")
        time.sleep(0.5)
    except Exception as e:
        print(f"[ERROR] Failed to open {port}: {e}")
        sys.exit(1)
    
    print("\n[INFO] Monitoring radar reception...")
    print("[INFO] Press Ctrl+C to stop\n")
    print("TIME            | FRAME # | TYPE    | DATA                           | STATUS")
    print("-" * 100)
    
    frame_count = 0
    radar_frames = 0
    text_lines = 0
    crc_errors = 0
    last_radar_time = 0
    
    buffer = bytearray()
    
    try:
        while True:
            # Read from serial
            byte = ser.read(1)
            if not byte:
                continue
            
            buffer.extend(byte)
            
            # Look for radar frame sync
            if len(buffer) >= 2 and buffer[-2] == 0xA5 and buffer[-1] == 0x5A:
                # Potential radar frame start
                if len(buffer) >= 15:
                    frame = bytes(buffer[-15:])
                    
                    # Extract and verify CRC
                    crc_expected = frame[14]
                    crc_calc = crc8(frame[:14])
                    
                    if crc_calc == crc_expected:
                        # Valid frame!
                        vx, vy, wz = struct.unpack('<fff', frame[2:14])
                        radar_frames += 1
                        last_radar_time = time.time()
                        
                        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        print(f"{now} | {radar_frames:6d} | RADAR   | vx={vx:+.3f} vy={vy:+.3f} wz={vy:+.3f} | ✓ OK")
                    else:
                        crc_errors += 1
                        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        print(f"{now} | {'---':6s} | RADAR   | CRC FAIL (calc={crc_calc:02x} exp={crc_expected:02x})         | ✗ ERROR")
                    
                    # Clear old buffer
                    if len(buffer) > 20:
                        buffer = buffer[-15:]
            
            # Look for text lines (STM32 logs)
            if byte == b'\n':
                line = buffer.decode('ascii', errors='ignore').rstrip()
                if line and not (line[0:2] == '\xa5\x5a'):
                    text_lines += 1
                    now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    print(f"{now} | {'---':6s} | LOG     | {line[:50]:50s} | -")
                buffer = bytearray()
    
    except KeyboardInterrupt:
        print("\n\n[INFO] Stopped by user")
    finally:
        ser.close()
        
        print("\n" + "="*100)
        print(f"[SUMMARY]")
        print(f"  Radar frames received: {radar_frames}")
        print(f"  CRC errors: {crc_errors}")
        if radar_frames > 0:
            print(f"  CRC error rate: {100*crc_errors/(radar_frames+crc_errors):.1f}%")
        print(f"  Text lines: {text_lines}")
        print(f"  Last radar frame: {time.time() - last_radar_time:.1f}s ago")

if __name__ == '__main__':
    main()

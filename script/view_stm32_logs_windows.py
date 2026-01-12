#!/usr/bin/env python3
"""
view_stm32_logs_windows.py

View STM32 logs from USB CDC on Windows, filtering out binary data.
Usage: python view_stm32_logs_windows.py COM3
"""

import serial
import sys
import time

def is_printable_text(data):
    """Check if data contains mostly printable ASCII characters"""
    if not data:
        return False
    
    # Count printable ASCII (0x20-0x7E) and common control chars (\r, \n, \t)
    printable_count = sum(1 for b in data if (32 <= b <= 126) or b in (9, 10, 13))
    ratio = printable_count / len(data) if len(data) > 0 else 0
    
    # Consider it text if >80% is printable
    return ratio > 0.8

def filter_binary_data(data):
    """Filter out binary radar/vision frames"""
    # Radar frame: starts with 0xA5 0x5A (15 bytes total)
    if len(data) >= 2:
        if data[0] == 0xA5 and data[1] == 0x5A:
            return None  # Radar frame, skip
        # Check for other binary patterns
        if data[0] < 0x20 and data[0] not in (0x09, 0x0A, 0x0D):
            if len(data) > 10:  # Likely binary data
                return None
    
    return data

def main():
    if len(sys.argv) < 2:
        print("Usage: python view_stm32_logs_windows.py <COM_PORT>")
        print("  Example: python view_stm32_logs_windows.py COM3")
        print("\nTo find COM port:")
        print("  1. Open Device Manager")
        print("  2. Expand 'Ports (COM & LPT)'")
        print("  3. Look for 'STM32 Virtual COM Port' or similar")
        print("  4. Note the COM number (e.g., COM3)")
        sys.exit(1)
    
    port = sys.argv[1]
    show_raw = '--raw' in sys.argv
    filter_binary = '--no-filter' not in sys.argv
    
    try:
        # Open serial port
        ser = serial.Serial(
            port=port,
            baudrate=115200,  # USB CDC ignores this, but set it anyway
            timeout=1.0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        print(f"Connected to {port}")
        print("Press Ctrl+C to exit\n")
        print("=" * 60)
        
        buffer = bytearray()
        
        while True:
            # Read available data
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer.extend(chunk)
                
                # Process complete lines
                while b'\n' in buffer or b'\r' in buffer:
                    # Find line ending
                    if b'\n' in buffer:
                        line_end = buffer.index(b'\n')
                    elif b'\r' in buffer:
                        line_end = buffer.index(b'\r')
                    else:
                        break
                    
                    # Extract line (including \r\n)
                    line = bytes(buffer[:line_end + 1])
                    buffer = buffer[line_end + 1:]
                    
                    # Skip empty lines
                    if not line.strip():
                        continue
                    
                    # Filter binary data
                    if filter_binary:
                        filtered = filter_binary_data(line)
                        if filtered is None:
                            continue  # Skip binary data
                        line = filtered
                    
                    # Check if it's printable text
                    if is_printable_text(line):
                        try:
                            # Try to decode as ASCII
                            text = line.decode('ascii', errors='replace')
                            # Remove replacement characters and control chars (except \r\n\t)
                            text = ''.join(c if (32 <= ord(c) <= 126) or c in '\r\n\t' else '' for c in text)
                            if text.strip():
                                print(text, end='', flush=True)
                        except:
                            pass
                    elif show_raw:
                        # Show raw hex for debugging
                        hex_str = ' '.join(f'{b:02X}' for b in line[:50])
                        print(f"[RAW] {hex_str}")
            else:
                # No data, small sleep to avoid CPU spinning
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\n\nExiting...")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print("\nTroubleshooting:")
        print("1. Check if COM port exists in Device Manager")
        print("2. Make sure no other program is using the port")
        print("3. Try a different COM port number")
        print("4. Unplug and replug USB cable")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Extract tagged spin-mode debug lines from:
  - a serial port (USB CDC), or
  - a log file / stdin

Tags:
  - SPINDBG,...  (from CmdController)
  - SPINGIM,...  (from Gimbal controller)
"""

import sys
import time
import argparse
from datetime import datetime


def find_serial_port():
    import serial.tools.list_ports

    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usbmodem" in port.device or "ttyACM" in port.device:
            return port.device
    return None


def iter_lines_from_file(path: str):
    if path == "-":
        for line in sys.stdin:
            yield line.rstrip("\n")
        return
    with open(path, "r", errors="ignore") as f:
        for line in f:
            yield line.rstrip("\n")


def iter_lines_from_serial(port: str, baud: int):
    import serial

    ser = serial.Serial(port, baud, timeout=0.2)
    time.sleep(0.2)
    buf = ""
    try:
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    yield line.strip()
            else:
                time.sleep(0.01)
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description="Extract SPINDBG/SPINGIM lines to CSV")
    src = ap.add_mutually_exclusive_group(required=False)
    src.add_argument("--file", default=None, help="Log file path; use '-' for stdin")
    src.add_argument("--port", default=None, help="Serial port, e.g. /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200, help="Serial baudrate (default 115200)")
    ap.add_argument("--out", default=None, help="Output CSV file (default: stdout)")
    ap.add_argument("--tag", choices=["SPINDBG", "SPINGIM", "BOTH"], default="BOTH", help="Which tag to keep")
    args = ap.parse_args()

    if args.file is None and args.port is None:
        # Default to serial auto-detect (same heuristic as other scripts)
        auto = find_serial_port()
        if auto is None:
            print("ERROR: no --file/--port and no USB CDC serial port found", file=sys.stderr)
            sys.exit(2)
        args.port = auto

    out_f = open(args.out, "w") if args.out else sys.stdout
    try:
        # Write header as comment so it won't break existing plotters
        out_f.write(f"# extracted_at={datetime.now().isoformat()} tag={args.tag}\n")
        if args.tag in ("SPINDBG", "BOTH"):
            out_f.write("# SPINDBG,ts_ms,spin,sw0,sw1,c_yaw_deg,g_yaw_total_deg,hold_yaw_deg,yaw_err_deg,yaw_rate_cmd,vx_cmd,vy_cmd,wz_cmd\n")
        if args.tag in ("SPINGIM", "BOTH"):
            out_f.write("# SPINGIM,ts_ms,enabled,vision,spin_hold,yaw_rate_cmd,yaw_raw,yaw_target,motor_rpm,imu_gyro_rpm,yaw_current_cmd\n")
        out_f.flush()

        if args.file is not None:
            lines = iter_lines_from_file(args.file)
        else:
            lines = iter_lines_from_serial(args.port, args.baud)

        for line in lines:
            if not line:
                continue
            if args.tag == "SPINDBG" and not line.startswith("SPINDBG,"):
                continue
            if args.tag == "SPINGIM" and not line.startswith("SPINGIM,"):
                continue
            if args.tag == "BOTH" and not (line.startswith("SPINDBG,") or line.startswith("SPINGIM,")):
                continue
            out_f.write(line + "\n")
            out_f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if args.out:
            out_f.close()


if __name__ == "__main__":
    main()


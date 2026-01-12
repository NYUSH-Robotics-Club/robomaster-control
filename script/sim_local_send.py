#!/usr/bin/env python3
"""
Simple local simulator: imports `encode_radar_cmd` from `cmd_vel_forwarder.py`,
prints hex frames and appends them to `sim_serial_dump.bin` to simulate serial output.
"""
import argparse
import importlib.util
import sys
import time
from pathlib import Path

# Load cmd_vel_forwarder as a module
from pathlib import Path
repo_root = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("cmd_vel_forwarder", str(repo_root / "script" / "cmd_vel_forwarder.py"))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

encode = getattr(mod, "encode_radar_cmd")

parser = argparse.ArgumentParser()
parser.add_argument('--vx', type=float, default=0.5)
parser.add_argument('--vy', type=float, default=0.0)
parser.add_argument('--wz', type=float, default=0.0)
parser.add_argument('--count', type=int, default=5)
parser.add_argument('--interval', type=float, default=0.1)
parser.add_argument('--out', type=str, default='script/sim_serial_dump.bin')
args = parser.parse_args()

outpath = Path(args.out)
# Ensure output directory
outpath.parent.mkdir(parents=True, exist_ok=True)

with outpath.open('ab') as f:
    for i in range(args.count):
        frame = encode(args.vx, args.vy, args.wz)
        print(f"Frame {i+1}/{args.count}: {frame.hex()}")
        f.write(frame)
        time.sleep(args.interval)

print(f"Wrote {args.count} frames to {outpath}")

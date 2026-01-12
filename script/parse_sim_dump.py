#!/usr/bin/env python3
"""
Parse and decode script/sim_serial_dump.bin created by sim_local_send.py
"""
import struct

FILE = 'script/sim_serial_dump.bin'
SYNC1 = 0xA5
SYNC2 = 0x5A
FRAME_SIZE = 15  # 2 + 12 + 1


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


def find_frames(buf: bytes):
    i = 0
    frames = []
    while i + 1 < len(buf):
        if buf[i] == SYNC1 and buf[i+1] == SYNC2:
            if i + FRAME_SIZE <= len(buf):
                frame = buf[i:i+FRAME_SIZE]
                crc_calc = crc8(frame[:FRAME_SIZE-1])
                crc_recv = frame[FRAME_SIZE-1]
                valid = (crc_calc == crc_recv)
                if valid:
                    vx, vy, wz = struct.unpack_from('<fff', frame, 2)
                    frames.append((i, True, vx, vy, wz, crc_recv))
                else:
                    frames.append((i, False, None, None, None, crc_recv))
                i += FRAME_SIZE
                continue
            else:
                break
        else:
            i += 1
    return frames


def main():
    try:
        with open(FILE, 'rb') as f:
            buf = f.read()
    except FileNotFoundError:
        print(FILE, 'not found')
        return

    print('Total bytes:', len(buf))
    frames = find_frames(buf)
    print('Found frames:', len(frames))
    for idx, valid, vx, vy, wz, crc in frames:
        if valid:
            print(f'offset {idx}: OK  vx={vx:.6f} vy={vy:.6f} wz={wz:.6f} crc=0x{crc:02x}')
        else:
            print(f'offset {idx}: BAD crc=0x{crc:02x}')

if __name__ == '__main__':
    main()

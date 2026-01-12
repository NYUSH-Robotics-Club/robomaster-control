#!/usr/bin/env python3
"""
cmd_vel_forwarder.py

Subscribe to ROS2 `/cmd_vel` or send one-shot velocity commands and forward
as radar frames over serial to the STM32 (USB CDC).

Frame format: [0xA5][0x5A][vx:f32][vy:f32][wz:f32][crc8]
CRC8 polynomial: 0x07, computed over the first (frame_size-1) bytes.

Usage (ROS2):
  python3 cmd_vel_forwarder.py --port /dev/ttyACM0 --baud 115200 --ros2

Usage (one-shot):
  python3 cmd_vel_forwarder.py --port /dev/ttyACM0 --baud 115200 --vx 0.5 --vy 0.0 --wz 0.0

"""

import argparse
import serial
import struct
import time
import sys
import threading


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
    crc = crc8(frame[:14])  # first 14 bytes (2 + 12)
    frame.append(crc)
    return bytes(frame)


class SerialForwarder:
    def __init__(self, port: str, baud: int = 115200, timeout=1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(0.2)
        # Start background reader to print incoming STM32 logs
        self._reader_run = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def close(self):
        if self.ser and self.ser.is_open:
            self._reader_run = False
            try:
                self.ser.close()
            except Exception:
                pass

    def send(self, vx: float, vy: float, wz: float):
        frame = encode_radar_cmd(vx, vy, wz)
        if not self.ser or not self.ser.is_open:
            self.open()
        self.ser.write(frame)
        # Note: flush() removed - let OS handle buffering for better performance

    def _reader_loop(self):
        # Read lines from STM32 and print to stdout
        while getattr(self, '_reader_run', False) and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline()
                if line:
                    # If contains non-printable bytes, show hex dump for debugging
                    is_binary = False
                    for b in line:
                        if b < 32 and b not in (9, 10, 13):
                            is_binary = True
                            break
                        if b > 126:
                            is_binary = True
                            break

                    if is_binary:
                        print('STM32> HEX:', line.hex())
                    else:
                        try:
                            s = line.decode('utf-8', errors='replace').rstrip('\r\n')
                        except Exception:
                            s = repr(line)
                        print('STM32>', s)
            except Exception:
                # Short sleep to avoid busy loop on errors
                time.sleep(0.05)


# ROS2 subscriber path
def ros2_run(forwarder: SerialForwarder, topic: str = '/cmd_vel'):
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
    except Exception as e:
        print('ROS2 import failed:', e, file=sys.stderr)
        print('Install ROS2 or run in non-ROS mode.', file=sys.stderr)
        return

    class CmdVelNode(Node):
        def __init__(self):
            super().__init__('cmd_vel_forwarder')
            self.subscription = self.create_subscription(
                Twist, topic, self.cb_twist, 10
            )
            self.get_logger().info('Subscribed to %s' % topic)

        def cb_twist(self, msg: Twist):
            vx = float(msg.linear.x)
            vy = float(msg.linear.y)
            wz = float(msg.angular.z)
            try:
                forwarder.send(vx, vy, wz)
                self.get_logger().debug('Forwarded vx=%.3f vy=%.3f wz=%.3f' % (vx, vy, wz))
            except Exception as e:
                self.get_logger().error('Serial send failed: %s' % e)

    rclpy.init()
    node = CmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True, help='Serial device (e.g., /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--ros2', action='store_true', help='Run as ROS2 node subscribing to /cmd_vel')
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--vx', type=float, default=0.0)
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--wz', type=float, default=0.0)
    parser.add_argument('--rate', type=float, default=20.0, help='Send rate (Hz) for periodic sends')
    args = parser.parse_args()

    fwd = SerialForwarder(args.port, args.baud)

    if args.ros2:
        print('Running in ROS2 subscriber mode. Topic=%s' % args.topic)
        try:
            fwd.open()
        except Exception as e:
            print('Failed to open serial port:', e)
            sys.exit(1)
        ros2_run(fwd, args.topic)
        fwd.close()
        return

    # Non-ROS mode: send periodic frames with provided vx,vy,wz
    try:
        fwd.open()
    except Exception as e:
        print('Failed to open serial port:', e)
        sys.exit(1)

    interval = 1.0 / max(1.0, args.rate)
    print('Sending frames at %.1f Hz to %s' % (1.0/interval, args.port))
    try:
        while True:
            fwd.send(args.vx, args.vy, args.wz)
            print('Sent vx=%.3f vy=%.3f wz=%.3f' % (args.vx, args.vy, args.wz))
            time.sleep(interval)
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        fwd.close()


if __name__ == '__main__':
    main()

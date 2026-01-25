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
import statistics
from collections import deque
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


def encode_radar_cmd(vx: float, vy: float, wz: float) -> bytes:
    frame = bytearray([0xA5, 0x5A])
    frame += struct.pack('<f', vx)
    frame += struct.pack('<f', vy)
    frame += struct.pack('<f', wz)
    crc = crc8(frame[:14])  # first 14 bytes (2 + 12)
    frame.append(crc)
    return bytes(frame)


class LatencyStats:
    """Collect and compute latency statistics."""
    def __init__(self, window_size: int = 100):
        self.latencies = deque(maxlen=window_size)
        self.total_count = 0
        self.last_report_time = time.time()
        self.report_interval = 5.0  # Report every 5 seconds
    
    def add(self, latency_ms: float):
        self.latencies.append(latency_ms)
        self.total_count += 1
    
    def get_stats(self) -> dict:
        if len(self.latencies) < 2:
            return None
        sorted_lat = sorted(self.latencies)
        n = len(sorted_lat)
        return {
            'count': self.total_count,
            'window': n,
            'min': sorted_lat[0],
            'max': sorted_lat[-1],
            'mean': statistics.mean(sorted_lat),
            'median': statistics.median(sorted_lat),
            'p95': sorted_lat[int(n * 0.95)] if n >= 20 else sorted_lat[-1],
            'p99': sorted_lat[int(n * 0.99)] if n >= 100 else sorted_lat[-1],
        }
    
    def should_report(self) -> bool:
        now = time.time()
        if now - self.last_report_time >= self.report_interval:
            self.last_report_time = now
            return True
        return False
    
    def format_report(self) -> str:
        stats = self.get_stats()
        if not stats:
            return "[LATENCY] Not enough data yet"
        return (f"[LATENCY] n={stats['count']} | "
                f"min={stats['min']:.2f}ms max={stats['max']:.2f}ms | "
                f"mean={stats['mean']:.2f}ms median={stats['median']:.2f}ms | "
                f"p95={stats['p95']:.2f}ms p99={stats['p99']:.2f}ms")


class SerialForwarder:
    def __init__(self, port: str, baud: int = 115200, timeout=1.0, measure_latency=False):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self.lock = threading.Lock()  # 线程锁
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 2.0  # 2秒重连冷却
        self.measure_latency = measure_latency
        self.latency_stats = LatencyStats() if measure_latency else None

    def open(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                return
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                time.sleep(0.2)
                # 启动读取线程（避免重复启动）
                if not getattr(self, '_reader_run', False):
                    self._reader_run = True
                    self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
                    self._reader_thread.start()
                print(f"Serial port {self.port} opened.")
            except Exception as e:
                print(f"Error opening serial port: {e}")
                self.ser = None

    def send(self, vx: float, vy: float, wz: float, recv_time_ns: int = None):
        """Send velocity command. If recv_time_ns is provided, measure latency."""
        frame = encode_radar_cmd(vx, vy, wz)
        
        # 检查是否连接，带有冷却机制
        if not self.ser or not self.ser.is_open:
            now = time.time()
            if now - self.last_reconnect_attempt > self.reconnect_interval:
                self.last_reconnect_attempt = now
                self.open()
            return None  # 如果还在冷却期或打开失败，直接丢弃这帧数据，不要阻塞

        send_time_ns = None
        try:
            with self.lock:
                self.ser.write(frame)
                send_time_ns = time.perf_counter_ns()
                # Note: flush() removed - let OS handle buffering for better performance
        except Exception as e:
            print(f"Serial write error: {e}")
            self.close()  # 出错时关闭，等待下次重连
            return None
        
        # Calculate and record latency if measurement enabled
        if self.measure_latency and recv_time_ns is not None and send_time_ns is not None:
            latency_ms = (send_time_ns - recv_time_ns) / 1e6
            self.latency_stats.add(latency_ms)
            return latency_ms
        return None

    def close(self):
        with self.lock:
            self._reader_run = False
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def _reader_loop(self):
        """Read and filter STM32 output, separating binary radar frames from text logs."""
        line_buffer = bytearray()
        
        while getattr(self, '_reader_run', False) and self.ser and self.ser.is_open:
            try:
                # Read available bytes
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                else:
                    data = self.ser.read(1)  # Blocking read with timeout
                
                if not data:
                    continue
                
                for byte in data:
                    # Check for radar frame header (0xA5 0x5A)
                    if len(line_buffer) >= 1 and line_buffer[-1] == 0xA5 and byte == 0x5A:
                        # Remove the 0xA5 from buffer and skip the next 13 bytes (rest of radar frame)
                        line_buffer = line_buffer[:-1]
                        # Read and discard remaining 13 bytes of radar frame
                        remaining = 13  # vx(4) + vy(4) + wz(4) + crc(1)
                        while remaining > 0 and self.ser.is_open:
                            skip = self.ser.read(min(remaining, self.ser.in_waiting or 1))
                            if skip:
                                remaining -= len(skip)
                            else:
                                break
                        continue
                    
                    # Handle newlines - flush buffer as a line
                    if byte in (0x0A, 0x0D):  # \n or \r
                        if len(line_buffer) > 0:
                            self._process_text_line(line_buffer)
                            line_buffer = bytearray()
                        continue
                    
                    # Accumulate printable ASCII characters
                    if 32 <= byte <= 126 or byte == 0x09:  # printable or tab
                        line_buffer.append(byte)
                    else:
                        # Non-printable byte encountered - might be start of binary data
                        # Flush any accumulated text first
                        if len(line_buffer) > 3:  # Only print if we have meaningful text
                            self._process_text_line(line_buffer)
                        line_buffer = bytearray()
                        
                        # If this looks like start of radar frame, skip it
                        if byte == 0xA5:
                            line_buffer.append(byte)  # Keep to check next byte
                            
            except Exception:
                time.sleep(0.05)
    
    def _process_text_line(self, line_buffer: bytearray):
        """Process and print a text line from STM32."""
        try:
            s = line_buffer.decode('ascii', errors='ignore').strip()
            if len(s) < 3:
                return
            
            # Known valid prefixes
            valid_prefixes = ('GIM,', 'CMD,', 'CHA,', 'SHO,', 'RAD,', 
                              '[DEBUG]', '[INFO]', '[WARN]', '[ERROR]',
                              'RADAR', 'YAW', 'PIT', 'Motor', 'Init')
            
            # Try to find a valid prefix in the string (might have leading garbage)
            for prefix in valid_prefixes:
                idx = s.find(prefix)
                if idx != -1:
                    # Extract from the valid prefix onwards
                    clean_line = s[idx:]
                    # Validate: should have proper structure (commas for data lines)
                    if prefix in ('GIM,', 'CMD,', 'CHA,', 'SHO,', 'RAD,'):
                        if clean_line.count(',') >= 2:
                            print(f'STM32> {clean_line}')
                    else:
                        print(f'STM32> {clean_line}')
                    return
        except Exception:
            pass
            except Exception:
                # Short sleep to avoid busy loop on errors
                time.sleep(0.05)


# ROS2 subscriber path
def ros2_run(forwarder: SerialForwarder, topic: str = '/cmd_vel', verbose_latency: bool = False):
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
            if forwarder.measure_latency:
                self.get_logger().info('Latency measurement ENABLED (stats every 5s)')

        def cb_twist(self, msg: Twist):
            recv_time_ns = time.perf_counter_ns()  # Record receive time immediately
            
            vx = float(msg.linear.x)
            vy = float(msg.linear.y)
            wz = float(msg.angular.z)
            try:
                latency_ms = forwarder.send(vx, vy, wz, recv_time_ns)
                
                # Verbose per-message latency logging
                if verbose_latency and latency_ms is not None:
                    self.get_logger().info(f'[LAT] {latency_ms:.3f}ms | vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}')
                else:
                    self.get_logger().debug('Forwarded vx=%.3f vy=%.3f wz=%.3f' % (vx, vy, wz))
                
                # Periodic stats report
                if forwarder.measure_latency and forwarder.latency_stats.should_report():
                    self.get_logger().info(forwarder.latency_stats.format_report())
                    
            except Exception as e:
                self.get_logger().error('Serial send failed: %s' % e)

    rclpy.init()
    node = CmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print final stats on exit
        if forwarder.measure_latency:
            print('\n' + '='*60)
            print('FINAL LATENCY STATISTICS')
            print(forwarder.latency_stats.format_report())
            print('='*60)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Forward /cmd_vel to STM32 via serial with optional latency measurement',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Latency Measurement Examples:
  # Basic latency measurement (stats every 5s)
  python3 cmd_vel_forwarder_updated.py --port /dev/ttyACM0 --ros2 --latency
  
  # Verbose mode (print each message latency)
  python3 cmd_vel_forwarder_updated.py --port /dev/ttyACM0 --ros2 --latency --verbose
""")
    parser.add_argument('--port', required=True, help='Serial device (e.g., /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--ros2', action='store_true', help='Run as ROS2 node subscribing to /cmd_vel')
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--vx', type=float, default=0.0)
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--wz', type=float, default=0.0)
    parser.add_argument('--rate', type=float, default=20.0, help='Send rate (Hz) for periodic sends')
    parser.add_argument('--latency', action='store_true', help='Enable latency measurement (ROS2 mode)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Print latency for each message')
    args = parser.parse_args()

    fwd = SerialForwarder(args.port, args.baud, measure_latency=args.latency)

    if args.ros2:
        print('Running in ROS2 subscriber mode. Topic=%s' % args.topic)
        if args.latency:
            print('Latency measurement ENABLED')
        try:
            fwd.open()
        except Exception as e:
            print('Failed to open serial port:', e)
            sys.exit(1)
        ros2_run(fwd, args.topic, verbose_latency=args.verbose)
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

#!/usr/bin/env python3
"""
cmd_vel_keyboard_fixed.py

Fixed keyboard control with proper thread safety and graceful shutdown.

Usage:
  python3 cmd_vel_keyboard_fixed.py --port COM9 --speed 0.3
  python3 cmd_vel_keyboard_fixed.py --port /dev/ttyACM0 --speed 0.3

Controls:
  W/S: Forward/Backward
  A/D: Strafe Left/Right  
  Q/E: Rotate (Not used for Swerve, but supported)
  SPACE: Emergency Stop
  ESC: Quit
"""

import argparse
import serial
import struct
import time
import sys
import threading
import statistics
from collections import deque

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False

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
        self.lock = threading.Lock()
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 2.0
        self.measure_latency = measure_latency
        self.latency_stats = LatencyStats() if measure_latency else None

    def open(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                return
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                time.sleep(0.2)
                if not getattr(self, '_reader_run', False):
                    self._reader_run = True
                    self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
                    self._reader_thread.start()
                print(f"[INFO] Serial port {self.port} opened at {self.baud} baud.")
            except Exception as e:
                print(f"[ERROR] Failed to open serial port: {e}")
                self.ser = None

    def send(self, vx: float, vy: float, wz: float, recv_time_ns: int = None):
        """Send velocity command. If recv_time_ns is provided, measure latency."""
        frame = encode_radar_cmd(vx, vy, wz)
        if not self.ser or not self.ser.is_open:
            now = time.time()
            if now - self.last_reconnect_attempt > self.reconnect_interval:
                self.last_reconnect_attempt = now
                self.open()
            return False, None
        
        send_time_ns = None
        try:
            with self.lock:
                self.ser.write(frame)
                send_time_ns = time.perf_counter_ns()
            
            # Calculate and record latency if measurement enabled
            latency_ms = None
            if self.measure_latency and recv_time_ns is not None and send_time_ns is not None:
                latency_ms = (send_time_ns - recv_time_ns) / 1e6
                self.latency_stats.add(latency_ms)
            
            return True, latency_ms
        except Exception as e:
            print(f"[ERROR] Serial write failed: {e}")
            self.close()
            return False, None

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
        
        while getattr(self, '_reader_run', False):
            if not self.ser or not self.ser.is_open:
                time.sleep(1.0)
                continue
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
                            
            except Exception as e:
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
                            print(f'[STM32] {clean_line}')
                    else:
                        print(f'[STM32] {clean_line}')
                    return
        except Exception:
            pass

def keyboard_run(forwarder, move_speed=0.3, turn_speed=1.0, send_rate=100):
    """Keyboard control with proper thread safety and smooth acceleration."""
    
    if not PYNPUT_AVAILABLE:
        print("[ERROR] pynput library not found. Install with: pip install pynput")
        sys.exit(1)

    print("\n" + "="*50)
    print(" KEYBOARD CONTROL MODE (Swerve Chassis)")
    print("="*50)
    print(" W/S: Move Forward/Backward")
    print(" A/D: Strafe Left/Right")
    print(" Q/E: Rotate Left/Right (experimental)")
    print(" SPACE: Emergency Stop")
    print(" ESC: Exit")
    print("="*50 + "\n")

    # STM32 filtering parameters (must match cmd_controller.c)
    RADAR_SMOOTH_ALPHA = 0.20       # Low-pass filter coefficient
    RADAR_MAX_DELTA_V = 0.05        # Max velocity change per cycle (m/s)
    RADAR_MAX_DELTA_W = 0.10        # Max angular velocity change per cycle (rad/s)

    # Thread-safe velocity state
    vel_lock = threading.Lock()
    target_vel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
    filtered_vel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
    
    # Shutdown flag
    shutdown_event = threading.Event()
    pressed_keys = set()

    def update_velocity():
        """Update target velocity based on pressed keys."""
        with vel_lock:
            # Forward/Backward (W/S)
            if 'w' in pressed_keys:
                target_vel['vx'] = move_speed
            elif 's' in pressed_keys:
                target_vel['vx'] = -move_speed
            else:
                target_vel['vx'] = 0.0

            # Strafe (A/D) - Note: A=left (+vy), D=right (-vy) in this coordinate system
            if 'a' in pressed_keys:
                target_vel['vy'] = move_speed
            elif 'd' in pressed_keys:
                target_vel['vy'] = -move_speed
            else:
                target_vel['vy'] = 0.0

            # Rotation (Q/E) - Usually not used for swerve but supported
            if 'q' in pressed_keys:
                target_vel['wz'] = turn_speed
            elif 'e' in pressed_keys:
                target_vel['wz'] = -turn_speed
            else:
                target_vel['wz'] = 0.0

    def on_press(key):
        try:
            char = key.char.lower()
            if char in ['w', 'a', 's', 'd', 'q', 'e']:
                pressed_keys.add(char)
                update_velocity()
        except AttributeError:
            # Special keys (Space, ESC, etc.)
            if key == keyboard.Key.space:
                # Emergency stop
                with vel_lock:
                    target_vel['vx'] = 0.0
                    target_vel['vy'] = 0.0
                    target_vel['wz'] = 0.0
                print("[WARN] Emergency Stop!")
                forwarder.send(0.0, 0.0, 0.0)
                # Send multiple times to ensure it's received
                for _ in range(5):
                    forwarder.send(0.0, 0.0, 0.0)
                    time.sleep(0.01)
            elif key == keyboard.Key.esc:
                print("[INFO] ESC pressed, shutting down...")
                shutdown_event.set()
                return False

    def on_release(key):
        try:
            char = key.char.lower()
            if char in pressed_keys:
                pressed_keys.discard(char)
                update_velocity()
        except AttributeError:
            pass

    # Start keyboard listener in daemon thread
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # Send initial stop command to clear any previous state
    print("[INFO] Sending initial stop command...")
    for _ in range(5):
        forwarder.send(0.0, 0.0, 0.0)
        time.sleep(0.01)
    time.sleep(0.2)

    # Main control loop
    interval = 1.0 / max(1.0, send_rate)
    last_print = time.time()
    last_vel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}

    print(f"[INFO] Starting send loop at {send_rate} Hz")

    try:
        while not shutdown_event.is_set():
            # Read current target velocity safely
            with vel_lock:
                target = target_vel.copy()
            
            # Apply smoothing/filtering (same as STM32 does)
            # 1. Low-pass filter
            lp_vx = filtered_vel['vx'] + RADAR_SMOOTH_ALPHA * (target['vx'] - filtered_vel['vx'])
            lp_vy = filtered_vel['vy'] + RADAR_SMOOTH_ALPHA * (target['vy'] - filtered_vel['vy'])
            lp_wz = filtered_vel['wz'] + RADAR_SMOOTH_ALPHA * (target['wz'] - filtered_vel['wz'])
            
            # 2. Delta cap (limit acceleration)
            dvx = lp_vx - filtered_vel['vx']
            if dvx > RADAR_MAX_DELTA_V:
                dvx = RADAR_MAX_DELTA_V
            elif dvx < -RADAR_MAX_DELTA_V:
                dvx = -RADAR_MAX_DELTA_V
            filtered_vel['vx'] = filtered_vel['vx'] + dvx
            
            dvy = lp_vy - filtered_vel['vy']
            if dvy > RADAR_MAX_DELTA_V:
                dvy = RADAR_MAX_DELTA_V
            elif dvy < -RADAR_MAX_DELTA_V:
                dvy = -RADAR_MAX_DELTA_V
            filtered_vel['vy'] = filtered_vel['vy'] + dvy
            
            dwz = lp_wz - filtered_vel['wz']
            if dwz > RADAR_MAX_DELTA_W:
                dwz = RADAR_MAX_DELTA_W
            elif dwz < -RADAR_MAX_DELTA_W:
                dwz = -RADAR_MAX_DELTA_W
            filtered_vel['wz'] = filtered_vel['wz'] + dwz
            
            # Send smoothed command
            forwarder.send(filtered_vel['vx'], filtered_vel['vy'], filtered_vel['wz'], None)
            
            # Print only when velocity changes or periodically
            now = time.time()
            if (filtered_vel != last_vel) or (now - last_print > 0.5):
                vel_magnitude = (filtered_vel['vx']**2 + filtered_vel['vy']**2)**0.5
                print(f"[CMD] vx={filtered_vel['vx']:+.2f} vy={filtered_vel['vy']:+.2f} wz={filtered_vel['wz']:+.2f} mag={vel_magnitude:.2f}", end='\r')
                last_print = now
                last_vel = filtered_vel.copy()
            
            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt received")
        shutdown_event.set()
    
    finally:
        # Ensure listener stops
        listener.stop()
        listener.join(timeout=1.0)
        
        # Send final stop command multiple times
        print("\n[INFO] Sending final stop command...")
        for _ in range(10):
            success, _ = forwarder.send(0.0, 0.0, 0.0)
            if not success:
                break
            time.sleep(0.02)
        
        time.sleep(0.2)
        forwarder.close()
        print("[INFO] Shutdown complete.")

def ros2_run(forwarder, topic='/cmd_vel', verbose_latency=False):
    """ROS2 subscriber mode - forward /cmd_vel messages to serial."""
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
    except Exception as e:
        print(f"[ERROR] ROS2 import failed: {e}")
        print("[ERROR] Install ROS2 or run in keyboard mode")
        sys.exit(1)

    class CmdVelNode(Node):
        def __init__(self, forwarder):
            super().__init__('cmd_vel_forwarder')
            self.forwarder = forwarder
            self.subscription = self.create_subscription(
                Twist, topic, self.cb_twist, 10
            )
            self.get_logger().info(f'[ROS2] Subscribed to {topic}')
            if forwarder.measure_latency:
                self.get_logger().info('[ROS2] Latency measurement ENABLED (stats every 5s)')

        def cb_twist(self, msg: Twist):
            recv_time_ns = time.perf_counter_ns()  # Record receive time immediately
            
            vx = float(msg.linear.x)
            vy = float(msg.linear.y)
            wz = float(msg.angular.z)
            success, latency_ms = self.forwarder.send(vx, vy, wz, recv_time_ns)
            vel_magnitude = (vx**2 + vy**2)**0.5
            
            # Verbose per-message latency logging
            if verbose_latency and latency_ms is not None:
                print(f"[NAV2 -> STM32] vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} mag={vel_magnitude:.2f} | lat={latency_ms:.3f}ms")
            else:
                print(f"[NAV2 -> STM32] vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} mag={vel_magnitude:.2f}    ", end='\r')
            
            # Periodic stats report
            if self.forwarder.measure_latency and self.forwarder.latency_stats.should_report():
                print(f"\n{self.forwarder.latency_stats.format_report()}")

    rclpy.init()
    node = CmdVelNode(forwarder)
    try:
        print("[INFO] Running ROS2 subscriber mode. Listening on /cmd_vel")
        print("[INFO] Press Ctrl+C to exit")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
        # Print final stats on exit
        if forwarder.measure_latency:
            print('\n' + '='*60)
            print('FINAL LATENCY STATISTICS')
            print(forwarder.latency_stats.format_report())
            print('='*60)
    finally:
        # Send final stop command
        print("[INFO] Sending final stop command...")
        for _ in range(10):
            forwarder.send(0.0, 0.0, 0.0)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()
        forwarder.close()

def oneshot_run(forwarder, vx, vy, wz, duration, rate):
    """One-shot mode - send fixed velocity commands for specified duration."""
    
    # STM32 filtering parameters
    RADAR_SMOOTH_ALPHA = 0.20
    RADAR_MAX_DELTA_V = 0.05
    RADAR_MAX_DELTA_W = 0.10
    
    # Smoothed velocity state
    filtered_vel = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
    target_vel = {'vx': vx, 'vy': vy, 'wz': wz}
    
    interval = 1.0 / max(1.0, rate)
    num_frames = int(duration * rate)
    
    print(f"\n[INFO] One-shot mode:")
    print(f"  Target: vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}")
    print(f"  Duration: {duration:.2f}s, Rate: {rate}Hz, Frames: {num_frames}")
    print("[INFO] Sending velocity commands...\n")
    
    # Send initial stop to clear state
    print("[INFO] Sending initial stop...")
    for _ in range(5):
        forwarder.send(0.0, 0.0, 0.0)
        time.sleep(0.01)
    time.sleep(0.2)
    
    try:
        # Send target velocity for duration
        for frame_idx in range(num_frames):
            # Apply smoothing (same as STM32)
            lp_vx = filtered_vel['vx'] + RADAR_SMOOTH_ALPHA * (target_vel['vx'] - filtered_vel['vx'])
            lp_vy = filtered_vel['vy'] + RADAR_SMOOTH_ALPHA * (target_vel['vy'] - filtered_vel['vy'])
            lp_wz = filtered_vel['wz'] + RADAR_SMOOTH_ALPHA * (target_vel['wz'] - filtered_vel['wz'])
            
            # Delta cap
            dvx = lp_vx - filtered_vel['vx']
            if dvx > RADAR_MAX_DELTA_V: dvx = RADAR_MAX_DELTA_V
            elif dvx < -RADAR_MAX_DELTA_V: dvx = -RADAR_MAX_DELTA_V
            filtered_vel['vx'] = filtered_vel['vx'] + dvx
            
            dvy = lp_vy - filtered_vel['vy']
            if dvy > RADAR_MAX_DELTA_V: dvy = RADAR_MAX_DELTA_V
            elif dvy < -RADAR_MAX_DELTA_V: dvy = -RADAR_MAX_DELTA_V
            filtered_vel['vy'] = filtered_vel['vy'] + dvy
            
            dwz = lp_wz - filtered_vel['wz']
            if dwz > RADAR_MAX_DELTA_W: dwz = RADAR_MAX_DELTA_W
            elif dwz < -RADAR_MAX_DELTA_W: dwz = -RADAR_MAX_DELTA_W
            filtered_vel['wz'] = filtered_vel['wz'] + dwz
            
            # Send
            forwarder.send(filtered_vel['vx'], filtered_vel['vy'], filtered_vel['wz'])
            
            # Progress print
            if (frame_idx + 1) % max(1, rate // 5) == 0:
                elapsed = (frame_idx + 1) * interval
                print(f"  [{frame_idx + 1}/{num_frames}] {elapsed:.2f}s: "
                      f"vx={filtered_vel['vx']:+.3f} vy={filtered_vel['vy']:+.3f} wz={filtered_vel['wz']:+.3f}")
            
            time.sleep(interval)
        
        # Send stop command
        print("\n[INFO] Sending stop command...")
        for _ in range(10):
            forwarder.send(0.0, 0.0, 0.0)
            time.sleep(0.02)
        
        print("[INFO] Done!")
        
    except KeyboardInterrupt:
        print("\n[WARN] Interrupted by user, sending stop...")
        for _ in range(10):
            forwarder.send(0.0, 0.0, 0.0)
            time.sleep(0.02)

def main():
    parser = argparse.ArgumentParser(description='Swerve chassis control (keyboard, ROS2, or one-shot velocity)')
    parser.add_argument('--port', required=True, help='Serial port (COM9, /dev/ttyACM0, etc.)')
    parser.add_argument('--baud', type=int, default=115200)
    
    # Mode selection
    parser.add_argument('--keyboard', action='store_true', help='Enable keyboard control mode')
    parser.add_argument('--ros2', action='store_true', help='Enable ROS2 /cmd_vel subscriber mode')
    
    # One-shot velocity mode
    parser.add_argument('--vx', type=float, help='X velocity (m/s) - enables one-shot mode')
    parser.add_argument('--vy', type=float, default=0.0, help='Y velocity (m/s)')
    parser.add_argument('--wz', type=float, default=0.0, help='Angular velocity (rad/s)')
    parser.add_argument('--duration', type=float, default=1.0, help='Duration in seconds (for one-shot mode)')
    
    # ROS2 mode parameters
    parser.add_argument('--topic', default='/cmd_vel', help='ROS2 topic name (default: /cmd_vel)')
    
    # Keyboard mode parameters
    parser.add_argument('--speed', type=float, default=0.3, help='Movement speed for keyboard (0.0-1.0)')
    parser.add_argument('--rate', type=int, default=200, help='Send rate in Hz (default 200 to match STM32 CmdController 200Hz)')
    
    # Latency measurement
    parser.add_argument('--latency', action='store_true', help='Enable latency measurement (ROS2 mode only)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Print latency for each message')
    
    args = parser.parse_args()

    fwd = SerialForwarder(args.port, args.baud, measure_latency=args.latency)
    
    try:
        fwd.open()
        if fwd.ser is None:
            sys.exit(1)
        
        # Mode selection logic
        if args.vx is not None:
            # One-shot velocity mode
            oneshot_run(fwd, args.vx, args.vy, args.wz, args.duration, args.rate)
        elif args.keyboard:
            if args.speed <= 0 or args.speed > 1.0:
                print("[ERROR] --speed must be between 0 and 1.0")
                sys.exit(1)
            keyboard_run(fwd, move_speed=args.speed, send_rate=args.rate)
        elif args.ros2:
            if args.latency:
                print('[INFO] Latency measurement ENABLED')
            ros2_run(fwd, topic=args.topic, verbose_latency=args.verbose)
        else:
            print("[ERROR] Please specify a mode:")
            print("\n[USAGE 1] One-shot velocity:")
            print("  python3 cmd_vel_keyboard_fixed.py --port /dev/ttyACM0 --vx 0.5 --vy 0.2 --duration 2.0")
            print("  python3 cmd_vel_keyboard_fixed.py --port /dev/ttyACM0 --vx -0.5 --vy 0.0 --wz 0.0 --duration 1.0")
            print("\n[USAGE 2] Keyboard control:")
            print("  python3 cmd_vel_keyboard_fixed.py --port /dev/ttyACM0 --keyboard --speed 0.5")
            print("\n[USAGE 3] ROS2 subscriber:")
            print("  python3 cmd_vel_keyboard_fixed.py --port /dev/ttyACM0 --ros2")
            sys.exit(1)
    finally:
        fwd.close()

if __name__ == '__main__':
    main()

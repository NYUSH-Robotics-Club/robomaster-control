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

class SerialForwarder:
    def __init__(self, port: str, baud: int = 115200, timeout=1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self.lock = threading.Lock()
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 2.0

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

    def send(self, vx: float, vy: float, wz: float):
        frame = encode_radar_cmd(vx, vy, wz)
        if not self.ser or not self.ser.is_open:
            now = time.time()
            if now - self.last_reconnect_attempt > self.reconnect_interval:
                self.last_reconnect_attempt = now
                self.open()
            return False
        try:
            with self.lock:
                self.ser.write(frame)
            return True
        except Exception as e:
            print(f"[ERROR] Serial write failed: {e}")
            self.close()
            return False

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
        while getattr(self, '_reader_run', False):
            if not self.ser or not self.ser.is_open:
                time.sleep(1.0)
                continue
            try:
                line = self.ser.readline()
                if line:
                    # Skip radar frames
                    if len(line) >= 2 and line[0] == 0xA5 and line[1] == 0x5A:
                        continue
                    # Try to decode as text
                    try:
                        s = line.decode('ascii', errors='ignore').rstrip('\r\n')
                        if s.strip():
                            print(f'[STM32] {s}')
                    except Exception:
                        pass
            except Exception:
                time.sleep(0.05)

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
            forwarder.send(filtered_vel['vx'], filtered_vel['vy'], filtered_vel['wz'])
            
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
            success = forwarder.send(0.0, 0.0, 0.0)
            if not success:
                break
            time.sleep(0.02)
        
        time.sleep(0.2)
        forwarder.close()
        print("[INFO] Shutdown complete.")

def ros2_run(forwarder, topic='/cmd_vel'):
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

        def cb_twist(self, msg: Twist):
            vx = float(msg.linear.x)
            vy = float(msg.linear.y)
            wz = float(msg.angular.z)
            self.forwarder.send(vx, vy, wz)
            self.get_logger().debug(f'[ROS2] vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}')

    rclpy.init()
    node = CmdVelNode(forwarder)
    try:
        print("[INFO] Running ROS2 subscriber mode. Listening on /cmd_vel")
        print("[INFO] Press Ctrl+C to exit")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
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
    
    args = parser.parse_args()

    fwd = SerialForwarder(args.port, args.baud)
    
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
            ros2_run(fwd, topic=args.topic)
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

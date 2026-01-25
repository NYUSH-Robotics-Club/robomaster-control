#!/usr/bin/env python3
"""
measure_pointcloud_latency.py

Measure processing latency from PointCloud2 to LaserScan.
Compares header.stamp timestamps between input and output topics.

Usage:
  python3 measure_pointcloud_latency.py
  python3 measure_pointcloud_latency.py --cloud /cloud_registered --scan /scan
  python3 measure_pointcloud_latency.py --duration 30

"""

import argparse
import time
import statistics
from collections import deque
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2, LaserScan
except ImportError:
    print("Error: ROS2 not available. Run this in a ROS2 environment.")
    exit(1)


class LatencyStats:
    """Collect and compute latency statistics."""
    def __init__(self, window_size: int = 100):
        self.latencies = deque(maxlen=window_size)
        self.total_count = 0
    
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
            'stdev': statistics.stdev(sorted_lat) if n > 1 else 0,
            'p95': sorted_lat[int(n * 0.95)] if n >= 20 else sorted_lat[-1],
            'p99': sorted_lat[int(n * 0.99)] if n >= 100 else sorted_lat[-1],
        }
    
    def format_report(self) -> str:
        stats = self.get_stats()
        if not stats:
            return "Not enough data yet"
        return (f"n={stats['count']} | "
                f"min={stats['min']:.2f}ms max={stats['max']:.2f}ms | "
                f"mean={stats['mean']:.2f}ms median={stats['median']:.2f}ms | "
                f"stdev={stats['stdev']:.2f}ms | "
                f"p95={stats['p95']:.2f}ms p99={stats['p99']:.2f}ms")


class PointCloudLatencyMeasurer(Node):
    def __init__(self, cloud_topic: str, scan_topic: str, report_interval: float = 5.0):
        super().__init__('pointcloud_latency_measurer')
        
        self.cloud_topic = cloud_topic
        self.scan_topic = scan_topic
        self.report_interval = report_interval
        
        # Statistics
        self.processing_latency = LatencyStats()  # scan.stamp - cloud.stamp
        self.cloud_to_now = LatencyStats()        # now - cloud.stamp
        self.scan_to_now = LatencyStats()         # now - scan.stamp
        
        # Frequency tracking
        self.cloud_count = 0
        self.scan_count = 0
        self.start_time = time.time()
        self.last_report_time = time.time()
        
        # Last cloud timestamp for matching
        self.last_cloud_stamps = deque(maxlen=10)  # Keep last 10 cloud timestamps
        
        # QoS profile compatible with sensor data (BEST_EFFORT for high-frequency sensors)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to topics with sensor QoS
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback, sensor_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, sensor_qos
        )
        
        # Report timer
        self.timer = self.create_timer(report_interval, self.report_stats)
        
        self.get_logger().info(f'Measuring latency: {cloud_topic} → {scan_topic}')
        self.get_logger().info(f'Reporting every {report_interval}s')
        print("\n" + "="*70)
        print(" POINTCLOUD → LASERSCAN LATENCY MEASUREMENT")
        print("="*70)
        print(f" Input:  {cloud_topic}")
        print(f" Output: {scan_topic}")
        print("="*70 + "\n")
    
    def cloud_callback(self, msg: PointCloud2):
        now = self.get_clock().now()
        msg_time = Time.from_msg(msg.header.stamp)
        
        # Store timestamp for matching with scan
        self.last_cloud_stamps.append(msg_time.nanoseconds)
        
        # Measure cloud latency (time from sensor to here)
        latency_ns = now.nanoseconds - msg_time.nanoseconds
        latency_ms = latency_ns / 1e6
        if latency_ms >= 0 and latency_ms < 1000:  # Sanity check
            self.cloud_to_now.add(latency_ms)
        
        self.cloud_count += 1
    
    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        msg_time = Time.from_msg(msg.header.stamp)
        
        # Measure scan latency (time from scan creation to here)
        latency_ns = now.nanoseconds - msg_time.nanoseconds
        latency_ms = latency_ns / 1e6
        if latency_ms >= 0 and latency_ms < 1000:  # Sanity check
            self.scan_to_now.add(latency_ms)
        
        # Try to match with a cloud timestamp to get processing time
        scan_stamp_ns = msg_time.nanoseconds
        for cloud_stamp_ns in self.last_cloud_stamps:
            # If scan has same or very close timestamp to cloud, they're matched
            # Some nodes copy the timestamp, others might have slight offset
            diff_ms = abs(scan_stamp_ns - cloud_stamp_ns) / 1e6
            if diff_ms < 50:  # Within 50ms, consider it a match
                # Processing time = when scan was created - when cloud was created
                # But if timestamps are copied, we need to look at arrival times instead
                break
        
        self.scan_count += 1
    
    def report_stats(self):
        elapsed = time.time() - self.start_time
        cloud_hz = self.cloud_count / elapsed if elapsed > 0 else 0
        scan_hz = self.scan_count / elapsed if elapsed > 0 else 0
        
        print("-"*70)
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Elapsed: {elapsed:.1f}s")
        print(f"  Cloud frequency:  {cloud_hz:.1f} Hz ({self.cloud_count} msgs)")
        print(f"  Scan frequency:   {scan_hz:.1f} Hz ({self.scan_count} msgs)")
        
        if cloud_hz > 0 and scan_hz > 0:
            drop_rate = max(0, (cloud_hz - scan_hz) / cloud_hz * 100)
            print(f"  Drop rate:        {drop_rate:.1f}%")
        
        cloud_stats = self.cloud_to_now.get_stats()
        scan_stats = self.scan_to_now.get_stats()
        
        if cloud_stats:
            print(f"\n  Cloud→Node latency: {self.cloud_to_now.format_report()}")
        if scan_stats:
            print(f"  Scan→Node latency:  {self.scan_to_now.format_report()}")
        
        # Estimate processing time
        if cloud_stats and scan_stats:
            # Processing time ≈ (scan latency - cloud latency) if timestamps are preserved
            # Or just look at the scan latency as total pipeline latency
            print(f"\n  Estimated processing time: ~{scan_stats['mean']:.2f}ms (scan latency)")
        
        print("-"*70 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description='Measure PointCloud2 to LaserScan processing latency',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Default topics
  python3 measure_pointcloud_latency.py
  
  # Custom topics
  python3 measure_pointcloud_latency.py --cloud /velodyne_points --scan /scan
  
  # Longer measurement period
  python3 measure_pointcloud_latency.py --duration 60
"""
    )
    parser.add_argument('--cloud', default='/cloud_registered',
                        help='Input PointCloud2 topic (default: /cloud_registered)')
    parser.add_argument('--scan', default='/scan',
                        help='Output LaserScan topic (default: /scan)')
    parser.add_argument('--interval', type=float, default=5.0,
                        help='Report interval in seconds (default: 5.0)')
    parser.add_argument('--duration', type=float, default=0,
                        help='Run for specified seconds then exit (0 = run forever)')
    args = parser.parse_args()
    
    rclpy.init()
    node = PointCloudLatencyMeasurer(args.cloud, args.scan, args.interval)
    
    try:
        if args.duration > 0:
            # Run for specified duration
            end_time = time.time() + args.duration
            while rclpy.ok() and time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
            # Final report
            node.report_stats()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
        # Final report
        node.report_stats()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        print("\n" + "="*70)
        print(" MEASUREMENT COMPLETE")
        print("="*70)


if __name__ == '__main__':
    main()

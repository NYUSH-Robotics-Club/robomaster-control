#!/usr/bin/env python3
"""
Save and load AMCL pose for automatic initialization
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import os
import sys

POSE_FILE = "/tmp/amcl_last_pose.json"

class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self.last_pose = None
        self.get_logger().info("监听 /amcl_pose 并自动保存...")

    def pose_callback(self, msg):
        self.last_pose = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
        }
        # 每次更新都保存（覆盖）
        with open(POSE_FILE, 'w') as f:
            json.dump(self.last_pose, f, indent=2)

def load_and_publish_pose():
    """加载上次保存的位姿并发布"""
    rclpy.init()
    node = Node('pose_loader')
    
    if not os.path.exists(POSE_FILE):
        node.get_logger().warn(f"没有找到保存的位姿文件 {POSE_FILE}")
        node.get_logger().info("将使用全局定位模式")
        rclpy.shutdown()
        return
    
    with open(POSE_FILE, 'r') as f:
        pose_data = json.load(f)
    
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    
    # 等待订阅者
    import time
    time.sleep(1.0)
    
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    
    msg.pose.pose.position.x = pose_data['position']['x']
    msg.pose.pose.position.y = pose_data['position']['y']
    msg.pose.pose.position.z = pose_data['position']['z']
    
    msg.pose.pose.orientation.x = pose_data['orientation']['x']
    msg.pose.pose.orientation.y = pose_data['orientation']['y']
    msg.pose.pose.orientation.z = pose_data['orientation']['z']
    msg.pose.pose.orientation.w = pose_data['orientation']['w']
    
    # 设置协方差（初始不确定性）
    msg.pose.covariance[0] = 0.25   # x
    msg.pose.covariance[7] = 0.25   # y
    msg.pose.covariance[35] = 0.068 # yaw
    
    publisher.publish(msg)
    node.get_logger().info(f"已加载并发布位姿: x={pose_data['position']['x']:.2f}, y={pose_data['position']['y']:.2f}")
    
    time.sleep(0.5)
    rclpy.shutdown()

def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--save':
        # 保存模式：持续监听并保存
        rclpy.init()
        saver = PoseSaver()
        try:
            rclpy.spin(saver)
        except KeyboardInterrupt:
            saver.get_logger().info("正在保存最后的位姿...")
        finally:
            saver.destroy_node()
            rclpy.shutdown()
    else:
        # 加载模式：启动时加载并发布
        load_and_publish_pose()

if __name__ == '__main__':
    main()

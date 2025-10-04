#!/usr/bin/env python3
"""
简单的目标点发布器（用于测试）

用法：
  python3 publish_goal.py --x 3.0 --y 2.0 --yaw 90
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
import argparse
import math


def euler_to_quaternion(roll, pitch, yaw):
    """欧拉角转四元数"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


class GoalPublisher(Node):
    def __init__(self, x, y, yaw_deg):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/nav_goal', 10)

        # 转换yaw为弧度
        self.yaw_rad = math.radians(yaw_deg)
        self.x = x
        self.y = y

        print(f"\n✅ 准备发布目标点到 /nav_goal:")
        print(f"   位置: ({x:.3f}, {y:.3f})")
        print(f"   朝向: {yaw_deg:.1f}° ({self.yaw_rad:.3f} rad)")
        print(f"   发布频率: 10 Hz，持续2秒\n")

        # 创建定时器，每100ms发布一次
        self.timer = self.create_timer(0.1, self.publish_goal)
        self.count = 0
        self.max_count = 20  # 发布20次（2秒）

    def publish_goal(self):
        """定时发布目标点"""
        if self.count >= self.max_count:
            print(f"✅ 已发布 {self.count} 次，完成\n")
            self.timer.cancel()
            # 发布完成后退出
            import sys
            sys.exit(0)
            return

        # 创建目标点消息
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0
        msg.pose.orientation = euler_to_quaternion(0.0, 0.0, self.yaw_rad)

        # 发布目标点
        self.publisher.publish(msg)
        self.count += 1

        if self.count % 5 == 0:
            print(f"📤 已发布 {self.count}/{self.max_count} 次...")


def main():
    parser = argparse.ArgumentParser(description='发布导航目标点')
    parser.add_argument('--x', type=float, default=3.0, help='目标点X坐标（米）')
    parser.add_argument('--y', type=float, default=2.0, help='目标点Y坐标（米）')
    parser.add_argument('--yaw', type=float, default=90.0, help='目标朝向（度）')
    args = parser.parse_args()

    rclpy.init()
    node = GoalPublisher(args.x, args.y, args.yaw)

    try:
        # 持续spin直到发布完成
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

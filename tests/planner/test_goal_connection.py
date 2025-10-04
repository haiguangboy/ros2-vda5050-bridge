#!/usr/bin/env python3
"""
快速测试目标点发布/订阅是否正常

终端1运行: python3 test_goal_connection.py
终端2运行: python3 publish_goal.py --x 3.0 --y 0.0 --yaw 90
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class GoalSubscriberTest(Node):
    def __init__(self):
        super().__init__('goal_subscriber_test')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/nav_goal',
            self.goal_callback,
            10)
        self.received_count = 0
        print("✅ 订阅器已启动，等待 /nav_goal 消息...")
        print("   (在另一个终端运行: python3 publish_goal.py --x 3.0 --y 0.0 --yaw 90)\n")

    def goal_callback(self, msg):
        self.received_count += 1
        x = msg.pose.position.x
        y = msg.pose.position.y

        # 从四元数提取yaw
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        print(f"📩 收到目标点 #{self.received_count}:")
        print(f"   位置: ({x:.3f}, {y:.3f})")
        print(f"   朝向: {math.degrees(yaw):.1f}° ({yaw:.3f} rad)")
        print(f"   Frame: {msg.header.frame_id}\n")


def main():
    rclpy.init()
    node = GoalSubscriberTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n✅ 测试结束，共接收 {node.received_count} 条消息")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

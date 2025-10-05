#!/usr/bin/env python3
"""
发布测试目标点到 /nav_goal

示例：
起点：(3.0, 0.0, -1.57)  # -90度
终点：(4.0, 1.0, 1.57)   # 90度
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import sys


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
    def __init__(self, x, y, yaw):
        super().__init__('goal_publisher')

        self.publisher = self.create_publisher(PoseStamped, '/nav_goal', 10)

        # 创建目标点消息
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = self.get_clock().now().to_msg()

        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        self.goal.pose.position.z = 0.0
        self.goal.pose.orientation = euler_to_quaternion(0.0, 0.0, yaw)

        # 创建定时器，每0.1秒发布一次
        self.timer = self.create_timer(0.1, self.publish_goal)
        self.count = 0
        self.max_count = 20  # 发布20次（2秒）

        print(f"✅ 准备发布目标点到 /nav_goal:")
        print(f"   位置: ({x:.3f}, {y:.3f})")
        print(f"   朝向: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
        print(f"   发布频率: 10 Hz，持续2秒\n")

    def publish_goal(self):
        if self.count >= self.max_count:
            print(f"✅ 已发布 {self.count} 次，完成\n")
            self.timer.cancel()
            sys.exit(0)

        self.goal.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.goal)
        self.count += 1

        if self.count % 5 == 0:
            print(f"📤 已发布 {self.count}/{self.max_count} 次...")


def main():
    import argparse

    parser = argparse.ArgumentParser(description='发布目标点到 /nav_goal')
    parser.add_argument('--x', type=float, default=4.0, help='目标X坐标（默认4.0）')
    parser.add_argument('--y', type=float, default=1.0, help='目标Y坐标（默认1.0）')
    parser.add_argument('--yaw', type=float, default=1.57, help='目标朝向（弧度，默认1.57即90度）')
    parser.add_argument('--yaw-deg', type=float, help='目标朝向（度数，会覆盖--yaw）')

    args = parser.parse_args()

    # 如果提供了度数，转换为弧度
    if args.yaw_deg is not None:
        yaw = math.radians(args.yaw_deg)
    else:
        yaw = args.yaw

    rclpy.init()

    publisher = GoalPublisher(args.x, args.y, yaw)

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

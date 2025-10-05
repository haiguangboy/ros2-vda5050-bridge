#!/usr/bin/env python3
"""
发布测试Odom数据到 /Odom

示例：
起点：(3.0, 0.0, -1.57)  # -90度
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
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


class OdomPublisher(Node):
    def __init__(self, x, y, yaw):
        super().__init__('odom_publisher')

        self.publisher = self.create_publisher(Odometry, '/Odom', 10)

        # 创建Odom消息
        self.odom = Odometry()
        self.odom.header.frame_id = "map"
        self.odom.child_frame_id = "base_link"

        self.odom.pose.pose.position.x = x
        self.odom.pose.pose.position.y = y
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, yaw)

        # 创建定时器，每0.1秒发布一次
        self.timer = self.create_timer(0.1, self.publish_odom)

        print(f"✅ 持续发布Odom数据到 /Odom:")
        print(f"   位置: ({x:.3f}, {y:.3f})")
        print(f"   朝向: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
        print(f"   发布频率: 10 Hz")
        print(f"   按 Ctrl+C 停止\n")

    def publish_odom(self):
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.odom)


def main():
    import argparse

    parser = argparse.ArgumentParser(description='发布Odom数据到 /Odom')
    parser.add_argument('--x', type=float, default=3.0, help='X坐标（默认3.0）')
    parser.add_argument('--y', type=float, default=0.0, help='Y坐标（默认0.0）')
    parser.add_argument('--yaw', type=float, default=-1.57, help='朝向（弧度，默认-1.57即-90度）')
    parser.add_argument('--yaw-deg', type=float, help='朝向（度数，会覆盖--yaw）')

    args = parser.parse_args()

    # 如果提供了度数，转换为弧度
    if args.yaw_deg is not None:
        yaw = math.radians(args.yaw_deg)
    else:
        yaw = args.yaw

    rclpy.init()

    publisher = OdomPublisher(args.x, args.y, yaw)

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print("\n🛑 停止发布")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
测试容器位姿发布器

发布容器位姿数据到 /container_pose 话题，用于测试完整的ContainerPose工作流程：
ROS2话题 -> MQTT桥接器 -> ActionMessage -> MQTT发布
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
import math

class ContainerPosePublisher(Node):
    def __init__(self):
        super().__init__('container_pose_publisher')

        # 创建发布器
        self.publisher = self.create_publisher(PoseStamped, '/container_pose', 10)

        # 创建定时器，每2秒发布一次
        self.timer = self.create_timer(2.0, self.publish_container_pose)

        self.pose_counter = 0

        self.get_logger().info('🚀 容器位姿发布器已启动')
        self.get_logger().info('📤 将每2秒发布一次容器位姿到 /container_pose 话题')

    def publish_container_pose(self):
        """发布容器位姿数据"""

        # 创建容器位姿消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # 生成测试位姿数据（模拟不同的容器位置）
        positions = [
            {"x": 2.0, "y": 1.0, "z": 0.5, "theta": 0.0, "name": "货架A"},
            {"x": 3.5, "y": 2.5, "z": 0.8, "theta": math.pi/4, "name": "货架B"},
            {"x": 1.5, "y": 3.0, "z": 0.3, "theta": math.pi/2, "name": "货架C"},
            {"x": 4.0, "y": 1.5, "z": 0.6, "theta": -math.pi/4, "name": "货架D"}
        ]

        current_pos = positions[self.pose_counter % len(positions)]

        # 设置位姿
        pose_msg.pose.position.x = current_pos["x"]
        pose_msg.pose.position.y = current_pos["y"]
        pose_msg.pose.position.z = current_pos["z"]

        # 将theta角度转换为四元数
        theta = current_pos["theta"]
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(theta / 2.0)

        # 发布消息
        self.publisher.publish(pose_msg)

        self.get_logger().info(
            f'📦 发布容器位姿 [{current_pos["name"]}]: '
            f'位置=({current_pos["x"]:.1f}, {current_pos["y"]:.1f}, {current_pos["z"]:.1f}) '
            f'角度={math.degrees(theta):.1f}°'
        )

        self.pose_counter += 1

def main(args=None):
    rclpy.init(args=args)

    print("🧪 启动容器位姿测试发布器")
    print("=" * 50)

    publisher = ContainerPosePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print("\n⏹️  用户中断，停止发布器")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()
        print("✅ 容器位姿发布器已停止")

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
测试GoToPose Service
模拟调度器调用/go_to_pose服务发送目标点
"""

import rclpy
from rclpy.node import Node
from forklift_interfaces.srv import GoToPose
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import sys


class GoToPoseClient(Node):
    def __init__(self):
        super().__init__('goto_pose_client')
        self.client = self.create_client(GoToPose, '/go_to_pose')

        print("⏳ 等待 /go_to_pose service...")
        if not self.client.wait_for_service(timeout_sec=10.0):
            print("❌ Service 未就绪，请先启动 unified_planner_workflow.py")
            sys.exit(1)

        print("✅ Service 已就绪\n")

    def send_goal(self, x, y, yaw_deg, mode=GoToPose.Request.MODE_NORMAL, timeout_sec=60.0):
        """
        发送目标点

        Args:
            x: X坐标
            y: Y坐标
            yaw_deg: 朝向角度（度）
            mode: 模式（0=NORMAL, 1=FORK）
            timeout_sec: 超时时间
        """
        request = GoToPose.Request()
        request.mode = mode
        request.timeout_sec = timeout_sec

        # 设置目标位置
        request.target = PoseStamped()
        request.target.header.frame_id = "map"
        request.target.header.stamp = self.get_clock().now().to_msg()
        request.target.pose.position.x = x
        request.target.pose.position.y = y
        request.target.pose.position.z = 0.0
        request.target.pose.orientation = self.yaw_to_quaternion(math.radians(yaw_deg))

        # 如果是FORK模式，设置托盘信息（示例）
        if mode == GoToPose.Request.MODE_FORK:
            request.pallet_pose.position.x = x + 0.5
            request.pallet_pose.position.y = y
            request.pallet_size.x = 1.2
            request.pallet_size.y = 0.8
            request.pallet_size.z = 0.15

        print(f"📤 发送目标点: ({x}, {y}, {yaw_deg}°)")
        mode_str = "NORMAL" if mode == GoToPose.Request.MODE_NORMAL else "FORK"
        print(f"   模式: {mode_str}")
        print(f"   超时: {timeout_sec}秒\n")

        # 调用service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            print("="*80)
            print("📥 收到响应")
            print("="*80)
            print(f"arrived: {response.arrived}")
            print(f"message: {response.message}")
            print("="*80 + "\n")
            return response
        else:
            print("❌ Service 调用失败\n")
            return None

    @staticmethod
    def yaw_to_quaternion(yaw):
        """将yaw角转换为四元数"""
        q = Quaternion()
        q.w = math.cos(yaw * 0.5)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        return q


def main():
    print("🧪 GoToPose Service 测试")
    print("="*80)
    print("模拟调度器使用GoToPose.srv格式发送目标点")
    print("="*80)
    print()

    rclpy.init()
    client = GoToPoseClient()

    try:
        # 发送第1个目标点（观察点）
        print("="*80)
        print("步骤1: 发送观察点")
        print("="*80)
        response1 = client.send_goal(x=3.0, y=0.5,yaw_deg=90, mode=GoToPose.Request.MODE_NORMAL)

        if response1 and response1.arrived:
            print("✅ 观察点已接受\n")
            print("💡 等待轨迹完成后，发送第2个目标点...\n")
            input("按Enter继续发送取货点...")

            # 发送第2个目标点（取货点）
            print("\n" + "="*80)
            print("步骤2: 发送取货点")
            print("="*80)
            response2 = client.send_goal(x=4.5, y=1.3,yaw_deg=-95,mode=GoToPose.Request.MODE_NORMAL)

            if response2 and response2.arrived:
                print("✅ 取货点已接受\n")
                print("🎉 所有目标点已发送！")
            else:
                print("❌ 取货点被拒绝")
        else:
            print("❌ 观察点被拒绝")

    except KeyboardInterrupt:
        print("\n⏹️  收到中断信号")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

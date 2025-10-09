#!/usr/bin/env python3
"""
测试GoToPose Service - 完整流程（观察点 → 取货 → 卸货）
发送顺序：观察点 → 取货点 → 卸货点

模拟完整的物流流程：
1. 前往观察点观察货物
2. 前往取货点叉取货物
3. 前往卸货点卸载货物
"""

import rclpy
from rclpy.node import Node
from forklift_interfaces.srv import GoToPose
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import sys

# ==================== 配置参数 ====================
ENABLE_CORRECTION_TRAJECTORY = True  # 误差消除轨迹
ENABLE_UNLOAD_TRAJECTORY = True  # 卸货轨迹


class GoToPoseClient(Node):
    def __init__(self):
        super().__init__('goto_pose_client')
        self.client = self.create_client(GoToPose, '/go_to_pose')

        print("⏳ 等待 /go_to_pose service...")
        if not self.client.wait_for_service(timeout_sec=10.0):
            print("❌ Service 未就绪，请先启动 unified_planner_workflow.py")
            sys.exit(1)

        print("✅ Service 已就绪\n")

    def send_goal(self, x, y, yaw_deg, mode=GoToPose.Request.MODE_NORMAL, timeout_sec=600.0):
        """发送目标点"""
        request = GoToPose.Request()
        request.mode = mode
        request.timeout_sec = float(timeout_sec)

        # 设置目标位置
        request.target = PoseStamped()
        request.target.header.frame_id = "map"
        request.target.header.stamp = self.get_clock().now().to_msg()
        request.target.pose.position.x = x
        request.target.pose.position.y = y
        request.target.pose.position.z = 0.0
        request.target.pose.orientation = self.yaw_to_quaternion(math.radians(yaw_deg))

        # 如果是FORK模式，设置托盘信息
        if mode == GoToPose.Request.MODE_FORK:
            request.pallet_pose.position.x = x
            request.pallet_pose.position.y = y
            request.pallet_pose.position.z = 1.5
            request.pallet_pose.orientation = self.yaw_to_quaternion(math.radians(yaw_deg))

            request.pallet_size.x = 1.2  # 长度
            request.pallet_size.y = 0.7  # 宽度
            request.pallet_size.z = 0.15  # 高度

        mode_str = ["NORMAL", "FORK"][mode] if mode < 2 else "NORMAL"
        print(f"📤 发送目标点: ({x}, {y}, {yaw_deg}°)")
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
    print("🧪 完整物流流程测试")
    print("="*80)
    print("流程：观察点 → 取货点 → 卸货点")
    print()
    print("📝 说明：")
    print("  - GoToPose service是同步调用，会阻塞直到轨迹执行完成")
    print("  - 观察点：MODE_NORMAL")
    print("  - 取货点：MODE_FORK（含误差消除轨迹）")
    print("  - 卸货点：MODE_UNLOAD（5段卸货轨迹）")
    print()
    print("⚠️  注意：整个流程可能需要较长时间！")
    print("="*80)
    print()

    rclpy.init()
    client = GoToPoseClient()

    try:
        # 步骤1: 观察点
        print("="*80)
        print("步骤1: 发送观察点")
        print("="*80)
        response1 = client.send_goal(x=3.0, y=0.0, yaw_deg=-90, mode=GoToPose.Request.MODE_NORMAL)

        if response1 and response1.arrived:
            print("✅ 观察点已到达！\n")
            input("👉 按Enter键继续发送取货点...")
            print()

            # 步骤2: 取货点
            print("="*80)
            print("步骤2: 发送取货点")
            print("="*80)
            pickup_yaw = 90 if ENABLE_CORRECTION_TRAJECTORY else 90
            response2 = client.send_goal(x=4.0, y=-1.0, yaw_deg=pickup_yaw, mode=GoToPose.Request.MODE_FORK)

            if response2 and response2.arrived:
                print("✅ 取货点已到达！\n")
                input("👉 按Enter键继续发送卸货点...")
                print()

                # 步骤3: 卸货点
                print("="*80)
                print("步骤3: 发送卸货点")
                print("="*80)
                print("💡 卸货轨迹：倒车回主干道 → 转向 → 沿主干道行驶 → 转向 → 到达卸货点")
                print("💡 使用MODE_NORMAL，系统会检测到取货后的目标点自动触发卸货轨迹")
                response3 = client.send_goal(x=1.0, y=2.0, yaw_deg=-90, mode=GoToPose.Request.MODE_NORMAL, timeout_sec=900.0)

                if response3 and response3.arrived:
                    print("✅ 卸货点已到达！\n")
                    print("🎉 完整物流流程完成！")
                else:
                    print("❌ 卸货点执行失败或超时")
            else:
                print("❌ 取货点执行失败或超时")
        else:
            print("❌ 观察点执行失败或超时")

    except KeyboardInterrupt:
        print("\n⏹️  收到中断信号")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
测试GoToPose Service
模拟调度器调用/go_to_pose服务发送目标点

注意：
- 如果unified_planner_workflow.py启用了误差消除轨迹（ENABLE_CORRECTION_TRAJECTORY=True）
  第2个目标点的yaw应该设为0°（因为车子会先回正）
- 如果禁用了误差消除轨迹，第2个目标点的yaw应该与第1个点保持一致（-90°）
"""

import rclpy
from rclpy.node import Node
from forklift_interfaces.srv import GoToPose
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import sys

# ==================== 配置参数 ====================
# 需要与unified_planner_workflow.py中的配置保持一致
ENABLE_CORRECTION_TRAJECTORY = True  # 是否启用误差消除轨迹


class GoToPoseClient(Node):
    def __init__(self):
        super().__init__('goto_pose_client')
        self.client = self.create_client(GoToPose, '/go_to_pose')

        print("⏳ 等待 /go_to_pose service...")
        if not self.client.wait_for_service(timeout_sec=10.0):
            print("❌ Service 未就绪，请先启动 unified_planner_workflow.py")
            sys.exit(1)

        print("✅ Service 已就绪\n")

    def send_goal(self, x, y, yaw_deg, mode=GoToPose.Request.MODE_NORMAL, timeout_sec=300.0):
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
        request.timeout_sec = float(timeout_sec)

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
            # 托盘位置通常与目标点位置一致或稍有偏移
            request.pallet_pose.position.x = x
            request.pallet_pose.position.y = y
            request.pallet_pose.position.z = 0.0
            request.pallet_pose.orientation = self.yaw_to_quaternion(math.radians(yaw_deg))

            # 托盘尺寸（示例值）
            # 注意：pallet_size.x 会被用作 container_width（容器宽度）
            request.pallet_size.x = 1.2  # 宽度（）
            request.pallet_size.y = 0.8  # 长度 会用作container_width（容器宽度）
            request.pallet_size.z = 0.15  # 高度

            # 计算theta（从yaw_deg转换）
            theta = math.radians(yaw_deg)

            print(f"   Pallet → ContainerPose mapping:")
            print(f"   x: {request.pallet_pose.position.x:.2f}")
            print(f"   y: {request.pallet_pose.position.y:.2f}")
            print(f"   z: {request.pallet_pose.position.z:.2f}")
            print(f"   theta: {theta:.3f}")
            print(f"   width: {request.pallet_size.y:.2f}")

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
            print(f"accepted: {response.arrived}")  # arrived字段表示"是否接受"
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
    print()
    print("📝 说明：")
    print("  - GoToPose service是同步调用，会阻塞直到轨迹执行完成")
    print("  - arrived=True：机器人已到达目标点")
    print("  - arrived=False：执行失败或超时")
    print()
    print(f"⚙️  配置：误差消除轨迹 {'启用' if ENABLE_CORRECTION_TRAJECTORY else '禁用'}")
    if ENABLE_CORRECTION_TRAJECTORY:
        print("     第2个目标点yaw将设为0°（车子会先回正）")
    else:
        print("     第2个目标点yaw将设为-90°（与第1个点保持一致）")
    print()
    print("⚠️  注意：service调用会等待轨迹完成才返回，可能需要较长时间！")
    print("="*80)
    print()

    rclpy.init()
    client = GoToPoseClient()

    try:
        # 发送第1个目标点（观察点）
        print("="*80)
        print("步骤1: 发送观察点")
        print("="*80)
        response1 = client.send_goal(x=3.0, y=0.0, yaw_deg=-90, mode=GoToPose.Request.MODE_NORMAL)

        if response1 and response1.arrived:
            print("✅ 观察点已到达！\n")

            # 等待用户确认后再发送第二个目标点
            input("👉 按Enter键继续发送第2个目标点（取货点）...")
            print()

            # 发送第2个目标点（取货点）
            print("="*80)
            print("步骤2: 发送取货点")
            print("="*80)

            # 根据是否启用误差消除轨迹，设置不同的yaw角度
            if ENABLE_CORRECTION_TRAJECTORY:
                # 启用误差消除：车子会先回正到0°，然后再规划取货轨迹
                # 所以第2个目标点的yaw设为0°
                pickup_yaw = 0
                print("💡 启用误差消除：车子会先回正+倒车0.6米，然后到达取货点")
            else:
                # 禁用误差消除：使用原流程，保持与观察点相同的朝向
                pickup_yaw = -90
                print("💡 禁用误差消除：直接从观察点到达取货点")

            # 第2个目标点使用MODE_FORK，需要提供托盘信息
            response2 = client.send_goal(x=4.0, y=1.0, yaw_deg=pickup_yaw, mode=GoToPose.Request.MODE_FORK)

            if response2 and response2.arrived:
                print("✅ 取货点已到达！\n")
                print("🎉 所有任务完成！")
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

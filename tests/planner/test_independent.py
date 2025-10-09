#!/usr/bin/env python3
"""
独立测试脚本 - 可以单独测试各个轨迹

测试模式：
1. 观察点测试（MODE_NORMAL）
2. 取货点测试（MODE_FORK）
3. 卸货点测试（MODE_NORMAL，需要先完成取货）
4. 完整流程测试（观察点 → 取货点 → 卸货点）

使用方法：
  python3 test_independent.py [模式]

  模式选项：
    observation  - 只测试观察点轨迹
    pickup       - 只测试取货点轨迹
    unload       - 只测试卸货点轨迹（需要先测试取货点以设置上下文）
    full         - 完整流程测试（默认）
"""

import rclpy
from rclpy.node import Node
from forklift_interfaces.srv import GoToPose
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import sys

# ==================== 配置参数 ====================
# 这些参数应该与 unified_planner_workflow.py 中的配置保持一致
ENABLE_CORRECTION_TRAJECTORY = True  # 误差消除轨迹
ENABLE_UNLOAD_TRAJECTORY = True  # 卸货轨迹

# 测试点位配置
TEST_POINTS = {
    'observation': {
        'x': 3.0,
        'y': 0.0,
        'yaw_deg': -90,
        'mode': 'NORMAL',
        'description': '观察点'
    },
    'pickup': {
        'x': 4.0,
        'y': -1.0,
        'yaw_deg': 90,
        'mode': 'FORK',
        'description': '取货点'
    },
    'unload': {
        'x': 1.0,
        'y': 2.0,
        'yaw_deg': -90,
        'mode': 'NORMAL',
        'description': '卸货点'
    }
}


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


def test_observation_only(client):
    """只测试观察点轨迹"""
    print("🧪 独立测试：观察点轨迹")
    print("="*80)
    print("📝 说明：")
    print("  - 测试 SimpleTrajectoryPlanner")
    print("  - 模式：MODE_NORMAL")
    print("="*80)
    print()

    point = TEST_POINTS['observation']
    print("="*80)
    print("发送观察点")
    print("="*80)
    response = client.send_goal(
        x=point['x'],
        y=point['y'],
        yaw_deg=point['yaw_deg'],
        mode=GoToPose.Request.MODE_NORMAL
    )

    if response and response.arrived:
        print("✅ 观察点测试完成！\n")
        return True
    else:
        print("❌ 观察点测试失败\n")
        return False


def test_pickup_only(client):
    """只测试取货点轨迹"""
    print("🧪 独立测试：取货点轨迹")
    print("="*80)
    print("📝 说明：")
    print("  - 测试 ComplexTrajectoryPlanner")
    if ENABLE_CORRECTION_TRAJECTORY:
        print("  - 包含误差消除轨迹（回正 + 倒车）")
    print("  - 模式：MODE_FORK")
    print("="*80)
    print()

    point = TEST_POINTS['pickup']
    print("="*80)
    print("发送取货点")
    print("="*80)
    response = client.send_goal(
        x=point['x'],
        y=point['y'],
        yaw_deg=point['yaw_deg'],
        mode=GoToPose.Request.MODE_FORK
    )

    if response and response.arrived:
        print("✅ 取货点测试完成！\n")
        print("💡 提示：现在可以运行 'python3 test_independent.py unload' 测试卸货轨迹")
        return True
    else:
        print("❌ 取货点测试失败\n")
        return False


def test_unload_only(client):
    """只测试卸货点轨迹"""
    print("🧪 独立测试：卸货点轨迹")
    print("="*80)
    print("📝 说明：")
    print("  - 测试卸货轨迹（3段）")
    print("  - 第1段：向前行驶回主干道")
    print("  - 第2段：右转 + 沿主干道前进")
    print("  - 第3段：左转 + 倒车到卸货点")
    print("  - 模式：MODE_NORMAL（需要上次是 MODE_FORK）")
    print()
    print("⚠️  注意：此测试需要先完成取货点测试以设置上下文！")
    print("         如果之前没有测试取货点，此测试会失败。")
    print("="*80)
    print()

    input("👉 按Enter键继续（确保已完成取货点测试）...")
    print()

    point = TEST_POINTS['unload']
    print("="*80)
    print("发送卸货点")
    print("="*80)
    print("💡 使用MODE_NORMAL，系统会检测到取货后的目标点自动触发卸货轨迹")
    response = client.send_goal(
        x=point['x'],
        y=point['y'],
        yaw_deg=point['yaw_deg'],
        mode=GoToPose.Request.MODE_NORMAL,
        timeout_sec=900.0
    )

    if response and response.arrived:
        print("✅ 卸货点测试完成！\n")
        return True
    else:
        print("❌ 卸货点测试失败\n")
        return False


def test_full_workflow(client):
    """完整流程测试"""
    print("🧪 完整物流流程测试")
    print("="*80)
    print("流程：观察点 → 取货点 → 卸货点")
    print()
    print("📝 说明：")
    print("  - GoToPose service是同步调用，会阻塞直到轨迹执行完成")
    print("  - 观察点：MODE_NORMAL")
    print("  - 取货点：MODE_FORK（含误差消除轨迹）")
    print("  - 卸货点：MODE_NORMAL（触发卸货轨迹）")
    print()
    print("⚠️  注意：整个流程可能需要较长时间！")
    print("="*80)
    print()

    # 步骤1: 观察点
    print("="*80)
    print("步骤1: 发送观察点")
    print("="*80)
    point = TEST_POINTS['observation']
    response1 = client.send_goal(
        x=point['x'],
        y=point['y'],
        yaw_deg=point['yaw_deg'],
        mode=GoToPose.Request.MODE_NORMAL
    )

    if response1 and response1.arrived:
        print("✅ 观察点已到达！\n")
        input("👉 按Enter键继续发送取货点...")
        print()

        # 步骤2: 取货点
        print("="*80)
        print("步骤2: 发送取货点")
        print("="*80)
        point = TEST_POINTS['pickup']
        response2 = client.send_goal(
            x=point['x'],
            y=point['y'],
            yaw_deg=point['yaw_deg'],
            mode=GoToPose.Request.MODE_FORK
        )

        if response2 and response2.arrived:
            print("✅ 取货点已到达！\n")
            input("👉 按Enter键继续发送卸货点...")
            print()

            # 步骤3: 卸货点
            print("="*80)
            print("步骤3: 发送卸货点")
            print("="*80)
            print("💡 卸货轨迹：向前回主干道 → 右转+前进 → 左转+倒车到卸货点")
            print("💡 使用MODE_NORMAL，系统会检测到取货后的目标点自动触发卸货轨迹")
            point = TEST_POINTS['unload']
            response3 = client.send_goal(
                x=point['x'],
                y=point['y'],
                yaw_deg=point['yaw_deg'],
                mode=GoToPose.Request.MODE_NORMAL,
                timeout_sec=900.0
            )

            if response3 and response3.arrived:
                print("✅ 卸货点已到达！\n")
                print("🎉 完整物流流程完成！")
                return True
            else:
                print("❌ 卸货点执行失败或超时")
                return False
        else:
            print("❌ 取货点执行失败或超时")
            return False
    else:
        print("❌ 观察点执行失败或超时")
        return False


def print_usage():
    """打印使用说明"""
    print("使用方法：")
    print("  python3 test_independent.py [模式]")
    print()
    print("模式选项：")
    print("  observation  - 只测试观察点轨迹（SimpleTrajectoryPlanner）")
    print("  pickup       - 只测试取货点轨迹（ComplexTrajectoryPlanner + 误差消除）")
    print("  unload       - 只测试卸货点轨迹（需要先完成取货点测试）")
    print("  full         - 完整流程测试（默认）")
    print()
    print("示例：")
    print("  python3 test_independent.py observation    # 只测试观察点")
    print("  python3 test_independent.py pickup         # 只测试取货点")
    print("  python3 test_independent.py                # 完整流程")


def main():
    # 解析命令行参数
    test_mode = 'full'  # 默认完整流程
    if len(sys.argv) > 1:
        test_mode = sys.argv[1].lower()
        if test_mode not in ['observation', 'pickup', 'unload', 'full']:
            print(f"❌ 无效的测试模式: {test_mode}")
            print()
            print_usage()
            sys.exit(1)

    rclpy.init()
    client = GoToPoseClient()

    try:
        if test_mode == 'observation':
            test_observation_only(client)
        elif test_mode == 'pickup':
            test_pickup_only(client)
        elif test_mode == 'unload':
            test_unload_only(client)
        else:  # full
            test_full_workflow(client)

    except KeyboardInterrupt:
        print("\n⏹️  收到中断信号")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

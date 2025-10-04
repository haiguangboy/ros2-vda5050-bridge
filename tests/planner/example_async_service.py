#!/usr/bin/env python3
"""
GoToPoseAsync Service 示例实现

展示如何使用异步导航service，结合状态查询service
这是一个理想的架构示例（需要先编译GoToPoseAsync.srv）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import Trigger
import json
import math
import time


# ==================== 服务端实现 ====================

class AsyncNavigationServer(Node):
    """
    异步导航服务器

    提供两个service：
    1. /go_to_pose_async - 接收导航请求，立即返回轨迹ID
    2. /trajectory_status - 查询轨迹执行状态
    """

    def __init__(self):
        super().__init__('async_navigation_server')

        # 注意：这里使用Trigger模拟GoToPoseAsync（因为还未编译）
        # 实际使用时应该导入编译后的GoToPoseAsync
        self.nav_service = self.create_service(
            Trigger, '/go_to_pose_async', self.handle_nav_request)

        self.status_service = self.create_service(
            Trigger, '/trajectory_status', self.handle_status_query)

        # 模拟轨迹状态
        self.current_trajectory = {
            'trajectory_id': '',
            'status': 'no_data',
            'timestamp': 0
        }

        print("✅ 异步导航服务器已启动")
        print(f"   Service: /go_to_pose_async")
        print(f"   Service: /trajectory_status\n")

    def handle_nav_request(self, request, response):
        """处理导航请求"""
        # 生成轨迹ID
        trajectory_id = f"traj_{int(time.time() * 1000)}"

        # 接受请求
        response.success = True
        response.message = json.dumps({
            'accepted': True,
            'trajectory_id': trajectory_id,
            'message': '导航请求已接受'
        })

        # 更新当前轨迹状态
        self.current_trajectory = {
            'trajectory_id': trajectory_id,
            'status': 'pending',
            'timestamp': int(time.time() * 1000)
        }

        print(f"📥 收到导航请求，轨迹ID: {trajectory_id}")

        # 模拟异步执行（实际应该调用规划器）
        # TODO: 调用轨迹规划器
        # self.plan_and_publish_trajectory(target_pose)

        return response

    def handle_status_query(self, request, response):
        """处理状态查询"""
        if self.current_trajectory['status'] != 'no_data':
            response.success = True
            response.message = json.dumps(self.current_trajectory)
        else:
            response.success = False
            response.message = json.dumps({
                'trajectory_id': '',
                'status': 'no_data',
                'timestamp': 0,
                'message': '暂无轨迹数据'
            })

        return response


# ==================== 客户端实现 ====================

class AsyncNavigationClient(Node):
    """
    异步导航客户端（状态机）

    展示如何使用异步导航service
    """

    def __init__(self):
        super().__init__('async_navigation_client')

        # Service客户端
        self.nav_client = self.create_client(Trigger, '/go_to_pose_async')
        self.status_client = self.create_client(Trigger, '/trajectory_status')

        # 等待service可用
        print("⏳ 等待service...")
        self.nav_client.wait_for_service()
        self.status_client.wait_for_service()
        print("✅ Service已就绪\n")

    def navigate_to(self, x, y, yaw_deg):
        """
        发起导航请求（异步，立即返回）

        Returns:
            trajectory_id: 轨迹ID（用于查询状态）
        """
        print(f"\n📍 发起导航请求:")
        print(f"   目标: ({x:.2f}, {y:.2f}), {yaw_deg}°")

        # 调用service
        request = Trigger.Request()
        future = self.nav_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            result = json.loads(response.message)
            trajectory_id = result['trajectory_id']
            print(f"   ✅ 请求已接受")
            print(f"   轨迹ID: {trajectory_id}\n")
            return trajectory_id
        else:
            print(f"   ❌ 请求被拒绝: {response.message}\n")
            return None

    def check_status(self, trajectory_id=None):
        """
        查询轨迹状态

        Args:
            trajectory_id: 轨迹ID（可选，如果为None则查询最新状态）

        Returns:
            status: pending/running/completed/failed/no_data
        """
        request = Trigger.Request()
        future = self.status_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            status_data = json.loads(response.message)
            return status_data['status']
        return 'no_data'

    def wait_for_completion(self, trajectory_id, timeout_sec=60.0):
        """
        等待轨迹完成

        Args:
            trajectory_id: 轨迹ID
            timeout_sec: 超时时间（秒）

        Returns:
            bool: 是否成功完成
        """
        print(f"⏳ 等待轨迹完成 (ID: {trajectory_id})...")

        start_time = time.time()
        while True:
            # 查询状态
            status = self.check_status(trajectory_id)

            if status == 'completed':
                print(f"   ✅ 轨迹已完成\n")
                return True
            elif status == 'failed':
                print(f"   ❌ 轨迹执行失败\n")
                return False
            elif time.time() - start_time > timeout_sec:
                print(f"   ⏰ 等待超时\n")
                return False

            # 每0.5秒查询一次
            time.sleep(0.5)


# ==================== 示例用法 ====================

def example_usage():
    """演示如何使用异步导航"""

    rclpy.init()

    # 创建客户端（状态机）
    client = AsyncNavigationClient()

    print("="*60)
    print("🎯 异步导航示例")
    print("="*60)

    # 场景1：单个目标点，等待完成
    print("\n【场景1】单个目标点，等待完成")
    print("-" * 60)

    trajectory_id = client.navigate_to(3.0, 0.0, 90)
    if trajectory_id:
        success = client.wait_for_completion(trajectory_id, timeout_sec=30)

    # 场景2：连续多个目标点，不等待
    print("\n【场景2】连续发布多个目标点")
    print("-" * 60)

    goals = [
        (3.0, 0.0, 90),
        (3.0, 3.0, 180),
        (0.0, 3.0, 270),
        (0.0, 0.0, 0)
    ]

    trajectory_ids = []
    for x, y, yaw in goals:
        traj_id = client.navigate_to(x, y, yaw)
        if traj_id:
            trajectory_ids.append(traj_id)

    # 场景3：定时查询状态（状态机模式）
    print("\n【场景3】定时查询状态")
    print("-" * 60)

    def status_timer_callback():
        status = client.check_status()
        print(f"   当前状态: {status}")

    # 创建定时器，每秒查询一次
    timer = client.create_timer(1.0, status_timer_callback)

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


def demo_comparison():
    """对比同步和异步模式"""

    print("\n" + "="*80)
    print("📊 同步 vs 异步导航对比演示")
    print("="*80)

    print("\n【同步模式】")
    print("-" * 80)
    print("# 伪代码")
    print("for goal in goals:")
    print("    response = navigate(goal)  # 阻塞，直到到达")
    print("    if response.arrived:")
    print("        print('到达')")
    print("")
    print("特点：")
    print("  ✅ 简单直观")
    print("  ❌ 阻塞等待，无法并行处理")
    print("  ❌ 状态机卡死在这里")

    print("\n【异步模式】")
    print("-" * 80)
    print("# 伪代码")
    print("for goal in goals:")
    print("    trajectory_id = navigate_async(goal)  # 立即返回")
    print("    # 状态机可以继续做其他事")
    print("")
    print("# 定时器回调")
    print("def check_status_timer():")
    print("    status = query_status(trajectory_id)")
    print("    if status == 'completed':")
    print("        on_navigation_complete()")
    print("")
    print("特点：")
    print("  ✅ 非阻塞，状态机可并行处理")
    print("  ✅ 可监控多个轨迹")
    print("  ❌ 需要定时查询状态")

    print("\n" + "="*80)


if __name__ == '__main__':
    # 演示对比
    demo_comparison()

    # 实际使用示例（需要服务器运行）
    # example_usage()

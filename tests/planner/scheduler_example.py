#!/usr/bin/env python3
"""
调度器示例代码
演示如何与轨迹规划器交互：发送目标点、查询状态、等待完成
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from example_interfaces.srv import Trigger
import json
import time
import math


class Scheduler(Node):
    def __init__(self):
        super().__init__('scheduler')

        # 发布器：发送目标点
        self.goal_publisher = self.create_publisher(PoseStamped, '/nav_goal', 10)

        # 服务客户端：查询轨迹状态
        self.status_client = self.create_client(Trigger, '/trajectory_status')

        print("✅ 调度器已启动")
        print("   发布器: /nav_goal (目标点)")
        print("   客户端: /trajectory_status (状态查询)\n")

    def send_observation_point(self, x=3.0, y=0.0, yaw_deg=90):
        """发送观察点"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation = self.yaw_to_quaternion(math.radians(yaw_deg))

        self.goal_publisher.publish(goal)
        print(f"📤 已发布观察点: ({x}, {y}, {yaw_deg}°)")

    def send_pickup_point(self, x=4.0, y=1.0, yaw_deg=90):
        """发送取货点"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation = self.yaw_to_quaternion(math.radians(yaw_deg))

        self.goal_publisher.publish(goal)
        print(f"📤 已发布取货点: ({x}, {y}, {yaw_deg}°)")

    def query_status(self):
        """查询轨迹状态"""
        request = Trigger.Request()
        future = self.status_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                status_data = json.loads(response.message)
                return status_data
        return None

    def wait_for_completion(self, timeout=60.0, check_interval=1.0):
        """
        等待轨迹完成

        Args:
            timeout: 超时时间（秒）
            check_interval: 查询间隔（秒）

        Returns:
            bool: 是否成功完成
        """
        start_time = time.time()
        last_status = None

        while (time.time() - start_time) < timeout:
            status = self.query_status()

            if status:
                current_status = status['status']

                # 只在状态变化时打印
                if current_status != last_status:
                    if current_status == 'running':
                        print(f"🏃 轨迹执行中: {status['trajectory_id']}")
                    elif current_status == 'completed':
                        print(f"✅ 轨迹完成: {status['trajectory_id']}")
                        return True
                    elif current_status == 'failed':
                        print(f"❌ 轨迹失败: {status['message']}")
                        return False

                    last_status = current_status

            time.sleep(check_interval)

        print(f"⏱️  超时：等待轨迹完成超过 {timeout} 秒")
        return False

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
    print("🤖 调度器示例")
    print("="*80)
    print("流程:")
    print("  1. 发送观察点 (3.0, 0.0, 90°)")
    print("  2. 等待观察点完成")
    print("  3. 发送取货点 (4.0, 1.0, 90°)")
    print("  4. 等待取货完成（前向 + 倒车）")
    print("="*80)
    print()

    rclpy.init()
    scheduler = Scheduler()

    # 等待服务可用
    print("⏳ 等待 /trajectory_status service...")
    if not scheduler.status_client.wait_for_service(timeout_sec=10.0):
        print("❌ Service 未就绪，请先启动 unified_planner_workflow.py")
        return

    print("✅ Service 已就绪\n")

    try:
        # ===== 步骤1: 发送观察点 =====
        print("="*80)
        print("步骤1: 发送观察点")
        print("="*80)
        scheduler.send_observation_point(x=3.0, y=0.0, yaw_deg=90)

        print("⏳ 等待观察点轨迹完成...")
        if scheduler.wait_for_completion(timeout=60.0):
            print("✅ 观察点任务完成\n")
            time.sleep(2)  # 等待2秒

            # ===== 步骤2: 发送取货点 =====
            print("="*80)
            print("步骤2: 发送取货点")
            print("="*80)
            scheduler.send_pickup_point(x=4.0, y=1.0, yaw_deg=90)

            print("⏳ 等待取货轨迹完成（前向 + 倒车）...")
            if scheduler.wait_for_completion(timeout=120.0):
                print("✅ 取货任务完成\n")

                print("="*80)
                print("🎉 所有任务完成！")
                print("="*80)
            else:
                print("\n❌ 取货任务超时或失败")
        else:
            print("\n❌ 观察点任务超时或失败")

    except KeyboardInterrupt:
        print("\n⏹️  收到中断信号")
    finally:
        scheduler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

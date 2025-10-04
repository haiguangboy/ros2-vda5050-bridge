#!/usr/bin/env python3
"""
轨迹状态查询客户端

用法：
  python3 query_trajectory_status.py
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
import json
import sys


class StatusQueryClient(Node):
    def __init__(self):
        super().__init__('status_query_client')
        self.client = self.create_client(Trigger, '/trajectory_status')

        # 等待service可用
        print("⏳ 等待 /trajectory_status service...")
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('❌ Service 未就绪')
            sys.exit(1)

        print("✅ Service 已就绪\n")

    def query_status(self):
        """查询轨迹状态"""
        request = Trigger.Request()

        print("📞 调用 /trajectory_status service...")

        # 同步调用service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()

            print("\n" + "="*80)
            print("📊 轨迹状态查询结果")
            print("="*80)

            if response.success:
                # 解析JSON响应
                status_data = json.loads(response.message)

                print(f"✅ 查询成功")
                print(f"\n状态信息:")
                print(f"  轨迹ID: {status_data.get('trajectory_id', 'N/A')}")
                print(f"  状态: {status_data.get('status', 'N/A')}")
                print(f"  时间戳: {status_data.get('timestamp', 0)}")
                print(f"  消息: {status_data.get('message', 'N/A')}")

                # 状态解释
                status = status_data.get('status', '')
                if status == 'pending':
                    print(f"\n📝 轨迹等待执行")
                elif status == 'running':
                    print(f"\n🏃 轨迹正在执行")
                elif status == 'completed':
                    print(f"\n✅ 轨迹已完成")
                elif status == 'failed':
                    print(f"\n❌ 轨迹执行失败")
                elif status == 'no_data':
                    print(f"\n⚠️  暂无轨迹状态数据")
            else:
                print(f"⚠️  暂无状态数据")
                print(f"   消息: {response.message}")

            print("="*80)
        else:
            print("❌ Service 调用失败")


def main():
    rclpy.init()
    client = StatusQueryClient()

    try:
        # 查询状态
        client.query_status()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

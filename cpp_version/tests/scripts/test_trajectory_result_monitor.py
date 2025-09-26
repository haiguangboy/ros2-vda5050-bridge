#!/usr/bin/env python3
# test_trajectory_result_monitor.py
#
# 测试ROS2轨迹结果监控功能
# 监听桥接器发布的轨迹执行结果
# 主题: /trajectory_result
#
# ⚠️  仅用于测试目的！
# 在生产环境中，真实的决策树会监听轨迹结果消息

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime

class TrajectoryResultMonitor(Node):
    def __init__(self):
        super().__init__('trajectory_result_monitor')

        self.subscription = self.create_subscription(
            String,
            '/trajectory_result',
            self.trajectory_result_callback,
            10
        )

        self.message_count = 0

        self.get_logger().info("📊 轨迹结果监控器已启动")
        self.get_logger().info("🔍 监听主题: /trajectory_result")
        self.get_logger().info("💡 提示: 使用轨迹状态发送器触发轨迹状态更新")
        self.get_logger().info("⏹️  按 Ctrl+C 停止")
        print()

    def trajectory_result_callback(self, msg):
        """轨迹结果消息回调"""
        try:
            self.message_count += 1
            current_time = datetime.now()

            print(f"\n📩 收到轨迹结果消息 #{self.message_count}")
            print(f"   时间: {current_time.strftime('%Y-%m-%d %H:%M:%S')}")

            # 解析JSON消息
            try:
                result_data = json.loads(msg.data)

                print(f"   基本信息:")
                print(f"     - 轨迹ID: {result_data.get('trajectoryId', 'N/A')}")
                print(f"     - 状态: {result_data.get('status', 'N/A')}")
                print(f"     - 时间戳: {result_data.get('timestamp', 'N/A')}")

                # 显示当前执行点
                if 'currentPointIndex' in result_data:
                    print(f"     - 当前点: {result_data['currentPointIndex']}")

                # 显示完成时间
                if 'finishTime' in result_data:
                    print(f"     - 完成时间: {result_data['finishTime']}")

                # 显示错误信息
                if result_data.get('status') == 'failed':
                    print(f"   ❌ 错误信息:")
                    print(f"     - 错误码: {result_data.get('errorCode', 'N/A')}")
                    print(f"     - 错误描述: {result_data.get('errorDesc', 'N/A')}")

                # 根据状态显示不同的标识
                status = result_data.get('status', '')
                if status == 'completed':
                    print(f"   ✅ 轨迹执行成功完成！")
                elif status == 'failed':
                    print(f"   ❌ 轨迹执行失败！")
                elif status == 'running':
                    print(f"   🔄 轨迹正在执行中...")
                elif status == 'pending':
                    print(f"   ⏳ 轨迹等待执行...")

                # 在消息数量较少时显示完整JSON
                if self.message_count <= 5:
                    print(f"   完整JSON (前5条消息):")
                    formatted_json = json.dumps(result_data, indent=4, ensure_ascii=False)
                    for line in formatted_json.split('\n'):
                        print(f"     {line}")

            except json.JSONDecodeError as e:
                print(f"   ❌ JSON解析失败: {e}")
                print(f"   原始消息: {msg.data}")

        except Exception as e:
            print(f"❌ 处理消息时出错: {e}")

def main():
    print("📊 启动ROS2轨迹结果监控测试")
    print("=" * 60)
    print("🔍 监听主题: /trajectory_result")
    print("📋 目的: 监控桥接器发布的轨迹执行结果")
    print("💡 提示: 这个消息是给决策树用的，表示/plan路径的执行结果")
    print()

    rclpy.init()

    try:
        monitor = TrajectoryResultMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n🛑 用户停止轨迹结果监控器")
    except Exception as e:
        print(f"\n❌ 轨迹结果监控器错误: {e}")
    finally:
        try:
            monitor.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("✅ 轨迹结果监控器已关闭")

if __name__ == '__main__':
    main()
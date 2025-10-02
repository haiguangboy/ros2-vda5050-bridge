#!/usr/bin/env python3
# test_navigation_monitor.py
#
# 测试ROS2标准导航消息监听功能
# 监听桥接器发布的标准导航消息
# 主题: /current_pose, /navigation_status, /navigation_feedback
#
# ⚠️  仅用于测试目的！
# 在生产环境中，真实的决策树会监听这些导航消息

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
import json
from datetime import datetime

class NavigationMonitor(Node):
    def __init__(self):
        super().__init__('navigation_monitor')

        # 创建订阅器
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.current_pose_callback,
            10
        )

        self.status_subscription = self.create_subscription(
            GoalStatus,
            '/navigation_status',
            self.navigation_status_callback,
            10
        )

        self.feedback_subscription = self.create_subscription(
            String,
            '/navigation_feedback',
            self.navigation_feedback_callback,
            10
        )

        self.message_counts = {
            'pose': 0,
            'status': 0,
            'feedback': 0
        }

        self.get_logger().info("🧭 ROS2标准导航消息监控器已启动")
        self.get_logger().info("🔍 监听主题:")
        self.get_logger().info("  - /current_pose (geometry_msgs/PoseStamped)")
        self.get_logger().info("  - /navigation_status (action_msgs/GoalStatus)")
        self.get_logger().info("  - /navigation_feedback (std_msgs/String)")
        self.get_logger().info("💡 提示: 使用轨迹状态发送器触发导航状态更新")
        self.get_logger().info("⏹️  按 Ctrl+C 停止")
        print()

    def current_pose_callback(self, msg):
        """当前位置消息回调"""
        try:
            self.message_counts['pose'] += 1
            current_time = datetime.now()

            print(f"\n📍 收到当前位置消息 #{self.message_counts['pose']}")
            print(f"   时间: {current_time.strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"   坐标系: {msg.header.frame_id}")
            print(f"   位置:")
            print(f"     x: {msg.pose.position.x:.3f} m")
            print(f"     y: {msg.pose.position.y:.3f} m")
            print(f"     z: {msg.pose.position.z:.3f} m")
            print(f"   姿态:")
            print(f"     x: {msg.pose.orientation.x:.3f}")
            print(f"     y: {msg.pose.orientation.y:.3f}")
            print(f"     z: {msg.pose.orientation.z:.3f}")
            print(f"     w: {msg.pose.orientation.w:.3f}")

        except Exception as e:
            print(f"❌ 处理位置消息时出错: {e}")

    def navigation_status_callback(self, msg):
        """导航状态消息回调"""
        try:
            self.message_counts['status'] += 1
            current_time = datetime.now()

            print(f"\n📊 收到导航状态消息 #{self.message_counts['status']}")
            print(f"   时间: {current_time.strftime('%Y-%m-%d %H:%M:%S')}")

            # 解析状态
            status_map = {
                0: "STATUS_UNKNOWN",
                1: "STATUS_ACCEPTED",
                2: "STATUS_EXECUTING",
                3: "STATUS_CANCELING",
                4: "STATUS_SUCCEEDED",
                5: "STATUS_CANCELED",
                6: "STATUS_ABORTED"
            }

            status_text = status_map.get(msg.status, f"UNKNOWN({msg.status})")
            print(f"   状态: {status_text}")
            print(f"   时间戳: {msg.goal_info.stamp.sec}.{msg.goal_info.stamp.nanosec}")

            # 显示状态对应的中文含义
            status_meaning = {
                0: "未知状态",
                1: "已接受",
                2: "执行中",
                3: "取消中",
                4: "成功完成",
                5: "已取消",
                6: "已中止"
            }

            if msg.status in status_meaning:
                print(f"   含义: {status_meaning[msg.status]}")

            # 根据状态显示图标
            if msg.status == 4:  # STATUS_SUCCEEDED
                print(f"   ✅ 导航成功完成！")
            elif msg.status == 6:  # STATUS_ABORTED
                print(f"   ❌ 导航已中止！")
            elif msg.status == 2:  # STATUS_EXECUTING
                print(f"   🔄 导航正在执行...")
            elif msg.status == 1:  # STATUS_ACCEPTED
                print(f"   ⏳ 导航任务已接受...")

        except Exception as e:
            print(f"❌ 处理导航状态消息时出错: {e}")

    def navigation_feedback_callback(self, msg):
        """导航反馈消息回调"""
        try:
            self.message_counts['feedback'] += 1
            current_time = datetime.now()

            print(f"\n💬 收到导航反馈消息 #{self.message_counts['feedback']}")
            print(f"   时间: {current_time.strftime('%Y-%m-%d %H:%M:%S')}")

            # 解析JSON反馈消息
            try:
                feedback_data = json.loads(msg.data)

                print(f"   基本信息:")
                print(f"     - 轨迹ID: {feedback_data.get('trajectoryId', 'N/A')}")
                print(f"     - 状态: {feedback_data.get('status', 'N/A')}")
                print(f"     - 时间戳: {feedback_data.get('timestamp', 'N/A')}")

                # 显示当前执行点
                if 'currentPointIndex' in feedback_data:
                    print(f"     - 当前点: {feedback_data['currentPointIndex']}")

                # 显示完成时间
                if 'finishTime' in feedback_data:
                    print(f"     - 完成时间: {feedback_data['finishTime']}")

                # 显示预计完成时间
                if 'estimatedFinishTime' in feedback_data:
                    print(f"     - 预计完成时间: {feedback_data['estimatedFinishTime']}")

                # 显示错误信息
                if feedback_data.get('status') == 'failed':
                    print(f"   ❌ 错误信息:")
                    print(f"     - 错误码: {feedback_data.get('errorCode', 'N/A')}")
                    print(f"     - 错误描述: {feedback_data.get('errorDesc', 'N/A')}")

                # 在消息数量较少时显示完整JSON
                if self.message_counts['feedback'] <= 3:
                    print(f"   完整JSON (前3条消息):")
                    formatted_json = json.dumps(feedback_data, indent=4, ensure_ascii=False)
                    for line in formatted_json.split('\n'):
                        print(f"     {line}")

            except json.JSONDecodeError as e:
                print(f"   ❌ JSON解析失败: {e}")
                print(f"   原始消息: {msg.data}")

        except Exception as e:
            print(f"❌ 处理导航反馈消息时出错: {e}")

def main():
    print("🧭 启动ROS2标准导航消息监控测试")
    print("=" * 60)
    print("🔍 监听主题:")
    print("  - /current_pose: 当前位置信息 (持续更新)")
    print("  - /navigation_status: 导航状态 (类似nav2控制器)")
    print("  - /navigation_feedback: 导航反馈 (详细状态信息)")
    print("📋 目的: 验证桥接器发布符合ROS2导航标准的消息")
    print("💡 这些消息替代了原来的/trajectory_result字符串消息")
    print()

    rclpy.init()

    try:
        monitor = NavigationMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n🛑 用户停止导航消息监控器")
    except Exception as e:
        print(f"\n❌ 导航消息监控器错误: {e}")
    finally:
        try:
            monitor.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("✅ 导航消息监控器已关闭")

if __name__ == '__main__':
    main()
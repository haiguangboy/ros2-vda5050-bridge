#!/usr/bin/env python3
"""
模拟轨迹执行器

功能：
1. 订阅 /plans 话题（接收轨迹）
2. 通过MQTT发布轨迹状态（running → completed）

简化版本：不依赖/Odom，只负责返回执行状态
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import time
from nav_msgs.msg import Path


# ==================== 配置参数 ====================

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
ROBOT_ID = "robot-001"

# 模拟执行延时（秒）
EXECUTION_DELAY = 1.0  # 每个轨迹模拟执行1秒


# ==================== 模拟执行器 ====================

class MockExecutor(Node):
    def __init__(self):
        super().__init__('mock_executor')

        # MQTT客户端
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect

        # ROS2订阅器
        self.path_subscriber = self.create_subscription(
            Path, '/plans', self.path_callback, 10)

        print("✅ 模拟执行器已启动")
        print("   订阅: /plans")
        print("   MQTT: 发布轨迹状态 (running → completed)\n")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT已连接\n")
        else:
            print(f"❌ MQTT连接失败: {rc}\n")

    def path_callback(self, msg):
        """接收轨迹并模拟执行"""
        print("\n" + "="*80)
        print("📥 收到轨迹指令")
        print("="*80)

        # 解析header获取轨迹ID和参数
        header_parts = msg.header.frame_id.split('|')
        if len(header_parts) >= 11:
            trajectory_id = header_parts[10]
            orientation = float(header_parts[3])
            flag = int(header_parts[4])
        else:
            trajectory_id = f"unknown_{int(time.time() * 1000)}"
            orientation = 0.0
            flag = 0

        num_points = len(msg.poses)

        print(f"📋 轨迹ID: {trajectory_id}")
        print(f"📍 路径点数: {num_points}")
        print(f"🔄 orientation: {orientation}")
        print(f"🌿 flag: {flag}")

        if num_points == 0:
            print("⚠️  空轨迹，忽略")
            return

        print("="*80)

        # 发布 "running" 状态
        self.publish_trajectory_status(trajectory_id, "running", "轨迹执行中")
        print(f"📤 发布MQTT状态: running")

        # 模拟执行（简单延时）
        print(f"⏳ 模拟执行轨迹... ({EXECUTION_DELAY}秒)")
        time.sleep(EXECUTION_DELAY)

        # 发布 "completed" 状态
        self.publish_trajectory_status(trajectory_id, "completed", "轨迹执行完成")
        print(f"📤 发布MQTT状态: completed")
        print(f"✅ 执行完成\n")

    def publish_trajectory_status(self, trajectory_id, status, message):
        """通过MQTT发布轨迹状态"""
        status_topic = f"EP/{ROBOT_ID}/cerebellum/embrain/trajectory_status"

        status_data = {
            "trajectoryId": trajectory_id,
            "status": status,
            "timestamp": str(int(time.time() * 1000)),  # 转换为字符串以匹配C++解析器
            "message": message
        }

        self.mqtt_client.publish(status_topic, json.dumps(status_data))


    def start_mqtt(self):
        """启动MQTT连接"""
        try:
            print(f"🚀 连接到MQTT代理: {MQTT_BROKER}:{MQTT_PORT}")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            time.sleep(1)  # 等待连接
            return True
        except Exception as e:
            print(f"❌ MQTT连接失败: {e}")
            return False

    def stop(self):
        """停止执行器"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        print("\n🛑 执行器已停止")


def main():
    print("🎮 模拟轨迹执行器（简化版）")
    print("="*80)
    print("功能:")
    print("  1. 接收 /plans 轨迹指令")
    print("  2. 发布MQTT状态消息（running → completed）")
    print("="*80)
    print()

    rclpy.init()

    executor = MockExecutor()

    # 启动MQTT
    if not executor.start_mqtt():
        return

    try:
        print("💡 执行器运行中，按 Ctrl+C 停止\n")
        rclpy.spin(executor)
    except KeyboardInterrupt:
        print("\n⏹️  收到停止信号")
    finally:
        executor.stop()
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
